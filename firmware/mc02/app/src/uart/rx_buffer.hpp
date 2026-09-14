#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <main.h>
#include <usart.h>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/utility/assert.hpp"

namespace libhcs::firmware::uart {

// 2 的幂 ring 上的连续 DMA 接收。单个循环 DMA 传输覆盖整个 ring, 启动后从不
// 停止或重启: USART_CR3_DMAR 在端口生命周期内保持置位, 由硬件自行回绕。
// 写位置不经中断维护: NDTR 对整个 ring 倒计数并在回绕时重载, 消费者在
// try_dequeue() 里直接从它推导位置。由此带来三点:
//   - 接收服务的截止期从一个 bank (32 字节时间)放宽到一整圈
//     (kBufferSize 字节时间), 硬件到达前无需任何中断推进缓冲地址;
//   - RX 中断频率降到 IDLE 事件频率(921600 baud 下每口约 9000 次/秒);
//   - 消除了唯一一处用全局 __disable_irq() 屏蔽 FDCAN 中断的地方: FDCAN 在
//     NVIC 优先级 1, 本板的 .itcm/.dtcm 正是花在它的转发路径上。
//
// 线路错误按吸收而非恢复处理: CR3.DDRE 清零使奇偶/帧/噪声错误不阻塞 DMA
// 请求, CR3.OVRDIS 置位则完全不再上报溢出; 改这两位前必读
// configure_rx_error_policy() 的极性说明。F407 无此两位, 勿照搬 c_board 的
// 拆建重启方案; 本类的重启路径只服务于真正的 DMA 控制器故障, 且用到了
// c_board 没有的 ICR(不破坏 RDR 即可清粘滞标志)和 RQR.RXFRQ(清空 16 项
// RXFIFO)。
//
// 消费者运行在主循环(try_dequeue), 中断路径只递增计数器、绝不触碰 USB。
//
// buffer_size 即 ring, 唯一要满足的约束是消费者必须在 DMA 绕完一圈前回来。
// 默认值适合连续流的端口, 只跑短请求/响应事务的端口可以小得多; 所有 ring 都
// 从 32 KB 区域里扣除, 两个 RS-485 口的取值见 uart.hpp 末尾。
template <typename T, size_t buffer_size = 2048>
class RxBuffer {
    friend T;

public:
    static constexpr size_t kBufferSize = buffer_size;
    static constexpr size_t kBufferMask = kBufferSize - 1;
    static_assert((kBufferSize & (kBufferSize - 1)) == 0);
    using IndexType = uint16_t;
    static_assert(kBufferSize <= std::numeric_limits<IndexType>::max());

    static constexpr size_t kMinFragmentSize = 32;
    static constexpr size_t kProtocolMaxPayloadSize =
        core::protocol::kProtocolBufferSize - sizeof(core::protocol::UartHeaderExtended);
    static_assert(0 < kMinFragmentSize && kMinFragmentSize <= kProtocolMaxPayloadSize);
    // 低于该值时 ring 连 try_dequeue() 转发非 idle 片段前需凑齐的字节都容不下,
    // 只会停滞而无法攒批; sample_write_position() 的落后断言也设在半圈处,
    // 需与其拉开明显距离。
    static_assert(kMinFragmentSize * 4 <= kBufferSize);

    bool try_dequeue() {
        // 先读 idle 计数再采样写指针: IDLE 中断在线路转静的瞬间发布计数, 先读
        // 计数总能与不早于该瞬间的写指针对配, 保证下方发布的块必含该边界。若
        // 先读 NDTR, 陈旧位置可能与更晚发生的 idle 事件配对, 为尚未入 ring 的
        // 字节误报边界。
        const auto idle_count = idle_count_.load(std::memory_order::acquire);
        const auto in = sample_write_position();
        const auto out = out_;
        const auto readable = static_cast<size_t>(static_cast<IndexType>(in - out));

        if (readable > kBufferSize) [[unlikely]] {
            // 异常: 环形队列已绕圈覆盖。debug 构建下 fail-fast, 尽早暴露时序
            // 与中断问题。
            core::utility::assert_debug_lazy([]() noexcept { return false; });

            // Release 兜底: 丢弃积压字节以重新同步流。
            out_ = in;
            consumed_idle_count_ = idle_count;
            return false;
        }

        const bool is_idle =
            (readable <= kProtocolMaxPayloadSize) ? (idle_count != consumed_idle_count_) : false;
        if (is_idle)
            consumed_idle_count_ = idle_count;
        else if (readable < kMinFragmentSize)
            return false;

        const auto size = std::min(readable, kProtocolMaxPayloadSize);
        const auto offset = out & kBufferMask;
        const auto first_size = std::min(size, kBufferSize - offset);
        const auto second_size = size - first_size;

        static_cast<T*>(this)->handle_uplink(
            {ring_.data() + offset, first_size}, {ring_.data(), second_size}, is_idle);

        out_ = static_cast<IndexType>(out + static_cast<IndexType>(size));
        return true;
    }

private:
    // 限定 wait_receiver_ready() 的 ISR.REACK 轮询上限。稳态下 RE 早已应答,
    // 循环通常首读即退出; 此上限只是防止接收器永不恢复时 restart_rx_dma() 在
    // DMA 错误中断里无限空转。
    static constexpr uint32_t kReceiverAckPollLimit = 1024;

    explicit RxBuffer(UART_HandleTypeDef* hal_uart_handle)
        : hal_uart_handle_(hal_uart_handle) {
        enable_fifo_mode();
        configure_rx_error_policy();
        bind_rx_dma_callbacks();
        start_rx_dma();
    }

    // 所有端口都打开 16 项 RX/TX FIFO, 由驱动而非 .ioc 决定。CubeMX 对四口的
    // 开关本就不一致(UART5/UART7 开、USART1/USART10 关)且无理由; 各口阈值处处
    // 相同(RXFTCFG/TXFTCFG 均为 1/8, DisableFifoMode 只清 CR1.FIFOEN 不动
    // CR3), 差别仅此一位。放在驱动里的两个理由:
    //   - TxBuffer::try_dequeue() 依据 ISR.TC 而非 DMA 完成来计时 idle 窗口,
    //     其前提正是 TXFIFO 可在 DMA 报完成后仍压着 16 字节(921600 baud 下约
    //     173 us, 与 300 us 窗口同量级); 该推理只在 FIFO 开启时成立。不变量与
    //     依赖它的代码放在一起, CubeMX 重新生成时才不会悄然失效。
    //   - configure_rx_error_policy() 置 CR3.OVRDIS, 溢出完全不上报, DMA 一旦
    //     晚到而丢字节将是静默的; 16 字节 FIFO 正是针对此的缓冲。
    // 与吞吐无关: 921600 baud 下一字节 10.8 us, DMA 仲裁快几个量级, 正常运行
    // 碰不到这层缓冲, 它是为异常情况准备的。
    //
    // 经 HAL 入口而非直写 CR1, 使 huart->FifoMode 保持真实; 该入口会自行在写
    // 位前后关断并恢复 UE, 这是必须的, 因为 FIFOEN 仅在 USART 禁止时才可写。
    void enable_fifo_mode() const {
        core::utility::assert_always(HAL_UARTEx_EnableFifoMode(hal_uart_handle_) == HAL_OK);
    }

    // STM32H7 把 DMA_HandleTypeDef::Instance 声明为 void*(F4 为
    // DMA_Stream_TypeDef*), 寄存器访问必须经此转换。UART 的 RX 流是 DMA1/DMA2
    // 流而非 BDMA: BDMA 只服务 D3 域, 本板所有 UART 都在 D2。
    [[nodiscard]] DMA_Stream_TypeDef* rx_dma_stream() const {
        return static_cast<DMA_Stream_TypeDef*>(hal_uart_handle_->hdmarx->Instance);
    }

    // 由流自身计数器取下个写入位置的 ring 偏移, 并补上偏移承载不了的圈数位。
    //
    // NDTR 从 kBufferSize 起倒计, 回绕时由硬件重载, 故偏移 = kBufferSize -
    // NDTR。刚回绕后 NDTR 读到 kBufferSize, 重载前一瞬可能读到 0, 两者按掩码
    // 都得偏移 0, 恰为流所在处。只需读这一个寄存器, 不存在跨硬件切换采样两个
    // 寄存器的竞态。
    IndexType sample_write_position() {
        const auto remaining = static_cast<size_t>(rx_dma_stream()->NDTR);
        core::utility::assert_debug(remaining <= kBufferSize);
        const auto offset = static_cast<IndexType>((kBufferSize - remaining) & kBufferMask);

        auto next = static_cast<IndexType>((in_ & ~kBufferMask) | offset);
        if (next < in_)
            next = static_cast<IndexType>(next + static_cast<IndexType>(kBufferSize));

        // 消费者落后整圈时数据会被原地覆盖, 上面的重构只会低报而非触发
        // try_dequeue() 里 readable > kBufferSize 的检查, 故须远早于此设陷:
        // 主循环每几微秒轮询一次, 半圈在 921600 baud 下是 11 ms。
        core::utility::assert_debug(
            static_cast<size_t>(static_cast<IndexType>(next - in_)) <= kBufferSize / 2);

        in_ = next;
        return next;
    }

    // 接收错误策略, 在任何 DMA 武装之前应用一次。
    //
    //   CR3.DDRE = 0    "DMA Disable on Reception Error" 保持关闭, 奇偶/帧/
    //                   噪声错误不阻塞 DMA 请求, 可疑字节照常入 ring, 交协议层
    //                   处置。
    //   CR3.OVRDIS = 1  关闭溢出检测。start_rx_dma() 刻意不置 CR3.EIE, 置位的
    //                   ORE 将无人清除并卡死接收; 连续 DMA 排空 RDR 足够快,
    //                   该标志在这里本就无信息量。
    //
    // 极性陷阱: 位名描述的是它使能的行为, 与旁边的 OVRDIS 同理。曾误置 DDRE
    // 并注释称可保 DMA 请求存活, 实际相反: DDRE 置 1 且 EIE 清零时, 一个坏
    // 字符即永久杀死端口, DMA 请求保持阻塞直至错误标志被清, 却没有任何中断会
    // 去清它(rx_error_callback() 只在 DMA 控制器故障时触发, 线路错误从不)。
    // 实测复现并经清 DDRE 修复, 见 host/examples/uart_cross_test.cpp 的
    // monitor 模式。
    //
    // 两位均仅在 USART 禁止时才可写, 故写入前后翻转 UE。此刻 TxBuffer 的构造
    // 函数只绑定回调, 尚无任何收发在进行。
    void configure_rx_error_policy() const {
        auto* instance = hal_uart_handle_->Instance;
        const bool was_enabled = (instance->CR1 & USART_CR1_UE) != 0U;

        ATOMIC_CLEAR_BIT(instance->CR1, USART_CR1_UE);
        ATOMIC_SET_BIT(instance->CR3, USART_CR3_OVRDIS);
        ATOMIC_CLEAR_BIT(instance->CR3, USART_CR3_DDRE);
        if (was_enabled)
            ATOMIC_SET_BIT(instance->CR1, USART_CR1_UE);
    }

    // 经专用清零寄存器清粘滞 RX 状态, 不触碰 RDR。F407 清 ORE 只能走"先读 SR
    // 再读 DR"序列, 那会从 DMA 流里偷走一个字节。
    void clear_rx_error_flags() const {
        WRITE_REG(
            hal_uart_handle_->Instance->ICR,
            USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF | USART_ICR_IDLECF);
    }

    // 丢弃 RDR 与 16 项 RXFIFO 中残留的字节(深度见 DS13313 Table 5)。
    // enable_fifo_mode() 对所有端口开启 FIFO, 不清的话中止前捕获的至多 16 字节
    // 会存活下来并被当作新到数据写到 ring 偏移 0。F407 没有该请求寄存器。
    void flush_rx_fifo() const { WRITE_REG(hal_uart_handle_->Instance->RQR, USART_RQR_RXFRQ); }

    // 接收器对 RE 的应答是异步的: 仅当 ISR.REACK 读回 1 才真正在采样线路,
    // F407 无此握手。CubeMX 使能后 RE 从不清除, 故通常首读即返回。
    void wait_receiver_ready() const {
        auto* instance = hal_uart_handle_->Instance;
        if ((instance->CR1 & USART_CR1_RE) == 0U)
            return;

        for (uint32_t i = 0; i < kReceiverAckPollLimit; ++i) {
            if ((instance->ISR & USART_ISR_REACK) != 0U)
                return;
        }
        core::utility::assert_debug_lazy([]() noexcept { return false; });
    }

    void bind_rx_dma_callbacks() {
        auto* hal_dma_handle = hal_uart_handle_->hdmarx;
        core::utility::assert_debug(hal_dma_handle != nullptr);

        // 传输完成不携带任何信息: 流自行回绕, 写位置由消费者读 NDTR 获得。
        // HAL_DMA_Start_IT 仍会开这个中断, 但 HAL_DMA_IRQHandler 对回调逐个
        // 判空, 只清标志而已。每圈一次(921600 baud 下 22 ms), 不值得压制。
        hal_dma_handle->XferCpltCallback = nullptr;
        hal_dma_handle->XferM1CpltCallback = nullptr;
        hal_dma_handle->XferErrorCallback = &T::hal_rx_dma_error_callback;
        hal_dma_handle->XferHalfCpltCallback = nullptr;
        hal_dma_handle->XferM1HalfCpltCallback = nullptr;
        hal_dma_handle->XferAbortCallback = nullptr;
    }

    void start_rx_dma() {
        auto* hal_dma_handle = hal_uart_handle_->hdmarx;
        core::utility::assert_debug(hal_dma_handle->Init.Mode == DMA_CIRCULAR);

        // 按构造即为单缓冲: HAL_DMA_Init() 写 CR 前把 DBM 与 CT 屏蔽掉(见
        // bsp/stm32h7xx-hal-driver/Src/stm32h7xx_hal_dma.c 的寄存器掩码), 本类
        // 也从不设置它们, 故流只有一个目标地址、无需重指向。restart_rx_dma()
        // 里的 HAL_DMA_Abort() 不动这两位, 重启后依然成立。
        core::utility::assert_debug((rx_dma_stream()->CR & (DMA_SxCR_DBM | DMA_SxCR_CT)) == 0U);

        in_ = 0;
        out_ = 0;
        idle_count_.store(0, std::memory_order::relaxed);
        consumed_idle_count_ = 0;

        const auto source =
            static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&hal_uart_handle_->Instance->RDR));
        const auto destination = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(ring_.data()));

        core::utility::assert_always(
            HAL_DMA_Start_IT(hal_dma_handle, source, destination, kBufferSize) == HAL_OK);

        // 把端口伪装成进行中的循环 ReceiveToIdle, 使 HAL_UART_IRQHandler 在
        // IDLE 时走 DMA_CIRCULAR 分支: 事件经 HAL_UARTEx_RxEventCallback 上报,
        // 且不触碰 DMAR、不中止流。
        hal_uart_handle_->RxState = HAL_UART_STATE_BUSY_RX;
        hal_uart_handle_->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

        // RxXferSize 刻意比 ring 大 1。HAL_UART_IRQHandler 的 IDLE 分支在
        // `0 < __HAL_DMA_GET_COUNTER(hdmarx) < huart->RxXferSize` 时上报事件,
        // 而流运行期间 NDTR 取值 1..kBufferSize; 若 RxXferSize 恰为
        // kBufferSize, 刚回绕的 NDTR == kBufferSize 会落进另一条
        // `nb_remaining == RxXferSize` 分支, 多加的 1 使硬件可能给出的每个
        // NDTR 都被同一条比较覆盖。
        // 此路径上 HAL 不再读 RxXferSize: UART_DMAReceiveCplt 从不安装
        // (bind_rx_dma_callbacks 令 XferCpltCallback 为空), RxXferCount 仅由该
        // IDLE 分支写入、我方从不读取。
        hal_uart_handle_->RxXferSize = static_cast<uint16_t>(kBufferSize + 1);
        hal_uart_handle_->RxXferCount = static_cast<uint16_t>(kBufferSize);

        // 清掉所有粘滞 RX 标志与 RDR/RXFIFO 残留字节, 再确认接收器存活, 使
        // DMA 请求从真实字节边界恢复, 而非接着重启前的残渣。
        clear_rx_error_flags();
        flush_rx_fifo();
        wait_receiver_ready();

        // 刻意保持 CR3.EIE 与 CR1.PEIE 清零, 这正是必须清 CR3.DDRE(而非仅更优)
        // 的原因: 错误中断关闭时无人清除错误标志, DDRE 置位将永久阻塞 DMA 请求。
        // 另外 HAL_UART_IRQHandler() 在 DMAR 置位时把任何 RX 错误一律判为阻塞
        // (判据为 `HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR) || ...`),
        // 随即执行 UART_EndRxTransfer() + HAL_DMA_Abort_IT(): 即使开着错误中断
        // 让硬件继续跑流, HAL 也会把它中止, 在本不需要的芯片上复刻 F407 行为。
        //
        // 线路错误改由 DDRE/OVRDIS 策略吸收; 真正的 DMA 控制器故障仍经流自身的
        // 错误中断到达(hal_rx_dma_error_callback -> rx_error_callback), 这正是
        // restart_rx_dma() 存在的目的。
        ATOMIC_SET_BIT(hal_uart_handle_->Instance->CR3, USART_CR3_DMAR);
        ATOMIC_SET_BIT(hal_uart_handle_->Instance->CR1, USART_CR1_IDLEIE);
    }

    // 供 TxBuffer 的半双工 turnaround 门使用, 该计数变化即"对端已应答完毕"。
    // 用 relaxed 而非 acquire, 因为这里只关心数值变化、计数器不发布任何数据;
    // 上方 try_dequeue() 之所以用 acquire, 恰是为了把边界与 ring 内容配对。
    [[nodiscard]] uint16_t idle_count() const {
        return idle_count_.load(std::memory_order::relaxed);
    }

    void uart_idle_event_callback() {
        // 只发布一个边界: 无锁、无寄存器写。消费者随后把本计数与紧随其后采样
        // 的写指针对配。
        idle_count_.fetch_add(1, std::memory_order::release);
    }

    void rx_error_callback() {
        // 只为 DMA 控制器故障(传输/FIFO/直接模式错误)到达, 线路错误不会:
        // CR3.EIE 保持清零后, 帧/噪声/奇偶毛刺不再触发 UART 中断, 清零的 DDRE
        // 使流照常穿过错误。uart.cpp 的 HAL 错误回调在 HAL 经其他路径置上 RX
        // ErrorCode 时仍可能路由到这里。
        // 刻意不加 debug 断言: mc02 承载 DBUS 与可热插拔端口, 端口必须自行恢复
        // 而非困死 debug 构建。
        restart_rx_dma();
    }

    void restart_rx_dma() {
        auto* hal_dma_handle = hal_uart_handle_->hdmarx;

        ATOMIC_CLEAR_BIT(hal_uart_handle_->Instance->CR1, USART_CR1_IDLEIE);
        ATOMIC_CLEAR_BIT(hal_uart_handle_->Instance->CR3, USART_CR3_DMAR);

        if ((rx_dma_stream()->CR & DMA_SxCR_EN) != 0U) {
            core::utility::assert_always(HAL_DMA_Abort(hal_dma_handle) == HAL_OK);
        }

        // HAL 的阻塞错误路径会把 UART_DMAAbortOnError 装为流的 abort 回调;
        // 重新绑定, 防止之后的 HAL_DMA_Abort 绕过我们弹回 HAL 错误机制。
        bind_rx_dma_callbacks();
        start_rx_dma();
    }

    UART_HandleTypeDef* hal_uart_handle_;

    // 随外层端口对象放置, uart.hpp 把它放进 .d2_sram(0x30000000 的 D2 SRAM),
    // 与写它的 DMA1 流同域。app.cpp 经 MPU region 1 把该区间映射为
    // non-cacheable, DMA 写无需维护缓存。不可像 can.hpp 那样挪进 .dtcm:
    // DMA1/DMA2 够不到 DTCM, 流会静默地什么都传不了。
    alignas(uint32_t) std::array<std::byte, kBufferSize> ring_{};

    // 仅消费者使用。in_ 承载 NDTR 读出的 ring 偏移放不下的圈数位; 两者均由
    // start_rx_dma() 复位, 它也在 DMA 错误中断里运行, 重启即按设计丢弃消费者
    // 位置。
    IndexType in_{0};
    IndexType out_{0};

    // 由 IDLE 中断发布、try_dequeue() 消费。是计数而非位置, 与写指针的配对
    // 论证见 try_dequeue()。
    std::atomic<uint16_t> idle_count_{0};
    uint16_t consumed_idle_count_{0};

    static_assert(std::atomic<uint16_t>::is_always_lock_free);
};

} // namespace libhcs::firmware::uart
