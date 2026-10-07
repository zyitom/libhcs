#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <usart.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/spec/c_board/ports.hpp"
#include "core/src/link/port.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/led/led.hpp"
#include "firmware/c_board/app/src/uart/rx_buffer.hpp"
#include "firmware/c_board/app/src/uart/tx_buffer.hpp"
#include "firmware/c_board/app/src/usb/helper.hpp"
#include "firmware/c_board/app/src/utility/loop_work.hpp"
#include "firmware/common/app/src/utility/event_counter.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"

namespace libhcs::firmware::uart {

namespace vc = libhcs::core::protocol::vendor_control;
namespace link = libhcs::core::link;

// 端口类共享的状态与行为: 身份标识、运行时波特率控制、接收字节向 USB 的交接。
// 与 mc02 的 UartCommon 同名同职责; 缓冲在派生类 TxBuffer / RxBuffer 里(收发
// 各一份文件)。本板与 mc02 的结构差异都是硬件事实: F407 没有 TXFIFO、没有
// ICR/RQR、没有 RXINV, DMA 是 32 字节双 bank 中断推进写指针; 也没有 RS-485
// 收发器(DEM/DEP), 因此没有 mc02 的 UartRs485 / half_duplex 分支。
class UartCommon : private core::utility::Immovable {
public:
    // ---- EP0 配置通道: 运行期速率与帧格式 + 读回 ----
    //
    // 与其他板同一份契约(core vendor_control.hpp): 先全部校验、后碰寄存器, 一次
    // 控制传输 STALL 即意味着端口原封不动; 读回的是硬件实际在跑的值, 不是上次
    // 请求。这块板此前两者皆无 -- 只有一个只写的带内配置字段, 那正是迁移除掉的
    // 失败模式(字段与其 DataIds 已从协议里删除)。

    // 纯校验: 不碰任何寄存器。编码与 core::protocol::vendor_control 的
    // UartParity/UartStopBits 一致(0 = 跳过; 校验 1=无 2=偶 3=奇; 停止位 1=1 2=2);
    // 字长是数据位数(7/8; 不提供 9 -- 接收是字节宽的 DMA 环, 第 9 位会被无声截断)。
    // 不提供 1.5 停止位: STM32 只在 5 位字长下实现它, 与 2 无法区分 -- 提供别名
    // 就是说谎。接收极性只接受 0(跳过)与 1(正常): F4 的 USART 没有接收反相位,
    // 反相请求只能 STALL, 不能假装应用了。
    //
    // F4 的帧(M)是含校验位的 8 或 9 位, 所以 7 个数据位只在带校验时存在
    // (7E/7O = 8 位帧); 7N1 需要本外设没有的 7 位帧。在解析出的对上核对(0 表示
    // 沿用端口当前值), 与 commit_framing() 同一套。
    [[nodiscard]] bool check_framing(
        uint32_t word_length, uint32_t parity, uint32_t stop_bits, uint32_t rx_polarity) const {
        if (rx_polarity > 1U)
            return false;
        if (word_length != 0U && word_length != 7U && word_length != 8U)
            return false;
        if (parity != 0U && (parity < 1U || parity > 3U))
            return false;
        if (stop_bits != 0U && stop_bits != 1U && stop_bits != 2U)
            return false;
        const uint32_t data_bits = word_length != 0U ? word_length : this->word_length();
        const uint32_t parity_code = parity != 0U ? parity : this->parity();
        return data_bits != 7U || parity_code != 1U;
    }

    // 提交一份 check_framing() 已经接受的帧格式 -- 这里因此没有失败路径, "STALL
    // 即零改动"由此成为机械事实。M/PCE/PS/STOP 只能在 UE=0 时写, 所以写入是
    // 一关一开一个窗口; 窗口里到达的字节会丢, 主机应先静默链路(与速率同一约定)。
    //
    // M 数的是整帧(数据 + 校验), 8 数据位带偶校验就是 9 位帧(M=1) -- 即 CubeMX
    // 给 DBUS 配的 WORDLENGTH_9B + PARITY_EVEN。所以字长与校验一并解析: 0 表示
    // 沿用端口当前值。
    // 第四参(rx_polarity)是统一签名的一部分: 本板没有 RX 反相硬件, check_framing
    // 已拒绝过非 0/1 的值, 这里无事可做。
    void commit_framing(uint32_t word_length, uint32_t parity, uint32_t stop_bits, uint32_t) {
        const uint32_t data_bits = word_length != 0U ? word_length : this->word_length();
        const uint32_t parity_code = parity != 0U ? parity : this->parity();
        uint32_t parity_bits = 0;
        switch (parity_code) {
        case 2U: parity_bits = USART_CR1_PCE; break;                // 偶
        case 3U: parity_bits = USART_CR1_PCE | USART_CR1_PS; break; // 奇
        default: break;                                             // 无
        }
        const uint32_t frame_bits = data_bits + (parity_bits != 0U ? 1U : 0U);
        const uint32_t m_bits = frame_bits == 9U ? USART_CR1_M : 0U;
        uint32_t stop_reg = 0;
        if (stop_bits == 2U)
            stop_reg = USART_CR2_STOP_1;

        auto* instance = hal_uart_handle_->Instance;
        // new_cr1 保持 UE 原样: 在这里把它掩掉, 最后一次写会把端口留在禁用态
        // (mc02 到 2026-10-03 就是这么错的, 经 EP0 配置过的串口双向全断)。
        const uint32_t new_cr1 =
            (instance->CR1 & ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS)) | m_bits | parity_bits;

        instance->CR1 = new_cr1 & ~USART_CR1_UE; // UE=0: 帧格式字段才可写
        if (stop_bits != 0U)
            instance->CR2 = (instance->CR2 & ~USART_CR2_STOP) | stop_reg;
        instance->CR1 = new_cr1; // UE 还原
    }

    // ---- 读回: 从活寄存器解码, 不是上次请求 ----

    // 数据位 = 帧 M 给出的位数(8 或 9)减去校验位。DBUS 的 9 位帧带偶校验, 读回是 8。
    [[nodiscard]] uint32_t word_length() const {
        const uint32_t cr1 = hal_uart_handle_->Instance->CR1;
        const uint32_t frame_bits = (cr1 & USART_CR1_M) != 0U ? 9U : 8U;
        return frame_bits - ((cr1 & USART_CR1_PCE) != 0U ? 1U : 0U);
    }

    [[nodiscard]] uint32_t parity() const {
        const uint32_t cr1 = hal_uart_handle_->Instance->CR1;
        if ((cr1 & USART_CR1_PCE) == 0U)
            return 1U;                               // 无
        return (cr1 & USART_CR1_PS) != 0U ? 3U : 2U; // 奇 : 偶
    }

    [[nodiscard]] uint32_t stop_bits() const {
        return (hal_uart_handle_->Instance->CR2 & USART_CR2_STOP) == USART_CR2_STOP_1 ? 2U : 1U;
    }

    // 一个字符在线上占的位数: 起始位 + 数据 + 校验 + 停止位。
    [[nodiscard]] uint32_t bits_per_character() const {
        return 1U + word_length() + (parity() != 1U ? 1U : 0U) + stop_bits();
    }

    // 没有接收反相硬件: 恒为正常(1)。
    [[nodiscard]] static uint32_t rx_polarity() { return 1U; }

    // 端口实际在跑的速率, 从 BRR 重构而不是取 Init.BaudRate: 后者只是上次请求
    // 值, 被拒绝的请求两者都不写。读 BRR 才让主机分得清"切换已生效"与"切换被
    // 拒, 仍是旧速率"。
    [[nodiscard]] uint32_t effective_baudrate() const {
        return baudrate_for(hal_uart_handle_->Instance->BRR & 0xFFFFU);
    }

    // 给定 BRR 值在本口时钟下的速率; EP0 用它在写入前核对 solve_brr() 的结果。
    [[nodiscard]] uint32_t baudrate_for(uint32_t brr) const {
        const uint32_t kernel_clock_hz = peripheral_clock_hz();
        if (kernel_clock_hz == 0U || brr == 0U) [[unlikely]]
            return 0;
        if (hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8) {
            // 8 倍过采样把 BRR[3] 记 0、低半字节右移一位; 先还原再除。
            const uint32_t usartdiv = (brr & 0xFFF0U) | ((brr & 0x0007U) << 1U);
            return usartdiv ? (2U * kernel_clock_hz) / usartdiv : 0U;
        }
        return kernel_clock_hz / brr;
    }

    // 速率由哪两个整数构成, 供 EP0 GET 上报与 SET 断言。`divisor` 就是 BRR 本身、
    // 未归一化 -- commit_brr()/compute_brr() 写下的那个整数, 读回比对不需要换算。
    // 见 UartDivisor。
    [[nodiscard]] uint16_t divisor_u16() const {
        return static_cast<uint16_t>(hal_uart_handle_->Instance->BRR & 0xFFFFU);
    }

    [[nodiscard]] uint8_t oversample() const {
        return hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8 ? 8U : 16U;
    }

    // 只解分频值不写: "先校验后提交"对子的校验一半。compute_brr() 是同一套算术,
    // 这里返回的值就是 apply_baudrate() 将要写入的。
    [[nodiscard]] bool solve_brr(uint32_t baudrate, uint32_t& brr) const {
        if (baudrate == 0U) [[unlikely]]
            return false;
        if (peripheral_clock_hz() == 0U) [[unlikely]]
            return false;
        brr = compute_brr(baudrate);
        // 与其他板同一条边界: 分频值为 0 或超出寄存器位宽, 表示本口表示不了
        // 这个速率。
        return brr >= 0x10U && brr <= 0xFFFFU;
    }

    // 提交一半。直写分频寄存器而不重跑 HAL_UART_Init: 其余设置保持 CubeMX 生成
    // 时的原样(这段代码绝不能背着 .ioc 重配外设), 也不动正在跑的 DMA。
    void commit_brr(uint32_t baudrate, uint32_t brr) {
        hal_uart_handle_->Init.BaudRate = baudrate;
        hal_uart_handle_->Instance->BRR = brr;
    }

    // 写后回读: 确认寄存器吃下了这次写入。比对的是分频整数不是波特率 -- 主机的
    // 内核时钟与本板不同, 速率在主机侧无法重算。见 UartDivisor。
    [[nodiscard]] bool
        verify_baudrate(uint16_t expected_divisor, uint8_t expected_oversample) const {
        return divisor_u16() == expected_divisor && oversample() == expected_oversample;
    }

    // ---- 端口接口(core/src/link/ 的通用 UART 操作按这一组原语工作) ----
    //
    // 全部是冷路径(EP0 的清单声明与读回)。solve/commit_baudrate 是 compute_brr/
    // commit_brr 的统一签名视图: 核心流程对三块板写同一份代码, BRR 的编码差异留在
    // 各自的求解器里。
    // 没声明的串口不工作: 接收器关着、接收 DMA 不武装(RxBuffer::stop_rx), 主机写来的字节
    // 不发; 上电与会话结束时挂起, 声明了才 resume。发送环里已收下的字节照常发完。只在主
    // 循环读写。
    [[nodiscard]] bool running() const { return !suspended_; }

    [[nodiscard]] bool solve(uint32_t baudrate, uint16_t& divisor, uint8_t& oversample) const {
        uint32_t brr = 0;
        if (!solve_brr(baudrate, brr))
            return false;
        divisor = static_cast<uint16_t>(brr);
        oversample = hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8 ? 8U : 16U;
        return true;
    }

    [[nodiscard]] bool commit_baudrate(uint32_t baudrate, uint16_t divisor, uint8_t) {
        commit_brr(baudrate, divisor);
        return true; // commit_brr 无失败路径: 校验已在 solve 阶段完成
    }

    [[nodiscard]] uint32_t baudrate_for(uint16_t divisor, uint8_t) const {
        return baudrate_for(static_cast<uint32_t>(divisor));
    }

    [[nodiscard]] bool framing_matches(
        uint32_t word_length, uint32_t parity, uint32_t stop_bits, uint32_t rx_polarity) const {
        return (word_length == 0U || word_length == this->word_length())
            && (parity == 0U || parity == this->parity())
            && (stop_bits == 0U || stop_bits == this->stop_bits())
            && (rx_polarity == 0U || rx_polarity == this->rx_polarity());
    }

    // kGetPortConfig 的应答: 硬件事实而非"上次请求"。波特率从 BRR 重构, 帧格式从
    // 活寄存器解码, control 恒为 0。divisor/oversample 一并上报 -- 见 UartDivisor。
    void read_config(vc::UartConfigPayload& out) const {
        out = {
            .baudrate = effective_baudrate(),
            .divisor = divisor_u16(),
            .oversample = oversample(),
            .word_length = static_cast<uint8_t>(word_length()),
            .parity = static_cast<uint8_t>(parity()),
            .stop_bits = static_cast<uint8_t>(stop_bits()),
            .control = 0,
            .rx_polarity = static_cast<uint8_t>(rx_polarity()),
        };
    }

    [[nodiscard]] core::link::PortStatus describe() const {
        return {.running = running(), .fd = false};
    }

    // 端口身份(丝印号)。ports.hpp 注册表在上电时核对: EP0 寻址本驱动用的就是
    // 这个身份。
    [[nodiscard]] data::DataId data_id() const { return data_id_; }

    // 运行时状态(core/src/link/port_status.hpp), 每个 keepalive 轮次在主循环读一次。
    // 硬件错误按 HAL 错误回调计(见 note_rx_errors()), 每次回调每一位各记一次。
    [[nodiscard]] data::UartStatusView read_status() const {
        return {
            .overrun = overrun_.count(),
            .parity = parity_errors_.count(),
            .framing = framing_errors_.count(),
            .noise = noise_errors_.count(),
            .tx_dropped = tx_dropped_.count(),
            .rx_dropped = rx_dropped_.count(),
        };
    }

protected:
    UartCommon(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle)
        : data_id_(data_id)
        , hal_uart_handle_(hal_uart_handle) {}

    // RxBuffer 与 TxBuffer 各存一份句柄, 派生端口无法无限定地访问
    // hal_uart_handle_; 把所需的两处访问经此路由, 避免到处写 UartCommon:: 限定。
    //
    // HAL 错误回调(中断上下文)的第一步: 按 ErrorCode 的位各记一次, 返回是否有接收
    // 错误。HAL 对非阻塞错误(PE/NE/FE)回调后即清 ErrorCode, 阻塞错误(ORE)由
    // rx_error_callback() 重启接收时清, 所以每次回调看到的都是新发生的错误。
    [[nodiscard]] bool note_rx_errors() {
        const uint32_t code = hal_uart_handle_->ErrorCode;
        if ((code & HAL_UART_ERROR_ORE) != 0U)
            overrun_.note();
        if ((code & HAL_UART_ERROR_PE) != 0U)
            parity_errors_.note();
        if ((code & HAL_UART_ERROR_FE) != 0U)
            framing_errors_.note();
        if ((code & HAL_UART_ERROR_NE) != 0U)
            noise_errors_.note();
        constexpr uint32_t rx_error_mask =
            HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE;
        return (code & rx_error_mask) != 0U;
    }

    void flag_dma_error() { hal_uart_handle_->ErrorCode |= HAL_UART_ERROR_DMA; }

    void handle_uplink(
        std::span<const std::byte> payload, std::span<const std::byte> payload2, bool is_idle) {
        if (suspended_) [[unlikely]]
            return; // 挂起那一刻环里还没转发的字节: 到此为止
        auto& serializer = usb::get_serializer();
        const auto result = serializer.write_uart(
            data_id_, {.uart_data = payload, .idle_delimited = is_idle}, payload2);
        if (result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]]
            rx_dropped_.note();
        core::utility::assert_always(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    // STM32F407: USART1 与 USART6 在 APB2 上, 其余 U(S)ART 在 APB1 上。
    [[nodiscard]] uint32_t peripheral_clock_hz() const {
        if (hal_uart_handle_ == &huart1 || hal_uart_handle_ == &huart6)
            return HAL_RCC_GetPCLK2Freq();
        return HAL_RCC_GetPCLK1Freq();
    }

    data::DataId data_id_;
    UART_HandleTypeDef* hal_uart_handle_;
    // 见 running()。上电挂起, 清单声明了才 resume。
    bool suspended_ = true;

    // 运行时状态的计数(read_status())。接收错误与上行丢弃在中断里记, 下行丢弃在
    // 主循环里记; 各自单写者。
    utility::EventCounter overrun_;
    utility::EventCounter parity_errors_;
    utility::EventCounter framing_errors_;
    utility::EventCounter noise_errors_;
    utility::EventCounter tx_dropped_;
    utility::EventCounter rx_dropped_;

private:
    [[nodiscard]] uint32_t compute_brr(uint32_t baudrate) const {
        const uint32_t peripheral_clock_hz_value = peripheral_clock_hz();
        if (hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8)
            return UART_BRR_SAMPLING8(peripheral_clock_hz_value, baudrate);
        return UART_BRR_SAMPLING16(peripheral_clock_hz_value, baudrate);
    }
};

// 全双工端口: UART1(huart6)、UART2(huart1)与 DBUS(huart3)。DBUS 只承载接收机
// 的上行流, 下行在 vendor.cpp 的分发处被拒(协议不往 kUartDbus 路由), 但它的
// TX DMA 流在 CubeMX 里是接好的, 端口类型因此与另两个口相同。
class Uart; // 前置: UartDmaMemory 的字段大小引用 RxBuffer<Uart> 的常量。

// 一口一份的 DMA 目标缓冲(RX 环、TX 环、TX 回绕暂存), 独立全局对象而非控制器对象
// 的成员: 段属性不能放在非静态数据成员上(GCC 报 "section attribute not allowed"),
// 与 mc02 的 Uart::DmaMemory 同一结构。独立成对象把"哪些字节绕开 D-cache"圈在
// .dmaram 一处(非缓存区, 见 app.cpp 的 configure_dmaram_mpu_region()), 控制状态
// 照常走缓存。
struct UartDmaMemory {
    alignas(uint32_t) std::array<std::byte, RxBuffer<Uart>::kBufferSize> rx_ring{};
    alignas(uint32_t) std::array<std::byte, TxBuffer::kBufferSize> tx_ring{};
    alignas(uint32_t) std::array<std::byte, TxBuffer::kStagingBufferSize> tx_staging{};
};
static_assert(
    sizeof(UartDmaMemory)
    == RxBuffer<Uart>::kBufferSize + TxBuffer::kBufferSize + TxBuffer::kStagingBufferSize);

class Uart
    : public UartCommon
    , private TxBuffer
    , private RxBuffer<Uart> {
    friend class RxBuffer<Uart>;

public:
    // EP0 口能力: 无(F4 的 USART 没有接收反相位)。
    static constexpr uint8_t kPortCapabilities = 0;

    using Lazy = utility::Lazy<Uart, data::DataId, UART_HandleTypeDef*, UartDmaMemory*>;

    // 参数是指针而非引用: Lazy 的构造参数落在 std::tuple 里, libstdc++ 15 的 tuple
    // 转换构造对引用成员不可用(consteval 下报 no matching function), 指针与 Lazy 里
    // 其余的 &huart6 形态一致。
    Uart(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle, UartDmaMemory* dma_memory)
        : UartCommon(data_id, hal_uart_handle)
        , TxBuffer(
              hal_uart_handle, &hal_tx_dma_complete_callback, &hal_tx_dma_error_callback,
              dma_memory->tx_ring, dma_memory->tx_staging)
        , RxBuffer(hal_uart_handle, dma_memory->rx_ring) {
        update_line_rate(); // CubeMX 的初始速率与帧格式
    }

    // 速率与帧格式只在这两处变(EP0 清单应用): 写完按新的字符时间重算发送侧的包间
    // 空隙。遮住 UartCommon 的同名原语, core 的 uart_apply() 经端口类型调到这里。
    [[nodiscard]] bool commit_baudrate(uint32_t baudrate, uint16_t divisor, uint8_t oversample) {
        const bool committed = UartCommon::commit_baudrate(baudrate, divisor, oversample);
        update_line_rate();
        return committed;
    }

    void commit_framing(
        uint32_t word_length, uint32_t parity, uint32_t stop_bits, uint32_t rx_polarity) {
        UartCommon::commit_framing(word_length, parity, stop_bits, rx_polarity);
        update_line_rate();
    }

    void handle_downlink(const data::UartDataView& data) {
        if (suspended_) [[unlikely]]
            return; // 没声明的口不发(没声明的口的最后一道)
        if (!TxBuffer::try_enqueue(data)) {
            tx_dropped_.note();
            led::led->downlink_buffer_full();
        }
    }

    // 见 UartCommon::running()。启动即在 loop::active 里置本口的位(位号 = DataId), 主循环
    // 从下一圈起轮询它; 清位由 ports.hpp 的 poll_uarts() 在口停下且发完之后做。与 mc02 同法。
    void suspend() {
        if (suspended_)
            return;
        suspended_ = true;
        RxBuffer::stop_rx();
        draining_ = !TxBuffer::drained();
    }
    void resume() {
        if (!suspended_)
            return;
        RxBuffer::start_rx();
        suspended_ = false;
        loop::set(loop::bit(data_id_));
    }

    // @return 这一口之后还要不要主循环来轮询(见 ports.hpp 的 poll_uarts())
    bool try_transmit() {
        if (!suspended_) {
            RxBuffer::try_dequeue();
            TxBuffer::try_dequeue();
            return true;
        }
        if (draining_) {
            // 停口之前已经收下的字节照常发完, 发完就不再轮询。
            TxBuffer::try_dequeue();
            draining_ = !TxBuffer::drained();
        }
        return draining_;
    }

    void tx_complete_callback() { TxBuffer::tx_complete_callback(); }

    void uart_error_callback() {
        if (note_rx_errors())
            RxBuffer::rx_error_callback();
    }

    void rx_dma_tc_callback() { RxBuffer::dma_tc_callback(); }

    void rx_dma_error_callback() {
        // DMA 控制器故障不该发生(线路错误走 uart_error_callback): 调试构建在此停住。
        core::utility::assert_debug_lazy([]() noexcept { return false; });
        flag_dma_error();
        RxBuffer::rx_error_callback();
    }

    void tx_dma_error_callback() { TxBuffer::tx_error_callback(); }

    void rx_event_callback() { RxBuffer::uart_idle_event_callback(); }

private:
    static void hal_rx_dma_tc_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_complete_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    // 速率与帧格式只经 commit_brr()/commit_framing() 改变(加上上电时的 CubeMX):
    // 从它们重算发送侧的包间隙。
    void update_line_rate() { TxBuffer::set_line_rate(effective_baudrate(), bits_per_character()); }

    // 停口时发送环里还有字节: 主循环继续轮询到发完(try_transmit())。只在主循环读写。
    bool draining_ = false;
};

// DMA 目标缓冲(.dmaram, 上电时由 App::App() 拷入): 每口一份, 与下面的 Lazy 一同列出。
[[gnu::section(".dmaram")]] inline constinit UartDmaMemory uart1_dma_memory{};
[[gnu::section(".dmaram")]] inline constinit UartDmaMemory uart2_dma_memory{};
[[gnu::section(".dmaram")]] inline constinit UartDmaMemory uart_dbus_dma_memory{};

inline constinit Uart::Lazy uart1{
    spec::c_board::Spec::Uarts::kUart1.data_id, &huart6, &uart1_dma_memory};
inline constinit Uart::Lazy uart2{
    spec::c_board::Spec::Uarts::kUart2.data_id, &huart1, &uart2_dma_memory};
inline constinit Uart::Lazy uart_dbus{
    spec::c_board::Spec::Uarts::kDbus.data_id, &huart3, &uart_dbus_dma_memory};

} // namespace libhcs::firmware::uart
