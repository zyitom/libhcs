#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

#include <main.h>
#include <usart.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/mc02/app/src/timer/timer.hpp"
#include "firmware/mc02/app/src/utility/ring_buffer.hpp"

namespace libhcs::firmware::uart {

// 带 idle 边界保持的环形 DMA 发送。ring 缓存下行包; 被标为 idle_delimited 的
// 包会在其边界记录检查点, try_dequeue 在线路静默满 300 us 前拒绝越过检查点
// 发送, 以保住按 idle 定界的设备所依赖的帧结构。
//
// half_duplex 区分独占总线的口与 RS-485 双线共享总线的口; 本板仅此一种区别,
// 且为 RS-485 固有属性, 故用裸 bool 而非策略类型。它守护的分支都在
// `if constexpr` 之后, TxBuffer<false> 的编译结果与没有这个参数时完全一致:
// 零额外测试、加载与存储, try_dequeue() 也保持无参形式。三个尺寸默认按连续流
// 端口取值, 只跑短请求/响应事务的端口用小得多即可; 每个字节都从 32 KB 区域里
// 扣除, 两个 RS-485 口的取值见 uart.hpp 末尾。
//
// 调尺寸时注意 staging_size: try_dequeue() 把每段钳制到它, 超过它的包会拆成
// 两次 DMA 突发、中间隔着主循环间隙。对流不可见, 但在半双工总线上会在帧中间
// 插入静默、破坏对端定界。令 staging_size == buffer_size 可杜绝此问题: 装得进
// ring 的包必能一次突发发完。
template <
    bool half_duplex, size_t buffer_size = 2048, size_t staging_size = 1024,
    size_t checkpoint_count = 256>
class TxBuffer {
public:
    // 半双工端口等待应答而保持静默的上限。
    //
    // RS-485 没有任何仲裁: 驱动器为推挽输出, 不像 CAN 的线与那样能让落败方
    // 退让, 两节点同时发送只会互相摧毁数据; 本口也无法先听后说, 收发器 RE# 与
    // DE 相连, 驱动的全程即失聪。防碰撞只能靠主节点记账: 发出请求后保持静默,
    // 直到被寻址节点应答。
    //
    // 取值刻意宽松, 因为它只管辖永远等不到应答的情形: 节点缺席、帧被拒、广播
    // 地址(此类协议通常规定广播不应答, 正因为 N 个节点同时应答会碰撞)。对端
    // 只要应答, IDLE 事件即提前放行, 永远到不了这个值: 取大在工作路径上零
    // 代价, 取小则会碰撞。half_duplex 为 false 时不使用。
    static constexpr auto kTurnaroundDeadline = std::chrono::microseconds(1000);

    static constexpr size_t kBufferSize = buffer_size;
    static constexpr size_t kBufferMask = kBufferSize - 1;
    static_assert((kBufferSize & (kBufferSize - 1)) == 0);
    using IndexType = uint16_t;
    static_assert(kBufferSize <= std::numeric_limits<IndexType>::max());

    static constexpr size_t kStagingBufferSize = staging_size;
    static_assert(kStagingBufferSize <= std::numeric_limits<IndexType>::max());
    static_assert(kStagingBufferSize <= kBufferSize);
    // 半双工端口绝不能拆包, staging 缓冲须覆盖 ring 能装下的任何包。见模板
    // 上方说明。
    static_assert(!half_duplex || kStagingBufferSize == kBufferSize);

    static constexpr size_t kMaxIdleCheckpointCount = checkpoint_count;
    static_assert((kMaxIdleCheckpointCount & (kMaxIdleCheckpointCount - 1)) == 0);

    explicit TxBuffer(
        UART_HandleTypeDef* hal_uart_handle, void (*dma_complete_callback)(DMA_HandleTypeDef*),
        void (*dma_error_callback)(DMA_HandleTypeDef*))
        : hal_uart_handle_(hal_uart_handle)
        , dma_complete_callback_(dma_complete_callback)
        , dma_error_callback_(dma_error_callback) {
        core::utility::assert_always(hal_uart_handle_ != nullptr);
        // 实例化本类的端口必须有 TX DMA 流; UART5 (DBUS)没有, 故它使用
        // UartRxOnly。
        core::utility::assert_always(tx_dma_handle() != nullptr);
        bind_tx_dma_callbacks();
    }

    bool try_enqueue(const data::UartDataView& data_view) {
        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);

        const auto size = data_view.uart_data.size();
        const auto writable = kBufferSize - static_cast<size_t>(static_cast<IndexType>(in - out));
        if (size > writable)
            return false;

        const auto offset = in & kBufferMask;

        // 半双工端口上每个入队包就是一次总线事务, 无论主机是否要求都成为边界。
        // 此事不能托付给 idle_delimited: 它表示"该设备按静默定界", 对带同步头、
        // 定长与 CRC 的协议(如 Unitree)不成立, 驱动这类设备的主机完全有理由不
        // 置位。若边界取决于它, 三个排队的命令会作为一段连续 DMA 突发发出,
        // 第一个节点还在应答时第二个已上线, 两线共享意味着两帧俱毁。turnaround
        // 是端口属性而非逐包请求, 故在此决定。
        const bool delimited = half_duplex || data_view.idle_delimited;

        if (delimited) {
            const auto begin_boundary = in;
            const auto end_boundary = static_cast<IndexType>(in + static_cast<IndexType>(size));

            // 优化: 复用当前生产者位置已有的逻辑 idle 边界。
            if (idle_boundary_before_in_) {
                if (size) {
                    // 非空: 只追加新的 end 边界。
                    if (!idle_checkpoints_.push_back(end_boundary))
                        return false;
                }
                // ZLP (size==0): 既有检查点已强制 idle 等待。
            } else {
                if (size) {
                    // 非空: 原子压入 [begin, end], 保证两侧隔离。
                    if (idle_checkpoints_.push_back_n(
                            [&, index = 0]() mutable noexcept {
                                return (index++ == 0) ? begin_boundary : end_boundary;
                            },
                            2, true)
                        != 2) {
                        return false;
                    }
                } else {
                    // ZLP: begin == end, 压入单个检查点以强制一次 IDLE 等待。
                    if (!idle_checkpoints_.push_back(begin_boundary))
                        return false;
                }
            }
        }

        if (size) {
            const auto slice = std::min(size, kBufferSize - offset);
            const bool wrapped = size != slice;
            if (wrapped)
                trailing_boundary_segmentable_ = !delimited;

            std::memcpy(ring_buffer_.data() + offset, data_view.uart_data.data(), slice);
            std::memcpy(ring_buffer_.data(), data_view.uart_data.data() + slice, size - slice);

            in_.store(
                static_cast<IndexType>(in + static_cast<IndexType>(size)),
                std::memory_order::release);

            idle_boundary_before_in_ = delimited;
        } else {
            // 零长非 idle 包不应清除既有边界。
            idle_boundary_before_in_ |= delimited;
        }

        return true;
    }

    // peer_idle_count 是 RxBuffer 的 IDLE 计数, 仅半双工实例化读取: 所属端口
    // 在自己的 `if constexpr` 之下传参, 全双工路径连其背后的原子加载都不会
    // 执行。默认参数让全双工路径保持无参调用形式, 该参数在彼处不被引用, 内联
    // 后消失。
    bool try_dequeue(uint16_t peer_idle_count = 0) {
        if (is_busy_.load(std::memory_order::acquire))
            return false;

        // DMA 完成只代表最后一个字节到达 TDR, 在 STM32H7 上并非发送结束: 所有
        // 端口都开了 FIFO 模式(见 RxBuffer::enable_fifo_mode, 该决定为何放在
        // 驱动里即由此而来), TXFIFO 深 16(DS13313 Table 5), 其后还可压着至多
        // 16 字节, 921600 baud 下约 173 us, 与它所供的 300 us idle 窗口同量级。
        // 按 DMA 完成计时会把仍在发送判成线路空闲, 而下方检查点的全部意义正是
        // 让 idle 定界包保持分隔。
        //
        // ISR.TC 才是真正报告"TXFIFO 已排空且最后停止位已发出"的标志;
        // start_tx_dma() 在武装前清 TCF, 故此处置位的 TC 必属刚完成的传输。
        // F407 没有 TXFIFO, 勿照搬 c_board 按 DMA 完成计时的做法。
        if (awaiting_line_completion_ && (hal_uart_handle_->Instance->ISR & USART_ISR_TC) != 0U) {
            awaiting_line_completion_ = false;
            tx_complete_timepoint_ = timer::timer->timepoint();
        }

        // 只有 idle 窗口需要等线路排空; 把下一段排在未满的 TXFIFO 之后无害且
        // 保吞吐, 故 dequeue 本身不受 TC 门控。
        if (!is_idle_ && !awaiting_line_completion_) {
            if constexpr (half_duplex) {
                // 这是总线 turnaround, 不是帧间隔。对端升起 IDLE 即其应答完毕、
                // 双线重新空闲, 一发生就放行, 仅当根本不会有应答时才退回超时。
                // 取两者较早者, 超时才能放心取大: 只管辖广播与节点缺席情形,
                // 绝不拖延正常交互。
                //
                // IDLE 计数来自本口本就要开的硬件: CR1.IDLEIE 已为 RxBuffer 的
                // 定界而使能, 故零额外中断、零额外寄存器访问, 也无需知道对端
                // 说了什么。
                is_idle_ = peer_idle_count != peer_idle_count_at_tx_
                        || timer::timer->check_expired(tx_complete_timepoint_, kTurnaroundDeadline);
            } else {
                is_idle_ = timer::timer->check_expired(
                    tx_complete_timepoint_, std::chrono::microseconds(300));
            }
        }

        core::utility::assert_debug_lazy(
            [&]() noexcept { return (tx_dma_stream()->CR & DMA_SxCR_EN) == 0U; });

        auto out = out_.load(std::memory_order::relaxed);
        if (in_flight_) {
            // 直接对 ring 做 DMA 时, 须等上一次 DMA 结束后才能推进 out_。
            out = static_cast<IndexType>(out + in_flight_);
            out_.store(out, std::memory_order::release);
            in_flight_ = 0;
        }

        const auto in = in_.load(std::memory_order::acquire);
        const auto readable = static_cast<size_t>(static_cast<IndexType>(in - out));
        if (!readable)
            return false;

        size_t size;
        do {
            size = readable;
            if (auto* idle = idle_checkpoints_.peek_front()) {
                const auto distance = static_cast<size_t>(static_cast<IndexType>(*idle - out));
                core::utility::assert_debug(distance <= readable);
                size = distance;
            }
            size = std::min(size, kStagingBufferSize);

            if (size)
                break;

            // size==0 表示 out 恰好停在检查点边界上; 保持该边界直到要求的
            // idle 窗口届满。
            if (!is_idle_)
                return false;

            idle_checkpoints_.pop_front([](const IndexType&) noexcept {});
        } while (true);
        is_idle_ = false;
        is_busy_.store(true, std::memory_order::relaxed);
        // turnaround 门的基线, 取在端口承诺发送之时而非发送结束之时。两者之间
        // 不会漏数: DE 随起始位拉高且 RE# 与之相连, 整个传输期间接收器失聪,
        // 不会计入 IDLE。在此取基线还能让 tx_error_callback() 提前终结传输时
        // 保持正确: 那条路径运行在 DMA 错误中断里, 无从记录对端计数; 若沿用
        // 上次交互的旧基线, 下一次轮询会把陈旧 IDLE 误当作本次应答, 在对端仍在
        // 驱动时放行总线。
        if constexpr (half_duplex)
            peer_idle_count_at_tx_ = peer_idle_count;

        const auto offset = out & kBufferMask;
        const auto slice = std::min(size, kBufferSize - offset);
        const bool wrapped = size != slice;

        if (wrapped && !trailing_boundary_segmentable_) {
            // 严格包跨回绕必须连续: 摊平进 staging, 一次 DMA 发出。
            std::memcpy(staging_buffer_.data(), ring_buffer_.data() + offset, slice);
            std::memcpy(staging_buffer_.data() + slice, ring_buffer_.data(), size - slice);
            out = static_cast<IndexType>(out + static_cast<IndexType>(size));
            out_.store(out, std::memory_order::release);

            start_tx_dma(
                reinterpret_cast<const uint8_t*>(staging_buffer_.data()),
                static_cast<uint16_t>(size));
            return true;
        }

        // 非严格路径可直接从 ring 流式发送, 完成时再提交进度。
        start_tx_dma(
            reinterpret_cast<const uint8_t*>(ring_buffer_.data() + offset),
            static_cast<uint16_t>(slice));
        in_flight_ = static_cast<IndexType>(slice);

        return true;
    }

    void tx_complete_callback() {
        // DMA 把最后一字节写入 UART TDR 后, 先停掉来自 UART 的 DMA 请求。
        ATOMIC_CLEAR_BIT(hal_uart_handle_->Instance->CR3, USART_CR3_DMAT);
        // 临时时间戳; try_dequeue() 会以 ISR.TC 报告线路真正排空的时刻覆盖它。
        // 两次写入都由下方 release 存储发布, 并在对应的 acquire 加载之后读取。
        tx_complete_timepoint_ = timer::timer->timepoint();
        awaiting_line_completion_ = true;
        is_busy_.store(false, std::memory_order::release);
    }

    void tx_error_callback() {
        // 顺序重要: 先停 DMA 请求, 避免清零与下方 flush 之间再有字节填入
        // TXFIFO。
        ATOMIC_CLEAR_BIT(hal_uart_handle_->Instance->CR3, USART_CR3_DMAT);
        flush_tx_fifo();
        core::utility::assert_debug_lazy([]() noexcept { return false; });
        tx_complete_timepoint_ = timer::timer->timepoint();
        // 传输被中途放弃, TC 在此不携带有效边界; 退而从放弃时刻起算 idle 窗口,
        // 不冒险等待一个可能描述残帧的标志。
        awaiting_line_completion_ = false;
        is_busy_.store(false, std::memory_order::release);
    }

private:
    DMA_HandleTypeDef* tx_dma_handle() const { return hal_uart_handle_->hdmatx; }

    // 见 rx_buffer.hpp 的对应注释: STM32H7 的 DMA 句柄 Instance 是 void*,
    // 寄存器访问需要显式转换。
    [[nodiscard]] DMA_Stream_TypeDef* tx_dma_stream() const {
        return static_cast<DMA_Stream_TypeDef*>(tx_dma_handle()->Instance);
    }

    void bind_tx_dma_callbacks() {
        auto* dma = tx_dma_handle();
        dma->XferCpltCallback = dma_complete_callback_;
        dma->XferErrorCallback = dma_error_callback_;
        dma->XferHalfCpltCallback = nullptr;
        dma->XferAbortCallback = nullptr;
    }

    // 丢弃被放弃的传输遗留在 16 项 TXFIFO 中的字节, 与
    // RxBuffer::flush_rx_fifo() 对应。TX DMA 出错只是中途停下, 已交给外设的
    // 字节仍在队列里, 清 CR3.DMAT 只挡住新请求。不清的话它们会先于下一个包
    // 发出, 按 idle 定界的下游设备将看到死帧尾巴粘着活帧头部, 而上方为分隔帧
    // 而设的检查点机制对此无从知晓。
    // F407 没有该请求寄存器, 勿照搬 c_board。enable_fifo_mode() 现对所有端口
    // 开 FIFO, 故此清理适用于每个端口。
    void flush_tx_fifo() const { WRITE_REG(hal_uart_handle_->Instance->RQR, USART_RQR_TXFRQ); }

    void start_tx_dma(const uint8_t* data, uint16_t size) {
        auto* dma = tx_dma_handle();
        bind_tx_dma_callbacks();

        core::utility::assert_always(
            HAL_DMA_Start_IT(
                dma, reinterpret_cast<uint32_t>(data),
                reinterpret_cast<uint32_t>(&hal_uart_handle_->Instance->TDR), size)
            == HAL_OK);

        // STM32H7 经 ICR 清状态, 故用 ICR 位名而非 F4 的 ISR 位名; 两者同在
        // bit 6, 但 ICR 位名才真正描述这次写操作。
        __HAL_UART_CLEAR_FLAG(hal_uart_handle_, UART_CLEAR_TCF);
        ATOMIC_SET_BIT(hal_uart_handle_->Instance->CR3, USART_CR3_DMAT);
    }

    UART_HandleTypeDef* hal_uart_handle_;
    void (*dma_complete_callback_)(DMA_HandleTypeDef*);
    void (*dma_error_callback_)(DMA_HandleTypeDef*);

    // 两个缓冲都在端口对象内, uart.hpp 把它放进 .d2_sram(0x30000000 的 D2
    // SRAM, 由 MPU region 1 (app.cpp)映射为 non-cacheable), DMA 无需维护缓存
    // 即可看到这些写入。
    alignas(uint32_t) std::array<std::byte, kBufferSize> ring_buffer_{};
    alignas(uint32_t) std::array<std::byte, kStagingBufferSize> staging_buffer_{};

    std::atomic<IndexType> in_{0};
    std::atomic<IndexType> out_{0};
    static_assert(std::atomic<IndexType>::is_always_lock_free);

    IndexType in_flight_{0};
    std::atomic<bool> is_busy_{false};

    // 仅生产者使用: 当前 in_ 位置是否已关联逻辑 idle 边界。
    bool idle_boundary_before_in_{false};
    bool trailing_boundary_segmentable_{false};
    bool is_idle_{true};
    // 由 DMA 完成中断置位, try_dequeue() 在 ISR.TC 确认 TXFIFO 与移位寄存器
    // 已空后清除。见彼处注释。
    bool awaiting_line_completion_{false};
    // 本端口发送离线那一刻 RxBuffer 的 IDLE 计数。仅半双工实例化存有内容,
    // 全双工改用空类型, 配合 [[no_unique_address]] 不占任何存储, 端口对象大小
    // 保持不变。触及该成员的语句都在 `if constexpr (half_duplex)` 之后, 空类型
    // 永不被赋值或比较。
    struct Unused {};
    [[no_unique_address]] std::conditional_t<half_duplex, uint16_t, Unused>
        peer_idle_count_at_tx_{};
    timer::Timer::TimePoint tx_complete_timepoint_{timer::Timer::TimePoint::min()};

    utility::RingBuffer<IndexType, kMaxIdleCheckpointCount> idle_checkpoints_;
};

} // namespace libhcs::firmware::uart
