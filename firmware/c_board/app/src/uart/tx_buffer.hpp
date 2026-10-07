#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#include <main.h>
#include <usart.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/common/app/src/utility/ring_buffer.hpp"

namespace libhcs::firmware::uart {

// characters 个字符在线上占的时间, 向上取整到定时器 tick。一个字符 = 起始位 + 数据位
// + 校验位 + 停止位(bits_per_character)。纯算术, 不碰寄存器。
[[nodiscard]] constexpr timer::Timer::Duration
    line_time(uint32_t characters, uint32_t bits_per_character, uint32_t baudrate) {
    const uint64_t bits = uint64_t{characters} * bits_per_character;
    return timer::Timer::Duration{
        static_cast<uint32_t>((bits * timer::Timer::kClockFrequency + baudrate - 1U) / baudrate)};
}
static_assert(line_time(4, 10, 921'600).count() == 174);  // 8N1: 43.4 us
static_assert(line_time(4, 10, 9'600).count() == 16'667); // 8N1: 4.17 ms

class TxBuffer {
public:
    // idle 定界包之后线路至少静默两个字符时间: 对端按"一个字符时间没有起始位"判帧尾,
    // 两个留一倍余量。旧的固定 300 us 在 9600 下不到一个字符(对端分不出帧), 在 921600
    // 下多等约 27 个字符。计时从 DMA 完成起算, 而 F407 的 DMA 完成只是最后一个字节进了
    // DR -- 移位寄存器里还有一个、DR 里还有一个, 至多两个字符仍在线上(没有 mc02 那样的
    // ISR.TC 判读, 见 tx_complete_callback), 所以从 DMA 完成算要等 2 + 2 个字符。
    static constexpr uint32_t kIdleGapCharacters = 2 + 2;

    static constexpr size_t kBufferSize = 2048;
    static constexpr size_t kBufferMask = kBufferSize - 1;
    static_assert((kBufferSize & (kBufferSize - 1)) == 0);
    using IndexType = uint16_t;
    static_assert(kBufferSize <= std::numeric_limits<IndexType>::max());

    static constexpr size_t kStagingBufferSize = 1024;
    static_assert(kStagingBufferSize <= std::numeric_limits<IndexType>::max());

    static constexpr size_t kMaxIdleCheckpointCount = 256;
    static_assert((kMaxIdleCheckpointCount & (kMaxIdleCheckpointCount - 1)) == 0);

    explicit TxBuffer(
        UART_HandleTypeDef* hal_uart_handle, void (*dma_complete_callback)(DMA_HandleTypeDef*),
        void (*dma_error_callback)(DMA_HandleTypeDef*), std::array<std::byte, kBufferSize>& tx_ring,
        std::array<std::byte, kStagingBufferSize>& tx_staging)
        : hal_uart_handle_(hal_uart_handle)
        , dma_complete_callback_(dma_complete_callback)
        , dma_error_callback_(dma_error_callback)
        , ring_buffer_(tx_ring)
        , staging_buffer_(tx_staging) {
        core::utility::assert_always(hal_uart_handle_ != nullptr);
        core::utility::assert_always(tx_dma_handle() != nullptr);
        bind_tx_dma_callbacks();
    }

    // 端口的速率或帧格式变了(EP0 清单提交时、构造时): 按新的字符时间重算包间空隙。
    // 冷路径; 每趟主循环只读存下的结果。baudrate 为 0 说明内核时钟没解出来, 保持原值。
    void set_line_rate(uint32_t baudrate, uint32_t bits_per_character) {
        if (baudrate != 0U) [[likely]]
            idle_gap_ = line_time(kIdleGapCharacters, bits_per_character, baudrate);
    }

    bool try_enqueue(const data::UartDataView& data_view) {
        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);

        const auto size = data_view.uart_data.size();
        const auto writable = kBufferSize - static_cast<size_t>(static_cast<IndexType>(in - out));
        if (size > writable)
            return false;

        const auto offset = in & kBufferMask;

        if (data_view.idle_delimited) {
            const auto begin_boundary = in;
            const auto end_boundary = static_cast<IndexType>(in + static_cast<IndexType>(size));

            // 优化: 复用当前生产者位置上已有的逻辑 idle 边界。
            if (idle_boundary_before_in_) {
                if (size) {
                    // 非空: 只追加新的'end'。
                    if (!idle_checkpoints_.push_back(end_boundary))
                        return false;
                }
                // ZLP(size==0)时: 已有检查点本身就强制了 idle 等待。
            } else {
                if (size) {
                    // 非空: [begin, end] 成对入队, 两侧都保证隔离。
                    if (idle_checkpoints_.push_back_n(
                            [&, index = 0]() mutable noexcept {
                                return (index++ == 0) ? begin_boundary : end_boundary;
                            },
                            2, true)
                        != 2) {
                        return false;
                    }
                } else {
                    // ZLP: 'begin' == 'end'。推入单个检查点, 强制一次 IDLE 等待。
                    if (!idle_checkpoints_.push_back(begin_boundary))
                        return false;
                }
            }
        }

        if (size) {
            const auto slice = std::min(size, kBufferSize - offset);
            const bool wrapped = size != slice;
            if (wrapped)
                trailing_boundary_segmentable_ = !data_view.idle_delimited;

            std::memcpy(ring_buffer_.data() + offset, data_view.uart_data.data(), slice);
            std::memcpy(ring_buffer_.data(), data_view.uart_data.data() + slice, size - slice);

            in_.store(
                static_cast<IndexType>(in + static_cast<IndexType>(size)),
                std::memory_order::release);

            idle_boundary_before_in_ = data_view.idle_delimited;
        } else {
            // 零长的非 idle 包不得清掉已有的边界。
            idle_boundary_before_in_ |= data_view.idle_delimited;
        }

        return true;
    }

    bool try_dequeue() {
        if (is_busy_.load(std::memory_order::acquire))
            return false;

        if (!is_idle_)
            is_idle_ = timer::timer->check_expired(tx_complete_timepoint_, idle_gap_);

        core::utility::assert_debug_lazy(
            [&]() noexcept { return (tx_dma_handle()->Instance->CR & DMA_SxCR_EN) == 0U; });

        auto out = out_.load(std::memory_order::relaxed);
        if (in_flight_) {
            // 直写环缓存的 DMA: 上一次 DMA 结束后才推进 out_。
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

            // size==0 说明 out 恰好停在一个检查点边界上。
            // 保持边界, 直到要求的 idle 窗口流逝完毕。
            if (!is_idle_)
                return false;

            idle_checkpoints_.pop_front([](const IndexType&) noexcept {});
        } while (true);
        is_idle_ = false;
        is_busy_.store(true, std::memory_order::relaxed);

        const auto offset = out & kBufferMask;
        const auto slice = std::min(size, kBufferSize - offset);
        const bool wrapped = size != slice;

        if (wrapped && !trailing_boundary_segmentable_) {
            // 严格包必须跨回绕保持连续。
            // 摊平进 staging, 一次 DMA 发完。
            std::memcpy(staging_buffer_.data(), ring_buffer_.data() + offset, slice);
            std::memcpy(staging_buffer_.data() + slice, ring_buffer_.data(), size - slice);
            out = static_cast<IndexType>(out + static_cast<IndexType>(size));
            out_.store(out, std::memory_order::release);

            start_tx_dma(
                reinterpret_cast<const uint8_t*>(staging_buffer_.data()),
                static_cast<uint16_t>(size));
            return true;
        }

        // 非严格路径可直接从环缓存流出; 进度在完成时提交。
        start_tx_dma(
            reinterpret_cast<const uint8_t*>(ring_buffer_.data() + offset),
            static_cast<uint16_t>(slice));
        in_flight_ = static_cast<IndexType>(slice);

        return true;
    }

    // 发送侧已彻底排空: ring 里没有待发字节, 也没有 DMA 在途。端口停口之后靠它
    // 判断还要不要继续轮询(见 Uart::try_transmit())。与 mc02 同一判据。
    [[nodiscard]] bool drained() const {
        return !is_busy_.load(std::memory_order::acquire) && in_flight_ == 0
            && in_.load(std::memory_order::relaxed) == out_.load(std::memory_order::relaxed);
    }

    void tx_complete_callback() {
        // DMA 把最后一个字节写进 UART DR 后, 停掉 UART 发起的 DMA 请求。
        ATOMIC_CLEAR_BIT(hal_uart_handle_->Instance->CR3, USART_CR3_DMAT);
        tx_complete_timepoint_ = timer::timer->timepoint();
        is_busy_.store(false, std::memory_order::release);
    }

    void tx_error_callback() {
        ATOMIC_CLEAR_BIT(hal_uart_handle_->Instance->CR3, USART_CR3_DMAT);
        core::utility::assert_debug_lazy([]() noexcept { return false; });
        tx_complete_timepoint_ = timer::timer->timepoint();
        is_busy_.store(false, std::memory_order::release);
    }

private:
    DMA_HandleTypeDef* tx_dma_handle() const { return hal_uart_handle_->hdmatx; }

    void bind_tx_dma_callbacks() {
        auto* dma = tx_dma_handle();
        dma->XferCpltCallback = dma_complete_callback_;
        dma->XferErrorCallback = dma_error_callback_;
        dma->XferHalfCpltCallback = nullptr;
        dma->XferAbortCallback = nullptr;
    }

    void start_tx_dma(const uint8_t* data, uint16_t size) {
        auto* dma = tx_dma_handle();
        bind_tx_dma_callbacks();

        core::utility::assert_always(
            HAL_DMA_Start_IT(
                dma, reinterpret_cast<uint32_t>(data),
                reinterpret_cast<uint32_t>(&hal_uart_handle_->Instance->DR), size)
            == HAL_OK);

        __HAL_UART_CLEAR_FLAG(hal_uart_handle_, UART_FLAG_TC);
        ATOMIC_SET_BIT(hal_uart_handle_->Instance->CR3, USART_CR3_DMAT);
    }

    UART_HandleTypeDef* hal_uart_handle_;
    void (*dma_complete_callback_)(DMA_HandleTypeDef*);
    void (*dma_error_callback_)(DMA_HandleTypeDef*);

    // 环缓存与回绕暂存都是 TX DMA 的源(start_tx_dma())。由 UartDmaMemory(uart.hpp)
    // 持有并传入引用: 独立对象才进得去 .dmaram 非缓存区(见 rx_buffer.hpp 的说明),
    // CPU 写入不落 D-cache, DMA 读到的永远是最新字节。
    std::array<std::byte, kBufferSize>& ring_buffer_;
    std::array<std::byte, kStagingBufferSize>& staging_buffer_;

    std::atomic<IndexType> in_{0};
    std::atomic<IndexType> out_{0};
    static_assert(std::atomic<IndexType>::is_always_lock_free);

    IndexType in_flight_{0};
    std::atomic<bool> is_busy_{false};

    // 仅生产者访问的状态: 当前 in_ 位置是否已经挂着逻辑 idle 边界。
    bool idle_boundary_before_in_{false};
    bool trailing_boundary_segmentable_{false};
    bool is_idle_{true};
    timer::Timer::TimePoint tx_complete_timepoint_{timer::Timer::TimePoint::min()};
    // 包间空隙, set_line_rate() 存下。
    timer::Timer::Duration idle_gap_{line_time(kIdleGapCharacters, 10, 115'200)};

    utility::RingBuffer<IndexType, kMaxIdleCheckpointCount> idle_checkpoints_;
};

} // namespace libhcs::firmware::uart
