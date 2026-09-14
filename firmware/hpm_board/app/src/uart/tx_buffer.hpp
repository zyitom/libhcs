#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <type_traits>

#include <hpm_common.h>
#include <hpm_dma_mgr.h>
#include <hpm_dmav2_drv.h>
#include <hpm_dmav2_regs.h>
#include <hpm_soc_feature.h>
#include <hpm_uart_drv.h>
#include <hpm_uart_regs.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/hpm_board/app/src/utility/ring_buffer.hpp"

namespace libhcs::firmware::uart {

class TxBuffer {
public:
    static constexpr size_t kBufferSize = 2048;
    static constexpr size_t kBufferMask = kBufferSize - 1;
    static_assert((kBufferSize & (kBufferSize - 1)) == 0);

    using BufferIndexType = uint16_t;
    static_assert(kBufferSize <= std::numeric_limits<uint16_t>::max());

    static constexpr size_t kMaxIdleCount = 256;

    TxBuffer(
        UART_Type* uart_base, uint32_t dmamux_src, std::byte* data_buffer,
        dma_mgr_linked_descriptor_t* linked_descriptor)
        : uart_base_(uart_base)
        , data_buffer_(data_buffer)
        , linked_descriptor_(linked_descriptor) {
        init_dma(dmamux_src);
    }

    bool try_enqueue(const data::UartDataView& data_view) {
        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);

        const auto writable =
            kBufferSize - static_cast<size_t>(static_cast<BufferIndexType>(in - out));

        const auto size = data_view.uart_data.size();
        if (size > writable)
            return false;

        if (data_view.idle_delimited) {
            const auto begin_idle = in;
            const auto end_idle =
                static_cast<BufferIndexType>(in + static_cast<BufferIndexType>(size));

            if (idle_boundary_before_in_) {
                if (size) {
                    if (!idle_checkpoints_.push_back(end_idle))
                        return false;
                }
            } else {
                if (size) {
                    if (idle_checkpoints_.push_back_n(
                            [&, i = 0]() mutable noexcept {
                                return (i++ == 0) ? begin_idle : end_idle;
                            },
                            2, true)
                        != 2) {
                        return false;
                    }
                } else {
                    if (!idle_checkpoints_.push_back(begin_idle))
                        return false;
                }
            }
        }

        if (size) {
            auto offset = in & kBufferMask;
            auto slice = std::min(size, kBufferSize - offset);
            std::memcpy(data_buffer_ + offset, data_view.uart_data.data(), slice);
            std::memcpy(data_buffer_, data_view.uart_data.data() + slice, size - slice);

            in_.store(
                static_cast<BufferIndexType>(in + static_cast<BufferIndexType>(size)),
                std::memory_order::release);

            idle_boundary_before_in_ = data_view.idle_delimited;
        } else {
            idle_boundary_before_in_ |= data_view.idle_delimited;
        }

        return true;
    }

    // 立即停掉在途的 TX DMA。波特率切换前使用, 有两个原因: 不停的话队列里
    // 剩余字节会按新速率移位输出、到达即乱码; 且 LCR.DLAB 置位期间 DLL
    // 别名 THR, 落在切换窗口内的 DMA 写会覆盖分频锁存器本身。调用方依赖的
    // 是"TX 可证明已停止", 而非"多半空闲"。
    //
    // 队列数据本身不动: out_ 不前移, 下次轮询时 try_dequeue() 从原位重新
    // 触发。DMA 已推入 FIFO 的字节会重发而非丢弃, 跨切换可能把一个移位到
    // 一半的字符重复一遍。这在 handle_config 的既有约定之内 -- 主机本应先
    // 静默链路 -- 且是两种失败模式中较安全的一种。
    void abort_transmit() {
        // 先 CHABORT 再禁用: 寄存器文档写明对未使能的通道写会被忽略, 若先
        // 清 CTRL.ENABLE 会静默跳过中止, 通道与 UART 的握手停在半途。中止
        // 还会置起 INTABORTSTS, 在下方清掉, 让下一次传输从干净的状态标志
        // 起步。
        if (dma_channel_is_enable(dma_.base, dma_.channel))
            dma_abort_channel(dma_.base, 1U << dma_.channel);
        core::utility::assert_always(dma_mgr_disable_channel(&dma_) == status_success);
        dma_clear_transfer_status(dma_.base, dma_.channel);
        tx_triggered_ = false;
        // 中止会丢弃通道写到一半的字节, 在途计数必须随之清零。留着它会让
        // 下一次 try_dequeue() 把 out_ 推过 DMA 实际从未发出的数据, 环形被
        // 永久错位, 之后每个字节都来自错误偏移。与造成静默 TX 症状的分频
        // 锁存器破坏是两回事 -- 这一条只在切换波特率后触发, 且是打乱数据流
        // 而非停发。
        in_flight_ = 0;
    }

    bool try_dequeue() {
        if (dma_channel_is_enable(dma_.base, dma_.channel))
            return false;
        auto out = out_.load(std::memory_order::relaxed);
        if (in_flight_) {
            out = static_cast<BufferIndexType>(out + in_flight_);
            out_.store(out, std::memory_order::release);
            in_flight_ = 0;
        }

        const auto in = in_.load(std::memory_order::acquire);
        const auto readable = static_cast<size_t>(static_cast<BufferIndexType>(in - out));
        if (!readable)
            return false;

        const auto offset = out & kBufferMask;

        size_t size;
        do {
            size = readable;
            if (auto* idle = idle_checkpoints_.peek_front()) {
                const auto distance =
                    static_cast<size_t>(static_cast<BufferIndexType>(*idle - out));
                core::utility::assert_debug(distance <= readable);
                size = distance;
            }

            if (size)
                break;

            if (tx_triggered_ && !uart_is_txline_idle(uart_base_))
                return false;

            idle_checkpoints_.pop_front([](const BufferIndexType&) noexcept {});
        } while (true);
        tx_triggered_ = true;
        uart_clear_txline_idle_flag(uart_base_);

        const auto slice = std::min(size, kBufferSize - offset);
        if (slice == size)
            trigger_dma(data_buffer_ + offset, slice, nullptr, 0);
        else
            trigger_dma(data_buffer_ + offset, slice, data_buffer_, size - slice);

        in_flight_ = static_cast<BufferIndexType>(size);

        return true;
    }

private:
    void init_dma(uint32_t dmamux_src) {
        dma_mgr_chn_conf_t config;
        dma_mgr_get_default_chn_config(&config);

        config.en_dmamux = true;
        config.dmamux_src = dmamux_src;
        config.priority = DMA_MGR_CHANNEL_PRIORITY_LOW;
        config.src_addr = 0;
        config.dst_addr = reinterpret_cast<uint32_t>(&uart_base_->THR);
        config.src_width = DMA_MGR_TRANSFER_WIDTH_BYTE;
        config.dst_width = DMA_MGR_TRANSFER_WIDTH_BYTE;
        config.src_addr_ctrl = DMA_MGR_ADDRESS_CONTROL_INCREMENT;
        config.dst_addr_ctrl = DMA_MGR_ADDRESS_CONTROL_FIXED;
        config.src_mode = DMA_MGR_HANDSHAKE_MODE_NORMAL;
        config.dst_mode = DMA_MGR_HANDSHAKE_MODE_HANDSHAKE;
        // 与 RX 侧同一约束: TX 请求在 uart_tx_fifo_trg_not_full 时发出, 只
        // 保证一个空槽, 若突发 8 字节, 可能往只剩一个空位的 FIFO 塞 8 个而
        // 丢弃其余。RX 侧同源的损坏是直接观测到的; 这一侧按同一规则修正,
        // 而不是指望 FIFO 恰好排空得比 DMA 灌入快。
        config.src_burst_size = DMA_MGR_NUM_TRANSFER_PER_BURST_1T;

        core::utility::assert_always(
            dma_mgr_request_resource(&dma_) == status_success
            && dma_mgr_setup_channel(&dma_, &config) == status_success
            && dma_mgr_config_linked_descriptor(&dma_, &config, linked_descriptor_)
                   == status_success);
        // 无需 cache flush: 描述符在 AHB SRAM。
    }

    void trigger_dma(const std::byte* src, size_t size, const std::byte* src2, size_t size2) {
        core::utility::assert_debug(src);
        // 无需 cache flush: 缓冲在 AHB SRAM。
        auto& ctrl = dma_.base->CHCTRL[dma_.channel];
        ctrl.SRCADDR = reinterpret_cast<uintptr_t>(src);
        ctrl.TRANSIZE = size;

        if (src2) {
            auto* raw_desc = reinterpret_cast<dma_linked_descriptor_t*>(linked_descriptor_);
            raw_desc->src_addr = reinterpret_cast<uintptr_t>(src2);
            raw_desc->trans_size = size2;
            ctrl.LLPOINTER = reinterpret_cast<uintptr_t>(linked_descriptor_);
        } else {
            ctrl.LLPOINTER = 0;
        }

        ctrl.CTRL |= DMAV2_CHCTRL_CTRL_ENABLE_MASK;
    }

    UART_Type* uart_base_;
    // 由调用方放入 AHB SRAM -- 天然非缓存。
    std::byte* data_buffer_;
    dma_mgr_linked_descriptor_t* linked_descriptor_;
    dma_resource_t dma_;

    std::atomic<BufferIndexType> in_{0}, out_{0};
    static_assert(std::atomic<BufferIndexType>::is_always_lock_free);
    BufferIndexType in_flight_ = 0;

    bool idle_boundary_before_in_ = false;
    bool tx_triggered_ = false;
    utility::RingBuffer<BufferIndexType, kMaxIdleCount> idle_checkpoints_;
};

} // namespace libhcs::firmware::uart
