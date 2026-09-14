#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include <hpm_common.h>
#include <hpm_dma_mgr.h>
#include <hpm_dmav2_drv.h>
#include <hpm_soc_feature.h>
#include <hpm_uart_regs.h>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/utility/assert.hpp"

namespace libhcs::firmware::uart {

template <typename T>
class RxBuffer {
    friend T;

public:
    static constexpr size_t kBufferSize = 2048;
    static constexpr size_t kBufferMask = kBufferSize - 1;
    static_assert((kBufferSize & (kBufferSize - 1)) == 0);

    // 每描述符 64 字节 DMA 传输: 32 个链式描述符覆盖整个 2 KiB 环。小传输
    // 降低帧协议的延迟 -- HT 在 32 B 触发(921600 baud 下 347 us), TC 在
    // 64 B(694 us)触发。
    static constexpr size_t kBufferTriggerIrqSize = HPM_L1C_CACHELINE_SIZE;
    static constexpr size_t kDmaTransSize = 2 * kBufferTriggerIrqSize;
    static constexpr size_t kDmaDescriptorCount = kBufferSize / kDmaTransSize;
    static_assert(kBufferSize % kDmaTransSize == 0);
    static_assert(kDmaDescriptorCount >= 2);

    static constexpr size_t kMinFragmentSize = 32;
    static constexpr size_t kMaxFragmentSize = kMinFragmentSize + kBufferTriggerIrqSize - 1;
    static constexpr size_t kProtocolMaxPayloadSize =
        core::protocol::kProtocolBufferSize - sizeof(core::protocol::UartHeaderExtended);
    static_assert(kMaxFragmentSize <= kProtocolMaxPayloadSize);

    using BufferIndexType = uint16_t;
    static_assert(kBufferSize <= std::numeric_limits<uint16_t>::max());

    void rx_idle_callback() { try_dequeue(true); }

private:
    RxBuffer(
        UART_Type* uart_base, uint32_t dmamux_src, std::byte* data_buffer,
        dma_mgr_linked_descriptor_t* linked_descriptors)
        : uart_base_(uart_base)
        , data_buffer_(data_buffer)
        , dma_linked_descriptors_(linked_descriptors) {
        init_dma(dmamux_src);
    }

    void dma_tc_half_tc_callback() { try_dequeue(false); }

    void init_dma(uint32_t dmamux_src) {
        dma_mgr_chn_conf_t config;
        dma_mgr_get_default_chn_config(&config);

        config.en_dmamux = true;
        config.dmamux_src = dmamux_src;
        config.priority = DMA_MGR_CHANNEL_PRIORITY_LOW;
        config.src_addr = reinterpret_cast<uintptr_t>(&uart_base_->RBR);
        config.dst_addr = reinterpret_cast<uintptr_t>(data_buffer_);
        config.src_width = DMA_MGR_TRANSFER_WIDTH_BYTE;
        config.dst_width = DMA_MGR_TRANSFER_WIDTH_BYTE;
        config.src_addr_ctrl = DMA_MGR_ADDRESS_CONTROL_FIXED;
        config.dst_addr_ctrl = DMA_MGR_ADDRESS_CONTROL_INCREMENT;
        config.src_mode = DMA_MGR_HANDSHAKE_MODE_HANDSHAKE;
        config.dst_mode = DMA_MGR_HANDSHAKE_MODE_NORMAL;
        // 每次握手只传一个字节: UART 在 uart_rx_fifo_trg_not_empty 时才发
        // RX DMA 请求, 即 FIFO 里只有一个字节时(见 Uart::init)。突发量超过
        // FIFO 保证的占用量, DMA 就会在空 FIFO 上再次读 RBR 并存下返回的
        // 任意值: 先前的 4T 配置下, 每个收到的字节都以"一个好字节跟三个
        // 垃圾字节"的形式落入环形。
        config.src_burst_size = DMA_MGR_NUM_TRANSFER_PER_BURST_1T;
        config.size_in_byte = kDmaTransSize;
        config.linked_ptr = reinterpret_cast<uintptr_t>(&dma_linked_descriptors_[1]);
        config.interrupt_mask = DMA_INTERRUPT_MASK_ABORT | DMA_INTERRUPT_MASK_ERROR;

        core::utility::assert_always(
            dma_mgr_request_resource(&dma_) == status_success
            && dma_mgr_setup_channel(&dma_, &config) == status_success);

        for (size_t i = 0; i < kDmaDescriptorCount; i++) {
            config.linked_ptr = reinterpret_cast<uintptr_t>(
                &dma_linked_descriptors_[(i + 1) % kDmaDescriptorCount]);
            core::utility::assert_always(
                dma_mgr_config_linked_descriptor(&dma_, &config, &dma_linked_descriptors_[i])
                == status_success);
            config.dst_addr += kDmaTransSize;
        }
        // 无需 cache 维护: 缓冲位于 AHB SRAM(非缓存)。

        auto callback = [](DMA_Type* /*base*/, uint32_t /*channel*/, void* user_data) {
            static_cast<RxBuffer*>(user_data)->dma_tc_half_tc_callback();
        };
        core::utility::assert_always(
            dma_mgr_install_chn_tc_callback(&dma_, callback, this) == status_success
            && dma_mgr_install_chn_half_tc_callback(&dma_, callback, this) == status_success
            && dma_mgr_enable_dma_irq_with_priority(&dma_, 1) == status_success
            && dma_mgr_enable_channel(&dma_) == status_success);
    }

    bool try_dequeue(bool is_idle) {
        const auto in = update_in();
        const auto out = out_.load(std::memory_order::acquire);

        const auto readable = static_cast<size_t>(static_cast<BufferIndexType>(in - out));
        if (!is_idle && readable < kMinFragmentSize)
            return false;

        if (readable > kProtocolMaxPayloadSize) [[unlikely]] {
#ifndef NDEBUG
            core::utility::assert_failed_always();
#endif
        } else {
            const auto offset = out & kBufferMask;
            const auto slice = std::min(readable, kBufferSize - offset);
            // 无需 cache invalidate: data_buffer_ 在 AHB SRAM。

            if (slice == readable) {
                static_cast<T*>(this)->handle_uplink({data_buffer_ + offset, slice}, {}, is_idle);
            } else {
                static_cast<T*>(this)->handle_uplink(
                    {data_buffer_ + offset, slice}, {data_buffer_, readable - slice}, is_idle);
            }
        }

        out_.store(
            static_cast<BufferIndexType>(out + static_cast<BufferIndexType>(readable)),
            std::memory_order::release);

        return true;
    }

    BufferIndexType update_in() {
        const auto in = in_.load(std::memory_order::relaxed);

        const size_t current_offset =
            dma_.base->CHCTRL[dma_.channel].DSTADDR - reinterpret_cast<uintptr_t>(data_buffer_);
        core::utility::assert_debug(current_offset < kBufferSize);

        auto new_in = static_cast<BufferIndexType>((in & ~kBufferMask) | current_offset);
        if (new_in < in)
            new_in += kBufferSize;

        in_.store(new_in, std::memory_order::release);
        return new_in;
    }

    UART_Type* uart_base_;
    // 由调用方放入 AHB SRAM -- 天然非缓存, 无需手工 invalidate。
    std::byte* data_buffer_;
    dma_mgr_linked_descriptor_t* dma_linked_descriptors_;
    dma_resource_t dma_;

    std::atomic<BufferIndexType> in_{0}, out_{0};
    static_assert(std::atomic<BufferIndexType>::is_always_lock_free);
};

} // namespace libhcs::firmware::uart
