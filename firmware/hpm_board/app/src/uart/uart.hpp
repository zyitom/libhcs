#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>
#include <utility>

#include <hpm_common.h>
#include <hpm_soc.h>
#include <hpm_soc_ip_feature.h>
#include <hpm_uart_drv.h>
#include <hpm_uart_regs.h>

#include "board_app.hpp"
#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/hpm_board/app/src/led/led.hpp"
#include "firmware/hpm_board/app/src/link/uplink.hpp"
#include "firmware/hpm_board/app/src/uart/rx_buffer.hpp"
#include "firmware/hpm_board/app/src/uart/tx_buffer.hpp"
#include "firmware/hpm_board/app/src/uart/uart_port.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::uart {

using board::UartPort;

class Uart
    : private core::utility::Immovable
    , private TxBuffer
    , private RxBuffer<Uart> {
    friend class RxBuffer<Uart>;

public:
    using Lazy = utility::Lazy<Uart, UartPort, size_t>;

    // Defined out-of-line below the AHB SRAM storage arrays.
    explicit Uart(UartPort port, size_t storage_index);

    [[nodiscard]] data::DataId data_id() const { return data_id_; }

    [[nodiscard]] data::DataId config_data_id() const { return config_data_id_; }

    // Runtime baudrate switch requested by the host over EP0
    // (usb/vendor_control.cpp). Returns whether the port is now running at
    // `baudrate`; false leaves the hardware exactly as it was, and the control
    // transfer's status stage is stalled so the host learns that -- which is
    // what the old in-band kUart*Config field could not express.
    //
    // RX bytes arriving inside the switch window may be garbled -- the line rate
    // changes mid-character and there is no way to synchronize with the peer.
    // That is accepted; the host is expected to quiesce the link first.
    //
    // Runs on the main loop, from tud_task(): the DLAB window below is only
    // safe with the transmit DMA provably stopped, which interrupt context
    // cannot guarantee.
    bool set_baudrate(uint32_t baudrate) {
        if (baudrate == 0) [[unlikely]]
            return false;

        // Bytes already handed to the DMA would be shifted out at the new rate.
        //
        // This must happen before uart_set_baudrate(), and not only to keep the
        // in-flight bytes from being clocked out wrong: DLL aliases THR at
        // offset 0x20 and DLM aliases IER at 0x24, selected by LCR.DLAB. While
        // DLAB is set -- which is exactly what uart_set_baudrate() does to reach
        // the latch -- a TX DMA write aimed at THR lands in the divisor latch
        // instead. The DMA must be provably stopped across the whole window, not
        // merely expected to be idle.
        TxBuffer::abort_transmit();

        const hpm_stat_t status = uart_set_baudrate(uart_base_, baudrate, uart_clock_hz_);
        // Unconditionally, and before anything else touches the port:
        // uart_set_baudrate() sets DLAB up front but returns early WITHOUT
        // clearing it when its solver rejects the baudrate, so on that path the
        // SDK hands the port back with 0x20 still aliased to the divisor latch.
        // Clearing it here covers both outcomes.
        uart_base_->LCR &= ~UART_LCR_DLAB_MASK;

        // Rejected: the divisor was left untouched, so the port is still running
        // at the old rate and the queued bytes are still valid for it. Skipping
        // the FIFO reset keeps them, which matches the SDK's own LIN sample --
        // it returns before its uart_reset_rx_fifo() on this path rather than
        // discarding data over a request that changed nothing.
        //
        // 80 MHz cannot represent every rate inside the SDK's 3% tolerance, and
        // reporting success here would tell the host a switch had happened when
        // it had not -- the one failure mode that makes a peer rate mismatch look
        // like a wiring or hardware fault.
        if (status != status_success) [[unlikely]]
            return false;

        // The FIFO still holds whatever the aborted DMA had already pushed.
        // Those bytes predate the new divisor, so shifting them out now would
        // put a burst of corrupt characters on the line at the new rate; the
        // peer would resynchronise eventually, but the first frame after every
        // switch would be garbage.
        uart_reset_tx_fifo(uart_base_);

        snapshot_divisor();
        return true;
    }

    void handle_downlink(const data::UartDataView& data) {
        if (!TxBuffer::try_enqueue(data))
            led::led->downlink_buffer_full();
    }

    void try_transmit() { TxBuffer::try_dequeue(); }

    void irq_handler() {
        if (uart_is_rxline_idle(uart_base_)) {
            uart_clear_rxline_idle_flag(uart_base_);
            RxBuffer::rx_idle_callback();
        }
    }

    // Kernel clock the baudrate divisor was computed from, and the divisor as it
    // was actually programmed. Exposed for the diagnostic record: a divisor is
    // only interpretable next to its clock.
    //
    // The divisor is served from a snapshot rather than read back on demand
    // because reading it requires setting LCR.DLAB, and DLAB re-points the
    // address the TX DMA writes to (THR at 0x20 becomes DLL) -- a diagnostic
    // read racing a live TX would overwrite the divisor with a data byte and
    // silently kill the port. Cheap and correct: nothing changes the divisor
    // except init and set_baudrate, and both snapshot it while TX is stopped.
    [[nodiscard]] uint32_t clock_hz() const { return uart_clock_hz_; }
    [[nodiscard]] uint32_t divisor() const { return uart_divisor_; }
    [[nodiscard]] uint32_t oscr() const { return uart_base_->OSCR; }
    [[nodiscard]] UART_Type* base() const { return uart_base_; }

    // The baudrate the hardware is really running, reconstructed from the divisor
    // and oversample rate actually programmed -- not the value the host asked for.
    // That distinction is the whole point: a rejected request leaves the divisor
    // untouched, so this still reports the old rate and the host can tell.
    //
    // OSCR holds the oversample rate, with 0 meaning 32 (the SDK encodes it that
    // way because the field is 5 bits).
    [[nodiscard]] uint32_t effective_baudrate() const {
        const uint32_t osc_field = uart_base_->OSCR & UART_OSCR_OSC_MASK;
        const uint32_t osc = osc_field ? osc_field : 32U;
        const uint32_t divisor = uart_divisor_;
        if (!osc || !divisor) [[unlikely]]
            return 0;
        return uart_clock_hz_ / (osc * divisor);
    }

private:
    // Caller must guarantee the TX DMA is stopped: this sets LCR.DLAB, during
    // which any DMA write intended for THR would hit the divisor latch instead.
    void snapshot_divisor() {
        const uint32_t lcr = uart_base_->LCR;
        uart_base_->LCR = lcr | UART_LCR_DLAB_MASK;
        uart_divisor_ = ((uart_base_->DLM & 0xFFU) << 8) | (uart_base_->DLL & 0xFFU);
        uart_base_->LCR = lcr & ~UART_LCR_DLAB_MASK;
    }

    [[nodiscard]] uint32_t init_uart(uint32_t irq_num, uint32_t baudrate, parity_setting_t parity) {
        const uint32_t uart_clock = board::init_uart(uart_base_);

        uart_config_t config{};
        uart_default_config(uart_base_, &config);
        config.fifo_enable = true;
        config.dma_enable = true;
        config.src_freq_in_hz = uart_clock;
        config.tx_fifo_level = uart_tx_fifo_trg_not_full;
        config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
        config.baudrate = baudrate;
        config.parity = static_cast<uint8_t>(parity);

        static_assert(HPM_IP_FEATURE_UART_TX_IDLE_DETECT == 1); // NOLINT(misc-redundant-expression)
        config.txidle_config.idle_cond = uart_rxline_idle_cond_state_machine_idle;
        config.txidle_config.detect_enable = true;
        config.txidle_config.threshold = 16;

        static_assert(HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1); // NOLINT(misc-redundant-expression)
        config.rxidle_config.detect_enable = true;
        config.rxidle_config.detect_irq_enable = true;
        config.rxidle_config.idle_cond = uart_rxline_idle_cond_state_machine_idle;
        config.rxidle_config.threshold = 10;

        core::utility::assert_always(uart_init(uart_base_, &config) == status_success);
        intc_m_enable_irq_with_priority(irq_num, 1);

        return uart_clock;
    }

    void handle_uplink(
        std::span<const std::byte> payload, std::span<const std::byte> payload2, bool is_idle) {
        if (!link::uplink_enabled())
            return;

        auto& serializer = link::uplink_serializer();
        core::utility::assert_debug(
            serializer.write_uart(
                data_id_, {.uart_data = payload, .idle_delimited = is_idle}, payload2)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    const data::DataId data_id_;
    const data::DataId config_data_id_;
    UART_Type* uart_base_;
    // Source clock captured at init: uart_set_baudrate needs it to recompute the
    // divisor on a runtime baudrate switch.
    uint32_t uart_clock_hz_;
    // Divisor as programmed, sampled by snapshot_divisor() at init and after each
    // switch. See the accessor above for why it is not read back on demand.
    uint32_t uart_divisor_ = 0;

public:
    // DMA buffer storage in a board-chosen non-cached region (AHB SRAM on
    // HPM5321, the AXI-SRAM non-cacheable window on HPM6E80 core1),
    // eliminating manual l1c_dc_flush/invalidate overhead on the UART data path.
    struct RxStorage {
        alignas(HPM_L1C_CACHELINE_SIZE) std::array<std::byte, RxBuffer<Uart>::kBufferSize> data;
        alignas(HPM_L1C_CACHELINE_SIZE) std::array<
            dma_mgr_linked_descriptor_t, RxBuffer<Uart>::kDmaDescriptorCount> descriptors;
    };
    struct TxStorage {
        alignas(HPM_L1C_CACHELINE_SIZE) std::array<std::byte, TxBuffer::kBufferSize> data;
        alignas(HPM_L1C_CACHELINE_SIZE) dma_mgr_linked_descriptor_t descriptor;
    };
};

constexpr size_t kUartCount = std::size(board::kUartPorts);

ATTR_PLACE_AT(libhcs_DMA_BUFFER_SECTION)
inline constinit Uart::RxStorage uart_rx_storage[kUartCount]{};
ATTR_PLACE_AT(libhcs_DMA_BUFFER_SECTION)
inline constinit Uart::TxStorage uart_tx_storage[kUartCount]{};

inline Uart::Uart(UartPort port, size_t storage_index)
    : TxBuffer(
          reinterpret_cast<UART_Type*>(port.base), port.dma_src_tx,
          uart_tx_storage[storage_index].data.data(), &uart_tx_storage[storage_index].descriptor)
    , RxBuffer(
          reinterpret_cast<UART_Type*>(port.base), port.dma_src_rx,
          uart_rx_storage[storage_index].data.data(),
          uart_rx_storage[storage_index].descriptors.data())
    , data_id_(port.data_id)
    , config_data_id_(port.config_data_id)
    , uart_base_(reinterpret_cast<UART_Type*>(port.base))
    , uart_clock_hz_(init_uart(port.irq_num, port.baudrate, port.parity)) {
    // Safe here for the same reason set_baudrate's call is: no TX has been
    // queued yet, so nothing can be writing THR while DLAB is set.
    snapshot_divisor();
}

namespace internal {

template <std::size_t index>
consteval Uart::Lazy make_uart() {
    return Uart::Lazy{board::kUartPorts[index], index};
}

template <std::size_t... indices>
consteval std::array<Uart::Lazy, sizeof...(indices)>
    make_uart_array(std::index_sequence<indices...>) {
    return {make_uart<indices>()...};
}

} // namespace internal

inline constinit auto uart_array =
    internal::make_uart_array(std::make_index_sequence<kUartCount>{});

} // namespace libhcs::firmware::uart
