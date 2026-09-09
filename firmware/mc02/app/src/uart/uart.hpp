#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <usart.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/led/led.hpp"
#include "firmware/mc02/app/src/uart/rx_buffer.hpp"
#include "firmware/mc02/app/src/uart/tx_buffer.hpp"
#include "firmware/mc02/app/src/usb/helper.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

namespace libhcs::firmware::uart {

// State and behaviour shared by both port flavours: identity, the runtime
// baudrate control surface, and the hand-off of received bytes to USB. It holds
// no buffers of its own, so the RX-only DBUS port does not carry a 3 KB TX ring
// it can never use.
class UartCommon : private core::utility::Immovable {
public:
    // Runtime baudrate switch requested by the host. Writes BRR directly rather
    // than re-running HAL_UART_Init, so every other setting stays exactly as
    // CubeMX generated it and the running DMA is not torn down.
    //
    // This mirrors UART_SetConfig() in the STM32H7 HAL. Unlike the F4 parts,
    // H7 UARTs take their kernel clock from a selectable source (not simply
    // PCLK) and pass it through Init.ClockPrescaler, so the divisor must be
    // computed from the resolved clock source -- using HAL_RCC_GetPCLKxFreq here
    // the way c_board does would silently produce the wrong baudrate.
    //
    // RX bytes arriving inside the switch window may be garbled; the host is
    // expected to quiesce the link first.
    bool handle_config(const data::UartConfigView& data) {
        if (!data.baudrate.has_value() || *data.baudrate == 0) [[unlikely]]
            return false;

        const uint32_t kernel_clock_hz = peripheral_clock_hz();
        if (kernel_clock_hz == 0U) [[unlikely]]
            return false;

        // Bounds per the HAL: a divisor outside them would misconfigure the
        // peripheral, so reject instead of writing garbage.
        constexpr uint32_t kBrrMin = 0x10U;
        constexpr uint32_t kBrrMax = 0xFFFFU;

        uint32_t brr;
        if (hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8) {
            const uint32_t usartdiv = UART_DIV_SAMPLING8(
                kernel_clock_hz, *data.baudrate, hal_uart_handle_->Init.ClockPrescaler);
            if (usartdiv < kBrrMin || usartdiv > kBrrMax) [[unlikely]]
                return false;
            // Oversampling-by-8 packs BRR[3] as zero and shifts the low nibble.
            brr = (usartdiv & 0xFFF0U) | ((usartdiv & 0x000FU) >> 1U);
        } else {
            const uint32_t usartdiv = UART_DIV_SAMPLING16(
                kernel_clock_hz, *data.baudrate, hal_uart_handle_->Init.ClockPrescaler);
            if (usartdiv < kBrrMin || usartdiv > kBrrMax) [[unlikely]]
                return false;
            brr = usartdiv;
        }

        hal_uart_handle_->Init.BaudRate = *data.baudrate;
        hal_uart_handle_->Instance->BRR = brr;
        return true;
    }

    // The baudrate actually programmed, reconstructed from BRR rather than from
    // Init.BaudRate: the latter is only what was last requested, and on a rejected
    // request neither it nor BRR is written, so reading BRR back is what lets the
    // host tell "switch applied" from "switch refused, old rate still running".
    [[nodiscard]] uint32_t effective_baudrate() const {
        const uint32_t kernel_clock_hz = peripheral_clock_hz();
        const uint32_t brr = hal_uart_handle_->Instance->BRR & 0xFFFFU;
        if (!kernel_clock_hz || !brr) [[unlikely]]
            return 0;
        const uint32_t presc = UARTPrescTable[hal_uart_handle_->Init.ClockPrescaler & 0x0FU];
        const uint32_t clock = presc ? kernel_clock_hz / presc : kernel_clock_hz;
        if (hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8) {
            // Oversampling by 8 stores BRR[3] as zero and the low nibble shifted
            // right by one, so undo that before dividing.
            const uint32_t usartdiv = (brr & 0xFFF0U) | ((brr & 0x0007U) << 1U);
            return usartdiv ? (2U * clock) / usartdiv : 0U;
        }
        return clock / brr;
    }

protected:
    UartCommon(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle)
        : data_id_(data_id)
        , hal_uart_handle_(hal_uart_handle) {}

    // RxBuffer and TxBuffer each keep their own copy of the handle, so the derived
    // ports cannot name hal_uart_handle_ unqualified. Route the two accesses they
    // need through here rather than sprinkling UartCommon:: qualifications.
    [[nodiscard]] bool has_rx_error() const {
        constexpr uint32_t rx_error_mask =
            HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE;
        return (hal_uart_handle_->ErrorCode & rx_error_mask) != 0U;
    }

    void flag_dma_error() { hal_uart_handle_->ErrorCode |= HAL_UART_ERROR_DMA; }

    void handle_uplink(
        std::span<const std::byte> payload, std::span<const std::byte> payload2, bool is_idle) {
        // Nothing written without a session survives: activate_session() clears
        // the ring when a host arrives. RxBuffer::try_dequeue() advances out_
        // either way, so the receive ring still drains here rather than backing
        // up into its wrapped-around fail-fast path.
        if (!usb::uplink_session_active())
            return;

        auto& serializer = usb::get_serializer();
        core::utility::assert_always(
            serializer.write_uart(
                data_id_, {.uart_data = payload, .idle_delimited = is_idle}, payload2)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    // Kernel clock feeding this UART's baudrate divider.
    //
    // c_board does the same job in two lines with HAL_RCC_GetPCLKxFreq(), and
    // that is correct there: on STM32F407 a UART is clocked straight off its APB
    // bus, so the bus frequency IS the kernel clock. STM32H7 inserts a
    // selectable clock source per UART group plus Init.ClockPrescaler, so the
    // source has to be resolved before the frequency means anything.
    //
    // Resolved with the HAL's own UART_GETCLOCKSOURCE, which dispatches on the
    // peripheral instance -- the same macro UART_SetConfig() uses to compute BRR
    // at init. Deliberately NOT HAL_RCCEx_GetPeriphCLKFreq(): that function's
    // if/else chain in this HAL version covers SAI/SPI/ADC/SDMMC/FDCAN but has
    // NO branch for either UART group, so RCC_PERIPHCLK_USART16910 and
    // RCC_PERIPHCLK_USART234578 both fall through to its final
    // `else { frequency = 0; }` and return 0. Those macros do exist, so nothing
    // warns at compile time. handle_config() then took its kernel_clock_hz == 0
    // early-out and returned without ever writing BRR: every runtime baudrate
    // request was silently ignored and the port stayed at the CubeMX 115200.
    //
    // Invisible to a same-board UART7<->UART10 loopback: both ends ignored the
    // switch identically, so the loop passed at every requested rate while
    // actually running at 115200 throughout. Only a cross-board test against an
    // end that did switch exposed it.
    [[nodiscard]] uint32_t peripheral_clock_hz() const {
        UART_ClockSourceTypeDef clocksource = UART_CLOCKSOURCE_UNDEFINED;
        UART_GETCLOCKSOURCE(hal_uart_handle_, clocksource);

        switch (clocksource) {
        case UART_CLOCKSOURCE_D2PCLK1: return HAL_RCC_GetPCLK1Freq();
        case UART_CLOCKSOURCE_D2PCLK2: return HAL_RCC_GetPCLK2Freq();
        case UART_CLOCKSOURCE_CSI: return CSI_VALUE;
        case UART_CLOCKSOURCE_LSE: return LSE_VALUE;
        case UART_CLOCKSOURCE_HSI:
            // HSI reaches the UARTs through its divider, so raw HSI_VALUE is
            // only right when that divider is 1. UART_SetConfig shifts the same
            // way. UART7/UART10 select HSI in the .ioc, so this path is live.
            if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U)
                return static_cast<uint32_t>(HSI_VALUE >> (__HAL_RCC_GET_HSI_DIVIDER() >> 3U));
            return static_cast<uint32_t>(HSI_VALUE);
        case UART_CLOCKSOURCE_PLL2: {
            PLL2_ClocksTypeDef pll2{};
            HAL_RCCEx_GetPLL2ClockFreq(&pll2);
            return pll2.PLL2_Q_Frequency;
        }
        case UART_CLOCKSOURCE_PLL3: {
            PLL3_ClocksTypeDef pll3{};
            HAL_RCCEx_GetPLL3ClockFreq(&pll3);
            return pll3.PLL3_Q_Frequency;
        }
        default: return 0;
        }
    }

    data::DataId data_id_;
    UART_HandleTypeDef* hal_uart_handle_;
};

// Full-duplex port: USART1, UART7 and USART10, each with both an RX and a TX DMA
// stream wired up by CubeMX.
class Uart
    : public UartCommon
    , private TxBuffer</*half_duplex=*/false>
    , private RxBuffer<Uart> {
    friend class RxBuffer<Uart>;

public:
    using Lazy = utility::Lazy<Uart, data::DataId, UART_HandleTypeDef*>;

    Uart(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle)
        : UartCommon(data_id, hal_uart_handle)
        , TxBuffer(hal_uart_handle, &hal_tx_dma_complete_callback, &hal_tx_dma_error_callback)
        , RxBuffer(hal_uart_handle) {}

    void handle_downlink(const data::UartDataView& data) {
        if (!TxBuffer::try_enqueue(data))
            led::led->downlink_buffer_full();
    }

    void try_transmit() {
        RxBuffer::try_dequeue();
        TxBuffer::try_dequeue();
    }

    void tx_complete_callback() { TxBuffer::tx_complete_callback(); }

    void uart_error_callback() {
        if (has_rx_error())
            RxBuffer::rx_error_callback();
    }

    void rx_dma_error_callback() {
        flag_dma_error();
        RxBuffer::rx_error_callback();
    }

    void tx_dma_error_callback() { TxBuffer::tx_error_callback(); }

    void rx_event_callback() { RxBuffer::uart_idle_event_callback(); }

private:
    static void hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_complete_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);
};

// Receive-only port, used for UART5 (DBUS).
//
// Two independent reasons it carries no TX path: CubeMX wires UART5 with an RX
// DMA stream only (bsp/cubemx/Core/Src/usart.c declares hdma_uart5_rx and no
// hdma_uart5_tx), and the protocol never routes a downlink to kUartDbus anyway --
// usb/vendor.hpp's uart_deserialized_callback dispatches kUart1/kUart7/kUart10
// and, when libhcs_APP_RS485_ENABLE, the two RS-485 ports. Only
// kUartDbusConfig is routed here, and that lands in UartCommon::handle_config.
class UartRxOnly
    : public UartCommon
    , private RxBuffer<UartRxOnly> {
    friend class RxBuffer<UartRxOnly>;

public:
    using Lazy = utility::Lazy<UartRxOnly, data::DataId, UART_HandleTypeDef*>;

    UartRxOnly(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle)
        : UartCommon(data_id, hal_uart_handle)
        , RxBuffer(hal_uart_handle) {}

    // Named to match the full-duplex port so app.cpp can poll every port
    // uniformly; here it only drains the receive ring.
    void try_transmit() { RxBuffer::try_dequeue(); }

    void uart_error_callback() {
        if (has_rx_error())
            RxBuffer::rx_error_callback();
    }

    void rx_dma_error_callback() {
        flag_dma_error();
        RxBuffer::rx_error_callback();
    }

    void rx_event_callback() { RxBuffer::uart_idle_event_callback(); }

private:
    static void hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);
};

// Half-duplex RS-485 port, used for USART2.
//
// On the wire nothing distinguishes this from Uart. CubeMX assigns PD4 as
// USART2_DE and MX_USART2_UART_Init calls HAL_RS485Ex_Init, which sets CR3.DEM,
// so the USART asserts DE before the start bit and releases it after the stop
// bit with no software involvement; the schematic ties the transceiver's RE# to
// that same net, so the receiver is disabled exactly while the driver is on and
// the port never hears its own transmission. RxBuffer is therefore used
// unchanged, and there is no direction GPIO anywhere in this driver. STM32F407's
// USART has no DEM/DEP at all, which is why c_board could not have done this.
//
// The difference is the bus discipline, and it lives entirely in
// TxBuffer<true> -- see the comment on kTurnaroundDeadline there. A
// distinct type rather than a runtime flag is what keeps every branch of it out
// of the three full-duplex ports, and it also makes it impossible to hand this
// port to code that assumes it owns its transmit line.
//
// Rings an eighth the size of a streaming port's, because a bus master never has
// more than one transaction on the wire. The ports above are sized for a device
// that talks continuously; here the sequence is bounded by construction -- send a
// request, stay quiet, receive one answer -- so what the ring must cover is a
// single exchange plus slack, not a stream.
//
// 256 bytes is a full lap in 533 us at 4.8 Mbaud while the main loop comes back
// every 12.5 us, a margin of 42x; sample_write_position()'s fallen-behind assert
// sits at half a lap and still has 21x. It holds three of the 78-byte replies
// this bus carries. On the transmit side 256 bytes is seven queued 34-byte
// commands, and 32 checkpoints is twice as many boundaries as the ring can hold
// packets, so neither runs out first.
//
// The staging buffer deliberately matches the ring rather than being smaller:
// try_dequeue() clamps each burst to it, and a packet split across two bursts
// would put main-loop silence in the middle of a frame -- harmless on a stream,
// fatal to a peer that frames on the bus going quiet.
//
// Together this is about 890 bytes per port instead of 5696. The two ports pay
// 1.8 KB of the 32 KB D2 SRAM region instead of 11.4 KB, which is what makes
// having both of them affordable at all.
inline constexpr size_t kRs485BufferSize = 256;
inline constexpr size_t kRs485CheckpointCount = 32;

class UartRs485
    : public UartCommon
    , private TxBuffer<
          /*half_duplex=*/true, kRs485BufferSize, kRs485BufferSize, kRs485CheckpointCount>
    , private RxBuffer<UartRs485, kRs485BufferSize> {
    friend class RxBuffer<UartRs485, kRs485BufferSize>;

public:
    using Lazy = utility::Lazy<UartRs485, data::DataId, UART_HandleTypeDef*>;

    UartRs485(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle)
        : UartCommon(data_id, hal_uart_handle)
        , TxBuffer(hal_uart_handle, &hal_tx_dma_complete_callback, &hal_tx_dma_error_callback)
        , RxBuffer(hal_uart_handle) {}

    void handle_downlink(const data::UartDataView& data) {
        if (!TxBuffer::try_enqueue(data))
            led::led->downlink_buffer_full();
    }

    // Feeds the raw IDLE counter to the turnaround gate, deliberately not the
    // consumed_idle_count_ bookkeeping RxBuffer::try_dequeue() keeps: whether
    // the bus is free again depends only on the peer having stopped talking, not
    // on whether the host has picked the bytes up yet.
    void try_transmit() {
        RxBuffer::try_dequeue();
        TxBuffer::try_dequeue(RxBuffer::idle_count());
    }

    void tx_complete_callback() { TxBuffer::tx_complete_callback(); }

    void uart_error_callback() {
        if (has_rx_error())
            RxBuffer::rx_error_callback();
    }

    void rx_dma_error_callback() {
        flag_dma_error();
        RxBuffer::rx_error_callback();
    }

    void tx_dma_error_callback() { TxBuffer::tx_error_callback(); }

    // Deliberately does NOT drain the ring here. Draining from this ISR instead
    // of from the main loop was built behind libhcs_APP_UART_RX_IN_ISR and
    // measured on two cross-wired boards; the switch is gone because it lost.
    // See firmware/mc02/AGENTS.md for the numbers. In short: it does remove this
    // port's per-pass NDTR read (main loop 5410 -> 5133 cycles per pass, 101 ->
    // 107 kHz), but that time is surplus -- the loop already runs four passes per
    // arriving USB packet -- and it costs 26% of the round trip on a 200-byte
    // message, because publishing only on IDLE gives up the chunked streaming
    // RxBuffer::try_dequeue() does at every kMinFragmentSize boundary while the
    // bytes are still arriving.
    void rx_event_callback() { RxBuffer::uart_idle_event_callback(); }

private:
    static void hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_complete_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);
};

// In D2 SRAM (.d2_sram, 0x30000000). Each object carries its own DMA rings, and
// DMA1/DMA2 -- which drive the UART RX streams and TX streams -- are
// D2-domain masters: keeping the rings in D2 SRAM keeps every transfer inside the
// domain instead of crossing the D2-to-D1 interconnect to the AXI SRAM and
// contending there with the M7. See the .d2_sram comment in
// bsp/linker/STM32H723VGTx_APP.ld for the second reason (AXI SRAM headroom under
// the non-cacheable MPU window) and for what app.cpp has to do at boot.
//
// Do NOT move these into .dtcm the way can.hpp places its objects: DTCM is
// reachable only by the core, so a DMA stream targeting it silently transfers
// nothing.
[[gnu::section(".d2_sram")]] inline constinit Uart::Lazy uart1{data::DataId::kUart1, &huart1};
[[gnu::section(".d2_sram")]] inline constinit Uart::Lazy uart7{data::DataId::kUart7, &huart7};
[[gnu::section(".d2_sram")]] inline constinit Uart::Lazy uart10{data::DataId::kUart10, &huart10};
[[gnu::section(".d2_sram")]] inline constinit UartRxOnly::Lazy uart_dbus{
    data::DataId::kUartDbus, &huart5};

// RS-485 ports on USART2 / USART3, named for the enclosure silkscreen (UART2 /
// UART3) rather than for a spare protocol slot. See UartRs485 for how they
// differ from the streaming ports above, and the kTurnaroundDeadline comment in
// tx_buffer.hpp for why.
//
// Settled against schematic CtrBoard-H7_V1.0-240124 sheet 5 (transceiver U5):
//
//   - No echo. Pins 2 (RE#) and 3 (DE) are tied to one net driven by
//     USART2_DE(PD04), so the receiver is off for the whole of our own
//     transmission and nothing we send comes back up. R11 pulls RO to 3V3
//     meanwhile, holding the idle level so the disabled receiver cannot present
//     a false start bit. Nothing upstream has to filter an echo.
//   - R17 pulls the DE net low, so the port powers up receiving and never
//     squats on the bus before software runs.
//   - R15 fits the 120R termination on-board, between A and B at this end.
//     Check the far end before adding another.
//   - DEAT/DEDT are 16/16 in the .ioc, one bit time either side at 4.8 Mbaud
//     with oversampling by 16, rather than CubeMX's default of 0.
//
// Both of the questions this comment used to leave open were answered on the
// bench with two boards cross-wired A-A / B-B on this port, running
// host/examples/rs485_cross_test with HCS_RS485_PORT=1. `[实测 2026-08-25]`
//
//   - Re-enabling the receiver at the end of our own transmission does NOT
//     raise a spurious IDLE. Had it, the turnaround gate would have released
//     early and collided; instead the half-duplex turnaround test kept 8 of 8
//     messages separate, and a silent bus delivered 0 bytes in 0 chunks over 5 s
//     at both ends. The kTurnaroundDeadline-only fallback is not needed.
//   - One bit time of DEAT is enough for this transceiver. A short DEAT would
//     truncate the leading edge first at the highest rate, and the baudrate
//     sweep passed both directions at every step through 4800000, with payloads
//     from 1 to 200 bytes and 200 ping-pong rounds at 0 lost / 0 corrupt.
//
// The USART3 port below now answers the same two questions the same way, on a
// one-board rig with USART2 wired to USART3 (HCS_RS485_RIG=single). Full suite
// ALL PASSED: sizes 1-200 B both directions, the sweep through 4800000, the
// turnaround gate, 5000 ping-pong rounds at 4800000 with 0 lost / 0 corrupt,
// collision recovery, a silent bus, and channel isolation. A one-sided baudrate
// change was checked to break the link, so the sweep is not passing on two ends
// that both ignored it. `[实测 2026-09-01]`
//
// What that run did find: a downlink the TX ring cannot hold is dropped with
// nothing but an LED to say so. 257 B and above never reach the bus (256 B and
// below are intact), and a burst of 32-byte commands loses everything past the
// ninth -- 12 queued deliver 9, 24 queued deliver 10, always the tail, never a
// truncated frame. The ring size is the documented design point above; the
// silence is not, and mc02 has the same gap on CAN (see the "未做" note in
// firmware/mc02/AGENTS.md: hpm_board's downlink flow control was never ported).
//
// Always compiled when libhcs_APP_RS485_ENABLE (the default). They are the
// enclosure's UART2 / UART3, not an overlay on a spare DataId. Diagnostic
// builds emit on kUart0, which is not a silkscreen port on this board, so they
// no longer collide with these two. Set the switch OFF to drop the objects,
// their 1.8 KB of D2 SRAM, and the per-pass NDTR polls.
#ifdef libhcs_APP_RS485_ENABLE
[[gnu::section(".d2_sram")]] inline constinit UartRs485::Lazy uart2{data::DataId::kUart2, &huart2};

// Second RS-485 port, on USART3 through transceiver U6 -- 485_DIR1 driven by
// USART3_DE on PB14, data on PD8/PD9, bus on connector P5. Electrically the same
// circuit as U5 above: RE# tied to DE, R12 pulling RO to 3V3 so the disabled
// receiver holds the idle level, R18 pulling the DE net low so the port powers
// up receiving, and R16 fitting the 120R termination at this end.
[[gnu::section(".d2_sram")]] inline constinit UartRs485::Lazy uart3{data::DataId::kUart3, &huart3};
#endif


} // namespace libhcs::firmware::uart
