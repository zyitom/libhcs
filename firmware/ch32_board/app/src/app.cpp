#include "firmware/ch32_board/app/src/app.hpp"

extern "C" {
#include "ch32h417.h"
#include "ch32h417_usbss_device.h"
#include "debug.h"
#include "usb_desc.h"
}

#include "firmware/ch32_board/app/src/can/can.hpp"
#include "firmware/ch32_board/app/src/led/led.hpp"
#include "firmware/ch32_board/app/src/timer/timer.hpp"
#include "firmware/ch32_board/app/src/uart/uart.hpp"
#include "firmware/ch32_board/app/src/usb/bench.hpp"
#include "firmware/ch32_board/app/src/usb/dfu_runtime.hpp"
#include "firmware/ch32_board/app/src/usb/vendor.hpp"
#include "firmware/ch32_board/boot/src/mailbox.hpp"

int main() { libhcs::firmware::app.init().run(); }

namespace libhcs::firmware {
namespace {

// Bring-up instrumentation for the USB 3.0 link state machine. The LTSSM state
// cannot be read with a debugger: halting resets the peripherals (see
// PITFALLS.md 4.4), so the only way to observe it is to have the firmware record
// it into RAM that survives a reset. Same 0x20170000 diag convention the V3F boot
// core uses, continuing past the words it owns (diag[0..10]).
//
// diag[11] bitmask of every LTSSM state seen, bit N = LINK_STATE value N
//          (0=U0 1=U1 2=U2 3=U3 4=DISABLE 5=RXDET 6=INACTIVE 7=POLLING
//           8=RECOVERY 9=HOTRST A=COMPLIANCE B=LOOPBACK)
// diag[12] last raw LINK_STATUS
// diag[13] number of LTSSM state transitions observed
// diag[14] USBSS_DevEnumStatus (non-zero once the host has configured us)
//
// TODO(usb-bringup): drop this once the SS link is understood and stable.
void poll_usb_link_diagnostics() {
    constexpr uintptr_t kBootDiagAddr = 0x20170000u;
    auto* diag = reinterpret_cast<volatile uint32_t*>(kBootDiagAddr);

    static uint32_t previous_state = 0xFFFFFFFFu;

    const uint32_t status = USBSSD->LINK_STATUS;
    const uint32_t state = (status & LINK_STATE_MASK) >> 8u;

    diag[11] |= 1u << state;
    diag[12] = status;
    if (state != previous_state) {
        previous_state = state;
        diag[13]++;
    }
    diag[14] = USBSS_DevEnumStatus;
}

// Bring-up tracer for the V5F init sequence. Every debugger halt resets this
// core (PITFALLS.md 4.4), so "where did it stop" cannot be read from the PC --
// but diag[30] lives in the shared-SRAM window no image links a section over, so
// it survives the reset and reads back as the last step that completed.
// TODO(usb-bringup): drop with the rest of the diag instrumentation.
void trace_init_step(uint32_t step) {
    auto* diag = reinterpret_cast<volatile uint32_t*>(0x20170000u);
    diag[30] = step;
}

uint32_t delay_hclk_clock() {
    if (HCLKClock != 0)
        return HCLKClock;

    const uint32_t fpre_shift[] = {0, 1, 2, 2};
    const uint32_t fpre = (RCC->CFGR0 & RCC_FPRE) >> 16;
    const uint32_t fallback = SystemCoreClock >> fpre_shift[fpre];
    return fallback != 0 ? fallback : HSI_VALUE;
}

} // namespace

App::App() {
    trace_init_step(1);
    SystemAndCoreClockUpdate();
    HCLKClock = delay_hclk_clock();
    Delay_Init();
    trace_init_step(2);
    USART_Printf_Init(921600);
    Chip = ((DBGMCU_GetCHIPID() >> 4) & 0x0F);
    trace_init_step(3);

    // Free-running timestamp source first: everything below may stamp events.
    timer::timer.init();
    trace_init_step(4);

    led::led.init();
    trace_init_step(5);

    // Every consumer the USB interrupt path touches must exist BEFORE the USBSS
    // interrupts are enabled: USBSS_Device_Init() arms the link state machine and
    // NVIC_EnableIRQ, and the very first LINK/EP interrupt reaches into
    // usb::vendor (and through handle_downlink into can/uart). Constructing them
    // afterwards races an already-live ISR against an uninitialised Lazy.
    usb::vendor.init();
    trace_init_step(6);

    for (auto& can : can::can_array) {
        can.init();
        trace_init_step(7);
    }
    for (auto& board_uart : uart::uart_array) {
        board_uart.init();
        trace_init_step(8);
    }

    // USB 3.0 SuperSpeed device bring-up (WCH USBSS controller), last. Once
    // USB_Timer_Init has armed the link-training helper timer and
    // USBSS_Device_Init has enabled the PHY, link and endpoints, the bulk
    // endpoints are the libhcs uplink/downlink and interrupts are live. The
    // string descriptors are built first: the host reads them during the
    // enumeration that USBSS_Device_Init kicks off.
    libhcs_usb_init_descriptors();
    trace_init_step(9);
    USB_Timer_Init();
    trace_init_step(10);

    // The link-diagnostic accumulators live outside every linked section, so
    // nothing zeroes them on reset -- do it before the first USB interrupt can
    // fire. See poll_usb_link_diagnostics().
    {
        auto* diag = reinterpret_cast<volatile uint32_t*>(0x20170000u);
        for (int i = 11; i <= 25; i++)
            diag[i] = 0;
    }

    USBSS_Device_Init(ENABLE);
    trace_init_step(11);

    // Dual-core handshake: this V5F core is the forwarding fast path. Signal the
    // V3F boot/offload core that bring-up is complete; V3F spins on this before
    // entering its offload loop. The mailbox itself is constructed by V3F before
    // it wakes us, so it is already valid here. See boot/src/main.cpp.
    boot::shared().v5f_ready = 1;
    trace_init_step(12);
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    // Liveness / loop-rate counter, diag[31]. Independent of USB: diag[13] only
    // moves while the LTSSM does, so it cannot distinguish "app died" from "link
    // settled". Free-run for a known time and divide.
    // TODO(usb-bringup): drop with the rest of the diag instrumentation.
    auto* diag = reinterpret_cast<volatile uint32_t*>(0x20170000u);
    uint32_t iterations = 0;

    while (true) {
        diag[31] = ++iterations;

        // Before any transmit: a link that went down took the armed IN chain
        // with it, and only this call can release the transmit gate afterwards
        // (see usb::ss::poll_link_reset). Cheap -- one register read.
        usb::ss::poll_link_reset();

#if libhcs_CH32_USB_BENCH
        // Bench builds keep the forwarding loop below running (so CAN/UART do
        // not spin down), but EP1 belongs to usb/bench.cpp: the protocol
        // transport is bypassed entirely and try_transmit() has nothing to send.
        usb::bench::poll();
#endif

        // Same shape as upstream hpm_board's run loop. CAN has no try_transmit:
        // downlink writes straight to the hardware TX mailboxes in
        // handle_downlink. The WCH USBSS and CAN/USART receive paths are all
        // interrupt-driven, so no polled USB task is needed (unlike TinyUSB's
        // tud_task()).
        usb::vendor->try_transmit();

        for (auto& board_uart : uart::uart_array)
            board_uart->try_transmit();

        // Drain V3F offload telemetry off the hot path: non-blocking (a single
        // acquire-load when empty), so it never stalls forwarding. This is the
        // cross-core mailbox consumer end.
        // TODO(usb-bringup): route these records into the USB SS uplink once the
        // SS data path is live and a telemetry DataId channel exists; for now
        // they are drained (and would be stamped via timer::timer on use).
        boot::shared().telemetry.pop_front_n(
            [](boot::TelemetryRecord&& record) noexcept { (void)record; });

        // Host asked us to reboot into DFU (see usb/dfu_runtime.cpp); handled
        // here rather than in the ISR so the control transfer completes first.
        usb::poll_dfu_runtime_reboot();

        poll_usb_link_diagnostics();
    }
}

} // namespace libhcs::firmware
