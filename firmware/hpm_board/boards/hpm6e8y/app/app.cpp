#include "firmware/hpm_board/boards/hpm6e8y/app/app.hpp"

#include <cstddef>
#include <cstdint>

#include <board.h>
#include <device/usbd.h>
#include <hpm_dma_mgr.h>
#include <hpm_l1c_drv.h>

#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/diag/can_diag.hpp"
#include "firmware/hpm_board/app/src/led/led.hpp"
#include "firmware/hpm_board/app/src/sync/pulse.hpp"
#include "firmware/hpm_board/app/src/sync/sof.hpp"
#include "firmware/hpm_board/app/src/sync/sof_probe.hpp"
#include "firmware/hpm_board/app/src/sync/timebase.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"
#include "firmware/hpm_board/app/src/uart/uart.hpp"
#include "firmware/hpm_board/app/src/usb/vendor.hpp"
#include "firmware/hpm_board/app/src/utility/boot_mailbox.hpp"
#include "firmware/hpm_board/app/src/utility/interrupt_lock.hpp"

int main() { libhcs::firmware::app.init().run(); }

namespace libhcs::firmware {

App::App() {
    {
        const utility::InterruptLockGuard guard;

        board_init();
        board_init_usb();
        dma_mgr_init();

        // Enable D-cache write-around: streaming writes bypass cache allocation,
        // keeping the 16 KiB D-cache available for hot control structures.
        l1c_dc_enable_writearound();

        boot::BootMailbox::clear();

        led::led.init();
        timer::timer.init();
    }

    {
        const utility::InterruptLockGuard guard;

        // Before the CAN and UART init() calls below: those arm driver ISRs that
        // serialize straight into the protocol stack, so the stack instance has
        // to exist first.
        //
        usb::vendor.init();

        // After usb::vendor.init(), never before: tud_init() -> dcd_init()
        // assigns USBINTR wholesale, so an earlier arm of the SOF enable would
        // be overwritten. A no-op unless the time base or the SOF probe is
        // compiled in.
        sync::sof_init();

        // Bounded by can_count(), not by the array size: on the hpm5321 image the
        // table is sized for the dual-CAN PCB and the single-CAN one leaves the
        // last slot unconstructed. Initializing it there would clock MCAN3 and
        // steal PA30/PA31 from the LED. The uninitialized Lazy stays inert.
        for (size_t i = 0; i < can::can_count(); ++i)
            can::can_array[i].init();

        for (auto& board_uart : uart::uart_array)
            board_uart.init();

        // After the CAN drivers, which are what bring PTPC0 up: this routes USB
        // Start-of-Frame into PTPC's hardware capture. A no-op unless the time
        // base is compiled in.
        sync::timebase::init_capture();

        // After the UART driver on purpose: this overrides UART0's pin mux to
        // put GPTMR0's compare output and capture input on those pads. UART0 is
        // unavailable in a build with the pulse test enabled.
        sync::pulse::init();
    }
}

namespace {

// LED source: steady green must mean "frames are being forwarded", so it
// follows the session that the CAN/UART drivers serialize into.
bool host_session_established() {
    return usb::vendor->session_established();
}

} // namespace

// Non-static to ensure instantiation
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    uint32_t last_tick = 0;
    while (true) {
        diag::note_main_loop();
        tud_task();

        // Drain the CAN software transmit queues immediately after tud_task(),
        // which is where TinyUSB delivers this pass's downlink frames. Placing
        // it later -- after the 1 kHz LED/telemetry block -- would put that
        // block's work between a frame arriving and reaching the wire.
        for (size_t i = 0; i < can::can_count(); ++i)
            can::can_array[i]->try_transmit();

        // Settle any USB bulk OUT arm still owed. The steady-state re-arm rides
        // on the receive completion callback instead; this only covers the first
        // arm and the release of a throttled endpoint, so it has to come after
        // the drain above -- that is what makes the queue look drained to the
        // throttle policy. See usb/vendor.hpp for the watermarks and the
        // bounded-stall escape.
        usb::vendor->poll_downlink_arm_if_pending();

        usb::poll_dfu_runtime_reboot();

        // LED bookkeeping runs here at the 1 kHz tick pace instead of inside the
        // mchtmr ISR: MTIP bypasses the PLIC priority threshold, so ISR-side work
        // would preempt even the priority-3 CAN ISR and tax the forwarding hot
        // path. The LED state reflects the session handshake (nonce + keepalive
        // lease), not mere USB enumeration: steady green means data is actually
        // being forwarded; an enumerated host without a live session stays on
        // the "waiting" blink.
        const uint32_t tick = timer::timer->tick_count();
        if (tick != last_tick) {
            last_tick = tick;
            led::led->set_host_connected(host_session_established());
            led::led->update(tick);

            // Shared time base: refit the microframe-to-local-timer line, and
            // re-arm the SOF enable so the hook survives a controller that was
            // reinitialized behind us. Both no-ops unless compiled in.
            sync::timebase::poll(tick);
            sync::pulse::poll(tick);
            sync::sof_rearm();

            // Ship any hardware Start-of-Frame captures the CAN receive path
            // queued. Paced off the 1 kHz tick because the probe runs at a few
            // hundred frames per second at most and the ring holds 16.
            usb::vendor->poll_sync_samples();
            usb::vendor->poll_pulse_captures();

            // USB SOF / FRINDEX validation telemetry (libhcs_APP_SOF_DIAG
            // builds only), on the same 1 kHz tick as the CAN telemetry below.
            sync::sof_probe::poll(tick);

            // CAN forwarding telemetry (libhcs_APP_CAN_DIAG builds only).
            // Paced off the same 1 kHz tick and emitted before the transport
            // pump below, so a record produced this tick leaves on this pass.
            diag::poll(tick);
        }

        // CAN interrupt-delivery watchdog. Must run every pass, not off the
        // 1 kHz tick: RX FIFO0 holds 32 elements, which at the rates this board
        // forwards is under two milliseconds of slack before frames are lost.
        for (size_t i = 0; i < can::can_count(); ++i)
            can::can_array[i]->poll();

        // Host transport pump.
        usb::vendor->try_transmit();

        for (auto& board_uart : uart::uart_array)
            board_uart->try_transmit();
    }
}

} // namespace libhcs::firmware
