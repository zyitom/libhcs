#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#if defined(libhcs_APP_USB_FULL_SPEED) && libhcs_APP_USB_FULL_SPEED
# include <hpm_soc.h>
#endif

#include <class/vendor/vendor_device.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <device/usbd_pvt.h>
#include <tusb.h>

#include "board_app.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/link/host_session.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"
#include "firmware/hpm_board/app/src/usb/usb_descriptors.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::usb {

void poll_dfu_runtime_reboot();

// USB vendor-class host transport: TinyUSB bring-up plus the transmission
// shape (max-packet chunking + ZLP termination). Session lifecycle and
// downlink dispatch live in the shared link::HostSession.
class Vendor : public link::HostSession {
public:
    using Lazy = utility::Lazy<Vendor>;

    Vendor() {
        usb::usb_descriptors.init();

        const tusb_rhport_init_t init_config{
            .role = TUSB_ROLE_DEVICE,
            .speed = board::usb_use_high_speed() ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL,
        };
        core::utility::assert_always(tusb_rhport_init(0, &init_config));

#if defined(libhcs_APP_USB_FULL_SPEED) && libhcs_APP_USB_FULL_SPEED
        // The speed field above is advisory only: the ChipIdea device driver
        // reports whatever the port negotiated and never forces it (grep
        // dcd_ci_hs.c -- it only READS PORTSC1_PORT_SPEED). Forcing full speed
        // is a controller bit: PFSC disables the high-speed chirp so the port
        // can only ever come up at 12 Mbit. Set after tusb_rhport_init, because
        // dcd_init resets the controller and would clear it. [RM: PORTSC1.PFSC]
        HPM_USB0->PORTSC1 |= USB_PORTSC1_PFSC_MASK;
#endif

        // tusb_rhport_init -> dcd_init already enabled the USB IRQ; pin its
        // priority explicitly so the CAN(3) > USB(2) > UART(1) preemption
        // hierarchy is owned here and cannot silently regress if the SDK default
        // changes. With preemptive interrupts on, this lets a CAN RX (3) preempt
        // a running USB ISR (2) -- keeping motor feedback off the USB ISR's tail.
        intc_m_enable_irq_with_priority(IRQn_USB0, 2);
    }

    // USB transfers have boundaries (unlike the EtherCAT PD stream): a short
    // packet finishes the transfer, so framing recovery can resynchronize.
    void handle_downlink(std::span<const std::byte> buffer, bool finished) {
        link::HostSession::handle_downlink(buffer);
        if (finished)
            finish_downlink_transfer();
    }

    // Re-arm the bulk OUT endpoint, unless the CAN transmit queues are too close
    // to full to absorb another packet. Runs once per main-loop pass.
    //
    // WHY THIS IS THROTTLED AT ALL. Left to itself the class driver re-arms the
    // endpoint the moment tud_vendor_rx_cb() returns, so a board whose software
    // queue is full keeps accepting frames it can only drop: the loss is silent
    // and the host never learns to slow down. Leaving the endpoint un-armed
    // makes the controller NAK instead, which stalls the host's in-flight URBs
    // and, once its 64-transfer pool is exhausted, blocks the sender inside
    // acquire_transmit_buffer(). That is the only backpressure signal this
    // protocol has -- there is no flow-control field on the wire.
    //
    // WHY IT IS NOT A NO-LOSS GUARANTEE. One 512-byte OUT packet holds up to ~46
    // eight-byte CAN records (11 bytes each), or ~170 DLC-0 ones, against a
    // 64-deep queue -- and up to 64 packets are already in flight by the time the
    // queue starts filling. This bounds the sustained rate; it cannot make
    // overflow impossible. Sizing the watermark for a worst-case packet would
    // mean throttling below a quarter of the queue and would cost throughput on
    // every normal burst.
    // Compiles away entirely with CFG_TUD_VENDOR_RX_MANUAL_XFER=0, where the
    // class driver re-arms the endpoint itself and there is no backpressure.
    // Keeping both paths alive makes the feature a one-macro A/B rather than a
    // revert, which is how its cost on the packet rate gets measured at all.
    void poll_downlink_arm() {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        // Reads the answer cached by the main-loop hook rather than walking the
        // CAN queues here: this runs in the receive completion callback, and
        // anything added to that path costs 1.2-1.4x its own time in packet
        // rate. Evaluating the policy per packet measured 2.3% for nothing.
        if (throttle_active_) {
            arm_pending_ = true;
            return;
        }
        // Keep the debt outstanding if the endpoint would not take the transfer
        // (not open yet, or one already in flight); the main-loop hook below then
        // retries. This is also what performs the very first arm, which manual
        // mode leaves to the application and nothing else would do.
        arm_pending_ = !tud_vendor_n_read_xfer(0);
#endif
    }

    // Main-loop hook. Two loads and a return unless an arm is actually owed --
    // which, in steady state, it never is: the rx completion callback has already
    // re-armed. Calling the full policy from the main loop as well cost 2.3% of
    // the packet rate for nothing, because it queried the CAN queues a second
    // time on every packet. What still has to reach the main loop is the initial
    // arm before any packet has arrived, and the release once a throttled queue
    // drains -- neither of which has a completion callback to ride on.
    // Main-loop hook. Re-evaluates the throttle off the hot path and caches the
    // answer for the receive callback above, then settles any arm still owed.
    //
    // Steady state is two bool loads and a return. The policy is re-evaluated
    // only when it is already engaged (so it can release), when an arm is owed,
    // or on a coarse tick -- every 16 passes is about 21 us, against a queue
    // that needs milliseconds to drain and watermarks with 16 slots of
    // headroom, so nothing can cross unnoticed. Evaluating it on every pass
    // would be ~770k queue walks a second for no benefit.
    void poll_downlink_arm_if_pending() {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        if (arm_pending_ || throttle_active_ || (++throttle_tick_ & 0xFU) == 0U)
            throttle_active_ = downlink_throttled();
        audit_downlink_arm();
        if (arm_pending_)
            poll_downlink_arm();
#endif
    }

    // The debt above is the ONLY thing that ever re-arms the endpoint, and it is
    // set by callbacks -- mount, suspend, session teardown. Anything that
    // cancels the armed transfer WITHOUT one of those leaves the board deaf
    // forever: arm_pending_ is false, so the hook does nothing, while the
    // hardware holds no transfer. That is not hypothetical, it is how TinyUSB's
    // own vendord_set_itf() behaves -- it stalls and clears both bulk endpoints
    // to abort them, and the automatic re-arm right after is inside
    // `#if CFG_TUD_VENDOR_RX_MANUAL_XFER == 0`, which this build compiles out.
    //
    // So stop trusting the debt and look at the endpoint. Every 256 passes is
    // about 340 us; the failure it catches is permanent, so the sampling rate
    // only has to be fast enough that a human never sees it. Steady state is one
    // increment and a mask, because arm_pending_ is false and short-circuits
    // ahead of the endpoint query.
    void audit_downlink_arm() {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        if (arm_pending_ || throttle_active_)
            return;
        if ((++arm_audit_tick_ & 0xFFU) != 0U)
            return;
        if (!usbd_edpt_busy(0, UsbDescriptors::kEpnumCdc0DataOut))
            arm_pending_ = true;
#endif
    }

    // Re-owe the initial arm after the endpoints are torn down. Manual transfer
    // mode leaves the very first arm to the application, and in steady state no
    // arm is owed -- the rx completion callback has already re-armed, so
    // arm_pending_ is false. A bus reset then destroys the transfer the hardware
    // was holding, and nothing above would ever set the debt again: the hook is
    // gated on a debt that no longer exists, so the board transmits fine and
    // never receives another byte until the MCU restarts.
    //
    // Measured 2026-09-03: after the host rebooted with the boards still powered
    // on VBUS, both bulk OUT endpoints NAKed every byte while EP0 control
    // transfers stayed healthy, and every session died on SESSION_ACK.
    //
    // Setting the debt is safe from any context: poll_downlink_arm() keeps it
    // outstanding while tud_vendor_n_read_xfer() refuses, so the main loop
    // retries until the endpoint is open again.
    // Flipped by the EP0 configuration handler once this host has read the
    // board's interface descriptor (usb/vendor_control.cpp). Cleared on every
    // teardown so a new host must perform the handshake for itself rather than
    // inheriting the previous one's.
    void set_ep0_handshake_done(bool done) { ep0_handshake_done_ = done; }

    bool session_allowed() const override { return ep0_handshake_done_; }

    void session_deactivated_callback() override { ep0_handshake_done_ = false; }

    void reset_downlink_arm() {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        arm_pending_ = true;
#endif
    }

    bool try_transmit() {
        const auto* batch = next_batch();
        if (!batch)
            return false;

        if (!tud_vendor_n_write_available(0))
            return false;

        const auto data = batch->data();

        const std::size_t max_packet_size = max_packet_size_;
        const auto target_size = std::min(data.size() - transmitted_size_, max_packet_size);

        if (target_size) {
            const auto* src = reinterpret_cast<const uint8_t*>(data.data() + transmitted_size_);
            core::utility::assert_debug(tud_vendor_n_write(0, src, target_size) == target_size);
        } else {
            // TinyUSB 0.21 direct mode submits a ZLP through the normal write
            // API. Its return value is the transferred length, hence zero for
            // both a successful ZLP and an error; write_available() above has
            // already proved that the endpoint is mounted and unclaimed.
            static constexpr uint8_t kZlpByte = 0;
            (void)tud_vendor_n_write(0, &kZlpByte, 0);
        }

        transmitted_size_ += target_size;
        if (transmitted_size_ == data.size() && target_size < max_packet_size) {
            finish_batch();
            transmitted_size_ = 0;
        }

        return true;
    }

protected:
    void session_activated_callback() override {
        // Cache the endpoint size for the transmit paths. tud_speed_get() is a
        // real call -- it lives in usbd.c and is not inlined across the TU
        // boundary, so reading it per pass cost a jal/ret around a single byte
        // load, on a function app.cpp calls once per traffic source per pass.
        //
        // Safe to cache here because the speed is fixed by enumeration and a
        // session cannot exist before enumeration completed: the host has to
        // reach the board over this very endpoint to open one. A renegotiation
        // means a bus reset, which drops the session and runs this again.
        max_packet_size_ = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512U : 64U;

        transmitted_size_ = 0;
        // The CAN pipe's pool is separate, so HostSession's own reset does not
        // reach it: a stale batch here would be transmitted into the new session
        // and decoded against the wrong stream.
    }

private:
    // Hysteresis. Engage near the top of the queue, release only once it has
    // really drained: with a single threshold the endpoint would re-arm on the
    // pass after every single dequeue, and the throttle would chatter at exactly
    // the depth where latency is already worst.
    static constexpr size_t kThrottleEngageDepth = can::Can::kTransmitQueueSize * 3 / 4;
    static constexpr size_t kThrottleReleaseDepth = can::Can::kTransmitQueueSize / 4;

    // Escape hatch. A bus that has stopped draining -- bus-off, or simply no
    // other node to acknowledge -- would otherwise hold the OUT endpoint closed
    // forever. That endpoint also carries the UART downlink and the session
    // keepalive, so a fault on one CAN bus would take the whole link down with
    // it, which is far worse than dropping the frames aimed at that bus. After
    // this long with the queue still above the release watermark, stop
    // withholding. A healthy bus drains all 64 slots in ~3.2 ms at the measured
    // ~19.8k frames/s, so 20 ms only elapses when the bus really is stuck; the
    // session lease is 1000 ms, well clear of it.
    static constexpr uint64_t kThrottleDeadlineQuarterUs = 20'000U * 4U;

    bool downlink_throttled() {
        const size_t depth = can::max_transmit_queue_depth();

        // Backpressure was abandoned for this episode. Keep the pipe open until
        // the bus proves it is draining again -- re-engaging at the engage
        // watermark would just restart the same 20 ms stall in a loop.
        if (throttle_abandoned_) {
            if (depth <= kThrottleReleaseDepth)
                throttle_abandoned_ = false;
            return false;
        }

        if (!throttling_) {
            if (depth < kThrottleEngageDepth)
                return false;
            throttling_ = true;
            throttle_started_ = timer::Timer::timestamp64_quarter_us();
            return true;
        }

        if (depth <= kThrottleReleaseDepth) {
            throttling_ = false;
            return false;
        }

        if (timer::Timer::timestamp64_quarter_us() - throttle_started_
            >= kThrottleDeadlineQuarterUs) {
            throttling_ = false;
            throttle_abandoned_ = true;
            return false;
        }

        return true;
    }

    size_t transmitted_size_ = 0;
    // Endpoint size in bytes, refreshed on every session activation. The full
    // speed value is the safe default: a batch chunked at 64 bytes is correct
    // on a high speed endpoint too, only slower, whereas the reverse would
    // overrun the endpoint.
    std::size_t max_packet_size_ = 64;

    // Starts true: the endpoint has to be armed once before any packet can
    // arrive, and only the main-loop hook can do it.
    bool arm_pending_ = true;

    // See set_ep0_handshake_done().
    bool ep0_handshake_done_ = false;

    // Cached answer of downlink_throttled(), refreshed once per main-loop pass.
    bool throttle_active_ = false;
    uint32_t throttle_tick_ = 0;

    // Separate from throttle_tick_ on purpose: that one only advances on passes
    // where nothing else already triggered a re-evaluation, so it is not a clock.
    // See audit_downlink_arm().
    uint32_t arm_audit_tick_ = 0;
    bool throttling_ = false;
    bool throttle_abandoned_ = false;
    uint64_t throttle_started_ = 0;
};

inline constinit Vendor::Lazy vendor;

} // namespace libhcs::firmware::usb
