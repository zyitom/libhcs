#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/link/interrupt_safe_buffer.hpp"
#include "firmware/hpm_board/app/src/sync/pulse.hpp"
#include "firmware/hpm_board/app/src/sync/timebase.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"
#include "firmware/hpm_board/app/src/uart/uart.hpp"

namespace libhcs::firmware::link {

// Host-session endpoint shared by every host transport of this application:
// the libhcs session lifecycle (kStart nonce + keepalive lease) and the
// downlink dispatch to the CAN/UART drivers, together with the uplink
// serializer and its interrupt-safe batch buffer. A transport (USB vendor
// class, EtherCAT PD stream) subclasses this and keeps only its transmission
// shape: how the current batch reaches the wire.
class HostSession
    : private core::protocol::DeserializeCallback
    , private core::utility::Immovable {
public:
    HostSession() = default;

    // 1000 ms session lease, in 0.25 us/tick units (4 MHz mchtmr).
    static constexpr uint64_t kSessionLeaseQuarterUs = 4'000'000U;

    core::protocol::Serializer& serializer() { return serializer_; }

    // True once the host session handshake (kStart nonce + keepalive lease) is
    // up and data is actually being forwarded -- distinct from mere transport
    // connectivity (USB enumeration / EtherCAT OP).
    bool session_established() const { return session_established_; }

    // True once the kStart ack has been queued: the uplink stream may carry
    // driver telemetry from this point on.
    bool uplink_enabled() const { return uplink_enabled_; }

    void handle_downlink(std::span<const std::byte> buffer) { deserializer_.feed(buffer); }

    // Drains the time base's pending hardware captures onto the uplink. Called
    // from the main loop; a no-op unless the time base is compiled in and a
    // probe frame has actually been captured.
    // Drains captured hardware pulses onto the uplink. The conversion needs the
    // history ring and a division, so it runs here rather than in the capture
    // interrupt.
    void poll_pulse_captures() {
        if constexpr (!sync::pulse::kEnabled)
            return;
        if (!session_established_)
            return;
        uint64_t captured_q16 = 0;
        while (sync::pulse::take_capture(captured_q16)) {
            (void)serializer_.write_pulse_report({
                .nonce = current_session_nonce_,
                .scheduled_microframe = pending_pulse_microframe_,
                .captured_microframe_q16 = captured_q16,
                .ticks_per_microframe_q16 = sync::pulse::measured_ticks_per_microframe_q16(),
                .flags = static_cast<uint8_t>(
                    (pending_pulse_armed_ ? data::kPulseArmed : data::PulseReportFlags{})
                    | data::kPulseCaptured),
            });
        }
    }

    void poll_sync_samples() {
        if constexpr (!sync::timebase::kEnabled)
            return;
        if (!session_established_)
            return;
        sync::timebase::CanCapture capture{};
        while (sync::timebase::take_can_capture(capture)) {
            // Best effort like every other telemetry path here: a full batch
            // pool drops the sample rather than stalling the loop.
            (void)serializer_.write_sync_sample({
                .nonce = current_session_nonce_,
                .tag = capture.tag,
                .microframe_q16 = capture.microframe_q16,
                .bus = capture.bus,
                .ptpc_ns = capture.ptpc_ns,
            });
        }
    }

    void finish_downlink_transfer() { deserializer_.finish_transfer(); }

    void deactivate_session() {
        uplink_enabled_ = false;
        session_established_ = false;
        session_deactivated_callback();
    }

protected:
    // Transport transmit path, step 1: refresh the session lease, then hand
    // out the batch currently in flight (popping the next pending one if none
    // is). Returns nullptr when the session is down or nothing is pending.
    // The batch stays current across calls until finish_batch(), so partial
    // transfers (USB packetization) and full-batch retries (ring
    // backpressure) both resume where they left off.
    const InterruptSafeBuffer::Batch* next_batch() {
        refresh_session_state();

        if (!session_established_)
            return nullptr;

        if (!transmitting_batch_)
            transmitting_batch_ = transmit_buffer_.pop_batch();
        return transmitting_batch_;
    }

    // Transport transmit path, step 2: the current batch fully reached the
    // wire; release it back to the pool.
    void finish_batch() {
        InterruptSafeBuffer::release_batch(transmitting_batch_);
        transmitting_batch_ = nullptr;
    }

    // A kStart with a new nonce reset the stream: the in-flight batch and all
    // pending batches were just dropped. Transports reset transfer progress.
    virtual void session_activated_callback() {}

    // Whether this transport will accept a kStart yet. A transport carrying an
    // out-of-band configuration handshake refuses until that handshake has
    // happened, so a host which never performed it cannot open a session and
    // then act on assumptions the board does not share -- concretely, a host
    // from before the frame type moved to EP0 would keep setting a per-frame
    // is_fdcan the firmware no longer reads, and get FD frames where it
    // expected classic ones with nothing reporting the mismatch.
    //
    // Default true, and it must stay so: the EtherCAT process-data transport
    // has no control endpoint to hold such a handshake on.
    virtual bool session_allowed() const { return true; }

    // The session ended -- lease expired, bus reset, or unplug. A transport
    // that gates on an out-of-band handshake forgets it here, so the NEXT host
    // has to perform its own: tud_mount_cb only fires on re-enumeration, and
    // swapping the host program without replugging the cable does not
    // re-enumerate. Without this, a host that never handshakes inherits the
    // previous host's -- measured, it walked straight through the gate.
    virtual void session_deactivated_callback() {}

    // Lets a transport drive a SECOND deserializer into this same session, for a
    // transport that carries the protocol on more than one pipe (the USB CAN
    // endpoint pair). The callbacks, and with them the session gating, stay
    // shared; only the framing state is per-pipe, which is exactly right --
    // interleaving two pipes' bytes into one deserializer would corrupt both.
    core::protocol::DeserializeCallback& deserialize_callback() { return *this; }

private:
    void activate_session(uint32_t nonce) {
        if (transmitting_batch_) {
            InterruptSafeBuffer::release_batch(transmitting_batch_);
            transmitting_batch_ = nullptr;
        }
        transmit_buffer_.clear();
        session_activated_callback();

        current_session_nonce_ = nonce;
        last_session_refresh_ = timer::Timer::timestamp64_quarter_us();
        session_established_ = true;
        uplink_enabled_ = false;
    }

    void refresh_session_state() {
        if (!session_established_)
            return;

        if (timer::Timer::timestamp64_quarter_us() - last_session_refresh_ < kSessionLeaseQuarterUs)
            return;

        deactivate_session();
    }

    bool can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        if (!session_established_)
            return true;
        // Bounded by can_count(), not kCanCount: this is host-supplied data, and
        // on the single-CAN hpm5321 the image still carries a second table slot
        // that was never constructed. A frame addressed to that unused DataId
        // (kCan2) must be rejected (false = "field not recognized on this board"),
        // not dispatched into an uninitialized Can.
        for (size_t i = 0; i < can::can_count(); i++) {
            if (can::kCanDataIds[i] != static_cast<data::DataId>(id))
                continue;
            can::can_array[i]->handle_downlink(data);
            return true;
        }
        return false;
    }

    bool uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        if (!session_established_)
            return true;
        for (auto& board_uart : uart::uart_array) {
            if (static_cast<core::protocol::FieldId>(board_uart->data_id()) == id) {
                board_uart->handle_downlink(data);
                return true;
            }
        }
        return false;
    }

    // In-band UART configuration. Retired on the USB transport: a rejected
    // baudrate could not be reported through this return value -- it means "this
    // field id was recognised", not "the operation succeeded" -- and echoing the
    // config field back on the uplink does not work either, because UartConfig
    // is a downlink-only channel by contract and the host's handler treats an
    // uplink one as a routing error, fails the deserializer and kills the link
    // (measured: UART went from PASS to 0/320). Configuration now rides EP0,
    // where the status stage carries the acknowledgement natively; see
    // usb/vendor_control.cpp and libhcs/protocol/vendor_control.hpp.
    bool uart_config_deserialized_callback(
        core::protocol::FieldId id, const data::UartConfigView& data) override {
        if (!session_established_)
            return true;
        // Refused, not ignored. false puts the deserializer into discard mode
        // for the rest of the transfer, which is the correct answer to a field
        // this transport no longer implements: silently dropping it would leave
        // a host believing it had switched a baudrate that never moved -- the
        // exact failure the move to EP0 exists to end.
        (void)id;
        (void)data;
        return false;
    }

    // This board has no GPIO application; GPIO commands from the host are ignored.
    bool gpio_digital_data_deserialized_callback(
        uint8_t channel_index, const data::GpioDigitalDataView& data) override {
        if (!session_established_)
            return true;
        (void)channel_index;
        (void)data;
        return false;
    }

    bool gpio_analog_data_deserialized_callback(
        uint8_t channel_index, const data::GpioAnalogDataView& data) override {
        if (!session_established_)
            return true;
        (void)channel_index;
        (void)data;
        return false;
    }

    bool gpio_digital_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (!session_established_)
            return true;
        (void)channel_index;
        (void)data;
        return false;
    }

    bool gpio_analog_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (!session_established_)
            return true;
        (void)channel_index;
        (void)data;
        return false;
    }

    void accelerometer_deserialized_callback(const data::ImuAccelerometerDataView& data) override {
        (void)data;
    }

    void gyroscope_deserialized_callback(const data::ImuGyroscopeDataView& data) override {
        (void)data;
    }

    void temperature_deserialized_callback(const data::ImuTemperatureDataView& data) override {
        (void)data;
    }

    void session_control_deserialized_callback(const data::SessionControlView& data) override {
        switch (data.type) {
        case data::SessionType::kStart: {
            // Silently refuse. The session protocol has no negative ack, and a
            // host that cannot open a session fails its own ack timeout in
            // about a second with a message of its own; saying nothing is the
            // only thing this layer can say.
            if (!session_allowed())
                return;

            const bool same_session = session_established_ && data.nonce == current_session_nonce_;

            if (!same_session)
                activate_session(data.nonce);
            else
                last_session_refresh_ = timer::Timer::timestamp64_quarter_us();

            const auto result = serializer_.write_session_control(
                {.type = data::SessionType::kStartAck, .nonce = data.nonce});
            core::utility::assert_always(
                result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            uplink_enabled_ = true;
            break;
        }
        case data::SessionType::kKeepalive:
            if (!session_established_ || data.nonce != current_session_nonce_)
                return;

            last_session_refresh_ = timer::Timer::timestamp64_quarter_us();
            {
                const auto result = serializer_.write_session_control(
                    {.type = data::SessionType::kKeepaliveAck, .nonce = data.nonce});
                core::utility::assert_always(
                    result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            }
            break;
        default: return;
        }
    }

    // Shared time base. The anchor rides the session field precisely because it
    // must live and die with the session: a board that lost its host has no
    // business keeping a timeline that the host may later assume is still
    // aligned. Nonce-checked like the keepalive, and answered in the same
    // exchange so the host gets the board's state without a second round trip.
    void time_anchor_deserialized_callback(const data::TimeAnchorView& data) override {
        if (!session_established_ || data.nonce != current_session_nonce_)
            return;

        sync::timebase::apply_anchor(data.microframe);

        const auto snapshot = sync::timebase::report();
        uint64_t ptpc_reference_units = 0;
        uint64_t ptpc_reference_microframe = 0;
        sync::timebase::ptpc_reference(ptpc_reference_units, ptpc_reference_microframe);
        (void)serializer_.write_time_status({
            .nonce = data.nonce,
            .microframe = snapshot.microframe,
            .timestamp_quarter_us = static_cast<uint32_t>(snapshot.timestamp_quarter_us),
            .ticks_per_microframe_q16 = snapshot.ticks_per_microframe_q16,
            .state = snapshot.state,
            .anomaly_count = snapshot.anomaly_count,
            .residual_mean_q16 = snapshot.residual_mean_q16,
            .residual_abs_max_q16 = snapshot.residual_abs_max_q16,
            .residual_count = static_cast<uint16_t>(snapshot.residual_count),
            .ptpc_units_per_microframe = sync::timebase::ptpc_units_per_microframe(),
            .ptpc_reference_units = ptpc_reference_units,
            .ptpc_reference_microframe = ptpc_reference_microframe,
            .ptpc_residual_mean = snapshot.ptpc_residual_mean,
            .ptpc_residual_abs_max = snapshot.ptpc_residual_abs_max,
            .ptpc_step_min = snapshot.ptpc_step_min,
            .ptpc_step_max = snapshot.ptpc_step_max,
            .ptpc_raw_ns = snapshot.ptpc_raw_ns,
            .ptpc_raw_microframe = snapshot.ptpc_raw_microframe,
        });
    }

    // Hardware pulse exchange. The host sends the identical target microframe to
    // every board; each board fires there and captures the others' pulses.
    void pulse_schedule_deserialized_callback(const data::PulseScheduleView& data) override {
        if (!session_established_ || data.nonce != current_session_nonce_)
            return;
        pending_pulse_microframe_ = data.microframe;
        pending_pulse_armed_ = sync::pulse::schedule(data.microframe);

        // Answer every schedule, armed or not. A board that cannot place the
        // target is silent otherwise, and on the host that is indistinguishable
        // from a board that fired and heard nothing back -- the two failures
        // that need opposite fixes.
        (void)serializer_.write_pulse_report({
            .nonce = current_session_nonce_,
            .scheduled_microframe = data.microframe,
            .captured_microframe_q16 = 0,
            .ticks_per_microframe_q16 = sync::pulse::measured_ticks_per_microframe_q16(),
            .flags = static_cast<uint8_t>(
                pending_pulse_armed_ ? data::kPulseArmed : data::PulseReportFlags{}),
        });
    }

    void error_callback() override {
        // Every transport below this layer delivers bytes losslessly (USB
        // bulk, EtherCAT stop-and-wait ARQ), so a deserialization error
        // implies a host/firmware framing bug. Recovery happens at the next
        // transfer boundary / link restart via finish_downlink_transfer().
    }

    core::protocol::Deserializer deserializer_{*this};

    InterruptSafeBuffer transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};

    const InterruptSafeBuffer::Batch* transmitting_batch_ = nullptr;
    bool uplink_enabled_ = false;
    bool session_established_ = false;
    uint32_t current_session_nonce_ = 0;
    uint64_t last_session_refresh_ = 0;
    uint64_t pending_pulse_microframe_ = 0;
    bool pending_pulse_armed_ = false;
};

} // namespace libhcs::firmware::link
