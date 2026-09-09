#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/ch32_board/app/src/can/can.hpp"
#include "firmware/ch32_board/app/src/link/interrupt_safe_buffer.hpp"
#include "firmware/ch32_board/app/src/timer/timer.hpp"
#include "firmware/ch32_board/app/src/uart/uart.hpp"

namespace libhcs::firmware::link {

// Host-session endpoint shared by every host transport of this application:
// the libhcs session lifecycle (kStart nonce + keepalive lease) and the
// downlink dispatch to the CAN/UART drivers, together with the uplink
// serializer and its interrupt-safe batch buffer. A transport (USB SuperSpeed
// vendor class here) subclasses this and keeps only its transmission shape: how
// the current batch reaches the wire.
//
// Ported from hpm_board's link::HostSession -- same structure and semantics,
// with this board's timer / CAN / UART drivers substituted. Keep the two in
// sync; protocol behaviour belongs here, not in the transport.
class HostSession
    : private core::protocol::DeserializeCallback
    , private core::utility::Immovable {
public:
    HostSession() = default;

    static constexpr auto kSessionLease = std::chrono::milliseconds{1000};

    core::protocol::Serializer& serializer() { return serializer_; }

    // True once the host session handshake (kStart nonce + keepalive lease) is
    // up and data is actually being forwarded -- distinct from mere transport
    // connectivity (USB enumeration).
    bool session_established() const { return session_established_; }

    // True once the kStart ack has been queued: the uplink stream may carry
    // driver telemetry from this point on.
    bool uplink_enabled() const { return uplink_enabled_; }

    void handle_downlink(std::span<const std::byte> buffer) { deserializer_.feed(buffer); }

    void finish_downlink_transfer() { deserializer_.finish_transfer(); }

    void deactivate_session() {
        uplink_enabled_ = false;
        session_established_ = false;
    }

protected:
    // Transport transmit path, step 1: refresh the session lease, then hand
    // out the batch currently in flight (popping the next pending one if none
    // is). Returns nullptr when the session is down or nothing is pending.
    // The batch stays current across calls until finish_batch(), so partial
    // transfers (USB packetization) and full-batch retries (ring backpressure)
    // both resume where they left off.
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

private:
    void activate_session(uint32_t nonce) {
        if (transmitting_batch_) {
            InterruptSafeBuffer::release_batch(transmitting_batch_);
            transmitting_batch_ = nullptr;
        }
        transmit_buffer_.clear();
        session_activated_callback();

        current_session_nonce_ = nonce;
        last_session_refresh_ = timer::timer->timepoint();
        session_established_ = true;
        uplink_enabled_ = false;
    }

    void refresh_session_state() {
        if (!session_established_)
            return;

        if (!timer::timer->check_expired(last_session_refresh_, kSessionLease))
            return;

        deactivate_session();
    }

    bool can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        if (!session_established_)
            return true;
        for (auto& can : can::can_array) {
            if (static_cast<core::protocol::FieldId>(can->data_id()) == id) {
                can->handle_downlink(data);
                return true;
            }
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

    // Runtime UART reconfiguration, addressed by config_data_id rather than
    // data_id so a config patch and a payload for the same port stay distinct.
    // handle_config's own bool reports whether the patch carried a usable
    // baudrate; the field was still ours either way, so the dispatch result does
    // not depend on it.
    bool uart_config_deserialized_callback(
        core::protocol::FieldId id, const data::UartConfigView& data) override {
        if (!session_established_)
            return true;
        for (auto& board_uart : uart::uart_array) {
            if (static_cast<core::protocol::FieldId>(board_uart->config_data_id()) == id) {
                board_uart->handle_config(data);
                return true;
            }
        }
        return false;
    }

    // This board has no GPIO application; GPIO commands from the host are ignored.
    bool gpio_digital_data_deserialized_callback(
        uint8_t, const data::GpioDigitalDataView&) override {
        return !session_established_;
    }

    bool gpio_analog_data_deserialized_callback(uint8_t, const data::GpioAnalogDataView&) override {
        return !session_established_;
    }

    bool gpio_digital_read_config_deserialized_callback(
        uint8_t, const data::GpioReadConfigView&) override {
        return !session_established_;
    }

    bool gpio_analog_read_config_deserialized_callback(
        uint8_t, const data::GpioReadConfigView&) override {
        return !session_established_;
    }

    void accelerometer_deserialized_callback(const data::ImuAccelerometerDataView&) override {}
    void gyroscope_deserialized_callback(const data::ImuGyroscopeDataView&) override {}
    void temperature_deserialized_callback(const data::ImuTemperatureDataView&) override {}

    void session_control_deserialized_callback(const data::SessionControlView& data) override {
        switch (data.type) {
        case data::SessionType::kStart: {
            const bool same_session = session_established_ && data.nonce == current_session_nonce_;

            if (!same_session)
                activate_session(data.nonce);
            else
                last_session_refresh_ = timer::timer->timepoint();

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

            last_session_refresh_ = timer::timer->timepoint();
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

    void error_callback() override {
        // USB bulk delivers bytes losslessly, so a deserialization error implies
        // a host/firmware framing bug. Recovery happens at the next transfer
        // boundary via finish_downlink_transfer().
    }

    core::protocol::Deserializer deserializer_{*this};

    InterruptSafeBuffer transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};

    const InterruptSafeBuffer::Batch* transmitting_batch_ = nullptr;
    bool uplink_enabled_ = false;
    bool session_established_ = false;
    uint32_t current_session_nonce_ = 0;
    timer::Timer::TimePoint last_session_refresh_ = timer::Timer::TimePoint::min();
};

} // namespace libhcs::firmware::link
