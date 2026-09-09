#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>

#include <class/vendor/vendor_device.h>
#include <device/usbd.h>
#include <main.h>
#include <tusb.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/can/can.hpp"
#include "firmware/mc02/app/src/gpio/gpio.hpp"
#include "firmware/mc02/app/src/sync/sof.hpp"
#include "firmware/mc02/app/src/sync/timebase.hpp"
#include "firmware/mc02/app/src/timer/timer.hpp"
#include "firmware/mc02/app/src/uart/uart.hpp"
#include "firmware/mc02/app/src/usb/interrupt_safe_buffer.hpp"
#include "firmware/mc02/app/src/usb/usb_descriptors.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

namespace libhcs::firmware::usb {

void poll_dfu_runtime_reboot();

class Vendor
    : private core::protocol::DeserializeCallback
    , private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Vendor>;

    static constexpr size_t kMaxPacketSize = 64;
    static constexpr auto kSessionLease = std::chrono::milliseconds{1000};

    Vendor() {
        usb::usb_descriptors.init();

        // Pin the USB interrupt priority here, because nothing else does, and
        // do it before the controller can raise one: tusb_rhport_init ->
        // dcd_init -> dcd_int_enable only calls NVIC_EnableIRQ, so setting the
        // priority afterwards would leave a window at the reset value. The
        // CubeMX HAL_PCD_MspInit that would have set a priority belongs to the
        // ST device stack, which this firmware does not link. NVIC priority
        // registers reset to 0, so without this line OTG_HS would run at
        // preempt priority 0 -- above FDCAN (1) -- and every CAN RX ISR could be
        // delayed by a full dwc2 interrupt (FIFO drain plus the endpoint state
        // machine). Setting it to 2 makes the documented FDCAN(1) > USB(2) >
        // UART/DMA(3) hierarchy true in the image, and keeps ownership of it
        // here where it cannot silently regress.
        HAL_NVIC_SetPriority(OTG_HS_IRQn, 2, 0);

        core::utility::assert_always(tusb_rhport_init(0, nullptr));
    }

    core::protocol::Serializer& serializer() { return serializer_; }

    // Session keepalive, lifted out of try_transmit().
    //
    // kSessionLease is 1 s, so testing it once per main-loop pass is already
    // about a thousand times finer than the thing it measures. It used to run at
    // the top of try_transmit(), i.e. nine times a pass, and each run read TIM5's
    // CNT through Timer::timepoint() -- a D2 peripheral access, in the domain USB
    // shares, on the board's hottest loop. Single call site is app.cpp's loop.
    void poll_session() { refresh_session_state(); }

    // True once the host has completed the nonce handshake and is holding the
    // keepalive lease, i.e. data is actually being forwarded. Distinct from mere
    // USB enumeration, which says nothing about whether a host is talking.
    bool session_established() const { return session_established_; }

    void deactivate_session() { session_established_ = false; }

    void handle_downlink(std::span<const std::byte> buffer, bool finished) {
        deserializer_.feed(buffer);
        if (finished)
            deserializer_.finish_transfer();
    }

    void finish_downlink_transfer() { deserializer_.finish_transfer(); }

    // Ordered cheapest-test-first, and deliberately does NOT refresh the session
    // -- see poll_session() below.
    //
    // The batch pool is plain RAM; tud_vendor_n_write_available() reads TinyUSB's
    // endpoint state, also RAM. Neither is worth doing when nothing is staged,
    // and app.cpp calls this once per traffic source -- several times a pass -- so
    // anything ahead of the "is there work" test is paid nine times over.
    // hpm_board's Vendor::try_transmit is ordered the same way.
    bool try_transmit() {
        if (!session_established_) {
            return false;
        }

        if (!transmitting_batch_) {
            transmitting_batch_ = transmit_buffer_.pop_batch();
        }
        if (!transmitting_batch_)
            return false;

        if (!tud_vendor_n_write_available(0))
            return false;

        const auto data = transmitting_batch_->data();

        const auto target_size = std::min(data.size() - transmitted_size_, kMaxPacketSize);

        const auto* src = reinterpret_cast<const uint8_t*>(data.data() + transmitted_size_);

        if (target_size) {
            core::utility::assert_debug(tud_vendor_n_write(0, src, target_size) == target_size);
        } else {
            // Terminate a batch whose length is an exact multiple of the endpoint size.
            // In non-buffered vendor mode (RX/TX_BUFSIZE == 0) TinyUSB submits a
            // zero-length write straight to the endpoint as a ZLP. The return value is 0
            // both on success and on a failed endpoint claim, so there is nothing to
            // assert; write_available() above already confirmed the endpoint is idle.
            tud_vendor_n_write(0, src, 0);
        }

        transmitted_size_ += target_size;
        if (transmitted_size_ == data.size() && target_size < kMaxPacketSize) {
            transmit_buffer_.release_batch(transmitting_batch_);
            transmitting_batch_ = nullptr;
            transmitted_size_ = 0;
        }

        return true;
    }

private:
    void activate_session(uint32_t nonce) {
        if (transmitting_batch_) {
            transmit_buffer_.release_batch(transmitting_batch_);
            transmitting_batch_ = nullptr;
            transmitted_size_ = 0;
        }
        transmit_buffer_.clear();

        current_session_nonce_ = nonce;
        last_session_refresh_ = timer::timer->timepoint();
        session_established_ = true;
    }

    bool can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        if (!session_established_)
            return true;
        switch (id) {
        case data::DataId::kCan1: can::can1->handle_downlink(data); return true;
        case data::DataId::kCan2: can::can2->handle_downlink(data); return true;
        case data::DataId::kCan3: can::can3->handle_downlink(data); return true;
        default: return false;
        }
    }

    bool uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        if (!session_established_)
            return true;
        switch (id) {
        case data::DataId::kUart1: uart::uart1->handle_downlink(data); return true;
#ifdef libhcs_APP_RS485_ENABLE
        case data::DataId::kUart2: uart::uart2->handle_downlink(data); return true;
        case data::DataId::kUart3: uart::uart3->handle_downlink(data); return true;
#endif
        case data::DataId::kUart7: uart::uart7->handle_downlink(data); return true;
        case data::DataId::kUart10: uart::uart10->handle_downlink(data); return true;
        default: return false;
        }
    }

    bool uart_config_deserialized_callback(
        core::protocol::FieldId id, const data::UartConfigView& data) override {
        if (!session_established_)
            return true;
        switch (id) {
        case data::DataId::kUartDbusConfig: return uart::uart_dbus->handle_config(data);
        case data::DataId::kUart1Config: return uart::uart1->handle_config(data);
#ifdef libhcs_APP_RS485_ENABLE
        case data::DataId::kUart2Config: return uart::uart2->handle_config(data);
        case data::DataId::kUart3Config: return uart::uart3->handle_config(data);
#endif
        case data::DataId::kUart7Config: return uart::uart7->handle_config(data);
        case data::DataId::kUart10Config: return uart::uart10->handle_config(data);
        default: return false;
        }
    }

    bool gpio_digital_data_deserialized_callback(
        uint8_t channel_index, const data::GpioDigitalDataView& data) override {
        if (!session_established_)
            return true;
        if (data.timestamp_quarter_us.has_value())
            return false;
        if (channel_index >= spec::mc02::kGpioDescriptors.size())
            return false;
        if (!spec::mc02::kGpioDescriptors[channel_index].supports(
                spec::GpioCapability::kDigitalWrite))
            return false;
        gpio::gpio->handle_digital_write(channel_index, data);
        return true;
    }

    bool gpio_analog_data_deserialized_callback(
        uint8_t channel_index, const data::GpioAnalogDataView& data) override {
        if (!session_established_)
            return true;
        if (channel_index >= spec::mc02::kGpioDescriptors.size())
            return false;
        if (!spec::mc02::kGpioDescriptors[channel_index].supports(
                spec::GpioCapability::kAnalogWrite))
            return false;
        gpio::gpio->handle_analog_write(channel_index, data);
        return true;
    }

    bool gpio_digital_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (!session_established_)
            return true;
        if (channel_index >= spec::mc02::kGpioDescriptors.size())
            return false;
        const auto& gpio = spec::mc02::kGpioDescriptors[channel_index];
        if (!data.supported(gpio))
            return false;
        gpio::gpio->handle_digital_read(channel_index, data);
        return true;
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
            const bool same_session = session_established_ && data.nonce == current_session_nonce_;

            if (!same_session)
                activate_session(data.nonce);
            else
                last_session_refresh_ = timer::timer->timepoint();

            const auto result = serializer_.write_session_control(
                {.type = data::SessionType::kStartAck, .nonce = data.nonce});
            core::utility::assert_always(
                result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
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

    // Shared time base. The anchor rides the session field precisely because it
    // must live and die with the session: a board that lost its host has no
    // business keeping a timeline that the host may later assume is still
    // aligned. Nonce-checked like the keepalive, and answered in the same
    // exchange so the host gets the board's state without a second round trip.
    //
    // The PTPC fields are hpm_board's CAN-timestamp-to-microframe bridge, which
    // has no counterpart on this part: the H723's FDCAN timestamp counter is not
    // fed by a disciplinable 1588 clock, and nothing here captures CAN frames on
    // the shared axis. Sent as zeros rather than omitted, because the payload
    // layout is shared by every board.
    //
    // The one exception is ptpc_units_per_microframe, which carries the exact
    // analogue of what it means on hpm_board: the rate of the board's HARDWARE
    // capture clock against the microframe axis. Here that is TIM2, and zero
    // means the capture is not being used because it is too coarse to beat the
    // interrupt path (sync/sof.cpp). It is the one field that tells the host
    // whether the .ioc still has TIM2 at the servo prescaler.
    void time_anchor_deserialized_callback(const data::TimeAnchorView& data) override {
        if (!session_established_ || data.nonce != current_session_nonce_)
            return;

        sync::timebase::apply_anchor(data.microframe);

        const auto snapshot = sync::timebase::report();

        // Report the pair as of NOW, not as of the last SOF.
        //
        // snapshot.microframe is the counter latched at the last Start-of-Frame,
        // which at full speed is 0..1 ms in the past -- uniform, mean 500 us.
        // The host pairs whatever we send with the MIDPOINT of this round trip,
        // so reporting the last SOF makes it associate that microframe with an
        // instant ~450 us later than the microframe actually was, and adds the
        // uniform 1 ms of that delay as noise on top.
        // [Measured 2026-09-07 by the causality probe in
        //  host/examples/mc02_time_sync_test.cpp --causality: the conversion
        //  landed +440..+491 us outside its own send/reply bracket on 100% of
        //  2185 probes, and the placement residual was uniform over exactly
        //  1 ms with sigma 288 us = 1000/sqrt(12).]
        //
        // Interpolating to the current instant costs one TIM5 read and one
        // multiply, and it is what microframe_at() is for. Falls back to the
        // latched pair when the timeline is not valid, which is the only case
        // where interpolation would be meaningless.
        auto reported_microframe = snapshot.microframe;
        auto reported_quarter_us = static_cast<uint32_t>(snapshot.timestamp_quarter_us);
        {
            const auto now_quarter_us =
                static_cast<uint32_t>(timer::timer->timepoint().time_since_epoch().count());
            uint64_t microframe_now = 0;
            if (sync::timebase::microframe_at(now_quarter_us, microframe_now)) {
                reported_microframe = microframe_now;
                reported_quarter_us = now_quarter_us;
            }
        }

        (void)serializer_.write_time_status({
            .nonce = data.nonce,
            .microframe = reported_microframe,
            .timestamp_quarter_us = reported_quarter_us,
            .ticks_per_microframe_q16 = snapshot.ticks_per_microframe_q16,
            .state = snapshot.state,
            .anomaly_count = snapshot.anomaly_count,
            .residual_mean_q16 = snapshot.residual_mean_q16,
            .residual_abs_max_q16 = snapshot.residual_abs_max_q16,
            .residual_count = static_cast<uint16_t>(snapshot.residual_count),
            .ptpc_units_per_microframe = sync::sof_capture_active()
                                           ? sync::timebase::kNominalCyclesPerMicroframe
                                                 / sync::sof_capture_cycles_per_tick()
                                           : 0,
            .ptpc_reference_units = 0,
            .ptpc_reference_microframe = 0,
            .ptpc_residual_mean = 0,
            .ptpc_residual_abs_max = 0,
            .ptpc_step_min = 0,
            .ptpc_step_max = 0,
            .ptpc_raw_ns = 0,
            .ptpc_raw_microframe = 0,
        });
    }

    void error_callback() override {
        // TODO: Report USB downlink deserialization errors through a dedicated error path.
    }

    void refresh_session_state() {
        if (!session_established_)
            return;

        if (!timer::timer->check_expired(last_session_refresh_, kSessionLease))
            return;

        deactivate_session();
    }

    core::protocol::Deserializer deserializer_{*this};

    InterruptSafeBuffer transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};

    const InterruptSafeBuffer::Batch* transmitting_batch_ = nullptr;
    size_t transmitted_size_ = 0;
    bool session_established_ = false;
    uint32_t current_session_nonce_ = 0;
    timer::Timer::TimePoint last_session_refresh_ = timer::Timer::TimePoint::min();
};

// Placed in zero-wait DTCM (.dtcm, copied at boot) so the forwarding ISR writes
// the serializer/USB batch buffers without ever touching the AXI bus.
[[gnu::section(".dtcm")]] inline constinit Vendor::Lazy vendor;

} // namespace libhcs::firmware::usb
