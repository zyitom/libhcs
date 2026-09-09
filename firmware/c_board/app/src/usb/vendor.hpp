#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>

#include <class/vendor/vendor_device.h>
#include <tusb.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/spec/c_board/gpio.hpp"
#include "core/include/libhcs/spec/gpio.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/can/can.hpp"
#include "firmware/c_board/app/src/gpio/gpio.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/c_board/app/src/uart/uart.hpp"
#include "firmware/c_board/app/src/usb/interrupt_safe_buffer.hpp"
#include "firmware/c_board/app/src/usb/usb_descriptors.hpp"
#include "firmware/c_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::usb {

void poll_dfu_runtime_reboot();

class Vendor
    : private core::protocol::DeserializeCallback
    , private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Vendor>;

    static constexpr size_t kMaxPacketSize = 64;
    static constexpr std::size_t kGpioChannelCount = std::size(spec::c_board::kGpioDescriptors);
    static constexpr auto kSessionLease = std::chrono::milliseconds{1000};

    Vendor() {
        usb::usb_descriptors.init();
        core::utility::assert_always(tusb_rhport_init(0, nullptr));
    }

    core::protocol::Serializer& serializer() { return serializer_; }

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

    bool try_transmit() {
        refresh_session_state();

        if (!session_established_) {
            return false;
        }

        if (!tud_vendor_n_write_available(0))
            return false;

        if (!transmitting_batch_) {
            transmitting_batch_ = transmit_buffer_.pop_batch();
        }
        if (!transmitting_batch_)
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
        last_session_refresh_ = timer::timer->timepoint48();
        session_established_ = true;
    }

    bool can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        if (!session_established_)
            return true;
        switch (id) {
        case data::DataId::kCan1: can::can1->handle_downlink(data); return true;
        case data::DataId::kCan2: can::can2->handle_downlink(data); return true;
        default: return false;
        }
    }

    bool uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        if (!session_established_)
            return true;
        switch (id) {
        case data::DataId::kUart1: uart::uart1->handle_downlink(data); return true;
        case data::DataId::kUart2: uart::uart2->handle_downlink(data); return true;
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
        case data::DataId::kUart2Config: return uart::uart2->handle_config(data);
        default: return false;
        }
    }

    bool gpio_digital_data_deserialized_callback(
        uint8_t channel_index, const data::GpioDigitalDataView& data) override {
        if (!session_established_)
            return true;
        if (data.timestamp_quarter_us.has_value())
            return false;
        if (channel_index >= kGpioChannelCount)
            return false;

        const auto& gpio = spec::c_board::kGpioDescriptors[channel_index];
        if (!gpio.supports(spec::GpioCapability::kDigitalWrite))
            return false;

        gpio::gpio->handle_digital_write(channel_index, data);
        return true;
    }

    bool gpio_analog_data_deserialized_callback(
        uint8_t channel_index, const data::GpioAnalogDataView& data) override {
        if (!session_established_)
            return true;
        if (channel_index >= kGpioChannelCount)
            return false;

        const auto& gpio = spec::c_board::kGpioDescriptors[channel_index];
        if (!gpio.supports(spec::GpioCapability::kAnalogWrite))
            return false;

        gpio::gpio->handle_analog_write(channel_index, data);
        return true;
    }

    bool gpio_digital_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (!session_established_)
            return true;
        if (channel_index >= kGpioChannelCount)
            return false;

        const auto& gpio = spec::c_board::kGpioDescriptors[channel_index];
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
                last_session_refresh_ = timer::timer->timepoint48();

            const auto result = serializer_.write_session_control(
                {.type = data::SessionType::kStartAck, .nonce = data.nonce});
            core::utility::assert_always(
                result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            break;
        }
        case data::SessionType::kKeepalive:
            if (!session_established_ || data.nonce != current_session_nonce_)
                return;

            last_session_refresh_ = timer::timer->timepoint48();
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
        // TODO: Report USB downlink deserialization errors through a dedicated error path.
    }

    void refresh_session_state() {
        if (!session_established_)
            return;

        if (!timer::timer->check_expired(
                last_session_refresh_, timer::Timer::to_duration48_checked(kSessionLease)))
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
    timer::Timer::TimePoint48 last_session_refresh_ = timer::Timer::TimePoint48::min();
};

// Placed in zero-wait CCM (.ccmram, copied at boot by App::App()) so the uplink
// serializer and USB batch buffers are written without touching the AHB bus or
// contending with DMA. FS USB is CPU PIO (no internal DMA), so this is CPU-only.
[[gnu::section(".ccmram")]] inline constinit Vendor::Lazy vendor;

} // namespace libhcs::firmware::usb
