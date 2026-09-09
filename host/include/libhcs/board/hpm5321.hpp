#pragma once

#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <stdexcept>
#include <string_view>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hcs_can_port.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/data/datas.hpp>
#include <libhcs/protocol/handler.hpp>
#include <libhcs/spec/hpm5321/can.hpp>
#include <libhcs/spec/hpm5321/uart.hpp>

namespace libhcs::board {

// Board interface for the HPM5321 boards. ONE class for BOTH PCBs -- the
// single-CAN one (PID 0x5321) and the dual-CAN one (0x5322) -- because they run
// one firmware image that tells itself apart from OTP word 25, and because the
// host learns how many buses are actually present by asking: kGetInterface over
// EP0 answers before the first session starts, so a compile-time bus count
// stopped being the only way to know. It has no IMU, no GPIO application
// channels and no DBUS, so those callbacks are intentionally absent.
//
// What the two PCBs still differ in is the port count, and that is checked at
// run time: transmitting on a bus this board does not have throws rather than
// going quietly nowhere.
class Hpm5321 final {
public:
    class Callback : public data::DataCallback {
    public:
        // Channel descriptors for this board. Addressing a channel through its
        // descriptor is what lets generic code reach data_id and, for UARTs,
        // config_data_id without a second lookup table.
        struct Spec {
            using Can = spec::hpm5321::CanDescriptor;
            static constexpr spec::hpm5321::internal::CanDescriptors kCans{};

            using Uart = spec::hpm5321::UartDescriptor;
            static constexpr spec::hpm5321::internal::UartDescriptors kUarts{};
        };

        struct View {
            using Can = data::CanDataView;
            using Uart = data::UartDataView;
            using UartConfig = data::UartConfigView;
            using ImuAccelerometer = data::ImuAccelerometerDataView;
            using ImuGyroscope = data::ImuGyroscopeDataView;
            using ImuTemperature = data::ImuTemperatureDataView;
        };

        // Receives from any CAN port, named as the enclosure labels it. One
        // callback with a port argument instead of one method per port: the old
        // per-port names were 0-based, so canN_receive_callback and the
        // connector printed CANN were never the same bus.
        //
        // Ports on this board: CanPort::kCan1, and CanPort::kCan2 on the
        // dual-CAN PCB. Read interface().can_count for which of them exist.
        virtual void can_receive(hcs::CanPort port, const libhcs::data::CanDataView& data) {
            (void)port;
            (void)data;
        }

        virtual void uart0_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }

        void accelerometer_receive_callback(
            const libhcs::data::ImuAccelerometerDataView& data) override {
            (void)data;
        }
        void gyroscope_receive_callback(const libhcs::data::ImuGyroscopeDataView& data) override {
            (void)data;
        }
        void temperature_receive_callback(
            const libhcs::data::ImuTemperatureDataView& data) override {
            (void)data;
        }

    public:
        bool can_receive_callback(data::DataId id, const data::CanDataView& data) final {
            switch (id) {
            case data::DataId::kCan1: can_receive(hcs::CanPort::kCan1, data); return true;
            case data::DataId::kCan2: can_receive(hcs::CanPort::kCan2, data); return true;
            default: return false;
            }
        }

        bool uart_receive_callback(data::DataId id, const data::UartDataView& data) final {
            switch (id) {
            case data::DataId::kUart0: uart0_receive_callback(data); return true;
            default: return false;
            }
        }

        bool gpio_digital_read_result_callback(
            uint8_t channel_index, const data::GpioDigitalDataView& data) final {
            (void)channel_index;
            (void)data;
            return false;
        }
        bool gpio_analog_read_result_callback(
            uint8_t channel_index, const data::GpioAnalogDataView& data) final {
            (void)channel_index;
            (void)data;
            return false;
        }
    };

    // Optional channel configuration, applied over EP0 and read back from the
    // hardware before this constructor returns. Leaving a field unset keeps
    // whatever the firmware brought that channel up with. A rejected baudrate,
    // or a CAN bus whose frame type is not the one asked for, throws here --
    // which is the whole reason configuration moved off the data stream, where
    // neither could be reported. See libhcs/board/hcs_config.hpp.
    using Configuration = hcs::Configuration;

    explicit Hpm5321(
        Callback& callback = default_callback_, std::string_view serial_filter = {},
        const AdvancedOptions& options = {}, const Configuration& configuration = {})
        : configuration_(configuration)
        , handler_(
              0xA511, kProductIds, serial_filter, options, callback,
              [this](host::protocol::Handler& handler) {
                  interface_ = hcs::apply(handler, configuration_);
              }) {}

    Hpm5321(const Hpm5321&) = delete;
    Hpm5321& operator=(const Hpm5321&) = delete;
    Hpm5321(Hpm5321&&) = delete;
    Hpm5321& operator=(Hpm5321&&) = delete;
    ~Hpm5321() = default;

    class PacketBuilder {
        friend class Hpm5321;

    public:
        // Transmits on the CAN port named as the enclosure labels it. Which
        // ports exist depends on the PCB, so the bound is the count the board
        // reported over EP0 rather than the image's capacity -- CAN2 on a
        // single-CAN board throws here instead of being serialized into a field
        // no firmware will ever read.
        PacketBuilder& can_transmit(hcs::CanPort port, const libhcs::data::CanDataView& data) {
            const auto index = static_cast<std::size_t>(port);
            if (index < 1 || index > can_count_) [[unlikely]]
                throw std::out_of_range{
                    std::format(
                        "Hpm5321: CAN port out of range (this board has CAN1..CAN{})",
                        can_count_)};
            static constexpr data::DataId kIds[]{data::DataId::kCan1, data::DataId::kCan2};
            if (!builder_.write_can(kIds[index - 1], data)) [[unlikely]]
                throw std::invalid_argument{"CAN transmission failed: Invalid CAN data"};
            return *this;
        }

        PacketBuilder& uart0_transmit(const libhcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart0, data)) [[unlikely]]
                throw std::invalid_argument{"UART0 transmission failed: Invalid UART data"};
            return *this;
        }

    private:
        PacketBuilder(host::protocol::Handler& handler, std::size_t can_count) noexcept
            : builder_(handler.start_transmit())
            , can_count_(can_count) {}

        host::protocol::Handler::PacketBuilder builder_;
        std::size_t can_count_;
    };
    // Whether this board's link is up, re-establishing, or gone for good.
    // kFaulted means the device disappeared: the transport refuses traffic and
    // only destroying this object and constructing a new one recovers it.
    [[nodiscard]] host::protocol::Handler::LinkState link_state() const noexcept {
        return handler_.link_state();
    }

    PacketBuilder start_transmit() noexcept {
        return PacketBuilder{handler_, interface_.can_count};
    }

    // Runtime reconfiguration, over EP0 rather than in the data stream. Unlike
    // the retired in-band config field these are synchronous and verified: the
    // call returns only once the board has confirmed the new setting, and
    // throws if it refused. They are NOT ordered against queued data -- bytes
    // already handed to the board go out at whichever rate the port reaches
    // them at, so quiesce the link before switching.
    void configure_uart0(uint32_t baudrate) { hcs::configure_uart(handler_, 0, baudrate); }

    // What UART0 is really running, reconstructed on the board from the
    // divisor actually programmed -- not the value that was last requested.
    uint32_t uart0_baudrate() { return hcs::read_uart_baudrate(handler_, 0); }

    // Frame type of each CAN bus, as the board reported it during construction.
    // It is a property of the bus, not of a frame: CanDataView::is_fdcan is
    // ignored by this board's firmware, which sends every frame in its bus's
    // mode. Read this instead of assuming.
    [[nodiscard]] bool can1_is_fd() const { return interface_.can_fd(0); }

    // Only meaningful when interface().can_count is 2.
    [[nodiscard]] bool can2_is_fd() const { return interface_.can_fd(1); }

    // The controller's own error registers for one CAN port. Read over EP0 on
    // the shipping image; see libhcs/board/hcs_config.hpp for how to read it.
    [[nodiscard]] hcs::vc::CanStatusPayload can_status(hcs::CanPort port) {
        return hcs::read_can_status(handler_, static_cast<std::size_t>(port) - 1);
    }

    // How much of a CAN round trip happens on the board. Cycles; divide by
    // cpu_hz. See libhcs/board/hcs_config.hpp.
    [[nodiscard]] hcs::vc::LatencyBreakdownPayload latency_breakdown(bool reset = false) {
        return hcs::read_latency_breakdown(handler_, reset);
    }

    // Channels the board reports it has. On a board directory that serves more
    // than one PCB (the hpm5321 image serves both the single- and dual-CAN
    // variants) this is the run-time truth, not the image's capacity.
    [[nodiscard]] const hcs::Interface& interface() const { return interface_; }

    // Cross-board timing measurement only; see Handler::send_pulse_schedule.
    // Not part of a control path: the board answers on the session stream, and
    // only firmware built with -Dlibhcs_PULSE_TEST=ON does anything with it.
    void send_pulse_schedule(uint64_t microframe) noexcept {
        handler_.send_pulse_schedule(microframe);
    }

private:
    // Both PCBs, in the order the scanner should prefer to report them. The
    // firmware picks its own product ID from OTP, so which one answers is a
    // property of the hardware, not of this call.
    static constexpr uint16_t kProductIdsStorage[]{0x5321, 0x5322};
    static constexpr std::span<const uint16_t> kProductIds{kProductIdsStorage};

    static inline Callback default_callback_{};
    // Both declared BEFORE handler_ on purpose. The before-session hook runs
    // while handler_ is still being constructed and touches both, so their
    // lifetimes must already have begun -- member initialisation runs in
    // declaration order. configuration_ is a COPY: the hook runs again on every
    // reconnect, long after the constructor argument has gone.
    hcs::Interface interface_{};
    Configuration configuration_;
    host::protocol::Handler handler_;
};

} // namespace libhcs::board
