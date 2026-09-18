#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string_view>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hcs_can_port.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/data/datas.hpp>
#include <libhcs/protocol/handler.hpp>
#include <libhcs/spec/gpio.hpp>
#include <libhcs/spec/mc02/can.hpp>
#include <libhcs/spec/mc02/gpio.hpp>
#include <libhcs/spec/mc02/uart.hpp>

namespace libhcs::board {

/**
 * @brief High-level host board interface for mc02.
 *
 * This class owns the transport and protocol stack for a single board connection.
 * The supplied `Callback` is stored by reference, is not owned by the board, and must outlive the
 * board instance.
 *
 * The board may start transport I/O during construction, so receive callbacks may be invoked before
 * the board constructor returns.
 *
 * A common usage pattern is for an enclosing user type to inherit `Callback` and declare the board
 * as its last data member. In that arrangement, early callbacks may access base subobjects and
 * members whose initialization has already completed before the board member begins construction.
 *
 * @warning Early callbacks must not depend on invariants established later in an enclosing
 * constructor body, on post-construction configuration, or on the board object itself having
 * finished construction. Delay board construction with `std::optional` or `std::unique_ptr` when
 * callback behavior depends on such state.
 *
 * Channel configuration rides EP0, not the data stream: since the EP0
 * configuration channel reached this board (2026-09-12) the constructor applies
 * and verifies the requested settings through hcs::apply() before the first
 * session opens, and a rejected baudrate or an unexpected CAN bus frame type
 * throws here -- which is the whole reason configuration moved off the data
 * stream, where neither could be reported. The in-band `uartN_config()`
 * PacketBuilder methods are gone with it: the firmware refuses kUart*Config
 * fields, so a caller that kept using one would fail the link rather than
 * switch a baudrate.
 */
class Mc02 final {
public:
    class Callback : public data::DataCallback {
    public:
        // Channel descriptors for this board. Addressing a channel through its
        // descriptor is what lets generic code reach data_id and, for UARTs,
        // config_data_id without a second lookup table.
        struct Spec {
            using Can = spec::mc02::CanDescriptor;
            static constexpr spec::mc02::internal::CanDescriptors kCans{};

            using Uart = spec::mc02::UartDescriptor;
            static constexpr spec::mc02::internal::UartDescriptors kUarts{};

            using Gpio = spec::mc02::GpioDescriptor;
            static constexpr spec::mc02::internal::GpioDescriptors kGpios{};
        };

        struct View {
            using Can = data::CanDataView;
            using Uart = data::UartDataView;
            using UartConfig = data::UartConfigView;
            using GpioDigital = data::GpioDigitalDataView;
            using GpioAnalog = data::GpioAnalogDataView;
            using ImuAccelerometer = data::ImuAccelerometerDataView;
            using ImuGyroscope = data::ImuGyroscopeDataView;
            using ImuTemperature = data::ImuTemperatureDataView;
        };

        virtual void can1_receive_callback(const libhcs::data::CanDataView& data) { (void)data; }
        virtual void can2_receive_callback(const libhcs::data::CanDataView& data) { (void)data; }
        virtual void can3_receive_callback(const libhcs::data::CanDataView& data) { (void)data; }

        virtual void dbus_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }
        virtual void uart1_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }
        virtual void uart2_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }
        virtual void uart3_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }
        virtual void uart7_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }
        virtual void uart10_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }

        // Telemetry from a diagnostic firmware (CAN_DIAG / LOOP_PROFILE /
        // USB_RX_HIST), on kUart0 -- not a silkscreen port on this board, so it
        // collides with nothing. Ignored by default so ordinary applications are
        // unaffected by a diagnostic build.
        virtual void diagnostic_receive_callback(const libhcs::data::UartDataView& data) {
            (void)data;
        }

        virtual void gpio_digital_read_result_callback(
            const libhcs::spec::mc02::GpioDescriptor& gpio,
            const libhcs::data::GpioDigitalDataView& data) {
            (void)gpio;
            (void)data;
        }
        virtual void gpio_analog_read_result_callback(
            const libhcs::spec::mc02::GpioDescriptor& gpio,
            const libhcs::data::GpioAnalogDataView& data) {
            (void)gpio;
            (void)data;
        }

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
            case data::DataId::kCan1: can1_receive_callback(data); return true;
            case data::DataId::kCan2: can2_receive_callback(data); return true;
            case data::DataId::kCan3: can3_receive_callback(data); return true;
            default: return false;
            }
        }

        bool uart_receive_callback(data::DataId id, const data::UartDataView& data) final {
            switch (id) {
            case data::DataId::kUartDbus: dbus_receive_callback(data); return true;
            case data::DataId::kUart1: uart1_receive_callback(data); return true;
            case data::DataId::kUart2: uart2_receive_callback(data); return true;
            case data::DataId::kUart3: uart3_receive_callback(data); return true;
            case data::DataId::kUart7: uart7_receive_callback(data); return true;
            case data::DataId::kUart10: uart10_receive_callback(data); return true;
            // kUart0 is not a silkscreen UART on this board. Diagnostic builds
            // emit on it; accepting it matters because returning false makes the
            // deserializer treat the frame as a protocol error and tear the
            // session down, so a diagnostic build would kill the link it is meant
            // to be diagnosing.
            case data::DataId::kUart0: diagnostic_receive_callback(data); return true;
            default: return false;
            }
        }

        bool gpio_digital_read_result_callback(
            uint8_t channel_index, const data::GpioDigitalDataView& data) final {
            if (channel_index >= spec::mc02::kGpioDescriptors.size()) [[unlikely]]
                return false;
            gpio_digital_read_result_callback(spec::mc02::kGpioDescriptors[channel_index], data);
            return true;
        }

        bool gpio_analog_read_result_callback(
            uint8_t channel_index, const data::GpioAnalogDataView& data) final {
            if (channel_index >= spec::mc02::kGpioDescriptors.size()) [[unlikely]]
                return false;
            gpio_analog_read_result_callback(spec::mc02::kGpioDescriptors[channel_index], data);
            return true;
        }
    };

    // Optional channel configuration, applied over EP0 and read back from the
    // hardware before this constructor returns. Leaving a field unset keeps
    // whatever the firmware brought that channel up with. A rejected baudrate,
    // or a CAN bus whose frame type is not the one asked for, throws here --
    // which is the whole reason configuration moved off the data stream, where
    // neither could be reported. See libhcs/board/hcs_config.hpp.
    using Configuration = hcs::Configuration;

    explicit Mc02(
        Callback& callback = default_callback_, std::string_view serial_filter = {},
        const AdvancedOptions& options = {}, const Configuration& configuration = {})
        : configuration_(configuration)
        , handler_(
              0xA511, 0x0723, serial_filter, options, callback,
              [this](host::protocol::Handler& handler) {
                  const std::scoped_lock guard{reconfigure_mutex_};
                  interface_.store(hcs::apply(handler, configuration_), std::memory_order_release);
              }) {}

    Mc02(const Mc02&) = delete;
    Mc02& operator=(const Mc02&) = delete;
    Mc02(Mc02&&) = delete;
    Mc02& operator=(Mc02&&) = delete;
    ~Mc02() = default;

    class PacketBuilder {
        friend class Mc02;

    public:
        // Transmits on the silkscreen CAN port named by the method. The frame
        // type is a property of the BUS, not of a frame: the firmware puts every
        // frame on the wire in its compiled bus mode (CAN-FD on all three mc02
        // buses) and reports that mode over EP0 -- read can1_is_fd() and
        // friends instead of assuming. There is no per-frame flag to set.
        PacketBuilder& can1_transmit(const libhcs::data::CanDataView& data) {
            if (!builder_.write_can(data::DataId::kCan1, data)) [[unlikely]]
                throw std::invalid_argument{"CAN1 transmission failed: Invalid CAN data"};
            return *this;
        }
        PacketBuilder& can2_transmit(const libhcs::data::CanDataView& data) {
            if (!builder_.write_can(data::DataId::kCan2, data)) [[unlikely]]
                throw std::invalid_argument{"CAN2 transmission failed: Invalid CAN data"};
            return *this;
        }
        PacketBuilder& can3_transmit(const libhcs::data::CanDataView& data) {
            if (!builder_.write_can(data::DataId::kCan3, data)) [[unlikely]]
                throw std::invalid_argument{"CAN3 transmission failed: Invalid CAN data"};
            return *this;
        }

        PacketBuilder& uart1_transmit(const libhcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart1, data)) [[unlikely]]
                throw std::invalid_argument{"UART1 transmission failed: Invalid UART data"};
            return *this;
        }

        PacketBuilder& uart2_transmit(const libhcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart2, data)) [[unlikely]]
                throw std::invalid_argument{"UART2 transmission failed: Invalid UART data"};
            return *this;
        }

        PacketBuilder& uart3_transmit(const libhcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart3, data)) [[unlikely]]
                throw std::invalid_argument{"UART3 transmission failed: Invalid UART data"};
            return *this;
        }

        PacketBuilder& uart7_transmit(const libhcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart7, data)) [[unlikely]]
                throw std::invalid_argument{"UART7 transmission failed: Invalid UART data"};
            return *this;
        }

        PacketBuilder& uart10_transmit(const libhcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart10, data)) [[unlikely]]
                throw std::invalid_argument{"UART10 transmission failed: Invalid UART data"};
            return *this;
        }

        // mc02 exposes four PWM-capable pins, each usable as digital output,
        // PWM/analog output, or digital input (read configuration below).
        PacketBuilder& gpio_digital_write(
            const libhcs::spec::mc02::GpioDescriptor& gpio,
            const libhcs::data::GpioDigitalDataView& data) {
            if (!gpio.supports(spec::GpioCapability::kDigitalWrite)
                || !builder_.write_gpio_digital_data(gpio.channel_index, data)) [[unlikely]]
                throw std::invalid_argument{"GPIO digital transmission failed: Invalid GPIO data"};
            return *this;
        }
        PacketBuilder& gpio_analog_write(
            const libhcs::spec::mc02::GpioDescriptor& gpio,
            const libhcs::data::GpioAnalogDataView& data) {
            if (!gpio.supports(spec::GpioCapability::kAnalogWrite)
                || !builder_.write_gpio_analog_data(gpio.channel_index, data)) [[unlikely]]
                throw std::invalid_argument{"GPIO analog transmission failed: Invalid GPIO data"};
            return *this;
        }
        PacketBuilder& gpio_digital_read(
            const libhcs::spec::mc02::GpioDescriptor& gpio,
            const libhcs::data::GpioReadConfigView& data) {
            if (!data.supported(gpio)
                || !builder_.write_gpio_digital_read_config(gpio.channel_index, data)) [[unlikely]]
                throw std::invalid_argument{
                    "GPIO digital read configuration transmission failed: Invalid GPIO data"};
            return *this;
        }

    private:
        explicit PacketBuilder(host::protocol::Handler& handler) noexcept
            : builder_(handler.start_transmit()) {}

        host::protocol::Handler::PacketBuilder builder_;
    };
    // Whether this board's link is up, re-establishing, or gone for good.
    // kFaulted means the device disappeared: the transport refuses traffic and
    // only destroying this object and constructing a new one recovers it.
    [[nodiscard]] host::protocol::Handler::LinkState link_state() const noexcept {
        return handler_.link_state();
    }

    PacketBuilder start_transmit() noexcept { return PacketBuilder{handler_}; }

    // Runtime reconfiguration, over EP0 rather than in the data stream. Unlike
    // the retired in-band config field these are synchronous and verified: the
    // call returns only once the board has confirmed the new setting, and
    // throws if it refused (baudrate 0, or a rate whose divisor falls outside
    // the HAL's BRR bounds). They are NOT ordered against queued data -- bytes
    // already handed to the board go out at whichever rate the port reaches
    // them at, so quiesce the link before switching. The RS-485 ports keep
    // their own firmware-side turnaround discipline; nothing here paces them.
    void configure_uart1(uint32_t baudrate) { hcs::configure_uart(handler_, 1, baudrate); }
    void configure_uart2(uint32_t baudrate) { hcs::configure_uart(handler_, 2, baudrate); }
    void configure_uart3(uint32_t baudrate) { hcs::configure_uart(handler_, 3, baudrate); }
    void configure_uart7(uint32_t baudrate) { hcs::configure_uart(handler_, 4, baudrate); }
    void configure_uart10(uint32_t baudrate) { hcs::configure_uart(handler_, 5, baudrate); }
    void configure_dbus(uint32_t baudrate) { hcs::configure_uart(handler_, 0, baudrate); }

    // Full-setting forms: baudrate plus framing (word length 7/8, parity
    // none/even/odd, stop bits 1/2), each field optional -- zero fields leave
    // that aspect of the port untouched. See hcs::UartSetting for the coding
    // and configure_uart() for the verify semantics. Framing changes the byte
    // stream the far end sees: quiesce the link and reconfigure the peer in
    // the same breath.
    void configure_uart1(const hcs::UartSetting& setting) {
        hcs::configure_uart(handler_, 1, setting);
    }
    void configure_uart2(const hcs::UartSetting& setting) {
        hcs::configure_uart(handler_, 2, setting);
    }
    void configure_uart3(const hcs::UartSetting& setting) {
        hcs::configure_uart(handler_, 3, setting);
    }
    void configure_uart7(const hcs::UartSetting& setting) {
        hcs::configure_uart(handler_, 4, setting);
    }
    void configure_uart10(const hcs::UartSetting& setting) {
        hcs::configure_uart(handler_, 5, setting);
    }
    void configure_dbus(const hcs::UartSetting& setting) {
        hcs::configure_uart(handler_, 0, setting);
    }

    // What each port is really running, reconstructed on the board from the
    // divisor actually programmed -- not the value that was last requested.
    uint32_t uart1_baudrate() { return hcs::read_uart_baudrate(handler_, 1); }
    uint32_t uart2_baudrate() { return hcs::read_uart_baudrate(handler_, 2); }
    uint32_t uart3_baudrate() { return hcs::read_uart_baudrate(handler_, 3); }
    uint32_t uart7_baudrate() { return hcs::read_uart_baudrate(handler_, 4); }
    uint32_t uart10_baudrate() { return hcs::read_uart_baudrate(handler_, 5); }
    uint32_t dbus_baudrate() { return hcs::read_uart_baudrate(handler_, 0); }

    // Full read-back (rate + framing) for one port, indexed by the EP0 UART
    // numbering documented above.
    hcs::UartSetting read_uart_setting(std::size_t port) {
        return hcs::read_uart_setting(handler_, port);
    }

    // Runtime frame-type switch per CAN bus, over EP0. This board's firmware
    // APPLIES the requested mode (the controllers stay FD-capable, so only what
    // the bus transmits changes -- no controller re-init), then the mode is
    // read back and the call returns; it throws if the board refused. Like
    // every EP0 reconfiguration it is NOT ordered against queued data: quiesce
    // the link before switching a bus mid-traffic. Mind the far end too -- an
    // FD peer keeps decoding classic frames, but a classic-only peer errors on
    // FD frames, which is why the firmware default is FD for every bus. A
    // reconnect re-runs the construction Configuration: a bus it names is
    // switched back to that mode, one it leaves unset keeps whatever the board
    // runs by then (the firmware default if the board reset). canN_is_fd()
    // follows the board either way.
    void configure_can1(bool fd) { configure_can(0, fd); }
    void configure_can2(bool fd) { configure_can(1, fd); }
    void configure_can3(bool fd) { configure_can(2, fd); }

    // Frame type of each CAN bus. Seed from the construction handshake, kept in
    // step by the configure_canN() calls above; the wire itself carries no
    // per-frame type flag any more. Read this instead of assuming.
    [[nodiscard]] bool can1_is_fd() const {
        return interface_.load(std::memory_order_relaxed).can_fd(0);
    }
    [[nodiscard]] bool can2_is_fd() const {
        return interface_.load(std::memory_order_relaxed).can_fd(1);
    }
    [[nodiscard]] bool can3_is_fd() const {
        return interface_.load(std::memory_order_relaxed).can_fd(2);
    }

    // The controller's own error registers for one CAN port: TEC/REC, the last
    // protocol error, bus state flags, and the forwarded-frame count that tells
    // "the bus delivers nothing" from "the bus delivers but something drops".
    // Read over EP0 on the shipping image; see libhcs/board/hcs_config.hpp for
    // how to interpret it.
    [[nodiscard]] hcs::vc::CanStatusPayload can_status(hcs::CanPort port) {
        return hcs::read_can_status(handler_, static_cast<std::size_t>(port) - 1);
    }

    // WHY the board most recently refused a configuration request (see
    // LastConfigErrorPayload). Sticky until the next refusal or reboot.
    [[nodiscard]] hcs::vc::LastConfigErrorPayload last_config_error() {
        return hcs::read_last_config_error(handler_);
    }

    // One CAN bus's full timing identity over EP0: the TX mode in force, the
    // arbitration/data rates and sample points the controller is actually
    // timed for (1 Mbit/s / 5 Mbit/s / 875 per mille on this board), and the
    // capability bits. Read-only -- see configure_canN() for the one field a
    // host may change.
    [[nodiscard]] hcs::vc::CanConfigPayload can_config(hcs::CanPort port) {
        return hcs::read_can_config(handler_, static_cast<std::size_t>(port) - 1);
    }

    // Channels the board reports it has. This image carries all three CAN buses
    // and all six UART indexes, so this is the static truth -- but it is read
    // over EP0 like every other board's, which keeps the construction handshake
    // uniform (and is what the session gate on the firmware keys off). A
    // snapshot: the live copy is atomic -- see the member declaration below.
    [[nodiscard]] hcs::Interface interface() const {
        return interface_.load(std::memory_order_relaxed);
    }

private:
    // Shared body of configure_canN(): apply the mode over EP0, then mirror it
    // into the cached mask so canN_is_fd() stays truthful without another
    // round trip.
    //
    // Serialized against the reconnect hook as a whole, EP0 exchange
    // included. Making only the mirror atomic (a CAS) is not enough: if the
    // hook's apply() lands between this request and this mirror, the board
    // ends in the hook's mode while the mask says this call's. Under one lock,
    // whichever runs second defines both.
    void configure_can(std::size_t bus, bool fd) {
        const std::scoped_lock guard{reconfigure_mutex_};
        hcs::request_can_mode(handler_, bus, fd, true);
        auto interface = interface_.load(std::memory_order_relaxed);
        const auto bit = static_cast<uint8_t>(1U << bus);
        interface.can_fd_mask = fd ? static_cast<uint8_t>(interface.can_fd_mask | bit)
                                   : static_cast<uint8_t>(interface.can_fd_mask & ~bit);
        interface_.store(interface, std::memory_order_release);
    }

    // mc02 uses the shared HCS vendor id (0xA511) and the fixed board-type PID
    // 0x0723; per-device identity lives in the serial number, so pass a
    // serial_filter to target a specific board when several are connected.
    static inline Callback default_callback_{};
    // Declared BEFORE handler_ on purpose. The before-session hook runs while
    // handler_ is still being constructed and touches both, so their lifetimes
    // must already have begun -- member initialisation runs in declaration
    // order. configuration_ is a COPY: the hook runs again on every reconnect,
    // long after the constructor argument has gone.
    //
    // interface_ is atomic because it is written off the reading threads: the
    // hook re-learns it on the keepalive thread at every reconnect, and
    // configure_can() writes it from whichever thread calls it, while
    // canN_is_fd()/interface() read it concurrently. The writers serialize on
    // reconfigure_mutex_; readers never take it. See hcs_config.hpp's
    // static_assert.
    std::atomic<hcs::Interface> interface_;
    Configuration configuration_;
    std::mutex reconfigure_mutex_;
    host::protocol::Handler handler_;
};

} // namespace libhcs::board
