#pragma once

// Board-agnostic runtime adapter for the example programs.
//
// Every example used to hard-code one concrete board type. This header lets an
// example talk to whichever of the five project boards is actually plugged in,
// chosen at runtime: c_board, mc02, ch32_board, hpm5321 and hpm5321_dual_can.
//
// Usage:
//   1. Derive a receiver from `examples::BoardReceiver` and override the
//      `on_*` hooks you care about (bus / port indices are 0-based: bus 0 is the
//      board's primary CAN, port 0 its primary UART).
//   2. Call `examples::connect_any(receiver)` to open the first board found.
//   3. Query capabilities (`can_bus_count()`, `gpio_channel_count()`, ...) and
//      transmit with `session->transmit([&](auto& tx){ tx.can(0, ...); })`.
//      A single `transmit` call batches every write into one USB packet, exactly
//      like chaining `board.start_transmit().canN_transmit(...)` would.
//
// Boards have different capability surfaces, so an example that needs a feature a
// board lacks (e.g. two CAN buses, or GPIO) should check the capability queries
// and bail out cleanly when it is missing. Calling an unsupported transmit (an
// out-of-range bus, or GPIO on a board without it) throws.

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <libhcs/board/c_board.hpp>
#include <libhcs/board/ch32_board.hpp>
#include <libhcs/board/mc02.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/data/datas.hpp>


// CAN ports are named as the silkscreen on THAT board. 5321 prints CAN1/CAN2.
// See libhcs/board/hcs_can_port.hpp.
using libhcs::board::hcs::CanPort;

namespace examples {

// Minimal non-owning callable view, so `transmit` can take a lambda without a
// heap allocation per packet (std::function would allocate for larger captures).
template <class Signature>
class FunctionRef;
template <class R, class... Args>
class FunctionRef<R(Args...)> {
public:
    template <class F>
    FunctionRef(F&& functor) noexcept // NOLINT(google-explicit-constructor)
        : object_(const_cast<void*>(static_cast<const void*>(std::addressof(functor))))
        , invoke_([](void* object, Args... args) -> R {
            return (*static_cast<std::remove_reference_t<F>*>(object))(
                std::forward<Args>(args)...);
        }) {}

    R operator()(Args... args) const { return invoke_(object_, std::forward<Args>(args)...); }

private:
    void* object_;
    R (*invoke_)(void*, Args...);
};

// Board-neutral receive sink. Override only what the example needs.
class BoardReceiver {
public:
    BoardReceiver() = default;
    BoardReceiver(const BoardReceiver&) = delete;
    BoardReceiver& operator=(const BoardReceiver&) = delete;
    virtual ~BoardReceiver() = default;

    virtual void on_can(int bus, const libhcs::data::CanDataView& data) {
        (void)bus;
        (void)data;
    }
    virtual void on_uart(int port, const libhcs::data::UartDataView& data) {
        (void)port;
        (void)data;
    }
    virtual void on_dbus(const libhcs::data::UartDataView& data) { (void)data; }
    // GPIO digital read result. Boards report the level of a channel that was
    // armed with gpio_digital_read(); channel indices match gpio_analog/digital.
    // Board telemetry from a diagnostics build (hpm_board's libhcs_CAN_DIAG or
    // mc02's libhcs_APP_CAN_DIAG), carried on an otherwise unused UART id.
    virtual void on_diagnostic(const libhcs::data::UartDataView& data) { (void)data; }
    virtual void on_gpio_digital(int channel, const libhcs::data::GpioDigitalDataView& data) {
        (void)channel;
        (void)data;
    }
    virtual void on_accelerometer(const libhcs::data::ImuAccelerometerDataView& data) { (void)data; }
    virtual void on_gyroscope(const libhcs::data::ImuGyroscopeDataView& data) { (void)data; }
    virtual void on_temperature(const libhcs::data::ImuTemperatureDataView& data) { (void)data; }
};

// Board-neutral packet builder, handed to the `transmit` callback. Writes are
// flushed as one USB packet when the callback returns.
class BoardTransmitter {
public:
    virtual ~BoardTransmitter() = default;

    virtual BoardTransmitter& can(int bus, const libhcs::data::CanDataView& data) = 0;
    virtual BoardTransmitter& uart(int port, const libhcs::data::UartDataView& data) = 0;
    // Runtime UART reconfiguration (baudrate). Boards whose UART baud is fixed
    // at compile time still need this when they are wired to a board configured
    // differently -- an mc02 UART1 at 115200 talking to an hpm5321 UART0 at
    // 921600 delivers nothing but framing errors until one side is changed.
    virtual BoardTransmitter& uart_config(
        int port, const libhcs::data::UartConfigView& config) {
        (void)port;
        (void)config;
        throw std::logic_error{"this board has no runtime UART configuration"};
    }

    virtual BoardTransmitter& gpio_analog(
        int channel, const libhcs::data::GpioAnalogDataView& data) {
        (void)channel;
        (void)data;
        throw std::logic_error{"this board has no GPIO analog-write channels"};
    }
    virtual BoardTransmitter& gpio_digital(
        int channel, const libhcs::data::GpioDigitalDataView& data) {
        (void)channel;
        (void)data;
        throw std::logic_error{"this board has no GPIO digital-write channels"};
    }
    // Arm a channel as a digital input. Without this the adapter could only write
    // pins, so a GPIO test could confirm that a write was accepted but never that
    // the pin actually moved -- which needs a wire to an input and a read back.
    virtual BoardTransmitter& gpio_digital_read(
        int channel, const libhcs::data::GpioReadConfigView& config) {
        (void)channel;
        (void)config;
        throw std::logic_error{"this board has no GPIO read channels"};
    }
};

// Type-erased handle to whichever board connected. Owns the transport stack.
class BoardSession {
public:
    virtual ~BoardSession() = default;

    virtual std::string_view name() const = 0;
    virtual int can_bus_count() const = 0;
    virtual int uart_port_count() const = 0;
    virtual bool has_dbus() const = 0;
    virtual bool has_imu() const = 0;
    virtual int gpio_channel_count() const = 0;

    virtual void transmit(FunctionRef<void(BoardTransmitter&)> build) = 0;
};

namespace detail {

// AdvancedOptions is non-copyable and its setters return a reference to the
// temporary, so the fluent expression is built inline at each board construction
// (it lives to the end of the full constructor call) rather than factored out.

// c_board: CAN2/CAN3, UART1/UART2, DBUS, GPIO (PWM + read), IMU.
class CBoardSession final : public BoardSession, public libhcs::board::CBoard::Callback {
public:
    explicit CBoardSession(BoardReceiver& receiver, std::string_view filter)
        : receiver_(receiver)
        , board_(
              *this, filter,
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true)) {}

    std::string_view name() const override { return "CBoard"; }
    int can_bus_count() const override { return 2; }
    int uart_port_count() const override { return 2; }
    bool has_dbus() const override { return true; }
    bool has_imu() const override { return true; }
    int gpio_channel_count() const override {
        return static_cast<int>(libhcs::spec::c_board::kGpioDescriptors.size());
    }

    void transmit(FunctionRef<void(BoardTransmitter&)> build) override {
        auto builder = board_.start_transmit();
        Transmitter transmitter{builder};
        build(transmitter);
    }

private:
    struct Transmitter final : BoardTransmitter {
        libhcs::board::CBoard::PacketBuilder& builder;
        explicit Transmitter(libhcs::board::CBoard::PacketBuilder& b) : builder(b) {}
        BoardTransmitter& can(int bus, const libhcs::data::CanDataView& data) override {
            switch (bus) {
            case 0: builder.can1_transmit(data); break;
            case 1: builder.can2_transmit(data); break;
            default: throw std::out_of_range{"CBoard: CAN bus index out of range (0-1)"};
            }
            return *this;
        }
        BoardTransmitter& uart(int port, const libhcs::data::UartDataView& data) override {
            switch (port) {
            case 0: builder.uart1_transmit(data); break;
            case 1: builder.uart2_transmit(data); break;
            default: throw std::out_of_range{"CBoard: UART port index out of range (0-1)"};
            }
            return *this;
        }
        // Bounds-checked like can()/uart() above: kGpioDescriptors is a fixed
        // array, so an out-of-range channel read past its end and handed a
        // garbage descriptor to the board. The firmware validates the channel
        // index too, but only after this side has already committed the
        // out-of-bounds read, which is UB regardless of what the board does.
    private:
        static const auto& gpio_descriptor(int channel) {
            const auto& descriptors = libhcs::spec::c_board::kGpioDescriptors;
            if (channel < 0 || static_cast<size_t>(channel) >= descriptors.size())
                throw std::out_of_range{"CBoard: GPIO channel out of range"};
            return descriptors[static_cast<size_t>(channel)];
        }

    public:
        BoardTransmitter& gpio_analog(
            int channel, const libhcs::data::GpioAnalogDataView& data) override {
            builder.gpio_analog_write(gpio_descriptor(channel), data);
            return *this;
        }
        BoardTransmitter& gpio_digital(
            int channel, const libhcs::data::GpioDigitalDataView& data) override {
            builder.gpio_digital_write(gpio_descriptor(channel), data);
            return *this;
        }
        BoardTransmitter& gpio_digital_read(
            int channel, const libhcs::data::GpioReadConfigView& config) override {
            builder.gpio_digital_read(gpio_descriptor(channel), config);
            return *this;
        }

    };

    void can1_receive_callback(const libhcs::data::CanDataView& data) override {
        receiver_.on_can(0, data);
    }
    void can2_receive_callback(const libhcs::data::CanDataView& data) override {
        receiver_.on_can(1, data);
    }
    void uart1_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_uart(0, data);
    }
    void uart2_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_uart(1, data);
    }
    void dbus_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_dbus(data);
    }
    void accelerometer_receive_callback(
        const libhcs::data::ImuAccelerometerDataView& data) override {
        receiver_.on_accelerometer(data);
    }
    void gyroscope_receive_callback(const libhcs::data::ImuGyroscopeDataView& data) override {
        receiver_.on_gyroscope(data);
    }
    void temperature_receive_callback(const libhcs::data::ImuTemperatureDataView& data) override {
        receiver_.on_temperature(data);
    }

    void gpio_digital_read_result_callback(
        const libhcs::spec::c_board::GpioDescriptor& gpio,
        const libhcs::data::GpioDigitalDataView& data) override {
        receiver_.on_gpio_digital(static_cast<int>(gpio.channel_index), data);
    }
    BoardReceiver& receiver_;
    libhcs::board::CBoard board_;
};

// mc02: CAN1/2/3, silkscreen UART1/2/3/7/10 plus DBUS, GPIO (PWM write only),
// IMU (accel + gyro, no temperature). UART2/UART3 are the enclosure RS-485
// ports (USART2/USART3). Dense BoardTransmitter indices are 0=UART1, 1=UART2,
// 2=UART3, 3=UART7, 4=UART10.
class Mc02Session final : public BoardSession, public libhcs::board::Mc02::Callback {
public:
    explicit Mc02Session(BoardReceiver& receiver, std::string_view filter)
        : receiver_(receiver)
        , board_(
              *this, filter,
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true)) {}

    std::string_view name() const override { return "Mc02"; }
    int can_bus_count() const override { return 3; }
    int uart_port_count() const override { return 5; }
    bool has_dbus() const override { return true; }
    bool has_imu() const override { return true; }
    int gpio_channel_count() const override {
        return static_cast<int>(libhcs::spec::mc02::kGpioDescriptors.size());
    }

    void transmit(FunctionRef<void(BoardTransmitter&)> build) override {
        auto builder = board_.start_transmit();
        Transmitter transmitter{builder, board_};
        build(transmitter);
    }

private:
    // UART configuration no longer rides the transmit stream for this board: it
    // moved to the EP0 control channel, where the board's answer (verified with
    // a read-back) is the acknowledgement the bulk stream cannot carry. The
    // transmitter's uart_config therefore reconfigures synchronously and throws
    // when the board refuses, instead of queueing an in-band field.
    struct Transmitter final : BoardTransmitter {
        libhcs::board::Mc02::PacketBuilder& builder;
        libhcs::board::Mc02& board;
        Transmitter(libhcs::board::Mc02::PacketBuilder& b, libhcs::board::Mc02& board_ref)
            : builder(b)
            , board(board_ref) {}
        BoardTransmitter& can(int bus, const libhcs::data::CanDataView& data) override {
            switch (bus) {
            case 0: builder.can1_transmit(data); break;
            case 1: builder.can2_transmit(data); break;
            case 2: builder.can3_transmit(data); break;
            default: throw std::out_of_range{"Mc02: CAN bus index out of range (0-2)"};
            }
            return *this;
        }
        BoardTransmitter& uart_config(
            int port, const libhcs::data::UartConfigView& config) override {
            if (!config.baudrate.has_value())
                return *this; // sparse patch semantics: nothing asked, nothing done
            switch (port) {
            case 0: board.configure_uart1(*config.baudrate); break;
            case 1: board.configure_uart2(*config.baudrate); break;
            case 2: board.configure_uart3(*config.baudrate); break;
            case 3: board.configure_uart7(*config.baudrate); break;
            case 4: board.configure_uart10(*config.baudrate); break;
            default: throw std::out_of_range{"Mc02: UART port out of range (0-4)"};
            }
            return *this;
        }
        BoardTransmitter& uart(int port, const libhcs::data::UartDataView& data) override {
            switch (port) {
            case 0: builder.uart1_transmit(data); break;
            case 1: builder.uart2_transmit(data); break;
            case 2: builder.uart3_transmit(data); break;
            case 3: builder.uart7_transmit(data); break;
            case 4: builder.uart10_transmit(data); break;
            default: throw std::out_of_range{"Mc02: UART port index out of range (0-4)"};
            }
            return *this;
        }
        // See the CBoard equivalent: unchecked indexing here was an out-of-bounds
        // read on a fixed array, independent of the firmware's own validation.
    private:
        static const auto& gpio_descriptor(int channel) {
            const auto& descriptors = libhcs::spec::mc02::kGpioDescriptors;
            if (channel < 0 || static_cast<size_t>(channel) >= descriptors.size())
                throw std::out_of_range{"Mc02: GPIO channel out of range"};
            return descriptors[static_cast<size_t>(channel)];
        }

    public:
        BoardTransmitter& gpio_analog(
            int channel, const libhcs::data::GpioAnalogDataView& data) override {
            builder.gpio_analog_write(gpio_descriptor(channel), data);
            return *this;
        }
        BoardTransmitter& gpio_digital(
            int channel, const libhcs::data::GpioDigitalDataView& data) override {
            builder.gpio_digital_write(gpio_descriptor(channel), data);
            return *this;
        }
        BoardTransmitter& gpio_digital_read(
            int channel, const libhcs::data::GpioReadConfigView& config) override {
            builder.gpio_digital_read(gpio_descriptor(channel), config);
            return *this;
        }

    };

    void can1_receive_callback(const libhcs::data::CanDataView& data) override {
        receiver_.on_can(0, data);
    }
    void can2_receive_callback(const libhcs::data::CanDataView& data) override {
        receiver_.on_can(1, data);
    }
    void can3_receive_callback(const libhcs::data::CanDataView& data) override {
        receiver_.on_can(2, data);
    }
    void uart1_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_uart(0, data);
    }
    void uart2_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_uart(1, data);
    }
    void uart3_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_uart(2, data);
    }
    void uart7_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_uart(3, data);
    }
    void uart10_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_uart(4, data);
    }
    void dbus_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_dbus(data);
    }
    void accelerometer_receive_callback(
        const libhcs::data::ImuAccelerometerDataView& data) override {
        receiver_.on_accelerometer(data);
    }
    void gyroscope_receive_callback(const libhcs::data::ImuGyroscopeDataView& data) override {
        receiver_.on_gyroscope(data);
    }

    void diagnostic_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_diagnostic(data);
    }

    void gpio_digital_read_result_callback(
        const libhcs::spec::mc02::GpioDescriptor& gpio,
        const libhcs::data::GpioDigitalDataView& data) override {
        receiver_.on_gpio_digital(static_cast<int>(gpio.channel_index), data);
    }
    BoardReceiver& receiver_;
    libhcs::board::Mc02 board_;
};

// ch32_board: CAN2/CAN3 and UART1/UART2 over a USB 3.0 SuperSpeed bulk pipe, no
// IMU / DBUS / GPIO. The logical channels are numbered from 1 on this board (it
// follows the STM32 boards' peripheral numbering), so bus 0 here is CAN2.
class Ch32BoardSession final : public BoardSession, public libhcs::board::Ch32Board::Callback {
public:
    explicit Ch32BoardSession(BoardReceiver& receiver, std::string_view filter)
        : receiver_(receiver)
        , board_(
              *this, filter,
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true)) {}

    std::string_view name() const override { return "Ch32Board"; }
    int can_bus_count() const override { return 2; }
    int uart_port_count() const override { return 2; }
    bool has_dbus() const override { return false; }
    bool has_imu() const override { return false; }
    int gpio_channel_count() const override { return 0; }

    void transmit(FunctionRef<void(BoardTransmitter&)> build) override {
        auto builder = board_.start_transmit();
        Transmitter transmitter{builder};
        build(transmitter);
    }

private:
    struct Transmitter final : BoardTransmitter {
        libhcs::board::Ch32Board::PacketBuilder& builder;
        explicit Transmitter(libhcs::board::Ch32Board::PacketBuilder& b) : builder(b) {}
        BoardTransmitter& can(int bus, const libhcs::data::CanDataView& data) override {
            switch (bus) {
            case 0: builder.can1_transmit(data); break;
            case 1: builder.can2_transmit(data); break;
            default: throw std::out_of_range{"Ch32Board: CAN bus index out of range (0-1)"};
            }
            return *this;
        }
        BoardTransmitter& uart(int port, const libhcs::data::UartDataView& data) override {
            switch (port) {
            case 0: builder.uart1_transmit(data); break;
            case 1: builder.uart2_transmit(data); break;
            default: throw std::out_of_range{"Ch32Board: UART port index out of range (0-1)"};
            }
            return *this;
        }
    };

    void can1_receive_callback(const libhcs::data::CanDataView& data) override {
        receiver_.on_can(0, data);
    }
    void can2_receive_callback(const libhcs::data::CanDataView& data) override {
        receiver_.on_can(1, data);
    }
    void uart1_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_uart(0, data);
    }
    void uart2_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_uart(1, data);
    }

    BoardReceiver& receiver_;
    libhcs::board::Ch32Board board_;
};

// hpm5321: a single CAN1 and UART0, no IMU / DBUS / GPIO.
class Hpm5321Session final : public BoardSession,
                             public libhcs::board::Hpm5321::Callback {
public:
    explicit Hpm5321Session(BoardReceiver& receiver, std::string_view filter)
        : receiver_(receiver)
        , board_(
              *this, filter,
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true)) {}

    std::string_view name() const override { return "Hpm5321"; }
    int can_bus_count() const override { return 1; }
    int uart_port_count() const override { return 1; }
    bool has_dbus() const override { return false; }
    bool has_imu() const override { return false; }
    int gpio_channel_count() const override { return 0; }

    void transmit(FunctionRef<void(BoardTransmitter&)> build) override {
        auto builder = board_.start_transmit();
        Transmitter transmitter{builder, board_};
        build(transmitter);
    }

private:
    struct Transmitter final : BoardTransmitter {
        libhcs::board::Hpm5321::PacketBuilder& builder;
        libhcs::board::Hpm5321& board;
        Transmitter(
            libhcs::board::Hpm5321::PacketBuilder& b,
            libhcs::board::Hpm5321& board_ref)
            : builder(b)
            , board(board_ref) {}
        BoardTransmitter& can(int bus, const libhcs::data::CanDataView& data) override {
            if (bus != 0)
                throw std::out_of_range{"Hpm5321: only CAN bus 0 exists"};
            builder.can_transmit(CanPort::kCan1, data);
            return *this;
        }
        BoardTransmitter& uart(int port, const libhcs::data::UartDataView& data) override {
            if (port != 0)
                throw std::out_of_range{"Hpm5321: only UART port 0 exists"};
            builder.uart0_transmit(data);
            return *this;
        }
        // Goes out on EP0, not in this batch: hpm_board configuration left the
        // data stream so that a rejected baudrate could be reported. It is
        // therefore applied IMMEDIATELY, ahead of anything queued in the builder
        // around it, and it throws instead of failing silently.
        BoardTransmitter& uart_config(
            int port, const libhcs::data::UartConfigView& config) override {
            if (port != 0)
                throw std::out_of_range{"Hpm5321: only UART port 0 exists"};
            if (config.baudrate.has_value())
                board.configure_uart0(*config.baudrate);
            return *this;
        }
    };

    void can_receive(
        CanPort port, const libhcs::data::CanDataView& data) override {
        switch (port) {
        case CanPort::kCan1: {
            receiver_.on_can(0, data);
            break;
        }
        default: break;
        }
    }
    void uart0_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_uart(0, data);
    }

    BoardReceiver& receiver_;
    libhcs::board::Hpm5321 board_;
};

// hpm5321_dual_can: CAN1 + CAN2, single UART0, no IMU / DBUS / GPIO.
class Hpm5321DualCanSession final : public BoardSession,
                                    public libhcs::board::Hpm5321::Callback {
public:
    explicit Hpm5321DualCanSession(BoardReceiver& receiver, std::string_view filter)
        : receiver_(receiver)
        , board_(
              *this, filter,
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true)) {}

    std::string_view name() const override { return "Hpm5321"; }
    int can_bus_count() const override { return 2; }
    int uart_port_count() const override { return 1; }
    bool has_dbus() const override { return false; }
    bool has_imu() const override { return false; }
    int gpio_channel_count() const override { return 0; }

    void transmit(FunctionRef<void(BoardTransmitter&)> build) override {
        auto builder = board_.start_transmit();
        Transmitter transmitter{builder, board_};
        build(transmitter);
    }

private:
    struct Transmitter final : BoardTransmitter {
        libhcs::board::Hpm5321::PacketBuilder& builder;
        libhcs::board::Hpm5321& board;
        Transmitter(
            libhcs::board::Hpm5321::PacketBuilder& b,
            libhcs::board::Hpm5321& board_ref)
            : builder(b)
            , board(board_ref) {}
        BoardTransmitter& can(int bus, const libhcs::data::CanDataView& data) override {
            switch (bus) {
            case 0: builder.can_transmit(CanPort::kCan1, data); break;
            case 1: builder.can_transmit(CanPort::kCan2, data); break;
            default: throw std::out_of_range{"Hpm5321: CAN bus out of range (0-1)"};
            }
            return *this;
        }
        BoardTransmitter& uart(int port, const libhcs::data::UartDataView& data) override {
            if (port != 0)
                throw std::out_of_range{"Hpm5321: only UART port 0 exists"};
            builder.uart0_transmit(data);
            return *this;
        }
        // EP0, not this batch -- see the note on Hpm5321Session::Transmitter.
        BoardTransmitter& uart_config(
            int port, const libhcs::data::UartConfigView& config) override {
            if (port != 0)
                throw std::out_of_range{"Hpm5321: only UART port 0 exists"};
            if (config.baudrate.has_value())
                board.configure_uart0(*config.baudrate);
            return *this;
        }
    };

    void can_receive(
        CanPort port, const libhcs::data::CanDataView& data) override {
        switch (port) {
        case CanPort::kCan1: {
            receiver_.on_can(0, data);
            break;
        }
        case CanPort::kCan2: {
            receiver_.on_can(1, data);
            break;
        }
        default: break;
        }
    }
    void uart0_receive_callback(const libhcs::data::UartDataView& data) override {
        receiver_.on_uart(0, data);
    }

    BoardReceiver& receiver_;
    libhcs::board::Hpm5321 board_;
};

} // namespace detail

// Open the first project board that responds. Each board type has a unique USB
// PID, so at most one adapter connects; the rest throw and are skipped. Returns
// nullptr if no board was found. Pass a serial filter to target one device when
// several are attached.
inline std::unique_ptr<BoardSession> connect_any(
    BoardReceiver& receiver, std::string_view serial_filter = {}) {
    try {
        return std::make_unique<detail::CBoardSession>(receiver, serial_filter);
    } catch (const std::runtime_error&) {
    }
    try {
        return std::make_unique<detail::Mc02Session>(receiver, serial_filter);
    } catch (const std::runtime_error&) {
    }
    try {
        return std::make_unique<detail::Ch32BoardSession>(receiver, serial_filter);
    } catch (const std::runtime_error&) {
    }
    try {
        return std::make_unique<detail::Hpm5321DualCanSession>(receiver, serial_filter);
    } catch (const std::runtime_error&) {
    }
    try {
        return std::make_unique<detail::Hpm5321Session>(receiver, serial_filter);
    } catch (const std::runtime_error&) {
    }
    return nullptr;
}

} // namespace examples
