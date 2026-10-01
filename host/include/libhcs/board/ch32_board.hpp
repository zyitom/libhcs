#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hcs_can_port.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/data/datas.hpp>
#include <libhcs/protocol/handler.hpp>
#include <libhcs/spec/ch32_board/can.hpp>
#include <libhcs/spec/ch32_board/uart.hpp>

namespace libhcs::board {

/**
 * @brief High-level host board interface for ch32_board (WCH CH32H417, PID 0xD403).
 *
 * The board carries CAN1/CAN2 (classic 2.0B -- the part has no CAN-FD) and
 * USART1/USART2. It has no IMU, no DBUS and no GPIO application channels, so
 * those callbacks are intentionally absent. Its distinguishing feature is the
 * transport: the firmware talks over a USB 3.0 SuperSpeed bulk pair rather than
 * the full/high-speed pipes the other boards use, which the host side reaches
 * through the same libusb transport with no board-specific code.
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
 */
class Ch32Board final {
public:
    class Callback : public data::DataCallback {
    public:
        // Channel descriptors for this board. Addressing a channel through its
        // descriptor is what lets generic code reach data_id and, for UARTs,
        // config_data_id without a second lookup table.
        struct Spec {
            using Can = spec::ch32_board::CanDescriptor;
            static constexpr spec::ch32_board::internal::CanDescriptors kCans{};

            using Uart = spec::ch32_board::UartDescriptor;
            static constexpr spec::ch32_board::internal::UartDescriptors kUarts{};
        };

        struct View {
            using Can = data::CanDataView;
            using Uart = data::UartDataView;
            using UartConfig = data::UartConfigView;
            using ImuAccelerometer = data::ImuAccelerometerDataView;
            using ImuGyroscope = data::ImuGyroscopeDataView;
            using ImuTemperature = data::ImuTemperatureDataView;
        };

        virtual void can1_receive_callback(const libhcs::data::CanDataView& data) { (void)data; }
        virtual void can2_receive_callback(const libhcs::data::CanDataView& data) { (void)data; }

        virtual void uart1_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }
        virtual void uart2_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }

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
        // One entry per kind with the port as its descriptor, like the GPIO callbacks: what code
        // written for every board model overrides. The default fans out to the per-port
        // callbacks above, which stay for code that handles one port by name.
        virtual void can_receive_callback(const Spec::Can& can, const View::Can& data) {
            switch (can.data_id) {
            case data::DataId::kCan1: can1_receive_callback(data); break;
            case data::DataId::kCan2: can2_receive_callback(data); break;
            default: break;
            }
        }

        bool can_receive_callback(data::DataId id, const data::CanDataView& data) final {
            const auto* can = Spec::kCans.find(id);
            if (can == nullptr)
                return false;
            can_receive_callback(*can, data);
            return true;
        }

        // Same shape as can_receive_callback(const Spec::Can&, ...).
        virtual void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) {
            switch (uart.data_id) {
            case data::DataId::kUart1: uart1_receive_callback(data); break;
            case data::DataId::kUart2: uart2_receive_callback(data); break;
            default: break;
            }
        }

        bool uart_receive_callback(data::DataId id, const data::UartDataView& data) final {
            const auto* uart = Spec::kUarts.find(id);
            if (uart == nullptr)
                return false;
            uart_receive_callback(*uart, data);
            return true;
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

    // ch32_board uses the shared HCS vendor id (0xA511) and the fixed board-type
    // PID 0xD403 (c_board 0xF407, mc02 0x0723); per-device identity lives in the
    // serial number, so pass a serial_filter to target a specific board when
    // several are connected.
    explicit Ch32Board(
        Callback& callback = default_callback_, std::string_view serial_filter = {},
        const AdvancedOptions& options = {})
        : handler_(0xA511, 0xD403, serial_filter, options, callback) {}

    Ch32Board(const Ch32Board&) = delete;
    Ch32Board& operator=(const Ch32Board&) = delete;
    Ch32Board(Ch32Board&&) = delete;
    Ch32Board& operator=(Ch32Board&&) = delete;
    ~Ch32Board() = default;

    class PacketBuilder {
        friend class Ch32Board;

    public:
        // Any port by its descriptor (Callback::Spec): the form code written for every board
        // model uses.
        PacketBuilder& can_transmit(
            const Callback::Spec::Can& can, const libhcs::data::CanDataView& data) {
            return can_transmit(hcs::can_port(can.data_id), data);
        }

        PacketBuilder& uart_transmit(
            const Callback::Spec::Uart& uart, const libhcs::data::UartDataView& data) {
            if (!builder_.write_uart(uart.data_id, data)) [[unlikely]]
                throw std::invalid_argument{"UART transmission failed: Invalid UART data"};
            return *this;
        }

        // Transmits on the CAN port named as the enclosure labels it.
        // Ports on this board: CanPort::kCan1, CanPort::kCan2 (silkscreen
        // CAN1..CAN2). Same entry point as every other board's, so a caller
        // that does not know which board it holds can still address a port.
        //
        // The part has no CAN-FD, so every payload must fit a classic 8-byte
        // frame; reject_long_payload() enforces that here.
        PacketBuilder& can_transmit(hcs::CanPort port, const libhcs::data::CanDataView& data) {
            // kCanN == DataId::kCanN on this board (silkscreen CAN1..CAN2).
            const auto index = std::to_underlying(port);
            if (index < 1 || index > spec::ch32_board::kCanIds.size()) [[unlikely]]
                throw std::out_of_range{
                    "Ch32Board: CAN port out of range (this board has CAN1..CAN2)"};
            hcs::reject_long_payload(data.can_data, spec::ch32_board::kCanNames[index - 1]);
            if (!builder_.write_can(spec::ch32_board::kCanIds[index - 1], data)) [[unlikely]]
                throw std::invalid_argument{"CAN transmission failed: Invalid CAN data"};
            return *this;
        }

        PacketBuilder& uart1_transmit(const libhcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart1, data)) [[unlikely]]
                throw std::invalid_argument{"UART1 transmission failed: Invalid UART data"};
            return *this;
        }

        // Runtime reconfiguration of UART1. Rides the same downlink stream as
        // the data above, so it is ordered against it: bytes queued earlier in
        // this batch are sent at the old baudrate, later ones at the new one.
        //
        // The firmware drops whatever is in flight when the rate changes and
        // ignores a baudrate its BRR divisor cannot represent (outside roughly
        // HCLK/65536 .. HCLK/16), keeping the previous rate. Quiesce the port
        // before switching if the peer is mid-frame.
        PacketBuilder& uart1_config(const libhcs::data::UartConfigView& config) {
            if (!builder_.write_uart_config(data::DataId::kUart1Config, config)) [[unlikely]]
                throw std::invalid_argument{"UART1 configuration failed: Invalid UART config"};
            return *this;
        }

        PacketBuilder& uart2_transmit(const libhcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart2, data)) [[unlikely]]
                throw std::invalid_argument{"UART2 transmission failed: Invalid UART data"};
            return *this;
        }

        // Runtime reconfiguration of UART2; see uart1_config above.
        PacketBuilder& uart2_config(const libhcs::data::UartConfigView& config) {
            if (!builder_.write_uart_config(data::DataId::kUart2Config, config)) [[unlikely]]
                throw std::invalid_argument{"UART2 configuration failed: Invalid UART config"};
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

private:
    static inline Callback default_callback_{};
    host::protocol::Handler handler_;
};

} // namespace libhcs::board
