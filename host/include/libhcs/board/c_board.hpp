#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hcs_can_port.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/data/datas.hpp>
#include <libhcs/protocol/handler.hpp>
#include <libhcs/spec/c_board/can.hpp>
#include <libhcs/spec/c_board/gpio.hpp>
#include <libhcs/spec/c_board/uart.hpp>
#include <libhcs/spec/gpio.hpp>

namespace libhcs::board {

/**
 * @brief High-level host board interface for C Board.
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
class CBoard final : public hcs::Reconfigurable {
public:
    class Callback : public data::DataCallback {
    public:
        // Channel descriptors for this board. Addressing a channel through its
        // descriptor is what lets generic code reach data_id and, for UARTs,
        // config_data_id without a second lookup table.
        struct Spec {
            using Can = spec::c_board::CanDescriptor;
            static constexpr spec::c_board::internal::CanDescriptors kCans{};

            using Uart = spec::c_board::UartDescriptor;
            static constexpr spec::c_board::internal::UartDescriptors kUarts{};

            using Gpio = spec::c_board::GpioDescriptor;
            static constexpr spec::c_board::internal::GpioDescriptors kGpios{};
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

        virtual void dbus_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }
        virtual void uart1_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }
        virtual void uart2_receive_callback(const libhcs::data::UartDataView& data) { (void)data; }

        virtual void gpio_digital_read_result_callback(
            const libhcs::spec::c_board::GpioDescriptor& gpio,
            const libhcs::data::GpioDigitalDataView& data) {
            (void)gpio;
            (void)data;
        }
        virtual void gpio_analog_read_result_callback(
            const libhcs::spec::c_board::GpioDescriptor& gpio,
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
            default: return false;
            }
        }

        bool uart_receive_callback(data::DataId id, const data::UartDataView& data) final {
            switch (id) {
            case data::DataId::kUartDbus: dbus_receive_callback(data); return true;
            case data::DataId::kUart1: uart1_receive_callback(data); return true;
            case data::DataId::kUart2: uart2_receive_callback(data); return true;
            default: return false;
            }
        }

        bool gpio_digital_read_result_callback(
            uint8_t channel_index, const data::GpioDigitalDataView& data) final {
            if (channel_index >= spec::c_board::kGpioDescriptors.size()) [[unlikely]]
                return false;
            gpio_digital_read_result_callback(spec::c_board::kGpioDescriptors[channel_index], data);
            return true;
        }

        bool gpio_analog_read_result_callback(
            uint8_t channel_index, const data::GpioAnalogDataView& data) final {
            if (channel_index >= spec::c_board::kGpioDescriptors.size()) [[unlikely]]
                return false;
            gpio_analog_read_result_callback(spec::c_board::kGpioDescriptors[channel_index], data);
            return true;
        }
    };

    // 构造期经 EP0 应用并回读, 被拒即抛; 见 libhcs/board/hcs_config.hpp。
    using Configuration = hcs::Configuration;

    explicit CBoard(
        Callback& callback = default_callback_, std::string_view serial_filter = {},
        const AdvancedOptions& options = {}, const Configuration& configuration = {})
        : configuration_(configuration)
        , handler_(
              0xA511, 0xF407, serial_filter, options, callback,
              [this](host::protocol::Handler& handler) {
                  // 读 kGetInterface 即 EP0 握手, 固件在此之前不应答会话。
                  const std::scoped_lock guard{reconfigure_mutex_};
                  interface_.store(hcs::apply(handler, configuration_), std::memory_order_release);
              }) {}

    CBoard(const CBoard&) = delete;
    CBoard& operator=(const CBoard&) = delete;
    CBoard(CBoard&&) = delete;
    CBoard& operator=(CBoard&&) = delete;
    ~CBoard() = default;

    class PacketBuilder {
        friend class CBoard;

    public:
        // Transmits on the CAN port named as the enclosure labels it.
        // Ports on this board: CanPort::kCan1, CanPort::kCan2 (silkscreen
        // CAN1..CAN2). Same entry point as every other board's, so a caller
        // that does not know which board it holds can still address a port.
        PacketBuilder& can_transmit(hcs::CanPort port, const libhcs::data::CanDataView& data) {
            // kCanN == DataId::kCanN on this board (silkscreen CAN1..CAN2).
            const auto index = std::to_underlying(port);
            if (index < 1 || index > spec::c_board::kCanIds.size()) [[unlikely]]
                throw std::out_of_range{
                    "CBoard: CAN port out of range (this board has CAN1..CAN2)"};
            hcs::reject_long_payload(data.can_data, spec::c_board::kCanNames[index - 1]);
            if (!builder_.write_can(spec::c_board::kCanIds[index - 1], data)) [[unlikely]]
                throw std::invalid_argument{"CAN transmission failed: Invalid CAN data"};
            return *this;
        }

        PacketBuilder& uart1_transmit(const libhcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart1, data)) [[unlikely]]
                throw std::invalid_argument{"UART1 transmission failed: Invalid UART data"};
            return *this;
        }

        // 带内旧路径, 只写不回报; 新代码用 CBoard::configure_uart1()。阶段 6 实测后删。
        //
        // Runtime reconfiguration of UART1. Rides the same downlink stream as
        // the data above, so it is ordered against it: bytes queued earlier in
        // this batch are sent at the old baudrate, later ones at the new one.
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

        // 带内旧路径, 只写不回报; 新代码用 CBoard::configure_uart2()。阶段 6 实测后删。
        //
        // Runtime reconfiguration of UART2. Rides the same downlink stream as
        // the data above, so it is ordered against it: bytes queued earlier in
        // this batch are sent at the old baudrate, later ones at the new one.
        PacketBuilder& uart2_config(const libhcs::data::UartConfigView& config) {
            if (!builder_.write_uart_config(data::DataId::kUart2Config, config)) [[unlikely]]
                throw std::invalid_argument{"UART2 configuration failed: Invalid UART config"};
            return *this;
        }

        PacketBuilder& gpio_digital_write(
            const libhcs::spec::c_board::GpioDescriptor& gpio,
            const libhcs::data::GpioDigitalDataView& data) {
            if (!gpio.supports(spec::GpioCapability::kDigitalWrite)
                || !builder_.write_gpio_digital_data(gpio.channel_index, data)) [[unlikely]]
                throw std::invalid_argument{"GPIO digital transmission failed: Invalid GPIO data"};
            return *this;
        }
        PacketBuilder& gpio_digital_read(
            const libhcs::spec::c_board::GpioDescriptor& gpio,
            const libhcs::data::GpioReadConfigView& data) {
            if (!data.supported(gpio)
                || !builder_.write_gpio_digital_read_config(gpio.channel_index, data)) [[unlikely]]
                throw std::invalid_argument{
                    "GPIO digital read configuration transmission failed: Invalid GPIO data"};
            return *this;
        }
        PacketBuilder& gpio_analog_write(
            const libhcs::spec::c_board::GpioDescriptor& gpio,
            const libhcs::data::GpioAnalogDataView& data) {
            if (!gpio.supports(spec::GpioCapability::kAnalogWrite)
                || !builder_.write_gpio_analog_data(gpio.channel_index, data)) [[unlikely]]
                throw std::invalid_argument{"GPIO analog transmission failed: Invalid GPIO data"};
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

    // EP0 运行期重配: 同步、板端确认后才返回, 被拒即抛; 与已排队数据不定序, 切换前先静默链路。
    // 下标即板端 EP0 顺序: 0=DBUS, 1=UART1, 2=UART2。
    void configure_dbus(uint32_t baudrate) { configure_uart(0, {.baudrate = baudrate}); }
    void configure_uart1(uint32_t baudrate) { configure_uart(1, {.baudrate = baudrate}); }
    void configure_uart2(uint32_t baudrate) { configure_uart(2, {.baudrate = baudrate}); }

    // 完整形式: 速率 + 帧格式(字长 7/8、校验、停止位 1/2), 0 字段不动。见 hcs::UartSetting。
    void configure_dbus(const hcs::UartSetting& setting) { configure_uart(0, setting); }
    void configure_uart1(const hcs::UartSetting& setting) { configure_uart(1, setting); }
    void configure_uart2(const hcs::UartSetting& setting) { configure_uart(2, setting); }

    // 硬件实际速率, 由板端 BRR 反推, 不是上次请求值。
    uint32_t dbus_baudrate() { return hcs::read_uart_baudrate(handler_, 0); }
    uint32_t uart1_baudrate() { return hcs::read_uart_baudrate(handler_, 1); }
    uint32_t uart2_baudrate() { return hcs::read_uart_baudrate(handler_, 2); }

    // 单口完整回读(速率 + 分频器 + 帧格式), 下标同上。
    hcs::UartSetting read_uart_setting(std::size_t port) {
        return hcs::read_uart_setting(handler_, port);
    }

    // 板子最近一次拒绝配置的原因, 粘滞到下次拒绝或复位。
    [[nodiscard]] hcs::vc::LastConfigErrorPayload last_config_error() {
        return hcs::read_last_config_error(handler_);
    }

    // 板子经 EP0 报告的通道数与能力位(快照; 重连时由 keepalive 线程刷新)。
    [[nodiscard]] hcs::Interface interface() const {
        return interface_.load(std::memory_order_relaxed);
    }

private:
    // configure_*() 的共用体: 整个 EP0 往返持重配锁, 并写回 configuration_ 供重连重放。
    void configure_uart(std::size_t port, const hcs::UartSetting& setting) {
        const std::scoped_lock guard{reconfigure_mutex_};
        hcs::reconfigure_uart(handler_, configuration_, port, setting);
    }

    static inline Callback default_callback_{};
    // 两者须声明在 handler_ 之前: 钩子在 handler_ 构造期间就会访问它们。
    std::atomic<hcs::Interface> interface_;
    Configuration configuration_;
    host::protocol::Handler handler_;
};

} // namespace libhcs::board
