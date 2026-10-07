#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/data/callback.hpp>
#include <libhcs/data/datas.hpp>
#include <libhcs/protocol/handler.hpp>
#include <libhcs/protocol/usb_identity.hpp>
#include <libhcs/spec/port.hpp>
#include <libhcs/time/sample_time.hpp>

// 一块 libhcs 板的通用部分: 传输、口清单驱动的回调分发、发送门、运行期重声明。
//
// 模板参数是板型的 Spec(core/include/libhcs/spec/<board>/ports.hpp): 口叫什么、是什么
// 类型是编译期的名字, 所以 --
//   - 接线表拿别的板型的口是编译错误;
//   - 回调按 DataId 编译期展开分发到类型化的口入口, 没有逐口 switch, 也没有"第几路";
//   - 每个口自动得到一个强类型句柄(CAN 口读状态/改设置/看帧型, UART 口读回/改设置,
//     GPIO 线读回/改方向), 句柄携带口身份(GPIO 线带线号), 位置换算不存在。
// 板子实际有哪些口、每个口能做什么(长帧、速率可设……)不在 Spec 里: 构造时从板子的口
// 清单读来(hcs_config.hpp 的 apply), 发送门用的是那份快照。
//
// 各板型文件(Hpm5321/Mc02/Hpm6e8y/CBoard)只剩: `using Spec = ...` + USB PID + 真正
// 特有的东西。构造成功 = 板子正运行着声明的配置(见 hcs_config.hpp 的 apply)。
namespace libhcs::board {

namespace vc = core::protocol::vendor_control;

template <typename BoardSpec>
class Board : public hcs::Reconfigurable {
public:
    using Spec = BoardSpec;
    using Configuration = hcs::Configuration;
    // GPIO 线的描述符类型(Spec::kGpios.kPwm3)。不经 Spec::GpioLine: 没有 GPIO 口的板型
    // (hpm)不必为它写一个别名。
    using Gpio = spec::GpioLineDescriptor<Spec>;

    // 本板型有没有 GPIO 口: 没有的板型上, GPIO 的发送与句柄是编译错误而不是运行期抛错。
    static constexpr bool kHasGpio =
        std::ranges::any_of(Spec::kPorts, [](const spec::PortDescriptor& p) {
            return p.kind == spec::PortKind::kGpio;
        });

    // 本板型有没有蜂鸣器口。
    static constexpr bool kHasBuzzer =
        std::ranges::any_of(Spec::kPorts, [](const spec::PortDescriptor& p) {
            return p.kind == spec::PortKind::kBuzzer;
        });

    // 回调: 每种口一个类型化入口, 参数是 spec 的描述符(值类型, 身份即 DataId)。
    // 基类把 DataCallback 的 DataId 入口按清单编译期展开到这里; 清单之外的口进
    // unlisted_* -- 板型类自己决定接不接(mc02 的诊断流骑在 DataId::kUart0 上, 不在
    // 本板清单里; 默认拒绝会让反序列化器把这次传输当协议错误拆掉会话)。
    class Callback : public data::DataCallback {
    public:
        // 每个入口都带 SampleTime: SDK 的 IO 线程在解码时算好的样本时刻
        // (libhcs/time/sample_time.hpp)。控制环只读它, 不碰任何时间对象。
        virtual void can_receive_callback(
            typename Spec::Can port, const data::CanDataView& d,
            const libhcs::time::SampleTime& time) {
            (void)port;
            (void)d;
            (void)time;
        }

        virtual void uart_receive_callback(
            typename Spec::Uart port, const data::UartDataView& d,
            const libhcs::time::SampleTime& time) {
            (void)port;
            (void)d;
            (void)time;
        }

        // 清单之外的口。返回 false = 不认识(反序列化器按协议错误处理)。
        virtual bool unlisted_can_receive_callback(data::DataId id, const data::CanDataView& d) {
            (void)id;
            (void)d;
            return false;
        }
        virtual bool unlisted_uart_receive_callback(data::DataId id, const data::UartDataView& d) {
            (void)id;
            (void)d;
            return false;
        }

        // 声明为输入的线的样本(电平, 声明了 kGpioInputTimestamp 时带板上时间戳)。
        virtual void gpio_digital_read_result_callback(
            Gpio pin, const data::GpioDigitalDataView& d, const libhcs::time::SampleTime& time) {
            (void)pin;
            (void)d;
            (void)time;
        }
        virtual void gpio_analog_read_result_callback(
            Gpio pin, const data::GpioAnalogDataView& d, const libhcs::time::SampleTime& time) {
            (void)pin;
            (void)d;
            (void)time;
        }

        // IMU 的三个入口给空默认, 没 IMU 的板型不必看见它们。
        void accelerometer_receive_callback(
            const data::ImuAccelerometerDataView& d,
            const libhcs::time::SampleTime& time) override {
            (void)d;
            (void)time;
        }
        void gyroscope_receive_callback(
            const data::ImuGyroscopeDataView& d, const libhcs::time::SampleTime& time) override {
            (void)d;
            (void)time;
        }
        void temperature_receive_callback(
            const data::ImuTemperatureDataView& d, const libhcs::time::SampleTime& time) override {
            (void)d;
            (void)time;
        }

        // ---- DataCallback 的 DataId 入口: 按清单编译期分发 ----
        bool can_receive_callback(
            data::DataId id, const data::CanDataView& d,
            const libhcs::time::SampleTime& time) final {
            return [&]<std::size_t... I>(std::index_sequence<I...>) {
                return (... || dispatch_can<I>(id, d, time));
            }(std::make_index_sequence<Spec::kPorts.size()>{});
        }

        bool uart_receive_callback(
            data::DataId id, const data::UartDataView& d,
            const libhcs::time::SampleTime& time) final {
            return [&]<std::size_t... I>(std::index_sequence<I...>) {
                return (... || dispatch_uart<I>(id, d, time));
            }(std::make_index_sequence<Spec::kPorts.size()>{});
        }

        // 本板型没有 GPIO 口, 或线号超出 spec 的线数: 不认识。
        bool gpio_digital_read_result_callback(
            std::uint8_t line, const data::GpioDigitalDataView& d,
            const libhcs::time::SampleTime& time) final {
            if (!has_line(line))
                return false;
            gpio_digital_read_result_callback(Gpio{line}, d, time);
            return true;
        }

        bool gpio_analog_read_result_callback(
            std::uint8_t line, const data::GpioAnalogDataView& d,
            const libhcs::time::SampleTime& time) final {
            if (!has_line(line))
                return false;
            gpio_analog_read_result_callback(Gpio{line}, d, time);
            return true;
        }

    private:
        template <std::size_t I>
        bool dispatch_can(
            data::DataId id, const data::CanDataView& d, const libhcs::time::SampleTime& time) {
            // 清单是编译期的, 分支在实例化时消失; 留下的是 DataId 比较。
            if constexpr (Spec::kPorts[I].kind == spec::PortKind::kCan) {
                if (Spec::kPorts[I].data_id != id)
                    return false;
                can_receive_callback(typename Spec::Can{id}, d, time);
                return true;
            } else {
                return false;
            }
        }

        template <std::size_t I>
        bool dispatch_uart(
            data::DataId id, const data::UartDataView& d, const libhcs::time::SampleTime& time) {
            if constexpr (Spec::kPorts[I].kind == spec::PortKind::kUart) {
                if (Spec::kPorts[I].data_id != id)
                    return false;
                uart_receive_callback(typename Spec::Uart{id}, d, time);
                return true;
            } else {
                return false;
            }
        }

        static constexpr bool has_line(std::uint8_t line) noexcept {
            if constexpr (kHasGpio)
                return line < Spec::kGpioLineCount;
            else
                return false;
        }
    };

    // 一次发送: 一个字段一个门。接口快照构造时取一次, 每个字段一次位测试。
    class PacketBuilder {
        friend class Board;

    public:
        // 发一帧 CAN。没声明的口抛 UndeclaredChannel; 8 字节以上的负载要求板子报过这个口
        // 承载长帧, 且它此刻在 FD 总线上(两者都在构造时取的快照里)。
        PacketBuilder& can_transmit(typename Spec::Can port, const data::CanDataView& d) {
            hcs::require_declared(interface_, port.data_id);
            if (d.can_data.size() > 8
                && !(interface_.long_frames(port.data_id) && interface_.can_fd(port.data_id)))
                [[unlikely]]
                throw std::invalid_argument{std::format(
                    "{} transmission failed: payloads over 8 bytes need a CAN-FD bus on a "
                    "port that carries long frames",
                    hcs::channel_name(port.data_id))};
            if (!builder_.write_can(port.data_id, d)) [[unlikely]]
                throw std::invalid_argument{std::format(
                    "{} transmission failed: invalid CAN data", hcs::channel_name(port.data_id))};
            return *this;
        }

        // 发一段串口字节。没声明的口抛 UndeclaredChannel。
        PacketBuilder& uart_transmit(typename Spec::Uart port, const data::UartDataView& d) {
            hcs::require_declared(interface_, port.data_id);
            if (!builder_.write_uart(port.data_id, d)) [[unlikely]]
                throw std::invalid_argument{std::format(
                    "{} transmission failed: invalid UART data", hcs::channel_name(port.data_id))};
            return *this;
        }

        // 写一根输出线的电平。没声明为输出的线抛 UndeclaredChannel; 线能不能写电平
        // 已在声明时由板子按能力核过(输出至少能写一样)。
        PacketBuilder& gpio_digital_write(Gpio pin, const data::GpioDigitalDataView& d)
            requires kHasGpio {
            hcs::require_gpio_direction(interface_, pin.line, true);
            if (!builder_.write_gpio_digital_data(pin.line, d)) [[unlikely]]
                throw std::invalid_argument{std::format(
                    "{} transmission failed: a write carries no timestamp",
                    spec::gpio_line_name(pin.line))};
            return *this;
        }

        // 写一根输出线的 PWM 占空比(0..65535 = 0..100%)。
        PacketBuilder& gpio_analog_write(Gpio pin, const data::GpioAnalogDataView& d)
            requires kHasGpio {
            hcs::require_gpio_direction(interface_, pin.line, true);
            if (!builder_.write_gpio_analog_data(pin.line, d)) [[unlikely]]
                throw std::invalid_argument{std::format(
                    "{} transmission failed: invalid GPIO data", spec::gpio_line_name(pin.line))};
            return *this;
        }

        // 让一根输入线立刻采样一次; 样本经 gpio_digital_read_result_callback 回来。
        PacketBuilder& gpio_read(Gpio pin) requires kHasGpio {
            hcs::require_gpio_direction(interface_, pin.line, false);
            if (!builder_.write_gpio_read(pin.line)) [[unlikely]]
                throw std::invalid_argument{std::format(
                    "{} transmission failed: invalid GPIO request",
                    spec::gpio_line_name(pin.line))};
            return *this;
        }

        // 让蜂鸣器从现在起放这个音(0 Hz 或音量 0 = 静音)。节拍由主机掌握: 每个音符一条。
        // 没声明蜂鸣器抛 UndeclaredChannel。
        PacketBuilder& buzzer_tone(const data::BuzzerToneDataView& d) requires kHasBuzzer {
            hcs::require_declared(interface_, data::DataId::kBuzzer);
            if (!builder_.write_buzzer_tone(d)) [[unlikely]]
                throw std::invalid_argument{"buzzer transmission failed"};
            return *this;
        }

    protected:
        PacketBuilder(host::protocol::Handler& handler, hcs::Interface interface) noexcept
            : builder_(handler.start_transmit())
            , interface_(interface) {}

    private:
        host::protocol::Handler::PacketBuilder builder_;
        hcs::Interface interface_;
    };

    // ---- 强类型口句柄: 一个口 = 板 + 身份 ----
    //
    // 由 spec 的描述符给出(板::handle(Spec::kCans.kCan1)), 帧型/状态/设置都落在这一
    // 个口上。CAN 的口知道自己的帧型、运行时状态与时序; UART 的口知道自己的速率、
    // 帧格式与运行时状态, 也能在运行期改设置。设置走 EP0(冷路径), 状态走数据流。
    class CanHandle {
        friend class Board;

    public:
        using Port = typename Spec::Can;
        [[nodiscard]] data::DataId data_id() const noexcept { return port_.data_id; }
        // 此刻是否以 CAN-FD 发送(构造握手 + 运行期重声明维护的快照)。
        [[nodiscard]] bool is_fd() const noexcept {
            return board_->interface_.load(std::memory_order_relaxed).can_fd(port_.data_id);
        }
        // 控制器的运行时状态: 板子随 keepalive 推送的最新快照(本会话, 至多晚一个轮次),
        // 无锁读, 不发任何 USB 请求 -- 控制环里调用也无妨。读法见 data::CanStatusView。
        [[nodiscard]] data::CanStatusView status() const noexcept {
            return board_->handler_.template status<data::CanStatusView>(port_.data_id);
        }
        // 寄存器重构的时序事实(速率 + 采样点)。
        [[nodiscard]] vc::CanConfigPayload config() const {
            return hcs::read_can_config(board_->handler_, port_.data_id);
        }
        // 运行期改设置(整份声明重放, 经重配锁)。
        void configure(const hcs::CanSetting& setting) { board_->configure_can(port_, setting); }

    private:
        CanHandle(Board& board, Port port) noexcept
            : board_(&board)
            , port_(port) {}

        Board* board_;
        Port port_;
    };

    class UartHandle {
        friend class Board;

    public:
        using Port = typename Spec::Uart;
        [[nodiscard]] data::DataId data_id() const noexcept { return port_.data_id; }
        // 硬件事实: 从实际编程的分频器重建的速率 + 活寄存器解出的帧格式。
        [[nodiscard]] hcs::UartSetting setting() const {
            return hcs::read_uart_setting(board_->handler_, port_.data_id);
        }
        [[nodiscard]] uint32_t baudrate() const {
            return hcs::read_uart_baudrate(board_->handler_, port_.data_id);
        }
        // 运行时的接收错误与板端丢弃: 板子随 keepalive 推送的最新快照, 同 CanHandle::status()。
        [[nodiscard]] data::UartStatusView status() const noexcept {
            return board_->handler_.template status<data::UartStatusView>(port_.data_id);
        }
        void configure(const hcs::UartSetting& setting) { board_->configure_uart(port_, setting); }

    private:
        UartHandle(Board& board, Port port) noexcept
            : board_(&board)
            , port_(port) {}

        Board* board_;
        Port port_;
    };

    class GpioHandle {
        friend class Board;

    public:
        using Port = Gpio;
        [[nodiscard]] std::uint8_t line() const noexcept { return port_.line; }
        // 板上此刻的设置(方向、上下拉、采样); 没声明的线是 vc::kGpioModeOff。
        [[nodiscard]] hcs::GpioSetting setting() const {
            return hcs::read_gpio_setting(board_->handler_, port_.line);
        }
        // 运行期改方向或采样(整份声明重放, 经重配锁; 其余线的输出值不受影响)。
        void configure(const hcs::GpioSetting& setting) { board_->configure_gpio(port_, setting); }

    private:
        GpioHandle(Board& board, Port port) noexcept
            : board_(&board)
            , port_(port) {}

        Board* board_;
        Port port_;
    };

    // 链路状态: kUp / kReestablishing / kFaulted(设备消失, 只能重建对象)。
    [[nodiscard]] host::protocol::Handler::LinkState link_state() const noexcept {
        return handler_.link_state();
    }

    // 按 DataId 读一个口的运行时状态(同 CanHandle/UartHandle::status(), 给按身份遍历口的
    // 调用方用): 板子随 keepalive 推送的快照, 无锁, 不发 USB 请求。variant 里是这个口的种类;
    // DataId::kSession 是链路本身(data::LinkStatusView, 下行流错误)。
    [[nodiscard]] data::PortStatusVariant port_status(data::DataId port) const noexcept {
        return handler_.port_status(port);
    }

    // 发送一批: PacketBuilder 的快照在构造时取, 各字段的门只花位测试。
    [[nodiscard]] PacketBuilder start_transmit() noexcept {
        return PacketBuilder{handler_, interface_.load(std::memory_order_relaxed)};
    }

    // 构造握手学到的快照(原子: 重连钩子在 keepalive 线程写, 发送路径读)。
    [[nodiscard]] hcs::Interface interface() const {
        return interface_.load(std::memory_order_relaxed);
    }

    // 强类型句柄。
    [[nodiscard]] CanHandle handle(typename Spec::Can port) noexcept {
        return CanHandle{*this, port};
    }
    [[nodiscard]] UartHandle handle(typename Spec::Uart port) noexcept {
        return UartHandle{*this, port};
    }
    [[nodiscard]] GpioHandle handle(Gpio pin) noexcept requires kHasGpio {
        return GpioHandle{*this, pin};
    }

    // 蜂鸣器此刻在放的音(从定时器寄存器重建; 0 Hz = 静音)。
    [[nodiscard]] vc::BuzzerConfigPayload buzzer_state() requires kHasBuzzer {
        return hcs::read_buzzer_state(handler_);
    }

    // 跨板对时测量专用; 见 Handler::send_pulse_schedule。不属于控制路径。
    void send_pulse_schedule(uint64_t microframe) noexcept {
        handler_.send_pulse_schedule(microframe);
    }

protected:
    // vendor_id / product_ids: 本板型固件的 USB VID 与 PID(一个镜像多个 PID 的, 按扫描顺序
    // 列出)。VID 不是全板统一的: HPM 板借达妙的 0x34B7(DMTool 兼容), mc02 与 c_board 是 0xA511。
    // interface_ 与 configuration_ 刻意先于 handler_ 构造: before-session 钩子在
    // handler_ 构造中就会运行并触碰两者。configuration_ 是一份拷贝: 钩子在每次
    // 重连时都会重跑, 远在构造参数消失之后。
    Board(
        uint16_t vendor_id, std::span<const uint16_t> product_ids,
        Callback& callback = default_callback_, std::string_view serial_filter = {},
        const AdvancedOptions& options = {}, const Configuration& configuration = {})
        : interface_{}
        , configuration_(configuration)
        , handler_(
              vendor_id, product_ids, serial_filter, options, callback,
              [this](host::protocol::Handler& handler) {
                  const std::scoped_lock guard{reconfigure_mutex_};
                  interface_.store(hcs::apply(handler, configuration_), std::memory_order_release);
                  // Only after the board accepted it: anchors go to a board running the time base.
                  handler.set_time_sync(configuration_.time_sync);
              }) {}

    Board(const Board&) = delete;
    Board& operator=(const Board&) = delete;
    Board(Board&&) = delete;
    Board& operator=(Board&&) = delete;
    ~Board() = default;

    // 运行期重声明(句柄的 configure() 也在用): 整个 EP0 事务持重配锁, 与重连钩子
    // 的 apply() 互斥。
    void configure_can(typename Spec::Can port, const hcs::CanSetting& setting) {
        const std::scoped_lock guard{reconfigure_mutex_};
        hcs::reconfigure_can(handler_, configuration_, interface_, port.data_id, setting);
    }

    void configure_uart(typename Spec::Uart port, const hcs::UartSetting& setting) {
        const std::scoped_lock guard{reconfigure_mutex_};
        hcs::reconfigure_uart(handler_, configuration_, interface_, port.data_id, setting);
    }

    void configure_gpio(Gpio pin, const hcs::GpioSetting& setting) {
        const std::scoped_lock guard{reconfigure_mutex_};
        hcs::reconfigure_gpio(handler_, configuration_, interface_, pin.line, setting);
    }

    // 各板型类的公共默认回调(全空实现)。
    static inline Callback default_callback_{};

    std::atomic<hcs::Interface> interface_;
    Configuration configuration_;
    host::protocol::Handler handler_;
};

} // namespace libhcs::board
