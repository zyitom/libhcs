#pragma once

#include <concepts>
#include <cstdint>

#include "core/include/libhcs/spec/c_board/ports.hpp"
#include "core/src/link/registry.hpp"
#include "firmware/c_board/app/src/buzzer/buzzer.hpp"
#include "firmware/c_board/app/src/can/can.hpp"
#include "firmware/c_board/app/src/gpio/gpio.hpp"
#include "firmware/c_board/app/src/spi/bmi088/service.hpp"
#include "firmware/c_board/app/src/uart/uart.hpp"
#include "firmware/c_board/app/src/utility/loop_work.hpp"

namespace libhcs::firmware::ports {

// 口表(spec/c_board/ports.hpp)到驱动的绑定: 身份与类型取自 spec 的具名描述符, 能力取自驱动
// 类型(kPortCapabilities), 本文件只说"哪个驱动对象"。Registry::matches() 的 static_assert
// 保证与口表一一对应 -- 漏绑、多绑、绑错类型都是编译错误。UART 驱动自己的身份(构造参数,
// 同样引用 spec 描述符)由 App 在初始化完驱动后以 Registry::identities_match() 断言一次。
//
// 与另两块板一样, 没声明的口不工作(2026-10-07 起): CAN 停在 INIT 模式不上总线, 串口与 IMU
// 不上报、不下发; 上电与会话结束时全部挂起, 清单声明了才 resume。清单里的设置项对 CAN、
// IMU 仍只是核对(位时序与传感器挡位是编译期预设)。GPIO 口的七根线清单声明了才启动, 没声明
// 的不碰(gpio/gpio.hpp)。
using Spec = spec::c_board::Spec;
using core::link::LazyPort;

using Can1 = LazyPort<Spec::Cans::kCan1, can::can1>;
using Can2 = LazyPort<Spec::Cans::kCan2, can::can2>;
static_assert(can::kCanPorts[0].data_id == Can1::data_id);
static_assert(can::kCanPorts[1].data_id == Can2::data_id);
using UartDbus = LazyPort<Spec::Uarts::kDbus, uart::uart_dbus>;
using Uart1 = LazyPort<Spec::Uarts::kUart1, uart::uart1>;
using Uart2 = LazyPort<Spec::Uarts::kUart2, uart::uart2>;

// IMU 的驱动对象常驻(不是 Lazy)。
struct Imu : core::link::PortOf<Spec::kImu> {
    static spi::bmi088::ImuPort* instance() { return &spi::bmi088::imu_port; }
};

// GPIO 口(PWM 排针)的驱动就是 GPIO 服务对象(gpio/gpio.hpp), 七根线是它的成员。
using GpioPort = LazyPort<Spec::kGpio, gpio::gpio>;

// 蜂鸣器口的适配对象常驻, 驱动(buzzer::buzzer)在上电时就位后口才算在。与 mc02 同一写法。
struct Buzzer : core::link::PortOf<Spec::kBuzzer> {
    static decltype(buzzer::buzzer_port)* instance() {
        return buzzer::buzzer.try_get() != nullptr ? &buzzer::buzzer_port : nullptr;
    }
};

using Registry =
    core::link::PortRegistry<Can1, Can2, UartDbus, Uart1, Uart2, Imu, GpioPort, Buzzer>;
static_assert(Registry::matches(Spec::kPorts));

// 串口的主循环入口, 与 mc02 同一写法: 注册表里的串口绑定在编译期展开成直线代码, 每个口
// 一次常量位测试(loop::active 里的位, 位号 = DataId), 置位的口内联调它的 try_transmit()。
// 轮询完一路调一次 after_each()(主循环在那里泵 USB)。端口说它不必再被轮询(停了且发完)
// 就清位。顺序就是口表的顺序(DBUS, UART1, UART2)。
template <std::invocable AfterEach>
inline void poll_uarts(uint32_t active, AfterEach&& after_each) {
    if ((active & loop::kUarts) == 0U)
        return;
    Registry::for_each([&](auto binding) {
        using Binding = decltype(binding);
        if constexpr (Binding::kind == spec::PortKind::kUart) {
            constexpr uint32_t kBit = loop::bit(Binding::data_id);
            if ((active & kBit) == 0U)
                return;
            // 位只由端口自己的 resume() 置上, 置位即说明驱动对象已构造。
            if (!Binding::driver().try_transmit())
                loop::clear(kBit);
            after_each();
        }
    });
}

} // namespace libhcs::firmware::ports
