#pragma once

#include <concepts>
#include <cstdint>

#include "core/include/libhcs/spec/mc02/ports.hpp"
#include "core/src/link/registry.hpp"
#include "firmware/mc02/app/src/buzzer/buzzer.hpp"
#include "firmware/mc02/app/src/can/can.hpp"
#include "firmware/mc02/app/src/gpio/gpio.hpp"
#include "firmware/mc02/app/src/spi/bmi088/service.hpp"
#include "firmware/mc02/app/src/uart/uart.hpp"
#include "firmware/mc02/app/src/utility/loop_work.hpp"

namespace libhcs::firmware::ports {

// 口表(spec/mc02/ports.hpp)到驱动的绑定: 身份与类型取自 spec 的具名描述符, 能力取自驱动
// 类型(kPortCapabilities), 本文件只说"哪个驱动对象"。Registry::matches() 的 static_assert
// 保证与口表一一对应 -- 漏绑、多绑、绑错类型都是编译错误。
//
// 驱动自己的身份(数据流按它打标)同样引用 spec 的描述符: CAN 的在 can.hpp 的 kCanPorts
// 表里(下面逐行钉死与绑定对齐), UART 的在构造参数里, 由 App 在初始化完驱动后以
// Registry::identities_match() 断言一次。本板只有一个 PCB 变体, 每个驱动上电都初始化;
// 上电全停是各驱动的构造状态, 不归绑定管。
using Spec = spec::mc02::Spec;
using core::link::LazyPort;

using Can1 = LazyPort<Spec::Cans::kCan1, can::can1>;
using Can2 = LazyPort<Spec::Cans::kCan2, can::can2>;
using Can3 = LazyPort<Spec::Cans::kCan3, can::can3>;
static_assert(can::kCanPorts[0].data_id == Can1::data_id);
static_assert(can::kCanPorts[1].data_id == Can2::data_id);
static_assert(can::kCanPorts[2].data_id == Can3::data_id);

using UartDbus = LazyPort<Spec::Uarts::kDbus, uart::uart_dbus>;
using Uart1 = LazyPort<Spec::Uarts::kUart1, uart::uart1>;
using Uart2 = LazyPort<Spec::Uarts::kUart2, uart::uart2>;
using Uart3 = LazyPort<Spec::Uarts::kUart3, uart::uart3>;
using Uart7 = LazyPort<Spec::Uarts::kUart7, uart::uart7>;
using Uart10 = LazyPort<Spec::Uarts::kUart10, uart::uart10>;

// IMU 的驱动对象常驻(不是 Lazy), 芯片由声明启动。
struct Imu : core::link::PortOf<Spec::kImu> {
    static spi::bmi088::ImuPort* instance() { return &spi::bmi088::imu_port; }
};

// GPIO 口(PWM 排针)的驱动就是 GPIO 服务对象(gpio/gpio.hpp), 四根线是它的成员。
using GpioPort = LazyPort<Spec::kGpio, gpio::gpio>;

// 蜂鸣器口的适配对象常驻, 驱动(buzzer::buzzer)在上电时就位后口才算在。
struct Buzzer : core::link::PortOf<Spec::kBuzzer> {
    static decltype(buzzer::buzzer_port)* instance() {
        return buzzer::buzzer.try_get() != nullptr ? &buzzer::buzzer_port : nullptr;
    }
};

using Registry = core::link::PortRegistry<
    Can1, Can2, Can3, UartDbus, Uart1, Uart2, Uart3, Uart7, Uart10, Imu, GpioPort, Buzzer>;
static_assert(Registry::matches(Spec::kPorts));

// 串口的主循环入口。注册表里的串口绑定在编译期展开成直线代码: 每个口一次常量位测试
// (loop::active 里的位, 位号 = DataId), 置位的口内联调它的 try_transmit() -- 没有按下标
// 查找, 也没有一张要与口表同序的分发表。轮询完一路调一次 after_each()(主循环在那里泵
// USB: 这一路刚出队的上行字节不必等到下一圈)。端口说它不必再被轮询(停了且发完)就清位。
// 顺序就是口表的顺序(DBUS, UART1, 2, 3, 7, 10)。
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
            // 位只由端口自己的 start() 置上, 置位即说明驱动对象已构造。
            if (!Binding::driver().try_transmit())
                loop::clear(kBit);
            after_each();
        }
    });
}

} // namespace libhcs::firmware::ports
