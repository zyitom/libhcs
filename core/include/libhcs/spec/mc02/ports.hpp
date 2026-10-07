#pragma once

#include <array>

#include <libhcs/data/datas.hpp>
#include <libhcs/spec/port.hpp>

// mc02 的口: 叫什么、是什么类型。主机接线表(Spec::kCans.kCan1)与固件绑定
// (firmware/mc02/app/src/ports.hpp)共用这一份名字。每个口能做什么由固件报告
// (kGetPortList), 不在这里。
// 三路 CAN、六路 UART、板载 IMU、GPIO 口(四根线)与蜂鸣器: 全仓库最大的口表, kMaxPorts 与
// kMaxManifestEntries 的上限由此而来。
namespace libhcs::spec::mc02 {

struct Spec {
    using Can = TypedPortDescriptor<Spec, PortKind::kCan>;
    using Uart = TypedPortDescriptor<Spec, PortKind::kUart>;
    using Imu = TypedPortDescriptor<Spec, PortKind::kImu>;
    using Gpio = TypedPortDescriptor<Spec, PortKind::kGpio>;
    using GpioLine = GpioLineDescriptor<Spec>;
    using Buzzer = TypedPortDescriptor<Spec, PortKind::kBuzzer>;

    // 丝印 CAN1..CAN3。
    struct Cans {
        static constexpr Can kCan1{data::DataId::kCan1};
        static constexpr Can kCan2{data::DataId::kCan2};
        static constexpr Can kCan3{data::DataId::kCan3};
    };
    // 机壳丝印: DBUS(带板上反相器)、UART1 / UART2 / UART3 / UART7 / UART10。UART2/UART3
    // 是 RS-485 收发器(USART2/USART3)。
    struct Uarts {
        static constexpr Uart kDbus{data::DataId::kUartDbus};
        static constexpr Uart kUart1{data::DataId::kUart1};
        static constexpr Uart kUart2{data::DataId::kUart2};
        static constexpr Uart kUart3{data::DataId::kUart3};
        static constexpr Uart kUart7{data::DataId::kUart7};
        static constexpr Uart kUart10{data::DataId::kUart10};
    };
    // GPIO 口 = PWM 排针, 四根线都在定时器通道上, 也都有自己的 EXTI 线: 每根都能当数字/PWM
    // 输出或数字输入(电平、周期、边沿, 带上下拉与时间戳)。线号 n 印作 PWM(n+1)。
    static constexpr Gpio kGpio{data::DataId::kGpio};
    static constexpr std::uint8_t kGpioLineCount = 4;
    struct Gpios {
        static constexpr GpioLine kPwm1{0}; // TIM2 CH1, PA0, EXTI0
        static constexpr GpioLine kPwm2{1}; // TIM2 CH3, PA2, EXTI2
        static constexpr GpioLine kPwm3{2}; // TIM1 CH1, PE9, EXTI9
        static constexpr GpioLine kPwm4{3}; // TIM1 CH3, PE13, EXTI13
    };
    static constexpr Cans kCans{};
    static constexpr Uarts kUarts{};
    static constexpr Gpios kGpios{};
    // 板载 IMU(BMI088): 身份 DataId::kImu, 不在丝印上。
    static constexpr Imu kImu{data::DataId::kImu};
    // 板载无源蜂鸣器(PB15, TIM12 CH2): 身份 DataId::kBuzzer, 不在丝印上。
    static constexpr Buzzer kBuzzer{data::DataId::kBuzzer};

    static constexpr std::array kPorts{
        port(Cans::kCan1),    port(Cans::kCan2),   port(Cans::kCan3),   port(Uarts::kDbus),
        port(Uarts::kUart1),  port(Uarts::kUart2), port(Uarts::kUart3), port(Uarts::kUart7),
        port(Uarts::kUart10), port(kImu),          port(kGpio),         port(kBuzzer),
    };
};
static_assert(no_duplicate_data_ids(Spec::kPorts));
static_assert(manifest_fits(Spec::kPorts, Spec::kGpioLineCount));

} // namespace libhcs::spec::mc02
