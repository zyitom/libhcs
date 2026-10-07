#pragma once

#include <array>

#include <libhcs/data/datas.hpp>
#include <libhcs/spec/port.hpp>

// c_board 的口: 叫什么、是什么类型。主机接线表(Spec::kCans.kCan1)与固件绑定
// (firmware/c_board/app/src/ports.hpp)共用这一份名字。每个口能做什么由固件报告
// (kGetPortList), 不在这里。
namespace libhcs::spec::c_board {

struct Spec {
    using Can = TypedPortDescriptor<Spec, PortKind::kCan>;
    using Uart = TypedPortDescriptor<Spec, PortKind::kUart>;
    using Imu = TypedPortDescriptor<Spec, PortKind::kImu>;
    using Gpio = TypedPortDescriptor<Spec, PortKind::kGpio>;
    using GpioLine = GpioLineDescriptor<Spec>;
    using Buzzer = TypedPortDescriptor<Spec, PortKind::kBuzzer>;

    // 丝印 CAN1 / CAN2(bxCAN)。
    struct Cans {
        static constexpr Can kCan1{data::DataId::kCan1};
        static constexpr Can kCan2{data::DataId::kCan2};
    };
    // DBUS(huart3)加丝印 UART1(huart6)/ UART2(huart1)。本板没有反相器: DBUS 在线上是
    // 什么极性就收什么极性。
    struct Uarts {
        static constexpr Uart kDbus{data::DataId::kUartDbus};
        static constexpr Uart kUart1{data::DataId::kUart1};
        static constexpr Uart kUart2{data::DataId::kUart2};
    };
    // GPIO 口 = PWM 排针的七根线(TIM1 CH1-4、TIM8 CH1-3)。PWM6 的 EXTI 线另有用途, 不能
    // 边沿触发; 逐线能力在固件里, 做不到的声明由板子拒绝。线号 n 印作 PWM(n+1)。
    static constexpr Gpio kGpio{data::DataId::kGpio};
    static constexpr std::uint8_t kGpioLineCount = 7;
    struct Gpios {
        static constexpr GpioLine kPwm1{0};
        static constexpr GpioLine kPwm2{1};
        static constexpr GpioLine kPwm3{2};
        static constexpr GpioLine kPwm4{3};
        static constexpr GpioLine kPwm5{4};
        static constexpr GpioLine kPwm6{5};
        static constexpr GpioLine kPwm7{6};
    };
    static constexpr Cans kCans{};
    static constexpr Uarts kUarts{};
    static constexpr Gpios kGpios{};
    // 板载 IMU(BMI088): 身份 DataId::kImu, 不在丝印上。
    static constexpr Imu kImu{data::DataId::kImu};
    // 板载无源蜂鸣器(PD14, TIM4 CH3, 额定 4 kHz): 身份 DataId::kBuzzer, 不在丝印上。
    static constexpr Buzzer kBuzzer{data::DataId::kBuzzer};

    static constexpr std::array kPorts{
        port(Cans::kCan1),   port(Cans::kCan2), port(Uarts::kDbus), port(Uarts::kUart1),
        port(Uarts::kUart2), port(kImu),        port(kGpio),        port(kBuzzer),
    };
};
static_assert(no_duplicate_data_ids(Spec::kPorts));
static_assert(manifest_fits(Spec::kPorts, Spec::kGpioLineCount));

} // namespace libhcs::spec::c_board
