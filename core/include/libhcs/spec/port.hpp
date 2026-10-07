#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include <libhcs/data/datas.hpp>

// 口的身份与类型的公共定义。每块板型一个 spec/<board>/ports.hpp：口叫什么（DataId）、
// 是什么类型 -- 主机接线表的类型检查与固件绑定共用这一份名字。EP0 上所有按口寻址的
// 请求以 DataId 寻址（wIndex 低字节 = DataId 数值），负载按类型解释；"第几路"这样的位置
// 编号不出现在线上。
//
// 口是能单独声明、单独启停、有自己驱动的一块硬件：一条 CAN 总线、一路 UART、板载 IMU、
// 一组 GPIO。口下面的子地址不是口 -- CAN 总线上的电机 id 是这样，GPIO 口上的线号也是：
// 一个引脚的地址是（kGpio，线号），线号走 wIndex 高字节、清单项的 line 字段与记录头的
// 3 位线号。线叫什么（丝印 PWM3）只在板型的 spec 里。
//
// 这里刻意**没有**"这块板此刻有哪些口、每个口能做什么"：那是板子自己的事实。固件在
// 上电时初始化驱动，kGetPortList 报告初始化了的口与每口的能力位（能力出自驱动类型，
// 见 core/src/link/registry.hpp），主机只认板子报的那一份 -- 同一事实不在两处各写一遍。
//
// DataId 即丝印编号（datas.hpp）：CAN/UART 字段 id 与连接器上印的号一致，人看到的名字
// 一律由 channel_name() 生成，任何一端都不做丝印号换算。

namespace libhcs::spec {

// 定长载荷的上限，每块板型的 ports.hpp 用 static_assert 声明自己不超限：超限在编译期死掉，
// 而不是在线上被截断。
//
// 口清单（kGetPortList）一口一项。mc02 最大：3 CAN + 6 UART + IMU + GPIO + 蜂鸣器 = 12。
inline constexpr std::size_t kMaxPorts = 12;
// 清单（kApplyManifest）与清单结果一项一个声明：GPIO 口一线一项，其余一口一项。mc02 最大：
// 11 个非 GPIO 口 + 4 根线 = 15；c_board 6 + 7 = 13。
inline constexpr std::size_t kMaxManifestEntries = 15;
// 一个 GPIO 口最多几根线：记录头里的线号只有 3 位（core/src/protocol/protocol.hpp）。
inline constexpr std::size_t kMaxGpioLines = 8;

// 口的当前状态位（kGetPortList 每项的 status 字段；CAN 的位对其他类型无意义）。
enum PortStatusFlags : std::uint8_t {
    // 口在它该在的状态里：CAN 在总线上、UART 收发开、IMU 在采样、GPIO 按声明的方向工作
    kPortRunning = 1U << 0,
    kPortFd = 1U << 1, // CAN 正以 CAN-FD 发送（经典 = 0）
};

// 口的类型。决定 EP0 负载的解释方式（vendor_control.hpp）与驱动绑定要求的接口
// （core/src/link/ 的各 kind ops）。
enum class PortKind : std::uint8_t {
    kCan = 0,
    kUart = 1,
    kImu = 2,
    // 一组 GPIO 线(一个 PWM 排针)。每根线单独声明为输出(数字电平 / PWM 占空比)或输入
    // (电平、周期采样、边沿), 设置是 vendor_control.hpp 的 GpioConfigPayload。
    kGpio = 3,
    // 板载蜂鸣器: 声明不带参数, 声明了才启动定时器通道(静音); 放什么音由记录流说。
    kBuzzer = 4,
};

// 口的能力位的线上编码（kGetPortList 每项的 capabilities 字段），按类型各有一组；一种
// 类型的位只在对应 kind 的口上有意义。哪个口置哪些位由固件的驱动类型决定
// （kPortCapabilities），不在 spec 里。
enum CanCapability : std::uint8_t {
    // 主机可在声明里改这路总线的 TX 帧型。清位 = 帧型是固件的编译期属性（c_board）。
    kCanCapModeSettable = 1U << 0,
    // 该口承载 CAN-FD 长帧（12-64 字节负载），双向。
    kCanCapFdLongFrames = 1U << 1,
    // 主机声明的速率是设置（板子解位时序）；清位 = 速率只作核对（速率由对端硬件决定，
    // 板子不改）。
    kCanCapRateSettable = 1U << 2,
};
enum UartCapability : std::uint8_t {
    // 端口有 RX 反相寄存器（mc02 DBUS 的 RXINV）：声明里的 rx_polarity 是设置；
    // 清位 = 极性是接线事实，声明里的非 Skip 极性只能是"无反相"。
    kUartCapRxPolaritySettable = 1U << 0,
};
// 一根 GPIO 线能做的事。声明里要用的每一项都得在这里有位, 否则校验阶段即拒绝
// (kConfigErrorUnsupportedMode, 值 = 缺的那些位)。同一个口上的线也可能不同: c_board
// 的 PWM6 所在的 EXTI 线另有用途, 没有 kGpioCapReadInterrupt。逐线的能力是板子自己的
// 接线事实, 只在板上用于校验, 不上报: GPIO 口在口清单里的能力字节是它的线数。
enum GpioCapability : std::uint8_t {
    kGpioCapDigitalWrite = 1U << 0,  // 输出: 写高/低电平
    kGpioCapAnalogWrite = 1U << 1,   // 输出: 写 PWM 占空比
    kGpioCapReadOnce = 1U << 2,      // 输入: 按请求读一次(kRead 记录)
    kGpioCapReadPeriodic = 1U << 3,  // 输入: 按声明的周期采样
    kGpioCapReadInterrupt = 1U << 4, // 输入: 边沿触发采样
    kGpioCapPullUp = 1U << 5,        // 输入: 内部上拉
    kGpioCapPullDown = 1U << 6,      // 输入: 内部下拉
    kGpioCapReadTimestamp = 1U << 7, // 输入: 样本带板上时间戳
};
// 定时器通道上的线(mc02、c_board 的 PWM 排针): 上面的全部。
inline constexpr std::uint8_t kGpioCapPwmPin = 0xFF;

// 一个口的名字与类型: 板型口表(Spec::kPorts)的一项。
struct PortDescriptor {
    data::DataId data_id;
    PortKind kind;

    constexpr bool operator==(const PortDescriptor&) const noexcept = default;
};

// ---- 类型化的口描述符：主机接线表与板端绑定都拿它当口的名字 ----
//
// 一个板型的 Spec::kCans.kCanN / Spec::kUarts.kUartN 就是这里的值类型：只带身份
// （DataId），类型在类型里。可拷贝、可做编译期常量、可做非类型模板参数。Tag 是板型的
// Spec 自己：两个板型的 CAN 描述符是不同的类型，把 mc02 的口交给 5321 的接线表是编译
// 错误，而不是运行时才暴露的错线。
template <typename Tag, PortKind KindV>
struct TypedPortDescriptor {
    data::DataId data_id;

    static constexpr PortKind kind = KindV;

    constexpr bool operator==(const TypedPortDescriptor&) const noexcept = default;
};

// GPIO 口上的一根线: 主机接线表拿它当引脚的名字(Spec::kGpios.kPwm3)。Tag 同上, 两个
// 板型的线是不同的类型。所属的口不必写: 一块板只有一个 GPIO 口(DataId::kGpio)。
template <typename Tag>
struct GpioLineDescriptor {
    std::uint8_t line;

    constexpr bool operator==(const GpioLineDescriptor&) const noexcept = default;
};

// 口表项只能经它从具名描述符造出：类型取自描述符的类型，于是"口表里的类型与描述符
// 不符"在构造上就不可能出现。
template <typename Tag, PortKind KindV>
constexpr PortDescriptor port(TypedPortDescriptor<Tag, KindV> descriptor) noexcept {
    return {.data_id = descriptor.data_id, .kind = KindV};
}

// 一个口的人名：主机报错与日志用它，固件不打印。kImu 不在丝印上，报错里给固定词。
constexpr std::string_view channel_name(data::DataId channel) noexcept {
    switch (channel) {
    case data::DataId::kCan0: return "CAN0";
    case data::DataId::kCan1: return "CAN1";
    case data::DataId::kCan2: return "CAN2";
    case data::DataId::kCan3: return "CAN3";
    case data::DataId::kUart0: return "UART0";
    case data::DataId::kUart1: return "UART1";
    case data::DataId::kUart2: return "UART2";
    case data::DataId::kUart3: return "UART3";
    case data::DataId::kUart7: return "UART7";
    case data::DataId::kUart10: return "UART10";
    case data::DataId::kUartDbus: return "DBUS";
    case data::DataId::kImu: return "the on-board IMU";
    case data::DataId::kGpio: return "GPIO";
    case data::DataId::kBuzzer: return "the buzzer";
    default: return "channel";
    }
}

// 一根 GPIO 线的人名。两块有 GPIO 的板(mc02、c_board)的线都按 PWM 排针的丝印排: 线 n
// 印作 PWM(n+1)。
constexpr std::string_view gpio_line_name(std::uint8_t line) noexcept {
    constexpr std::array<std::string_view, kMaxGpioLines> kNames{"PWM1", "PWM2", "PWM3", "PWM4",
                                                                 "PWM5", "PWM6", "PWM7", "PWM8"};
    return line < kNames.size() ? kNames[line] : "GPIO line";
}

// ---- 清单的编译期核对 ----
//
// 板型的 Spec::kPorts 是固件绑定与主机模板共同的口表。"同一 DataId 出现两次"是编译
// 错误——按 DataId 的编译期分发与一一绑定都以"口表内无重复"为前提。

// 一组带 data_id 的对象(描述符、固件绑定)两两不同身份。
template <typename... Descriptors>
constexpr bool distinct_data_ids(const Descriptors&... descriptors) noexcept {
    const std::array<data::DataId, sizeof...(Descriptors)> ids{descriptors.data_id...};
    for (std::size_t i = 0; i < sizeof...(Descriptors); ++i) {
        for (std::size_t j = i + 1; j < sizeof...(Descriptors); ++j) {
            if (ids[i] == ids[j])
                return false;
        }
    }
    return true;
}

constexpr bool no_duplicate_data_ids(std::span<const PortDescriptor> ports) noexcept {
    for (std::size_t i = 0; i < ports.size(); ++i) {
        for (std::size_t j = i + 1; j < ports.size(); ++j) {
            if (ports[i].data_id == ports[j].data_id)
                return false;
        }
    }
    return true;
}

// 板型的清单装得下: 每个非 GPIO 口一项, GPIO 口每根线一项。
constexpr bool
    manifest_fits(std::span<const PortDescriptor> ports, std::size_t gpio_lines) noexcept {
    std::size_t entries = gpio_lines;
    for (const auto& port : ports)
        entries += port.kind == PortKind::kGpio ? 0U : 1U;
    return ports.size() <= kMaxPorts && gpio_lines <= kMaxGpioLines
        && entries <= kMaxManifestEntries;
}

// 口表里 data_id 那一项; 没有时为空。
constexpr std::optional<PortDescriptor>
    find_descriptor(std::span<const PortDescriptor> ports, data::DataId data_id) noexcept {
    for (const auto& port : ports) {
        if (port.data_id == data_id)
            return port;
    }
    return std::nullopt;
}

} // namespace libhcs::spec
