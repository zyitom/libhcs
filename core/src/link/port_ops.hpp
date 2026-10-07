#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <type_traits>
#include <utility>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/protocol/vendor_control.hpp"
#include "core/include/libhcs/spec/port.hpp"
#include "core/src/link/port.hpp"

// 按口类型的通用操作 -- 各固件 vendor_control.cpp 里逐字重复的那部分语义, 收敛成
// 一份。差异只在驱动原语与能力位: 同一份流程, hpm5321 以"速率可设 + 模式可设"实例化,
// mc02 以"只有模式可设", c_board 以"都不可设", 三块板的行为与今天逐字段一致。
//
// 流程的形状是"先全量校验后统一提交": validate() 不写任何寄存器(读现值不算写), 但要
// 判完一切可预见的拒绝(含速率求解); 全部通过才轮到 apply(); apply() 先写后回读, 失败
// 即报 kConfigErrorVerifyFailed。清单事务
// 的两阶段提交(见 ep0.hpp)正好落在这两个调用上。

namespace libhcs::core::link {

namespace vc = libhcs::core::protocol::vendor_control;

// kind ops 的名字空间: 按口类型的通用流程。
namespace port_ops {

// 声明是完整的: 清单里一个口的设置必须把这个口怎么工作说全, 不存在"这一项沿用固件
// 当前"。固件当前的值可能来自 .ioc 的占位、上一个主机、或 CDC 在会话外的改写(实测
// ModemManager 把波特率设成过 9596), 主机看不见也就无从对账。
//
// 一个必填字段: 它的值, 与它在设置载荷里的字节偏移(拒绝时报给主机, 指明缺的是哪一项)。
struct RequiredField {
    std::uint32_t value;
    std::size_t offset;
};

// 第一个为 0 的必填字段即拒绝原因; 都给了则通过。
constexpr PortOutcome first_missing(std::initializer_list<RequiredField> fields) noexcept {
    for (const RequiredField& field : fields) {
        if (field.value == 0U)
            return PortOutcome::refuse(
                vc::ConfigErrorReason::kConfigErrorIncomplete,
                static_cast<std::uint32_t>(field.offset));
    }
    return PortOutcome::pass();
}

// ---- CAN ----
//
// 驱动原语(各板 CAN 驱动提供):
//   running() / suspend() / resume()   生命周期; resume 即"上总线" -- 被接受的声明
//                                      就是上线(常开的板 resume 是空操作)
//   fd_now()                           此刻实际发送的帧型
//   timing()                           寄存器重构的位时序事实
//   setting()                          libhcs 当前设置(清单上一次应用的结果)
//   expected_of(setting)               一份设置应用后应有的时序(hpm: 采样点钉 87.5‰;
//                                      mc02/c_board: 速率不变, 即 timing())
//   apply_setting(setting)             写硬件; false = 板子拒绝(求解失败等), 未生效
//   read_config(payload&)              kGetPortConfig 的应答
//   read_status()                      运行时状态(data::CanStatusView), 每个 keepalive
//                                      轮次读一次, 随数据流上报(port_status.hpp)
template <typename Can>
concept CanPortDriver =
    requires(Can& bus, const Can& cbus, const CanSetting& setting, vc::CanConfigPayload& config) {
        { bus.running() } -> std::same_as<bool>;
        { bus.suspend() } -> std::same_as<void>;
        { bus.resume() } -> std::same_as<void>;
        { bus.fd_now() } -> std::same_as<bool>;
        { cbus.timing() } -> std::convertible_to<CanTimingValue>;
        { cbus.setting() } -> std::convertible_to<CanSetting>;
        { cbus.expected_of(setting) } -> std::convertible_to<CanTimingValue>;
        { bus.apply_setting(setting) } -> std::same_as<bool>;
        { bus.read_config(config) } -> std::same_as<void>;
        { bus.read_status() } -> std::same_as<data::CanStatusView>;
    };

// 速率可设的板(hpm)另提供一条纯求解原语, 让"凑不出这个速率"在校验阶段就被发现:
//   unrepresentable_rate(setting)      0 = 这份设置能原样落到硬件上; 否则返回落不下的
//                                      那一段速率(仲裁段或数据段)。不碰任何寄存器。
// 校验阶段只要漏判一种可预见的拒绝, 它就会推迟到应用阶段, 触发整份清单的回滚 -- 运行期
// 改一路 CAN 的速率填错, 整块板的口就一起下线。所以求解必须在这里, 应用阶段剩下的
// 失败只有硬件回读不一致。
template <typename Can>
concept RateSettableCanDriver =
    CanPortDriver<Can> && requires(const Can& cbus, const CanSetting& setting) {
        { cbus.unrepresentable_rate(setting) } -> std::same_as<std::uint32_t>;
    };

// 声明的校验半边(不碰硬件)。kCaps 是该口驱动类型的能力(kPortCapabilities) -- 编译期常量, 按能力
// 分出的分支在实例化时就消失, 速率不可设的板也就不必提供求解原语。
template <std::uint8_t kCaps, CanPortDriver Can>
PortOutcome can_validate(Can& bus, const vc::CanConfigPayload& p) {
    constexpr bool mode_settable = (kCaps & spec::kCanCapModeSettable) != 0U;
    constexpr bool rate_settable = (kCaps & spec::kCanCapRateSettable) != 0U;

    if (p.mode > std::to_underlying(vc::CanMode::kCanFd))
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadRequest, p.mode);
    constexpr auto kAllowedTimingBit = vc::kCanConfigApplyTiming;
    const auto allowed_control = static_cast<std::uint8_t>(
        static_cast<unsigned>(vc::kCanConfigApply)
        | (rate_settable ? static_cast<unsigned>(kAllowedTimingBit) : 0U));
    if ((p.control & ~allowed_control) != 0U)
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadRequest, p.control);

    const bool apply_mode = (p.control & vc::kCanConfigApply) != 0U;
    const bool apply_timing = (p.control & vc::kCanConfigApplyTiming) != 0U;
    const bool want_fd = p.mode == std::to_underlying(vc::CanMode::kCanFd);

    // 帧型与速率是总线的接线事实, 主机必须说全; 采样点是板子的策略, 仍可只作核对或不给。
    if (!apply_mode)
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadRequest, p.control);
    using Payload = vc::CanConfigPayload;
    if (auto missing = first_missing({
            {        p.arbitration_baudrate,offsetof(Payload, arbitration_baudrate)            },
            {want_fd ? p.data_baudrate : 1U,
             offsetof(Payload,        data_baudrate)}, // 经典总线没有数据段
    });
        !missing.ok())
        return missing;

    // 速率与采样点是非零即核对的字段; 核对失败一律 kConfigErrorRateUnrepresentable,
    // 与旧协议各板的写法一致。
    const auto asserted = [&](std::uint32_t asserted_value, std::uint32_t actual) {
        return asserted_value == 0U || asserted_value == actual
                 ? PortOutcome::pass()
                 : PortOutcome::refuse(
                       vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, asserted_value);
    };

    if constexpr (!rate_settable) {
        // 速率是板子的整定事实: 声明只能断言, 不能改。
        const CanTimingValue truth = bus.timing();
        if (auto o = asserted(p.arbitration_baudrate, truth.arbitration_baudrate); !o.ok())
            return o;
        if (auto o = asserted(p.data_baudrate, truth.data_baudrate); !o.ok())
            return o;
        if (auto o = asserted(p.nominal_sample_point, truth.nominal_sample_point); !o.ok())
            return o;
        if (auto o = asserted(p.data_sample_point, truth.data_sample_point); !o.ok())
            return o;
    } else {
        // 速率可设: 目标设置 = 当前设置上改帧型(带 kCanConfigApply)与速率(带
        // kCanConfigApplyTiming, 非零才是设置)。核对基准: 帧型不变且不应用速率时比
        // 硬件现值, 否则比目标设置应用后应有的时序。
        if (apply_timing && p.data_baudrate != 0U && !want_fd) {
            // 经典总线没有数据段: 给经典目标下发数据段速率即预期有误。
            return PortOutcome::refuse(
                vc::ConfigErrorReason::kConfigErrorUnsupportedMode, p.data_baudrate);
        }
        static_assert(
            RateSettableCanDriver<Can>,
            "a port whose spec says the rate is settable needs a pure solver");
        CanSetting target = bus.setting();
        target.fd = want_fd;
        if (apply_timing) {
            if (p.arbitration_baudrate != 0U)
                target.arbitration_baudrate = p.arbitration_baudrate;
            if (p.data_baudrate != 0U)
                target.data_baudrate = p.data_baudrate;
        }
        if (const std::uint32_t rate = bus.unrepresentable_rate(target); rate != 0U)
            return PortOutcome::refuse(
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, rate);
        const CanTimingValue expected =
            (!apply_timing && want_fd == bus.fd_now()) ? bus.timing() : bus.expected_of(target);
        if (auto o = asserted(p.arbitration_baudrate, expected.arbitration_baudrate); !o.ok())
            return o;
        if (auto o = asserted(p.data_baudrate, expected.data_baudrate); !o.ok())
            return o;
        if (auto o = asserted(p.nominal_sample_point, expected.nominal_sample_point); !o.ok())
            return o;
        if (auto o = asserted(p.data_sample_point, expected.data_sample_point); !o.ok())
            return o;
    }

    // 帧型: 不可设的板只接受现值(kConfigErrorModeFixed); 可设的板不带
    // kCanConfigApply 时退化为纯核对。
    if (want_fd != bus.fd_now()) {
        if (!mode_settable)
            return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorModeFixed, p.mode);
        if (!apply_mode)
            return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorUnsupportedMode, p.mode);
    }
    return PortOutcome::pass();
}

// 声明的应用半边(写硬件 + 回读)。validate 已通过, 这里不再重复校验。
template <std::uint8_t kCaps, CanPortDriver Can>
PortOutcome can_apply(Can& bus, const vc::CanConfigPayload& p) {
    constexpr bool mode_settable = (kCaps & spec::kCanCapModeSettable) != 0U;
    constexpr bool rate_settable = (kCaps & spec::kCanCapRateSettable) != 0U;
    const bool apply_timing = (p.control & vc::kCanConfigApplyTiming) != 0U && rate_settable;
    const bool want_fd = p.mode == std::to_underlying(vc::CanMode::kCanFd);
    const bool mode_changes = want_fd != bus.fd_now();

    if (mode_settable && (mode_changes || apply_timing)) {
        CanSetting target = bus.setting();
        target.fd = want_fd;
        if (apply_timing) {
            if (p.arbitration_baudrate != 0U)
                target.arbitration_baudrate = p.arbitration_baudrate;
            if (p.data_baudrate != 0U)
                target.data_baudrate = p.data_baudrate;
        }
        // 校验阶段已用同一求解判过这份设置能落地; 这里的拒绝只剩"求解与硬件不一致"
        // 这种不该发生的情况, 照样整体回滚(清单事务的语义, 见 ep0.hpp)。数据段变过
        // 就报数据段速率, 与旧协议一致。
        if (!bus.apply_setting(target)) {
            const bool data_changed = want_fd && apply_timing && p.data_baudrate != 0U;
            return PortOutcome::refuse(
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable,
                data_changed ? target.data_baudrate : target.arbitration_baudrate);
        }
        // 写后回读: 主机不再回读, ACK 前确认帧型与时序都已如所求。
        if (bus.fd_now() != want_fd || !(bus.timing() == bus.expected_of(target))) [[unlikely]]
            return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorVerifyFailed, p.mode);
    }

    bus.resume(); // 被接受的设置就是声明: 总线按声明上线
    return PortOutcome::pass();
}

template <CanPortDriver Can>
void can_read_config(const Can& bus, vc::CanConfigPayload& out) {
    bus.read_config(out);
}

template <CanPortDriver Can>
PortStatus can_describe(const Can& bus) {
    return {.running = bus.running(), .fd = bus.fd_now()};
}

// ---- UART ----
//
// 驱动原语(各板 UART 驱动提供):
//   running() / suspend() / resume()
//   check_framing(wl, par, sb, rx_pol)     纯校验, 不碰寄存器
//   solve(rate, div&, over&)               纯求解; 拒绝即速率不可表示
//   baudrate_for(div, over)                纯: 分频器对应的实际速率
//   commit_baudrate(rate, div, over)       写入(hpm 内部重解一次, 仍可能失败)
//   verify_baudrate(div, over)             回读: 写后核对, 写前判"没变"
//   commit_framing(wl, par, sb, rx_pol)    一次写入; 无失败路径
//   framing_matches(wl, par, sb, rx_pol)   帧格式回读
//   read_config(payload&)                  kGetPortConfig 的应答
//   read_status()                          运行时接收错误计数
template <typename Uart>
concept UartPortDriver = requires(Uart& port, const Uart& cport, vc::UartConfigPayload& config) {
    { port.running() } -> std::same_as<bool>;
    { port.suspend() } -> std::same_as<void>;
    { port.resume() } -> std::same_as<void>;
    { cport.check_framing(0U, 0U, 0U, 0U) } -> std::same_as<bool>;
    {
        port.solve(0U, std::declval<std::uint16_t&>(), std::declval<std::uint8_t&>())
    } -> std::same_as<bool>;
    { cport.baudrate_for(0U, 0U) } -> std::same_as<std::uint32_t>;
    { port.commit_baudrate(0U, 0U, 0U) } -> std::same_as<bool>;
    { cport.verify_baudrate(0U, 0U) } -> std::same_as<bool>;
    { port.commit_framing(0U, 0U, 0U, 0U) } -> std::same_as<void>;
    { cport.framing_matches(0U, 0U, 0U, 0U) } -> std::same_as<bool>;
    { port.read_config(config) } -> std::same_as<void>;
    { port.read_status() } -> std::same_as<data::UartStatusView>;
};

template <UartPortDriver Uart>
PortOutcome uart_validate(Uart& port, const vc::UartConfigPayload& p) {
    // 清单里的 UART 项是设置, 不是断言: kUartConfigApply 必须在, 速率与帧格式必须说全。
    // 分频器与过采样仍是可选的断言(0 = 不核对), 它们是速率的整数形态, 不是另一项设置。
    if (p.control != vc::kUartConfigApply)
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadRequest, p.control);
    using Payload = vc::UartConfigPayload;
    if (auto missing = first_missing({
            {   p.baudrate, offsetof(Payload,    baudrate)},
            {p.word_length, offsetof(Payload, word_length)},
            {     p.parity, offsetof(Payload,      parity)},
            {  p.stop_bits, offsetof(Payload,   stop_bits)},
            {p.rx_polarity, offsetof(Payload, rx_polarity)},
    });
        !missing.ok())
        return missing;

    // 帧格式先纯校验(不碰寄存器), 波特率先解不写: 求解与断言全部前置到动寄存器之前,
    // STALL 严格等于"什么都没改"。
    if (!port.check_framing(p.word_length, p.parity, p.stop_bits, p.rx_polarity))
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorFramingUnsupported);

    std::uint16_t divisor = 0;
    std::uint8_t oversample = 0;
    if (!port.solve(p.baudrate, divisor, oversample))
        return PortOutcome::refuse(
            vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, p.baudrate);
    // 宽松兜底前置: 解出的速率离请求太远同样算不可表示。
    if (!rate_plausible(p.baudrate, port.baudrate_for(divisor, oversample)))
        return PortOutcome::refuse(
            vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, p.baudrate);
    // 调用方回显了分频器/过采样就要求它与求解结果相等 -- 这一步让"整条电气身份一起
    // 断言"成立: 主机说不出自己期望的分频器, 就不该假装验证过速率。
    if (p.divisor != 0U && p.divisor != divisor)
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorVerifyFailed, p.baudrate);
    if (p.oversample != 0U && p.oversample != oversample)
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorVerifyFailed, p.baudrate);
    return PortOutcome::pass();
}

template <UartPortDriver Uart>
PortOutcome uart_apply(Uart& port, const vc::UartConfigPayload& p) {
    std::uint16_t divisor = 0;
    std::uint8_t oversample = 0;
    // validate 阶段已解过一次, 这里失败说明求解器与自身前后不一致 -- 与"求解器拒绝"
    // 同答, 硬件未被触碰(commit_baudrate 先解后写)。
    if (!port.solve(p.baudrate, divisor, oversample)) [[unlikely]]
        return PortOutcome::refuse(
            vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, p.baudrate);
    // 设置没变就不碰硬件(与 GPIO configure()、hpm CAN apply_setting() 同一规矩): 运行期改
    // 任意一个口或重连都会重放整份清单, 而写分频器/帧格式要停口(STM32 拉 UE, hpm 中止发送
    // DMA 并开 DLAB), 收发中的字节会丢。挂起的口(接手、回滚之后)照常整份重写。
    if (port.running() && port.verify_baudrate(divisor, oversample)
        && port.framing_matches(p.word_length, p.parity, p.stop_bits, p.rx_polarity))
        return PortOutcome::pass();
    if (!port.commit_baudrate(p.baudrate, divisor, oversample)) [[unlikely]]
        return PortOutcome::refuse(
            vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, p.baudrate);
    // 写入后回读: 确认分频器真的收下了。与"求解器拒绝"是两回事, 见
    // kConfigErrorVerifyFailed。
    if (!port.verify_baudrate(divisor, oversample)) [[unlikely]]
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorVerifyFailed, p.baudrate);
    port.commit_framing(p.word_length, p.parity, p.stop_bits, p.rx_polarity);
    // 写后回读帧格式: 主机不再回读, ACK 必须等于已生效。
    if (!port.framing_matches(p.word_length, p.parity, p.stop_bits, p.rx_polarity)) [[unlikely]]
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorVerifyFailed);
    port.resume(); // 被接受的设置就是声明
    return PortOutcome::pass();
}

template <UartPortDriver Uart>
PortStatus uart_describe(const Uart& port) {
    return {.running = port.running(), .fd = false};
}

// ---- IMU ----
//
// 板子只有一颗、行为也确实各板不同(mc02 解档位并启动, c_board 只核对编译期设置),
// 不抽公共流程: 驱动直接提供同一组方法, 注册表按 kind 分发到它们。
template <typename Imu>
concept ImuPortDriver = requires(Imu& imu, const Imu& cimu, vc::ImuConfigPayload& config) {
    { imu.running() } -> std::same_as<bool>;
    { imu.suspend() } -> std::same_as<void>;
    { imu.resume() } -> std::same_as<void>;
    { imu.validate(config) } -> std::same_as<PortOutcome>;
    { imu.apply(config) } -> std::same_as<PortOutcome>;
    { imu.read_config(config) } -> std::same_as<void>;
};

// 量程与速率是主机换算原始值用的数, 必须由主机说全; 档位能不能落到芯片上由驱动判。
template <ImuPortDriver Imu>
PortOutcome imu_validate(Imu& imu, const vc::ImuConfigPayload& p) {
    using Payload = vc::ImuConfigPayload;
    if (auto missing = first_missing({
            { p.accelerometer_range_g, offsetof(Payload,  accelerometer_range_g)},
            { p.accelerometer_rate_hz, offsetof(Payload,  accelerometer_rate_hz)},
            {   p.gyroscope_range_dps, offsetof(Payload,    gyroscope_range_dps)},
            {     p.gyroscope_rate_hz, offsetof(Payload,      gyroscope_rate_hz)},
            {p.gyroscope_bandwidth_hz, offsetof(Payload, gyroscope_bandwidth_hz)},
    });
        !missing.ok())
        return missing;
    return imu.validate(p);
}

// ---- GPIO ----
//
// 一个 GPIO 口 = 一组线(PWM 排针), 一线一项声明。设置(vc::GpioConfigPayload)就是一根线
// 的声明的全部: 方向, 以及输入怎么采样。
//
// 线的原语(各板的引脚驱动提供):
//   running() / suspend() / resume()   suspend: 输出拉低(比较值 0, 不放成高阻 -- 悬空的
//                                      信号线会被电调读成杂波)、输入撤掉边沿与周期采样、
//                                      下行的写与读请求一律丢弃; resume: 按当前设置重新
//                                      接收写入 / 开始采样
//   configure(setting)                 写引脚寄存器(方向、复用、上下拉、边沿)。设置与现在
//                                      的相同时什么都不写: 运行期改别的口会重放整份清单,
//                                      正在跑的 PWM 不能因此掉到 0
//   matches(setting)                   写后回读: 方向与上下拉寄存器
//   read_config(payload&)              GET 的应答; 没声明的线报 kGpioModeOff
template <typename Line>
concept GpioLineDriver = requires(
    Line& line, const Line& cline, const vc::GpioConfigPayload& setting,
    vc::GpioConfigPayload& config) {
    { line.running() } -> std::same_as<bool>;
    { line.suspend() } -> std::same_as<void>;
    { line.resume() } -> std::same_as<void>;
    { line.configure(setting) } -> std::same_as<void>;
    { cline.matches(setting) } -> std::same_as<bool>;
    { cline.read_config(config) } -> std::same_as<void>;
};

// 口的原语(各板的 GPIO 服务提供):
//   kLineCount, kLineCapabilities[]    线数与逐线能力: 接线事实, 编译期常量
//   line(index)                        第 index 根线的驱动; 越界为 nullptr
//   suspend() / resume() / describe()  整口: 注册表的遍历、会话结束、归属交接用
template <typename Port>
concept GpioPortDriver = requires(Port& port, std::uint8_t index) {
    { Port::kLineCount } -> std::convertible_to<std::uint8_t>;
    { Port::kLineCapabilities[index] } -> std::convertible_to<std::uint8_t>;
    requires GpioLineDriver<std::remove_pointer_t<decltype(port.line(index))>>;
    { port.suspend() } -> std::same_as<void>;
    { port.resume() } -> std::same_as<void>;
};

// 声明的校验半边。只看设置与这根线的能力(口的 kLineCapabilities), 不需要驱动实例: 线能
// 做什么是接线事实, 不随状态变。缺能力的拒绝带上缺的那些位。
constexpr PortOutcome gpio_validate(std::uint8_t caps, const vc::GpioConfigPayload& p) noexcept {
    const auto require = [caps](unsigned needed) {
        const unsigned missing = needed & ~static_cast<unsigned>(caps);
        return missing == 0U
                 ? PortOutcome::pass()
                 : PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorUnsupportedMode, missing);
    };

    switch (p.mode) {
    case vc::kGpioModeOutput:
        // 采样字段只对输入有意义: 输出上给了, 是主机把两种声明弄混了。
        if (p.pull != vc::kGpioPullNone || p.input_flags != 0U || p.period_ms != 0U)
            return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadRequest, p.mode);
        // 电平与占空比至少能写一样。
        if ((caps & (spec::kGpioCapDigitalWrite | spec::kGpioCapAnalogWrite)) == 0U)
            return require(spec::kGpioCapDigitalWrite);
        return PortOutcome::pass();

    case vc::kGpioModeInput: {
        constexpr unsigned kEdges = vc::kGpioInputRisingEdge | vc::kGpioInputFallingEdge;
        constexpr unsigned kKnownFlags = kEdges | vc::kGpioInputTimestamp;
        if (p.pull > vc::kGpioPullDown)
            return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadRequest, p.pull);
        if ((p.input_flags & ~kKnownFlags) != 0U)
            return PortOutcome::refuse(
                vc::ConfigErrorReason::kConfigErrorBadRequest, p.input_flags);
        unsigned needed = 0;
        if (p.period_ms != 0U)
            needed |= spec::kGpioCapReadPeriodic;
        if ((p.input_flags & kEdges) != 0U)
            needed |= spec::kGpioCapReadInterrupt;
        // 不周期采样也不等边沿的输入, 只能靠按请求读(kRead)取样。
        if (needed == 0U)
            needed |= spec::kGpioCapReadOnce;
        if (p.pull == vc::kGpioPullUp)
            needed |= spec::kGpioCapPullUp;
        if (p.pull == vc::kGpioPullDown)
            needed |= spec::kGpioCapPullDown;
        if ((p.input_flags & vc::kGpioInputTimestamp) != 0U)
            needed |= spec::kGpioCapReadTimestamp;
        return require(needed);
    }

    default:
        // kGpioModeOff 只出现在 GET 里: "声明一根线但不用它"就是不声明它。
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadRequest, p.mode);
    }
}

// 一根线的声明的校验: 线号在口里, 设置在这根线的能力之内。
template <GpioPortDriver Port>
constexpr PortOutcome
    gpio_validate_line(std::uint8_t line, const vc::GpioConfigPayload& p) noexcept {
    if (line >= Port::kLineCount)
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadIndex, line);
    return gpio_validate(Port::kLineCapabilities[line], p);
}

// 声明的应用半边(写 + 回读 + 启用)。validate 已通过。
template <GpioLineDriver Line>
PortOutcome gpio_apply(Line& line, const vc::GpioConfigPayload& p) {
    line.configure(p);
    if (!line.matches(p)) [[unlikely]]
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorVerifyFailed, p.mode);
    line.resume();
    return PortOutcome::pass();
}

// ---- 蜂鸣器 ----
//
// 声明不带参数: 设置载荷全零(频率与音量只出现在 GET 的应答里)。驱动原语:
//   running() / suspend() / resume()   suspend 静音; resume 只让声明过的蜂鸣器重新接收音
//   apply(payload) -> PortOutcome      启动定时器通道(静音)并回读通道使能; 已在跑的不动,
//                                      重放清单不打断正在响的音
//   read_config(payload&)              此刻在放的音, 从定时器寄存器重建
constexpr PortOutcome buzzer_validate(const vc::BuzzerConfigPayload& p) noexcept {
    if (p.frequency_hz != 0U || p.loudness != 0U || p.reserved != 0U)
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadRequest);
    return PortOutcome::pass();
}

} // namespace port_ops

} // namespace libhcs::core::link
