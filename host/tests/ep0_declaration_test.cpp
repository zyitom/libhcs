// The host half of "configuration is declaration", run against a fake board
// built from the SAME pure-logic core the firmware runs
// (core/src/link/: the port registry, the per-kind setting ops, the EP0
// dispatch, the manifest two-phase commit). The old test used a hand-written
// SimulatedBoard that reimplemented the firmware's side of the protocol --
// firmware changed, it stayed. Here the fake board IS the core with in-memory
// drivers behind it, so a protocol change breaks this test at compile time
// instead of drifting.
//
// What the host side must keep: it reads the port list (pure), validates the
// wiring against it, sends one manifest, reads the per-port results, records
// what was declared, and refuses to transmit on anything else. That the
// firmware keeps its half on real hardware is what the on-board test checks
// (hcs_core/test/test_bench_boards.cpp).

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <ostream>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <libhcs/board/hcs_config.hpp>
#include <libhcs/protocol/vendor_control.hpp>
#include <libhcs/spec/hpm5321/ports.hpp>
#include <libhcs/spec/mc02/ports.hpp>

#include "core/src/link/ep0.hpp"
#include "core/src/link/ownership.hpp"
#include "core/src/link/port_status.hpp"
#include "core/src/link/registry.hpp"

namespace {

namespace hcs = libhcs::board::hcs;
namespace vc = libhcs::core::protocol::vendor_control;
namespace spec = libhcs::spec;
namespace link = libhcs::core::link;
namespace ep0 = libhcs::core::link::ep0;
namespace data = libhcs::data;
using libhcs::data::DataId;

// Whether a thrown message says what it should; on failure the whole message is shown.
testing::AssertionResult mentions(const std::string& message, const std::string& expected) {
    if (message.find(expected) != std::string::npos)
        return testing::AssertionSuccess();
    return testing::AssertionFailure()
        << "expected the error to mention \"" << expected << "\", got: \"" << message << "\"";
}

// 本项目 BMI088 一直用的那一组(HCS 的 device::kBmi088Setup): 声明必须说全。
constexpr hcs::ImuSetting kBmi088{
    .accelerometer_range_g = 6,
    .accelerometer_rate_hz = 1600,
    .gyroscope_range_dps = 2000,
    .gyroscope_rate_hz = 2000,
    .gyroscope_bandwidth_hz = 230};

// ---- 内存驱动: 端口寄存器的最小替身, 只供核心的 kind ops 驱动 ----

struct FakeCan {
    bool is_running = false;
    bool fd = true; // 上电帧型(hpm 的策略; mc02 侧测试同样以 FD 上电)
    link::CanSetting setting_value{
        .fd = true, .arbitration_baudrate = 1'000'000, .data_baudrate = 5'000'000};
    link::CanTimingValue timing_value{1'000'000, 5'000'000, 875, 875};
    std::uint32_t applies = 0;   // 真重新初始化控制器的次数: "没碰硬件"的断言用
    bool garbles_timing = false; // 写进去的时序回读不符: 只有应用阶段看得见的失败

    void suspend() { is_running = false; }
    void resume() { is_running = true; }
    [[nodiscard]] bool running() const { return is_running; }
    [[nodiscard]] bool fd_now() const { return fd; }
    [[nodiscard]] link::CanTimingValue timing() const { return timing_value; }
    [[nodiscard]] const link::CanSetting& setting() const { return setting_value; }
    // hpm 语义: 采样点钉 875‰, 经典模式没有数据段。
    [[nodiscard]] link::CanTimingValue expected_of(const link::CanSetting& s) const {
        return {
            s.arbitration_baudrate, s.fd ? s.data_baudrate : 0U, 875,
            s.fd ? std::uint16_t{875} : std::uint16_t{0}};
    }
    // 纯求解(hpm 的 RateSettableCanDriver): 87.5‰ 采样点下凑不出 7 Mbit。校验阶段就判,
    // 不等应用阶段。
    [[nodiscard]] static std::uint32_t unrepresentable_rate(const link::CanSetting& s) {
        if (s.arbitration_baudrate == 7'000'000)
            return s.arbitration_baudrate;
        if (s.fd && s.data_baudrate == 7'000'000)
            return s.data_baudrate;
        return 0;
    }
    // hpm 语义: 在总线上且与硬件现状一致时只记下选择, 否则重新初始化控制器。
    [[nodiscard]] bool apply_setting(const link::CanSetting& s) {
        setting_value = s;
        if (is_running && fd == s.fd && timing_value == expected_of(s))
            return true;
        fd = s.fd;
        timing_value = expected_of(s);
        if (garbles_timing)
            timing_value.data_sample_point = 750;
        ++applies;
        return true;
    }
    void read_config(vc::CanConfigPayload& out) const {
        out.mode = std::to_underlying(fd ? vc::CanMode::kCanFd : vc::CanMode::kClassic);
        out.arbitration_baudrate = timing_value.arbitration_baudrate;
        out.data_baudrate = timing_value.data_baudrate;
        out.nominal_sample_point = timing_value.nominal_sample_point;
        out.data_sample_point = timing_value.data_sample_point;
    }
    [[nodiscard]] data::CanStatusView read_status() const { return status; }
    [[nodiscard]] link::PortStatus describe() const { return {.running = is_running, .fd = fd}; }

    data::CanStatusView status{}; // read_status() 的读数: 运行时状态的替身
};

// 一个串口: 分频器与帧格式寄存器的替身, 初值是上电的 115200 8N1。
struct FakeUart {
    using Framing = std::array<std::uint32_t, 4>; // 字长, 校验, 停止位, 接收极性
    std::uint32_t baudrate = 115200;
    std::uint16_t divisor = 100;                  // 184.32 MHz / (16 * 115200)
    Framing framing{8, 1, 1, 1};
    bool is_running = false;
    std::uint32_t writes = 0; // 真写分频器或帧格式的次数: "重放清单不碰串口"的断言用

    void suspend() { is_running = false; }
    void resume() { is_running = true; }
    [[nodiscard]] bool running() const { return is_running; }
    [[nodiscard]] static bool
        check_framing(std::uint32_t wl, std::uint32_t par, std::uint32_t sb, std::uint32_t pol) {
        return pol <= 1U && (wl == 0U || wl == 7U || wl == 8U) && (par == 0U || par <= 3U)
            && (sb == 0U || sb == 1U || sb == 2U);
    }
    // 求解器: 123 baud 凑不出(测试里的"不可表示"); 其余按 184.32 MHz 时钟整除。
    [[nodiscard]] static bool
        solve(std::uint32_t rate, std::uint16_t& divisor, std::uint8_t& oversample) {
        if (rate == 123 || rate == 0U)
            return false;
        constexpr std::uint32_t kClockHz = 184'320'000U; // 115200 * 1600
        divisor = static_cast<std::uint16_t>(kClockHz / (16U * rate));
        oversample = 16;
        return divisor != 0U;
    }
    [[nodiscard]] static std::uint32_t
        baudrate_for(std::uint16_t divisor, std::uint8_t oversample) {
        return oversample != 0U && divisor != 0U ? 184'320'000U / (16U * divisor) : 0U;
    }
    [[nodiscard]] std::uint32_t effective_baudrate() const { return baudrate; }
    [[nodiscard]] std::uint16_t divisor_u16() const { return divisor; }
    [[nodiscard]] static std::uint8_t oversample() { return 16; }
    bool commit_baudrate(std::uint32_t rate, std::uint16_t new_divisor, std::uint8_t) {
        baudrate = rate;
        divisor = new_divisor;
        ++writes;
        return true;
    }
    [[nodiscard]] bool verify_baudrate(std::uint16_t expected, std::uint8_t over) const {
        return divisor == expected && over == oversample();
    }
    void commit_framing(std::uint32_t wl, std::uint32_t par, std::uint32_t sb, std::uint32_t pol) {
        framing = {wl, par, sb, pol};
        ++writes;
    }
    [[nodiscard]] bool framing_matches(
        std::uint32_t wl, std::uint32_t par, std::uint32_t sb, std::uint32_t pol) const {
        return framing == Framing{wl, par, sb, pol};
    }
    void read_config(vc::UartConfigPayload& out) const {
        out.baudrate = baudrate;
        out.divisor = divisor_u16();
        out.oversample = oversample();
    }
    [[nodiscard]] data::UartStatusView read_status() const { return status; }
    [[nodiscard]] link::PortStatus describe() const { return {.running = is_running, .fd = false}; }

    data::UartStatusView status{};
};

struct FakeImu {
    static constexpr std::uint8_t kPortCapabilities = 0;

    bool is_running = false;
    vc::ImuConfigPayload setting{};                    // 最近一次被接受的声明
    std::optional<vc::ConfigErrorReason> sensor_fault; // 芯片不配合时报这个原因

    void suspend() { is_running = false; }
    void resume() { is_running = true; }
    [[nodiscard]] bool running() const { return is_running; }
    // 档位核对(物理量 0 = 默认), 与 BMI088 的档位表一致。
    [[nodiscard]] link::PortOutcome validate(const vc::ImuConfigPayload& p) const {
        const auto known = [](auto value, auto... choices) { return ((value == choices) || ...); };
        if (p.accelerometer_range_g != 0U && !known(p.accelerometer_range_g, 3, 6, 12, 24))
            return link::PortOutcome::refuse(
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, p.accelerometer_range_g);
        if (p.accelerometer_rate_hz != 0U
            && !known(p.accelerometer_rate_hz, 12, 25, 50, 100, 200, 400, 800, 1600))
            return link::PortOutcome::refuse(
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, p.accelerometer_rate_hz);
        if (p.gyroscope_rate_hz != 0U && !known(p.gyroscope_rate_hz, 100, 200, 400, 1000, 2000))
            return link::PortOutcome::refuse(
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, p.gyroscope_rate_hz);
        return link::PortOutcome::pass();
    }
    [[nodiscard]] link::PortOutcome apply(const vc::ImuConfigPayload& p) {
        if (sensor_fault.has_value())
            return link::PortOutcome::refuse(*sensor_fault);
        setting = p;
        is_running = true;
        return link::PortOutcome::pass();
    }
    void read_config(vc::ImuConfigPayload& out) const { out = setting; }
    [[nodiscard]] link::PortStatus describe() const { return {.running = is_running, .fd = false}; }
};

// 一根 GPIO 线: 引脚寄存器(方向 + 上下拉, 即 setting)与输出比较值的替身。
struct FakeGpio {
    vc::GpioConfigPayload setting{}; // 写进引脚寄存器的设置; mode 0 = 从没声明过
    bool is_running = false;
    std::uint16_t output = 0;        // 输出比较值: 挂起即归零
    std::uint32_t writes = 0;        // 真写引脚寄存器的次数: "重放清单不碰引脚"的断言用
    bool garbles = false;            // 写进去的方向回读不符

    void suspend() {
        is_running = false;
        output = 0;
    }
    void resume() { is_running = setting.mode != vc::kGpioModeOff; }
    [[nodiscard]] bool running() const { return is_running; }
    // 与固件同样的规矩: 设置没变就不写引脚, 正在跑的输出值保持。
    void configure(const vc::GpioConfigPayload& p) {
        if (std::memcmp(&p, &setting, sizeof(p)) == 0)
            return;
        setting = p;
        output = 0;
        ++writes;
    }
    [[nodiscard]] bool matches(const vc::GpioConfigPayload& p) const {
        return !garbles && setting.mode == p.mode && setting.pull == p.pull;
    }
    void read_config(vc::GpioConfigPayload& out) const {
        out = is_running ? setting : vc::GpioConfigPayload{};
    }
};

// GPIO 口: 一组线, 逐线能力是接线事实(port_ops.hpp 的 GpioPortDriver)。最后一根线少了边沿
// 中断(真 mc02 的四根都有; c_board 的 PWM6 才是这样): 缺能力的拒绝要有一根线来测。
struct FakeGpioPort {
    static constexpr std::uint8_t kLineCount = 4;
    static constexpr std::array<std::uint8_t, kLineCount> kLineCapabilities{
        spec::kGpioCapPwmPin, spec::kGpioCapPwmPin, spec::kGpioCapPwmPin,
        spec::kGpioCapPwmPin & ~spec::kGpioCapReadInterrupt};
    static constexpr std::uint8_t kPortCapabilities = kLineCount;

    std::array<FakeGpio, kLineCount> lines{};

    FakeGpio* line(std::uint8_t index) { return index < kLineCount ? &lines[index] : nullptr; }
    void suspend() {
        for (FakeGpio& l : lines)
            l.suspend();
    }
    void resume() {
        for (FakeGpio& l : lines)
            l.resume();
    }
    [[nodiscard]] link::PortStatus describe() const {
        return {
            .running = std::ranges::any_of(lines, [](const FakeGpio& l) { return l.is_running; }),
            .fd = false};
    }
};

// 蜂鸣器: 定时器通道(started)与在放的音的替身。
struct FakeBuzzer {
    static constexpr std::uint8_t kPortCapabilities = 0;
    bool started = false;
    bool is_running = false;
    std::uint16_t frequency_hz = 0;
    std::uint8_t loudness = 0;

    [[nodiscard]] bool running() const { return is_running; }
    void suspend() {
        is_running = false;
        frequency_hz = 0;
        loudness = 0;
    }
    void resume() { is_running = started; }
    link::PortOutcome apply(const vc::BuzzerConfigPayload&) {
        started = true;
        is_running = true;
        return link::PortOutcome::pass();
    }
    void read_config(vc::BuzzerConfigPayload& out) const {
        out = {.frequency_hz = frequency_hz, .loudness = loudness, .reserved = 0};
    }
    [[nodiscard]] link::PortStatus describe() const { return {.running = is_running, .fd = false}; }
};

// ---- 板形: 驱动 + 口清单(与真实板型的 spec 同构) ----

// 能力跟着驱动类型走(固件里是 kPortCapabilities, core/src/link/registry.hpp): 同一个内存
// 驱动按板型的能力实例化。
template <std::uint8_t kCaps>
struct FakeCanWith : FakeCan {
    static constexpr std::uint8_t kPortCapabilities = kCaps;
};
template <std::uint8_t kCaps>
struct FakeUartWith : FakeUart {
    static constexpr std::uint8_t kPortCapabilities = kCaps;
};

// mc02: 3 CAN(只有模式可设, 不承载长帧) + DBUS(RX 反相可设) + 5 个普通 UART + IMU
// + GPIO 口(4 根线, PWM4 没有边沿中断)。
struct Mc02Drivers {
    std::array<FakeCanWith<spec::kCanCapModeSettable>, 3> can{};
    FakeUartWith<spec::kUartCapRxPolaritySettable> dbus;
    std::array<FakeUartWith<0>, 5> others{}; // UART1, 2, 3, 7, 10
    FakeImu imu;
    FakeGpioPort gpio_port;
    FakeBuzzer buzzer;

    // 测试侧按线号看 GPIO: 0 = PWM1, ... 3 = PWM4。
    FakeGpio& gpio(std::size_t k) { return gpio_port.lines[k]; }

    // 口表里第 k 个 UART(DBUS 在前), 驱动的确切类型。
    template <std::size_t k>
    auto& uart_slot() {
        if constexpr (k == 0)
            return dbus;
        else
            return others[k - 1];
    }
    // 测试侧按口表顺序看 UART 的状态: 0 = DBUS, 1 = UART1, ... 5 = UART10。
    FakeUart& uart(std::size_t k) { return k == 0 ? static_cast<FakeUart&>(dbus) : others[k - 1]; }
};
// hpm5321: 2 CAN(全能力) + 1 UART。
struct HpmDrivers {
    std::array<
        FakeCanWith<
            spec::kCanCapModeSettable | spec::kCanCapFdLongFrames | spec::kCanCapRateSettable>,
        2>
        can{};
    FakeUartWith<0> uart0;

    template <std::size_t k>
    auto& uart_slot() {
        static_assert(k == 0);
        return uart0;
    }
    FakeUart& uart(std::size_t /*k*/) { return uart0; }
};

// 板形 = 真实板型的口表 + 它的具名描述符(与固件 ports.hpp 绑定的是同一组) + 内存驱动。
struct Mc02Shape {
    using Drivers = Mc02Drivers;
    using Spec = spec::mc02::Spec;
    static constexpr auto kDescriptors = std::tuple{
        Spec::Cans::kCan1,    Spec::Cans::kCan2,   Spec::Cans::kCan3,   Spec::Uarts::kDbus,
        Spec::Uarts::kUart1,  Spec::Uarts::kUart2, Spec::Uarts::kUart3, Spec::Uarts::kUart7,
        Spec::Uarts::kUart10, Spec::kImu,          Spec::kGpio,         Spec::kBuzzer};
};

// dual_can 是"本 PCB 实际装了几个口"的运行时事实: 单 CAN 板的第二个 CAN 驱动不初始化,
// 口清单里就没有它, 清单里声明它也会被拒。
struct HpmShape {
    using Drivers = HpmDrivers;
    using Spec = spec::hpm5321::Spec;
    static constexpr auto kDescriptors =
        std::tuple{Spec::Cans::kCan1, Spec::Cans::kCan2, Spec::Uarts::kUart0};
};

// 驱动的地址由板构造时登记: 绑定经它取实例。测试一次只有一块板在, 单指针足够。
template <typename Shape>
struct DriversPtr {
    static typename Shape::Drivers* drivers;
    static bool dual_can; // 仅 hpm 形有意义
};
template <typename Shape>
typename Shape::Drivers* DriversPtr<Shape>::drivers = nullptr;
template <typename Shape>
bool DriversPtr<Shape>::dual_can = true;

// 绑定 = 固件同款的 PortOf<描述符>(身份与类型取自 spec) + 驱动怎么到。能力由驱动类型给。
template <typename Shape, std::size_t I>
struct Binding : link::PortOf<std::get<I>(Shape::kDescriptors)> {
    using Base = link::PortOf<std::get<I>(Shape::kDescriptors)>;

    // 同类口里的第几个 = 驱动的下标。
    static constexpr std::size_t kOrdinal = [] {
        std::size_t ordinal = 0;
        for (std::size_t j = 0; j < I; ++j)
            ordinal += Shape::Spec::kPorts[j].kind == Base::kind ? 1U : 0U;
        return ordinal;
    }();

    static auto* instance() {
        // 三个分支类型不同; if constexpr 保证每个实例化只留一个返回类型。空指针 = 驱动
        // 没初始化 = 本 PCB 没有这个口(没有这块板、单 CAN 板的 CAN2)。
        auto* drivers = DriversPtr<Shape>::drivers;
        if constexpr (Base::kind == spec::PortKind::kCan) {
            const bool fitted = kOrdinal != 1U || DriversPtr<Shape>::dual_can;
            return drivers != nullptr && fitted ? &drivers->can[kOrdinal] : nullptr;
        } else if constexpr (Base::kind == spec::PortKind::kUart) {
            return drivers != nullptr ? &drivers->template uart_slot<kOrdinal>() : nullptr;
        } else if constexpr (Base::kind == spec::PortKind::kImu) {
            return drivers != nullptr ? &drivers->imu : nullptr;
        } else if constexpr (Base::kind == spec::PortKind::kGpio) {
            return drivers != nullptr ? &drivers->gpio_port : nullptr;
        } else {
            return drivers != nullptr ? &drivers->buzzer : nullptr;
        }
    }
};

template <typename Shape, std::size_t... I>
auto make_registry(std::index_sequence<I...>) -> link::PortRegistry<Binding<Shape, I>...>;
template <typename Shape>
using RegistryFor = decltype(make_registry<Shape>(
    std::make_index_sequence<std::tuple_size_v<decltype(Shape::kDescriptors)>>{}));

static_assert(RegistryFor<Mc02Shape>::matches(Mc02Shape::Spec::kPorts));
static_assert(RegistryFor<HpmShape>::matches(HpmShape::Spec::kPorts));

// 注册表的一一对应核对: 复制一个绑定忘了改身份、同时漏掉另一个口, 数量对得上、每个绑定
// 也都在清单里, 仍然必须是编译错误(这里用 constexpr 的 false 代替编译失败来检验)。
static_assert(
    !link::PortRegistry<Binding<HpmShape, 0>, Binding<HpmShape, 0>, Binding<HpmShape, 2>>::matches(
        spec::hpm5321::Spec::kPorts));
static_assert(!link::PortRegistry<Binding<HpmShape, 0>, Binding<HpmShape, 2>>::matches(
    spec::hpm5321::Spec::kPorts));

// ---- 假板: 核心注册表 + 板级上下文, 与固件同一份逻辑 ----

// 交接步骤的静态日志(Ownership 的步骤必须是静态函数)。
struct OwnershipStep {
    static std::vector<std::string>& events() {
        static std::vector<std::string> entries;
        return entries;
    }
    static void to_libhcs() { events().push_back("->libhcs"); }
    static void to_tools() { events().push_back("->tools"); }
};

// SETUP/DATA 两段共用的请求视图, 与固件胶水里的那个同形。
struct RequestView {
    std::uint8_t request_;
    std::uint16_t windex_ = 0;
    std::uint16_t wlength_ = 0;
    bool in_ = false;

    [[nodiscard]] std::uint8_t brequest() const { return request_; }
    [[nodiscard]] std::uint16_t windex() const { return windex_; }
    [[nodiscard]] std::uint16_t wlength() const { return wlength_; }
    [[nodiscard]] bool is_in() const { return in_; }
};

struct Transfer {
    vc::Request request;
    std::uint16_t index;

    friend bool operator==(const Transfer&, const Transfer&) = default;
};

void PrintTo(const Transfer& transfer, std::ostream* out) {
    *out << "{0x" << std::hex << static_cast<int>(std::to_underlying(transfer.request)) << ", "
         << std::dec << transfer.index << "}";
}

template <typename Shape>
class FakeBoard {
public:
    using Drivers = typename Shape::Drivers;
    using Registry = RegistryFor<Shape>;
    // 与 hpm 固件同样的交接: 附加功能的一步(这里记账) + 核心的端口一步(真的挂起/恢复)。
    using Ownership = link::Ownership<OwnershipStep, link::PortHandoff<Registry>>;

    FakeBoard() {
        DriversPtr<Shape>::drivers = &drivers_;
        DriversPtr<Shape>::dual_can = true;
    }
    ~FakeBoard() { DriversPtr<Shape>::drivers = nullptr; }

    FakeBoard(const FakeBoard&) = delete;
    FakeBoard& operator=(const FakeBoard&) = delete;

    Drivers& drivers() { return drivers_; }

    // ---- Ep0Channel: 核心的 SETUP/DATA 走到这里 ----

    bool vendor_control_in(
        std::uint8_t request, std::uint16_t index, void* payload, std::size_t size) {
        log.push_back({static_cast<vc::Request>(request), index});
        const RequestView view{request, index, static_cast<std::uint16_t>(size), true};
        const auto decision = ep0::setup<Registry>(*this, view);
        if (decision.action != ep0::SetupAction::kReply || decision.reply_size != size)
            return false; // 长度不齐 = 两端口径不一, STALL
        std::memcpy(payload, reply_.data(), size);
        return true;
    }

    bool vendor_control_out(
        std::uint8_t request, std::uint16_t index, const void* payload, std::size_t size) {
        log.push_back({static_cast<vc::Request>(request), index});
        const RequestView view{request, index, static_cast<std::uint16_t>(size), false};
        const auto decision = ep0::setup<Registry>(*this, view);
        if (decision.action != ep0::SetupAction::kAcceptOut || decision.reply_size != size)
            return false;
        const auto* bytes = static_cast<const std::byte*>(payload);
        staged_.assign(bytes, bytes + size);
        return ep0::data<Registry>(*this, view, std::span{staged_});
    }

    // ---- 板级上下文(核心经它锁存/切换归属) ----

    void* reply_buffer() { return reply_.data(); }

    void record_error(vc::Request request, std::uint16_t index, link::PortOutcome outcome) {
        last_error_ = {
            .request = std::to_underlying(request),
            .reason = std::to_underlying(outcome.reason),
            .index = index,
            .value = outcome.value,
            .reserved = 0};
    }

    void on_manifest_begin() { ownership_.begin_claim(); }

    void on_manifest_accepted(std::uint64_t now) {
        manifest_accepted = true;
        ownership_.commit_claim(now);
    }

    void on_manifest_failed() { ownership_.abort_claim(); }

    // 共享时间基准, 与固件同一个接口。板形可以不报这项能力(模拟 c_board)。
    static constexpr std::uint8_t kBoardCaps = [] {
        if constexpr (requires { Shape::kBoardCaps; })
            return Shape::kBoardCaps;
        else
            return std::uint8_t{vc::kBoardCapTimeSync};
    }();

    [[nodiscard]] bool set_time_sync(bool on) {
        if (on && refuse_time_sync)
            return false;
        time_sync_running = on;
        return true;
    }

    static std::uint64_t now() { return clock_; }

    bool read_latency(bool) { return false; }

    ep0::ManifestJournal& journal() { return journal_; }

    void fill_last_error(vc::LastConfigErrorPayload& out) const { out = last_error_; }

    [[nodiscard]] const vc::LastConfigErrorPayload& last_error() const { return last_error_; }

    // ---- 测试侧 ----

    // 会话结束(租约到期/挂起/拔线): 板子交还附加功能, 交接步骤把各口恢复成附加功能
    // 要的状态(单 CAN 形的第二个槽位不在, 保持不动)。
    void end_session() {
        manifest_accepted = false;
        ownership_.release();
    }

    Ownership& ownership() { return ownership_; }

    std::vector<Transfer> log;
    bool manifest_accepted = false;
    bool time_sync_running = false;
    bool refuse_time_sync = false; // 测试侧: 让时间基准开不起来
    static inline std::uint64_t clock_ = 0;

private:
    Drivers drivers_{};
    ep0::ManifestJournal journal_{};
    vc::LastConfigErrorPayload last_error_{};
    Ownership ownership_{4'000};
    std::array<std::byte, 128> reply_{}; // 最大的 IN 应答是清单结果(116 字节)
    std::vector<std::byte> staged_;
};
static_assert(hcs::Ep0Channel<FakeBoard<Mc02Shape>>);

// 两种板形, 对应旧测试里的 mc02_like() / hpm5321_like()。
using Mc02Like = FakeBoard<Mc02Shape>;
using HpmLike = FakeBoard<HpmShape>;

// 没有时间基准的板(c_board 那样): 口与 hpm 形相同, 只是不报这项能力。
struct NoTimeBaseShape : HpmShape {
    static constexpr std::uint8_t kBoardCaps = 0;
};
using NoTimeBaseLike = FakeBoard<NoTimeBaseShape>;

// 板子类(Board<Spec>)构造时做的那件事。
template <typename Shape>
hcs::Interface declare(FakeBoard<Shape>& board, const hcs::Configuration& configuration) {
    return hcs::apply(board, configuration);
}

std::string apply_error(auto& board, const hcs::Configuration& configuration) {
    try {
        (void)declare(board, configuration);
    } catch (const std::exception& error) {
        return error.what();
    }
    return {};
}

// 每个口跑没跑, 按清单顺序; 便于整板断言。
std::vector<bool> running(const Mc02Drivers& drivers) {
    std::vector<bool> state;
    for (const auto& bus : drivers.can)
        state.push_back(bus.is_running);
    state.push_back(drivers.dbus.is_running);
    for (const auto& port : drivers.others)
        state.push_back(port.is_running);
    state.push_back(drivers.imu.is_running);
    return state;
}

TEST(Ep0Declaration, OnlyTheConfiguredPortsAreDeclared) {
    Mc02Like board;
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan2, hcs::kClassic1M);
    configuration.declare_uart(DataId::kUart7, hcs::uart_8n1(115'200));

    const hcs::Interface interface = declare(board, configuration);

    // 一份口清单, 一次清单, 一次逐口结果: 没有任何逐口的 SET。
    EXPECT_EQ(
        board.log, (std::vector<Transfer>{
                       {      vc::Request::kGetPortList, 0},
                       {    vc::Request::kApplyManifest, 0},
                       {vc::Request::kGetManifestResult, 0},
    }));
    EXPECT_TRUE(board.drivers().can[1].is_running);
    EXPECT_FALSE(board.drivers().can[0].is_running) << "an undeclared bus stays off";
    EXPECT_TRUE(board.drivers().uart(4).is_running); // UART7
    EXPECT_FALSE(board.drivers().uart(0).is_running) << "DBUS is not declared";
    EXPECT_FALSE(board.drivers().imu.is_running);
    // 主机自己的记录与板子的状态一致, 口对口。
    EXPECT_TRUE(interface.declared(DataId::kCan2));
    EXPECT_TRUE(interface.declared(DataId::kUart7));
    EXPECT_FALSE(interface.declared(DataId::kCan1));
    EXPECT_FALSE(interface.declared(DataId::kUartDbus));
    EXPECT_FALSE(interface.declared(DataId::kImu));
}

TEST(Ep0Declaration, APortStartsWithTheDeclaredSetting) {
    Mc02Like board;
    ASSERT_TRUE(board.drivers().can[0].fd); // 上电即 CAN-FD
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kClassic1M);
    configuration.declare_uart(
        DataId::kUartDbus, hcs::UartSetting{
                               .baudrate = 100'000,
                               .word_length = vc::kUartWordLength8,
                               .parity = vc::kUartParityEven,
                               .stop_bits = vc::kUartStopBits1,
                               .rx_polarity = vc::kUartRxPolarityNormal});

    const hcs::Interface interface = declare(board, configuration);

    EXPECT_TRUE(board.drivers().can[0].is_running);
    EXPECT_FALSE(board.drivers().can[0].fd);
    EXPECT_FALSE(interface.can_fd(DataId::kCan1)) << "the mirror follows the declared mode";
    EXPECT_TRUE(interface.can_fd(DataId::kCan2))
        << "an undeclared bus still reports what it would run";
    EXPECT_TRUE(board.drivers().uart(0).is_running);
    EXPECT_EQ(board.drivers().uart(0).baudrate, 100'000U);
}

TEST(Ep0Declaration, AnEmptyConfigurationDeclaresNothing) {
    Mc02Like board;
    // 板子在附加功能手里时口是开的(hpm 的策略); 一份空清单 = "这些口都不归你用"。
    board.drivers().can[0].is_running = true;
    board.drivers().uart(0).is_running = true;

    (void)declare(board, {});

    EXPECT_EQ(
        board.log, (std::vector<Transfer>{
                       {      vc::Request::kGetPortList, 0},
                       {    vc::Request::kApplyManifest, 0},
                       {vc::Request::kGetManifestResult, 0},
    }));
    EXPECT_EQ(running(board.drivers()), std::vector<bool>(3 + 6 + 1, false));
}

TEST(Ep0Declaration, ANewDeclarationReplacesTheOldOne) {
    HpmLike board;
    hcs::Configuration both;
    both.declare_can(DataId::kCan1, hcs::kClassic1M);
    both.declare_can(DataId::kCan2, hcs::kClassic1M);
    (void)declare(board, both);
    ASSERT_TRUE(board.drivers().can[1].is_running);

    hcs::Configuration first_only;
    first_only.declare_can(DataId::kCan1, hcs::kClassic1M);
    const hcs::Interface interface = declare(board, first_only);
    EXPECT_TRUE(board.drivers().can[0].is_running);
    EXPECT_FALSE(board.drivers().can[1].is_running)
        << "the old round's declaration did not survive";
    EXPECT_TRUE(interface.declared(DataId::kCan1));
    EXPECT_FALSE(interface.declared(DataId::kCan2));
}

// A host that reconnects with a changed wiring, or one that redeclares at run
// time: the stored configuration carries across, the replay starts everything
// declared, and the host's record follows what the board accepted.
TEST(Ep0Declaration, ARunTimeRedeclareDeclaresThePortAndIsReplayed) {
    Mc02Like board;
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kFd1M5M);
    std::atomic<hcs::Interface> interface{declare(board, configuration)};
    ASSERT_FALSE(board.drivers().uart(2).is_running);
    ASSERT_FALSE(interface.load().declared(DataId::kUart2));
    ASSERT_FALSE(interface.load().declared(DataId::kCan2));

    hcs::reconfigure_uart(board, configuration, interface, DataId::kUart2, hcs::uart_8n1(921'600));
    EXPECT_TRUE(board.drivers().uart(2).is_running);
    EXPECT_TRUE(interface.load().declared(DataId::kUart2));
    ASSERT_TRUE(configuration.find(DataId::kUart2) != nullptr);
    EXPECT_EQ(configuration.find(DataId::kUart2)->uart.baudrate, 921'600U);

    hcs::reconfigure_can(board, configuration, interface, DataId::kCan2, hcs::kFd1M5M);
    EXPECT_TRUE(board.drivers().can[1].is_running);
    EXPECT_TRUE(interface.load().declared(DataId::kCan2));

    board.end_session();
    ASSERT_EQ(running(board.drivers()), std::vector<bool>(3 + 6 + 1, true))
        << "the board is back in the extras' hands: ports are open again";

    const hcs::Interface replayed = declare(board, configuration);
    EXPECT_TRUE(board.drivers().can[0].is_running);
    EXPECT_TRUE(board.drivers().can[1].is_running);
    EXPECT_TRUE(board.drivers().uart(2).is_running);
    EXPECT_EQ(board.drivers().uart(2).baudrate, 921'600U);
    EXPECT_TRUE(replayed.declared(DataId::kCan1));
    EXPECT_TRUE(replayed.declared(DataId::kCan2));
    EXPECT_TRUE(replayed.declared(DataId::kUart2));
}

// The transmit gate: a declared port passes; an undeclared one throws a type of
// its own, naming the port as the enclosure prints it.
TEST(Ep0Declaration, TransmittingOnAnUndeclaredPortThrows) {
    Mc02Like board;
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kFd1M5M);
    configuration.declare_uart(DataId::kUart7, hcs::uart_8n1(115'200));
    const hcs::Interface interface = declare(board, configuration);

    EXPECT_NO_THROW(hcs::require_declared(interface, DataId::kCan1));
    EXPECT_NO_THROW(hcs::require_declared(interface, DataId::kUart7));

    try {
        hcs::require_declared(interface, DataId::kCan2);
        ADD_FAILURE() << "an undeclared bus passed the gate";
    } catch (const hcs::UndeclaredChannel& error) {
        EXPECT_TRUE(mentions(error.what(), "CAN2 is not declared"));
    }
    try {
        hcs::require_declared(interface, DataId::kUartDbus);
        ADD_FAILURE() << "an undeclared port passed the gate";
    } catch (const hcs::UndeclaredChannel& error) {
        EXPECT_TRUE(mentions(error.what(), "DBUS is not declared"));
    }
    // A caller that handles programming errors as a class still catches it.
    EXPECT_THROW(hcs::require_declared(interface, DataId::kCan3), std::logic_error);
}

// Nothing is declared until the board accepts a manifest, and a refused one does
// not count: the record follows what the board accepted, not what the host asked.
TEST(Ep0Declaration, OnlyAnAcceptedManifestCountsAsDeclared) {
    EXPECT_FALSE(hcs::Interface{}.declared(DataId::kCan1))
        << "a board object before its declaration";

    Mc02Like board;
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kFd1M5M);
    std::atomic<hcs::Interface> interface{declare(board, configuration)};

    EXPECT_THROW(
        hcs::reconfigure_uart(board, configuration, interface, DataId::kUart1, hcs::uart_8n1(123)),
        std::runtime_error);
    EXPECT_FALSE(interface.load().declared(DataId::kUart1));
    EXPECT_FALSE(configuration.find(DataId::kUart1) != nullptr)
        << "a refused setting must not be replayed";

    // A port the board does not have is refused against the port list, before a
    // manifest is sent: the only transfer is the (pure) list read.
    const auto transfers = board.log.size();
    EXPECT_THROW(
        hcs::reconfigure_uart(board, configuration, interface, DataId::kUart0, hcs::UartSetting{}),
        std::runtime_error);
    ASSERT_EQ(board.log.size(), transfers + 1);
    EXPECT_EQ(board.log.back().request, vc::Request::kGetPortList);
}

// On a board that sets rates, the host sends them as the setting; on one that
// fixes them, as an assertion the board must agree with.
TEST(Ep0Declaration, RatesAreSetWhereTheBoardAllowsAndAssertedWhereItDoesNot) {
    HpmLike board;
    hcs::Configuration slow;
    slow.declare_can(DataId::kCan1, hcs::CanSetting{.fd = false, .arbitration_baudrate = 500'000});
    (void)declare(board, slow);
    EXPECT_EQ(board.drivers().can[0].setting_value.arbitration_baudrate, 500'000U);
    EXPECT_TRUE(board.drivers().can[0].is_running);

    Mc02Like fixed;
    EXPECT_TRUE(mentions(apply_error(fixed, slow), "rate unrepresentable"));
    EXPECT_FALSE(fixed.drivers().can[0].is_running) << "a refused declaration leaves the bus off";
    EXPECT_EQ(fixed.drivers().can[0].setting_value.arbitration_baudrate, 1'000'000U);
}

// A refusal is never silent: the constructor throws, naming the port and the
// reason the board latched -- and nothing at all takes effect.
TEST(Ep0Declaration, ARefusedPortThrowsWithTheBoardsReason) {
    Mc02Like board;
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kFd1M5M);
    configuration.declare_uart(DataId::kUart1, hcs::uart_8n1(123));

    const std::string error = apply_error(board, configuration);
    EXPECT_TRUE(mentions(error, "UART1"));
    EXPECT_TRUE(mentions(error, "123"));
    EXPECT_TRUE(mentions(error, "rate unrepresentable"));
    EXPECT_FALSE(board.drivers().uart(1).is_running);
    EXPECT_EQ(board.drivers().uart(1).baudrate, 115'200U) << "a refused request changes nothing";
    EXPECT_FALSE(board.drivers().can[0].is_running)
        << "the two-phase commit started nothing: the whole manifest failed";
    EXPECT_FALSE(board.manifest_accepted) << "ownership does not move on a refused declaration";
}

TEST(Ep0Declaration, DeclaringAPortTheBoardLacksThrows) {
    HpmLike board;
    hcs::Configuration third_bus;
    third_bus.declare_can(DataId::kCan3, hcs::kClassic1M);
    EXPECT_TRUE(mentions(apply_error(board, third_bus), "wiring uses CAN3"));
    EXPECT_TRUE(mentions(apply_error(board, third_bus), "this board has no CAN3"));

    hcs::Configuration second_port;
    second_port.declare_uart(DataId::kUart1, hcs::uart_8n1(115'200));
    EXPECT_TRUE(mentions(apply_error(board, second_port), "wiring uses UART1"));
}

// 单 CAN 的 hpm5321: 第二路 CAN 不在本 PCB 上。主机从口清单就知道, 清单一封不发。
TEST(Ep0Declaration, TheSingleCanPcbRefusesItsSecondBusWithoutAManifest) {
    HpmLike board;
    DriversPtr<HpmShape>::dual_can = false;
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kClassic1M);
    configuration.declare_can(DataId::kCan2, hcs::kClassic1M);

    EXPECT_TRUE(mentions(apply_error(board, configuration), "this board has no CAN2"));
    EXPECT_EQ(
        board.log, (std::vector<Transfer>{
                       {vc::Request::kGetPortList, 0}
    }))
        << "nothing is sent to a board whose port list already refuses the wiring";
}

// The on-board IMU is a port like the others: it runs only when the
// Configuration names it, with exactly the ranges and rates the host stated.
TEST(Ep0Declaration, TheImuRunsOnlyWhenDeclaredAndAsDeclared) {
    Mc02Like board;
    hcs::Configuration without;
    without.declare_can(DataId::kCan1, hcs::kFd1M5M);
    (void)declare(board, without);
    EXPECT_FALSE(board.drivers().imu.is_running) << "an IMU nobody declared must stay off";

    hcs::Configuration with = without;
    with.declare_imu(kBmi088);
    board.log.clear();
    (void)declare(board, with);
    EXPECT_TRUE(board.drivers().imu.is_running);
    EXPECT_EQ(board.drivers().imu.setting.accelerometer_range_g, 6);
    EXPECT_EQ(board.drivers().imu.setting.accelerometer_rate_hz, 1600);
    EXPECT_EQ(board.drivers().imu.setting.gyroscope_range_dps, 2000);
    EXPECT_EQ(board.drivers().imu.setting.gyroscope_rate_hz, 2000);
    EXPECT_EQ(board.drivers().imu.setting.gyroscope_bandwidth_hz, 230);

    // The next round does not inherit it.
    (void)declare(board, without);
    EXPECT_FALSE(board.drivers().imu.is_running);
}

// Declaring an IMU the board does not have, or a range the sensor does not
// offer, throws at construction instead of leaving the host scaling samples
// with numbers the board never took.
TEST(Ep0Declaration, AnImuTheBoardCannotRunAsDeclaredThrows) {
    HpmLike no_imu;
    hcs::Configuration configuration;
    configuration.declare_imu(kBmi088);
    EXPECT_TRUE(mentions(apply_error(no_imu, configuration), "this board has no"));
    EXPECT_EQ(
        no_imu.log, (std::vector<Transfer>{
                        {vc::Request::kGetPortList, 0}
    }))
        << "nothing is sent to a board that said it has no IMU";

    Mc02Like board;
    hcs::Configuration bad_range;
    hcs::ImuSetting five_g = kBmi088;
    five_g.accelerometer_range_g = 5;
    bad_range.declare_imu(five_g);
    const std::string error = apply_error(board, bad_range);
    EXPECT_TRUE(mentions(error, "the on-board IMU"));
    EXPECT_TRUE(mentions(error, "rate unrepresentable"));
    EXPECT_FALSE(board.drivers().imu.is_running);

    // 芯片不配合: 校验全过, 应用阶段的回读失败 -- kConfigErrorVerifyFailed。板子已归
    // libhcs 时整体回滚 = 所有口(含 IMU)挂起。
    hcs::Configuration can_only;
    can_only.declare_can(DataId::kCan1, hcs::kFd1M5M);
    (void)declare(board, can_only);
    board.drivers().imu.sensor_fault = vc::ConfigErrorReason::kConfigErrorVerifyFailed;
    hcs::Configuration good = can_only;
    good.declare_imu(kBmi088);
    EXPECT_TRUE(mentions(apply_error(board, good), "read-back disagrees"));
    EXPECT_FALSE(board.drivers().imu.is_running);
    EXPECT_FALSE(board.drivers().can[0].is_running);
}

// Mixed versions stop at the first transfer: nothing is configured on a board
// whose wire format this SDK may misread.
TEST(Ep0Declaration, AVersionMismatchIsRefusedBeforeAnythingIsConfigured) {
    Mc02Like board;
    // 指纹算在编译期, 测试里改不了 -- 直接验证"版本不符的清单被拒"。一个 v9 主机的
    // 逐口 SET 在这块板上落进 default 分支(请求已退役), 同样是第一笔传输就被拒。
    std::array<std::byte, 4> stale{};
    stale[0] = std::byte{0x12};
    stale[1] = std::byte{0x34};
    stale[2] = std::byte{1};
    stale[3] = std::byte{0};
    EXPECT_FALSE(board.vendor_control_out(
        std::to_underlying(vc::Request::kApplyManifest), 0, stale.data(), stale.size()));
    EXPECT_EQ(
        board.last_error().reason,
        std::to_underlying(vc::ConfigErrorReason::kConfigErrorBadRequest));
    EXPECT_FALSE(board.drivers().can[0].is_running);
    EXPECT_FALSE(board.manifest_accepted);
}

// ---- GPIO 口: 一组线, 与 CAN/UART 同一套声明 ----

using Pwm = spec::mc02::Spec::Gpios;

// 一根线一项声明: 声明了才用, 方向是声明的一部分, 没声明的线不动。主机的记录(声明了
// 哪些线、哪些是输出)与板子一致, 读回的是线实际在跑的设置。整个排针在口清单里只有一行,
// 能力字节是线数。
TEST(Ep0Declaration, GpioLinesRunOnlyAsDeclared) {
    Mc02Like board;
    hcs::Configuration configuration;
    configuration.declare_gpio(Pwm::kPwm1, hcs::kGpioOutput);
    const hcs::GpioSetting input{
        .mode = vc::kGpioModeInput,
        .pull = vc::kGpioPullUp,
        .input_flags = vc::kGpioInputRisingEdge | vc::kGpioInputTimestamp,
        .period_ms = 10};
    configuration.declare_gpio(Pwm::kPwm3, input);

    const hcs::Interface interface = declare(board, configuration);

    EXPECT_TRUE(board.drivers().gpio(0).is_running);
    EXPECT_EQ(board.drivers().gpio(0).setting.mode, vc::kGpioModeOutput);
    EXPECT_TRUE(board.drivers().gpio(2).is_running);
    EXPECT_EQ(board.drivers().gpio(2).setting.period_ms, 10U);
    EXPECT_FALSE(board.drivers().gpio(1).is_running) << "an undeclared line is not touched";
    EXPECT_EQ(board.drivers().gpio(1).writes, 0U);
    EXPECT_FALSE(board.drivers().gpio(3).is_running);

    EXPECT_TRUE(interface.declared(DataId::kGpio));
    EXPECT_TRUE(interface.line_declared(0));
    EXPECT_TRUE(interface.output(0));
    EXPECT_TRUE(interface.line_declared(2));
    EXPECT_FALSE(interface.output(2));
    EXPECT_FALSE(interface.line_declared(1));

    EXPECT_EQ(hcs::read_gpio_setting(board, Pwm::kPwm3.line), input);
    EXPECT_EQ(hcs::read_gpio_setting(board, Pwm::kPwm2.line).mode, vc::kGpioModeOff);
    EXPECT_THROW((void)hcs::read_gpio_setting(board, 4), std::runtime_error) << "mc02 has 4 lines";

    const hcs::PortList list = hcs::read_port_list(board);
    const auto* gpio = list.find(DataId::kGpio);
    ASSERT_NE(gpio, nullptr);
    EXPECT_EQ(gpio->kind, spec::PortKind::kGpio);
    EXPECT_EQ(gpio->capabilities, 4U);
}

// 线号只属于 GPIO 口: 别的口的 wIndex 高字节不是 0 即"没有这个口"。
TEST(Ep0Declaration, ALineNumberOnAPortThatHasNoLinesIsNoPort) {
    Mc02Like board;
    vc::UartConfigPayload payload{};
    EXPECT_FALSE(board.vendor_control_in(
        std::to_underlying(vc::Request::kGetPortConfig),
        static_cast<std::uint16_t>(std::to_underlying(DataId::kUart7) | (1U << 8U)), &payload,
        sizeof(payload)));
    EXPECT_EQ(board.last_error().reason, vc::ConfigErrorReason::kConfigErrorBadIndex);
}

// 线做不到的采样在校验阶段就被拒, 报错带线名与缺的能力, 什么都不动。板上没有的线
// 在主机侧就被拒。
TEST(Ep0Declaration, ASamplingTheLineCannotDoIsRefusedBeforeAnythingIsTouched) {
    Mc02Like board;
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kFd1M5M);
    configuration.declare_gpio(
        Pwm::kPwm4,
        hcs::GpioSetting{.mode = vc::kGpioModeInput, .input_flags = vc::kGpioInputFallingEdge});

    const std::string error = apply_error(board, configuration);
    EXPECT_TRUE(mentions(error, "PWM4"));
    EXPECT_TRUE(mentions(error, "mode unsupported"));
    EXPECT_TRUE(mentions(error, std::to_string(spec::kGpioCapReadInterrupt)));
    EXPECT_FALSE(board.drivers().can[0].is_running) << "a refused manifest starts nothing";
    EXPECT_EQ(board.drivers().gpio(3).writes, 0U);

    // 输出上带采样字段, 是主机把两种声明弄混了(板子拒绝); "声明但关着"不是声明
    // (主机在发出前就拒绝)。
    hcs::Configuration pulled_output;
    pulled_output.declare_gpio(
        Pwm::kPwm1, hcs::GpioSetting{.mode = vc::kGpioModeOutput, .pull = vc::kGpioPullDown});
    EXPECT_TRUE(mentions(apply_error(board, pulled_output), "bad request"));
    hcs::Configuration off;
    off.declare_gpio(Pwm::kPwm1, hcs::GpioSetting{.mode = vc::kGpioModeOff});
    EXPECT_TRUE(mentions(apply_error(board, off), "missing: mode"));
    EXPECT_EQ(board.drivers().gpio(0).writes, 0U);

    hcs::Configuration fifth_line;
    fifth_line.declare_gpio(4, hcs::kGpioOutput);
    EXPECT_TRUE(mentions(apply_error(board, fifth_line), "PWM5"));
    EXPECT_TRUE(mentions(apply_error(board, fifth_line), "has 4 lines"));

    // 同一份采样, 有中断能力的线接受。
    hcs::Configuration fine;
    fine.declare_gpio(
        Pwm::kPwm1,
        hcs::GpioSetting{.mode = vc::kGpioModeInput, .input_flags = vc::kGpioInputFallingEdge});
    EXPECT_NO_THROW((void)declare(board, fine));
}

// 蜂鸣器: 声明不带参数, 声明了才启动(静音), 不声明不碰; 读回的是在放的音。声明里塞了
// 参数的(GET 才有的字段)被板子拒绝。
TEST(Ep0Declaration, TheBuzzerStartsOnlyWhenDeclared) {
    Mc02Like board;
    hcs::Configuration without;
    without.declare_can(DataId::kCan1, hcs::kFd1M5M);
    const hcs::Interface quiet = declare(board, without);
    EXPECT_FALSE(board.drivers().buzzer.started) << "an undeclared buzzer is not touched";
    EXPECT_FALSE(quiet.declared(DataId::kBuzzer));

    hcs::Configuration with = without;
    with.declare_buzzer();
    const hcs::Interface interface = declare(board, with);
    EXPECT_TRUE(board.drivers().buzzer.started);
    EXPECT_TRUE(interface.declared(DataId::kBuzzer));
    board.drivers().buzzer.frequency_hz = 2093;
    board.drivers().buzzer.loudness = 128;
    const auto state = hcs::read_buzzer_state(board);
    EXPECT_EQ(state.frequency_hz, 2093U);
    EXPECT_EQ(state.loudness, 128U);

    (void)declare(board, without);
    EXPECT_FALSE(board.drivers().buzzer.is_running) << "a buzzer left out of the manifest";
    EXPECT_EQ(board.drivers().buzzer.frequency_hz, 0U) << "a suspended buzzer is silent";

    vc::ManifestPayload manifest{};
    manifest.version = vc::kVersion;
    manifest.entry_count = 1;
    manifest.entries[0].data_id = std::to_underlying(DataId::kBuzzer);
    manifest.entries[0].kind = std::to_underlying(spec::PortKind::kBuzzer);
    manifest.entries[0].setting[0] = 1; // a frequency in a declaration
    EXPECT_FALSE(board.vendor_control_out(
        std::to_underlying(vc::Request::kApplyManifest), 0, &manifest,
        offsetof(vc::ManifestPayload, entries) + sizeof(vc::ManifestEntry)));
    EXPECT_EQ(board.last_error().reason, vc::ConfigErrorReason::kConfigErrorBadRequest);
}

// 运行期改别的口会重放整份清单; 设置没变的线不被重写, 正在输出的 PWM 不掉到 0。
// 没进清单的线(或整板换手)才回到低电平。
TEST(Ep0Declaration, ReplayingTheManifestKeepsARunningOutput) {
    Mc02Like board;
    hcs::Configuration configuration;
    configuration.declare_gpio(Pwm::kPwm1, hcs::kGpioOutput);
    std::atomic<hcs::Interface> interface{declare(board, configuration)};
    board.drivers().gpio(0).output = 1234; // 主机写过的占空比
    const auto writes = board.drivers().gpio(0).writes;

    hcs::reconfigure_uart(board, configuration, interface, DataId::kUart7, hcs::uart_8n1(115'200));
    EXPECT_EQ(board.drivers().gpio(0).output, 1234U);
    EXPECT_EQ(board.drivers().gpio(0).writes, writes);
    EXPECT_TRUE(board.drivers().gpio(0).is_running);

    hcs::reconfigure_gpio(
        board, configuration, interface, Pwm::kPwm1.line,
        hcs::GpioSetting{.mode = vc::kGpioModeInput, .period_ms = 5});
    EXPECT_EQ(board.drivers().gpio(0).output, 0U) << "a line that changes direction starts over";
    EXPECT_FALSE(interface.load().output(Pwm::kPwm1.line));

    hcs::Configuration without;
    (void)declare(board, without);
    EXPECT_FALSE(board.drivers().gpio(0).is_running);
}

// 串口同一规矩: 重放时速率与帧格式都没变的口不重写(写分频器/帧格式要停口, 收发中的字节
// 会丢), 变了的口与接手时挂起过的口照常重写。
TEST(Ep0Declaration, ReplayingTheManifestLeavesAnUnchangedUartAlone) {
    Mc02Like board;
    hcs::Configuration configuration;
    configuration.declare_uart(DataId::kUart7, hcs::uart_8n1(921'600));
    std::atomic<hcs::Interface> interface{declare(board, configuration)};
    FakeUart& uart7 = board.drivers().uart(4);
    ASSERT_TRUE(uart7.is_running);
    const auto writes = uart7.writes;
    ASSERT_GT(writes, 0U);

    hcs::reconfigure_uart(board, configuration, interface, DataId::kUart1, hcs::uart_8n1(115'200));
    EXPECT_EQ(uart7.writes, writes) << "replaying UART7's unchanged setting rewrote it";
    EXPECT_TRUE(uart7.is_running);
    EXPECT_TRUE(board.drivers().uart(1).is_running);

    hcs::reconfigure_uart(
        board, configuration, interface, DataId::kUart7,
        hcs::UartSetting{
            .baudrate = 921'600,
            .word_length = vc::kUartWordLength8,
            .parity = vc::kUartParityEven,
            .stop_bits = vc::kUartStopBits1,
            .rx_polarity = vc::kUartRxPolarityNormal});
    EXPECT_GT(uart7.writes, writes) << "a changed framing was not written";
    EXPECT_TRUE(uart7.framing_matches(8, vc::kUartParityEven, 1, 1));

    // 会话结束后板子归附加功能; 再声明即接手, 端口先全部挂起, 声明的口从头写一遍。
    board.end_session();
    const auto before_claim = uart7.writes;
    (void)declare(board, configuration);
    EXPECT_GT(uart7.writes, before_claim) << "a port taken over from the extras was not rewritten";
}

// 发送门的 GPIO 半边: 写要声明成输出的线, 读请求要声明成输入的线。
TEST(Ep0Declaration, UsingALineAgainstItsDeclaredDirectionThrows) {
    Mc02Like board;
    hcs::Configuration configuration;
    configuration.declare_gpio(Pwm::kPwm1, hcs::kGpioOutput);
    configuration.declare_gpio(Pwm::kPwm2, hcs::GpioSetting{.mode = vc::kGpioModeInput});
    const hcs::Interface interface = declare(board, configuration);

    EXPECT_NO_THROW(hcs::require_gpio_direction(interface, Pwm::kPwm1.line, true));
    EXPECT_NO_THROW(hcs::require_gpio_direction(interface, Pwm::kPwm2.line, false));
    try {
        hcs::require_gpio_direction(interface, Pwm::kPwm2.line, true);
        ADD_FAILURE() << "a write to an input passed the gate";
    } catch (const hcs::UndeclaredChannel& error) {
        EXPECT_TRUE(mentions(error.what(), "PWM2 is declared as an input"));
    }
    EXPECT_THROW(
        hcs::require_gpio_direction(interface, Pwm::kPwm1.line, false), hcs::UndeclaredChannel);
    try {
        hcs::require_gpio_direction(interface, Pwm::kPwm3.line, true);
        ADD_FAILURE() << "an undeclared line passed the gate";
    } catch (const hcs::UndeclaredChannel& error) {
        EXPECT_TRUE(mentions(error.what(), "PWM3 is not declared"));
    }
}

// ---- 声明是完整的 ----
//
// 一个口怎么工作由声明说全, 没有"沿用固件当前"。主机在发出前按字段名拒绝; 绕过主机
// 板卡类直接发清单的, 板子同样拒绝, 指出缺的是哪个字段(载荷里的偏移), 什么都不动。
TEST(Ep0Declaration, ADeclarationThatLeavesASettingUnsetIsRefused) {
    Mc02Like board;
    hcs::Configuration partial;
    partial.declare_uart(DataId::kUart7, hcs::UartSetting{.baudrate = 115'200});
    std::string error = apply_error(board, partial);
    EXPECT_TRUE(mentions(error, "UART7"));
    EXPECT_TRUE(mentions(error, "word_length, parity, stop_bits, rx_polarity"));
    EXPECT_EQ(board.log.size(), 1U) << "refused after the port list, before any manifest";

    hcs::Configuration no_rate;
    no_rate.declare_can(
        DataId::kCan1, hcs::CanSetting{.fd = true, .arbitration_baudrate = 1'000'000});
    EXPECT_TRUE(mentions(apply_error(board, no_rate), "missing: data_baudrate"));

    hcs::Configuration half_imu;
    half_imu.declare_imu(hcs::ImuSetting{.accelerometer_range_g = 6});
    EXPECT_TRUE(mentions(apply_error(board, half_imu), "accelerometer_rate_hz"));

    // 不经主机的检查: 一份帧格式缺校验位的 UART 清单。
    vc::ManifestPayload manifest{};
    manifest.version = vc::kVersion;
    manifest.entry_count = 1;
    manifest.entries[0].data_id = std::to_underlying(DataId::kUart7);
    manifest.entries[0].kind = std::to_underlying(spec::PortKind::kUart);
    vc::UartConfigPayload uart{};
    uart.baudrate = 115'200;
    uart.word_length = vc::kUartWordLength8;
    uart.stop_bits = vc::kUartStopBits1;
    uart.rx_polarity = vc::kUartRxPolarityNormal;
    uart.control = vc::kUartConfigApply;
    std::memcpy(manifest.entries[0].setting, &uart, sizeof(uart));
    EXPECT_FALSE(board.vendor_control_out(
        std::to_underlying(vc::Request::kApplyManifest), 0, &manifest,
        offsetof(vc::ManifestPayload, entries) + sizeof(vc::ManifestEntry)));
    EXPECT_EQ(board.last_error().reason, vc::ConfigErrorReason::kConfigErrorIncomplete);
    EXPECT_EQ(board.last_error().value, offsetof(vc::UartConfigPayload, parity));
    EXPECT_FALSE(board.drivers().uart(4).is_running);

    // 只作断言的字段仍是可选的: 不给 UART 分频器, 声明照样成立。
    hcs::Configuration whole;
    whole.declare_uart(DataId::kUart7, hcs::uart_8n1(115'200));
    EXPECT_NO_THROW((void)declare(board, whole));
}

// ---- 共享时间基准: 声明的一部分, 不是口 ----

// 板子在口清单里报它能做, 主机在清单头里要它; 要了才开, 下一份没要它的清单把它关掉。
TEST(Ep0TimeSync, RunsExactlyWhileADeclarationAsksForIt) {
    Mc02Like board;
    EXPECT_NE(hcs::read_port_list(board).board_caps & vc::kBoardCapTimeSync, 0U);

    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kFd1M5M);
    (void)declare(board, configuration);
    EXPECT_FALSE(board.time_sync_running) << "nobody asked for it";

    configuration.enable_time_sync();
    (void)declare(board, configuration);
    EXPECT_TRUE(board.time_sync_running);
    EXPECT_TRUE(board.drivers().can[0].is_running);

    // 运行期改一个口会重放整份清单: 时间基准跟着声明走, 不掉。
    std::atomic<hcs::Interface> interface{hcs::Interface{}};
    hcs::reconfigure_uart(board, configuration, interface, DataId::kUart7, hcs::uart_8n1(115'200));
    EXPECT_TRUE(board.time_sync_running) << "a replayed declaration dropped the time base";

    configuration.enable_time_sync(false);
    (void)declare(board, configuration);
    EXPECT_FALSE(board.time_sync_running) << "a declaration without it left it running";
}

// 板子没报这项能力: 主机在发清单之前就拒, 报错说清是哪一项, 板上什么都不动。
TEST(Ep0TimeSync, IsRefusedBeforeAnythingIsSentToABoardWithoutOne) {
    NoTimeBaseLike board;
    EXPECT_EQ(hcs::read_port_list(board).board_caps, 0U);
    board.log.clear();

    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kFd1M5M);
    configuration.enable_time_sync();
    EXPECT_NE(apply_error(board, configuration).find("time_sync"), std::string::npos);
    EXPECT_FALSE(std::ranges::any_of(board.log, [](const Transfer& transfer) {
        return transfer.request == vc::Request::kApplyManifest;
    })) << "the manifest went out although the board reports no time base";
    EXPECT_FALSE(board.manifest_accepted);
}

// 绕过主机检查直接发: 板子自己也拒(兜底), 不认识的整板请求位同样拒。
TEST(Ep0TimeSync, TheBoardRefusesAFlagItCannotHonour) {
    const auto send = [](auto& board, std::uint8_t flags) {
        vc::ManifestPayload manifest{};
        manifest.version = vc::kVersion;
        manifest.flags = flags;
        return board.vendor_control_out(
            std::to_underlying(vc::Request::kApplyManifest), 0, &manifest,
            offsetof(vc::ManifestPayload, entries));
    };

    NoTimeBaseLike without;
    EXPECT_FALSE(send(without, vc::kManifestFlagTimeSync));
    EXPECT_EQ(without.last_error().reason, vc::ConfigErrorReason::kConfigErrorBadRequest);
    EXPECT_FALSE(without.time_sync_running);
    EXPECT_FALSE(without.manifest_accepted);

    Mc02Like with;
    EXPECT_FALSE(send(with, 0x80)) << "an unknown board-wide request bit";
    EXPECT_EQ(with.last_error().reason, vc::ConfigErrorReason::kConfigErrorBadRequest);
    EXPECT_TRUE(send(with, vc::kManifestFlagTimeSync));
    EXPECT_TRUE(with.time_sync_running);
}

// 时间基准开不起来, 与某个口应用失败同一结局: 整份清单回滚, 板子还给接手前的主人(附加
// 功能的口原样恢复), 不放行会话。它在端口之前开, 所以没有哪个口按这份声明上过线。
TEST(Ep0TimeSync, FailingToStartItRollsTheWholeDeclarationBack) {
    HpmLike board;
    board.refuse_time_sync = true;
    OwnershipStep::events().clear();
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kClassic1M); // 上电是 FD: 看得出有没有被应用
    configuration.enable_time_sync();
    EXPECT_FALSE(apply_error(board, configuration).empty());
    EXPECT_FALSE(board.time_sync_running);
    EXPECT_FALSE(board.manifest_accepted);
    EXPECT_EQ(board.ownership().owner(), link::Owner::kTools);
    EXPECT_EQ(OwnershipStep::events(), (std::vector<std::string>{"->libhcs", "->tools"}));
    EXPECT_TRUE(board.drivers().can[0].fd) << "the declared setting reached a port";

    vc::ManifestResultPayload result{};
    ASSERT_TRUE(board.vendor_control_in(
        std::to_underlying(vc::Request::kGetManifestResult), 0, &result, sizeof(result)));
    EXPECT_EQ(result.outcome, std::to_underlying(vc::ManifestOutcome::kManifestRolledBack));
}

// kVersion is computed from the wire format, so it moves by itself. This pin
// is the moment to stop and act on that: a new value means every board must be
// reflashed together with the host. Update the number below only after that is
// planned, in the same change that moved it.
TEST(Ep0Declaration, TheWireVersionIsTheOneTheBoardsWereFlashedWith) {
    EXPECT_EQ(vc::kVersion, 0x5bec) // v17, 2026-10-06: the UART status body gains
                                    // `unattributed` (HPM errors of unknown kind)
        << "the EP0 wire format or its semantics changed: reflash every board with firmware "
           "built from this tree, then pin the new value here";
}

// ---- 校验要判完一切可预见的拒绝 ----
//
// 凑不出的 CAN 速率是纯计算就能判的事: 在校验阶段被拒, 一个寄存器都不碰, 附加功能
// 照常用它的口。推到应用阶段的话, 它会让整份清单回滚、整块板的口一起下线。
TEST(Ep0Declaration, ACanRateTheBoardCannotSolveIsRefusedBeforeAnythingIsTouched) {
    HpmLike board;
    board.drivers().can[0].is_running = true; // 附加功能手里开着的口
    board.drivers().uart(0).is_running = true;

    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kClassic1M);
    configuration.declare_can(
        DataId::kCan2,
        hcs::CanSetting{.fd = true, .arbitration_baudrate = 1'000'000, .data_baudrate = 7'000'000});

    const std::string error = apply_error(board, configuration);
    EXPECT_TRUE(mentions(error, "CAN2"));
    EXPECT_TRUE(mentions(error, "rate unrepresentable"));
    EXPECT_TRUE(mentions(error, "7000000"));
    EXPECT_TRUE(mentions(error, "Nothing was changed on the board"));
    EXPECT_EQ(board.drivers().can[0].applies + board.drivers().can[1].applies, 0U);
    EXPECT_TRUE(board.drivers().can[0].is_running) << "the extras keep their ports";
    EXPECT_TRUE(board.drivers().uart(0).is_running);
    EXPECT_EQ(board.ownership().owner(), link::Owner::kTools);
    vc::ManifestResultPayload result{};
    board.journal().fill(result);
    EXPECT_EQ(result.outcome, std::to_underlying(vc::ManifestOutcome::kManifestRefused));
}

// ---- 应用阶段的失败: 整体回滚, 板子还给接手前的主人 ----
//
// 校验全过之后还能失败的只剩硬件回读不符。从附加功能手里接过来的板子原样还回去: 端口
// 全部恢复给 DMTool / CDC 用, 附加功能的端点重新应答。
TEST(Ep0Declaration, AReadBackFailureGivesTheBoardBackToTheExtras) {
    HpmLike board;
    OwnershipStep::events().clear();
    board.drivers().can[0].is_running = true;
    board.drivers().can[1].is_running = true;
    board.drivers().uart(0).is_running = true;
    board.drivers().can[1].garbles_timing = true;

    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kClassic1M);
    configuration.declare_can(DataId::kCan2, hcs::kFd1M5M);

    const std::string error = apply_error(board, configuration);
    EXPECT_TRUE(mentions(error, "CAN2"));
    EXPECT_TRUE(mentions(error, "read-back disagrees"));
    EXPECT_TRUE(mentions(error, "rolled the whole declaration back"));
    EXPECT_EQ(board.ownership().owner(), link::Owner::kTools);
    EXPECT_FALSE(board.manifest_accepted);
    EXPECT_EQ(OwnershipStep::events(), (std::vector<std::string>{"->libhcs", "->tools"}))
        << "taken before the ports were touched, given back after the rollback";
    EXPECT_TRUE(board.drivers().can[0].is_running) << "every port is the extras' again";
    EXPECT_TRUE(board.drivers().can[1].is_running);
    EXPECT_TRUE(board.drivers().uart(0).is_running);
    EXPECT_EQ(board.last_error().index, std::to_underlying(DataId::kCan2))
        << "the latch names the port that failed";
    vc::ManifestResultPayload result{};
    board.journal().fill(result);
    EXPECT_EQ(result.outcome, std::to_underlying(vc::ManifestOutcome::kManifestRolledBack));
    EXPECT_EQ(result.entry_count, 2U);
    EXPECT_EQ(
        result.entries[1].reason,
        std::to_underlying(vc::ConfigErrorReason::kConfigErrorVerifyFailed));
}

// 已经归 libhcs 的板子(运行中重声明)留在 libhcs: 所有口挂起, 等主机重新声明 -- 配置
// 即声明, 没有一份被完整接受的声明就没有口在线。
TEST(Ep0Declaration, AReadBackFailureWhileLibhcsOwnsTheBoardLeavesEveryPortOff) {
    HpmLike board;
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kClassic1M);
    configuration.declare_uart(DataId::kUart0, hcs::uart_8n1(921'600));
    std::atomic<hcs::Interface> interface{declare(board, configuration)};
    board.ownership().on_session_started();
    ASSERT_TRUE(board.drivers().can[0].is_running);

    board.drivers().can[1].garbles_timing = true;
    EXPECT_THROW(
        hcs::reconfigure_can(board, configuration, interface, DataId::kCan2, hcs::kFd1M5M),
        std::runtime_error);
    EXPECT_EQ(board.ownership().owner(), link::Owner::kLibhcs);
    EXPECT_TRUE(board.ownership().session_allowed());
    EXPECT_FALSE(board.drivers().can[0].is_running);
    EXPECT_FALSE(board.drivers().can[1].is_running);
    EXPECT_FALSE(board.drivers().uart(0).is_running);
    EXPECT_EQ(configuration.find(DataId::kCan2), nullptr) << "the failed setting is not stored";
}

// 接手时端口全部先挂起: 附加功能(DMTool)改过的东西 -- 速率、帧型、自动重传 -- 一概
// 不带进 libhcs。设置与硬件现状"看上去一致"的口也从 libhcs 的设置重新初始化一次。
TEST(Ep0Declaration, TakingTheBoardFromTheExtrasStartsEveryDeclaredPortFromScratch) {
    HpmLike board;
    board.drivers().can[0].is_running = true; // DMTool 正以同样的 1M/5M 用着它
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kFd1M5M);

    (void)declare(board, configuration);
    EXPECT_EQ(board.drivers().can[0].applies, 1U);
    EXPECT_TRUE(board.drivers().can[0].is_running);

    // 已经归 libhcs 的板子重声明同一份设置: 不是易手, 运行中的总线不重来。
    (void)declare(board, configuration);
    EXPECT_EQ(board.drivers().can[0].applies, 1U);
}

// 热路径的按口分发(固件的下行数据与主循环轮询走的就是它): 按类型与 DataId 找到驱动;
// 类型不符、或本 PCB 没有这个口(单 CAN 板的 CAN2), 都是"不认识", 不碰任何驱动。
TEST(Ep0Declaration, DispatchFindsThePortByKindAndIdentityOnly) {
    HpmLike board;
    using Registry = HpmLike::Registry;
    const auto touch = [](auto, auto& port) {
        port.is_running = true;
        return true;
    };

    EXPECT_TRUE(Registry::dispatch<spec::PortKind::kCan>(DataId::kCan2, touch));
    EXPECT_TRUE(board.drivers().can[1].is_running);
    EXPECT_FALSE(board.drivers().can[0].is_running) << "the wrong bus was reached";

    EXPECT_FALSE(Registry::dispatch<spec::PortKind::kUart>(DataId::kCan1, touch))
        << "a CAN identity reached a UART";
    EXPECT_FALSE(board.drivers().can[0].is_running);
    EXPECT_FALSE(Registry::dispatch<spec::PortKind::kCan>(DataId::kUart0, touch));

    // f 说不收(例如只收不发的口)就是不认识。
    EXPECT_FALSE(Registry::dispatch<spec::PortKind::kUart>(DataId::kUart0, [](auto, auto&) {
        return false;
    }));

    DriversPtr<HpmShape>::dual_can = false; // 单 CAN 的 PCB: CAN2 的驱动没初始化
    board.drivers().can[1].is_running = false;
    EXPECT_FALSE(Registry::dispatch<spec::PortKind::kCan>(DataId::kCan2, touch));
    EXPECT_FALSE(board.drivers().can[1].is_running);
    DriversPtr<HpmShape>::dual_can = true;
}

// 运行时状态不走 EP0(v16): 退役的 kGetPortStatus 码不被重用, 旧主机问它得到 STALL。
TEST(Ep0Declaration, TheRetiredStatusRequestStalls) {
    HpmLike board;
    std::array<std::byte, 24> reply{};
    EXPECT_FALSE(board.vendor_control_in(
        0x4C, std::to_underlying(DataId::kCan1), reply.data(), reply.size()));
}

// 运行时状态的出处: 注册表里在跑(被声明)的 CAN / UART 口, 读驱动的 read_status()。
// 没声明的口挂起着, 不报; 别类的口(GPIO、IMU)没有这个原语, 编译期就不参与。
TEST(Ep0Declaration, OnlyRunningCanAndUartPortsOfferTheirStatus) {
    Mc02Like board; // 有 IMU、GPIO、蜂鸣器: 它们不该出现在下面
    auto& drivers = board.drivers();
    drivers.can[0].is_running = true;
    drivers.can[0].status = {.tec = 9, .last_error = data::CanLastError::kAck};
    drivers.can[1].is_running = false; // 没声明
    drivers.can[1].status = {.tec = 200};
    drivers.uart(4).is_running = true; // UART7
    drivers.uart(4).status = {.framing = 3};

    struct Round {
        std::vector<std::pair<DataId, data::CanStatusView>> cans;
        std::vector<std::pair<DataId, data::UartStatusView>> uarts;
        void offer(DataId port, const data::CanStatusView& status) { cans.emplace_back(port, status); }
        void offer(DataId port, const data::UartStatusView& status) {
            uarts.emplace_back(port, status);
        }
    } round;
    link::offer_port_status<RegistryFor<Mc02Shape>>(round);

    ASSERT_EQ(round.cans.size(), 1U);
    EXPECT_EQ(round.cans[0].first, DataId::kCan1);
    EXPECT_EQ(round.cans[0].second.tec, 9);
    ASSERT_EQ(round.uarts.size(), 1U);
    EXPECT_EQ(round.uarts[0].first, DataId::kUart7);
    EXPECT_EQ(round.uarts[0].second.framing, 3U);
}

// 口能做什么由板子说了算: 长帧门禁用的是板子在口清单里报的能力(出自驱动类型),
// 主机没有自己的一份。
TEST(Ep0Declaration, TheLongFrameGateFollowsWhatTheBoardReports) {
    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kFd1M5M);

    HpmLike hpm;
    const hcs::Interface carries = declare(hpm, configuration);
    EXPECT_TRUE(carries.long_frames(DataId::kCan1));
    EXPECT_TRUE(carries.long_frames(DataId::kCan2)) << "reported even when undeclared";

    Mc02Like mc02;
    const hcs::Interface refuses = declare(mc02, configuration);
    EXPECT_FALSE(refuses.long_frames(DataId::kCan1)) << "mc02's CAN driver does not carry them";
    EXPECT_TRUE(refuses.can_fd(DataId::kCan1));

    vc::PortListPayload list{};
    ASSERT_TRUE(mc02.vendor_control_in(
        std::to_underlying(vc::Request::kGetPortList), 0, &list, sizeof(list)));
    EXPECT_EQ(list.ports[3].capabilities, spec::kUartCapRxPolaritySettable) << "DBUS";
    EXPECT_EQ(list.ports[4].capabilities, 0U) << "UART1";
}

// ---- 归属: 清单通过校验才易手, 全部生效才落定 ----
//
// kGetPortList 是纯读, 附加功能不因一次读接口就交出板子。
TEST(Ep0Declaration, TheBoardChangesHandsOnlyWhenAManifestIsApplied) {
    Mc02Like board;
    OwnershipStep::events().clear();

    hcs::Configuration configuration;
    configuration.declare_can(DataId::kCan1, hcs::kFd1M5M);

    // 一次纯读不改变归属。
    (void)hcs::read_port_list(board);
    EXPECT_FALSE(board.manifest_accepted);
    EXPECT_FALSE(board.ownership().session_allowed());

    // 清单被接受: 归属切换, 会话门打开。
    (void)declare(board, configuration);
    EXPECT_TRUE(board.manifest_accepted);
    EXPECT_TRUE(board.ownership().session_allowed());
    EXPECT_EQ(OwnershipStep::events(), std::vector<std::string>{"->libhcs"});

    // 会话结束: 交还。
    OwnershipStep::events().clear();
    board.end_session();
    EXPECT_FALSE(board.ownership().session_allowed());
    EXPECT_EQ(OwnershipStep::events(), std::vector<std::string>{"->tools"});
}

} // namespace
