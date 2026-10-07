#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <type_traits>
#include <utility>

#include "core/include/libhcs/protocol/vendor_control.hpp"
#include "core/include/libhcs/spec/port.hpp"
#include "core/src/link/port.hpp"

// 编译期端口注册表 -- 板子的口组合是一组类型, 不是运行时的虚函数表。
//
// 模板参数是"绑定"类型, 每口一个, 由各固件的 ports.hpp 提供。绑定从 PortOf<描述符>
// 派生(身份与类型取自 spec 的具名描述符), 自己只写"硬件怎么到":
//   static <驱动*> instance();   驱动对象; 上电时没初始化(本 PCB 没有这个口, 例如单 CAN
//                                的 hpm5321 的 CAN2)就是 null
//
// 口有没有、能做什么, 都由板子自己的事实决定, 不由 spec 决定:
//   - 有没有: 驱动初始化了就有。kGetPortList 只报 instance() 非空的口, EP0 对其余的口
//     一律 BadIndex。没有另一张"本 PCB 有哪些口"的表要与驱动的初始化保持一致。
//   - 能做什么: 驱动类型的 kPortCapabilities(spec/port.hpp 的能力位编码)。同一个驱动
//     在哪块板上都是同样的能力(hpm5321 与 hpm6e8y 的 MCAN 驱动是同一个), 能力跟着
//     实现走, 主机从口清单里学到它。
//
// 遍历与按 DataId 的分发都以 fold expression 展开成直线代码: 无虚调用、无堆、无查表。
// EP0 的请求处理与"遍历各口"的工作(suspend/resume 全部、端口清单的组装)共用这一层。
//
// 绑定与板型口表(Spec::kPorts)的一致性由 matches() 在编译期核对: 一一对应, 同 DataId、
// 同类型。漏绑、多绑、绑错类型都是编译错误。

namespace libhcs::core::link {

// 绑定的公共半边: 身份与类型取自 spec 的具名描述符。派生的绑定只补 instance()。
template <auto kDescriptor>
struct PortOf {
    static constexpr data::DataId data_id = kDescriptor.data_id;
    static constexpr spec::PortKind kind = decltype(kDescriptor)::kind;
};

// 驱动是一个惰性构造的全局对象(固件的 Lazy: init() 之前 try_get() 为空)时的绑定, 一行
// 写完: using Can1 = LazyPort<Spec::Cans::kCan1, can::can1>;
template <auto kDescriptor, auto& kDriver>
struct LazyPort : PortOf<kDescriptor> {
    static auto* instance() { return kDriver.try_get(); }
    // 调用方另有证据说明驱动已构造(例如主循环的位只由驱动自己的 start() 置上)时用: 不判空,
    // 热路径上省一次加载与分支(debug 构建下 Lazy 仍断言)。
    static auto& driver() { return *kDriver; }
};

template <typename B>
using DriverOf = std::remove_pointer_t<decltype(B::instance())>;

// 绑定的口能做什么: 它的驱动类型说了算。
template <typename B>
inline constexpr std::uint8_t kCapabilitiesOf = DriverOf<B>::kPortCapabilities;

template <typename B>
concept PortBinding = requires {
    { B::data_id } -> std::same_as<const data::DataId&>;
    { B::kind } -> std::same_as<const spec::PortKind&>;
    { B::instance() } -> std::same_as<DriverOf<B>*>;
    { DriverOf<B>::kPortCapabilities } -> std::convertible_to<std::uint8_t>;
};

template <typename... Bindings>
requires(PortBinding<Bindings> && ...) class PortRegistry {
public:
    static constexpr std::size_t size() noexcept { return sizeof...(Bindings); }

    // 编译期核对: 与板型的口表一一对应。顺序不要求一致, 数量、身份、类型要求全同。
    template <std::size_t n>
    static constexpr bool matches(const std::array<spec::PortDescriptor, n>& ports) noexcept {
        if constexpr (sizeof...(Bindings) != n || n > spec::kMaxPorts) {
            return false;
        } else {
            // 绑定之间无重复 + 每个绑定都在口表里 + 数量相等 == 一一对应。少了第一条,
            // "复制一个绑定忘了改身份、同时漏掉另一个口"就能蒙混过关: 数量对得上, 每个
            // 绑定也都在口表里, 被漏掉的口却在 EP0 上永远寻址不到。
            return spec::distinct_data_ids(Bindings{}...)
                && (binding_in_spec<Bindings>(ports) && ...);
        }
    }

    // 每个绑定一次, 编译期展开。f 是泛型可调用, 以绑定类型被调用: f(Binding{})。
    template <typename F>
    static void for_each(F&& f) {
        (f(std::type_identity_t<Bindings>{}), ...);
    }

    // 上电时初始化了的每个口(= 本 PCB 实有的口), 以 (绑定, 驱动引用) 调用 f。
    // suspend/resume 全部、端口清单的组装都走它。
    template <typename F>
    static void for_each_live(F&& f) {
        for_each([&f](auto binding) {
            if (auto* port = binding.instance())
                f(binding, *port);
        });
    }

    static void suspend_all() {
        for_each_live([](auto, auto& port) { port.suspend(); });
    }

    static void resume_all() {
        for_each_live([](auto, auto& port) { port.resume(); });
    }

    // 绑定的身份与驱动自己报的身份一致(驱动带 data_id() 时): 数据流按驱动的身份打标,
    // EP0 按绑定的身份寻址, 两者指向同一个连接器才算接对。驱动的身份在构造参数里,
    // 编译期看不到, 由固件在初始化完驱动后断言一次(冷路径, 只在上电时跑)。
    [[nodiscard]] static bool identities_match() {
        bool match = true;
        for_each_live([&match](auto binding, auto& port) {
            if constexpr (requires { port.data_id(); })
                match = match && port.data_id() == binding.data_id;
        });
        return match;
    }

    // 按 DataId 找口。找到且驱动已初始化时以 (Binding, 驱动引用) 调用 f 并返回 true;
    // DataId 不在注册表里返回 false; 在但本 PCB 没初始化它时把原因写进 miss 并返回
    // true。两种对主机都是"没有这个口"。
    template <typename F>
    static bool call(data::DataId id, PortOutcome& miss, F&& f) {
        return (... || (Bindings::data_id == id && call_binding<Bindings>(miss, f)));
    }

    // 热路径的按口分发(下行数据、主循环轮询): kind 为 kKind、DataId 为 id、且本 PCB 有它
    // (驱动已初始化)时, 以 (绑定, 驱动引用) 调用 f 并返回 f 的 bool 结果; 否则返回 false。
    // 别类的绑定在编译期就被筛掉(if constexpr), 剩下的是与手写 switch 同形的一串比较 --
    // 板上不再需要另一份"DataId -> 驱动对象"的手写表。
    template <spec::PortKind kKind, typename F>
    static bool dispatch(data::DataId id, F&& f) {
        return (... || dispatch_one<kKind, Bindings>(id, f));
    }

private:
    template <spec::PortKind kKind, typename B, typename F>
    static bool dispatch_one(data::DataId id, F& f) {
        if constexpr (B::kind != kKind) {
            return false;
        } else {
            if (B::data_id != id)
                return false;
            auto* port = B::instance();
            return port != nullptr && f(std::type_identity_t<B>{}, *port);
        }
    }

    template <typename B, std::size_t n>
    static constexpr bool
        binding_in_spec(const std::array<spec::PortDescriptor, n>& ports) noexcept {
        for (const auto& port : ports) {
            if (port.data_id == B::data_id)
                return port.kind == B::kind;
        }
        return false;
    }

    template <typename B, typename F>
    static bool call_binding(PortOutcome& miss, F&& f) {
        auto* port = B::instance();
        if (port == nullptr) {
            // 本 PCB 没初始化这个驱动: 没有这个口。
            miss = PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadIndex);
            return true;
        }
        f(std::type_identity_t<B>{}, *port);
        return true;
    }
};

// 归属交接(ownership.hpp)的端口一步, 固件与主机测试用的是这同一步:
//   to_libhcs(): 从附加功能手里接过来的口一律先挂起 -- 附加功能(DMTool 的 SETUP_BUARD)
//     可能改过速率、帧型、自动重传, 挂起的口在清单应用时从 libhcs 的设置重新初始化,
//     什么都不会从附加功能那边漏进 libhcs("CAN 单发"就是这样保住的)。
//   to_tools(): 交还时全部恢复 -- 附加功能(DMTool 仿真、CDC 串口桥)不走 EP0, 通道必须
//     是通的, 与上电状态相同。
template <typename Registry>
struct PortHandoff {
    static void to_libhcs() { Registry::suspend_all(); }
    static void to_tools() { Registry::resume_all(); }
};

} // namespace libhcs::core::link
