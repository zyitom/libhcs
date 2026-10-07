#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <type_traits>
#include <utility>

// 板子此刻服务谁: libhcs, 还是板上的附加功能(DMTool 仿真与 CDC 串口桥)。两者严格互斥:
// 任何时刻只有一方能动端口、收发消息, 另一方被完全屏蔽 -- 不是"不处理", 是对它的端点
// 不应答、不给它端口。
//
// ---- 规则 ----
//
// 上电时板子归附加功能。libhcs 的 kApplyManifest 是唯一的接手入口, 它是一次事务:
//   - 校验阶段(不碰硬件)被拒: 什么都没发生, 附加功能照常用它的口, 归属不动。
//   - 校验通过: 先把板子从附加功能手里拿过来(begin_claim -> 各步 to_libhcs(): 附加功能
//     停下、它的端点不再应答, 端口全部挂起 -- 附加功能改过的速率、帧型、自动重传
//     一概不带进 libhcs), 然后才由清单动端口。清单应用期间附加功能看不到任何中间态。
//   - 应用全部成功: commit_claim(), 板子归 libhcs, 直到下面三件事之一发生才交还:
//       * 接受清单后一个租约之内没开出会话(主机只声明了端口就走了, 或进程在声明后死了);
//       * 会话结束(租约到期);
//       * 总线复位、挂起或拔线。
//   - 应用中途失败: abort_claim()。这一轮是从附加功能手里拿来的, 就原样交还(各步
//     to_tools(), 端口恢复成附加功能要的状态, CDC 写回它的线路编码); 板子本来就归
//     libhcs, 就留在 libhcs, 端口全部挂起(配置即声明: 没有一份被完整接受的声明, 就
//     没有口在线), 由主机重新声明或等租约到期交还。
// 每一次交接都把双方带到干净状态: 让出的一方停下并清空自己的队列, 接手的一方从头
// 开始, 数据不会跨过交接点。
//
// 校验/接手/应用/提交或回退全部在同一个 EP0 数据段处理器里完成, 中间没有主循环轮次:
// 不存在"板子已归 libhcs 但声明的配置还没生效"的窗口, 也不存在"附加功能还在用、端口
// 已被 libhcs 改了一半"的窗口。
//
// ---- 结构 ----
//
// 状态机不认识任何外设。交接的每一步是一个满足 HandoffStep 的类型, 由使用方按顺序
// 列出: 接手时按列出的顺序执行各步的 to_libhcs(), 交还时按相反的顺序执行
// to_tools() -- 与构造、析构同一个约定, 后接手的先放手。纯头文件、不含芯片头, 主机侧
// 单元测试直接实例化它(host/tests/board_ownership_test.cpp)。
//
// 只在主循环上运行(EP0 请求、TinyUSB 回调、1 kHz tick 都在主循环), 不需要同步。

namespace libhcs::core::link {

enum class Owner : std::uint8_t {
    kTools,  // 附加功能: DMTool 仿真与 CDC 串口桥。上电状态
    kLibhcs, // 从清单接手起
};

// 交接的一步。to_libhcs() 在板子从附加功能易手给 libhcs 时执行一次, to_tools() 在交还
// 时执行一次; 两者成对出现。
template <typename T>
concept HandoffStep = requires {
    { T::to_libhcs() } -> std::same_as<void>;
    { T::to_tools() } -> std::same_as<void>;
};

template <HandoffStep... Steps>
class Ownership {
public:
    // claim_timeout: 清单接受后等主机开会话的时限, 单位与传给 commit_claim() / poll()
    // 的时刻相同。
    constexpr explicit Ownership(std::uint64_t claim_timeout) noexcept
        : claim_timeout_(claim_timeout) {}

    [[nodiscard]] constexpr Owner owner() const noexcept {
        return state_ == State::kTools ? Owner::kTools : Owner::kLibhcs;
    }

    // 现在是否接受 libhcs 的 kStart: 没被清单声明过的主机开不了会话。清单还在应用中
    // (kClaiming)不算: 那一刻没有会话请求能插进来, 写成这样只是不给中间态开门。
    [[nodiscard]] constexpr bool session_allowed() const noexcept {
        return state_ == State::kClaimed || state_ == State::kServing;
    }

    // 清单通过校验、即将动端口: 先把板子拿过来。只在真正易手时执行各步 to_libhcs():
    // 板子已经归 libhcs 时附加功能早已停下、端点早已隔离, 中间没有任何交还(交还只经
    // release(), 它把状态带回 kTools), 再跑一遍只会让运行中的口无谓地重来。
    void begin_claim() {
        if (state_ != State::kTools)
            return;
        state_ = State::kClaiming;
        (Steps::to_libhcs(), ...);
    }

    // 清单全部生效。会话已在时会话继续说了算; 否则从此刻起计声明时限。
    constexpr void commit_claim(std::uint64_t now) noexcept {
        if (state_ != State::kServing) {
            state_ = State::kClaimed;
            claimed_at_ = now;
        }
    }

    // 清单应用中途失败(端口已被核心整体挂起)。从附加功能手里拿来的就原样还回去;
    // 本来归 libhcs 的留在 libhcs。
    void abort_claim() {
        if (state_ == State::kClaiming)
            release();
    }

    // libhcs 会话建立: 板子从此由会话租约守着, 不再受声明时限约束。
    constexpr void on_session_started() noexcept {
        if (state_ == State::kClaimed)
            state_ = State::kServing;
    }

    // 交还附加功能: 会话结束、总线复位、挂起或拔线。已经归附加功能时什么都不做。
    void release() {
        if (state_ == State::kTools)
            return;
        state_ = State::kTools;
        to_tools_in_reverse(std::index_sequence_for<Steps...>{});
    }

    // 声明时限的检查, 主循环按 1 kHz 调用。时钟只在真有清单在等会话时才读: 读一次
    // 时刻在板上是几次外设寄存器读, 而这个状态绝大多数时候不成立。
    template <std::invocable Clock>
    requires std::convertible_to<std::invoke_result_t<Clock>, std::uint64_t>
    void poll(Clock&& now) {
        if (state_ != State::kClaimed)
            return;
        if (std::forward<Clock>(now)() - claimed_at_ >= claim_timeout_)
            release();
    }

private:
    enum class State : std::uint8_t {
        kTools,    // 归附加功能
        kClaiming, // 从附加功能手里接过来, 清单正在应用(只存在于一次 EP0 数据段内)
        kClaimed,  // 清单已接受, 等主机开会话
        kServing,  // libhcs 会话在
    };

    template <std::size_t... indices>
    static void to_tools_in_reverse(std::index_sequence<indices...> /*unused*/) {
        using Ordered = std::tuple<Steps...>;
        (std::tuple_element_t<sizeof...(Steps) - 1 - indices, Ordered>::to_tools(), ...);
    }

    std::uint64_t claim_timeout_;
    std::uint64_t claimed_at_ = 0;
    State state_ = State::kTools;
};

} // namespace libhcs::core::link
