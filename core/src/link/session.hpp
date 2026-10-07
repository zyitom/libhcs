#pragma once

#include <cstdint>

// 主机会话: kStart 的 nonce 握手与 keepalive 租约。这段协议逻辑原先在 mc02、
// c_board 与 hpm 的固件里各写一份, 细节已经漂移(租约 4 s / 1 s / 4 s, 到期是否
// 忘掉 EP0 握手, 租约检查放在毫秒杂务还是每趟 try_transmit), 2026-10-05 收拢成
// 这一个纯状态机 -- 形状与 ownership.hpp 相同: 不碰外设、不碰传输层, "现在几点"
// 由使用方注入, 决策对应的动作(应答、清批量、停口)也由使用方执行。
//
// ---- 与使用方的分工 ----
//
//   - 本状态机: 会话是否在、当前 nonce、租约还有没有效, 以及 kStart / kKeepalive /
//     到期检查这三个入口的裁决。
//   - 板子的 Vendor / HostSession: 只剩"板子才有"的部分 -- 时间从哪个外设来
//     (各板统一换算成 1/4 us 的 uint32_t, 低 32 位, 无符号差回绕比较), 门是否打开
//     (mc02/c_board 的 EP0 握手标志、hpm 的归属状态), 新会话时清什么(批量、GPIO
//     输出、传输进度), 到期时停什么(Registry::suspend_all() / 交还归属)。
//
// 门(open/closed)不在状态机里: 它的来源各板不同(hpm 的门跟着归属状态机走, 不是
// 一个布尔位), 会话开始时作为参数传入即可。
//
// ---- 时间 ----
//
// 单位是 1/4 us, 载体 uint32_t, 按 2^32 回绕(约 1073 s 一圈)。租约 4 s =
// 16'000'000 tick, 远小于 2^31, 无符号差 now - last_refresh 恒可比较。三块板的
// 定时器都跑 1/4 us: mc02 的 TIM5 是 32 位计数(到点回绕), c_board 与 hpm 传 64 位
// 计数的低 32 位 -- 低 32 位自身回绕, 差值语义不变。
//
// ---- 租约 ----
//
// 4 s, 从最近一次 kStart 或 kKeepalive 起算。主机轮次是 250 ms
// (host/src/protocol/handler.cpp 的 kSessionRefreshInterval), 租约是轮次的 16 倍:
// 轮次落在到期边界上只会把失效推迟一个轮次, 不会误杀。租期必须大于轮次, 否则轮次
// 与租期同长时 1/16 的轮次会落在到期边界上, 板子静默丢 keepalive
// [实测 2026-09-13, hpm: 租期与轮次同长时 ~70% 轮次超时]。
//
// 到期一律忘掉 EP0 握手(由使用方在 poll() 返回 true 时执行自己的停口与忘门):
// tud_mount_cb 只在重新枚举时触发, 不重新插拔地更换主机程序不会重新枚举, 不忘门
// 的板子会让从不握手的新主机继承上一台的通行门 [实测于 hpm_board, 径直穿门而过]。
//
// 只在主循环运行(EP0 处理器与会话状态机都经 tud_task() 到达, 租约检查在毫秒杂务),
// 不需要同步。

namespace libhcs::core::link {

class Session {
public:
    // 会话租约, 单位 1/4 us: 4 s。
    static constexpr uint32_t kLeaseTicks = 16'000'000U;

    // kStart 的裁决。gate_open 为假(EP0 握手门没开 / 板子不归 libhcs)时一律
    // kRefused, 一言不发 -- 会话协议没有否定应答, 开不了会话的主机约一个轮次后
    // 自行触发 ack 超时并给出自己的报错, 沉默是本层唯一能说的话。
    enum class Start : std::uint8_t {
        kRefused, // 门没开: 什么都不做, 也不应答。
        kOpened,  // 新会话(nonce 变了, 或此前没有会话): 使用方先清批量与旧主机的
                  // 输出, 再应答 kStartAck。
        kRenewed, // 同 nonce 重开(主机重试或轮次): 租约已刷新, 照常应答, 板子
                  // 不得清任何东西 -- 在跑的口与在途的批量原样继续。
    };

    [[nodiscard]] bool established() const noexcept { return established_; }

    // 当前会话的 nonce。仅在 established() 为真时有意义; 时间锚与脉冲调度等搭乘
    // 会话的字段用它校验来源。
    [[nodiscard]] std::uint32_t nonce() const noexcept { return nonce_; }

    [[nodiscard]] Start on_start(std::uint32_t nonce, bool gate_open, std::uint32_t now) noexcept {
        if (!gate_open)
            return Start::kRefused;
        if (established_ && nonce == nonce_) {
            last_refresh_ = now;
            return Start::kRenewed;
        }
        established_ = true;
        nonce_ = nonce;
        last_refresh_ = now;
        return Start::kOpened;
    }

    // kKeepalive: 会话在且 nonce 对上才刷新租约并应答。
    [[nodiscard]] bool on_keepalive(std::uint32_t nonce, std::uint32_t now) noexcept {
        if (!established_ || nonce != nonce_)
            return false;
        last_refresh_ = now;
        return true;
    }

    // 会话到此为止: 租约到期、总线复位、挂起、拔线都走这里。此后 keepalive 一律
    // 不应答, 同 nonce 的 kStart 也算新会话。
    void end() noexcept { established_ = false; }

    // 租约检查, 主循环的毫秒杂务调用(读一次板级时刻; 不放在每趟主循环都走的
    // 发送路径上)。返回 true 表示租约刚刚到期, 此后 established() 为假 -- 使用方
    // 要停掉所有口并忘掉 EP0 握手。
    [[nodiscard]] bool poll(std::uint32_t now) noexcept {
        if (!established_)
            return false;
        if (now - last_refresh_ < kLeaseTicks)
            return false;
        end();
        return true;
    }

private:
    bool established_ = false;
    std::uint32_t nonce_ = 0;
    std::uint32_t last_refresh_ = 0;
};

} // namespace libhcs::core::link
