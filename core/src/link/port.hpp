#pragma once

#include <cstdint>

#include "core/include/libhcs/protocol/vendor_control.hpp"
#include "core/include/libhcs/spec/port.hpp"

// 端口操作的公共类型: 口的拒绝结果与口的当前状态。纯逻辑核心(core/src/link/)与
// 各固件的驱动都只认这一层, 不认任何芯片头 -- 主机测试用假驱动实例化同一份逻辑。

namespace libhcs::core::link {

namespace vc = libhcs::core::protocol::vendor_control;

// 一个口上的操作为什么被拒。两字节正好是拒绝原因锁存(port 侧)与逐口清单结果
// (主机侧)共用的形状: 原因码 + 肇事值(速率、模式字节等; 与值无关的拒绝为 0)。
struct PortOutcome {
    vc::ConfigErrorReason reason = vc::ConfigErrorReason::kConfigErrorNone;
    std::uint32_t value = 0;

    [[nodiscard]] constexpr bool ok() const noexcept {
        return reason == vc::ConfigErrorReason::kConfigErrorNone;
    }

    static constexpr PortOutcome pass() noexcept { return {}; }
    static constexpr PortOutcome
        refuse(vc::ConfigErrorReason reason, std::uint32_t value = 0) noexcept {
        return {reason, value};
    }

    friend constexpr bool operator==(const PortOutcome&, const PortOutcome&) = default;
};

// 一个口此刻的状态, kGetPortList 每项的 status 字段的来源。
struct PortStatus {
    bool running = false; // spec::kPortRunning
    bool fd = false;      // spec::kPortFd -- 仅 CAN 有意义
};

// 一路 CAN 的设置值就是声明里的那一个(vc::CanSetting): 主机、核心、各板 CAN 驱动的
// apply_setting() 收同一个类型。速率可设的板(hpm)两项都是设置; 速率写死的板(mc02/c_board)
// 只看 fd, 速率字段必须等于硬件现值(校验阶段已保证)。
using CanSetting = vc::CanSetting;

// 一路 CAN 的位时序事实(速率 + 采样点), 从寄存器重构或由设置解出。速率可设的板
// 用它核对"应用后应有的时序", 速率写死的板它就是校验的基准。
struct CanTimingValue {
    std::uint32_t arbitration_baudrate = 0;
    std::uint32_t data_baudrate = 0;
    std::uint16_t nominal_sample_point = 0;
    std::uint16_t data_sample_point = 0;

    friend constexpr bool operator==(const CanTimingValue&, const CanTimingValue&) = default;
};

// 宽松兜底: 实际速率偏离请求超 10% 视为离谱; 严格判据仍是分频器整数(UartDivisor)。
// 各固件 vendor_control.cpp 原先各有一份, 收敛至此。
constexpr bool rate_plausible(std::uint32_t requested, std::uint32_t achieved) noexcept {
    const std::uint64_t error = achieved > requested ? achieved - requested : requested - achieved;
    return error * 10U <= static_cast<std::uint64_t>(requested);
}

} // namespace libhcs::core::link
