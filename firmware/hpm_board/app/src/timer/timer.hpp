#pragma once

#include <atomic>
#include <cstdint>

#include <hpm_clock_drv.h>
#include <hpm_mchtmr_drv.h>
#include <hpm_soc.h>

#include "board_app.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::timer {

class Timer {
public:
    using Lazy = utility::Lazy<Timer>;

    static constexpr uint64_t kTimerFrequencyHz = 4'000'000U;

    static constexpr uint32_t kTickFrequencyHz = 1'000U;
    static constexpr uint64_t kTickPeriodTicks =
        (static_cast<uint64_t>(kTimerFrequencyHz) + (kTickFrequencyHz / 2U)) / kTickFrequencyHz;

    Timer() {
        // 当前核的机器定时器(core0 板上是 mchtmr0, ECAT 桥的现场总线核上
        // 是 mchtmr1)时钟必须为 4 MHz; 各板据此配置分频。
        core::utility::assert_always(
            clock_get_frequency(board::kMchtmrClockName) == kTimerFrequencyHz);

        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
        next_tick_compare_value_ = timestamp64_quarter_us() + kTickPeriodTicks;
        mchtmr_set_compare_value(HPM_MCHTMR, next_tick_compare_value_);

        enable_mchtmr_irq();
    }

    // 刻意保持最小: 机器定时器中断(MTIP)绕过 PLIC 优先级门槛, 嵌套中断
    // 包装层一旦重新打开 mstatus.MIE, 它就能抢占乃至优先级 3 的 CAN ISR。
    // 在这里做的任何事都会加到转发热路径的最坏延迟上 -- 因此 LED 记账改由
    // 主循环依据 tick_count() 驱动。
    void irq_handler() {
        tick_counter_.store(
            tick_counter_.load(std::memory_order::relaxed) + 1, std::memory_order::relaxed);

        do {
            next_tick_compare_value_ += kTickPeriodTicks;
        } while (next_tick_compare_value_ <= timestamp64_quarter_us());
        mchtmr_set_compare_value(HPM_MCHTMR, next_tick_compare_value_);
    }

    // 1 kHz tick 计数器, 供主循环节奏型工作(LED 花样)使用。
    uint32_t tick_count() const { return tick_counter_.load(std::memory_order::relaxed); }

    static uint32_t timestamp_quarter_us() {
        // 经 32 位 MMIO 视图读 MTIME, 避免别名访问 SDK 的 uint64_t 字段。
        const volatile uint32_t* const mtime_words =
            reinterpret_cast<volatile uint32_t*>(HPM_MCHTMR_BASE);
        return mtime_words[0];
    }

    static uint64_t timestamp64_quarter_us() {
        // riscv32 上读 64 位 MTIME MMIO 寄存器会编译成两次 32 位加载。
        // 按高-低-高顺序读取并在回绕时重试, 保证返回的 64 位时间戳自洽。
        const volatile uint32_t* const mtime_words =
            reinterpret_cast<volatile uint32_t*>(HPM_MCHTMR_BASE);
        uint32_t hi1 = 0;
        uint32_t lo = 0;
        uint32_t hi2 = 0;

        do {
            hi1 = mtime_words[1];
            lo = mtime_words[0];
            hi2 = mtime_words[1];
        } while (hi1 != hi2);

        return (static_cast<uint64_t>(hi2) << 32U) | lo;
    }

private:
    uint64_t next_tick_compare_value_ = 0;
    std::atomic<uint32_t> tick_counter_{0};
    static_assert(std::atomic<uint32_t>::is_always_lock_free);
};

inline constinit Timer::Lazy timer;

} // namespace libhcs::firmware::timer
