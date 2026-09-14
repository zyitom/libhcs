#pragma once

#include <chrono>
#include <concepts>
#include <cstdint>
#include <limits>
#include <ratio>
#include <type_traits>

#include <main.h>
#include <tim.h>

#include "core/src/utility/assert.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

namespace libhcs::firmware::timer {

// IMU/GPIO 遥测的时间戳源。
//
// 协议时间戳单位是 quarter-us(4 MHz tick), 与 c_board、hpm_board 一致。mc02 的
// SYSCLK 为 550 MHz, 各定时器内核跑 275 MHz; 550 MHz 含因子 11, 任何整数预分频
// 都得不到恰好 4 MHz。改为把 TIM5(自由运行 32 位定时器)预分频到恰好 1 MHz,
// timepoint() 返回 CNT << 2 -- 精确的 quarter-us 值(分辨率 1 us, 对 <= 2 kHz 的
// IMU 足够)。TIM5 的 ARR 取 0x3FFFFFFF, 使 (CNT << 2) 铺满整个 uint32 区间并在
// 2^32 quarter-us(约 1073 s)处干净回绕; 主机只消费回绕安全的差值。
class Timer {
public:
    using Lazy = utility::Lazy<Timer>;

    static constexpr uint32_t kClockFrequency = 4'000'000;
    using TickPeriod = std::ratio<1, kClockFrequency>;

    // 单位为 1/4 us
    using Duration = std::chrono::duration<uint32_t, TickPeriod>;
    using TimePoint = std::chrono::time_point<uint32_t, Duration>;

    // 真窗口至少保留半个计数周期, 到期检查才能无状态地做。
    static constexpr uint32_t kMaxDurationTicks = uint32_t{1} << 31;

    static constexpr TIM_HandleTypeDef* kTimer = &htim5;

    Timer() {
        core::utility::assert_always(HAL_TIM_Base_Start(kTimer) == HAL_OK);
    }

    TimePoint timepoint() const {
        return TimePoint{Duration{kTimer->Instance->CNT << 2u}};
    }

    [[nodiscard]] bool check_expired(TimePoint start_point, Duration delay) const {
        core::utility::assert_debug(delay.count() <= kMaxDurationTicks);

        const uint32_t start_ticks = start_point.time_since_epoch().count();
        const uint32_t now_ticks = timepoint().time_since_epoch().count();
        const Duration elapsed_duration{static_cast<uint32_t>(now_ticks - start_ticks)};
        return elapsed_duration >= delay;
    }

    [[nodiscard]] bool check_reached(TimePoint deadline) const {
        const uint32_t deadline_ticks = deadline.time_since_epoch().count();
        const uint32_t now_ticks = timepoint().time_since_epoch().count();
        const uint32_t elapsed_ticks = now_ticks - deadline_ticks;
        return elapsed_ticks < kMaxDurationTicks;
    }

    // 基于自由运行的 TIM5 计数器忙等给定时长。直接读 CNT, 故可在中断关闭时使用
    // (如 InterruptLockGuard 下的初始化阶段); 要求 timer.init() 已启动 TIM5。
    void spin_wait(Duration delay) const {
        core::utility::assert_debug(delay.count() <= kMaxDurationTicks);

        const TimePoint start = timepoint();
        while (!check_expired(start, delay))
            ;
    }

    template <std::integral Rep, typename Period>
    [[nodiscard]] static constexpr Duration
        to_duration_checked(std::chrono::duration<Rep, Period> duration) {
        static_assert(Period::num > 0 && Period::den > 0);

        const uint64_t count = count_to_u64_checked(duration.count());
        using InputDuration = std::chrono::duration<uint64_t, Period>;
        const InputDuration duration_u64{count};

        constexpr Duration max_duration{kMaxDurationTicks};
        const InputDuration max_input_duration =
            std::chrono::duration_cast<InputDuration>(max_duration);

        core::utility::assert_debug(duration_u64 <= max_input_duration);
        const Duration delay_duration = std::chrono::ceil<Duration>(duration_u64);

        core::utility::assert_debug(delay_duration.count() <= kMaxDurationTicks);
        return delay_duration;
    }

private:
    template <std::integral Rep>
    [[nodiscard]] static uint64_t count_to_u64_checked(Rep count) {
        if constexpr (std::is_signed_v<Rep>)
            core::utility::assert_debug(count >= 0);

        if constexpr (sizeof(Rep) > sizeof(uint64_t)) {
            core::utility::assert_debug(
                count <= static_cast<Rep>(std::numeric_limits<uint64_t>::max()));
        }
        return static_cast<uint64_t>(count);
    }
};

inline constinit Timer::Lazy timer;

} // namespace libhcs::firmware::timer
