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

// Timestamp source for IMU/GPIO telemetry.
//
// The protocol expresses timestamps in quarter-microseconds (a 4 MHz tick), the
// same unit used by c_board and hpm_board. On mc02 the SYSCLK is 550 MHz, so
// every timer kernel runs at 275 MHz; because 550 MHz carries a factor of 11, no
// integer prescaler yields exactly 4 MHz. Instead TIM5 (a free-running 32-bit
// timer) is prescaled to exactly 1 MHz and timepoint() returns CNT << 2, which is
// an exact quarter-us value (1 us resolution -- ample for a <= 2 kHz IMU). TIM5's
// ARR is 0x3FFFFFFF so that (CNT << 2) spans the full uint32 range and wraps
// cleanly at 2^32 quarter-us (~1073 s); the host only consumes wrap-safe deltas.
class Timer {
public:
    using Lazy = utility::Lazy<Timer>;

    static constexpr uint32_t kClockFrequency = 4'000'000;
    using TickPeriod = std::ratio<1, kClockFrequency>;

    // 1/4 us
    using Duration = std::chrono::duration<uint32_t, TickPeriod>;
    using TimePoint = std::chrono::time_point<uint32_t, Duration>;

    // Keep the true-window at least half-cycle for stateless expiration checks.
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

    // Busy-wait for the given duration off the free-running TIM5 counter. Reads
    // CNT directly, so it works with interrupts disabled (e.g. during init under an
    // InterruptLockGuard); requires timer.init() to have started TIM5.
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
