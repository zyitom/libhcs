#pragma once

#include <chrono>
#include <concepts>
#include <cstdint>
#include <limits>
#include <ratio>
#include <type_traits>

extern "C" {
#include "ch32h417.h"
}

#include "core/src/utility/assert.hpp"
#include "firmware/ch32_board/app/src/board_app.hpp"
#include "firmware/ch32_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::timer {

// Timestamp source for telemetry, ported from mc02. The protocol expresses
// timestamps in quarter-microseconds (a 4 MHz tick), shared with c_board and
// hpm_board. TIM10 is used because it is one of the few 32-bit timers on this
// part (TIM2..TIM8 are 16-bit; TIM12 is taken by the USBSS link timer). It is
// prescaled to exactly 1 MHz and timepoint() returns CNT << 2, an exact
// quarter-us value (1 us resolution). The ARR is left at full 32-bit scale so
// (CNT << 2) wraps cleanly and the host consumes only wrap-safe deltas.
//
// NOTE (bring-up): the prescaler is derived from SystemCoreClock assuming the
// TIM2 kernel clock equals SystemCoreClock. If the HB1 timer clock is divided
// down on this part, adjust kTimerKernelClock once measured on target.
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

    // TIM10 base; a plain accessor, not a constexpr member (the WCH TIM10 macro
    // is a reinterpret_cast and cannot be constexpr).
    static TIM_TypeDef* timer_instance() { return TIM10; }
    static constexpr uint32_t kCounterFrequency = 1'000'000; // 1 MHz -> CNT << 2 = 1/4 us

    Timer() { init_timer(); }

    void init() { init_timer(); }

    // CNT_32 is the 32-bit counter view (valid for TIM9..TIM12).
    TimePoint timepoint() const { return TimePoint{Duration{timer_instance()->CNT_32 << 2u}}; }

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

    // Busy-wait for the given duration off the free-running counter. Reads CNT
    // directly, so it works with interrupts disabled.
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
    void init_timer() {
        RCC_HB2PeriphClockCmd(RCC_HB2Periph_TIM10, ENABLE);

        // Timers are clocked from the peripheral bus, NOT the core clock: on the
        // V5F core SystemCoreClock is 400 MHz while HCLK is 100 MHz, so deriving
        // the prescaler from SystemCoreClock made every timestamp run 4x fast.
        // The vendor's own USB_Timer_Init() uses HCLK_Frequency the same way.
        const uint32_t kernel = board::peripheral_clock();
        const uint32_t prescaler = (kernel / kCounterFrequency) - 1u;
        core::utility::assert_always(kernel == (prescaler + 1u) * kCounterFrequency);

        TIM_TimeBaseInitTypeDef base = {};
        base.TIM_Period = 0xFFFFu; // 16-bit struct field; widened to 32-bit below
        base.TIM_Prescaler = static_cast<uint16_t>(prescaler);
        base.TIM_ClockDivision = TIM_CKD_DIV1;
        base.TIM_CounterMode = TIM_CounterMode_Up;
        base.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(timer_instance(), &base);

        // Use the full 32-bit auto-reload so the 1 MHz counter wraps at ~4295 s
        // instead of every 65 ms; the session lease (1 s) and telemetry deltas
        // depend on this wide range.
        timer_instance()->ATRLR_32 = 0xFFFFFFFFu;

        TIM_Cmd(timer_instance(), ENABLE);
    }

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
