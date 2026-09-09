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
        // The running core's machine timer (mchtmr0 on core0 boards, mchtmr1
        // on the ECAT bridge's fieldbus core) must be clocked at 4 MHz; every
        // board configures the divider accordingly.
        core::utility::assert_always(
            clock_get_frequency(board::kMchtmrClockName) == kTimerFrequencyHz);

        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
        next_tick_compare_value_ = timestamp64_quarter_us() + kTickPeriodTicks;
        mchtmr_set_compare_value(HPM_MCHTMR, next_tick_compare_value_);

        enable_mchtmr_irq();
    }

    // Deliberately minimal: the machine timer interrupt (MTIP) bypasses the
    // PLIC priority threshold, so once the nested-IRQ wrapper re-enables
    // mstatus.MIE it can preempt even the priority-3 CAN ISR. Any work done
    // here is worst-case latency added to the forwarding hot path -- the LED
    // bookkeeping is therefore driven from the main loop off tick_count().
    void irq_handler() {
        tick_counter_.store(
            tick_counter_.load(std::memory_order::relaxed) + 1, std::memory_order::relaxed);

        do {
            next_tick_compare_value_ += kTickPeriodTicks;
        } while (next_tick_compare_value_ <= timestamp64_quarter_us());
        mchtmr_set_compare_value(HPM_MCHTMR, next_tick_compare_value_);
    }

    // 1 kHz tick counter for main-loop paced work (LED patterns).
    uint32_t tick_count() const { return tick_counter_.load(std::memory_order::relaxed); }

    static uint32_t timestamp_quarter_us() {
        // Read MTIME through its 32-bit MMIO view to avoid aliasing the SDK's uint64_t field.
        const volatile uint32_t* const mtime_words =
            reinterpret_cast<volatile uint32_t*>(HPM_MCHTMR_BASE);
        return mtime_words[0];
    }

    static uint64_t timestamp64_quarter_us() {
        // On riscv32, reading the 64-bit MTIME MMIO register compiles to two 32-bit loads.
        // Read high-low-high and retry on rollover so the returned 64-bit timestamp is stable.
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
