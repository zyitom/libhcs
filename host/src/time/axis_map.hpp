#pragma once

// The boards' shared microframe axis, placed on this machine's steady_clock.
//
// An AxisMap is one straight line between the two, as a plain value: take a
// copy, then convert as many timestamps as you like without touching a lock, a
// clock or the Timeline again. That is what a real-time thread needs, and why
// this is a separate header from the Timeline that produces it.
//
// steady_clock is CLOCK_MONOTONIC here -- the clock a control loop schedules
// on and the one `trace-cmd -C mono` records -- so a converted timestamp can
// be subtracted from a loop tick or laid over a kernel trace as it is.
//
// HOW LONG A COPY STAYS GOOD. The line is exact at its reference and drifts
// from the truth at the rate the two clocks' ratio changes, which is the rate
// NTP re-steers CLOCK_MONOTONIC: parts per million per adjustment. Refreshed
// once per control cycle it is as good as the Timeline's own; held for a
// second it is still within a microsecond. Do not keep one across a
// reconnect.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

#include <libhcs/time/sof_stamp.hpp>

namespace libhcs::host::time {

struct AxisMap {
    using Clock = std::chrono::steady_clock;

    enum class Source : std::uint8_t {
        // No line. Every conversion refuses rather than answering from a
        // guess: before the first fit there is no relation between the two
        // axes at all, and an open-loop number would look entirely plausible.
        kNone = 0,
        // Fitted through the USB round trips that carry the boards' reports.
        // Microseconds on a high-speed link, tens of them on a full-speed one.
        kRoundTrip = 1,
        // Read from the host controller's own microframe counter -- the same
        // counter the boards follow, so only an integer offset was learned.
        // Sub-microsecond.
        kController = 2,
    };

    Source source = Source::kNone;
    double reference_microframe = 0.0; // on the boards' axis
    std::int64_t reference_ns = 0;     // steady_clock, since its epoch
    double period_ns = 125'000.0;      // steady_clock nanoseconds per microframe

    [[nodiscard]] constexpr bool valid() const noexcept { return source != Source::kNone; }

    // Requires valid().
    [[nodiscard]] double microframe_at(Clock::time_point when) const noexcept {
        const auto ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(when.time_since_epoch()).count();
        return reference_microframe + (static_cast<double>(ns - reference_ns) / period_ns);
    }

    // Requires valid().
    [[nodiscard]] Clock::time_point time_of(double microframe) const noexcept {
        const double ns = (microframe - reference_microframe) * period_ns;
        return Clock::time_point{
            std::chrono::nanoseconds{reference_ns + static_cast<std::int64_t>(std::llround(ns))}};
    }

    // When a stamped record was taken. `around` is the caller's idea of the
    // present -- the arrival time, or the start of the control cycle that is
    // handling the record; see libhcs::time::unwrap() for how far off it may
    // be (three quarters of 2 s behind the record's arrival, a quarter ahead).
    // Empty without a map.
    [[nodiscard]] std::optional<Clock::time_point>
        time_of(libhcs::time::SofStamp stamp, Clock::time_point around) const noexcept {
        if (!valid())
            return std::nullopt;
        constexpr auto ticks = static_cast<double>(libhcs::time::SofStamp::kTicksPerMicroframe);
        const auto around_ticks =
            static_cast<std::int64_t>(std::llround(microframe_at(around) * ticks));
        const std::int64_t position = libhcs::time::unwrap(stamp, around_ticks);
        return time_of(static_cast<double>(position) / ticks);
    }
};

} // namespace libhcs::host::time
