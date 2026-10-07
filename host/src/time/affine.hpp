#pragma once

// One straight line between two clocks, as a plain immutable value.
//
// Affine<From, To> answers "what instant on To's clock is the same instant as
// this one on From's" with a reference pair and a rate -- the only shape any
// two free-running counters of nearly equal rate ever need. It replaces the
// two hand-rolled "reference + slope" structs the codebase used to carry side
// by side (AxisMap for microframe -> host, and the old BoardClock struct for
// board quarter-us -> microframe), which differed only in field names.
//
// PRECISION. Only the offset from the reference crosses a double: the
// reference points are integer clock ticks, and the deltas a fit window
// produces are orders below 2^53, so no mantissa is spent on the offset --
// the same trick the round-trip fit has always used, now written into the
// type. The composition of two maps multiplies two rates and keeps the inner
// map's reference, so a board timestamp converts to host time through one
// multiplication.
//
// VALIDITY. rate == 0 means "no line": an Affine built by default refuses to
// convert rather than answering from a guess, exactly like the AxisMap it
// came from. inverse() and compose() keep that property: inverting or joining
// a map with no line produces a map with no line, never one that answers.

#include <chrono>
#include <cmath>
#include <cstdint>

#include <libhcs/time/sample_time.hpp>

#include "host/src/time/axis_map.hpp"
#include "host/src/time/frame_clock.hpp"

namespace libhcs::host::time {

// The host side of every mapping here: this machine's CLOCK_MONOTONIC.
using HostClock = libhcs::time::HostClock;

template <class From, class To>
struct Affine {
    typename From::time_point from0{};
    typename To::time_point to0{};
    double rate = 0.0; // To ticks per From tick; 0 = no line

    [[nodiscard]] bool valid() const noexcept { return rate > 0.0; }

    [[nodiscard]] typename To::time_point operator()(typename From::time_point t) const noexcept {
        return to0 + typename To::duration{std::llround(double((t - from0).count()) * rate)};
    }

    [[nodiscard]] Affine<To, From> inverse() const noexcept {
        if (!valid())
            return {};
        return Affine<To, From>{.from0 = to0, .to0 = from0, .rate = 1.0 / rate};
    }
};

template <class A, class B, class C>
[[nodiscard]] Affine<A, C> compose(const Affine<B, C>& outer, const Affine<A, B>& inner) noexcept {
    return Affine<A, C>{
        .from0 = inner.from0, .to0 = outer(inner.to0), .rate = inner.rate * outer.rate};
}

// The line an AxisMap carries, in Affine form.
//
// The reference pair lands on the FrameClock tick grid (whole Q16
// microframes), so the host time the old AxisMap::time_of computed for a
// tick-grid position is reproduced to within a nanosecond: the correction on
// to0 absorbs the rounding of the reference itself, and everything else is
// the same multiplication.
[[nodiscard]] inline Affine<FrameClock, HostClock> affine_of(const AxisMap& map) noexcept {
    if (!map.valid())
        return {};
    const auto q16 = static_cast<double>(FrameClock::kTicksPerMicroframe);
    const auto from0_ticks = std::llround(map.reference_microframe * q16);
    // Absorb the rounding of the reference into to0, so the line passes
    // through the same host instant the AxisMap computed there.
    const auto correction_ns = std::llround(
        (static_cast<double>(from0_ticks) - (map.reference_microframe * q16)) / q16
        * map.period_ns);
    return Affine<FrameClock, HostClock>{
        .from0 = FrameClock::time_point{FrameClock::duration{from0_ticks}},
        .to0 = HostClock::time_point{std::chrono::nanoseconds{map.reference_ns}}
             + std::chrono::nanoseconds{correction_ns},
        .rate = map.period_ns / q16,
    };
}

} // namespace libhcs::host::time
