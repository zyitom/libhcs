#pragma once

// One host controller's USB microframe axis, in std::chrono clothing.
//
// Every board on one controller counts the same Start-of-Frame packets, so a
// position on this axis names the same instant for all of them. The clock's
// tick is a microframe divided into 65536 parts (a Q16 microframe): whole
// enough for the 14-bit hardware stamps on the wire (16384 per microframe,
// which divide by exactly 4) and fine enough that a fitted rate lives in the
// multiplier rather than in accumulated rounding.
//
// Internal: applications see conversions through libhcs::time::SampleTime.

#include <chrono>
#include <cstdint>

namespace libhcs::host::time {

struct FrameClock {
    using rep = std::int64_t;
    using period = std::ratio<1, 8000LL * 65536>;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<FrameClock>;
    static constexpr bool is_steady = true;

    // Microframes per second, and therefore ticks in one microframe -- the
    // Q16 fraction the wire's fractional microframes speak. Both exact.
    static constexpr std::int64_t kMicroframesPerSecond = 8000;
    static constexpr std::int64_t kTicksPerMicroframe = period::den / kMicroframesPerSecond;
};

} // namespace libhcs::host::time
