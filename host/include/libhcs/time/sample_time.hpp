#pragma once

// When an uplink sample was taken, as one value handed to the application's
// callback beside the data (host/TIME_DESIGN.md).
//
// The SDK's IO thread computes it while the record is decoded, so the
// application never touches a time object, a lock or a clock: it copies this
// value and answers two different questions with it.
//
//   HOW OLD IS THE DATA. `host` is the sample's moment on this machine's
//   CLOCK_MONOTONIC -- the clock a control loop schedules on and the one
//   `trace-cmd -C mono` records. Always present: when the record carried no
//   timestamp usable for host-time conversion (no stamp, the time base not
//   enabled, or the axis not locked yet) it falls back to the record's arrival
//   time, which carries USB jitter (0.1-1 ms) -- and `source` says so, rather
//   than leaving the precision for the caller to guess.
//
//   HOW LONG BETWEEN TWO SAMPLES OF ONE BOARD. `board` is the moment on that
//   board's own quarter-microsecond timer, unwrapped to 64 bits so it never
//   goes backwards at the 32-bit wrap (~18 min). Present for everything the
//   board timestamps with that timer -- onboard IMU samples, timestamped GPIO
//   -- absent for CAN frames (stamped on the USB microframe axis instead, see
//   CanDataView::sof_stamp) and UART (no stamp at all). Subtracting two of
//   these is the dt an IMU integrator wants: it does not go through any host
//   conversion, so it is exact even during the first seconds before the time
//   base locks.

#include <chrono>
#include <cstdint>
#include <optional>

namespace libhcs::time {

// This machine's CLOCK_MONOTONIC: what hcs_sync::Tick and `trace-cmd -C mono`
// sit on. Same type as std::chrono::steady_clock::time_point.
using HostClock = std::chrono::steady_clock;
using HostTime = HostClock::time_point;

// One board's own timer, in quarter microseconds, unwrapped: the 32-bit
// readings on the wire are extended to a 64-bit count by the SDK (per link),
// so a time_point of this clock is monotonic across the hardware wrap.
struct BoardClock {
    using rep = std::int64_t;
    using period = std::ratio<1, 4'000'000>;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<BoardClock>;
    static constexpr bool is_steady = true;
};

struct SampleTime {
    // How much `host` is worth. A board stamp places the sample to about a
    // microsecond; an arrival time carries USB scheduling jitter.
    enum class Source : std::uint8_t {
        kHostArrival = 0, // `host` is when the record reached this machine
        kBoardStamp = 1,  // `host` is converted from the board's own timestamp
    };

    HostTime host{};
    Source source = Source::kHostArrival;
    // On the board's own clock (64-bit, unwrapped). IMU samples and timestamped
    // GPIO always carry one; CAN frames and UART never do. Defaulted so a
    // designated initializer can leave it out.
    std::optional<BoardClock::time_point> board = std::nullopt;

    friend constexpr bool operator==(const SampleTime&, const SampleTime&) = default;
};

} // namespace libhcs::time
