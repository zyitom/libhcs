#pragma once

// The conversion from "what the record carries" to libhcs::time::SampleTime:
// pure functions of the record's stamp, the arrival time and the two current
// mappings. Nothing here reads a clock or takes a lock, so the unit test feeds
// constructed data directly (host/tests/sample_time_test.cpp).
//
// The caller (the protocol handler's IO thread) reads steady_clock ONCE per
// USB receive transfer and hands the same instant to every record in it.

#include <optional>

#include <libhcs/time/sample_time.hpp>
#include <libhcs/time/sof_stamp.hpp>

#include "host/src/time/affine.hpp"
#include "host/src/time/frame_clock.hpp"

namespace libhcs::host::time {

// A CAN frame (stamped on the microframe axis) or a UART chunk (never
// stamped). The 24-bit stamp is unwrapped against where the axis says the
// arrival sits -- a better reference than the arrival time alone, because it
// is on the stamps' own axis -- and then placed on the host clock through the
// axis. `board` stays empty either way: a CAN stamp lives on the microframe
// axis, not on the board's quarter-microsecond timer.
[[nodiscard]] inline libhcs::time::SampleTime time_of_stamped(
    std::optional<libhcs::time::SofStamp> stamp, libhcs::time::HostTime arrival,
    const Affine<FrameClock, HostClock>& axis) noexcept {
    if (!stamp || !axis.valid())
        return {.host = arrival, .source = libhcs::time::SampleTime::Source::kHostArrival};
    // The axis speaks Q16 microframes; the stamp speaks 1/16384 microframes.
    const auto around_ticks = axis.inverse()(arrival).time_since_epoch().count() / 4;
    const std::int64_t position = libhcs::time::unwrap(*stamp, around_ticks);
    const auto frame = FrameClock::time_point{FrameClock::duration{position * 4}};
    return {.host = axis(frame), .source = libhcs::time::SampleTime::Source::kBoardStamp};
}

// An IMU sample or a timestamped GPIO input: stamped with the board's own
// quarter-microsecond timer, which `board` (already extended by the caller's
// BoardTimebase) places on the 64-bit board axis. When the board clock and
// the microframe axis are both locked, `host` converts the stamp; otherwise
// it falls back to the arrival time and says so, while `board` is kept --
// the board's own timeline needs no synchronization and always works.
[[nodiscard]] inline libhcs::time::SampleTime time_of_board_ticks(
    std::optional<libhcs::time::BoardClock::time_point> board, libhcs::time::HostTime arrival,
    const Affine<FrameClock, HostClock>& axis,
    const Affine<libhcs::time::BoardClock, FrameClock>& board_map) noexcept {
    if (board && axis.valid() && board_map.valid())
        return {
            .host = compose(axis, board_map)(*board),
            .source = libhcs::time::SampleTime::Source::kBoardStamp,
            .board = *board};
    return {
        .host = arrival, .source = libhcs::time::SampleTime::Source::kHostArrival, .board = board};
}

} // namespace libhcs::host::time
