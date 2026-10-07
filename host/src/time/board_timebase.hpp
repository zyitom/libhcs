#pragma once

// One board's own quarter-microsecond clock, mapped onto the shared microframe
// axis -- the internal state behind the `board` field of libhcs::time::SampleTime.
//
// One BoardTimebase per link (per Handler), owned by that link's IO thread:
// every board timestamp that arrives -- the time-status reports, the three IMU
// streams, timestamped GPIO -- passes through extend() in arrival order, and
// the time-status reports rebuild the mapping. No locking: single writer that
// is also the only reader.
//
// TWO JOBS.
//
//   extend(): the wire carries 32 bits of quarter microseconds, which wrap
//   every ~18 minutes. The 64-bit extension keeps the previous reading and
//   interprets each new delta as int32 -- small backwards steps (the IMU, GPIO
//   and time-status streams interleave) come out as small negative deltas, and
//   the wrap comes out as +1 step. This is the whole 32-to-64-bit story, in
//   one place.
//
//   observe(): every kTimeStatus pairs a position on the microframe axis (48
//   whole bits plus a 1/65536 fraction) with the board timer's reading at that
//   moment and the board's own fitted rate between the two. One report is one
//   straight line from the board clock to the axis; reports come four times a
//   second, so a timestamp is at most a quarter second from the reference it
//   converts against.

#include <cstdint>

#include <libhcs/data/datas.hpp>
#include <libhcs/time/sample_time.hpp>

#include "host/src/time/affine.hpp"
#include "host/src/time/frame_clock.hpp"

namespace libhcs::host::time {

// The line in a board's time report: `reference_board` is the report's own
// timestamp, already extended onto the 64-bit board axis by BoardTimebase.
// Invalid when the board's time line is not valid (not anchored yet, or the
// counter faulted) or its fit has not converged -- the same criteria the old
// BoardClock::from applied.
[[nodiscard]] inline Affine<libhcs::time::BoardClock, FrameClock> board_map_of(
    const data::TimeStatusView& status,
    libhcs::time::BoardClock::time_point reference_board) noexcept {
    if (status.state != data::TimeState::kValid || status.ticks_per_microframe_q16 == 0)
        return {};
    // FrameClock ticks per board tick: 65536 per microframe, and the wire says
    // how many board ticks one microframe is worth (Q16). Exact for the
    // nominal 500 << 16: 65536^2 / (500 << 16) = 65536/500.
    const double rate = 65536.0 * 65536.0 / static_cast<double>(status.ticks_per_microframe_q16);
    // Whole microframes on the wire are 48 bits; times 65536 they hold an
    // int64 for the first ~208 days after an axis origin.
    const auto to0_ticks =
        static_cast<std::int64_t>(status.microframe) * 65536LL + status.microframe_fraction_q16;
    return Affine<libhcs::time::BoardClock, FrameClock>{
        .from0 = reference_board,
        .to0 = FrameClock::time_point{FrameClock::duration{to0_ticks}},
        .rate = rate,
    };
}

class BoardTimebase {
public:
    // 32-bit reading -> 64-bit quarter-microsecond instant, never wrapping.
    [[nodiscard]] libhcs::time::BoardClock::time_point extend(std::uint32_t raw) noexcept {
        if (!has_last_) {
            has_last_ = true;
            extended_ = raw;
        } else {
            extended_ += static_cast<std::int32_t>(raw - last_raw_);
        }
        last_raw_ = raw;
        return libhcs::time::BoardClock::time_point{
            libhcs::time::BoardClock::duration{static_cast<std::int64_t>(extended_)}};
    }

    // One kTimeStatus: rebuilds the board-clock -> microframe-axis line. The
    // report's own reading is extended like any other board timestamp, so the
    // reference lives on the same 64-bit axis as everything extend() returns.
    void observe(const data::TimeStatusView& status) noexcept {
        map_ = board_map_of(status, extend(status.timestamp_quarter_us));
    }

    // Board quarter-us -> FrameClock ticks. Invalid until the first valid
    // time-status report (and again after reset()).
    [[nodiscard]] const Affine<libhcs::time::BoardClock, FrameClock>& map() const noexcept {
        return map_;
    }

    // A new session reboots the board's fit; the extension base and the map
    // both start over.
    void reset() noexcept {
        has_last_ = false;
        extended_ = 0;
        last_raw_ = 0;
        map_ = {};
    }

private:
    bool has_last_ = false;
    std::uint32_t last_raw_ = 0;
    std::uint64_t extended_ = 0;
    Affine<libhcs::time::BoardClock, FrameClock> map_{};
};

} // namespace libhcs::host::time
