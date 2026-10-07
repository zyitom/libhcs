// The affine mapping between two clocks (host/src/time/affine.hpp) and the
// FrameClock the microframe axis sits on: composition, inversion, precision
// across large int64 values, and the factory that reproduces what the old
// AxisMap::time_of computed. Everything runs on constructed numbers.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <random>

#include <gtest/gtest.h>

#include "host/src/time/affine.hpp"
#include "host/src/time/axis_map.hpp"
#include "host/src/time/board_timebase.hpp"
#include "host/src/time/frame_clock.hpp"

namespace {

using libhcs::host::time::Affine;
using libhcs::host::time::affine_of;
using libhcs::host::time::AxisMap;
using libhcs::host::time::compose;
using libhcs::host::time::FrameClock;
using libhcs::host::time::HostClock;
using BoardClock = libhcs::time::BoardClock;
using HostTime = libhcs::time::HostTime;

HostTime at_ns(std::int64_t ns) { return HostTime{std::chrono::nanoseconds{ns}}; }
std::int64_t ns_of(HostTime when) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(when.time_since_epoch()).count();
}
FrameClock::time_point at_ticks(std::int64_t ticks) {
    return FrameClock::time_point{FrameClock::duration{ticks}};
}
std::int64_t ticks_of(FrameClock::time_point when) { return when.time_since_epoch().count(); }
BoardClock::time_point at_quarter_us(std::int64_t quarter_us) {
    return BoardClock::time_point{BoardClock::duration{quarter_us}};
}

constexpr double kQ16 = static_cast<double>(FrameClock::kTicksPerMicroframe);

// ---- FrameClock -------------------------------------------------------------

static_assert(FrameClock::period::num == 1);
static_assert(FrameClock::period::den == 8000LL * 65536);
static_assert(FrameClock::is_steady);

TEST(FrameClock, OneMicroframeIsQ16Ticks) {
    static_assert(FrameClock::kTicksPerMicroframe == 65536);
    const auto one = std::chrono::duration_cast<std::chrono::nanoseconds>(
        FrameClock::duration{FrameClock::kTicksPerMicroframe});
    EXPECT_EQ(one.count(), 125'000);
    static_assert(
        FrameClock::kMicroframesPerSecond * FrameClock::kTicksPerMicroframe
        == FrameClock::period::den);
}

// ---- Affine -----------------------------------------------------------------

TEST(Affine, RefusesToConvertWithoutALine) {
    const Affine<FrameClock, HostClock> none;
    EXPECT_FALSE(none.valid());
    EXPECT_EQ(ns_of(none(at_ticks(1'000'000))), 0); // to0 of an empty map
    // Inverting an invalid map stays invalid instead of inventing a rate.
    EXPECT_FALSE(none.inverse().valid());
}

TEST(Affine, ConvertsThroughReferenceAndRate) {
    const Affine<FrameClock, HostClock> line{
        .from0 = at_ticks(8'000'000LL * 65536),
        .to0 = at_ns(5'000'000'000'000),
        .rate = 125'004.9 / kQ16,
    };
    const auto ticks = static_cast<std::int64_t>(1234.5 * kQ16);
    const auto when = line(at_ticks(8'000'000LL * 65536 + ticks));
    EXPECT_EQ(ns_of(when), 5'000'000'000'000 + std::llround(1234.5 * 125'004.9));
    // And back: the round trip loses only the rounding of one llround.
    const auto back = line.inverse()(when);
    EXPECT_LE(std::abs(ticks_of(back) - (8'000'000LL * 65536 + ticks)), 1);
}

TEST(Affine, InverseRoundTripsAcrossLargeInt64Values) {
    // steady_clock ~1e15 ns, microframe axis days deep (~1e13 Q16 ticks):
    // the regions the real conversions live in, where a naive double fit of
    // absolute values would have burned its mantissa long ago.
    std::mt19937 random{7};
    std::uniform_real_distribution<double> jitter{-0.5, 0.5};
    for (const std::int64_t base_ns :
         {1'000'000'000'000LL, 41'000'000'000'000LL, 9'200'000'000'000LL}) {
        const Affine<FrameClock, HostClock> line{
            .from0 = at_ticks(6'000'000'000'000),
            .to0 = at_ns(base_ns),
            .rate = (125'000.0 * (1.0 + 30e-6)) / kQ16,
        };
        for (const std::int64_t delta_ticks : {0LL, 65536LL, 500'000'000LL, 2'600'000'000'000LL}) {
            const std::int64_t ticks =
                6'000'000'000'000 + delta_ticks + static_cast<std::int64_t>(jitter(random));
            const HostTime when = line(at_ticks(ticks));
            const FrameClock::time_point back = line.inverse()(when);
            // One llround each way: at most a tick of drift across the round trip.
            EXPECT_LE(std::abs(ticks_of(back) - ticks), 1) << base_ns << " + " << delta_ticks;
            // The rate carries the crystal offset: a second apart, the two
            // clocks disagree by ~30 us -- never lost, only scaled.
            const HostTime later = line(at_ticks(ticks + 8000LL * 65536));
            EXPECT_NEAR(ns_of(later) - ns_of(when), 1e9 * (1.0 + 30e-6), 2.0)
                << base_ns << " + " << delta_ticks;
        }
    }
}

TEST(Affine, ComposesLikeTheTwoLinesItJoins) {
    // board -> microframe axis -> host, computed in one hop and in two.
    const Affine<BoardClock, FrameClock> board_to_axis{
        .from0 = at_quarter_us(0xFFFF'F000),
        .to0 = at_ticks(8'000'000LL * 65536 + 32768),
        .rate = 65536.0 * 65536.0 / (500U << 16U),
    };
    const Affine<FrameClock, HostClock> axis_to_host{
        .from0 = at_ticks(8'000'000LL * 65536),
        .to0 = at_ns(42'000'000'000),
        .rate = 125'000.0 / kQ16,
    };
    const auto joined = compose(axis_to_host, board_to_axis);
    ASSERT_TRUE(joined.valid());
    EXPECT_EQ(joined.from0, board_to_axis.from0);

    for (const std::int64_t later : {0LL, 0x1000LL, 0x2000LL, 4'000'000LL}) {
        const auto board = at_quarter_us(0xFFFF'F000 + later);
        EXPECT_EQ(joined(board), axis_to_host(board_to_axis(board))) << later;
    }
}

TEST(Affine, CompositionOfInvalidMapsStaysInvalid) {
    const Affine<BoardClock, FrameClock> invalid;
    const Affine<FrameClock, HostClock> valid{
        .from0 = at_ticks(0), .to0 = at_ns(1'000), .rate = 125'000.0 / kQ16};
    EXPECT_FALSE(compose(valid, invalid).valid());
    // The inverse direction joins the same way and stays invalid too.
    EXPECT_FALSE(compose(invalid.inverse(), valid.inverse()).valid());
}

// ---- the factory from AxisMap -----------------------------------------------

TEST(AffineOf, ReproducesAxisMapTimeOfWithinANanosecond) {
    // Random lines over the ranges the fits produce, probed at tick-grid
    // positions -- where CAN stamps and board-clock reports actually land.
    std::mt19937 random{11};
    std::uniform_real_distribution<double> reference_mf{0.0, 700'000'000.0};
    std::uniform_real_distribution<double> period{125'000.0 * 0.9999, 125'000.0 * 1.0001};
    std::uniform_int_distribution<std::int64_t> reference_ns{1'000'000'000'000, 9'000'000'000'000};
    std::uniform_int_distribution<std::int64_t> probe{-400'000LL * 65536, 400'000LL * 65536};
    for (int i = 0; i < 2'000; ++i) {
        const AxisMap map{
            .source = AxisMap::Source::kController,
            .reference_microframe = reference_mf(random),
            .reference_ns = reference_ns(random),
            .period_ns = period(random),
        };
        const Affine<FrameClock, HostClock> line = affine_of(map);
        ASSERT_TRUE(line.valid());
        const std::int64_t ticks = std::llround(map.reference_microframe * kQ16) + probe(random);
        const auto by_old = map.time_of(static_cast<double>(ticks) / kQ16);
        const auto by_new = line(at_ticks(ticks));
        EXPECT_LE(std::abs(ns_of(by_new) - ns_of(by_old)), 1)
            << "ticks " << ticks << " on line " << i;
    }
}

TEST(AffineOf, InvalidAxisMapGivesAnInvalidLine) { EXPECT_FALSE(affine_of(AxisMap{}).valid()); }

// ---- the factory from a board's TimeStatusView ------------------------------

TEST(BoardMapOf, ReproducesTheOldBoardClockLine) {
    // The old host::time::BoardClock::microframe_at interpreted a reading's
    // int32 delta from the reference and divided by the wire rate. The Affine
    // does the same arithmetic with the delta scaled to Q16 microframes; the
    // two must agree to well under a microframe.
    libhcs::data::TimeStatusView status{};
    status.state = libhcs::data::TimeState::kValid;
    status.microframe = 8'000'000;          // 1000 s into the axis
    status.microframe_fraction_q16 = 32768; // and half a microframe
    status.timestamp_quarter_us = 0xFFFF'F000U;
    status.ticks_per_microframe_q16 = static_cast<std::uint32_t>(500.01 * 65536.0);
    const auto map = libhcs::host::time::board_map_of(status, at_quarter_us(0xFFFF'F000));
    ASSERT_TRUE(map.valid());

    const double old_rate = static_cast<double>(status.ticks_per_microframe_q16) / 65536.0;
    for (const std::int64_t later : {0, 0x1000, 0x2000, 4'000'000, -1'000}) {
        const double by_old = 8'000'000.5 + (static_cast<double>(later) / old_rate);
        const FrameClock::time_point by_new = map(at_quarter_us(0xFFFF'F000 + later));
        const double microframe = static_cast<double>(ticks_of(by_new)) / kQ16;
        // The Affine answers in whole FrameClock ticks (half a tick of
        // rounding = ~1 ns); the old line answered in bare doubles.
        EXPECT_NEAR(microframe, by_old, 1e-5) << later;
    }
}

TEST(BoardMapOf, RefusesAReportWithoutATimeLine) {
    libhcs::data::TimeStatusView status{};
    status.state = libhcs::data::TimeState::kValid;
    status.ticks_per_microframe_q16 = 500U << 16U;
    EXPECT_TRUE(libhcs::host::time::board_map_of(status, at_quarter_us(1)).valid());
    status.ticks_per_microframe_q16 = 0; // fit not converged
    EXPECT_FALSE(libhcs::host::time::board_map_of(status, at_quarter_us(1)).valid());
    status.ticks_per_microframe_q16 = 500U << 16U;
    status.state = libhcs::data::TimeState::kInvalid;
    EXPECT_FALSE(libhcs::host::time::board_map_of(status, at_quarter_us(1)).valid());
    status.state = libhcs::data::TimeState::kWaitingAnchor;
    EXPECT_FALSE(libhcs::host::time::board_map_of(status, at_quarter_us(1)).valid());
}

} // namespace
