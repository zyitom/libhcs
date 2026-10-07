// The conversions the SDK's IO thread performs at decode time
// (host/src/time/sample_timer.hpp), and the per-link board clock they lean on
// (host/src/time/board_timebase.hpp). Four cases decide what a SampleTime
// means -- a CAN frame with a stamp, an IMU sample with a board stamp, a
// record arriving before the mappings locked, and a UART chunk that never
// carries one -- plus the 32-bit wrap of the board's own timer.

#include <chrono>
#include <cstdint>
#include <optional>
#include <random>

#include <gtest/gtest.h>

#include <libhcs/time/sample_time.hpp>
#include <libhcs/time/sof_stamp.hpp>

#include "host/src/time/affine.hpp"
#include "host/src/time/board_timebase.hpp"
#include "host/src/time/frame_clock.hpp"
#include "host/src/time/sample_timer.hpp"

namespace {

using libhcs::host::time::Affine;
using libhcs::host::time::board_map_of;
using libhcs::host::time::BoardTimebase;
using libhcs::host::time::FrameClock;
using libhcs::host::time::HostClock;
using libhcs::host::time::time_of_board_ticks;
using libhcs::host::time::time_of_stamped;
using libhcs::time::SampleTime;
using libhcs::time::SofStamp;
using HostTime = libhcs::time::HostTime;
using BoardTime = libhcs::time::BoardClock::time_point;

HostTime at_ns(std::int64_t ns) { return HostTime{std::chrono::nanoseconds{ns}}; }
std::int64_t ns_of(HostTime when) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(when.time_since_epoch()).count();
}
BoardTime at_quarter_us(std::int64_t quarter_us) {
    return BoardTime{libhcs::time::BoardClock::duration{quarter_us}};
}

// The axis every case converts against: microframe m happened at
// base_ns + m * 125 us, with a small crystal offset in the rate.
constexpr double kPeriodNs = 125'003.0;
Affine<FrameClock, HostClock> locked_axis(std::int64_t base_ns, double base_mf) {
    return Affine<FrameClock, HostClock>{
        .from0 = FrameClock::time_point{FrameClock::duration{
            static_cast<std::int64_t>(std::llround(base_mf * 65536.0))}},
        .to0 = at_ns(base_ns),
        .rate = kPeriodNs / 65536.0,
    };
}

// One valid time report, pairing board quarter-us `board_raw` (unwrapped by
// the caller into the 64-bit board axis) with microframe position `mf`.
BoardTimebase timebase_with(libhcs::data::TimeStatusView status) {
    BoardTimebase timebase;
    timebase.observe(status);
    return timebase;
}

libhcs::data::TimeStatusView valid_status(
    std::uint64_t microframe, std::uint16_t fraction_q16, std::uint32_t board_quarter_us,
    std::uint32_t ticks_per_microframe_q16) {
    libhcs::data::TimeStatusView status{};
    status.state = libhcs::data::TimeState::kValid;
    status.microframe = microframe;
    status.microframe_fraction_q16 = fraction_q16;
    status.timestamp_quarter_us = board_quarter_us;
    status.ticks_per_microframe_q16 = ticks_per_microframe_q16;
    return status;
}

// ---- CAN: a frame stamped on the microframe axis -----------------------------

TEST(SampleTime, CanStampLandsOnTheHostClock) {
    constexpr std::int64_t base_ns = 77'000'000'000'000;
    constexpr double base_mf = 123'456'789.0;
    const auto axis = locked_axis(base_ns, base_mf);

    for (const double ahead : {0.0, 17.25, 1023.999, 1024.0, 4000.75, 50'000.125}) {
        const double microframe = base_mf + ahead;
        // The stamp is the position modulo 2^24 ticks, to the nearest stamp
        // tick -- which is all the wire carries, so the truth to check against
        // is the quantized position.
        const auto position =
            static_cast<std::int64_t>(std::llround(microframe * SofStamp::kTicksPerMicroframe));
        const SofStamp stamp{static_cast<std::uint32_t>(position) & SofStamp::kMask};
        const double stamped_mf = static_cast<double>(position) / SofStamp::kTicksPerMicroframe;
        const auto truth = at_ns(base_ns + std::llround((stamped_mf - base_mf) * kPeriodNs));

        // Arrivals from "immediately" to "well past the documented 96 ms
        // unwrap window" all resolve to the same instant.
        for (const auto late :
             {std::chrono::microseconds{0}, std::chrono::microseconds{300},
              std::chrono::microseconds{20'000}, std::chrono::microseconds{90'000}}) {
            const SampleTime time = time_of_stamped(stamp, truth + late, axis);
            EXPECT_EQ(time.source, SampleTime::Source::kBoardStamp);
            EXPECT_EQ(time.host, truth)
                << "record " << ahead << " microframes in, seen " << late.count() << " us later";
            EXPECT_FALSE(time.board.has_value()) << "a CAN stamp is not on the board clock";
        }
        // A control loop passes the START of its cycle; the record may have
        // arrived a little after that.
        const SampleTime time = time_of_stamped(stamp, truth - std::chrono::milliseconds{2}, axis);
        EXPECT_EQ(time.host, truth);
    }
}

// ---- IMU: a sample stamped with the board's own timer ------------------------

TEST(SampleTime, BoardStampConvertsThroughBothMappings) {
    constexpr std::int64_t base_ns = 42'000'000'000'000;
    constexpr double base_mf = 8'000'000.0;
    const auto axis = locked_axis(base_ns, base_mf);
    // The board says: at board clock 0xFFFF'F000 it was microframe 8'000'000.5,
    // and its timer runs 500.01 ticks per microframe -- a Q16 count on the
    // wire, so the rate the mapping uses is the truncated integer.
    const auto ticks_q16 = static_cast<std::uint32_t>(500.01 * 65536.0);
    const auto timebase = timebase_with(valid_status(8'000'000, 32768, 0xFFFF'F000U, ticks_q16));
    ASSERT_TRUE(timebase.map().valid());

    const double board_ticks_per_mf = static_cast<double>(ticks_q16) / 65536.0;
    for (const std::int64_t later : {0, 0x1000, 0x2000, 4'000'000}) {
        const auto board = at_quarter_us(0xFFFF'F000 + later);
        const SampleTime time = time_of_board_ticks(board, at_ns(1), axis, timebase.map());
        EXPECT_EQ(time.source, SampleTime::Source::kBoardStamp);
        ASSERT_TRUE(time.board.has_value());
        EXPECT_EQ(*time.board, board) << "the 64-bit board time rides along unchanged";
        // Half a microframe past the reference, plus the board ticks elapsed
        // (0x1000 wraps the board's 32-bit reading -- the caller extended it).
        const double expected_ns =
            static_cast<double>(base_ns) + ((0.5 + (later / board_ticks_per_mf)) * kPeriodNs);
        EXPECT_NEAR(static_cast<double>(ns_of(time.host)), expected_ns, 4.0) << later;
    }
}

// The intervals an IMU integrator wants: differences of `board` never lie,
// even while the host-side conversion is still falling back to arrival times.
TEST(SampleTime, BoardIntervalsWorkBeforeTheAxisLocks) {
    BoardTimebase timebase;                            // nothing observed: the board map is invalid
    const auto first = timebase.extend(0xFFFF'F000U);
    const auto second = timebase.extend(0x0000'1000U); // across the 32-bit wrap

    const auto t1 = time_of_board_ticks(first, at_ns(1'000), locked_axis(1, 1), timebase.map());
    const auto t2 = time_of_board_ticks(second, at_ns(1'001), locked_axis(1, 1), timebase.map());
    EXPECT_EQ(t1.source, SampleTime::Source::kHostArrival);
    EXPECT_EQ(t2.source, SampleTime::Source::kHostArrival);
    EXPECT_EQ(t1.host, at_ns(1'000));
    EXPECT_EQ(t2.host, at_ns(1'001));
    ASSERT_TRUE(t1.board.has_value());
    ASSERT_TRUE(t2.board.has_value());
    // 0x2000 quarter microseconds, taken straight off the board's timeline.
    EXPECT_EQ(*t2.board - *t1.board, libhcs::time::BoardClock::duration{0x2000});
}

// ---- falling back, and UART --------------------------------------------------

TEST(SampleTime, UnlockedMappingsFallBackToTheArrivalTime) {
    const auto arrival = at_ns(5'000'000'000);
    const Affine<FrameClock, HostClock> no_axis;
    const BoardTimebase no_board_map;

    // A CAN stamp nobody can place yet.
    const auto can = time_of_stamped(SofStamp{123}, arrival, no_axis);
    EXPECT_EQ(can.source, SampleTime::Source::kHostArrival);
    EXPECT_EQ(can.host, arrival);
    EXPECT_FALSE(can.board.has_value());

    // A frame with no stamp at all.
    const auto unstamped = time_of_stamped(std::nullopt, arrival, locked_axis(1, 1));
    EXPECT_EQ(unstamped.source, SampleTime::Source::kHostArrival);
    EXPECT_EQ(unstamped.host, arrival);

    // An IMU sample: the host time falls back, the board time is kept.
    const auto imu =
        time_of_board_ticks(at_quarter_us(4'000), arrival, no_axis, no_board_map.map());
    EXPECT_EQ(imu.source, SampleTime::Source::kHostArrival);
    EXPECT_EQ(imu.host, arrival);
    ASSERT_TRUE(imu.board.has_value());
    EXPECT_EQ(*imu.board, at_quarter_us(4'000));

    // UART: no stamp, nothing to keep.
    const auto uart = SampleTime{.host = arrival};
    EXPECT_EQ(uart.source, SampleTime::Source::kHostArrival);
    EXPECT_FALSE(uart.board.has_value());
}

TEST(SampleTime, ValueSemanticsForTheCallbackParameter) {
    const auto time = SampleTime{
        .host = at_ns(7), .source = SampleTime::Source::kBoardStamp, .board = at_quarter_us(9)};
    const auto copy = time; // a plain copyable value: what a callback receives
    EXPECT_EQ(copy, time);
    EXPECT_NE(copy, SampleTime{.host = at_ns(7)});
}

// ---- BoardTimebase: the 32-bit wrap, and the streams interleaving ------------

TEST(BoardTimebase, ExtendsAcrossThe32BitWrap) {
    BoardTimebase timebase;
    EXPECT_EQ(timebase.extend(0xFFFF'F000U), at_quarter_us(0xFFFF'F000));
    EXPECT_EQ(timebase.extend(0xFFFF'FF00U), at_quarter_us(0xFFFF'FF00));
    // The wrap: the raw reading falls back to nearly zero, the 64-bit axis
    // keeps counting.
    EXPECT_EQ(timebase.extend(0x0000'0100U), at_quarter_us(0x1'0000'0100));
    EXPECT_EQ(timebase.extend(0x0000'0400U), at_quarter_us(0x1'0000'0400));
}

TEST(BoardTimebase, ToleratesSmallBackwardsSteps) {
    BoardTimebase timebase;
    (void)timebase.extend(100'000);
    // The IMU, GPIO and time-status streams interleave: a reading a few
    // hundred us older than the previous one is a small negative delta, not a
    // jump back by 18 minutes.
    EXPECT_EQ(timebase.extend(99'900), at_quarter_us(99'900));
    EXPECT_EQ(timebase.extend(100'100), at_quarter_us(100'100));
}

TEST(BoardTimebase, StreamsInterleaveWithoutLosingTheBase) {
    BoardTimebase timebase;
    // An accelerometer reading just before the wrap, a time-status reference
    // just after it, then a gyroscope reading between the two -- in arrival
    // order every raw reading lands where the board's own timeline says,
    // whatever stream it came from.
    (void)timebase.extend(0xFFFF'FF00U);
    const auto reference = timebase.extend(0x0000'0100U);
    EXPECT_EQ(reference, at_quarter_us(0x1'0000'0100));
    const auto between = timebase.extend(0xFFFF'FF80U);
    EXPECT_EQ(between, at_quarter_us(0xFFFF'FF80));
}

TEST(BoardTimebase, ObserveBuildsTheMappingFromTheExtendedReference) {
    BoardTimebase timebase;
    // A reading from before the reference lands behind it, not 18 minutes
    // ahead: the reference itself went through extend(), so the pairing is on
    // the same 64-bit axis as every other reading.
    (void)timebase.extend(0xFFFF'F000U);
    timebase.observe(valid_status(8'000'000, 0, 0x0000'0000U, 500U << 16U));
    ASSERT_TRUE(timebase.map().valid());

    // The reference: board raw 0 (extended: 0x1'0000'0000) = microframe 8e6.
    const auto reference = timebase.extend(0x0000'0000U);
    (void)reference;
    // One microframe is 500 board ticks: a reading 500 ticks later sits one
    // microframe ahead on the axis.
    const auto later = timebase.extend(0x0000'01F4U);
    const auto mapped = timebase.map()(later);
    EXPECT_NEAR(
        static_cast<double>(mapped.time_since_epoch().count()) / 65536.0, 8'000'001.0, 1e-6);
}

TEST(BoardTimebase, ResetStartsTheExtensionAndTheMappingOver) {
    BoardTimebase timebase;
    (void)timebase.extend(0xFFFF'F000U);
    timebase.observe(valid_status(8'000'000, 0, 0x0000'0100U, 500U << 16U));
    ASSERT_TRUE(timebase.map().valid());

    timebase.reset();
    EXPECT_FALSE(timebase.map().valid()) << "a new session reboots the board's fit";
    // A fresh base: the first raw reading after the reset is taken as it is,
    // not continued from the old session's 64-bit axis.
    EXPECT_EQ(timebase.extend(123), at_quarter_us(123));
}

TEST(BoardTimebase, AnInvalidReportRevokesTheMapping) {
    BoardTimebase timebase;
    timebase.observe(valid_status(8'000'000, 0, 100, 500U << 16U));
    ASSERT_TRUE(timebase.map().valid());

    libhcs::data::TimeStatusView invalid = valid_status(8'100'000, 0, 200, 500U << 16U);
    invalid.state = libhcs::data::TimeState::kInvalid;
    timebase.observe(invalid);
    EXPECT_FALSE(timebase.map().valid())
        << "the board revoked its time line; converting its stamps would be a guess";
}

// ---- the whole chain, end to end ---------------------------------------------

// A board whose clock and axis agree: two reports 1 s apart build a mapping,
// and an IMU sample taken between them lands within a microsecond of where
// the board's own timeline says it should.
TEST(SampleTime, EndToEndBoardStampLandsWhereTheTimelineSays) {
    constexpr std::int64_t base_ns = 9'000'000'000'000;
    // Board and host agree on the rate to the last count the wire carries:
    // 500 board ticks per microframe, 125 us per microframe.
    constexpr double ticks_per_mf = 500.0;
    BoardTimebase timebase;
    std::mt19937 random{3};
    std::uniform_int_distribution<std::uint32_t> jitter{0, 3}; // quantisation of one report

    double mf = 4'000'000.0;
    std::uint64_t board = 1'000'000'000; // 250 s into the board's timer
    for (int i = 0; i < 8; ++i) {
        timebase.observe(valid_status(
            static_cast<std::uint64_t>(mf), jitter(random), static_cast<std::uint32_t>(board),
            static_cast<std::uint32_t>(ticks_per_mf * 65536.0)));
        mf += 8000.0;
        board += static_cast<std::uint64_t>(8000.0 * ticks_per_mf);
    }
    ASSERT_TRUE(timebase.map().valid());

    const auto axis = locked_axis(base_ns, 4'000'000.0);
    const double sample_ahead_mf = 40'000.0; // 5 s after the first report
    const auto sample_board =
        at_quarter_us(1'000'000'000 + static_cast<std::int64_t>(sample_ahead_mf * ticks_per_mf));
    const auto time = time_of_board_ticks(sample_board, at_ns(base_ns + 1), axis, timebase.map());
    ASSERT_EQ(time.source, SampleTime::Source::kBoardStamp);

    const double truth_ns = static_cast<double>(base_ns) + (sample_ahead_mf * kPeriodNs);
    // The Q16 fraction the report jittered over is a sub-microsecond error.
    EXPECT_NEAR(static_cast<double>(ns_of(time.host)), truth_ns, 4.0);
    EXPECT_EQ(time.board, sample_board);
}

} // namespace
