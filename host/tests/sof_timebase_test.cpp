// The shared time base's arithmetic (core/src/time/sof_timebase.hpp), the one copy all three
// boards run in their SOF interrupt and main loop. Here it is fed a simulated bus whose local
// clock is known exactly, so the fit, the wrap resolution and the conversions along the line
// can be checked against the right answer rather than against themselves.

#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/time/sof_timebase.hpp"

namespace {

using libhcs::core::time::kMicroframeModulus;
using libhcs::core::time::kSampleCount;
using libhcs::core::time::kSampleDecimation;
using libhcs::core::time::SofTimebase;
using libhcs::data::TimeState;

// hpm_board: the fit clock is the protocol's quarter-us (4 MHz), nothing to convert.
struct QuarterUs {
    static constexpr std::uint32_t kNominalTicksPerMicroframe = 500U;
    static constexpr std::uint64_t kResidualAbsMaxLimit = 0xFFFFFFFFU;
    static constexpr std::int64_t to_quarter_us(std::int64_t ticks) { return ticks; }
};

// mc02: the fit clock is the 550 MHz cycle counter, published in quarter-us.
struct Cycles550 {
    static constexpr std::uint32_t kNominalTicksPerMicroframe = 68'750U;
    static constexpr std::uint64_t kResidualAbsMaxLimit = std::numeric_limits<std::uint64_t>::max();
    static constexpr std::int64_t to_quarter_us(std::int64_t cycles) { return cycles * 4 / 550; }
};

constexpr std::uint64_t kHostBase = 16384ULL * 4242; // the absolute axis's origin, a whole wrap

// One board on a bus: the hardware microframe counter and a local clock running at a known
// rate. sof() delivers one SOF interrupt the way the board's note_sof() does.
template <typename Policy>
struct Board {
    SofTimebase<Policy> timebase;
    std::uint32_t step = 1;              // microframes per SOF interrupt
    std::uint64_t microframe = 1000;     // unwrapped hardware microframe
    std::uint64_t clock_at_zero = 7'000; // local clock at microframe 0
    std::uint64_t ticks_q16 = 500ULL << 16U;
    std::uint32_t tick_ms = 0;

    [[nodiscard]] std::uint64_t clock_at(std::uint64_t at) const {
        return clock_at_zero + ((at * ticks_q16) >> 16U);
    }

    void sof(std::uint32_t advance, std::int64_t clock_error = 0) {
        microframe += advance;
        const auto now = static_cast<std::uint32_t>(clock_at(microframe) + clock_error);
        timebase.note_time(now);
        if (timebase.note_microframe(static_cast<std::uint32_t>(microframe) & 0x3FFFU, step))
            timebase.note_sample(now);
    }

    // Runs the bus long enough to fill the fit window, polling every millisecond.
    void run(std::uint64_t microframes) {
        for (std::uint64_t done = 0; done < microframes; done += step) {
            sof(step);
            if (microframe % 8 == 0)
                poll();
        }
    }

    void poll() {
        if (!timebase.fit_due(++tick_ms))
            return;
        typename SofTimebase<Policy>::Window window;
        timebase.copy_window(window);
        if (const auto fit = SofTimebase<Policy>::compute_fit(window))
            timebase.publish_fit(*fit);
    }

    [[nodiscard]] std::uint64_t absolute() const { return kHostBase + microframe; }
};

constexpr std::uint64_t kWindow = static_cast<std::uint64_t>(kSampleCount) * kSampleDecimation;

TEST(SofTimebase, AnsweringNeedsBothAFitAndAnAnchor) {
    Board<QuarterUs> board;
    EXPECT_EQ(board.timebase.snapshot().state, TimeState::kInvalid);

    board.run(64);
    EXPECT_EQ(board.timebase.snapshot().state, TimeState::kWaitingAnchor);
    board.timebase.apply_anchor(board.absolute());
    EXPECT_FALSE(board.timebase.answers()) << "anchored, but no fit yet";

    board.run(kWindow + 320); // a full window plus a fit period
    EXPECT_EQ(board.timebase.snapshot().state, TimeState::kValid);
    EXPECT_TRUE(board.timebase.answers());
    EXPECT_EQ(board.timebase.current_microframe(), board.absolute());
}

TEST(SofTimebase, AnExactClockFitsExactlyAndConvertsBothWays) {
    Board<QuarterUs> board;
    board.ticks_q16 = (500ULL << 16U) + 33; // 1 ppm fast: a slope that is not nominal
    board.run(kWindow + 400);
    board.timebase.apply_anchor(board.absolute());
    ASSERT_TRUE(board.timebase.answers());

    const auto snapshot = board.timebase.snapshot();
    EXPECT_EQ(snapshot.ticks_per_microframe_q16, board.ticks_q16);

    // Along the line, both directions, a second ahead of the fit.
    for (std::uint64_t ahead : {0ULL, 1ULL, 999ULL, 8000ULL}) {
        const std::uint64_t at = board.microframe + ahead;
        const auto expected = static_cast<std::int64_t>(board.clock_at(at));
        EXPECT_NEAR(
            static_cast<double>(board.timebase.time_of(kHostBase + at)),
            static_cast<double>(expected), 1.0)
            << ahead;
        // Whole microframes at a time: a quarter of the way into microframe `at` is `at`.
        EXPECT_EQ(board.timebase.microframe_at(expected + 125), kHostBase + at) << ahead;
    }

    // Halfway between two microframes: the fraction says so.
    std::uint64_t microframe = 0;
    std::uint16_t fraction = 0;
    const auto midway = static_cast<std::int64_t>(board.clock_at(board.microframe)) + 250;
    ASSERT_TRUE(board.timebase.microframe_q16_at(midway, microframe, fraction));
    EXPECT_EQ(microframe, board.absolute());
    EXPECT_NEAR(fraction, 0x8000, 8);
}

TEST(SofTimebase, ALocalClockThatWrapsInsideTheWindowStillFitsExactly) {
    Board<QuarterUs> board;
    // The 32-bit clock wraps half way through the first window.
    board.clock_at_zero = 0xFFFFFFFFULL - (board.microframe + kWindow / 2) * 500ULL;
    board.run(kWindow + 400);
    board.timebase.apply_anchor(board.absolute());
    ASSERT_TRUE(board.timebase.answers());
    EXPECT_EQ(board.timebase.snapshot().ticks_per_microframe_q16, 500U << 16U);
    EXPECT_GT(board.timebase.latest_time(), 0xFFFFFFFFULL) << "extended past the wrap";
    EXPECT_EQ(
        board.timebase.microframe_at(static_cast<std::int64_t>(board.clock_at(board.microframe))),
        board.absolute());
}

TEST(SofTimebase, TheAnchorResolvesTheWrapNearestTheHostsEstimate) {
    Board<QuarterUs> board;
    board.run(100);
    // The host is off by a little under half a wrap either way: the counter still lands
    // on the right multiple of 16384.
    board.timebase.apply_anchor(board.absolute() + 8000);
    EXPECT_EQ(board.timebase.current_microframe(), board.absolute());
    board.timebase.apply_anchor(board.absolute() - 8000);
    EXPECT_EQ(board.timebase.current_microframe(), board.absolute());
    EXPECT_EQ(board.timebase.snapshot().anomaly_count, 0U);
}

TEST(SofTimebase, AnAnchorThatMovesTheWrapInvalidatesTheTimeline) {
    Board<QuarterUs> board;
    board.run(kWindow + 400);
    board.timebase.apply_anchor(board.absolute());
    ASSERT_TRUE(board.timebase.answers());

    board.timebase.apply_anchor(board.absolute() + kMicroframeModulus);
    EXPECT_EQ(board.timebase.snapshot().state, TimeState::kInvalid);
    EXPECT_EQ(board.timebase.snapshot().anomaly_count, 1U);
    EXPECT_FALSE(board.timebase.answers());
}

TEST(SofTimebase, AFewMissedInterruptsCostTheWindowNotTheTimeline) {
    Board<QuarterUs> board;
    board.run(kWindow + 400);
    board.timebase.apply_anchor(board.absolute());
    ASSERT_TRUE(board.timebase.answers());

    board.sof(3); // two SOF interrupts missed
    const auto snapshot = board.timebase.snapshot();
    EXPECT_EQ(snapshot.anomaly_count, 1U);
    EXPECT_EQ(snapshot.state, TimeState::kValid) << "the counter is still right";
    EXPECT_EQ(snapshot.microframe, board.absolute());
    EXPECT_FALSE(board.timebase.answers()) << "but the fit window is gone";

    board.run(kWindow + 400);
    EXPECT_TRUE(board.timebase.answers());
}

TEST(SofTimebase, AStuckCounterInvalidates) {
    Board<QuarterUs> board;
    board.run(kWindow + 400);
    board.timebase.apply_anchor(board.absolute());
    board.sof(0);
    EXPECT_EQ(board.timebase.snapshot().state, TimeState::kInvalid);
    EXPECT_FALSE(board.timebase.answers());
}

TEST(SofTimebase, ASlopeFarFromNominalIsNeverPublished) {
    Board<QuarterUs> board;
    board.ticks_q16 = 520ULL << 16U; // 4% fast: not a crystal
    board.run(kWindow + 400);
    board.timebase.apply_anchor(board.absolute());
    EXPECT_FALSE(board.timebase.answers());
    EXPECT_EQ(board.timebase.snapshot().ticks_per_microframe_q16, 0U);
}

TEST(SofTimebase, ResidualsAreForecastErrorsAgainstThePublishedFit) {
    Board<QuarterUs> board;
    board.run(kWindow + 400);
    board.timebase.apply_anchor(board.absolute());
    board.timebase.clear_residuals();

    // Run to the next sample and make that one 100 ticks late.
    while ((board.microframe + 1) % kSampleDecimation != 0)
        board.sof(1);
    board.sof(1, 100);
    const auto snapshot = board.timebase.snapshot();
    EXPECT_EQ(snapshot.residual_count, 1U);
    EXPECT_EQ(snapshot.residual_mean_q16, -(100 << 16));
    EXPECT_EQ(snapshot.residual_abs_max_q16, 100U << 16U);

    board.timebase.clear_residuals();
    EXPECT_EQ(board.timebase.snapshot().residual_count, 0U);
}

TEST(SofTimebase, AFullSpeedCycleCounterBoardPublishesTheSameNominalAsEveryOther) {
    Board<Cycles550> board;
    board.step = 8;                     // one SOF per 1 ms frame
    board.microframe = 2400;
    board.ticks_q16 = 68'750ULL << 16U; // 550 MHz cycles per microframe
    board.clock_at_zero = 0xF000'0000;  // and through its 7.8 s wrap
    board.run(kWindow + 800);
    board.timebase.apply_anchor(board.absolute());
    ASSERT_TRUE(board.timebase.answers());
    EXPECT_EQ(board.timebase.snapshot().ticks_per_microframe_q16, 500U << 16U)
        << "68750 cycles per microframe is exactly 500 quarter-us";
    EXPECT_EQ(board.timebase.current_microframe(), board.absolute());
}

TEST(SofTimebase, FloorDivisionRoundsTowardNegativeInfinity) {
    using libhcs::core::time::floor_div;
    EXPECT_EQ(floor_div(7, 2), 3);
    EXPECT_EQ(floor_div(-7, 2), -4);
    EXPECT_EQ(floor_div(-8, 2), -4);
    EXPECT_EQ(floor_div(0, 16384), 0);
}

} // namespace
