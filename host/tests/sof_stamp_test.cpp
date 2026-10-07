// Placing a record on the shared USB microframe axis.
//
// Two pieces, both pure arithmetic and both shared by the two ends of the
// cable: the 24-bit stamp a record carries (and how a receiver gets the full
// position back from it), and the ring a board uses to turn "the timestamp
// counter read N at the frame's first edge" into such a stamp. The board runs
// the ring in its highest-priority interrupt; here it runs against a simulated
// counter, where the right answer is known exactly.

#include <cmath>
#include <cstdint>
#include <optional>
#include <random>

#include <gtest/gtest.h>

#include "core/include/libhcs/time/sof_stamp.hpp"
#include "core/src/time/counter_link.hpp"
#include "core/src/time/sof_capture_ring.hpp"
#include "core/src/time/usb_sof_bits.hpp"

namespace {

using libhcs::core::time::CounterLink;
using libhcs::core::time::SofCaptureRing;
using libhcs::time::SofStamp;
using libhcs::time::unwrap;

constexpr std::uint32_t kNominal = 120'000;           // PTPC units per microframe, nominal
constexpr std::uint32_t kPtpcModulus = 1'000'000'000; // its nanosecond word wraps here

using Ring = SofCaptureRing<16>;

// A counter the hardware latches at every Start-of-Frame, and the frame index
// the device controller shows at that SOF.
struct Bus {
    double rate = 120'218.37; // counts per microframe -- NOT nominal, on purpose
    std::uint32_t counter_at_zero = 123'456'789;
    std::uint32_t frame_at_zero = 5000;
    std::uint32_t modulus = kPtpcModulus;

    // Counter value at a position, in microframes since the simulation began.
    [[nodiscard]] std::uint32_t counter(double position) const {
        const auto advanced = static_cast<std::uint64_t>(std::llround(position * rate));
        const std::uint64_t value = counter_at_zero + advanced;
        return static_cast<std::uint32_t>(modulus == 0 ? value : value % modulus);
    }
    [[nodiscard]] std::uint32_t frame_index(std::uint32_t microframe) const {
        return (frame_at_zero + microframe) & 0x3FFFU;
    }
    // The stamp a perfect board would produce for a position.
    [[nodiscard]] std::uint32_t ticks(double position) const {
        const auto exact = std::llround((frame_at_zero + position) * SofStamp::kTicksPerMicroframe);
        return static_cast<std::uint32_t>(exact) & SofStamp::kMask;
    }
    void capture(Ring& ring, std::uint32_t microframe) const {
        ring.push(frame_index(microframe), counter(microframe));
    }
};

// Distance between two stamps around the 24-bit wrap, in ticks.
std::uint32_t distance(std::uint32_t a, std::uint32_t b) {
    const std::uint32_t forward = (a - b) & SofStamp::kMask;
    const std::uint32_t backward = (b - a) & SofStamp::kMask;
    return forward < backward ? forward : backward;
}

// One tick is 7.6 ns. The ring's own arithmetic loses under one tick to the
// integer divisions; the simulated counter adds a count of rounding.
constexpr std::uint32_t kOneTick = 1;

#define EXPECT_STAMP_NEAR(ring, bus, position, tolerance)                          \
    do {                                                                           \
        const auto stamp_ = (ring).locate((bus).counter(position));                \
        ASSERT_TRUE(stamp_.has_value()) << "position " << (position);              \
        EXPECT_LE(distance(stamp_->ticks, (bus).ticks(position)), (tolerance))     \
            << "position " << (position) << ": got " << stamp_->ticks << ", want " \
            << (bus).ticks(position);                                              \
    } while (false)

TEST(SofStamp, PacksTheFrameIndexAboveTheFraction) {
    const auto stamp = SofStamp::from(0x2AB, 0x2555);
    EXPECT_EQ(stamp.frame_index(), 0x2ABU);
    EXPECT_EQ(stamp.fraction(), 0x2555U);
    EXPECT_EQ(stamp.ticks, (0x2ABU << 14U) | 0x2555U);

    // A fraction past one microframe carries into the frame index, and the
    // frame index wraps at 10 bits -- the low bits of the hardware counter.
    EXPECT_EQ(SofStamp::from(10, 16384 + 5), SofStamp::from(11, 5));
    EXPECT_EQ(SofStamp::from(0x3FF, 16384), SofStamp::from(0, 0));
    EXPECT_EQ(SofStamp::from(0x3FFF, 0), SofStamp::from(0x3FF, 0));
}

TEST(SofStamp, UnwrapReturnsThePositionNearestBehindTheReference) {
    constexpr std::int64_t span = SofStamp::kModulus;
    const std::int64_t truth = (37 * span) + 123'456;
    const SofStamp stamp{static_cast<std::uint32_t>(truth % span)};

    // The reference is "about now": anywhere from the instant itself to well
    // ninety milliseconds later gives the same answer.
    constexpr std::int64_t behind = (span / 4) * 3; // how far back the window reaches
    constexpr std::int64_t ahead = span / 4;        // and how far forward
    for (const std::int64_t late : {std::int64_t{0}, std::int64_t{1}, span / 2, behind})
        EXPECT_EQ(unwrap(stamp, truth + late), truth) << "reference " << late << " ticks later";

    // And a reference slightly BEFORE the event still resolves it: a control
    // loop passes the start of its cycle, and the record may have arrived after.
    for (const std::int64_t early : {std::int64_t{1}, span / 8, ahead - 1})
        EXPECT_EQ(unwrap(stamp, truth - early), truth) << "reference " << early << " ticks early";

    // Past those bounds the neighbouring wrap wins -- the documented limit.
    EXPECT_EQ(unwrap(stamp, truth + behind + 1), truth + span);
    EXPECT_EQ(unwrap(stamp, truth - ahead), truth - span);
}

TEST(SofStamp, TheSpanIsAnEighthOfASecond) {
    // 2^10 microframes of 125 us: what unwrap()'s window is measured against.
    EXPECT_EQ(SofStamp::kModulus / SofStamp::kTicksPerMicroframe * 125U, 128'000U);
}

TEST(SofStamp, UnwrapWorksAcrossZeroAndForNegativeReferences) {
    constexpr std::int64_t span = SofStamp::kModulus;
    EXPECT_EQ(unwrap(SofStamp{5}, 10), 5);
    EXPECT_EQ(unwrap(SofStamp{SofStamp::kMask}, 3), -1);
    EXPECT_EQ(unwrap(SofStamp{100}, -span + 150), -span + 100);
}

TEST(SofCaptureRing, AnEmptyRingHasNoAnswer) {
    const Ring ring{
        {.modulus = kPtpcModulus, .nominal_per_microframe = kNominal}
    };
    EXPECT_FALSE(ring.locate(12345).has_value());
}

TEST(SofCaptureRing, ASingleCaptureHasNoAnswer) {
    // One capture has a phase but no rate, and the nominal rate is not the
    // real one (0.18% off here: 3.6 us across 16 microframes). Two captures
    // are a quarter of a millisecond away; refusing until then costs nothing.
    const Bus bus;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    bus.capture(ring, 10);
    EXPECT_FALSE(ring.locate(bus.counter(10.0)).has_value());
    bus.capture(ring, 11);
    EXPECT_STAMP_NEAR(ring, bus, 10.5, kOneTick);
}

TEST(SofCaptureRing, PlacesAnEventAnywhereAlongTheRing) {
    const Bus bus;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 40; ++microframe)
        bus.capture(ring, microframe);

    // Anywhere among the captures the line was fitted to (26..40), including
    // exactly on one and a hair before the next.
    for (const double position : {26.0, 28.25, 30.5, 33.999, 37.123, 39.0, 39.9999, 40.0})
        EXPECT_STAMP_NEAR(ring, bus, position, kOneTick);
}

// The counter's rate need not be known: it is measured across the ring. Make
// that explicit with one that runs 2.5% off nominal.
TEST(SofCaptureRing, TheCountersRateDoesNotHaveToBeKnown) {
    Bus bus;
    bus.rate = kNominal * 1.025;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 40; ++microframe)
        bus.capture(ring, microframe);

    for (const double position : {26.5, 33.3, 40.0, 41.7, 44.9})
        EXPECT_STAMP_NEAR(ring, bus, position, kOneTick);
}

// Nor does it have to be steady: inside a bracket the rate is that bracket's
// own. Make that explicit with a counter whose rate is wrong by more, and
// differently every microframe.
TEST(SofCaptureRing, TheCountersRateDoesNotHaveToBeSteady) {
    std::mt19937 random{20261003};
    std::uniform_real_distribution<double> wander{-0.01, 0.01}; // +-1% per microframe

    Ring ring{
        {.modulus = 0, .nominal_per_microframe = kNominal}
    };
    std::uint32_t counter = 77;
    std::uint32_t captures[64];
    double rates[64];
    for (std::uint32_t microframe = 0; microframe < 64; ++microframe) {
        captures[microframe] = counter;
        ring.push(1000 + microframe, counter);
        rates[microframe] = kNominal * (1.0 + wander(random));
        counter += static_cast<std::uint32_t>(std::llround(rates[microframe]));
    }
    for (std::uint32_t microframe = 52; microframe < 63; ++microframe) {
        for (const double fraction : {0.1, 0.5, 0.9}) {
            const auto event =
                captures[microframe]
                + static_cast<std::uint32_t>(std::llround(rates[microframe] * fraction));
            const auto stamp = ring.locate(event);
            ASSERT_TRUE(stamp.has_value());
            const auto want = static_cast<std::uint32_t>(
                std::llround((1000 + microframe + fraction) * SofStamp::kTicksPerMicroframe));
            EXPECT_LE(distance(stamp->ticks, want), kOneTick) << microframe << " + " << fraction;
        }
    }
}

// What the HPM5321 actually does [measured 2026-10-05]: neighbouring SOF
// captures sit some 25 ns off a straight line, but over a few milliseconds
// the captures wander by hundreds of nanoseconds against the counter. A line
// through milliseconds of captures, carried forward, missed by 100-140 ns;
// the local rate must not.
TEST(SofCaptureRing, FollowsACounterThatWanders) {
    std::mt19937 random{20261005};
    std::uniform_int_distribution<int> jitter{-12, 12}; // +-12.5 ns, white
    const Bus bus;
    // +-300 counts (+-310 ns) of slow wander with an 8 ms period. That is as
    // much as the measurement allows: its rate swing (29 counts a microframe)
    // matches the 23-count spread of neighbouring intervals measured on the
    // board, and a 3.9 ms line carried a millisecond forward misses it by
    // about the 100 ns that version measured.
    const auto wander = [](double microframe) {
        return 300.0 * std::sin(2.0 * 3.14159265358979 * microframe / 64.0);
    };
    const auto counter_at = [&](double position) {
        return static_cast<std::uint32_t>(
            static_cast<std::int64_t>(bus.counter(position)) + std::llround(wander(position)));
    };

    double sum_squares = 0.0;
    std::uint32_t worst = 0;
    int samples = 0;
    for (int trial = 0; trial < 300; ++trial) {
        Ring ring{
            {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
        };
        const std::uint32_t last = 40U + static_cast<std::uint32_t>(trial % 64);
        for (std::uint32_t microframe = 0; microframe <= last; ++microframe)
            ring.push(
                bus.frame_index(microframe),
                static_cast<std::uint32_t>(
                    static_cast<std::int64_t>(counter_at(microframe)) + jitter(random)));
        // A frame inside the newest bracket, and frames past the newest capture.
        for (const double back : {-0.6, 0.4, 1.3}) {
            const double position = static_cast<double>(last) + back;
            const auto stamp = ring.locate(counter_at(position));
            ASSERT_TRUE(stamp.has_value()) << "trial " << trial << " position " << position;
            const std::uint32_t error = distance(stamp->ticks, bus.ticks(position));
            sum_squares += static_cast<double>(error) * error;
            worst = error > worst ? error : worst;
            samples++;
        }
    }
    const double rms = std::sqrt(sum_squares / samples);
    // The wander is 41 stamp ticks either way; the local rate keeps the error
    // near the captures' own jitter (1.7 ticks peak).
    EXPECT_LT(rms, 2.5) << "RMS error in ticks";
    EXPECT_LE(worst, 8U) << "worst error in ticks";
}

// The frame's first edge is usually a microframe or two before the interrupt
// that reads it, but the newest SOF may also not have been recorded yet -- the
// reader outranks the writer. The local rate is carried forward, two
// milliseconds at most.
TEST(SofCaptureRing, CarriesTheLocalRatePastTheNewestCapture) {
    const Bus bus;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 40; ++microframe)
        bus.capture(ring, microframe);

    for (const double position : {40.0, 40.3, 41.7, 47.9, 55.9})
        EXPECT_STAMP_NEAR(ring, bus, position, kOneTick);

    // Sixteen microframes on, the SOF side has stopped.
    EXPECT_FALSE(ring.locate(bus.counter(56.5)).has_value());
    EXPECT_FALSE(ring.locate(bus.counter(400.0)).has_value());
}

// A start-over in push() silences the captures before it at once, since the
// counter may have wrapped in the gap.
TEST(SofCaptureRing, AStartOverSilencesTheOldCapturesAtOnce) {
    const Bus bus;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 20; ++microframe)
        bus.capture(ring, microframe);
    ASSERT_TRUE(ring.locate(bus.counter(20.5)).has_value());

    ring.push(bus.frame_index(3020), bus.counter(3020));
    EXPECT_FALSE(ring.locate(bus.counter(20.5)).has_value());
    EXPECT_FALSE(ring.locate(bus.counter(3020.5)).has_value()); // one capture, no line

    ring.push(bus.frame_index(3021), bus.counter(3021));
    EXPECT_STAMP_NEAR(ring, bus, 3020.5, kOneTick);
}

TEST(SofCaptureRing, RefusesAnEventOlderThanEverythingItKept) {
    const Bus bus;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 40; ++microframe)
        bus.capture(ring, microframe);

    // Capacity 16, of which 15 are readable: captures 26..40.
    EXPECT_TRUE(ring.locate(bus.counter(26.5)).has_value());
    EXPECT_FALSE(ring.locate(bus.counter(25.5)).has_value());
    EXPECT_FALSE(ring.locate(bus.counter(3.0)).has_value());
}

// If the trigger only latches every other Start-of-Frame -- the HPM5321's
// TRGM SOF signal toggles once per SOF, so a one-edge capture does exactly
// this [measured 2026-10-05: 50.0% fresh] -- the ring spans twice the
// microframes and nothing else changes.
TEST(SofCaptureRing, WorksWhenOnlyEveryOtherSofIsCaptured) {
    const Bus bus;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 60; microframe += 2)
        bus.capture(ring, microframe);

    for (const double position : {40.0, 40.7, 41.0, 41.5, 53.25, 59.999, 60.4})
        EXPECT_STAMP_NEAR(ring, bus, position, kOneTick);
}

// A full-speed port sees one SOF per millisecond; its frame index steps by 8.
// Sixteen of them would span more counts than the fixed-point arithmetic
// takes, so the ring uses the newest ones that fit.
TEST(SofCaptureRing, WorksOnAFullSpeedPort) {
    const Bus bus;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 160; microframe += 8)
        bus.capture(ring, microframe);

    for (const double position : {120.0, 123.4, 127.99, 151.5, 160.0, 163.0, 175.0})
        EXPECT_STAMP_NEAR(ring, bus, position, kOneTick);
}

TEST(SofCaptureRing, FollowsTheCounterAcrossItsWrap) {
    Bus bus;
    // 1e9 lands between microframes 30 and 31 of this run.
    bus.counter_at_zero = kPtpcModulus - static_cast<std::uint32_t>(30.4 * bus.rate);
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 36; ++microframe)
        bus.capture(ring, microframe);
    ASSERT_LT(bus.counter(31.0), bus.counter(30.0)) << "the wrap is not where the test wants it";

    for (const double position : {29.5, 30.0, 30.2, 30.39, 30.41, 30.9, 31.0, 31.5, 35.0, 36.8})
        EXPECT_STAMP_NEAR(ring, bus, position, kOneTick);
}

TEST(SofCaptureRing, FollowsTheFrameIndexAcrossItsWrap) {
    Bus bus;
    bus.frame_at_zero = 0x3FFF - 30; // frame index returns to zero mid-ring
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 40; ++microframe)
        bus.capture(ring, microframe);

    for (const double position : {29.5, 30.5, 31.0, 31.5, 39.9})
        EXPECT_STAMP_NEAR(ring, bus, position, kOneTick);
}

// One capture attributed to the wrong SOF makes the brackets either side of it
// half or double the believable width, and spoils the rate of anything carried
// forward from it. Those are refused; the rest of the ring keeps answering.
TEST(SofCaptureRing, AMisattributedCaptureMakesItsNeighboursRefuse) {
    const Bus bus;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 40; ++microframe) {
        if (microframe == 35)
            ring.push(bus.frame_index(35), bus.counter(34.5)); // latched half a microframe early
        else
            bus.capture(ring, microframe);
    }
    EXPECT_FALSE(ring.locate(bus.counter(34.2)).has_value()); // bracket 34..35 is half width
    EXPECT_FALSE(ring.locate(bus.counter(35.5)).has_value()); // bracket 35..36 is 1.5 wide
    for (const double position : {33.5, 36.5, 40.5})
        EXPECT_STAMP_NEAR(ring, bus, position, kOneTick);

    // As the newest capture it would set the rate carried forward: refused.
    ring.push(bus.frame_index(41), bus.counter(40.5));
    EXPECT_FALSE(ring.locate(bus.counter(41.2)).has_value());
}

TEST(SofCaptureRing, IgnoresARepeatOfTheNewestFrame) {
    const Bus bus;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 20; ++microframe)
        bus.capture(ring, microframe);
    const auto before = ring.pushed();
    ring.push(bus.frame_index(20), bus.counter(20.0) + 999);
    EXPECT_EQ(ring.pushed(), before);
    EXPECT_STAMP_NEAR(ring, bus, 20.5, kOneTick);
}

// After a gap in the SOF stream the old captures say nothing about the new
// ones, and the counter has wrapped in between: one of them would eventually
// look like "the capture just before this event".
TEST(SofCaptureRing, AGapInTheSofStreamStartsTheRingOver) {
    const Bus bus;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 20; ++microframe)
        bus.capture(ring, microframe);
    EXPECT_GT(ring.pushed(), 1U);

    // The stream resumes 3000 microframes later.
    for (std::uint32_t microframe = 3020; microframe <= 3024; ++microframe)
        bus.capture(ring, microframe);
    EXPECT_EQ(ring.pushed(), 5U);
    EXPECT_FALSE(ring.locate(bus.counter(19.5)).has_value()); // the old captures are gone
    EXPECT_STAMP_NEAR(ring, bus, 3022.5, kOneTick);
}

TEST(SofCaptureRing, ClearForgetsEverything) {
    const Bus bus;
    Ring ring{
        {.modulus = bus.modulus, .nominal_per_microframe = kNominal}
    };
    for (std::uint32_t microframe = 0; microframe <= 20; ++microframe)
        bus.capture(ring, microframe);
    ring.clear();
    EXPECT_FALSE(ring.locate(bus.counter(19.5)).has_value());
}

// The mc02's counter: TIM2 at 275 MHz, wrapping at its auto-reload value
// (5,500,000: it doubles as a 50 Hz servo PWM), known only at run time; one
// SOF per millisecond on its full-speed port.
TEST(SofCaptureRing, TakesACounterConfiguredAtRunTime) {
    using SmallRing = SofCaptureRing<8>;
    Bus bus;
    bus.rate = 34'375.0 * (1.0 + 23e-6);       // a crystal 23 ppm off the host's
    bus.modulus = 5'500'000;
    bus.counter_at_zero = 5'500'000 - 200'000; // wraps between frames 0 and 8

    SmallRing ring{
        {.modulus = 0, .nominal_per_microframe = 1}
    };
    ring.configure({.modulus = bus.modulus, .nominal_per_microframe = 34'375});
    for (std::uint32_t microframe = 0; microframe <= 80; microframe += 8)
        ring.push(bus.frame_index(microframe), bus.counter(microframe));

    // The newest 7 of 8 are readable: frames 32..80, and up to 16 past.
    for (const double position : {34.0, 55.5, 79.9, 80.0, 84.4, 95.0})
        EXPECT_STAMP_NEAR(ring, bus, position, kOneTick);
}

TEST(SofCaptureRing, RefusesACounterTooFastForItsArithmetic) {
    using Counter = Ring::Counter;
    EXPECT_TRUE(Ring::supports(Counter{.modulus = 0, .nominal_per_microframe = 125'000}));
    EXPECT_FALSE(Ring::supports(Counter{.modulus = 0, .nominal_per_microframe = 130'000}));
    EXPECT_FALSE(Ring::supports(Counter{.modulus = 0, .nominal_per_microframe = 0}));

    Ring ring{
        {.modulus = 0, .nominal_per_microframe = 130'000}
    };
    for (std::uint32_t microframe = 0; microframe < 8; ++microframe)
        ring.push(microframe, microframe * 130'000U);
    EXPECT_FALSE(ring.locate(3 * 130'000U).has_value());
}

// ---- CounterLink: a 16-bit hardware stamp spliced onto a 32-bit counter ----

// The mc02's arrangement: TIM3 ticks once per two TIM5 ticks (shift 1); the
// offset between them is whatever it happened to be at power-up.
TEST(CounterLink, WidensAStampToTheMiddleOfItsTick) {
    for (const std::uint32_t shift : {0U, 1U, 2U}) {
        const CounterLink link{.shift = shift, .offset = 0x89AB'CDEFU};
        const std::uint32_t tick = std::uint32_t{1} << shift;
        // Events all around the wide counter's wrap and the narrow one's.
        for (const std::uint32_t event :
             {0x0000'0000U, 0x0000'0005U, 0x89AB'CDEFU, 0x89AB'CDF0U, 0xFFFF'FFF0U, 0xFFFF'FFFFU,
              0x1234'5678U}) {
            const std::uint16_t stamp = link.narrow_at(event);
            // The tick the event fell in starts where (event - offset) is a multiple of tick.
            const std::uint32_t tick_start = event - ((event - link.offset) & (tick - 1U));
            for (const std::uint32_t later : {0U, 1U, 777U, 40'000U * tick}) {
                const auto wide = link.widen(stamp, event + later, 49'152);
                ASSERT_TRUE(wide.has_value()) << "shift " << shift << " event " << event;
                EXPECT_EQ(*wide, tick_start + (tick >> 1U))
                    << "shift " << shift << " event " << event << " read " << later << " later";
            }
        }
    }
}

TEST(CounterLink, RefusesAStampOlderThanItsWindow) {
    const CounterLink link{.shift = 1, .offset = 12345};
    const std::uint32_t event = 1'000'000;
    const std::uint16_t stamp = link.narrow_at(event);
    EXPECT_TRUE(link.widen(stamp, event + (2 * 49'151), 49'152).has_value());
    EXPECT_FALSE(link.widen(stamp, event + (2 * 49'152) + 2, 49'152).has_value());
    // A full narrow wrap later the stamp reads as new again: that is what the
    // window is for, and why the caller keeps it well under 65536.
    EXPECT_TRUE(link.widen(stamp, event + (2 * 65'536), 49'152).has_value());
}

TEST(CounterLink, AgreesWithABracketedReadingAndCatchesAReload) {
    const CounterLink link{.shift = 1, .offset = 0xFFFF'FF00U};
    for (std::uint32_t step = 0; step < 300; ++step) {
        const std::uint32_t wide = 0xFFFF'F000U + (step * 37U); // across the wide wrap
        // The narrow counter sampled 5 wide ticks after `before`, 9 before `after`.
        const std::uint16_t narrow = link.narrow_at(wide + 5U);
        EXPECT_TRUE(link.agrees(wide, narrow, wide + 14U, 4)) << wide;
    }
    // Someone wrote TIM3's counter: the narrow reading is far off the link.
    EXPECT_FALSE(link.agrees(100, static_cast<std::uint16_t>(link.narrow_at(105) + 300), 114, 4));
    // An offset measured a couple of clocks late is still within the slack.
    const CounterLink late{.shift = 1, .offset = link.offset + 2U};
    EXPECT_TRUE(late.agrees(1000, link.narrow_at(1005), 1014, 4));
}

// ---- bit stuffing in a full-speed SOF packet ----

TEST(UsbSofBits, TheCrc5MatchesTheSpecificationsExample) {
    // USB-IF token CRC example: address 0x15, endpoint 0xE -> CRC5 0x17.
    EXPECT_EQ(libhcs::core::time::usb_token_crc5(0x15U | (0xEU << 7U)), 0x17U);
}

TEST(UsbSofBits, CountsTheStuffedBitsOfEveryFrameNumber) {
    using libhcs::core::time::full_speed_sof_stuffed_bits;
    EXPECT_EQ(full_speed_sof_stuffed_bits(0x000), 0U);
    EXPECT_EQ(full_speed_sof_stuffed_bits(0x03F), 1U);
    EXPECT_EQ(full_speed_sof_stuffed_bits(0x7FF), 2U);
    EXPECT_EQ(full_speed_sof_stuffed_bits(0x7FF + 0x800), 2U); // only 11 bits count

    std::uint32_t histogram[3] = {};
    for (std::uint32_t frame = 0; frame < 2048; ++frame) {
        const auto bits = full_speed_sof_stuffed_bits(frame);
        ASSERT_LE(bits, 2U) << frame;
        histogram[bits]++;
    }
    EXPECT_EQ(histogram[0], 1828U);
    EXPECT_EQ(histogram[1], 212U);
    EXPECT_EQ(histogram[2], 8U);
}

} // namespace
