// The host side of the shared time base: placing the boards' microframe axis
// on this machine's CLOCK_MONOTONIC.
//
// Everything here runs on made-up data where the truth is known. The parts
// that touch hardware -- reading the controller's counter, the USB round trip
// -- are not in this file; what is tested is what those readings are turned
// into, which is where the mistakes were.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <libhcs/time/sof_stamp.hpp>

#include "host/src/time/axis_fit.hpp"
#include "host/src/time/axis_map.hpp"
#include "host/src/time/usb_frame_axis.hpp"

namespace {

using libhcs::host::time::AxisMap;
using libhcs::host::time::UsbFrameAxis;
using libhcs::host::time::detail::ControllerLine;
using libhcs::host::time::detail::Edge;
using libhcs::host::time::detail::fit_controller;
using libhcs::host::time::detail::OffsetBracket;
using libhcs::host::time::detail::PublishedAxisMap;
using libhcs::host::time::detail::robust_line;
using libhcs::time::SofStamp;

using Clock = AxisMap::Clock;

Clock::time_point at_ns(std::int64_t ns) { return Clock::time_point{std::chrono::nanoseconds{ns}}; }
std::int64_t ns_of(Clock::time_point when) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(when.time_since_epoch()).count();
}

// ---- AxisMap ---------------------------------------------------------------

TEST(AxisMap, RefusesToConvertWithoutALine) {
    const AxisMap none;
    EXPECT_FALSE(none.valid());
    EXPECT_FALSE(none.time_of(SofStamp{123}, at_ns(1'000'000'000)).has_value());
}

TEST(AxisMap, ConvertsBothWays) {
    const AxisMap map{
        .source = AxisMap::Source::kController,
        .reference_microframe = 8'000'000.0,
        .reference_ns = 5'000'000'000'000,
        .period_ns = 125'004.9,
    };
    const double microframe = 8'000'000.0 + 1234.5;
    const auto when = map.time_of(microframe);
    EXPECT_EQ(ns_of(when), 5'000'000'000'000 + std::llround(1234.5 * 125'004.9));
    EXPECT_NEAR(map.microframe_at(when), microframe, 1e-5);
}

// A record stamped by a board comes back as the instant it happened, wherever
// in the 128 ms span of the stamp it fell and however late (up to the
// documented 96 ms) the caller's idea of "now" is.
TEST(AxisMap, PlacesAStampedRecordOnTheHostClock) {
    const AxisMap map{
        .source = AxisMap::Source::kController,
        .reference_microframe = 123'456'789.0,
        .reference_ns = 77'000'000'000'000,
        .period_ns = 125'000.5,
    };
    constexpr double ticks = SofStamp::kTicksPerMicroframe;

    for (const double ahead : {0.0, 17.25, 1023.999, 1024.0, 4000.75, 50'000.125}) {
        const double microframe = map.reference_microframe + ahead;
        // The stamp is the position modulo 2^24 ticks, to the nearest tick.
        const auto position = static_cast<std::int64_t>(std::llround(microframe * ticks));
        const SofStamp stamp{static_cast<std::uint32_t>(position) & SofStamp::kMask};
        const auto truth = map.time_of(static_cast<double>(position) / ticks);

        for (const auto late :
             {std::chrono::microseconds{0}, std::chrono::microseconds{300},
              std::chrono::microseconds{20'000}, std::chrono::microseconds{90'000}}) {
            const auto got = map.time_of(stamp, truth + late);
            ASSERT_TRUE(got.has_value());
            EXPECT_EQ(*got, truth)
                << "record " << ahead << " microframes in, seen " << late.count() << " us later";
        }
        // A control loop passes the START of its cycle; the record may have
        // arrived a little after that.
        const auto got = map.time_of(stamp, truth - std::chrono::milliseconds{2});
        ASSERT_TRUE(got.has_value());
        EXPECT_EQ(*got, truth);
    }
}

// ---- robust_line -----------------------------------------------------------

TEST(RobustLine, RecoversAnExactLine) {
    std::vector<double> x;
    std::vector<double> y;
    for (int i = 0; i < 50; ++i) {
        x.push_back(1e10 + (i * 667.0));
        y.push_back(3e13 + (i * 667.0 * 125'000.0125));
    }
    const auto line = robust_line(x, y);
    ASSERT_TRUE(line.valid);
    EXPECT_NEAR(line.slope, 125'000.0125, 1e-6);
    EXPECT_NEAR(line.at(x.back()), y.back(), 1.0);
    EXPECT_EQ(line.used, 50U);
}

// The reason it exists: a handful of samples displaced by a preemption must
// not move the line. Plain least squares is included for contrast -- the
// number it gets is why this is not plain least squares.
TEST(RobustLine, IsNotMovedBySamplesThatArePlainlyWrong) {
    std::mt19937 random{1};
    std::uniform_real_distribution<double> noise{-300.0, 300.0};
    std::vector<double> x;
    std::vector<double> y;
    for (int i = 0; i < 256; ++i) {
        x.push_back(i * 667.0);
        double value = (i * 667.0 * 125'000.0) + noise(random);
        // Eight late samples, bunched at the leading end where the line is used.
        if (i >= 240 && i % 2 == 0)
            value += 30'000.0;
        y.push_back(value);
    }
    const double truth = x.back() * 125'000.0;

    const auto robust = robust_line(x, y);
    ASSERT_TRUE(robust.valid);
    EXPECT_LT(std::fabs(robust.at(x.back()) - truth), 100.0);
    EXPECT_LE(robust.used, 248U);

    const auto plain = robust_line(x, y, 1e9); // nothing is ever rejected
    ASSERT_TRUE(plain.valid);
    EXPECT_GT(std::fabs(plain.at(x.back()) - truth), 1'500.0);
}

TEST(RobustLine, NeedsAFewSamples) {
    const std::vector<double> x{1, 2, 3};
    const std::vector<double> y{1, 2, 3};
    EXPECT_FALSE(robust_line(x, y).valid);
}

// ---- fit_controller ---------------------------------------------------------

// A host as seen through its controller: the microframe counter and
// CLOCK_MONOTONIC_RAW run off one crystal; CLOCK_MONOTONIC is RAW with a rate
// correction NTP changes when it pleases.
struct Host {
    double raw_period_ns = 125'000.0125; // 0.1 ppm between counter and TSC
    double raw_at_zero = 4.0e13;
    double mono_minus_raw_at_zero = 123'456'789.0;
    double steer_ppm = 39.1; // what CLOCK_MONOTONIC runs fast by
    double resteer_at_microframe = 1e18;
    double resteer_ppm = 39.1;

    [[nodiscard]] double raw(double microframe) const {
        return raw_at_zero + (microframe * raw_period_ns);
    }
    [[nodiscard]] double mono(double microframe) const {
        const double elapsed = microframe * raw_period_ns;
        double correction = mono_minus_raw_at_zero;
        if (microframe <= resteer_at_microframe) {
            correction += elapsed * steer_ppm * 1e-6;
        } else {
            const double before = resteer_at_microframe * raw_period_ns;
            correction += before * steer_ppm * 1e-6;
            correction += (elapsed - before) * resteer_ppm * 1e-6;
        }
        return raw(microframe) + correction;
    }
};

// 256 edges at 12 Hz, as the edge thread collects them.
std::vector<Edge> edges_of(const Host& host, std::uint32_t seed, double jitter_ns = 400.0) {
    std::mt19937 random{seed};
    std::uniform_real_distribution<double> noise{-jitter_ns, jitter_ns};
    std::vector<Edge> edges;
    for (int i = 0; i < 256; ++i) {
        const double microframe = 100'000.0 + (i * 667.0);
        const double error = noise(random); // the same instant read on both clocks
        edges.push_back({
            .microframe = microframe,
            .mono_ns = host.mono(microframe) + error,
            .raw_ns = host.raw(microframe) + error,
            .width_ns = 1'500.0,
        });
    }
    return edges;
}

TEST(FitController, PlacesTheCounterOnMonotonic) {
    const Host host;
    const auto edges = edges_of(host, 7);
    const ControllerLine line = fit_controller(edges, 12, 0.0);
    ASSERT_TRUE(line.valid);

    EXPECT_NEAR(line.raw_period_ns, host.raw_period_ns, 1e-3);
    EXPECT_NEAR(line.mono_period_ns, host.raw_period_ns * (1.0 + 39.1e-6), 0.05);

    // Where it is used: at the leading end, and a few milliseconds either side.
    const double newest = edges.back().microframe;
    for (const double offset : {-40.0, 0.0, 8.0, 80.0})
        EXPECT_NEAR(line.mono_at(newest + offset), host.mono(newest + offset), 250.0) << offset;
}

// A read that was preempted reports a wide window. Those are dropped before
// anything is fitted.
TEST(FitController, DropsEdgesThatAdmitTheyAreUncertain) {
    const Host host;
    auto edges = edges_of(host, 11);
    for (std::size_t i = 200; i < edges.size(); i += 5) {
        edges[i].mono_ns += 20'000.0;
        edges[i].raw_ns += 20'000.0;
        edges[i].width_ns = 41'500.0;
    }
    const ControllerLine line = fit_controller(edges, 12, 0.0);
    ASSERT_TRUE(line.valid);
    const double newest = edges.back().microframe;
    EXPECT_NEAR(line.mono_at(newest), host.mono(newest), 250.0);
    EXPECT_LT(line.used, edges.size());
}

// NTP re-steers CLOCK_MONOTONIC by 15 ppm, two seconds before the newest edge.
// The two-stage fit is back on the truth already; a single line fitted against
// CLOCK_MONOTONIC over the same 21 s window -- what this code did before --
// is still several microseconds off, and stays off until the window has slid
// past the step.
TEST(FitController, FollowsMonotonicBeingReSteered) {
    Host host;
    const double newest = 100'000.0 + (255 * 667.0);
    host.resteer_at_microframe = newest - 16'000.0; // 2 s ago
    host.resteer_ppm = host.steer_ppm + 15.0;
    const auto edges = edges_of(host, 3, 100.0);

    const ControllerLine line = fit_controller(edges, 12, 0.0);
    ASSERT_TRUE(line.valid);
    EXPECT_NEAR(line.mono_at(newest), host.mono(newest), 250.0);
    EXPECT_NEAR(line.mono_at(newest + 80.0), host.mono(newest + 80.0), 250.0);

    std::vector<double> x;
    std::vector<double> y;
    for (const Edge& edge : edges) {
        x.push_back(edge.microframe);
        y.push_back(edge.mono_ns);
    }
    const auto single = robust_line(x, y);
    ASSERT_TRUE(single.valid);
    EXPECT_GT(std::fabs(single.at(newest) - host.mono(newest)), 3'000.0);
}

// The counter is sampled inside the register read and the clock after it, so
// the window's middle reads late; the caller says by how much.
TEST(FitController, TakesTheSamplingBiasOut) {
    const Host host;
    const auto edges = edges_of(host, 5);
    const ControllerLine without = fit_controller(edges, 12, 0.0);
    const ControllerLine with = fit_controller(edges, 12, 420.0);
    ASSERT_TRUE(without.valid);
    ASSERT_TRUE(with.valid);
    const double newest = edges.back().microframe;
    EXPECT_NEAR(with.mono_at(newest), without.mono_at(newest) - 420.0, 1.0);
}

TEST(FitController, NeedsEnoughEdges) {
    const Host host;
    auto edges = edges_of(host, 9);
    edges.resize(7);
    EXPECT_FALSE(fit_controller(edges, 12, 0.0).valid);
}

// ---- OffsetBracket ----------------------------------------------------------

// One exchange with a board whose axis is `offset` microframes ahead of the
// host counter: the request leaves at host position `sent`, the board writes
// its position `down` microframes later, the reply is seen `up` after that.
void exchange(OffsetBracket& bracket, std::int64_t offset, double sent, double down, double up) {
    const double board = sent + down + static_cast<double>(offset);
    bracket.observe(board, sent, sent + down + up);
}

TEST(OffsetBracket, FindsTheIntegerOnAHighSpeedLink) {
    OffsetBracket bracket{128, 0.05, 4};
    std::mt19937 random{4};
    std::uniform_real_distribution<double> down{0.25, 0.45};
    std::uniform_real_distribution<double> up{0.30, 0.50};
    for (int i = 0; i < 4; ++i)
        exchange(bracket, 987'654'321, 1000.0 + (i * 2000.37), down(random), up(random));

    const auto estimate = bracket.estimate();
    EXPECT_TRUE(estimate.exact);
    EXPECT_EQ(estimate.offset, 987'654'321);
}

// The case rounding a midpoint estimate has almost no margin for. The request
// takes a tenth of a microframe, the reply eight tenths: the board sampled
// 0.35 before the middle of the round trip, which leaves that estimate 0.15
// from the rounding boundary -- and a board that reports whole microframes
// loses another half on average, which puts it across. The bounds do not care
// where in the round trip the board sampled.
TEST(OffsetBracket, DoesNotAssumeTheRoundTripIsSymmetric) {
    OffsetBracket bracket{128, 0.05, 4};
    for (int i = 0; i < 6; ++i)
        exchange(bracket, 5000, 100.0 + (i * 2000.61), 0.10, 0.80);
    const auto estimate = bracket.estimate();
    EXPECT_TRUE(estimate.exact);
    EXPECT_EQ(estimate.offset, 5000);

    OffsetBracket mirrored{128, 0.05, 4};
    for (int i = 0; i < 6; ++i)
        exchange(mirrored, -5000, 100.0 + (i * 2000.61), 0.80, 0.10);
    EXPECT_TRUE(mirrored.estimate().exact);
    EXPECT_EQ(mirrored.estimate().offset, -5000);
}

TEST(OffsetBracket, WaitsForTheMinimumNumberOfExchanges) {
    OffsetBracket bracket{128, 0.05, 4};
    for (int i = 0; i < 3; ++i) {
        exchange(bracket, 42, 10.0 + (i * 2000.0), 0.3, 0.3);
        EXPECT_FALSE(bracket.estimate().exact);
    }
    exchange(bracket, 42, 9000.0, 0.3, 0.3);
    EXPECT_TRUE(bracket.estimate().exact);
}

// A full-speed link's round trip spans many microframes. The bounds stay
// honest -- they contain the truth -- but several integers fit, and it refuses
// to pick one.
TEST(OffsetBracket, RefusesToGuessWhenSeveralIntegersFit) {
    OffsetBracket bracket{128, 0.05, 4};
    std::mt19937 random{8};
    std::uniform_real_distribution<double> down{1.5, 6.0};
    std::uniform_real_distribution<double> up{2.0, 8.0};
    for (int i = 0; i < 100; ++i)
        exchange(bracket, 777, 50.0 + (i * 2000.13), down(random), up(random));
    const auto estimate = bracket.estimate();
    EXPECT_FALSE(estimate.exact);
    EXPECT_LT(estimate.lower, 777.0);
    EXPECT_GT(estimate.upper, 777.0);
}

// But the axis is one for all boards: a single high-speed board on the same
// controller settles it.
TEST(OffsetBracket, OneFastBoardSettlesItForEveryone) {
    OffsetBracket bracket{128, 0.05, 4};
    for (int i = 0; i < 20; ++i)
        exchange(bracket, 777, 50.0 + (i * 2000.13), 3.0, 5.0);
    EXPECT_FALSE(bracket.estimate().exact);
    for (int i = 0; i < 2; ++i)
        exchange(bracket, 777, 90'000.0 + (i * 2000.13), 0.3, 0.4);
    EXPECT_TRUE(bracket.estimate().exact);
    EXPECT_EQ(bracket.estimate().offset, 777);
}

// The relation changed -- a board came back anchored onto another wrap. The
// old bounds are false now; the lock goes at once and is earned again.
TEST(OffsetBracket, AContradictionDropsTheLockImmediately) {
    OffsetBracket bracket{128, 0.05, 4};
    for (int i = 0; i < 8; ++i)
        exchange(bracket, 1000, 10.0 + (i * 2000.0), 0.3, 0.4);
    ASSERT_TRUE(bracket.estimate().exact);

    exchange(bracket, 1000 + 16384, 20'000.0, 0.3, 0.4);
    EXPECT_FALSE(bracket.estimate().exact);
    EXPECT_EQ(bracket.estimate().observations, 1U);

    for (int i = 1; i < 4; ++i)
        exchange(bracket, 1000 + 16384, 20'000.0 + (i * 2000.0), 0.3, 0.4);
    EXPECT_TRUE(bracket.estimate().exact);
    EXPECT_EQ(bracket.estimate().offset, 1000 + 16384);
}

// The bounds are computed from two fitted lines, each good to a fraction of a
// microsecond. A bound that is a little too tight for that reason must not
// read as a contradiction.
TEST(OffsetBracket, ToleratesBoundsThatAreSlightlyOff) {
    OffsetBracket bracket{128, 0.05, 4};
    for (int i = 0; i < 8; ++i) {
        const double sent = 10.0 + (i * 2000.0);
        // The reply is logged 0.02 microframes (2.5 us) before the board's
        // position says it could have been written.
        bracket.observe(sent + 0.30 + 250.0, sent, sent + 0.28);
    }
    EXPECT_TRUE(bracket.estimate().exact);
    EXPECT_EQ(bracket.estimate().offset, 250);
}

TEST(OffsetBracket, ForgetsExchangesBeyondItsCapacity) {
    OffsetBracket bracket{8, 0.05, 4};
    for (int i = 0; i < 100; ++i)
        exchange(bracket, 5, 10.0 + (i * 2000.0), 0.3, 0.4);
    EXPECT_EQ(bracket.estimate().observations, 8U);
    EXPECT_TRUE(bracket.estimate().exact);
}

// ---- PublishedAxisMap -------------------------------------------------------

// Every map a reader gets is one the writer stored whole -- never the fields
// of two different ones.
TEST(PublishedAxisMap, AReaderNeverSeesAHalfWrittenMap) {
    PublishedAxisMap published;
    EXPECT_FALSE(published.load().valid());

    std::atomic<bool> stop{false};
    std::thread writer{[&] {
        for (std::int64_t value = 1; !stop.load(std::memory_order_relaxed); ++value) {
            published.store(
                AxisMap{
                    .source = AxisMap::Source::kController,
                    .reference_microframe = static_cast<double>(value),
                    .reference_ns = value,
                    .period_ns = static_cast<double>(value),
                });
        }
    }};

    std::int64_t newest_seen = 0;
    for (int i = 0; i < 2'000'000; ++i) {
        const AxisMap map = published.load();
        if (!map.valid())
            continue;
        ASSERT_EQ(map.reference_microframe, static_cast<double>(map.reference_ns));
        ASSERT_EQ(map.period_ns, static_cast<double>(map.reference_ns));
        ASSERT_GE(map.reference_ns, newest_seen) << "the published map went backwards";
        newest_seen = map.reference_ns;
    }
    stop.store(true, std::memory_order_relaxed);
    writer.join();
    EXPECT_GT(newest_seen, 0);
}

// ---- UsbFrameAxis (the round-trip fit, one axis per controller) -------------

// The pre-refactor Timeline, frozen here as a reference the refactor is checked
// against (host/TIME_DESIGN.md step 2): same fit, same gates, no registry and
// no controller source. It is deliberately NOT the production code -- if the
// two ever disagree, the extraction changed something it should not have.
class ReferenceAxis {
public:
    using Clock = std::chrono::steady_clock;
    static constexpr std::chrono::nanoseconds kMicroframePeriod{125'000};
    static constexpr std::size_t kSampleCapacity = 1024;

    void observe(double microframe, Clock::time_point sent_at, Clock::time_point received_at) {
        const auto sampled_at = sent_at + ((received_at - sent_at) / 2);
        const std::scoped_lock guard{mutex_};
        if (fitted_) {
            const double predicted_ns =
                fit_reference_ns_ + ((microframe - fit_reference_microframe_) * fit_period_ns_);
            const double error_ns = static_cast<double>(to_ns(sampled_at)) - predicted_ns;
            if (std::fabs(error_ns) > 1e9) {
                sample_head_ = 0;
                sample_count_ = 0;
                fitted_ = false;
                publish_locked();
            }
        }
        samples_[(sample_head_ + sample_count_) % kSampleCapacity] =
            Sample{.microframe = microframe, .host_ns = to_ns(sampled_at)};
        if (sample_count_ < kSampleCapacity)
            sample_count_++;
        else
            sample_head_ = (sample_head_ + 1) % kSampleCapacity;
        if (sample_count_ >= 16)
            refit_locked();
    }

    [[nodiscard]] uint64_t anchor_for(Clock::time_point when) const {
        const std::scoped_lock guard{mutex_};
        if (!fitted_) {
            const auto elapsed =
                std::chrono::duration_cast<std::chrono::nanoseconds>(when - origin_);
            return static_cast<uint64_t>(elapsed.count() / kMicroframePeriod.count());
        }
        const double offset_ns = static_cast<double>(to_ns(when)) - fit_reference_ns_;
        return static_cast<uint64_t>(
            std::llround(fit_reference_microframe_ + (offset_ns / fit_period_ns_)));
    }

    [[nodiscard]] AxisMap axis_map() const noexcept { return published_.load(); }

private:
    struct Sample {
        double microframe;
        int64_t host_ns;
    };

    static int64_t to_ns(Clock::time_point when) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(when.time_since_epoch())
            .count();
    }

    void publish_locked() noexcept {
        if (!fitted_) {
            published_.store(AxisMap{});
            return;
        }
        published_.store(
            AxisMap{
                .source = AxisMap::Source::kRoundTrip,
                .reference_microframe = fit_reference_microframe_,
                .reference_ns = static_cast<std::int64_t>(std::llround(fit_reference_ns_)),
                .period_ns = fit_period_ns_,
            });
    }

    void refit_locked() {
        const Sample& base = samples_[sample_head_];
        double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
        for (std::size_t index = 0; index < sample_count_; index++) {
            const Sample& sample = samples_[(sample_head_ + index) % kSampleCapacity];
            const double x = sample.microframe - base.microframe;
            const auto y = static_cast<double>(sample.host_ns - base.host_ns);
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
        }
        const auto count = static_cast<double>(sample_count_);
        const double denominator = (count * sum_xx) - (sum_x * sum_x);
        if (denominator <= 0.0)
            return;
        const double period_ns = (count * sum_xy - sum_x * sum_y) / denominator;
        const auto nominal = static_cast<double>(kMicroframePeriod.count());
        if (period_ns < nominal * 0.97 || period_ns > nominal * 1.03)
            return;
        const double intercept_ns = (sum_y - period_ns * sum_x) / count;
        fit_reference_microframe_ = base.microframe;
        fit_reference_ns_ = static_cast<double>(base.host_ns) + intercept_ns;
        fit_period_ns_ = period_ns;
        fitted_ = true;
        publish_locked();
    }

    mutable std::mutex mutex_;
    Clock::time_point origin_ = Clock::now();
    Sample samples_[kSampleCapacity];
    std::size_t sample_head_ = 0;
    std::size_t sample_count_ = 0;
    bool fitted_ = false;
    double fit_reference_microframe_ = 0.0;
    double fit_reference_ns_ = 0.0;
    double fit_period_ns_ = 0.0;
    PublishedAxisMap published_;
};

// Identical observe/anchor sequences must give identical anchors and identical
// maps on the refactored axis and on the frozen pre-refactor one. The axis
// under test is the registry's unbound entry (no bus: no controller source),
// which is the same fallback path the reference implements.
TEST(UsbFrameAxis, MatchesThePreRefactorRoundTripFit) {
    const auto axis = UsbFrameAxis::for_usb_bus(-1);
    ReferenceAxis reference;
    EXPECT_FALSE(axis->axis_map().valid()) << "nothing has been observed yet";

    // A board whose axis reads microframe m at host time base + m * period,
    // reporting every 250 ms through a round trip of 80 + 60 us with jitter --
    // then one report a second off the line, which both must treat as an axis
    // restart rather than an outlier.
    constexpr double period_ns = 125'003.0;
    constexpr std::int64_t base_ns = 9'000'000'000'000;
    std::mt19937 random{12};
    std::uniform_real_distribution<double> jitter{-15'000.0, 15'000.0};

    double microframe = 5'000'000.25;
    for (int i = 0; i < 64; ++i, microframe += 2000.0) {
        const double sampled_ns = static_cast<double>(base_ns) + (microframe * period_ns);
        const auto sent = at_ns(std::llround(sampled_ns - 70'000.0 + jitter(random)));
        const auto received = at_ns(std::llround(sampled_ns + 70'000.0 + jitter(random)));
        axis->observe(microframe, sent, received);
        reference.observe(microframe, sent, received);
    }
    // Anchors agree, bit for bit, while the fit stands.
    std::mt19937 probe_random{4};
    std::uniform_int_distribution<std::int64_t> probe_ns{
        base_ns - 1'000'000'000, base_ns + std::llround((microframe + 4000.0) * period_ns)};
    for (int i = 0; i < 1000; ++i) {
        const auto when = at_ns(probe_ns(probe_random));
        ASSERT_EQ(axis->anchor_for(when), reference.anchor_for(when));
    }

    // Restart the fit on both: the next report cannot be on the fitted line.
    // (After this each axis anchors open loop from its own construction
    // instant -- an origin the next fit replaces, and one the two objects
    // legitimately do not share -- so anchors stop being comparable until the
    // fit has been earned again below.)
    const auto off_axis = at_ns(base_ns + std::llround(microframe * period_ns) + 2'000'000'000);
    axis->observe(
        microframe, off_axis - std::chrono::microseconds{50},
        off_axis + std::chrono::microseconds{50});
    reference.observe(
        microframe, off_axis - std::chrono::microseconds{50},
        off_axis + std::chrono::microseconds{50});
    EXPECT_FALSE(axis->axis_map().valid()) << "a second-off sample restarts the axis";

    // The maps agree: every conversion either refuses on both or answers the
    // same value on both (the fit refills after the restart, 16 samples in).
    for (int i = 0; i < 64; ++i, microframe += 2000.0) {
        const double sampled_ns = static_cast<double>(base_ns) + (microframe * period_ns);
        const auto sent = at_ns(std::llround(sampled_ns - 70'000.0 + jitter(random)));
        const auto received = at_ns(std::llround(sampled_ns + 70'000.0 + jitter(random)));
        axis->observe(microframe, sent, received);
        reference.observe(microframe, sent, received);
    }
    const AxisMap axis_map = axis->axis_map();
    const AxisMap reference_map = reference.axis_map();
    ASSERT_TRUE(axis_map.valid());
    ASSERT_TRUE(reference_map.valid());
    EXPECT_EQ(axis_map.reference_microframe, reference_map.reference_microframe);
    EXPECT_EQ(axis_map.reference_ns, reference_map.reference_ns);
    // The two fits are compiled in different translation units: FP contraction
    // may round differently by ULPs, which is far under the resolution the
    // published line is used at.
    EXPECT_DOUBLE_EQ(axis_map.period_ns, reference_map.period_ns);
}

// Buses behind one controller share the axis object; the registry keeps it for
// the process's life. A bus without a PCI device shares the unbound entry --
// the same object, so the two calls agree with each other.
TEST(UsbFrameAxis, RegistryHandsOutTheSameAxisForTheSameBus) {
    const auto first = UsbFrameAxis::for_usb_bus(-1);
    const auto again = UsbFrameAxis::for_usb_bus(-1);
    EXPECT_EQ(first.get(), again.get());
    EXPECT_EQ(first.get(), UsbFrameAxis::for_usb_bus(-2).get())
        << "buses without a PCI device share the unbound axis";
}

} // namespace
