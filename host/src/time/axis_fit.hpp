#pragma once

// The arithmetic behind the host side of the shared time base, kept free of
// threads, clocks and hardware so it can be run against data where the right
// answer is known (host/tests/time_axis_test.cpp).
//
// Three pieces:
//   robust_line()     a straight line that a few bad samples cannot move;
//   fit_controller()  host microframe counter -> CLOCK_MONOTONIC, in two stages;
//   OffsetBracket     the whole-microframe offset between that counter and the
//                     boards' axis, from causality rather than from symmetry.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <vector>

#include "host/src/time/axis_map.hpp"

namespace libhcs::host::time::detail {

struct Line {
    bool valid = false;
    double x0 = 0.0;
    double y0 = 0.0;
    double slope = 0.0;
    // Robust scatter of the samples about the line (1.4826 * median absolute
    // residual), over the samples the final pass kept.
    double sigma = 0.0;
    std::size_t used = 0;

    [[nodiscard]] double at(double x) const noexcept { return y0 + (slope * (x - x0)); }
};

// Least squares, then least squares again without the samples that sat more
// than `reject_sigmas` robust sigmas off the first line.
//
// Why not plain least squares: the samples here are instants read off a
// non-real-time host, and their errors are one-sided and heavy-tailed -- a
// preemption between two clock reads displaces one sample by tens of
// microseconds while the rest scatter by a fraction of one. Measured on this
// project's development machine, untuned: scatter 3 us by a plain sigma with
// individual samples 20-40 us out. One such sample in a window of 256 moves a
// plain fit's end point by more than the whole scatter of the others.
[[nodiscard]] inline Line
    robust_line(std::span<const double> x, std::span<const double> y, double reject_sigmas = 3.5) {
    Line line;
    const std::size_t count = std::min(x.size(), y.size());
    if (count < 4)
        return line;

    // Referenced to the first sample so the sums stay small: microframe
    // numbers reach 10^10 within a day and nanosecond stamps 10^13, and
    // squaring either raw would spend the mantissa on the offset.
    const double base_x = x[0];
    const double base_y = y[0];

    std::vector<bool> keep(count, true);
    std::vector<double> residual(count);
    double slope = 0.0;
    double intercept = 0.0;
    double sigma = 0.0;
    std::size_t used = 0;

    for (int pass = 0; pass < 3; ++pass) {
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_xx = 0.0;
        double sum_xy = 0.0;
        used = 0;
        for (std::size_t i = 0; i < count; ++i) {
            if (!keep[i])
                continue;
            const double dx = x[i] - base_x;
            const double dy = y[i] - base_y;
            sum_x += dx;
            sum_y += dy;
            sum_xx += dx * dx;
            sum_xy += dx * dy;
            ++used;
        }
        if (used < 4)
            return line;
        const auto n = static_cast<double>(used);
        const double denominator = (n * sum_xx) - (sum_x * sum_x);
        if (!(denominator > 0.0))
            return line;
        slope = ((n * sum_xy) - (sum_x * sum_y)) / denominator;
        intercept = (sum_y - (slope * sum_x)) / n;

        std::vector<double> magnitude;
        magnitude.reserve(used);
        for (std::size_t i = 0; i < count; ++i) {
            residual[i] = (y[i] - base_y) - (intercept + (slope * (x[i] - base_x)));
            if (keep[i])
                magnitude.push_back(std::fabs(residual[i]));
        }
        const auto middle = static_cast<std::ptrdiff_t>(magnitude.size() / 2);
        std::ranges::nth_element(magnitude, magnitude.begin() + middle);
        sigma = 1.4826 * magnitude[magnitude.size() / 2];

        // Re-select from ALL samples each pass, not only the survivors: a
        // sample wrongly dropped while the line was still being pulled by an
        // outlier gets back in once the line has moved.
        const double limit = reject_sigmas * sigma;
        bool changed = false;
        for (std::size_t i = 0; i < count; ++i) {
            const bool inside = !(limit > 0.0) || std::fabs(residual[i]) <= limit;
            changed = changed || (inside != keep[i]);
            keep[i] = inside;
        }
        if (!changed)
            break;
    }

    line.valid = true;
    line.x0 = base_x;
    line.y0 = base_y + intercept;
    line.slope = slope;
    line.sigma = sigma;
    line.used = used;
    return line;
}

// One observed transition of the host controller's microframe counter: the
// counter stepped to `microframe` somewhere inside a window `width_ns` wide,
// whose middle read `mono_ns` on CLOCK_MONOTONIC and `raw_ns` on
// CLOCK_MONOTONIC_RAW.
struct Edge {
    double microframe;
    double mono_ns;
    double raw_ns;
    double width_ns;
};

struct ControllerLine {
    bool valid = false;
    // mono_ns = reference_mono_ns + (microframe - reference_microframe) * mono_period_ns
    double reference_microframe = 0.0;
    double reference_mono_ns = 0.0;
    double mono_period_ns = 0.0;
    // The same slope against CLOCK_MONOTONIC_RAW: the figure to quote in ppm,
    // because nothing steers that clock.
    double raw_period_ns = 0.0;
    double sigma_ns = 0.0; // robust scatter of one edge about the line
    std::size_t used = 0;

    [[nodiscard]] double mono_at(double microframe) const noexcept {
        return reference_mono_ns + ((microframe - reference_microframe) * mono_period_ns);
    }
    [[nodiscard]] double microframe_at(double mono_ns) const noexcept {
        return reference_microframe + ((mono_ns - reference_mono_ns) / mono_period_ns);
    }
};

// Host microframe counter -> CLOCK_MONOTONIC, in two stages.
//
// STAGE ONE, long: microframe -> CLOCK_MONOTONIC_RAW, a robust line through
// every edge in the window. These two are as good as one oscillator -- the
// controller and the TSC hang off the same crystal, measured within 0.1 ppm
// of each other -- so the relation is a true straight line for as long as
// anyone cares to average it, and the window can be long.
//
// STAGE TWO, short: CLOCK_MONOTONIC_RAW -> CLOCK_MONOTONIC, from the last few
// edges only. This relation is NOT a straight line: NTP re-steers the rate of
// CLOCK_MONOTONIC whenever it likes, by parts per million at a time. But it is
// noiseless -- both clocks are read back to back in the same thread -- so a
// second of it is enough, and a second is how long a re-steer takes to be
// followed.
//
// Why not fit CLOCK_MONOTONIC directly, as this did until 2026-10-03: the
// noise calls for a long window and the steering for a short one, and one
// window cannot be both. With the 21 s window the noise needs, a rate step of
// d ppm leaves the line's leading end off by about 3*d microseconds while the
// window slides across it -- 45 us for the 15 ppm a millisecond phase
// correction produces.
//
// `sample_bias_ns` is how much later an edge's window middle reads than the
// transition itself. The counter is sampled somewhere inside the register
// read, the stamp is taken after it; the caller passes half the read's cost,
// which centres an unknown that is otherwise entirely on one side.
[[nodiscard]] inline ControllerLine
    fit_controller(std::span<const Edge> edges, std::size_t recent, double sample_bias_ns) {
    ControllerLine result;
    if (edges.size() < 8)
        return result;

    // An edge caught in a wide window says less than one caught in a narrow
    // window, and it says so itself: the width is the uncertainty. Drop the
    // wide ones before any fitting -- they are the preempted reads.
    std::vector<double> widths;
    widths.reserve(edges.size());
    for (const Edge& edge : edges)
        widths.push_back(edge.width_ns);
    const auto middle = static_cast<std::ptrdiff_t>(widths.size() / 2);
    std::ranges::nth_element(widths, widths.begin() + middle);
    const double width_limit = 3.0 * widths[widths.size() / 2];

    std::vector<double> microframe;
    std::vector<double> raw;
    std::vector<double> mono_minus_raw;
    microframe.reserve(edges.size());
    raw.reserve(edges.size());
    mono_minus_raw.reserve(edges.size());
    for (const Edge& edge : edges) {
        if (width_limit > 0.0 && edge.width_ns > width_limit)
            continue;
        microframe.push_back(edge.microframe);
        raw.push_back(edge.raw_ns - sample_bias_ns);
        mono_minus_raw.push_back(edge.mono_ns - edge.raw_ns);
    }
    if (microframe.size() < 8)
        return result;

    const Line raw_line = robust_line(microframe, raw);
    if (!raw_line.valid || !(raw_line.slope > 0.0))
        return result;

    // Stage two, on the tail. x is RAW time relative to the newest edge, so
    // the intercept is the clock difference "now".
    const std::size_t tail = std::min(std::max<std::size_t>(recent, 4), raw.size());
    const std::size_t first = raw.size() - tail;
    const double newest_raw = raw.back();
    std::vector<double> tail_x;
    std::vector<double> tail_y;
    tail_x.reserve(tail);
    tail_y.reserve(tail);
    for (std::size_t i = first; i < raw.size(); ++i) {
        tail_x.push_back(raw[i] - newest_raw);
        tail_y.push_back(mono_minus_raw[i]);
    }
    const Line difference = robust_line(tail_x, tail_y);
    if (!difference.valid)
        return result;
    // The kernel never steers by more than 500 ppm; ten times that is a
    // broken pair of readings, not a steered clock.
    if (std::fabs(difference.slope) > 5e-3)
        return result;

    const double reference_microframe = microframe.back();
    const double reference_raw = raw_line.at(reference_microframe);

    result.valid = true;
    result.reference_microframe = reference_microframe;
    result.reference_mono_ns = reference_raw + difference.at(reference_raw - newest_raw);
    result.raw_period_ns = raw_line.slope;
    result.mono_period_ns = raw_line.slope * (1.0 + difference.slope);
    result.sigma_ns = raw_line.sigma;
    result.used = raw_line.used;
    return result;
}

// The whole number of microframes between the boards' axis and the host
// controller's counter.
//
// Both count the Start-of-Frame packets of one controller, so they differ by a
// constant integer and nothing has to be fitted -- only found. Every exchange
// with a board gives a bound on it from causality alone: the board wrote its
// position after the host sent the request and before the host saw the reply,
// so
//
//     board - host_counter(received)  <  offset  <  board - host_counter(sent)
//
// and the offset lies in the intersection of all such intervals. On a
// high-speed link a round trip is shorter than a microframe, so the
// intersection soon holds exactly one integer. That integer is not an
// estimate: no assumption about where inside the round trip the board sampled
// went into it.
//
// What this replaces is rounding the median of (board - host_counter(middle
// of the round trip)). That assumes the outbound and return legs take equally
// long; they do not, the measured imbalance is a third of a microframe, and
// the rounding boundary is at a half. It held with that margin only while the
// board's position was exact -- and until 2026-10-03 the boards reported whole
// microframes, a value up to one microframe stale, which moved the median by
// half a microframe towards the boundary.
class OffsetBracket {
public:
    struct Estimate {
        bool exact = false;      // exactly one integer satisfies every bound
        std::int64_t offset = 0; // valid when exact
        double lower = 0.0;      // tightest bounds so far, for diagnostics
        double upper = 0.0;
        std::size_t observations = 0;
    };

    // `slack` widens every bound by that many microframes on each side, to
    // cover what the bound itself is computed from: the host counter line is
    // good to a fraction of a microsecond, the board's own fit likewise.
    OffsetBracket(std::size_t capacity, double slack, std::size_t minimum)
        : capacity_(std::max<std::size_t>(capacity, 1))
        , slack_(slack)
        , minimum_(std::max<std::size_t>(minimum, 1)) {}

    void clear() noexcept { bounds_.clear(); }

    // One exchange. `board` is the position the board reported; the other two
    // are the host counter (fractional microframes) when the request was sent
    // and when the reply was seen.
    void observe(double board, double host_at_send, double host_at_receive) {
        if (!(host_at_receive >= host_at_send))
            return;
        const Bound bound{.lower = board - host_at_receive, .upper = board - host_at_send};

        // A new bound that cannot hold together with the old ones means the
        // relation itself changed: a board re-anchored onto another wrap, or
        // a board that is not on this controller at all. The old bounds
        // describe something that is no longer true, so they go -- and with
        // them the lock, until enough new exchanges agree again.
        const auto [lower, upper] = intersection();
        if (!bounds_.empty()
            && (bound.lower - slack_ > upper + slack_ || bound.upper + slack_ < lower - slack_))
            bounds_.clear();

        bounds_.push_back(bound);
        while (bounds_.size() > capacity_)
            bounds_.pop_front();
    }

    [[nodiscard]] Estimate estimate() const noexcept {
        Estimate result;
        result.observations = bounds_.size();
        if (bounds_.empty())
            return result;
        const auto [lower, upper] = intersection();
        result.lower = lower;
        result.upper = upper;
        if (bounds_.size() < minimum_)
            return result;

        const double first = std::ceil(lower - slack_);
        const double last = std::floor(upper + slack_);
        if (first == last) {
            result.exact = true;
            result.offset = static_cast<std::int64_t>(first);
        }
        return result;
    }

private:
    struct Bound {
        double lower;
        double upper;
    };

    [[nodiscard]] Bound intersection() const noexcept {
        Bound result{.lower = -1e300, .upper = 1e300};
        for (const Bound& bound : bounds_) {
            result.lower = std::max(result.lower, bound.lower);
            result.upper = std::min(result.upper, bound.upper);
        }
        return result;
    }

    std::size_t capacity_;
    double slack_;
    std::size_t minimum_;
    std::deque<Bound> bounds_;
};

// An AxisMap that one thread replaces and any thread reads without waiting.
//
// Two slots and a count of completed writes: write n fills slot (n mod 2) and
// then publishes n. A reader copies the slot the count points at and keeps the
// copy if the count has not moved -- the only slot a writer may be in the
// middle of is the other one. It never waits for the writer, so a real-time
// reader cannot be held up by a writer that lost the CPU mid-update; it
// retries only if a whole write completed during its copy, which at a dozen
// writes a second it essentially never sees twice.
//
// One writer at a time; callers that have several serialise them.
class PublishedAxisMap {
public:
    void store(const AxisMap& map) noexcept {
        const std::uint32_t next = writes_.load(std::memory_order_relaxed) + 1U;
        Slot& slot = slots_[next & 1U];
        slot.source.store(map.source, std::memory_order_relaxed);
        slot.reference_microframe.store(map.reference_microframe, std::memory_order_relaxed);
        slot.reference_ns.store(map.reference_ns, std::memory_order_relaxed);
        slot.period_ns.store(map.period_ns, std::memory_order_relaxed);
        writes_.store(next, std::memory_order_seq_cst);
    }

    [[nodiscard]] AxisMap load() const noexcept {
        while (true) {
            const std::uint32_t seen = writes_.load(std::memory_order_seq_cst);
            const Slot& slot = slots_[seen & 1U];
            const AxisMap map{
                .source = slot.source.load(std::memory_order_relaxed),
                .reference_microframe = slot.reference_microframe.load(std::memory_order_relaxed),
                .reference_ns = slot.reference_ns.load(std::memory_order_relaxed),
                .period_ns = slot.period_ns.load(std::memory_order_relaxed),
            };
            if (writes_.load(std::memory_order_seq_cst) == seen)
                return map;
        }
    }

private:
    struct Slot {
        std::atomic<AxisMap::Source> source{AxisMap::Source::kNone};
        std::atomic<double> reference_microframe{0.0};
        std::atomic<std::int64_t> reference_ns{0};
        std::atomic<double> period_ns{125'000.0};
    };

    Slot slots_[2];
    std::atomic<std::uint32_t> writes_{0};
};

} // namespace libhcs::host::time::detail
