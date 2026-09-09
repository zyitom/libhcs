#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <mutex>
#include <stop_token>
#include <thread>
#include <vector>

#include <pthread.h>
#include <sched.h>

#include <libhcs/time/microframe_timebase.hpp>

namespace libhcs::host::time {
namespace {

std::int64_t to_ns(MicroframeTimebase::Clock::time_point when) noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(when.time_since_epoch()).count();
}

struct LineFit {
    bool valid = false;
    double reference_microframe = 0.0;
    double intercept_ns = 0.0;
    double slope_ns = 0.0;
    double residual_sigma_ns = 0.0;
};

// Least squares of time against microframe number. Referenced to the first
// sample so the sums stay small: microframe numbers run to 10^10 within a day
// and squaring those raw would lose the precision this whole class exists for.
LineFit fit(const std::vector<double>& microframes, const std::vector<double>& times_ns) {
    LineFit result;
    const std::size_t count = microframes.size();
    if (count < 8)
        return result;

    const double base_microframe = microframes.front();
    const double base_ns = times_ns.front();

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        const double x = microframes[i] - base_microframe;
        const double y = times_ns[i] - base_ns;
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }
    const double n = static_cast<double>(count);
    const double denominator = (n * sum_xx) - (sum_x * sum_x);
    if (denominator == 0.0)
        return result;

    const double slope = ((n * sum_xy) - (sum_x * sum_y)) / denominator;
    const double intercept = (sum_y - (slope * sum_x)) / n;

    std::vector<double> absolute;
    absolute.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const double x = microframes[i] - base_microframe;
        absolute.push_back(std::fabs((times_ns[i] - base_ns) - ((slope * x) + intercept)));
    }
    std::ranges::sort(absolute);

    result.valid = true;
    result.reference_microframe = base_microframe;
    result.intercept_ns = base_ns + intercept;
    result.slope_ns = slope;
    // Robust, for the same reason microframe_source_test reports it that way: a
    // handful of edges displaced by something outside the poll loop would
    // otherwise dominate a plain sigma and make a good fit look bad.
    result.residual_sigma_ns = 1.4826 * absolute[absolute.size() / 2];
    return result;
}

} // namespace

struct MicroframeTimebase::Impl {
    MicroframeSource source;
    Options options;

    mutable std::mutex mutex;

    std::deque<double> edge_microframes;
    std::deque<double> edge_steady_ns;
    std::deque<double> edge_raw_ns;
    LineFit steady_fit; // what host_time_of uses
    LineFit raw_fit;    // CLOCK_MONOTONIC_RAW, for the ppm figure only

    std::deque<double> offsets;
    bool locked = false;
    std::int64_t offset = 0;
    double offset_spread = 0.0;
    double offset_drift = 0.0;

    std::uint64_t edge_misses = 0;

    std::jthread thread;

    Impl(MicroframeSource moved, const Options& opts)
        : source(std::move(moved))
        , options(opts) {}

    void refit_locked() {
        const std::vector<double> microframes{edge_microframes.begin(), edge_microframes.end()};
        const std::vector<double> steady{edge_steady_ns.begin(), edge_steady_ns.end()};
        const std::vector<double> raw{edge_raw_ns.begin(), edge_raw_ns.end()};
        steady_fit = fit(microframes, steady);
        raw_fit = fit(microframes, raw);
    }

    // Microframe the host counter held at a given steady-clock instant. Used
    // only to turn a board report into an offset estimate.
    [[nodiscard]] std::optional<double> host_microframe_at_locked(std::int64_t steady_ns) const {
        if (!steady_fit.valid || steady_fit.slope_ns <= 0.0)
            return std::nullopt;
        return steady_fit.reference_microframe
             + ((static_cast<double>(steady_ns) - steady_fit.intercept_ns) / steady_fit.slope_ns);
    }

    void reassess_lock_locked() {
        if (offsets.size() < options.offset_minimum) {
            locked = false;
            return;
        }
        // KEPT UNROUNDED ON PURPOSE. Rounding each estimate to an integer first
        // looks harmless -- the answer is an integer, after all -- but when the
        // true fractional offset sits near a half, rounding flips neighbouring
        // samples between two integers and the halves below then differ by a
        // whole microframe of pure quantization. That read as 1 microframe of
        // drift on a link with none, three runs in four, right at the tolerance.
        // Round once, at the end. [Caught by repeat runs, 2026-09-08.]
        std::vector<double> sorted{offsets.begin(), offsets.end()};
        std::ranges::sort(sorted);
        const double median = sorted[sorted.size() / 2];

        // SPREAD IS MEASURED BY MAD, NOT BY MIN-TO-MAX.
        //
        // The first version used the full span, and it does not work: the span
        // of a jittery sample grows with how many samples you have, so a
        // perfectly healthy offset crept from 1 to 2 microframes purely by
        // collecting more reports and sat on the tolerance. The round-trip
        // jitter feeding these estimates spans 242 us -- about two microframes
        // -- so that was the span reporting the jitter, not a problem.
        // [Caught by repeat runs of microframe_timebase_test, 2026-09-08.]
        std::vector<double> deviations;
        deviations.reserve(sorted.size());
        for (const double value : sorted)
            deviations.push_back(std::fabs(value - median));
        std::ranges::sort(deviations);
        const double mad = deviations[deviations.size() / 2];

        // AND THE REAL QUESTION IS DRIFT, NOT SCATTER. Scatter is the round
        // trip; a WALKING offset is the thing that means these are two clocks
        // and not one. Compare the median of the older half against the newer
        // half in insertion order -- jitter cancels, a walk does not.
        const std::size_t half = offsets.size() / 2;
        std::vector<double> older{
            offsets.begin(), offsets.begin() + static_cast<std::ptrdiff_t>(half)};
        std::vector<double> newer{
            offsets.begin() + static_cast<std::ptrdiff_t>(half), offsets.end()};
        std::ranges::sort(older);
        std::ranges::sort(newer);
        const double drift = std::fabs(newer[newer.size() / 2] - older[older.size() / 2]);

        offset = std::llround(median);
        offset_spread = mad;
        offset_drift = drift;
        locked = mad <= options.offset_tolerance_microframes
              && drift <= options.offset_drift_tolerance_microframes && steady_fit.valid;
    }

    void hunt(const std::stop_token& stop) {
        if (options.core >= 0) {
            cpu_set_t set;
            CPU_ZERO(&set);
            CPU_SET(options.core, &set);
            (void)sched_setaffinity(0, sizeof(set), &set);
        }
        (void)pthread_setname_np(pthread_self(), "mf-timebase");

        const auto interval =
            options.edge_hertz > 0.0
                ? std::chrono::nanoseconds{static_cast<std::int64_t>(1e9 / options.edge_hertz)}
                : std::chrono::nanoseconds{0};

        while (!stop.stop_requested()) {
            const auto edge = source.sample_edge();
            {
                const std::scoped_lock guard{mutex};
                if (!edge) {
                    ++edge_misses;
                    // A source that has gone invalid can never come back, and
                    // holding a lock against it would keep handing out
                    // timestamps derived from a dead counter.
                    if (source.state() != MicroframeSource::State::kValid)
                        locked = false;
                } else {
                    edge_microframes.push_back(static_cast<double>(edge->microframe));
                    edge_steady_ns.push_back(static_cast<double>(to_ns(edge->midpoint())));
                    edge_raw_ns.push_back(static_cast<double>(edge->raw_midpoint_ns()));
                    while (edge_microframes.size() > options.edge_capacity) {
                        edge_microframes.pop_front();
                        edge_steady_ns.pop_front();
                        edge_raw_ns.pop_front();
                    }
                    refit_locked();
                    reassess_lock_locked();
                }
            }
            if (interval.count() > 0)
                std::this_thread::sleep_for(interval);
        }
    }
};

MicroframeTimebase::MicroframeTimebase(MicroframeSource source, const Options& options)
    : impl_(std::make_unique<Impl>(std::move(source), options)) {
    impl_->thread = std::jthread{[this](const std::stop_token& stop) { impl_->hunt(stop); }};
}

MicroframeTimebase::~MicroframeTimebase() {
    if (impl_ && impl_->thread.joinable()) {
        impl_->thread.request_stop();
        impl_->thread.join();
    }
}

std::unique_ptr<MicroframeTimebase> MicroframeTimebase::open_for_usb_bus(int bus_number) {
    return open_for_usb_bus(bus_number, Options{});
}

std::unique_ptr<MicroframeTimebase>
    MicroframeTimebase::open_for_usb_bus(int bus_number, const Options& options) {
    auto source = MicroframeSource::for_usb_bus(bus_number);
    if (!source)
        return nullptr;
    return std::make_unique<MicroframeTimebase>(std::move(*source), options);
}

void MicroframeTimebase::observe_board(
    std::uint64_t board_microframe, Clock::time_point sampled_at) {
    const std::scoped_lock guard{impl_->mutex};
    const auto host_microframe = impl_->host_microframe_at_locked(to_ns(sampled_at));
    if (!host_microframe)
        return;

    // Stored unrounded; the rounding happens once, on the median. See
    // reassess_lock_locked for why doing it per sample corrupts the drift test.
    impl_->offsets.push_back(static_cast<double>(board_microframe) - *host_microframe);
    while (impl_->offsets.size() > impl_->options.offset_capacity)
        impl_->offsets.pop_front();
    impl_->reassess_lock_locked();
}

std::optional<MicroframeTimebase::Clock::time_point>
    MicroframeTimebase::host_time_of(std::uint64_t board_microframe) const {
    const std::scoped_lock guard{impl_->mutex};
    if (!impl_->locked || !impl_->steady_fit.valid)
        return std::nullopt;

    const double host_microframe =
        static_cast<double>(board_microframe) - static_cast<double>(impl_->offset);
    const double ns =
        impl_->steady_fit.intercept_ns
        + (impl_->steady_fit.slope_ns * (host_microframe - impl_->steady_fit.reference_microframe));
    return Clock::time_point{std::chrono::nanoseconds{static_cast<std::int64_t>(std::llround(ns))}};
}

bool MicroframeTimebase::locked() const {
    const std::scoped_lock guard{impl_->mutex};
    return impl_->locked;
}

MicroframeTimebase::Status MicroframeTimebase::status() const {
    const std::scoped_lock guard{impl_->mutex};
    const double edges = static_cast<double>(impl_->edge_microframes.size());
    return Status{
        .attached = true,
        .locked = impl_->locked,
        .offset_microframes = impl_->offset,
        .offset_spread = impl_->offset_spread,
        .offset_drift = impl_->offset_drift,
        .edges = impl_->edge_microframes.size(),
        .observations = impl_->offsets.size(),
        .fitted_period_ns = impl_->raw_fit.slope_ns,
        .residual_sigma_ns = impl_->steady_fit.residual_sigma_ns,
        .fitted_phase_ns =
            edges > 0.0 ? impl_->steady_fit.residual_sigma_ns / std::sqrt(edges) : 0.0,
        .edge_misses = impl_->edge_misses,
        .source_state = impl_->source.state(),
    };
}

} // namespace libhcs::host::time
