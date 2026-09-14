#include <cmath>
#include <cstddef>

#include <libhcs/time/microframe_timebase.hpp>
#include <libhcs/time/timeline.hpp>

namespace libhcs::host::time {
namespace {

int64_t to_ns(Timeline::Clock::time_point when) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(when.time_since_epoch()).count();
}

} // namespace

Timeline::Timeline()
    : origin_(Clock::now())
    // Sampled once, on purpose. See the header: a live steady->system offset
    // would let an NTP step move every microframe's wall-clock time, including
    // ones already used to schedule something.
    , unix_origin_(std::chrono::system_clock::now()) {}

Timeline& Timeline::instance() {
    static Timeline singleton;
    return singleton;
}

Timeline& timeline() { return Timeline::instance(); }

void Timeline::attach_microframe_timebase(MicroframeTimebase* timebase) {
    const std::scoped_lock guard{mutex_};
    microframe_timebase_ = timebase;
}

bool Timeline::microframe_timebase_locked() const {
    const std::scoped_lock guard{mutex_};
    return microframe_timebase_ != nullptr && microframe_timebase_->locked();
}

uint64_t Timeline::anchor_for(Clock::time_point when) const {
    const std::scoped_lock guard{mutex_};
    if (!fitted_) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(when - origin_);
        return static_cast<uint64_t>(elapsed.count() / kMicroframePeriod.count());
    }
    const double offset_ns = static_cast<double>(to_ns(when)) - fit_reference_ns_;
    return fit_reference_microframe_
         + static_cast<uint64_t>(std::llround(offset_ns / fit_period_ns_));
}

void Timeline::observe(uint64_t microframe, Clock::time_point sampled_at) {
    const std::scoped_lock guard{mutex_};

    // A board that has not been anchored yet reports its own boot-relative
    // origin, which is not on this axis and would wreck the fit. The caller
    // filters on state, but reject the obviously-off case here too: once
    // fitted, a sample more than a second off the line cannot be a jitter
    // outlier, it is a different axis.
    if (fitted_) {
        const double predicted_ns =
            fit_reference_ns_
            + (static_cast<double>(static_cast<int64_t>(microframe - fit_reference_microframe_))
               * fit_period_ns_);
        const double error_ns = static_cast<double>(to_ns(sampled_at)) - predicted_ns;
        if (std::fabs(error_ns) > 1e9) {
            // Treat it as a restart of the axis rather than an outlier to drop:
            // silently ignoring it forever would strand the fit on a dead board.
            sample_head_ = 0;
            sample_count_ = 0;
            fitted_ = false;
        }
    }

    // The microframe timebase needs board reports only to learn the constant
    // integer offset between the two counters. Once it has locked, this
    // timestamp stops mattering to it at all -- which is what makes the session
    // packet non-timing-sensitive.
    if (microframe_timebase_ != nullptr)
        microframe_timebase_->observe_board(microframe, sampled_at);

    samples_[(sample_head_ + sample_count_) % kSampleCapacity] =
        Sample{.microframe = microframe, .host_ns = to_ns(sampled_at)};
    if (sample_count_ < kSampleCapacity) {
        sample_count_++;
    } else {
        sample_head_ = (sample_head_ + 1) % kSampleCapacity;
    }

    // Two points are enough to define a line, but not enough for the jitter to
    // average out; wait for a window that makes the offset estimate worth
    // trusting before publishing a fit.
    if (sample_count_ >= 16)
        refit_locked();
}

void Timeline::refit_locked() {
    // Ordinary least squares of host_ns against microframe. Both are taken
    // relative to the oldest sample so the doubles keep their precision --
    // absolute steady_clock nanoseconds are ~10^13 and would leave only
    // microsecond resolution in a double's 53-bit mantissa.
    const Sample& base = samples_[sample_head_];
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    for (std::size_t index = 0; index < sample_count_; index++) {
        const Sample& sample = samples_[(sample_head_ + index) % kSampleCapacity];
        const auto x =
            static_cast<double>(static_cast<int64_t>(sample.microframe - base.microframe));
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
    // A period that is not within a few percent of nominal is a broken window,
    // not a crystal offset; publishing it would be worse than staying unfitted.
    const auto nominal = static_cast<double>(kMicroframePeriod.count());
    if (period_ns < nominal * 0.97 || period_ns > nominal * 1.03)
        return;

    const double intercept_ns = (sum_y - period_ns * sum_x) / count;
    fit_reference_microframe_ = base.microframe;
    fit_reference_ns_ = static_cast<double>(base.host_ns) + intercept_ns;
    fit_period_ns_ = period_ns;
    fitted_ = true;
}

bool Timeline::locked() const {
    const std::scoped_lock guard{mutex_};
    return fitted_;
}

std::size_t Timeline::sample_count() const {
    const std::scoped_lock guard{mutex_};
    return sample_count_;
}

// Callers must hold mutex_.
Timeline::Clock::time_point Timeline::host_time_of_locked(uint64_t microframe) const {
    // Prefer the controller's own counter when it is locked. It is the same
    // axis, read without a round trip, and it is not an estimate of the fit
    // below -- it is the quantity the fit below is trying to estimate.
    if (microframe_timebase_ != nullptr) {
        if (const auto direct = microframe_timebase_->host_time_of(microframe))
            return *direct;
    }

    if (!fitted_) {
        return origin_
             + std::chrono::nanoseconds{
                 static_cast<int64_t>(microframe) * kMicroframePeriod.count()};
    }
    const double ns =
        fit_reference_ns_
        + (static_cast<double>(static_cast<int64_t>(microframe - fit_reference_microframe_))
           * fit_period_ns_);
    return Clock::time_point{std::chrono::nanoseconds{static_cast<int64_t>(std::llround(ns))}};
}

Timeline::Clock::time_point Timeline::host_time_of(uint64_t microframe) const {
    const std::scoped_lock guard{mutex_};
    return host_time_of_locked(microframe);
}

Timeline::Clock::time_point Timeline::host_time_of(double microframe) const {
    // Deliberately expressed as two calls to the integer form rather than as its
    // own arithmetic: whichever source is answering -- the MFINDEX counter or the
    // round-trip fit -- both endpoints come from the SAME one, so the
    // interpolation cannot straddle a switch between them. "Same one" is
    // enforced here, not assumed: a single mutex acquisition covers both
    // endpoints, so a timebase attaching or dropping lock between two
    // independent calls cannot mix sources into one interpolation.
    const double whole = std::floor(microframe);
    const std::scoped_lock guard{mutex_};
    const auto base = host_time_of_locked(static_cast<uint64_t>(whole));
    const auto next = host_time_of_locked(static_cast<uint64_t>(whole) + 1);
    const double step_ns = std::chrono::duration<double, std::nano>{next - base}.count();
    return base
         + std::chrono::nanoseconds{
             static_cast<int64_t>(std::llround(step_ns * (microframe - whole)))};
}

std::chrono::system_clock::time_point Timeline::unix_time_of(uint64_t microframe) const {
    const auto host = host_time_of(microframe);
    const std::scoped_lock guard{mutex_};
    return unix_origin_
         + std::chrono::duration_cast<std::chrono::system_clock::duration>(host - origin_);
}

uint64_t Timeline::microframe_at_unix(std::chrono::system_clock::time_point when) const {
    Clock::time_point host{};
    {
        const std::scoped_lock guard{mutex_};
        host = origin_ + std::chrono::duration_cast<Clock::duration>(when - unix_origin_);
    }
    return anchor_for(host);
}

double Timeline::measured_period_ns() const {
    const std::scoped_lock guard{mutex_};
    return fitted_ ? fit_period_ns_ : 0.0;
}

} // namespace libhcs::host::time
