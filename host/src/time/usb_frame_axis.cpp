#include "host/src/time/usb_frame_axis.hpp"

#include <map>
#include <string>

#include "host/src/time/axis_fit.hpp"

namespace libhcs::host::time {
namespace {

int64_t to_ns(UsbFrameAxis::Clock::time_point when) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(when.time_since_epoch()).count();
}

// The registry: one axis per host controller, keyed by the controller's PCI
// address. Entries live for the process -- handlers keep converting against
// their axis across reconnects -- and the mutex only guards creation and
// lookup, never a conversion. A bus whose PCI device cannot be found (a
// non-xHCI bus, a non-USB transport) shares the "" entry: one unbound axis is
// no worse than the single process-wide axis this used to be.
std::mutex& registry_mutex() {
    static std::mutex mutex;
    return mutex;
}

std::map<std::string, std::shared_ptr<UsbFrameAxis>>& registry() {
    static std::map<std::string, std::shared_ptr<UsbFrameAxis>> map;
    return map;
}

} // namespace

std::shared_ptr<UsbFrameAxis> UsbFrameAxis::for_usb_bus(int bus_number) {
    const std::string key = pci_device_for_usb_bus(bus_number);
    const std::scoped_lock guard{registry_mutex()};
    std::shared_ptr<UsbFrameAxis>& axis = registry()[key];
    if (!axis) {
        axis = std::shared_ptr<UsbFrameAxis>(new UsbFrameAxis());
        // Reads that controller's own counter for the life of the process.
        // Failing (no permission, not a PCI xHCI) is the round-trip fit's job
        // to cover; a shared "" entry has no counter to read at all.
        (void)axis->use_controller_of_usb_bus(bus_number);
    }
    return axis;
}

struct UsbFrameAxis::Published {
    detail::PublishedAxisMap map;
};

UsbFrameAxis::UsbFrameAxis()
    : origin_(Clock::now())
    // Sampled once, on purpose. See the header: a live steady->system offset
    // would let an NTP step move every microframe's wall-clock time, including
    // ones already used to schedule something.
    , unix_origin_(std::chrono::system_clock::now())
    , published_(std::make_unique<Published>()) {}

UsbFrameAxis::~UsbFrameAxis() = default;

bool UsbFrameAxis::use_controller_of_usb_bus(int bus_number) {
    const std::string wanted = pci_device_for_usb_bus(bus_number);
    if (wanted.empty())
        return false;

    const std::scoped_lock guard{mutex_};
    if (controller_)
        return controller_->pci_device() == wanted;

    controller_ = MicroframeTimebase::open_for_usb_bus(bus_number);
    controller_view_.store(controller_.get(), std::memory_order_release);
    return controller_ != nullptr;
}

std::optional<MicroframeTimebase::Status> UsbFrameAxis::controller_status() const {
    const std::scoped_lock guard{mutex_};
    if (!controller_)
        return std::nullopt;
    return controller_->status();
}

AxisMap UsbFrameAxis::axis_map() const noexcept libhcs_NONBLOCKING {
    if (const auto* controller = controller_view_.load(std::memory_order_acquire)) {
        const AxisMap direct = controller->axis_map();
        if (direct.valid())
            return direct;
    }
    return published_->map.load();
}

uint64_t UsbFrameAxis::anchor_for(Clock::time_point when) const {
    const std::scoped_lock guard{mutex_};
    if (!fitted_) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(when - origin_);
        return static_cast<uint64_t>(elapsed.count() / kMicroframePeriod.count());
    }
    const double offset_ns = static_cast<double>(to_ns(when)) - fit_reference_ns_;
    return static_cast<uint64_t>(
        std::llround(fit_reference_microframe_ + (offset_ns / fit_period_ns_)));
}

void UsbFrameAxis::observe(
    double microframe, Clock::time_point sent_at, Clock::time_point received_at) {
    // Midpoint of the round trip, not the arrival instant: the board wrote its
    // position somewhere between our send and this arrival, and splitting the
    // difference cancels the bulk of the USB transit bias. What is left is the
    // down/up asymmetry, which is the floor on how well THIS fit can place a
    // microframe on the host clock.
    const auto sampled_at = sent_at + ((received_at - sent_at) / 2);

    const std::scoped_lock guard{mutex_};

    // A board that has not been anchored yet reports its own boot-relative
    // origin, which is not on this axis and would wreck the fit. The caller
    // filters on state, but reject the obviously-off case here too: once
    // fitted, a sample more than a second off the line cannot be a jitter
    // outlier, it is a different axis.
    if (fitted_) {
        const double predicted_ns =
            fit_reference_ns_ + ((microframe - fit_reference_microframe_) * fit_period_ns_);
        const double error_ns = static_cast<double>(to_ns(sampled_at)) - predicted_ns;
        if (std::fabs(error_ns) > 1e9) {
            // Treat it as a restart of the axis rather than an outlier to drop:
            // silently ignoring it forever would strand the fit on a dead board.
            sample_head_ = 0;
            sample_count_ = 0;
            fitted_ = false;
            publish_locked();
        }
    }

    // The controller source needs board reports only to bound the constant
    // integer offset between the two counters, and it bounds it with both ends
    // of the round trip -- see detail::OffsetBracket.
    if (controller_)
        controller_->observe_board(microframe, sent_at, received_at);

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

void UsbFrameAxis::publish_locked() noexcept {
    if (!fitted_) {
        published_->map.store(AxisMap{});
        return;
    }
    published_->map.store(
        AxisMap{
            .source = AxisMap::Source::kRoundTrip,
            .reference_microframe = fit_reference_microframe_,
            .reference_ns = static_cast<std::int64_t>(std::llround(fit_reference_ns_)),
            .period_ns = fit_period_ns_,
        });
}

void UsbFrameAxis::refit_locked() {
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
    publish_locked();
}

bool UsbFrameAxis::locked() const {
    const std::scoped_lock guard{mutex_};
    return fitted_;
}

std::size_t UsbFrameAxis::sample_count() const {
    const std::scoped_lock guard{mutex_};
    return sample_count_;
}

UsbFrameAxis::Clock::time_point UsbFrameAxis::host_time_of(double microframe) const {
    // One map for the whole conversion: whichever source is answering -- the
    // controller's counter or the round-trip fit -- the line cannot change
    // under the arithmetic.
    const AxisMap map = axis_map();
    if (map.valid())
        return map.time_of(microframe);

    // No fit yet: the open-loop axis the anchors are being generated from.
    const std::scoped_lock guard{mutex_};
    return origin_
         + std::chrono::nanoseconds{static_cast<int64_t>(
             std::llround(microframe * static_cast<double>(kMicroframePeriod.count())))};
}

UsbFrameAxis::Clock::time_point UsbFrameAxis::host_time_of(uint64_t microframe) const {
    return host_time_of(static_cast<double>(microframe));
}

std::chrono::system_clock::time_point UsbFrameAxis::unix_time_of(uint64_t microframe) const {
    const auto host = host_time_of(microframe);
    const std::scoped_lock guard{mutex_};
    return unix_origin_
         + std::chrono::duration_cast<std::chrono::system_clock::duration>(host - origin_);
}

uint64_t UsbFrameAxis::microframe_at_unix(std::chrono::system_clock::time_point when) const {
    Clock::time_point host{};
    {
        const std::scoped_lock guard{mutex_};
        host = origin_ + std::chrono::duration_cast<Clock::duration>(when - unix_origin_);
    }
    return anchor_for(host);
}

double UsbFrameAxis::measured_period_ns() const {
    const std::scoped_lock guard{mutex_};
    return fitted_ ? fit_period_ns_ : 0.0;
}

} // namespace libhcs::host::time
