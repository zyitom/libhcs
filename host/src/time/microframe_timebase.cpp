#include "host/src/time/microframe_timebase.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <pthread.h>
#include <sched.h>

#include "host/src/time/axis_fit.hpp"
#include "host/src/time/axis_map.hpp"
#include "host/src/time/microframe_source.hpp"

namespace libhcs::host::time {
namespace {

std::int64_t to_ns(MicroframeTimebase::Clock::time_point when) noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(when.time_since_epoch()).count();
}

} // namespace

struct MicroframeTimebase::Impl {
    MicroframeSource source;
    Options options;
    std::string pci_device;
    // How much later an edge's bracket middle reads than the transition: the
    // counter is sampled inside the register read and the clock is read after
    // it. Half the read is the centred guess for an offset that is otherwise
    // one-sided by the whole read (0.8-1.3 us here).
    double sample_bias_ns;

    mutable std::mutex mutex;

    std::deque<detail::Edge> edges;
    detail::ControllerLine line;

    detail::OffsetBracket bracket;
    bool locked = false;
    std::int64_t offset = 0;

    std::uint64_t edge_misses = 0;
    // The source's state as the edge thread last saw it. Cached because asking
    // the source takes its mutex, which the edge thread holds for the whole of
    // a hunt -- and the other caller here is a board's IO thread.
    MicroframeSource::State source_state = MicroframeSource::State::kValid;

    detail::PublishedAxisMap published;

    std::jthread thread;

    Impl(MicroframeSource moved, const Options& opts)
        : source(std::move(moved))
        , options(opts)
        , pci_device(source.pci_device())
        , sample_bias_ns(static_cast<double>(source.read_cost().count()) / 2.0)
        , bracket(opts.offset_capacity, opts.offset_slack_microframes, opts.offset_minimum) {}

    // Everything a reader may use, recomputed from the state above. Callers
    // hold `mutex`, which is also what serialises the two writers (the edge
    // thread and whichever thread delivers board reports).
    void publish_locked() noexcept {
        const auto estimate = bracket.estimate();
        locked = estimate.exact && line.valid && source_state == MicroframeSource::State::kValid;
        if (!locked) {
            published.store(AxisMap{});
            return;
        }
        offset = estimate.offset;
        published.store(
            AxisMap{
                .source = AxisMap::Source::kController,
                .reference_microframe = line.reference_microframe + static_cast<double>(offset),
                .reference_ns = static_cast<std::int64_t>(std::llround(line.reference_mono_ns)),
                .period_ns = line.mono_period_ns,
            });
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

        std::vector<detail::Edge> window;
        while (!stop.stop_requested()) {
            const auto edge = source.sample_edge();
            const auto state_now = source.state();
            {
                const std::scoped_lock guard{mutex};
                source_state = state_now;
                if (!edge) {
                    ++edge_misses;
                } else {
                    edges.push_back(
                        detail::Edge{
                            .microframe = static_cast<double>(edge->microframe),
                            .mono_ns = static_cast<double>(to_ns(edge->midpoint())),
                            .raw_ns = static_cast<double>(edge->raw_midpoint_ns()),
                            .width_ns = static_cast<double>(edge->uncertainty().count()),
                        });
                    while (edges.size() > options.edge_capacity)
                        edges.pop_front();
                    window.assign(edges.begin(), edges.end());
                    line = detail::fit_controller(window, options.recent_edges, sample_bias_ns);
                }
                // Also on a miss: a source that has gone invalid can never
                // come back, and a lock held against it would keep handing out
                // timestamps derived from a dead counter.
                publish_locked();
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
    double board_microframe, Clock::time_point sent_at, Clock::time_point received_at) {
    const std::scoped_lock guard{impl_->mutex};
    // Without the counter's line there is nothing to bound the offset against.
    // Nothing is lost by dropping the report: the next one comes in a quarter
    // of a second, and the line is there after eight edges.
    if (!impl_->line.valid)
        return;
    impl_->bracket.observe(
        board_microframe, impl_->line.microframe_at(static_cast<double>(to_ns(sent_at))),
        impl_->line.microframe_at(static_cast<double>(to_ns(received_at))));
    impl_->publish_locked();
}

AxisMap MicroframeTimebase::axis_map() const noexcept libhcs_NONBLOCKING {
    return impl_->published.load();
}

bool MicroframeTimebase::locked() const {
    const std::scoped_lock guard{impl_->mutex};
    return impl_->locked;
}

MicroframeTimebase::Status MicroframeTimebase::status() const {
    const std::scoped_lock guard{impl_->mutex};
    const auto estimate = impl_->bracket.estimate();
    return Status{
        .locked = impl_->locked,
        .offset_microframes = impl_->offset,
        .offset_lower = estimate.lower,
        .offset_upper = estimate.upper,
        .edges = impl_->edges.size(),
        .edges_used = impl_->line.used,
        .observations = estimate.observations,
        .raw_period_ns = impl_->line.raw_period_ns,
        .mono_period_ns = impl_->line.mono_period_ns,
        .residual_sigma_ns = impl_->line.sigma_ns,
        .edge_misses = impl_->edge_misses,
        .source_state = impl_->source_state,
    };
}

const std::string& MicroframeTimebase::pci_device() const noexcept { return impl_->pci_device; }

} // namespace libhcs::host::time
