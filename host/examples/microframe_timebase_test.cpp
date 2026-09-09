// End to end: does routing Timeline through the host's own microframe counter
// actually replace the round trip, and by how much?
//
// THE MEASUREMENT TRICK. There is no third, better clock to judge both paths
// against -- so use the better of the two as the reference for the worse one.
// The MFINDEX path is fitted to about 20 ns and the round-trip path to
// microseconds, two and a half orders apart, so the scatter of
//
//     (round-trip estimate) - (MFINDEX estimate)
//
// is the round-trip path's own error with a negligible contribution from the
// reference. That number is not a claim about the new path; it is a measurement
// of the OLD one, and it is the quantity that decides whether any of this was
// worth building.
//
// WHAT LOCKING MEANS HERE. The two counters differ by a whole number of
// microframes and nothing else, so the only thing to learn is that integer.
// Section 2 shows it being learned and, more importantly, shows its SPREAD --
// if that grows, the two counters are not one clock and the class refuses to
// lock rather than handing out a confident wrong answer.
//
// Needs root and one board on usb3.
//   sudo ./microframe_timebase_test [seconds]

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/time/microframe_timebase.hpp>
#include <libhcs/time/timeline.hpp>

namespace {

using Board = libhcs::board::Hpm5321;
using libhcs::host::time::MicroframeTimebase;
using libhcs::host::time::timeline;
using Clock = MicroframeTimebase::Clock;

constexpr int kBus = 3;

struct Comparison {
    std::uint64_t microframe;
    double round_trip_ns; // when the report actually arrived
    double mfindex_ns;    // where the microframe axis says that microframe was
};

class Node final : public Board::Callback {
public:
    Node(std::string_view serial, MicroframeTimebase* timebase)
        : timebase_(timebase) {
        options_.dangerously_skip_version_checks = true;
        // Off by default; without it no kTimeStatus is exchanged at all and the
        // offset can never be learned.
        options_.set_enable_time_sync(true);
        board_ = std::make_unique<Board>(*this, serial, options_);
    }

    std::vector<Comparison> take() {
        const std::scoped_lock guard{mutex_};
        return comparisons_;
    }

    std::uint64_t reports() const { return reports_.load(std::memory_order_relaxed); }

private:
    void time_status_callback(const libhcs::data::TimeStatusView& data) override {
        if (data.state != libhcs::data::TimeState::kValid)
            return;
        reports_.fetch_add(1, std::memory_order_relaxed);

        // Sampled inside the callback, so this is the arrival of the report --
        // half a round trip after the midpoint the Timeline uses. That is a
        // CONSTANT bias, which the median below absorbs; the scatter, which is
        // what the comparison is about, is unaffected by it.
        const auto arrived = Clock::now();
        const auto direct = timebase_->host_time_of(data.microframe);
        if (!direct)
            return;

        const std::scoped_lock guard{mutex_};
        comparisons_.push_back(
            Comparison{
                .microframe = data.microframe,
                .round_trip_ns = static_cast<double>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(arrived.time_since_epoch())
                        .count()),
                .mfindex_ns = static_cast<double>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(direct->time_since_epoch())
                        .count()),
            });
    }

    MicroframeTimebase* timebase_;
    libhcs::board::AdvancedOptions options_;
    std::unique_ptr<Board> board_;
    mutable std::mutex mutex_;
    std::vector<Comparison> comparisons_;
    std::atomic<std::uint64_t> reports_{0};
};

std::vector<std::string> enumerate_serials() {
    std::vector<std::string> found;
    DIR* dir = opendir("/sys/bus/usb/devices");
    if (dir == nullptr)
        return found;
    while (const dirent* entry = readdir(dir)) {
        const std::string base = std::string{"/sys/bus/usb/devices/"} + entry->d_name;
        const auto read_line = [&](const char* leaf) -> std::string {
            FILE* file = fopen((base + "/" + leaf).c_str(), "re");
            if (file == nullptr)
                return {};
            char buffer[256] = {};
            const bool ok = fgets(buffer, sizeof(buffer), file) != nullptr;
            fclose(file);
            if (!ok)
                return {};
            std::string text{buffer};
            while (!text.empty() && (text.back() == '\n' || text.back() == '\r'))
                text.pop_back();
            return text;
        };
        if (read_line("idVendor") != "a511" || read_line("idProduct") != "5322")
            continue;
        const std::string serial = read_line("serial");
        if (!serial.empty())
            found.push_back(serial);
    }
    closedir(dir);
    std::ranges::sort(found);
    found.erase(std::unique(found.begin(), found.end()), found.end());
    return found;
}

double median_of(std::vector<double> values) {
    if (values.empty())
        return 0.0;
    std::ranges::sort(values);
    return values[values.size() / 2];
}

// Median absolute deviation, scaled to a sigma. Robust for the same reason it
// is used in microframe_source_test: a couple of descheduled callbacks would
// otherwise set the number.
double robust_sigma(const std::vector<double>& values, double centre) {
    if (values.empty())
        return 0.0;
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (const double value : values)
        deviations.push_back(std::fabs(value - centre));
    return 1.4826 * median_of(deviations);
}

} // namespace

int main(int argc, char** argv) {
    const int seconds = argc > 1 ? static_cast<int>(std::strtol(argv[1], nullptr, 10)) : 25;

    const std::vector<std::string> serials = enumerate_serials();
    if (serials.empty()) {
        printf("no board found\n");
        return 1;
    }

    printf("=== 1. bringing up the microframe timebase ===\n");
    MicroframeTimebase::Options options;
    options.edge_hertz = 12.0;
    auto timebase = MicroframeTimebase::open_for_usb_bus(kBus, options);
    if (!timebase) {
        printf("  unavailable -- Timeline keeps using the round trip, which is the\n");
        printf("  designed fallback. Nothing further to measure.\n");
        return 1;
    }
    printf("  source open on usb%d, hunting at %.0f Hz\n", kBus, options.edge_hertz);

    timeline().attach_microframe_timebase(timebase.get());
    Node node{serials.front(), timebase.get()};
    printf("  board %s attached\n\n", serials.front().c_str());

    printf("=== 2. learning the integer offset ===\n");
    const auto started = Clock::now();
    bool locked = false;
    for (int i = 0; i < seconds * 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
        if (i % 30 == 0) {
            const auto progress = timebase->status();
            printf(
                "    t=%4.1fs  edges=%zu  board reports=%" PRIu64 "  offset observations=%zu  %s\n",
                std::chrono::duration<double>{Clock::now() - started}.count(), progress.edges,
                node.reports(), progress.observations, progress.locked ? "LOCKED" : "");
        }
        if (!locked && timebase->locked()) {
            locked = true;
            const auto status = timebase->status();
            printf(
                "  LOCKED after %.1f s: offset %" PRId64
                " microframes, scatter %.3f, from %zu reports\n",
                std::chrono::duration<double>{Clock::now() - started}.count(),
                status.offset_microframes, status.offset_spread, status.observations);
        }
    }

    const auto status = timebase->status();
    if (!status.locked) {
        printf(
            "  NEVER LOCKED. scatter %.3f, drift %.3f microframes over %zu reports.\n",
            status.offset_spread, status.offset_drift, status.observations);
        printf("  A spread wider than the tolerance means the two counters are not\n");
        printf("  one clock -- check the board is really on usb%d.\n", kBus);
        timeline().attach_microframe_timebase(nullptr);
        return 1;
    }

    printf(
        "  offset held at %lld microframes\n", static_cast<long long>(status.offset_microframes));
    printf(
        "  scatter (MAD)  : %.3f microframes (tolerance %.2f) -- round-trip jitter\n",
        status.offset_spread, options.offset_tolerance_microframes);
    printf(
        "  drift          : %.3f microframes (tolerance %.2f) -- THIS is the one-clock test\n",
        status.offset_drift, options.offset_drift_tolerance_microframes);
    printf("  a walking offset would mean two clocks, and would drop the lock.\n");

    printf("\n=== 3. the fit behind it ===\n");
    printf("  edges in window   : %zu\n", status.edges);
    printf("  per-edge sigma    : %.0f ns (robust)\n", status.residual_sigma_ns);
    printf("  fitted phase      : %.0f ns\n", status.fitted_phase_ns);
    printf(
        "  microframe period : %.1f ns vs CLOCK_MONOTONIC_RAW (%+.2f ppm)\n",
        status.fitted_period_ns, ((status.fitted_period_ns / 125000.0) - 1.0) * 1e6);
    printf("  edge misses       : %llu\n", static_cast<unsigned long long>(status.edge_misses));

    printf("\n=== 4. what the round trip was actually worth ===\n");
    const std::vector<Comparison> comparisons = node.take();
    if (comparisons.size() < 16) {
        printf("  only %zu comparable reports; run longer\n", comparisons.size());
        timeline().attach_microframe_timebase(nullptr);
        return 1;
    }

    std::vector<double> differences;
    differences.reserve(comparisons.size());
    for (const Comparison& comparison : comparisons)
        differences.push_back(comparison.round_trip_ns - comparison.mfindex_ns);

    const double centre = median_of(differences);
    const double sigma = robust_sigma(differences, centre);
    std::vector<double> sorted = differences;
    std::ranges::sort(sorted);

    printf("  %zu board reports compared against the microframe axis\n", comparisons.size());
    printf("  constant part (uplink transit + half round trip) : %.1f us\n", centre / 1000.0);
    printf("  scatter about it, robust sigma                   : %.2f us\n", sigma / 1000.0);
    printf(
        "  full span                                        : %.1f us\n",
        (sorted.back() - sorted.front()) / 1000.0);
    printf("\n  That scatter is the ROUND TRIP's error, not this path's: the\n");
    printf(
        "  reference it is measured against is %.0fx tighter.\n",
        sigma / std::max(1.0, status.fitted_phase_ns));
    printf(
        "  Timeline's own fit averages it down by sqrt(N) over %d samples,\n",
        static_cast<int>(libhcs::host::time::Timeline::kSampleCapacity));
    printf(
        "  landing near %.2f us -- against %.0f ns from the counter directly.\n",
        sigma / std::sqrt(static_cast<double>(libhcs::host::time::Timeline::kSampleCapacity))
            / 1000.0,
        status.fitted_phase_ns);

    printf("\n=== 5. the fallback really falls back ===\n");
    const std::uint64_t probe = comparisons.back().microframe;
    const auto with_source = timeline().host_time_of(probe);
    timeline().attach_microframe_timebase(nullptr);
    const auto without_source = timeline().host_time_of(probe);
    printf(
        "  attached : %lld ns\n", static_cast<long long>(with_source.time_since_epoch().count()));
    printf(
        "  detached : %lld ns\n",
        static_cast<long long>(without_source.time_since_epoch().count()));
    printf(
        "  they differ by %.1f us, which is the improvement, and the detached\n",
        std::chrono::duration<double, std::micro>{without_source - with_source}.count());
    printf("  call still answered -- Timeline never depends on the source existing.\n");

    return 0;
}
