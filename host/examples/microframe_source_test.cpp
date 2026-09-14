// What the host-side microframe counter is actually worth, measured.
//
// Every number this prints was an estimate somewhere in the design discussion,
// and two of them were pure arithmetic with nothing behind it: the phase
// precision of one edge, and what edge hunting costs. Those are sections 4 and
// 5. The rest exists so that those two can be trusted -- a phase number from a
// counter that turned out not to be running is worse than no number.
//
// SECTION 4 IS THE POINT. It reports the same quantity twice, from independent
// routes, and they have to agree:
//
//   the BRACKET   each edge is caught between two reads, so its uncertainty is
//                 that interval -- a bound, not an estimate, and it cannot be
//                 optimistic. A uniform edge inside it has sigma = width/sqrt(12).
//   the RESIDUAL  fitting host time against microframe number over hundreds of
//                 edges leaves a residual whose sigma is the real per-edge
//                 scatter, whatever its cause.
//
// If the residual is much larger than the bracket predicts, something outside
// the read loop is moving the edges and the bracket is not the whole story. If
// it is much smaller, the fit is being flattered by correlated samples. Only
// agreement licenses dividing by sqrt(N) to claim a fitted phase.
//
// RATE USES CLOCK_MONOTONIC_RAW, and section 6 shows why in the most direct
// way available: it measures the same period against both clocks. This kernel
// slews CLOCK_MONOTONIC at a saturated -500 ppm, which is 600x the crystal
// disagreement the measurement is trying to resolve, so a period fitted against
// it is not a little wrong, it is measuring NTP.
//
// Needs root: the counter is in the controller's PCI BAR. No board required --
// this measures the host end alone, deliberately, so a bad number here cannot
// be blamed on firmware.
//
//   sudo ./microframe_source_test [usb_bus] [edges]

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <libhcs/time/microframe_source.hpp>

namespace {

using libhcs::host::time::MicroframeSource;
using Clock = MicroframeSource::Clock;

constexpr int kDefaultBus = 3;
constexpr std::size_t kDefaultEdges = 300;
constexpr std::size_t kWarmupEdges = 30;

struct Fit {
    double period_ns;
    double residual_sigma_ns;
    // Median absolute deviation, scaled to a sigma. A plain sigma is dominated
    // by a handful of outliers -- one run in three here came back at 758 ns
    // against a typical 310 ns purely because a few edges were displaced by
    // something outside the poll loop. Reporting both separates "the method is
    // this precise" from "this run was disturbed", which averaging over runs
    // would have destroyed.
    double residual_mad_ns;
    double residual_max_ns;
    std::size_t outliers; // residuals beyond 5x the robust sigma
    std::size_t samples;
};

// Least squares of host time against microframe number. The slope is the
// microframe period as this clock sees it; the residual sigma is the per-sample
// scatter that section 4 cross-checks against the bracket width.
Fit fit_period(const std::vector<std::uint64_t>& microframes, const std::vector<double>& times_ns) {
    const std::size_t n = microframes.size();
    if (n < 3)
        return Fit{
            .period_ns = 0.0,
            .residual_sigma_ns = 0.0,
            .residual_mad_ns = 0.0,
            .residual_max_ns = 0.0,
            .outliers = 0,
            .samples = n,
        };

    const std::uint64_t base_microframe = microframes.front();
    const double base_ns = times_ns.front();

    double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double x = static_cast<double>(microframes[i] - base_microframe);
        const double y = times_ns[i] - base_ns;
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }
    const double count = static_cast<double>(n);
    const double denominator = (count * sum_xx) - (sum_x * sum_x);
    if (denominator == 0.0)
        return Fit{
            .period_ns = 0.0,
            .residual_sigma_ns = 0.0,
            .residual_mad_ns = 0.0,
            .residual_max_ns = 0.0,
            .outliers = 0,
            .samples = n,
        };

    const double slope = ((count * sum_xy) - (sum_x * sum_y)) / denominator;
    const double intercept = (sum_y - (slope * sum_x)) / count;

    double sum_squared = 0.0;
    std::vector<double> absolute;
    absolute.reserve(n);
    double worst = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double x = static_cast<double>(microframes[i] - base_microframe);
        const double residual = (times_ns[i] - base_ns) - ((slope * x) + intercept);
        sum_squared += residual * residual;
        absolute.push_back(std::fabs(residual));
        worst = std::max(worst, std::fabs(residual));
    }

    std::vector<double> sorted = absolute;
    std::ranges::sort(sorted);
    // 1.4826 makes the MAD a consistent estimator of sigma for a normal sample.
    const double mad = 1.4826 * sorted[sorted.size() / 2];
    const std::size_t outliers = static_cast<std::size_t>(std::ranges::count_if(
        absolute, [mad](double value) { return mad > 0.0 && value > 5.0 * mad; }));

    return Fit{
        .period_ns = slope,
        .residual_sigma_ns = std::sqrt(sum_squared / (count - 2.0)),
        .residual_mad_ns = mad,
        .residual_max_ns = worst,
        .outliers = outliers,
        .samples = n,
    };
}

double percentile(std::vector<double> values, double fraction) {
    if (values.empty())
        return 0.0;
    std::ranges::sort(values);
    const std::size_t index =
        static_cast<std::size_t>(fraction * static_cast<double>(values.size() - 1));
    return values[index];
}

double ppm_of(double period_ns) {
    return ((period_ns / static_cast<double>(MicroframeSource::kMicroframePeriod.count())) - 1.0)
         * 1e6;
}

void print_discovery(int bus) {
    printf("=== 1. what this machine has ===\n");
    const std::vector<std::string> controllers = libhcs::host::time::usb_controller_pci_devices();
    if (controllers.empty()) {
        printf("  no PCI USB controller found -- this is the ARM SoC case, where xHCI\n");
        printf("  is a platform device with no BAR in sysfs. Nothing here can work.\n");
        return;
    }
    for (const std::string& device : controllers) {
        const std::vector<int> buses = libhcs::host::time::usb_buses_of_pci_device(device);
        printf("  %s  buses:", device.c_str());
        for (const int number : buses)
            printf(" usb%d", number);
        if (buses.empty())
            printf(" (none)");

        auto probe = MicroframeSource::for_pci_device(device);
        if (probe)
            printf(
                "   -> usable, %.1f Hz, read %lld ns\n", probe->validated_rate_hz(),
                static_cast<long long>(probe->read_cost().count()));
        else
            printf(
                "   -> unusable: %.*s\n",
                static_cast<int>(MicroframeSource::describe(probe.error()).size()),
                MicroframeSource::describe(probe.error()).data());
    }
    printf("\n  the board's bus is usb%d\n", bus);
}

// A counter that only looked right would pass every check above and still be
// useless, so this one spans more than two wraps (2.048 s each) and insists the
// 64-bit extension stays monotonic and keeps pace with elapsed time.
bool check_continuity(MicroframeSource& source) {
    printf("\n=== 2. wrap handling and continuity ===\n");
    constexpr auto duration = std::chrono::milliseconds{5000};
    constexpr auto interval = std::chrono::milliseconds{20};

    const auto started = Clock::now();
    auto first = source.sample();
    if (!first) {
        printf(
            "  FAILED: first sample rejected (%.*s)\n",
            static_cast<int>(MicroframeSource::describe(source.state()).size()),
            MicroframeSource::describe(source.state()).data());
        return false;
    }

    std::uint64_t previous = first->microframe;
    std::uint64_t samples = 1;
    std::uint64_t backwards = 0;
    auto last = *first;

    while (Clock::now() - started < duration) {
        std::this_thread::sleep_for(interval);
        const auto sample = source.sample();
        if (!sample) {
            printf(
                "  FAILED after %" PRIu64 " samples: %.*s\n", samples,
                static_cast<int>(MicroframeSource::describe(source.state()).size()),
                MicroframeSource::describe(source.state()).data());
            return false;
        }
        if (sample->microframe < previous)
            ++backwards;
        previous = sample->microframe;
        last = *sample;
        ++samples;
    }

    const double elapsed_s = static_cast<double>(last.raw_ns - first->raw_ns) / 1e9;
    const double advanced = static_cast<double>(last.microframe - first->microframe);
    const double wraps = advanced / static_cast<double>(MicroframeSource::kCounterModulus);
    printf(
        "  %llu samples over %.2f s, %.1f wraps crossed\n",
        static_cast<unsigned long long>(samples), elapsed_s, wraps);
    printf(
        "  counter advanced %.0f microframes, elapsed time says %.0f\n", advanced,
        elapsed_s * 8000.0);
    printf("  went backwards: %llu times\n", static_cast<unsigned long long>(backwards));
    const bool ok =
        backwards == 0 && wraps > 2.0 && std::fabs(advanced - (elapsed_s * 8000.0)) < 100.0;
    printf("  %s\n", ok ? "PASS -- the extension survives wraps" : "FAIL");
    return ok;
}

} // namespace

int main(int argc, char** argv) {
    const int bus = argc > 1 ? static_cast<int>(std::strtol(argv[1], nullptr, 10)) : kDefaultBus;
    const std::size_t edge_count =
        argc > 2 ? static_cast<std::size_t>(std::strtol(argv[2], nullptr, 10)) : kDefaultEdges;

    print_discovery(bus);

    auto opened = MicroframeSource::for_usb_bus(bus);
    if (!opened) {
        printf(
            "\nCANNOT OPEN usb%d: %.*s\n", bus,
            static_cast<int>(MicroframeSource::describe(opened.error()).size()),
            MicroframeSource::describe(opened.error()).data());
        return 1;
    }
    MicroframeSource source = std::move(*opened);

    printf("\n  using %s for usb%d\n", source.pci_device().c_str(), bus);
    printf("  validated at %.1f Hz (nominal 8000)\n", source.validated_rate_hz());
    printf(
        "  counter modulus: %u (%d-bit wrap)\n", source.counter_modulus(),
        source.counter_modulus() == 1024 ? 10 : 14);

    printf("\n=== 3. cost of one read ===\n");
    printf("  MFINDEX read      : %lld ns\n", static_cast<long long>(source.read_cost().count()));
    printf("  USB round trip    : ~44000 ns  [measured previously, for scale]\n");
    printf(
        "  ratio             : %.0fx cheaper\n",
        44000.0 / static_cast<double>(std::max<std::int64_t>(1, source.read_cost().count())));

    if (!check_continuity(source))
        return 1;

    // ---- 4 and 5: the two numbers that were arithmetic until now ----
    printf("\n=== 4. phase precision of one edge ===\n");

    for (std::size_t i = 0; i < kWarmupEdges; ++i) {
        (void)source.sample_edge();
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    const MicroframeSource::EdgeStats after_warmup = source.edge_stats();
    printf(
        "  guard after %zu warm-up edges: %lld ns (started at 80000)\n", kWarmupEdges,
        static_cast<long long>(after_warmup.guard.count()));

    std::vector<std::uint64_t> microframes;
    std::vector<double> raw_times;
    std::vector<double> monotonic_times;
    std::vector<double> brackets;
    microframes.reserve(edge_count);
    raw_times.reserve(edge_count);
    monotonic_times.reserve(edge_count);
    brackets.reserve(edge_count);

    const MicroframeSource::EdgeStats before = source.edge_stats();
    const auto measure_started = Clock::now();
    for (std::size_t i = 0; i < edge_count; ++i) {
        const auto edge = source.sample_edge();
        if (edge) {
            microframes.push_back(edge->microframe);
            raw_times.push_back(static_cast<double>(edge->raw_midpoint_ns()));
            monotonic_times.push_back(
                static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        edge->midpoint().time_since_epoch())
                                        .count()));
            brackets.push_back(static_cast<double>(edge->uncertainty().count()));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
    const auto measure_elapsed = Clock::now() - measure_started;
    const MicroframeSource::EdgeStats after = source.edge_stats();

    if (microframes.size() < 10) {
        printf("  only %zu edges caught -- cannot say anything\n", microframes.size());
        return 1;
    }

    const double bracket_p50 = percentile(brackets, 0.50);
    const double bracket_p99 = percentile(brackets, 0.99);
    const double bracket_sigma = bracket_p50 / std::sqrt(12.0);

    const Fit raw_fit = fit_period(microframes, raw_times);
    const double fitted_phase =
        raw_fit.residual_mad_ns / std::sqrt(static_cast<double>(raw_fit.samples));

    printf(
        "  edges caught      : %zu of %zu over %.1f s\n", microframes.size(), edge_count,
        std::chrono::duration<double>{measure_elapsed}.count());
    printf("\n  route A -- the bracket (a bound, cannot be optimistic)\n");
    printf("    width p50       : %.0f ns\n", bracket_p50);
    printf("    width p99       : %.0f ns\n", bracket_p99);
    printf("    implied sigma   : %.0f ns   (width / sqrt(12), uniform edge)\n", bracket_sigma);
    printf("\n  route B -- the fit residual (the real scatter)\n");
    printf("    sigma           : %.0f ns   (outlier-sensitive)\n", raw_fit.residual_sigma_ns);
    printf(
        "    robust sigma    : %.0f ns   (from the MAD -- this is the method's precision)\n",
        raw_fit.residual_mad_ns);
    printf(
        "    worst residual  : %.0f ns, %zu edges beyond 5 robust sigma\n", raw_fit.residual_max_ns,
        raw_fit.outliers);
    if (raw_fit.residual_sigma_ns > 2.0 * raw_fit.residual_mad_ns)
        printf(
            "    this run was DISTURBED: the plain sigma is more than twice the\n"
            "    robust one, so a few displaced edges dominate it. The robust\n"
            "    number is what the method delivers; the gap is what the machine\n"
            "    did to it.\n");
    printf(
        "\n  agreement        : %.2fx  (B robust / A)\n",
        raw_fit.residual_mad_ns / std::max(1.0, bracket_sigma));
    const double agreement = raw_fit.residual_mad_ns / std::max(1.0, bracket_sigma);
    if (agreement > 0.5 && agreement < 2.0)
        printf(
            "    the two routes agree -- the read interval IS the error, and\n"
            "    averaging it down by sqrt(N) is legitimate\n");
    else if (agreement >= 2.0)
        printf(
            "    DISAGREE: the residual is larger than the bracket allows, so\n"
            "    something outside the poll loop is moving the edges. Do NOT\n"
            "    divide by sqrt(N) until that is explained.\n");
    else
        printf(
            "    DISAGREE: the residual is smaller than the bracket. Samples are\n"
            "    probably correlated; treat the bracket as the honest bound.\n");

    printf("\n  fitted phase over %zu edges: %.0f ns\n", raw_fit.samples, fitted_phase);
    printf("  for comparison, the USB round trip the Timeline uses today is the\n");
    printf("  ~19 us weak link this replaces.\n");

    printf("\n=== 5. what edge hunting cost ===\n");
    const std::uint64_t attempts = after.attempts - before.attempts;
    const std::uint64_t hits = after.hits - before.hits;
    const std::uint64_t misses = after.misses - before.misses;
    const std::uint64_t cold = after.cold_hunts - before.cold_hunts;
    const std::uint64_t reads = after.reads - before.reads;
    const auto busy = after.busy - before.busy;
    const double busy_per_edge =
        hits > 0 ? static_cast<double>(busy.count()) / static_cast<double>(hits) : 0.0;
    const double wall_s = std::chrono::duration<double>{measure_elapsed}.count();

    printf(
        "  attempts / hits / misses : %llu / %llu / %llu\n",
        static_cast<unsigned long long>(attempts), static_cast<unsigned long long>(hits),
        static_cast<unsigned long long>(misses));
    printf("  cold hunts (full period) : %llu\n", static_cast<unsigned long long>(cold));
    printf(
        "  reads per edge           : %.1f\n",
        hits > 0 ? static_cast<double>(reads) / static_cast<double>(hits) : 0.0);
    printf("  busy-poll per edge       : %.1f us\n", busy_per_edge / 1000.0);
    printf("  final guard              : %lld ns\n", static_cast<long long>(after.guard.count()));
    const std::uint64_t wake_samples = after.wake_samples - before.wake_samples;
    printf(
        "  sleep wake error mean    : %+.1f us\n",
        wake_samples > 0
            ? static_cast<double>((after.wake_error_total - before.wake_error_total).count())
                  / static_cast<double>(wake_samples) / 1000.0
            : 0.0);
    printf(
        "  sleep wake error high    : %+.1f us  <- what the guard has to cover\n",
        static_cast<double>(after.wake_error_high.count()) / 1000.0);
    printf(
        "  overshoots (waited a full period) : %llu of %llu\n",
        static_cast<unsigned long long>(after.overshoots - before.overshoots),
        static_cast<unsigned long long>(hits));
    printf(
        "  predicted edge lead      : %+.1f us  <- should equal the poll duration\n",
        wake_samples > 0 ? static_cast<double>(
                               (after.predicted_lead_total - before.predicted_lead_total).count())
                               / static_cast<double>(wake_samples) / 1000.0
                         : 0.0);
    printf(
        "\n  at this edge rate the hunter burned %.4f%% of one core\n",
        100.0 * static_cast<double>(busy.count()) / (wall_s * 1e9));
    printf("  holding 25 ns of phase needs re-anchoring at ~12 Hz, which costs\n");
    printf(
        "  %.4f%% of one core at the per-edge cost measured above.\n",
        100.0 * 12.0 * busy_per_edge / 1e9);

    printf("\n=== 6. rate, and why it must use CLOCK_MONOTONIC_RAW ===\n");
    const Fit monotonic_fit = fit_period(microframes, monotonic_times);
    printf(
        "  period vs CLOCK_MONOTONIC_RAW : %.1f ns  (%+.1f ppm)\n", raw_fit.period_ns,
        ppm_of(raw_fit.period_ns));
    printf(
        "  period vs CLOCK_MONOTONIC     : %.1f ns  (%+.1f ppm)\n", monotonic_fit.period_ns,
        ppm_of(monotonic_fit.period_ns));
    const double slew_ppm = ppm_of(monotonic_fit.period_ns) - ppm_of(raw_fit.period_ns);
    printf(
        "  difference                    : %+.1f ppm  <- the kernel's slew, not a crystal\n",
        slew_ppm);
    if (std::fabs(slew_ppm) > 50.0)
        printf(
            "\n  CLOCK_MONOTONIC is being disciplined hard right now. Any ppm figure\n"
            "  taken against it is measuring NTP. This is exactly the trap that\n"
            "  made two earlier measurements disagree by 576 ppm.\n");
    else
        printf(
            "\n  the two clocks agree at the moment, which does NOT make MONOTONIC\n"
            "  safe -- the slew comes and goes with NTP. Keep using RAW.\n");

    return 0;
}
