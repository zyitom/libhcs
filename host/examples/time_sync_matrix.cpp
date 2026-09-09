// The three time-alignment questions, answered in ONE run with EVERY attached
// board on ONE axis: board<->x86, boards<->x86, board<->board.
//
// The pieces existed already and each was checked on its own -- time_sync_test
// on a pair of HPM5321, mc02_time_sync_test on the mc02, microframe_timebase_test
// on the host counter. What no tool did was open them TOGETHER, and that is the
// only configuration in which the interesting failure can appear: the boards
// differ in USB speed (mc02 is Full Speed, one SOF per 1 ms frame; hpm5321 is
// High Speed, one per 125 us microframe) and in local timer (DWT at 550 MHz vs
// the HPM machine timer), so "each board is internally consistent" and "the
// boards agree with each other" are genuinely different claims.
//
// WHAT EACH SECTION MEASURES, and what limits it.
//
//   1. BOARD <-> x86. Two numbers per board, and they are not the same thing:
//      the board's own out-of-sample fit residual (how well the board knows
//      where it is on the microframe axis -- no host clock in the path), and
//      the host placement scatter (how well THIS machine can name the instant
//      of a reported microframe). The second is one to three orders worse and
//      is a property of the USB round trip, not of the timeline.
//
//   2. BOARDS <-> x86. Every board feeds one process-wide Timeline, so the
//      test is whether they land on the same ABSOLUTE axis. The failure to
//      catch is not a microsecond of skew, it is a whole wrap: the hardware
//      counter is 14 bits and rolls every 2.048 s, so a board that resolved a
//      different wrap is internally perfect and 16384 microframes out. That is
//      why the residuals below are also checked against multiples of 16384.
//
//   3. BOARD <-> BOARD. Pairwise, on the absolute microframe numbers. The
//      resolution of the host-observed comparison is capped at about one
//      microframe by uplink jitter, so it PROVES the wrap agreement and bounds
//      the skew, but it cannot see the sub-microsecond term. For that the
//      boards' own fit residuals are differenced -- what is common to two
//      boards running identical code cancels, leaving the term that would
//      become real skew when both act "at microframe k". Neither is a
//      substitute for a hardware two-way exchange (pulse_skew_test, which
//      measured 20 ns on the HPM5321 pair but needs -Dlibhcs_PULSE_TEST=ON
//      firmware and a UART0 cross-cable, so it cannot cover mc02 here).
//
// The host-side microframe counter is used when it can be opened (needs root):
// it replaces the USB round trip inside Timeline::host_time_of, which makes
// section 1's host placement a measurement of the round trip rather than a
// measurement through it. Without root everything still runs on the round-trip
// fit, which is the designed fallback, and the output says which path was used.
//
// Requires firmware built with the time base on (-Dlibhcs_TIME_SYNC=ON for
// hpm_board, -Dlibhcs_APP_TIME_SYNC=ON for mc02).
//
// Run:
//   ./time_sync_matrix [seconds] [serial-prefix ...]
//   sudo ./time_sync_matrix 30
//   sudo ./time_sync_matrix 40 D4-46A1     # one board, to isolate its own axis

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <dirent.h>

#include <libhcs/board/common.hpp>
#include <libhcs/board/mc02.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/time/microframe_timebase.hpp>
#include <libhcs/time/timeline.hpp>

namespace {

using Clock = libhcs::host::time::Timeline::Clock;
using libhcs::host::time::MicroframeTimebase;
using libhcs::host::time::timeline;

constexpr double kMicroframeUs = 125.0;
constexpr double kMicroframeNs = 125000.0;
// The hardware counter is 14 bits. A disagreement that is a multiple of this is
// a wrap resolution failure, not skew.
constexpr int64_t kFrindexModulus = 16384;
// One residual_mean_q16 LSB, in microseconds: Q16 ticks of a quarter-us timer.
constexpr double kResidualLsbUs = 0.25 / 65536.0;

// Reports arrive on the session keepalive; anything inside this window of the
// start is discarded, because the Timeline's fit is still open loop and the
// per-board wrap state machine may not have anchored yet.
constexpr std::chrono::seconds kWarmup{4};

struct Report {
    libhcs::data::TimeStatusView status;
    Clock::time_point arrival;
};

class Collector {
public:
    void push(const libhcs::data::TimeStatusView& status) {
        const std::scoped_lock guard{mutex_};
        reports_.push_back({status, Clock::now()});
    }

    std::vector<Report> take() const {
        const std::scoped_lock guard{mutex_};
        return reports_;
    }

private:
    mutable std::mutex mutex_;
    std::vector<Report> reports_;
};

// One attached board, whatever its type. The three sections only ever need its
// identity, its USB speed and its stream of kTimeStatus, so the board classes
// themselves stay behind this.
class Unit {
public:
    Unit(std::string serial, std::string kind, bool full_speed, int bus)
        : serial_(std::move(serial))
        , kind_(std::move(kind))
        , full_speed_(full_speed)
        , bus_(bus) {}

    virtual ~Unit() = default;
    Unit(const Unit&) = delete;
    Unit& operator=(const Unit&) = delete;
    Unit(Unit&&) = delete;
    Unit& operator=(Unit&&) = delete;

    const std::string& serial() const { return serial_; }
    const std::string& kind() const { return kind_; }
    bool full_speed() const { return full_speed_; }
    int bus() const { return bus_; }
    // Serials are long; the leading group is what the other tools print.
    std::string label() const { return serial_.substr(0, serial_.find('-', 3)); }
    std::vector<Report> reports() const { return collector_.take(); }

protected:
    Collector collector_;

private:
    std::string serial_;
    std::string kind_;
    bool full_speed_;
    int bus_;
};

class Hpm5321Unit final : public Unit, public libhcs::board::Hpm5321::Callback {
public:
    Hpm5321Unit(const std::string& serial, int bus)
        : Unit(serial, "hpm5321_dual_can", false, bus) {
        options_.set_enable_time_sync(true);
        board_ = std::make_unique<libhcs::board::Hpm5321>(*this, serial, options_);
    }

private:
    void time_status_callback(const libhcs::data::TimeStatusView& data) override {
        collector_.push(data);
    }

    libhcs::board::AdvancedOptions options_;
    std::unique_ptr<libhcs::board::Hpm5321> board_;
};

class Mc02Unit final : public Unit, public libhcs::board::Mc02::Callback {
public:
    Mc02Unit(const std::string& serial, int bus)
        : Unit(serial, "mc02", true, bus) {
        options_.set_enable_time_sync(true);
        board_ = std::make_unique<libhcs::board::Mc02>(*this, serial, options_);
    }

private:
    void time_status_callback(const libhcs::data::TimeStatusView& data) override {
        collector_.push(data);
    }

    libhcs::board::AdvancedOptions options_;
    std::unique_ptr<libhcs::board::Mc02> board_;
};

struct Device {
    std::string serial;
    std::string product;
    int bus = 0;
};

std::string read_attribute(const std::string& base, const char* leaf) {
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
}

// Every project board that carries a time base, in one sweep: the two product
// ids are hpm5321_dual_can (5322) and mc02 (0723).
std::vector<Device> enumerate_devices() {
    std::vector<Device> found;
    DIR* dir = opendir("/sys/bus/usb/devices");
    if (dir == nullptr)
        return found;
    while (const dirent* entry = readdir(dir)) {
        const std::string base = std::string{"/sys/bus/usb/devices/"} + entry->d_name;
        if (read_attribute(base, "idVendor") != "a511")
            continue;
        const std::string product = read_attribute(base, "idProduct");
        if (product != "5322" && product != "0723")
            continue;
        const std::string serial = read_attribute(base, "serial");
        if (serial.empty())
            continue;
        found.push_back(
            Device{
                .serial = serial,
                .product = product,
                .bus = std::atoi(read_attribute(base, "busnum").c_str()),
            });
    }
    closedir(dir);
    std::ranges::sort(found, [](const Device& left, const Device& right) {
        return left.serial < right.serial;
    });
    return found;
}

struct Stats {
    std::size_t count = 0;
    double median = 0.0;
    double mean = 0.0;
    // Robust 1 sigma: MAD scaled to a normal. A single re-anchor or a stalled
    // URB would otherwise dominate a plain standard deviation and hide the
    // steady-state number this is about.
    double spread = 0.0;
    double min = 0.0;
    double max = 0.0;
};

Stats summarise(std::vector<double> values) {
    Stats out;
    if (values.empty())
        return out;
    std::ranges::sort(values);
    out.count = values.size();
    out.min = values.front();
    out.max = values.back();
    out.median = values[values.size() / 2];
    double sum = 0.0;
    for (const double value : values)
        sum += value;
    out.mean = sum / static_cast<double>(values.size());
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (const double value : values)
        deviations.push_back(std::fabs(value - out.median));
    std::ranges::sort(deviations);
    out.spread = deviations[deviations.size() / 2] * 1.4826;
    return out;
}

// How many whole counter wraps a disagreement is, or zero when it is not one.
// Stated as a separate question from "how big is it" on purpose: 2.048 s of
// error and 2 us of error have nothing to do with each other, and only the
// first one can be present while every board reports itself healthy.
int64_t wrap_multiple(double microframes) {
    return std::llround(microframes / static_cast<double>(kFrindexModulus));
}

const char* state_name(libhcs::data::TimeState state) {
    switch (state) {
    case libhcs::data::TimeState::kInvalid: return "invalid";
    case libhcs::data::TimeState::kWaitingAnchor: return "waiting-anchor";
    case libhcs::data::TimeState::kValid: return "valid";
    }
    return "?";
}

std::string unix_string(std::chrono::system_clock::time_point when) {
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(when);
    const auto micros =
        std::chrono::duration_cast<std::chrono::microseconds>(when - seconds).count();
    const std::time_t as_time = std::chrono::system_clock::to_time_t(seconds);
    std::tm broken{};
    localtime_r(&as_time, &broken);
    char buffer[64] = {};
    std::strftime(buffer, sizeof(buffer), "%H:%M:%S", &broken);
    char out[96] = {};
    std::snprintf(out, sizeof(out), "%s.%06lld", buffer, static_cast<long long>(micros));
    return out;
}

// Per-board view used by all three sections, computed once.
struct Analysis {
    std::vector<Report> reports; // valid, past the warmup
    std::size_t total = 0;
    std::size_t valid = 0;
    uint32_t anomalies_during_run = 0;
    std::size_t state_transitions = 0;
    double ticks_per_microframe = 0.0;
    Stats board_residual_us; // the board's own out-of-sample fit error
    Stats placement_us;      // arrival minus host_time_of(microframe)
    double phase_us = 0.0;   // placement spread / sqrt(N)
};

Analysis analyse(const Unit& unit, Clock::time_point started) {
    Analysis out;
    const std::vector<Report> all = unit.reports();
    out.total = all.size();
    if (all.empty())
        return out;

    for (std::size_t index = 0; index < all.size(); index++) {
        if (all[index].status.state == libhcs::data::TimeState::kValid)
            out.valid++;
        if (index > 0 && all[index].status.state != all[index - 1].status.state)
            out.state_transitions++;
    }
    out.anomalies_during_run = all.back().status.anomaly_count - all.front().status.anomaly_count;
    out.ticks_per_microframe = all.back().status.ticks_per_microframe_q16 / 65536.0;

    std::vector<double> residual;
    std::vector<double> placement;
    for (const Report& report : all) {
        if (report.status.state != libhcs::data::TimeState::kValid)
            continue;
        if (report.arrival - started < kWarmup)
            continue;
        out.reports.push_back(report);
        if (report.status.residual_count != 0)
            residual.push_back(report.status.residual_mean_q16 * kResidualLsbUs);
        const auto host = timeline().host_time_of(report.status.microframe);
        placement.push_back(
            std::chrono::duration<double, std::micro>{report.arrival - host}.count());
    }
    out.board_residual_us = summarise(residual);
    out.placement_us = summarise(placement);
    if (out.placement_us.count > 1) {
        out.phase_us =
            out.placement_us.spread / std::sqrt(static_cast<double>(out.placement_us.count));
    }
    return out;
}

// Pairwise agreement on the ABSOLUTE microframe, as the host can see it.
//
// Each report of the second board is matched with the temporally nearest report
// of the first, and the comparison is made on the difference of both quantities,
// so every constant -- each board's own transit asymmetry included -- has to
// cancel for the residual to sit at zero.
struct Pairing {
    Stats residual_microframes;
    Stats derived_skew_us; // difference of the two boards' own fit residuals
    std::size_t matched = 0;
    std::size_t rejected_far = 0;
};

Pairing pair_up(const Analysis& first, const Analysis& second) {
    Pairing out;
    std::vector<double> residual;
    std::vector<double> skew;
    for (const Report& right : second.reports) {
        const Report* best = nullptr;
        double best_gap = 0.0;
        for (const Report& left : first.reports) {
            const double gap =
                std::fabs(std::chrono::duration<double, std::milli>{right.arrival - left.arrival}
                              .count());
            if (best == nullptr || gap < best_gap) {
                best = &left;
                best_gap = gap;
            }
        }
        if (best == nullptr)
            continue;
        // Reports come one per session refresh, so a match further away than
        // half of that is a gap in one stream, not a pairing.
        if (best_gap > 400.0) {
            out.rejected_far++;
            continue;
        }
        const double arrival_microframes =
            std::chrono::duration<double, std::micro>{right.arrival - best->arrival}.count()
            / kMicroframeUs;
        const double counted_microframes = static_cast<double>(right.status.microframe)
                                         - static_cast<double>(best->status.microframe);
        residual.push_back(counted_microframes - arrival_microframes);
        if (right.status.residual_count != 0 && best->status.residual_count != 0) {
            skew.push_back(
                (right.status.residual_mean_q16 - best->status.residual_mean_q16)
                * kResidualLsbUs);
        }
        out.matched++;
    }
    out.residual_microframes = summarise(residual);
    out.derived_skew_us = summarise(skew);
    return out;
}

} // namespace

int main(int argc, char** argv) {
    const int duration_s = argc > 1 ? std::atoi(argv[1]) : 25;
    if (duration_s <= 0) {
        fprintf(stderr, "seconds must be positive\n");
        return 1;
    }

    // Selecting a subset matters for more than convenience: the MFINDEX lock is
    // learned from a median over EVERY attached board's reports, so a board whose
    // own axis is offset or drifting can be masked by the others. Isolating one
    // board is the only way its counter is checked against the host's on its own.
    std::vector<std::string> filters;
    for (int index = 2; index < argc; index++)
        filters.emplace_back(argv[index]);

    std::vector<Device> devices = enumerate_devices();
    if (!filters.empty()) {
        std::erase_if(devices, [&filters](const Device& device) {
            return std::ranges::none_of(filters, [&device](const std::string& filter) {
                return device.serial.starts_with(filter);
            });
        });
    }
    if (devices.empty()) {
        fprintf(stderr, "no time-base capable board found (a511:5322 or a511:0723)\n");
        return 1;
    }

    printf("=== 0. rig ===\n");
    for (const Device& device : devices) {
        printf(
            "  %-40s  %s  usb bus %d\n", device.serial.c_str(),
            device.product == "0723" ? "mc02     (Full Speed, 1 ms frames)"
                                     : "hpm5321  (High Speed, 125 us microframes)",
            device.bus);
    }

    // Boards on two controllers are on two crystals, and no amount of fitting
    // will make them one axis. Say so before measuring, not after.
    bool one_controller = true;
    for (const Device& device : devices)
        one_controller = one_controller && device.bus == devices.front().bus;
    if (!one_controller) {
        printf(
            "  WARNING: not all boards are on one USB controller. Boards on different\n"
            "  controllers follow different SOF generators and CANNOT share an axis.\n");
    } else {
        printf("  all on one controller -> one SOF generator -> one axis is possible\n");
    }

    // The host's own microframe counter, when this process is allowed to map the
    // controller's BAR. It replaces the round trip inside host_time_of().
    std::unique_ptr<MicroframeTimebase> timebase;
    if (one_controller)
        timebase = MicroframeTimebase::open_for_usb_bus(devices.front().bus);
    if (timebase) {
        timeline().attach_microframe_timebase(timebase.get());
        printf("  host microframe counter: open on usb%d (MFINDEX)\n", devices.front().bus);
    } else {
        printf(
            "  host microframe counter: unavailable (needs root) -- Timeline stays on the\n"
            "  USB round trip, its designed fallback. Section 1's host placement is then\n"
            "  measured THROUGH the round trip instead of AGAINST it.\n");
    }

    std::vector<std::unique_ptr<Unit>> units;
    try {
        for (const Device& device : devices) {
            if (device.product == "0723")
                units.push_back(std::make_unique<Mc02Unit>(device.serial, device.bus));
            else
                units.push_back(std::make_unique<Hpm5321Unit>(device.serial, device.bus));
        }
    } catch (const std::exception& error) {
        fprintf(stderr, "error opening boards: %s\n", error.what());
        timeline().attach_microframe_timebase(nullptr);
        return 2;
    }

    const auto started = Clock::now();
    printf(
        "\nsampling %d s with %zu boards on one Timeline (first %lld s discarded as warmup)\n",
        duration_s, units.size(), static_cast<long long>(kWarmup.count()));
    std::this_thread::sleep_for(std::chrono::seconds{duration_s});

    std::vector<Analysis> analyses;
    analyses.reserve(units.size());
    for (const std::unique_ptr<Unit>& unit : units)
        analyses.push_back(analyse(*unit, started));

    printf("\n=== 1. board <-> x86, one board at a time ===\n");
    for (std::size_t index = 0; index < units.size(); index++) {
        const Unit& unit = *units[index];
        const Analysis& analysis = analyses[index];
        printf("\n  [%s] %s\n", unit.label().c_str(), unit.kind().c_str());
        if (analysis.reports.empty()) {
            printf(
                "    no valid time status -- firmware built without the time base?"
                " (%zu reports, %zu valid)\n",
                analysis.total, analysis.valid);
            continue;
        }
        printf(
            "    reports %zu (%zu valid, %zu used)   state %s   anomalies during run %u"
            "   state changes %zu\n",
            analysis.total, analysis.valid, analysis.reports.size(),
            state_name(analysis.reports.back().status.state), analysis.anomalies_during_run,
            analysis.state_transitions);
        printf(
            "    board crystal vs host SOF: %.4f ticks/microframe (%+.1f ppm)\n",
            analysis.ticks_per_microframe, (analysis.ticks_per_microframe / 500.0 - 1.0) * 1e6);
        if (analysis.board_residual_us.count != 0) {
            printf(
                "    board timeline (own out-of-sample fit, no host clock in the path):\n"
                "      bias %+.4f us   spread (1 sigma) %.4f us   worst report %+.4f us\n",
                analysis.board_residual_us.mean, analysis.board_residual_us.spread,
                std::fabs(analysis.board_residual_us.min) > std::fabs(analysis.board_residual_us.max)
                    ? analysis.board_residual_us.min
                    : analysis.board_residual_us.max);
        }
        printf(
            "    host placement (arrival minus %s):\n"
            "      offset %+.1f us (constant transit asymmetry, not removable here)\n"
            "      per-report spread %.1f us   worst %+.1f us   -> fitted phase %.2f us\n",
            timebase ? "MFINDEX time of that microframe" : "fitted host time of that microframe",
            analysis.placement_us.median, analysis.placement_us.spread,
            std::fabs(analysis.placement_us.min - analysis.placement_us.median)
                    > std::fabs(analysis.placement_us.max - analysis.placement_us.median)
                ? analysis.placement_us.min
                : analysis.placement_us.max,
            analysis.phase_us);
    }

    printf("\n=== 2. boards <-> x86, all at once on one absolute axis ===\n");
    printf("  Timeline: %s, %zu samples, measured period %.4f ns (%+.2f ppm vs 125000)\n",
           timeline().locked() ? "fitted" : "NOT FITTED", timeline().sample_count(),
           timeline().measured_period_ns(),
           (timeline().measured_period_ns() / kMicroframeNs - 1.0) * 1e6);
    if (timebase) {
        const auto status = timebase->status();
        printf(
            "  MFINDEX path: %s, offset %lld microframes, scatter %.3f, drift %.3f, phase %.0f ns\n",
            status.locked ? "LOCKED" : "not locked",
            static_cast<long long>(status.offset_microframes), status.offset_spread,
            status.offset_drift, status.fitted_phase_ns);
        // The same 125 us interval, measured against the two host clocks. The
        // MFINDEX fit uses CLOCK_MONOTONIC_RAW and the Timeline fit uses
        // CLOCK_MONOTONIC, so the DIFFERENCE between these two lines is not a
        // disagreement about the board -- it is the kernel's NTP slew, live.
        // Read the current slew with adjtimex(2) and it will match.
        printf(
            "  MFINDEX period: %.4f ns vs CLOCK_MONOTONIC_RAW (%+.2f ppm)"
            "  -> Timeline minus MFINDEX = %+.2f ppm of NTP slew\n",
            status.fitted_period_ns, (status.fitted_period_ns / kMicroframeNs - 1.0) * 1e6,
            (timeline().measured_period_ns() - status.fitted_period_ns) / kMicroframeNs * 1e6);
    }
    printf(
        "\n  %-10s %-8s %8s %12s %12s %10s\n", "board", "speed", "reports", "offset us",
        "spread us", "wraps off");
    bool wrap_clean = true;
    for (std::size_t index = 0; index < units.size(); index++) {
        const Analysis& analysis = analyses[index];
        if (analysis.reports.empty())
            continue;
        // The offset itself is a constant per board and is NOT the alignment
        // question; the wrap column is. A board that resolved a different wrap
        // shows a whole multiple of 16384 here while every other number it
        // reports stays perfect.
        const int64_t wraps = wrap_multiple(analysis.placement_us.median / kMicroframeUs);
        wrap_clean = wrap_clean && wraps == 0;
        printf(
            "  %-10s %-8s %8zu %12.1f %12.1f %10lld\n", units[index]->label().c_str(),
            units[index]->full_speed() ? "full" : "high", analysis.reports.size(),
            analysis.placement_us.median, analysis.placement_us.spread,
            static_cast<long long>(wraps));
    }
    printf(
        "\n  wrap resolution: %s\n",
        wrap_clean ? "every board resolved the SAME wrap (the 2.048 s failure is absent)"
                   : "MISMATCH -- at least one board is a whole 2.048 s counter wrap out");

    // The same instant, named by each board's last report. This is the shared
    // axis in the form a user actually consumes it.
    printf("\n  last report of each board, converted through the shared axis:\n");
    for (std::size_t index = 0; index < units.size(); index++) {
        if (analyses[index].reports.empty())
            continue;
        const Report& last = analyses[index].reports.back();
        printf(
            "    %-10s microframe %-12llu -> %s local\n", units[index]->label().c_str(),
            static_cast<unsigned long long>(last.status.microframe),
            unix_string(timeline().unix_time_of(last.status.microframe)).c_str());
    }

    printf("\n=== 3. board <-> board ===\n");
    if (units.size() < 2) {
        printf("  only one board attached; nothing to compare.\n");
    }
    for (std::size_t left = 0; left < units.size(); left++) {
        for (std::size_t right = left + 1; right < units.size(); right++) {
            if (analyses[left].reports.empty() || analyses[right].reports.empty())
                continue;
            const Pairing pairing = pair_up(analyses[left], analyses[right]);
            printf(
                "\n  %s vs %s   (%s vs %s)\n", units[right]->label().c_str(),
                units[left]->label().c_str(), units[right]->kind().c_str(),
                units[left]->kind().c_str());
            if (pairing.residual_microframes.count == 0) {
                printf("    no report pairs close enough in time to compare\n");
                continue;
            }
            const Stats& residual = pairing.residual_microframes;
            const int64_t wraps = wrap_multiple(residual.median);
            printf(
                "    absolute microframe agreement over %zu pairs (%zu rejected as too far apart):\n"
                "      median %+.2f microframes (%+.1f us)   spread %.2f   range %+.2f .. %+.2f\n",
                pairing.matched, pairing.rejected_far, residual.median,
                residual.median * kMicroframeUs, residual.spread, residual.min, residual.max);
            printf(
                "      wraps apart: %lld  -> %s\n", static_cast<long long>(wraps),
                wraps == 0 ? "same wrap, so the two boards are counting the same microframes"
                           : "DIFFERENT WRAP: internally consistent, mutually 2.048 s out");
            // Worth being blunt about: this residual is not independent evidence.
            // Substituting the definition of the placement error above turns it
            // into exactly the negative difference of the two section-2 offsets,
            // and the two lines below are printed together so that identity can
            // be checked rather than asserted. What the residual DOES establish
            // on its own is the wrap, which no single-board number can.
            const double from_offsets =
                -(analyses[right].placement_us.median - analyses[left].placement_us.median)
                / kMicroframeUs;
            printf(
                "      same quantity from the section-2 offsets: %+.2f microframes"
                " (must agree)\n",
                from_offsets);
            printf(
                "      so the CONSTANT above is host-side transit asymmetry -- the full-speed\n"
                "      board answers a frame later than a high-speed one -- and not axis\n"
                "      disagreement. Only the wrap column is independent evidence here;\n"
                "      uplink jitter caps the rest at about one microframe.\n");
            if (pairing.derived_skew_us.count != 0) {
                printf(
                    "    derived skew from the boards' own fit residuals (%zu pairs):\n"
                    "      bias %+.4f us   1 sigma %.4f us   (~%.3f us at 3 sigma)\n",
                    pairing.derived_skew_us.count, pairing.derived_skew_us.mean,
                    pairing.derived_skew_us.spread, 3.0 * pairing.derived_skew_us.spread);
                printf(
                    "      Derived, not observed: identical code on both ends cancels, leaving\n"
                    "      each board's microframe-to-local-timer conversion. It does NOT cover\n"
                    "      the actuation path -- for that see pulse_skew_test.\n");
            }
        }
    }

    timeline().attach_microframe_timebase(nullptr);
    return 0;
}
