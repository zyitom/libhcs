// Shared time base across every attached HPM5321: does each board resolve the
// SAME absolute microframe, and what is that timeline worth in Unix time?
//
// sof_probe answered the prior question -- the hardware microframe counter runs
// clean and identically on every board (firmware/hpm_board/SOF_TIMEBASE.md).
// What it could not answer is whether the two boards agree on WHICH microframe,
// because the counter wraps every 2.048 s and each board boots with its own
// origin. That is what the host anchor resolves, and it is what this tool
// checks.
//
// The measurement to look at is the cross-board residual:
//
//     (microframe_i - microframe_0) - (arrival_i - arrival_0) / 125 us
//
// computed on the ABSOLUTE microframe numbers the boards report. Two properties
// matter, and they fail differently:
//
//   * a residual near zero says the boards are counting the same microframes;
//   * a residual that is a MULTIPLE OF 16384 says they resolved different
//     wraps -- each board internally consistent, mutually a whole 2.048 s
//     apart. That is the failure this tool exists to catch, because nothing on
//     a single board can see it.
//
// The residual's resolution is limited by USB uplink jitter (tens of
// microseconds), so it proves agreement to about a microframe, not to a
// microsecond. Sub-microframe agreement is a property of the hardware counter
// and was measured in step 1; nothing here can improve on it or damage it.
//
// Requires firmware built with -Dlibhcs_TIME_SYNC=ON.
//
// Run:
//   ./time_sync_test [seconds] [serial ...]

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
#include <thread>
#include <vector>

#include <dirent.h>

#include <libhcs/board/hpm5321.hpp>
#include <libhcs/time/timeline.hpp>

namespace {

using Clock = libhcs::host::time::Timeline::Clock;
using libhcs::host::time::timeline;

constexpr double kMicroframeUs = 125.0;
constexpr int64_t kFrindexModulus = 16384;

struct Report {
    libhcs::data::TimeStatusView status;
    Clock::time_point arrival;
};

class Receiver final : public libhcs::board::Hpm5321::Callback {
public:
    std::vector<Report> take() const {
        const std::scoped_lock guard{mutex_};
        return reports_;
    }

private:
    void time_status_callback(const libhcs::data::TimeStatusView& data) override {
        const std::scoped_lock guard{mutex_};
        reports_.push_back({data, Clock::now()});
    }

    mutable std::mutex mutex_;
    std::vector<Report> reports_;
};

std::vector<std::string> enumerate_boards() {
    std::vector<std::string> found;
    DIR* dir = opendir("/sys/bus/usb/devices");
    if (!dir)
        return found;
    while (const dirent* entry = readdir(dir)) {
        const std::string base = std::string{"/sys/bus/usb/devices/"} + entry->d_name;
        const auto read_line = [&](const char* leaf) -> std::string {
            const std::string path = base + "/" + leaf;
            FILE* file = fopen(path.c_str(), "re");
            if (!file)
                return {};
            char buffer[256] = {};
            if (!fgets(buffer, sizeof(buffer), file)) {
                fclose(file);
                return {};
            }
            fclose(file);
            std::string value{buffer};
            while (!value.empty() && (value.back() == '\n' || value.back() == '\r'))
                value.pop_back();
            return value;
        };
        if (read_line("idVendor") != "a511" || read_line("idProduct") != "5322")
            continue;
        const std::string serial = read_line("serial");
        if (!serial.empty())
            found.push_back(serial);
    }
    closedir(dir);
    std::sort(found.begin(), found.end());
    return found;
}

const char* state_name(libhcs::data::TimeState state) {
    switch (state) {
    case libhcs::data::TimeState::kInvalid: return "invalid";
    case libhcs::data::TimeState::kWaitingAnchor: return "waiting-anchor";
    case libhcs::data::TimeState::kValid: return "valid";
    }
    return "?";
}

} // namespace

int main(int argc, char** argv) {
    const int duration_s = argc > 1 ? std::atoi(argv[1]) : 20;
    if (duration_s <= 0) {
        fprintf(stderr, "seconds must be positive\n");
        return 1;
    }
    std::vector<std::string> serials;
    for (int index = 2; index < argc; index++)
        serials.emplace_back(argv[index]);
    if (serials.empty())
        serials = enumerate_boards();
    if (serials.empty()) {
        fprintf(stderr, "no hpm5321_dual_can boards (a511:5322) found\n");
        return 1;
    }

    printf("boards: %zu\n", serials.size());

    std::vector<std::unique_ptr<Receiver>> receivers;
    std::vector<std::unique_ptr<libhcs::board::Hpm5321>> boards;
    // One options object per board: AdvancedOptions is deliberately non-copyable.
    std::vector<std::unique_ptr<libhcs::board::AdvancedOptions>> options;
    try {
        for (const std::string& serial : serials) {
            receivers.push_back(std::make_unique<Receiver>());
            options.push_back(std::make_unique<libhcs::board::AdvancedOptions>());
            options.back()->set_enable_time_sync(true);
            boards.push_back(
                std::make_unique<libhcs::board::Hpm5321>(
                    *receivers.back(), serial, *options.back()));
        }
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    printf("sampling for %d s (anchor rides the keepalive, one per 250 ms)...\n", duration_s);
    std::this_thread::sleep_for(std::chrono::seconds{duration_s});

    std::vector<std::vector<Report>> reports;
    reports.reserve(receivers.size());
    for (const auto& receiver : receivers)
        reports.push_back(receiver->take());

    std::vector<double> board_residual_us(serials.size(), 0.0);
    std::vector<double> board_spread_us(serials.size(), 0.0);
    std::vector<bool> board_has_residual(serials.size(), false);

    for (size_t index = 0; index < serials.size(); index++) {
        printf("\n=== board %s ===\n", serials[index].c_str());
        if (reports[index].empty()) {
            printf("  no time status -- is the firmware built with -Dlibhcs_TIME_SYNC=ON?\n");
            continue;
        }
        const Report& first = reports[index].front();
        const Report& last = reports[index].back();
        size_t valid = 0;
        size_t transitions = 0;
        for (size_t report = 0; report < reports[index].size(); report++) {
            if (reports[index][report].status.state == libhcs::data::TimeState::kValid)
                valid++;
            if (report > 0
                && reports[index][report].status.state != reports[index][report - 1].status.state)
                transitions++;
        }
        // The anomaly counter is free-running since boot and the enumeration
        // window alone contributes hundreds, so the total says nothing. What
        // matters is whether it moved DURING the run: any increase means the
        // timeline was invalidated and re-anchored while we were watching.
        const uint32_t anomalies_during_run =
            last.status.anomaly_count - first.status.anomaly_count;
        // The fitted rate is the board crystal against the host's USB clock; the
        // nominal is 500 ticks per microframe on the 4 MHz machine timer.
        const double ticks_per_microframe = last.status.ticks_per_microframe_q16 / 65536.0;
        printf(
            "  reports %zu (%zu valid)   state %s   microframe %llu\n", reports[index].size(),
            valid, state_name(last.status.state),
            static_cast<unsigned long long>(last.status.microframe));
        printf(
            "  anomalies during the run %u (since boot %u)   state transitions %zu\n",
            anomalies_during_run, last.status.anomaly_count, transitions);
        printf(
            "  fitted local ticks per microframe %.4f (nominal 500) -> board crystal %+.1f ppm\n",
            ticks_per_microframe, (ticks_per_microframe / 500.0 - 1.0) * 1e6);

        // Prediction error of this board's own fit. One timer tick is 0.25 us
        // and the payload carries Q16 ticks.
        //
        // Read the two statistics for what they are. Each report already carries
        // the MEAN over ~33 out-of-sample predictions, so interrupt jitter is
        // suppressed in it by sqrt(33); what survives across reports is the slow
        // wandering of the fitted line itself, which is the part that turns into
        // skew. So:
        //
        //   * the mean of those means is the systematic BIAS. Least squares
        //     forces it to ~0, so a nonzero value here would mean something is
        //     wrong, not that the timeline is off by that much.
        //   * their SPREAD is the real figure of merit -- how far the board's
        //     idea of "now" moves between one report and the next.
        //
        // The worst single sample is neither: it is dominated by the interrupt
        // jitter of that one observation, and a scheduled action never pays it,
        // because it fires off a timer comparison rather than out of the handler.
        std::vector<double> report_means_us;
        double residual_abs_max_us = 0.0;
        uint64_t residual_samples = 0;
        for (const Report& report : reports[index]) {
            if (report.status.state != libhcs::data::TimeState::kValid
                || report.status.residual_count == 0)
                continue;
            report_means_us.push_back(report.status.residual_mean_q16 / 65536.0 / 4.0);
            residual_abs_max_us =
                std::max(residual_abs_max_us, report.status.residual_abs_max_q16 / 65536.0 / 4.0);
            residual_samples += report.status.residual_count;
        }
        if (!report_means_us.empty()) {
            double sum = 0.0;
            double worst = 0.0;
            for (const double value : report_means_us) {
                sum += value;
                worst = std::max(worst, std::fabs(value));
            }
            const double bias = sum / static_cast<double>(report_means_us.size());
            double variance = 0.0;
            for (const double value : report_means_us)
                variance += (value - bias) * (value - bias);
            variance /= static_cast<double>(report_means_us.size());
            const double spread = std::sqrt(variance);

            board_residual_us[index] = bias;
            board_spread_us[index] = spread;
            board_has_residual[index] = true;
            printf(
                "  fit prediction error over %llu out-of-sample points in %zu reports:\n"
                "    bias %+.4f us   spread (1 sigma) %.4f us   worst report mean %.4f us\n"
                "    worst single sample %.3f us (interrupt jitter, not timeline error)\n",
                static_cast<unsigned long long>(residual_samples), report_means_us.size(), bias,
                spread, worst, residual_abs_max_us);
        }
    }

    // How well THIS HOST can place a microframe the board reported. Ported from
    // mc02_time_sync_test: the host pairs the board's reported instant with the
    // ROUND-TRIP MIDPOINT, so any delay between the moment the board latched its
    // counter and the moment it answered appears here as bias plus uniform
    // noise. Removing that delay is what the reply-time interpolation in
    // HostSession::time_anchor_deserialized_callback is for.
    for (size_t index = 0; index < serials.size(); index++) {
        std::vector<double> placement_us;
        for (const Report& report : reports[index]) {
            if (report.status.state != libhcs::data::TimeState::kValid)
                continue;
            const auto fitted = timeline().host_time_of(report.status.microframe);
            placement_us.push_back(
                std::chrono::duration<double, std::micro>{report.arrival - fitted}.count());
        }
        if (placement_us.size() < 10)
            continue;
        double sum = 0.0;
        for (const double value : placement_us)
            sum += value;
        const double mean = sum / static_cast<double>(placement_us.size());
        double variance = 0.0;
        double worst = 0.0;
        for (const double value : placement_us) {
            variance += (value - mean) * (value - mean);
            worst = std::max(worst, std::fabs(value - mean));
        }
        const double sigma = std::sqrt(variance / static_cast<double>(placement_us.size()));
        // Robust spread as well as sigma. The tail here is host scheduling --
        // single samples 100..900 us out -- and a handful of them dominate a
        // standard deviation computed over ~800 points, which is enough to hide
        // a systematic term of a few tens of microseconds. The 16th..84th
        // percentile half-width is the same quantity as sigma for a Gaussian but
        // is blind to that tail.
        std::vector<double> sorted = placement_us;
        std::sort(sorted.begin(), sorted.end());
        const auto pick = [&sorted](double q) {
            return sorted[static_cast<size_t>(q * static_cast<double>(sorted.size() - 1))];
        };
        const double robust = (pick(0.84) - pick(0.16)) / 2.0;
        printf(
            "\n=== host placement, board %s ===\n"
            "  offset %+.1f us (median %+.1f)   sigma %.1f us   robust spread %.1f us"
            "   worst %.1f us\n"
            "  -> fitted phase uncertainty ~%.2f us (robust / sqrt(%zu))\n",
            serials[index].c_str(), mean, pick(0.5), sigma, robust, worst,
            robust / std::sqrt(static_cast<double>(placement_us.size())), placement_us.size());
    }

    // The number the whole exercise is for. The microframe COUNTER is provably
    // identical on every board (SOF_TIMEBASE.md section 2), and the wrap is
    // provably identical (above), so the only place cross-board skew can come
    // from is each board's own conversion of a microframe to its local timer --
    // which is precisely the prediction error each board just reported. Terms
    // common to both boards (identical code on identical silicon) cancel in the
    // difference, so this estimate is tighter than either board's raw number.
    if (serials.size() >= 2 && board_has_residual[0]) {
        printf("\n=== estimated cross-board skew ===\n");
        for (size_t board = 1; board < serials.size(); board++) {
            if (!board_has_residual[board])
                continue;
            const double bias_difference = board_residual_us[board] - board_residual_us[0];
            // Two independent boards, so their spreads add in quadrature.
            const double combined_spread = std::sqrt(
                board_spread_us[board] * board_spread_us[board]
                + board_spread_us[0] * board_spread_us[0]);
            printf(
                "  %s vs reference: bias %+.4f us, 1 sigma %.4f us  (~%.3f us at 3 sigma)\n",
                serials[board].c_str(), bias_difference, combined_spread,
                std::fabs(bias_difference) + 3.0 * combined_spread);
        }
        printf(
            "  Derived, not observed. The microframe COUNTER is identical on every board and\n"
            "  so is the resolved wrap, so the only place skew can enter is each board's own\n"
            "  microframe-to-local-timer conversion -- which is what these residuals measure.\n"
            "  It does NOT cover the actuation path: firing an action at a computed local time\n"
            "  adds whatever that path costs, and measuring THAT needs two boards acting at\n"
            "  once (a scope, or CAN TSU two-way transfer).\n");
    }

    printf("\n=== host timeline (Unix mapping) ===\n");
    auto& line = timeline();
    printf(
        "  fitted %s   samples %zu   measured microframe period %.4f ns (nominal 125000)\n",
        line.locked() ? "yes" : "no", line.sample_count(), line.measured_period_ns());
    if (line.locked()) {
        printf(
            "  host clock vs USB SOF clock: %+.2f ppm\n",
            (line.measured_period_ns() / 125000.0 - 1.0) * 1e6);
    }
    if (!reports.empty() && !reports[0].empty()) {
        const Report& last = reports[0].back();
        const auto unix_time = line.unix_time_of(last.status.microframe);
        const auto seconds =
            std::chrono::duration_cast<std::chrono::seconds>(unix_time.time_since_epoch());
        const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
            unix_time.time_since_epoch() - seconds);
        const std::time_t as_time = seconds.count();
        char buffer[64] = {};
        std::tm tm_value{};
        localtime_r(&as_time, &tm_value);
        std::strftime(buffer, sizeof(buffer), "%F %T", &tm_value);
        printf(
            "  microframe %llu  ->  %s.%06lld local time\n",
            static_cast<unsigned long long>(last.status.microframe), buffer,
            static_cast<long long>(micros.count()));

        // Round trip: a microframe converted to Unix time and back must land on
        // itself. It checks the two directions against each other, nothing more
        // -- absolute accuracy against true UTC is bounded by this machine's own
        // clock discipline, which no measurement here can see.
        const uint64_t back = line.microframe_at_unix(unix_time);
        printf(
            "  round trip microframe -> unix -> microframe: %lld microframe error\n",
            static_cast<long long>(static_cast<int64_t>(back - last.status.microframe)));
    }
    return 0;
}
