// Shared USB-SOF time base on mc02: how tight is the board's own timeline, and
// how well can it be placed on this x86 machine's clock?
//
// The mc02 port of the time base differs from hpm_board's in three ways that
// change what can be measured, so this is a separate tool rather than a flag on
// time_sync_test:
//
//   * FULL SPEED. mc02 has no high-speed PHY, so it gets one SOF per 1 ms frame
//     and its frame counter steps by 8 microframes per interrupt. Everything
//     downstream is on the same 125 us microframe axis, but the fitted rate and
//     the anomaly rules are checked against a step of 8, not 1.
//   * NO PTPC. The HPM part latches SOF into a 1588 clock in hardware and
//     timestamps CAN frames on the same counter, which is what let hpm_board
//     measure cross-board skew directly. The H723 has no equivalent, so the
//     PTPC columns of the status payload are zero here and every number below
//     comes from the board's own out-of-sample fit residual.
//   * THE LOCAL CLOCK IS DWT->CYCCNT at 550 MHz, converted to the protocol's
//     quarter-microsecond tick on the way out. 68750 cycles per microframe is
//     exactly 500 quarter-us, so the nominal-500 arithmetic below is unchanged.
//
// WHAT THE TWO SECTIONS MEAN, because they answer different questions and only
// one of them is limited by this PC:
//
//   "fit prediction error" is the BOARD's number. Each report carries the mean
//   of ~30 predictions the fit had not seen when it was published, so interrupt
//   jitter is already suppressed inside it; the SPREAD of those means across
//   reports is how far the board's idea of "now" wanders, and it is what would
//   become skew between two boards running this code. The host clock is not in
//   this path at all.
//
//   "host placement" is THIS MACHINE's number: the scatter of (report arrival
//   minus the fitted host time of the microframe it reported). It is dominated
//   by USB round-trip jitter, and at full speed the schedule quantizes to 1 ms,
//   so expect tens to hundreds of microseconds. That is the floor on naming a
//   microframe in x86 time -- and it is unrelated to the board number above.
//
// THE --causality FLAG puts a RIGOROUS TWO-SIDED BOUND on the one error this
// measurement otherwise cannot see: the constant offset left by the USB
// down/up transit asymmetry.
//
// It works without any external reference, purely from causality. Ask the board
// for an immediate pin read; it stamps TIM5 the moment it services the request.
// Record the host's monotonic clock at send (T0) and at reply (T1). The true
// monotonic instant of the board's stamp MUST lie in [T0, T1] -- it cannot have
// happened before we asked or after we heard back. So with t_calc the time our
// conversion produces and e = t_calc - t_true the error we are hunting:
//
//     e <= t_calc - T0   and   e >= t_calc - T1,  for EVERY sample
//  => max(t_calc - T1) <= e <= min(t_calc - T0)
//
// Both bounds tighten as samples accumulate, and they need nothing but the host
// clock. A bound that straddles zero means the conversion is consistent with the
// board's own causality; a bound that excludes zero would prove a real bias and
// say which way it points.
//
// THE --load FLAG answers "does bulk traffic pull the timeline apart". It floods
// the downlink with records addressed to an unwired UART port for the whole run,
// so the board is deserializing at its ceiling while the SOF handler is trying
// to timestamp. Two things could go wrong and they are distinguishable in the
// output: SOF interrupts arriving late shows up as a larger fit residual, SOF
// interrupts being MISSED shows up in the anomaly count.
//
// Requires firmware built with -Dlibhcs_APP_TIME_SYNC=ON.
//
// Run:
//   ./mc02_time_sync_test [seconds] [--load] [--causality] [serial ...]

#include <algorithm>
#include <array>
#include <atomic>
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
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>

#include <libhcs/board/mc02.hpp>
#include <libhcs/time/microframe_timebase.hpp>
#include <libhcs/time/timeline.hpp>

namespace {

using Clock = libhcs::host::time::Timeline::Clock;
using libhcs::host::time::timeline;

constexpr double kMicroframeUs = 125.0;

struct Report {
    libhcs::data::TimeStatusView status;
    Clock::time_point arrival;
};

// One IMU record: the board's own TIM5 stamp, plus when this process saw it.
struct Sample {
    uint32_t timestamp_quarter_us;
    Clock::time_point arrival;
};

// One causality probe: the board's stamp bracketed by host send and reply.
struct Bracket {
    uint32_t timestamp_quarter_us;
    Clock::time_point sent;
    Clock::time_point replied;
};

class Receiver final : public libhcs::board::Mc02::Callback {
public:
    std::vector<Report> take() const {
        const std::scoped_lock guard{mutex_};
        return reports_;
    }

    std::vector<Sample> take_samples() const {
        const std::scoped_lock guard{mutex_};
        return samples_;
    }

    // Called by the probe loop just before it asks for a pin read.
    void arm_bracket(Clock::time_point sent) {
        const std::scoped_lock guard{mutex_};
        pending_sent_ = sent;
        pending_armed_ = true;
    }

    std::vector<Bracket> take_brackets() const {
        const std::scoped_lock guard{mutex_};
        return brackets_;
    }

private:
    void time_status_callback(const libhcs::data::TimeStatusView& data) override {
        const std::scoped_lock guard{mutex_};
        reports_.push_back({data, Clock::now()});
    }

    // The gyroscope is the fastest thing on this board that carries a TIM5
    // timestamp, which makes it the natural probe for "can an ordinary telemetry
    // record be placed on the shared axis". Nothing about the gyro reading
    // matters here, only its stamp.
    void gyroscope_receive_callback(const libhcs::data::ImuGyroscopeDataView& data) override {
        const std::scoped_lock guard{mutex_};
        samples_.push_back({data.timestamp_quarter_us, Clock::now()});
    }

    void gpio_digital_read_result_callback(
        const libhcs::spec::mc02::GpioDescriptor& gpio,
        const libhcs::data::GpioDigitalDataView& data) override {
        (void)gpio;
        const auto now = Clock::now();
        const std::scoped_lock guard{mutex_};
        if (!pending_armed_ || !data.timestamp_quarter_us.has_value())
            return;
        pending_armed_ = false;
        brackets_.push_back({*data.timestamp_quarter_us, pending_sent_, now});
    }

    mutable std::mutex mutex_;
    std::vector<Report> reports_;
    std::vector<Sample> samples_;
    std::vector<Bracket> brackets_;
    Clock::time_point pending_sent_{};
    bool pending_armed_ = false;
};

// The bus the first mc02 sits on. Two controllers are two crystals, so the
// counter must come from the one this board is actually attached to.
int usb_bus_of_first_board() {
    DIR* dir = opendir("/sys/bus/usb/devices");
    if (dir == nullptr)
        return -1;
    int bus = -1;
    while (const dirent* entry = readdir(dir)) {
        const std::string base = std::string{"/sys/bus/usb/devices/"} + entry->d_name;
        const auto read_line = [&](const char* leaf) -> std::string {
            FILE* file = fopen((base + "/" + leaf).c_str(), "re");
            if (file == nullptr)
                return {};
            char buffer[64] = {};
            const bool ok = fgets(buffer, sizeof(buffer), file) != nullptr;
            fclose(file);
            if (!ok)
                return {};
            std::string text{buffer};
            while (!text.empty() && (text.back() == '\n' || text.back() == '\r'))
                text.pop_back();
            return text;
        };
        if (read_line("idVendor") != "a511" || read_line("idProduct") != "0723")
            continue;
        bus = std::atoi(read_line("busnum").c_str());
        break;
    }
    closedir(dir);
    return bus;
}

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
        if (read_line("idVendor") != "a511" || read_line("idProduct") != "0723")
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

struct Statistics {
    double mean;
    double sigma;
    double worst_absolute;
    std::size_t count;
};

Statistics summarize(const std::vector<double>& values) {
    Statistics result{0.0, 0.0, 0.0, values.size()};
    if (values.empty())
        return result;
    double sum = 0.0;
    for (const double value : values) {
        sum += value;
        result.worst_absolute = std::max(result.worst_absolute, std::fabs(value));
    }
    result.mean = sum / static_cast<double>(values.size());
    double variance = 0.0;
    for (const double value : values)
        variance += (value - result.mean) * (value - result.mean);
    result.sigma = std::sqrt(variance / static_cast<double>(values.size()));
    return result;
}

} // namespace

int main(int argc, char** argv) {
    std::vector<std::string> positional;
    bool with_load = false;
    bool with_causality = false;
    for (int index = 1; index < argc; index++) {
        const std::string argument{argv[index]};
        if (argument == "--load")
            with_load = true;
        else if (argument == "--causality")
            with_causality = true;
        else
            positional.push_back(argument);
    }

    const int duration_s = !positional.empty() ? std::atoi(positional[0].c_str()) : 30;
    if (duration_s <= 0) {
        fprintf(stderr, "seconds must be positive\n");
        return 1;
    }
    std::vector<std::string> serials;
    for (size_t index = 1; index < positional.size(); index++)
        serials.emplace_back(positional[index]);
    if (serials.empty())
        serials = enumerate_boards();
    if (serials.empty()) {
        fprintf(stderr, "no mc02 boards (a511:0723) found\n");
        return 1;
    }

    printf("boards: %zu\n", serials.size());

    // The host's own microframe counter, when this process may map the xHCI BAR.
    // Without it every number below is measured THROUGH the USB round trip; with
    // it, host_time_of() answers from the controller's counter (26 ns) and the
    // round trip becomes the thing being measured rather than the ruler. It
    // matters most for --causality, whose bound is otherwise dominated by the
    // conversion's own error instead of by the round trip it is supposed to
    // bracket.
    std::unique_ptr<libhcs::host::time::MicroframeTimebase> timebase;
    {
        const int bus = usb_bus_of_first_board();
        if (bus > 0)
            timebase = libhcs::host::time::MicroframeTimebase::open_for_usb_bus(bus);
        if (timebase) {
            libhcs::host::time::timeline().attach_microframe_timebase(timebase.get());
            printf("host microframe counter: open on usb%d (MFINDEX path)\n", bus);
        } else {
            printf("host microframe counter: unavailable (needs root) -- USB round trip only\n");
        }
    }

    std::vector<std::unique_ptr<Receiver>> receivers;
    std::vector<std::unique_ptr<libhcs::board::Mc02>> boards;
    // One options object per board: AdvancedOptions is deliberately non-copyable.
    std::vector<std::unique_ptr<libhcs::board::AdvancedOptions>> options;
    try {
        for (const std::string& serial : serials) {
            receivers.push_back(std::make_unique<Receiver>());
            options.push_back(std::make_unique<libhcs::board::AdvancedOptions>());
            options.back()->set_enable_time_sync(true);
            boards.push_back(
                std::make_unique<libhcs::board::Mc02>(*receivers.back(), serial, *options.back()));
        }
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    printf(
        "sampling for %d s (anchor rides the keepalive)%s...\n", duration_s,
        with_load ? ", downlink flooded" : "");

    // One flood thread per board, submitting 8-byte records to UART1 (nothing is
    // wired to it, so the firmware drops the surplus and no line rate enters the
    // measurement). Submission blocks on real USB completions, so this saturates
    // rather than spins.
    std::atomic<bool> stop_load{false};
    std::vector<std::thread> load_threads;
    std::vector<uint64_t> load_packets(boards.size(), 0);
    if (with_load) {
        for (size_t index = 0; index < boards.size(); index++) {
            load_threads.emplace_back([&, index] {
                const std::array<std::byte, 8> payload{};
                uint64_t sent = 0;
                while (!stop_load.load(std::memory_order_relaxed)) {
                    auto builder = boards[index]->start_transmit();
                    builder.uart1_transmit(
                        {.uart_data = std::span<const std::byte>{payload}, .idle_delimited = true});
                    sent++;
                }
                load_packets[index] = sent;
            });
        }
    }

    // Causality probe: one immediate pin read every 20 ms, bracketed by the host
    // clock either side. Deliberately slow -- the bound tightens with the NUMBER
    // of samples, not the rate, and a fast loop would just add bus traffic.
    std::vector<std::thread> probe_threads;
    if (with_causality) {
        for (size_t index = 0; index < boards.size(); index++) {
            probe_threads.emplace_back([&, index] {
                while (!stop_load.load(std::memory_order_relaxed)) {
                    receivers[index]->arm_bracket(Clock::now());
                    try {
                        auto builder = boards[index]->start_transmit();
                        builder.gpio_digital_read(
                            libhcs::spec::mc02::kGpioDescriptors.kPwm3,
                            {.asap = true, .capture_timestamp = true});
                    } catch (const std::exception&) {
                        return;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds{20});
                }
            });
        }
    }

    const auto load_start = Clock::now();
    std::this_thread::sleep_for(std::chrono::seconds{duration_s});
    stop_load.store(true, std::memory_order_relaxed);
    for (std::thread& thread : load_threads)
        thread.join();
    for (std::thread& thread : probe_threads)
        thread.join();
    if (with_load) {
        const double elapsed = std::chrono::duration<double>{Clock::now() - load_start}.count();
        for (size_t index = 0; index < boards.size(); index++) {
            printf(
                "load offered to board %zu: %.0f packets/s over %.1f s\n", index,
                static_cast<double>(load_packets[index]) / elapsed, elapsed);
        }
    }

    std::vector<std::vector<Report>> reports;
    std::vector<std::vector<Sample>> samples;
    std::vector<std::vector<Bracket>> brackets;
    reports.reserve(receivers.size());
    for (const auto& receiver : receivers) {
        reports.push_back(receiver->take());
        samples.push_back(receiver->take_samples());
        brackets.push_back(receiver->take_brackets());
    }

    std::vector<double> board_bias_us(serials.size(), 0.0);
    std::vector<double> board_spread_us(serials.size(), 0.0);
    std::vector<bool> board_has_residual(serials.size(), false);

    for (size_t index = 0; index < serials.size(); index++) {
        printf("\n=== board %s ===\n", serials[index].c_str());
        if (reports[index].empty()) {
            printf("  no time status -- is the firmware built with -Dlibhcs_APP_TIME_SYNC=ON?\n");
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
        const double ticks_per_microframe = last.status.ticks_per_microframe_q16 / 65536.0;
        printf(
            "  reports %zu (%zu valid)   state %s   microframe %llu\n", reports[index].size(),
            valid, state_name(last.status.state),
            static_cast<unsigned long long>(last.status.microframe));
        printf(
            "  anomalies during the run %u (since boot %u)   state transitions %zu\n",
            anomalies_during_run, last.status.anomaly_count, transitions);

        // Axis-scale check. This used to assert that consecutive reported
        // microframes differed by a multiple of 8 -- true while the board sent
        // the value LATCHED at the last SOF, since a full-speed counter only
        // moves in 8s. The board now interpolates to the instant it answers, so
        // that assertion is obsolete; what still has to hold, and is the thing
        // the old check was really protecting, is that the axis advances at
        // 8000 microframes per second. A frame/microframe scale error would show
        // up here as a factor of 8.
        {
            std::vector<double> rates;
            for (size_t report = 1; report < reports[index].size(); report++) {
                const auto& previous = reports[index][report - 1];
                const auto& current = reports[index][report];
                if (previous.status.state != libhcs::data::TimeState::kValid
                    || current.status.state != libhcs::data::TimeState::kValid)
                    continue;
                const double seconds =
                    std::chrono::duration<double>{current.arrival - previous.arrival}.count();
                if (seconds <= 0.0)
                    continue;
                const auto advance =
                    static_cast<double>(current.status.microframe - previous.status.microframe);
                rates.push_back(advance / seconds);
            }
            if (rates.size() >= 10) {
                std::sort(rates.begin(), rates.end());
                const double median = rates[rates.size() / 2];
                printf(
                    "  microframe axis rate: %.1f /s over %zu intervals (nominal 8000)  -> %s\n",
                    median, rates.size(),
                    (median > 7900.0 && median < 8100.0) ? "scale correct"
                                                         : "WRONG SCALE (frame/microframe factor)");
            }
        }

        printf(
            "  fitted local ticks per microframe %.4f (nominal 500) -> board crystal %+.1f ppm\n",
            ticks_per_microframe, (ticks_per_microframe / 500.0 - 1.0) * 1e6);

        // Prediction error of this board's own fit, in quarter-microsecond ticks
        // carried as Q16. See the header comment for how to read the two
        // statistics; the worst single sample is neither, it is the interrupt
        // jitter of one observation and a scheduled action never pays it.
        std::vector<double> report_means_us;
        double residual_abs_max_us = 0.0;
        uint64_t residual_samples = 0;
        uint64_t residual_reports = 0;
        for (const Report& report : reports[index]) {
            if (report.status.state != libhcs::data::TimeState::kValid
                || report.status.residual_count == 0)
                continue;
            report_means_us.push_back(report.status.residual_mean_q16 / 65536.0 / 4.0);
            residual_abs_max_us =
                std::max(residual_abs_max_us, report.status.residual_abs_max_q16 / 65536.0 / 4.0);
            residual_samples += report.status.residual_count;
            residual_reports++;
        }
        if (!report_means_us.empty()) {
            const Statistics summary = summarize(report_means_us);
            board_bias_us[index] = summary.mean;
            board_spread_us[index] = summary.sigma;
            board_has_residual[index] = true;
            // The spread of report means depends on how many samples each
            // report averages, which depends on the anchor period -- so it is
            // NOT comparable across configurations. Folding that back out gives
            // the per-sample sigma, which is a property of the board alone.
            // (Halving the anchor period inflates the spread by sqrt(2) while
            // this number stays put; that is the check, not a regression.)
            const double samples_per_report =
                residual_reports == 0
                    ? 0.0
                    : static_cast<double>(residual_samples) / static_cast<double>(residual_reports);
            printf(
                "  fit prediction error over %llu out-of-sample points in %zu reports:\n"
                "    bias %+.4f us   spread (1 sigma) %.4f us   worst report mean %.4f us\n"
                "    -> per-sample sigma %.4f us (spread x sqrt(%.1f per report);"
                " anchor-rate independent)\n"
                "    worst single sample %.3f us (interrupt jitter, not timeline error)\n",
                static_cast<unsigned long long>(residual_samples), summary.count, summary.mean,
                summary.sigma, summary.worst_absolute,
                summary.sigma * std::sqrt(samples_per_report), samples_per_report,
                residual_abs_max_us);
        }

        // How well THIS MACHINE can name the microframe the board reported. The
        // arrival timestamp is taken in the uplink callback, so this carries the
        // whole board-to-host path; the Timeline fit it is compared against was
        // built from round-trip midpoints, which removes the mean transit but
        // none of the jitter. Reported as a spread rather than an offset for
        // exactly that reason -- the offset is a constant this measurement
        // cannot separate from the transit it did not cancel.
        std::vector<double> placement_us;
        for (const Report& report : reports[index]) {
            if (report.status.state != libhcs::data::TimeState::kValid)
                continue;
            const auto fitted = timeline().host_time_of(report.status.microframe);
            placement_us.push_back(
                std::chrono::duration<double, std::micro>{report.arrival - fitted}.count());
        }
        if (placement_us.size() >= 2) {
            const Statistics summary = summarize(placement_us);
            // The spread is PER-SAMPLE noise, which is not what the fitted line
            // is worth: the line is built from the whole window, so its phase
            // uncertainty falls as sqrt(N). Both are printed because quoting the
            // per-sample spread as "the accuracy" overstates it by an order of
            // magnitude, and quoting only the fitted uncertainty hides that a
            // SINGLE exchange tells you very little.
            //
            // Neither number touches the OFFSET. The Timeline fits round-trip
            // midpoints, which cancels the symmetric part of the transit and
            // leaves the down/up asymmetry as a systematic this measurement
            // cannot see, let alone remove.
            const double fitted_phase_us =
                summary.sigma / std::sqrt(static_cast<double>(summary.count));
            printf(
                "  host placement of a reported microframe, %zu samples:\n"
                "    offset %+.1f us (transit asymmetry, NOT removable here)\n"
                "    per-sample spread (1 sigma) %.1f us   worst %.1f us\n"
                "    -> fitted line's phase uncertainty ~%.1f us (spread / sqrt(N))\n",
                summary.count, summary.mean, summary.sigma, summary.worst_absolute,
                fitted_phase_us);

            // READ THIS ONLY WHEN THE RUN IS NOT LONGER THAN THE TIMELINE'S OWN
            // WINDOW (kSampleCapacity * the 250 ms anchor period; 64 s at the
            // default 256). The fit compared against here is the one standing at
            // the END of the run, so on a longer run the early reports are being
            // extrapolated backwards past everything the fit ever saw, and the
            // residual measures that extrapolation rather than the fit. A 280 s
            // run at the default capacity reports 989 us instead of 288 us for
            // exactly this reason -- it is an artefact of the measurement, not a
            // property of the link.
            // Derived from the observed report rate rather than from the SDK's
            // anchor period, which is private to the handler and has changed
            // once already.
            const double reports_per_second = static_cast<double>(placement_us.size()) / duration_s;
            const double window_seconds =
                libhcs::host::time::Timeline::kSampleCapacity / reports_per_second;
            if (static_cast<double>(duration_s) > window_seconds)
                printf(
                    "    NOTE: this run (%d s) is longer than the %.0f s timeline window, so the\n"
                    "    numbers above and below overstate the error. Re-run for <= %.0f s.\n",
                    duration_s, window_seconds, window_seconds);

            // WHERE THAT SPREAD COMES FROM, tested rather than assumed. A
            // full-speed bus can only move a reply on a 1 ms frame boundary, so
            // if frame quantization is the whole story the errors are UNIFORM
            // over one frame -- and a uniform distribution of width 1 ms has
            // sigma 1000/sqrt(12) = 288.7 us and a flat histogram. Host
            // scheduling noise would instead pile up in the middle with long
            // tails. The two are easy to tell apart and they imply different
            // fixes, so the shape is printed rather than inferred from sigma.
            std::vector<double> centred;
            centred.reserve(placement_us.size());
            for (const double value : placement_us)
                centred.push_back(value - summary.mean);
            std::sort(centred.begin(), centred.end());
            // Binned over a fixed +-600 us rather than over min..max: a handful
            // of queueing outliers would otherwise stretch the axis and make a
            // genuinely flat core look peaked. Outliers are counted separately
            // instead of being allowed to distort the picture.
            constexpr int kBins = 10;
            constexpr double kHalfSpanUs = 600.0;
            std::array<int, kBins> histogram{};
            int outside = 0;
            for (const double value : centred) {
                if (std::fabs(value) > kHalfSpanUs) {
                    outside++;
                    continue;
                }
                int bin = static_cast<int>((value + kHalfSpanUs) / (2 * kHalfSpanUs) * kBins);
                bin = std::clamp(bin, 0, kBins - 1);
                histogram[static_cast<size_t>(bin)]++;
            }
            printf(
                "    residual shape, -600..+600 us in 120 us bins (uniform over one 1 ms frame"
                " would be flat):\n      ");
            for (const int count : histogram)
                printf("%5d", count);
            printf("   | %d outside\n", outside);
        }

        // The causality bound. Same TIM5 -> microframe -> host chain as the IMU
        // section, but against a reference that cannot be argued with: the
        // board's stamp happened after we asked and before we heard back.
        //
        // Taken as PERCENTILES, not min/max. The strict bound is a min and a max
        // over every probe, which makes it maximally sensitive to a single bad
        // sample -- and there are bad samples, because a delayed uplink callback
        // or a stale reference report can push one conversion outside its own
        // bracket. The first attempt used min/max and returned an IMPOSSIBLE
        // interval (lower above upper), which is the signature of exactly that.
        // The 1st/99th percentiles give the same bound while tolerating a
        // one-percent tail, and the count of true violations is printed so the
        // tail cannot hide.
        if (brackets[index].size() >= 100) {
            std::vector<double> to_t0;
            std::vector<double> to_t1;
            size_t outside = 0;
            double widest = 0.0;
            for (const Bracket& bracket : brackets[index]) {
                const Report* base = nullptr;
                for (const Report& report : reports[index]) {
                    if (report.status.state != libhcs::data::TimeState::kValid
                        || report.arrival > bracket.replied)
                        continue;
                    base = &report;
                }
                if (base == nullptr || base->status.ticks_per_microframe_q16 == 0)
                    continue;
                const double ticks_per_microframe = base->status.ticks_per_microframe_q16 / 65536.0;
                const auto delta_ticks = static_cast<int32_t>(
                    bracket.timestamp_quarter_us - base->status.timestamp_quarter_us);
                const double microframe = static_cast<double>(base->status.microframe)
                                        + (delta_ticks / ticks_per_microframe);
                const auto calc = timeline().host_time_of(microframe);
                const double a =
                    std::chrono::duration<double, std::micro>{calc - bracket.sent}.count();
                const double b =
                    std::chrono::duration<double, std::micro>{calc - bracket.replied}.count();
                to_t0.push_back(a);
                to_t1.push_back(b);
                if (a < 0.0 || b > 0.0)
                    outside++;
                widest = std::max(
                    widest,
                    std::chrono::duration<double, std::micro>{bracket.replied - bracket.sent}
                        .count());
            }
            if (to_t0.size() >= 100) {
                std::sort(to_t0.begin(), to_t0.end());
                std::sort(to_t1.begin(), to_t1.end());
                const auto pick = [](const std::vector<double>& v, double q) {
                    return v[static_cast<size_t>(q * static_cast<double>(v.size() - 1))];
                };
                const double lower = pick(to_t1, 0.99);
                const double upper = pick(to_t0, 0.01);
                // Three outcomes, and they must not be confused. A bound is only a
                // bound while lower <= upper; when the two CROSS, the
                // constant-offset model does not fit at all, and the crossing
                // width is this method's own resolution -- not a proven bias.
                const char* verdict =
                    (lower > upper)
                        ? "bounds cross -- no constant offset resolvable; the gap is this"
                          " method's resolution, NOT a bias"
                    : (lower <= 0.0 && upper >= 0.0) ? "consistent with zero"
                                                     : "EXCLUDES ZERO -- a real bias is proven";
                printf(
                    "  causality bound on the constant offset, %zu probes"
                    " (widest round trip %.0f us):\n"
                    "    %+.1f us .. %+.1f us   (1st/99th pct)\n      -> %s\n"
                    "    %zu/%zu conversions fell outside their own bracket (%.1f%%)\n",
                    to_t0.size(), widest, lower, upper, verdict, outside, to_t0.size(),
                    100.0 * static_cast<double>(outside) / static_cast<double>(to_t0.size()));
            }
        }

        // THE POINT OF ALL THIS, for a logging use case: take an ordinary
        // telemetry record -- a gyroscope sample, stamped by the board in TIM5
        // quarter-microseconds like every other record -- and name the instant it
        // was taken in this machine's clock.
        //
        // The conversion needs no new protocol field. Each status report is a
        // matched (TIM5 stamp, microframe) pair plus the fitted rate, so a
        // record's own stamp interpolates onto the microframe axis, and the host
        // Timeline carries that axis onto steady_clock.
        //
        // The check is that (arrival - converted) is CONSTANT. Its mean is the
        // board-to-host transit and says nothing; its SPREAD is what a log entry
        // would be wrong by, relative to other events on this machine. If the two
        // board clocks were not actually the same clock, this would not be
        // constant at all -- it would drift at the ratio between them.
        if (!samples[index].empty()) {
            std::vector<double> error_us;
            size_t placed = 0;
            for (const Sample& sample : samples[index]) {
                // Newest status at or before this record's arrival.
                const Report* base = nullptr;
                for (const Report& report : reports[index]) {
                    if (report.status.state != libhcs::data::TimeState::kValid
                        || report.arrival > sample.arrival)
                        continue;
                    base = &report;
                }
                if (base == nullptr || base->status.ticks_per_microframe_q16 == 0)
                    continue;
                const double ticks_per_microframe = base->status.ticks_per_microframe_q16 / 65536.0;
                // Wrap-safe: TIM5 quarter-us is a 32-bit free-running counter.
                const auto delta_ticks = static_cast<int32_t>(
                    sample.timestamp_quarter_us - base->status.timestamp_quarter_us);
                const double microframe = static_cast<double>(base->status.microframe)
                                        + (delta_ticks / ticks_per_microframe);
                const auto host_time = timeline().host_time_of(microframe);
                error_us.push_back(
                    std::chrono::duration<double, std::micro>{sample.arrival - host_time}.count());
                placed++;
            }
            if (placed >= 2) {
                // DRIFT is the statistic that discriminates, not spread. These
                // records are delivered in batches, so (arrival - converted)
                // carries tens of milliseconds of one-sided queueing noise that
                // says nothing about the clocks. What a rate error between TIM5
                // and the microframe axis WOULD do is make that quantity walk
                // steadily across the run, so the run is split in half and the
                // two medians compared. A board-crystal-sized error (35 ppm)
                // over 40 s would show as 1.4 ms of walk; agreement at the
                // microsecond level over the same span bounds it near zero.
                std::vector<double> first_half(
                    error_us.begin(), error_us.begin() + (error_us.size() / 2));
                std::vector<double> second_half(
                    error_us.begin() + (error_us.size() / 2), error_us.end());
                const auto median = [](std::vector<double>& values) {
                    std::sort(values.begin(), values.end());
                    return values[values.size() / 2];
                };
                std::vector<double> all = error_us;
                const double walk = median(second_half) - median(first_half);
                const double span_s =
                    std::chrono::duration<double>{
                        samples[index].back().arrival - samples[index].front().arrival}
                        .count();
                printf(
                    "  IMU record -> x86 time, %zu gyroscope samples placed:\n"
                    "    arrival minus converted: median %+.1f us"
                    "   (spread %.0f us is delivery batching, not clock error)\n"
                    "    half-to-half walk %+.1f us over %.0f s -> TIM5 vs microframe axis"
                    " %+.2f ppm\n",
                    placed, median(all), summarize(error_us).sigma, walk, span_s,
                    span_s > 0 ? walk / (span_s * 1e6) * 1e6 : 0.0);
            }
        }
    }

    if (serials.size() >= 2 && board_has_residual[0]) {
        printf("\n=== estimated cross-board skew ===\n");
        for (size_t board = 1; board < serials.size(); board++) {
            if (!board_has_residual[board])
                continue;
            const double bias_difference = board_bias_us[board] - board_bias_us[0];
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
            "  Derived, not observed. Both boards see the same SOF packets, so the microframe\n"
            "  COUNTER and the resolved wrap are identical by construction; the only place skew\n"
            "  can enter is each board's own microframe-to-cycle-counter fit, which is what the\n"
            "  residuals above measure. It does NOT cover the actuation path, and unlike\n"
            "  hpm_board there is no hardware capture here to check it against.\n");
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
            "  round trip microframe -> unix -> microframe: %lld microframe error (%.1f us)\n",
            static_cast<long long>(static_cast<int64_t>(back - last.status.microframe)),
            static_cast<double>(static_cast<int64_t>(back - last.status.microframe))
                * kMicroframeUs);
    }
    // The Timeline does not own the source; detach before it goes out of scope.
    libhcs::host::time::timeline().attach_microframe_timebase(nullptr);
    return 0;
}
