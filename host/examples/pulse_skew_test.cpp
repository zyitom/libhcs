// Cross-board skew measured by a HARDWARE pulse exchange over the UART0 wires.
//
// The two boards are already cross-wired A.TXD<->B.RXD both ways, and those two
// pins are GPTMR0 channel 1's compare output and capture input. So the existing
// cable is a bidirectional timing link with no CPU at either end: the pulse
// leaves on a comparator match and arrives on a hardware capture.
//
// WHAT THIS MEASURES, precisely. Both boards are told to fire at the SAME
// absolute microframe of the shared USB-SOF timeline. Each board reports where
// it captured the OTHER board's pulse, on its own axis. With d the one-way path
// delay and s the difference between the two boards' ideas of when microframe k
// happens,
//
//     offset_A (A's report) = d - s        offset_B (B's report) = d + s
//     s = (offset_B - offset_A) / 2        d = (offset_B + offset_A) / 2
//
// so the path delay -- cable, pad, synchroniser, Schmitt trigger -- cancels in
// the difference and appears alone in the sum. Note what s is: not the fit
// residual of section 5.2 of SOF_TIMEBASE.md, but the ACTUATION skew, the thing
// two boards would really differ by when both act "at microframe k".
//
// STAGED, AND THE ORDER MATTERS (SOF_TIMEBASE.md 7.1). Five earlier attempts at
// a direct measurement failed by reading a cross-board number before the
// instrument was known good:
//
//   stage 1  the fitted ticks per microframe, from both boards. It must land
//            near 3000.25 (the board crystal is ~+80 ppm fast against the host's
//            USB clock), and the two boards must agree to a few ppm. A round
//            3000.000 would mean the fit is not running at all.
//   stage 2  one-way offsets. Their SPREAD is this instrument's noise floor;
//            nothing below it can be believed. Their MEAN is the path delay.
//   stage 3  the two-way skew, reported only if stage 2 is clean.
//
// Requires firmware built with -Dlibhcs_PULSE_TEST=ON (which takes UART0 over)
// and -Dlibhcs_TIME_SYNC=ON (which supplies the microframe axis).
//
// Run:
//   ./pulse_skew_test [rounds] [lead_microframes]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>

#include <libhcs/board/hpm5321.hpp>
#include <libhcs/time/timeline.hpp>

namespace {

// One microframe is 125 us, and reports carry Q16 microframes.
constexpr double kNsPerQ16 = 125000.0 / 65536.0;
// One GPTMR tick at 24 MHz, in nanoseconds. The quantisation of every capture.
constexpr double kTickNs = 1000.0 / 24.0;

struct Report {
    uint64_t scheduled;
    uint64_t captured_q16;
    uint32_t ticks_per_microframe_q16;
    uint8_t flags;
};

class Receiver final : public libhcs::board::Hpm5321::Callback {
public:
    // Reports are keyed by the microframe they were scheduled for, which is what
    // makes a round on one board comparable with the same round on the other.
    std::map<uint64_t, std::vector<Report>> take() const {
        const std::scoped_lock guard{mutex_};
        return reports_;
    }

    bool valid() const {
        const std::scoped_lock guard{mutex_};
        return valid_;
    }

    // Drops everything collected so far, so the warm-up rounds cannot be
    // confused with the measurement.
    void forget() {
        const std::scoped_lock guard{mutex_};
        reports_.clear();
    }

private:
    void pulse_report_callback(const libhcs::data::PulseReportView& data) override {
        const std::scoped_lock guard{mutex_};
        reports_[data.scheduled_microframe].push_back(
            {data.scheduled_microframe, data.captured_microframe_q16,
             data.ticks_per_microframe_q16, data.flags});
    }

    void time_status_callback(const libhcs::data::TimeStatusView& data) override {
        const std::scoped_lock guard{mutex_};
        valid_ = data.state == libhcs::data::TimeState::kValid;
    }

    mutable std::mutex mutex_;
    std::map<uint64_t, std::vector<Report>> reports_;
    bool valid_ = false;
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

struct Stats {
    size_t count = 0;
    double mean = 0.0;
    double sigma = 0.0;
    double min = 0.0;
    double max = 0.0;
};

Stats summarise(std::vector<double> values) {
    Stats out;
    out.count = values.size();
    if (values.empty())
        return out;
    std::sort(values.begin(), values.end());
    double sum = 0.0;
    for (const double value : values)
        sum += value;
    out.mean = sum / static_cast<double>(values.size());
    double variance = 0.0;
    for (const double value : values)
        variance += (value - out.mean) * (value - out.mean);
    out.sigma = std::sqrt(variance / static_cast<double>(values.size()));
    out.min = values.front();
    out.max = values.back();
    return out;
}

void print_stats(const char* label, const Stats& stats) {
    if (stats.count == 0) {
        printf("  %-26s no samples\n", label);
        return;
    }
    printf(
        "  %-26s n=%-5zu mean %+9.1f ns   sigma %8.1f ns   min %+9.1f   max %+9.1f\n", label,
        stats.count, stats.mean, stats.sigma, stats.min, stats.max);
}

// The capture of one round on one board, or nothing if the board did not arm or
// did not hear the pulse.
const Report* capture_of(
    const std::map<uint64_t, std::vector<Report>>& reports, uint64_t scheduled, bool& armed) {
    const auto found = reports.find(scheduled);
    armed = false;
    if (found == reports.end())
        return nullptr;
    const Report* capture = nullptr;
    for (const Report& report : found->second) {
        if ((report.flags & libhcs::data::kPulseArmed) != 0)
            armed = true;
        if ((report.flags & libhcs::data::kPulseCaptured) != 0)
            capture = &report;
    }
    return capture;
}

double ticks_per_microframe(const std::map<uint64_t, std::vector<Report>>& reports) {
    // The last non-zero value seen. The fit moves only by crystal drift, so any
    // recent report carries the same number.
    double latest = 0.0;
    for (const auto& [scheduled, list] : reports) {
        for (const Report& report : list) {
            if (report.ticks_per_microframe_q16 != 0)
                latest = static_cast<double>(report.ticks_per_microframe_q16) / 65536.0;
        }
    }
    return latest;
}

} // namespace

int main(int argc, char** argv) {
    const int rounds = argc > 1 ? std::atoi(argv[1]) : 400;
    const int lead = argc > 2 ? std::atoi(argv[2]) : 400;
    if (rounds <= 0 || lead < 32 || lead > 4000) {
        fprintf(stderr, "usage: pulse_skew_test [rounds>0] [lead_microframes 32..4000]\n");
        return 1;
    }

    const auto serials = enumerate_boards();
    if (serials.size() < 2) {
        fprintf(stderr, "need two hpm5321 dual-CAN boards, found %zu\n", serials.size());
        return 1;
    }

    Receiver rx_a;
    Receiver rx_b;
    libhcs::board::AdvancedOptions options_a;
    libhcs::board::AdvancedOptions options_b;
    options_a.set_enable_time_sync(true);
    options_b.set_enable_time_sync(true);

    std::vector<uint64_t> targets;
    try {
        libhcs::board::Hpm5321 board_a{rx_a, serials[0], options_a};
        libhcs::board::Hpm5321 board_b{rx_b, serials[1], options_b};
        printf("board A = %s\nboard B = %s\n", serials[0].c_str(), serials[1].c_str());

        printf("waiting for both timelines to become valid ...\n");
        for (int waited = 0; waited < 100 && !(rx_a.valid() && rx_b.valid()); waited++)
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        if (!rx_a.valid() || !rx_b.valid()) {
            fprintf(
                stderr, "timeline not valid (A %d, B %d) -- firmware built with TIME_SYNC=ON?\n",
                static_cast<int>(rx_a.valid()), static_cast<int>(rx_b.valid()));
            return 2;
        }

        // AND wait for the HOST's own axis to lock, which is a separate thing
        // and the reason an early revision of this tool worked exactly once per
        // power cycle. Before it has 16 samples, Timeline::anchor_for() is open
        // loop from THIS PROCESS's start, i.e. microframe ~0. A freshly booted
        // board has no anchor yet, adopts that numbering, and everything agrees;
        // a board that is already anchored (any later run) keeps its own epoch,
        // so every target lands absurdly in the past and the board refuses to
        // arm all 400 rounds. Waiting for the fit makes the host adopt the
        // BOARD's numbering, which is what the anchor is supposed to do.
        printf("waiting for the host microframe axis to lock onto the boards ...\n");
        for (int waited = 0; waited < 200 && !libhcs::host::time::timeline().locked(); waited++)
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        if (!libhcs::host::time::timeline().locked()) {
            fprintf(stderr, "host timeline never locked (needs 16 status samples)\n");
            return 2;
        }

        // One round every lead+64 microframes, so a round is always finished --
        // pulse fired, captured, reported -- before the next one is scheduled.
        // Overlapping rounds would make the board's "scheduled" label ambiguous.
        const auto period =
            std::chrono::microseconds{static_cast<int64_t>(lead + 64) * 125};
        printf(
            "%d rounds, %d microframes (%.1f ms) of lead, one round every %.1f ms\n", rounds, lead,
            lead * 0.125, static_cast<double>(period.count()) / 1000.0);

        auto next = std::chrono::steady_clock::now();

        // Warm-up, discarded. The board brings the GPTMR up on the FIRST
        // schedule it receives (pulse.cpp init() explains why it is not done at
        // boot), and its microframe-to-tick fit then needs a second of SOF
        // samples before it will arm anything. Counting these rounds would just
        // report "not armed" for the start of every run.
        const int warmup = 24;
        printf("warming up (%d discarded rounds while the board's fit converges)\n", warmup);
        for (int round = 0; round < warmup; round++) {
            board_a.send_pulse_schedule(libhcs::host::time::timeline().anchor_now()
                                        + static_cast<uint64_t>(lead));
            board_b.send_pulse_schedule(libhcs::host::time::timeline().anchor_now()
                                        + static_cast<uint64_t>(lead));
            next += period;
            const auto now = std::chrono::steady_clock::now();
            if (next < now)
                next = now;
            std::this_thread::sleep_until(next);
        }
        rx_a.forget();
        rx_b.forget();

        for (int round = 0; round < rounds; round++) {
            const uint64_t target = libhcs::host::time::timeline().anchor_now()
                                  + static_cast<uint64_t>(lead);
            // The IDENTICAL value to both boards: that is what makes the path
            // delay cancel in the difference of the two reports.
            board_a.send_pulse_schedule(target);
            board_b.send_pulse_schedule(target);
            targets.push_back(target);

            next += period;
            const auto now = std::chrono::steady_clock::now();
            if (next < now)
                next = now;
            std::this_thread::sleep_until(next);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{300});
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    const auto reports_a = rx_a.take();
    const auto reports_b = rx_b.take();

    printf("\n=== stage 1: the clock relationship each board measured ===\n");
    const double slope_a = ticks_per_microframe(reports_a);
    const double slope_b = ticks_per_microframe(reports_b);
    if (slope_a == 0.0 || slope_b == 0.0) {
        printf(
            "  board A %.4f  board B %.4f ticks/microframe\n"
            "  FAIL: a board never published a fit. Either the pulse module is not compiled in\n"
            "  (-Dlibhcs_PULSE_TEST=ON) or the SOF sample ring keeps resetting.\n",
            slope_a, slope_b);
        return 3;
    }
    const double ppm_a = (slope_a / 3000.0 - 1.0) * 1e6;
    const double ppm_b = (slope_b / 3000.0 - 1.0) * 1e6;
    printf(
        "  board A %.4f ticks/microframe  (%+.1f ppm vs the host's USB clock)\n"
        "  board B %.4f ticks/microframe  (%+.1f ppm)\n"
        "  boards differ by %+.2f ppm\n",
        slope_a, ppm_a, slope_b, ppm_b, ppm_a - ppm_b);
    if (std::abs(ppm_a) > 500.0 || std::abs(ppm_b) > 500.0) {
        printf("  FAIL: that is not a crystal offset. Stop here; nothing below is meaningful.\n");
        return 3;
    }
    printf(
        "  Expected around +80 ppm, matching the machine-timer fit in SOF_TIMEBASE.md 2.2.\n"
        "  Exactly 3000.0000 would mean the fit never ran.\n");

    size_t armed_rounds = 0;
    size_t both_captured = 0;
    std::vector<double> offset_a;
    std::vector<double> offset_b;
    std::vector<double> skew;
    std::vector<double> delay;
    for (const uint64_t target : targets) {
        bool armed_a = false;
        bool armed_b = false;
        const Report* capture_a = capture_of(reports_a, target, armed_a);
        const Report* capture_b = capture_of(reports_b, target, armed_b);
        if (armed_a && armed_b)
            armed_rounds++;
        if (capture_a == nullptr || capture_b == nullptr)
            continue;
        both_captured++;
        const double a = (static_cast<double>(capture_a->captured_q16)
                          - static_cast<double>(target << 16))
                       * kNsPerQ16;
        const double b = (static_cast<double>(capture_b->captured_q16)
                          - static_cast<double>(target << 16))
                       * kNsPerQ16;
        offset_a.push_back(a);
        offset_b.push_back(b);
        skew.push_back((b - a) / 2.0);
        delay.push_back((b + a) / 2.0);
    }

    printf("\n=== stage 2: did the exchange happen, and how noisy is it ===\n");
    printf(
        "  rounds requested %zu   both boards armed %zu   both captured %zu\n", targets.size(),
        armed_rounds, both_captured);
    if (both_captured == 0) {
        printf(
            "  FAIL: no round produced a capture on both boards.\n"
            "  armed but silent  -> the comparator fired and the far pin heard nothing:\n"
            "                       check the UART0 cross wiring, both directions.\n"
            "  not armed         -> the board refused the target: the timeline or the fit\n"
            "                       was not ready, or the lead fell outside 16..8000.\n");
        return 4;
    }
    print_stats("A hears B's pulse", summarise(offset_a));
    print_stats("B hears A's pulse", summarise(offset_b));
    printf(
        "  One GPTMR tick is %.1f ns, so a sigma near %.0f ns is the quantisation floor and\n"
        "  anything much larger is real jitter in the firing or capture path.\n",
        kTickNs, kTickNs / std::sqrt(12.0));

    printf("\n=== stage 3: two-way result ===\n");
    const Stats skew_stats = summarise(skew);
    const Stats delay_stats = summarise(delay);
    print_stats("path delay  (sum/2)", delay_stats);
    print_stats("cross-board skew (diff/2)", skew_stats);
    printf(
        "\n  The path delay is cable plus pad plus Schmitt trigger, one way. It is a constant\n"
        "  and is NOT part of the skew: it enters both reports with the same sign.\n"
        "  The skew is what two boards would differ by when both act at the same microframe.\n"
        "  Its own quantisation floor is %.0f ns (one tick, halved by the two-way average),\n"
        "  so treat a sigma at or below that as 'not resolvable by this instrument'.\n",
        kTickNs / 2.0);
    return 0;
}
