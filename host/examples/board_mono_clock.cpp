// Put a board timestamp on the host's CLOCK_MONOTONIC, so a board event and a
// line in journald can be compared directly.
//
// This is the smallest complete recipe. Copy board_stamp_to_monotonic() into
// whatever consumes board telemetry; the rest of this file is a demonstration
// that measures its own error.
//
// WHY MONOTONIC AND NOT UNIX. libhcs::host::time::Timeline maps the microframe
// axis onto std::chrono::steady_clock, and on Linux/libstdc++ that IS
// CLOCK_MONOTONIC -- verified, the two read within 71 ns of each other, which is
// just the gap between the two calls. journald stamps every entry with
// __MONOTONIC_TIMESTAMP on the same clock, so a board event converted here lines
// up with the system journal with nothing in between.
//
// CLOCK_REALTIME is deliberately not used. It is the only clock NTP steps, and a
// machine that is offline for a day accumulates seconds of error that arrive as
// a single jump the moment it reconnects -- at which point every timestamp
// produced before the jump disagrees with every timestamp after it. Convert to
// Unix time at DISPLAY time if a human needs it, and keep the stored axis
// monotonic.
//
// Requires firmware built with the time base enabled
// (-Dlibhcs_TIME_SYNC=ON for hpm_board, -Dlibhcs_APP_TIME_SYNC=ON for mc02)
// and the host side opened with AdvancedOptions::set_enable_time_sync(true).
//
// Run:
//   ./board_mono_clock [seconds] [serial]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <libhcs/board/hpm5321.hpp>
#include <libhcs/time/timeline.hpp>

namespace {

using Clock = libhcs::host::time::Timeline::Clock;
using libhcs::host::time::timeline;

// ---------------------------------------------------------------------------
// THE RECIPE. Everything else in this file exists to check it.
//
// A board stamps its telemetry in quarter-microseconds off its own local timer,
// which has no relation to any host clock. Each kTimeStatus is a matched pair --
// the same local timer AND the microframe the board was at -- plus the fitted
// rate between them. That pair is the bridge: interpolate the record's stamp
// onto the microframe axis, then let the Timeline carry the axis to this host.
//
// Returns nothing when the board's timeline is not valid or the host's fit has
// not converged, which are the two states where any answer would be a guess.
// ---------------------------------------------------------------------------
std::optional<Clock::time_point> board_stamp_to_monotonic(
    const libhcs::data::TimeStatusView& status, uint32_t board_quarter_us) {
    if (status.state != libhcs::data::TimeState::kValid)
        return std::nullopt;
    if (status.ticks_per_microframe_q16 == 0 || !timeline().locked())
        return std::nullopt;

    const double ticks_per_microframe = status.ticks_per_microframe_q16 / 65536.0;

    // Wrap-safe: the board's quarter-microsecond counter is a free-running
    // 32-bit value, so the distance between two readings has to be taken as a
    // signed difference rather than an unsigned one.
    const auto delta_ticks = static_cast<int32_t>(board_quarter_us - status.timestamp_quarter_us);

    const double microframe =
        static_cast<double>(status.microframe) + (delta_ticks / ticks_per_microframe);

    // The FRACTIONAL overload, deliberately: a record's stamp lands between two
    // microframes, and truncating that fraction quantises every answer to the
    // 125 us grid. See libhcs/time/timeline.hpp for why that trap earned an
    // overload of its own.
    return timeline().host_time_of(microframe);
}

int64_t to_ns(Clock::time_point when) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(when.time_since_epoch()).count();
}

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

} // namespace

int main(int argc, char** argv) {
    const int duration_s = argc > 1 ? std::atoi(argv[1]) : 30;
    const std::string serial = argc > 2 ? argv[2] : std::string{};

    Receiver receiver;
    libhcs::board::AdvancedOptions options;
    options.set_enable_time_sync(true);

    std::unique_ptr<libhcs::board::Hpm5321> board;
    try {
        board =
            std::make_unique<libhcs::board::Hpm5321>(receiver, serial, options);
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    printf("sampling %d s...\n", duration_s);
    std::this_thread::sleep_for(std::chrono::seconds{duration_s});

    const auto reports = receiver.take();
    if (reports.size() < 3) {
        fprintf(stderr, "no time status -- is the firmware built with the time base on?\n");
        return 1;
    }

    // Out-of-sample check: convert each report's OWN board stamp using the
    // PREVIOUS report as the bridge. That is exactly what a telemetry record
    // does -- it arrives between two status packets and has to be placed with
    // the older one -- so the error measured here is the error a real record
    // would carry. Converting a report with itself would be circular and always
    // read zero.
    //
    // The truth to compare against is the report's own arrival instant. That
    // carries the board-to-host transit, so the MEAN is transit and says
    // nothing; the SPREAD is what a log entry would be wrong by, relative to
    // other events on this machine.
    std::vector<double> error_us;
    for (size_t index = 1; index < reports.size(); index++) {
        const auto converted = board_stamp_to_monotonic(
            reports[index - 1].status, reports[index].status.timestamp_quarter_us);
        if (!converted)
            continue;
        error_us.push_back(
            std::chrono::duration<double, std::micro>{reports[index].arrival - *converted}.count());
    }

    if (error_us.size() < 10) {
        fprintf(stderr, "timeline never locked -- run longer\n");
        return 1;
    }

    std::vector<double> sorted = error_us;
    std::sort(sorted.begin(), sorted.end());
    const auto pick = [&sorted](double q) {
        return sorted[static_cast<size_t>(q * static_cast<double>(sorted.size() - 1))];
    };
    // Robust spread rather than a standard deviation: the host's scheduling tail
    // puts occasional single samples hundreds of microseconds out, and a few of
    // those dominate a sigma computed over hundreds of points.
    const double robust = (pick(0.84) - pick(0.16)) / 2.0;

    const auto& last = reports.back();
    const auto converted_last =
        board_stamp_to_monotonic(last.status, last.status.timestamp_quarter_us);

    printf("\n=== board stamp -> host CLOCK_MONOTONIC ===\n");
    printf(
        "  timeline locked: %s   samples %zu\n", timeline().locked() ? "yes" : "no",
        timeline().sample_count());
    if (converted_last) {
        printf(
            "  board quarter-us %u  ->  monotonic %lld ns\n", last.status.timestamp_quarter_us,
            static_cast<long long>(to_ns(*converted_last)));
    }
    printf("  host CLOCK_MONOTONIC now: %lld ns\n", static_cast<long long>(to_ns(Clock::now())));
    printf(
        "\n  conversion error over %zu out-of-sample points:\n"
        "    median %+.1f us (board-to-host transit; not an error)\n"
        "    robust spread %.1f us   <- what a log entry is worth\n",
        error_us.size(), pick(0.5), robust);
    printf(
        "\n  Compare against journald with:\n"
        "    journalctl -o json | jq -r '.__MONOTONIC_TIMESTAMP'   # microseconds, same clock\n");
    return 0;
}
