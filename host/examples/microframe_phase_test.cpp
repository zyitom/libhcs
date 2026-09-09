// Does the phase of a downlink flush inside the USB microframe change latency?
//
// THE CLAIM UNDER TEST. A bulk OUT transfer handed to libusb does not go on the
// wire when transmit() returns; it goes when the host controller next schedules
// the endpoint, and on a high-speed link that grid is 125 us wide. If that is
// the dominant quantisation, then flushing at different offsets inside one
// microframe must produce a SAWTOOTH: a flush landing just before a boundary
// leaves almost immediately, one landing just after waits nearly a full 125 us.
// A flat curve means the transfer is not gated the way the claim assumes, and
// aligning the control loop to the SOF grid would buy nothing.
//
// HOW THE CLOCK IS OBTAINED. Both boards run the shared USB-SOF time base, so
// there is one absolute microframe axis that the host and every board agree on
// (firmware/hpm_board/SOF_TIMEBASE.md). timeline().host_time_of(N) converts a
// microframe to this host's steady_clock, which is what lets the flush be aimed
// at a chosen offset inside a chosen microframe.
//
// WHAT IS ACTUALLY MEASURED. The probe frame's arrival is timestamped by the
// MCAN timestamp unit of every OTHER controller on the bus, at CAN
// start-of-frame, with no software in the path, and reported on the shared
// axis. So
//
//     latency = capture_microframe - flush_microframe
//
// covers: USB downlink transit, the board's parse, the MCAN transmit queue, and
// CAN arbitration up to SOF. Only the first term can depend on where in the
// microframe the flush happened; everything after it runs the same code on the
// same data every time. Any phase dependence in the total is therefore the USB
// scheduling term, which is the quantity in question.
//
// THE FLUSH TIME IS MEASURED, NOT ASSUMED. Spinning to a deadline overshoots by
// a variable amount, and folding that overshoot into the phase axis would smear
// exactly the edge this test exists to find. The host clock is read immediately
// after the flush returns and that reading, not the target, sets the sample's
// phase.
//
// PHASE ORDER IS INTERLEAVED, DELIBERATELY. Sweeping phase monotonically in
// time lets any slow drift -- thermal, clock, USB traffic from elsewhere --
// masquerade as a phase effect. Each round walks every phase bucket once, so a
// drift spreads across all buckets instead of aligning with one.
//
// Requires firmware built with -Dlibhcs_TIME_SYNC=ON, and root (or CAP_SYS_NICE
// plus an rtprio limit) for SCHED_FIFO -- without it scheduler noise is wider
// than the effect.
//
// Run:
//   sudo ./microframe_phase_test [rounds] [phase_steps]

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>
#include <pthread.h>
#include <sched.h>

#include <libhcs/board/hpm5321.hpp>
#include <libhcs/time/timeline.hpp>


// CAN ports are named as the silkscreen on THAT board. 5321 prints CAN1/CAN2.
// See libhcs/board/hcs_can_port.hpp.
using libhcs::board::hcs::CanPort;

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kQ16 = 65536.0;

// How far ahead of "now" a flush is aimed. Must exceed the worst wake-up plus
// spin entry cost, or the deadline is already past when the spin starts and the
// sample lands at an unintended phase. 8 microframes is 1 ms.
constexpr uint64_t kLeadMicroframes = 8;

// The spin covers only the last stretch; sleeping through the rest keeps a core
// from being pinned at 100% for the whole run.
constexpr auto kSpinWindow = std::chrono::microseconds{300};

struct Capture {
    uint64_t microframe_q16;
    uint8_t bus;
};

class Receiver final : public libhcs::board::Hpm5321::Callback {
public:
    std::map<uint32_t, std::vector<Capture>> take() const {
        const std::scoped_lock guard{mutex_};
        return captures_;
    }

    bool valid() const {
        const std::scoped_lock guard{mutex_};
        return valid_;
    }

private:
    void sync_sample_callback(const libhcs::data::SyncSampleView& data) override {
        const std::scoped_lock guard{mutex_};
        captures_[data.tag].push_back({data.microframe_q16, data.bus});
    }

    void time_status_callback(const libhcs::data::TimeStatusView& data) override {
        const std::scoped_lock guard{mutex_};
        valid_ = data.state == libhcs::data::TimeState::kValid;
    }

    mutable std::mutex mutex_;
    std::map<uint32_t, std::vector<Capture>> captures_;
    bool valid_ = false;
};

std::vector<std::string> enumerate_boards() {
    std::vector<std::string> found;
    DIR* dir = opendir("/sys/bus/usb/devices");
    if (!dir)
        return found;

    while (const dirent* entry = readdir(dir)) {
        const std::string name = entry->d_name;
        if (name.empty() || name[0] == '.')
            continue;

        const std::string base = std::string{"/sys/bus/usb/devices/"} + name + "/";
        const auto read_line = [&base](const char* file) {
            FILE* handle = fopen((base + file).c_str(), "re");
            if (!handle)
                return std::string{};
            char buffer[128]{};
            const bool ok = fgets(buffer, sizeof buffer, handle) != nullptr;
            fclose(handle);
            if (!ok)
                return std::string{};
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

bool raise_realtime() {
    sched_param param{};
    param.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0)
        return false;

    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(2, &set);
    (void)pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    return true;
}

struct Trial {
    uint32_t tag;
    double flush_microframe; // measured, fractional
    double phase_us;         // measured position inside the microframe
};

double quantile(std::vector<double>& values, double q) {
    if (values.empty())
        return std::nan("");
    std::sort(values.begin(), values.end());
    const auto index =
        static_cast<size_t>(q / 100.0 * static_cast<double>(values.size() - 1) + 0.5);
    return values[std::min(index, values.size() - 1)];
}

} // namespace

int main(int argc, char** argv) {
    const int rounds = argc > 1 ? std::atoi(argv[1]) : 200;
    const int phase_steps = argc > 2 ? std::atoi(argv[2]) : 25;
    if (rounds <= 0 || phase_steps <= 1) {
        fprintf(stderr, "rounds must be positive and phase_steps > 1\n");
        return 1;
    }

    const auto serials = enumerate_boards();
    if (serials.size() < 2) {
        fprintf(stderr, "need two hpm5321 boards, found %zu\n", serials.size());
        return 1;
    }

    if (!raise_realtime())
        fprintf(stderr, "warning: no SCHED_FIFO -- scheduler noise may hide the effect\n");

    Receiver rx_a;
    Receiver rx_b;
    libhcs::board::AdvancedOptions options_a;
    libhcs::board::AdvancedOptions options_b;
    options_a.set_enable_time_sync(true);
    options_b.set_enable_time_sync(true);

    std::vector<Trial> trials;
    trials.reserve(static_cast<size_t>(rounds) * static_cast<size_t>(phase_steps));

    try {
        libhcs::board::Hpm5321 board_a{rx_a, serials[0], options_a};
        libhcs::board::Hpm5321 board_b{rx_b, serials[1], options_b};

        printf("waiting for both timelines to become valid ...\n");
        for (int waited = 0; waited < 100 && !(rx_a.valid() && rx_b.valid()); waited++)
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        if (!rx_a.valid() || !rx_b.valid()) {
            fprintf(stderr, "timeline not valid -- firmware built with TIME_SYNC=ON?\n");
            return 2;
        }

        auto& tl = libhcs::host::time::timeline();
        for (int waited = 0; waited < 200 && !tl.locked(); waited++)
            std::this_thread::sleep_for(std::chrono::milliseconds{50});
        if (!tl.locked()) {
            fprintf(stderr, "host timeline never locked\n");
            return 2;
        }

        const double period_ns = tl.measured_period_ns();
        const double period_us = period_ns / 1000.0;
        printf("timeline locked: %.4f ns/microframe, %d phase steps x %d rounds\n", period_ns,
               phase_steps, rounds);
        printf("probing CAN id 0x%03X\n", libhcs::data::kSyncProbeCanId);

        uint32_t tag = 1;
        for (int round = 0; round < rounds; round++) {
            // Interleaved: every phase bucket is visited once per round.
            for (int step = 0; step < phase_steps; step++) {
                const double phase_target_us =
                    period_us * static_cast<double>(step) / static_cast<double>(phase_steps);

                const uint64_t target_mf = tl.anchor_now() + kLeadMicroframes;
                const auto boundary = tl.host_time_of(target_mf);
                const auto deadline =
                    boundary + std::chrono::nanoseconds{static_cast<int64_t>(phase_target_us * 1000.0)};

                const auto spin_from = deadline - kSpinWindow;
                if (Clock::now() < spin_from)
                    std::this_thread::sleep_until(spin_from);
                while (Clock::now() < deadline) { }

                std::array<std::byte, 8> payload{};
                std::memcpy(payload.data(), &tag, sizeof(tag));
                {
                    auto builder = board_a.start_transmit();
                    builder.can_transmit(CanPort::kCan1, 
                        {.can_id = libhcs::data::kSyncProbeCanId,
                         .can_data = payload,
                         .is_fdcan = true});
                }
                const auto flushed_at = Clock::now();

                // Measured phase, not the target: the spin overshoots.
                const double offset_ns =
                    static_cast<double>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(flushed_at - boundary)
                            .count());
                const double flush_mf =
                    static_cast<double>(target_mf) + offset_ns / period_ns;
                double phase_us = std::fmod(offset_ns / 1000.0, period_us);
                if (phase_us < 0.0)
                    phase_us += period_us;

                trials.push_back({tag, flush_mf, phase_us});
                tag++;
            }
        }

        // Captures ride the normal uplink; give the last ones time to land.
        std::this_thread::sleep_for(std::chrono::milliseconds{400});
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    auto captures_a = rx_a.take();
    auto captures_b = rx_b.take();

    const double period_us = libhcs::host::time::timeline().measured_period_ns() / 1000.0;

    // One bucket per phase step, filled by MEASURED phase so spin overshoot puts
    // a sample where it actually landed rather than where it was aimed.
    std::vector<std::vector<double>> buckets(static_cast<size_t>(phase_steps));
    size_t matched = 0;
    for (const auto& trial : trials) {
        const auto* list = &captures_a;
        auto iter = list->find(trial.tag);
        if (iter == list->end()) {
            list = &captures_b;
            iter = list->find(trial.tag);
        }
        if (iter == list->end() || iter->second.empty())
            continue;

        uint64_t earliest = iter->second.front().microframe_q16;
        for (const auto& capture : iter->second)
            earliest = std::min(earliest, capture.microframe_q16);

        const double latency_us =
            (static_cast<double>(earliest) / kQ16 - trial.flush_microframe) * period_us;
        if (!std::isfinite(latency_us) || latency_us < 0.0 || latency_us > 4000.0)
            continue;

        auto index = static_cast<size_t>(trial.phase_us / period_us
                                         * static_cast<double>(phase_steps));
        index = std::min(index, buckets.size() - 1);
        buckets[index].push_back(latency_us);
        matched++;
    }

    printf("\nmatched %zu of %zu trials\n\n", matched, trials.size());
    if (matched == 0) {
        fprintf(stderr, "no captures matched -- is the probe frame reaching another controller?\n");
        return 3;
    }

    printf("  phase_us      n      p50       p90       p99      min\n");
    double min_p50 = 1e9;
    double max_p50 = -1e9;
    for (size_t index = 0; index < buckets.size(); index++) {
        const double phase =
            period_us * static_cast<double>(index) / static_cast<double>(phase_steps);
        if (buckets[index].empty()) {
            printf("  %7.2f      0        -         -         -        -\n", phase);
            continue;
        }
        auto values = buckets[index];
        const double p50 = quantile(values, 50.0);
        const double p90 = quantile(values, 90.0);
        const double p99 = quantile(values, 99.0);
        const double lo = *std::min_element(values.begin(), values.end());
        printf("  %7.2f  %5zu  %7.2f   %7.2f   %7.2f  %7.2f\n", phase, values.size(), p50, p90,
               p99, lo);
        min_p50 = std::min(min_p50, p50);
        max_p50 = std::max(max_p50, p50);
    }

    const double swing = max_p50 - min_p50;
    printf("\np50 swing across phase: %.2f us (min %.2f, max %.2f)\n", swing, min_p50, max_p50);
    printf("one microframe is %.2f us\n", period_us);
    if (swing > 0.5 * period_us)
        printf("VERDICT: strong phase dependence -- aligning the flush to the SOF grid is worth"
               " up to %.0f us.\n", swing);
    else if (swing > 0.15 * period_us)
        printf("VERDICT: partial phase dependence (%.0f%% of a microframe).\n",
               swing / period_us * 100.0);
    else
        printf("VERDICT: flat -- the flush is not gated on the microframe boundary the way the"
               " claim assumes; SOF alignment buys nothing here.\n");
    return 0;
}
