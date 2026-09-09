// What does splitting one downlink batch into two packets actually buy?
//
// THE SITUATION THIS MODELS. A control tick computes some commands early and
// others late: in HCS the gimbal PID finishes mid-chain while the chassis power
// solve finishes at the end, and today BOTH ride one packet flushed after the
// slow one. The question is whether flushing the early command as soon as it is
// ready gets it onto the CAN bus sooner, and what that costs the late one.
//
//   BATCHED  wait until T+gap, then flush ONE packet holding both frames
//   SPLIT    flush frame 1 at T, flush frame 2 at T+gap
//
// Frame 1 is the early command, frame 2 the late one, and `gap` stands in for
// the compute time between them. Both modes are measured against the SAME
// reference microframe T, so the two arrival times are directly comparable --
// measuring each frame against its own flush would hide exactly the head start
// this test exists to price.
//
//   gain = arrival(frame1, BATCHED) - arrival(frame1, SPLIT)     want > 0
//   cost = arrival(frame2, SPLIT)   - arrival(frame2, BATCHED)   want ~ 0
//
// WHY BOTH NUMBERS. A split that pulls frame 1 forward by the full gap but
// pushes frame 2 back by as much has moved latency around rather than removed
// it, and on this hardware the two frames go to different actuators whose
// commands are not interchangeable. The cost column is what says whether the
// trade is real.
//
// WHAT THE EARLIER PHASE TEST SETTLED, AND WHY IT MATTERS HERE. Bulk transfers
// turned out NOT to be gated on the 125 us microframe boundary
// (microframe_phase_test: p50 swing 16 us against a 125 us microframe, no
// sawtooth). So the gain is expected to be continuous in `gap` rather than
// quantised to whole microframes -- which is precisely why a gap far smaller
// than a microframe is worth measuring at all.
//
// Requires firmware built with -Dlibhcs_TIME_SYNC=ON, and root for SCHED_FIFO.
//
// Run:
//   sudo ./flush_split_test [rounds]

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
constexpr uint64_t kLeadMicroframes = 8;
constexpr auto kSpinWindow = std::chrono::microseconds{300};

// The compute gaps to price, in microseconds. Spans from "the two commands are
// ready together" up past one microframe, so the shape either continues through
// the 125 us mark or breaks at it.
constexpr double kGapsUs[] = {0.0, 10.0, 25.0, 50.0, 75.0, 100.0, 150.0, 250.0};
constexpr size_t kGapCount = sizeof(kGapsUs) / sizeof(kGapsUs[0]);

struct Capture {
    uint64_t microframe_q16;
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
        captures_[data.tag].push_back({data.microframe_q16});
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
    uint32_t tag1;
    uint32_t tag2;
    size_t gap_index;
    bool split;
    double reference_microframe; // T, the moment frame 1 became ready
};

double quantile(std::vector<double>& values, double q) {
    if (values.empty())
        return std::nan("");
    std::sort(values.begin(), values.end());
    const auto index =
        static_cast<size_t>(q / 100.0 * static_cast<double>(values.size() - 1) + 0.5);
    return values[std::min(index, values.size() - 1)];
}

void spin_until(Clock::time_point deadline) {
    const auto spin_from = deadline - kSpinWindow;
    if (Clock::now() < spin_from)
        std::this_thread::sleep_until(spin_from);
    while (Clock::now() < deadline) { }
}

} // namespace

int main(int argc, char** argv) {
    const int rounds = argc > 1 ? std::atoi(argv[1]) : 300;
    if (rounds <= 0) {
        fprintf(stderr, "rounds must be positive\n");
        return 1;
    }

    const auto serials = enumerate_boards();
    if (serials.size() < 2) {
        fprintf(stderr, "need two hpm5321 boards, found %zu\n", serials.size());
        return 1;
    }

    if (!raise_realtime())
        fprintf(stderr, "warning: no SCHED_FIFO -- scheduler noise may swamp small gaps\n");

    Receiver rx_a;
    Receiver rx_b;
    libhcs::board::AdvancedOptions options_a;
    libhcs::board::AdvancedOptions options_b;
    options_a.set_enable_time_sync(true);
    options_b.set_enable_time_sync(true);

    std::vector<Trial> trials;
    trials.reserve(static_cast<size_t>(rounds) * kGapCount * 2);

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
        printf("timeline locked: %.4f ns/microframe, %d rounds x %zu gaps x 2 modes\n", period_ns,
               rounds, kGapCount);

        uint32_t tag = 1;
        for (int round = 0; round < rounds; round++) {
            for (size_t gap_index = 0; gap_index < kGapCount; gap_index++) {
                // Both modes for the same gap, back to back and in alternating
                // order, so a drift cannot settle on one mode.
                for (int which = 0; which < 2; which++) {
                    const bool split = ((round + which) % 2) == 0;
                    const double gap_us = kGapsUs[gap_index];

                    const uint64_t reference_mf = tl.anchor_now() + kLeadMicroframes;
                    const auto reference = tl.host_time_of(reference_mf);
                    const auto late = reference
                                    + std::chrono::nanoseconds{
                                          static_cast<int64_t>(gap_us * 1000.0)};

                    const uint32_t tag1 = tag++;
                    const uint32_t tag2 = tag++;
                    std::array<std::byte, 8> payload1{};
                    std::array<std::byte, 8> payload2{};
                    std::memcpy(payload1.data(), &tag1, sizeof(tag1));
                    std::memcpy(payload2.data(), &tag2, sizeof(tag2));

                    if (split) {
                        spin_until(reference);
                        {
                            auto builder = board_a.start_transmit();
                            builder.can_transmit(
                                CanPort::kCan1,
                                {.can_id = libhcs::data::kSyncProbeCanId,
                                 .can_data = payload1,
                                 .is_fdcan = true});
                        }
                        spin_until(late);
                        {
                            auto builder = board_a.start_transmit();
                            builder.can_transmit(
                                CanPort::kCan1,
                                {.can_id = libhcs::data::kSyncProbeCanId,
                                 .can_data = payload2,
                                 .is_fdcan = true});
                        }
                    } else {
                        spin_until(late);
                        auto builder = board_a.start_transmit();
                        builder.can_transmit(
                            CanPort::kCan1,
                            {.can_id = libhcs::data::kSyncProbeCanId,
                             .can_data = payload1,
                             .is_fdcan = true});
                        builder.can_transmit(
                            CanPort::kCan1,
                            {.can_id = libhcs::data::kSyncProbeCanId,
                             .can_data = payload2,
                             .is_fdcan = true});
                    }

                    trials.push_back({tag1, tag2, gap_index, split,
                                      static_cast<double>(reference_mf)});
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds{400});
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    const auto captures_a = rx_a.take();
    const auto captures_b = rx_b.take();
    const double period_us = libhcs::host::time::timeline().measured_period_ns() / 1000.0;

    const auto arrival_us = [&](uint32_t needle, double reference_mf) -> double {
        const auto* list = &captures_a;
        auto iter = list->find(needle);
        if (iter == list->end()) {
            list = &captures_b;
            iter = list->find(needle);
        }
        if (iter == list->end() || iter->second.empty())
            return std::nan("");
        uint64_t earliest = iter->second.front().microframe_q16;
        for (const auto& capture : iter->second)
            earliest = std::min(earliest, capture.microframe_q16);
        return (static_cast<double>(earliest) / kQ16 - reference_mf) * period_us;
    };

    // [gap][split] -> arrival of each frame, measured from the same reference T.
    std::vector<std::vector<double>> first[2];
    std::vector<std::vector<double>> second[2];
    for (int mode = 0; mode < 2; mode++) {
        first[mode].resize(kGapCount);
        second[mode].resize(kGapCount);
    }

    size_t matched = 0;
    for (const auto& trial : trials) {
        const double a1 = arrival_us(trial.tag1, trial.reference_microframe);
        const double a2 = arrival_us(trial.tag2, trial.reference_microframe);
        if (!std::isfinite(a1) || !std::isfinite(a2))
            continue;
        if (a1 < 0.0 || a1 > 4000.0 || a2 < 0.0 || a2 > 4000.0)
            continue;
        const int mode = trial.split ? 1 : 0;
        first[mode][trial.gap_index].push_back(a1);
        second[mode][trial.gap_index].push_back(a2);
        matched++;
    }

    printf("\nmatched %zu of %zu trials\n", matched, trials.size());
    if (matched == 0) {
        fprintf(stderr, "no captures matched\n");
        return 3;
    }

    printf("\nAll times are p50 microseconds after the reference T (frame 1 ready).\n");
    printf("gain = frame1 arrives this much EARLIER when split (positive is good)\n");
    printf("cost = frame2 arrives this much LATER when split (positive is bad)\n\n");
    printf("   gap_us       n   f1_batched  f1_split |    gain  |  f2_batched  f2_split |   cost\n");

    for (size_t index = 0; index < kGapCount; index++) {
        auto& fb = first[0][index];
        auto& fs = first[1][index];
        auto& sb = second[0][index];
        auto& ss = second[1][index];
        if (fb.empty() || fs.empty() || sb.empty() || ss.empty()) {
            printf("   %6.1f       0            -         -  |       -  |           -         - |      -\n",
                   kGapsUs[index]);
            continue;
        }
        const double f1b = quantile(fb, 50.0);
        const double f1s = quantile(fs, 50.0);
        const double f2b = quantile(sb, 50.0);
        const double f2s = quantile(ss, 50.0);
        printf("   %6.1f   %5zu     %8.2f  %8.2f  | %7.2f  |    %8.2f  %8.2f | %6.2f\n",
               kGapsUs[index], std::min(fb.size(), fs.size()), f1b, f1s, f1b - f1s, f2b, f2s,
               f2s - f2b);
    }

    printf("\nRead the gain column against the gap column: a gain that tracks the gap means the\n");
    printf("early command really does leave as soon as it is ready. A gain stuck near zero means\n");
    printf("something downstream re-serialises the two packets and splitting is pointless.\n");
    return 0;
}
