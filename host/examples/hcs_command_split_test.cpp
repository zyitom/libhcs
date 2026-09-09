// Prices the gimbal/chassis flush split on HCS's REAL command layout.
//
// flush_split_test answered the question for two CAN-FD frames on one bus. That
// is not what a tick actually sends. hcs_core's OmniInfantry::command_update()
// stages five CLASSIC 8-byte frames across two buses, in this order:
//
//     CAN2   0x1FE supercap  ->  0x145 gimbal yaw   ->  0x200 chassis wheels
//     CAN3   0x142 gimbal pitch                     ->  0x200 shooter
//
// Two differences from the earlier test change the arithmetic, both in the same
// direction:
//
//   CLASSIC, NOT FD. An 8-byte classic frame at 1 Mbit occupies the bus for
//   ~110-135 us, against ~40-50 us for the same payload with BRS at 1M/5M. Every
//   frame that queues behind another pays that, so the queueing term the split
//   removes is more than twice what flush_split_test measured.
//
//   THE GIMBAL FRAME IS NOT FIRST. On CAN2 the yaw command is staged behind the
//   supercap frame, so even in the batched case it already waits one full frame
//   time before it reaches the wire. Splitting moves it to the head of an empty
//   queue, which is a second, separate saving on top of the compute gap.
//
// WHAT IS COMPARED. `gap` stands for the compute time between the gimbal PID
// finishing and the chassis power solve finishing.
//
//   BATCHED  all five frames in one packet, flushed at T+gap (today's behaviour)
//   SPLIT    the two gimbal frames flushed at T; the other three at T+gap
//
// Both modes are measured from the same reference T, so gimbal arrival times are
// directly comparable. The chassis/shooter frames are reported too: a split that
// buys the gimbal time by taking it from the chassis is a different trade than
// one that helps both, and only the numbers distinguish them.
//
// THE PROBE ID SUBSTITUTION, STATED PLAINLY. Only kSyncProbeCanId is timestamped
// by the board's capture path, so all five frames go out under that id and are
// told apart by a tag in the payload. Arbitration is unaffected: one controller
// sources every frame here, so they leave its transmit queue in FIFO order
// regardless of identifier, exactly as the real five would. What this does NOT
// model is contention against other nodes on a live robot bus.
//
// Requires firmware built with -Dlibhcs_TIME_SYNC=ON, and root for SCHED_FIFO.
//
// Run:
//   sudo ./hcs_command_split_test [rounds]

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
constexpr uint64_t kLeadMicroframes = 12;
constexpr auto kSpinWindow = std::chrono::microseconds{300};

// Compute gaps to price. The interesting region is small: if the chassis solve
// costs only tens of microseconds the split still has to earn its keep.
constexpr double kGapsUs[] = {0.0, 25.0, 50.0, 100.0, 200.0, 400.0};
constexpr size_t kGapCount = sizeof(kGapsUs) / sizeof(kGapsUs[0]);

// Slots in a trial, named after the frames they stand in for.
enum Slot : size_t {
    kSupercap = 0,   // CAN2 0x1FE, staged first today
    kGimbalYaw,      // CAN2 0x145
    kChassis,        // CAN2 0x200
    kGimbalPitch,    // CAN3 0x142
    kShooter,        // CAN3 0x200
    kSlotCount,
};

const char* slot_name(size_t slot) {
    switch (slot) {
    case kSupercap: return "supercap  CAN2 0x1FE";
    case kGimbalYaw: return "gimbal yaw CAN2 0x145";
    case kChassis: return "chassis   CAN2 0x200";
    case kGimbalPitch: return "gimbal pitch CAN3 0x142";
    case kShooter: return "shooter   CAN3 0x200";
    default: return "?";
    }
}

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
    uint32_t tags[kSlotCount];
    size_t gap_index;
    bool split;
    double reference_microframe;
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

std::array<std::byte, 8> tagged(uint32_t tag) {
    std::array<std::byte, 8> payload{};
    std::memcpy(payload.data(), &tag, sizeof(tag));
    return payload;
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

    // Classic 8-byte frames, matching what the DJI ESCs and LK motors actually
    // speak. The bus is configured for FD, which accepts classic frames; the
    // point of forcing it here is the wire time, not the framing.
    const auto probe = [](const std::array<std::byte, 8>& payload) {
        return libhcs::data::CanDataView{.can_id = libhcs::data::kSyncProbeCanId,
                                          .can_data = payload,
                                          .is_fdcan = false};
    };

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

        printf("timeline locked: %.4f ns/microframe\n", tl.measured_period_ns());
        printf("layout: CAN2 [supercap, gimbal_yaw, chassis]  CAN3 [gimbal_pitch, shooter]\n");
        printf("classic 8-byte frames, %d rounds x %zu gaps x 2 modes\n\n", rounds, kGapCount);

        uint32_t tag = 1;
        for (int round = 0; round < rounds; round++) {
            for (size_t gap_index = 0; gap_index < kGapCount; gap_index++) {
                for (int which = 0; which < 2; which++) {
                    const bool split = ((round + which) % 2) == 0;
                    const double gap_us = kGapsUs[gap_index];

                    const uint64_t reference_mf = tl.anchor_now() + kLeadMicroframes;
                    const auto reference = tl.host_time_of(reference_mf);
                    const auto late =
                        reference
                        + std::chrono::nanoseconds{static_cast<int64_t>(gap_us * 1000.0)};

                    Trial trial{};
                    trial.gap_index = gap_index;
                    trial.split = split;
                    trial.reference_microframe = static_cast<double>(reference_mf);
                    for (size_t slot = 0; slot < kSlotCount; slot++)
                        trial.tags[slot] = tag++;

                    if (split) {
                        // Gimbal alone, at the head of an empty queue on both buses.
                        spin_until(reference);
                        {
                            auto builder = board_a.start_transmit();
                            builder.can_transmit(
                                CanPort::kCan1, probe(tagged(trial.tags[kGimbalYaw])));
                            builder.can_transmit(
                                CanPort::kCan2, probe(tagged(trial.tags[kGimbalPitch])));
                        }
                        // Everything else once the slow solve finishes.
                        spin_until(late);
                        {
                            auto builder = board_a.start_transmit();
                            builder.can_transmit(
                                CanPort::kCan1, probe(tagged(trial.tags[kSupercap])));
                            builder.can_transmit(
                                CanPort::kCan1, probe(tagged(trial.tags[kChassis])));
                            builder.can_transmit(
                                CanPort::kCan2, probe(tagged(trial.tags[kShooter])));
                        }
                    } else {
                        // Today: one packet, HCS's staging order.
                        spin_until(late);
                        auto builder = board_a.start_transmit();
                        builder.can_transmit(CanPort::kCan1, probe(tagged(trial.tags[kSupercap])));
                        builder.can_transmit(CanPort::kCan1, probe(tagged(trial.tags[kGimbalYaw])));
                        builder.can_transmit(CanPort::kCan1, probe(tagged(trial.tags[kChassis])));
                        builder.can_transmit(
                            CanPort::kCan2, probe(tagged(trial.tags[kGimbalPitch])));
                        builder.can_transmit(CanPort::kCan2, probe(tagged(trial.tags[kShooter])));
                    }

                    trials.push_back(trial);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds{500});
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

    // [mode][gap][slot] -> the samples for that cell
    using SlotSamples = std::array<std::vector<double>, kSlotCount>;
    std::vector<std::vector<SlotSamples>> data(2);
    for (int mode = 0; mode < 2; mode++)
        data[mode].resize(kGapCount);

    size_t matched = 0;
    size_t dropped = 0;
    for (const auto& trial : trials) {
        double values[kSlotCount];
        bool ok = true;
        for (size_t slot = 0; slot < kSlotCount; slot++) {
            values[slot] = arrival_us(trial.tags[slot], trial.reference_microframe);
            if (!std::isfinite(values[slot]) || values[slot] < 0.0 || values[slot] > 8000.0)
                ok = false;
        }
        if (!ok) {
            dropped++;
            continue;
        }
        const int mode = trial.split ? 1 : 0;
        for (size_t slot = 0; slot < kSlotCount; slot++)
            data[mode][trial.gap_index][slot].push_back(values[slot]);
        matched++;
    }

    printf("matched %zu trials, dropped %zu\n", matched, dropped);
    if (matched == 0) {
        fprintf(stderr, "no captures matched\n");
        return 3;
    }

    printf("\np50 microseconds after T (the moment the gimbal command is ready).\n");
    printf("negative delta = SPLIT is faster.\n");

    for (size_t index = 0; index < kGapCount; index++) {
        printf("\n--- compute gap %.0f us ---\n", kGapsUs[index]);
        printf("  %-24s  batched    split    delta\n", "frame");
        for (size_t slot = 0; slot < kSlotCount; slot++) {
            auto& batched = data[0][index][slot];
            auto& split = data[1][index][slot];
            if (batched.empty() || split.empty()) {
                printf("  %-24s        -        -        -\n", slot_name(slot));
                continue;
            }
            const double b = quantile(batched, 50.0);
            const double s = quantile(split, 50.0);
            printf("  %-24s %8.1f %8.1f %8.1f\n", slot_name(slot), b, s, s - b);
        }
    }

    printf("\n=== summary: what the gimbal gains ===\n");
    printf("  gap_us     yaw_gain  pitch_gain   chassis_delta  shooter_delta\n");
    for (size_t index = 0; index < kGapCount; index++) {
        auto& yb = data[0][index][kGimbalYaw];
        auto& ys = data[1][index][kGimbalYaw];
        auto& pb = data[0][index][kGimbalPitch];
        auto& ps = data[1][index][kGimbalPitch];
        auto& cb = data[0][index][kChassis];
        auto& cs = data[1][index][kChassis];
        auto& hb = data[0][index][kShooter];
        auto& hs = data[1][index][kShooter];
        if (yb.empty() || ys.empty())
            continue;
        printf("  %6.0f   %9.1f  %10.1f   %13.1f  %13.1f\n", kGapsUs[index],
               quantile(yb, 50.0) - quantile(ys, 50.0), quantile(pb, 50.0) - quantile(ps, 50.0),
               quantile(cs, 50.0) - quantile(cb, 50.0), quantile(hs, 50.0) - quantile(hb, 50.0));
    }
    printf("\n(gain columns positive = gimbal arrives earlier when split;\n");
    printf(" delta columns positive = that frame arrives LATER when split.)\n");
    return 0;
}
