// Splits the board-level one-way latency into "fixed floor" and "per-frame
// pipeline step", to find out whether the firmware is on the critical path or
// hidden behind the CAN wire.
//
// WHY: dual_board_test latency gives one number (p50 ~99 us at 1 kHz duty).
// HOST_TUNING.md 9.3 accounts for 50.3 us of it as CAN-FD wire time for an
// 8-byte frame, leaving ~27 us of "everything else" -- USB down, firmware
// turnaround on both boards, USB up, host wakeup. That residue is where any
// remaining host/firmware optimisation would have to come from, but a single
// end-to-end number cannot say which part of it is what.
//
// HOW: submit ONE USB packet carrying N CAN frames with consecutive sequence
// numbers, then timestamp each frame as it comes back from the far board.
//
//   pos0 latency          = USB down + fw A + wire + fw B + USB up + wakeup
//   pos[i] - pos[i-1]     = the steady-state pipeline step
//
// The step is the discriminator:
//   step ~= CAN wire time  -> the wire is the bottleneck and both firmwares are
//                             fully pipelined behind it. Nothing to win there.
//   step >  CAN wire time  -> something in firmware or USB uplink serialises
//                             per frame, and that difference is real headroom.
//
// All N frames ride one USB packet, so USB downlink cost is paid once and shows
// up only in pos0 -- that is what separates the fixed floor from the step.
//
// WIRING: two boards, A.CAN1 <-> B.CAN1, 120 ohm at each end.
// Needs root.  sudo ./can_path_split [burst] [trials]

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <dirent.h>
#include <pthread.h>
#include <sched.h>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hpm5321.hpp>


// CAN ports are named as the silkscreen on THAT board. 5321 prints CAN1/CAN2.
// See libhcs/board/hcs_can_port.hpp.
using libhcs::board::hcs::CanPort;

namespace {

using Board = libhcs::board::Hpm5321;
using Clock = std::chrono::steady_clock;

constexpr uint32_t kCanIdBase = 0x560;
constexpr size_t kPayloadSize = 8;
constexpr int kMaxBurst = 32;
constexpr auto kBurstTimeout = std::chrono::milliseconds{50};

std::string g_serial_a;
std::string g_serial_b;

void configure_thread(int core, int priority, const char* name) noexcept {
    if (name)
        (void)pthread_setname_np(pthread_self(), name);
    if (core >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(core, &set);
        if (sched_setaffinity(0, sizeof(set), &set) != 0)
            perror("sched_setaffinity");
    }
    sched_param parameter{};
    parameter.sched_priority = priority;
    if (sched_setscheduler(0, SCHED_FIFO, &parameter) != 0)
        perror("sched_setscheduler");
}

struct NodeOptions final : libhcs::board::AdvancedOptions {
    int io_core = -1;
    const char* thread_name = nullptr;
};

uint32_t get_u32_le(const std::byte* src) {
    return static_cast<uint32_t>(std::to_integer<uint8_t>(src[0]))
         | static_cast<uint32_t>(std::to_integer<uint8_t>(src[1])) << 8
         | static_cast<uint32_t>(std::to_integer<uint8_t>(src[2])) << 16
         | static_cast<uint32_t>(std::to_integer<uint8_t>(src[3])) << 24;
}

void put_u32_le(std::byte* dst, uint32_t value) {
    dst[0] = static_cast<std::byte>(value & 0xFF);
    dst[1] = static_cast<std::byte>((value >> 8) & 0xFF);
    dst[2] = static_cast<std::byte>((value >> 16) & 0xFF);
    dst[3] = static_cast<std::byte>((value >> 24) & 0xFF);
}

// One burst in flight at a time, so a slot index is just the position in the
// burst and no per-frame bookkeeping is needed.
struct BurstCatcher {
    std::atomic<uint64_t> arrival_ns[kMaxBurst] = {};
    // B's hardware TSU stamp of the CAN RX, carried in the uplink frame. Its
    // epoch is unrelated to the host clock, so the absolute difference is
    // meaningless -- but the offset is CONSTANT, so the SHAPE of
    // (host arrival - board stamp) is not. That is what splits the path:
    // if the bimodality survives here it happened after B latched the frame
    // (device uplink queueing or host wakeup); if it flattens, it happened
    // before (downlink, board A, or the CAN wire).
    std::atomic<uint32_t> board_us[kMaxBurst] = {};
    std::atomic<bool> board_us_valid[kMaxBurst] = {};
    std::atomic<int> received = 0;
    Clock::time_point start;
    int burst = 0;
};

void catch_frame(void* context, const libhcs::data::CanDataView& data) {
    auto* catcher = static_cast<BurstCatcher*>(context);
    if (data.can_data.size() != kPayloadSize)
        return;
    const uint32_t position = get_u32_le(data.can_data.data());
    if (position >= static_cast<uint32_t>(catcher->burst))
        return;
    const auto now = Clock::now();
    // Only the first arrival for a position counts; a duplicate would otherwise
    // overwrite the timestamp with a later one and flatten the step.
    uint64_t expected = 0;
    const auto ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - catcher->start).count());
    if (catcher->arrival_ns[position].compare_exchange_strong(
            expected, ns, std::memory_order_relaxed)) {
        if (data.timestamp_us.has_value()) {
            catcher->board_us[position].store(*data.timestamp_us, std::memory_order_relaxed);
            catcher->board_us_valid[position].store(true, std::memory_order_relaxed);
        }
        catcher->received.fetch_add(1, std::memory_order_release);
    }
}

class Node final : public Board::Callback {
public:
    Node(std::string_view serial, int io_core, const char* thread_name) {
        options_.io_core = io_core;
        options_.thread_name = thread_name;
        options_.dangerously_skip_version_checks = true;
        options_.thread_setup = [](const libhcs::board::AdvancedOptions& self) noexcept {
            const auto& options = static_cast<const NodeOptions&>(self);
            configure_thread(options.io_core, 90, options.thread_name);
        };
        board_ = std::make_unique<Board>(*this, serial, options_);
    }

    Board& board() { return *board_; }
    void watch(BurstCatcher* catcher) { catcher_.store(catcher, std::memory_order_release); }

private:
    void can_receive(
        CanPort port, const libhcs::data::CanDataView& data) override {
        switch (port) {
        case CanPort::kCan1: {
            if (BurstCatcher* catcher = catcher_.load(std::memory_order_acquire))
                catch_frame(catcher, data);
            break;
        }
        default: break;
        }
    }

    NodeOptions options_;
    std::unique_ptr<Board> board_;
    std::atomic<BurstCatcher*> catcher_ = nullptr;
};

std::vector<std::string> enumerate_serials() {
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
        std::string serial = read_line("serial");
        if (!serial.empty())
            found.push_back(std::move(serial));
    }
    closedir(dir);
    std::sort(found.begin(), found.end());
    return found;
}

double percentile(std::vector<double>& values, double fraction) {
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    const auto index = static_cast<size_t>(fraction * static_cast<double>(values.size() - 1));
    return values[index];
}

int run(int burst, int trials) {
    Node board_a{g_serial_a, 7, "split-a"};
    Node board_b{g_serial_b, 6, "split-b"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    BurstCatcher catcher;
    catcher.burst = burst;
    board_b.watch(&catcher);

    std::vector<std::vector<double>> latency(static_cast<size_t>(burst));
    std::vector<double> board_steps;  // B's own view of the CAN inter-arrival
    std::vector<double> uplink_delta; // host step - board step == uplink-only jitter
    int complete = 0, incomplete = 0;

    for (int trial = 0; trial < trials; ++trial) {
        for (int i = 0; i < burst; ++i) {
            catcher.arrival_ns[i].store(0, std::memory_order_relaxed);
            catcher.board_us_valid[i].store(false, std::memory_order_relaxed);
        }
        catcher.received.store(0, std::memory_order_release);

        catcher.start = Clock::now();
        const auto start_abs_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(catcher.start.time_since_epoch())
                .count());
        {
            // One builder scope == one USB packet, so the whole burst crosses USB
            // once. Anything that scales with position therefore happened after
            // the packet landed on the board.
            auto builder = board_a.board().start_transmit();
            for (int i = 0; i < burst; ++i) {
                std::byte payload[kPayloadSize];
                put_u32_le(payload, static_cast<uint32_t>(i));
                put_u32_le(payload + 4, 0);
                builder.can_transmit(CanPort::kCan1, 
                    {.can_id = kCanIdBase, .can_data = payload, .is_fdcan = true});
            }
        }

        const auto deadline = Clock::now() + kBurstTimeout;
        while (catcher.received.load(std::memory_order_acquire) < burst
               && Clock::now() < deadline) {}

        if (catcher.received.load(std::memory_order_acquire) < burst) {
            ++incomplete;
        } else {
            ++complete;
            for (int i = 0; i < burst; ++i)
                latency[static_cast<size_t>(i)].push_back(
                    static_cast<double>(catcher.arrival_ns[i].load(std::memory_order_relaxed))
                    / 1e3);
            // Compare host and board WITHIN one burst. Consecutive frames are
            // ~51 us apart, so the two free-running clocks cannot drift
            // measurably between them -- unlike a cross-trial comparison, where
            // 2 ms of spacing lets even 1000 ppm of drift swamp a 14 us signal.
            //
            //   board_step = when B latched frame i vs i-1   (CAN side)
            //   host_step  = when the host saw them          (CAN + uplink)
            //   divergence = host_step - board_step          (uplink only)
            for (int i = 1; i < burst; ++i) {
                if (!catcher.board_us_valid[i].load(std::memory_order_relaxed)
                    || !catcher.board_us_valid[i - 1].load(std::memory_order_relaxed))
                    continue;
                const auto board_step = static_cast<int32_t>(
                    catcher.board_us[i].load(std::memory_order_relaxed)
                    - catcher.board_us[i - 1].load(std::memory_order_relaxed));
                const double host_step =
                    (static_cast<double>(catcher.arrival_ns[i].load(std::memory_order_relaxed))
                     - static_cast<double>(
                         catcher.arrival_ns[i - 1].load(std::memory_order_relaxed)))
                    / 1e3;
                board_steps.push_back(static_cast<double>(board_step));
                uplink_delta.push_back(host_step - static_cast<double>(board_step));
            }
        }
        // Let the bus and both FIFOs drain before the next burst, so each trial
        // starts from the same idle state a 1 kHz control loop would see.
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }

    printf(
        "burst=%d  trials=%d complete / %d incomplete  (CAN-FD 1M/5M, 8-byte payload)\n", burst,
        complete, incomplete);
    if (complete == 0) {
        fprintf(stderr, "no complete burst -- check the A.CAN1 <-> B.CAN1 wiring\n");
        return 1;
    }

    // Is the spread above min a quantization grid or continuous jitter? A grid
    // (the host controller retrying a NAKing bulk IN endpoint on a fixed period)
    // shows up as discrete clusters and could in principle be dodged; continuous
    // spread cannot. 2 us bins resolve a 125 us microframe grid easily.
    {
        auto samples = latency[0]; // copy: percentile() sorts in place
        std::sort(samples.begin(), samples.end());
        const double floor_us = samples.front();
        constexpr double kBin = 2.0;
        const auto bins = static_cast<size_t>((samples.back() - floor_us) / kBin) + 1;
        std::vector<int> histogram(bins, 0);
        for (const double value : samples)
            ++histogram[static_cast<size_t>((value - floor_us) / kBin)];
        printf(
            "\n  pos0 distribution, 2us bins above min=%.1f us (n=%zu)\n", floor_us,
            samples.size());
        const int peak = *std::max_element(histogram.begin(), histogram.end());
        for (size_t i = 0; i < bins; ++i) {
            if (histogram[i] * 100 < peak) // hide the long empty tail
                continue;
            const int width = peak > 0 ? histogram[i] * 50 / peak : 0;
            printf(
                "  +%5.0f us %6d |%s\n", static_cast<double>(i) * kBin, histogram[i],
                std::string(static_cast<size_t>(width), '#').c_str());
        }
        printf("\n");
    }

    // Split the per-frame step into its CAN half and its uplink half. The CAN
    // half is what B's own timestamp unit saw; the uplink half is everything
    // that happened to the frame after that. Whichever one carries the ~14 us
    // bimodality of 8.5.1 is where the time is actually lost.
    if (!board_steps.empty()) {
        const auto report = [](const char* label, std::vector<double> values) {
            std::sort(values.begin(), values.end());
            printf(
                "  %-28s n=%zu  min %.1f  p50 %.1f  p90 %.1f  max %.1f us\n", label, values.size(),
                values.front(), values[values.size() / 2], values[values.size() * 90 / 100],
                values.back());
            const double floor_us = values.front();
            constexpr double kBin = 2.0;
            const auto bins = static_cast<size_t>((values.back() - floor_us) / kBin) + 1;
            std::vector<int> histogram(bins, 0);
            for (const double value : values)
                ++histogram[static_cast<size_t>((value - floor_us) / kBin)];
            const int peak = *std::max_element(histogram.begin(), histogram.end());
            for (size_t i = 0; i < bins; ++i) {
                if (histogram[i] * 40 < peak)
                    continue;
                printf(
                    "    %+6.0f us %6d |%s\n", floor_us + static_cast<double>(i) * kBin,
                    histogram[i],
                    std::string(static_cast<size_t>(histogram[i] * 40 / peak), '#').c_str());
            }
        };
        report("CAN half (B's TSU step)", board_steps);
        report("uplink half (host - board)", uplink_delta);
        printf("\n");
    } else {
        printf("  (no hardware timestamps on the uplink -- cannot split the halves)\n\n");
    }

    printf("  pos      p50        min       step(p50)\n");
    double previous = 0.0;
    for (int i = 0; i < burst; ++i) {
        auto& samples = latency[static_cast<size_t>(i)];
        const double p50 = percentile(samples, 0.50);
        const double min = samples.front(); // percentile() sorted it
        if (i == 0)
            printf("  %3d  %8.1f   %8.1f          --\n", i, p50, min);
        else
            printf("  %3d  %8.1f   %8.1f    %8.1f\n", i, p50, min, p50 - previous);
        previous = p50;
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    const int burst = argc > 1 ? std::atoi(argv[1]) : 8;
    const int trials = argc > 2 ? std::atoi(argv[2]) : 2000;
    if (burst < 2 || burst > kMaxBurst) {
        fprintf(stderr, "burst must be 2..%d\n", kMaxBurst);
        return 1;
    }

    try {
        const auto serials = enumerate_serials();
        if (serials.size() < 2) {
            fprintf(stderr, "need two hpm5321_dual_can boards, found %zu\n", serials.size());
            return 1;
        }
        g_serial_a = serials[0];
        g_serial_b = serials[1];
        configure_thread(-1, 80, "split-main");
        return run(burst, trials);
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }
}
