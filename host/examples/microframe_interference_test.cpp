// Does reading MFINDEX disturb the USB data path on the same controller?
//
// This is the one cost of the microframe time source that cannot be computed.
// The CPU duty cycle is arithmetic and tiny (measured at 24 us per edge, so
// 0.03% of a core at the 12 Hz that holding phase actually needs). What no
// arithmetic settles is whether hammering the controller's register interface
// during those windows interferes with the transfers it is retiring at the same
// moment -- and on this machine that question has form: a USB3 camera on this
// same controller cost the board p50 +4 us / p99 +9 us through interrupt
// contention while provably not competing for bandwidth.
//
// METHOD, AND WHY IT IS INTERLEAVED. The same firmware image on this rig has
// measured 47344 packets/s and then 49706 twenty minutes later. That drift is
// larger than any effect being looked for here, so a "before, then after" run
// would be measuring the clock on the wall. Blocks therefore alternate A/B/B/A
// within each round, and the round order flips, so any monotonic drift cancels
// to first order instead of landing on whichever condition ran second.
//
// WHAT TO LOOK AT. Not p50. A burst 24 us long inside a 125 us period can only
// touch a couple of the ~21 us packets around it, so even a severe local effect
// is a small fraction of all packets and cannot move a median. It has to show
// in the upper tail or it is not there -- hence p99 and p99.9 of the per-packet
// submit interval, which is back-pressured by real completions and so measures
// what the controller retires rather than how fast memcpy runs.
//
// Needs root (raw USB device access, and the BAR mapping).
//   sudo ./microframe_interference_test [hunt_hz] [rounds] [block_ms]
//
// hunt_hz 0 means hunt continuously -- a worst case nothing would ever run, but
// it bounds the effect from above.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>
#include <pthread.h>
#include <sched.h>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/time/microframe_source.hpp>

using libhcs::board::hcs::CanPort;

namespace {

using Board = libhcs::board::Hpm5321;
using Clock = std::chrono::steady_clock;
using libhcs::host::time::MicroframeSource;

constexpr std::uint32_t kCanIdBase = 0x560;
constexpr std::size_t kPayloadSize = 8;
constexpr int kBoardCore = 7; // isolated on this machine
constexpr int kHunterCore = 5;

void configure_thread(int core, int priority, const char* name) noexcept {
    if (name != nullptr)
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

private:
    NodeOptions options_;
    std::unique_ptr<Board> board_;
};

std::vector<std::string> enumerate_serials() {
    std::vector<std::string> found;
    DIR* dir = opendir("/sys/bus/usb/devices");
    if (dir == nullptr)
        return found;
    while (const dirent* entry = readdir(dir)) {
        const std::string base = std::string{"/sys/bus/usb/devices/"} + entry->d_name;
        const auto read_line = [&](const char* leaf) -> std::string {
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
        };
        if (read_line("idVendor") != "a511" || read_line("idProduct") != "5322")
            continue;
        const std::string serial = read_line("serial");
        if (!serial.empty())
            found.push_back(serial);
    }
    closedir(dir);
    std::ranges::sort(found);
    found.erase(std::unique(found.begin(), found.end()), found.end());
    return found;
}

double percentile(std::vector<double>& values, double fraction) {
    if (values.empty())
        return 0.0;
    std::ranges::sort(values);
    const auto index = static_cast<std::size_t>(fraction * static_cast<double>(values.size() - 1));
    return values[index];
}

// Busy-poll edges on a separate core for the duration of a block. Pinning it
// away from the board's IO thread is deliberate: CPU contention is the boring
// explanation and is easy to remove, which leaves the controller's own register
// interface as the only thing left to blame if an effect shows up.
class Hunter {
public:
    Hunter(MicroframeSource& source, double hertz)
        : source_(source)
        , hertz_(hertz) {}

    void start() {
        running_.store(true, std::memory_order_release);
        edges_.store(0, std::memory_order_relaxed);
        thread_ = std::thread{[this] {
            configure_thread(kHunterCore, 80, "mf-hunter");
            const auto interval =
                hertz_ > 0.0 ? std::chrono::nanoseconds{static_cast<std::int64_t>(1e9 / hertz_)}
                             : std::chrono::nanoseconds{0};
            while (running_.load(std::memory_order_acquire)) {
                if (source_.sample_edge())
                    edges_.fetch_add(1, std::memory_order_relaxed);
                if (interval.count() > 0)
                    std::this_thread::sleep_for(interval);
            }
        }};
    }

    std::uint64_t stop() {
        running_.store(false, std::memory_order_release);
        if (thread_.joinable())
            thread_.join();
        return edges_.load(std::memory_order_relaxed);
    }

private:
    MicroframeSource& source_;
    double hertz_;
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> edges_{0};
    std::thread thread_;
};

struct BlockResult {
    std::uint64_t packets;
    double seconds;
};

// One flood block. Each start_transmit() scope flushes exactly one USB packet,
// and acquire_transmit_buffer() blocks until a pooled transfer comes back from
// a completion callback -- so the loop is paced by what the controller actually
// retires.
BlockResult
    run_block(Node& node, std::chrono::milliseconds duration, std::vector<double>& intervals) {
    const std::byte payload[kPayloadSize] = {};
    const auto started = Clock::now();
    const auto deadline = started + duration;

    std::uint64_t packets = 0;
    auto previous = Clock::now();
    while (Clock::now() < deadline) {
        {
            auto builder = node.board().start_transmit();
            builder.can_transmit(
                CanPort::kCan1, {.can_id = kCanIdBase, .can_data = payload});
        }
        const auto now = Clock::now();
        intervals.push_back(
            static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(now - previous).count()));
        previous = now;
        ++packets;
    }
    const auto elapsed = Clock::now() - started;
    return BlockResult{
        .packets = packets,
        .seconds = std::chrono::duration<double>{elapsed}.count(),
    };
}

void report(const char* label, const std::vector<double>& rates, std::vector<double> intervals) {
    double sum = 0.0;
    for (const double rate : rates)
        sum += rate;
    const double mean = rates.empty() ? 0.0 : sum / static_cast<double>(rates.size());

    printf(
        "  %-24s %9.0f pkt/s   p50 %6.1f us  p99 %7.1f us  p99.9 %8.1f us\n", label, mean,
        percentile(intervals, 0.50) / 1000.0, percentile(intervals, 0.99) / 1000.0,
        percentile(intervals, 0.999) / 1000.0);
}

} // namespace

int main(int argc, char** argv) {
    const double hunt_hz = argc > 1 ? std::strtod(argv[1], nullptr) : 12.0;
    const int rounds = argc > 2 ? static_cast<int>(std::strtol(argv[2], nullptr, 10)) : 6;
    const auto block = std::chrono::milliseconds{
        argc > 3 ? static_cast<int>(std::strtol(argv[3], nullptr, 10)) : 1500};

    const std::vector<std::string> serials = enumerate_serials();
    if (serials.empty()) {
        printf("no board found\n");
        return 1;
    }

    auto opened = MicroframeSource::for_usb_bus(3);
    if (!opened) {
        printf(
            "cannot open the microframe source: %.*s\n",
            static_cast<int>(MicroframeSource::describe(opened.error()).size()),
            MicroframeSource::describe(opened.error()).data());
        return 1;
    }
    MicroframeSource source = std::move(*opened);

    printf("=== edge hunting vs the USB data path ===\n");
    printf("  board            : %s\n", serials.front().c_str());
    printf(
        "  controller       : %s (board on usb3, hunter on the same one --\n",
        source.pci_device().c_str());
    printf("                     that is the point; a different controller would\n");
    printf("                     not be a test of anything)\n");
    printf(
        "  hunt rate        : %s\n",
        hunt_hz > 0.0 ? (std::to_string(static_cast<int>(hunt_hz)) + " Hz").c_str()
                      : "continuous (worst case)");
    printf("  board IO core %d, hunter core %d, both SCHED_FIFO\n", kBoardCore, kHunterCore);
    printf(
        "  %d rounds x 2 blocks x %lld ms, order flipped each round\n\n", rounds,
        static_cast<long long>(block.count()));

    Node node{serials.front(), kBoardCore, "mf-flood"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    // Warm the pool and the hunter's period estimate so neither condition pays
    // a startup cost the other does not.
    std::vector<double> warmup;
    (void)run_block(node, std::chrono::milliseconds{400}, warmup);
    for (int i = 0; i < 20; ++i)
        (void)source.sample_edge();

    std::vector<double> quiet_rates;
    std::vector<double> hunted_rates;
    std::vector<double> quiet_intervals;
    std::vector<double> hunted_intervals;
    std::uint64_t total_edges = 0;

    for (int round = 0; round < rounds; ++round) {
        // A/B on even rounds, B/A on odd: a linear drift over the run then
        // contributes with opposite sign to the two halves.
        const bool hunt_first = (round % 2) == 1;

        for (int half = 0; half < 2; ++half) {
            const bool hunting = (half == 0) == hunt_first;
            if (hunting) {
                Hunter hunter{source, hunt_hz};
                hunter.start();
                const BlockResult result = run_block(node, block, hunted_intervals);
                total_edges += hunter.stop();
                hunted_rates.push_back(static_cast<double>(result.packets) / result.seconds);
            } else {
                const BlockResult result = run_block(node, block, quiet_intervals);
                quiet_rates.push_back(static_cast<double>(result.packets) / result.seconds);
            }
        }
        printf("  round %d/%d done\n", round + 1, rounds);
    }

    printf("\n=== result ===\n");
    report("A quiet", quiet_rates, quiet_intervals);
    report("B hunting", hunted_rates, hunted_intervals);

    double quiet_sum = 0.0;
    double hunted_sum = 0.0;
    for (const double rate : quiet_rates)
        quiet_sum += rate;
    for (const double rate : hunted_rates)
        hunted_sum += rate;
    const double quiet_mean = quiet_sum / static_cast<double>(quiet_rates.size());
    const double hunted_mean = hunted_sum / static_cast<double>(hunted_rates.size());
    const double change = ((hunted_mean / quiet_mean) - 1.0) * 100.0;

    // Spread of the per-block rates within one condition. If the A-to-B
    // difference is not clear of this, the run has not resolved anything and
    // saying "no effect" would be as unsupported as claiming one.
    const auto spread = [](const std::vector<double>& values, double mean) {
        double sum = 0.0;
        for (const double value : values)
            sum += (value - mean) * (value - mean);
        return values.size() > 1 ? std::sqrt(sum / static_cast<double>(values.size() - 1)) : 0.0;
    };
    const double quiet_sigma = spread(quiet_rates, quiet_mean);
    const double hunted_sigma = spread(hunted_rates, hunted_mean);
    const double noise = std::sqrt((quiet_sigma * quiet_sigma) + (hunted_sigma * hunted_sigma))
                       / std::sqrt(static_cast<double>(quiet_rates.size()));

    printf("\n  throughput change : %+.2f%%\n", change);
    printf("  block-to-block noise on that difference : +-%.2f%%\n", 100.0 * noise / quiet_mean);
    printf("  edges hunted      : %llu\n", static_cast<unsigned long long>(total_edges));

    if (std::fabs(hunted_mean - quiet_mean) < 2.0 * noise)
        printf(
            "\n  NOT RESOLVED as an effect: the difference is inside twice the\n"
            "  block-to-block noise. That is a bound, not a proof of zero --\n"
            "  what it says is that any effect is smaller than %.2f%%.\n",
            100.0 * 2.0 * noise / quiet_mean);
    else
        printf(
            "\n  REAL EFFECT: the difference clears twice the block-to-block\n"
            "  noise. Look at the tail figures above to see where it lands.\n");

    return 0;
}
