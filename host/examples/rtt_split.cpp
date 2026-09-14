// Split the CAN loopback round trip into every measured term.
//
// Reads THREE instruments in one run:
//   1. Host-clock RTT per ping-pong (submit -> echo callback), like
//      can_loopback_latency but with the production thread layout
//      (submitter CPU3 FIFO85, SDK event thread CPU7 FIFO80).
//   2. The board's own EP0 latency breakdown (diag/latency.hpp): downlink =
//      bulk-OUT callback -> frame in the MCAN TX FIFO; uplink = CAN RX
//      interrupt -> frame serialized into the uplink batch. Cycles, with the
//      board-reported cpu_hz. This is the board's share of the round trip,
//      read without touching the wire and without cross-clock guessing.
//   3. HCS_PING_LEN payload sweep (0/1/8): RTT differences are pure CAN wire
//      time differences -- the only clock-domain-free way to size the wire.
//
// Env:  HCS_PING_COUNT (default 20000)
//       HCS_LATENCY_GAP_US (default 1000)
//       HCS_PING_LEN (default 8; 0..8, firmware drops FD frames with DLC > 8)
// Args: [serial_filter] [csv_path]
//
// Run as root: sudo ./rtt_split AF-90A7 /tmp/rtt_split.csv

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hpm5321.hpp>

namespace {

using libhcs::board::hcs::CanPort;

constexpr uint32_t kPingCanId = 0x555;  // unique on the loopback wire
const uint32_t kPingCount = [] {
    const char* env = std::getenv("HCS_PING_COUNT");
    return (env != nullptr && *env != '\0') ? static_cast<uint32_t>(std::atoi(env)) : 20000u;
}();
const auto kInterPingGap = std::chrono::microseconds{[] {
    const char* env = std::getenv("HCS_LATENCY_GAP_US");
    return (env != nullptr && *env != '\0') ? std::atoi(env) : 1000;
}()};
// Payload length in bytes (0..8). RTT(len_a) - RTT(len_b) = wire(len_a) -
// wire(len_b): the only clock-domain-free way to size the CAN wire time.
const uint32_t kPingLen = [] {
    const char* env = std::getenv("HCS_PING_LEN");
    return (env != nullptr && *env != '\0') ? static_cast<uint32_t>(std::atoi(env)) : 8u;
}();
constexpr auto kEchoTimeout = std::chrono::milliseconds{50};

std::atomic<bool> g_running{true};
void on_sigint(int) { g_running.store(false, std::memory_order_relaxed); }

// Production thread layout: submitter where the control loop's senders live,
// SDK USB event thread where the probe's IO thread lives. Do not "simplify" --
// recv_ns would then measure CFS instead of USB.
void pin_self_cpu3_fifo85() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0)
        printf("Warning: failed to pin submitter to CPU 3\n");
    sched_param param{};
    param.sched_priority = 85;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
        printf("Warning: no SCHED_FIFO for submitter\n");
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        printf("Warning: mlockall failed\n");
}

class SplitCallback final : public libhcs::board::Hpm5321::Callback {
public:
    // Echo arrives on CAN2 (the receiving end of the joined wire), on the SDK
    // USB event thread.
    void can_receive(libhcs::board::hcs::CanPort port,
        const libhcs::data::CanDataView& data) override {
        if (port != CanPort::kCan2)
            return;
        const auto recv = now_ns();
        if (data.can_id != kPingCanId)
            return;

        // len >= 4: match by the seq echoed in the payload. Shorter payloads
        // carry no seq, but ping-pong keeps exactly one frame in flight, so
        // arrival order is the match.
        if (kPingLen >= 4) {
            uint32_t seq = 0;
            std::memcpy(&seq, data.can_data.data(), sizeof(seq));
            if (seq != ping_seq_.load(std::memory_order_acquire))
                return;
        }
        if (got_echo_.exchange(true, std::memory_order_acq_rel))
            return;  // already matched

        recv_ns_.store(recv, std::memory_order_relaxed);
        had_ts_.store(data.timestamp_us.has_value(), std::memory_order_relaxed);
    }

    static int64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    std::atomic<uint32_t> ping_seq_{0xFFFFFFFFU};
    std::atomic<int64_t> send_ns_{0};
    std::atomic<int64_t> recv_ns_{0};
    std::atomic<bool> got_echo_{false};
    std::atomic<bool> had_ts_{false};
};

double percentile(std::vector<double>& sorted, double p) {
    if (sorted.empty())
        return 0.0;
    std::sort(sorted.begin(), sorted.end());
    return sorted[static_cast<size_t>(static_cast<double>(sorted.size() - 1) * p)];
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, on_sigint);
    pin_self_cpu3_fifo85();

    const char* serial = argc > 1 ? argv[1] : "";
    const char* csv_path = argc > 2 ? argv[2] : "rtt_split.csv";

    libhcs::board::AdvancedOptions options;
    options.set_io_thread_affinity(7, 80);

    printf("Connecting (serial filter '%s') ...\n", serial);
    SplitCallback callback;
    auto board = std::make_unique<libhcs::board::Hpm5321>(callback, serial, options);

    FILE* csv = std::fopen(csv_path, "w");
    if (!csv) {
        fprintf(stderr, "Cannot open %s\n", csv_path);
        return 1;
    }
    std::fprintf(csv, "seq,submit_ns,recv_ns,board_us\n");

    printf("Measuring %u round trips, payload %u bytes, gap %lld us -> %s\n\n", kPingCount,
        kPingLen, static_cast<long long>(kInterPingGap.count()), csv_path);

    std::vector<double> rtt_us;
    rtt_us.reserve(kPingCount);
    uint32_t lost = 0, no_ts = 0;

    for (uint32_t seq = 1; seq <= kPingCount && g_running.load(std::memory_order_relaxed);
         ++seq) {
        std::array<std::byte, 8> frame{};
        std::memcpy(frame.data(), &seq, sizeof(seq));

        callback.got_echo_.store(false, std::memory_order_relaxed);
        callback.send_ns_.store(SplitCallback::now_ns(), std::memory_order_relaxed);
        callback.ping_seq_.store(seq, std::memory_order_release);

        board->start_transmit().can_transmit(CanPort::kCan1,
            {.can_id = kPingCanId, .can_data = {frame.data(), kPingLen}});

        const auto deadline = std::chrono::steady_clock::now() + kEchoTimeout;
        bool got = false;
        while (!callback.got_echo_.load(std::memory_order_acquire)) {
            if (!g_running.load(std::memory_order_relaxed)
                || std::chrono::steady_clock::now() >= deadline)
                break;
            std::this_thread::sleep_for(std::chrono::microseconds{20});
        }
        if (callback.got_echo_.load(std::memory_order_acquire)) {
            const auto submit = callback.send_ns_.load(std::memory_order_relaxed);
            const auto recv = callback.recv_ns_.load(std::memory_order_relaxed);
            rtt_us.push_back(static_cast<double>(recv - submit) / 1e3);
            if (!callback.had_ts_.load(std::memory_order_relaxed))
                ++no_ts;
            std::fprintf(csv, "%u,%lld,%lld,%u\n", seq, static_cast<long long>(submit),
                static_cast<long long>(recv), 0u);
        } else {
            ++lost;
        }
        std::this_thread::sleep_for(kInterPingGap);
    }
    std::fclose(csv);

    printf("=== RTT (host clock, %zu samples, %u lost, %u without board ts) ===\n",
        rtt_us.size(), lost, no_ts);
    printf("  min/p50/p99/max = %.1f / %.1f / %.1f / %.1f us\n", percentile(rtt_us, 0.0),
        percentile(rtt_us, 0.50), percentile(rtt_us, 0.99), percentile(rtt_us, 1.0));

    // The board's own share, straight from its EP0 counters. Read AFTER the
    // run so the segments cover exactly these pings; reset=true keeps
    // consecutive runs independent.
    const auto lb = board->latency_breakdown(true);
    const double mhz = static_cast<double>(lb.cpu_hz) / 1e6;
    printf("=== Board latency breakdown (cpu_hz=%u) ===\n", lb.cpu_hz);
    if (lb.downlink_count != 0) {
        printf("  downlink (OUT cb -> TX FIFO): n=%u min/p50avg/max = %.2f / %.2f / %.2f us\n",
            lb.downlink_count, static_cast<double>(lb.downlink_min_cycles) / mhz,
            static_cast<double>(lb.downlink_sum_cycles) / lb.downlink_count / mhz,
            static_cast<double>(lb.downlink_max_cycles) / mhz);
    } else {
        printf("  downlink: no samples\n");
    }
    if (lb.uplink_count != 0) {
        printf("  uplink   (CAN ISR -> serialized): n=%u min/p50avg/max = %.2f / %.2f / %.2f us\n",
            lb.uplink_count, static_cast<double>(lb.uplink_min_cycles) / mhz,
            static_cast<double>(lb.uplink_sum_cycles) / lb.uplink_count / mhz,
            static_cast<double>(lb.uplink_max_cycles) / mhz);
    } else {
        printf("  uplink: no samples\n");
    }
    if (no_ts != 0)
        printf("  note: %u echoes had no TSU timestamp (normal for DLC 0)\n", no_ts);
    return 0;
}
