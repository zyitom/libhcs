// Round-trip CAN forwarding latency for any two-CAN-bus board (hpm5321_dual_can,
// c_board, mc02), measured entirely in the host clock -- so there is NO
// cross-clock-domain error (unlike subtracting the board's hardware timestamp
// from a host timestamp, which mixes two unsynchronized clocks; see rx_monitor,
// which only measures jitter).
//
// WIRING: join CAN bus 0 and bus 1 onto ONE wire -- CAN1_H<->CAN2_H,
// CAN1_L<->CAN2_L -- with a 120 ohm terminator at EACH end. The host sends a
// tagged frame on bus 0; the board drives it onto the wire; bus 1 receives it
// (and ACKs bus 0); the board forwards it back to the host. The measured round
// trip therefore covers:
//
//   RTT = USB down + board TX + CAN wire (once) + board RX + USB up + host stack
//
// Subtract the CAN frame's on-wire time to get the board+USB round trip; halve
// it for a rough one-way (board RX + USB up) estimate.
//
// One frame is in flight at a time (ping-pong) so each sample is a clean,
// queue-free round trip. For stable numbers run with realtime scheduling, e.g.
//   sudo chrt -f 80 ./can_loopback_latency
// (the tool also best-effort sets SCHED_FIFO + pins a CPU itself).

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include "common/multi_board.hpp"

namespace {

constexpr bool kUseCanFd = false;          // what YOUR bus runs (no per-frame switch)
constexpr uint32_t kPingCanId = 0x555;     // unique on the loopback bus
constexpr uint32_t kPingCount = 2000;      // samples to collect
// Pause between pings. Tunable because it is the knob that decides whether the
// bulk IN endpoint has been NAKing when the echo arrives: a long gap leaves the
// pipe idle, and the host controller's backoff for a NAKing endpoint then sits
// in the measured round trip. Sweeping it separates that backoff from the fixed
// costs (CAN wire, device turnaround) that do not depend on the gap.
const auto kInterPingGap = std::chrono::microseconds{[] {
    const char* env = std::getenv("HCS_LATENCY_GAP_US");
    return (env != nullptr && *env != '\0') ? std::atoi(env) : 1000;
}()};
constexpr auto kEchoTimeout = std::chrono::milliseconds{50};     // give-up per ping

// Approximate on-wire time of one frame, subtracted to isolate board+USB.
// Classic 1 Mbit standard-id 8-byte frame ~= 108-115 bit times. CAN-FD with BRS
// (1 Mbit arb / 5 Mbit data) is much shorter; adjust if your bus runs FD.
constexpr double kCanWireTimeUs = kUseCanFd ? 50.0 : 115.0;

std::atomic<bool> g_running{true};
void on_sigint(int) { g_running.store(false, std::memory_order_relaxed); }

void setup_realtime() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0)
        printf("Warning: failed to pin CPU\n");
    sched_param param{};
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
        printf("Warning: no SCHED_FIFO (run as root / chrt -f 80 for stable numbers)\n");
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        printf("Warning: mlockall failed\n");
}

double percentile(const std::vector<double>& sorted, double p) {
    if (sorted.empty())
        return 0.0;
    const auto idx = static_cast<size_t>((static_cast<double>(sorted.size() - 1) * p));
    return sorted[idx];
}

}  // namespace

class LoopbackTester : public examples::BoardReceiver {
public:
    void bind(examples::BoardSession* board) { board_ = board; }

    // Send ping <seq> on CAN bus 0 and arm matching of its echo. Records the send
    // instant as close as possible to the transmit call.
    void send_ping(uint32_t seq) {
        std::array<std::byte, 8> frame{};
        std::memcpy(frame.data(), &seq, sizeof(seq));

        got_echo_.store(false, std::memory_order_relaxed);
        send_tp_ = std::chrono::steady_clock::now();
        ping_seq_.store(seq, std::memory_order_release);  // publishes send_tp_

        board_->transmit([&](examples::BoardTransmitter& tx) {
            tx.can(0, {.can_id = kPingCanId, .can_data = frame});
        });
    }

    bool wait_echo() {
        const auto deadline = std::chrono::steady_clock::now() + kEchoTimeout;
        while (!got_echo_.load(std::memory_order_acquire)) {
            if (!g_running.load(std::memory_order_relaxed)
                || std::chrono::steady_clock::now() >= deadline)
                return false;
            std::this_thread::sleep_for(std::chrono::microseconds{20});
        }
        return true;
    }

    double last_rtt_us() const {
        return static_cast<double>(rtt_ns_.load(std::memory_order_relaxed)) / 1e3;
    }

private:
    // Echo arrives on CAN bus 1 (the receiving end of the joined wire).
    void on_can(int bus, const libhcs::data::CanDataView& data) override {
        if (bus != 1)
            return;
        const auto now = std::chrono::steady_clock::now();
        if (data.can_id != kPingCanId || data.can_data.size() < sizeof(uint32_t))
            return;

        uint32_t seq = 0;
        std::memcpy(&seq, data.can_data.data(), sizeof(seq));

        // acquire on ping_seq_ makes send_tp_ visible
        if (seq != ping_seq_.load(std::memory_order_acquire))
            return;
        if (got_echo_.exchange(true, std::memory_order_acq_rel))
            return;  // already matched this seq

        rtt_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now - send_tp_).count(),
            std::memory_order_relaxed);
    }

    std::atomic<uint32_t> ping_seq_{0xFFFFFFFFU};
    std::chrono::steady_clock::time_point send_tp_;
    std::atomic<bool> got_echo_{false};
    std::atomic<int64_t> rtt_ns_{0};

    examples::BoardSession* board_ = nullptr;
};

int main() {
    std::signal(SIGINT, on_sigint);
    setup_realtime();

    printf("CAN loopback latency (CAN-FD=%s)\n", kUseCanFd ? "on" : "off");
    printf("  Wire CAN bus 0 <-> bus 1 together (120 ohm at each end).\n");
    printf("  Sending id 0x%03X on bus 0, timing the echo on bus 1.\n", kPingCanId);
    printf("Connecting ...\n");

    LoopbackTester agent;
    auto board = examples::connect_any(agent);
    if (!board) {
        fprintf(stderr, "No compatible board found.\n");
        return 1;
    }
    if (board->can_bus_count() < 2) {
        fprintf(stderr, "%.*s has only %d CAN bus; this test needs two.\n",
            static_cast<int>(board->name().size()), board->name().data(),
            board->can_bus_count());
        return 1;
    }
    agent.bind(board.get());
    printf("Connected: %.*s. Measuring %u round trips ...\n\n",
        static_cast<int>(board->name().size()), board->name().data(), kPingCount);

    std::vector<double> rtt_us;
    rtt_us.reserve(kPingCount);
    uint32_t lost = 0;

    for (uint32_t seq = 0; seq < kPingCount && g_running.load(std::memory_order_relaxed); ++seq) {
        agent.send_ping(seq);
        if (agent.wait_echo())
            rtt_us.push_back(agent.last_rtt_us());
        else
            ++lost;
        std::this_thread::sleep_for(kInterPingGap);
    }

    if (rtt_us.empty()) {
        fprintf(stderr,
            "No echoes received. Check the bus0<->bus1 loopback wiring, termination,\n"
            "and that kUseCanFd matches the bus's compiled mode. (%u lost)\n", lost);
        return 1;
    }

    std::sort(rtt_us.begin(), rtt_us.end());
    double sum = 0.0;
    for (double v : rtt_us)
        sum += v;
    const double avg = sum / static_cast<double>(rtt_us.size());
    const double min = rtt_us.front();
    const double max = rtt_us.back();
    const double p50 = percentile(rtt_us, 0.50);
    const double p99 = percentile(rtt_us, 0.99);

    // board+USB round trip = RTT minus the single on-wire frame time.
    const double board_usb_rtt = avg - kCanWireTimeUs;
    const double one_way = board_usb_rtt / 2.0;

    printf("=== Round-trip latency (host clock, %zu samples, %u lost) ===\n",
           rtt_us.size(), lost);
    printf("  RTT us   min/p50/avg/p99/max = %.1f / %.1f / %.1f / %.1f / %.1f\n",
           min, p50, avg, p99, max);
    printf("\n");
    printf("  Derived (approx, -%.0f us CAN wire time):\n", kCanWireTimeUs);
    printf("    board+USB round trip ~= %.1f us\n", board_usb_rtt);
    printf("    one-way (board RX + USB up) ~= %.1f us\n", one_way);
    printf("\n");
    printf("  Note: one-way assumes symmetric down/up paths. For ground truth of\n");
    printf("  the board-internal part, use a scope on CAN_H/L + a board GPIO.\n");
    return 0;
}
