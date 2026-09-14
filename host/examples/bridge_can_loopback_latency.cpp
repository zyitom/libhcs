// Queue-free CAN-FD loopback latency over USB. Wire the first pair as one
// terminated bus:
//   HPM6E8Y (usb):              silk CAN0 <-> CAN1
//   HPM5321 DualCan (usb-5321): silk CAN1 <-> CAN2
// One frame is in flight at a time, so the host-clock RTT includes transport
// downlink, board CAN TX, one CAN frame on the wire, board CAN RX, and
// transport uplink.
//
// Run:
//   sudo ./bridge_can_loopback_latency usb [samples] [io_core] [main_core]
//
// A third mode, "usb-paced", models a fixed-rate control loop instead of a
// back-to-back ping-pong: one sample is sent every 1/hz second (default 1000
// Hz), and the RTT distribution a 1 kHz loop would see over USB is reported
// with the jitter (p99-p50) called out.
//   sudo ./bridge_can_loopback_latency usb-paced [hz] [samples] [io_core] [main_core]

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string_view>
#include <thread>
#include <vector>

#include <pthread.h>
#include <sched.h>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/board/hpm6e8y.hpp>


// CAN ports are named as the silkscreen on THAT board. 6E8Y starts at CAN0;
// 5321 prints CAN1/CAN2. See libhcs/board/hcs_can_port.hpp.
using libhcs::board::hcs::CanPort;

namespace {

constexpr uint32_t kCanId = 0x556;
constexpr size_t kPayloadSize = 8;

// This probe used to difference classic-vs-FD round trips (HCS_CAN_CLASSIC=1)
// to separate CAN serialization time from the rest of the loop. That axis is
// gone with the per-frame is_fdcan flag: the board now puts every frame on the
// wire in its bus's compiled mode, so the CAN serialization share is a fixed
// term of every measurement instead of a switchable one.
constexpr uint32_t kWarmupSamples = 100;
constexpr auto kEchoTimeout = std::chrono::milliseconds{20};

uint32_t mix(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

void put_u32_le(std::byte* dst, uint32_t value) {
    dst[0] = static_cast<std::byte>(value);
    dst[1] = static_cast<std::byte>(value >> 8);
    dst[2] = static_cast<std::byte>(value >> 16);
    dst[3] = static_cast<std::byte>(value >> 24);
}

uint32_t get_u32_le(const std::byte* src) {
    return static_cast<uint32_t>(std::to_integer<uint8_t>(src[0]))
         | static_cast<uint32_t>(std::to_integer<uint8_t>(src[1])) << 8
         | static_cast<uint32_t>(std::to_integer<uint8_t>(src[2])) << 16
         | static_cast<uint32_t>(std::to_integer<uint8_t>(src[3])) << 24;
}

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

double percentile(const std::vector<double>& sorted, double fraction) {
    if (sorted.empty())
        return 0.0;
    const size_t index =
        static_cast<size_t>(fraction * static_cast<double>(sorted.size() - 1));
    return sorted[index];
}

// Two board classes so the SAME measurement code (warm-up, queue-free
// discipline, percentile math) serves both. Comparing boards with two
// different tools would not be a comparison.
class Receiver final
    : public libhcs::board::Hpm6e8y::Callback
    , public libhcs::board::Hpm5321::Callback {
public:
    using Clock = std::chrono::steady_clock;

    void arm(uint32_t sequence, Clock::time_point send_time) {
        while ((state_.load(std::memory_order_acquire) & kStateMask) == kWritingState) {}
        send_time_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(send_time.time_since_epoch())
                .count(),
            std::memory_order_relaxed);
        state_.store(static_cast<uint64_t>(sequence) << kStateBits, std::memory_order_release);
    }

    bool wait(uint32_t sequence, double& rtt_us) {
        const auto deadline = Clock::now() + kEchoTimeout;
        const uint64_t received_state =
            (static_cast<uint64_t>(sequence) << kStateBits) | kReceivedState;
        while (state_.load(std::memory_order_acquire) != received_state) {
            if (Clock::now() >= deadline)
                return false;
        }
        rtt_us = static_cast<double>(rtt_ns_.load(std::memory_order_relaxed)) / 1e3;
        return true;
    }

    uint64_t invalid() const { return invalid_.load(std::memory_order_relaxed); }
    uint64_t unexpected() const { return unexpected_.load(std::memory_order_relaxed); }

    // 6E8Y first pair echoes on silk CAN1; 5321 DualCan echoes on silk CAN2.
    void set_echo_port(CanPort port) { echo_port_ = port; }

private:
    void can_receive(
        CanPort port, const libhcs::data::CanDataView& data) override {
        if (port != echo_port_) {
            unexpected_.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        const auto receive_time = Clock::now();
        if (data.can_id != kCanId || data.is_extended_can_id
            || data.is_remote_transmission || data.can_data.size() != kPayloadSize) {
            invalid_.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        const uint32_t sequence = get_u32_le(data.can_data.data());
        if (get_u32_le(data.can_data.data() + 4) != mix(sequence)) {
            invalid_.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        uint64_t expected_state = static_cast<uint64_t>(sequence) << kStateBits;
        if (!state_.compare_exchange_strong(
                expected_state, expected_state | kWritingState, std::memory_order_acquire,
                std::memory_order_relaxed)) {
            unexpected_.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        const int64_t receive_time_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(receive_time.time_since_epoch())
                .count();
        rtt_ns_.store(
            receive_time_ns - send_time_ns_.load(std::memory_order_relaxed),
            std::memory_order_relaxed);
        state_.store(
            (static_cast<uint64_t>(sequence) << kStateBits) | kReceivedState,
            std::memory_order_release);
    }

    static constexpr unsigned kStateBits = 2;
    static constexpr uint64_t kStateMask = (1U << kStateBits) - 1U;
    static constexpr uint64_t kWritingState = 1;
    static constexpr uint64_t kReceivedState = 2;

    std::atomic<uint64_t> state_{kReceivedState};
    std::atomic<int64_t> send_time_ns_{0};
    std::atomic<int64_t> rtt_ns_{0};
    std::atomic<uint64_t> invalid_{0};
    std::atomic<uint64_t> unexpected_{0};
    CanPort echo_port_{CanPort::kCan2};
};

template <typename Board>
int run_latency(
    Board& board, Receiver& receiver, uint32_t samples, std::string_view transport,
    CanPort tx_port) {
    std::vector<double> rtts_us;
    rtts_us.reserve(samples);
    uint32_t timeouts = 0;

    printf(
        "%.*s: %u measured CAN-FD round trips after %u warm-up samples\n",
        static_cast<int>(transport.size()), transport.data(), samples, kWarmupSamples);

    for (uint32_t sequence = 0; sequence < samples + kWarmupSamples; ++sequence) {
        std::byte payload[kPayloadSize];
        put_u32_le(payload, sequence);
        put_u32_le(payload + 4, mix(sequence));

        receiver.arm(sequence, Receiver::Clock::now());
        board.start_transmit().can_transmit(tx_port,
            {.can_id = kCanId, .can_data = payload});

        double rtt_us = 0.0;
        if (!receiver.wait(sequence, rtt_us)) {
            if (sequence >= kWarmupSamples)
                ++timeouts;
            continue;
        }
        if (sequence >= kWarmupSamples)
            rtts_us.push_back(rtt_us);
    }

    if (rtts_us.empty()) {
        fprintf(
            stderr,
            "no valid loopback frames received (invalid=%llu unexpected=%llu)\n"
            "  invalid>0   : frames came back but failed the id/dlc/fd/payload check\n"
            "  both zero   : nothing reached the echo port at all\n",
            static_cast<unsigned long long>(receiver.invalid()),
            static_cast<unsigned long long>(receiver.unexpected()));
        return 2;
    }

    std::sort(rtts_us.begin(), rtts_us.end());
    double sum = 0.0;
    for (const double value : rtts_us)
        sum += value;

    printf(
        "samples=%zu timeout=%u invalid=%llu unexpected=%llu\n", rtts_us.size(), timeouts,
        static_cast<unsigned long long>(receiver.invalid()),
        static_cast<unsigned long long>(receiver.unexpected()));
    printf(
        "rtt us: min %.1f  p50 %.1f  p90 %.1f  p99 %.1f  p99.9 %.1f  avg %.1f  max %.1f\n",
        rtts_us.front(), percentile(rtts_us, 0.50), percentile(rtts_us, 0.90),
        percentile(rtts_us, 0.99), percentile(rtts_us, 0.999),
        sum / static_cast<double>(rtts_us.size()), rtts_us.back());

    const bool clean = rtts_us.size() == samples && timeouts == 0 && receiver.invalid() == 0
                    && receiver.unexpected() == 0;
    printf("result: %s\n", clean ? "PASS" : "CHECK counters above");
    return clean ? 0 : 2;
}

// Print the RTT distribution plus the jitter line (p99-p50 explicitly, since
// jitter is the metric a control loop cares about). rtts_us is sorted in place.
void print_rtt_distribution(std::vector<double>& rtts_us) {
    std::sort(rtts_us.begin(), rtts_us.end());
    double sum = 0.0;
    for (const double value : rtts_us)
        sum += value;
    const double p50 = percentile(rtts_us, 0.50);
    const double p99 = percentile(rtts_us, 0.99);
    const double p999 = percentile(rtts_us, 0.999);
    printf(
        "rtt us: min %.1f  p50 %.1f  p90 %.1f  p99 %.1f  p99.9 %.1f  avg %.1f  max %.1f\n",
        rtts_us.front(), p50, percentile(rtts_us, 0.90), p99, p999,
        sum / static_cast<double>(rtts_us.size()), rtts_us.back());
    printf(
        "jitter us: p99-p50 %.1f  p99.9-p50 %.1f  max-p50 %.1f\n", p99 - p50, p999 - p50,
        rtts_us.back() - p50);
}

// Paced (fixed-rate control loop) variant of run_latency: instead of sending
// the next sample the instant the previous echo returns, send exactly one
// sample per 1/hz tick. Because the loopback RTT (~125us over USB) is far below
// a 1 kHz period, one sample is outstanding at a time, so this measures the RTT
// a real timed control loop observes. The tick boundary is hit by busy-waiting
// (SCHED_FIFO), and the achieved tick period is reported so scheduler jitter is
// visible.
template <typename Board>
int run_latency_paced(
    Board& board, Receiver& receiver, uint32_t samples, uint32_t hz, std::string_view transport,
    CanPort tx_port) {
    using Clock = Receiver::Clock;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>{1.0 / static_cast<double>(hz)});

    std::vector<double> rtts_us;
    rtts_us.reserve(samples);
    std::vector<double> tick_us;
    tick_us.reserve(samples);
    uint32_t timeouts = 0;

    printf(
        "%.*s paced: %u round trips at %u Hz after %u warm-up samples\n",
        static_cast<int>(transport.size()), transport.data(), samples, hz, kWarmupSamples);

    auto next_tick = Clock::now() + period;
    Clock::time_point last_tick{};
    bool have_last = false;
    for (uint32_t sequence = 0; sequence < samples + kWarmupSamples; ++sequence) {
        while (Clock::now() < next_tick) {}
        const auto tick_time = Clock::now();
        if (have_last && sequence >= kWarmupSamples)
            tick_us.push_back(
                std::chrono::duration<double, std::micro>(tick_time - last_tick).count());
        last_tick = tick_time;
        have_last = true;

        std::byte payload[kPayloadSize];
        put_u32_le(payload, sequence);
        put_u32_le(payload + 4, mix(sequence));

        receiver.arm(sequence, tick_time);
        board.start_transmit().can_transmit(tx_port,
            {.can_id = kCanId, .can_data = payload});

        double rtt_us = 0.0;
        if (receiver.wait(sequence, rtt_us)) {
            if (sequence >= kWarmupSamples)
                rtts_us.push_back(rtt_us);
        } else if (sequence >= kWarmupSamples) {
            ++timeouts;
        }

        next_tick += period;
        // Resync if a slow iteration overran a whole period, so the cadence does
        // not spiral into a permanent catch-up.
        const auto now = Clock::now();
        if (next_tick < now)
            next_tick = now + period;
    }

    if (rtts_us.empty()) {
        fprintf(
            stderr,
            "no valid loopback frames received (invalid=%llu unexpected=%llu)\n"
            "  invalid>0   : frames came back but failed the id/dlc/fd/payload check\n"
            "  both zero   : nothing reached the echo port at all\n",
            static_cast<unsigned long long>(receiver.invalid()),
            static_cast<unsigned long long>(receiver.unexpected()));
        return 2;
    }

    printf(
        "samples=%zu timeout=%u invalid=%llu unexpected=%llu\n", rtts_us.size(), timeouts,
        static_cast<unsigned long long>(receiver.invalid()),
        static_cast<unsigned long long>(receiver.unexpected()));
    print_rtt_distribution(rtts_us);
    if (!tick_us.empty()) {
        std::sort(tick_us.begin(), tick_us.end());
        printf(
            "tick period us: min %.1f  p50 %.1f  max %.1f (target %.1f)\n", tick_us.front(),
            percentile(tick_us, 0.50), tick_us.back(), 1e6 / static_cast<double>(hz));
    }

    const bool clean = rtts_us.size() == samples && timeouts == 0 && receiver.invalid() == 0
                    && receiver.unexpected() == 0;
    printf("result: %s\n", clean ? "PASS" : "CHECK counters above");
    return clean ? 0 : 2;
}

int parse_int(const char* value, int fallback) {
    return value ? std::atoi(value) : fallback;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(
            stderr,
            "usage: %s usb [samples] [io_core] [main_core]\n"
            "       %s usb-5321 [samples] [io_core] [main_core]\n"
            "       %s usb-paced [hz] [samples] [io_core] [main_core]\n",
            argv[0], argv[0], argv[0]);
        return 1;
    }

    const std::string_view mode = argv[1];
    const bool paced = mode == "usb-paced";
    const bool use_5321 = mode == "usb-5321";
    if (!paced && !use_5321 && mode != "usb") {
        fprintf(stderr, "transport must be usb, usb-5321, or usb-paced\n");
        return 1;
    }

    // Paced mode takes an extra leading hz argument: usb-paced [hz] [samples] ...
    int hz = 1000;
    int sample_value;
    int io_core;
    int main_core;
    if (paced) {
        hz = parse_int(argc > 2 ? argv[2] : nullptr, 1000);
        sample_value = parse_int(argc > 3 ? argv[3] : nullptr, 2000);
        io_core = parse_int(argc > 4 ? argv[4] : nullptr, -1);
        main_core = parse_int(argc > 5 ? argv[5] : nullptr, -1);
        if (hz <= 0) {
            fprintf(stderr, "hz must be positive\n");
            return 1;
        }
    } else {
        const int argument_base = 2;
        sample_value = parse_int(argc > argument_base ? argv[argument_base] : nullptr, 2000);
        io_core = parse_int(argc > argument_base + 1 ? argv[argument_base + 1] : nullptr, -1);
        main_core = parse_int(argc > argument_base + 2 ? argv[argument_base + 2] : nullptr, -1);
    }
    if (sample_value <= 0) {
        fprintf(stderr, "samples must be positive\n");
        return 1;
    }

    auto options = libhcs::board::bind_advanced_options([io_core]() noexcept {
        configure_thread(io_core, 80, "bridge-lat-io");
    });

    // Diagnostic escape hatch: the USB scanner demands an exact firmware/host
    // build match, which blocks measuring a board that is a few commits behind
    // the tree. Set HCS_SKIP_VERSION_CHECK=1 only when the wire protocol is
    // known to be unchanged across those commits; the tool still validates
    // every echoed payload, so a real mismatch shows up as invalid frames.
    if (const char* skip = std::getenv("HCS_SKIP_VERSION_CHECK"); skip && skip[0] == '1')
        options.set_dangerously_skip_version_checks(true);

    Receiver receiver;
    try {
        if (use_5321) {
            // HPM5321 DualCan (PID 0x5322): silk CAN1 -> CAN2. Same physical
            // first-pair shape as 6E8Y's silk CAN0 -> CAN1.
            receiver.set_echo_port(CanPort::kCan2);
            libhcs::board::Hpm5321 board{receiver, {}, options};
            configure_thread(main_core, 70, "bridge-lat-main");
            return run_latency(
                board, receiver, static_cast<uint32_t>(sample_value), "USB/HPM5321",
                CanPort::kCan1);
        }

        receiver.set_echo_port(CanPort::kCan1);
        libhcs::board::Hpm6e8y board{receiver, {}, options};
        configure_thread(main_core, 70, "bridge-lat-main");
        if (paced)
            return run_latency_paced(
                board, receiver, static_cast<uint32_t>(sample_value), static_cast<uint32_t>(hz),
                "USB", CanPort::kCan0);
        return run_latency(
            board, receiver, static_cast<uint32_t>(sample_value), "USB", CanPort::kCan0);
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }
}
