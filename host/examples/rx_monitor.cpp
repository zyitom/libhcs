#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <sys/mman.h>
#include <thread>

// CPU core for the real-time data-collection thread. Set to -1 to disable.
static constexpr int kRtCpu = 7;

#include "common/multi_board.hpp"

// Receive-only monitor. Auto-detects any project board (c_board, mc02, hpm5321,
// hpm5321_dual_can). CAN channels are named as the ENCLOSURE labels them
// (1-based): CAN1 is the board's first CAN bus. UART columns are the dense
// BoardTransmitter indices (0-based). On mc02 those are silkscreen UART1,
// UART2, UART3, UART7, UART10.
// Columns: host time, channel, size, host delta, hw-ts, hw-ts delta, fwd jitter
//   hw-ts delta  = delta between consecutive CAN hardware timestamps (1 us/tick)
//   fwd jitter   = host delta - hw-ts delta  (USB forwarding latency jitter)
//
// Run as root or: sudo chrt -f 80 ./rx_monitor

namespace {

std::atomic<bool> g_running{true};
void on_sigint(int) { g_running.store(false, std::memory_order_relaxed); }

// --- Channels -----------------------------------------------------------------
// UART columns are dense BoardTransmitter indices (port 0..4), not DataId
// values. mc02 maps those to silkscreen UART1 / UART2 / UART3 / UART7 / UART10.
enum Channel : uint8_t {
    kCan1,
    kCan2,
    kCan3,
    kUart0,
    kUart1,
    kUart2,
    kUart3,
    kUartPort4,
    kDbus,
    kChannelCount
};
static constexpr const char* kChannelName[kChannelCount] =
    {"CAN1", "CAN2", "CAN3", "UART0", "UART1", "UART2", "UART3", "UART4", "DBUS"};

// --- Lock-free SPSC log queue -------------------------------------------------
struct LogEntry {
    uint64_t timestamp_ns;
    uint64_t delta_ns;
    uint64_t hw_ts_delta_ns;       // delta between hw timestamps (ns)
    int64_t fwd_jitter_ns;         // host delta - hw_ts delta
    uint32_t size;
    uint32_t hw_timestamp_us;
    bool has_hw_ts;
    Channel channel;
};

constexpr size_t kLogQueueSize = 1 << 14;
constexpr size_t kLogQueueMask = kLogQueueSize - 1;

struct LogQueue {
    alignas(64) std::atomic<uint64_t> head{0};
    alignas(64) std::atomic<uint64_t> tail{0};
    LogEntry entries[kLogQueueSize];

    void push(LogEntry entry) {
        const uint64_t h = head.load(std::memory_order_relaxed);
        if (h - tail.load(std::memory_order_acquire) >= kLogQueueSize)
            return;
        entries[h & kLogQueueMask] = entry;
        head.store(h + 1, std::memory_order_release);
    }

    bool pop(LogEntry& out) {
        const uint64_t t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_acquire))
            return false;
        out = entries[t & kLogQueueMask];
        tail.store(t + 1, std::memory_order_release);
        return true;
    }
} g_log_queue;

std::chrono::steady_clock::time_point g_start;

// --- Per-channel stats --------------------------------------------------------
struct RxStat {
    std::atomic<uint64_t> frames{0};
    std::atomic<uint64_t> bytes{0};
    std::atomic<uint64_t> interval_min_ns{UINT64_MAX};
    std::atomic<uint64_t> interval_max_ns{0};
    std::atomic<uint64_t> interval_sum_ns{0};
    std::atomic<int64_t> jitter_abs_sum_ns{0};
    std::atomic<int64_t> jitter_max_ns{0};
    std::chrono::steady_clock::time_point last{std::chrono::steady_clock::time_point::min()};
    uint32_t last_hw_ts{0};
    bool has_last_hw_ts{false};

    void record(Channel ch, std::size_t size, uint32_t hw_ts = 0, bool has_hw_ts = false) {
        const auto now = std::chrono::steady_clock::now();
        frames.fetch_add(1, std::memory_order_relaxed);
        bytes.fetch_add(size, std::memory_order_relaxed);

        uint64_t host_delta_ns = 0;
        uint64_t hw_delta_ns = 0;
        int64_t fwd_jitter_ns = 0;

        if (last != std::chrono::steady_clock::time_point::min()) {
            host_delta_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(now - last).count());
            interval_sum_ns.fetch_add(host_delta_ns, std::memory_order_relaxed);
            uint64_t cur = interval_min_ns.load(std::memory_order_relaxed);
            while (host_delta_ns < cur
                   && !interval_min_ns.compare_exchange_weak(
                       cur, host_delta_ns, std::memory_order_relaxed, std::memory_order_relaxed))
                ;
            cur = interval_max_ns.load(std::memory_order_relaxed);
            while (host_delta_ns > cur
                   && !interval_max_ns.compare_exchange_weak(
                       cur, host_delta_ns, std::memory_order_relaxed, std::memory_order_relaxed))
                ;

            if (has_hw_ts && has_last_hw_ts) {
                // hw_ts runs at 1 us/tick on the CAN peripheral
                hw_delta_ns = static_cast<uint64_t>(hw_ts - last_hw_ts) * 1000UL;
                // Forwarding jitter: positive = host later than hw, negative = host earlier
                fwd_jitter_ns = static_cast<int64_t>(host_delta_ns)
                              - static_cast<int64_t>(hw_delta_ns);
                auto abs_jitter = fwd_jitter_ns < 0 ? -fwd_jitter_ns : fwd_jitter_ns;
                jitter_abs_sum_ns.fetch_add(abs_jitter, std::memory_order_relaxed);
                int64_t jc = jitter_max_ns.load(std::memory_order_relaxed);
                while (abs_jitter > jc
                       && !jitter_max_ns.compare_exchange_weak(
                           jc, abs_jitter, std::memory_order_relaxed, std::memory_order_relaxed))
                    ;
            }
        }
        last = now;
        if (has_hw_ts) {
            last_hw_ts = hw_ts;
            has_last_hw_ts = true;
        }

        const uint64_t ts = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now - g_start).count());
        g_log_queue.push({.timestamp_ns = ts, .delta_ns = host_delta_ns,
                          .hw_ts_delta_ns = hw_delta_ns, .fwd_jitter_ns = fwd_jitter_ns,
                          .size = static_cast<uint32_t>(size),
                          .hw_timestamp_us = hw_ts, .has_hw_ts = has_hw_ts,
                          .channel = ch});
    }

    struct Snapshot {
        uint64_t frames, bytes, min_ns, max_ns, sum_ns;
        int64_t jitter_abs_sum_ns, jitter_max_ns;
    };
    Snapshot take() {
        return {
            .frames  = frames.exchange(0, std::memory_order_relaxed),
            .bytes   = bytes.exchange(0, std::memory_order_relaxed),
            .min_ns  = interval_min_ns.exchange(UINT64_MAX, std::memory_order_relaxed),
            .max_ns  = interval_max_ns.exchange(0, std::memory_order_relaxed),
            .sum_ns  = interval_sum_ns.exchange(0, std::memory_order_relaxed),
            .jitter_abs_sum_ns = jitter_abs_sum_ns.exchange(0, std::memory_order_relaxed),
            .jitter_max_ns = jitter_max_ns.exchange(0, std::memory_order_relaxed),
        };
    }
};

RxStat g_stats[kChannelCount];

void print_summary(Channel ch, const RxStat::Snapshot& s, double dt) {
    if (s.frames == 0)
        return;
    const double avg_us = s.frames > 1
        ? static_cast<double>(s.sum_ns) / static_cast<double>(s.frames - 1) / 1e3 : 0.0;
    const double min_us = s.min_ns == UINT64_MAX ? 0.0 : static_cast<double>(s.min_ns) / 1e3;
    const double max_us = static_cast<double>(s.max_ns) / 1e3;
    const double avg_jitter_us = s.frames > 1
        ? static_cast<double>(s.jitter_abs_sum_ns) / static_cast<double>(s.frames - 1) / 1e3
        : 0.0;
    const double max_jitter_us = static_cast<double>(s.jitter_max_ns) / 1e3;
    printf("  %-5s  %7.0f frame/s  %8.0f B/s  |  host delta us  min/avg/max = "
           "%6.1f/%6.1f/%6.1f  |  fwd jitter us  avg/max = %5.1f/%5.1f\n",
        kChannelName[ch], static_cast<double>(s.frames) / dt,
        static_cast<double>(s.bytes) / dt, min_us, avg_us, max_us,
        avg_jitter_us, max_jitter_us);
}

// --- RT / non-RT thread isolation ---------------------------------------------
// Only the data-collection thread runs SCHED_FIFO. The print thread is left at
// SCHED_OTHER so heavy terminal I/O never starves real-time USB event handling.

void setup_realtime() {
    if (kRtCpu >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(kRtCpu, &cpuset);
        if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0)
            printf("Warning: failed to pin to CPU %d\n", kRtCpu);
        else
            printf("RT: pinned to CPU %d\n", kRtCpu);
    }
    struct sched_param param{};
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
        printf("Warning: failed to set SCHED_FIFO (run as root or grant CAP_SYS_NICE)\n");
    else
        printf("RT: SCHED_FIFO priority 80\n");
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        printf("Warning: mlockall failed\n");
}

void set_non_rt(const char* name) {
    // Demote this thread to SCHED_OTHER for non-real-time terminal I/O.
    struct sched_param param{};
    param.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &param) != 0)
        printf("Warning: %s: failed to set SCHED_OTHER\n", name);
}

// --- Board-neutral monitor ----------------------------------------------------
// One receiver for every board; connect_any() routes the connected board's CAN /
// UART / DBUS traffic here with 0-based bus/port indices.
class Monitor : public examples::BoardReceiver {
private:
    void on_can(int bus, const libhcs::data::CanDataView& data) override {
        if (bus < 0 || bus > 2)
            return;
        const auto ch = static_cast<Channel>(kCan1 + bus);
        g_stats[ch].record(ch, data.can_data.size(), data.timestamp_us.value_or(0),
            data.timestamp_us.has_value());
    }
    void on_uart(int port, const libhcs::data::UartDataView& data) override {
        if (port < 0 || port > 4)
            return;
        const auto ch = static_cast<Channel>(kUart0 + port);
        g_stats[ch].record(ch, data.uart_data.size());
    }
    void on_dbus(const libhcs::data::UartDataView& data) override {
        g_stats[kDbus].record(kDbus, data.uart_data.size());
    }
};

} // namespace

int main() {
    std::signal(SIGINT, on_sigint);

    // Only the main thread (USB event loop) runs with RT priority.
    // The print thread is demoted to SCHED_OTHER inside the thread function.
    setup_realtime();

    g_start = std::chrono::steady_clock::now();

    printf("RX monitor - auto-detecting board...\n");

    Monitor monitor;
    auto board = examples::connect_any(monitor);
    if (!board) {
        fprintf(stderr, "No compatible device found.\n");
        return 1;
    }
    printf("Connected: %.*s (%d CAN, %d UART%s)\n",
        static_cast<int>(board->name().size()), board->name().data(),
        board->can_bus_count(), board->uart_port_count(),
        board->has_dbus() ? " + DBUS" : "");

    printf("Listening. Ctrl-C to stop.\n\n");
    printf("%-13s  %-5s  %5s  %9s  %12s  %10s  %12s\n",
           "time(ms)", "ch", "bytes", "host_d(us)", "hw_ts(us)",
           "hw_d(us)", "fwd_jit(us)");
    printf("%-13s  %-5s  %5s  %9s  %12s  %10s  %12s\n",
           "--------", "--", "-----", "---------", "----------",
           "--------", "-----------");

    std::thread print_thread([] {
        // Print thread: SCHED_OTHER — terminal I/O is inherently non-real-time.
        set_non_rt("print");
        LogEntry e;
        while (g_running.load(std::memory_order_relaxed)) {
            while (g_log_queue.pop(e)) {
                const double t_ms = static_cast<double>(e.timestamp_ns) / 1e6;
                const double host_d_us = static_cast<double>(e.delta_ns) / 1e3;
                char hw_buf[16], hw_d_buf[16], jitter_buf[16];
                if (e.has_hw_ts) {
                    snprintf(hw_buf, sizeof(hw_buf), "%12u", e.hw_timestamp_us);
                    if (e.hw_ts_delta_ns)
                        snprintf(hw_d_buf, sizeof(hw_d_buf), "%10.1f",
                                 static_cast<double>(e.hw_ts_delta_ns) / 1e3);
                    else
                        snprintf(hw_d_buf, sizeof(hw_d_buf), "%10s", "(first)");
                    if (e.delta_ns)
                        snprintf(jitter_buf, sizeof(jitter_buf), "%12.1f",
                                 static_cast<double>(e.fwd_jitter_ns) / 1e3);
                    else
                        snprintf(jitter_buf, sizeof(jitter_buf), "%12s", "(first)");
                } else {
                    snprintf(hw_buf, sizeof(hw_buf), "%12s", "-");
                    snprintf(hw_d_buf, sizeof(hw_d_buf), "%10s", "-");
                    snprintf(jitter_buf, sizeof(jitter_buf), "%12s", "-");
                }
                printf("%13.3f  %-5s  %5u  %9.1f  %12s  %10s  %12s\n",
                    t_ms, kChannelName[e.channel], e.size, host_d_us,
                    hw_buf, hw_d_buf, jitter_buf);
            }
            std::this_thread::sleep_for(std::chrono::microseconds{500});
        }
        // Drain remaining entries on shutdown
        while (g_log_queue.pop(e)) {
            const double t_ms = static_cast<double>(e.timestamp_ns) / 1e6;
            const double host_d_us = static_cast<double>(e.delta_ns) / 1e3;
            char hw_buf[16], hw_d_buf[16], jitter_buf[16];
            if (e.has_hw_ts) {
                snprintf(hw_buf, sizeof(hw_buf), "%12u", e.hw_timestamp_us);
                if (e.hw_ts_delta_ns)
                    snprintf(hw_d_buf, sizeof(hw_d_buf), "%10.1f",
                             static_cast<double>(e.hw_ts_delta_ns) / 1e3);
                else
                    snprintf(hw_d_buf, sizeof(hw_d_buf), "%10s", "(first)");
                if (e.delta_ns)
                    snprintf(jitter_buf, sizeof(jitter_buf), "%12.1f",
                             static_cast<double>(e.fwd_jitter_ns) / 1e3);
                else
                    snprintf(jitter_buf, sizeof(jitter_buf), "%12s", "(first)");
            } else {
                snprintf(hw_buf, sizeof(hw_buf), "%12s", "-");
                snprintf(hw_d_buf, sizeof(hw_d_buf), "%10s", "-");
                snprintf(jitter_buf, sizeof(jitter_buf), "%12s", "-");
            }
            printf("%13.3f  %-5s  %5u  %9.1f  %12s  %10s  %12s\n",
                t_ms, kChannelName[e.channel], e.size, host_d_us,
                hw_buf, hw_d_buf, jitter_buf);
        }
    });

    auto last_report = std::chrono::steady_clock::now();
    while (g_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
        const auto now = std::chrono::steady_clock::now();
        if (now - last_report < std::chrono::seconds{5})
            continue;
        const double dt = std::chrono::duration<double>(now - last_report).count();
        last_report = now;
        printf("\n--- 5s summary ---\n");
        for (int i = 0; i < kChannelCount; ++i)
            print_summary(static_cast<Channel>(i), g_stats[i].take(), dt);
        printf("------------------\n\n");
    }

    print_thread.join();
    printf("\nStopped.\n");
    return 0;
}
