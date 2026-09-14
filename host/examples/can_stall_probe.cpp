// CAN forwarding stall probe for the HPM6E8Y single-core USB image.
//
// Drives the same CAN-FD loopback flood as usb_canfd_stress, but also decodes
// the telemetry that a libhcs_APP_CAN_DIAG firmware emits on the UART0 uplink,
// and prints a full before/after dump the moment forwarding stops. The point is
// to answer one question without a debugger: when the host stops receiving
// frames, has the CAN interrupt stopped arriving, or has the controller stopped
// receiving?
//
//   * ISR entry count frozen + RXF0S fill level non-zero + PLIC pending bit set
//     -> the interrupt was lost between the PLIC gateway and the core.
//   * ISR entry count frozen + RXF0S empty + PSR showing bus-off
//     -> the controller left the bus and did not come back.
//   * ISR entry count still climbing + frames count frozen
//     -> the drain loop or the serializer is the problem, not delivery.
//   * alloc_fail climbing -> the uplink batch pool is the bottleneck.
//
// WIRING: same as usb_canfd_stress -- CAN0<->CAN1 and CAN2<->CAN3, each pair a
// single bus with a 120 ohm terminator at each end.
//
// Build: cmake --preset linux-release -S host -DBUILD_EXAMPLES=ON
// Run:   sudo chrt -f 80 ./can_stall_probe [seconds] [rate_fps_per_stream]

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <mutex>
#include <string_view>
#include <thread>

#include <libhcs/board/hpm6e8y.hpp>


// CAN ports are named as the silkscreen labels them. On this board that is
// CAN0..CAN3. See libhcs/board/hcs_can_port.hpp.
using libhcs::board::hcs::CanPort;

namespace {

constexpr uint32_t kCanId0 = 0x200;
constexpr uint32_t kCanId2 = 0x202;
constexpr size_t kPayloadBytes = 8;

// Must match firmware/hpm_board/app/src/diag/can_diag.hpp.
constexpr uint8_t kRecordMagic = 0xD1U;
constexpr uint8_t kRecordVersion = 5U;
constexpr size_t kPlicWordCount = 6;
constexpr size_t kMaxCan = 4;

// How long forwarding may be silent, while the host is still transmitting,
// before this counts as a stall. The flood keeps every bus busy continuously, so
// a quarter second of nothing is already far outside normal behaviour.
constexpr auto kStallThreshold = std::chrono::milliseconds{250};
constexpr size_t kHistoryDepth = 12;

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true, std::memory_order_relaxed); }

uint32_t mix(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

void put_u32_le(std::byte* p, uint32_t v) {
    p[0] = static_cast<std::byte>(v);
    p[1] = static_cast<std::byte>(v >> 8);
    p[2] = static_cast<std::byte>(v >> 16);
    p[3] = static_cast<std::byte>(v >> 24);
}

uint32_t get_u32_le(const std::byte* p) {
    return static_cast<uint32_t>(std::to_integer<uint8_t>(p[0]))
         | static_cast<uint32_t>(std::to_integer<uint8_t>(p[1])) << 8
         | static_cast<uint32_t>(std::to_integer<uint8_t>(p[2])) << 16
         | static_cast<uint32_t>(std::to_integer<uint8_t>(p[3])) << 24;
}

void encode(std::byte* out, uint32_t seq) {
    put_u32_le(out, seq);
    put_u32_le(out + 4, mix(seq));
}

struct CanTelemetry {
    uint32_t isr_entries = 0;
    uint32_t frames = 0;
    uint32_t tx_fail = 0;
    uint32_t irq_recovered = 0;
    uint32_t ir = 0;
    uint32_t rxf0s = 0;
    uint32_t psr = 0;
    uint32_t ecr = 0;
    uint32_t txfqs = 0;
    // Bit-timing registers as actually programmed. Worth printing because the
    // SDK derives them from a requested baudrate plus a sample-point window, so
    // what the controller runs is not obvious from the source.
    uint32_t nbtp = 0;
    uint32_t dbtp = 0;
};

struct Telemetry {
    bool valid = false;
    uint8_t sequence = 0;
    uint8_t can_count = 0;
    uint32_t tick = 0;
    uint32_t alloc_fail = 0;
    uint32_t main_loop_iters = 0;
    std::array<uint32_t, kPlicWordCount> plic_pending{};
    std::array<uint32_t, kPlicWordCount> plic_enable{};
    std::array<uint32_t, kPlicWordCount> plic_trigger{};
    uint32_t plic_threshold = 0;
    std::array<CanTelemetry, kMaxCan> can{};
};

bool decode(std::span<const std::byte> payload, Telemetry& out) {
    if (payload.size() < 8)
        return false;
    if (std::to_integer<uint8_t>(payload[0]) != kRecordMagic)
        return false;
    if (std::to_integer<uint8_t>(payload[1]) != kRecordVersion)
        return false;

    out.sequence = std::to_integer<uint8_t>(payload[2]);
    out.can_count = std::to_integer<uint8_t>(payload[3]);
    if (out.can_count > kMaxCan)
        return false;

    const uint32_t declared = get_u32_le(payload.data() + 4);
    if (declared != payload.size())
        return false;

    const std::byte* cursor = payload.data() + 8;
    out.tick = get_u32_le(cursor);
    cursor += 4;
    out.alloc_fail = get_u32_le(cursor);
    cursor += 4;
    out.main_loop_iters = get_u32_le(cursor);
    cursor += 4;
    for (size_t i = 0; i < kPlicWordCount; i++, cursor += 4)
        out.plic_pending[i] = get_u32_le(cursor);
    for (size_t i = 0; i < kPlicWordCount; i++, cursor += 4)
        out.plic_enable[i] = get_u32_le(cursor);
    for (size_t i = 0; i < kPlicWordCount; i++, cursor += 4)
        out.plic_trigger[i] = get_u32_le(cursor);
    out.plic_threshold = get_u32_le(cursor);
    cursor += 4;

    for (size_t i = 0; i < out.can_count; i++) {
        CanTelemetry& can = out.can[i];
        can.isr_entries = get_u32_le(cursor);
        cursor += 4;
        can.frames = get_u32_le(cursor);
        cursor += 4;
        can.tx_fail = get_u32_le(cursor);
        cursor += 4;
        can.irq_recovered = get_u32_le(cursor);
        cursor += 4;
        can.ir = get_u32_le(cursor);
        cursor += 4;
        can.rxf0s = get_u32_le(cursor);
        cursor += 4;
        can.psr = get_u32_le(cursor);
        cursor += 4;
        can.ecr = get_u32_le(cursor);
        cursor += 4;
        can.txfqs = get_u32_le(cursor);
        cursor += 4;
        can.nbtp = get_u32_le(cursor);
        cursor += 4;
        can.dbtp = get_u32_le(cursor);
        cursor += 4;
    }

    out.valid = true;
    return true;
}

// M_CAN PSR.LEC / PSR.ACT decoding, enough to name the states that matter.
const char* activity_name(uint32_t psr) {
    switch ((psr >> 3) & 0x3U) {
    case 0: return "sync";
    case 1: return "idle";
    case 2: return "rx";
    default: return "tx";
    }
}

const char* last_error_name(uint32_t lec) {
    switch (lec) {
    case 0: return "none";
    case 1: return "stuff";
    case 2: return "form";
    case 3: return "ack";
    case 4: return "bit1";
    case 5: return "bit0";
    case 6: return "crc";
    default: return "no-change";
    }
}

void print_telemetry(const Telemetry& t) {
    if (!t.valid) {
        printf("    (no telemetry -- is the firmware built with libhcs_CAN_DIAG=ON?)\n");
        return;
    }
    printf(
        "    seq=%-3u tick=%-8u alloc_fail=%-6u loop=%u/100ms (%.2f us/iter)\n", t.sequence,
        t.tick, t.alloc_fail, t.main_loop_iters,
        t.main_loop_iters ? 100000.0 / t.main_loop_iters : 0.0);
    // MCAN0..3 are sources 90..93 (word 2), USB0 is 127 (word 3).
    printf(
        "    plic pending[2]=0x%08x enable[2]=0x%08x trigger[2]=0x%08x"
        "  (MCAN0..3 = bits 26..29; trigger 1=edge 0=level)\n",
        t.plic_pending[2], t.plic_enable[2], t.plic_trigger[2]);
    printf(
        "    plic pending[3]=0x%08x enable[3]=0x%08x  (USB0 = bit 31)\n", t.plic_pending[3],
        t.plic_enable[3]);
    for (size_t i = 0; i < t.can_count; i++) {
        const CanTelemetry& c = t.can[i];
        const uint32_t rx_fill = c.rxf0s & 0x7fU;
        const bool rx_full = (c.rxf0s & (1U << 24)) != 0;
        const bool rx_lost = (c.rxf0s & (1U << 25)) != 0;
        const bool bus_off = (c.psr & (1U << 7)) != 0;
        const bool err_passive = (c.psr & (1U << 5)) != 0;
        const bool warning = (c.psr & (1U << 6)) != 0;
        printf(
            "    can%zu isr=%-10u frames=%-10u txfail=%-6u recovered=%-4u ir=0x%08x\n", i,
            c.isr_entries, c.frames, c.tx_fail, c.irq_recovered, c.ir);
        printf(
            "         rxf0 fill=%-3u full=%u lost=%u | psr act=%s lec=%s dlec=%s"
            " busoff=%u ep=%u ew=%u | ecr tx=%u rx=%u | txfqs=0x%08x\n",
            rx_fill, static_cast<unsigned>(rx_full), static_cast<unsigned>(rx_lost),
            activity_name(c.psr), last_error_name(c.psr & 0x7U),
            last_error_name((c.psr >> 8) & 0x7U), static_cast<unsigned>(bus_off),
            static_cast<unsigned>(err_passive), static_cast<unsigned>(warning), c.ecr & 0xffU,
            (c.ecr >> 8) & 0x7fU, c.txfqs);
        // Decoded bit timing. A data-phase sample point that differs from the
        // peer's is invisible in every other counter here but breaks CAN-FD
        // outright (see the 75% vs 87.5% case fixed in can.hpp).
        const uint32_t nbrp = ((c.nbtp >> 16) & 0x1ffU) + 1U;
        const uint32_t ntseg1 = ((c.nbtp >> 8) & 0xffU) + 1U;
        const uint32_t ntseg2 = (c.nbtp & 0x7fU) + 1U;
        const uint32_t dbrp = ((c.dbtp >> 16) & 0x1fU) + 1U;
        const uint32_t dtseg1 = ((c.dbtp >> 8) & 0x1fU) + 1U;
        const uint32_t dtseg2 = ((c.dbtp >> 4) & 0xfU) + 1U;
        printf(
            "      timing: nominal brp=%u tseg1=%u tseg2=%u sp=%.1f%% | data brp=%u tseg1=%u "
            "tseg2=%u sp=%.1f%% tdc=%u\n",
            nbrp, ntseg1, ntseg2, 100.0 * (1 + ntseg1) / (1 + ntseg1 + ntseg2), dbrp, dtseg1,
            dtseg2, 100.0 * (1 + dtseg1) / (1 + dtseg1 + dtseg2), (c.dbtp >> 23) & 1U);
    }
}

struct Stream {
    std::atomic<uint64_t> tx{0};
    std::atomic<uint64_t> rx{0};
    std::atomic<uint64_t> corrupt{0};
    std::atomic<uint64_t> lost{0};
    bool started = false;
    uint32_t next_expected = 0;

    void verify(const libhcs::data::CanDataView& data, uint32_t expected_can_id) {
        rx.fetch_add(1, std::memory_order_relaxed);
        if (data.can_data.size() != kPayloadBytes || data.can_id != expected_can_id) {
            corrupt.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        const std::byte* p = data.can_data.data();
        const uint32_t seq = get_u32_le(p);
        if (get_u32_le(p + 4) != mix(seq)) {
            corrupt.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (!started) {
            started = true;
        } else if (seq > next_expected) {
            lost.fetch_add(seq - next_expected, std::memory_order_relaxed);
        }
        next_expected = seq + 1;
    }
};

class Receiver final : public libhcs::board::Hpm6e8y::Callback {
public:
    Stream pair01;
    Stream pair23;

    Telemetry latest() const {
        const std::scoped_lock guard{mutex_};
        return history_.empty() ? Telemetry{} : history_.back();
    }

    std::deque<Telemetry> history() const {
        const std::scoped_lock guard{mutex_};
        return history_;
    }

    uint64_t telemetry_count() const { return telemetry_count_.load(std::memory_order_relaxed); }

private:
    void can_receive(
        CanPort port, const libhcs::data::CanDataView& data) override {
        switch (port) {
        case CanPort::kCan1: {
            pair01.verify(data, kCanId0);
            break;
        }
        case CanPort::kCan3: {
            pair23.verify(data, kCanId2);
            break;
        }
        default: break;
        }
    }

    void uart0_receive_callback(const libhcs::data::UartDataView& data) override {
        Telemetry decoded;
        if (!decode(data.uart_data, decoded))
            return;
        telemetry_count_.fetch_add(1, std::memory_order_relaxed);
        const std::scoped_lock guard{mutex_};
        history_.push_back(decoded);
        while (history_.size() > kHistoryDepth)
            history_.pop_front();
    }

    mutable std::mutex mutex_;
    std::deque<Telemetry> history_;
    std::atomic<uint64_t> telemetry_count_{0};
};

} // namespace

int main(int argc, char** argv) {
    const int duration_s = argc > 1 ? std::atoi(argv[1]) : 20;
    const uint32_t rate = argc > 2 ? static_cast<uint32_t>(std::atoi(argv[2])) : 14000;
    const std::string_view serial_filter = argc > 3 ? argv[3] : std::string_view{};
    if (duration_s <= 0 || rate == 0U || rate > 1'000'000U) {
        fprintf(stderr, "seconds must be positive and rate must be in [1, 1000000]\n");
        return 1;
    }

    std::signal(SIGINT, on_sigint);

    Receiver rx;
    try {
        libhcs::board::Hpm6e8y board{rx, serial_filter};
        printf("HPM6E8Y USB board connected, session established.\n");
        printf("CAN stall probe: %u f/s per stream for %d s\n\n", rate, duration_s);

        using clock = std::chrono::steady_clock;
        const auto start = clock::now();
        const auto deadline = start + std::chrono::seconds{duration_s};
        const auto send_period = std::chrono::duration_cast<clock::duration>(
            std::chrono::nanoseconds{1'000'000'000U / rate});
        auto next_send = start;
        auto next_report = start + std::chrono::seconds{1};
        auto last_progress = start;
        uint64_t last_rx01 = 0;
        uint64_t last_rx23 = 0;
        uint64_t sent = 0;
        bool stall_reported = false;

        while (clock::now() < deadline && !g_stop.load(std::memory_order_relaxed)) {
            auto now = clock::now();

            if (now >= next_send) {
                std::byte a[kPayloadBytes];
                std::byte b[kPayloadBytes];
                encode(a, static_cast<uint32_t>(sent));
                encode(b, static_cast<uint32_t>(sent));
                board.start_transmit()
                    .can_transmit(
                        CanPort::kCan0, {.can_id = kCanId0, .can_data = a})
                    .can_transmit(
                        CanPort::kCan2, {.can_id = kCanId2, .can_data = b});
                rx.pair01.tx.fetch_add(1, std::memory_order_relaxed);
                rx.pair23.tx.fetch_add(1, std::memory_order_relaxed);
                ++sent;

                next_send += send_period;
                now = clock::now();
                if (next_send <= now)
                    next_send += send_period * ((now - next_send) / send_period + 1);
            }

            // Tracked per stream, not on the sum: the failure mode is one
            // controller going deaf while the other keeps forwarding, and a sum
            // hides exactly that.
            const uint64_t rx01 = rx.pair01.rx.load();
            const uint64_t rx23 = rx.pair23.rx.load();
            const uint64_t rx_total = rx01 + rx23;
            if (rx01 != last_rx01 && rx23 != last_rx23) {
                last_rx01 = rx01;
                last_rx23 = rx23;
                last_progress = now;
            } else if (!stall_reported && rx_total > 0 && now - last_progress > kStallThreshold) {
                stall_reported = true;
                const auto silent_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_progress);
                printf(
                    "\n*** FORWARDING STALLED: no frame for %lld ms at %.1f s "
                    "(tx keeps running) ***\n",
                    static_cast<long long>(silent_ms.count()),
                    std::chrono::duration<double>(now - start).count());
                printf("--- telemetry history, oldest first ---\n");
                const auto snapshots = rx.history();
                for (const Telemetry& snapshot : snapshots) {
                    printf("  [record %u]\n", snapshot.sequence);
                    print_telemetry(snapshot);
                }
                printf("--- end history ---\n\n");
                fflush(stdout);
            }

            if (now >= next_report) {
                printf(
                    "[%5.1fs] tx=%llu rx01=%llu rx23=%llu lost=%llu/%llu corrupt=%llu/%llu"
                    " telemetry=%llu\n",
                    std::chrono::duration<double>(now - start).count(),
                    static_cast<unsigned long long>(sent),
                    static_cast<unsigned long long>(rx.pair01.rx.load()),
                    static_cast<unsigned long long>(rx.pair23.rx.load()),
                    static_cast<unsigned long long>(rx.pair01.lost.load()),
                    static_cast<unsigned long long>(rx.pair23.lost.load()),
                    static_cast<unsigned long long>(rx.pair01.corrupt.load()),
                    static_cast<unsigned long long>(rx.pair23.corrupt.load()),
                    static_cast<unsigned long long>(rx.telemetry_count()));
                print_telemetry(rx.latest());
                next_report += std::chrono::seconds{1};
            }

            auto wake_time = std::min(next_send, next_report);
            wake_time = std::min(wake_time, deadline);
            if (wake_time > clock::now())
                std::this_thread::sleep_until(wake_time);
        }

        printf("\n=== final telemetry ===\n");
        print_telemetry(rx.latest());
        printf(
            "  tx=%llu rx01=%llu rx23=%llu  stall_detected=%s\n",
            static_cast<unsigned long long>(sent),
            static_cast<unsigned long long>(rx.pair01.rx.load()),
            static_cast<unsigned long long>(rx.pair23.rx.load()), stall_reported ? "YES" : "no");
        return stall_reported ? 2 : 0;
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }
}
