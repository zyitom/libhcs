// CAN-FD loopback stress test for the HPM6E8Y board over USB (PID 0x6E84) --
// the USB-transport twin of ecat_canfd_stress. Same wiring and same checks, so
// the two can be compared directly to see how the transport (USB vendor bulk
// vs EtherCAT ARQ stream) affects CAN loopback loss/throughput.
//
// WIRING: two independent loopback pairs, each a single CAN bus with a 120 ohm
// terminator at EACH end:
//   pair 0/1 : CAN0_H<->CAN1_H, CAN0_L<->CAN1_L
//   pair 2/3 : CAN2_H<->CAN3_H, CAN2_L<->CAN3_L
// The host floods CAN-FD frames out CAN0 and CAN2; the partner (CAN1 / CAN3)
// receives, and the firmware forwards every received frame back to the host,
// which checks it.
//
// Each frame's payload is 8 bytes: a 32-bit little-endian sequence number plus a
// 32-bit hash of it. The receiver validates the hash (corruption) and the
// sequence continuity (loss / reordering), all in host time.
//
// Build:  cmake --preset linux-debug -S host   (no EtherCAT backend needed)
// Run:    sudo ./usb_canfd_stress [seconds] [rate_fps_per_stream] [serial_filter]
//         (for steadier pacing:  sudo chrt -f 80 ./usb_canfd_stress 30 8000)

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string_view>
#include <thread>

#include <libhcs/board/hpm6e8y.hpp>


// CAN ports are named as the silkscreen labels them. On this board that is
// CAN0..CAN3. See libhcs/board/hcs_can_port.hpp.
using libhcs::board::hcs::CanPort;

namespace {

// Distinct standard IDs per source so a receiver can attribute frames and a
// miswire (frame arriving on the wrong bus) is visible.
constexpr uint32_t kCanId0 = 0x200; // sent on CAN0, expected on CAN1
constexpr uint32_t kCanId2 = 0x202; // sent on CAN2, expected on CAN3
constexpr size_t kPayloadBytes = 8;
constexpr size_t kCorruptionEvidenceCount = 8;
constexpr auto kDrainTimeout = std::chrono::seconds{2};

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true, std::memory_order_relaxed); }

// A cheap 32-bit avalanche mix, so a single flipped payload bit is caught.
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

// Encodes seq + hash(seq) into an 8-byte frame payload.
void encode(std::byte* out, uint32_t seq) {
    put_u32_le(out, seq);
    put_u32_le(out + 4, mix(seq));
}

enum class CorruptionKind : uint8_t {
    kLength,
    kHeader,
    kHash,
};

struct CorruptionEvidence {
    std::atomic<bool> ready{false};
    CorruptionKind kind = CorruptionKind::kLength;
    size_t length = 0;
    uint32_t can_id = 0;
    bool is_fdcan = false;
    bool is_extended_can_id = false;
    bool is_remote_transmission = false;
    std::array<std::byte, kPayloadBytes> payload{};
};

// Per loopback pair. Counters are read live by the main thread; the sequence
// state is touched only from the single USB receive thread. Evidence is
// published only after its contents are ready.
struct Stream {
    std::atomic<uint64_t> tx{0};
    std::atomic<uint64_t> rx{0};
    std::atomic<uint64_t> lost{0};    // forward gaps in the sequence
    std::atomic<uint64_t> reorder{0}; // a seq at or below one already seen
    std::atomic<uint64_t> corrupt{0}; // bad length / not FD / hash mismatch
    std::atomic<uint64_t> bad_length{0};
    std::atomic<uint64_t> bad_header{0};
    std::atomic<uint64_t> bad_hash{0};
    std::atomic<size_t> evidence_total{0};
    std::array<CorruptionEvidence, kCorruptionEvidenceCount> evidence{};

    bool started = false;
    uint32_t next_expected = 0;

    void record_corruption(CorruptionKind kind, const libhcs::data::CanDataView& data) {
        corrupt.fetch_add(1, std::memory_order_relaxed);
        const size_t index = evidence_total.fetch_add(1, std::memory_order_relaxed);
        if (index >= evidence.size())
            return;

        CorruptionEvidence& item = evidence[index];
        item.kind = kind;
        item.length = data.can_data.size();
        item.can_id = data.can_id;
        item.is_fdcan = data.is_fdcan;
        item.is_extended_can_id = data.is_extended_can_id;
        item.is_remote_transmission = data.is_remote_transmission;
        const size_t copied = std::min(item.payload.size(), data.can_data.size());
        std::copy_n(data.can_data.begin(), copied, item.payload.begin());
        item.ready.store(true, std::memory_order_release);
    }

    void verify(const libhcs::data::CanDataView& data, uint32_t expected_can_id) {
        if (data.can_data.size() != kPayloadBytes) {
            bad_length.fetch_add(1, std::memory_order_relaxed);
            record_corruption(CorruptionKind::kLength, data);
            rx.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (!data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission
            || data.can_id != expected_can_id) {
            bad_header.fetch_add(1, std::memory_order_relaxed);
            record_corruption(CorruptionKind::kHeader, data);
            rx.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        const std::byte* p = data.can_data.data();
        const uint32_t seq = get_u32_le(p);
        if (get_u32_le(p + 4) != mix(seq)) {
            bad_hash.fetch_add(1, std::memory_order_relaxed);
            record_corruption(CorruptionKind::kHash, data);
            rx.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        if (!started) {
            started = true;
            lost.fetch_add(seq, std::memory_order_relaxed);
            next_expected = seq + 1;
            rx.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (seq == next_expected) {
            next_expected = seq + 1;
        } else if (seq > next_expected) {
            lost.fetch_add(seq - next_expected, std::memory_order_relaxed);
            next_expected = seq + 1;
        } else {
            reorder.fetch_add(1, std::memory_order_relaxed);
        }
        rx.fetch_add(1, std::memory_order_relaxed);
    }
};

class Receiver final : public libhcs::board::Hpm6e8y::Callback {
public:
    Stream pair01; // CAN0 -> CAN1
    Stream pair23; // CAN2 -> CAN3
    std::atomic<uint64_t> unexpected{0};

private:
    void can_receive(
        CanPort port, const libhcs::data::CanDataView& data) override {
        switch (port) {
        case CanPort::kCan1: pair01.verify(data, kCanId0); break;
        case CanPort::kCan3: pair23.verify(data, kCanId2); break;
        // The transmitters never receive their own frames on a one-way bus;
        // anything landing here signals a miswire or an unexpected bus partner.
        case CanPort::kCan0: {
        case CanPort::kCan2:
            unexpected.fetch_add(1, std::memory_order_relaxed);
            break;
        }
        default: break;
        }
    }
};

void print_line(const char* tag, const Stream& s, uint64_t rx_delta, double dt_s) {
    const double fps = dt_s > 0 ? static_cast<double>(rx_delta) / dt_s : 0.0;
    const uint64_t tx = s.tx.load();
    const uint64_t received = s.rx.load();
    const uint64_t missing = tx > received ? tx - received : 0;
    printf(
        "  %s  tx=%-10llu rx=%-10llu missing=%-8llu gap=%-8llu corrupt=%-6llu"
        " reorder=%-6llu  %8.0f f/s  %6.2f KiB/s\n",
        tag, static_cast<unsigned long long>(tx), static_cast<unsigned long long>(received),
        static_cast<unsigned long long>(missing),
        static_cast<unsigned long long>(s.lost.load()),
        static_cast<unsigned long long>(s.corrupt.load()),
        static_cast<unsigned long long>(s.reorder.load()), fps,
        fps * kPayloadBytes / 1024.0);
}

const char* corruption_kind_name(CorruptionKind kind) {
    switch (kind) {
    case CorruptionKind::kLength: return "length";
    case CorruptionKind::kHeader: return "header";
    case CorruptionKind::kHash: return "hash";
    }
    return "unknown";
}

void print_corruption_evidence(const char* tag, const Stream& stream) {
    if (stream.corrupt.load(std::memory_order_relaxed) == 0)
        return;

    printf(
        "  %s corruption classes: length=%llu header=%llu hash=%llu\n", tag,
        static_cast<unsigned long long>(stream.bad_length.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(stream.bad_header.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(stream.bad_hash.load(std::memory_order_relaxed)));

    const size_t count = std::min(
        stream.evidence_total.load(std::memory_order_relaxed), stream.evidence.size());
    for (size_t index = 0; index < count; ++index) {
        const CorruptionEvidence& item = stream.evidence[index];
        if (!item.ready.load(std::memory_order_acquire))
            continue;

        printf(
            "    bad[%zu] kind=%s len=%zu id=0x%08x fd=%u ext=%u rtr=%u payload=", index,
            corruption_kind_name(item.kind), item.length, item.can_id,
            static_cast<unsigned>(item.is_fdcan),
            static_cast<unsigned>(item.is_extended_can_id),
            static_cast<unsigned>(item.is_remote_transmission));
        const size_t payload_size = std::min(item.length, item.payload.size());
        for (size_t byte_index = 0; byte_index < payload_size; ++byte_index)
            printf("%02x", std::to_integer<unsigned>(item.payload[byte_index]));
        if (item.length == kPayloadBytes) {
            const uint32_t seq = get_u32_le(item.payload.data());
            printf(
                " seq=%u hash=0x%08x expected=0x%08x", seq,
                get_u32_le(item.payload.data() + 4), mix(seq));
        }
        putchar('\n');
    }
}

} // namespace

int main(int argc, char** argv) {
    const int duration_s = argc > 1 ? std::atoi(argv[1]) : 10;
    const uint32_t rate = argc > 2 ? static_cast<uint32_t>(std::atoi(argv[2])) : 5000;
    const std::string_view serial_filter = argc > 3 ? argv[3] : std::string_view{};
    if (duration_s <= 0 || rate == 0U || rate > 1'000'000U) {
        fprintf(stderr, "seconds must be positive and rate must be in [1, 1000000]\n");
        return 1;
    }

    std::signal(SIGINT, on_sigint);

    Receiver rx;
    try {
        // Session establishment happens inside the constructor; returning means
        // the USB device is open and the protocol handshake passed.
        libhcs::board::Hpm6e8y board{rx, serial_filter};
        printf("HPM6E8Y USB board connected, session established.\n");
        printf(
            "CAN-FD stress: CAN0->CAN1 and CAN2->CAN3, target %u f/s per stream for %d s "
            "(Ctrl-C to stop early)\n\n",
            rate, duration_s);

        using clock = std::chrono::steady_clock;
        const auto start = clock::now();
        const auto deadline = start + std::chrono::seconds{duration_s};
        const auto send_period = std::chrono::duration_cast<clock::duration>(
            std::chrono::nanoseconds{1'000'000'000U / rate});
        auto next_send = start;
        auto next_report = start + std::chrono::seconds{1};
        uint64_t last_rx01 = 0, last_rx23 = 0;
        auto last_report = start;

        uint64_t sent = 0;
        uint64_t missed_pacing_slots = 0;
        while (clock::now() < deadline && !g_stop.load(std::memory_order_relaxed)) {
            auto now = clock::now();
            const double elapsed_s = std::chrono::duration<double>(now - start).count();

            if (now >= next_send) {
                std::byte a[kPayloadBytes];
                std::byte b[kPayloadBytes];
                encode(a, static_cast<uint32_t>(sent));
                encode(b, static_cast<uint32_t>(sent));

                // One packet carries both pairs' frames. The builder API does
                // not surface transport enqueue failure; the final tx/rx
                // equality check catches a command lost at any layer.
                board.start_transmit()
                    .can_transmit(CanPort::kCan0,
                        {.can_id = kCanId0, .can_data = a, .is_fdcan = true})
                    .can_transmit(CanPort::kCan2,
                        {.can_id = kCanId2, .can_data = b, .is_fdcan = true});

                rx.pair01.tx.fetch_add(1, std::memory_order_relaxed);
                rx.pair23.tx.fetch_add(1, std::memory_order_relaxed);
                ++sent;

                next_send += send_period;
                now = clock::now();
                if (next_send <= now) {
                    const auto skipped =
                        static_cast<uint64_t>((now - next_send) / send_period) + 1U;
                    missed_pacing_slots += skipped;
                    next_send += send_period * skipped;
                }
            }

            if (now >= next_report) {
                const double dt = std::chrono::duration<double>(now - last_report).count();
                const uint64_t rx01 = rx.pair01.rx.load();
                const uint64_t rx23 = rx.pair23.rx.load();
                printf("[%5.1fs]\n", elapsed_s);
                print_line("01", rx.pair01, rx01 - last_rx01, dt);
                print_line("23", rx.pair23, rx23 - last_rx23, dt);
                last_rx01 = rx01;
                last_rx23 = rx23;
                last_report = now;
                next_report += std::chrono::seconds{1};
            }

            auto wake_time = std::min(next_send, next_report);
            wake_time = std::min(wake_time, deadline);
            if (wake_time > clock::now())
                std::this_thread::sleep_until(wake_time);
        }

        // Drain queued transport and CAN traffic, but do not hang forever when
        // the tail was genuinely dropped.
        const auto drain_deadline = clock::now() + kDrainTimeout;
        while (clock::now() < drain_deadline
               && (rx.pair01.rx.load() < rx.pair01.tx.load()
                   || rx.pair23.rx.load() < rx.pair23.tx.load())) {
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }

        printf("\n=== summary ===\n");
        print_line("01", rx.pair01, 0, 0);
        print_line("23", rx.pair23, 0, 0);
        print_corruption_evidence("01", rx.pair01);
        print_corruption_evidence("23", rx.pair23);
        printf("  missed pacing slots: %llu\n",
               static_cast<unsigned long long>(missed_pacing_slots));
        const uint64_t unexpected = rx.unexpected.load();
        if (unexpected != 0)
            printf("  WARNING: %llu frame(s) arrived on a transmitter's own bus (miswire?)\n",
                   static_cast<unsigned long long>(unexpected));

        const bool clean = rx.pair01.tx.load() > 0 && rx.pair23.tx.load() > 0
                        && rx.pair01.rx.load() == rx.pair01.tx.load()
                        && rx.pair23.rx.load() == rx.pair23.tx.load()
                        && rx.pair01.corrupt.load() == 0 && rx.pair23.corrupt.load() == 0
                        && rx.pair01.lost.load() == 0 && rx.pair23.lost.load() == 0
                        && rx.pair01.reorder.load() == 0 && rx.pair23.reorder.load() == 0
                        && unexpected == 0;
        printf("  result: %s\n", clean ? "PASS (no loss, no corruption)"
                                        : "CHECK counters above");
        return clean ? 0 : 2;
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }
}
