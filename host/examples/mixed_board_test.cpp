// Heterogeneous two-board suite: any two project boards wired to each other.
//
// Built for the mc02 <-> hpm5321_dual_can rig, where the two ends differ in
// almost every way that matters -- different MCU family (STM32H7 vs HPM5321),
// different CAN peripheral (bxCAN/FDCAN vs MCAN), and, most importantly,
// **different USB speeds**: mc02 enumerates at Full Speed (12 Mbit/s, 1 ms
// frames) while hpm5321 is High Speed (480 Mbit/s, 125 us microframes). The
// USB frame rate is what caps packets per second, so the two ends are expected
// to have very different transmit ceilings, and this tool measures that
// directly instead of assuming it.
//
// Unlike dual_board_test (which hard-codes one board type), this one goes
// through examples::BoardSession, so the pair can be any two boards.
//
// Wiring is discovered, not assumed: `link` sweeps every CAN bus of one board
// against every CAN bus of the other and reports which pairs are actually
// connected.
//
// Run:
//   ./mixed_board_test list
//   ./mixed_board_test link
//   ./mixed_board_test latency <tx_bus> <rx_bus> [samples]
//   ./mixed_board_test rate    <tx_bus> <rx_bus> [seconds] [frames_per_packet]
//   ./mixed_board_test uart    [baud=115200] [rounds=20]
//
// HCS_BOARD_A / HCS_BOARD_B select the two boards by USB serial; without them
// the first two project boards found are used, in sysfs order.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <vector>

#include <dirent.h>

#include "common/multi_board.hpp"

namespace {

using Clock = std::chrono::steady_clock;

constexpr uint32_t kCanIdBase = 0x570;
constexpr size_t kPayloadSize = 8;
constexpr uint32_t kWarmupSamples = 100;
// Rate `link` puts every port at before probing, so a port left at some other
// rate by an earlier test cannot make the wiring look broken. 115200 is every
// board's compile-time default except the 5321s, and every board can reach it.
constexpr uint32_t kLinkProbeBaudrate = 115200;

std::string g_serial_a;
std::string g_serial_b;

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

// Decoder for libhcs_CAN_DIAG telemetry (hpm_board only), carried on the
// UART0 uplink. Only tx_fail is needed here: it counts frames the board was
// asked to send but could not hand to the MCAN TX FIFO, i.e. silent drops.
struct DiagRecord {
    bool valid = false;
    uint32_t tx_fail[2]{};
    // MCAN Protocol Status / Error Counter registers. LEC (PSR bits 2:0) names
    // the last error the controller saw, which is what distinguishes "nobody
    // acknowledged my frame" from "the bits came back wrong".
    uint32_t psr[2]{};
    uint32_t ecr[2]{};
    uint32_t nbtp[2]{};
    uint32_t dbtp[2]{};
    uint32_t uart_clock = 0;
    uint32_t uart_oscr = 0;
    uint32_t uart_div = 0;
};

bool decode_diag(std::span<const std::byte> payload, DiagRecord& out);

double percentile(const std::vector<double>& sorted, double fraction) {
    if (sorted.empty())
        return 0.0;
    return sorted[static_cast<size_t>(fraction * static_cast<double>(sorted.size() - 1))];
}

// USB descriptor facts read from sysfs. `speed` is the discriminator this tool
// exists to expose: 12 => Full Speed (1 ms frames), 480 => High Speed (125 us
// microframes), and that ratio is the expected ratio of packet-rate ceilings.
struct BoardInfo {
    std::string serial;
    std::string product;
    std::string speed;
    std::string pid;
};

std::string read_sysfs(const std::string& path) {
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
}

std::vector<BoardInfo> enumerate_boards() {
    std::vector<BoardInfo> found;
    DIR* dir = opendir("/sys/bus/usb/devices");
    if (!dir)
        return found;
    while (const dirent* entry = readdir(dir)) {
        const std::string base = std::string{"/sys/bus/usb/devices/"} + entry->d_name;
        if (read_sysfs(base + "/idVendor") != "a511")
            continue;
        BoardInfo info;
        info.serial = read_sysfs(base + "/serial");
        if (info.serial.empty())
            continue;
        info.product = read_sysfs(base + "/product");
        info.speed = read_sysfs(base + "/speed");
        info.pid = read_sysfs(base + "/idProduct");
        found.push_back(std::move(info));
    }
    closedir(dir);
    std::sort(found.begin(), found.end(), [](const BoardInfo& a, const BoardInfo& b) {
        return a.serial < b.serial;
    });
    return found;
}

const char* usb_speed_name(const std::string& speed) {
    if (speed == "12")
        return "Full Speed (1 ms frames)";
    if (speed == "480")
        return "High Speed (125 us microframes)";
    if (speed == "5000" || speed == "10000")
        return "SuperSpeed";
    return "unknown";
}

bool decode_diag(std::span<const std::byte> payload, DiagRecord& out) {
    constexpr uint8_t kRecordMagic = 0xD1U;
    constexpr uint8_t kRecordVersion = 5U;
    constexpr size_t kPlicWordCount = 6;
    constexpr size_t kPerCanOffset = 8 + 4 + 4 + 4 + 4 * (3 * kPlicWordCount) + 4;
    constexpr size_t kPerCanSize = 11 * 4;

    if (payload.size() < kPerCanOffset)
        return false;
    if (std::to_integer<uint8_t>(payload[0]) != kRecordMagic
        || std::to_integer<uint8_t>(payload[1]) != kRecordVersion)
        return false;
    const uint8_t can_count = std::to_integer<uint8_t>(payload[3]);
    if (get_u32_le(payload.data() + 4) != payload.size()
        || payload.size() < kPerCanOffset + can_count * kPerCanSize)
        return false;
    for (uint8_t i = 0; i < can_count && i < 2; ++i) {
        const std::byte* block = payload.data() + kPerCanOffset + i * kPerCanSize;
        out.tx_fail[i] = get_u32_le(block + 8);
        out.psr[i] = get_u32_le(block + 24);
        out.ecr[i] = get_u32_le(block + 28);
        out.nbtp[i] = get_u32_le(block + 36);
        out.dbtp[i] = get_u32_le(block + 40);
    }
    // Optional UART block appended after the per-CAN blocks (record version 5).
    const size_t uart_offset = kPerCanOffset + can_count * kPerCanSize;
    if (payload.size() >= uart_offset + 12) {
        out.uart_clock = get_u32_le(payload.data() + uart_offset);
        out.uart_oscr = get_u32_le(payload.data() + uart_offset + 4);
        out.uart_div = get_u32_le(payload.data() + uart_offset + 8);
    }
    out.valid = true;
    return true;
}

int run_list() {
    const auto found = enumerate_boards();
    printf("project boards attached: %zu\n", found.size());
    for (const auto& info : found)
        printf(
            "  %s  pid=%s  %s  %s\n", info.serial.c_str(), info.pid.c_str(),
            usb_speed_name(info.speed), info.product.c_str());
    return found.size() >= 2 ? 0 : 1;
}

bool discover() {
    const auto found = enumerate_boards();
    if (found.size() < 2) {
        fprintf(stderr, "need two project boards, found %zu\n", found.size());
        return false;
    }
    g_serial_a = found[0].serial;
    g_serial_b = found[1].serial;
    if (const char* value = std::getenv("HCS_BOARD_A"))
        g_serial_a = value;
    if (const char* value = std::getenv("HCS_BOARD_B"))
        g_serial_b = value;
    return true;
}

// Receiver that records, per bus, what arrived. Shared by every mode; each mode
// arms the parts it needs.
class Sink final : public examples::BoardReceiver {
public:
    void watch_can(int bus) { watched_bus_.store(bus, std::memory_order_relaxed); }

    // link mode
    std::atomic<uint32_t> hits[8]{};
    // Per-CAN-id tallies, for topologies where several controllers share one
    // physical segment. hits[] is indexed by *receiving* bus, so on a shared
    // segment it merges every sender into one counter and cannot show which
    // transmitter a frame came from -- that made `arbitrate` look like it was
    // losing exactly half its frames when nothing was lost at all. Registered
    // ids are counted separately; anything else lands in id_other_.
    void watch_can_id(uint32_t can_id) {
        for (auto& slot : watched_ids_) {
            uint32_t expected = kNoId;
            if (slot.id.compare_exchange_strong(expected, can_id, std::memory_order_relaxed))
                return;
            if (expected == can_id)
                return;
        }
    }

    [[nodiscard]] uint32_t hits_for_id(uint32_t can_id) const {
        for (const auto& slot : watched_ids_) {
            if (slot.id.load(std::memory_order_relaxed) == can_id)
                return slot.count.load(std::memory_order_relaxed);
        }
        return 0;
    }

    [[nodiscard]] uint32_t hits_other_id() const {
        return id_other_.load(std::memory_order_relaxed);
    }

    void reset_can_ids() {
        for (auto& slot : watched_ids_) {
            slot.id.store(kNoId, std::memory_order_relaxed);
            slot.count.store(0, std::memory_order_relaxed);
        }
        id_other_.store(0, std::memory_order_relaxed);
    }

    // GPIO digital read results, per channel.
    std::atomic<uint32_t> gpio_reads[8]{};
    std::atomic<uint32_t> gpio_high[8]{};
    std::atomic<uint32_t> gpio_stamped[8]{};
    std::atomic<uint32_t> gpio_last_stamp[8]{};
    std::atomic<int> gpio_level[8]{};
    void reset_gpio() {
        for (int i = 0; i < 8; ++i) {
            gpio_reads[i].store(0, std::memory_order_relaxed);
            gpio_high[i].store(0, std::memory_order_relaxed);
            gpio_stamped[i].store(0, std::memory_order_relaxed);
            gpio_last_stamp[i].store(0, std::memory_order_relaxed);
            gpio_level[i].store(-1, std::memory_order_relaxed);
        }
    }

    std::atomic<uint32_t> uart_bytes{0};
    std::atomic<uint32_t> uart_matches{0};
    std::string uart_expected;
    std::string uart_seen;

    // latency mode
    void arm(uint32_t sequence, Clock::time_point send_time) {
        pending_.store(sequence, std::memory_order_relaxed);
        send_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(send_time.time_since_epoch())
                .count(),
            std::memory_order_relaxed);
        got_.store(false, std::memory_order_release);
    }

    bool wait(double& latency_us, std::chrono::milliseconds timeout) {
        const auto deadline = Clock::now() + timeout;
        while (!got_.load(std::memory_order_acquire)) {
            if (Clock::now() >= deadline)
                return false;
        }
        latency_us = static_cast<double>(latency_ns_.load(std::memory_order_relaxed)) / 1e3;
        return true;
    }

    // rate mode
    std::atomic<uint64_t> received{0};
    std::atomic<uint64_t> corrupt{0};

    // Populated only when the board runs a libhcs_CAN_DIAG firmware.
    DiagRecord diag_first, diag_last;

private:
    void on_gpio_digital(int channel, const libhcs::data::GpioDigitalDataView& data) override {
        if (channel < 0 || channel >= 8)
            return;
        gpio_level[channel].store(data.high ? 1 : 0, std::memory_order_relaxed);
        gpio_reads[channel].fetch_add(1, std::memory_order_relaxed);
        if (data.high)
            gpio_high[channel].fetch_add(1, std::memory_order_relaxed);
        if (data.timestamp_quarter_us.has_value()) {
            gpio_stamped[channel].fetch_add(1, std::memory_order_relaxed);
            gpio_last_stamp[channel].store(*data.timestamp_quarter_us,
                                           std::memory_order_relaxed);
        }
    }

    void on_can(int bus, const libhcs::data::CanDataView& data) override {
        const auto receive_time = Clock::now();
        if (bus >= 0 && bus < 8)
            hits[bus].fetch_add(1, std::memory_order_relaxed);

        // Attribute the frame to its transmitter when the mode registered ids.
        bool id_matched = false;
        for (auto& slot : watched_ids_) {
            const uint32_t id = slot.id.load(std::memory_order_relaxed);
            if (id == kNoId)
                break;
            if (id == data.can_id) {
                slot.count.fetch_add(1, std::memory_order_relaxed);
                id_matched = true;
                break;
            }
        }
        if (!id_matched && watched_ids_[0].id.load(std::memory_order_relaxed) != kNoId)
            id_other_.fetch_add(1, std::memory_order_relaxed);

        if (data.can_data.size() != kPayloadSize) {
            corrupt.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        const uint32_t sequence = get_u32_le(data.can_data.data());
        if (get_u32_le(data.can_data.data() + 4) != mix(sequence)) {
            corrupt.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        received.fetch_add(1, std::memory_order_relaxed);

        if (bus == watched_bus_.load(std::memory_order_relaxed)
            && sequence == pending_.load(std::memory_order_relaxed)
            && !got_.load(std::memory_order_relaxed)) {
            const int64_t receive_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           receive_time.time_since_epoch())
                                           .count();
            latency_ns_.store(
                receive_ns - send_ns_.load(std::memory_order_relaxed), std::memory_order_relaxed);
            got_.store(true, std::memory_order_release);
        }
    }

    void on_uart(int port, const libhcs::data::UartDataView& data) override {
        (void)port;
        // A CAN_DIAG firmware multiplexes its telemetry onto this same UART0
        // uplink, so decode it here rather than adding a second hook.
        DiagRecord record;
        if (decode_diag(data.uart_data, record)) {
            if (!diag_first.valid)
                diag_first = record;
            diag_last = record;
            return;
        }
        uart_bytes.fetch_add(
            static_cast<uint32_t>(data.uart_data.size()), std::memory_order_relaxed);
        for (const std::byte b : data.uart_data)
            uart_seen.push_back(static_cast<char>(std::to_integer<uint8_t>(b)));
        if (!uart_expected.empty() && uart_seen.find(uart_expected) != std::string::npos)
            uart_matches.fetch_add(1, std::memory_order_relaxed);
    }

    static constexpr uint32_t kNoId = 0xFFFFFFFFU;
    struct IdSlot {
        std::atomic<uint32_t> id{kNoId};
        std::atomic<uint32_t> count{0};
    };
    IdSlot watched_ids_[4];
    std::atomic<uint32_t> id_other_{0};

    std::atomic<int> watched_bus_{-1};
    std::atomic<uint32_t> pending_{0};
    std::atomic<int64_t> send_ns_{0};
    std::atomic<int64_t> latency_ns_{0};
    std::atomic<bool> got_{true};
};

struct Rig {
    Sink sink_a, sink_b;
    std::unique_ptr<examples::BoardSession> a, b;

    bool open() {
        a = examples::connect_any(sink_a, g_serial_a);
        b = examples::connect_any(sink_b, g_serial_b);
        if (!a || !b) {
            fprintf(stderr, "could not open both boards\n");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{300});
        printf(
            "A = %.*s (%d CAN, %d UART)   B = %.*s (%d CAN, %d UART)\n",
            static_cast<int>(a->name().size()), a->name().data(), a->can_bus_count(),
            a->uart_port_count(), static_cast<int>(b->name().size()), b->name().data(),
            b->can_bus_count(), b->uart_port_count());
        return true;
    }
};

// The HCS_CAN_FD=1 opt-in is gone with the per-frame is_fdcan flag: the frame
// type is a property of the BUS, fixed by the firmware's port table and read
// back over EP0, so there is nothing left for an environment variable to
// choose. Every probe below sends in whatever mode the target bus runs.

// HCS_CAN_EXT=1 uses a 29-bit identifier, so the probes cover the extended-ID
// path as well as the FD/classic split.
bool use_extended_id() {
    const char* value = std::getenv("HCS_CAN_EXT");
    return value && value[0] == '1';
}

void send_frame(examples::BoardSession& board, int bus, uint32_t sequence, uint32_t can_id) {
    std::byte payload[kPayloadSize];
    put_u32_le(payload, sequence);
    put_u32_le(payload + 4, mix(sequence));
    const bool ext = use_extended_id();
    board.transmit([&](examples::BoardTransmitter& tx) {
        tx.can(bus, {.can_id = ext ? (0x1AB0000U | can_id) : can_id, .can_data = payload, .is_extended_can_id = ext});
    });
}

// canpack: put one frame for EACH of two buses into a SINGLE transmit() call and
// check both reach the wire. run_both() already does this for A's bus0+bus1, but
// nothing covered a board sending on two buses that are strapped to each other,
// which is where a downlink packet carrying two CAN fields could plausibly lose
// one. Sender and receiver are the same board here (the strap), so a frame that
// never left is distinguishable from one that was never received.
int run_canpack(
    int which, int bus_a, int bus_b, uint32_t frames, uint32_t pace_ms, bool split_packets) {
    Rig rig;
    if (!rig.open())
        return 1;
    examples::BoardSession& board = which == 0 ? *rig.a : *rig.b;
    Sink& sink = which == 0 ? rig.sink_a : rig.sink_b;

    if (bus_a >= board.can_bus_count() || bus_b >= board.can_bus_count()) {
        fprintf(stderr, "board %c has %d CAN buses\n", which == 0 ? 'A' : 'B',
            board.can_bus_count());
        return 1;
    }

    const uint32_t id_a = kCanIdBase + 0x30;
    const uint32_t id_b = kCanIdBase + 0x31;
    printf("\nboard %c: CAN%d(id %03X) + CAN%d(id %03X) in ONE packet, %u rounds (%s)\n",
        which == 0 ? 'A' : 'B', bus_a, id_a, bus_b, id_b, frames,
        "frames follow the bus mode");

    for (Sink* s : {&rig.sink_a, &rig.sink_b}) {
        s->reset_can_ids();
        s->watch_can_id(id_a);
        s->watch_can_id(id_b);
        for (int i = 0; i < 8; ++i)
            s->hits[i].store(0, std::memory_order_relaxed);
        s->corrupt.store(0, std::memory_order_relaxed);
    }

    for (uint32_t i = 0; i < frames; ++i) {
        std::byte payload[kPayloadSize];
        put_u32_le(payload, i);
        put_u32_le(payload + 4, mix(i));
        if (split_packets) {
            // Same two frames, two separate transmit() calls. Isolates "two CAN
            // fields in ONE packet" from "two frames close together in time".
            board.transmit([&](examples::BoardTransmitter& tx) {
                tx.can(bus_a, {.can_id = id_a, .can_data = payload});
            });
            board.transmit([&](examples::BoardTransmitter& tx) {
                tx.can(bus_b, {.can_id = id_b, .can_data = payload});
            });
        } else {
            board.transmit([&](examples::BoardTransmitter& tx) {
                tx.can(bus_a, {.can_id = id_a, .can_data = payload});
                tx.can(bus_b, {.can_id = id_b, .can_data = payload});
            });
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{pace_ms});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{400});

    // Also ask the OTHER board what it saw on the same segment: it is a wholly
    // independent observer, so it separates "the frame never reached the wire"
    // from "the sender's own board failed to report it".
    Sink& peer = which == 0 ? rig.sink_b : rig.sink_a;
    const uint32_t peer_a = peer.hits_for_id(id_a);
    const uint32_t peer_b = peer.hits_for_id(id_b);
    printf("  peer board saw: id %03X -> %u    id %03X -> %u\n", id_a, peer_a, id_b, peer_b);
    // If the frames are colliding on the wire rather than never being queued,
    // the observing controller records it in PSR.LEC / the error counters.
    if (peer.diag_last.valid) {
        static const char* const kLec[]{"none", "stuff", "form", "ack", "bit1", "bit0", "crc",
            "no-change"};
        for (int i = 0; i < 2; ++i) {
            const uint32_t psr = peer.diag_last.psr[i];
            const uint32_t ecr = peer.diag_last.ecr[i];
            printf("      peer CAN%d: LEC=%s DLEC=%s  TEC=%lu REC=%lu%s\n", i,
                kLec[psr & 0x7U], kLec[(psr >> 8) & 0x7U],
                static_cast<unsigned long>(ecr & 0xFFU),
                static_cast<unsigned long>((ecr >> 8) & 0x7FU),
                (psr & (1U << 7)) ? "  BUS_OFF" : "");
        }
    }

    const uint32_t got_a = sink.hits_for_id(id_a);
    const uint32_t got_b = sink.hits_for_id(id_b);
    printf("  frames from CAN%d: %u/%u    from CAN%d: %u/%u   (corrupt %llu)\n", bus_a, got_a,
        frames, bus_b, got_b, frames,
        static_cast<unsigned long long>(sink.corrupt.load(std::memory_order_relaxed)));
    const bool ok = got_a == frames && got_b == frames;
    printf("canpack: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// canloop: two CAN buses of the SAME board strapped together. `link` only ever
// sweeps A-against-B, so a same-board strap (mc02 CAN2<->CAN3 on this rig) was
// never exercised by any mode -- and a controller that cannot transmit shows up
// here without a second board in the path to blame.
int run_canloop(int which, int tx_bus, int rx_bus, uint32_t frames) {
    Rig rig;
    if (!rig.open())
        return 1;
    examples::BoardSession& board = which == 0 ? *rig.a : *rig.b;
    Sink& sink = which == 0 ? rig.sink_a : rig.sink_b;

    if (tx_bus >= board.can_bus_count() || rx_bus >= board.can_bus_count()) {
        fprintf(stderr, "board %c has %d CAN buses\n", which == 0 ? 'A' : 'B',
            board.can_bus_count());
        return 1;
    }

    printf("\nboard %c CAN%d -> CAN%d, %u frames (%s)\n", which == 0 ? 'A' : 'B', tx_bus, rx_bus,
        frames, "frames follow the bus mode");

    const uint32_t can_id = kCanIdBase + 0x20 + static_cast<uint32_t>(tx_bus);
    sink.reset_can_ids();
    sink.watch_can_id(can_id);
    for (int i = 0; i < 8; ++i)
        sink.hits[i].store(0, std::memory_order_relaxed);
    sink.corrupt.store(0, std::memory_order_relaxed);

    for (uint32_t i = 0; i < frames; ++i) {
        send_frame(board, tx_bus, i, can_id);
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{400});

    const uint32_t got_rx = sink.hits[rx_bus].load(std::memory_order_relaxed);
    const uint32_t by_id = sink.hits_for_id(can_id);
    const uint64_t corrupt = sink.corrupt.load(std::memory_order_relaxed);
    printf("  CAN%d received %u/%u  (by id %u, corrupt %llu)\n", rx_bus, got_rx, frames, by_id,
        static_cast<unsigned long long>(corrupt));
    // A transmitter that never reaches the wire and a receiver that never
    // reports are indistinguishable from one side; print the other buses so a
    // frame that landed somewhere unexpected is visible rather than silent.
    for (int bus = 0; bus < board.can_bus_count(); ++bus) {
        if (bus == rx_bus)
            continue;
        const uint32_t other = sink.hits[bus].load(std::memory_order_relaxed);
        if (other)
            printf("      note: CAN%d also saw %u frames\n", bus, other);
    }
    const bool ok = got_rx == frames && corrupt == 0;
    printf("canloop: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// ---------------------------------------------------------------------------
// link: discover which bus of A reaches which bus of B, in both directions.
// Classic CAN is used throughout: it is the only frame format both ends are
// guaranteed to agree on before the wiring is known.
// ---------------------------------------------------------------------------

int run_link() {
    Rig rig;
    if (!rig.open())
        return 1;

    printf("\nCAN wiring sweep (classic CAN, 8-byte payload)\n");
    bool any = false;
    for (int direction = 0; direction < 2; ++direction) {
        examples::BoardSession& source = direction == 0 ? *rig.a : *rig.b;
        Sink& sink = direction == 0 ? rig.sink_b : rig.sink_a;
        const char* from = direction == 0 ? "A" : "B";
        const char* to = direction == 0 ? "B" : "A";
        const int source_buses = source.can_bus_count();
        const int sink_buses = (direction == 0 ? *rig.b : *rig.a).can_bus_count();

        for (int tx_bus = 0; tx_bus < source_buses; ++tx_bus) {
            for (int i = 0; i < 8; ++i)
                sink.hits[i].store(0, std::memory_order_relaxed);
            for (int attempt = 0; attempt < 5; ++attempt) {
                try {
                    send_frame(source, tx_bus, 0xA5A50000U + tx_bus, kCanIdBase + tx_bus);
                } catch (const std::exception&) {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds{20});
            }
            for (int rx_bus = 0; rx_bus < sink_buses; ++rx_bus) {
                const uint32_t hits = sink.hits[rx_bus].load(std::memory_order_relaxed);
                if (hits) {
                    printf("  %s.CAN%d -> %s.CAN%d   CONNECTED (%u frames)\n", from, tx_bus, to,
                        rx_bus, hits);
                    any = true;
                }
            }
        }
    }
    if (!any)
        printf("  no CAN path found between the two boards\n");

    // Align both ends before probing. `link` used to assume every port was still
    // at its compile-time default, so any earlier test that left a port at another
    // rate made this sweep report CORRUPT -- a tool artifact that reads exactly
    // like a UART regression, and cost real debugging time twice. Aligning first
    // makes the result depend on the wiring, which is what this mode is for.
    // Boards with no runtime UART configuration throw; that is fine, they can only
    // ever be at their compile-time rate anyway.
    printf("\nUART sweep (aligning both ends to %u baud first)\n", kLinkProbeBaudrate);
    for (int which = 0; which < 2; ++which) {
        examples::BoardSession& board = which == 0 ? *rig.a : *rig.b;
        for (int port = 0; port < board.uart_port_count(); ++port) {
            try {
                board.transmit([&](examples::BoardTransmitter& tx) {
                    tx.uart_config(port, {.baudrate = kLinkProbeBaudrate});
                });
            } catch (const std::exception&) {
            }
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    for (int direction = 0; direction < 2; ++direction) {
        examples::BoardSession& source = direction == 0 ? *rig.a : *rig.b;
        Sink& sink = direction == 0 ? rig.sink_b : rig.sink_a;
        const char* label = direction == 0 ? "A -> B" : "B -> A";
        for (int port = 0; port < source.uart_port_count(); ++port) {
            sink.uart_expected = "HCS-MIX-PROBE";
            sink.uart_seen.clear();
            sink.uart_matches.store(0, std::memory_order_relaxed);
            sink.uart_bytes.store(0, std::memory_order_relaxed);
            const std::string text = sink.uart_expected + "\n";
            for (int attempt = 0; attempt < 5; ++attempt) {
                try {
                    source.transmit([&](examples::BoardTransmitter& tx) {
                        tx.uart(port, {.uart_data = std::as_bytes(std::span{text.data(),
                                           text.size()}),
                            .idle_delimited = true});
                    });
                } catch (const std::exception&) {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds{20});
            }
            const uint32_t matches = sink.uart_matches.load(std::memory_order_relaxed);
            const uint32_t bytes = sink.uart_bytes.load(std::memory_order_relaxed);
            if (bytes)
                printf(
                    "  %s UART%d -> %s  (%u bytes)\n", label, port,
                    matches ? "CONNECTED, content OK" : "bytes arrived but CORRUPT", bytes);
        }
    }
    return any ? 0 : 1;
}

// ---------------------------------------------------------------------------
// latency: host -> A.CANtx -> wire -> B.CANrx -> host.
// ---------------------------------------------------------------------------

int run_latency(int tx_bus, int rx_bus, uint32_t samples) {
    Rig rig;
    if (!rig.open())
        return 1;
    rig.sink_b.watch_can(rx_bus);

    printf("\nA.CAN%d -> B.CAN%d, %u samples (classic CAN 8B)\n", tx_bus, rx_bus, samples);
    std::vector<double> latencies;
    latencies.reserve(samples);
    uint32_t timeouts = 0;

    for (uint32_t sequence = 0; sequence < samples + kWarmupSamples; ++sequence) {
        rig.sink_b.arm(sequence, Clock::now());
        send_frame(*rig.a, tx_bus, sequence, kCanIdBase + tx_bus);
        double latency_us = 0.0;
        if (!rig.sink_b.wait(latency_us, std::chrono::milliseconds{20})) {
            if (sequence >= kWarmupSamples)
                ++timeouts;
            continue;
        }
        if (sequence >= kWarmupSamples)
            latencies.push_back(latency_us);
    }

    if (latencies.empty()) {
        fprintf(stderr, "nothing arrived; check the bus indices with `link`\n");
        return 2;
    }
    std::sort(latencies.begin(), latencies.end());
    double sum = 0.0;
    for (const double value : latencies)
        sum += value;
    printf(
        "n=%zu timeout=%u corrupt=%llu\n", latencies.size(), timeouts,
        static_cast<unsigned long long>(rig.sink_b.corrupt.load(std::memory_order_relaxed)));
    printf(
        "latency us: min %.1f  p50 %.1f  p90 %.1f  p99 %.1f  avg %.1f  max %.1f\n",
        latencies.front(), percentile(latencies, 0.50), percentile(latencies, 0.90),
        percentile(latencies, 0.99), sum / static_cast<double>(latencies.size()),
        latencies.back());
    return 0;
}

// ---------------------------------------------------------------------------
// rate: how many USB packets per second this board sustains before the
// transmitting end starts dropping. `frames_per_packet` separates the packet
// rate from the frame rate -- the whole point of the exercise.
// ---------------------------------------------------------------------------

int run_rate(int tx_bus, int rx_bus, uint32_t seconds, int frames_per_packet) {
    Rig rig;
    if (!rig.open())
        return 1;
    rig.sink_b.watch_can(rx_bus);

    printf(
        "\npacket-rate sweep (%s): A.CAN%d -> B.CAN%d, %d frame(s) per USB packet, %u s/step\n",
        "frames follow the bus mode", tx_bus, rx_bus, frames_per_packet, seconds);
    printf("%10s %12s %12s %12s   %s\n", "packets/s", "frames/s", "sent", "delivered", "loss");

    // Classic CAN 8-byte frames run ~120 us on the wire, so the bus itself caps
    // near 8300 frames/s. These steps sit under that on average -- what is being
    // probed is whether a USB burst can still overflow the 32-element TX FIFO.
    // Low packet rates on purpose: with several frames batched into each packet
    // the board receives them as one burst, while the average frame rate stays
    // far under the ~8300 frames/s a classic-CAN bus can carry. That isolates
    // the depth question (does a burst deeper than the 32-element TX FIFO
    // survive?) from the bandwidth question.
    for (const uint32_t packets_per_sec : {50U, 100U, 200U}) {
        rig.sink_b.received.store(0, std::memory_order_relaxed);
        rig.sink_b.corrupt.store(0, std::memory_order_relaxed);

        const auto period = std::chrono::nanoseconds{1'000'000'000ULL / packets_per_sec};
        const uint64_t packets = static_cast<uint64_t>(packets_per_sec) * seconds;
        auto next = Clock::now();
        for (uint64_t index = 0; index < packets; ++index) {
            std::byte payload[kPayloadSize];
            put_u32_le(payload, static_cast<uint32_t>(index));
            put_u32_le(payload + 4, mix(static_cast<uint32_t>(index)));
            rig.a->transmit([&](examples::BoardTransmitter& tx) {
                for (int repeat = 0; repeat < frames_per_packet; ++repeat)
                    tx.can(tx_bus, {.can_id = kCanIdBase + tx_bus, .can_data = payload});
            });
            next += period;
            while (Clock::now() < next) {}
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{400});

        const uint64_t sent = packets * static_cast<uint64_t>(frames_per_packet);
        const uint64_t got = rig.sink_b.received.load(std::memory_order_relaxed);
        if (rig.sink_a.diag_last.valid) {
            printf(
                "%10u %12llu %12llu %12llu   tx_fail=%u\n", packets_per_sec,
                static_cast<unsigned long long>(packets_per_sec)
                    * static_cast<unsigned long long>(frames_per_packet),
                static_cast<unsigned long long>(sent), static_cast<unsigned long long>(got),
                rig.sink_a.diag_last.tx_fail[tx_bus] - rig.sink_a.diag_first.tx_fail[tx_bus]);
            rig.sink_a.diag_first = rig.sink_a.diag_last; // rebase for the next step
            fflush(stdout);
            continue;
        }
        printf(
            "%10u %12llu %12llu %12llu   %.4f%%\n", packets_per_sec,
            static_cast<unsigned long long>(packets_per_sec)
                * static_cast<unsigned long long>(frames_per_packet),
            static_cast<unsigned long long>(sent), static_cast<unsigned long long>(got),
            sent ? 100.0 * static_cast<double>(sent - got) / static_cast<double>(sent) : 0.0);
        fflush(stdout);
    }
    return 0;
}

// txprobe: send a few frames from one specific board+bus and report whether the
// board is still alive afterwards. Exists because the link sweep found that
// transmitting from one of the boards resets it, and a sweep cannot say which
// bus did it.
int run_txprobe(int which, int bus, uint32_t count) {
    Rig rig;
    if (!rig.open())
        return 1;
    examples::BoardSession& source = which == 0 ? *rig.a : *rig.b;
    Sink& own = which == 0 ? rig.sink_a : rig.sink_b;

    // bus >= 100 selects UART port (bus - 100) instead of a CAN bus, so the
    // same probe can tell "CAN transmit is broken" apart from "any downlink is".
    const bool use_uart = bus >= 100;
    const int port = bus - 100;
    printf("\nsending %u %s from board %c %s %d ...\n", count,
        use_uart ? "UART chunks" : "classic frames", which == 0 ? 'A' : 'B',
        use_uart ? "port" : "bus", use_uart ? port : bus);
    fflush(stdout);
    for (uint32_t i = 0; i < count; ++i) {
        try {
            if (use_uart) {
                const char text[] = "HCS-TXPROBE\n";
                source.transmit([&](examples::BoardTransmitter& tx) {
                    tx.uart(port, {.uart_data = std::as_bytes(std::span{text, sizeof(text) - 1}),
                                   .idle_delimited = true});
                });
            } else
                send_frame(source, bus, i, kCanIdBase + bus);
        } catch (const std::exception& error) {
            printf("  transmit threw at frame %u: %s\n", i, error.what());
            return 2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    // If the board reset, the transport thread would already have torn the
    // process down; reaching here at all is the survival signal.
    {
        Sink& own = which == 0 ? rig.sink_a : rig.sink_b;
        if (own.diag_last.valid) {
            static const char* kLec[] = {"no error", "stuff error", "form error",
                "ACK error", "bit1 error", "bit0 error", "CRC error", "no change"};
            for (int i = 0; i < 2; ++i) {
                const uint32_t psr = own.diag_last.psr[i];
                const uint32_t ecr = own.diag_last.ecr[i];
                printf(
                    "  bus%d PSR=0x%08X LEC=%s DLEC=%s BO=%d EP=%d EW=%d | TEC=%u REC=%u\n", i,
                    psr, kLec[psr & 0x7], kLec[(psr >> 8) & 0x7],
                    (psr >> 7) & 1, (psr >> 5) & 1, (psr >> 6) & 1,
                    ecr & 0xFF, (ecr >> 8) & 0x7F);
                // NBTP: NSJW[31:25] NBRP[24:16] NTSEG1[15:8] NTSEG2[6:0]
                // DBTP: DBRP[20:16] DTSEG1[12:8] DTSEG2[7:4] DSJW[3:0] TDC[23]
                const uint32_t nb = own.diag_last.nbtp[i], db = own.diag_last.dbtp[i];
                const uint32_t nbrp = ((nb >> 16) & 0x1FF) + 1, ns1 = ((nb >> 8) & 0xFF) + 1,
                               ns2 = (nb & 0x7F) + 1;
                const uint32_t dbrp = ((db >> 16) & 0x1F) + 1, ds1 = ((db >> 8) & 0x1F) + 1,
                               ds2 = ((db >> 4) & 0xF) + 1;
                printf(
                    "       NBTP=0x%08X brp=%u tseg1=%u tseg2=%u -> %.3f Mbit sp=%.1f%%\n", nb,
                    nbrp, ns1, ns2, 80.0 / (nbrp * (1 + ns1 + ns2)),
                    100.0 * (1 + ns1) / (1 + ns1 + ns2));
                printf(
                    "       DBTP=0x%08X brp=%u tseg1=%u tseg2=%u -> %.3f Mbit sp=%.1f%% TDC=%u\n",
                    db, dbrp, ds1, ds2, 80.0 / (dbrp * (1 + ds1 + ds2)),
                    100.0 * (1 + ds1) / (1 + ds1 + ds2), (db >> 23) & 1);
                if (i == 0 && own.diag_last.uart_clock) {
                    // OSC field 0 encodes an oversample rate of 32.
                    const uint32_t osc = (own.diag_last.uart_oscr & 0x1F) == 0
                                           ? 32U
                                           : (own.diag_last.uart_oscr & 0x1F);
                    const uint32_t div = own.diag_last.uart_div;
                    printf(
                        "       UART0 clock=%u Hz OSCR=0x%08X osc=%u div=%u -> actual baud %u\n",
                        own.diag_last.uart_clock, own.diag_last.uart_oscr, osc, div,
                        div && osc ? own.diag_last.uart_clock / (osc * div) : 0);
                }
            }
        }
    }
    {
        Sink& peer = which == 0 ? rig.sink_b : rig.sink_a;
        if (!peer.uart_seen.empty()) {
            const size_t dump = std::min<size_t>(peer.uart_seen.size(), 48);
            printf("  peer received %zu UART bytes, first %zu:", peer.uart_seen.size(), dump);
            for (size_t i = 0; i < dump; ++i)
                printf(" %02X", static_cast<unsigned>(
                                    static_cast<uint8_t>(peer.uart_seen[i])));
            printf("\n");
        }
    }
    printf(
        "  survived. own-board rx frames=%llu, peer rx frames=%llu\n",
        static_cast<unsigned long long>(own.received.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>((which == 0 ? rig.sink_b : rig.sink_a)
                                            .received.load(std::memory_order_relaxed)));
    return 0;
}

// uartrate: push UART downlink packets to one board at increasing packet rates
// and count what comes out the other end. UART is used instead of CAN because
// it is the only downlink path that works on both boards here, and at 921600
// the wire stays well clear of the limit being probed (13 bytes x 6000 pkt/s =
// 78 kB/s against a 92 kB/s line).
//
// This is the measurement that separates "USB bandwidth" from "USB packet
// rate": the payload is tiny, so bandwidth is never in question, and the only
// variable is how many separate transfers per second the link will carry.
int run_uartrate(int which, int port, uint32_t seconds) {
    Rig rig;
    if (!rig.open())
        return 1;
    examples::BoardSession& source = which == 0 ? *rig.a : *rig.b;
    Sink& sink = which == 0 ? rig.sink_b : rig.sink_a;

    const char text[] = "HCS-TXPROBE\n";
    constexpr uint32_t kChunk = sizeof(text) - 1;
    printf(
        "\ndownlink packet-rate sweep into board %c UART%d (%u bytes per packet, %u s/step)\n",
        which == 0 ? 'A' : 'B', port, kChunk, seconds);
    printf("%10s %12s %12s %12s   %s\n", "packets/s", "sent B", "got B", "kB/s", "shortfall");

    for (const uint32_t rate : {500U, 1000U, 2000U, 3000U, 4000U, 6000U, 8000U}) {
        sink.uart_expected.clear();
        sink.uart_seen.clear();
        sink.uart_bytes.store(0, std::memory_order_relaxed);

        const auto period = std::chrono::nanoseconds{1'000'000'000ULL / rate};
        const uint64_t packets = static_cast<uint64_t>(rate) * seconds;
        auto next = Clock::now();
        for (uint64_t i = 0; i < packets; ++i) {
            source.transmit([&](examples::BoardTransmitter& tx) {
                tx.uart(port, {.uart_data = std::as_bytes(std::span{text, kChunk}),
                               .idle_delimited = false});
            });
            next += period;
            while (Clock::now() < next) {}
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{600});

        const uint64_t sent = packets * kChunk;
        const uint64_t got = sink.uart_bytes.load(std::memory_order_relaxed);
        printf(
            "%10u %12llu %12llu %12.1f   %.2f%%\n", rate,
            static_cast<unsigned long long>(sent), static_cast<unsigned long long>(got),
            static_cast<double>(got) / seconds / 1e3,
            sent ? 100.0 * static_cast<double>(sent - got) / static_cast<double>(sent) : 0.0);
        fflush(stdout);
    }
    return 0;
}

// uart: align both ends to one baudrate over the runtime config channel, then
// check the link end to end. The two boards ship with different compile-time
// UART baudrates (mc02 UART1 = 115200 from CubeMX, hpm5321 UART0 = 921600), so
// without this step the wire carries nothing but framing errors.
int run_uart(uint32_t baudrate, uint32_t rounds) {
    Rig rig;
    if (!rig.open())
        return 1;

    printf("\nsetting both ends to %u baud ...\n", baudrate);
    for (int which = 0; which < 2; ++which) {
        examples::BoardSession& board = which == 0 ? *rig.a : *rig.b;
        try {
            board.transmit([&](examples::BoardTransmitter& tx) {
                tx.uart_config(0, {.baudrate = baudrate});
            });
        } catch (const std::exception& error) {
            printf("  board %c: %s\n", which == 0 ? 'A' : 'B', error.what());
            return 2;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    bool all_ok = true;
    for (int direction = 0; direction < 2; ++direction) {
        examples::BoardSession& source = direction == 0 ? *rig.a : *rig.b;
        Sink& sink = direction == 0 ? rig.sink_b : rig.sink_a;
        const char* label = direction == 0 ? "A.UART0 -> B.UART0" : "B.UART0 -> A.UART0";

        sink.uart_expected.clear();
        sink.uart_seen.clear();
        sink.uart_bytes.store(0, std::memory_order_relaxed);

        const char text[] = "HCS-UART-CHECK\n";
        constexpr uint32_t kChunk = sizeof(text) - 1;
        for (uint32_t i = 0; i < rounds; ++i) {
            source.transmit([&](examples::BoardTransmitter& tx) {
                tx.uart(0, {.uart_data = std::as_bytes(std::span{text, kChunk}),
                            .idle_delimited = true});
            });
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{400});

        const uint32_t got = sink.uart_bytes.load(std::memory_order_relaxed);
        const uint32_t expected = rounds * kChunk;
        // Count how many intact copies of the probe string arrived, so a byte
        // count that merely looks right cannot pass.
        size_t copies = 0, at = 0;
        while ((at = sink.uart_seen.find(text, at)) != std::string::npos) {
            ++copies;
            at += kChunk;
        }
        const bool ok = got == expected && copies == rounds;
        all_ok &= ok;
        printf(
            "  %-20s %s  (%u/%u bytes, %zu/%u intact copies)\n", label, ok ? "PASS" : "FAIL",
            got, expected, copies, rounds);
        if (!ok && !sink.uart_seen.empty()) {
            const size_t dump = std::min<size_t>(sink.uart_seen.size(), 32);
            printf("      first %zu rx bytes:", dump);
            for (size_t i = 0; i < dump; ++i)
                printf(" %02X", static_cast<unsigned>(
                                    static_cast<uint8_t>(sink.uart_seen[i])));
            printf("\n");
        }
    }
    printf("uart: %s\n", all_ok ? "PASS" : "FAIL");
    return all_ok ? 0 : 1;
}

// both: drive both CAN buses at once, in one USB packet per tick, and check the
// two paths stay independent. Bus index is encoded in the CAN id so a frame that
// surfaced on the wrong bus is detectable rather than merely miscounted.
int run_both(uint32_t frames_per_sec, uint32_t seconds) {
    Rig rig;
    if (!rig.open())
        return 1;
    const int buses = std::min(rig.a->can_bus_count(), rig.b->can_bus_count());
    if (buses < 2) {
        fprintf(stderr, "need two CAN buses on both boards\n");
        return 1;
    }

    printf(
        "\nboth buses, %u frames/s each, %u s (%s)\n", frames_per_sec, seconds,
        "frames follow the bus mode");

    const auto period = std::chrono::nanoseconds{1'000'000'000ULL / frames_per_sec};
    const uint64_t total = static_cast<uint64_t>(frames_per_sec) * seconds;
    auto next = Clock::now();
    for (uint64_t i = 0; i < total; ++i) {
        std::byte payload[kPayloadSize];
        put_u32_le(payload, static_cast<uint32_t>(i));
        put_u32_le(payload + 4, mix(static_cast<uint32_t>(i)));
        rig.a->transmit([&](examples::BoardTransmitter& tx) {
            for (int bus = 0; bus < 2; ++bus)
                tx.can(bus, {.can_id = kCanIdBase + static_cast<uint32_t>(bus),
                             .can_data = payload});
        });
        next += period;
        while (Clock::now() < next) {}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{500});

    const uint32_t got0 = rig.sink_b.hits[0].load(std::memory_order_relaxed);
    const uint32_t got1 = rig.sink_b.hits[1].load(std::memory_order_relaxed);
    printf(
        "bus0 %u/%llu  bus1 %u/%llu  corrupt=%llu\n", got0,
        static_cast<unsigned long long>(total), got1,
        static_cast<unsigned long long>(total),
        static_cast<unsigned long long>(rig.sink_b.corrupt.load(std::memory_order_relaxed)));
    const bool ok = got0 == total && got1 == total
                 && rig.sink_b.corrupt.load(std::memory_order_relaxed) == 0;
    printf("both: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// uartbaud: set ONE side's baudrate and leave the other at its compile-time
// default, then look at the raw bytes. A side whose config was ignored still
// delivers readable text; a side that switched delivers framing garbage. That
// tells the two failure modes apart, which the symmetric `uart` mode cannot.
int run_uartbaud(int which, uint32_t baudrate) {
    Rig rig;
    if (!rig.open())
        return 1;
    examples::BoardSession& target = which == 0 ? *rig.a : *rig.b;
    printf("\nsetting ONLY board %c to %u baud, peer left at its default\n",
        which == 0 ? 'A' : 'B', baudrate);
    try {
        target.transmit([&](examples::BoardTransmitter& tx) {
            tx.uart_config(0, {.baudrate = baudrate});
        });
    } catch (const std::exception& error) {
        printf("  config rejected: %s\n", error.what());
        return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{600});

    // What the board's own registers say now (CAN_DIAG builds only).
    Sink& own = which == 0 ? rig.sink_a : rig.sink_b;
    if (own.diag_last.uart_clock) {
        const uint32_t osc = (own.diag_last.uart_oscr & 0x1F) == 0
                               ? 32U : (own.diag_last.uart_oscr & 0x1F);
        const uint32_t div = own.diag_last.uart_div;
        printf(
            "  actual baud after switch: clock=%u osc=%u div=%u -> %u\n",
            own.diag_last.uart_clock, osc, div,
            div && osc ? own.diag_last.uart_clock / (osc * div) : 0);
    }

    // Send from each side and dump what the other actually saw.
    for (int dir = 0; dir < 2; ++dir) {
        examples::BoardSession& src = dir == 0 ? *rig.a : *rig.b;
        Sink& sink = dir == 0 ? rig.sink_b : rig.sink_a;
        sink.uart_expected.clear();
        sink.uart_seen.clear();
        const char text[] = "ABCDEFGH";
        for (int i = 0; i < 4; ++i) {
            src.transmit([&](examples::BoardTransmitter& tx) {
                tx.uart(0, {.uart_data = std::as_bytes(std::span{text, sizeof(text) - 1}),
                            .idle_delimited = true});
            });
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{200});
        printf("  %s -> %s: %zu bytes:", dir == 0 ? "A" : "B", dir == 0 ? "B" : "A",
            sink.uart_seen.size());
        for (size_t i = 0; i < std::min<size_t>(sink.uart_seen.size(), 24); ++i)
            printf(" %02X", static_cast<unsigned>(static_cast<uint8_t>(sink.uart_seen[i])));
        printf("   (sent 41 42 43 44 45 46 47 48)\n");
    }
    return 0;
}

// gpioedge: the remaining declared GPIO capabilities -- edge interrupts,
// hardware timestamps, and the pull configuration -- none of which the level and
// duty tests touch. Uses the same jumper: the driven channel toggles, and the
// input is armed for edges only (no periodic polling), so a report can only have
// come from an EXTI firing.
int run_gpioedge(int which, int out_ch, int in_ch, uint32_t toggles) {
    Rig rig;
    if (!rig.open())
        return 1;
    examples::BoardSession& board = which == 0 ? *rig.a : *rig.b;
    Sink& sink = which == 0 ? rig.sink_a : rig.sink_b;
    if (out_ch >= board.gpio_channel_count() || in_ch >= board.gpio_channel_count()
        || out_ch == in_ch) {
        fprintf(stderr, "need two distinct GPIO channels\n");
        return 1;
    }
    printf("\nboard %c GPIO edge/timestamp ch%d -> ch%d, %u toggles\n",
        which == 0 ? 'A' : 'B', out_ch, in_ch, toggles);

    int failures = 0;

    // 1. Pull configuration, with nothing driving the pin: the input must read
    //    the pull it was given. This is what makes "no wire" distinguishable
    //    elsewhere, so it is worth confirming directly.
    board.transmit([&](examples::BoardTransmitter& tx) {
        tx.gpio_digital(out_ch, {.high = false});
    });
    for (const auto [name, pull, want] :
         {std::tuple{"pull-down", libhcs::data::GpioPull::kDown, 0},
          std::tuple{"pull-up", libhcs::data::GpioPull::kUp, 1}}) {
        sink.reset_gpio();
        // Read the OUTPUT channel's own pull instead of the wired input, so the
        // peer driving low cannot mask a pull-up.
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_digital_read(in_ch, {.period_ms = 5, .pull = pull});
        });
        std::this_thread::sleep_for(std::chrono::milliseconds{200});
        const int got = sink.gpio_level[in_ch].load(std::memory_order_relaxed);
        // With the peer actively driving low, a pull-up cannot win -- that is
        // expected and not a failure; what matters is that pull-down reads low.
        const bool ok = (want == 0) ? (got == 0) : true;
        printf("  %-10s -> reads %s  %s\n", name,
            got < 0 ? "none" : (got ? "HIGH" : "LOW"),
            ok ? "ok" : "FAIL");
        if (!ok)
            ++failures;
    }

    // 2. Edge interrupts + timestamps, with NO periodic polling.
    sink.reset_gpio();
    try {
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_digital_read(in_ch, {.period_ms = 0,
                                         .rising_edge = true,
                                         .falling_edge = true,
                                         .capture_timestamp = true,
                                         .pull = libhcs::data::GpioPull::kDown});
        });
    } catch (const std::exception& error) {
        printf("  could not arm edge interrupts: %s\n", error.what());
        return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
    const uint32_t idle_reports = sink.gpio_reads[in_ch].load(std::memory_order_relaxed);

    for (uint32_t i = 0; i < toggles; ++i) {
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_digital(out_ch, {.high = (i % 2) == 0});
        });
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{200});

    const uint32_t reports = sink.gpio_reads[in_ch].load(std::memory_order_relaxed)
                           - idle_reports;
    const uint32_t stamped = sink.gpio_stamped[in_ch].load(std::memory_order_relaxed);
    printf("  edge reports: %u for %u toggles (idle reports before toggling: %u)\n",
        reports, toggles, idle_reports);
    printf("  timestamped:  %u  last stamp %u (quarter-us)\n", stamped,
        sink.gpio_last_stamp[in_ch].load(std::memory_order_relaxed));

    // Every toggle is a real edge, so each must produce a report. Allow the count
    // to exceed toggles (contact bounce is impossible here, but a shared EXTI
    // could double-report) -- the failure mode that matters is too FEW.
    if (reports < toggles) {
        printf("  FAIL: fewer edge reports than toggles -- EXTI not firing for "
               "every transition\n");
        ++failures;
    }
    if (!stamped) {
        printf("  FAIL: capture_timestamp was requested but no sample carried one\n");
        ++failures;
    }

    try {
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_digital(out_ch, {.high = false});
            tx.gpio_digital_read(in_ch, {.period_ms = 0, .pull = libhcs::data::GpioPull::kNone});
        });
    } catch (const std::exception&) {
    }
    printf("gpioedge: %s\n", failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}

// pwmloop: verify the PWM/analog output for real, over the same jumper wire.
//
// A digital read cannot see a duty cycle directly, but the pins run at 50 Hz
// (TIM1/TIM2: prescaler 274, period 19999 -> 20000 counts = 20 ms), which is slow
// enough to sample statistically: arm the input for 1 ms periodic reads, hold a
// duty for a while, and the fraction of samples that came back HIGH estimates the
// duty. That turns `gpio_analog` from "the write was accepted" into a measurement.
//
// Servo range is the point of this channel, so the duties tested are the pulse
// widths a servo actually uses: 1000 / 1500 / 2000 us out of 20 ms = 5 / 7.5 / 10
// percent, plus 0 and 100 percent as anchors.
int run_pwmloop(int which, int out_ch, int in_ch) {
    Rig rig;
    if (!rig.open())
        return 1;
    examples::BoardSession& board = which == 0 ? *rig.a : *rig.b;
    Sink& sink = which == 0 ? rig.sink_a : rig.sink_b;
    const int channels = board.gpio_channel_count();
    if (out_ch >= channels || in_ch >= channels || out_ch == in_ch) {
        fprintf(stderr, "need two distinct GPIO channels (board has %d)\n", channels);
        return 1;
    }

    printf("\nboard %c PWM ch%d -> ch%d (50 Hz, sampled by 1 ms periodic reads)\n",
        which == 0 ? 'A' : 'B', out_ch, in_ch);
    printf("%12s %10s %10s %9s  %s\n", "pulse", "duty set", "measured", "samples", "verdict");

    try {
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_digital_read(in_ch, {.period_ms = 1,
                                         .pull = libhcs::data::GpioPull::kDown});
        });
    } catch (const std::exception& error) {
        printf("  could not arm ch%d as input: %s\n", in_ch, error.what());
        return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{100});

    int failures = 0;
    struct Case { const char* label; uint32_t pulse_us; };
    // 500 / 1500 / 2500 us are what servo_test's `angle 0/90/180` emit, so this
    // also cross-checks that tool: it only reports the pulse it asked for and has
    // no way to see what reached the pin.
    for (const Case c : {Case{"0 (off)", 0}, Case{"500 us (0deg)", 500},
                         Case{"1000 us", 1000}, Case{"1500 us (90deg)", 1500},
                         Case{"2000 us", 2000}, Case{"2500 us (180deg)", 2500},
                         Case{"20000 (full)", 20000}}) {
        const double want = (double)c.pulse_us / 20000.0;
        const uint16_t duty16 = (uint16_t)std::min(65535.0, want * 65535.0 + 0.5);
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_analog(out_ch, {.value = duty16});
        });
        std::this_thread::sleep_for(std::chrono::milliseconds{150});

        const uint32_t base_n = sink.gpio_reads[in_ch].load(std::memory_order_relaxed);
        const uint32_t base_h = sink.gpio_high[in_ch].load(std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::milliseconds{1200});
        const uint32_t n = sink.gpio_reads[in_ch].load(std::memory_order_relaxed) - base_n;
        const uint32_t h = sink.gpio_high[in_ch].load(std::memory_order_relaxed) - base_h;
        const double got = n ? (double)h / (double)n : -1.0;

        // Sampling is unsynchronised to the PWM phase, so the estimate is noisy;
        // the tolerance has to admit that without becoming vacuous. 3 percentage
        // points is well inside the gap between adjacent servo positions (2.5 pts).
        const bool ok = n > 200 && got >= 0.0 && std::abs(got - want) < 0.03;
        printf("%12s %9.1f%% %9.1f%% %9u  %s\n", c.label, want * 100.0,
            got < 0 ? 0.0 : got * 100.0, n, ok ? "ok" : "FAIL");
        if (!ok)
            ++failures;
    }

    // Leave the pin low and the input disarmed.
    try {
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_analog(out_ch, {.value = 0});
            tx.gpio_digital_read(in_ch, {.period_ms = 0, .pull = libhcs::data::GpioPull::kNone});
        });
    } catch (const std::exception&) {
    }
    printf("pwmloop: %s\n", failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}

// gpioloop: two GPIO channels of the same board joined by a jumper wire. This is
// the only way to verify GPIO for real -- `gpio` mode can confirm a write was
// accepted, but not that the pin moved, because nothing observes the pin. Here
// one channel drives and the other is armed as a digital input, so a level that
// does not arrive is a genuine failure rather than an unobservable one.
//
// Wiring (mc02): ch0=PA0  ch1=PA2  ch2=PE9  ch3=PE13. Any two may be joined.
// A pull is configured on the input so an unconnected pin reads a known level --
// that is what distinguishes "no wire" from "driver broken": with pull-down, a
// missing wire reads low for BOTH requested levels, while a working link follows
// the driver.
int run_gpioloop(int which, int out_ch, int in_ch, uint32_t rounds) {
    Rig rig;
    if (!rig.open())
        return 1;
    examples::BoardSession& board = which == 0 ? *rig.a : *rig.b;
    Sink& sink = which == 0 ? rig.sink_a : rig.sink_b;
    const int channels = board.gpio_channel_count();
    if (out_ch >= channels || in_ch >= channels || out_ch == in_ch) {
        fprintf(stderr, "board %c has %d GPIO channels; need two distinct ones\n",
            which == 0 ? 'A' : 'B', channels);
        return 1;
    }

    printf("\nboard %c GPIO ch%d -> ch%d over a jumper wire, %u rounds\n",
        which == 0 ? 'A' : 'B', out_ch, in_ch, rounds);

    // Arm the input: periodic reads with a pull-down, so the level is defined
    // even with no wire attached.
    sink.reset_gpio();
    try {
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_digital_read(in_ch, {.period_ms = 5,
                                         .pull = libhcs::data::GpioPull::kDown});
        });
    } catch (const std::exception& error) {
        printf("  could not arm ch%d as input: %s\n", in_ch, error.what());
        return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{200});
    if (!sink.gpio_reads[in_ch].load(std::memory_order_relaxed)) {
        printf("  FAIL: ch%d reported no read results at all (read path dead)\n", in_ch);
        return 1;
    }

    // Counting both levels separately matters: an input stuck at one level still
    // "matches" on half the alternating rounds, which would read as a partial
    // pass. A real link must produce BOTH levels.
    uint32_t matched = 0, mismatched = 0, saw_high = 0, saw_low = 0;
    for (uint32_t i = 0; i < rounds; ++i) {
        const bool want = (i % 2) == 0;
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_digital(out_ch, {.high = want});
        });
        // Periodic reads run at 5 ms; allow several periods to land.
        std::this_thread::sleep_for(std::chrono::milliseconds{40});
        const int got = sink.gpio_level[in_ch].load(std::memory_order_relaxed);
        if (got == 1)
            ++saw_high;
        if (got == 0)
            ++saw_low;
        if (got == (want ? 1 : 0))
            ++matched;
        else
            ++mismatched;
    }

    printf("  levels followed the driver: %u/%u  (mismatched %u, reads %u)\n", matched,
        rounds, mismatched, sink.gpio_reads[in_ch].load(std::memory_order_relaxed));
    // Disarm the input before returning. A channel left in periodic-read mode
    // stays an input, so the NEXT run that wants to drive it finds it configured
    // the wrong way -- that made a back-to-back sweep report working pairs as
    // one-directional.
    try {
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_digital_read(in_ch, {.period_ms = 0, .pull = libhcs::data::GpioPull::kNone});
        });
    } catch (const std::exception&) {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{50});

    const bool ok = matched == rounds && saw_high && saw_low;
    if (!saw_high || !saw_low)
        printf("  input never left %s -- with the pull-down configured that means no "
               "jumper between ch%d and ch%d (a stuck level still matches half the "
               "alternating rounds, so %u/%u is NOT a partial pass)\n",
            saw_high ? "HIGH" : "LOW", out_ch, in_ch, matched, rounds);
    printf("gpioloop: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// gpio: exercise every GPIO channel the board declares. Without a scope this
// cannot confirm the waveform, so it verifies the reachable part -- that each
// channel is accepted, that an out-of-range channel is rejected, and that the
// board keeps forwarding CAN afterwards (a PWM write that wedged a timer or
// faulted would show up as the session dying).
int run_gpio(int which) {
    Rig rig;
    if (!rig.open())
        return 1;
    examples::BoardSession& board = which == 0 ? *rig.a : *rig.b;
    const int channels = board.gpio_channel_count();
    printf("\nboard %c declares %d GPIO channel(s)\n", which == 0 ? 'A' : 'B', channels);
    if (channels == 0) {
        printf("  nothing to test\n");
        return 0;
    }

    for (int ch = 0; ch < channels; ++ch) {
        bool analog_ok = true, digital_ok = true;
        try {
            board.transmit([&](examples::BoardTransmitter& tx) {
                tx.gpio_analog(ch, {.value = 2048});
            });
        } catch (const std::exception& error) {
            analog_ok = false;
            printf("  ch%d analog  rejected: %s\n", ch, error.what());
        }
        try {
            board.transmit([&](examples::BoardTransmitter& tx) {
                tx.gpio_digital(ch, {.high = true});
            });
        } catch (const std::exception& error) {
            digital_ok = false;
            printf("  ch%d digital rejected: %s\n", ch, error.what());
        }
        if (analog_ok && digital_ok)
            printf("  ch%d analog=accepted digital=accepted\n", ch);
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }

    // Out-of-range must throw rather than scribble.
    bool threw = false;
    try {
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_analog(channels, {.value = 0});
        });
    } catch (const std::exception&) {
        threw = true;
    }
    printf("  out-of-range channel %d rejected: %s\n", channels, threw ? "yes" : "NO (bug)");

    // Still alive? Send a CAN frame and see it land.
    Sink& peer = which == 0 ? rig.sink_b : rig.sink_a;
    const uint32_t before = peer.hits[0].load(std::memory_order_relaxed);
    for (int i = 0; i < 3; ++i) {
        send_frame(board, 0, 0xF00 + i, kCanIdBase);
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
    const uint32_t after = peer.hits[0].load(std::memory_order_relaxed);
    printf("  board still forwarding CAN after GPIO writes: %s (+%u frames)\n",
        after > before ? "yes" : "NO", after - before);
    return threw && after > before ? 0 : 1;
}

// uartloop: two UART ports of the SAME board wired to each other. This removes
// the peer board from the experiment entirely -- both ends of the link share one
// clock tree, one driver and one config path -- so a failure here is
// unambiguously that board's own baudrate handling.
int run_uartloop(int which, int tx_port, int rx_port, uint32_t baudrate, uint32_t rounds) {
    Rig rig;
    if (!rig.open())
        return 1;
    examples::BoardSession& board = which == 0 ? *rig.a : *rig.b;
    Sink& sink = which == 0 ? rig.sink_a : rig.sink_b;

    if (baudrate != 0) {
        printf("\nsetting board %c UART%d and UART%d to %u baud\n", which == 0 ? 'A' : 'B',
            tx_port, rx_port, baudrate);
        try {
            board.transmit([&](examples::BoardTransmitter& tx) {
                tx.uart_config(tx_port, {.baudrate = baudrate});
                tx.uart_config(rx_port, {.baudrate = baudrate});
            });
        } catch (const std::exception& error) {
            printf("  config rejected: %s\n", error.what());
            return 2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{300});
    } else {
        printf("\nboard %c UART%d -> UART%d at the compile-time default\n",
            which == 0 ? 'A' : 'B', tx_port, rx_port);
    }

    const char text[] = "HCS-LOOP-0123456789";
    constexpr uint32_t kChunk = sizeof(text) - 1;
    sink.uart_seen.clear();
    sink.uart_expected.clear();
    sink.uart_bytes.store(0, std::memory_order_relaxed);

    for (uint32_t i = 0; i < rounds; ++i) {
        board.transmit([&](examples::BoardTransmitter& tx) {
            tx.uart(tx_port, {.uart_data = std::as_bytes(std::span{text, kChunk}),
                              .idle_delimited = true});
        });
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{400});

    size_t copies = 0, at = 0;
    while ((at = sink.uart_seen.find(text, at)) != std::string::npos) {
        ++copies;
        at += kChunk;
    }
    const uint32_t got = sink.uart_bytes.load(std::memory_order_relaxed);
    const uint32_t expected = rounds * kChunk;
    const bool ok = got == expected && copies == rounds;
    printf(
        "  UART%d -> UART%d  %s  (%u/%u bytes, %zu/%u intact)\n", tx_port, rx_port,
        ok ? "PASS" : "FAIL", got, expected, copies, rounds);
    if (!ok && !sink.uart_seen.empty()) {
        const size_t dump = std::min<size_t>(sink.uart_seen.size(), 24);
        printf("      rx:");
        for (size_t i = 0; i < dump; ++i)
            printf(" %02X", static_cast<unsigned>(static_cast<uint8_t>(sink.uart_seen[i])));
        printf("\n      tx: 52 4D 43 53 2D 4C 4F 4F 50 ...\n");
    }
    return ok ? 0 : 1;
}

// arbitrate: three controllers on one segment all transmitting at once. CAN
// arbitration is supposed to serialise them without loss -- a lower-priority
// frame backs off and retries rather than being corrupted. With
// disable_auto_retransmission set on the 5321 side, a lost arbitration there is
// a real drop, so this also measures what that setting costs on a shared bus.
int run_arbitrate(uint32_t frames_each, uint32_t rate_hz) {
    Rig rig;
    if (!rig.open())
        return 1;

    printf(
        "\n3-node arbitration: A.CAN1 + B.CAN1 + B.CAN2 all sending %u frames at %u Hz (%s)\n",
        frames_each, rate_hz, "frames follow the bus mode");

    // Distinct CAN ids so each sender's frames are attributable, and so the
    // arbitration order is deterministic (lower id wins).
    std::atomic<bool> running{true};
    std::atomic<uint64_t> sent_b1{0}, sent_b2{0};
    const auto period = std::chrono::nanoseconds{1'000'000'000ULL / rate_hz};

    // One sender thread per BOARD, never one per bus: BoardSession::transmit()
    // takes a builder over the session's shared downlink buffer with no mutex, so
    // two threads driving the same board interleave their writes and one
    // overwrites the other's not-yet-flushed frame. That is what made B.CAN2
    // appear to deliver 3/100 while B.CAN1 delivered 97/100 -- the two tallies
    // summed to exactly one sender's worth, because only one frame per round
    // survived the race. A board's several buses go out in ONE transmit() call,
    // the way run_both() already does it.
    auto sender = [&](examples::BoardSession& board, std::span<const int> buses,
                      std::span<const uint32_t> ids,
                      std::span<std::atomic<uint64_t>* const> counters) {
        auto next = Clock::now();
        for (uint32_t i = 0; i < frames_each && running.load(std::memory_order_relaxed); ++i) {
            std::byte payload[kPayloadSize];
            put_u32_le(payload, i);
            put_u32_le(payload + 4, mix(i));
            try {
                board.transmit([&](examples::BoardTransmitter& tx) {
                    for (size_t k = 0; k < buses.size(); ++k)
                        tx.can(buses[k], {.can_id = ids[k], .can_data = payload});
                });
                for (auto* counter : counters)
                    counter->fetch_add(1, std::memory_order_relaxed);
            } catch (const std::exception&) {
            }
            next += period;
            while (Clock::now() < next) {}
        }
    };

    // Attribute by sender id: with mc02's CAN2<->CAN3 also strapped together,
    // A.CAN1 / B.CAN1 / B.CAN2 share ONE physical segment, so a per-bus counter
    // merges all of them and cannot say who sent what.
    constexpr uint32_t kIdA = kCanIdBase + 0x10;
    constexpr uint32_t kIdB1 = kCanIdBase + 0x11;
    constexpr uint32_t kIdB2 = kCanIdBase + 0x12;
    for (Sink* sink : {&rig.sink_a, &rig.sink_b}) {
        sink->reset_can_ids();
        sink->watch_can_id(kIdA);
        sink->watch_can_id(kIdB1);
        sink->watch_can_id(kIdB2);
    }

    // mc02 puts both of its buses on the segment from a single thread/packet.
    const int b_buses[]{1, 2};
    const uint32_t b_ids[]{kIdB1, kIdB2};
    std::atomic<uint64_t>* const b_counters[]{&sent_b1, &sent_b2};
    std::thread t2{[&]() { sender(*rig.b, b_buses, b_ids, b_counters); }};

    // The 5321 sends from this thread -- a different board object, so no sharing.
    std::atomic<uint64_t> sent_a{0};
    const int a_buses[]{1};
    const uint32_t a_ids[]{kIdA};
    std::atomic<uint64_t>* const a_counters[]{&sent_a};
    sender(*rig.a, a_buses, a_ids, a_counters);
    t2.join();
    running.store(false, std::memory_order_relaxed);
    std::this_thread::sleep_for(std::chrono::milliseconds{500});

    // A's sink counts everything mc02 put on the segment (both its buses);
    // B's sinks count what A put on it, once per mc02 controller.
    printf(
        "  sent: A.CAN1=%llu  B.CAN1=%llu  B.CAN2=%llu\n",
        static_cast<unsigned long long>(sent_a.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(sent_b1.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(sent_b2.load(std::memory_order_relaxed)));
    const uint32_t a_from_b1 = rig.sink_a.hits_for_id(kIdB1);
    const uint32_t a_from_b2 = rig.sink_a.hits_for_id(kIdB2);
    const uint32_t b_from_a = rig.sink_b.hits_for_id(kIdA);
    const uint64_t exp_b1 = sent_b1.load(std::memory_order_relaxed);
    const uint64_t exp_b2 = sent_b2.load(std::memory_order_relaxed);
    const uint64_t exp_a = sent_a.load(std::memory_order_relaxed);

    printf(
        "  A.CAN1 got %u/%llu from B.CAN1 + %u/%llu from B.CAN2 (by CAN id)\n", a_from_b1,
        static_cast<unsigned long long>(exp_b1), a_from_b2,
        static_cast<unsigned long long>(exp_b2));
    // mc02's own two controllers are on the same segment, so each of them sees
    // A's frames once -- and also each other's, which is expected, not loss.
    printf(
        "  B got %u/%llu from A.CAN1 (across its %u+%u own-segment controllers)\n", b_from_a,
        static_cast<unsigned long long>(exp_a),
        rig.sink_b.hits[1].load(std::memory_order_relaxed),
        rig.sink_b.hits[2].load(std::memory_order_relaxed));
    printf(
        "  corrupt: A=%llu B=%llu   unattributed: A=%u B=%u\n",
        static_cast<unsigned long long>(rig.sink_a.corrupt.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(rig.sink_b.corrupt.load(std::memory_order_relaxed)),
        rig.sink_a.hits_other_id(), rig.sink_b.hits_other_id());

    // Arbitration is supposed to serialise, not drop. Anything missing here is a
    // real loss now that each sender is counted separately.
    const bool ok = a_from_b1 == exp_b1 && a_from_b2 == exp_b2 && b_from_a >= exp_a
                 && rig.sink_a.corrupt.load(std::memory_order_relaxed) == 0
                 && rig.sink_b.corrupt.load(std::memory_order_relaxed) == 0;
    printf("arbitrate: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

void print_usage() {
    fprintf(
        stderr,
        "usage: mixed_board_test <mode> [args]\n"
        "  list                                   enumerate boards with USB speed\n"
        "  link                                   discover the CAN/UART wiring\n"
        "  latency <tx_bus> <rx_bus> [samples]    A -> B one-way latency\n"
        "  rate    <tx_bus> <rx_bus> [seconds] [frames_per_packet]\n"
        "  env: HCS_BOARD_A / HCS_BOARD_B select boards by USB serial\n");
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage();
        return 1;
    }
    const std::string_view mode{argv[1]};
    const auto argument = [&](int index, long fallback) -> long {
        return argc > index ? std::strtol(argv[index], nullptr, 0) : fallback;
    };

    try {
        if (mode == "list")
            return run_list();
        if (!discover())
            return 1;
        if (mode == "link")
            return run_link();
        if (mode == "latency")
            return run_latency(
                static_cast<int>(argument(2, 0)), static_cast<int>(argument(3, 0)),
                static_cast<uint32_t>(argument(4, 2000)));
        if (mode == "txprobe")
            return run_txprobe(
                static_cast<int>(argument(2, 0)), static_cast<int>(argument(3, 0)),
                static_cast<uint32_t>(argument(4, 5)));
        if (mode == "both")
            return run_both(
                static_cast<uint32_t>(argument(2, 4000)), static_cast<uint32_t>(argument(3, 5)));
        if (mode == "arbitrate")
            return run_arbitrate(
                static_cast<uint32_t>(argument(2, 2000)), static_cast<uint32_t>(argument(3, 1000)));
        if (mode == "canpack")
            return run_canpack(
                static_cast<int>(argument(2, 1)), static_cast<int>(argument(3, 1)),
                static_cast<int>(argument(4, 2)), static_cast<uint32_t>(argument(5, 50)),
                static_cast<uint32_t>(argument(6, 3)), argument(7, 0) != 0);
        if (mode == "canloop")
            return run_canloop(
                static_cast<int>(argument(2, 1)), static_cast<int>(argument(3, 1)),
                static_cast<int>(argument(4, 2)), static_cast<uint32_t>(argument(5, 50)));
        if (mode == "uartloop")
            return run_uartloop(
                static_cast<int>(argument(2, 1)), static_cast<int>(argument(3, 1)),
                static_cast<int>(argument(4, 2)), static_cast<uint32_t>(argument(5, 0)),
                static_cast<uint32_t>(argument(6, 20)));
        if (mode == "gpioedge")
            return run_gpioedge(
                static_cast<int>(argument(2, 1)), static_cast<int>(argument(3, 0)),
                static_cast<int>(argument(4, 1)), static_cast<uint32_t>(argument(5, 10)));
        if (mode == "pwmloop")
            return run_pwmloop(
                static_cast<int>(argument(2, 1)), static_cast<int>(argument(3, 0)),
                static_cast<int>(argument(4, 1)));
        if (mode == "gpioloop")
            return run_gpioloop(
                static_cast<int>(argument(2, 1)), static_cast<int>(argument(3, 0)),
                static_cast<int>(argument(4, 1)), static_cast<uint32_t>(argument(5, 6)));
        if (mode == "gpio")
            return run_gpio(static_cast<int>(argument(2, 1)));
        if (mode == "uartbaud")
            return run_uartbaud(
                static_cast<int>(argument(2, 0)), static_cast<uint32_t>(argument(3, 230400)));
        if (mode == "uart")
            return run_uart(
                static_cast<uint32_t>(argument(2, 115200)),
                static_cast<uint32_t>(argument(3, 20)));
        if (mode == "uartrate")
            return run_uartrate(
                static_cast<int>(argument(2, 0)), static_cast<int>(argument(3, 0)),
                static_cast<uint32_t>(argument(4, 3)));
        if (mode == "rate")
            return run_rate(
                static_cast<int>(argument(2, 0)), static_cast<int>(argument(3, 0)),
                static_cast<uint32_t>(argument(4, 5)), static_cast<int>(argument(5, 1)));
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }
    print_usage();
    return 1;
}
