// Two-board conformance and latency suite for a pair of HPM5321 DualCan boards
// wired to each other:
//
//   board A CAN1 <-> board B CAN1      (one terminated bus)
//   board A CAN2 <-> board B CAN2      (a second, independent bus)
//   board A UART0 TX/RX <-> board B UART0 RX/TX
//
// Every existing example assumes ONE board with a loopback cable between two of
// its own buses, which cannot distinguish "the bridge works" from "the bridge
// talks to itself". With two boards each frame crosses a real bus between two
// independent CAN controllers with independent clocks, so bit timing, sample
// point and the arbitration/data-phase switch are exercised for real.
//
// Both boards carry the same USB VID:PID, so each one is selected by its USB
// serial (see `dual_board_test list`).
//
// Run:
//   ./dual_board_test list
//   ./dual_board_test link
//   ./dual_board_test latency [bus] [samples] [core_a] [core_b] [core_main]
//   ./dual_board_test dual    [samples]
//   ./dual_board_test uart    [rounds]
//   ./dual_board_test stress  [frames_per_sec] [seconds]
//
// HCS_CAN_CLASSIC=1 switches the CAN payload from CAN-FD (1 Mbit/s
// arbitration + 5 Mbit/s data with BRS) to classic CAN. Differencing the two
// runs separates wire time from the rest of the path, which no single run can.
//
// HCS_CAN_CROSSED=1 is for a rig whose two CAN cables are swapped (A.CAN1 wired
// to B.CAN2 and A.CAN2 to B.CAN1). `link` diagnoses that case by name; this flag
// then lets `latency` measure through it, because the crossing changes only
// which controller receives -- the electrical path and both firmware paths are
// unchanged, and the measured numbers match a correctly wired rig. `link` itself
// deliberately keeps FAILing so the miswiring never becomes invisible.
//
// HCS_LATENCY_GAP_US sets the idle time between latency samples. The default 0
// sends the next frame as soon as the echo lands -- a ~10 kHz ping-pong that
// never lets the core idle, which makes the tool BLIND to every host setting
// that only bites once it does. Measured on this rig: at gap 0 the CPU governor
// changes nothing (p50 100.6 either way), at gap 1000 it is worth 16-18 us of
// p50 (120 -> 102) because the extra host latency makes the reply miss its USB
// microframe and wait for the next. Set it to 1000 to reproduce a 1 kHz control
// loop before drawing any conclusion about host tuning. See HOST_TUNING.md 1.2.

#include <cmath>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

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
constexpr uint32_t kWarmupSamples = 100;
constexpr auto kEchoTimeout = std::chrono::milliseconds{20};

// The two boards on this rig, by USB serial. Filled in by `discover()` so the
// tool keeps working when the pair is swapped for another one.
std::string g_serial_a;
std::string g_serial_b;

// The HCS_CAN_CLASSIC opt-out this tool used to honour is gone with the
// per-frame is_fdcan flag: the frame type is a property of the BUS, fixed by
// the firmware's port table, and every bus on this rig's Hpm5321 boards runs
// CAN-FD. Frames go out in the bus's mode and the host cannot -- and need not
// -- choose.

// Opt-in for a rig whose two CAN cables are swapped (A.CAN1 wired to B.CAN2 and
// A.CAN2 to B.CAN1). Frames still cross a real bus between two independent
// controllers, so the measurement stays valid -- only the landing bus index
// moves. Deliberately NOT auto-detected: `link` must keep failing on a miswired
// rig, otherwise the tool would hide the very defect it exists to find.
bool crossed_rig() {
    const char* crossed = std::getenv("HCS_CAN_CROSSED");
    return crossed != nullptr && crossed[0] == '1';
}

// Idle time inserted between latency samples, in microseconds. 0 (the default)
// is a back-to-back ping-pong, which measures the link at full tilt but keeps
// the core busy; a real 1 kHz control loop leaves it idle 99% of the time and
// behaves differently under CPU frequency scaling. See HOST_TUNING.md 1.1.
// Spin the gap out instead of sleeping through it. Same duty cycle either way;
// the difference is whether the measuring thread pays a scheduler wake-up on
// every sample. Separating those two is the only way to tell a cold USB pipe
// from a cold application thread -- both look like "p50 rises when the loop
// goes idle", and only one of them is fixable by the application.
bool latency_gap_spin() {
    const char* raw = std::getenv("HCS_LATENCY_GAP_SPIN");
    return raw != nullptr && raw[0] == '1';
}

long latency_gap_us() {
    const char* raw = std::getenv("HCS_LATENCY_GAP_US");
    if (raw == nullptr || *raw == '\0')
        return 0;
    char* end = nullptr;
    const long value = std::strtol(raw, &end, 10);
    if (end == raw || *end != '\0' || value < 0)
        return 0;
    return value;
}

void idle_for(long gap_us) {
    if (gap_us <= 0)
        return;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::microseconds{gap_us};
    if (!latency_gap_spin()) {
        std::this_thread::sleep_for(std::chrono::microseconds{gap_us});
        return;
    }
    while (std::chrono::steady_clock::now() < deadline) { }
}

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

    if (priority <= 0)
        return; // name-only call: the SDK option owns affinity and priority

    sched_param parameter{};
    parameter.sched_priority = priority;
    if (sched_setscheduler(0, SCHED_FIFO, &parameter) != 0)
        perror("sched_setscheduler");
}

double percentile(const std::vector<double>& sorted, double fraction) {
    if (sorted.empty())
        return 0.0;
    const size_t index = static_cast<size_t>(fraction * static_cast<double>(sorted.size() - 1));
    return sorted[index];
}

struct Stats {
    double min = 0.0, p50 = 0.0, p90 = 0.0, p99 = 0.0, p999 = 0.0, avg = 0.0, max = 0.0;
    size_t count = 0;
};

// HCS_LATENCY_HIST=1 prints the shape behind the percentiles. Percentiles
// cannot distinguish "one mode with a long tail" from "two modes", and that
// distinction decides whether a fix should attack a rate (how often the slow
// path is taken) or a magnitude (how slow it is). Buckets are 2 us, which is
// well under the 15.6 us host service interval that separates the modes.
void print_histogram(const std::vector<double>& sorted) {
    if (const char* on = std::getenv("HCS_LATENCY_HIST"); !on || on[0] != '1')
        return;
    if (sorted.size() < 2)
        return;

    constexpr double kBucketUs = 2.0;
    const double lo = std::floor(sorted.front() / kBucketUs) * kBucketUs;
    const double hi = std::ceil(sorted.back() / kBucketUs) * kBucketUs;
    const size_t buckets = static_cast<size_t>((hi - lo) / kBucketUs) + 1;
    std::vector<size_t> counts(buckets, 0);
    for (const double value : sorted)
        counts[static_cast<size_t>((value - lo) / kBucketUs)]++;

    const size_t peak = *std::max_element(counts.begin(), counts.end());
    for (size_t i = 0; i < buckets; ++i) {
        if (counts[i] == 0)
            continue;
        const size_t bar = peak ? counts[i] * 60 / peak : 0;
        printf(
            "  %6.1f  %6zu  %5.2f%%  %s\n", lo + static_cast<double>(i) * kBucketUs, counts[i],
            100.0 * static_cast<double>(counts[i]) / static_cast<double>(sorted.size()),
            std::string(bar, '#').c_str());
    }
}

Stats summarize(std::vector<double> values) {
    Stats stats;
    if (values.empty())
        return stats;
    std::sort(values.begin(), values.end());
    double sum = 0.0;
    for (const double value : values)
        sum += value;
    stats.count = values.size();
    stats.min = values.front();
    stats.p50 = percentile(values, 0.50);
    stats.p90 = percentile(values, 0.90);
    stats.p99 = percentile(values, 0.99);
    stats.p999 = percentile(values, 0.999);
    stats.avg = sum / static_cast<double>(values.size());
    stats.max = values.back();
    print_histogram(values);
    return stats;
}

void print_stats(const char* label, const Stats& stats) {
    printf(
        "%-22s n=%-7zu min %7.1f  p50 %7.1f  p90 %7.1f  p99 %7.1f  p99.9 %7.1f  avg %7.1f  max "
        "%7.1f\n",
        label, stats.count, stats.min, stats.p50, stats.p90, stats.p99, stats.p999, stats.avg,
        stats.max);
}

// A board plus the receive hooks the tests need. Each board runs its own
// transport event thread, so a two-board test has two of them; `io_core` pins
// that thread and is what keeps the two boards off each other's core.
// AdvancedOptions is deliberately non-copyable and its `thread_setup` hook is a
// plain function pointer taking the options object back, so the per-board core
// travels in a derived options type rather than in a capturing lambda.
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
        // Affinity and priority now go through the SDK option rather than this
        // tool's own thread_setup, so what the tool measures is the same code
        // path an application gets from set_io_thread_affinity(). The hook is
        // kept for the thread NAME, which the SDK deliberately does not touch.
        if (io_core >= 0)
            options_.set_io_thread_affinity(io_core, 90);
        options_.thread_setup = [](const libhcs::board::AdvancedOptions& self) noexcept {
            const auto& options = static_cast<const NodeOptions&>(self);
            configure_thread(-1, 0, options.thread_name);
        };
        board_ = std::make_unique<Board>(*this, serial, options_);
    }

    Board& board() { return *board_; }

    // Per-bus receive sinks. A test installs a handler on the bus it watches;
    // frames arriving on an unwatched bus are counted, not dropped silently,
    // so cross-bus leakage shows up instead of being invisible.
    using CanHandler = void (*)(void*, int bus, const libhcs::data::CanDataView&);
    using UartHandler = void (*)(void*, const libhcs::data::UartDataView&);

    void set_can_handler(void* context, CanHandler handler) {
        can_context_ = context;
        can_handler_.store(handler, std::memory_order_release);
    }
    void set_uart_handler(void* context, UartHandler handler) {
        uart_context_ = context;
        uart_handler_.store(handler, std::memory_order_release);
    }

    uint64_t unhandled_can() const { return unhandled_can_.load(std::memory_order_relaxed); }
    uint64_t can_count(int bus) const { return can_count_[bus].load(std::memory_order_relaxed); }

private:
    void can_receive(
        CanPort port, const libhcs::data::CanDataView& data) override {
        switch (port) {
        case CanPort::kCan1: {
            dispatch_can(0, data);
            break;
        }
        case CanPort::kCan2: {
            dispatch_can(1, data);
            break;
        }
        default: break;
        }
    }
    void uart0_receive_callback(const libhcs::data::UartDataView& data) override {
        const auto handler = uart_handler_.load(std::memory_order_acquire);
        if (handler)
            handler(uart_context_, data);
    }

    void dispatch_can(int bus, const libhcs::data::CanDataView& data) {
        can_count_[bus].fetch_add(1, std::memory_order_relaxed);
        const auto handler = can_handler_.load(std::memory_order_acquire);
        if (handler)
            handler(can_context_, bus, data);
        else
            unhandled_can_.fetch_add(1, std::memory_order_relaxed);
    }

    NodeOptions options_;
    std::unique_ptr<Board> board_;

    void* can_context_ = nullptr;
    void* uart_context_ = nullptr;
    std::atomic<CanHandler> can_handler_{nullptr};
    std::atomic<UartHandler> uart_handler_{nullptr};
    std::atomic<uint64_t> unhandled_can_{0};
    std::atomic<uint64_t> can_count_[2]{};
};

void transmit_can(Board& board, int bus, const libhcs::data::CanDataView& data) {
    auto builder = board.start_transmit();
    if (bus == 0)
        builder.can_transmit(CanPort::kCan1, data);
    else
        builder.can_transmit(CanPort::kCan2, data);
}

} // namespace

// ---------------------------------------------------------------------------
// list: enumerate the attached boards so the other modes can name them.
// ---------------------------------------------------------------------------

#include <dirent.h>


namespace {

// The USB serials are read from sysfs rather than from libusb, because the
// board handle needed for a libusb query is exactly what we are trying to pick.
std::vector<std::pair<std::string, std::string>> enumerate_boards() {
    std::vector<std::pair<std::string, std::string>> found; // serial, product
    DIR* dir = opendir("/sys/bus/usb/devices");
    if (!dir)
        return found;
    while (const dirent* entry = readdir(dir)) {
        const std::string base = std::string{"/sys/bus/usb/devices/"} + entry->d_name;
        const auto read_line = [&](const char* leaf) -> std::string {
            std::string path = base + "/" + leaf;
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
        const std::string serial = read_line("serial");
        if (!serial.empty())
            found.emplace_back(serial, read_line("product"));
    }
    closedir(dir);
    std::sort(found.begin(), found.end());
    return found;
}

bool discover() {
    const auto found = enumerate_boards();
    if (found.size() < 2) {
        fprintf(stderr, "need two hpm5321_dual_can boards (a511:5322), found %zu\n", found.size());
        return false;
    }
    // Deterministic A/B assignment: sorted serial order. HCS_BOARD_A /
    // HCS_BOARD_B override it when the physical wiring says otherwise.
    g_serial_a = found[0].first;
    g_serial_b = found[1].first;
    if (const char* value = std::getenv("HCS_BOARD_A"))
        g_serial_a = value;
    if (const char* value = std::getenv("HCS_BOARD_B"))
        g_serial_b = value;
    return true;
}

int run_list() {
    const auto found = enumerate_boards();
    printf("hpm5321_dual_can boards attached: %zu\n", found.size());
    for (const auto& [serial, product] : found)
        printf("  %s   %s\n", serial.c_str(), product.c_str());
    return found.size() >= 2 ? 0 : 1;
}

// ---------------------------------------------------------------------------
// link: prove every wired path carries data, in both directions.
// ---------------------------------------------------------------------------

struct LinkProbe {
    std::atomic<uint32_t> received{0};
    std::atomic<uint32_t> matched{0};
    std::atomic<int> last_bus{-1};
    std::atomic<uint32_t> last_value{0};
    std::atomic<bool> saw_timestamp{false};
};

void link_can_handler(void* context, int bus, const libhcs::data::CanDataView& data) {
    auto* probe = static_cast<LinkProbe*>(context);
    probe->received.fetch_add(1, std::memory_order_relaxed);
    probe->last_bus.store(bus, std::memory_order_relaxed);
    if (data.timestamp_us.has_value())
        probe->saw_timestamp.store(true, std::memory_order_relaxed);
    if (data.can_data.size() == kPayloadSize) {
        const uint32_t value = get_u32_le(data.can_data.data());
        if (get_u32_le(data.can_data.data() + 4) == mix(value)) {
            probe->last_value.store(value, std::memory_order_relaxed);
            probe->matched.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

bool link_can_direction(
    Node& source, Node& sink, int bus, const char* label, uint32_t token) {
    LinkProbe probe;
    sink.set_can_handler(&probe, link_can_handler);
    source.set_can_handler(nullptr, nullptr);

    std::byte payload[kPayloadSize];
    put_u32_le(payload, token);
    put_u32_le(payload + 4, mix(token));

    constexpr uint32_t kAttempts = 20;
    for (uint32_t attempt = 0; attempt < kAttempts; ++attempt) {
        transmit_can(
            source.board(), bus,
            {.can_id = kCanIdBase + static_cast<uint32_t>(bus),
             .can_data = payload});
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
        if (probe.matched.load(std::memory_order_relaxed) > 0)
            break;
    }

    const uint32_t matched = probe.matched.load(std::memory_order_relaxed);
    const uint32_t received = probe.received.load(std::memory_order_relaxed);
    const int landed = probe.last_bus.load(std::memory_order_relaxed);
    const bool ok =
        matched > 0 && landed == bus && probe.last_value.load(std::memory_order_relaxed) == token;
    printf(
        "  %-26s %s  (rx=%u matched=%u landed_on_can=%d hw_timestamp=%s)\n", label,
        // Reported as the enclosure numbers it; -1 still means "nowhere".
        ok ? "PASS" : "FAIL", received, matched, landed < 0 ? -1 : landed + 1,
        probe.saw_timestamp.load(std::memory_order_relaxed) ? "yes" : "no");
    // A frame that arrived intact on the OTHER bus is a wiring fault, not a
    // firmware fault: the payload and the hardware timestamp are both good, only
    // the pair of cables is swapped. Say so, because the raw FAIL above sends
    // people hunting through the CAN driver instead of looking at the desk.
    if (matched > 0 && landed == 1 - bus)
        printf(
            "  %-26s   -> intact frame landed on CAN%d: the two CAN cables are\n"
            "  %-26s      swapped. Swap them at ONE board, or set HCS_CAN_CROSSED=1\n"
            "  %-26s      to measure through the crossing anyway.\n",
            "", landed + 1, "", "");
    sink.set_can_handler(nullptr, nullptr);
    return ok;
}

struct UartProbe {
    std::atomic<uint32_t> bytes{0};
    std::atomic<uint32_t> matches{0};
    std::string expected;
    std::string accumulated;
};

void link_uart_handler(void* context, const libhcs::data::UartDataView& data) {
    auto* probe = static_cast<UartProbe*>(context);
    probe->bytes.fetch_add(static_cast<uint32_t>(data.uart_data.size()), std::memory_order_relaxed);
    for (const std::byte b : data.uart_data)
        probe->accumulated.push_back(static_cast<char>(std::to_integer<uint8_t>(b)));
    if (probe->accumulated.find(probe->expected) != std::string::npos)
        probe->matches.fetch_add(1, std::memory_order_relaxed);
}

bool link_uart_direction(Node& source, Node& sink, const char* label) {
    UartProbe probe;
    probe.expected = "HCS-LINK-PROBE";
    sink.set_uart_handler(&probe, link_uart_handler);

    const std::string text = probe.expected + "\n";
    for (uint32_t attempt = 0; attempt < 20; ++attempt) {
        auto builder = source.board().start_transmit();
        builder.uart0_transmit(
            {.uart_data = std::as_bytes(std::span{text.data(), text.size()}),
             .idle_delimited = true});
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
        if (probe.matches.load(std::memory_order_relaxed) > 0)
            break;
    }

    const bool ok = probe.matches.load(std::memory_order_relaxed) > 0;
    printf(
        "  %-26s %s  (rx_bytes=%u)\n", label, ok ? "PASS" : "FAIL",
        probe.bytes.load(std::memory_order_relaxed));
    if (!ok && !probe.accumulated.empty()) {
        // Bytes arrived but did not match: dump the head so a baud/framing
        // problem is distinguishable from a wiring problem at a glance.
        const size_t dump = std::min<size_t>(probe.accumulated.size(), 32);
        printf("      first %zu rx bytes:", dump);
        for (size_t i = 0; i < dump; ++i)
            printf(" %02X", static_cast<unsigned>(static_cast<uint8_t>(probe.accumulated[i])));
        printf("\n      expected        :");
        for (size_t i = 0; i < std::min<size_t>(probe.expected.size(), 32); ++i)
            printf(" %02X", static_cast<unsigned>(static_cast<uint8_t>(probe.expected[i])));
        printf("\n");
    }
    sink.set_uart_handler(nullptr, nullptr);
    return ok;
}

int run_link() {
    Node board_a{g_serial_a, -1, "hcs-a"};
    Node board_b{g_serial_b, -1, "hcs-b"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    printf("link check (frames follow the bus mode: CAN-FD 1M/5M BRS)\n");
    // CAN numbers everywhere in this tool are the ones printed on the
    // enclosure, 1-based. They used to be the 0-based logical index, which cost
    // an afternoon of physical-layer debugging on a port that had no cable in
    // it, because "CAN1" meant different connectors to the two people saying it.
    bool all_ok = true;
    all_ok &= link_can_direction(
        board_a, board_b, 0, "A.CAN1 -> B.CAN1", 0x11110000);
    all_ok &= link_can_direction(
        board_b, board_a, 0, "B.CAN1 -> A.CAN1", 0x22220000);
    all_ok &= link_can_direction(
        board_a, board_b, 1, "A.CAN2 -> B.CAN2", 0x33330000);
    all_ok &= link_can_direction(
        board_b, board_a, 1, "B.CAN2 -> A.CAN2", 0x44440000);
    all_ok &= link_uart_direction(board_a, board_b, "A.UART0 -> B.UART0");
    all_ok &= link_uart_direction(board_b, board_a, "B.UART0 -> A.UART0");

    printf("link: %s\n", all_ok ? "PASS" : "FAIL");
    return all_ok ? 0 : 1;
}

// ---------------------------------------------------------------------------
// latency: host -> A.CANx -> wire -> B.CANx -> host, one traversal (x is 1 or 2).
//
// This is the number a distributed controller actually pays: one USB downlink,
// one CAN frame on a real bus between two chips, one USB uplink. The single
// board loopback tools cannot measure it because both ends share one chip.
//
// Board B stamps arrival in hardware (PTPC, 1 us resolution). Subtracting the
// per-run minimum of (host_receive - board_stamp) yields the USB uplink jitter
// alone, with no clock synchronisation between host and board required: the
// two clocks differ by a constant offset plus drift, and the minimum over a
// short run absorbs the offset.
// ---------------------------------------------------------------------------

class LatencyProbe {
public:
    void arm(uint32_t sequence, Clock::time_point send_time) {
        while ((state_.load(std::memory_order_acquire) & kStateMask) == kWritingState) {}
        send_time_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(send_time.time_since_epoch())
                .count(),
            std::memory_order_relaxed);
        state_.store(static_cast<uint64_t>(sequence) << kStateBits, std::memory_order_release);
    }

    bool wait(uint32_t sequence, double& latency_us, double& uplink_us, bool& has_uplink) {
        const auto deadline = Clock::now() + kEchoTimeout;
        const uint64_t received_state =
            (static_cast<uint64_t>(sequence) << kStateBits) | kReceivedState;
        while (state_.load(std::memory_order_acquire) != received_state) {
            if (Clock::now() >= deadline)
                return false;
        }
        latency_us = static_cast<double>(latency_ns_.load(std::memory_order_relaxed)) / 1e3;
        has_uplink = has_uplink_.load(std::memory_order_relaxed);
        uplink_us = uplink_us_.load(std::memory_order_relaxed);
        return true;
    }

    // Where a frame is expected to LAND, and the id it must carry. On a correctly
    // wired rig they are the same bus, which is why they are set together. On a
    // rig whose two CAN cables are swapped the frame lands on the other
    // controller while the id still keys off the TRANSMIT bus, so the two have to
    // be decoupled -- see set_landing_bus() and HCS_CAN_CROSSED.
    void set_expected_bus(int bus) {
        expected_bus_ = bus;
        expected_id_ = kCanIdBase + static_cast<uint32_t>(bus);
    }
    void set_landing_bus(int bus) { expected_bus_ = bus; }

    uint64_t invalid() const { return invalid_.load(std::memory_order_relaxed); }
    uint64_t unexpected() const { return unexpected_.load(std::memory_order_relaxed); }
    uint64_t wrong_bus() const { return wrong_bus_.load(std::memory_order_relaxed); }

    static void handler(void* context, int bus, const libhcs::data::CanDataView& data) {
        static_cast<LatencyProbe*>(context)->on_can(bus, data);
    }

private:
    void on_can(int bus, const libhcs::data::CanDataView& data) {
        const auto receive_time = Clock::now();
        if (bus != expected_bus_) {
            wrong_bus_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (data.can_id != expected_id_ || data.is_extended_can_id
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
        latency_ns_.store(
            receive_time_ns - send_time_ns_.load(std::memory_order_relaxed),
            std::memory_order_relaxed);

        if (data.timestamp_us.has_value()) {
            // Wrap-safe difference in microseconds between the host's arrival
            // clock and the board's hardware stamp. Only differences between
            // samples are meaningful, so the constant offset is left in and
            // removed by the caller.
            const auto board_us = static_cast<double>(*data.timestamp_us);
            uplink_us_.store(
                static_cast<double>(receive_time_ns) / 1e3 - board_us, std::memory_order_relaxed);
            has_uplink_.store(true, std::memory_order_relaxed);
        } else {
            has_uplink_.store(false, std::memory_order_relaxed);
        }

        state_.store(
            (static_cast<uint64_t>(sequence) << kStateBits) | kReceivedState,
            std::memory_order_release);
    }

    static constexpr unsigned kStateBits = 2;
    static constexpr uint64_t kStateMask = (1U << kStateBits) - 1U;
    static constexpr uint64_t kWritingState = 1;
    static constexpr uint64_t kReceivedState = 2;

    int expected_bus_ = 0;
    uint32_t expected_id_ = kCanIdBase;
    std::atomic<uint64_t> state_{kReceivedState};
    std::atomic<int64_t> send_time_ns_{0};
    std::atomic<int64_t> latency_ns_{0};
    std::atomic<double> uplink_us_{0.0};
    std::atomic<bool> has_uplink_{false};
    std::atomic<uint64_t> invalid_{0};
    std::atomic<uint64_t> unexpected_{0};
    std::atomic<uint64_t> wrong_bus_{0};
};

int run_latency(int bus, uint32_t samples, int core_a, int core_b, int core_main) {
    Node board_a{g_serial_a, core_a, "hcs-a"};
    Node board_b{g_serial_b, core_b, "hcs-b"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    configure_thread(core_main, 80, "dual-main");

    LatencyProbe probe;
    probe.set_expected_bus(bus);
    const int landing_bus = crossed_rig() ? 1 - bus : bus;
    probe.set_landing_bus(landing_bus);
    board_b.set_can_handler(&probe, LatencyProbe::handler);

    printf(
        "A.CAN%d -> B.CAN%d, %u samples after %u warm-up (frames follow the bus mode: "
        "CAN-FD 1M/5M BRS)%s\n", bus + 1,
        landing_bus + 1, samples,
        kWarmupSamples,
        crossed_rig() ? "  [HCS_CAN_CROSSED=1: swapped-cable rig]" : "");

    std::vector<double> latencies;
    std::vector<double> uplinks;
    latencies.reserve(samples);
    uplinks.reserve(samples);
    uint32_t timeouts = 0;
    Clock::time_point measure_start{};
    Clock::time_point measure_end{};

    for (uint32_t sequence = 0; sequence < samples + kWarmupSamples; ++sequence) {
        std::byte payload[kPayloadSize];
        put_u32_le(payload, sequence);
        put_u32_le(payload + 4, mix(sequence));

        if (sequence == kWarmupSamples)
            measure_start = Clock::now();
        probe.arm(sequence, Clock::now());
        transmit_can(
            board_a.board(), bus,
            {.can_id = kCanIdBase + static_cast<uint32_t>(bus),
             .can_data = payload});

        double latency_us = 0.0;
        double uplink_us = 0.0;
        bool has_uplink = false;
        if (!probe.wait(sequence, latency_us, uplink_us, has_uplink)) {
            if (sequence >= kWarmupSamples)
                ++timeouts;
            continue;
        }
        if (sequence >= kWarmupSamples) {
            latencies.push_back(latency_us);
            if (has_uplink)
                uplinks.push_back(uplink_us);
        }
        // Idle the core between samples when asked. Back-to-back (the default)
        // keeps the CPU busy enough to hold its own clock up, which makes this
        // tool blind to every host setting that only bites once the core sleeps.
        // Set this to 1000 to reproduce a 1 kHz control loop's duty cycle.
        idle_for(latency_gap_us());
    }
    measure_end = Clock::now();
    const double measured_span_us =
        std::chrono::duration<double, std::micro>(measure_end - measure_start).count();

    if (latencies.empty()) {
        fprintf(
            stderr, "no frames crossed (invalid=%llu unexpected=%llu wrong_bus=%llu)\n",
            static_cast<unsigned long long>(probe.invalid()),
            static_cast<unsigned long long>(probe.unexpected()),
            static_cast<unsigned long long>(probe.wrong_bus()));
        return 2;
    }

    printf(
        "delivered=%zu timeout=%u invalid=%llu unexpected=%llu wrong_bus=%llu\n", latencies.size(),
        timeouts, static_cast<unsigned long long>(probe.invalid()),
        static_cast<unsigned long long>(probe.unexpected()),
        static_cast<unsigned long long>(probe.wrong_bus()));
    print_stats("A->B latency us", summarize(latencies));

    if (uplinks.size() >= 2) {
        // host_clock - board_clock drifts linearly: the two oscillators differ
        // in rate, and the board's microsecond conversion carries a fixed
        // scale error of its own. Subtracting only the offset would report
        // that drift as jitter (it dominates it), so fit and remove the linear
        // trend; the residual is the actual variation of the
        // board-stamp -> host-callback path, i.e. USB uplink jitter.
        const size_t n = uplinks.size();
        double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const double x = static_cast<double>(i);
            sum_x += x;
            sum_y += uplinks[i];
            sum_xx += x * x;
            sum_xy += x * uplinks[i];
        }
        const double count = static_cast<double>(n);
        const double denominator = count * sum_xx - sum_x * sum_x;
        const double slope =
            denominator != 0.0 ? (count * sum_xy - sum_x * sum_y) / denominator : 0.0;
        const double intercept = (sum_y - slope * sum_x) / count;

        std::vector<double> residuals;
        residuals.reserve(n);
        for (size_t i = 0; i < n; ++i)
            residuals.push_back(uplinks[i] - (intercept + slope * static_cast<double>(i)));
        const double base = *std::min_element(residuals.begin(), residuals.end());
        for (double& value : residuals)
            value -= base;
        print_stats("  of which uplink us", summarize(residuals));

        // The fitted slope is drift per sample; divided by the mean sample
        // spacing it is the host-vs-board clock rate error. A large value means
        // the board's kCanTimestampNsPerUs is mis-scaled for this board.
        const double mean_period_us = measured_span_us / count;
        if (mean_period_us > 0.0)
            printf(
                "  clock drift: %+.0f ppm (host steady_clock vs board PTPC, over %.2f s)\n",
                slope / mean_period_us * 1e6, measured_span_us / 1e6);
    } else {
        printf("  (board reported no hardware timestamps; uplink split unavailable)\n");
    }

    return timeouts == 0 && probe.invalid() == 0 && probe.wrong_bus() == 0 ? 0 : 1;
}

// ---------------------------------------------------------------------------
// contend: the same A->B latency, but measured while the rig is busy in both
// directions on the other bus. This is the question "what does a second board
// cost me", asked directly: with N USB boards the host controller, its
// interrupt and both boards' endpoints are shared, and an idle-rig latency
// number says nothing about that.
// ---------------------------------------------------------------------------

int run_contend(
    uint32_t load_hz, uint32_t samples, int core_a, int core_b, int core_main, int load_core) {
    Node board_a{g_serial_a, core_a, "hcs-a"};
    Node board_b{g_serial_b, core_b, "hcs-b"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    configure_thread(core_main, 80, "dual-main");

    // Measured path stays A.CAN1 -> B.CAN1 so the number is directly comparable
    // with `latency 0`; the load runs on bus 1 in the opposite direction, so
    // both boards are transmitting and receiving at once.
    LatencyProbe probe;
    probe.set_expected_bus(0);

    struct Sink {
        std::atomic<uint64_t> count{0};
    } load_sink;

    board_b.set_can_handler(&probe, LatencyProbe::handler);
    board_a.set_can_handler(&load_sink, [](void* context, int, const libhcs::data::CanDataView&) {
        static_cast<Sink*>(context)->count.fetch_add(1, std::memory_order_relaxed);
    });

    std::atomic<bool> running{true};
    std::atomic<uint64_t> load_sent{0};
    std::thread load_thread{[&]() {
        // The load generator paces by spinning, so it must own a core: left
        // unpinned it competes with the measuring thread and the transport
        // event threads, and what gets measured is the host scheduler rather
        // than the effect of a second board on the USB path.
        configure_thread(load_core, 70, "dual-load");
        const auto period = std::chrono::nanoseconds{1'000'000'000ULL / load_hz};
        auto next = Clock::now();
        for (uint64_t sequence = 0; running.load(std::memory_order_relaxed); ++sequence) {
            std::byte payload[kPayloadSize];
            put_u32_le(payload, static_cast<uint32_t>(sequence));
            put_u32_le(payload + 4, mix(static_cast<uint32_t>(sequence)));
            transmit_can(
                board_b.board(), 1,
                {.can_id = kCanIdBase + 1, .can_data = payload});
            load_sent.fetch_add(1, std::memory_order_relaxed);
            next += period;
            while (running.load(std::memory_order_relaxed) && Clock::now() < next) {}
        }
    }};

    printf(
        "A.CAN1 -> B.CAN1 latency while A.CAN2 -> B.CAN2 runs at %u frames/s (CAN-FD)\n",
        load_hz);
    std::this_thread::sleep_for(std::chrono::milliseconds{200});

    std::vector<double> latencies;
    latencies.reserve(samples);
    uint32_t timeouts = 0;
    for (uint32_t sequence = 0; sequence < samples + kWarmupSamples; ++sequence) {
        std::byte payload[kPayloadSize];
        put_u32_le(payload, sequence);
        put_u32_le(payload + 4, mix(sequence));
        probe.arm(sequence, Clock::now());
        transmit_can(
            board_a.board(), 0, {.can_id = kCanIdBase, .can_data = payload});

        double latency_us = 0.0, uplink_us = 0.0;
        bool has_uplink = false;
        if (!probe.wait(sequence, latency_us, uplink_us, has_uplink)) {
            if (sequence >= kWarmupSamples)
                ++timeouts;
            continue;
        }
        if (sequence >= kWarmupSamples)
            latencies.push_back(latency_us);
    }

    running.store(false, std::memory_order_relaxed);
    load_thread.join();
    std::this_thread::sleep_for(std::chrono::milliseconds{200});

    printf(
        "measured=%zu timeout=%u   background: sent=%llu delivered=%llu\n", latencies.size(),
        timeouts, static_cast<unsigned long long>(load_sent.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(load_sink.count.load(std::memory_order_relaxed)));
    print_stats("A->B under load us", summarize(latencies));
    return latencies.empty() ? 2 : 0;
}

// ---------------------------------------------------------------------------
// uartcontend: CAN latency while a UART stream saturates the same USB pipe.
// ---------------------------------------------------------------------------

// CAN frames and UART bytes share ONE bulk endpoint in each direction, so a UART
// chunk queued ahead of a CAN frame delays it. That is head-of-line blocking, and
// no CAN-only measurement can see it: `contend` loads the second CAN bus, which
// competes for CPU and controllers but is the same kind of traffic. This mode
// loads the endpoint with traffic that has nothing to do with CAN at all.
//
// The stream runs A.UART0 -> B.UART0, which puts bytes on exactly the two legs
// the measured frame uses: board A's downlink and board B's uplink. The measured
// path, sample count and warm-up match `latency 0` and `contend`, so all three
// numbers are directly comparable.
//
// Both threads transmit on board A here (unlike `contend`, where the load sits on
// board B). That is safe: acquire_transmit_buffer() pops an exclusive transfer
// from a mutex-protected free list, so each thread fills its own buffer.

struct UartLoadSink {
    std::atomic<uint64_t> bytes{0};
};

void uart_load_handler(void* context, const libhcs::data::UartDataView& data) {
    static_cast<UartLoadSink*>(context)->bytes.fetch_add(
        data.uart_data.size(), std::memory_order_relaxed);
}

int run_uart_contend(
    uint32_t samples, uint32_t uart_kbps, int core_a, int core_b, int core_main, int load_core) {
    Node board_a{g_serial_a, core_a, "hcs-a"};
    Node board_b{g_serial_b, core_b, "hcs-b"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    configure_thread(core_main, 80, "dual-main");

    LatencyProbe probe;
    probe.set_expected_bus(0);
    board_b.set_can_handler(&probe, LatencyProbe::handler);

    UartLoadSink load_sink;
    board_b.set_uart_handler(&load_sink, uart_load_handler);

    std::atomic<bool> running{true};
    std::atomic<uint64_t> load_sent{0};
    std::thread load_thread{[&]() {
        configure_thread(load_core, 70, "dual-load");
        // 64 bytes is one chunk the firmware forwards as a unit; at 921600 8N1
        // that is 694 us of wire time, so ~92 kB/s is the line rate. The period
        // follows the requested rate, which is what makes the load a sweep: the
        // question is not only "does a saturated UART hurt CAN" but "how much
        // does MY UART rate hurt". Asking for more than the line rate would
        // measure the board's queue overflow instead of the blocking.
        constexpr size_t kChunk = 64;
        const auto kPeriod = std::chrono::microseconds{
            static_cast<int64_t>(kChunk * 1000ULL / (uart_kbps ? uart_kbps : 1))};
        const auto load_start = Clock::now();
        uint64_t chunk_index = 0;
        uint64_t index = 0;
        while (running.load(std::memory_order_relaxed)) {
            std::byte payload[kChunk];
            for (std::byte& b : payload)
                b = static_cast<std::byte>(index++);
            {
                auto builder = board_a.board().start_transmit();
                builder.uart0_transmit({.uart_data = payload, .idle_delimited = false});
            }
            load_sent.fetch_add(kChunk, std::memory_order_relaxed);

            // Absolute schedule, not an incrementally advanced deadline: chunk N
            // is due at start + N*period no matter what happened to chunks 0..N-1.
            // An incremental `next += period` plus a clamp still lets the loop run
            // free for as long as it is behind, so a stall anywhere -- the board
            // applying backpressure, the measurement thread timing out and
            // stretching the run -- turns into a burst afterwards. That burst is
            // what overflows the board's UART queue, and then the run measures
            // queue overflow instead of head-of-line blocking.
            ++chunk_index;
            const auto due = load_start + kPeriod * chunk_index;
            while (running.load(std::memory_order_relaxed) && Clock::now() < due) {}
        }
    }};

    printf(
        "A.CAN1 -> B.CAN1 latency while A.UART0 -> B.UART0 runs at %u kB/s (CAN-FD)\n",
        uart_kbps);
    std::this_thread::sleep_for(std::chrono::milliseconds{200});

    const auto start = Clock::now();
    std::vector<double> latencies;
    latencies.reserve(samples);
    uint32_t timeouts = 0;
    for (uint32_t sequence = 0; sequence < samples + kWarmupSamples; ++sequence) {
        std::byte payload[kPayloadSize];
        put_u32_le(payload, sequence);
        put_u32_le(payload + 4, mix(sequence));
        probe.arm(sequence, Clock::now());
        transmit_can(
            board_a.board(), 0, {.can_id = kCanIdBase, .can_data = payload});

        double latency_us = 0.0, uplink_us = 0.0;
        bool has_uplink = false;
        if (!probe.wait(sequence, latency_us, uplink_us, has_uplink)) {
            if (sequence >= kWarmupSamples)
                ++timeouts;
            continue;
        }
        if (sequence >= kWarmupSamples)
            latencies.push_back(latency_us);
        // Same duty-cycle control as run_latency. Without it this mode is always
        // a back-to-back ping-pong, which keeps both USB pipes permanently hot
        // and therefore cannot separate "background traffic warms the pipe" from
        // "background traffic blocks the head of the line" -- the two effects
        // this mode exists to weigh against each other.
        idle_for(latency_gap_us());
    }
    const double seconds = std::chrono::duration<double>(Clock::now() - start).count();

    running.store(false, std::memory_order_relaxed);
    load_thread.join();
    std::this_thread::sleep_for(std::chrono::milliseconds{200});

    const uint64_t sent = load_sent.load(std::memory_order_relaxed);
    const uint64_t got = load_sink.bytes.load(std::memory_order_relaxed);
    // The offered rate is printed next to the requested one because they diverge
    // exactly when the run is invalid: CAN timeouts stretch the wall clock, the
    // load keeps running, and sent/delivered then reflects a much longer run than
    // the sample count suggests. A run whose offered rate is not close to the
    // request, or whose measured count is short of the request, must be rejected
    // rather than read -- the percentiles look plausible either way.
    const double offered_kbps = static_cast<double>(sent) / seconds / 1000.0;
    printf(
        "measured=%zu/%u timeout=%u  elapsed=%.1f s  uart: offered=%.1f kB/s (asked %u) "
        "delivered=%.1f kB/s\n",
        latencies.size(), samples, timeouts, seconds, offered_kbps, uart_kbps,
        static_cast<double>(got) / seconds / 1000.0);
    if (latencies.size() < samples || offered_kbps > uart_kbps * 1.25)
        printf("  WARNING: run is not comparable -- see the note above.\n");
    print_stats("A->B under UART load us", summarize(latencies));
    return latencies.empty() ? 2 : 0;
}

// ---------------------------------------------------------------------------
// uartlatency: the UART path's OWN latency and loss, end to end.
// ---------------------------------------------------------------------------

// Everything else here treats UART as a load generator for the CAN measurement.
// This measures the UART link itself: host -> A.USB -> A.UART0 -> wire ->
// B.UART0 -> B.USB -> host, one chunk at a time with the loop idle in between.
//
// The wire time is a first-class term and must not be subtracted away: at
// 921600 8N1 a byte is 10 bits, so 10.85 us each. An 8-byte chunk spends 87 us
// on the wire, a 64-byte one 694 us -- for anything but the smallest chunk the
// UART wire dominates the USB path completely. Sweeping the chunk size is what
// makes that visible instead of leaving it hidden in one number.
//
// Byte-stream, not framed: the sequence number is carried in the first four
// bytes and the receiver reassembles across callbacks, because a chunk may
// arrive split or coalesced.

class UartLatencyProbe {
public:
    void arm(uint32_t sequence, size_t chunk_bytes, Clock::time_point send_time) {
        const std::scoped_lock guard{mutex_};
        expected_ = sequence;
        chunk_bytes_ = chunk_bytes;
        send_time_ = send_time;
        received_ = 0;
        arrived_ = false;
        mismatched_ = false;
    }

    bool wait(double& latency_us, bool& mismatched) {
        const auto deadline = Clock::now() + std::chrono::milliseconds{50};
        std::unique_lock guard{mutex_};
        if (!cv_.wait_until(guard, deadline, [this] { return arrived_; }))
            return false;
        latency_us = latency_us_;
        mismatched = mismatched_;
        return true;
    }

    static void handler(void* context, const libhcs::data::UartDataView& data) {
        static_cast<UartLatencyProbe*>(context)->on_uart(data);
    }

private:
    void on_uart(const libhcs::data::UartDataView& data) {
        const auto receive_time = Clock::now();
        const std::scoped_lock guard{mutex_};
        if (arrived_ || chunk_bytes_ == 0)
            return;
        for (const std::byte b : data.uart_data) {
            if (received_ < sizeof(header_))
                header_[received_] = b;
            if (++received_ < chunk_bytes_)
                continue;
            // Whole chunk in: the timestamp is taken at the callback that
            // completed it, so the reported latency covers the last byte.
            arrived_ = true;
            mismatched_ = get_u32_le(header_) != expected_;
            latency_us_ = std::chrono::duration<double, std::micro>(receive_time - send_time_).count();
            cv_.notify_one();
            return;
        }
    }

    std::mutex mutex_;
    std::condition_variable cv_;
    uint32_t expected_ = 0;
    size_t chunk_bytes_ = 0;
    size_t received_ = 0;
    bool arrived_ = false;
    bool mismatched_ = false;
    double latency_us_ = 0.0;
    Clock::time_point send_time_{};
    std::byte header_[4]{};
};

int run_uart_latency(
    uint32_t samples, size_t chunk_bytes, int core_a, int core_b, int core_main) {
    if (chunk_bytes < 4 || chunk_bytes > 509) {
        printf("chunk must be 4..509 bytes (509 keeps the transfer inside one USB packet)\n");
        return 2;
    }
    Node board_a{g_serial_a, core_a, "hcs-a"};
    Node board_b{g_serial_b, core_b, "hcs-b"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    configure_thread(core_main, 80, "dual-main");

    UartLatencyProbe probe;
    board_b.set_uart_handler(&probe, UartLatencyProbe::handler);

    const double wire_us = static_cast<double>(chunk_bytes) * 10.0 * 1e6 / 921600.0;
    printf(
        "A.UART0 -> B.UART0 latency, %u samples x %zu bytes @921600 "
        "(wire time alone = %.1f us)\n",
        samples, chunk_bytes, wire_us);

    std::vector<double> latencies;
    latencies.reserve(samples);
    uint32_t timeouts = 0, mismatches = 0;
    std::vector<std::byte> payload(chunk_bytes);

    for (uint32_t sequence = 0; sequence < samples + kWarmupSamples; ++sequence) {
        put_u32_le(payload.data(), sequence);
        for (size_t i = 4; i < chunk_bytes; ++i)
            payload[i] = static_cast<std::byte>(sequence + i);

        const auto now = Clock::now();
        probe.arm(sequence, chunk_bytes, now);
        {
            auto builder = board_a.board().start_transmit();
            builder.uart0_transmit({.uart_data = payload, .idle_delimited = false});
        }

        double latency_us = 0.0;
        bool mismatched = false;
        if (!probe.wait(latency_us, mismatched)) {
            if (sequence >= kWarmupSamples)
                ++timeouts;
            // Let the stream drain so the next sample starts on a clean boundary.
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
            continue;
        }
        if (sequence >= kWarmupSamples) {
            if (mismatched)
                ++mismatches;
            latencies.push_back(latency_us);
        }
        // Idle between samples: this measures one chunk's trip, not throughput.
        std::this_thread::sleep_for(std::chrono::microseconds{500});
    }

    printf(
        "measured=%zu/%u  timeout=%u  mismatch=%u\n", latencies.size(), samples, timeouts,
        mismatches);
    print_stats("A.UART0 -> B.UART0 us", summarize(latencies));
    if (!latencies.empty()) {
        const auto stats = summarize(latencies);
        printf("  minus wire time: p50 %.1f us  (USB down + board + USB up)\n", stats.p50 - wire_us);
    }
    return latencies.empty() ? 2 : 0;
}

// ---------------------------------------------------------------------------
// dual: drive both buses at once and check they stay independent.
// ---------------------------------------------------------------------------

struct DualCounter {
    std::atomic<uint64_t> received[2]{};
    std::atomic<uint64_t> corrupt{0};
    std::atomic<uint64_t> misrouted{0};
};

void dual_handler(void* context, int bus, const libhcs::data::CanDataView& data) {
    auto* counter = static_cast<DualCounter*>(context);
    if (data.can_data.size() != kPayloadSize) {
        counter->corrupt.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const uint32_t sequence = get_u32_le(data.can_data.data());
    if (get_u32_le(data.can_data.data() + 4) != mix(sequence)) {
        counter->corrupt.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    // The bus index is encoded in the CAN id, so a frame that surfaced on the
    // other bus is detectable rather than merely miscounted.
    if (data.can_id != kCanIdBase + static_cast<uint32_t>(bus)) {
        counter->misrouted.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    counter->received[bus].fetch_add(1, std::memory_order_relaxed);
}

int run_dual(uint32_t samples) {
    Node board_a{g_serial_a, -1, "hcs-a"};
    Node board_b{g_serial_b, -1, "hcs-b"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    DualCounter counter;
    board_b.set_can_handler(&counter, dual_handler);

    printf("both buses in parallel: %u frames per bus (CAN-FD)\n", samples);

    for (uint32_t sequence = 0; sequence < samples; ++sequence) {
        std::byte payload[kPayloadSize];
        put_u32_le(payload, sequence);
        put_u32_le(payload + 4, mix(sequence));
        // One USB packet carrying one frame for each bus: this is what a real
        // controller does per control tick, and it is where a shared-buffer bug
        // between the two CAN paths would surface.
        auto builder = board_a.board().start_transmit();
        builder.can_transmit(
            CanPort::kCan1, {.can_id = kCanIdBase + 0, .can_data = payload});
        builder.can_transmit(
            CanPort::kCan2, {.can_id = kCanIdBase + 1, .can_data = payload});
        std::this_thread::sleep_for(std::chrono::microseconds{300});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    const uint64_t bus0 = counter.received[0].load(std::memory_order_relaxed);
    const uint64_t bus1 = counter.received[1].load(std::memory_order_relaxed);
    const uint64_t corrupt = counter.corrupt.load(std::memory_order_relaxed);
    const uint64_t misrouted = counter.misrouted.load(std::memory_order_relaxed);
    printf(
        "CAN1 %llu/%u  CAN2 %llu/%u  corrupt=%llu misrouted=%llu\n",
        static_cast<unsigned long long>(bus0), samples, static_cast<unsigned long long>(bus1),
        samples, static_cast<unsigned long long>(corrupt),
        static_cast<unsigned long long>(misrouted));

    const bool ok = bus0 == samples && bus1 == samples && corrupt == 0 && misrouted == 0;
    printf("dual: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// ---------------------------------------------------------------------------
// uart: A.UART0 -> B.UART0 at the board's 921600 baud, integrity + throughput.
// ---------------------------------------------------------------------------

struct UartStream {
    std::atomic<uint64_t> bytes{0};
    std::atomic<uint64_t> chunks{0};
    std::atomic<uint64_t> mismatches{0};
    // Received bytes must be a prefix-consistent continuation of the generated
    // pseudo-random stream, so a single counter is enough to validate order and
    // content without buffering the whole run.
    uint64_t verified = 0;
};

uint8_t stream_byte(uint64_t index) {
    return static_cast<uint8_t>(mix(static_cast<uint32_t>(index)) >> 13);
}

void uart_stream_handler(void* context, const libhcs::data::UartDataView& data) {
    auto* stream = static_cast<UartStream*>(context);
    for (const std::byte b : data.uart_data) {
        if (std::to_integer<uint8_t>(b) != stream_byte(stream->verified))
            stream->mismatches.fetch_add(1, std::memory_order_relaxed);
        ++stream->verified;
    }
    stream->bytes.fetch_add(
        static_cast<uint64_t>(data.uart_data.size()), std::memory_order_relaxed);
    stream->chunks.fetch_add(1, std::memory_order_relaxed);
}

int run_uart(uint32_t rounds) {
    Node board_a{g_serial_a, -1, "hcs-a"};
    Node board_b{g_serial_b, -1, "hcs-b"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    UartStream stream;
    board_b.set_uart_handler(&stream, uart_stream_handler);

    // 64 bytes per round is one UART chunk the firmware forwards as a unit; at
    // 921600 8N1 that is 694 us of wire time, so the pacing below stays under
    // the line rate and measures the link rather than the host's ability to
    // overrun it.
    constexpr size_t kChunk = 64;
    const uint64_t total = static_cast<uint64_t>(rounds) * kChunk;
    printf(
        "A.UART0 -> B.UART0: %u rounds x %zu bytes = %llu bytes @921600\n", rounds, kChunk,
        static_cast<unsigned long long>(total));

    uint64_t index = 0;
    const auto start = Clock::now();
    for (uint32_t round = 0; round < rounds; ++round) {
        std::byte payload[kChunk];
        for (size_t i = 0; i < kChunk; ++i)
            payload[i] = static_cast<std::byte>(stream_byte(index++));
        auto builder = board_a.board().start_transmit();
        builder.uart0_transmit({.uart_data = payload, .idle_delimited = false});
        std::this_thread::sleep_for(std::chrono::microseconds{800});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
    const double seconds = std::chrono::duration<double>(Clock::now() - start).count();

    const uint64_t bytes = stream.bytes.load(std::memory_order_relaxed);
    const uint64_t mismatches = stream.mismatches.load(std::memory_order_relaxed);
    printf(
        "received %llu/%llu bytes in %llu chunks, mismatches=%llu, %.1f kB/s (line rate %.1f "
        "kB/s)\n",
        static_cast<unsigned long long>(bytes), static_cast<unsigned long long>(total),
        static_cast<unsigned long long>(stream.chunks.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(mismatches), static_cast<double>(bytes) / seconds / 1e3,
        921600.0 / 10.0 / 1e3);

    const bool ok = bytes == total && mismatches == 0;
    printf("uart: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// ---------------------------------------------------------------------------
// stress: sustained flood across both buses, loss accounting and stall
// detection. This is the regime the MCAN ISR fix addressed, so it is also the
// regression test for it.
// ---------------------------------------------------------------------------

// Decoder for the telemetry a libhcs_CAN_DIAG firmware emits on the UART0
// uplink (firmware/hpm_board/app/src/diag/can_diag.cpp). Only the three
// numbers that attribute a stress-test loss are pulled out:
//
//   board frames  - what the receiving board's CAN ISR actually took off the
//                   wire. Equal to what was sent => nothing was lost on CAN.
//   alloc_fail    - uplink batch pool exhausted, i.e. the board dropped an
//                   already-received frame because the host was not draining
//                   USB fast enough. This is host backpressure, not a CAN fault.
//   isr entries   - frozen while frames climb would mean a lost interrupt.
//
// Layout constants mirror can_stall_probe.cpp, which decodes the same record.
struct DiagRecord {
    bool valid = false;
    uint32_t tick = 0;
    uint32_t alloc_fail = 0;
    uint32_t isr_entries[2]{};
    uint32_t frames[2]{};
    // Incremented whenever mcan_transmit_via_txfifo_nonblocking() is refused
    // because the MCAN TX FIFO is full: the frame the host asked for is
    // DROPPED, and the board lights the cyan LED. On the transmitting board
    // this, not anything on the receive side, is what a stress-test shortfall
    // usually is.
    uint32_t tx_fail[2]{};
};

bool decode_diag(std::span<const std::byte> payload, DiagRecord& out) {
    constexpr uint8_t kRecordMagic = 0xD1U;
    constexpr uint8_t kRecordVersion = 4U;
    constexpr size_t kPlicWordCount = 6;
    // 8-byte header, then tick / alloc_fail / main_loop, then three PLIC word
    // arrays and the threshold; the per-CAN blocks follow.
    constexpr size_t kPerCanOffset = 8 + 4 + 4 + 4 + 4 * (3 * kPlicWordCount) + 4;
    constexpr size_t kPerCanSize = 9 * 4;

    if (payload.size() < kPerCanOffset)
        return false;
    if (std::to_integer<uint8_t>(payload[0]) != kRecordMagic
        || std::to_integer<uint8_t>(payload[1]) != kRecordVersion)
        return false;
    const uint8_t can_count = std::to_integer<uint8_t>(payload[3]);
    if (get_u32_le(payload.data() + 4) != payload.size()
        || payload.size() < kPerCanOffset + can_count * kPerCanSize)
        return false;

    out.tick = get_u32_le(payload.data() + 8);
    out.alloc_fail = get_u32_le(payload.data() + 12);
    for (uint8_t i = 0; i < can_count && i < 2; ++i) {
        const std::byte* block = payload.data() + kPerCanOffset + i * kPerCanSize;
        out.isr_entries[i] = get_u32_le(block);
        out.frames[i] = get_u32_le(block + 4);
        out.tx_fail[i] = get_u32_le(block + 8);
    }
    out.valid = true;
    return true;
}

struct StressCounter {
    std::atomic<uint64_t> received[2]{};
    std::atomic<uint64_t> corrupt{0};
    std::atomic<uint64_t> last_sequence[2]{};
    std::atomic<uint64_t> gaps[2]{};
};

void stress_handler(void* context, int bus, const libhcs::data::CanDataView& data) {
    auto* counter = static_cast<StressCounter*>(context);
    if (data.can_data.size() != kPayloadSize) {
        counter->corrupt.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const uint32_t sequence = get_u32_le(data.can_data.data());
    if (get_u32_le(data.can_data.data() + 4) != mix(sequence)) {
        counter->corrupt.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const uint64_t previous =
        counter->last_sequence[bus].exchange(sequence, std::memory_order_relaxed);
    if (sequence != previous + 1 && counter->received[bus].load(std::memory_order_relaxed) != 0)
        counter->gaps[bus].fetch_add(1, std::memory_order_relaxed);
    counter->received[bus].fetch_add(1, std::memory_order_relaxed);
}

int run_stress(uint32_t frames_per_sec, uint32_t seconds, int buses, int per_packet) {
    // These arrive straight from argv. Out-of-range values used to be accepted
    // silently and produce nonsense -- `buses`/`per_packet` sit where a reader
    // of the usage line expects core numbers, so passing "7 6" once meant
    // "7 buses, 6 frames per packet" and skewed every number in the report.
    if (frames_per_sec == 0 || seconds == 0) {
        fprintf(stderr, "stress: frames_per_sec and seconds must be non-zero\n");
        return 1;
    }
    if (buses < 1 || buses > 2 || per_packet < 1) {
        fprintf(
            stderr,
            "stress: buses must be 1 or 2 and per_packet >= 1 (got buses=%d "
            "per_packet=%d)\n",
            buses, per_packet);
        return 1;
    }

    Node board_a{g_serial_a, -1, "hcs-a"};
    Node board_b{g_serial_b, -1, "hcs-b"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    StressCounter counter;
    board_b.set_can_handler(&counter, stress_handler);

    // Present only on a libhcs_CAN_DIAG build; on a normal one no record ever
    // arrives and the attribution block below simply stays quiet. Both boards
    // are watched, because the two ends fail differently: the transmitter drops
    // on a full MCAN TX FIFO (tx_fail, cyan LED), the receiver drops on a full
    // uplink batch pool (alloc_fail, yellow LED).
    static DiagRecord first_tx, last_tx, first_rx, last_rx;
    board_a.set_uart_handler(nullptr, [](void*, const libhcs::data::UartDataView& data) {
        DiagRecord record;
        if (!decode_diag(data.uart_data, record))
            return;
        if (!first_tx.valid)
            first_tx = record;
        last_tx = record;
    });
    board_b.set_uart_handler(nullptr, [](void*, const libhcs::data::UartDataView& data) {
        DiagRecord record;
        if (!decode_diag(data.uart_data, record))
            return;
        if (!first_rx.valid)
            first_rx = record;
        last_rx = record;
    });

    printf(
        "stress: %u frames/s per bus on %d bus(es) for %u s (frames follow the bus mode: "
        "CAN-FD 1M/5M BRS)\n", frames_per_sec, buses, seconds);

    // `frames_per_sec` counts FRAMES per bus, and `per_packet` of them ride in
    // one USB packet, so the loop iterates frames_per_sec/per_packet times a
    // second and the period scales with per_packet. Getting this wrong is not
    // cosmetic: pacing per ITERATION would silently send per_packet times the
    // requested frame rate.
    const uint64_t total_per_bus = static_cast<uint64_t>(frames_per_sec) * seconds;
    const auto period = std::chrono::nanoseconds{
        1'000'000'000ULL * static_cast<uint64_t>(per_packet) / frames_per_sec};
    const auto start = Clock::now();
    auto next = start;

    // Progress is sampled once a second so a stall (forwarding stops while the
    // sender keeps going) is visible in the timeline, not just in the total.
    uint64_t last_report_bus0 = 0, last_report_bus1 = 0;
    uint32_t reported_second = 0;

    for (uint64_t sequence = 0; sequence < total_per_bus;) {
        {
            // `per_packet` frames go into ONE USB packet. The transmit builder
            // batches every write until it goes out of scope, so this is the
            // difference between N packets per second and N/per_packet -- which
            // matters because the USB HS microframe rate (8000/s) caps packets,
            // not frames.
            //
            // Every frame carries its OWN sequence number. Reusing one number
            // for the whole batch would make the receiver see per_packet-1
            // repeats and count each as a gap, reporting loss that never
            // happened.
            auto builder = board_a.board().start_transmit();
            for (int repeat = 0; repeat < per_packet && sequence < total_per_bus;
                 ++repeat, ++sequence) {
                std::byte payload[kPayloadSize];
                put_u32_le(payload, static_cast<uint32_t>(sequence));
                put_u32_le(payload + 4, mix(static_cast<uint32_t>(sequence)));
                builder.can_transmit(CanPort::kCan1, 
                    {.can_id = kCanIdBase + 0, .can_data = payload});
                if (buses > 1)
                    builder.can_transmit(CanPort::kCan2, 
                        {.can_id = kCanIdBase + 1, .can_data = payload});
            }
        }

        next += period;
        while (Clock::now() < next) {}

        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - start);
        if (static_cast<uint32_t>(elapsed.count()) > reported_second) {
            reported_second = static_cast<uint32_t>(elapsed.count());
            const uint64_t bus0 = counter.received[0].load(std::memory_order_relaxed);
            const uint64_t bus1 = counter.received[1].load(std::memory_order_relaxed);
            printf(
                "  t=%2us  CAN1 +%llu  CAN2 +%llu\n", reported_second,
                static_cast<unsigned long long>(bus0 - last_report_bus0),
                static_cast<unsigned long long>(bus1 - last_report_bus1));
            fflush(stdout);
            last_report_bus0 = bus0;
            last_report_bus1 = bus1;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{500});

    const uint64_t bus0 = counter.received[0].load(std::memory_order_relaxed);
    const uint64_t bus1 = counter.received[1].load(std::memory_order_relaxed);
    printf(
        "sent %llu/bus  received CAN1=%llu (%.4f%% lost)  CAN2=%llu (%.4f%% lost)\n",
        static_cast<unsigned long long>(total_per_bus), static_cast<unsigned long long>(bus0),
        100.0 * static_cast<double>(total_per_bus - bus0) / static_cast<double>(total_per_bus),
        static_cast<unsigned long long>(bus1),
        100.0 * static_cast<double>(total_per_bus - bus1) / static_cast<double>(total_per_bus));
    printf(
        "sequence gaps CAN1=%llu CAN2=%llu  corrupt=%llu\n",
        static_cast<unsigned long long>(counter.gaps[0].load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(counter.gaps[1].load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(counter.corrupt.load(std::memory_order_relaxed)));

    if (first_tx.valid && last_tx.valid)
        printf(
            "TX board (A): MCAN TX FIFO refused bus0=%u bus1=%u frames -- dropped before "
            "reaching the wire (cyan LED)\n",
            last_tx.tx_fail[0] - first_tx.tx_fail[0], last_tx.tx_fail[1] - first_tx.tx_fail[1]);
    if (first_rx.valid && last_rx.valid)
        printf(
            "RX board (B): CAN ISR took bus0=%u bus1=%u frames; uplink alloc_fail=%u\n",
            last_rx.frames[0] - first_rx.frames[0], last_rx.frames[1] - first_rx.frames[1],
            last_rx.alloc_fail - first_rx.alloc_fail);

    const bool ok = bus0 == total_per_bus && bus1 == total_per_bus
                 && counter.corrupt.load(std::memory_order_relaxed) == 0;
    printf("stress: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// ---------------------------------------------------------------------------
// frames: which CAN frame shapes actually survive the bridge. Sweeps payload
// length, standard vs extended identifiers and RTR, for classic and FD. The
// board's MCAN message RAM uses 8-byte elements, so this is where a payload
// the hardware cannot store shows up as a silent drop rather than as an error.
// ---------------------------------------------------------------------------

struct FrameEcho {
    std::atomic<bool> got{false};
    std::atomic<uint32_t> can_id{0};
    std::atomic<size_t> length{0};
    std::atomic<bool> is_extended{false};
    std::atomic<bool> is_rtr{false};
    std::atomic<bool> payload_ok{false};
};

void frame_handler(void* context, int /*bus*/, const libhcs::data::CanDataView& data) {
    auto* echo = static_cast<FrameEcho*>(context);
    echo->can_id.store(data.can_id, std::memory_order_relaxed);
    echo->length.store(data.can_data.size(), std::memory_order_relaxed);
    echo->is_extended.store(data.is_extended_can_id, std::memory_order_relaxed);
    echo->is_rtr.store(data.is_remote_transmission, std::memory_order_relaxed);
    bool ok = true;
    for (size_t i = 0; i < data.can_data.size(); ++i)
        ok = ok && std::to_integer<uint8_t>(data.can_data[i]) == static_cast<uint8_t>(i + 1);
    echo->payload_ok.store(ok, std::memory_order_relaxed);
    echo->got.store(true, std::memory_order_release);
}

bool frame_case(
    Node& source, Node& sink, uint32_t can_id, size_t length, bool extended, bool rtr) {
    FrameEcho echo;
    sink.set_can_handler(&echo, frame_handler);

    std::vector<std::byte> payload(length);
    for (size_t i = 0; i < length; ++i)
        payload[i] = static_cast<std::byte>(i + 1);

    bool sent = true;
    try {
        transmit_can(
            source.board(), 0,
            {.can_id = can_id,
             .can_data = payload,
             .is_extended_can_id = extended,
             .is_remote_transmission = rtr});
    } catch (const std::exception&) {
        sent = false;
    }

    if (sent) {
        const auto deadline = Clock::now() + std::chrono::milliseconds{50};
        while (!echo.got.load(std::memory_order_acquire) && Clock::now() < deadline) {}
    }

    const bool arrived = echo.got.load(std::memory_order_acquire);
    const char* verdict;
    if (!sent)
        verdict = "REJECTED-BY-HOST";
    else if (!arrived)
        verdict = "DROPPED";
    else if (
        echo.can_id.load(std::memory_order_relaxed) != can_id
        || echo.is_extended.load(std::memory_order_relaxed) != extended
        || echo.is_rtr.load(std::memory_order_relaxed) != rtr)
        verdict = "ATTR-MISMATCH";
    else if (
        echo.length.load(std::memory_order_relaxed) != (rtr ? 0U : length)
        || !echo.payload_ok.load(std::memory_order_relaxed))
        verdict = "PAYLOAD-MISMATCH";
    else
        verdict = "ok";

    // The first column names the WIRE format, which is the bus's compiled mode
    // -- the host no longer chooses it per frame, so every case here is sent
    // the same way.
    printf(
        "  %-8s %-3s id=0x%-8X len=%-2zu %-4s -> %-16s", "bus-mode",
        extended ? "ext" : "std", can_id, length, rtr ? "RTR" : "", verdict);
    if (arrived)
        printf(
            " (rx len=%zu ext=%d rtr=%d)", echo.length.load(std::memory_order_relaxed),
            static_cast<int>(echo.is_extended.load(std::memory_order_relaxed)),
            static_cast<int>(echo.is_rtr.load(std::memory_order_relaxed)));
    printf("\n");

    sink.set_can_handler(nullptr, nullptr);
    return std::string_view{verdict} == "ok";
}

int run_frames() {
    Node board_a{g_serial_a, -1, "hcs-a"};
    Node board_b{g_serial_b, -1, "hcs-b"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    printf("frame-shape sweep, A.CAN1 -> B.CAN1\n");
    size_t ok_count = 0, total = 0;
    const auto check = [&](bool result) {
        ++total;
        ok_count += result ? 1 : 0;
    };

    // There is no classic-vs-FD axis any more: every frame goes out in the
    // bus's compiled mode (CAN-FD on this rig). The payload-0..8 and
    // extended-ID blocks keep covering the shapes the wire protocol carries.
    printf(" payload 0..8, standard ids:\n");
    for (size_t length = 0; length <= 8; ++length)
        check(frame_case(board_a, board_b, 0x100 + length, length, false, false));

    printf(" payload beyond 8 bytes (valid FD DLCs on the wire; the protocol\n");
    printf(" caps the host view at 8, so these are rejected before the bus):\n");
    for (const size_t length : {12U, 16U, 20U, 24U, 32U, 48U, 64U})
        check(frame_case(board_a, board_b, 0x300, length, false, false));

    printf(" extended identifiers:\n");
    check(frame_case(board_a, board_b, 0x1ABCDEF, 8, true, false));
    check(frame_case(board_a, board_b, 0x7FF, 8, false, false));
    check(frame_case(board_a, board_b, 0x1FFFFFFF, 8, true, false));

    printf(" remote transmission request:\n");
    check(frame_case(board_a, board_b, 0x400, 0, false, true));

    printf("frames: %zu/%zu shapes round-tripped intact\n", ok_count, total);
    return ok_count == total ? 0 : 1;
}

// ---------------------------------------------------------------------------
// uartraw: send one known pattern once and dump every received chunk verbatim,
// with arrival times. Ground truth for whatever the `uart` mode is complaining
// about -- chunk boundaries, duplication and reordering are all visible here.
// ---------------------------------------------------------------------------

struct RawDump {
    Clock::time_point start;
    std::atomic<uint32_t> chunk_index{0};
};

void uart_raw_handler(void* context, const libhcs::data::UartDataView& data) {
    auto* dump = static_cast<RawDump*>(context);
    const double at_us =
        std::chrono::duration<double, std::micro>(Clock::now() - dump->start).count();
    printf(
        "  chunk %2u  t=%8.1fus  %2zu bytes:", dump->chunk_index.fetch_add(1), at_us,
        data.uart_data.size());
    for (const std::byte b : data.uart_data)
        printf(" %02X", static_cast<unsigned>(std::to_integer<uint8_t>(b)));
    printf("\n");
}

int run_uart_raw(uint32_t length, bool idle_delimited) {
    Node board_a{g_serial_a, -1, "hcs-a"};
    Node board_b{g_serial_b, -1, "hcs-b"};
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    RawDump dump;
    dump.start = Clock::now();
    board_b.set_uart_handler(&dump, uart_raw_handler);

    std::vector<std::byte> payload(length);
    for (uint32_t i = 0; i < length; ++i)
        payload[i] = static_cast<std::byte>(i);

    printf(
        "A.UART0 -> B.UART0: one %u-byte ramp 00..%02X, idle_delimited=%s\n", length,
        static_cast<unsigned>((length - 1) & 0xFF), idle_delimited ? "true" : "false");
    printf("  sent :");
    for (uint32_t i = 0; i < length; ++i)
        printf(" %02X", static_cast<unsigned>(std::to_integer<uint8_t>(payload[i])));
    printf("\n");

    {
        auto builder = board_a.board().start_transmit();
        builder.uart0_transmit({.uart_data = payload, .idle_delimited = idle_delimited});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{500});

    printf("  total chunks: %u\n", dump.chunk_index.load(std::memory_order_relaxed));
    return 0;
}

void print_usage() {
    fprintf(
        stderr, "usage: dual_board_test <mode> [args]\n"
                "  list                                          enumerate attached boards\n"
                "  link                                          verify every wired direction\n"
                "  latency [bus] [samples] [ca] [cb] [cmain]     A.CANx -> B.CANx one-way latency\n"
                "          bus is the number printed on the enclosure: 1 or 2\n"
                "  dual    [samples]                             both buses in parallel\n"
                "  uart    [rounds]                              A.UART0 -> B.UART0 integrity\n"
                "  uartlatency [samples] [bytes] [ca] [cb] [cmain]\n"
                "          the UART path's OWN latency; sweep [bytes] to see the 10.85 us/byte\n"
                "          wire time separate from the USB path\n"
                "  uartcontend [samples] [uart_kBps] [ca] [cb] [cmain] [cload]\n"
                "          CAN latency while a UART stream shares the same pipe; 92 kB/s is\n"
                "          the 921600 8N1 line rate, sweep it to map YOUR rate onto the cost\n"
                "  stress  [fps] [seconds] [buses] [per_packet]  sustained flood, loss accounting\n"
                "          fps is FRAMES/s per bus; per_packet frames share one USB packet,\n"
                "          so packets/s is fps/per_packet.  buses 1..2, per_packet >= 1\n"
                "env: HCS_CAN_CLASSIC=1 classic CAN instead of CAN-FD\n"
                "     HCS_BOARD_A / HCS_BOARD_B  override the serial-order A/B assignment\n");
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
        printf("board A = %s\nboard B = %s\n", g_serial_a.c_str(), g_serial_b.c_str());

        if (mode == "link")
            return run_link();
        if (mode == "latency")
            // The bus argument is the enclosure number (1 or 2); run_latency
            // still indexes from 0 internally.
            return run_latency(
                static_cast<int>(argument(2, 1)) - 1, static_cast<uint32_t>(argument(3, 3000)),
                static_cast<int>(argument(4, -1)), static_cast<int>(argument(5, -1)),
                static_cast<int>(argument(6, -1)));
        if (mode == "dual")
            return run_dual(static_cast<uint32_t>(argument(2, 2000)));
        if (mode == "frames")
            return run_frames();
        if (mode == "contend")
            return run_contend(
                static_cast<uint32_t>(argument(2, 4000)), static_cast<uint32_t>(argument(3, 3000)),
                static_cast<int>(argument(4, -1)), static_cast<int>(argument(5, -1)),
                static_cast<int>(argument(6, -1)), static_cast<int>(argument(7, -1)));
        if (mode == "uartcontend")
            return run_uart_contend(
                static_cast<uint32_t>(argument(2, 3000)), static_cast<uint32_t>(argument(3, 92)),
                static_cast<int>(argument(4, -1)), static_cast<int>(argument(5, -1)),
                static_cast<int>(argument(6, -1)), static_cast<int>(argument(7, -1)));
        if (mode == "uartlatency")
            return run_uart_latency(
                static_cast<uint32_t>(argument(2, 3000)), static_cast<size_t>(argument(3, 8)),
                static_cast<int>(argument(4, -1)), static_cast<int>(argument(5, -1)),
                static_cast<int>(argument(6, -1)));
        if (mode == "uart")
            return run_uart(static_cast<uint32_t>(argument(2, 2000)));
        if (mode == "uartraw")
            return run_uart_raw(static_cast<uint32_t>(argument(2, 16)), argument(3, 1) != 0);
        if (mode == "stress")
            return run_stress(
                static_cast<uint32_t>(argument(2, 4000)), static_cast<uint32_t>(argument(3, 10)),
                static_cast<int>(argument(4, 2)), static_cast<int>(argument(5, 1)));
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    print_usage();
    return 1;
}
