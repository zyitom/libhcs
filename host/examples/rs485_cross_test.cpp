#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dirent.h>

#include <libhcs/board/mc02.hpp>

// RS-485 test for mc02, over either of two rigs (HCS_RS485_RIG, default by
// board count):
//
//   - "cross": two boards wired A-A / B-B on the SAME port, chosen by
//     HCS_RS485_PORT -- 2 is UART3 / USART3 (connector P5, transceiver U6,
//     DE on PB14) and 1 is UART2 / USART2 (transceiver U5, DE on PD4).
//   - "single": ONE board with its own two RS-485 ports wired to each other,
//     A = UART2 (USART2) and B = UART3 (USART3). This is the only rig that
//     exercises USART3 when just one board is on the bench, and it is a real
//     two-node bus: separate USARTs, separate transceivers, separate DE pins,
//     120R fitted at each end.
//
// Both power up at 4800000 and are electrically the same circuit, so every test
// applies unchanged to either rig. UART2/UART3 are first-class silkscreen ports
// (kUart2 / kUart3) on every firmware build -- there is no compile switch to
// turn them on.
//
// Why a separate program from uart_cross_test: that one is the TTL TX<->RX
// suite. This one owns the half-duplex turnaround, DE, and the two enclosure
// RS-485 transceivers, and talks to libhcs::board::Mc02 directly so a misrouted
// TTL frame cannot pass as an RS-485 result. (The ports are also reachable as
// BoardTransmitter indices 1 and 2.)
//
// The two properties a self-loopback (one port's TX back into its own RX) cannot
// check, and how each rig here still covers them:
//
//   - Direction control. DE is raised by the USART itself (CR3.DEM, set by
//     HAL_RS485Ex_Init), so an end that never releases the bus still passes any
//     test where only one end transmits. Both directions are exercised, and the
//     sender is checked for silence while it transmits. Two ports on one board
//     have independent DE pins, so the single rig checks this as well as the
//     cross rig does.
//   - Baudrate. A silently ignored config request leaves every end at whatever
//     CubeMX programmed -- 4800000 -- and a link whose ends move together passes
//     at every requested rate while running at none of them (see
//     firmware/mc02/AGENTS.md). On the single rig both ends are the same
//     firmware, so that common-mode blindness is the default; test_baudrate_mismatch
//     moves ONE end and requires the link to break, which is what sees through it.
//
// Framing contract the firmware presents: RxBuffer::try_dequeue() publishes a
// chunk as soon as kMinFragmentSize (32) bytes are in the ring, without waiting
// for the line to go idle. A message is "everything up to and including the next
// chunk flagged idle_delimited", never "one chunk" -- and when the message
// length is an exact multiple of 32 that final chunk is empty.
//
// Half-duplex pacing is the firmware's, not this program's: TxBuffer with
// half_duplex=true releases one queued packet per bus transaction, gated until
// either the peer's IDLE counter moves or kTurnaroundDeadline (1000 us) expires.
// test_turnaround below measures exactly that gate.

namespace {

using libhcs::board::Mc02;
using libhcs::data::UartDataView;
using Clock = std::chrono::steady_clock;

constexpr uint32_t k_base_baudrate = 115200;
constexpr auto k_message_timeout = std::chrono::milliseconds(500);

// Staging buffer for a UartRs485 port is kRs485BufferSize (256) bytes; stay
// under it so a size sweep tests fragmentation rather than backpressure.
constexpr std::size_t k_max_payload = 200;

uint32_t g_baudrate = k_base_baudrate;
// Which RS-485 port the whole test drives. 2 (UART3 / USART3, connector P5) is
// the default because that is the port the original rig was wired for and every
// number in this file's history came from. 1 selects UART2 / USART2 /
// transceiver U5, which is otherwise identical electrically -- same
// RE#-tied-to-DE topology, same on-board 120R, same 4800000 power-up baudrate.
int g_port = 2;
int g_ping_pong_rounds = 200;

std::vector<std::pair<std::string, std::string>> enumerate_boards() {
    std::vector<std::pair<std::string, std::string>> found; // serial, product
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
        if (read_line("idVendor") != "a511" || read_line("idProduct") != "0723")
            continue;
        const std::string serial = read_line("serial");
        if (!serial.empty())
            found.emplace_back(serial, read_line("product"));
    }
    closedir(dir);
    std::sort(found.begin(), found.end());
    return found;
}

// Distinguishable at every position: a plain incrementing ramp repeats every 256
// bytes, so a dropped or duplicated 256-byte run would compare equal.
std::vector<std::byte> make_pattern(std::size_t size, uint32_t seed) {
    std::vector<std::byte> out(size);
    uint32_t state = 0x9E3779B9U ^ seed;
    for (std::size_t i = 0; i < size; ++i) {
        state = state * 1664525U + 1013904223U;
        out[i] = static_cast<std::byte>((state >> 24) ^ static_cast<uint32_t>(i));
    }
    return out;
}

std::size_t
    first_difference(std::span<const std::byte> expected, std::span<const std::byte> actual) {
    const auto common = std::min(expected.size(), actual.size());
    for (std::size_t i = 0; i < common; ++i) {
        if (expected[i] != actual[i])
            return i;
    }
    return common;
}

std::string hex_preview(std::span<const std::byte> data, std::size_t limit = 16) {
    std::string out;
    char scratch[8];
    for (std::size_t i = 0; i < std::min(limit, data.size()); ++i) {
        std::snprintf(scratch, sizeof(scratch), "%02x ", static_cast<unsigned>(data[i]));
        out += scratch;
    }
    if (data.size() > limit)
        out += "...";
    return out;
}

class Node;

// One mc02 and the endpoints living on it. Both RS-485 ports arrive through a
// single Mc02 callback object, so the routing has to be here rather than in
// Node: on the one-board rig (USART2 wired to USART3 on the same board) the two
// endpoints under test are two ports of THIS object, and each has to reassemble
// its own stream. Traffic on a channel nobody claimed is counted rather than
// dropped, so a firmware that misroutes a downlink shows up as a number instead
// of a silent pass.
class BoardHost final : public Mc02::Callback {
public:
    explicit BoardHost(std::string_view serial) { board_ = std::make_unique<Mc02>(*this, serial); }

    Mc02& board() { return *board_; }

    // Claim one RS-485 port for an endpoint. Unclaimed ports keep counting into
    // the leak detector.
    void attach(int port, Node* node) { (port == 1 ? node1_ : node2_) = node; }

private:
    void uart1_receive_callback(const UartDataView& data) override;
    void uart2_receive_callback(const UartDataView& data) override;
    void uart3_receive_callback(const UartDataView& data) override;
    void uart7_receive_callback(const UartDataView& data) override;
    void uart10_receive_callback(const UartDataView& data) override;
    void dbus_receive_callback(const UartDataView& data) override;
    void count_unclaimed(const UartDataView& data, const char* name);

    Node* node1_ = nullptr;
    Node* node2_ = nullptr;
    std::unique_ptr<Mc02> board_;
};

// One endpoint of the bus: an RS-485 port on some board, plus the reassembly
// state for it. Two endpoints make a link; whether they sit on one board or two
// is the rig's business, not this class's.
class Node {
public:
    Node(std::string tag, BoardHost& host, int port)
        : tag_(std::move(tag))
        , host_(&host)
        , port_(port) {
        host.attach(port, this);
    }

    ~Node() { stop_responder(); }

    const std::string& tag() const { return tag_; }

    int port() const { return port_; }

    void reset() {
        const std::lock_guard guard{mutex_};
        partial_.clear();
        messages_.clear();
        all_bytes_.clear();
        chunk_count_ = 0;
        other_channel_chunks_ = 0;
        other_channel_name_ = nullptr;
        dbus_chunks_ = 0;
        dbus_bytes_ = 0;
    }

    void send(std::span<const std::byte> payload, bool idle_delimited = true) {
        const std::lock_guard guard{transmit_mutex_};
        if (port_ == 1)
            host_->board().start_transmit().uart2_transmit(
                {.uart_data = payload, .idle_delimited = idle_delimited});
        else
            host_->board().start_transmit().uart3_transmit(
                {.uart_data = payload, .idle_delimited = idle_delimited});
    }

    void set_baudrate(uint32_t baudrate) {
        const std::lock_guard guard{transmit_mutex_};
        if (port_ == 1)
            host_->board().start_transmit().uart2_config({.baudrate = baudrate});
        else
            host_->board().start_transmit().uart3_config({.baudrate = baudrate});
    }

    std::size_t message_count() {
        const std::lock_guard guard{mutex_};
        return messages_.size();
    }

    std::size_t chunk_count() {
        const std::lock_guard guard{mutex_};
        return chunk_count_;
    }

    std::size_t byte_count() {
        const std::lock_guard guard{mutex_};
        return all_bytes_.size();
    }

    std::size_t pending_bytes() {
        const std::lock_guard guard{mutex_};
        return partial_.size();
    }

    std::size_t other_channel_chunks() {
        const std::lock_guard guard{mutex_};
        return other_channel_chunks_;
    }

    const char* other_channel_name() {
        const std::lock_guard guard{mutex_};
        return other_channel_name_ ? other_channel_name_ : "none";
    }

    std::pair<std::size_t, std::size_t> dbus_traffic() {
        const std::lock_guard guard{mutex_};
        return {dbus_chunks_, dbus_bytes_};
    }

    std::vector<std::vector<std::byte>> take_messages() {
        const std::lock_guard guard{mutex_};
        return std::exchange(messages_, {});
    }

    // Waits for at least `count` complete messages. Returns what arrived, which
    // may be fewer on timeout -- the caller reports the shortfall.
    std::vector<std::vector<std::byte>>
        wait_for_messages(std::size_t count, std::chrono::milliseconds timeout) {
        std::unique_lock lock{mutex_};
        condition_.wait_for(lock, timeout, [&] { return messages_.size() >= count; });
        return std::exchange(messages_, {});
    }

    // Echoes every complete message back onto the bus from a dedicated thread.
    // Not from the receive callback: that runs on the transport's completion
    // thread, and submitting a transfer from inside a completion is a
    // reentrancy question this test has no reason to answer.
    void start_responder() {
        stop_responder();
        responder_running_ = true;
        responder_ = std::thread{[this] {
            while (responder_running_) {
                std::vector<std::byte> reply;
                {
                    std::unique_lock lock{mutex_};
                    condition_.wait_for(lock, std::chrono::milliseconds(20), [&] {
                        return !reply_queue_.empty() || !responder_running_;
                    });
                    if (!responder_running_)
                        return;
                    if (reply_queue_.empty())
                        continue;
                    reply = std::move(reply_queue_.front());
                    reply_queue_.pop_front();
                }
                send(reply);
            }
        }};
    }

    void stop_responder() {
        if (!responder_.joinable())
            return;
        {
            const std::lock_guard guard{mutex_};
            responder_running_ = false;
        }
        condition_.notify_all();
        responder_.join();
        const std::lock_guard guard{mutex_};
        reply_queue_.clear();
    }

    // Called by BoardHost for the port this endpoint claimed. Reassembles
    // idle-delimited messages out of the chunk stream; which physical port feeds
    // it is the rig's business, not this function's.
    void accept(const UartDataView& data) {
        const std::lock_guard guard{mutex_};
        ++chunk_count_;
        all_bytes_.insert(all_bytes_.end(), data.uart_data.begin(), data.uart_data.end());
        partial_.insert(partial_.end(), data.uart_data.begin(), data.uart_data.end());
        if (!data.idle_delimited)
            return;
        auto message = std::exchange(partial_, {});
        if (responder_running_)
            reply_queue_.push_back(message);
        messages_.push_back(std::move(message));
        condition_.notify_all();
    }

    // Channels nothing this test sends should ever land on; a nonzero count
    // means the firmware routed a downlink or an uplink to the wrong channel id.
    void count_other(const UartDataView& data, const char* name) {
        if (data.uart_data.empty())
            return;
        const std::lock_guard guard{mutex_};
        ++other_channel_chunks_;
        other_channel_name_ = name;
    }

    // DBUS is UART5, RX-only at 100000 8E1, and it shares the uplink ring with
    // every other channel. An unconnected receiver floats, so it can fill that
    // ring on its own -- which looks exactly like an RS-485 fault from the
    // outside, because the only symptom either produces is the LED. Counted
    // separately for that reason: "the board is busy" and "the 485 port is busy"
    // are different findings.
    void note_dbus(const UartDataView& data) {
        if (data.uart_data.empty())
            return;
        const std::lock_guard guard{mutex_};
        dbus_chunks_ += 1;
        dbus_bytes_ += data.uart_data.size();
    }

private:
    std::string tag_;
    BoardHost* host_;
    int port_;
    std::mutex mutex_;
    std::mutex transmit_mutex_;
    std::condition_variable condition_;

    std::vector<std::byte> partial_;
    std::vector<std::byte> all_bytes_;
    std::vector<std::vector<std::byte>> messages_;
    std::deque<std::vector<std::byte>> reply_queue_;
    std::size_t chunk_count_ = 0;
    std::size_t other_channel_chunks_ = 0;
    const char* other_channel_name_ = nullptr;
    std::size_t dbus_chunks_ = 0;
    std::size_t dbus_bytes_ = 0;

    bool responder_running_ = false;
    std::thread responder_;
};

void BoardHost::uart1_receive_callback(const UartDataView& data) {
    count_unclaimed(data, "UART1");
}
void BoardHost::uart2_receive_callback(const UartDataView& data) {
    if (node1_ != nullptr)
        node1_->accept(data);
    else
        count_unclaimed(data, "UART2");
}
void BoardHost::uart3_receive_callback(const UartDataView& data) {
    if (node2_ != nullptr)
        node2_->accept(data);
    else
        count_unclaimed(data, "UART3");
}
void BoardHost::uart7_receive_callback(const UartDataView& data) {
    count_unclaimed(data, "UART7");
}
void BoardHost::uart10_receive_callback(const UartDataView& data) {
    count_unclaimed(data, "UART10");
}

void BoardHost::dbus_receive_callback(const UartDataView& data) {
    for (Node* node : {node1_, node2_}) {
        if (node != nullptr)
            node->note_dbus(data);
    }
}

// Attributed to every endpoint on this board: whichever one the report reads
// from, the finding "the firmware put bytes on a channel nobody addressed" is
// the same.
void BoardHost::count_unclaimed(const UartDataView& data, const char* name) {
    for (Node* node : {node1_, node2_}) {
        if (node != nullptr)
            node->count_other(data, name);
    }
}

int g_failures = 0;

void report(bool ok, const std::string& what, const std::string& detail = {}) {
    printf(
        "  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(), detail.empty() ? "" : " -- ",
        detail.c_str());
    fflush(stdout);
    if (!ok)
        ++g_failures;
}

void settle() { std::this_thread::sleep_for(std::chrono::milliseconds(60)); }

// One payload across the bus in one direction, checked for content, for framing
// (exactly one idle-delimited message), and for the sender staying silent --
// RE# is tied to DE on this board, so our own transmission must not come back.
bool exchange_once(Node& from, Node& to, std::size_t size, uint32_t seed, std::string& detail) {
    from.reset();
    to.reset();
    const auto payload = make_pattern(size, seed);
    from.send(payload);

    const auto received = to.wait_for_messages(1, k_message_timeout);
    // A late second message would arrive after the wait returned, so give the
    // line a moment before judging the count.
    settle();
    const auto extra = to.take_messages();

    char scratch[256];
    if (received.empty()) {
        std::snprintf(
            scratch, sizeof(scratch), "%zu bytes sent, nothing arrived (%zu partial)", size,
            to.pending_bytes());
        detail = scratch;
        return false;
    }
    if (received.size() + extra.size() != 1) {
        std::snprintf(
            scratch, sizeof(scratch), "expected 1 message, got %zu",
            received.size() + extra.size());
        detail = scratch;
        return false;
    }
    const auto& message = received.front();
    if (message.size() != size || first_difference(payload, message) != size) {
        const auto index = first_difference(payload, message);
        std::snprintf(
            scratch, sizeof(scratch), "sent %zu bytes, got %zu, first difference at byte %zu", size,
            message.size(), index);
        detail = scratch;
        detail += "\n         sent: "
                + hex_preview(std::span{payload}.subspan(std::min(index, payload.size())));
        detail += "\n         recv: "
                + hex_preview(std::span{message}.subspan(std::min(index, message.size())));
        return false;
    }
    if (from.byte_count() != 0) {
        std::snprintf(
            scratch, sizeof(scratch),
            "sender saw %zu bytes of its own transmission (RE# not following DE?)",
            from.byte_count());
        detail = scratch;
        return false;
    }
    detail.clear();
    return true;
}

// Sizes chosen around kMinFragmentSize (32): a message shorter than one fragment
// arrives only because the line went idle, a message that is an exact multiple
// arrives as N full chunks plus an empty idle-delimited one, and the odd sizes
// land mid-fragment.
void test_sizes(Node& a, Node& b) {
    printf("\n== Payload sizes at %u baud ==\n", g_baudrate);
    constexpr std::size_t sizes[] = {1, 8, 31, 32, 33, 64, 96, 128, k_max_payload};
    uint32_t seed = 1;
    for (const auto size : sizes) {
        std::string detail;
        const bool forward = exchange_once(a, b, size, seed++, detail);
        report(
            forward, a.tag() + " -> " + b.tag() + ", " + std::to_string(size) + " bytes", detail);
        settle();
        const bool reverse = exchange_once(b, a, size, seed++, detail);
        report(
            reverse, b.tag() + " -> " + a.tag() + ", " + std::to_string(size) + " bytes", detail);
        settle();
    }
}

// The check a loopback cannot make. Both ends are moved together; if one end
// ignored the request the other's bytes arrive at the wrong bit rate and the
// comparison fails on content, not on silence.
void test_baudrates(Node& a, Node& b) {
    printf("\n== Baudrate sweep ==\n");
    constexpr uint32_t rates[] = {115200, 460800, 921600, 1500000, 3000000, 4800000};
    uint32_t seed = 1000;
    for (const auto rate : rates) {
        a.set_baudrate(rate);
        b.set_baudrate(rate);
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        std::string detail;
        const bool forward = exchange_once(a, b, 64, seed++, detail);
        report(forward, std::to_string(rate) + " baud, " + a.tag() + " -> " + b.tag(), detail);
        settle();
        const bool reverse = exchange_once(b, a, 64, seed++, detail);
        report(reverse, std::to_string(rate) + " baud, " + b.tag() + " -> " + a.tag(), detail);
        settle();
    }
    a.set_baudrate(g_baudrate);
    b.set_baudrate(g_baudrate);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
}

// Back-to-back packets with no peer answering. The half-duplex TxBuffer holds
// each packet until the peer's IDLE counter moves or kTurnaroundDeadline
// (1000 us) expires; with a silent peer only the deadline releases them, so N
// packets should take about (N-1) ms and still arrive as N separate messages.
void test_turnaround(Node& a, Node& b) {
    printf("\n== Half-duplex turnaround (silent peer) ==\n");
    constexpr int kPackets = 8;
    a.reset();
    b.reset();
    const auto start = Clock::now();
    for (int i = 0; i < kPackets; ++i)
        a.send(make_pattern(16, static_cast<uint32_t>(2000 + i)));

    const auto messages = b.wait_for_messages(kPackets, std::chrono::milliseconds(3000));
    const auto elapsed = Clock::now() - start;
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(elapsed).count();

    char scratch[256];
    std::snprintf(
        scratch, sizeof(scratch), "%zu of %d messages in %.2f ms (%.2f ms per packet)",
        messages.size(), kPackets, elapsed_ms, elapsed_ms / kPackets);
    report(
        messages.size() == kPackets, "packets kept separate across the turnaround gate", scratch);

    bool contents_ok = messages.size() == kPackets;
    for (std::size_t i = 0; i < messages.size() && contents_ok; ++i) {
        const auto expected = make_pattern(16, static_cast<uint32_t>(2000 + i));
        contents_ok = messages[i].size() == expected.size()
                   && first_difference(expected, messages[i]) == expected.size();
    }
    report(contents_ok, "turnaround packets arrived intact and in order");
    settle();
}

// Request/response, which is what the turnaround gate is built for: the peer's
// answer raises IDLE on the initiator and releases the next packet early, so
// this should run far faster than the 1 ms deadline path above.
//
// The figure is a host-observed round trip: USB out, bus, USB in, responder
// thread wakeup, USB out, bus, USB in. It is not a bus turnaround measurement --
// the two USB traversals dominate -- but it is the number an application driving
// a 485 device through this port will actually see.
void test_ping_pong(Node& a, Node& b) {
    printf("\n== Ping-pong round trip (%d rounds) ==\n", g_ping_pong_rounds);
    a.reset();
    b.reset();
    b.start_responder();
    settle();

    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(g_ping_pong_rounds));
    int lost = 0;
    int corrupt = 0;
    for (int round = 0; round < g_ping_pong_rounds; ++round) {
        const auto payload = make_pattern(16, static_cast<uint32_t>(3000 + round));
        a.take_messages();
        const auto start = Clock::now();
        a.send(payload);
        const auto replies = a.wait_for_messages(1, k_message_timeout);
        const auto elapsed = Clock::now() - start;
        if (replies.empty()) {
            ++lost;
            continue;
        }
        if (replies.front().size() != payload.size()
            || first_difference(payload, replies.front()) != payload.size())
            ++corrupt;
        samples.push_back(
            std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(elapsed).count());
    }
    b.stop_responder();

    char scratch[256];
    std::snprintf(
        scratch, sizeof(scratch), "%d lost, %d corrupt of %d", lost, corrupt, g_ping_pong_rounds);
    report(lost == 0 && corrupt == 0, "every request answered with its own payload", scratch);

    if (!samples.empty()) {
        std::sort(samples.begin(), samples.end());
        const auto percentile = [&](double fraction) {
            const auto index = static_cast<std::size_t>(fraction * (samples.size() - 1));
            return samples[index];
        };
        double sum = 0;
        for (const auto value : samples)
            sum += value;
        printf(
            "  round trip: min %.0f us, mean %.0f us, p50 %.0f us, p99 %.0f us, max %.0f us\n",
            samples.front(), sum / static_cast<double>(samples.size()), percentile(0.50),
            percentile(0.99), samples.back());
    }
    settle();
}

// Two drivers on the bus at once. RS-485 has no arbitration and both
// transceivers are push-pull, so the bytes on the wire during the overlap are
// undefined -- what matters is that neither port wedges: after the collision,
// both directions must still carry a clean message.
void test_collision_recovery(Node& a, Node& b) {
    printf("\n== Collision recovery ==\n");
    a.reset();
    b.reset();
    for (int i = 0; i < 4; ++i) {
        a.send(make_pattern(64, static_cast<uint32_t>(4000 + i)));
        b.send(make_pattern(64, static_cast<uint32_t>(5000 + i)));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    char scratch[128];
    std::snprintf(
        scratch, sizeof(scratch), "%s saw %zu bytes, %s saw %zu bytes during the overlap",
        a.tag().c_str(), a.byte_count(), b.tag().c_str(), b.byte_count());
    printf("  [INFO] %s\n", scratch);

    std::string detail;
    const bool forward = exchange_once(a, b, 64, 4100, detail);
    report(forward, "link usable after collision, " + a.tag() + " -> " + b.tag(), detail);
    settle();
    const bool reverse = exchange_once(b, a, 64, 4200, detail);
    report(reverse, "link usable after collision, " + b.tag() + " -> " + a.tag(), detail);
    settle();
}

// Listen with nobody transmitting. An idle RS-485 pair that no driver is holding
// sits at whatever the termination and the leakage currents decide, which for a
// terminated link with no fail-safe bias is a differential voltage near zero --
// inside the receiver's undefined window. The receiver then outputs noise, the
// USART reads it as start bits, and the port produces uplink traffic forever
// with nothing on the bus. That is what a quiet link must be checked for, and
// the LED is the symptom: uplink_buffer_full() blinks yellow for 5 s per event,
// so a permanently yellow board is a port that never stops receiving.
void test_quiet_bus(Node& a, Node& b) {
    printf("\n== Quiet bus (nobody transmitting, 5 s) ==\n");
    a.reset();
    b.reset();
    std::this_thread::sleep_for(std::chrono::seconds(5));

    char scratch[256];
    for (Node* node : {&a, &b}) {
        const auto bytes = node->byte_count();
        std::snprintf(
            scratch, sizeof(scratch), "%s received %zu bytes in %zu chunks on a silent bus",
            node->tag().c_str(), bytes, node->chunk_count());
        report(bytes == 0, std::string{"idle line stays quiet on "} + node->tag(), scratch);
    }

    // Anything arriving here shares the uplink ring with the RS-485 port, so it
    // fills the same buffer and lights the same LED. Reported whether or not it
    // is zero: "the 485 port is silent and the board is still busy" is the
    // finding that separates a port fault from a board-level one.
    for (Node* node : {&a, &b}) {
        const auto [chunks, bytes] = node->dbus_traffic();
        std::snprintf(
            scratch, sizeof(scratch), "%s: %zu bytes in %zu chunks over 5 s", node->tag().c_str(),
            bytes, chunks);
        printf("  [INFO] DBUS (UART5, shares the uplink ring) %s\n", scratch);
    }
    fflush(stdout);
}

// Does a single small message still leave immediately, now that a bulk OUT
// transfer asks for 1024 bytes instead of 64?
//
// The hazard is specific: a transfer completes on a full 1024 bytes OR on a
// short packet. A downlink whose total is an exact multiple of the 64-byte
// endpoint has no short packet, so it would sit in the controller until enough
// further traffic arrived -- not slower, stuck. The host sets
// LIBUSB_TRANSFER_ADD_ZERO_PACKET (host/src/transport/usb/usb.cpp:422) so libusb
// appends a zero-length packet in exactly that case, which is short and ends the
// transfer. This test is here because that is a claim about two codebases
// agreeing, and it is worth more as a measurement than as a reading.
//
// Sweeping payload size one byte at a time crosses every 64-byte boundary in
// transfer size without needing to know the per-record header width. A stall
// shows up as lost messages at one specific size; an added delay shows up as a
// step in the round-trip curve at the same place.
void test_latency_vs_size(Node& a, Node& b) {
    printf("\n== Round-trip vs message size (ZLP / transfer-boundary check) ==\n");
    int kRounds = 20;
    if (const char* value = std::getenv("HCS_RS485_ROUNDS"))
        kRounds = std::atoi(value);
    constexpr std::size_t kMaxSize = 200;

    a.set_baudrate(4800000);
    b.set_baudrate(4800000);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    b.start_responder();
    settle();

    std::vector<std::pair<std::size_t, double>> curve;
    std::vector<std::size_t> stalled;
    for (std::size_t size = 1; size <= kMaxSize; ++size) {
        std::vector<double> samples;
        int lost = 0;
        for (int round = 0; round < kRounds; ++round) {
            const auto payload = make_pattern(size, static_cast<uint32_t>(9000 + size));
            a.take_messages();
            const auto start = Clock::now();
            a.send(payload);
            const auto replies = a.wait_for_messages(1, k_message_timeout);
            if (replies.empty()) {
                ++lost;
                continue;
            }
            samples.push_back(std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(
                                  Clock::now() - start)
                                  .count());
        }
        if (lost != 0)
            stalled.push_back(size);
        if (!samples.empty()) {
            std::sort(samples.begin(), samples.end());
            curve.emplace_back(size, samples[samples.size() / 2]);
        }
    }
    b.stop_responder();

    char scratch[256];
    std::snprintf(
        scratch, sizeof(scratch), "%zu of %zu sizes lost messages", stalled.size(), kMaxSize);
    report(stalled.empty(), "every message size round-trips", scratch);
    if (!stalled.empty()) {
        printf("  stalled sizes:");
        for (const auto size : stalled)
            printf(" %zu", size);
        printf("\n");
    }

    // The line time grows with size, so the curve must slope. What would betray a
    // transfer that waited is a jump at one size that the neighbouring sizes do
    // not share, so report the largest single-byte step rather than the slope.
    double worst_step = 0;
    std::size_t worst_at = 0;
    for (std::size_t i = 1; i < curve.size(); ++i) {
        const double step = curve[i].second - curve[i - 1].second;
        if (step > worst_step) {
            worst_step = step;
            worst_at = curve[i].first;
        }
    }
    printf(
        "  p50 round trip: %zu B %.0f us -> %zu B %.0f us\n", curve.front().first,
        curve.front().second, curve.back().first, curve.back().second);
    // Full curve on request: the endpoints alone cannot show whether a step sits
    // at a 64-byte boundary, which is the whole question here.
    if (std::getenv("HCS_RS485_CURVE") != nullptr) {
        for (const auto& [size, us] : curve)
            printf("CURVE %zu %.0f\n", size, us);
    }
    std::snprintf(
        scratch, sizeof(scratch), "largest one-byte jump %.0f us at size %zu", worst_step,
        worst_at);
    // One 64-byte packet is 133 us of line time at 4.8 Mbaud; a transfer that
    // waited for more traffic would cost far more than that, so anything under
    // 200 us is the line, not the USB layer.
    report(worst_step < 200.0, "no size pays a transfer-boundary penalty", scratch);

    a.set_baudrate(g_baudrate);
    b.set_baudrate(g_baudrate);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    settle();
}

// Moves ONE end and checks the link breaks.
//
// Every other baudrate check here moves both ends together, and that shape
// cannot tell "the config request was applied" from "the config request was
// dropped at both ends identically" -- which is exactly how the
// HAL_RCCEx_GetPeriphCLKFreq() bug survived a UART7<->UART10 loopback for
// months (firmware/mc02/AGENTS.md). On the one-board rig both ends are the same
// firmware, so common-mode blindness is the default and this is the only check
// that sees through it: with the two ports at different bit rates the bytes MUST
// arrive wrong, and if they arrive intact the config request did nothing.
void test_baudrate_mismatch(Node& a, Node& b) {
    printf("\n== Baudrate mismatch (proves the config request is applied) ==\n");
    a.set_baudrate(115200);
    b.set_baudrate(921600);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    a.reset();
    b.reset();
    const auto payload = make_pattern(64, 7000);
    a.send(payload);
    const auto received = b.wait_for_messages(1, k_message_timeout);
    settle();

    const bool intact = received.size() == 1 && received.front().size() == payload.size()
                     && first_difference(payload, received.front()) == payload.size();
    char scratch[192];
    std::snprintf(
        scratch, sizeof(scratch), "%s at 115200 -> %s at 921600 delivered %zu message(s), %zu B",
        a.tag().c_str(), b.tag().c_str(), received.size(), b.byte_count());
    report(!intact, "a one-sided baudrate change actually changes the bit rate", scratch);

    a.set_baudrate(g_baudrate);
    b.set_baudrate(g_baudrate);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    settle();
}

// What happens to a downlink larger than the port's 256-byte TX ring.
//
// UartRs485 sizes TxBuffer at kRs485BufferSize (256) while the protocol carries
// payloads up to kProtocolBufferSize (1023), so there is a range of perfectly
// legal host requests the port cannot hold. TxBuffer::try_enqueue() returns
// false for them and handle_downlink() turns that into led->downlink_buffer_full()
// -- no uplink, no error to the host. This measures where the cliff is and
// whether anything worse than a clean drop happens at it (a truncated message
// would be far worse than none, because the peer would act on half a command).
void test_oversize(Node& a, Node& b) {
    printf("\n== Downlink larger than the 256-byte TX ring ==\n");
    a.set_baudrate(4800000);
    b.set_baudrate(4800000);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    constexpr std::size_t sizes[] = {200, 254, 255, 256, 257, 300, 512, 1000};
    bool truncated_anywhere = false;
    for (const auto size : sizes) {
        a.reset();
        b.reset();
        const auto payload = make_pattern(size, static_cast<uint32_t>(8000 + size));
        a.send(payload);
        const auto received = b.wait_for_messages(1, std::chrono::milliseconds(300));
        settle();
        const auto extra = b.take_messages();
        const std::size_t delivered = b.byte_count();
        const char* verdict = nullptr;
        if (delivered == 0)
            verdict = "dropped silently";
        else if (
            received.size() + extra.size() == 1 && delivered == size
            && first_difference(payload, received.front()) == size)
            verdict = "delivered intact";
        else {
            verdict = "TRUNCATED / corrupt";
            truncated_anywhere = true;
        }
        printf(
            "  [INFO] %4zu B -> %-20s (%zu bytes, %zu message(s) on the wire)\n", size, verdict,
            delivered, received.size() + extra.size());
        fflush(stdout);
    }
    report(!truncated_anywhere, "an oversized downlink is never half-delivered");

    a.set_baudrate(g_baudrate);
    b.set_baudrate(g_baudrate);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    settle();
}

// A burst of separate commands queued faster than the bus can carry them.
//
// This is the normal shape of RS-485 master traffic -- a poll cycle over several
// nodes -- and it is where the 256-byte ring is actually spent, because the
// half-duplex gate holds each packet for up to kTurnaroundDeadline (1000 us)
// while the host keeps enqueuing. Anything the ring cannot hold is dropped with
// only an LED to say so, so the number that matters is how many of N go out.
void test_burst(Node& a, Node& b) {
    printf("\n== Command burst (half-duplex gate vs the 256-byte ring) ==\n");
    a.set_baudrate(4800000);
    b.set_baudrate(4800000);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    constexpr std::size_t kPacketSize = 32;
    constexpr int counts[] = {4, 8, 12, 16, 24};
    bool any_loss = false;
    for (const auto count : counts) {
        a.reset();
        b.reset();
        for (int i = 0; i < count; ++i)
            a.send(make_pattern(kPacketSize, static_cast<uint32_t>(6000 + i)));
        auto messages =
            b.wait_for_messages(static_cast<std::size_t>(count), std::chrono::milliseconds(3000));
        settle();
        for (auto& late : b.take_messages())
            messages.push_back(std::move(late));

        // Which commands were lost matters more than how many: dropping the tail
        // of a poll cycle is a different fault from dropping the middle, and a
        // short message would mean the ring cut a command in half.
        std::string missing;
        std::size_t next = 0;
        bool truncated = false;
        for (int i = 0; i < count; ++i) {
            const auto expected = make_pattern(kPacketSize, static_cast<uint32_t>(6000 + i));
            if (next < messages.size() && messages[next].size() == expected.size()
                && first_difference(expected, messages[next]) == expected.size()) {
                ++next;
                continue;
            }
            missing += " " + std::to_string(i);
        }
        for (const auto& message : messages)
            truncated = truncated || message.size() != kPacketSize;
        if (next != messages.size() || !missing.empty())
            any_loss = true;
        printf(
            "  [INFO] %2d x %zu B queued (%zu bytes) -> %zu arrived%s%s%s\n", count, kPacketSize,
            kPacketSize * static_cast<std::size_t>(count), messages.size(),
            missing.empty() ? "" : ", missing indices", missing.c_str(),
            truncated ? "  [SHORT MESSAGE]" : "");
        fflush(stdout);
    }
    report(!any_loss, "every queued command reaches the bus");

    a.set_baudrate(g_baudrate);
    b.set_baudrate(g_baudrate);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    settle();
}

void test_channel_isolation(Node& a, Node& b) {
    printf("\n== Channel isolation ==\n");
    char scratch[128];
    std::snprintf(
        scratch, sizeof(scratch), "%zu chunks on %s, %zu on %s", a.other_channel_chunks(),
        a.tag().c_str(), b.other_channel_chunks(), b.tag().c_str());
    report(
        a.other_channel_chunks() == 0 && b.other_channel_chunks() == 0,
        "no traffic leaked onto the other UART channels", scratch);
}

} // namespace

int main(int argc, char** argv) {
    if (const char* value = std::getenv("HCS_RS485_PORT"))
        g_port = std::atoi(value);
    if (g_port != 1 && g_port != 2) {
        printf("HCS_RS485_PORT must be 1 (USART2, connector U5) or 2 (USART3, P5)\n");
        return 1;
    }
    if (const char* value = std::getenv("HCS_RS485_BAUDRATE"))
        g_baudrate = static_cast<uint32_t>(std::strtoul(value, nullptr, 10));
    if (const char* value = std::getenv("HCS_RS485_ROUNDS"))
        g_ping_pong_rounds = std::atoi(value);
    const std::string mode = argc > 1 ? argv[1] : "";
    const bool sweep_only = mode == "baudrate";
    // Observes without ever transmitting, so it can answer "is the port
    // producing traffic that nobody sent" on its own.
    const bool quiet_only = mode == "quiet";
    const bool latency_only = mode == "latency";
    // Everything that probes a boundary rather than normal operation: the
    // per-port config actually taking effect, the TX ring ceiling, and a queued
    // command burst.
    const bool limits_only = mode == "limits";
    // Request/response only, for a long soak: the shape real master traffic has,
    // repeated HCS_RS485_ROUNDS times.
    const bool ping_pong_only = mode == "pingpong";

    const auto boards = enumerate_boards();
    printf("mc02 boards found: %zu\n", boards.size());
    for (const auto& [serial, product] : boards)
        printf("  %s  %s\n", serial.c_str(), product.c_str());
    if (boards.empty()) {
        printf("\nNo mc02 found.\n");
        return 1;
    }

    // Two rigs, same tests. "cross" is two boards wired A-A / B-B on the same
    // port; "single" is one board with its two RS-485 ports wired to each other,
    // which is the only way to exercise USART3 when just one board is on the
    // bench. HCS_RS485_RIG forces one; the default follows the board count.
    bool single_board = boards.size() < 2;
    if (const char* value = std::getenv("HCS_RS485_RIG"))
        single_board = std::string{value} == "single";
    if (!single_board && boards.size() < 2) {
        printf(
            "\nHCS_RS485_RIG=cross needs two mc02 boards wired A-A / B-B on the RS-485 port "
            "under test (HCS_RS485_PORT=1 -> USART2/U5, 2 -> USART3/P5).\n");
        return 1;
    }

    printf("\nOpening boards...\n");
    std::vector<std::unique_ptr<BoardHost>> hosts;
    hosts.push_back(std::make_unique<BoardHost>(boards[0].first));
    if (!single_board)
        hosts.push_back(std::make_unique<BoardHost>(boards[1].first));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // In the single-board rig the two endpoints are the board's own two ports,
    // so g_port stops meaning "the port under test" -- both are under test.
    Node a{"A", *hosts[0], single_board ? 1 : g_port};
    Node b{"B", single_board ? *hosts[0] : *hosts[1], single_board ? 2 : g_port};

    if (single_board)
        printf(
            "One board, UART2 / USART2 (transceiver U5, kUart2) wired to UART3 / USART3 "
            "(connector P5, kUart3). A = port 1, B = port 2. Setting both to %u baud "
            "(both USARTs power up at 4800000).\n",
            g_baudrate);
    else
        printf(
            "Driving RS-485 port %d (%s). Setting both ends to %u baud "
            "(both USARTs power up at 4800000).\n",
            g_port, g_port == 1 ? "USART2, transceiver U5" : "USART3, connector P5", g_baudrate);
    a.set_baudrate(g_baudrate);
    b.set_baudrate(g_baudrate);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    if (latency_only) {
        test_latency_vs_size(a, b);
        printf(
            "\n%s (%d failure%s)\n", g_failures == 0 ? "ALL PASSED" : "FAILURES", g_failures,
            g_failures == 1 ? "" : "s");
        return g_failures == 0 ? 0 : 1;
    }

    if (quiet_only) {
        test_quiet_bus(a, b);
        printf(
            "\n%s (%d failure%s)\n", g_failures == 0 ? "ALL PASSED" : "FAILURES", g_failures,
            g_failures == 1 ? "" : "s");
        return g_failures == 0 ? 0 : 1;
    }

    // Cheapest possible proof the wiring and the firmware build are right, run
    // before anything that would be confusing to read on a dead link.
    std::string detail;
    a.reset();
    b.reset();
    const bool link = exchange_once(a, b, 8, 0, detail);
    printf("\n== Link check ==\n");
    report(link, "A -> B carries a short message", detail);
    if (!link) {
        printf("\nNothing crossed the bus. Check, in order:\n"
               "  1. Host and firmware agree on silkscreen ids: UART2 is kUart2, UART3\n"
               "     is kUart3. An older host that still writes a spare id for these\n"
               "     ports will drop the session rather than stay silent.\n"
               "  2. A-A and B-B, not A-B (a crossed pair is silent, not corrupt).\n"
               "  3. Termination: 120R is fitted on-board at each end (R16), so a short\n"
               "     link has 60R total, which is correct for two ends.\n");
        return 1;
    }
    settle();

    if (sweep_only) {
        test_baudrates(a, b);
    } else if (ping_pong_only) {
        test_ping_pong(a, b);
    } else if (limits_only) {
        test_baudrate_mismatch(a, b);
        test_oversize(a, b);
        test_burst(a, b);
    } else {
        test_sizes(a, b);
        test_baudrates(a, b);
        test_baudrate_mismatch(a, b);
        test_turnaround(a, b);
        test_ping_pong(a, b);
        test_collision_recovery(a, b);
        test_oversize(a, b);
        test_burst(a, b);
        test_latency_vs_size(a, b);
        test_quiet_bus(a, b);
        test_channel_isolation(a, b);
    }

    printf(
        "\n%s (%d failure%s)\n", g_failures == 0 ? "ALL PASSED" : "FAILURES", g_failures,
        g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
