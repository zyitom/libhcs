#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dirent.h>

#include "common/multi_board.hpp"

// Cross-board UART regression test for two mc02 boards wired TX<->RX on the same
// port. Port indices match Mc02Session / the enclosure silkscreen: 0 = UART1
// (USART1), 1 = UART2 (USART2 RS-485), 2 = UART3 (USART3 RS-485), 3 = UART7,
// 4 = UART10 (USART10). This program is the TTL suite -- pick 0, 3, or 4 with
// HCS_UART_PORT. Modes: no argument runs the full suite on that port, "monitor"
// isolates a dead direction (see below), and "concurrent [ports...]" drives
// several wired ports at once. RS-485 is host/examples/rs485_cross_test.
//
// Worth covering both 0 and 3 rather than either alone: UART7 runs with the
// USART FIFO enabled and USART1 with it disabled (usart.c MX_UART7_Init vs
// MX_USART1_UART_Init), and the transmit path's idle window keys off ISR.TC
// precisely because a 16-entry TXFIFO puts "DMA finished" and "line drained" up
// to 173 us apart at 921600 baud. Only the FIFO-enabled port exercises that.
//
// This exists because a same-board loopback cannot fail the way this firmware
// can. Looping UART7 back to UART10 on one board proves the two ends agree with
// each other, not that either end is doing what the host asked -- a baudrate
// request that is silently ignored leaves both ends at 115200 and the loop still
// passes at every requested rate (see firmware/mc02/AGENTS.md). Every check here
// therefore crosses a wire between two independent boards.
//
// Framing contract the firmware presents, which the reassembly below relies on:
// RxBuffer::try_dequeue() publishes a chunk as soon as kMinFragmentSize (32)
// bytes are in the ring, without waiting for the line to go idle. An
// idle-delimited message therefore arrives as one or more non-idle chunks
// followed by a chunk carrying idle_delimited -- and when the message length is
// an exact multiple of 32 that last chunk is empty. A message is "everything up
// to and including the next chunk flagged idle_delimited", never "one chunk".

namespace {

constexpr int k_default_port = 4; // UART10
constexpr uint32_t k_base_baudrate = 115200;
constexpr uint32_t k_fast_baudrate = 921600;
constexpr std::size_t k_stream_chunk = 256;
constexpr int k_port_count = 5; // mc02 silkscreen UART1 / UART2 / UART3 / UART7 / UART10

int g_port = k_default_port;
std::size_t g_stream_bytes = 4096;
// Printing the firmware's diagnostic channel would interleave with the test
// tables, so it is opt-in through HCS_UART_DIAG=1.
bool g_print_diagnostic = false;

// Serials come from sysfs, not libusb: picking a board by serial is exactly what
// a libusb handle would be needed for. Same approach as dual_board_test.
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

// One board plus the reassembly state for the port under test. Chunks arriving
// on any other port are counted rather than dropped, so cross-port leakage shows
// up as a number instead of silently corrupting the message under test.
class Node final : public examples::BoardReceiver {
public:
    bool open(std::string_view serial, std::string_view tag) {
        tag_ = tag;
        session_ = examples::connect_any(*this, serial);
        return session_ != nullptr;
    }

    void reset() { reset_port(g_port); }

    // Per-port, so the concurrent mode can drive two wired ports at once without
    // one port's traffic landing in the other's accumulator.
    void reset_port(int port) {
        const std::lock_guard guard{mutex_};
        auto& p = slot(port);
        p.partial.clear();
        p.messages.clear();
        p.all_bytes.clear();
        p.chunk_count = 0;
        p.byte_count = 0;
        other_port_chunks_ = 0;
    }

    std::vector<std::byte> all_bytes_of(int port) {
        const std::lock_guard guard{mutex_};
        return slot(port).all_bytes;
    }

    std::size_t all_byte_count_of(int port) {
        const std::lock_guard guard{mutex_};
        return slot(port).all_bytes.size();
    }

    // Every byte received since the last reset, ignoring idle boundaries. A
    // sender that paces chunks apart -- as the stream test must, to stay under
    // the wire rate -- leaves the line idle between them, so the receiving
    // firmware correctly reports an idle boundary after each chunk. Framing is
    // checked by the frame tests; the stream test only cares that the byte
    // sequence survived.
    std::vector<std::byte> all_bytes() { return all_bytes_of(g_port); }

    std::size_t all_byte_count() { return all_byte_count_of(g_port); }

    std::size_t message_count() {
        const std::lock_guard guard{mutex_};
        return slot(g_port).messages.size();
    }

    std::vector<std::vector<std::byte>> take_messages() {
        const std::lock_guard guard{mutex_};
        return std::exchange(slot(g_port).messages, {});
    }

    std::size_t pending_bytes() {
        const std::lock_guard guard{mutex_};
        return slot(g_port).partial.size();
    }

    std::size_t chunk_count() {
        const std::lock_guard guard{mutex_};
        return slot(g_port).chunk_count;
    }

    std::size_t other_port_chunks() {
        const std::lock_guard guard{mutex_};
        return other_port_chunks_;
    }

    void send(std::span<const std::byte> payload, bool idle_delimited) {
        send_on(g_port, payload, idle_delimited);
    }

    // The SDK's packet builder is not reentrant, so the concurrent mode's two
    // sender threads have to serialise around it; the wire is what the test is
    // loading, not the USB submit path.
    void send_on(int port, std::span<const std::byte> payload, bool idle_delimited) {
        const std::lock_guard guard{transmit_mutex_};
        session_->transmit([&](examples::BoardTransmitter& tx) {
            tx.uart(port, {.uart_data = payload, .idle_delimited = idle_delimited});
        });
    }

    void set_baudrate(uint32_t baudrate) { set_baudrate_on(g_port, baudrate); }

    void set_baudrate_on(int port, uint32_t baudrate) {
        const std::lock_guard guard{transmit_mutex_};
        session_->transmit(
            [&](examples::BoardTransmitter& tx) { tx.uart_config(port, {.baudrate = baudrate}); });
    }

private:
    // kUart0 carries the firmware's diagnostic channel (see
    // firmware/mc02/app/src/diag/), which a libhcs_APP_LOOP_PROFILE build uses
    // to emit one plain-text main-loop timing line every 500 ms. Printed rather
    // than parsed: the format is a debugging aid, not an interface.
    void on_diagnostic(const libhcs::data::UartDataView& data) override {
        if (!g_print_diagnostic || data.uart_data.empty())
            return;
        const std::lock_guard guard{mutex_};
        diagnostic_.append(
            reinterpret_cast<const char*>(data.uart_data.data()), data.uart_data.size());
        if (data.idle_delimited) {
            printf("  [%s] %s\n", tag_.c_str(), diagnostic_.c_str());
            fflush(stdout);
            diagnostic_.clear();
        }
    }

    void on_uart(int port, const libhcs::data::UartDataView& data) override {
        const std::lock_guard guard{mutex_};
        if (port < 0 || port >= k_port_count) {
            ++other_port_chunks_;
            return;
        }
        auto& p = slot(port);
        ++p.chunk_count;
        p.byte_count += data.uart_data.size();
        p.all_bytes.insert(p.all_bytes.end(), data.uart_data.begin(), data.uart_data.end());
        p.partial.insert(p.partial.end(), data.uart_data.begin(), data.uart_data.end());
        if (data.idle_delimited)
            p.messages.push_back(std::exchange(p.partial, {}));
    }

    struct PortState {
        std::vector<std::byte> partial;
        std::vector<std::byte> all_bytes;
        std::vector<std::vector<std::byte>> messages;
        std::size_t chunk_count = 0;
        std::size_t byte_count = 0;
    };

    PortState& slot(int port) { return ports_[static_cast<std::size_t>(port)]; }

    std::unique_ptr<examples::BoardSession> session_;
    std::string tag_;
    std::string diagnostic_;
    std::mutex mutex_;
    std::mutex transmit_mutex_;
    std::array<PortState, k_port_count> ports_;
    std::size_t other_port_chunks_ = 0;
};

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

bool wait_for_messages(Node& node, std::size_t count, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (node.message_count() >= count)
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    return node.message_count() >= count;
}

int g_failures = 0;

void report(const char* name, bool ok, const std::string& detail) {
    printf("[%s] %-26s %s\n", ok ? " ok " : "FAIL", name, detail.c_str());
    if (!ok)
        ++g_failures;
}

// One idle-delimited message across the wire, reassembled on the far side.
// Returns the received bytes, or an empty optional-ish flag through `received`.
bool exchange_once(
    Node& from, Node& to, std::span<const std::byte> payload, std::chrono::milliseconds timeout,
    std::vector<std::byte>& received) {
    // Let anything the previous case left in flight land before the reset, so a
    // late idle chunk from it cannot be counted as this case's reply.
    std::this_thread::sleep_for(std::chrono::milliseconds{80});
    to.reset();
    from.send(payload, true);
    const bool arrived = wait_for_messages(to, 1, timeout);
    auto messages = to.take_messages();
    if (!messages.empty())
        received = std::move(messages.front());
    else
        received.clear();
    return arrived;
}

void test_link(Node& a, Node& b) {
    const auto payload = make_pattern(8, 1);
    std::vector<std::byte> received;

    bool ok = exchange_once(a, b, payload, std::chrono::milliseconds{1000}, received);
    ok = ok && received.size() == payload.size()
      && first_difference(payload, received) == payload.size();
    report("link A->B", ok, ok ? "8 bytes" : "no/!= payload, check wiring and GND");

    ok = exchange_once(b, a, payload, std::chrono::milliseconds{1000}, received);
    ok = ok && received.size() == payload.size()
      && first_difference(payload, received) == payload.size();
    report("link B->A", ok, ok ? "8 bytes" : "no/!= payload, check wiring and GND");
}

// The regression that motivated the ring buffer: the previous one-shot
// ReceiveToIdle_DMA path lost whatever arrived while the HAL had the stream
// aborted, once per 64-byte buffer. Also the only check that makes the write
// position wrap the 2048-byte ring, repeatedly.
void test_stream(Node& from, Node& to, const char* name, uint32_t baudrate) {
    const auto payload = make_pattern(g_stream_bytes, 7);
    std::this_thread::sleep_for(std::chrono::milliseconds{80});
    to.reset();

    // Paced to stay under the wire rate: the firmware's TX ring is 2048 bytes,
    // and dumping the whole message at once would overrun it -- the drop would
    // then look like a receive fault. The 30% margin leaves a gap between
    // chunks, so the receiving firmware reports an idle boundary after each one;
    // that is expected here and is why this check ignores framing and compares
    // the byte stream.
    const auto chunk_period = std::chrono::microseconds{
        static_cast<int64_t>(k_stream_chunk) * 10 * 1000000 / baudrate * 13 / 10};

    auto next = std::chrono::steady_clock::now();
    for (std::size_t offset = 0; offset < payload.size(); offset += k_stream_chunk) {
        const auto size = std::min(k_stream_chunk, payload.size() - offset);
        const bool last = offset + size >= payload.size();
        from.send(std::span{payload}.subspan(offset, size), last);
        next += chunk_period;
        std::this_thread::sleep_until(next);
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{3000};
    while (to.all_byte_count() < payload.size() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
    const auto received = to.all_bytes();

    char detail[256];
    if (received.size() == payload.size()
        && first_difference(payload, received) == payload.size()) {
        snprintf(
            detail, sizeof(detail), "%zu bytes exact in %zu chunks", received.size(),
            to.chunk_count());
        report(name, true, detail);
    } else {
        snprintf(
            detail, sizeof(detail), "sent %zu, got %zu, first diff at %zu", payload.size(),
            received.size(), first_difference(payload, received));
        report(name, false, detail);
    }
}

// Exact multiples of the 32-byte publish threshold are the case where the chunk
// carrying idle_delimited is empty; the message must still be delimited.
void test_frame_sizes(Node& from, Node& to) {
    for (const std::size_t size :
         {std::size_t{1}, std::size_t{31}, std::size_t{32}, std::size_t{64}, std::size_t{128},
          std::size_t{256}}) {
        const auto payload = make_pattern(size, static_cast<uint32_t>(size));
        std::vector<std::byte> received;
        const bool arrived =
            exchange_once(from, to, payload, std::chrono::milliseconds{1500}, received);
        const bool ok =
            arrived && received.size() == size && first_difference(payload, received) == size;

        char name[64];
        char detail[128];
        snprintf(name, sizeof(name), "frame %zu bytes", size);
        snprintf(
            detail, sizeof(detail), "got %zu bytes in %zu chunks", received.size(),
            to.chunk_count());
        report(name, ok, detail);
    }
}

// Two idle-delimited messages sent back to back must stay two messages. The
// previous transmit path concatenated whatever had accumulated into one DMA
// burst and dropped the flag, so the far side saw a single run of bytes.
void test_idle_separation(Node& from, Node& to) {
    const auto first = make_pattern(40, 11);
    const auto second = make_pattern(40, 12);

    std::this_thread::sleep_for(std::chrono::milliseconds{80});
    to.reset();
    from.send(first, true);
    from.send(second, true);

    const bool arrived = wait_for_messages(to, 2, std::chrono::milliseconds{2000});
    auto messages = to.take_messages();

    char detail[192];
    const bool ok = arrived && messages.size() == 2 && messages[0].size() == first.size()
                 && messages[1].size() == second.size()
                 && first_difference(first, messages[0]) == first.size()
                 && first_difference(second, messages[1]) == second.size();
    snprintf(
        detail, sizeof(detail), "%zu messages%s", messages.size(),
        messages.size() == 1 ? " (concatenated)" : "");
    report("idle separation", ok, detail);
}

// The check a same-board loopback structurally cannot make: switch one end only
// and require the link to break, then switch the other and require it back.
void test_baudrate(Node& a, Node& b) {
    const auto payload = make_pattern(64, 21);
    std::vector<std::byte> received;

    a.set_baudrate(k_fast_baudrate);
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
    exchange_once(a, b, payload, std::chrono::milliseconds{500}, received);
    const bool broke =
        received.size() != payload.size() || first_difference(payload, received) != payload.size();
    report(
        "baud mismatch breaks link", broke,
        broke ? "A at 921600, B at 115200: payload did not survive"
              : "payload survived a 8x baudrate mismatch -- BRR was never written");

    b.set_baudrate(k_fast_baudrate);
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
    // The mismatched traffic above left partial garbage in B's ring; drain it.
    b.reset();
    bool ok = exchange_once(a, b, payload, std::chrono::milliseconds{1000}, received);
    ok = ok && received.size() == payload.size()
      && first_difference(payload, received) == payload.size();
    report("baud match restores link", ok, ok ? "both at 921600" : "still broken at 921600");

    // A stream at the higher rate, to confirm the ring keeps up when the wire is
    // 8x faster than everything above ran at.
    if (ok)
        test_stream(a, b, "stream A->B @921600", k_fast_baudrate);

    a.set_baudrate(k_base_baudrate);
    b.set_baudrate(k_base_baudrate);
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
    a.reset();
    b.reset();
    ok = exchange_once(a, b, payload, std::chrono::milliseconds{1000}, received);
    ok = ok && received.size() == payload.size()
      && first_difference(payload, received) == payload.size();
    report("baud restore to 115200", ok, ok ? "both back at 115200" : "restore failed");
}

// Two wired ports driven at once, both directions, at the highest rate the rig
// runs. This is the load the single-port cases never produce: four DMA streams
// per board active simultaneously, all four rings in D2 SRAM, plus the USB
// uplink carrying both ports' traffic. If moving the rings out of AXI SRAM into
// the DMA controllers' own domain had introduced a contention or coherency
// problem, this is where it would show.
void test_concurrent(Node& a, Node& b, const std::vector<int>& ports, uint32_t baudrate) {
    for (const int port : ports) {
        a.set_baudrate_on(port, baudrate);
        b.set_baudrate_on(port, baudrate);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
    for (const int port : ports) {
        a.reset_port(port);
        b.reset_port(port);
    }

    const auto chunk_period = std::chrono::microseconds{
        static_cast<int64_t>(k_stream_chunk) * 10 * 1000000 / baudrate * 13 / 10};

    struct Leg {
        Node* from;
        Node* to;
        int port;
        std::vector<std::byte> payload;
    };
    std::vector<Leg> legs;
    for (const int port : ports) {
        legs.push_back(
            {&a, &b, port, make_pattern(g_stream_bytes, static_cast<uint32_t>(100 + port))});
        legs.push_back(
            {&b, &a, port, make_pattern(g_stream_bytes, static_cast<uint32_t>(200 + port))});
    }

    std::vector<std::thread> senders;
    senders.reserve(legs.size());
    for (const auto& leg : legs) {
        senders.emplace_back([&leg, chunk_period] {
            auto next = std::chrono::steady_clock::now();
            for (std::size_t offset = 0; offset < leg.payload.size(); offset += k_stream_chunk) {
                const auto size = std::min(k_stream_chunk, leg.payload.size() - offset);
                const bool last = offset + size >= leg.payload.size();
                leg.from->send_on(leg.port, std::span{leg.payload}.subspan(offset, size), last);
                next += chunk_period;
                std::this_thread::sleep_until(next);
            }
        });
    }
    for (auto& sender : senders)
        sender.join();

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{5000};
    while (std::chrono::steady_clock::now() < deadline) {
        bool complete = true;
        for (const auto& leg : legs)
            complete = complete && leg.to->all_byte_count_of(leg.port) >= leg.payload.size();
        if (complete)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }

    for (const auto& leg : legs) {
        const auto received = leg.to->all_bytes_of(leg.port);
        const bool ok = received.size() == leg.payload.size()
                     && first_difference(leg.payload, received) == leg.payload.size();
        char name[64];
        char detail[160];
        snprintf(
            name, sizeof(name), "concurrent p%d %s", leg.port, leg.from == &a ? "A->B" : "B->A");
        snprintf(
            detail, sizeof(detail), "sent %zu, got %zu, first diff at %zu", leg.payload.size(),
            received.size(), first_difference(leg.payload, received));
        report(name, ok, detail);
    }
}

// Push every wired port at (or past) wire rate until something gives, and report
// what did. The correctness modes above deliberately pace below the wire rate, so
// they prove the path is exact -- they say nothing about where its ceiling is.
//
// Bytes are counted, not compared: once a ring overflows the stream loses a hole
// in the middle and a positional compare would only report "differs at N". The
// interesting numbers are offered vs delivered per leg, and the aggregate rate.
//
// Where the loss appears tells you which stage gave way:
//   delivered < offered, board LED flashing downlink-full  -> the TX ring filled;
//     the host offered faster than the wire drains.
//   delivered < offered, LED flashing uplink-full          -> the uplink batch
//     pool filled; USB could not carry what the UARTs received.
//   delivered == offered at every rate tried               -> no ceiling found;
//     raise the baud or add a port rather than reasoning about it.
void test_saturate(
    Node& a, Node& b, const std::vector<int>& ports, uint32_t baudrate, int seconds,
    int percent_of_wire) {
    for (const int port : ports) {
        a.set_baudrate_on(port, baudrate);
        b.set_baudrate_on(port, baudrate);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{150});
    for (const int port : ports) {
        a.reset_port(port);
        b.reset_port(port);
    }

    // 10 bits on the wire per byte (8N1). At 100 percent this is exactly the time
    // the chunk occupies the line, so the sender offers precisely wire rate.
    const auto chunk_period = std::chrono::nanoseconds{
        static_cast<int64_t>(k_stream_chunk) * 10 * 1000000000LL / baudrate * 100
        / percent_of_wire};

    struct Leg {
        Node* from;
        Node* to;
        int port;
        std::atomic<uint64_t> offered{0};
    };
    std::vector<std::unique_ptr<Leg>> legs;
    for (const int port : ports) {
        legs.push_back(std::unique_ptr<Leg>{
            new Leg{&a, &b, port, {}}
        });
        legs.push_back(std::unique_ptr<Leg>{
            new Leg{&b, &a, port, {}}
        });
    }

    const auto payload = make_pattern(k_stream_chunk, 42);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{seconds};

    std::vector<std::thread> senders;
    senders.reserve(legs.size());
    for (const auto& leg : legs) {
        senders.emplace_back([&leg, &payload, chunk_period, deadline] {
            auto next = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() < deadline) {
                leg->from->send_on(leg->port, payload, false);
                leg->offered.fetch_add(payload.size(), std::memory_order::relaxed);
                next += chunk_period;
                std::this_thread::sleep_until(next);
            }
        });
    }
    const auto started = std::chrono::steady_clock::now();
    for (auto& sender : senders)
        sender.join();

    // Let the tail drain: the last chunks are still on the wire and in the rings.
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
    const auto elapsed = std::chrono::duration<double>{
        std::chrono::steady_clock::now() - started
        - std::chrono::milliseconds{
            500}}.count();

    printf(
        "\n%u baud, %d%% of wire, %d s, chunk %zu B\n", baudrate, percent_of_wire, seconds,
        k_stream_chunk);
    printf("%-14s %12s %12s %8s %10s\n", "leg", "offered", "delivered", "loss", "KB/s");

    uint64_t total_offered = 0;
    uint64_t total_delivered = 0;
    for (const auto& leg : legs) {
        const auto offered = leg->offered.load(std::memory_order::relaxed);
        const auto delivered = static_cast<uint64_t>(leg->to->all_byte_count_of(leg->port));
        total_offered += offered;
        total_delivered += delivered;
        char name[32];
        snprintf(name, sizeof(name), "p%d %s", leg->port, leg->from == &a ? "A->B" : "B->A");
        printf(
            "%-14s %12llu %12llu %7.2f%% %10.1f\n", name, static_cast<unsigned long long>(offered),
            static_cast<unsigned long long>(delivered),
            offered
                ? 100.0 * static_cast<double>(offered - delivered) / static_cast<double>(offered)
                : 0.0,
            static_cast<double>(delivered) / elapsed / 1024.0);
    }
    const double loss = total_offered ? 100.0 * static_cast<double>(total_offered - total_delivered)
                                            / static_cast<double>(total_offered)
                                      : 0.0;
    printf(
        "%-14s %12llu %12llu %7.2f%% %10.1f\n", "TOTAL",
        static_cast<unsigned long long>(total_offered),
        static_cast<unsigned long long>(total_delivered), loss,
        static_cast<double>(total_delivered) / elapsed / 1024.0);
}

} // namespace

int main(int argc, char** argv) {
    if (const char* value = std::getenv("HCS_UART_PORT"))
        g_port = std::atoi(value);
    if (g_port < 0 || g_port >= k_port_count) {
        fprintf(
            stderr,
            "HCS_UART_PORT must be 0..4 (UART1/UART2/UART3/UART7/UART10); TTL suite is 0, 3, or 4\n");
        return 1;
    }
    if (const char* value = std::getenv("HCS_UART_DIAG"))
        g_print_diagnostic = std::atoi(value) != 0;
    if (const char* value = std::getenv("HCS_UART_BYTES"))
        g_stream_bytes = static_cast<std::size_t>(std::atoll(value));

    const auto found = enumerate_boards();
    if (argc > 1 && std::strcmp(argv[1], "list") == 0) {
        printf("mc02 boards attached: %zu\n", found.size());
        for (const auto& [serial, product] : found)
            printf("  %s   %s\n", serial.c_str(), product.c_str());
        return found.size() >= 2 ? 0 : 1;
    }
    if (found.size() < 2) {
        fprintf(stderr, "need two mc02 boards (a511:0723), found %zu\n", found.size());
        return 1;
    }

    std::string serial_a = found[0].first;
    std::string serial_b = found[1].first;
    if (const char* value = std::getenv("HCS_BOARD_A"))
        serial_a = value;
    if (const char* value = std::getenv("HCS_BOARD_B"))
        serial_b = value;

    Node a;
    Node b;
    if (!a.open(serial_a, "A")) {
        fprintf(stderr, "failed to open board A (%s)\n", serial_a.c_str());
        return 1;
    }
    if (!b.open(serial_b, "B")) {
        fprintf(stderr, "failed to open board B (%s)\n", serial_b.c_str());
        return 1;
    }

    printf(
        "A = %s\nB = %s\nport index %d, stream %zu bytes\n\n", serial_a.c_str(), serial_b.c_str(),
        g_port, g_stream_bytes);

    // Separates "the far end never transmitted" from "this end is picking up
    // something on a floating pin": phase 1 has nobody transmitting, so any chunk
    // reported is self-generated.
    if (argc > 1 && std::strcmp(argv[1], "saturate") == 0) {
        std::vector<int> ports;
        for (int i = 2; i < argc; ++i)
            ports.push_back(std::atoi(argv[i]));
        if (ports.empty())
            ports = {0, 3};
        uint32_t baud = k_fast_baudrate;
        int seconds = 5;
        int percent = 100;
        if (const char* value = std::getenv("HCS_UART_BAUD"))
            baud = static_cast<uint32_t>(std::atol(value));
        if (const char* value = std::getenv("HCS_UART_SECONDS"))
            seconds = std::atoi(value);
        if (const char* value = std::getenv("HCS_UART_PERCENT"))
            percent = std::atoi(value);
        test_saturate(a, b, ports, baud, seconds, percent);
        for (const int port : ports) {
            a.set_baudrate_on(port, k_base_baudrate);
            b.set_baudrate_on(port, k_base_baudrate);
        }
        return 0;
    }

    if (argc > 1 && std::strcmp(argv[1], "concurrent") == 0) {
        std::vector<int> ports;
        for (int i = 2; i < argc; ++i)
            ports.push_back(std::atoi(argv[i]));
        if (ports.empty())
            ports = {0, 3};
        test_concurrent(a, b, ports, k_fast_baudrate);
        for (const int port : ports) {
            a.set_baudrate_on(port, k_base_baudrate);
            b.set_baudrate_on(port, k_base_baudrate);
        }
        printf(
            "\n%s (%d failure%s)\n", g_failures ? "FAILED" : "PASSED", g_failures,
            g_failures == 1 ? "" : "s");
        return g_failures ? 1 : 0;
    }

    if (argc > 1 && std::strcmp(argv[1], "monitor") == 0) {
        a.reset();
        b.reset();
        std::this_thread::sleep_for(std::chrono::seconds{3});
        printf(
            "silent 3s : A chunks=%zu bytes=%zu | B chunks=%zu bytes=%zu\n", a.chunk_count(),
            a.all_byte_count(), b.chunk_count(), b.all_byte_count());

        const auto probe = make_pattern(64, 3);
        a.reset();
        b.reset();
        for (int i = 0; i < 10; ++i) {
            a.send(probe, true);
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        }
        printf(
            "A sent 10x64B : B chunks=%zu bytes=%zu (A saw chunks=%zu bytes=%zu)\n",
            b.chunk_count(), b.all_byte_count(), a.chunk_count(), a.all_byte_count());

        a.reset();
        b.reset();
        for (int i = 0; i < 10; ++i) {
            b.send(probe, true);
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        }
        printf(
            "B sent 10x64B : A chunks=%zu bytes=%zu (B saw chunks=%zu bytes=%zu)\n",
            a.chunk_count(), a.all_byte_count(), b.chunk_count(), b.all_byte_count());
        return 0;
    }

    // Both ends to a known rate first: the CubeMX default for UART10 is 115200,
    // but a previous run of this test may have left them elsewhere.
    a.set_baudrate(k_base_baudrate);
    b.set_baudrate(k_base_baudrate);
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
    a.reset();
    b.reset();

    test_link(a, b);
    test_frame_sizes(a, b);
    test_idle_separation(a, b);
    test_stream(a, b, "stream A->B @115200", k_base_baudrate);
    test_stream(b, a, "stream B->A @115200", k_base_baudrate);
    test_baudrate(a, b);

    if (a.other_port_chunks() || b.other_port_chunks())
        printf(
            "\nnote: chunks on other ports A=%zu B=%zu\n", a.other_port_chunks(),
            b.other_port_chunks());

    printf(
        "\n%s (%d failure%s)\n", g_failures ? "FAILED" : "PASSED", g_failures,
        g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
