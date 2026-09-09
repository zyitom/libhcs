// Two boards listening to ONE IMU UART line: which one hands the data to the
// host first, and by how much.
//
// The rig is an IMU whose TX pin is strapped to a UART RX pin on an mc02 AND to
// a UART RX pin on an hpm5321. Both boards therefore see the same edges at the
// same instant, which is the only reason a useful number can be extracted at
// all: neither board timestamps UART receptions (UartDataView carries no
// timestamp field, unlike the GPIO and CAN views), so the host cannot ask "when
// did this byte arrive at the pin?". What it CAN do is subtract -- the unknown
// wire instant is common to both boards and cancels in A minus B.
//
// So this tool reports two different things, and they must not be confused:
//
//   * DELTA (exact): host arrival time of packet k through A minus through B.
//     The IMU's transmit instant cancels, so this is a true measurement of how
//     much one board's forwarding path costs over the other's.
//   * PHASE JITTER (exact up to one constant): the IMU streams at a fixed
//     period, so a least-squares line through the arrival times of one board is
//     that board's view of the IMU's own clock. The residuals are that path's
//     jitter, absolutely -- only the constant offset is unknowable.
//
// Absolute one-way latency is NOT reported, because it cannot be measured on
// this wiring. It would need either a board-side receive timestamp or a stimulus
// the host itself emits.
//
// Packets are cut on the line-idle flag the firmware already sets, so "arrival"
// means the moment the host learns the packet is complete -- the number a
// consumer of the IMU actually waits for.
//
// Run:
//   ./imu_uart_latency probe   [seconds]   # which port is the IMU actually on
//   ./imu_uart_latency latency [seconds]   # A vs B on the shared line
//
// HCS_BOARD_A / HCS_BOARD_B pick the boards by USB serial (default: first two
// found). HCS_UART_BAUD sets the line rate (default 921600).
// HCS_UART_A_PORT / HCS_UART_B_PORT pick the port on each board (default 0).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <utility>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include <dirent.h>

#include "common/multi_board.hpp"

namespace {

constexpr uint32_t k_default_baudrate = 921600;
constexpr int k_max_ports = 3;

using Clock = std::chrono::steady_clock;

int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
        .count();
}

uint32_t baudrate() {
    const char* value = std::getenv("HCS_UART_BAUD");
    return value ? static_cast<uint32_t>(std::strtoul(value, nullptr, 10)) : k_default_baudrate;
}

// getenv returning nullptr has to be folded to an empty view explicitly: the
// GNU ?: shorthand the other examples use is off the table here (C11/C++23 with
// GNU extensions disabled, see the repository guide).
std::string_view env_or_empty(const char* name) {
    const char* value = std::getenv(name);
    return value ? std::string_view{value} : std::string_view{};
}

int port_env(const char* name) {
    const char* value = std::getenv(name);
    return value ? static_cast<int>(std::strtol(value, nullptr, 10)) : 0;
}

// One uplink chunk exactly as the board published it. The timestamp is taken in
// the receive callback, before any parsing, so the tool's own work stays out of
// the measured interval.
struct Chunk {
    int64_t stamp_ns;
    int port;
    bool idle_delimited;
    std::vector<uint8_t> bytes;
};

class Sink final : public examples::BoardReceiver {
public:
    void on_uart(int port, const libhcs::data::UartDataView& data) override {
        const int64_t stamp = now_ns();
        std::lock_guard<std::mutex> guard{mutex_};
        Chunk& chunk = chunks_.emplace_back();
        chunk.stamp_ns = stamp;
        chunk.port = port;
        chunk.idle_delimited = data.idle_delimited;
        chunk.bytes.reserve(data.uart_data.size());
        for (const std::byte value : data.uart_data)
            chunk.bytes.push_back(std::to_integer<uint8_t>(value));
    }

    std::vector<Chunk> take() {
        std::lock_guard<std::mutex> guard{mutex_};
        return std::move(chunks_);
    }

private:
    std::mutex mutex_;
    std::vector<Chunk> chunks_;
};

// A line-idle-delimited message, reassembled from the chunks that carried it.
// first_ns is when the host saw the leading fragment, last_ns when the firmware
// declared the line idle -- the instant the message is usable.
struct Packet {
    int64_t first_ns;
    int64_t last_ns;
    std::vector<uint8_t> bytes;
};

std::vector<Packet> reassemble(const std::vector<Chunk>& chunks, int port) {
    std::vector<Packet> packets;
    Packet pending{};
    bool open = false;
    for (const Chunk& chunk : chunks) {
        if (chunk.port != port)
            continue;
        if (!open) {
            pending = Packet{chunk.stamp_ns, chunk.stamp_ns, {}};
            open = true;
        }
        pending.bytes.insert(pending.bytes.end(), chunk.bytes.begin(), chunk.bytes.end());
        pending.last_ns = chunk.stamp_ns;
        if (chunk.idle_delimited) {
            // A floating RX pin produces a stream of empty idle-delimited chunks
            // with no bytes behind them. Those are line noise, not messages, and
            // would otherwise be reported as a port carrying zero-byte packets.
            if (!pending.bytes.empty())
                packets.push_back(std::move(pending));
            open = false;
        }
    }
    // The first message is dropped by the caller: capture may have started in
    // the middle of it, which would make its length and timing meaningless.
    return packets;
}

double percentile(std::vector<double> values, double fraction) {
    if (values.empty())
        return 0.0;
    const size_t index = static_cast<size_t>(fraction * static_cast<double>(values.size() - 1));
    std::nth_element(values.begin(), values.begin() + static_cast<ptrdiff_t>(index), values.end());
    return values[index];
}

double mean_of(const std::vector<double>& values) {
    if (values.empty())
        return 0.0;
    double sum = 0.0;
    for (const double value : values)
        sum += value;
    return sum / static_cast<double>(values.size());
}

double stddev_of(const std::vector<double>& values) {
    if (values.size() < 2)
        return 0.0;
    const double average = mean_of(values);
    double sum = 0.0;
    for (const double value : values)
        sum += (value - average) * (value - average);
    return std::sqrt(sum / static_cast<double>(values.size() - 1));
}

void print_stats(const char* label, std::vector<double> values, const char* unit) {
    if (values.empty()) {
        printf("  %-22s (no samples)\n", label);
        return;
    }
    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    printf(
        "  %-22s n=%zu  min %8.1f  p50 %8.1f  p90 %8.1f  p99 %8.1f  max %8.1f  mean %8.1f  "
        "sd %7.1f %s\n",
        label, sorted.size(), sorted.front(), percentile(sorted, 0.50), percentile(sorted, 0.90),
        percentile(sorted, 0.99), sorted.back(), mean_of(sorted), stddev_of(sorted), unit);
}

// Least-squares fit of arrival time against packet index. The IMU's own emission
// is periodic, so the slope recovers its period and the residuals are the path's
// jitter -- the one absolute statement this wiring supports about a single board.
struct Fit {
    double period_us;
    std::vector<double> residual_us;
};

Fit fit_period(const std::vector<int64_t>& stamps_ns) {
    Fit fit{0.0, {}};
    const size_t count = stamps_ns.size();
    if (count < 3)
        return fit;
    double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
    for (size_t i = 0; i < count; ++i) {
        const double x = static_cast<double>(i);
        const double y = static_cast<double>(stamps_ns[i] - stamps_ns[0]) / 1000.0;
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }
    const double n = static_cast<double>(count);
    const double denominator = n * sum_xx - sum_x * sum_x;
    if (denominator == 0.0)
        return fit;
    const double slope = (n * sum_xy - sum_x * sum_y) / denominator;
    const double intercept = (sum_y - slope * sum_x) / n;
    fit.period_us = slope;
    fit.residual_us.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const double x = static_cast<double>(i);
        const double y = static_cast<double>(stamps_ns[i] - stamps_ns[0]) / 1000.0;
        fit.residual_us.push_back(y - (slope * x + intercept));
    }
    return fit;
}

uint64_t hash_bytes(const std::vector<uint8_t>& bytes) {
    uint64_t hash = 1469598103934665603ULL;
    for (const uint8_t value : bytes) {
        hash ^= value;
        hash *= 1099511628211ULL;
    }
    return hash;
}

// connect_any() walks every board adapter in turn and keeps the first that does
// not throw. That works for one board, but not here: the wrong adapter gets far
// enough to open and claim the USB device before it fails, and the device is
// still busy when the correct adapter is tried a moment later -- so the second
// board never opens at all. Reading the PID out of sysfs first and constructing
// exactly one adapter avoids the whole sequence.
struct UsbDevice {
    std::string serial;
    std::string product_id;
};

std::string read_sysfs(const std::string& directory, const char* attribute) {
    std::string path = directory + "/" + attribute;
    FILE* file = std::fopen(path.c_str(), "r");
    if (!file)
        return {};
    char buffer[128] = {};
    const size_t read = std::fread(buffer, 1, sizeof(buffer) - 1, file);
    std::fclose(file);
    std::string value{buffer, read};
    while (!value.empty() && (value.back() == '\n' || value.back() == '\r'))
        value.pop_back();
    return value;
}

std::vector<UsbDevice> project_boards() {
    std::vector<UsbDevice> devices;
    DIR* directory = ::opendir("/sys/bus/usb/devices");
    if (!directory)
        return devices;
    while (const dirent* entry = ::readdir(directory)) {
        const std::string path = std::string{"/sys/bus/usb/devices/"} + entry->d_name;
        if (read_sysfs(path, "idVendor") != "a511")
            continue;
        UsbDevice device{read_sysfs(path, "serial"), read_sysfs(path, "idProduct")};
        if (!device.serial.empty())
            devices.push_back(std::move(device));
    }
    ::closedir(directory);
    std::sort(devices.begin(), devices.end(),
              [](const UsbDevice& lhs, const UsbDevice& rhs) { return lhs.serial < rhs.serial; });
    return devices;
}

std::unique_ptr<examples::BoardSession> connect_by_pid(
    examples::BoardReceiver& receiver, std::string_view serial_filter) {
    for (const UsbDevice& device : project_boards()) {
        if (!serial_filter.empty() && device.serial.find(serial_filter) == std::string::npos)
            continue;
        try {
            if (device.product_id == "0723")
                return std::make_unique<examples::detail::Mc02Session>(receiver, device.serial);
            if (device.product_id == "5322")
                return std::make_unique<examples::detail::Hpm5321DualCanSession>(
                    receiver, device.serial);
            if (device.product_id == "c402")
                return std::make_unique<examples::detail::CBoardSession>(receiver, device.serial);
        } catch (const std::exception& error) {
            fprintf(stderr, "  %s (pid %s): %s\n", device.serial.c_str(),
                    device.product_id.c_str(), error.what());
        }
    }
    return nullptr;
}

struct Rig {
    Sink sink_a, sink_b;
    std::unique_ptr<examples::BoardSession> a, b;

    bool open() {
        a = connect_by_pid(sink_a, env_or_empty("HCS_BOARD_A"));
        b = connect_by_pid(sink_b, env_or_empty("HCS_BOARD_B"));
        // One board is enough for `probe`, which is what tells the operator where
        // the IMU is wired; only `latency` needs the pair. Failing both modes
        // because one board is unreachable would hide the half that still works.
        if (!a && !b) {
            fprintf(stderr, "could not open any board\n");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{300});
        for (const auto& [label, board] : {std::pair{"A", a.get()}, std::pair{"B", b.get()}}) {
            if (board)
                printf("%s = %.*s (%d UART)\n", label, static_cast<int>(board->name().size()),
                       board->name().data(), board->uart_port_count());
            else
                printf("%s = not connected\n", label);
        }
        return true;
    }

    // A board whose UART is still at its build-time default delivers nothing but
    // framing errors from a 921600 IMU, which looks exactly like "not wired".
    // Aligning both ends first removes that ambiguity from every mode below.
    void set_baudrate(uint32_t baud) const {
        for (examples::BoardSession* board : {a.get(), b.get()}) {
            if (!board)
                continue;
            for (int port = 0; port < board->uart_port_count(); ++port) {
                try {
                    board->transmit([&](examples::BoardTransmitter& tx) {
                        tx.uart_config(port, {.baudrate = baud});
                    });
                } catch (const std::exception&) {
                    printf("  (%.*s port %d: no runtime baudrate control)\n",
                           static_cast<int>(board->name().size()), board->name().data(), port);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{200});
    }
};

void report_ports(const char* label, const std::vector<Chunk>& chunks, double seconds) {
    struct PortStats {
        uint32_t chunks;
        uint32_t bytes;
        uint32_t idle;
        std::vector<uint8_t> first;
    };
    PortStats stats[k_max_ports] = {};
    for (const Chunk& chunk : chunks) {
        if (chunk.port < 0 || chunk.port >= k_max_ports)
            continue;
        PortStats& slot = stats[chunk.port];
        slot.chunks++;
        slot.bytes += static_cast<uint32_t>(chunk.bytes.size());
        slot.idle += chunk.idle_delimited ? 1 : 0;
        if (slot.first.empty())
            slot.first = chunk.bytes;
    }
    for (int port = 0; port < k_max_ports; ++port) {
        const PortStats& slot = stats[port];
        if (slot.chunks == 0)
            continue;
        printf(
            "  %s port %d: %u chunks (%u idle-delimited), %u bytes, %.0f B/s", label, port,
            slot.chunks, slot.idle, slot.bytes, static_cast<double>(slot.bytes) / seconds);
        const size_t dump = std::min<size_t>(slot.first.size(), 16);
        printf("  first:");
        for (size_t i = 0; i < dump; ++i)
            printf(" %02X", slot.first[i]);
        printf("\n");
    }
}

void report_board(const char* label, const std::vector<Packet>& packets, uint32_t baud);

int run_probe(Rig& rig, double seconds) {
    printf("\nlistening %.1f s on every UART port of both boards at %u baud\n", seconds,
           baudrate());
    rig.sink_a.take();
    rig.sink_b.take();
    std::this_thread::sleep_for(std::chrono::duration<double>{seconds});
    const std::vector<Chunk> chunks_a = rig.sink_a.take();
    const std::vector<Chunk> chunks_b = rig.sink_b.take();
    if (chunks_a.empty() && chunks_b.empty()) {
        printf("  nothing received on either board -- IMU not powered, not wired, or the\n"
               "  line rate is not %u baud\n", baudrate());
        return 1;
    }
    report_ports("A", chunks_a, seconds);
    report_ports("B", chunks_b, seconds);
    // Per-port timing on whatever is connected: with only one board present this
    // is the whole result, and it is the half that does not need the pair.
    for (const auto& [label, chunks] : {std::pair{"A", &chunks_a}, std::pair{"B", &chunks_b}}) {
        for (int port = 0; port < k_max_ports; ++port) {
            std::vector<Packet> packets = reassemble(*chunks, port);
            if (packets.size() < 4)
                continue;
            packets.erase(packets.begin());
            char title[32];
            std::snprintf(title, sizeof(title), "%s port %d", label, port);
            report_board(title, packets, baudrate());
        }
    }
    return 0;
}

void report_board(const char* label, const std::vector<Packet>& packets, uint32_t baud) {
    if (packets.size() < 3) {
        printf("  %s: %zu packets -- not enough to characterise\n", label, packets.size());
        return;
    }
    std::vector<int64_t> stamps;
    std::vector<double> spread_us, length;
    stamps.reserve(packets.size());
    for (const Packet& packet : packets) {
        stamps.push_back(packet.last_ns);
        spread_us.push_back(static_cast<double>(packet.last_ns - packet.first_ns) / 1000.0);
        length.push_back(static_cast<double>(packet.bytes.size()));
    }
    const Fit fit = fit_period(stamps);
    const double bytes = mean_of(length);
    // 10 bits per byte on the wire: 1 start + 8 data + 1 stop, the framing every
    // board in this repo uses.
    const double wire_us = bytes * 10.0 * 1e6 / static_cast<double>(baud);
    printf("\n%s: %zu packets, mean %.1f bytes (%.1f us on the wire at %u baud)\n", label,
           packets.size(), bytes, wire_us, baud);
    printf("  IMU period from this board's arrivals: %.2f us (%.1f Hz)\n", fit.period_us,
           fit.period_us > 0.0 ? 1e6 / fit.period_us : 0.0);
    print_stats("phase jitter", fit.residual_us, "us");
    print_stats("first->idle spread", spread_us, "us");
}

int run_latency(Rig& rig, double seconds) {
    if (!rig.a || !rig.b) {
        fprintf(stderr, "latency needs both boards on the line; only one is reachable\n");
        return 1;
    }
    const int port_a = port_env("HCS_UART_A_PORT");
    const int port_b = port_env("HCS_UART_B_PORT");
    const uint32_t baud = baudrate();
    printf("\ncapturing %.1f s: A port %d vs B port %d at %u baud\n", seconds, port_a, port_b,
           baud);
    rig.sink_a.take();
    rig.sink_b.take();
    std::this_thread::sleep_for(std::chrono::duration<double>{seconds});
    const std::vector<Chunk> chunks_a = rig.sink_a.take();
    const std::vector<Chunk> chunks_b = rig.sink_b.take();

    std::vector<Packet> packets_a = reassemble(chunks_a, port_a);
    std::vector<Packet> packets_b = reassemble(chunks_b, port_b);
    // Capture can start mid-message on either side; the leading fragment would
    // otherwise be counted as a short packet and would never match its twin.
    if (!packets_a.empty())
        packets_a.erase(packets_a.begin());
    if (!packets_b.empty())
        packets_b.erase(packets_b.begin());
    if (packets_a.size() < 3 || packets_b.size() < 3) {
        fprintf(stderr, "too few packets (A %zu, B %zu) -- run `probe` to find the wired port\n",
                packets_a.size(), packets_b.size());
        return 1;
    }

    report_board("A", packets_a, baud);
    report_board("B", packets_b, baud);

    // The two boards start capturing at different points in the stream, so the
    // pairing is found from the content rather than assumed: identical payloads
    // vote on the index offset, and the winning offset is applied to every pair.
    std::unordered_map<uint64_t, std::vector<size_t>> index_b;
    for (size_t i = 0; i < packets_b.size(); ++i)
        index_b[hash_bytes(packets_b[i].bytes)].push_back(i);
    std::unordered_map<int64_t, uint32_t> votes;
    for (size_t i = 0; i < packets_a.size(); ++i) {
        const auto found = index_b.find(hash_bytes(packets_a[i].bytes));
        if (found == index_b.end() || found->second.size() != 1)
            continue;
        votes[static_cast<int64_t>(found->second[0]) - static_cast<int64_t>(i)]++;
    }
    if (votes.empty()) {
        fprintf(stderr,
                "no packet payload appears uniquely on both boards -- the two ports are not on\n"
                "the same line, or one of them is dropping bytes\n");
        return 1;
    }
    int64_t offset = 0;
    uint32_t best = 0;
    for (const auto& [candidate, count] : votes) {
        if (count > best) {
            best = count;
            offset = candidate;
        }
    }

    std::vector<double> delta_us, delta_first_us;
    uint32_t mismatched = 0;
    for (size_t i = 0; i < packets_a.size(); ++i) {
        const int64_t j = static_cast<int64_t>(i) + offset;
        if (j < 0 || static_cast<size_t>(j) >= packets_b.size())
            continue;
        const Packet& pa = packets_a[i];
        const Packet& pb = packets_b[static_cast<size_t>(j)];
        if (pa.bytes != pb.bytes) {
            mismatched++;
            continue;
        }
        delta_us.push_back(static_cast<double>(pa.last_ns - pb.last_ns) / 1000.0);
        delta_first_us.push_back(static_cast<double>(pa.first_ns - pb.first_ns) / 1000.0);
    }
    if (delta_us.empty()) {
        fprintf(stderr, "packets aligned at offset %lld but none matched byte for byte\n",
                static_cast<long long>(offset));
        return 1;
    }

    // The delta is only worth quoting if it holds still. Both USB links carry the
    // stream as bulk transfers, which are not tied to the frame, but the IMU's
    // 1 kHz and the host's 1 kHz frame clock are close enough that a phase beat
    // between them would drift the number over tens of seconds -- and a single
    // median would hide that completely. Splitting the run says whether it drifts.
    constexpr size_t k_blocks = 6;
    if (delta_us.size() >= k_blocks * 20) {
        printf("\nstability: median delta per %zu-th of the run\n  ", k_blocks);
        const size_t span = delta_us.size() / k_blocks;
        for (size_t block = 0; block < k_blocks; ++block) {
            const auto begin = delta_us.begin() + static_cast<ptrdiff_t>(block * span);
            const auto end = block + 1 == k_blocks
                               ? delta_us.end()
                               : delta_us.begin() + static_cast<ptrdiff_t>((block + 1) * span);
            printf("%8.1f", percentile(std::vector<double>{begin, end}, 0.50));
        }
        printf("  us\n");
    }

    printf("\nA minus B, same IMU packet, %zu matched pairs (index offset %lld, %u mismatched)\n",
           delta_us.size(), static_cast<long long>(offset), mismatched);
    print_stats("delta (idle/complete)", delta_us, "us");
    print_stats("delta (first fragment)", delta_first_us, "us");
    const double median = percentile(delta_us, 0.50);
    printf("\n  => %s delivers the completed IMU packet %.1f us %s than %s (median).\n",
           median > 0.0 ? "B" : "A", std::fabs(median), "earlier", median > 0.0 ? "A" : "B");
    printf("  The IMU's transmit instant is common to both boards and cancels here, so this\n"
           "  difference is exact. Absolute one-way latency is not measurable on this wiring:\n"
           "  no board timestamps UART receptions.\n");
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "probe";
    const double seconds = argc > 2 ? std::strtod(argv[2], nullptr) : 3.0;

    Rig rig;
    if (!rig.open())
        return 1;
    rig.set_baudrate(baudrate());

    if (mode == "probe")
        return run_probe(rig, seconds);
    if (mode == "latency")
        return run_latency(rig, seconds);
    fprintf(stderr, "usage: %s probe|latency [seconds]\n", argv[0]);
    return 2;
}
