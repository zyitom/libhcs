#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>

#include <libhcs/board/mc02.hpp>

// Bare USB packet-rate probe for mc02, and the A/B rig for "is the main loop on
// the critical path".
//
// WHY NOT usb_packet_rate: that tool is bound to Hpm5321 and all
// of its modes carry CAN records, so its ceiling is entangled with the CAN wire.
// mc02 needs a measurement with no CAN and no UART line rate in it at all.
//
// WHAT IS MEASURED: one record per USB packet, flooded downlink (host -> board).
// The board deserializes every packet, which is the path in question --
// tud_task_ext, dcd_int_handler and Deserializer::process_stream are three of
// the five functions in the hot working set. Records are addressed to a UART
// port with nothing wired to it, so the port's TX ring fills and the surplus is
// dropped inside the firmware. That drop is the point: it removes the line rate
// from the measurement while leaving the USB and deserialize path fully
// exercised. Bytes delivered to a wire are explicitly not what this counts.
//
// Submission is back-pressured by real USB completions -- the transport blocks
// until a pooled transfer is returned by its completion callback -- so a flood
// loop reports what the controller retires, not how fast memcpy runs.
//
// THE CEILING TO COMPARE AGAINST: mc02's USB is Full-Speed with 64-byte bulk
// endpoints (OTG_HS running FS -- LQFP100 brings out no HS PHY). The often-quoted
// 19 bulk transactions per 1 ms frame is the figure for *full* 64-byte data
// packets; a frame's budget is bytes, not transactions, so small packets fit
// more of them. These records are far below 64 bytes, so no single constant is
// the ceiling here -- which is exactly why the ballast sweep below, and not a
// percentage against a guessed maximum, is what identifies the bottleneck.
//
// THE A/B: build the firmware with -Dlibhcs_APP_LOOP_BALLAST_CYCLES=N to add a
// known busy-wait to each main-loop pass, and with -Dlibhcs_APP_LOOP_PROFILE=ON
// so the board reports its own loop rate on the kUart0 uplink, printed here.
// Sweep N and plot packet rate against loop period:
//
//   rate falls as the loop lengthens -> the loop is on the critical path, and
//       placement / memory-ordering / atomics work buys throughput
//   rate is flat while the loop lengthens -> the ceiling is host scheduling, and
//       board-side optimisation is unfalsifiable until that changes
//
// THE BATCH KNOB: records_per_packet writes N records into one builder, which
// the SDK flushes as ONE USB transfer. Sweeping it separates the two costs that
// a one-record-per-packet flood conflates -- per-transfer overhead (host URB
// submission plus bus turnaround) from per-record cost (deserialize, dispatch).
// If record rate scales with N, the ceiling is per-transfer and batching is the
// escape; if it is flat, the cost is genuinely per record.
//
//   ./mc02_packet_rate [seconds] [payload_bytes] [records_per_packet]

namespace {

using libhcs::board::Mc02;
using Clock = std::chrono::steady_clock;

constexpr int k_default_seconds = 5;
constexpr std::size_t k_default_payload = 8;

int g_seconds = k_default_seconds;
std::size_t g_payload_bytes = k_default_payload;
int g_records_per_packet = 1;

std::string first_board_serial() {
    std::vector<std::string> found;
    DIR* dir = opendir("/sys/bus/usb/devices");
    if (!dir)
        return {};
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
        if (const auto serial = read_line("serial"); !serial.empty())
            found.push_back(serial);
    }
    closedir(dir);
    std::sort(found.begin(), found.end());
    if (found.empty())
        return {};
    // HCS_BOARD_INDEX selects which board when several are attached, so two
    // instances can flood two boards at once. That pair is what separates a
    // shared host-side cost from a per-device one: a software cost paid per URB
    // on the host CPU is shared, so two boards would split one board's rate,
    // while a per-endpoint or bus-side limit lets each board keep its own.
    std::size_t index = 0;
    if (const char* value = std::getenv("HCS_BOARD_INDEX"))
        index = static_cast<std::size_t>(std::strtoul(value, nullptr, 10));
    return index < found.size() ? found[index] : found.front();
}

class Probe final : public Mc02::Callback {
public:
    explicit Probe(std::string_view serial) { board_ = std::make_unique<Mc02>(*this, serial); }

    // The builder flushes when it goes out of scope, so one call is exactly one
    // USB transfer regardless of how many records were written into it.
    void send_packet(std::span<const std::byte> payload, int records) {
        auto builder = board_->start_transmit();
        for (int i = 0; i < records; ++i)
            builder.uart1_transmit({.uart_data = payload, .idle_delimited = true});
    }

    std::vector<std::string> take_profile_lines() {
        const std::lock_guard guard{mutex_};
        return std::exchange(profile_lines_, {});
    }

private:
    // A libhcs_APP_LOOP_PROFILE build emits one plain-text line every 500 ms on
    // the kUart0 uplink: "loop n=<passes> khz=<passes per ms> avg=<cycles>
    // max=<cycles> | <section> <permille> <avg> <max> ...". Printed rather than
    // parsed beyond the loop rate; the format is a debugging aid, not an
    // interface.
    void diagnostic_receive_callback(const libhcs::data::UartDataView& data) override {
        if (data.uart_data.empty() && !data.idle_delimited)
            return;
        const std::lock_guard guard{mutex_};
        pending_.append(
            reinterpret_cast<const char*>(data.uart_data.data()), data.uart_data.size());
        if (data.idle_delimited && !pending_.empty())
            profile_lines_.push_back(std::exchange(pending_, {}));
    }

    std::mutex mutex_;
    std::string pending_;
    std::vector<std::string> profile_lines_;
    std::unique_ptr<Mc02> board_;
};

// "loop n=... khz=<N> ..." -> N, or -1 when the build has no profiler.
int parse_loop_khz(const std::string& line) {
    const auto pos = line.find("khz=");
    if (pos == std::string::npos)
        return -1;
    return std::atoi(line.c_str() + pos + 4);
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 1)
        g_seconds = std::atoi(argv[1]);
    if (argc > 2)
        g_payload_bytes = static_cast<std::size_t>(std::strtoul(argv[2], nullptr, 10));
    if (argc > 3)
        g_records_per_packet = std::atoi(argv[3]);
    if (g_seconds <= 0 || g_payload_bytes == 0 || g_records_per_packet <= 0) {
        printf("usage: mc02_packet_rate [seconds] [payload_bytes] [records_per_packet]\n");
        return 1;
    }

    const auto serial = first_board_serial();
    if (serial.empty()) {
        printf("No mc02 board found (a511:0723).\n");
        return 1;
    }
    printf("Board: %s\n", serial.c_str());
    printf(
        "Flooding %zu-byte UART1 records, %d per USB packet, for %d s.\n", g_payload_bytes,
        g_records_per_packet, g_seconds);
    printf("Nothing is wired to UART1; the firmware drops the surplus, which is intended.\n\n");

    Probe probe{serial};
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    probe.take_profile_lines();

    const std::vector<std::byte> payload(g_payload_bytes, std::byte{0xA5});

    const auto start = Clock::now();
    const auto deadline = start + std::chrono::seconds(g_seconds);
    std::uint64_t packets = 0;
    while (Clock::now() < deadline) {
        // The inner count only reduces how often the clock is read; each
        // send_packet call is still exactly one USB transfer.
        for (int i = 0; i < 256; ++i) {
            probe.send_packet(payload, g_records_per_packet);
            ++packets;
        }
    }
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - start).count();

    const double packets_per_second = static_cast<double>(packets) / elapsed;
    const double records_per_second =
        packets_per_second * static_cast<double>(g_records_per_packet);
    const double bytes_per_second = records_per_second * static_cast<double>(g_payload_bytes);

    printf("packets   : %llu in %.3f s\n", static_cast<unsigned long long>(packets), elapsed);
    printf("packet rate: %.0f USB transfers/s\n", packets_per_second);
    printf(
        "record rate: %.0f records/s (%d per transfer)\n", records_per_second,
        g_records_per_packet);
    printf("payload    : %.0f bytes/s\n", bytes_per_second);

    const auto lines = probe.take_profile_lines();
    int best_khz = -1;
    for (const auto& line : lines)
        best_khz = std::max(best_khz, parse_loop_khz(line));
    if (best_khz < 0) {
        printf("\nNo loop profile on the kUart0 uplink. Rebuild with\n"
               "  -Dlibhcs_APP_LOOP_PROFILE=ON\n"
               "to get the loop rate alongside the packet rate.\n");
    } else {
        printf(
            "\nloop rate  : %d kHz (%.2f us per pass) during the run\n", best_khz,
            best_khz ? 1000.0 / best_khz : 0.0);
        for (const auto& line : lines)
            printf("  %s\n", line.c_str());
    }
    return 0;
}
