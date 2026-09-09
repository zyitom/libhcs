// Step 1 of the USB-SOF cross-board time base: does FRINDEX advance by exactly
// one per SOF interrupt, on every board at once?
//
// The design that this validates uses the device controller's frame index as the
// precision source of a shared timeline -- it is updated by hardware from the
// host's SOF packets, so every board on the same host controller counts the same
// edges, with no software path to add skew. Two things can break that before any
// of the algorithm is written, and both look like a working counter:
//
//   * the SOF-received status flag leading the FRINDEX increment, so a handler
//     that reads the register at interrupt entry samples the PREVIOUS value.
//     Signature: a delta histogram concentrated on 0 and 2 instead of on 1, and
//     a nonzero "advanced within ISR" count. Cost if missed: a fixed 125 us
//     offset, on a design that budgets ~1 us.
//   * missed interrupts. Signature: deltas above 1, or -- worse, because the
//     delta stays 1 -- an ISR-to-ISR interval that is a multiple of 125 us. The
//     interval column is the only thing that can see the second case.
//
// Run it with every board attached; the tool reads them all at once, because
// "each board is individually consistent" is a weaker claim than the design
// needs. The cross-board section pairs each board's records against the first
// board's by arrival time and reports
//
//     (frame_i - frame_0) - (arrival_i - arrival_0) / 125 us
//
// which is zero when both boards are counting the same microframes. What matters
// there is that the residual is CONSTANT and small: a fixed offset is just the
// two records having been produced at slightly different points, while a drifting
// one means the two counters are not the same clock.
//
// Requires firmware built with -Dlibhcs_SOF_DIAG=ON. Without it the boards emit
// no records and this prints the "no records" warning.
//
// The optional rate argument puts a paced CAN load on every board while
// sampling, which is not decoration: the SOF handler shares its interrupt vector
// with the device stack and its core with the CAN driver, so "delta is always 1"
// on idle boards is a weaker claim than the design needs. It is what decides
// whether the missed-interrupt branch of the timeline state machine is a real
// path or dead code. The load is always paced, never a flood -- start_transmit()
// blocks when the transfer pool empties, and a blocked sender cannot see the
// stop flag.
//
// Run:
//   ./sof_probe [seconds] [rate_per_board] [serial ...]

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <cmath>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dirent.h>

#include <libhcs/board/hpm5321.hpp>


// CAN ports are named as the silkscreen on THAT board. 5321 prints CAN1/CAN2.
// See libhcs/board/hcs_can_port.hpp.
using libhcs::board::hcs::CanPort;

namespace {

using Clock = std::chrono::steady_clock;

// Must match firmware/hpm_board/app/src/sync/sof_probe.hpp.
constexpr uint8_t kRecordMagic = 0xD2U;
constexpr uint8_t kRecordVersion = 2U;
constexpr size_t kDeltaBuckets = 10;
constexpr size_t kAnomalyCapacity = 12;
constexpr size_t kAnomalyWords = 7;
constexpr size_t kFixedSize = 4 + 4 * (18 + kDeltaBuckets);

constexpr uint32_t kFrindexMask = 0x3FFFU;
constexpr double kMicroframeUs = 125.0;

uint32_t get_u32_le(const std::byte* p) {
    return static_cast<uint32_t>(std::to_integer<uint8_t>(p[0]))
         | static_cast<uint32_t>(std::to_integer<uint8_t>(p[1])) << 8
         | static_cast<uint32_t>(std::to_integer<uint8_t>(p[2])) << 16
         | static_cast<uint32_t>(std::to_integer<uint8_t>(p[3])) << 24;
}

struct Anomaly {
    uint32_t previous_frame;
    uint32_t current_frame;
    uint32_t delta;
    uint32_t interval_ticks;
    uint32_t usbsts;
    uint32_t portsc1;
    uint32_t timestamp;
    uint32_t record_tick_ms;
};

struct Record {
    uint8_t sequence;
    bool sof_enabled;
    uint32_t tick_ms;
    uint32_t sof_count;
    std::array<uint32_t, kDeltaBuckets> histogram;
    uint32_t advanced_within_isr;
    uint32_t anomaly_total;
    uint32_t last_frame;
    uint32_t microframe_low;
    uint32_t microframe_high;
    uint32_t emit_frame;
    uint32_t interval_min;
    uint32_t interval_max;
    uint64_t interval_sum;
    uint32_t interval_count;
    uint32_t timer_hz;
    std::vector<Anomaly> anomalies;
    Clock::time_point arrival;
};

bool decode(std::span<const std::byte> payload, Record& out) {
    if (payload.size() < kFixedSize)
        return false;
    if (std::to_integer<uint8_t>(payload[0]) != kRecordMagic)
        return false;
    if (std::to_integer<uint8_t>(payload[1]) != kRecordVersion)
        return false;
    const std::byte* const base = payload.data();
    if (get_u32_le(base + 4) != payload.size())
        return false;

    out.sequence = std::to_integer<uint8_t>(payload[2]);
    out.sof_enabled = (std::to_integer<uint8_t>(payload[3]) & 0x01U) != 0U;
    const std::byte* cursor = base + 8;
    const auto next = [&cursor]() {
        const uint32_t value = get_u32_le(cursor);
        cursor += 4;
        return value;
    };
    out.tick_ms = next();
    out.sof_count = next();
    for (uint32_t& bucket : out.histogram)
        bucket = next();
    out.advanced_within_isr = next();
    out.anomaly_total = next();
    out.last_frame = next();
    (void)next(); // last SOF timestamp; only the firmware needs it
    out.microframe_low = next();
    out.microframe_high = next();
    out.emit_frame = next();
    (void)next(); // emit timestamp, ditto
    out.interval_min = next();
    out.interval_max = next();
    const uint32_t sum_low = next();
    const uint32_t sum_high = next();
    out.interval_sum = (static_cast<uint64_t>(sum_high) << 32U) | sum_low;
    out.interval_count = next();
    out.timer_hz = next();

    const uint32_t stored = next();
    if (stored > kAnomalyCapacity)
        return false;
    if (payload.size() != kFixedSize + stored * kAnomalyWords * 4U)
        return false;
    out.anomalies.clear();
    for (uint32_t index = 0; index < stored; index++) {
        Anomaly anomaly{};
        anomaly.previous_frame = next();
        anomaly.current_frame = next();
        anomaly.delta = next();
        anomaly.interval_ticks = next();
        anomaly.usbsts = next();
        anomaly.portsc1 = next();
        anomaly.timestamp = next();
        anomaly.record_tick_ms = out.tick_ms;
        out.anomalies.push_back(anomaly);
    }
    return true;
}

class Receiver final : public libhcs::board::Hpm5321::Callback {
public:
    std::vector<Record> take() {
        const std::scoped_lock guard{mutex_};
        return records_;
    }

    uint64_t foreign_payloads() const {
        const std::scoped_lock guard{mutex_};
        return foreign_;
    }

private:
    void uart0_receive_callback(const libhcs::data::UartDataView& data) override {
        Record record{};
        record.arrival = Clock::now();
        if (!decode(data.uart_data, record)) {
            const std::scoped_lock guard{mutex_};
            foreign_++;
            return;
        }
        const std::scoped_lock guard{mutex_};
        records_.push_back(std::move(record));
    }

    mutable std::mutex mutex_;
    std::vector<Record> records_;
    uint64_t foreign_ = 0;
};

// Serials come from sysfs, not libusb: opening a board is what we are trying to
// choose, so the query cannot need a handle first.
std::vector<std::string> enumerate_boards() {
    std::vector<std::string> found;
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
        if (read_line("idVendor") != "a511" || read_line("idProduct") != "5322")
            continue;
        const std::string serial = read_line("serial");
        if (!serial.empty())
            found.push_back(serial);
    }
    closedir(dir);
    std::sort(found.begin(), found.end());
    return found;
}

// 14-bit difference mapped to the signed range, so a wrap does not read as an
// 8000-microframe jump.
int32_t signed_frame_delta(uint32_t later, uint32_t earlier) {
    const int32_t raw = static_cast<int32_t>((later - earlier) & kFrindexMask);
    return raw >= 0x2000 ? raw - 0x4000 : raw;
}

// Only the bits that can explain a frame index that stopped advancing. The port
// state is what separates "the bus was not running" from "the counter is broken",
// and those two have opposite consequences for the design.
std::string describe_portsc(uint32_t portsc1) {
    static constexpr std::array<const char*, 4> kSpeed{"full", "low", "high", "undefined"};
    std::string out{kSpeed[(portsc1 >> 26U) & 0x3U]};
    out += "-speed";
    if ((portsc1 & (1U << 0U)) == 0U)
        out += ", disconnected";
    if ((portsc1 & (1U << 7U)) != 0U)
        out += ", suspended";
    if ((portsc1 & (1U << 8U)) != 0U)
        out += ", reset";
    if ((portsc1 & (1U << 23U)) != 0U)
        out += ", PHY-low-power";
    if ((portsc1 & (1U << 2U)) != 0U)
        out += ", port-enable-change";
    return out;
}

void report_board(const std::string& serial, const std::vector<Record>& records) {
    printf("\n=== board %s ===\n", serial.c_str());
    if (records.empty()) {
        printf("  no records -- is the firmware built with -Dlibhcs_SOF_DIAG=ON?\n");
        return;
    }
    const Record& first = records.front();
    const Record& last = records.back();
    if (!last.sof_enabled)
        printf("  WARNING: the SOF interrupt enable is not set in USBINTR\n");
    if (last.sof_count == 0) {
        printf("  records arrived but NO SOF interrupt ever fired (sof_count 0).\n");
        return;
    }

    // Sequence numbers are a byte, and records are only dropped when the uplink
    // batch pool is full; a gap is worth seeing because a lost record takes a
    // period of interval statistics with it.
    uint32_t gaps = 0;
    for (size_t index = 1; index < records.size(); index++) {
        const uint8_t expected = static_cast<uint8_t>(records[index - 1].sequence + 1U);
        if (records[index].sequence != expected)
            gaps++;
    }

    const uint64_t microframes =
        (static_cast<uint64_t>(last.microframe_high) << 32U) | last.microframe_low;
    printf(
        "  records %zu (seq gaps %u)   SOF interrupts %u   microframes counted %llu\n",
        records.size(), gaps, last.sof_count, static_cast<unsigned long long>(microframes));

    // EVERY counter in the record is free-running since boot, and the board spends
    // its first moments enumerating -- a window in which the port is full-speed,
    // the frame index is pinned at zero and the controller still raises SOF every
    // millisecond. Those transitions are real but they are not what the design
    // runs on, and left in the totals they drown the steady state. So the primary
    // figures below are differences against the FIRST record, which is the one
    // that carries the whole pre-session history.
    std::array<uint32_t, kDeltaBuckets> histogram{};
    uint64_t transitions = 0;
    for (size_t bucket = 0; bucket < kDeltaBuckets; bucket++) {
        histogram[bucket] = last.histogram[bucket] - first.histogram[bucket];
        transitions += histogram[bucket];
    }
    const uint32_t anomalies = last.anomaly_total - first.anomaly_total;
    const uint32_t races = last.advanced_within_isr - first.advanced_within_isr;

    printf(
        "  --- steady state (%zu records after the first, %llu transitions) ---\n",
        records.size() - 1, static_cast<unsigned long long>(transitions));
    for (size_t bucket = 0; bucket < kDeltaBuckets; bucket++) {
        if (histogram[bucket] == 0)
            continue;
        const double share =
            transitions == 0 ? 0.0 : 100.0 * histogram[bucket] / static_cast<double>(transitions);
        if (bucket + 1 == kDeltaBuckets)
            printf("    delta >=%zu : %10u  %8.5f%%\n", bucket, histogram[bucket], share);
        else
            printf("    delta  %zu  : %10u  %8.5f%%\n", bucket, histogram[bucket], share);
    }
    printf(
        "    anomalies %u   FRINDEX advanced inside the ISR %u\n", anomalies, races);

    // Per-record anomaly counts: a design that must never silently mis-count
    // cares less about the rate than about whether the events cluster (one bad
    // moment) or scatter (a standing hazard).
    uint32_t dirty_records = 0;
    for (size_t index = 1; index < records.size(); index++) {
        if (records[index].anomaly_total != records[index - 1].anomaly_total)
            dirty_records++;
    }
    printf(
        "    100 ms periods containing an anomaly: %u of %zu\n", dirty_records,
        records.size() - 1);

    uint32_t min_ticks = 0xFFFFFFFFU;
    uint32_t max_ticks = 0;
    uint64_t sum_ticks = 0;
    uint64_t samples = 0;
    for (size_t index = 1; index < records.size(); index++) {
        const Record& record = records[index];
        if (record.interval_count == 0)
            continue;
        min_ticks = std::min(min_ticks, record.interval_min);
        max_ticks = std::max(max_ticks, record.interval_max);
        sum_ticks += record.interval_sum;
        samples += record.interval_count;
    }
    const double us_per_tick = last.timer_hz == 0 ? 0.0 : 1e6 / static_cast<double>(last.timer_hz);
    if (samples != 0) {
        printf(
            "    ISR-to-ISR interval us: min %.3f  mean %.4f  max %.3f  (n=%llu)\n",
            min_ticks * us_per_tick,
            static_cast<double>(sum_ticks) / static_cast<double>(samples) * us_per_tick,
            max_ticks * us_per_tick, static_cast<unsigned long long>(samples));
    }

    printf(
        "  --- since boot, including the pre-session window ---\n"
        "    delta 0 %u   delta 1 %u   anomalies %u   advanced inside the ISR %u\n",
        last.histogram[0], last.histogram[1], last.anomaly_total, last.advanced_within_isr);

    // Aggregate captured anomaly detail over the WHOLE run, grouped by the fields
    // that separate the possible causes. Version 1 of this record kept only 8
    // entries per period; they filled during enumeration and everything after
    // went unseen, which is why the port state and timestamp are in the entry now.
    struct Group {
        uint32_t delta;
        uint32_t portsc1;
        uint32_t usbsts;
        uint64_t count;
        uint32_t interval_min;
        uint32_t interval_max;
        uint32_t first_tick_ms;
        uint32_t last_tick_ms;
        uint32_t sample_frame;
    };
    std::vector<Group> groups;
    uint64_t captured = 0;
    for (const Record& record : records) {
        for (const Anomaly& anomaly : record.anomalies) {
            captured++;
            const auto match = std::find_if(groups.begin(), groups.end(), [&](const Group& group) {
                return group.delta == anomaly.delta && group.portsc1 == anomaly.portsc1
                    && group.usbsts == anomaly.usbsts;
            });
            if (match == groups.end()) {
                groups.push_back(
                    {anomaly.delta, anomaly.portsc1, anomaly.usbsts, 1, anomaly.interval_ticks,
                     anomaly.interval_ticks, anomaly.record_tick_ms, anomaly.record_tick_ms,
                     anomaly.current_frame});
                continue;
            }
            match->count++;
            match->interval_min = std::min(match->interval_min, anomaly.interval_ticks);
            match->interval_max = std::max(match->interval_max, anomaly.interval_ticks);
            match->last_tick_ms = anomaly.record_tick_ms;
        }
    }
    if (groups.empty())
        return;
    std::sort(groups.begin(), groups.end(), [](const Group& left, const Group& right) {
        return left.count > right.count;
    });
    printf(
        "  captured anomaly detail (%llu of %u; the ring holds %zu per 100 ms):\n",
        static_cast<unsigned long long>(captured), last.anomaly_total, kAnomalyCapacity);
    for (const Group& group : groups) {
        printf(
            "    delta %u  PORTSC1 0x%08X  USBSTS 0x%08X  n=%llu\n"
            "      interval us %.3f..%.3f   board ms %u..%u   e.g. FRINDEX 0x%04X   port %s\n",
            group.delta, group.portsc1, group.usbsts,
            static_cast<unsigned long long>(group.count), group.interval_min * us_per_tick,
            group.interval_max * us_per_tick, group.first_tick_ms, group.last_tick_ms,
            group.sample_frame, describe_portsc(group.portsc1).c_str());
    }
}

void report_cross_board(
    const std::vector<std::string>& serials, const std::vector<std::vector<Record>>& records) {
    if (serials.size() < 2)
        return;
    printf("\n=== cross-board frame agreement (reference: %s) ===\n", serials[0].c_str());
    const std::vector<Record>& reference = records[0];
    if (reference.empty()) {
        printf("  reference board produced no records\n");
        return;
    }

    for (size_t board = 1; board < serials.size(); board++) {
        std::vector<int32_t> residuals;
        for (const Record& record : records[board]) {
            // Nearest reference record in arrival time. Both boards emit at
            // 10 Hz, so the pair is at most ~50 ms apart -- far inside the
            // 2.048 s FRINDEX wrap that signed_frame_delta() has to survive.
            const Record* best = nullptr;
            double best_gap = 0.0;
            for (const Record& candidate : reference) {
                const double gap = std::chrono::duration<double, std::micro>{
                    record.arrival - candidate.arrival}
                                       .count();
                if (best == nullptr || std::abs(gap) < std::abs(best_gap)) {
                    best = &candidate;
                    best_gap = gap;
                }
            }
            if (best == nullptr)
                continue;
            const int32_t frame_delta = signed_frame_delta(record.emit_frame, best->emit_frame);
            const auto time_delta =
                static_cast<int32_t>(std::llround(best_gap / kMicroframeUs));
            residuals.push_back(frame_delta - time_delta);
        }
        if (residuals.empty()) {
            printf("  %s: no records\n", serials[board].c_str());
            continue;
        }
        std::sort(residuals.begin(), residuals.end());
        double sum = 0.0;
        for (const int32_t value : residuals)
            sum += value;
        printf(
            "  %s: n=%zu  residual microframes  min %d  p50 %d  max %d  mean %.2f\n",
            serials[board].c_str(), residuals.size(), residuals.front(),
            residuals[residuals.size() / 2], residuals.back(),
            sum / static_cast<double>(residuals.size()));
    }
    printf(
        "  A constant residual is the two records being produced at slightly different\n"
        "  points plus uplink transit; a DRIFTING one means the counters are not the\n"
        "  same clock, which is what would sink the design.\n");
}

} // namespace

int main(int argc, char** argv) {
    const int duration_s = argc > 1 ? std::atoi(argv[1]) : 20;
    if (duration_s <= 0) {
        fprintf(stderr, "seconds must be positive\n");
        return 1;
    }

    const uint32_t rate = argc > 2 ? static_cast<uint32_t>(std::atoi(argv[2])) : 0;

    std::vector<std::string> serials;
    for (int index = 3; index < argc; index++)
        serials.emplace_back(argv[index]);
    if (serials.empty())
        serials = enumerate_boards();
    if (serials.empty()) {
        fprintf(stderr, "no hpm5321_dual_can boards (a511:5322) found\n");
        return 1;
    }

    printf("boards: %zu\n", serials.size());
    for (const std::string& serial : serials)
        printf("  %s\n", serial.c_str());

    std::vector<std::unique_ptr<Receiver>> receivers;
    std::vector<std::unique_ptr<libhcs::board::Hpm5321>> boards;
    try {
        for (const std::string& serial : serials) {
            receivers.push_back(std::make_unique<Receiver>());
            boards.push_back(std::make_unique<libhcs::board::Hpm5321>(
                *receivers.back(), serial));
        }
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    std::atomic<bool> running{true};
    std::atomic<uint64_t> sent{0};
    std::thread load;
    if (rate > 0) {
        load = std::thread{[&]() {
            const std::array<std::byte, 8> payload{};
            const auto period = std::chrono::duration_cast<Clock::duration>(
                std::chrono::nanoseconds{1'000'000'000U / rate});
            auto next = Clock::now();
            while (running.load(std::memory_order_relaxed)) {
                next += period;
                for (auto& board : boards) {
                    auto builder = board->start_transmit();
                    builder.can_transmit(
                        CanPort::kCan1, {.can_id = 0x5A0, .can_data = payload, .is_fdcan = true});
                    sent.fetch_add(1, std::memory_order_relaxed);
                }
                // Clamp: once next falls behind now, "next += period" degenerates
                // into a flood, and the flood is what deadlocks the join.
                const auto now = Clock::now();
                if (next < now)
                    next = now;
                std::this_thread::sleep_until(next);
            }
        }};
        printf("load: %u CAN frames/s per board\n", rate);
    }

    printf("sampling for %d s ...\n", duration_s);
    std::this_thread::sleep_for(std::chrono::seconds{duration_s});
    running.store(false, std::memory_order_relaxed);
    if (load.joinable())
        load.join();
    if (rate > 0)
        printf(
            "load packets pushed: %llu (%.0f/s)\n", static_cast<unsigned long long>(sent.load()),
            static_cast<double>(sent.load()) / duration_s);

    std::vector<std::vector<Record>> records;
    records.reserve(receivers.size());
    for (size_t index = 0; index < receivers.size(); index++) {
        records.push_back(receivers[index]->take());
        const uint64_t foreign = receivers[index]->foreign_payloads();
        if (foreign != 0)
            printf(
                "note: %s delivered %llu UART0 payloads that are not SOF records\n",
                serials[index].c_str(), static_cast<unsigned long long>(foreign));
    }

    for (size_t index = 0; index < serials.size(); index++)
        report_board(serials[index], records[index]);
    report_cross_board(serials, records);
    return 0;
}
