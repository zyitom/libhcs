// Topology discovery + the remaining GPIO coverage (the TIM2 pair), designed
// so that no wiring the user could plausibly have made can create an output
// fight: at every instant at most ONE push-pull driver exists on the bench and
// everything else is an input.
//
// Phase 1 (discovery): drive each of the eight pins high, one at a time, and
// read all other pins asap with a pull-down. A pin connected to the driver
// reads high; an isolated pin reads low against its own pull-down. This prints
// the full net map of the bench.
//
// Phase 2 (duty coverage): for every discovered net that spans two pins, drive
// one member through the 25/50/75 % sweep and measure duty + frequency on
// another member. This is what validates the TIM2 pair (PA0/PA2), whose duty
// accuracy was the last uncovered GPIO corner -- its duty conversion runs on
// the 5.5 M count ARR with the 64-bit intermediate.
//
// Usage: mc02_gpio_topology_test [serial_a] [serial_b]

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <libhcs/board/common.hpp>
#include <libhcs/board/mc02.hpp>

namespace {

using namespace std::chrono_literals;

constexpr int kChPa0 = 0;
constexpr int kChPa2 = 1;
constexpr int kChPe9 = 2;
constexpr int kChPe13 = 3;
constexpr uint16_t kDuty0 = 0;
constexpr uint16_t kDuty25 = 16384;
constexpr uint16_t kDuty50 = 32768;
constexpr uint16_t kDuty75 = 49151;
const char* kDefaultSerialA = "D4-46A1-3924-BAEB-DCE9-FBBA-C628";
const char* kDefaultSerialB = "D4-18F5-D4CF-09D2-4903-509F-0696";

struct Pin {
    int board;   // 0 = A, 1 = B
    int channel;
    const char* name;
};

constexpr Pin kPins[8] = {
    {0, kChPa0, "A.PA0"},
    {0, kChPa2, "A.PA2"},
    {0, kChPe9, "A.PE9"},
    {0, kChPe13, "A.PE13"},
    {1, kChPa0, "B.PA0"},
    {1, kChPa2, "B.PA2"},
    {1, kChPe9, "B.PE9"},
    {1, kChPe13, "B.PE13"},
};

class LevelCounters {
public:
    void push(std::size_t key, bool high) {
        const std::lock_guard<std::mutex> lock{mutex_};
        auto& counter = counters_[key];
        if (counter.have_previous && high && !counter.previous_high) ++counter.rising;
        counter.previous_high = high;
        counter.have_previous = true;
        ++counter.total;
        if (high) ++counter.high;
        counter.last_high = high;
    }

    struct Snapshot {
        std::size_t total = 0;
        std::size_t rising = 0;
        std::size_t high = 0;
        bool last_high = false;
    };

    Snapshot snapshot(std::size_t key) {
        const std::lock_guard<std::mutex> lock{mutex_};
        const auto& counter = counters_[key];
        return {.total = counter.total,
                .rising = counter.rising,
                .high = counter.high,
                .last_high = counter.last_high};
    }

private:
    struct Counter {
        std::size_t total = 0;
        std::size_t rising = 0;
        std::size_t high = 0;
        bool previous_high = false;
        bool have_previous = false;
        bool last_high = false;
    };
    std::mutex mutex_;
    Counter counters_[8];
};

class BoardCallback final : public libhcs::board::Mc02::Callback {
public:
    void gpio_digital_read_result_callback(
        const libhcs::spec::mc02::GpioDescriptor& gpio,
        const libhcs::data::GpioDigitalDataView& data) override {
        // Key: board A channels 0-3 -> 0-3, board B channels 0-3 -> 4-7.
        counters_.push(base_ + gpio.channel_index, data.high);
    }

    void set_base(std::size_t base) { base_ = base; }
    LevelCounters& counters() { return counters_; }

private:
    std::size_t base_ = 0;
    LevelCounters counters_;
};

const char* link_name(libhcs::host::protocol::Handler::LinkState state) {
    switch (state) {
    case libhcs::host::protocol::Handler::LinkState::kUp: return "up";
    case libhcs::host::protocol::Handler::LinkState::kSessionDown: return "session-down";
    default: return "FAULTED";
    }
}

void drive(libhcs::board::Mc02& board, int channel, uint16_t duty) {
    board.start_transmit().gpio_analog_write(
        libhcs::spec::mc02::kGpioDescriptors[channel], {.value = duty});
}

void arm(libhcs::board::Mc02& board, int channel, const libhcs::data::GpioReadConfigView& config) {
    board.start_transmit().gpio_digital_read(
        libhcs::spec::mc02::kGpioDescriptors[channel], config);
}

libhcs::data::GpioReadConfigView pull_down_asap_config() {
    return {.period_ms = 0,
            .asap = true,
            .rising_edge = false,
            .falling_edge = false,
            .capture_timestamp = false,
            .pull = libhcs::data::GpioPull::kDown};
}

libhcs::data::GpioReadConfigView continuous_config() {
    return {.period_ms = 1,
            .asap = true,
            .rising_edge = true,
            .falling_edge = true,
            .capture_timestamp = true,
            .pull = libhcs::data::GpioPull::kDown};
}

// Arms one asap read and returns the level of the sample that comes back.
bool asap_level(libhcs::board::Mc02& board, LevelCounters& counters, std::size_t key,
    int channel) {
    const auto before = counters.snapshot(key).total;
    arm(board, channel, pull_down_asap_config());
    std::this_thread::sleep_for(50ms);
    const auto after = counters.snapshot(key);
    return after.total > before && after.last_high;
}

LevelCounters::Snapshot operator-(
    const LevelCounters::Snapshot& after, const LevelCounters::Snapshot& before) {
    return {.total = after.total - before.total,
            .rising = after.rising - before.rising,
            .high = after.high - before.high,
            .last_high = after.last_high};
}

void sleep_us(int64_t us) { std::this_thread::sleep_for(std::chrono::microseconds{us}); }

} // namespace

int main(int argc, char** argv) {
    const std::string serial_a = argc > 1 ? argv[1] : kDefaultSerialA;
    const std::string serial_b = argc > 2 ? argv[2] : kDefaultSerialB;

    BoardCallback callback_a;
    BoardCallback callback_b;
    callback_a.set_base(0);
    callback_b.set_base(4);
    libhcs::board::Mc02 board_a{callback_a, serial_a,
        libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true)};
    libhcs::board::Mc02 board_b{callback_b, serial_b,
        libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true)};
    std::printf("[topo] board A link %s, board B link %s\n", link_name(board_a.link_state()),
        link_name(board_b.link_state()));
    auto& counters_a = callback_a.counters();
    auto& counters_b = callback_b.counters();

    // Everything to quiet pull-down input first.
    for (const int channel : {kChPa0, kChPa2, kChPe9, kChPe13}) {
        arm(board_a, channel, continuous_config());
        arm(board_b, channel, continuous_config());
    }
    sleep_us(300000);

    // ---- Phase 1: net discovery. One driver at a time, all others read.
    std::printf("[topo] discovery: driving each pin high, reading the other seven\n");
    bool connected[8][8] = {};
    for (int driver = 0; driver < 8; ++driver) {
        libhcs::board::Mc02& driver_board = kPins[driver].board == 0 ? board_a : board_b;
        auto& driver_counters = kPins[driver].board == 0 ? counters_a : counters_b;
        const std::size_t driver_key = static_cast<std::size_t>(driver);
        drive(driver_board, kPins[driver].channel, 65535U); // constant high
        sleep_us(30000); // compare preloads in at the next period boundary
        for (int reader = 0; reader < 8; ++reader) {
            if (reader == driver) continue;
            libhcs::board::Mc02& reader_board = kPins[reader].board == 0 ? board_a : board_b;
            auto& reader_counters = kPins[reader].board == 0 ? counters_a : counters_b;
            const bool high = asap_level(reader_board, reader_counters,
                static_cast<std::size_t>(reader), kPins[reader].channel);
            connected[driver][reader] = high;
            connected[reader][driver] = high; // continuity is symmetric
        }
        // Driver back to quiet input before the next iteration.
        drive(driver_board, kPins[driver].channel, kDuty0);
        arm(driver_board, kPins[driver].channel, continuous_config());
        sleep_us(30000);
    }
    std::printf("[topo] connectivity (driver high -> reader high):\n");
    for (int driver = 0; driver < 8; ++driver) {
        std::printf("  %-7s drives high, reads high on:", kPins[driver].name);
        for (int reader = 0; reader < 8; ++reader)
            if (connected[driver][reader]) std::printf(" %s", kPins[reader].name);
        std::printf("\n");
    }

    // ---- Phase 2: duty sweep per discovered net. Driver = first member,
    // recorder = second member (never the same pin).
    std::printf("[topo] duty sweep per net\n");
    bool all_pass = true;
    bool tim2_covered = false;
    for (int member = 0; member < 8; ++member) {
        bool is_driver = false;
        for (int other = 0; other < 8; ++other)
            if (connected[member][other]) is_driver = true;
        if (!is_driver) continue;
        // Pick the first connected partner as recorder.
        int recorder = -1;
        for (int other = 0; other < 8; ++other) {
            if (connected[member][other]) {
                recorder = other;
                break;
            }
        }
        if (recorder < 0) continue;
        // Each undirected net once: skip if the partner has a smaller index
        // (it already drove this net).
        bool partner_drives = false;
        for (int other = 0; other < 8; ++other)
            if (other < member && connected[member][other]) partner_drives = true;
        if (partner_drives) continue;

        libhcs::board::Mc02& driver_board = kPins[member].board == 0 ? board_a : board_b;
        libhcs::board::Mc02& recorder_board = kPins[recorder].board == 0 ? board_a : board_b;
        auto& recorder_counters =
            kPins[recorder].board == 0 ? counters_a : counters_b;
        const std::size_t recorder_key = static_cast<std::size_t>(recorder);
        const int channel = kPins[member].channel;
        arm(recorder_board, kPins[recorder].channel, continuous_config());
        std::printf("  net [%s - %s] (channel %d):\n", kPins[member].name, kPins[recorder].name,
            channel);
        bool net_pass = true;
        for (const uint16_t duty : {kDuty25, kDuty50, kDuty75}) {
            drive(driver_board, channel, duty);
            sleep_us(30000); // compare preloads in
            const auto before = recorder_counters.snapshot(recorder_key);
            sleep_us(1500000);
            drive(driver_board, channel, kDuty0);
            sleep_us(30000);
            const auto window = recorder_counters.snapshot(recorder_key) - before;
            const double duty_pct =
                window.total > 0
                    ? 100.0 * static_cast<double>(window.high) / static_cast<double>(window.total)
                    : 0.0;
            const double target = 100.0 * static_cast<double>(duty) / 65535.0;
            const double hz = static_cast<double>(window.rising) / 1.5;
            const bool ok = window.total > 500 && duty_pct >= target - 4.0
                && duty_pct <= target + 4.0 && hz >= 47.0 && hz <= 53.0;
            net_pass = net_pass && ok;
            if (channel == kChPa0 || channel == kChPa2) tim2_covered = true;
            std::printf("    duty %5.1f%% -> %5.1f%%, %5.1f Hz  %s\n", target, duty_pct, hz,
                ok ? "ok" : "FAIL");
        }
        drive(driver_board, channel, kDuty0);
        arm(driver_board, channel, continuous_config());
        all_pass = all_pass && net_pass;
    }
    std::printf("[topo] TIM2 (PA0/PA2) duty coverage: %s\n", tim2_covered ? "COVERED" : "still "
                                                                                       "missing");
    std::printf("[topo] RESULT: %s\n", all_pass ? "PASS" : "FAIL");
    return all_pass ? 0 : 1;
}
