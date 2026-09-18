// Full-matrix hardware verification of the mc02 GPIO driver (firmware
// mc02/app/src/gpio). Both boards run the current-tree firmware and talk over
// the normal board::Mc02 stack.
//
// Wiring: PE9<->PE9 (channel 2) and PE13<->PE13 (channel 3), GND<->GND.
// Channels 0/1 (PA0/PA2, the TIM2 pair) are NOT wired on this bench: they are
// still driven (case 8) to prove cross-timer independence on the wired pair,
// but their own duty accuracy needs PA0/PA2 jumpers to observe.
//
// Cases:
//   1  digital write high/low -> constant level seen by the recorder
//   2  duty sweep 0 / 25 / 50 / 75 / 100 % -> duty + 50 Hz frequency accuracy
//   3  asap one-shot read matches the driven level
//   4  periodic sampling rate tracks period_ms (10 ms and 1 ms)
//   5  edge capture: rising-only / falling-only / both, plus timestamp
//      presence following capture_timestamp
//   6  pulls: pull-up and pull-down each define the shared idle line and both
//      boards read the same level
//   7  input -> output -> input churn regression (the EXTI disarm path)
//   8  all four channels driven at once; the wired TIM1 pair stays exact,
//      proving TIM2 PWM activity does not disturb it
//
// Exactly one side drives a wire at any moment; handovers re-arm the previous
// driver as an input first.
//
// Usage: mc02_gpio_matrix [serial_a] [serial_b]

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include <libhcs/board/common.hpp>
#include <libhcs/board/mc02.hpp>
#include <libhcs/protocol/handler.hpp>

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
constexpr uint16_t kDuty100 = 65535;
const char* kDefaultSerialA = "D4-46A1-3924-BAEB-DCE9-FBBA-C628";
const char* kDefaultSerialB = "D4-18F5-D4CF-09D2-4903-509F-0696";

// Cumulative per-channel counters; windows are deltas between snapshots.
class LevelCounters {
public:
    void push(std::size_t channel, bool high, bool has_timestamp) {
        const std::lock_guard<std::mutex> lock{mutex_};
        auto& counter = counters_[channel];
        if (counter.have_previous && high && !counter.previous_high) ++counter.rising;
        counter.previous_high = high;
        counter.have_previous = true;
        ++counter.total;
        if (high) ++counter.high;
        if (has_timestamp) ++counter.timestamped;
        counter.last_high = high;
    }

    struct Snapshot {
        std::size_t total = 0;
        std::size_t rising = 0;
        std::size_t high = 0;
        std::size_t timestamped = 0;
        bool last_high = false;
    };

    Snapshot snapshot(std::size_t channel) {
        const std::lock_guard<std::mutex> lock{mutex_};
        const auto& counter = counters_[channel];
        return {.total = counter.total,
                .rising = counter.rising,
                .high = counter.high,
                .timestamped = counter.timestamped,
                .last_high = counter.last_high};
    }

private:
    struct Counter {
        std::size_t total = 0;
        std::size_t rising = 0;
        std::size_t high = 0;
        std::size_t timestamped = 0;
        bool previous_high = false;
        bool have_previous = false;
        bool last_high = false;
    };
    std::mutex mutex_;
    Counter counters_[4];
};

LevelCounters::Snapshot operator-(
    const LevelCounters::Snapshot& after, const LevelCounters::Snapshot& before) {
    return {.total = after.total - before.total,
            .rising = after.rising - before.rising,
            .high = after.high - before.high,
            .timestamped = after.timestamped - before.timestamped,
            .last_high = after.last_high};
}

class BoardCallback final : public libhcs::board::Mc02::Callback {
public:
    void gpio_digital_read_result_callback(
        const libhcs::spec::mc02::GpioDescriptor& gpio,
        const libhcs::data::GpioDigitalDataView& data) override {
        counters_.push(gpio.channel_index, data.high, data.timestamp_quarter_us.has_value());
    }

    LevelCounters& counters() { return counters_; }

private:
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

void digital_write(libhcs::board::Mc02& board, int channel, bool high) {
    board.start_transmit().gpio_digital_write(
        libhcs::spec::mc02::kGpioDescriptors[channel], {.high = high});
}

void arm(libhcs::board::Mc02& board, int channel, const libhcs::data::GpioReadConfigView& config) {
    board.start_transmit().gpio_digital_read(
        libhcs::spec::mc02::kGpioDescriptors[channel], config);
}

libhcs::data::GpioReadConfigView continuous_config() {
    return {.period_ms = 1,
            .asap = true,
            .rising_edge = true,
            .falling_edge = true,
            .capture_timestamp = true,
            .pull = libhcs::data::GpioPull::kDown};
}

libhcs::data::GpioReadConfigView edge_only_config(bool rising, bool falling, bool timestamped) {
    return {.period_ms = 0,
            .asap = false,
            .rising_edge = rising,
            .falling_edge = falling,
            .capture_timestamp = timestamped,
            .pull = libhcs::data::GpioPull::kDown};
}

libhcs::data::GpioReadConfigView pull_config(libhcs::data::GpioPull pull) {
    return {.period_ms = 0,
            .asap = true,
            .rising_edge = false,
            .falling_edge = false,
            .capture_timestamp = false,
            .pull = pull};
}

// Arms one asap read with the given config and returns the level of the
// sample that comes back.
bool asap_level(
    libhcs::board::Mc02& board, LevelCounters& counters, int channel,
    const libhcs::data::GpioReadConfigView& config) {
    const auto index = static_cast<std::size_t>(channel);
    const auto before = counters.snapshot(index).total;
    arm(board, channel, config);
    std::this_thread::sleep_for(200ms);
    const auto after = counters.snapshot(index);
    return after.total > before && after.last_high;
}

void sleep_us(int64_t us) { std::this_thread::sleep_for(std::chrono::microseconds{us}); }

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string serial_a = argc > 1 ? argv[1] : kDefaultSerialA;
    const std::string serial_b = argc > 2 ? argv[2] : kDefaultSerialB;

    BoardCallback callback_a;
    BoardCallback callback_b;
    auto board_a = std::make_unique<libhcs::board::Mc02>(callback_a, serial_a,
        libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true));
    auto board_b = std::make_unique<libhcs::board::Mc02>(callback_b, serial_b,
        libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true));
    std::printf("[matrix] board A link %s, board B link %s\n", link_name(board_a->link_state()),
        link_name(board_b->link_state()));

    auto& counters_a = callback_a.counters();
    auto& counters_b = callback_b.counters();
    (void)counters_a;

    // Everything starts as a quiet pull-down input on both boards.
    for (const int channel : {kChPa0, kChPa2, kChPe9, kChPe13}) {
        arm(*board_a, channel, continuous_config());
        arm(*board_b, channel, continuous_config());
    }
    sleep_us(300000);

    // ---- Case 1: digital write constant levels.
    {
        std::printf("[case 1] digital write\n");
        bool pass = true;
        for (const int channel : {kChPe9, kChPe13}) {
            const auto index = static_cast<std::size_t>(channel);
            const auto base = counters_b.snapshot(index);
            digital_write(*board_a, channel, true);
            sleep_us(30000); // the compare preloads in at the next period boundary
            const auto high_start = counters_b.snapshot(index);
            sleep_us(400000);
            const auto high_window = counters_b.snapshot(index) - high_start;
            digital_write(*board_a, channel, false);
            sleep_us(30000);
            const auto low_start = counters_b.snapshot(index);
            sleep_us(400000);
            const auto low_window = counters_b.snapshot(index) - low_start;
            const bool high_ok =
                high_window.total > 300 && high_window.high == high_window.total;
            const bool low_ok = low_window.total > 300 && low_window.high == 0;
            pass = pass && high_ok && low_ok;
            std::printf("  ch%d: high %zu/%zu samples, low %zu/%zu high -> %s\n", channel,
                high_window.high, high_window.total, low_window.high, low_window.total,
                high_ok && low_ok ? "ok" : "FAIL");
            arm(*board_a, channel, continuous_config()); // driver back to input
        }
        std::printf("[case 1] RESULT: %s\n", pass ? "PASS" : "FAIL");
    }

    // ---- Case 2: duty sweep on both wired channels.
    {
        std::printf("[case 2] duty sweep\n");
        bool pass = true;
        for (const int channel : {kChPe9, kChPe13}) {
            const auto index = static_cast<std::size_t>(channel);
            arm(*board_b, channel, continuous_config());
            for (const uint16_t duty : {kDuty0, kDuty25, kDuty50, kDuty75, kDuty100}) {
                drive(*board_a, channel, duty);
                sleep_us(30000); // let the compare preload in at the period boundary
                const auto before = counters_b.snapshot(index);
                sleep_us(1500000);
                drive(*board_a, channel, kDuty0);
                sleep_us(30000); // let the zero-compare land before the snapshot
                const auto window = counters_b.snapshot(index) - before;
                const double duty_pct =
                    window.total > 0 ? 100.0 * static_cast<double>(window.high)
                                           / static_cast<double>(window.total)
                                     : 0.0;
                const double target = 100.0 * static_cast<double>(duty) / 65535.0;
                const double hz =
                    static_cast<double>(window.rising) / 1.5; // 1.5 s steady-state window
                const bool duty_ok =
                    window.total > 300 && duty_pct >= target - 4.0 && duty_pct <= target + 4.0;
                const bool freq_ok =
                    duty == kDuty0 || duty == kDuty100 || (hz >= 47.0 && hz <= 53.0);
                pass = pass && duty_ok && freq_ok;
                std::printf("  ch%d duty %5.1f%% -> %5.1f%%, %5.1f Hz  %s\n", channel, target,
                    duty_pct, hz, duty_ok && freq_ok ? "ok" : "FAIL");
            }
            arm(*board_a, channel, continuous_config());
        }
        std::printf("[case 2] RESULT: %s\n", pass ? "PASS" : "FAIL");
    }

    // ---- Case 3: asap one-shot read follows the driven level.
    {
        std::printf("[case 3] asap read\n");
        bool pass = true;
        for (const int channel : {kChPe9, kChPe13}) {
            const auto none = pull_config(libhcs::data::GpioPull::kNone);
            digital_write(*board_a, channel, true);
            sleep_us(100000);
            const bool high_seen = asap_level(*board_b, counters_b, channel, none);
            digital_write(*board_a, channel, false);
            sleep_us(100000);
            const bool low_seen = !asap_level(*board_b, counters_b, channel, none);
            arm(*board_a, channel, continuous_config());
            pass = pass && high_seen && low_seen;
            std::printf("  ch%d: high seen %d, low seen %d\n", channel, high_seen, low_seen);
        }
        std::printf("[case 3] RESULT: %s\n", pass ? "PASS" : "FAIL");
    }

    // ---- Case 4: periodic sampling rate tracks period_ms. Board A drives a
    // 50 % square so levels keep changing; the rate is the recorder's sample
    // count per second.
    {
        std::printf("[case 4] periodic sampling rate\n");
        bool pass = true;
        drive(*board_a, kChPe9, kDuty50);
        for (const uint16_t period_ms : {10U, 1U}) {
            const auto index = static_cast<std::size_t>(kChPe9);
            arm(*board_b, kChPe9,
                {.period_ms = period_ms,
                 .asap = true,
                 .rising_edge = false,
                 .falling_edge = false,
                 .capture_timestamp = false,
                 .pull = libhcs::data::GpioPull::kDown});
            sleep_us(300000);
            const auto base = counters_b.snapshot(index).total;
            sleep_us(1000000);
            const std::size_t rate = counters_b.snapshot(index).total - base;
            const std::size_t expected = period_ms == 10U ? 100U : 1000U;
            const bool ok = rate > expected / 2 && rate < expected * 2;
            pass = pass && ok;
            std::printf("  period %2u ms -> %zu samples/s (expect ~%zu) %s\n", period_ms, rate,
                expected, ok ? "ok" : "FAIL");
        }
        drive(*board_a, kChPe9, kDuty0);
        arm(*board_a, kChPe9, continuous_config());
        std::printf("[case 4] RESULT: %s\n", pass ? "PASS" : "FAIL");
    }

    // ---- Case 5: edge capture directions + timestamp presence. Board A
    // toggles digital high/low; the recorder is edge-only (no periodic
    // sampling), so every arriving sample is one edge.
    {
        std::printf("[case 5] edge capture\n");
        bool pass = true;
        for (const auto& [rising, falling, timestamped, label] :
            {std::tuple{true, false, true, "rising+ts"},
             std::tuple{false, true, true, "falling+ts"},
             std::tuple{true, true, false, "both+no-ts"}}) {
            const auto index = static_cast<std::size_t>(kChPe9);
            arm(*board_b, kChPe9, edge_only_config(rising, falling, timestamped));
            sleep_us(200000);
            const auto base = counters_b.snapshot(index);
            // high, low, high, low, high, low: three of each edge.
            for (int toggle = 0; toggle < 6; ++toggle) {
                digital_write(*board_a, kChPe9, toggle % 2 == 0);
                sleep_us(100000);
            }
            sleep_us(200000);
            const auto window = counters_b.snapshot(index) - base;
            const std::size_t expected = rising && falling ? 6U : 3U;
            const bool count_ok = window.total >= expected && window.total <= expected + 2;
            const bool ts_ok = timestamped ? window.timestamped == window.total
                                           : window.timestamped == 0;
            // Stream-transition counts are only meaningful when samples
            // alternate (both edges sampled); rising-only arrives as all-high
            // samples, falling-only as all-low.
            const bool direction_ok =
                rising && falling
                    ? window.rising == 3U && window.high == 3U
                    : (rising ? window.high == window.total : window.high == 0);
            pass = pass && count_ok && ts_ok && direction_ok;
            std::printf("  %-12s: %zu edges (expect ~%zu), rising %zu, timestamped %zu -> %s\n",
                label, window.total, expected, window.rising, window.timestamped,
                count_ok && ts_ok && direction_ok ? "ok" : "FAIL");
        }
        arm(*board_b, kChPe9, continuous_config());
        arm(*board_a, kChPe9, continuous_config()); // stop driving: hand the wire back
        std::printf("[case 5] RESULT: %s\n", pass ? "PASS" : "FAIL");
    }

    // ---- Case 6: pulls. Nobody drives; one pull defines the shared line and
    // both boards read the same level.
    {
        std::printf("[case 6] pulls\n");
        bool pass = true;
        arm(*board_a, kChPe9, pull_config(libhcs::data::GpioPull::kNone)); // driver -> input
        sleep_us(50000);
        // Each board reads with the pull under test on ITS OWN pin, while the
        // other side sits at no-pull so the two 40 kOhm internal resistors
        // never fight on the same wire (up vs down would park the line at
        // mid-rail, an undefined input level).
        const auto none = pull_config(libhcs::data::GpioPull::kNone);
        arm(*board_a, kChPe9, none);
        sleep_us(50000);
        const bool up_high_b = asap_level(*board_b, counters_b, kChPe9,
            pull_config(libhcs::data::GpioPull::kUp));
        const bool down_low_b = !asap_level(*board_b, counters_b, kChPe9,
            pull_config(libhcs::data::GpioPull::kDown));
        arm(*board_b, kChPe9, none);
        sleep_us(50000);
        const bool up_high_a = asap_level(*board_a, counters_a, kChPe9,
            pull_config(libhcs::data::GpioPull::kUp));
        const bool down_low_a = !asap_level(*board_a, counters_a, kChPe9,
            pull_config(libhcs::data::GpioPull::kDown));
        pass = up_high_b && down_low_b && up_high_a && down_low_a;
        std::printf("  own-pin pull: B up %d down %d; A up %d down %d (expect 1 0 1 0)\n",
            up_high_b, down_low_b, up_high_a, down_low_a);
        for (const int channel : {kChPe9, kChPe13}) {
            arm(*board_a, channel, continuous_config());
            arm(*board_b, channel, continuous_config());
        }
        std::printf("[case 6] RESULT: %s\n", pass ? "PASS" : "FAIL");
    }

    // ---- Case 7: input -> output -> input churn (EXTI disarm regression).
    {
        std::printf("[case 7] mode churn\n");
        arm(*board_b, kChPe9, continuous_config());
        const auto index = static_cast<std::size_t>(kChPe9);
        const auto base = counters_b.snapshot(index);
        drive(*board_a, kChPe9, kDuty50); // input -> output (the modified path)
        sleep_us(1000000);
        drive(*board_a, kChPe9, kDuty0);
        arm(*board_a, kChPe9, continuous_config()); // output -> input
        sleep_us(500000);
        const auto window = counters_b.snapshot(index) - base;
        const bool pass = window.rising >= 40 && window.rising <= 60;
        std::printf("  churn cycle: %zu rising seen (expect ~50) -> %s\n", window.rising,
            pass ? "PASS" : "FAIL");
        std::printf("[case 7] RESULT: %s\n", pass ? "PASS" : "FAIL");
    }

    // ---- Case 8: all four channels driven at once; the wired TIM1 pair must
    // stay exact while the unwired TIM2 pair runs.
    {
        std::printf("[case 8] cross-timer independence\n");
        arm(*board_b, kChPe9, continuous_config());
        arm(*board_b, kChPe13, continuous_config());
        const auto base9 = counters_b.snapshot(static_cast<std::size_t>(kChPe9));
        const auto base13 = counters_b.snapshot(static_cast<std::size_t>(kChPe13));
        drive(*board_a, kChPa0, kDuty50);
        drive(*board_a, kChPa2, kDuty25);
        drive(*board_a, kChPe9, kDuty50);
        drive(*board_a, kChPe13, kDuty50);
        sleep_us(2000000);
        drive(*board_a, kChPa0, kDuty0);
        drive(*board_a, kChPa2, kDuty0);
        drive(*board_a, kChPe9, kDuty0);
        drive(*board_a, kChPe13, kDuty0);
        const double hz9 = static_cast<double>(
            (counters_b.snapshot(static_cast<std::size_t>(kChPe9)) - base9).rising) / 2.0;
        const double hz13 = static_cast<double>(
            (counters_b.snapshot(static_cast<std::size_t>(kChPe13)) - base13).rising) / 2.0;
        const bool pass = hz9 >= 47.0 && hz9 <= 53.0 && hz13 >= 47.0 && hz13 <= 53.0;
        std::printf("  TIM2 (ch0/ch1) driven blind; TIM1 pair held %.1f Hz / %.1f Hz -> %s\n",
            hz9, hz13, pass ? "PASS" : "FAIL");
        std::printf("[case 8] RESULT: %s\n", pass ? "PASS" : "FAIL");
    }

    // Shutdown: both boards end with every channel as quiet pull-down input.
    for (const int channel : {kChPa0, kChPa2, kChPe9, kChPe13}) {
        arm(*board_a, channel, continuous_config());
        arm(*board_b, channel, continuous_config());
    }
    sleep_us(200000);
    std::printf("[matrix] done\n");
    return 0;
}
