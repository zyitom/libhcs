// Focused confirmation of the P2 lease-expiry numbers with a healthy recorder.
// In the full mc02_gpio_session_test run the root-commit board's uplink was
// already degrading while it recorded (10-11 rising edges per 2 s window
// instead of ~100), which leaves the "quiet tail" verdict technically
// ambiguous: no samples could also mean a dead stream. This run does the
// whole measurement inside the fresh-link window and prints per-second sample
// counts, so the tail is provably sampled, not merely silent.
//
// Wiring: same bench (PE9<->PE9, PE13<->PE13). Usage:
//   mc02_gpio_lease_confirm [serial_new_tree] [serial_old_tree]

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
#include <libhcs/protocol/handler.hpp>

namespace {

using namespace std::chrono_literals;

constexpr int kChPe9 = 2;
constexpr int kChPe13 = 3;
constexpr uint16_t kDuty50 = 32768;
constexpr uint16_t kDuty25 = 16384;
const char* kDefaultSerialNew = "D4-46A1-3924-BAEB-DCE9-FBBA-C628";
const char* kDefaultSerialOld = "D4-18F5-D4CF-09D2-4903-509F-0696";

class LevelCounters {
public:
    void push(std::size_t channel, bool high) {
        const std::lock_guard<std::mutex> lock{mutex_};
        auto& counter = counters_[channel];
        if (counter.have_previous && high && !counter.previous_high) ++counter.rising;
        counter.previous_high = high;
        counter.have_previous = true;
        ++counter.total;
        if (high) ++counter.high;
    }

    std::size_t total(std::size_t channel) {
        const std::lock_guard<std::mutex> lock{mutex_};
        return counters_[channel].total;
    }

    std::size_t rising(std::size_t channel) {
        const std::lock_guard<std::mutex> lock{mutex_};
        return counters_[channel].rising;
    }

    std::size_t high(std::size_t channel) {
        const std::lock_guard<std::mutex> lock{mutex_};
        return counters_[channel].high;
    }

private:
    struct Counter {
        std::size_t total = 0;
        std::size_t rising = 0;
        std::size_t high = 0;
        bool previous_high = false;
        bool have_previous = false;
    };
    std::mutex mutex_;
    Counter counters_[4];
};

class NewBoardCallback final : public libhcs::board::Mc02::Callback {
public:
    void gpio_digital_read_result_callback(
        const libhcs::spec::mc02::GpioDescriptor& gpio,
        const libhcs::data::GpioDigitalDataView& data) override {
        counters_.push(gpio.channel_index, data.high);
    }

    LevelCounters& counters() { return counters_; }

private:
    LevelCounters counters_;
};

class LegacyOldBoard final : public libhcs::data::DataCallback {
public:
    bool open(const std::string& serial) {
        handler_.reset();
        try {
            handler_ = std::make_unique<libhcs::host::protocol::Handler>(0xA511, 0x0723, serial,
                libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true), *this);
        } catch (const std::exception& error) {
            std::printf("  legacy open failed: %s\n", error.what());
            handler_.reset();
            return false;
        }
        return true;
    }

    void arm_input(int channel) {
        auto builder = handler_->start_transmit();
        builder.write_gpio_digital_read_config(static_cast<uint8_t>(channel),
            {.period_ms = 1,
             .asap = true,
             .rising_edge = true,
             .falling_edge = true,
             .capture_timestamp = false,
             .pull = libhcs::data::GpioPull::kDown});
    }

    LevelCounters& counters() { return counters_; }

private:
    bool can_receive_callback(libhcs::data::DataId, const libhcs::data::CanDataView&) override {
        return true;
    }
    bool uart_receive_callback(libhcs::data::DataId, const libhcs::data::UartDataView&) override {
        return true;
    }
    bool gpio_digital_read_result_callback(
        uint8_t channel_index, const libhcs::data::GpioDigitalDataView& data) override {
        counters_.push(channel_index, data.high);
        return true;
    }
    bool gpio_analog_read_result_callback(
        uint8_t, const libhcs::data::GpioAnalogDataView&) override {
        return true;
    }
    void accelerometer_receive_callback(const libhcs::data::ImuAccelerometerDataView&) override {}
    void gyroscope_receive_callback(const libhcs::data::ImuGyroscopeDataView&) override {}
    void temperature_receive_callback(const libhcs::data::ImuTemperatureDataView&) override {}

    std::unique_ptr<libhcs::host::protocol::Handler> handler_;
    LevelCounters counters_;
};

void drive(libhcs::board::Mc02& board, int channel, uint16_t duty) {
    board.start_transmit().gpio_analog_write(
        libhcs::spec::mc02::kGpioDescriptors[channel], {.value = duty});
}

void arm_input(libhcs::board::Mc02& board, int channel) {
    board.start_transmit().gpio_digital_read(libhcs::spec::mc02::kGpioDescriptors[channel],
        {.period_ms = 1,
         .asap = true,
         .rising_edge = true,
         .falling_edge = true,
         .capture_timestamp = false,
         .pull = libhcs::data::GpioPull::kDown});
}

void sleep_us(int64_t us) { std::this_thread::sleep_for(std::chrono::microseconds{us}); }

} // namespace

int main(int argc, char** argv) {
    const std::string serial_new = argc > 1 ? argv[1] : kDefaultSerialNew;
    const std::string serial_old = argc > 2 ? argv[2] : kDefaultSerialOld;

    NewBoardCallback new_callback;
    auto board_new = std::make_unique<libhcs::board::Mc02>(new_callback, serial_new,
        libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true));

    LegacyOldBoard board_old;
    if (!board_old.open(serial_old)) {
        std::printf("[confirm] FAIL: old board did not connect\n");
        return 1;
    }
    auto& old_counters = board_old.counters();

    // Recorder at full rate on both wires; the whole run stays inside the
    // fresh-link window.
    arm_input(*board_new, kChPe9);
    arm_input(*board_new, kChPe13);
    board_old.arm_input(kChPe9);
    board_old.arm_input(kChPe13);
    sleep_us(300000);

    drive(*board_new, kChPe9, kDuty25);
    drive(*board_new, kChPe13, kDuty50);
    const int64_t drive_start = 0; // sample-count deltas below carry the timing
    (void)drive_start;
    sleep_us(2000000);
    for (const auto& [channel, duty] : {std::pair{kChPe9, 25.0}, std::pair{kChPe13, 50.0}}) {
        const auto index = static_cast<std::size_t>(channel);
        const std::size_t total = old_counters.total(index);
        const std::size_t rising = old_counters.rising(index);
        const std::size_t high = old_counters.high(index);
        std::printf("[confirm] while driven ch%d: %zu samples (expect ~2300), %zu rising "
                    "(expect ~100), duty %.1f%% (target %.0f%%)\n",
            channel, total, rising, total ? 100.0 * high / total : 0.0, duty);
    }

    // Stop refreshing the session: destroy the board object.
    board_new.reset();
    std::printf("[confirm] session destroyed; watching per-second recorder health + level\n");
    for (int second = 0; second < 7; ++second) {
        const std::size_t base_total_9 = old_counters.total(static_cast<std::size_t>(kChPe9));
        const std::size_t base_rise_9 = old_counters.rising(static_cast<std::size_t>(kChPe9));
        const std::size_t base_high_9 = old_counters.high(static_cast<std::size_t>(kChPe9));
        const std::size_t base_total_13 = old_counters.total(static_cast<std::size_t>(kChPe13));
        const std::size_t base_rise_13 = old_counters.rising(static_cast<std::size_t>(kChPe13));
        const std::size_t base_high_13 = old_counters.high(static_cast<std::size_t>(kChPe13));
        sleep_us(1000000);
        std::printf(
            "  +%-2ds  ch2: %4zu samples %3zu rising %3zu high   ch3: %4zu samples %3zu rising "
            "%3zu high\n",
            second + 1, old_counters.total(static_cast<std::size_t>(kChPe9)) - base_total_9,
            old_counters.rising(static_cast<std::size_t>(kChPe9)) - base_rise_9,
            old_counters.high(static_cast<std::size_t>(kChPe9)) - base_high_9,
            old_counters.total(static_cast<std::size_t>(kChPe13)) - base_total_13,
            old_counters.rising(static_cast<std::size_t>(kChPe13)) - base_rise_13,
            old_counters.high(static_cast<std::size_t>(kChPe13)) - base_high_13);
    }
    std::printf("[confirm] done (lease 4 s: edges must stop in the +4s/+5s row, and every row "
                "after must show samples > 0 with rising = high = 0)\n");
    return 0;
}
