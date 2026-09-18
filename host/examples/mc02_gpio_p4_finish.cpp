// Closes the P4 phase of mc02_gpio_session_test. The first full run was
// spoiled by the root-commit board's legacy link decaying mid-run (its
// keepalive behaviour is incompatible with the current host stack; the link
// dies within seconds to minutes and takes the recorder stream with it). This
// program runs the whole round trip inside the fresh-link window and gates
// every leg on a liveness check, so a failure is attributable instead of
// ambiguous:
//
//   leg 1  new-tree board: edge-input -> PWM output (the modified
//          configure_hal_gpio_output path); old board must see ~100 edges
//          in 2 s at 50 Hz.
//   leg 2  new-tree board: PWM output -> edge-input; its own stream must be
//          silent (no phantom edges from stale EXTI state).
//   leg 3  old board drives; new-tree board's input must still count edges
//          after the round trip (~75 in 1.5 s).
//   leg 4  control on the untouched PE13 wire: new-tree board drives; old
//          board sees edges.
//
// Usage: mc02_gpio_p4_finish [serial_new_tree] [serial_old_tree]

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <libhcs/board/common.hpp>
#include <libhcs/board/mc02.hpp>
#include <libhcs/protocol/handler.hpp>

namespace {

using namespace std::chrono_literals;

constexpr int kChPe9 = 2;
constexpr int kChPe13 = 3;
constexpr uint16_t kDuty50 = 32768;
const char* kDefaultSerialNew = "D4-46A1-3924-BAEB-DCE9-FBBA-C628";
const char* kDefaultSerialOld = "D4-18F5-D4CF-09D2-4903-509F-0696";

class LevelCounters {
public:
    void push(std::size_t channel, bool high) {
        const std::lock_guard<std::mutex> lock{mutex_};
        auto& counter = counters_[channel];
        if (counter.have_previous && high && !counter.previous_high)
            ++counter.rising;
        counter.previous_high = high;
        counter.have_previous = true;
        ++counter.total;
    }

    std::size_t total(std::size_t channel) {
        const std::lock_guard<std::mutex> lock{mutex_};
        return counters_[channel].total;
    }

    std::size_t rising(std::size_t channel) {
        const std::lock_guard<std::mutex> lock{mutex_};
        return counters_[channel].rising;
    }

private:
    struct Counter {
        std::size_t total = 0;
        std::size_t rising = 0;
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

const char* link_name(libhcs::host::protocol::Handler::LinkState state);

class LegacyOldBoard final : public libhcs::data::DataCallback {
public:
    bool open(const std::string& serial) {
        handler_.reset();
        try {
            handler_ = std::make_unique<libhcs::host::protocol::Handler>(
                0xA511, 0x0723, serial,
                libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true), *this);
        } catch (const std::exception& error) {
            std::printf("  legacy open failed: %s\n", error.what());
            handler_.reset();
            return false;
        }
        return true;
    }

    void drive(int channel, uint16_t duty) {
        auto builder = handler_->start_transmit();
        builder.write_gpio_analog_data(static_cast<uint8_t>(channel), {.value = duty});
    }

    void arm_input(int channel) {
        auto builder = handler_->start_transmit();
        builder.write_gpio_digital_read_config(
            static_cast<uint8_t>(channel), {.period_ms = 1,
                                            .asap = true,
                                            .rising_edge = true,
                                            .falling_edge = true,
                                            .capture_timestamp = false,
                                            .pull = libhcs::data::GpioPull::kDown});
    }

    [[nodiscard]] const char* link_state() const { return link_name(handler_->link_state()); }

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

const char* link_name(libhcs::host::protocol::Handler::LinkState state) {
    switch (state) {
    case libhcs::host::protocol::Handler::LinkState::kUp: return "up";
    case libhcs::host::protocol::Handler::LinkState::kSessionDown: return "session-down";
    default: return "FAULTED";
    }
}

void arm_input(libhcs::board::Mc02& board, int channel) {
    board.start_transmit().gpio_digital_read(
        libhcs::spec::mc02::kGpioDescriptors[channel], {.period_ms = 1,
                                                        .asap = true,
                                                        .rising_edge = true,
                                                        .falling_edge = true,
                                                        .capture_timestamp = false,
                                                        .pull = libhcs::data::GpioPull::kDown});
}

void drive(libhcs::board::Mc02& board, int channel, uint16_t duty) {
    board.start_transmit().gpio_analog_write(
        libhcs::spec::mc02::kGpioDescriptors[channel], {.value = duty});
}

void sleep_us(int64_t us) { std::this_thread::sleep_for(std::chrono::microseconds{us}); }

} // namespace

int main(int argc, char** argv) {
    const std::string serial_new = argc > 1 ? argv[1] : kDefaultSerialNew;
    const std::string serial_old = argc > 2 ? argv[2] : kDefaultSerialOld;

    NewBoardCallback new_callback;
    libhcs::board::Mc02 board_new{
        new_callback, serial_new,
        libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true)};
    std::printf("[p4] new-tree board link: %s\n", link_name(board_new.link_state()));

    LegacyOldBoard board_old;
    if (!board_old.open(serial_old)) {
        std::printf("[p4] FAIL: old board did not connect\n");
        return 1;
    }
    std::printf("[p4] old board link (fresh): %s\n", board_old.link_state());

    auto& new_counters = new_callback.counters();
    auto& old_counters = board_old.counters();

    // Both boards sample both wires continuously from now on. The clock is
    // running: the legacy link is only good for a short while.
    arm_input(board_new, kChPe9);
    arm_input(board_new, kChPe13);
    board_old.arm_input(kChPe9);
    board_old.arm_input(kChPe13);
    sleep_us(400000);
    const bool old_alive = old_counters.total(static_cast<std::size_t>(kChPe9)) >= 100;
    std::printf(
        "[p4] recorder stream: old board %s (%zu ch2 samples in 0.4 s)\n",
        old_alive ? "alive" : "DEAD -- results inconclusive",
        old_counters.total(static_cast<std::size_t>(kChPe9)));

    // Leg 1: new-tree board switches edge-input -> PWM output on ch2 (the
    // modified configure_hal_gpio_output); old board must see the edges.
    const std::size_t old_base_1 = old_counters.rising(static_cast<std::size_t>(kChPe9));
    drive(board_new, kChPe9, kDuty50);
    sleep_us(2000000);
    const std::size_t leg1 = old_counters.rising(static_cast<std::size_t>(kChPe9)) - old_base_1;
    std::printf(
        "[p4] leg1 new-tree input->output drive: old board saw %zu rising in 2 s "
        "(expect ~100) -> %s\n",
        leg1, leg1 > 60 ? "PASS" : "FAIL");

    // Leg 2: back to edge-input; the new-tree board's own stream must be
    // silent -- no phantom edges from stale EXTI state.
    drive(board_new, kChPe9, 0);
    arm_input(board_new, kChPe9);
    sleep_us(1500000);
    {
        const std::size_t new_base = new_counters.total(static_cast<std::size_t>(kChPe9));
        const std::size_t new_rise_base = new_counters.rising(static_cast<std::size_t>(kChPe9));
        sleep_us(1000000);
        const std::size_t samples = new_counters.total(static_cast<std::size_t>(kChPe9)) - new_base;
        const std::size_t phantom =
            new_counters.rising(static_cast<std::size_t>(kChPe9)) - new_rise_base;
        std::printf(
            "[p4] leg2 new-tree output->input: %zu samples in 1 s, %zu rising -> %s\n", samples,
            phantom, samples > 500 && phantom == 0 ? "PASS" : "FAIL");
    }

    // Leg 3: the old board drives; the new-tree board's input must still count
    // edges after the round trip. The legacy link is perishable, so re-open it
    // until its uplink streams again before asking it to drive.
    for (int attempt = 0; attempt < 3; ++attempt) {
        const std::size_t before = old_counters.total(static_cast<std::size_t>(kChPe9));
        board_old.arm_input(kChPe9);
        board_old.arm_input(kChPe13);
        sleep_us(300000);
        if (old_counters.total(static_cast<std::size_t>(kChPe9)) - before >= 100)
            break;
        std::printf("[p4] leg3 old link dead (state %s), re-opening\n", board_old.link_state());
        if (!board_old.open(serial_old))
            break;
    }
    const std::size_t new_base_3 = new_counters.rising(static_cast<std::size_t>(kChPe9));
    board_old.drive(kChPe9, kDuty50);
    sleep_us(1500000);
    board_old.drive(kChPe9, 0);
    board_old.arm_input(kChPe9);
    sleep_us(200000);
    const std::size_t leg3 = new_counters.rising(static_cast<std::size_t>(kChPe9)) - new_base_3;
    std::printf(
        "[p4] leg3 old board drives ch2: new-tree input saw %zu rising in 1.5 s "
        "(expect ~75) -> %s\n",
        leg3, leg3 > 40 ? "PASS" : "FAIL");

    // Leg 4: control on the untouched PE13 wire -- new-tree board drives.
    const std::size_t old_base_4 = old_counters.rising(static_cast<std::size_t>(kChPe13));
    drive(board_new, kChPe13, kDuty50);
    sleep_us(1500000);
    drive(board_new, kChPe13, 0);
    arm_input(board_new, kChPe13);
    sleep_us(200000);
    const std::size_t leg4 = old_counters.rising(static_cast<std::size_t>(kChPe13)) - old_base_4;
    std::printf(
        "[p4] leg4 control ch3 new-tree drives: old board saw %zu rising in 1.5 s "
        "(expect ~75) -> %s\n",
        leg4, leg4 > 40 ? "PASS" : "FAIL");

    std::printf(
        "[p4] final link states: new %s, old %s\n", link_name(board_new.link_state()),
        board_old.link_state());
    std::printf("[p4] done\n");
    return 0;
}
