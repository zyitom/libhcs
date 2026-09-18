// Focused discriminator for the P4 anomaly of mc02_gpio_session_test: after
// several minutes of session churn, BOTH directions of the PE9 wire went
// silent while PE13 had worked moments before. Suspects: a loose jumper, a
// faulted board link, or a stuck firmware output. This probe drives each wire
// in both directions with BOTH boards continuously sampling (1 kHz) and an
// asap-liveness gate before every drive, so a silent wire can only mean the
// wire itself or a stuck output -- never a dead recorder stream.
//
// Wiring: same bench as mc02_gpio_session_test (PE9<->PE9, PE13<->PE13).
// Usage: mc02_gpio_wire_probe [serial_new_tree] [serial_old_tree]

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

// Cumulative per-channel sample and rising-edge counts.
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

// New-tree board receive path.
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

// Legacy (root-commit firmware) driver: raw Handler, no EP0 handshake. The
// root firmware's keepalive behaviour degrades this link after some tens of
// seconds, so it must be treated as perishable: re-open before use.
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

    // Continuous 1 kHz sampling was just armed; samples must keep arriving.
    bool liveness(int channel) {
        const std::size_t before = counters_.total(static_cast<std::size_t>(channel));
        std::this_thread::sleep_for(300ms);
        return counters_.total(static_cast<std::size_t>(channel)) - before >= 100;
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
    std::printf("[probe] new-tree board link: %s\n", link_name(board_new.link_state()));

    LegacyOldBoard board_old;
    if (!board_old.open(serial_old)) {
        std::printf("[probe] FAIL: old board did not connect\n");
        return 1;
    }
    std::printf("[probe] old board link (fresh): %s\n", board_old.link_state());

    auto& new_counters = new_callback.counters();
    auto& old_counters = board_old.counters();

    constexpr int kRounds = 2;
    for (int round = 0; round < kRounds; ++round) {
        for (const int channel : {kChPe9, kChPe13}) {
            const auto index = static_cast<std::size_t>(channel);
            // Both sides sample the wire continuously at 1 kHz.
            arm_input(board_new, channel);
            board_old.arm_input(channel);
            sleep_us(300000);
            const bool new_alive =
                new_counters.total(index) - new_counters.total(index) >= 0; // placeholder
            (void)new_alive;
            const std::size_t new_total_before = new_counters.total(index);
            const std::size_t old_total_before = old_counters.total(index);
            sleep_us(300000);
            const bool new_alive_real = new_counters.total(index) - new_total_before >= 100;
            const bool old_alive_real = old_counters.total(index) - old_total_before >= 100;
            std::printf(
                "--- round %d ch%d: new link %s (%s), old link %s (%s)\n", round, channel,
                link_name(board_new.link_state()), new_alive_real ? "alive" : "DEAD",
                board_old.link_state(), old_alive_real ? "alive" : "DEAD");

            // New-tree board drives; old board must see the edges.
            const std::size_t old_base = old_counters.rising(index);
            drive(board_new, channel, kDuty50);
            sleep_us(1200000);
            drive(board_new, channel, 0);
            arm_input(board_new, channel); // output -> input before the old board drives
            sleep_us(200000);
            const std::size_t new_to_old = old_counters.rising(index) - old_base;
            std::printf(
                "    new drives -> old sees %zu rising in 1.2 s (expect ~60)\n", new_to_old);

            // Old board drives; new board must see the edges.
            const std::size_t new_base = new_counters.rising(index);
            board_old.drive(channel, kDuty50);
            sleep_us(1200000);
            board_old.drive(channel, 0);
            board_old.arm_input(channel);
            sleep_us(200000);
            const std::size_t old_to_new = new_counters.rising(index) - new_base;
            std::printf(
                "    old drives -> new sees %zu rising in 1.2 s (expect ~60)\n", old_to_new);
        }
    }
    std::printf("[probe] done\n");
    return 0;
}
