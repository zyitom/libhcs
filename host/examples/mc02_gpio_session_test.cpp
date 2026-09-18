// Hardware test for the GPIO session-end changes: Gpio::stop_outputs() (zeroes
// every PWM compare register when a session ends) and the EXTI disarm in
// configure_hal_gpio_output() (HAL_GPIO_DeInit before switching a channel from
// edge-triggered input to output).
//
// Wiring: two mc02 boards, PE9<->PE9 (channel 2) and PE13<->PE13 (channel 3),
// GND<->GND, both USB-connected to this host. Only one side ever drives a wire
// at a time; every handover re-arms the previous driver as an input first, so
// two push-pull outputs never face each other.
//
// The current-tree board goes through the normal board::Mc02 stack. The
// root-commit board predates the EP0 configuration channel that stack requires
// (vendor request 0x40, which that firmware stalls), so it is driven through a
// raw host::protocol::Handler with no out-of-band handshake instead.
//
// Phases (driver -> recorder):
//   P1  root-commit board drives ch2; its session object is destroyed; the
//       4 s lease expires on the board. Firmware WITHOUT stop_outputs keeps
//       toggling the line forever -- the exact bug the change fixes.
//   P2  current-tree board drives ch2 (25 %) + ch3 (50 %); session destroyed;
//       both lines must go constant low within lease + one 50 Hz period.
//   P3  the session is destroyed and re-established INSIDE the lease window
//       with a fresh nonce; the board's activate_session() must zero both
//       outputs immediately -- far too early for lease expiry to explain.
//   P4  ch2 on the current-tree board round-trips edge-input -> PWM output ->
//       edge-input (the modified configure_hal_gpio_output path); the recorder
//       must see clean PWM during the output leg and a silent line after the
//       return to input.
//
// Usage: mc02_gpio_session_test [serial_of_new_tree_board] [serial_of_old_board]

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <libhcs/board/common.hpp>
#include <libhcs/protocol/handler.hpp>

#include "common/multi_board.hpp"

namespace {

using namespace std::chrono_literals;

constexpr int kChPe9 = 2;  // PE9, TIM1_CH1
constexpr int kChPe13 = 3; // PE13, TIM1_CH3
// mc02 runs both timers at 50 Hz (20 ms period); duty16 of 65535 is full on.
constexpr uint16_t kDuty50 = 32768;
constexpr uint16_t kDuty25 = 16384;
// Firmware lease: four 1 s keepalive rounds (firmware/mc02 vendor.hpp, 4 s).
constexpr int64_t kLeaseUs = 4000000;

// Default roles from the USB product strings on this bench: the g4b6c1c4 tree
// (HEAD + the GPIO changes) and the root-commit tree (no changes).
const char* kDefaultSerialNew = "D4-46A1-3924-BAEB-DCE9-FBBA-C628";
const char* kDefaultSerialOld = "D4-18F5-D4CF-09D2-4903-509F-0696";

const std::chrono::steady_clock::time_point kProgramStart = std::chrono::steady_clock::now();

int64_t host_us_now() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now() - kProgramStart)
        .count();
}

// Records every GPIO digital sample a board pushes, tagged with host receive
// time. Phase-level timing only needs host-side stamps (transport latency is
// ~1 ms, far below the 20 ms PWM period and the 4 s lease).
class GpioRecorder final : public examples::BoardReceiver {
public:
    struct Sample {
        int64_t host_us;
        uint32_t board_quarter_us;
        bool high;
    };

    void on_gpio_digital(int channel, const libhcs::data::GpioDigitalDataView& data) override {
        if (channel < 0 || static_cast<std::size_t>(channel) >= std::size(samples_))
            return;
        const std::lock_guard<std::mutex> lock{mutex_};
        samples_[static_cast<std::size_t>(channel)].push_back(
            {.host_us = host_us_now(),
             .board_quarter_us = data.timestamp_quarter_us.value_or(0U),
             .high = data.high});
    }

    std::vector<Sample> snapshot(std::size_t channel) {
        const std::lock_guard<std::mutex> lock{mutex_};
        return samples_[channel];
    }

private:
    std::mutex mutex_;
    std::vector<Sample> samples_[4];
};

// Root-commit firmware client: a raw protocol Handler with no EP0 handshake.
// Implements the DataCallback surface, forwards GPIO reads to the recorder and
// swallows everything else.
class LegacyOldBoard final : public libhcs::data::DataCallback {
public:
    explicit LegacyOldBoard(GpioRecorder& recorder)
        : recorder_(recorder) {}

    bool open(const std::string& serial) {
        try {
            handler_ = std::make_unique<libhcs::host::protocol::Handler>(
                0xA511, 0x0723, serial,
                libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true), *this);
        } catch (const std::exception& error) {
            std::printf("  legacy open of %s failed: %s\n", serial.c_str(), error.what());
            handler_.reset();
            return false;
        }
        return true;
    }

    void close() { handler_.reset(); }

    void drive(int channel, uint16_t duty) {
        auto builder = handler_->start_transmit();
        builder.write_gpio_analog_data(static_cast<uint8_t>(channel), {.value = duty});
    }

    void arm_input(int channel, uint16_t period_ms, bool edges) {
        auto builder = handler_->start_transmit();
        builder.write_gpio_digital_read_config(
            static_cast<uint8_t>(channel), {.period_ms = period_ms,
                                            .asap = true,
                                            .rising_edge = edges,
                                            .falling_edge = edges,
                                            .capture_timestamp = true,
                                            .pull = libhcs::data::GpioPull::kDown});
    }

    std::string_view name() const { return "LegacyMc02"; }

private:
    bool can_receive_callback(libhcs::data::DataId, const libhcs::data::CanDataView&) override {
        return true;
    }
    bool uart_receive_callback(libhcs::data::DataId, const libhcs::data::UartDataView&) override {
        return true;
    }
    bool gpio_digital_read_result_callback(
        uint8_t channel_index, const libhcs::data::GpioDigitalDataView& data) override {
        recorder_.on_gpio_digital(static_cast<int>(channel_index), data);
        return true;
    }
    bool gpio_analog_read_result_callback(
        uint8_t, const libhcs::data::GpioAnalogDataView&) override {
        return true;
    }
    void accelerometer_receive_callback(const libhcs::data::ImuAccelerometerDataView&) override {}
    void gyroscope_receive_callback(const libhcs::data::ImuGyroscopeDataView&) override {}
    void temperature_receive_callback(const libhcs::data::ImuTemperatureDataView&) override {}

    GpioRecorder& recorder_;
    std::unique_ptr<libhcs::host::protocol::Handler> handler_;
};

struct WindowStats {
    std::size_t total = 0;
    std::size_t high_count = 0;
    std::size_t rising = 0;
    int64_t last_high_host_us = -1;
};

WindowStats stats_in(const std::vector<GpioRecorder::Sample>& samples, int64_t t0, int64_t t1) {
    WindowStats stats;
    bool have_previous = false;
    bool previous_high = false;
    for (const auto& sample : samples) {
        if (sample.host_us < t0 || sample.host_us > t1)
            continue;
        ++stats.total;
        if (sample.high) {
            ++stats.high_count;
            stats.last_high_host_us = sample.host_us;
        }
        if (have_previous && sample.high && !previous_high)
            ++stats.rising;
        previous_high = sample.high;
        have_previous = true;
    }
    return stats;
}

void arm_input(examples::BoardSession& board, int channel, uint16_t period_ms, bool edges) {
    board.transmit([&](examples::BoardTransmitter& tx) {
        tx.gpio_digital_read(
            channel, {.period_ms = period_ms,
                      .asap = true,
                      .rising_edge = edges,
                      .falling_edge = edges,
                      .capture_timestamp = true,
                      .pull = libhcs::data::GpioPull::kDown});
    });
}

void drive(examples::BoardSession& board, int channel, uint16_t duty) {
    board.transmit(
        [&](examples::BoardTransmitter& tx) { tx.gpio_analog(channel, {.value = duty}); });
}

void sleep_us(int64_t us) { std::this_thread::sleep_for(std::chrono::microseconds{us}); }

std::unique_ptr<examples::BoardSession>
    open_board(GpioRecorder& recorder, const std::string& serial) {
    return examples::connect_any(recorder, serial);
}

// Re-arms a channel as a quiet pull-down input and waits for the asap sample,
// so a former driver stops pushing the wire before anyone else touches it.
void make_input(examples::BoardSession& board, int channel, GpioRecorder& recorder) {
    arm_input(board, channel, 1, true);
    sleep_us(200000);
    const auto samples = recorder.snapshot(static_cast<std::size_t>(channel));
    std::printf(
        "  ch%d re-armed input (%zu samples so far, last level %d)\n", channel, samples.size(),
        samples.empty() ? -1 : static_cast<int>(samples.back().high));
}

void legacy_make_input(LegacyOldBoard& board, int channel, GpioRecorder& recorder) {
    board.arm_input(channel, 1, true);
    sleep_us(200000);
    const auto samples = recorder.snapshot(static_cast<std::size_t>(channel));
    std::printf(
        "  ch%d re-armed input on legacy board (%zu samples so far)\n", channel, samples.size());
}

} // namespace

int main(int argc, char** argv) {
    const std::string serial_new = argc > 1 ? argv[1] : kDefaultSerialNew;
    const std::string serial_old = argc > 2 ? argv[2] : kDefaultSerialOld;

    GpioRecorder recorder_new;
    GpioRecorder recorder_old;

    // ---- P0: bring both boards up and arm every channel as pull-down input.
    std::printf("[P0] opening current-tree board %s\n", serial_new.c_str());
    auto board_new = open_board(recorder_new, serial_new);
    if (board_new == nullptr) {
        std::printf("FAIL: current-tree board (%s) did not connect\n", serial_new.c_str());
        return 1;
    }
    std::printf("[P0] opening root-commit board %s\n", serial_old.c_str());
    LegacyOldBoard board_old{recorder_old};
    if (!board_old.open(serial_old)) {
        std::printf("FAIL: root-commit board (%s) did not connect\n", serial_old.c_str());
        return 1;
    }
    std::printf(
        "[P0] both sessions up (%s / %s)\n", std::string(board_new->name()).c_str(),
        std::string(board_old.name()).c_str());

    for (const int channel : {kChPe9, kChPe13}) {
        arm_input(*board_new, channel, 5, true);
        board_old.arm_input(channel, 1, true);
    }
    sleep_us(500000);
    std::printf("[P0] all four channels armed as pull-down inputs\n");

    // ---- P1: root-commit firmware drives ch2; destroy the session; expect the
    // line to KEEP toggling after the lease expires (no stop_outputs).
    std::printf("[P1] root-commit board drives ch2 at 50%% duty\n");
    board_old.drive(kChPe9, kDuty50);
    sleep_us(3000000);
    {
        const int64_t now = host_us_now();
        const auto during =
            stats_in(recorder_new.snapshot(static_cast<std::size_t>(kChPe9)), now - 2000000, now);
        std::printf(
            "[P1] while driven: %zu samples, %zu rising edges, duty %.1f%%\n", during.total,
            during.rising,
            during.total
                ? 100.0 * static_cast<double>(during.high_count) / static_cast<double>(during.total)
                : 0.0);
    }
    const int64_t destroy_old = host_us_now();
    board_old.close();
    std::printf(
        "[P1] root-commit session destroyed at t=%lld us; waiting 7 s for lease\n",
        static_cast<long long>(destroy_old));
    sleep_us(7000000);
    {
        const auto samples = recorder_new.snapshot(static_cast<std::size_t>(kChPe9));
        const auto late = stats_in(samples, destroy_old + 5000000, destroy_old + 7000000);
        const auto tail = stats_in(samples, destroy_old, destroy_old + 7000000);
        if (late.rising > 5) {
            std::printf(
                "[P1] RESULT: line STILL toggles 5-7 s after session loss (%zu rising edges in "
                "window) -- firmware without stop_outputs holds the last duty cycle forever. "
                "This is the bug the change fixes, reproduced live.\n",
                late.rising);
        } else {
            const int64_t drop =
                tail.last_high_host_us < 0 ? -1 : tail.last_high_host_us - destroy_old;
            std::printf(
                "[P1] RESULT: line went constant low, last high %lld us after destroy -- "
                "this board's firmware already contains stop_outputs.\n",
                static_cast<long long>(drop));
        }
    }
    // Bring the old board back and stop its output explicitly (its firmware
    // will not have done so on its own if P1 showed the held duty cycle).
    if (!board_old.open(serial_old)) {
        std::printf("FAIL: root-commit board did not reconnect\n");
        return 1;
    }
    board_old.drive(kChPe9, 0);
    legacy_make_input(board_old, kChPe9, recorder_old);
    std::printf("[P1] root-commit ch2 stopped and re-armed as input\n");

    // ---- P2: current-tree board drives both wires; destroy the session; both
    // lines must go constant low within lease (4 s) + one PWM period (20 ms).
    std::printf("[P2] current-tree board drives ch2 at 25%% and ch3 at 50%% duty\n");
    drive(*board_new, kChPe9, kDuty25);
    drive(*board_new, kChPe13, kDuty50);
    sleep_us(3000000);
    for (const auto& [channel, duty] : {
             std::pair{ kChPe9, 25.0},
             std::pair{kChPe13, 50.0}
    }) {
        const int64_t now = host_us_now();
        const auto during =
            stats_in(recorder_old.snapshot(static_cast<std::size_t>(channel)), now - 2000000, now);
        const double measured = during.total ? 100.0 * static_cast<double>(during.high_count)
                                                   / static_cast<double>(during.total)
                                             : 0.0;
        const double frequency = during.rising / 2.0; // rising edges per second, 2 s window
        std::printf(
            "[P2] ch%d: %zu rising edges, duty %.1f%% (target %.0f%%), %.0f Hz\n", channel,
            during.rising, measured, duty, frequency);
    }
    const int64_t destroy_new = host_us_now();
    board_new.reset();
    std::printf(
        "[P2] current-tree session destroyed at t=%lld us; waiting 7 s\n",
        static_cast<long long>(destroy_new));
    sleep_us(7000000);
    {
        bool pass = true;
        for (const int channel : {kChPe9, kChPe13}) {
            const auto samples = recorder_old.snapshot(static_cast<std::size_t>(channel));
            const auto tail = stats_in(samples, destroy_new, destroy_new + 7000000);
            const auto quiet = stats_in(samples, destroy_new + 5000000, destroy_new + 7000000);
            const int64_t drop =
                tail.last_high_host_us < 0 ? -1 : tail.last_high_host_us - destroy_new;
            const bool stopped = quiet.rising == 0 && quiet.high_count == 0;
            const bool in_time = drop >= 0 && drop <= kLeaseUs + 250000;
            pass = pass && stopped && in_time;
            std::printf(
                "[P2] ch%d: last high %lld us after destroy (lease 4 s + <=20 ms), quiet tail "
                "%s -> %s\n",
                channel, static_cast<long long>(drop), stopped ? "clean" : "STILL ACTIVE",
                stopped && in_time ? "PASS" : "FAIL");
        }
        std::printf("[P2] RESULT: %s (stop_outputs on lease expiry)\n", pass ? "PASS" : "FAIL");
    }

    // ---- P3: rebuild the session INSIDE the lease window with a fresh nonce.
    // The board's activate_session() must zero both outputs at that moment --
    // several seconds before lease expiry could explain a drop.
    std::printf("[P3] reopening current-tree board within the lease window\n");
    const int64_t reopen_start = host_us_now();
    board_new = open_board(recorder_new, serial_new);
    if (board_new == nullptr) {
        std::printf("FAIL: current-tree board did not reconnect for P3\n");
        return 1;
    }
    const int64_t reopen_done = host_us_now();
    std::printf(
        "[P3] new session established; reopen took %lld us (lease window intact)\n",
        static_cast<long long>(reopen_done - reopen_start));
    drive(*board_new, kChPe9, kDuty50);
    drive(*board_new, kChPe13, kDuty50);
    sleep_us(2000000);
    for (const int channel : {kChPe9, kChPe13}) {
        const int64_t now = host_us_now();
        const auto during =
            stats_in(recorder_old.snapshot(static_cast<std::size_t>(channel)), now - 1500000, now);
        std::printf("[P3] ch%d driving again: %zu rising edges in 1.5 s\n", channel, during.rising);
    }
    const int64_t destroy_replace = host_us_now();
    board_new.reset();
    board_new = open_board(recorder_new, serial_new);
    if (board_new == nullptr) {
        std::printf("FAIL: current-tree board did not reconnect for the replacement session\n");
        return 1;
    }
    const int64_t replace_done = host_us_now();
    std::printf(
        "[P3] replacement session (fresh nonce) established %lld us after the old object died; "
        "board must zero outputs now, not at lease expiry\n",
        static_cast<long long>(replace_done - destroy_replace));
    sleep_us(2000000);
    {
        bool pass = true;
        for (const int channel : {kChPe9, kChPe13}) {
            const auto samples = recorder_old.snapshot(static_cast<std::size_t>(channel));
            const auto tail = stats_in(samples, destroy_replace, replace_done + 2000000);
            const auto quiet = stats_in(samples, replace_done + 500000, replace_done + 2000000);
            const int64_t drop =
                tail.last_high_host_us < 0 ? -1 : tail.last_high_host_us - destroy_replace;
            const bool stopped = quiet.rising == 0 && quiet.high_count == 0;
            // Lease expiry cannot explain anything before destroy + 3 s (the
            // last keepalive left <= 1 s of lease on the board).
            const bool too_early_for_lease = drop >= 0 && drop < 3000000;
            pass = pass && stopped && too_early_for_lease;
            std::printf(
                "[P3] ch%d: last high %lld us after destroy, quiet tail %s -> %s\n", channel,
                static_cast<long long>(drop), stopped ? "clean" : "STILL ACTIVE",
                stopped && too_early_for_lease ? "PASS" : "FAIL");
        }
        std::printf(
            "[P3] RESULT: %s (stop_outputs on session replacement via activate_session)\n",
            pass ? "PASS" : "FAIL");
    }

    // ---- P4: ch2 round-trips edge-input -> PWM output -> edge-input on the
    // current-tree board (the modified configure_hal_gpio_output). ch3 keeps
    // driving so the two channels prove their independence.
    std::printf("[P4] ch3 stays driven; ch2 round-trips input -> output -> input\n");
    drive(*board_new, kChPe13, kDuty50);
    make_input(*board_new, kChPe9, recorder_new);
    sleep_us(1500000);
    {
        const int64_t now = host_us_now();
        const auto idle =
            stats_in(recorder_new.snapshot(static_cast<std::size_t>(kChPe9)), now - 1000000, now);
        std::printf("[P4] ch2 idle as input: %zu rising edges (expect 0)\n", idle.rising);
    }
    drive(*board_new, kChPe9, kDuty50); // input -> output, the modified path
    sleep_us(2000000);
    {
        const int64_t now = host_us_now();
        const auto during =
            stats_in(recorder_old.snapshot(static_cast<std::size_t>(kChPe9)), now - 1500000, now);
        std::printf(
            "[P4] ch2 as output after edge-input: %zu rising edges seen by recorder\n",
            during.rising);
    }
    arm_input(*board_new, kChPe9, 5, true); // output -> input again
    sleep_us(2000000);
    {
        const auto samples = recorder_new.snapshot(static_cast<std::size_t>(kChPe9));
        const auto idle = stats_in(samples, host_us_now() - 1500000, host_us_now());
        const bool clean = idle.rising == 0 && idle.high_count == 0;
        std::printf(
            "[P4] ch2 back as input: %zu rising edges, %zu high samples -> %s\n", idle.rising,
            idle.high_count, clean ? "PASS (no phantom edges)" : "FAIL");
    }
    // Prove the channel still works as an input after the round trip: the old
    // board drives, the current-tree board must see the edges again.
    board_old.drive(kChPe9, kDuty50);
    sleep_us(2000000);
    {
        const int64_t now = host_us_now();
        const auto during =
            stats_in(recorder_new.snapshot(static_cast<std::size_t>(kChPe9)), now - 1500000, now);
        std::printf(
            "[P4] ch2 input after round trip, old board drives: %zu rising edges -> %s\n",
            during.rising, during.rising > 50 ? "PASS" : "FAIL");
    }
    board_old.drive(kChPe9, 0);
    legacy_make_input(board_old, kChPe9, recorder_old);

    // ---- Shutdown: everything ends as output-low or pull-down input, both
    // sessions closed, no wire left driven.
    drive(*board_new, kChPe9, 0);
    drive(*board_new, kChPe13, 0);
    sleep_us(1000000);
    board_new.reset();
    board_old.close();
    std::printf("[done] boards closed; both channels low, no line driven\n");
    return 0;
}
