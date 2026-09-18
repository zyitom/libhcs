// Robustness characterization for the firmware's GPIO channel-index
// validation. Channel 4 fits the 6-bit protocol header (0..63) but is one past
// the mc02 descriptor table (0..3) -- wire-representable, board-invalid.
//
// The designed firmware response to such a frame is NOT a session teardown:
// the deserializer enters discard mode -- the rest of THIS transfer is dropped,
// error_callback() fires (still a TODO no-op on the firmware), and parsing
// resumes at the next transfer boundary. The session survives and the outputs
// keep the last valid duty cycle. This program pins that behavior down:
//
//   1  a valid 50 % PWM drives the wire (~75 rising / 1.5 s)
//   2  one transfer containing [poison, valid 25 % write]: discard mode must
//      drop BOTH -- the wire stays at 50 %, proving the firmware parsed and
//      rejected the poison instead of never seeing it
//   3  the next transfer's valid 25 % write must land (duty visibly changes)
//   4  closing the session must zero the output through the normal lease path
//
// Wiring: PE9<->PE9 (channel 2) watched by the second board.
// Usage: mc02_gpio_invalid_channel_test [serial_dut] [serial_recorder]

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/board/mc02.hpp>
#include <libhcs/protocol/handler.hpp>

namespace {

using namespace std::chrono_literals;

constexpr int kChPe9 = 2;
constexpr int kChPoison = 4; // first channel index past the mc02 descriptor table
constexpr uint16_t kDuty25 = 16384;
constexpr uint16_t kDuty50 = 32768;
const char* kDefaultSerialA = "D4-46A1-3924-BAEB-DCE9-FBBA-C628";
const char* kDefaultSerialB = "D4-18F5-D4CF-09D2-4903-509F-0696";

class RecorderCallback final : public libhcs::board::Mc02::Callback {
public:
    void gpio_digital_read_result_callback(
        const libhcs::spec::mc02::GpioDescriptor& gpio,
        const libhcs::data::GpioDigitalDataView& data) override {
        if (gpio.channel_index != kChPe9) return;
        const std::lock_guard<std::mutex> lock{mutex_};
        ++total_;
        if (data.high) ++high_;
        if (data.high && !previous_high_) ++rising_;
        previous_high_ = data.high;
    }

    struct Snapshot {
        std::size_t total = 0;
        std::size_t rising = 0;
        std::size_t high = 0;
    };

    Snapshot snapshot() const {
        const std::lock_guard<std::mutex> lock{mutex_};
        return {.total = total_, .rising = rising_, .high = high_};
    }

private:
    mutable std::mutex mutex_;
    std::size_t total_ = 0;
    std::size_t rising_ = 0;
    std::size_t high_ = 0;
    bool previous_high_ = false;
};


// Data-callback shell for the raw handler; GPIO traffic is not expected back.
class DutCallback final : public libhcs::data::DataCallback {
    bool can_receive_callback(libhcs::data::DataId, const libhcs::data::CanDataView&) override {
        return true;
    }
    bool uart_receive_callback(libhcs::data::DataId, const libhcs::data::UartDataView&) override {
        return true;
    }
    bool gpio_digital_read_result_callback(
        uint8_t, const libhcs::data::GpioDigitalDataView&) override {
        return true;
    }
    bool gpio_analog_read_result_callback(
        uint8_t, const libhcs::data::GpioAnalogDataView&) override {
        return true;
    }
    void accelerometer_receive_callback(const libhcs::data::ImuAccelerometerDataView&) override {}
    void gyroscope_receive_callback(const libhcs::data::ImuGyroscopeDataView&) override {}
    void temperature_receive_callback(const libhcs::data::ImuTemperatureDataView&) override {}
};

void sleep_us(int64_t us) { std::this_thread::sleep_for(std::chrono::microseconds{us}); }

double duty_pct(const RecorderCallback::Snapshot& window) {
    return window.total > 0
        ? 100.0 * static_cast<double>(window.high) / static_cast<double>(window.total)
        : 0.0;
}

RecorderCallback::Snapshot operator-(
    const RecorderCallback::Snapshot& after, const RecorderCallback::Snapshot& before) {
    return {.total = after.total - before.total,
            .rising = after.rising - before.rising,
            .high = after.high - before.high};
}

} // namespace

int main(int argc, char** argv) {
    const std::string serial_dut = argc > 1 ? argv[1] : kDefaultSerialA;
    const std::string serial_recorder = argc > 2 ? argv[2] : kDefaultSerialB;

    RecorderCallback recorder_callback;
    libhcs::board::Mc02 recorder{recorder_callback, serial_recorder,
        libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true)};
    recorder.start_transmit().gpio_digital_read(
        libhcs::spec::mc02::kGpioDescriptors[kChPe9],
        {.period_ms = 1,
         .asap = true,
         .rising_edge = true,
         .falling_edge = true,
         .capture_timestamp = false,
         .pull = libhcs::data::GpioPull::kDown});
    std::printf("[poison] recorder link up, ch2 armed\n");

    DutCallback dut_callback;
    auto dut = std::make_unique<libhcs::host::protocol::Handler>(0xA511, 0x0723, serial_dut,
        libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true), dut_callback,
        [](libhcs::host::protocol::Handler& handler) {
            std::ignore = libhcs::board::hcs::apply(handler, {});
        });
    std::printf("[poison] dut link up\n");

    // Step 1: a valid 50 % PWM drives the wire.
    {
        auto builder = dut->start_transmit();
        builder.write_gpio_analog_data(kChPe9, {.value = kDuty50});
    }
    const auto base1 = recorder_callback.snapshot();
    sleep_us(1500000);
    const auto window1 = recorder_callback.snapshot() - base1;
    std::printf("[poison] step 1 driven: %zu rising, duty %.1f%% (expect ~75 edges, 50%%)\n",
        window1.rising, duty_pct(window1));

    // Step 2: poison + valid write in ONE transfer -- discard mode must drop
    // both; the wire keeps the old 50 %.
    {
        auto builder = dut->start_transmit();
        builder.write_gpio_analog_data(kChPoison, {.value = kDuty50});
        builder.write_gpio_analog_data(kChPe9, {.value = kDuty25});
    }
    const auto base2 = recorder_callback.snapshot();
    sleep_us(1500000);
    const auto window2 = recorder_callback.snapshot() - base2;
    const bool discarded =
        window2.rising >= 60 && duty_pct(window2) > 45.0 && duty_pct(window2) < 55.0;
    std::printf("[poison] step 2 poisoned transfer: %zu rising, duty %.1f%% (still 50%% = poison "
                "rejected, valid write collateral-dropped) -> %s\n",
        window2.rising, duty_pct(window2), discarded ? "ok" : "FAIL");

    // Step 3: the next transfer's valid 25 % write must land.
    {
        auto builder = dut->start_transmit();
        builder.write_gpio_analog_data(kChPe9, {.value = kDuty25});
    }
    const auto base3 = recorder_callback.snapshot();
    sleep_us(1500000);
    const auto window3 = recorder_callback.snapshot() - base3;
    const bool recovered =
        window3.rising >= 60 && duty_pct(window3) > 22.0 && duty_pct(window3) < 28.0;
    std::printf("[poison] step 3 recovery transfer: %zu rising, duty %.1f%% (expect 25%%) -> %s\n",
        window3.rising, duty_pct(window3), recovered ? "ok" : "FAIL");

    // Step 4: closing the session must zero the output through the normal
    // lease path (4 s lease from the last keepalive).
    dut.reset();
    sleep_us(4500000); // lease (4 s minus the last keepalive) keeps the PWM up
    const auto base4 = recorder_callback.snapshot();
    sleep_us(2000000);
    const auto window4 = recorder_callback.snapshot() - base4;
    const bool stopped = window4.total > 500 && window4.rising == 0 && window4.high == 0;
    std::printf("[poison] step 4 session closed: %zu samples, %zu rising after the lease window "
                "(must be silent) -> %s\n",
        window4.total, window4.rising, stopped ? "ok" : "FAIL");

    const bool pass = discarded && recovered && stopped;
    std::printf("[poison] RESULT: %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
