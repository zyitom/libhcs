// Motor 3519 PID position-control sine-wave test, on any two-CAN-bus board
// (hpm5321_dual_can, c_board, mc02).
//
// Drives one motor (ID 2) on EACH CAN bus (bus 0 and bus 1) at the same time,
// using CAN-FD frames:
//   - Control frame:  CAN ID 0x200, motor-2 torque-current at bytes [2:3]
//   - Feedback frame: CAN ID 0x202 (0x200 + motor_id), 1 ms period
//   - Torque-current range: [-16384, 16384]  ->  [-20.5 A, 20.5 A]
//   - Angle range:          [0, 8191]        ->  [0 deg, 360 deg]
//
// Velocity is derived from consecutive position samples divided by the
// CAN hardware timestamp delta (1 us resolution), NOT from the motor's
// built-in speed estimate.  Stop with Ctrl-C.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <optional>
#include <thread>

#include "common/multi_board.hpp"

namespace {

// Bus configuration
constexpr unsigned kBusCount = 2;       // CAN bus 0 + bus 1
// No classic/FD switch any more: frames go out in the bus's compiled mode.

// Motor / CAN identifiers (3519 control protocol)
constexpr unsigned kMotorId = 2;
constexpr uint32_t kControlCanId = 0x200;
constexpr uint32_t kFeedbackCanId = 0x202;

// Pure proportional position control
constexpr float kKp = 8.0F;     // position error -> torque current
constexpr float kMaxCurrent = 12000.0F;  // ~15 A (75 % of full-scale)

// Sine trajectory
constexpr float kAmplitude = 3000.0F;  // +-132 deg
constexpr float kFrequency = 2.0F;     // Hz

// Control loop timing
constexpr float kLoopHz = 1000.0F;
constexpr auto kLoopPeriod = std::chrono::microseconds{1000};

// Shutdown
std::atomic<bool> g_running{true};
void on_sigint(int) { g_running.store(false); }

float wrap_delta(float delta) {
    while (delta > 4096.0F) delta -= 8192.0F;
    while (delta < -4096.0F) delta += 8192.0F;
    return delta;
}

}  // namespace

class MotorPidSine : public examples::BoardReceiver {
public:
    void bind(examples::BoardSession* board) { board_ = board; }

    struct Sample {
        uint16_t raw_angle = 0;
        bool valid = false;
        std::optional<uint32_t> timestamp_us;
    };

    Sample get_sample(unsigned bus) const {
        std::lock_guard lock(mutex_);
        return sample_[bus];
    }

    void send_control(unsigned bus, int16_t current) {
        std::array<std::byte, 8> frame{};
        frame[2] = static_cast<std::byte>((current >> 8) & 0xFF);
        frame[3] = static_cast<std::byte>(current & 0xFF);
        board_->transmit([&](examples::BoardTransmitter& tx) {
            tx.can(static_cast<int>(bus),
                {.can_id = kControlCanId, .can_data = frame});
        });
    }

private:
    void on_can(int bus, const libhcs::data::CanDataView& data) override {
        if (bus >= 0 && bus < static_cast<int>(kBusCount))
            store_sample(static_cast<unsigned>(bus), data);
    }

    void store_sample(unsigned bus, const libhcs::data::CanDataView& data) {
        if (data.can_id != kFeedbackCanId)
            return;
        const auto b = data.can_data;
        if (b.size() < 8)
            return;

        Sample s;
        s.raw_angle = static_cast<uint16_t>(
            (std::to_integer<unsigned>(b[0]) << 8) | std::to_integer<unsigned>(b[1]));
        s.timestamp_us = data.timestamp_us;
        s.valid = true;

        std::lock_guard lock(mutex_);
        sample_[bus] = s;
    }

    mutable std::mutex mutex_;
    Sample sample_[kBusCount];

    examples::BoardSession* board_ = nullptr;
};

namespace {

// Per-bus control state, advanced independently for bus 0 and bus 1.
struct BusState {
    bool present = false;
    float sine_centre = 0.0F;

    uint16_t prev_raw_angle = 0;
    uint16_t prev_angle_vel = 0;
    uint32_t prev_ts_us = 0;
    bool have_prev = false;
    int32_t full_turns = 0;

    float last_angle = 0.0F;
    float last_target = 0.0F;
    float last_vel_rpm = 0.0F;
    float last_out = 0.0F;
};

}  // namespace

int main() {
    std::signal(SIGINT, on_sigint);

    printf("Motor PID Sine -- two-CAN-bus bridge (CAN-FD, hw-timestamp velocity)\n");
    printf("  buses        : bus 0 + bus 1 (frames follow the bus mode: CAN-FD)\n");
    printf("  motor ID     : %u\n", kMotorId);
    printf("  control CAN  : 0x%03X\n", kControlCanId);
    printf("  feedback CAN : 0x%03X\n", kFeedbackCanId);
    printf("  Pure-P  Kp=%.1f  Imax=%.0f\n", kKp, kMaxCurrent);
    printf("  sine amp=%.0f  freq=%.1f Hz (%.0f rpm peak)  loop=%.0f Hz\n",
           kAmplitude, kFrequency,
           kAmplitude * 2.0F * float(M_PI) * kFrequency * 60.0F / 8192.0F,
           kLoopHz);
    printf("Connecting ...\n");

    MotorPidSine agent;
    auto board = examples::connect_any(agent);
    if (!board) {
        fprintf(stderr, "No compatible board found.\n");
        return 1;
    }
    if (board->can_bus_count() < 2) {
        fprintf(stderr, "%.*s has only %d CAN bus; this test needs two.\n",
            static_cast<int>(board->name().size()), board->name().data(),
            board->can_bus_count());
        return 1;
    }
    agent.bind(board.get());
    printf("Connected: %.*s.  Waiting for motor feedback ...\n",
        static_cast<int>(board->name().size()), board->name().data());

    std::array<BusState, kBusCount> bus{};

    // Wait for first feedback on either bus (up to 3 s)
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{3};
        while (g_running.load() && std::chrono::steady_clock::now() < deadline) {
            bool all_present = true;
            for (unsigned i = 0; i < kBusCount; ++i) {
                if (bus[i].present)
                    continue;
                auto s = agent.get_sample(i);
                if (s.valid) {
                    bus[i].present = true;
                    bus[i].sine_centre = static_cast<float>(s.raw_angle);
                    bus[i].prev_raw_angle = s.raw_angle;
                    bus[i].prev_angle_vel = s.raw_angle;
                    printf("  bus %u motor present -- initial angle %u (%.1f deg)%s\n", i,
                           s.raw_angle, s.raw_angle * 360.0F / 8192.0F,
                           s.timestamp_us ? "" : "  (no hw timestamp!)");
                } else {
                    all_present = false;
                }
            }
            if (all_present)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
    }

    unsigned active_buses = 0;
    for (const auto& b : bus)
        active_buses += b.present ? 1U : 0U;
    if (active_buses == 0) {
        fprintf(stderr, "ERROR: No motor feedback on any CAN bus within 3 s.\n");
        return 1;
    }
    for (unsigned i = 0; i < kBusCount; ++i)
        if (!bus[i].present)
            printf("  WARNING: bus %u has no motor -- skipping that bus.\n", i);

    // Phase 1: hold position while ramping current (2 s)
    constexpr float kHoldKp = 2.0F;
    constexpr float kRampDuration = 2.0F;
    constexpr float kRampRate = kMaxCurrent / kRampDuration;
    printf("Phase 1 -- holding position during current ramp (2 s) ...\n");
    {
        auto ramp_start = std::chrono::steady_clock::now();
        auto next_loop = ramp_start;
        while (g_running.load()) {
            const auto now = std::chrono::steady_clock::now();
            const float elapsed = std::chrono::duration<float>(now - ramp_start).count();
            if (elapsed >= kRampDuration)
                break;
            const float current_limit = std::min(kRampRate * elapsed, kMaxCurrent);

            for (unsigned i = 0; i < kBusCount; ++i) {
                if (!bus[i].present)
                    continue;
                auto s = agent.get_sample(i);
                const float err =
                    wrap_delta(bus[i].sine_centre - static_cast<float>(s.raw_angle));
                const float out = std::clamp(kHoldKp * err, -current_limit, current_limit);
                agent.send_control(i, static_cast<int16_t>(out));
            }

            next_loop += kLoopPeriod;
            if (next_loop < now) next_loop = now;
            std::this_thread::sleep_until(next_loop);
        }
    }

    // Re-centre each bus's sine wave on the motor's actual position after ramp.
    for (unsigned i = 0; i < kBusCount; ++i) {
        if (!bus[i].present)
            continue;
        auto s = agent.get_sample(i);
        if (s.valid)
            bus[i].sine_centre = static_cast<float>(s.raw_angle);
        printf("Phase 2 -- bus %u sine centre %.0f (%.1f deg)\n", i, bus[i].sine_centre,
               bus[i].sine_centre * 360.0F / 8192.0F);
    }

    // Phase 2: sine on every active bus
    uint64_t loop_count = 0;
    auto next_loop = std::chrono::steady_clock::now();
    auto last_report = next_loop;

    while (g_running.load()) {
        const auto now = std::chrono::steady_clock::now();

        const float t = static_cast<float>(loop_count) / kLoopHz;
        const float target_angle =
            kAmplitude * std::sin(2.0F * float(M_PI) * kFrequency * t);

        for (unsigned i = 0; i < kBusCount; ++i) {
            auto& bs = bus[i];
            if (!bs.present)
                continue;

            const float target = bs.sine_centre + target_angle;

            auto s = agent.get_sample(i);
            const uint16_t raw_now = s.raw_angle;

            // Multi-turn unwrap.
            const int16_t delta16 = static_cast<int16_t>(raw_now - bs.prev_raw_angle);
            if (delta16 > 4096)
                bs.full_turns--;
            else if (delta16 < -4096)
                bs.full_turns++;
            bs.prev_raw_angle = raw_now;
            const float continuous_angle =
                static_cast<float>(bs.full_turns) * 8192.0F + static_cast<float>(raw_now);

            // Derived velocity from hw timestamp delta.
            float derived_vel_rpm = 0.0F;
            if (bs.have_prev && s.timestamp_us.has_value()) {
                const int32_t ts_delta_us =
                    static_cast<int32_t>(s.timestamp_us.value() - bs.prev_ts_us);
                if (ts_delta_us > 0) {
                    const float dt = static_cast<float>(ts_delta_us) * 1.0e-6F;
                    const float d_angle = wrap_delta(
                        static_cast<float>(raw_now) - static_cast<float>(bs.prev_angle_vel));
                    derived_vel_rpm = (d_angle / dt) * 60.0F / 8192.0F;
                }
            }
            if (s.timestamp_us.has_value()) {
                bs.prev_ts_us = s.timestamp_us.value();
                bs.prev_angle_vel = raw_now;
                bs.have_prev = true;
            }

            // Pure P controller.
            const float pos_error = wrap_delta(target - continuous_angle);
            const float output = std::clamp(kKp * pos_error, -kMaxCurrent, kMaxCurrent);
            agent.send_control(i, static_cast<int16_t>(output));

            bs.last_angle = continuous_angle;
            bs.last_target = target;
            bs.last_vel_rpm = derived_vel_rpm;
            bs.last_out = output;
        }

        next_loop += kLoopPeriod;
        if (next_loop < now)
            next_loop = now;
        std::this_thread::sleep_until(next_loop);
        ++loop_count;

        // Report (1 Hz)
        if (now - last_report >= std::chrono::seconds{1}) {
            last_report = now;
            for (unsigned i = 0; i < kBusCount; ++i) {
                if (!bus[i].present)
                    continue;
                const auto& bs = bus[i];
                printf("bus %u | ang %6.0f (%5.1f deg) | tgt %6.0f (%5.1f deg) | "
                       "vel %6.1f rpm | out %6.0f | turn %d\n",
                       i, bs.last_angle, bs.last_angle * 360.0F / 8192.0F, bs.last_target,
                       bs.last_target * 360.0F / 8192.0F, bs.last_vel_rpm, bs.last_out,
                       bs.full_turns);
            }
            printf("  loop %lu Hz\n", loop_count);
            loop_count = 0;
        }
    }

    printf("\nZeroing motors ...\n");
    for (int i = 0; i < 5; ++i) {
        for (unsigned b = 0; b < kBusCount; ++b)
            if (bus[b].present)
                agent.send_control(b, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    printf("Stopped.\n");
    return 0;
}
