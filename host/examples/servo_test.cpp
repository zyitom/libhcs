#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include "common/multi_board.hpp"

// Interactive servo/PWM tester for any board that exposes PWM-capable GPIO
// channels (c_board: 7 channels, mc02: 4 channels). Channels are numbered 1..N
// on the command line; the board's pins run at 50 Hz (20 ms period).
// Standard servo pulse range: ~1000-2000 us.

namespace {

constexpr uint32_t k_servo_min_us = 500;
constexpr uint32_t k_servo_max_us = 2500;
constexpr uint32_t k_servo_center_us = 1500;
constexpr uint32_t k_pwm_period_us = 20000;

uint16_t pulse_us_to_duty16(uint32_t pulse_us) {
    return static_cast<uint16_t>(pulse_us * 65535u / k_pwm_period_us);
}

uint32_t angle_to_pulse_us(int deg) {
    if (deg < 0)
        deg = 0;
    if (deg > 180)
        deg = 180;
    return k_servo_min_us
        + (k_servo_max_us - k_servo_min_us) * static_cast<uint32_t>(deg) / 180;
}

} // namespace

class ServoTester : public examples::BoardReceiver {
public:
    void bind(examples::BoardSession* board) { board_ = board; }

    int channel_count() const { return board_->gpio_channel_count(); }

    // User-facing channels are 1-based; descriptor indices are 0-based.
    bool valid_channel(int ch) const { return ch >= 1 && ch <= channel_count(); }

    void set_pulse(int ch, uint32_t pulse_us) {
        if (!valid_channel(ch)) {
            printf("Invalid channel %d (valid: 1-%d)\n", ch, channel_count());
            return;
        }
        if (pulse_us < k_servo_min_us || pulse_us > k_servo_max_us)
            printf("Warning: %u us is outside normal servo range (%u-%u us)\n",
                pulse_us, k_servo_min_us, k_servo_max_us);

        const uint16_t duty = pulse_us_to_duty16(pulse_us);
        board_->transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_analog(ch - 1, {.value = duty});
        });
        printf("CH%d -> %u us\n", ch, pulse_us);
    }

    void set_angle(int ch, int deg) {
        if (!valid_channel(ch)) {
            printf("Invalid channel %d (valid: 1-%d)\n", ch, channel_count());
            return;
        }
        const uint32_t pulse_us = angle_to_pulse_us(deg);
        const uint16_t duty = pulse_us_to_duty16(pulse_us);
        board_->transmit([&](examples::BoardTransmitter& tx) {
            tx.gpio_analog(ch - 1, {.value = duty});
        });
        printf("CH%d -> %d deg (%u us)\n", ch, deg, pulse_us);
    }

    void center_all() {
        const uint16_t duty = pulse_us_to_duty16(k_servo_center_us);
        const int count = channel_count();
        board_->transmit([&](examples::BoardTransmitter& tx) {
            for (int i = 0; i < count; ++i)
                tx.gpio_analog(i, {.value = duty});
        });
        printf("All %d channels -> center (%u us)\n", count, k_servo_center_us);
    }

    void sweep(int ch) {
        if (!valid_channel(ch)) {
            printf("Invalid channel %d (valid: 1-%d)\n", ch, channel_count());
            return;
        }
        printf("Sweeping CH%d (0 -> 180 -> 0 deg)...\n", ch);
        for (int deg = 0; deg <= 180; deg += 3) {
            const uint32_t pulse_us = angle_to_pulse_us(deg);
            const uint16_t duty = pulse_us_to_duty16(pulse_us);
            board_->transmit([&](examples::BoardTransmitter& tx) {
                tx.gpio_analog(ch - 1, {.value = duty});
            });
            printf("\r  %3d deg (%u us)   ", deg, pulse_us);
            fflush(stdout);
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
        for (int deg = 180; deg >= 0; deg -= 3) {
            const uint32_t pulse_us = angle_to_pulse_us(deg);
            const uint16_t duty = pulse_us_to_duty16(pulse_us);
            board_->transmit([&](examples::BoardTransmitter& tx) {
                tx.gpio_analog(ch - 1, {.value = duty});
            });
            printf("\r  %3d deg (%u us)   ", deg, pulse_us);
            fflush(stdout);
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
        printf("\nDone.\n");
    }

private:
    examples::BoardSession* board_ = nullptr;
};

static void print_help(int channel_count) {
    printf(
        "\nCommands:\n"
        "  set <ch> <us>      Set channel to pulse width in us   e.g. set 1 1500\n"
        "  angle <ch> <deg>   Set channel by angle (0-180 deg)   e.g. angle 2 90\n"
        "  sweep <ch>         Sweep channel 0->180->0 deg        e.g. sweep 3\n"
        "  center             Center all channels (1500 us)\n"
        "  help               Show this message\n"
        "  quit               Exit\n"
        "\nChannels: 1-%d  |  Pulse range: %u-%u us  |  Frequency: 50 Hz\n\n",
        channel_count, k_servo_min_us, k_servo_max_us);
}

int main() {
    printf("Servo tester - auto-detecting board...\n");

    ServoTester tester;
    auto board = examples::connect_any(tester);
    if (!board) {
        fprintf(stderr, "No compatible board found.\n");
        return 1;
    }
    if (board->gpio_channel_count() == 0) {
        fprintf(stderr, "%.*s has no PWM/servo GPIO channels.\n",
            static_cast<int>(board->name().size()), board->name().data());
        return 1;
    }
    tester.bind(board.get());

    printf("Connected: %.*s (%d servo channels). Type 'help' for commands.\n",
        static_cast<int>(board->name().size()), board->name().data(),
        board->gpio_channel_count());
    tester.center_all();

    char line[128];
    while (true) {
        printf("> ");
        fflush(stdout);
        if (!fgets(line, sizeof(line), stdin))
            break;

        char cmd[32]{};
        int arg1 = 0;
        int arg2 = 0;
        const int n = sscanf(line, "%31s %d %d", cmd, &arg1, &arg2);
        if (n <= 0)
            continue;

        try {
            if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0
                || strcmp(cmd, "q") == 0) {
                break;
            } else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0) {
                print_help(tester.channel_count());
            } else if (strcmp(cmd, "center") == 0) {
                tester.center_all();
            } else if (strcmp(cmd, "set") == 0 && n >= 3) {
                tester.set_pulse(arg1, static_cast<uint32_t>(arg2));
            } else if (strcmp(cmd, "angle") == 0 && n >= 3) {
                tester.set_angle(arg1, arg2);
            } else if (strcmp(cmd, "sweep") == 0 && n >= 2) {
                tester.sweep(arg1);
            } else {
                printf("Unknown command or missing arguments. Type 'help' for usage.\n");
            }
        } catch (const std::exception& e) {
            printf("Error: %s\n", e.what());
        }
    }

    return 0;
}
