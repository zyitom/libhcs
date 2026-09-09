#pragma once

#include <cstdint>

#include "board_app.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::led {

// Plain GPIO RGB LED backend, used by boards whose LED is driven by simply
// pulling a pad high (active-high) instead of a WS2812 serial protocol. It
// exposes the same interface as Ws2812 so the shared Led driver is agnostic to
// the underlying hardware. Without PWM the channels can only be on or off, so
// the per-channel value is thresholded at its midpoint, turning the WS2812
// brightness ramps into blinks while keeping the blink-based light language.
class GpioLed : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<GpioLed>;

    static constexpr uint8_t kOnThreshold = 128;

    GpioLed() { board::init_led_pins(); }

    // This mirrors the stateful LED backend interface even though this GPIO
    // implementation does not need instance data.
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    bool set_value(uint8_t red, uint8_t green, uint8_t blue) {
        red_pin_.set_active(red >= kOnThreshold);
        green_pin_.set_active(green >= kOnThreshold);
        blue_pin_.set_active(blue >= kOnThreshold);
        return true;
    }

private:
    // Resolved once at construction rather than read from the board table on
    // every update. Boards that serve more than one PCB pick their LED pads from
    // the runtime identity (see boards/hpm5321/app/board_app.hpp), and this runs
    // at 1 kHz -- no reason to re-resolve three pins per tick. GpioPin is an
    // 8-byte POD, so caching all three costs 24 bytes.
    GpioPin red_pin_ = board::led_red_pin();
    GpioPin green_pin_ = board::led_green_pin();
    GpioPin blue_pin_ = board::led_blue_pin();
};

inline constinit GpioLed::Lazy gpio_led;

} // namespace libhcs::firmware::led
