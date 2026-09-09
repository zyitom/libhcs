#pragma once

#include <cstdint>

#include <gpio.h>
#include <main.h>

#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

// Debounced push-button on a GPIO input.
//
// Two halves, same split as buzzer.hpp: everything above "mc02 binding" names no
// pin of this board and can be copied into another STM32 project unedited; the
// binding at the bottom is the only part that knows about PA15.
//
// Construction is inert and start() only latches the current level -- it
// configures no pin, because a push-button input is exactly the kind of thing
// MX_GPIO_Init() has already set up from the .ioc, and re-doing it here would be
// a second source of truth for the pull direction.
//
// Nothing in the mc02 app calls this yet. It is written and left unstarted on
// purpose: the driver is the reusable part, deciding what a long press should do
// is the board's business.

namespace libhcs::firmware::key {

enum class Event : uint8_t {
    kNone = 0,
    // Edge events, reported once each on the poll() that observes them.
    kPressed = 1,
    kReleased = 2,
    // Fired once per press, `long_press_ms` after kPressed, while still held.
    // A press long enough to fire this still reports kReleased afterwards.
    kLongPressed = 3,
};

struct Config {
    GPIO_TypeDef* port;
    uint16_t pin;
    // Level the pin sits at with the button NOT pressed. A button to ground with
    // a pull-up reads GPIO_PIN_SET when free.
    GPIO_PinState released_state;
    // A level must hold this long before it counts. 20 ms covers the contact
    // bounce of every tactile switch worth using; raising it only adds latency.
    uint16_t debounce_ms;
    uint16_t long_press_ms;
};

class Key : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Key>;

    static constexpr uint16_t kDefaultDebounceMs = 20;
    static constexpr uint16_t kDefaultLongPressMs = 2000;

    Key() = default;

    // Latch the current level as the starting point, so a board that boots with
    // the button already held does not report a spurious kPressed on the first
    // poll. Deliberately does not touch GPIO configuration; see the file header.
    void start(const Config& config) {
        config_ = config;
        stable_pressed_ = raw_pressed();
        candidate_pressed_ = stable_pressed_;
        last_change_tick_ = HAL_GetTick();
        long_press_fired_ = stable_pressed_;
        started_ = true;
    }

    [[nodiscard]] bool started() const { return started_; }
    [[nodiscard]] bool pressed() const { return stable_pressed_; }

    // Call from the main loop. Returns at most one event per call; a press that
    // is both long and then released reports kLongPressed on one poll and
    // kReleased on a later one, never both at once.
    Event poll() {
        if (!started_)
            return Event::kNone;

        const uint32_t tick = HAL_GetTick();
        const bool raw = raw_pressed();

        if (raw != candidate_pressed_) {
            candidate_pressed_ = raw;
            last_change_tick_ = tick;
            return Event::kNone;
        }

        if (candidate_pressed_ != stable_pressed_) {
            if (tick - last_change_tick_ < config_.debounce_ms)
                return Event::kNone;
            stable_pressed_ = candidate_pressed_;
            // Restart the hold timer from the debounced edge, not from the raw
            // one, so long_press_ms measures the press the caller was told about.
            last_change_tick_ = tick;
            if (stable_pressed_) {
                long_press_fired_ = false;
                return Event::kPressed;
            }
            return Event::kReleased;
        }

        if (stable_pressed_ && !long_press_fired_
            && tick - last_change_tick_ >= config_.long_press_ms) {
            long_press_fired_ = true;
            return Event::kLongPressed;
        }

        return Event::kNone;
    }

private:
    [[nodiscard]] bool raw_pressed() const {
        return HAL_GPIO_ReadPin(config_.port, config_.pin) != config_.released_state;
    }

    Config config_{};
    uint32_t last_change_tick_ = 0;
    bool stable_pressed_ = false;
    bool candidate_pressed_ = false;
    bool long_press_fired_ = false;
    bool started_ = false;
};

// ---------------------------------------------------------------------------
// mc02 binding. Everything above this line is board-agnostic.
// ---------------------------------------------------------------------------
//
// PA15, pulled up, shorted to ground when pressed -- so free reads high. The
// bootloader already relies on exactly this polarity to decide whether to stay
// in DFU (bootloader/src/main.cpp: KEY low at reset means stay).
//
// PA15 is also JTDI, but the SWD debug port only uses SWDIO/SWCLK and the .ioc
// has claimed the pin as GPIO_Input, so there is no conflict with a debugger.
//
// Returned by a function rather than held in a constinit object because the
// GPIOA macro expands to an integer cast to a pointer, which is not a constant
// expression -- the same reason power.hpp reads its port macros inside function
// bodies instead of storing them.
[[nodiscard]] inline Config mc02_config() {
    return Config{
        .port = KEY_GPIO_Port,
        .pin = KEY_Pin,
        .released_state = GPIO_PIN_SET,
        .debounce_ms = Key::kDefaultDebounceMs,
        .long_press_ms = Key::kDefaultLongPressMs,
    };
}

inline constinit Key::Lazy key;

} // namespace libhcs::firmware::key
