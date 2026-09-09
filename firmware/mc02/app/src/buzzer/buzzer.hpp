#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <main.h>
#include <tim.h>

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

// Passive-buzzer driver on a timer PWM channel.
//
// The file has two halves and they are meant to be separated:
//
//   1. Everything up to "mc02 binding" is board-agnostic. It names no handle,
//      no pin and no clock of this board -- the timer, the channel and the
//      counter rate all arrive as constructor arguments. Copying this half into
//      another STM32 project needs no edits.
//   2. The binding at the bottom is the only mc02-specific part: which timer,
//      which channel, what the counter ticks at. Another board rewrites those
//      three values and nothing else.
//
// Construction is inert on purpose -- it touches no register. Nothing sounds,
// and no timer channel is enabled, until someone calls start(). A project that
// only wants the driver present can construct it and never start it; a project
// that has not run its MX_TIMx_Init() yet must not start it. Same discipline as
// power.hpp, for the same reason: what the hardware does at power-on should be
// the board's decision, not a side effect of a constructor running.

namespace libhcs::firmware::buzzer {

// Equal-temperament pitch, so melodies can be written in note names instead of
// raw hertz.
//
// The vendor BSP this board's peripherals were traced from spells this out as
// ~130 `extern const float BUZZER_FREQUENCY_*` globals in a .cpp -- half a
// kilobyte of float constants that, being extern, cannot fold at the use site
// and must be loaded at run time. One octave plus a shift is exact instead:
// octaves are powers of two by definition, so C6 is C4 << 2 with no error
// introduced, and every pitch below folds to an immediate at compile time.
enum class Pitch : uint8_t {
    kC = 0,
    kCSharp = 1,
    kD = 2,
    kDSharp = 3,
    kE = 4,
    kF = 5,
    kFSharp = 6,
    kG = 7,
    kGSharp = 8,
    kA = 9,
    kASharp = 10,
    kB = 11,
};

// Octave 4 in scientific pitch notation (A4 = 440 Hz), rounded to whole hertz.
// The rounding error is at most 0.5 Hz here and doubles with the octave, so
// even A7 lands within 4 Hz of true -- far under the pitch resolution of a
// piezo transducer, and under what the 1 MHz counter can express anyway.
inline constexpr uint16_t kOctave4Hz[]{262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};

[[nodiscard]] constexpr uint16_t pitch(Pitch semitone, unsigned octave) {
    const uint32_t base = kOctave4Hz[static_cast<unsigned>(semitone)];
    const uint32_t scaled = octave >= 4 ? base << (octave - 4) : base >> (4 - octave);
    return static_cast<uint16_t>(scaled);
}

// One step of a melody. A zero frequency is a rest -- the channel goes quiet for
// the duration instead of sounding a note.
struct Note {
    uint16_t frequency_hz;
    uint16_t duration_ms;
};

class Buzzer : private core::utility::Immovable {
public:
    // Volume is duty cycle, and a passive buzzer is loudest at 50%: past that
    // the pulse merely widens without adding drive energy at the fundamental.
    // So kLoudnessMax maps to a 50% duty, not 100%.
    static constexpr uint8_t kLoudnessMax = 255;
    static constexpr uint8_t kDefaultLoudness = 128;

    // Below this the transducer is inaudible and the reload would overflow the
    // 16-bit counter on a 1 MHz tick anyway (1 MHz / 16 Hz > 65535).
    static constexpr uint16_t kMinFrequencyHz = 100;

    // `tick_hz` is the timer's counter rate after its prescaler, not the kernel
    // clock: the caller has already chosen a prescaler in CubeMX, and every note
    // here is just `ARR = tick_hz / f - 1`. Passing it in rather than deriving
    // it from PSC and the RCC tree keeps this half of the file free of any
    // clock-tree knowledge, which is what makes it portable.
    constexpr Buzzer(TIM_HandleTypeDef* timer, uint32_t channel, uint32_t tick_hz)
        : timer_(timer)
        , channel_(channel)
        , tick_hz_(tick_hz) {}

    // Enable the PWM channel, silent. Must run after the board's MX_TIMx_Init()
    // -- that is what puts the pin into its alternate function.
    void start() {
        core::utility::assert_always(HAL_TIM_PWM_Start(timer_, channel_) == HAL_OK);
        mute();
        started_ = true;
    }

    [[nodiscard]] bool started() const { return started_; }

    // Sound one tone until the next call. Takes effect immediately: the update
    // event below reloads the shadow ARR that TIM_AUTORELOAD_PRELOAD_ENABLE
    // would otherwise hold back until the current period ends -- audible as a
    // stale note of up to ARR microseconds when stepping down in frequency.
    void set_tone(uint16_t frequency_hz, uint8_t loudness) {
        if (frequency_hz < kMinFrequencyHz || loudness == 0) {
            mute();
            return;
        }

        const uint32_t reload = tick_hz_ / frequency_hz;
        // Half of `reload` is the 50% ceiling, so the divisor is 2 * kLoudnessMax
        // rather than kLoudnessMax.
        const uint32_t compare = (reload * loudness) / (2U * kLoudnessMax);

        __HAL_TIM_SET_AUTORELOAD(timer_, reload - 1);
        __HAL_TIM_SET_COMPARE(timer_, channel_, compare);
        __HAL_TIM_SET_COUNTER(timer_, 0);
        timer_->Instance->EGR = TIM_EGR_UG;
    }

    void set_tone(Pitch semitone, unsigned octave, uint8_t loudness = kDefaultLoudness) {
        set_tone(pitch(semitone, octave), loudness);
    }

    // Silence without stopping the timer. Leaves ARR alone: the next set_tone()
    // rewrites it anyway, and keeping the channel running means no PWM restart
    // glitch between the notes of a melody.
    void mute() { __HAL_TIM_SET_COMPARE(timer_, channel_, 0); }

    // Start a melody and return; poll() advances it. `notes` is held by
    // reference for the duration, so it must have static storage -- the
    // kBootMelody-style constants below, not a local array.
    void play(std::span<const Note> notes, uint8_t loudness = kDefaultLoudness) {
        if (notes.empty()) {
            stop();
            return;
        }

        sequence_ = notes;
        loudness_ = loudness;
        index_ = 0;
        note_started_tick_ = HAL_GetTick();
        set_tone(sequence_[0].frequency_hz, loudness_);
    }

    void stop() {
        sequence_ = {};
        mute();
    }

    [[nodiscard]] bool playing() const { return !sequence_.empty(); }

    // Polled from the main loop. Idle cost is one load and a branch, which is
    // why it can sit unconditionally in a forwarding loop; the same reason the
    // LED is paced off HAL_GetTick applies here, except that a melody is
    // measured in milliseconds by definition, so a loop-iteration count would
    // not merely look wrong, it would race through the whole melody in a
    // millisecond.
    void poll() {
        if (sequence_.empty())
            return;

        const uint32_t tick = HAL_GetTick();
        if (tick - note_started_tick_ < sequence_[index_].duration_ms)
            return;

        note_started_tick_ = tick;
        if (++index_ >= sequence_.size()) {
            stop();
            return;
        }
        set_tone(sequence_[index_].frequency_hz, loudness_);
    }

private:
    TIM_HandleTypeDef* timer_;
    uint32_t channel_;
    uint32_t tick_hz_;

    std::span<const Note> sequence_;
    std::size_t index_ = 0;
    uint32_t note_started_tick_ = 0;
    uint8_t loudness_ = kDefaultLoudness;
    bool started_ = false;
};

// Short rising chirp at startup. Its job is to say "this firmware reached its
// main loop" without a host attached and without line of sight to the LED, which
// is the one thing neither the WS2812 nor the USB link can report on its own: a
// board that faults during bring-up looks identical to an unpowered one.
inline constexpr Note kBootMelody[]{
    {.frequency_hz = pitch(Pitch::kC, 6), .duration_ms = 60},
    {.frequency_hz = pitch(Pitch::kE, 6), .duration_ms = 60},
    {.frequency_hz = pitch(Pitch::kG, 6), .duration_ms = 90},
};

// Two-tone alternation, for a condition the operator needs to notice while
// looking somewhere else.
inline constexpr Note kAlarmMelody[]{
    {.frequency_hz = pitch(Pitch::kA, 5), .duration_ms = 150},
    {.frequency_hz = 0, .duration_ms = 50},
    {.frequency_hz = pitch(Pitch::kA, 5), .duration_ms = 150},
    {.frequency_hz = 0, .duration_ms = 50},
    {.frequency_hz = pitch(Pitch::kA, 5), .duration_ms = 150},
};

// ---------------------------------------------------------------------------
// mc02 binding. Everything above this line is board-agnostic.
// ---------------------------------------------------------------------------
//
// PB15, TIM12 CH2. mc02_slave.ioc gives TIM12 Prescaler = 274 and Period = 249,
// and the APB1 timer kernel runs at 275 MHz (APB1 at 137.5 MHz with
// D2PPRE1 = DIV2 doubles the timer clock), so the counter ticks at exactly
// 275 MHz / 275 = 1 MHz -- the value passed below. The generated defaults happen
// to sound 1 MHz / 250 = 4 kHz.
//
// Nothing called MX_TIM12_Init() before this file existed: the timer was in the
// .ioc but never started, so PB15 stayed an unconfigured GPIO and the buzzer was
// silent. That call now sits in App() next to the other MX_TIM*_Init.
//
// Main-loop only: play() and poll() touch no atomics and are never called from
// an ISR, unlike the LED, whose buffer-full counters are set from interrupt
// context.
inline constexpr uint32_t kMc02TimerTickHz = 1000000;

inline constinit utility::Lazy<Buzzer, TIM_HandleTypeDef*, uint32_t, uint32_t> buzzer{
    &htim12, TIM_CHANNEL_2, kMc02TimerTickHz};

} // namespace libhcs::firmware::buzzer
