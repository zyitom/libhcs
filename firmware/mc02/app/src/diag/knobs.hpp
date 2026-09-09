#pragma once

#include <cstdint>

// Debugger-writable knobs and read-only mirrors, for bringing a peripheral up
// against real hardware without a host, a protocol change or a rebuild between
// each attempt. Ozone's Watched Data window updates and accepts writes while the
// program runs (Cortex-M7 background memory access), so these turn a debug
// session into a control panel: type a frequency, hear it.
//
// Why mirrors rather than watching the objects directly. The raw samples are
// already watchable at libhcs::firmware::adc::mc02_samples, but reading a
// battery voltage from them means averaging sixteen half-words and applying the
// divider by hand on every glance. millivolts() cannot be watched instead --
// Ozone evaluates symbol expressions, it does not call target functions, so the
// only way to see a computed value is for the target to compute it into a
// variable. That is what battery_mv is.
//
// Why writing tone_hz beats poking TIM12->ARR from the debugger, which needs no
// firmware support at all: writing the register tests the transducer and
// nothing else, while writing a frequency here goes through the same
// Buzzer::set_tone() the application uses, so the reload arithmetic and the
// duty-cycle mapping are under test too.
//
// Compiled out entirely by default. Cost when enabled is one HAL_GetTick() read
// and a compare per main-loop pass; the work below runs at kPollIntervalMs.
// That bound is deliberate -- this board's downlink packet rate is a strong,
// non-monotonic function of the main-loop period (see firmware/mc02/AGENTS.md),
// so unconditional per-pass work would move throughput benchmarks by more than
// the effect most of them are trying to measure.

namespace libhcs::firmware::diag::knobs {

#if defined(libhcs_APP_DEBUG_KNOBS) && libhcs_APP_DEBUG_KNOBS

inline constexpr bool kEnabled = true;

// Write to sound a tone; 0 silences. Anything under Buzzer::kMinFrequencyHz
// (100 Hz) also silences, which is what set_tone() does with it.
inline volatile uint16_t tone_hz = 0;
// 0..255. 255 maps to a 50% duty cycle, the loudest a passive buzzer gets.
inline volatile uint8_t tone_loudness = 128;

// Note-name alternative to tone_hz, so a pitch can be typed as a pitch.
// tone_semitone takes a buzzer::Pitch value (0 = C, 1 = C#, ... 11 = B) and
// tone_octave a scientific-pitch octave, so {9, 4} is A4 = 440 Hz. They go
// through the same buzzer::pitch() the kBootMelody constants use, which is the
// point: the octave shift is under test, not just the PWM.
//
// The two knobs cannot both drive the channel, so tone_semitone arbitrates:
// kSemitoneOff, the default, hands control back to tone_hz, which leaves a
// build that never touches these behaving exactly as it did before they
// existed. Out-of-range values silence instead -- a debugger writes these bytes
// with no validation of its own, and pitch() would otherwise index
// kOctave4Hz[] off its end or shift past what its uint16_t return can hold.
inline constexpr int8_t kSemitoneOff = -1;
inline constexpr int8_t kSemitoneMax = 11;

// Above this the 1 MHz counter cannot express the pitch usefully: B9 is
// 15808 Hz, a reload of 63, and one octave further overflows pitch().
inline constexpr uint8_t kMaxOctave = 9;

inline volatile int8_t tone_semitone = kSemitoneOff;
inline volatile uint8_t tone_octave = 5;

// Read-only mirrors of the battery gauge, refreshed at kPollIntervalMs.
inline volatile uint32_t battery_raw = 0;
inline volatile uint32_t battery_mv = 0;
inline volatile uint8_t battery_started = 0;

void poll();

#else

inline constexpr bool kEnabled = false;

inline void poll() {}

#endif

} // namespace libhcs::firmware::diag::knobs
