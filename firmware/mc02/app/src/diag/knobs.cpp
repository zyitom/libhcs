#include "firmware/mc02/app/src/diag/knobs.hpp"

#if defined(libhcs_APP_DEBUG_KNOBS) && libhcs_APP_DEBUG_KNOBS

# include <main.h>

# include "firmware/mc02/app/src/adc/battery.hpp"
# include "firmware/mc02/app/src/buzzer/buzzer.hpp"

namespace libhcs::firmware::diag::knobs {

namespace {

// Fast enough that a typed frequency sounds immediately, slow enough that the
// per-pass cost stays at one tick read. Sixteen ADC samples take 1.25 ms to
// refill, so this also never reports the same averaging window twice.
constexpr uint32_t kPollIntervalMs = 20;

uint32_t last_poll_tick = 0;
uint16_t applied_tone_hz = 0;
uint8_t applied_loudness = 0;

// Resolve the two ways of asking for a tone into the one frequency poll()
// applies. Each volatile is read exactly once: the debugger can land a write
// between two reads of the same knob, and re-reading tone_semitone after the
// range check would let a stale value through it.
uint16_t requested_frequency() {
    const int8_t semitone = tone_semitone;
    if (semitone == kSemitoneOff)
        return tone_hz;
    const uint8_t octave = tone_octave;
    if (semitone < 0 || semitone > kSemitoneMax || octave > kMaxOctave)
        return 0;
    return buzzer::pitch(static_cast<buzzer::Pitch>(semitone), octave);
}

} // namespace

void poll() {
    const uint32_t tick = HAL_GetTick();
    if (tick - last_poll_tick < kPollIntervalMs)
        return;
    last_poll_tick = tick;

    battery_raw = adc::battery->raw();
    battery_mv = adc::battery->millivolts();
    battery_started = adc::battery->started() ? 1U : 0U;

    // Edge-triggered: re-applying the same tone every 20 ms would restart the
    // counter through set_tone()'s EGR write and audibly chop the note.
    const uint16_t requested_hz = requested_frequency();
    const uint8_t requested_loudness = tone_loudness;
    if (requested_hz == applied_tone_hz && requested_loudness == applied_loudness)
        return;
    applied_tone_hz = requested_hz;
    applied_loudness = requested_loudness;

    // stop() before set_tone(): a melody still in flight owns the channel, and
    // its next poll() would overwrite the tone within a note otherwise. The boot
    // chirp is 210 ms, so without this a frequency typed early is simply lost.
    buzzer::buzzer->stop();
    buzzer::buzzer->set_tone(requested_hz, requested_loudness);
}

} // namespace libhcs::firmware::diag::knobs

#endif
