#pragma once

// Direct cross-board skew measurement over the existing UART0 wires, using
// GPTMR hardware compare and capture -- no CPU in either the send or the
// receive instant.
//
// WHY THIS EXISTS. Every earlier attempt measured the timeline through the CAN
// timestamp unit, whose timebase is PTPC. PTPC hangs off PLL0, which the
// HPM5300 datasheet describes as fractional-N with spread-spectrum support, and
// its instantaneous rate wanders by hundreds of ppm. That wander -- not the
// timeline, and not the boards -- is what produced every bogus figure from
// 452 us down to 30 us.
//
// GPTMR is in CLK_SRC_GROUP_COMMON, so it can be clocked straight from the
// 24 MHz crystal, the same source the machine timer divides by 6. No PLL, no
// fractional divider, no spread spectrum: the rate is as steady as the crystal.
//
// WHAT IT IS NOT: an exact 3000 ticks per microframe. The microframe axis is the
// USB HOST's clock (SOF), the GPTMR counter is THIS BOARD's crystal, and the two
// are independent oscillators -- measured about +80 ppm apart, i.e. ~3000.25
// ticks per microframe. Assuming 3000 puts 4 us of error into a 50 ms lead, so
// the ratio is FITTED here, exactly as timebase.cpp fits the machine timer.
// (An earlier revision of this file assumed the exact 3000 on the grounds that
// "both come from the crystal". Only one of them does.)
//
// WIRING: none to add. UART0 already crosses between the two boards
// (A.TXD<->B.RXD both ways), and those pins carry GPTMR0 channel 1's compare
// output and capture input as ALT1:
//
//   PB08  UART0 TXD  /  GPTMR0_COMP_1   -- pulse out
//   PB09  UART0 RXD  /  GPTMR0_CAPT_1   -- pulse in
//
// So the existing cable becomes a bidirectional hardware timing link. UART0 is
// unavailable while this is enabled, which is why it is its own build option.
//
// MEASUREMENT. Each board fires a pulse at an agreed microframe and captures
// the other's. Board A's pulse leaves at microframe a_tx and is captured at
// b_rx; board B's leaves at b_tx and is captured at a_rx. Then
//
//     skew = ((b_rx - a_tx) - (a_rx - b_tx)) / 2
//
// Cable propagation and the pad/synchroniser delay appear with the same sign in
// both directions and cancel in the difference -- the standard two-way
// exchange. What survives is the difference between the two boards' own ideas of
// when microframe k happens, which is exactly what an action scheduled on the
// shared axis would inherit.
//
// Resolution is one GPTMR tick, 41.7 ns. That is coarse against the ~42 ns the
// timeline is expected to achieve, but it is coarse in a harmless way: the
// quantisation is zero-mean and dithered by the real jitter, so over a thousand
// exchanges the MEAN converges to well under a nanosecond, and the SPREAD only
// grows by the quadrature term (42 -> 44 ns), which can be subtracted out.

#include <cstdint>

namespace libhcs::firmware::sync::pulse {

#if defined(libhcs_APP_PULSE_TEST) && libhcs_APP_PULSE_TEST

inline constexpr bool kEnabled = true;

// Nominal ticks per microframe: 24 MHz GPTMR, 125 us microframe. The starting
// point of the fit and the sanity bound around it, never the value used.
inline constexpr std::uint32_t kNominalTicksPerMicroframe = 3000;

// Same shape as the machine-timer fit in timebase.cpp, for the same reason: the
// BASELINE (128 x 64 microframes = 1.024 s) sets the slope accuracy and the
// sample COUNT averages the SOF interrupt's entry jitter down. Scheduling off a
// single SOF sample instead -- what the first revision did -- puts that one
// sample's +-0.5 us straight into the pulse, ten times the effect being
// measured.
inline constexpr std::uint32_t kSampleDecimation = 64;
inline constexpr std::uint32_t kSampleCount = 128;

// Pin mux, GPTMR clock, channel setup. Must run after the UART driver, whose
// pin mux this deliberately overrides.
void init();

// SOF interrupt: record where the GPTMR counter stood at this microframe.
void note_sof(std::uint64_t microframe);

// Main loop, 1 kHz tick: recomputes the microframe-to-tick line. Does real work
// only every refit period.
void poll(std::uint32_t tick_ms);

// Arm a hardware pulse for the given absolute microframe. False when the fit is
// not ready, or the target is too near (the compare must be written before the
// counter reaches it) or too far to trust.
bool schedule(std::uint64_t microframe);

// Main loop: hands back one captured pulse, converted to the shared axis in
// Q16 microframes. False when nothing new arrived.
bool take_capture(std::uint64_t& microframe_q16);

// Capture interrupt entry point (bound to IRQn_GPTMR0 in the .cpp).
void isr_handler();

// The fitted ticks per microframe, Q16. Published so the clock relationship is
// verified on hardware rather than trusted. Expect ~196624000 (3000.25), NOT
// 3000<<16: the excess is the board crystal running fast against the host's USB
// clock, and it must agree with the +80 ppm the machine-timer fit reports.
std::uint32_t measured_ticks_per_microframe_q16();

#else

inline constexpr bool kEnabled = false;

inline void init() {}
inline void note_sof(std::uint64_t) {}
inline void poll(std::uint32_t) {}
inline bool schedule(std::uint64_t) { return false; }
inline bool take_capture(std::uint64_t&) { return false; }
inline std::uint32_t measured_ticks_per_microframe_q16() { return 0; }

#endif

} // namespace libhcs::firmware::sync::pulse
