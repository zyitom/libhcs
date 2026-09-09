#pragma once

// Shared cross-board time base on mc02, built on the USB microframe counter.
//
// WHAT IT IS. Every board on one USB host controller sees the same SOF packets,
// so the frame counter reads the same number on all of them at the same instant
// -- a clock distributed by hardware, with no software path to add skew. This
// module turns that into a usable timeline: a 64-bit microframe counter, a
// sliding-window fit against a local high-resolution clock so any local instant
// can be named in microframes and any microframe in local ticks, and a state
// machine that refuses to answer at all when the counter cannot be trusted.
//
// WHY THE HOST TIMESTAMP CANNOT DEGRADE IT. The counter's low 14 bits come
// straight from the hardware frame counter -- exact, and identical on every
// board. The host's anchor message supplies only the WRAP: which multiple of
// 16384 microframes those bits belong to. So the anchor is quantized to 2.048 s
// and its error has to exceed +-1.024 s before it changes anything. Absolute
// correctness needs an accurate host; CROSS-BOARD agreement does not depend on
// the host clock in any way.
//
// This is a port of firmware/hpm_board/app/src/sync/timebase.{hpp,cpp}, minus
// everything that was specific to the HPM part (PTPC capture, the hardware
// SOF-to-PTPC trigger route, CAN TSU conversion, the GPTMR pulse layer). Three
// things had to change for this chip, and each is where a wrong assumption would
// hide, so each is stated here rather than left in the diff:
//
//  1. FULL SPEED ONLY, and the frame counter counts FRAMES. mc02 has no ULPI
//     PHY: USB_OTG_HS runs off the embedded full-speed PHY at 12 Mbit. DWC2's
//     DSTS.FNSOF holds a microframe number at high speed but a FRAME number at
//     full speed, so one SOF interrupt per 1 ms and one count per interrupt,
//     where hpm_board's EHCI FRINDEX counted microframes and stepped by 8.
//     Multiplying by 8 puts both boards on the same microframe axis with the
//     same 16384 modulus -- see frame_scale() in the .cpp.
//
//  2. THE LOCAL CLOCK IS THE CYCLE COUNTER, not the protocol's quarter-us tick.
//     mc02's timer::Timer is TIM5 prescaled to 1 MHz and reported as CNT << 2,
//     so its quarter-microsecond values move in steps of four: 1 us of
//     quantization, which is the same order as the interrupt jitter this module
//     exists to measure. Sampling with a ruler as coarse as the thing being
//     measured makes the result uninterpretable, so the fit runs on DWT->CYCCNT
//     (550 MHz, 1.8 ns) and converts to quarter-microseconds only where the
//     protocol demands it. The conversion is exact in the direction that
//     matters: 68750 cycles per microframe is exactly 500 quarter-us, so the
//     host's nominal-500 arithmetic needs no change.
//
//  3. HARDWARE SOF CAPTURE EXISTS ON THIS PART, but is deliberately not used
//     here. This module was first written on the false premise that it does not
//     exist; that was wrong and the measurement is recorded so nobody repeats it.
//     MEASURED 2026-09-07 by sweeping TIM2's SMCR.TS across ITR0..ITR13 with CH2
//     mapped to TRC: ITR5 captures at the SOF rate and every other source reads
//     exactly zero, and the interval between captures bottoms out at 999 TIM2
//     ticks (1 MHz) -- the 1 ms full-speed frame. So USB1_OTG_HS_SOF -> TIM2
//     ITR5 is real, the direct analogue of the HPM's TRGM route into PTPC. TIM5
//     has no such source: its ITR5 is USB2_OTG_FS, an instance the H723 does not
//     have, and a full sweep of TIM5 found nothing at the SOF rate.
//     (Note this is NOT reachable by grepping the HAL. Unlike STM32F4, which
//     names the route TIM_TIM2_USBFS_SOF, the H7 HAL only exposes TIM_TS_ITR0..13
//     and leaves the per-timer meaning to RM0468's internal-trigger table.)
//
//     IF YOU WIRE IT UP, SET TIM2's PRESCALER TO 0 FIRST. TIM2 is currently at
//     1 MHz for the 50 Hz servo PWM on PA0/PA2 (gpio.hpp), and a capture into a
//     1 MHz counter is WORSE than what this software path already achieves:
//
//       TIM2 @ 275 MHz capture   per-sample sigma ~0.001 us  (3.6 ns / sqrt(12))
//       this software ISR path   per-sample sigma  0.1365 us [measured 2026-09-07]
//       TIM2 @ 1 MHz capture     per-sample sigma  0.289 us  (1 us / sqrt(12))
//
//     A hardware capture is exact at the EDGE but is only ever as good as the
//     counter it is latched into, and latching an exact edge into a coarse
//     counter throws the exactness away. That is why the HPM's equivalent works:
//     it latches into PTPC, whose unit is 1.04 ns.
//
//     Note also that the interrupt path is far better than its worst sample
//     suggests: the worst is ~2.5 us but that is 19 sigma, a rare preemption,
//     and least squares over 128 samples buries it. The entry latency's MEAN is
//     not an error at all -- it is a constant the fit absorbs into the offset,
//     and it cancels outright between two boards running this same code.
//
//     Prescaler 0 stays compatible with the servo output because TIM2 is 32-bit:
//     50 Hz then needs ARR 5499999, which fits. It is a .ioc edit, so it is not
//     done here.
//
// WHAT INVALIDATES IT. Any microframe delta that is not the expected step. A
// delta of a few steps is a few missed interrupts: the counter can still be
// corrected (it advances by the real delta) but the fit window is discarded,
// because samples either side of the gap would tilt the line. A delta of 0, or
// eight steps and more unaccounted for, means the counter itself is in doubt --
// the timeline goes invalid and nothing may be scheduled against it until a
// fresh anchor arrives.
//
// Compiled out unless libhcs_APP_TIME_SYNC. Enabling it adds a 1 kHz interrupt
// to the board.

#include <cstdint>

#include "core/include/libhcs/data/datas.hpp"

namespace libhcs::firmware::sync::timebase {

// CPU cycles per microframe: 550 MHz SYSCLK, 125 us microframe. SYSCLK is
// PLL1P = 24 MHz / 3 * (68 + 6144/8192) = 550 MHz exactly (main.c
// SystemClock_Config); DWT->CYCCNT counts CPU cycles, so this is exact and not
// a rounding.
inline constexpr std::uint32_t kCyclesPerMicrosecond = 550U;
inline constexpr std::uint32_t kNominalCyclesPerMicroframe = kCyclesPerMicrosecond * 125U;

// 68750 cycles is exactly 500 quarter-microseconds, so the protocol's nominal
// value is unchanged from the 4 MHz boards and the host needs no board-specific
// scaling. Asserted rather than trusted.
static_assert(kNominalCyclesPerMicroframe * 4U % kCyclesPerMicrosecond == 0U);
static_assert(kNominalCyclesPerMicroframe * 4U / kCyclesPerMicrosecond == 500U);

#if defined(libhcs_APP_TIME_SYNC) && libhcs_APP_TIME_SYNC

inline constexpr bool kEnabled = true;

// One fit sample every 64 microframes over a 128-sample ring. At full speed the
// counter advances 8 microframes per SOF, so that is one sample every 8th
// interrupt: 8 ms apart, a 1.024 s baseline, 512 bytes of RAM -- the same
// spacing and the same window hpm_board uses at high speed, which is what makes
// the two boards' residual numbers directly comparable.
//
// The baseline sets the slope accuracy (endpoint noise divided by the window)
// and the sample count averages the phase noise down. The window must also stay
// clear of the cycle counter's 7.81 s wrap at 550 MHz, which 1.024 s does with
// 7x to spare.
inline constexpr std::uint32_t kSampleDecimation = 64U;
inline constexpr std::uint32_t kSampleCount = 128U;

struct Snapshot {
    data::TimeState state;
    // Absolute microframe when kValid; the board's own origin otherwise.
    std::uint64_t microframe;
    std::uint64_t timestamp_quarter_us;
    // Fitted local ticks per microframe, Q16, in QUARTER-MICROSECONDS -- the
    // protocol's unit, converted from cycles on the way out. Zero until the fit
    // converges.
    std::uint32_t ticks_per_microframe_q16;
    std::uint32_t anomaly_count;
    // Out-of-sample prediction error of this board's own fit since the last
    // report(), in Q16 quarter-microseconds. The mean is the term that becomes
    // cross-board skew; see data::TimeStatusView for why the extremum is not.
    std::int32_t residual_mean_q16;
    std::uint32_t residual_abs_max_q16;
    std::uint32_t residual_count;
};

// Microframes the counter advances per SOF interrupt at the CURRENT port speed:
// 8 at full speed, 1 at high speed. Published because every consumer of the
// microframe stream needs the step, not just this module.
std::uint32_t microframes_per_sof();

// Transmission time of the SOF packet itself at the CURRENT port speed, in
// nanoseconds. The device raises the SOF-received flag when the packet has been
// RECEIVED, so every timestamp taken in that interrupt is late by this much --
// 0.13 us at 480 Mbit, 2.92 us at 12 Mbit. Identical across boards of the same
// speed, so it cancels between two mc02 boards and appears as a constant ~2.8 us
// offset against a high-speed board.
// [Measured 2026-08-20 on an HS+FS pair of HPM boards: -2854 / -2764 / -2810 ns
//  against a computed 2.78 us -- agreement within 1%.]
std::uint32_t sof_packet_delay_ns();

// ISR path, called from sync::sof_isr_entry() with the values read there.
// `frame` is DSTS.FNSOF as read; `now_cycles` is DWT->CYCCNT.
void note_sof(std::uint32_t frame, std::uint32_t now_cycles);

// Main loop: recomputes the fit. Cheap enough to call every pass; it does real
// work only every kFitPeriodMs.
void poll(std::uint32_t tick_ms);

// Applies a host anchor. Idempotent while the resolved wrap is unchanged; a
// CHANGED wrap on an already-valid timeline is treated as a fault, not silently
// accepted, because it can only mean the counter or the host estimate moved by
// more than a second.
void apply_anchor(std::uint64_t host_microframe);

Snapshot snapshot();

// Same, but consumes the residual accumulators. One caller only -- whatever
// ships the periodic status -- so the window each mean covers is well defined.
Snapshot report();

// Timeline queries. The local unit is TIM5 QUARTER-MICROSECONDS -- what
// timer::Timer::timepoint() returns, and what every IMU / timestamped-GPIO
// record on this board already carries -- so microframe_at() converts an
// existing telemetry timestamp straight onto the shared axis. Both return false
// when the timeline is not valid, so a caller cannot accidentally schedule
// against a dead clock.
bool local_time_of(std::uint64_t microframe, std::uint32_t& out_quarter_us);
bool microframe_at(std::uint32_t quarter_us, std::uint64_t& out_microframe);

#else

inline constexpr bool kEnabled = false;

// Must mirror the enabled Snapshot field for field: callers outside this header
// read these members unconditionally, so a stub that lags the real struct breaks
// the TIME_SYNC=OFF build only -- the configuration least likely to be compiled
// while the time base is being worked on.
struct Snapshot {
    data::TimeState state;
    std::uint64_t microframe;
    std::uint64_t timestamp_quarter_us;
    std::uint32_t ticks_per_microframe_q16;
    std::uint32_t anomaly_count;
    std::int32_t residual_mean_q16;
    std::uint32_t residual_abs_max_q16;
    std::uint32_t residual_count;
};

inline void note_sof(std::uint32_t, std::uint32_t) {}
inline void poll(std::uint32_t) {}
inline std::uint32_t microframes_per_sof() { return 8; }
inline std::uint32_t sof_packet_delay_ns() { return 0; }
inline void apply_anchor(std::uint64_t) {}
inline Snapshot snapshot() { return {data::TimeState::kInvalid, 0, 0, 0, 0, 0, 0, 0}; }
inline Snapshot report() { return snapshot(); }
inline bool local_time_of(std::uint64_t, std::uint32_t&) { return false; }
inline bool microframe_at(std::uint32_t, std::uint64_t&) { return false; }

#endif

} // namespace libhcs::firmware::sync::timebase
