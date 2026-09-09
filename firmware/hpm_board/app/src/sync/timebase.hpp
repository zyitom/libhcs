#pragma once

// Shared cross-board time base, built on the USB microframe counter.
//
// WHAT IT IS. Every board on one USB host controller sees the same SOF packets,
// so FRINDEX is the same number on all of them at the same instant -- a clock
// distributed by hardware, with no software path to add skew. This module turns
// that into a usable timeline: a 64-bit microframe counter, a sliding-window fit
// against the local machine timer so any local instant can be named in
// microframes and any microframe can be named in local ticks, and a state
// machine that refuses to answer at all when the counter cannot be trusted.
//
// WHY THE HOST TIMESTAMP CANNOT DEGRADE IT. The counter's low 14 bits are
// FRINDEX itself -- hardware-exact and identical on every board. The host's
// anchor message supplies only the WRAP: which multiple of 16384 those bits
// belong to. So the anchor is quantized to 2.048 s and its error has to exceed
// +-1.024 s before it changes anything at all. Two boards given the same anchor
// therefore compute a bit-identical absolute microframe even if that anchor is
// wildly wrong -- absolute correctness needs an accurate host, but CROSS-BOARD
// agreement does not depend on the host clock in any way. That is the whole
// point of the split, and it is why the anchor must be computed ONCE per round
// on the host and sent unchanged to every board.
//
// WHAT INVALIDATES IT. Any microframe delta that is not 1. A delta of 2..7 is a
// few missed interrupts: the counter can still be corrected (it advances by the
// real delta) but the fit window is discarded, because those samples would drag
// the line. A delta of 0, or 8 and above, means the counter itself is in doubt
// -- the timeline goes invalid and nothing may be scheduled against it until a
// fresh anchor arrives. On the validated hardware neither branch fires in steady
// state (SOF_TIMEBASE.md), but they are the safety net, not dead code to delete.
//
// Compiled out unless libhcs_APP_TIME_SYNC. Enabling it adds an 8 kHz
// interrupt to the board, which is why it is not on by default.

#include <cstdint>

#include "core/include/libhcs/data/datas.hpp"

namespace libhcs::firmware::sync::timebase {

#if defined(libhcs_APP_TIME_SYNC) && libhcs_APP_TIME_SYNC

inline constexpr bool kEnabled = true;

// One fit sample every 64 microframes over a 128-sample ring: an 8192-microframe
// (1.024 s) baseline for 512 bytes of RAM. The baseline is what sets the slope
// accuracy (endpoint noise divided by the window), and the sample count is what
// averages the phase noise down -- 128 samples take the ~2 us of interrupt
// jitter to under 0.2 us.
inline constexpr std::uint32_t kSampleDecimation = 64U;
inline constexpr std::uint32_t kSampleCount = 128U;

// Nominal local ticks per microframe: 4 MHz machine timer, 125 us microframe.
inline constexpr std::uint32_t kNominalTicksPerMicroframe = 500U;

struct Snapshot {
    data::TimeState state;
    // Absolute microframe when kValid; the board's own origin otherwise.
    std::uint64_t microframe;
    std::uint64_t timestamp_quarter_us;
    // Fitted local ticks per microframe, Q16. Zero until the fit converges.
    std::uint32_t ticks_per_microframe_q16;
    std::uint32_t anomaly_count;
    // Out-of-sample prediction error of this board's own fit since the last
    // report(), in Q16 timer ticks. The mean is the term that becomes
    // cross-board skew; see data::TimeStatusView for why the extremum is not.
    std::int32_t residual_mean_q16;
    std::uint32_t residual_abs_max_q16;
    std::uint32_t residual_count;
    // The same statistic for the PTPC fit, in PTPC units (1 unit is ~1.04 ns).
    std::int32_t ptpc_residual_mean;
    std::uint32_t ptpc_residual_abs_max;
    std::uint32_t ptpc_step_min;
    std::uint32_t ptpc_step_max;
    std::uint32_t ptpc_raw_ns;
    std::uint64_t ptpc_raw_microframe;
};

// Routes USB0's Start-of-Frame to PTPC0's input capture through the trigger
// matrix, so every microframe hardware-latches the PTPC counter.
//
// This is the whole point of the module's second revision. Sampling PTPC by
// reading its counter inside the SOF interrupt put the interrupt's entry jitter
// -- microseconds -- into every sample, and no amount of fitting recovers what
// the sampling threw away. TRGM connects the two in silicon: HPM_TRGM0 lists
// USB0_SOF as an input source and PTPC0's capture as an output destination, so
// the latch happens at the edge with no software involved at all. It is the
// same trick EtherCAT's distributed clocks use -- capture the reference event in
// hardware, never in an interrupt.
//
// Must run AFTER the CAN driver, which is what brings PTPC0 up.
void init_capture();

// Microframes per SOF interrupt at the CURRENT port speed: 1 at high speed,
// 8 at full speed (FRINDEX counts microframes either way, but a full-speed port
// only receives one SOF per 1 ms frame). Published because every consumer of
// the microframe stream needs the step, not just this module.
std::uint32_t microframes_per_sof();

// Transmission time of the SOF packet itself at the CURRENT port speed, in
// nanoseconds. The device raises the SOF-received flag when the packet has been
// RECEIVED, so every timestamp taken in that interrupt is late by this much --
// 0.13 us at 480 Mbit, 2.92 us at 12 Mbit. Identical across boards of the same
// speed, so it cancels in an all-high-speed rig and appears as a constant
// ~2.8 us offset the moment one board is full speed.
// [Measured 2026-08-20 on an HS+FS pair: -2854 / -2764 / -2810 ns against a
//  computed 2.78 us -- agreement within 1%.]
std::uint32_t sof_packet_delay_ns();

// ISR path, called from sync::sof_isr_entry() with the values read there.
void note_sof(std::uint32_t frindex, std::uint32_t now_quarter_us);

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

// Timeline queries. Both return false when the timeline is not valid, so a
// caller cannot accidentally schedule against a dead clock.
bool local_time_of(std::uint64_t microframe, std::uint64_t& out_quarter_us);
bool microframe_at(std::uint64_t quarter_us, std::uint64_t& out_microframe);

// PTPC0, the timebase the MCAN Timestamp Units are fed from, expressed as a
// single monotonic counter of "reported nanoseconds" (sec * 1e9 + ns). One unit
// is 1/960 of a real microsecond on this board -- PTPC's nanoseconds run slow by
// exactly the digital-step error that board::kCanTimestampNsPerUs encodes.
//
// Why the time base cares about a CAN peripheral's clock: the TSU captures a
// frame's Start-of-Frame in HARDWARE, so a CAN timestamp is the one event on
// this board that is free of interrupt jitter. Put two boards' TSU captures of
// the same physical frame on the shared microframe axis and the difference is
// the cross-board skew, measured rather than derived.
// Nominal PTPC units per microframe: 125 us x 960 units/us.
inline constexpr std::uint64_t kNominalPtpcUnitsPerMicroframe = 120'000;

// Converts a TSU capture to the shared axis, in Q16 microframes (1 LSB is
// 1.9 ns). False when the timeline is not valid or PTPC has not been related to
// the machine timer yet.
bool microframe_q16_at_ptpc(std::uint32_t ptpc_ns, std::uint64_t& out_microframe_q16);

// One hardware Start-of-Frame capture waiting to be reported.
struct CanCapture {
    std::uint32_t tag;
    std::uint64_t microframe_q16;
    std::uint8_t bus;
    // The raw hardware capture, forwarded so the host can convert it itself.
    std::uint32_t ptpc_ns;
};

// ISR path: the CAN receive handler calls this for probe frames only, and it
// does nothing but push the raw capture. The conversion to the shared axis
// involves two 64-bit divisions and belongs nowhere near the highest-priority
// interrupt on this board.
void note_can_capture(std::uint32_t tag, std::uint32_t ptpc_ns, std::uint8_t bus);

// Main loop: converts and pops one pending capture. False when none is ready or
// the timeline cannot place it.
bool take_can_capture(CanCapture& out);

// Fitted PTPC units per microframe. Published so the fit can be sanity-checked
// against kNominalPtpcUnitsPerMicroframe on real hardware rather than trusted.
std::uint32_t ptpc_units_per_microframe();

// The fitted PTPC line's anchor point, for host-side verification.
void ptpc_reference(std::uint64_t& units, std::uint64_t& microframe);

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
    std::int32_t ptpc_residual_mean;
    std::uint32_t ptpc_residual_abs_max;
    std::uint32_t ptpc_step_min;
    std::uint32_t ptpc_step_max;
    std::uint32_t ptpc_raw_ns;
    std::uint64_t ptpc_raw_microframe;
};

inline void note_sof(std::uint32_t, std::uint32_t) {}
inline void poll(std::uint32_t) {}
inline std::uint32_t microframes_per_sof() { return 1; }
inline std::uint32_t sof_packet_delay_ns() { return 0; }
inline void apply_anchor(std::uint64_t) {}
inline Snapshot snapshot() { return {data::TimeState::kInvalid, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; }
inline Snapshot report() { return snapshot(); }
inline bool local_time_of(std::uint64_t, std::uint64_t&) { return false; }
inline bool microframe_at(std::uint64_t, std::uint64_t&) { return false; }
struct CanCapture {
    std::uint32_t tag;
    std::uint64_t microframe_q16;
    std::uint8_t bus;
    std::uint32_t ptpc_ns;
};

inline void init_capture() {}
inline void note_can_capture(std::uint32_t, std::uint32_t, std::uint8_t) {}
inline bool take_can_capture(CanCapture&) { return false; }
inline bool microframe_q16_at_ptpc(std::uint32_t, std::uint64_t&) { return false; }
inline std::uint32_t ptpc_units_per_microframe() { return 0; }
inline void ptpc_reference(std::uint64_t& units, std::uint64_t& microframe) {
    units = 0;
    microframe = 0;
}

#endif

} // namespace libhcs::firmware::sync::timebase
