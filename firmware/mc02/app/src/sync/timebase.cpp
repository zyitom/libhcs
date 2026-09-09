#include "firmware/mc02/app/src/sync/timebase.hpp"

#if defined(libhcs_APP_TIME_SYNC) && libhcs_APP_TIME_SYNC

# include <algorithm>

# include <main.h>

# include "firmware/mc02/app/src/timer/timer.hpp"
# include "firmware/mc02/app/src/utility/interrupt_lock.hpp"

namespace libhcs::firmware::sync::timebase {
namespace {

// The microframe axis is 14 bits wide on every board in this repo, whatever the
// hardware counter underneath looks like. An EHCI FRINDEX is 14 bits of
// microframes; DWC2 at full speed gives 11 bits of frames, which times eight is
// the same 16384-microframe (2.048 s) modulus. Keeping them identical is what
// lets one host anchor serve every board unchanged.
constexpr std::uint32_t kMicroframeMask = 0x3FFFU;
constexpr std::uint64_t kMicroframeModulus = 0x4000U;

// DSTS.ENUMSPD, DWC2 encoding: 0 high speed, 1 full speed on a 30/60 MHz PHY,
// 2 low speed, 3 full speed on the 48 MHz internal PHY. mc02 always enumerates
// as 3; the high-speed branch exists so the module is not silently wrong if this
// code is ever reused on a part with a ULPI PHY.
constexpr std::uint32_t kEnumSpeedHigh = 0U;

constexpr std::uint32_t kFitPeriodMs = 20U;

// Acceptance window for the fitted slope. A value more than a few percent off
// nominal is not a crystal offset, it is a broken window; refusing it keeps a
// bad fit from being published at all.
constexpr std::int64_t kNominalQ16 = static_cast<std::int64_t>(kNominalCyclesPerMicroframe) << 16U;

// Sum of (x - mean)^2 for x = 0..N-1, which for evenly spaced samples is a
// constant the fit never has to compute: N(N^2-1)/12.
constexpr std::int64_t kSxx = static_cast<std::int64_t>(kSampleCount)
                            * ((static_cast<std::int64_t>(kSampleCount) * kSampleCount) - 1) / 12;

// ------------------------------------------------------------------
// ISR-owned state. Written only by note_sof(); read by the main loop under
// utility::InterruptLockGuard. Plain scalars rather than atomics because every
// reader takes that guard -- and unlike a counter, these have to be read as a
// consistent SET, which no per-variable atomic would give.
// ------------------------------------------------------------------

// Local, boot-relative microframe count. Seeded from the first frame reading so
// that (counter mod 16384) equals the hardware counter's contribution forever
// after, which is what lets the anchor be a pure multiple of 16384.
std::uint64_t counter = 0;
bool counter_seeded = false;
std::uint32_t previous_index = 0;

// 64-bit extension of DWT->CYCCNT. The counter wraps every 7.81 s at 550 MHz;
// SOF arrives every 1 ms, so "the low word went backwards" is an unambiguous
// wrap detector here with three and a half orders of margin.
std::uint32_t previous_time = 0;
std::uint32_t time_high = 0;
bool time_seeded = false;

// TIM5, sampled in the same interrupt, purely so the status report can pair a
// microframe with a reading of the clock the REST of this firmware timestamps
// with.
//
// The fit runs on CYCCNT because TIM5 quantizes to 1 us (timer.hpp: 1 MHz, read
// as CNT << 2), which is the same order as the jitter being measured. But every
// other uplink record -- IMU, timestamped GPIO -- carries a TIM5 quarter-us
// stamp, and CYCCNT and TIM5 have unrelated origins that change every boot. If
// the status published a CYCCNT-derived time instead, the host would hold two
// clocks it could never relate, and no telemetry record could be placed on the
// microframe axis at all. Publishing TIM5 is what makes the axis usable for
// anything other than the time base itself.
//
// Sampled AFTER the cycle counter, so the extra peripheral read (about 0.25 us
// on this D2 bus) cannot lengthen the timestamp path. 1 kHz.
std::uint32_t previous_timer_quarter_us = 0;

data::TimeState state = data::TimeState::kInvalid;
std::uint32_t anomaly_count = 0;
std::int64_t anchor_offset = 0;
bool anchored = false;

// Fit ring. sample[i] is the low 32 bits of the cycle counter at microframe
// (ring_oldest_microframe + i * kSampleDecimation), in insertion order starting
// at ring_head.
std::uint32_t sample[kSampleCount];
std::uint32_t ring_head = 0;
std::uint32_t ring_count = 0;
std::uint64_t ring_oldest_microframe = 0;

// ------------------------------------------------------------------
// Fit result. Written only by poll(), read by everything; guarded the same way.
// ------------------------------------------------------------------
// The reference is a whole cycle (1.8 ns) rather than Q16: the fit averages 128
// samples precisely so the PHASE is better than one sample's jitter, and 1.8 ns
// of rounding is three orders below the jitter it is averaging. The SLOPE stays
// Q16, because there a part per million compounds over the extrapolation.
//
// Q16 cycles per microframe is 68750 * 65536 = 4.5e9, which does NOT fit in the
// uint32 hpm_board uses for its 4 MHz timer. It is held in 64 bits here and
// narrowed only on the way out, after conversion to quarter-microseconds brings
// it back to 500 * 65536.
bool fit_valid = false;
std::uint64_t fit_reference_microframe = 0;
std::uint64_t fit_reference_time = 0;
std::uint64_t fit_cycles_per_microframe_q16 = 0;

// Out-of-sample prediction error, accumulated between reports, in Q16 cycles.
//
// Every decimated sample is first PREDICTED from the fit currently published --
// a fit computed before this sample existed -- and only then added to the
// window. So the residual is a genuine forecast error, which is exactly what a
// scheduled action would experience, rather than the in-sample residual of a
// line fitted through the point being tested.
std::int64_t residual_sum_q16 = 0;
std::uint32_t residual_count = 0;
std::uint64_t residual_abs_max_q16 = 0;

std::uint32_t last_fit_tick = 0;

void reset_fit_window() {
    ring_head = 0;
    ring_count = 0;
    fit_valid = false;
    fit_cycles_per_microframe_q16 = 0;
    residual_sum_q16 = 0;
    residual_count = 0;
    residual_abs_max_q16 = 0;
}

void invalidate() {
    state = data::TimeState::kInvalid;
    anchored = false;
    counter_seeded = false;
    reset_fit_window();
}

std::uint64_t now_extended(std::uint32_t low) {
    return (static_cast<std::uint64_t>(time_high) << 32U) | low;
}

USB_OTG_DeviceTypeDef* device_registers() {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<USB_OTG_DeviceTypeDef*>(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
}

std::uint32_t enumerated_speed() {
    const auto status = static_cast<std::uint32_t>(device_registers()->DSTS);
    return (status & static_cast<std::uint32_t>(USB_OTG_DSTS_ENUMSPD))
        >> static_cast<std::uint32_t>(USB_OTG_DSTS_ENUMSPD_Pos);
}

// Microframes one count of DSTS.FNSOF is worth at the current port speed.
//
// This is the whole full-speed difference in one function. DWC2 documents FNSOF
// as "frame or microframe number of the received SOF": a microframe number when
// the core enumerated at high speed, a FRAME number otherwise. mc02 is always
// the latter, so each count is eight microframes and the register only ever
// carries the 11-bit frame number the host put on the wire.
//
// Masking to 11 bits at full speed rather than to the register's full 14 is not
// a conservative guess, it is the measured width. FNSOF is a 14-bit FIELD, but
// at full speed only the 11 bits the host puts on the wire are populated:
// probing the raw register for 30 s (about 14 full wraps) gave a maximum of
// exactly 2047, so bits 13..11 are always zero and there is no wider counter to
// exploit. [Measured 2026-09-07.] hpm_board gets 14 real bits only because at
// HIGH speed the field is 11 frame bits plus 3 microframe bits.
//
// So the wrap the host anchor has to resolve is 2048 frames = 16384 microframes
// = 2.048 s on both boards, and it could not have been made longer here.
std::uint32_t frame_scale() { return enumerated_speed() == kEnumSpeedHigh ? 1U : 8U; }

// Integer division that rounds toward negative infinity. C++ truncates toward
// zero, which would make the wrap resolution below asymmetric about zero and let
// two boards straddling the origin pick different wraps.
std::int64_t floor_div(std::int64_t numerator, std::int64_t denominator) {
    const std::int64_t quotient = numerator / denominator;
    const std::int64_t remainder = numerator % denominator;
    return (remainder != 0 && ((remainder < 0) != (denominator < 0))) ? quotient - 1 : quotient;
}

// Cycles to quarter-microseconds. Exact for whole microframes (68750 -> 500) and
// truncating otherwise; used only where the protocol's unit is required.
std::int64_t cycles_to_quarter_us(std::int64_t cycles) {
    return cycles * 4 / static_cast<std::int64_t>(kCyclesPerMicrosecond);
}

std::int64_t quarter_us_to_cycles(std::int64_t quarter_us) {
    return quarter_us * static_cast<std::int64_t>(kCyclesPerMicrosecond) / 4;
}

} // namespace

std::uint32_t microframes_per_sof() { return frame_scale(); }

std::uint32_t sof_packet_delay_ns() {
    // 64 bit times at 480 Mbit, 35 bit times at 12 Mbit.
    return enumerated_speed() == kEnumSpeedHigh ? 133U : 2917U;
}

void note_sof(std::uint32_t frame, std::uint32_t now_cycles) {
    // Cycle-counter wrap extension, before anything that uses the timestamp.
    if (!time_seeded) {
        time_seeded = true;
    } else if (now_cycles < previous_time) {
        time_high++;
    }
    previous_time = now_cycles;
    previous_timer_quarter_us = timer::timer->timepoint().time_since_epoch().count();

    // How much the microframe axis is expected to move per SOF interrupt, and
    // the mask that keeps the hardware counter's own wrap from being read as a
    // jump. One SOF per 1 ms frame at full speed, so the axis steps by 8 --
    // exactly, every time, with the low three bits pinned at zero. Treating that
    // as an anomaly is what kept a full-speed HPM board permanently kInvalid.
    const std::uint32_t scale = frame_scale();
    const std::uint32_t index =
        (frame & ((static_cast<std::uint32_t>(kMicroframeModulus) / scale) - 1U)) * scale;

    if (!counter_seeded) {
        counter_seeded = true;
        previous_index = index;
        // Seeding WITH the frame index, not with zero: the low 14 bits of the
        // counter must equal the hardware reading for the anchor arithmetic to
        // reduce to a multiple of 16384.
        counter = index;
        state = data::TimeState::kWaitingAnchor;
        reset_fit_window();
        ring_oldest_microframe = counter;
        return;
    }

    const std::uint32_t delta = (index - previous_index) & kMicroframeMask;
    previous_index = index;
    counter += delta;

    if (delta != scale) [[unlikely]] {
        anomaly_count++;
        if (delta > scale && delta < scale * 8U) {
            // A few missed interrupts. The counter is still right -- it advanced
            // by the real delta -- so the timeline survives; only the fit window
            // is spoiled, because the samples either side of the gap would tilt
            // the line.
            reset_fit_window();
            ring_oldest_microframe = counter;
        } else {
            // Delta 0 (the bus is not running: this is what the enumeration
            // window looks like) or eight frames and more unaccounted for.
            // Either way the counter is no longer trustworthy.
            invalidate();
        }
        return;
    }

    if (state == data::TimeState::kInvalid) {
        state = data::TimeState::kWaitingAnchor;
        reset_fit_window();
        ring_oldest_microframe = counter;
    }

    if (counter % kSampleDecimation != 0U)
        return;

    if (fit_valid) {
        // Both sides taken RELATIVE to the fit reference before the Q16 shift.
        // Shifting the absolute cycle count would overflow int64 after about six
        // days of uptime -- a 550 MHz counter leaves 48 bits of headroom where
        // hpm_board's 4 MHz timer leaves 61. Differences are bounded by the fit
        // window, so this form has no such horizon.
        const auto distance = static_cast<std::int64_t>(counter - fit_reference_microframe);
        const std::int64_t predicted_q16 =
            distance * static_cast<std::int64_t>(fit_cycles_per_microframe_q16);
        const std::int64_t actual_q16 =
            static_cast<std::int64_t>(now_extended(now_cycles) - fit_reference_time) << 16U;
        const std::int64_t residual_q16 = predicted_q16 - actual_q16;

        residual_sum_q16 += residual_q16;
        residual_count++;
        const auto magnitude =
            static_cast<std::uint64_t>(residual_q16 < 0 ? -residual_q16 : residual_q16);
        residual_abs_max_q16 = std::max(residual_abs_max_q16, magnitude);
    }

    if (ring_count < kSampleCount) {
        if (ring_count == 0U)
            ring_oldest_microframe = counter;
        sample[(ring_head + ring_count) % kSampleCount] = now_cycles;
        ring_count++;
        return;
    }
    sample[ring_head] = now_cycles;
    ring_head = (ring_head + 1U) % kSampleCount;
    ring_oldest_microframe += kSampleDecimation;
}

void poll(std::uint32_t tick_ms) {
    if (tick_ms - last_fit_tick < kFitPeriodMs)
        return;
    last_fit_tick = tick_ms;

    std::uint32_t local_sample[kSampleCount];
    std::uint32_t count = 0;
    std::uint64_t oldest_microframe = 0;
    std::uint64_t now = 0;

    {
        // Bounded and short: 128 word copies, well under a microsecond at
        // 550 MHz, once per 20 ms. Masking rather than a lock-free snapshot
        // because the fit needs the ring and its origin to be mutually
        // consistent, and the established pattern in this firmware for that is
        // the interrupt guard.
        const utility::InterruptLockGuard guard;
        count = ring_count;
        oldest_microframe = ring_oldest_microframe;
        now = now_extended(previous_time);
        for (std::uint32_t index = 0; index < count; index++)
            local_sample[index] = sample[(ring_head + index) % kSampleCount];
    }

    if (count < kSampleCount)
        return;

    // Least squares against evenly spaced x = 0..N-1, with y taken relative to
    // the first sample so the arithmetic stays inside 32 bits before widening.
    // Sxy is accumulated in the doubled form sum((2x - (N-1)) * y) to keep the
    // half-integer mean out of integer arithmetic.
    //
    // Widths, since the cycle counter makes these an order larger than
    // hpm_board's: y spans one window, 1.024 s * 550 MHz = 5.6e8, inside
    // int32; sum_xy2 tops out near 9e12 and its Q16 shift near 6e17, both inside
    // int64 with a decade to spare.
    const std::uint32_t base = local_sample[0];
    std::int64_t sum_y = 0;
    std::int64_t sum_xy2 = 0;
    for (std::uint32_t index = 0; index < count; index++) {
        const auto y =
            static_cast<std::int64_t>(static_cast<std::int32_t>(local_sample[index] - base));
        sum_y += y;
        sum_xy2 += ((2LL * index) - static_cast<std::int64_t>(count - 1U)) * y;
    }

    // Cycles per decimated sample, then per microframe, both Q16.
    const std::int64_t slope_q16 = (sum_xy2 << 16U) / (2 * kSxx);
    const std::int64_t per_microframe_q16 = slope_q16 / kSampleDecimation;

    // A slope more than a few percent off nominal is not a crystal offset, it is
    // a broken window; refusing it keeps a bad fit from being published at all.
    if (per_microframe_q16 < kNominalQ16 - (kNominalQ16 / 32)
        || per_microframe_q16 > kNominalQ16 + (kNominalQ16 / 32))
        return;

    // The ring holds only the low 32 bits of the cycle counter. Rebuild the
    // oldest sample's full value by hanging it off the current time, which is at
    // most one window (1.024 s) later and therefore at most one wrap away --
    // the 7.81 s wrap period is what makes that claim true.
    std::uint64_t base_extended = (now & ~static_cast<std::uint64_t>(0xFFFFFFFFU)) | base;
    if (base_extended > now)
        base_extended -= static_cast<std::uint64_t>(1) << 32U;

    // Evaluate the fitted line at the NEWEST sample rather than at the centroid:
    // every query extrapolates forward from now, so anchoring the reference at
    // the leading edge is what keeps the extrapolation arm short.
    const auto newest_index = static_cast<std::int64_t>(count - 1U);
    const std::int64_t mean_y_q16 = (sum_y << 16U) / count;
    const std::int64_t offset_q16 = mean_y_q16 + ((slope_q16 * newest_index) / 2);

    const utility::InterruptLockGuard guard;
    fit_reference_microframe =
        oldest_microframe + (static_cast<std::uint64_t>(newest_index) * kSampleDecimation);
    fit_reference_time = base_extended + static_cast<std::uint64_t>((offset_q16 + 32768) >> 16U);
    fit_cycles_per_microframe_q16 = static_cast<std::uint64_t>(per_microframe_q16);
    fit_valid = true;
    if (state == data::TimeState::kWaitingAnchor && anchored)
        state = data::TimeState::kValid;
}

void apply_anchor(std::uint64_t host_microframe) {
    const utility::InterruptLockGuard guard;

    if (state == data::TimeState::kInvalid && !counter_seeded)
        return;

    // Resolve the wrap: pick the multiple of 16384 that puts the counter closest
    // to the host's estimate. Rounding rather than truncating is what makes the
    // decision insensitive to which side of the estimate the counter sits on.
    const auto difference = static_cast<std::int64_t>(host_microframe - counter);
    const std::int64_t wraps = floor_div(
        difference + static_cast<std::int64_t>(kMicroframeModulus / 2),
        static_cast<std::int64_t>(kMicroframeModulus));
    const std::int64_t offset = wraps * static_cast<std::int64_t>(kMicroframeModulus);

    if (!anchored) {
        anchor_offset = offset;
        anchored = true;
    } else if (offset != anchor_offset) {
        // A changed wrap on a live timeline cannot be accepted quietly. Either
        // the counter lost more than a second or the host's estimate did, and
        // both mean everything scheduled since the last anchor was scheduled
        // against the wrong second.
        anomaly_count++;
        invalidate();
        return;
    }

    if (fit_valid)
        state = data::TimeState::kValid;
    else if (state == data::TimeState::kInvalid)
        state = data::TimeState::kWaitingAnchor;
}

namespace {

Snapshot snapshot_locked() {
    const std::int64_t mean_cycles_q16 =
        residual_count == 0 ? 0 : residual_sum_q16 / static_cast<std::int64_t>(residual_count);

    return {
        .state = state,
        .microframe = anchored ? static_cast<std::uint64_t>(
                                     static_cast<std::int64_t>(counter) + anchor_offset)
                               : counter,
        // TIM5 at the last SOF, NOT the cycle counter: this is the pair the host
        // needs to convert an IMU or GPIO record's own quarter-us stamp onto the
        // microframe axis, and from there onto its own clock.
        .timestamp_quarter_us = previous_timer_quarter_us,
        // Converted to the protocol's quarter-microsecond tick on the way out,
        // so 68750 cycles per microframe is published as the same 500 * 65536
        // every other board reports and the host needs no per-board scaling.
        .ticks_per_microframe_q16 = static_cast<std::uint32_t>(
            cycles_to_quarter_us(static_cast<std::int64_t>(fit_cycles_per_microframe_q16))),
        .anomaly_count = anomaly_count,
        .residual_mean_q16 = static_cast<std::int32_t>(cycles_to_quarter_us(mean_cycles_q16)),
        .residual_abs_max_q16 = static_cast<std::uint32_t>(
            cycles_to_quarter_us(static_cast<std::int64_t>(residual_abs_max_q16))),
        // Clamped, not truncated: the protocol field is 16 bits, and a silent
        // wrap would turn "the host stopped anchoring for a minute" into a
        // plausible-looking small count.
        .residual_count = residual_count > 0xFFFFU ? 0xFFFFU : residual_count,
    };
}

} // namespace

Snapshot snapshot() {
    const utility::InterruptLockGuard guard;
    return snapshot_locked();
}

Snapshot report() {
    const utility::InterruptLockGuard guard;
    const Snapshot result = snapshot_locked();
    // Cleared here rather than left free-running: the mean is only meaningful
    // over a bounded window, and a sum that spans a re-anchor would blend two
    // different fits.
    residual_sum_q16 = 0;
    residual_count = 0;
    residual_abs_max_q16 = 0;
    return result;
}

// Both directions speak TIM5 QUARTER-MICROSECONDS -- the unit every other uplink
// record on this board is stamped in -- while the fit itself stays on the cycle
// counter. The bridge between them is the pair captured in the SOF interrupt:
// (previous_time cycles, previous_timer_quarter_us). Both counters ride the same
// 550 MHz PLL and TIM5 is exactly SYSCLK/550, so the only error in crossing is
// TIM5's own 1 us quantization -- which is also the resolution of the timestamps
// being converted, so nothing is lost.
namespace {

// Cycle-counter value corresponding to a TIM5 reading, using the last SOF as the
// anchor. The subtraction is deliberately done in int32 so it is correct across
// TIM5's wrap; a telemetry record is converted within seconds of being taken.
std::int64_t cycles_at_quarter_us(std::uint32_t quarter_us) {
    const auto delta_quarter_us = static_cast<std::int32_t>(quarter_us - previous_timer_quarter_us);
    return static_cast<std::int64_t>(now_extended(previous_time))
         + quarter_us_to_cycles(delta_quarter_us);
}

std::uint32_t quarter_us_at_cycles(std::int64_t cycles) {
    const std::int64_t delta = cycles - static_cast<std::int64_t>(now_extended(previous_time));
    // Wraps with the counter, which is what the caller wants: TIM5 quarter-us is
    // a free-running 32-bit value and every consumer takes differences.
    return previous_timer_quarter_us + static_cast<std::uint32_t>(cycles_to_quarter_us(delta));
}

} // namespace

bool local_time_of(std::uint64_t microframe, std::uint32_t& out_quarter_us) {
    const utility::InterruptLockGuard guard;
    if (state != data::TimeState::kValid || !fit_valid)
        return false;

    const auto local_microframe =
        static_cast<std::uint64_t>(static_cast<std::int64_t>(microframe) - anchor_offset);
    const auto distance = static_cast<std::int64_t>(local_microframe - fit_reference_microframe);
    const std::int64_t cycles =
        ((distance * static_cast<std::int64_t>(fit_cycles_per_microframe_q16)) + 32768) >> 16U;
    out_quarter_us = quarter_us_at_cycles(static_cast<std::int64_t>(fit_reference_time) + cycles);
    return true;
}

bool microframe_at(std::uint32_t quarter_us, std::uint64_t& out_microframe) {
    const utility::InterruptLockGuard guard;
    if (state != data::TimeState::kValid || !fit_valid)
        return false;

    const std::int64_t distance_cycles =
        cycles_at_quarter_us(quarter_us) - static_cast<std::int64_t>(fit_reference_time);
    const std::int64_t microframes =
        (distance_cycles << 16U) / static_cast<std::int64_t>(fit_cycles_per_microframe_q16);
    out_microframe = static_cast<std::uint64_t>(
        static_cast<std::int64_t>(fit_reference_microframe) + microframes + anchor_offset);
    return true;
}

} // namespace libhcs::firmware::sync::timebase

#endif
