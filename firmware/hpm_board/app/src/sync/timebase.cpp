#include "firmware/hpm_board/app/src/sync/timebase.hpp"

#if defined(libhcs_APP_TIME_SYNC) && libhcs_APP_TIME_SYNC

# include <cstddef>

# include <hpm_ptpc_drv.h>
# include <hpm_soc.h>
# include <hpm_trgm_drv.h>
# include <hpm_trgmmux_src.h>

# include "firmware/hpm_board/app/src/sync/pulse.hpp"
# include "firmware/hpm_board/app/src/utility/interrupt_lock.hpp"

namespace libhcs::firmware::sync::timebase {
namespace {

constexpr std::uint32_t kFrindexMask = 0x3FFFU;
// PORTSC1.PSPD encoding, ChipIdea/EHCI: 0 full, 1 low, 2 high.
constexpr std::uint32_t kPortSpeedHigh = 2U;
constexpr std::uint64_t kFrindexModulus = 0x4000U;
constexpr std::uint32_t kFitPeriodMs = 20U;

// Sum of (x - mean)^2 for x = 0..N-1, which for evenly spaced samples is a
// constant the fit never has to compute: N(N^2-1)/12.
constexpr std::int64_t kSxx =
    static_cast<std::int64_t>(kSampleCount)
    * (static_cast<std::int64_t>(kSampleCount) * kSampleCount - 1) / 12;

// ------------------------------------------------------------------
// ISR-owned state. Written only by note_sof(); read by the main loop under
// utility::InterruptLockGuard. Plain scalars rather than atomics because every
// reader takes that guard -- and unlike a counter, these have to be read as a
// consistent SET, which no per-variable atomic would give.
// ------------------------------------------------------------------

// Local, boot-relative microframe count. Seeded from the first FRINDEX reading
// so that (counter mod 16384) == FRINDEX forever after, which is what lets the
// anchor be a pure multiple of 16384.
std::uint64_t counter = 0;
bool counter_seeded = false;
std::uint32_t previous_frindex = 0;

// 64-bit extension of the 32-bit machine timer reading the ISR takes. The timer
// wraps every ~1073 s at 4 MHz; SOF arrives every 125 us, so "the low word went
// backwards" is an unambiguous wrap detector here.
std::uint32_t previous_time = 0;
std::uint32_t time_high = 0;
bool time_seeded = false;

data::TimeState state = data::TimeState::kInvalid;
std::uint32_t anomaly_count = 0;
std::int64_t anchor_offset = 0;
bool anchored = false;

// Fit ring. sample[i] is the low 32 bits of the machine timer at microframe
// (ring_oldest_microframe + i * kSampleDecimation), in insertion order starting
// at ring_head.
std::uint32_t sample[kSampleCount];
std::uint32_t ring_head = 0;
std::uint32_t ring_count = 0;
std::uint64_t ring_oldest_microframe = 0;

// ------------------------------------------------------------------
// Fit result. Written only by poll(), read by everything; guarded the same way.
// ------------------------------------------------------------------
// The reference is a whole tick (0.25 us) rather than Q16: the fit averages 128
// samples precisely so the PHASE is better than one sample's jitter, and 0.25 us
// of rounding is far below the ~1 us this whole mechanism targets. The SLOPE
// stays Q16, because there a part per million compounds over the extrapolation.
bool fit_valid = false;
std::uint64_t fit_reference_microframe = 0;
std::uint64_t fit_reference_time = 0;
std::uint32_t fit_ticks_per_microframe_q16 = 0;

// Out-of-sample prediction error, accumulated between reports.
//
// Every decimated sample is first PREDICTED from the fit currently published --
// a fit computed before this sample existed -- and only then added to the
// window. So the residual is a genuine forecast error, which is exactly what a
// scheduled action would experience, rather than the in-sample residual of a
// line fitted through the point being tested.
std::int64_t residual_sum_q16 = 0;
std::uint32_t residual_count = 0;
std::uint32_t residual_abs_max_q16 = 0;

// PTPC-to-machine-timer relation. The ratio is fixed by the clock tree (both
// dividers hang off the same crystal), so only the offset is tracked -- and it
// is tracked with an exponential average rather than a fit, because there is no
// slope to estimate and a single reading carries the machine timer's 0.25 us
// quantization.
//
// The ratio is nevertheless MEASURED and published, so "same crystal, therefore
// exactly 240" is a claim the hardware can contradict instead of an assumption
// buried in a constant.
// PTPC gets its OWN least-squares fit against the microframe counter, in
// parallel with the machine timer's.
//
// The first implementation assumed the ratio between the two clocks was exactly
// 240 (both dividers hang off one crystal) and tracked only the offset with an
// exponential average. That was wrong by 452 us on hardware: an exponential
// average of a quantity with ANY residual slope lags by time-constant times
// slope, and the assumed ratio is not exact enough to make the slope zero.
// Fitting removes the assumption entirely -- and the same-board control cannot
// catch this class of error, because both captures of one frame go through the
// same conversion and the error cancels.
std::uint32_t ptpc_sample[kSampleCount];
bool ptpc_fit_valid = false;
std::uint32_t ptpc_fit_reference_ns = 0;
std::uint64_t ptpc_fit_reference_microframe = 0;
// Units per microframe, Q16. Nominally 125 us * 960 units/us = 120000.
std::uint64_t ptpc_units_per_microframe_q16 = 0;

// Pending hardware captures. Small on purpose: the probe runs at a few hundred
// frames per second at most, and a backlog would mean the main loop stopped,
// which is a bigger problem than a dropped measurement sample.
constexpr std::uint32_t kCaptureCapacity = 16;

struct RawCapture {
    std::uint32_t tag;
    std::uint32_t ptpc_ns;
    std::uint8_t bus;
};

RawCapture captures[kCaptureCapacity];
std::uint32_t capture_head = 0;
std::uint32_t capture_count = 0;
std::uint32_t capture_dropped = 0;

// The same out-of-sample treatment the machine-timer fit gets, applied to PTPC.
// This is the discriminator the whole investigation turns on: if these residuals
// are clean the PTPC SAMPLES are fine and the fit storage is at fault; if they
// are dirty the samples themselves are, and no amount of fitting will help.
// Raw step between consecutive PTPC samples, 8 ms apart, expected 7'680'000
// units. Reported because the unwrapping above assumes the nanosecond counter
// rolls over at exactly 1e9 -- if it does not, every window containing a
// rollover is silently mis-unwrapped, and that is indistinguishable from noisy
// samples in any statistic computed after the fact.
// Full-rate history of hardware SOF captures: one entry per microframe, not
// decimated.
//
// This is what replaces extrapolating a fitted line. PTPC hangs off PLL0, which
// the datasheet says is fractional-N with spread-spectrum support, and its
// instantaneous rate wanders by hundreds of ppm -- no line fitted over a second
// can predict it 28 ms ahead, which is what left 14..17 us of residual even
// after the sampling was made perfect. But a CAN capture always falls BETWEEN
// two SOF captures 125 us apart, so interpolating between them never
// extrapolates at all: the same 500 ppm of wander costs 0.06 us instead of
// 17 us.
constexpr std::uint32_t kSofHistory = 32;

struct SofCapture {
    std::uint64_t microframe;
    std::uint32_t ptpc_ns;
};

SofCapture sof_history[kSofHistory];
std::uint32_t sof_history_head = 0;
std::uint32_t sof_history_count = 0;
std::uint32_t ownership_glitches = 0;

// Latest raw pair, published untouched for host-side verification.
std::uint32_t ptpc_raw_ns = 0;
std::uint64_t ptpc_raw_microframe = 0;

std::uint32_t ptpc_step_min = 0xFFFFFFFFU;
std::uint32_t ptpc_step_max = 0;
std::uint32_t ptpc_step_previous = 0;
bool ptpc_step_seeded = false;

std::int64_t ptpc_residual_sum = 0;
std::uint32_t ptpc_residual_count = 0;
std::uint32_t ptpc_residual_abs_max = 0;

std::uint32_t last_fit_tick = 0;

void reset_fit_window() {
    ring_head = 0;
    ring_count = 0;
    fit_valid = false;
    ptpc_fit_valid = false;
    fit_ticks_per_microframe_q16 = 0;
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

// PTPC's nanosecond word ALONE, deliberately -- the seconds register is never
// read.
//
// The first version composed sec * 1e9 + ns behind a read-ns/read-sec/read-ns
// retry loop. That guard is wrong: it defends against a rollover landing between
// the reads, but the two registers are not latched together, so a coherent pair
// is not something the reader can construct. Measured cost of getting this
// wrong: the PTPC samples jittered by about one microframe while the machine
// timer sampled in the SAME interrupt stayed clean to 0.038 us, which then
// showed up as tens of microseconds of entirely fictional cross-board skew.
//
// Reading one register removes the problem instead of guarding it. Nothing here
// needs an absolute epoch: the fit unwraps within its own window, and a capture
// is only ever converted relative to a reference less than ~30 ms away, which is
// far inside the 1e9-unit (1.04 s) rollover. Modular arithmetic covers the rest.
constexpr std::int64_t kPtpcNsModulus = 1'000'000'000;

// The value PTPC latched at the last Start-of-Frame edge, not the counter as of
// now. Reading this is timing-insensitive by construction: the number was fixed
// in hardware when the edge arrived, so it does not matter how late the
// interrupt that reads it happens to be.
std::uint32_t read_ptpc_ns() { return ptpc_get_capture_ns(HPM_PTPC, PTPC_PTPC_0); }

// Shortest signed distance between two PTPC nanosecond readings.
std::int64_t ptpc_ns_difference(std::uint32_t later, std::uint32_t earlier) {
    std::int64_t difference = static_cast<std::int64_t>(later) - static_cast<std::int64_t>(earlier);
    if (difference > kPtpcNsModulus / 2)
        difference -= kPtpcNsModulus;
    else if (difference < -kPtpcNsModulus / 2)
        difference += kPtpcNsModulus;
    return difference;
}

// Integer division that rounds toward negative infinity. C++ truncates toward
// zero, which would make the wrap resolution below asymmetric about zero and
// let two boards straddling the origin pick different wraps.
std::int64_t floor_div(std::int64_t numerator, std::int64_t denominator) {
    const std::int64_t quotient = numerator / denominator;
    const std::int64_t remainder = numerator % denominator;
    return (remainder != 0 && ((remainder < 0) != (denominator < 0))) ? quotient - 1 : quotient;
}

} // namespace

std::uint32_t microframes_per_sof() {
    const std::uint32_t speed =
        (HPM_USB0->PORTSC1 & USB_PORTSC1_PSPD_MASK) >> USB_PORTSC1_PSPD_SHIFT;
    return speed == kPortSpeedHigh ? 1U : 8U;
}

std::uint32_t sof_packet_delay_ns() {
    const std::uint32_t speed =
        (HPM_USB0->PORTSC1 & USB_PORTSC1_PSPD_MASK) >> USB_PORTSC1_PSPD_SHIFT;
    // 64 bit times at 480 Mbit, 35 bit times at 12 Mbit.
    return speed == kPortSpeedHigh ? 133U : 2917U;
}

bool microframe_q16_at_ptpc(std::uint32_t ptpc_ns, std::uint64_t& out_microframe_q16);

void note_sof(std::uint32_t frindex, std::uint32_t now_quarter_us) {
    // Machine-timer wrap extension, before anything that uses the timestamp.
    if (!time_seeded) {
        time_seeded = true;
    } else if (now_quarter_us < previous_time) {
        time_high++;
    }
    previous_time = now_quarter_us;

    if (!counter_seeded) {
        counter_seeded = true;
        previous_frindex = frindex;
        // Seeding WITH the frame index, not with zero: the low 14 bits of the
        // counter must equal FRINDEX for the anchor arithmetic to reduce to a
        // multiple of 16384.
        counter = frindex;
        state = data::TimeState::kWaitingAnchor;
        reset_fit_window();
        ring_oldest_microframe = counter;
        return;
    }

    const std::uint32_t delta = (frindex - previous_frindex) & kFrindexMask;
    previous_frindex = frindex;
    counter += delta;

    // How much FRINDEX is expected to move per SOF interrupt. FRINDEX always
    // counts MICROFRAMES, but a full-speed port only receives one SOF per 1 ms
    // frame, so it steps by 8 -- exactly, every time, with the low three bits
    // pinned at zero. Treating that as an anomaly is what kept a full-speed
    // board permanently kInvalid.
    // [Measured 2026-08-20, mixed pair on one xHCI: HS delta 1 at 100.00000%
    //  (159187 transitions), FS delta 8 at 100.00000% (19898), ISR interval
    //  125.0099 us vs 1000.0784 us -- exactly 8x, same clock.]
    const std::uint32_t step = microframes_per_sof();

    if (delta != step) [[unlikely]] {
        anomaly_count++;
        if (delta > step && delta < step * 8U) {
            // A few missed interrupts. The counter is still right -- it advanced
            // by the real delta -- so the timeline survives; only the fit window
            // is spoiled, because the samples either side of the gap would tilt
            // the line.
            reset_fit_window();
            ring_oldest_microframe = counter;
        } else {
            // Delta 0 (the bus is not running: this is what the enumeration
            // window looks like) or a whole frame and more unaccounted for.
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

    // EVERY microframe, not every 64th. The history is what the interpolation
    // brackets a CAN capture with, so its spacing IS the interpolation span --
    // recording it on the decimated schedule made that span 8 ms instead of
    // 125 us and handed PTPC's wander back the two orders of magnitude the
    // hardware capture had just taken away. Cost of getting it right: one
    // register read and one ring store per SOF.
    const std::uint32_t ptpc_now = read_ptpc_ns();

    // Which SOF does the latched value belong to? The capture and the interrupt
    // race: on one of the two boards here the handler consistently reads the
    // register before the current edge's value lands, and sees the previous
    // one. That is a whole microframe of error and it is silent, so it is
    // detected per sample rather than assumed away -- the free-running counter
    // read here is always AFTER the current edge, so a gap of more than half a
    // microframe means the latched value is one edge old.
    {
        const std::uint32_t counter_now = ptpc_get_timestamp_ns(HPM_PTPC, PTPC_PTPC_0);
        std::int64_t age =
            static_cast<std::int64_t>(counter_now) - static_cast<std::int64_t>(ptpc_now);
        if (age < 0)
            age += kPtpcNsModulus;
        // How many whole microframes old is the latched value? Rounding the age
        // rather than thresholding it handles 0, 1 or 2 edges of staleness with
        // one expression, and -- more to the point -- it degrades gracefully:
        // a binary test sitting near its threshold flips sample to sample, and
        // each flip is a silent 125 us step.
        const std::int64_t edges_old =
            (age + static_cast<std::int64_t>(kNominalPtpcUnitsPerMicroframe / 2))
            / static_cast<std::int64_t>(kNominalPtpcUnitsPerMicroframe);
        const std::uint64_t owner = counter - static_cast<std::uint64_t>(edges_old);

        // Consecutive owners must differ by exactly one. Anything else is the
        // ownership decision flipping, which is the one failure this whole block
        // exists to prevent -- so it is counted rather than hoped away.
        if (sof_history_count != 0) {
            const std::uint32_t last_index =
                (sof_history_head + sof_history_count - 1U) % kSofHistory;
            if (owner != sof_history[last_index].microframe + 1U)
                ownership_glitches++;
        }
        sof_history[(sof_history_head + sof_history_count) % kSofHistory] = {owner, ptpc_now};
        if (sof_history_count < kSofHistory)
            sof_history_count++;
        else
            sof_history_head = (sof_history_head + 1U) % kSofHistory;

        if (static_cast<std::uint32_t>(age) < ptpc_step_min)
            ptpc_step_min = static_cast<std::uint32_t>(age);
        if (static_cast<std::uint32_t>(age) > ptpc_step_max)
            ptpc_step_max = static_cast<std::uint32_t>(age);
    }

    // Full rate, same reason the SOF history above is: the pulse module's
    // interpolation span is its sample spacing.
    pulse::note_sof(anchored ? static_cast<std::uint64_t>(
                        static_cast<std::int64_t>(counter) + anchor_offset)
                             : counter);

    if (counter % kSampleDecimation != 0U)
        return;

    if (ptpc_fit_valid) {
        const auto ahead = static_cast<std::int64_t>(counter - ptpc_fit_reference_microframe);
        const std::int64_t predicted =
            (static_cast<std::int64_t>(ptpc_fit_reference_ns)
             + ((ahead * static_cast<std::int64_t>(ptpc_units_per_microframe_q16)) >> 16U))
            % kPtpcNsModulus;
        const std::int64_t residual =
            ptpc_ns_difference(ptpc_now, static_cast<std::uint32_t>(predicted));
        ptpc_residual_sum += residual;
        ptpc_residual_count++;
        const auto magnitude = static_cast<std::uint64_t>(residual < 0 ? -residual : residual);
        if (magnitude > ptpc_residual_abs_max && magnitude <= 0xFFFFFFFFU)
            ptpc_residual_abs_max = static_cast<std::uint32_t>(magnitude);
    }

    if (fit_valid) {
        const auto distance = static_cast<std::int64_t>(counter - fit_reference_microframe);
        const std::int64_t predicted_q16 =
            (static_cast<std::int64_t>(fit_reference_time) << 16U)
            + distance * static_cast<std::int64_t>(fit_ticks_per_microframe_q16);
        const std::int64_t actual_q16 =
            static_cast<std::int64_t>(now_extended(now_quarter_us)) << 16U;
        const std::int64_t residual_q16 = predicted_q16 - actual_q16;

        residual_sum_q16 += residual_q16;
        residual_count++;
        const auto magnitude =
            static_cast<std::uint64_t>(residual_q16 < 0 ? -residual_q16 : residual_q16);
        if (magnitude > residual_abs_max_q16 && magnitude <= 0xFFFFFFFFU)
            residual_abs_max_q16 = static_cast<std::uint32_t>(magnitude);
    }

    if (ring_count < kSampleCount) {
        if (ring_count == 0U)
            ring_oldest_microframe = counter;
        sample[(ring_head + ring_count) % kSampleCount] = now_quarter_us;
        ptpc_sample[(ring_head + ring_count) % kSampleCount] = ptpc_now;
        ring_count++;
        return;
    }
    sample[ring_head] = now_quarter_us;
    ptpc_sample[ring_head] = ptpc_now;
    ring_head = (ring_head + 1U) % kSampleCount;
    ring_oldest_microframe += kSampleDecimation;
}

void note_can_capture(std::uint32_t tag, std::uint32_t ptpc_ns, std::uint8_t bus) {
    if (capture_count >= kCaptureCapacity) {
        capture_dropped++;
        return;
    }
    captures[(capture_head + capture_count) % kCaptureCapacity] = {tag, ptpc_ns, bus};
    capture_count++;
}

bool take_can_capture(CanCapture& out) {
    RawCapture raw{};
    {
        const utility::InterruptLockGuard guard;
        if (capture_count == 0)
            return false;
        raw = captures[capture_head];
        capture_head = (capture_head + 1) % kCaptureCapacity;
        capture_count--;
    }

    std::uint64_t microframe_q16 = 0;
    if (!microframe_q16_at_ptpc(raw.ptpc_ns, microframe_q16))
        return false;

    out = {raw.tag, microframe_q16, raw.bus, raw.ptpc_ns};
    return true;
}

void init_capture() {
    // USB0 SOF -> PTPC0 capture. Passed through unchanged rather than converted
    // to an edge pulse: SOF is already a single-cycle strobe, and re-shaping it
    // would only add a TRGM clock of uncertainty.
    trgm_output_t output{};
    output.invert = false;
    output.type = trgm_output_same_as_input;
    output.input = HPM_TRGM0_INPUT_SRC_USB0_SOF;
    trgm_output_config(HPM_TRGM0, HPM_TRGM0_OUTPUT_SRC_MCAN_PTPC0_CAP, &output);

    // Overwrite on every edge. "Capture keep" holds the first value until it is
    // read, which is the wrong policy here: a sample missed by the main loop
    // must not make the next one stale.
    ptpc_disable_capture_keep(HPM_PTPC, PTPC_PTPC_0);
    ptpc_config_capture(HPM_PTPC, PTPC_PTPC_0, ptpc_capture_trigger_on_rising_edge);
}

void ptpc_reference(std::uint64_t& units, std::uint64_t& microframe) {
    const utility::InterruptLockGuard guard;
    units = ptpc_fit_reference_ns;
    microframe = ptpc_fit_reference_microframe;
}

std::uint32_t ptpc_units_per_microframe() {
    const utility::InterruptLockGuard guard;
    return static_cast<std::uint32_t>(ptpc_units_per_microframe_q16 >> 16U);
}

bool microframe_q16_at_ptpc(std::uint32_t ptpc_ns, std::uint64_t& out_microframe_q16) {
    const utility::InterruptLockGuard guard;
    if (state != data::TimeState::kValid)
        return false;

    // Interpolate between the two hardware SOF captures that bracket this one.
    // Walk newest to oldest and stop at the first entry the capture is at or
    // after; the history covers 4 ms, and a capture is converted within one
    // main-loop tick of arriving.
    for (std::uint32_t back = 1; back < sof_history_count; back++) {
        const std::uint32_t newer_index =
            (sof_history_head + sof_history_count - back) % kSofHistory;
        const std::uint32_t older_index =
            (sof_history_head + sof_history_count - back - 1U) % kSofHistory;
        const SofCapture& newer = sof_history[newer_index];
        const SofCapture& older = sof_history[older_index];

        const std::int64_t span = ptpc_ns_difference(newer.ptpc_ns, older.ptpc_ns);
        const std::int64_t offset = ptpc_ns_difference(ptpc_ns, older.ptpc_ns);
        if (span <= 0)
            continue;
        if (offset < 0 || offset > span)
            continue;

        const std::int64_t microframe_span =
            static_cast<std::int64_t>(newer.microframe - older.microframe);
        if (microframe_span <= 0)
            continue;

        out_microframe_q16 = static_cast<std::uint64_t>(
            ((static_cast<std::int64_t>(older.microframe) + anchor_offset) << 16U)
            + (offset * microframe_span * 65536) / span);
        return true;
    }

    if (!ptpc_fit_valid)
        return false;

    // Straight from PTPC to microframes through PTPC's own fitted line. The
    // machine timer is not in this path at all, so nothing depends on the two
    // dividers being in an exact ratio -- and the distance is modular, so no
    // absolute epoch is needed either.
    const std::int64_t distance = ptpc_ns_difference(ptpc_ns, ptpc_fit_reference_ns);
    const std::int64_t microframes_q16 =
        (distance << 32U) / static_cast<std::int64_t>(ptpc_units_per_microframe_q16);

    out_microframe_q16 = static_cast<std::uint64_t>(
        ((static_cast<std::int64_t>(ptpc_fit_reference_microframe) + anchor_offset) << 16U)
        + microframes_q16);
    return true;
}

void poll(std::uint32_t tick_ms) {
    if (tick_ms - last_fit_tick < kFitPeriodMs)
        return;
    last_fit_tick = tick_ms;

    std::uint32_t local_sample[kSampleCount];
    std::uint32_t local_ptpc[kSampleCount];
    std::uint32_t count = 0;
    std::uint64_t oldest_microframe = 0;
    std::uint64_t now = 0;

    {
        // Bounded and short: 128 word copies, about a microsecond, twice per
        // 20 ms tick. Masking rather than a lock-free snapshot because the fit
        // needs the ring and its origin to be mutually consistent, and the
        // established pattern in this firmware for that is the interrupt guard.
        const utility::InterruptLockGuard guard;
        count = ring_count;
        oldest_microframe = ring_oldest_microframe;
        now = now_extended(previous_time);
        for (std::uint32_t index = 0; index < count; index++) {
            local_sample[index] = sample[(ring_head + index) % kSampleCount];
            local_ptpc[index] = ptpc_sample[(ring_head + index) % kSampleCount];
        }
    }

    if (count < kSampleCount)
        return;

    // Least squares against evenly spaced x = 0..N-1, with y taken relative to
    // the first sample so the arithmetic stays inside 32 bits before widening.
    // Sxy is accumulated in the doubled form sum((2x - (N-1)) * y) to keep the
    // half-integer mean out of integer arithmetic.
    const std::uint32_t base = local_sample[0];
    std::int64_t sum_y = 0;
    std::int64_t sum_xy2 = 0;
    for (std::uint32_t index = 0; index < count; index++) {
        const auto y = static_cast<std::int64_t>(
            static_cast<std::int32_t>(local_sample[index] - base));
        sum_y += y;
        sum_xy2 += (2LL * index - static_cast<std::int64_t>(count - 1U)) * y;
    }

    // Ticks per decimated sample, then per microframe, both Q16.
    const std::int64_t slope_q16 = (sum_xy2 << 16U) / (2 * kSxx);
    const std::int64_t per_microframe_q16 = slope_q16 / kSampleDecimation;

    // A slope more than a few percent off nominal is not a crystal offset, it is
    // a broken window; refusing it keeps a bad fit from being published at all.
    constexpr std::int64_t kNominalQ16 =
        static_cast<std::int64_t>(kNominalTicksPerMicroframe) << 16U;
    if (per_microframe_q16 < kNominalQ16 - kNominalQ16 / 32
        || per_microframe_q16 > kNominalQ16 + kNominalQ16 / 32)
        return;

    // The ring holds only the low 32 bits of the timer. Rebuild the oldest
    // sample's full value by hanging it off the current time, which is at most
    // one window (1.024 s) later and therefore at most one wrap away.
    std::uint64_t base_extended = (now & ~static_cast<std::uint64_t>(0xFFFFFFFFU)) | base;
    if (base_extended > now)
        base_extended -= static_cast<std::uint64_t>(1) << 32U;

    // Evaluate the fitted line at the NEWEST sample rather than at the centroid:
    // every query extrapolates forward from now, so anchoring the reference at
    // the leading edge is what keeps the extrapolation arm short.
    const std::int64_t mean_y_q16 = (sum_y << 16U) / count;
    const std::int64_t offset_q16 =
        mean_y_q16 + (slope_q16 * static_cast<std::int64_t>(count - 1U)) / 2;

    // The same least squares again, against PTPC instead of the machine timer.
    // Two independent fits rather than one fit plus a fixed conversion: the two
    // clock dividers are not in an exact enough integer ratio for the shortcut,
    // and getting that wrong cost 452 us of apparent skew before it was caught.
    // Unwrapped inside the window: consecutive samples are 8 ms apart, two
    // orders below the rollover, so a reading that went backwards can only mean
    // the nanosecond counter wrapped.
    const std::uint32_t ptpc_base = local_ptpc[0];
    std::int64_t ptpc_sum_y = 0;
    std::int64_t ptpc_sum_xy2 = 0;
    std::int64_t y = 0;
    std::uint32_t ptpc_previous = ptpc_base;
    for (std::uint32_t index = 0; index < count; index++) {
        std::int64_t step =
            static_cast<std::int64_t>(local_ptpc[index]) - static_cast<std::int64_t>(ptpc_previous);
        if (step < 0)
            step += kPtpcNsModulus;
        y += step;
        ptpc_previous = local_ptpc[index];
        ptpc_sum_y += y;
        ptpc_sum_xy2 += (2LL * index - static_cast<std::int64_t>(count - 1U)) * y;
    }
    const std::int64_t ptpc_slope_q16 = (ptpc_sum_xy2 << 16U) / (2 * kSxx);
    const std::int64_t ptpc_per_microframe_q16 = ptpc_slope_q16 / kSampleDecimation;

    constexpr std::int64_t kNominalPtpcQ16 =
        static_cast<std::int64_t>(kNominalPtpcUnitsPerMicroframe) << 16U;
    if (ptpc_per_microframe_q16 < kNominalPtpcQ16 - kNominalPtpcQ16 / 32
        || ptpc_per_microframe_q16 > kNominalPtpcQ16 + kNominalPtpcQ16 / 32)
        return;

    const std::int64_t ptpc_offset_q16 =
        ((ptpc_sum_y << 16U) / count) + (ptpc_slope_q16 * static_cast<std::int64_t>(count - 1U)) / 2;

    const utility::InterruptLockGuard guard;
    ptpc_fit_reference_ns = static_cast<std::uint32_t>(
        (static_cast<std::int64_t>(ptpc_base) + ((ptpc_offset_q16 + 32768) >> 16U))
        % kPtpcNsModulus);
    ptpc_fit_reference_microframe =
        oldest_microframe + static_cast<std::uint64_t>(count - 1U) * kSampleDecimation;
    ptpc_units_per_microframe_q16 = static_cast<std::uint64_t>(ptpc_per_microframe_q16);
    ptpc_fit_valid = true;

    fit_reference_microframe =
        oldest_microframe + static_cast<std::uint64_t>(count - 1U) * kSampleDecimation;
    fit_reference_time = base_extended + static_cast<std::uint64_t>((offset_q16 + 32768) >> 16U);
    fit_ticks_per_microframe_q16 = static_cast<std::uint32_t>(per_microframe_q16);
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
        difference + static_cast<std::int64_t>(kFrindexModulus / 2),
        static_cast<std::int64_t>(kFrindexModulus));
    const std::int64_t offset = wraps * static_cast<std::int64_t>(kFrindexModulus);

    if (!anchored) {
        anchor_offset = offset;
        anchored = true;
    } else if (offset != anchor_offset) {
        // The plan's rule, and it matters: a changed wrap on a live timeline
        // cannot be accepted quietly. Either the counter lost more than a second
        // or the host's estimate did, and both mean everything scheduled since
        // the last anchor was scheduled against the wrong second.
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
    return {
        .state = state,
        .microframe = anchored ? static_cast<std::uint64_t>(
                          static_cast<std::int64_t>(counter) + anchor_offset)
                               : counter,
        .timestamp_quarter_us = now_extended(previous_time),
        .ticks_per_microframe_q16 = fit_ticks_per_microframe_q16,
        .anomaly_count = anomaly_count,
        .residual_mean_q16 = residual_count == 0
                               ? 0
                               : static_cast<std::int32_t>(
                                     residual_sum_q16 / static_cast<std::int64_t>(residual_count)),
        .residual_abs_max_q16 = residual_abs_max_q16,
        .residual_count = static_cast<std::uint16_t>(
            residual_count > 0xFFFFU ? 0xFFFFU : residual_count),
        .ptpc_residual_mean = ptpc_residual_count == 0
                                ? 0
                                : static_cast<std::int32_t>(
                                      ptpc_residual_sum
                                      / static_cast<std::int64_t>(ptpc_residual_count)),
        .ptpc_residual_abs_max = ptpc_residual_abs_max,
        .ptpc_step_min = ptpc_step_min,
        .ptpc_step_max = ptpc_step_max,
        .ptpc_raw_ns = ownership_glitches,
        .ptpc_raw_microframe = ptpc_raw_microframe,
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
    ptpc_residual_sum = 0;
    ptpc_residual_count = 0;
    ptpc_residual_abs_max = 0;
    ptpc_step_min = 0xFFFFFFFFU;
    ptpc_step_max = 0;
    return result;
}

bool local_time_of(std::uint64_t microframe, std::uint64_t& out_quarter_us) {
    const utility::InterruptLockGuard guard;
    if (state != data::TimeState::kValid || !fit_valid)
        return false;

    const auto local_microframe =
        static_cast<std::uint64_t>(static_cast<std::int64_t>(microframe) - anchor_offset);
    const auto distance = static_cast<std::int64_t>(local_microframe - fit_reference_microframe);
    const std::int64_t ticks =
        (distance * static_cast<std::int64_t>(fit_ticks_per_microframe_q16) + 32768) >> 16U;
    const std::int64_t result = static_cast<std::int64_t>(fit_reference_time) + ticks;
    if (result < 0)
        return false;
    out_quarter_us = static_cast<std::uint64_t>(result);
    return true;
}

bool microframe_at(std::uint64_t quarter_us, std::uint64_t& out_microframe) {
    const utility::InterruptLockGuard guard;
    if (state != data::TimeState::kValid || !fit_valid)
        return false;

    const auto distance_ticks =
        static_cast<std::int64_t>(quarter_us) - static_cast<std::int64_t>(fit_reference_time);
    const std::int64_t microframes =
        (distance_ticks << 16U) / static_cast<std::int64_t>(fit_ticks_per_microframe_q16);
    out_microframe = static_cast<std::uint64_t>(
        static_cast<std::int64_t>(fit_reference_microframe) + microframes + anchor_offset);
    return true;
}

} // namespace libhcs::firmware::sync::timebase

#endif
