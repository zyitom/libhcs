#include "firmware/hpm_board/app/src/sync/pulse.hpp"

#if defined(libhcs_APP_PULSE_TEST) && libhcs_APP_PULSE_TEST

# include <hpm_clock_drv.h>
# include <hpm_gptmr_drv.h>
# include <hpm_iomux.h>
# include <hpm_soc.h>

# include "core/src/utility/assert.hpp"
# include "firmware/hpm_board/app/src/sync/timebase.hpp"
# include "firmware/hpm_board/app/src/utility/interrupt_lock.hpp"

namespace libhcs::firmware::sync::pulse {
namespace {

constexpr std::uint8_t kChannel = 1;
// CMP1 drives the pin HIGH and CMP0 drives it low -- the opposite of what
// "cmp_initial_polarity_high = false" reads like, and measured rather than
// assumed: with the target on CMP0, the far board captured its rising edge
// exactly kPulseWidthTicks late, and halving the width halved the offset
// (10128 ns -> 5133 ns). So the timing edge goes on CMP1. [实测 2026-08-20]
constexpr std::uint8_t kRisingComparator = 1;
constexpr std::uint8_t kFallingComparator = 0;
// 10 us at 24 MHz. Long enough to cross the cable and the receiver's Schmitt
// trigger, far shorter than the minimum lead, so the line is idle again well
// before the next round is armed.
constexpr std::uint32_t kPulseWidthTicks = 240;

// How far ahead a pulse must be armed. The compare register has to be written
// before the counter passes it, and the main loop is the writer, so this covers
// one comfortable main-loop period plus the host's scheduling slack. The upper
// bound keeps the extrapolation short enough that the fit's slope error stays
// well under one tick.
constexpr std::int64_t kMinLeadMicroframes = 16;
constexpr std::int64_t kMaxLeadMicroframes = 8000;

constexpr std::uint32_t kFitPeriodMs = 64;

// The fit is rejected outside +-1% of nominal. Two independent crystals are tens
// of ppm apart, never percent, so a percent means the ring was corrupted or the
// clock tree is not what init() asked for.
constexpr std::int64_t kMinTicksPerMicroframeQ16 =
    static_cast<std::int64_t>(kNominalTicksPerMicroframe) * 65536 * 99 / 100;
constexpr std::int64_t kMaxTicksPerMicroframeQ16 =
    static_cast<std::int64_t>(kNominalTicksPerMicroframe) * 65536 * 101 / 100;

// SOF samples, decimated. Only the counter is stored: samples are exactly
// kSampleDecimation microframes apart by construction (any gap resets the ring),
// so the microframe of entry i is base_microframe + i * kSampleDecimation. That
// keeps every element a single 32-bit word, which matters because the fit runs
// in the main loop against an interrupt that appends.
std::uint32_t sample_ticks[kSampleCount];
std::uint32_t sample_head = 0;
std::uint32_t sample_count = 0;
std::uint64_t sample_base_microframe = 0;
std::uint64_t next_sample_microframe = 0;
std::uint64_t last_microframe = 0;
bool sampling_started = false;
// Bumped on every append. The fit takes it before and after summing the ring and
// retries if it moved -- a seqlock, so the 8 kHz interrupt is never blocked by
// the fit, and the fit never reads a half-updated ring.
volatile std::uint32_t sample_sequence = 0;

// Published line: ticks = reference_ticks + slope * (microframe - reference).
bool fit_valid = false;
std::uint64_t fit_reference_microframe = 0;
std::int64_t fit_reference_ticks_q16 = 0;
std::uint32_t fit_slope_q16 = 0;
std::uint32_t last_fit_tick_ms = 0;

// Captured pulses waiting for the main loop to convert them. The ISR only stores
// the raw counter value; the conversion needs the fit and a division, neither of
// which belongs in an interrupt.
constexpr std::uint32_t kCaptureCapacity = 8;
std::uint32_t capture_ticks[kCaptureCapacity];
std::uint32_t capture_head = 0;
std::uint32_t capture_count = 0;

// False until the first host schedule request brings the peripheral up; see
// init() for why the bring-up is not done at boot.
bool hardware_ready = false;

std::uint32_t counter_now() {
    return gptmr_channel_get_counter(HPM_GPTMR0, kChannel, gptmr_counter_type_normal);
}

void reset_samples(std::uint64_t microframe, std::uint32_t ticks) {
    sample_head = 0;
    sample_count = 1;
    sample_ticks[0] = ticks;
    sample_base_microframe = microframe;
    next_sample_microframe = microframe + kSampleDecimation;
    // The published line refers to microframe numbers from the epoch that just
    // ended, so it is not merely stale, it is wrong. Anything scheduled against
    // it would be off by the whole epoch shift.
    fit_valid = false;
}

void push_sample(std::uint32_t ticks) {
    sample_ticks[(sample_head + sample_count) % kSampleCount] = ticks;
    if (sample_count < kSampleCount) {
        sample_count++;
    } else {
        sample_head = (sample_head + 1U) % kSampleCount;
        sample_base_microframe += kSampleDecimation;
    }
}

// One pass of the fit over the current ring. False when the ring moved
// underneath it (the caller retries) or there is not enough of it yet.
bool try_fit() {
    const std::uint32_t sequence_before = sample_sequence;
    const std::uint32_t head = sample_head;
    const std::uint32_t count = sample_count & ~1U; // whole halves only
    const std::uint64_t base = sample_base_microframe;
    if (count < 32U)
        return false;

    const std::uint32_t half = count / 2U;
    const std::uint32_t origin = sample_ticks[head];
    std::int64_t first_half_sum = 0;
    std::int64_t second_half_sum = 0;
    for (std::uint32_t index = 0; index < count; index++) {
        // Relative to the oldest sample and signed, so the counter's 179 s wrap
        // is just arithmetic.
        const auto value = static_cast<std::int64_t>(
            static_cast<std::int32_t>(sample_ticks[(head + index) % kSampleCount] - origin));
        if (index < half)
            first_half_sum += value;
        else
            second_half_sum += value;
    }
    if (sample_sequence != sequence_before)
        return false;

    // Difference of the two half-window means: the means are half*half samples
    // apart, i.e. half*half*kSampleDecimation microframes. Within a few percent
    // of least squares for slope, and it cannot overflow, which least squares on
    // absolute counter values very much can.
    const std::int64_t denominator =
        static_cast<std::int64_t>(half) * static_cast<std::int64_t>(half) * kSampleDecimation;
    const std::int64_t slope_q16 = ((second_half_sum - first_half_sum) << 16) / denominator;
    if (slope_q16 < kMinTicksPerMicroframeQ16 || slope_q16 > kMaxTicksPerMicroframeQ16)
        return false;

    // Phase from the mean of every sample, referred back to the oldest one. The
    // mean sample sits (count-1)/2 decimation periods after it.
    const std::int64_t mean_q16 = ((first_half_sum + second_half_sum) << 16) / count;
    const std::int64_t mean_offset_microframes =
        static_cast<std::int64_t>(kSampleDecimation) * (count - 1U);
    const std::int64_t reference_ticks_q16 = (static_cast<std::int64_t>(origin) << 16) + mean_q16
                                           - (slope_q16 * mean_offset_microframes) / 2;

    const utility::InterruptLockGuard guard;
    fit_reference_microframe = base;
    fit_reference_ticks_q16 = reference_ticks_q16;
    fit_slope_q16 = static_cast<std::uint32_t>(slope_q16);
    fit_valid = true;
    return true;
}

struct Fit {
    std::uint64_t reference_microframe;
    std::int64_t reference_ticks_q16;
    std::uint32_t slope_q16;
};

bool take_fit(Fit& out) {
    const utility::InterruptLockGuard guard;
    if (!fit_valid)
        return false;
    out = {fit_reference_microframe, fit_reference_ticks_q16, fit_slope_q16};
    return true;
}

void bring_up_hardware() {
    // GPTMR0 is NOT in the board's resource group -- board.c adds gpio, mchtmr,
    // ptpc and friends, and each driver adds its own (see init_can_clock in
    // boards/hpm5321/app/board_app.cpp). Without this the peripheral stays
    // clock-gated and the very first register access below stalls the core
    // forever: no USB, no DFU, a board recoverable only by pulling PA07 low or
    // by JTAG. [Measured the hard way, 2026-08-20: this exact omission bricked
    // both boards.]
    clock_add_to_group(clock_gptmr0, 0);

    // Crystal, not PLL. This is the entire point of using GPTMR here; see the
    // header. Divider 1 keeps the full 24 MHz.
    clock_set_source_divider(clock_gptmr0, clk_src_osc24m, 1);
    core::utility::assert_always(clock_get_frequency(clock_gptmr0) == 24'000'000U);

    // Steal the UART0 pins. Deliberate and documented: UART0 does not work while
    // this build option is on.
    HPM_IOC->PAD[IOC_PAD_PB08].FUNC_CTL = IOC_PB08_FUNC_CTL_GPTMR0_COMP_1;
    HPM_IOC->PAD[IOC_PAD_PB09].FUNC_CTL = IOC_PB09_FUNC_CTL_GPTMR0_CAPT_1;

    gptmr_channel_config_t config{};
    gptmr_channel_get_default_config(HPM_GPTMR0, &config);
    // Capture the incoming edge in hardware, and let the counter free-run over
    // its full range: a reload would fold the line the fit describes.
    config.mode = gptmr_work_mode_capture_at_rising_edge;
    config.reload = 0xFFFFFFFFU;
    config.enable_cmp_output = true;
    config.cmp_initial_polarity_high = false;
    // Park both comparators out of reach so no pulse is emitted before one is
    // asked for.
    config.cmp[0] = 0xFFFFFFFFU;
    config.cmp[1] = 0xFFFFFFFFU;
    core::utility::assert_always(
        gptmr_channel_config(HPM_GPTMR0, kChannel, &config, false) == status_success);

    gptmr_clear_status(HPM_GPTMR0, GPTMR_CH_CAP_IRQ_MASK(kChannel));
    gptmr_enable_irq(HPM_GPTMR0, GPTMR_CH_CAP_IRQ_MASK(kChannel));
    intc_m_enable_irq_with_priority(IRQn_GPTMR0, 2);

    gptmr_start_counter(HPM_GPTMR0, kChannel);
    hardware_ready = true;
}

} // namespace

void init() {
    // Deliberately empty. Everything this module touches -- a peripheral clock
    // gate, a pin mux, a timer channel, an interrupt -- is brought up on the
    // FIRST host schedule request instead, which can only arrive once USB has
    // enumerated and a session exists.
    //
    // The reason is recoverability, not tidiness. This is an experimental
    // measurement path; a fault in it during boot leaves a board with no USB and
    // therefore no DFU, and this board has no button (PA07 is JTAG_TMS), so the
    // only way back is a debugger. Deferring the bring-up turns any such fault
    // from "brick" into "the diagnostic does not work", with the flashing path
    // still alive.
}

void note_sof(std::uint64_t microframe) {
    if (!hardware_ready)
        return;

    // One plain register read, on the same interrupt that already reads FRINDEX
    // and the machine timer, minus the time the SOF packet itself took on the
    // wire. Without that subtraction a full-speed board's whole axis sits about
    // 2.8 us behind a high-speed one -- measured, and equal to the packet-time
    // difference to within 1%. Subtracting each board's OWN packet time puts
    // every board on the instant the packet started, whatever its speed.
    const std::uint32_t ticks =
        counter_now() - (timebase::sof_packet_delay_ns() * 24U) / 1000U;

    // Any step other than +1 means the NUMBERING moved, not just time: a missed
    // SOF interrupt, or -- far more common -- the timeline taking a host anchor
    // from a fresh host process, which shifts the epoch by whole seconds in
    // EITHER direction. The ring and the fit both describe the old epoch, so
    // both are discarded.
    //
    // Getting this wrong is not a small error: an earlier revision only handled
    // a FORWARD jump and returned early whenever the counter appeared to move
    // backwards, which wedged the sampler permanently -- the fit froze on the
    // dead epoch and every schedule was refused for the whole run. It looked
    // exactly like broken wiring. [实测 2026-08-20]
    // "Consecutive" is one SOF apart, which is one microframe at high speed and
    // EIGHT at full speed -- FRINDEX counts microframes either way, but a
    // full-speed port only receives one SOF per 1 ms frame. Hard-coding +1 here
    // wedges the sampler on a full-speed link exactly the way it wedges on an
    // epoch jump: the ring resets every SOF and no fit is ever published.
    const std::uint64_t step = timebase::microframes_per_sof();
    const bool consecutive = sampling_started && microframe == last_microframe + step;
    last_microframe = microframe;
    if (!consecutive) {
        sampling_started = true;
        reset_samples(microframe, ticks);
        sample_sequence = sample_sequence + 1U;
        return;
    }
    if (microframe != next_sample_microframe)
        return;
    push_sample(ticks);
    next_sample_microframe += kSampleDecimation;
    sample_sequence = sample_sequence + 1U;
}

void poll(std::uint32_t tick_ms) {
    if (!hardware_ready)
        return;
    if (tick_ms - last_fit_tick_ms < kFitPeriodMs)
        return;
    last_fit_tick_ms = tick_ms;
    // Three attempts is generous: a sample arrives every 8 ms and the sum takes
    // microseconds. Failing all three leaves the previous fit in place, which is
    // still good to well under a tick over one refit period.
    for (std::uint32_t attempt = 0; attempt < 3U; attempt++) {
        if (try_fit())
            return;
    }
}

void isr_handler() {
    if (!gptmr_check_status(HPM_GPTMR0, GPTMR_CH_CAP_IRQ_MASK(kChannel)))
        return;
    gptmr_clear_status(HPM_GPTMR0, GPTMR_CH_CAP_IRQ_MASK(kChannel));

    // The hardware latched this at the edge, so when the interrupt runs is
    // irrelevant -- which is the whole reason for doing it this way.
    const std::uint32_t ticks =
        gptmr_channel_get_counter(HPM_GPTMR0, kChannel, gptmr_counter_type_rising_edge);
    if (capture_count >= kCaptureCapacity)
        return;
    capture_ticks[(capture_head + capture_count) % kCaptureCapacity] = ticks;
    capture_count++;
}

bool schedule(std::uint64_t microframe) {
    if (!hardware_ready)
        bring_up_hardware();

    Fit fit{};
    if (!take_fit(fit))
        return false;

    const auto ahead_of_reference =
        static_cast<std::int64_t>(microframe - fit.reference_microframe);
    // Lead is measured from where the counter is NOW, not from the fit's
    // reference point, which sits about half a window in the past.
    const std::int64_t now_offset_microframes =
        (static_cast<std::int64_t>(
             static_cast<std::int32_t>(counter_now() - static_cast<std::uint32_t>(
                                                           fit.reference_ticks_q16 >> 16)))
         << 16)
        / static_cast<std::int64_t>(fit.slope_q16);
    const std::int64_t lead = ahead_of_reference - now_offset_microframes;
    if (lead < kMinLeadMicroframes || lead > kMaxLeadMicroframes)
        return false;

    const std::int64_t target_q16 =
        fit.reference_ticks_q16 + static_cast<std::int64_t>(fit.slope_q16) * ahead_of_reference;
    const auto target =
        static_cast<std::uint32_t>(static_cast<std::uint64_t>(target_q16 >> 16) & 0xFFFFFFFFU);

    // The output goes high at CMP0 and low again at CMP1, so BOTH have to move
    // every round. Arming CMP0 alone would work exactly once: the pin stays high
    // afterwards (CMP1 was parked at 0xFFFFFFFF), and a second CMP0 match that
    // does not change the level emits no edge for the far side to capture.
    gptmr_update_cmp(HPM_GPTMR0, kChannel, kRisingComparator, target);
    gptmr_update_cmp(HPM_GPTMR0, kChannel, kFallingComparator, target + kPulseWidthTicks);
    return true;
}

bool take_capture(std::uint64_t& microframe_q16) {
    Fit fit{};
    std::uint32_t ticks = 0;
    {
        const utility::InterruptLockGuard guard;
        if (capture_count == 0 || !fit_valid)
            return false;
        ticks = capture_ticks[capture_head];
        capture_head = (capture_head + 1U) % kCaptureCapacity;
        capture_count--;
        fit = {fit_reference_microframe, fit_reference_ticks_q16, fit_slope_q16};
    }

    // Invert the same line the pulse was scheduled on. No bracketing search and
    // no interpolation between two neighbouring SOF samples: either would hand
    // back the single-sample interrupt jitter that the fit exists to average
    // away.
    const auto delta_ticks = static_cast<std::int64_t>(static_cast<std::int32_t>(
        ticks - static_cast<std::uint32_t>(fit.reference_ticks_q16 >> 16)));
    const std::int64_t delta_ticks_q16 = (delta_ticks << 16) - (fit.reference_ticks_q16 & 0xFFFF);
    const std::int64_t offset_q16 =
        (delta_ticks_q16 << 16) / static_cast<std::int64_t>(fit.slope_q16);

    const std::int64_t absolute_q16 =
        static_cast<std::int64_t>(fit.reference_microframe << 16) + offset_q16;
    if (absolute_q16 < 0)
        return false;
    microframe_q16 = static_cast<std::uint64_t>(absolute_q16);
    return true;
}

std::uint32_t measured_ticks_per_microframe_q16() {
    const utility::InterruptLockGuard guard;
    return fit_valid ? fit_slope_q16 : 0U;
}

} // namespace libhcs::firmware::sync::pulse

extern "C" {
SDK_DECLARE_EXT_ISR_M(IRQn_GPTMR0, hcs_gptmr0_isr)
void hcs_gptmr0_isr(void) { libhcs::firmware::sync::pulse::isr_handler(); }
}

#endif
