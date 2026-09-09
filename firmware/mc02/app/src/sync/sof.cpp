#include "firmware/mc02/app/src/sync/sof.hpp"

#include <cstdint>

#include <main.h>

#include "firmware/mc02/app/src/sync/timebase.hpp"

namespace libhcs::firmware::sync {
namespace {

// The DWC2 register blocks. USB_OTG_HS_PERIPH_BASE is the global block; the
// device block sits at a fixed offset from it. Reached through CMSIS rather than
// through TinyUSB's dwc2_regs_t so that nothing here depends on TinyUSB's
// internal layout.
USB_OTG_GlobalTypeDef* global_registers() {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<USB_OTG_GlobalTypeDef*>(USB_OTG_HS_PERIPH_BASE);
}

USB_OTG_DeviceTypeDef* device_registers() {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<USB_OTG_DeviceTypeDef*>(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
}

// ---- Hardware SOF capture, TIM2 ITR5 ----
//
// USB1_OTG_HS_SOF is wired to TIM2's internal trigger 5 on this part, so mapping
// a TIM2 capture channel to TRC latches the counter at the SOF edge with no
// software in the path -- the direct analogue of the HPM's TRGM route into PTPC.
// [Measured 2026-09-07: sweeping SMCR.TS over ITR0..ITR13 gives captures at the
//  SOF rate on ITR5 and exactly zero on every other source, with a minimum
//  capture interval of 999 ticks at TIM2's 1 MHz -- the 1 ms full-speed frame.
//  TIM5 has no such source; its ITR5 is USB2_OTG_FS, which the H723 lacks.]
//
// This does NOT replace the interrupt timestamp; it CORRECTS it. The capture
// says how long before this handler ran the edge actually arrived, and taking
// that off the cycle counter removes the interrupt entry latency -- both its
// constant part and its jitter -- from every sample.
//
// It is applied only when it is actually an improvement, because a capture is
// exact at the edge but no better than the counter it is latched into, and TIM2
// shipped at 1 MHz for the 50 Hz servo PWM. The interrupt path's own per-sample
// sigma is 0.1365 us (75 CPU cycles) [measured 2026-09-07], so:
//
//   TIM2 @ 275 MHz, 2 cycles/tick   tick far below the jitter -> jitter removed
//   TIM2 @ 1 MHz, 550 cycles/tick   tick far above it         -> no effect
//
// The second case is worth stating precisely, because the obvious guess is wrong
// and it was measured: forcing the correction on at 1 MHz changed the fitted
// residual not at all (0.0314 us against 0.031-0.039 us uncorrected, 0 anomalies,
// same fitted rate). It does NOT inject the tick's quantization noise, because
// when the jitter is much smaller than a tick the computed age is almost always
// the SAME integer -- so the correction is a constant, and subtracting a constant
// cannot change a standard deviation. The fit was already absorbing that constant
// into its offset. A coarse capture is therefore not harmful, merely useless, and
// it is gated off so the two extra peripheral reads per millisecond are not paid
// for nothing.

std::uint32_t capture_reload = 0;
std::uint32_t capture_cycles_per_tick = 0;
std::uint32_t capture_max_age_ticks = 0;
bool capture_enabled = false;

// Per-sample sigma of the interrupt path, in CPU cycles, measured on this board.
// The capture is only useful when a tick is well below this, so that the age it
// computes actually tracks the jitter instead of rounding to a constant.
constexpr std::uint32_t kIsrSigmaCycles = 75;
constexpr std::uint32_t kCaptureWorthwhileCycles = kIsrSigmaCycles / 3U;

void capture_init() {
    if constexpr (!timebase::kEnabled)
        return;

    // CH2 mapped to TRC. CH1 and CH3 are the servo PWM outputs and are not
    // touched, and SMS is left at 0 so the counter is never slaved to the
    // trigger -- only the capture channel consumes it. CubeMX leaves TIM2's
    // SMCR at its reset value (MX_TIM2_Init does PWM setup only), so the
    // read-modify-write below cannot disturb a slave mode that was configured.
    TIM2->CCER &= ~TIM_CCER_CC2E;
    TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_CC2S) | (0x3UL << TIM_CCMR1_CC2S_Pos);
    TIM2->SMCR = (TIM2->SMCR & ~TIM_SMCR_TS) | TIM_TS_ITR5;
    TIM2->CCER |= TIM_CCER_CC2E;
    (void)TIM2->CCR2;

    capture_reload = TIM2->ARR + 1U;
    // SYSCLK is exactly twice TIM2's kernel clock on this clock tree (AHB /2,
    // APB1 /2, then the timer's x2), so a TIM2 tick is an exact whole number of
    // CPU cycles and no clock-rate query is needed -- only the prescaler, which
    // is what changes when the .ioc does.
    capture_cycles_per_tick = 2U * (TIM2->PSC + 1U);
    // Half a millisecond, in ticks. A capture older than that cannot belong to
    // the SOF being handled, so it is refused rather than turned into a wild
    // correction.
    capture_max_age_ticks = capture_cycles_per_tick == 0U
                              ? 0U
                              : (timebase::kCyclesPerMicrosecond * 500U) / capture_cycles_per_tick;
    capture_enabled = capture_cycles_per_tick != 0U
                   && capture_cycles_per_tick < kCaptureWorthwhileCycles && capture_reload > 1U;
}

// Cycles to subtract from the interrupt timestamp, or 0 when the capture cannot
// be trusted (which degrades exactly to the pre-capture behaviour).
std::uint32_t capture_age_cycles() {
    if (!capture_enabled)
        return 0;
    const std::uint32_t capture = TIM2->CCR2;
    const std::uint32_t counter = TIM2->CNT;
    // TIM2 wraps at ARR + 1, not at 2^32, so the difference is taken modulo the
    // reload rather than with plain unsigned arithmetic.
    const std::uint32_t age =
        counter >= capture ? counter - capture : (counter + capture_reload) - capture;
    if (age > capture_max_age_ticks)
        return 0;
    return age * capture_cycles_per_tick;
}

} // namespace

void sof_isr_entry() {
    if constexpr (!timebase::kEnabled)
        return;

    // Read the cycle counter FIRST, before deciding whether this interrupt is
    // even a SOF. GINTSTS is an AHB read that costs tens of nanoseconds and can
    // stall behind bus traffic; taking the timestamp after it would fold that
    // variable stall into every sample. CYCCNT is a core-local register, two
    // cycles, so the unconditional read costs nothing on the interrupts that
    // turn out not to be SOFs.
    std::uint32_t now = DWT->CYCCNT;

    USB_OTG_GlobalTypeDef* const global = global_registers();
    const std::uint32_t status = global->GINTSTS;
    if ((status & USB_OTG_GINTSTS_SOF) == 0U)
        return;

    const std::uint32_t frame =
        (device_registers()->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos;

    // Wind the timestamp back to the edge the hardware latched. Read here rather
    // than before the SOF test so the extra peripheral accesses are paid once per
    // millisecond instead of on every USB interrupt; the few tens of nanoseconds
    // between this and the cycle-counter read above are the same on every pass,
    // so they are a constant the fit absorbs.
    now -= capture_age_cycles();

    // Write-one-to-clear, and only this bit: GINTSTS holds read-only bits for
    // the endpoint interrupts, so a read-modify-write here would be both
    // unnecessary and a chance to lose an edge.
    global->GINTSTS = USB_OTG_GINTSTS_SOF;

    timebase::note_sof(frame, now);
}

void sof_init() {
    if constexpr (timebase::kEnabled) {
        capture_init();
        global_registers()->GINTMSK |= USB_OTG_GINTMSK_SOFM;
    }
}

bool sof_capture_active() { return capture_enabled; }

std::uint32_t sof_capture_cycles_per_tick() { return capture_cycles_per_tick; }

void sof_rearm() {
    // Only the interrupt enable, not capture_init(): the capture configuration
    // is stable once set, and rewriting CCMR1/SMCR at the tick rate would be
    // peripheral traffic for nothing.
    if constexpr (timebase::kEnabled)
        global_registers()->GINTMSK |= USB_OTG_GINTMSK_SOFM;
}

} // namespace libhcs::firmware::sync

#if defined(libhcs_APP_TIME_SYNC) && libhcs_APP_TIME_SYNC

// Link-time interposition on TinyUSB's device interrupt handler; see sof.hpp for
// why the hook cannot simply be the first statement of the vector. Enabled by
// -Wl,--wrap=dcd_int_handler, which the app CMakeLists adds only in this
// configuration -- with the time base off neither symbol exists in the link.
extern "C" {

// The two leading underscores are the linker's ABI for --wrap, not a style
// choice, so the reserved-identifier and naming checks are silenced here rather
// than obeyed.
// NOLINTBEGIN(bugprone-reserved-identifier,readability-identifier-naming)
void __real_dcd_int_handler(std::uint8_t rhport);

void __wrap_dcd_int_handler(std::uint8_t rhport) {
    libhcs::firmware::sync::sof_isr_entry();
    __real_dcd_int_handler(rhport);
}
// NOLINTEND(bugprone-reserved-identifier,readability-identifier-naming)
}

#endif
