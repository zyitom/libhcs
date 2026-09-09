#include "firmware/hpm_board/app/src/sync/sof.hpp"

#include <cstdint>

#include <hpm_soc.h>
#include <hpm_usb_regs.h>

#include "firmware/hpm_board/app/src/sync/sof_probe.hpp"
#include "firmware/hpm_board/app/src/sync/timebase.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"

namespace libhcs::firmware::sync {

void sof_isr_entry() {
    if constexpr (!sof_probe::kEnabled && !timebase::kEnabled)
        return;

    USB_Type* const usb = HPM_USB0;
    const std::uint32_t status = usb->USBSTS;
    if ((status & USB_USBSTS_SRI_MASK) == 0U)
        return;

    // Earliest possible read, before the acknowledge and before any
    // bookkeeping. Reading late would let the controller finish an increment
    // that began after the status bit was set -- which is exactly the race the
    // probe exists to detect, and a late read hides it. The second read is the
    // probe's discriminator for that race; the compiler drops it when the probe
    // is compiled out.
    const std::uint32_t frame = usb->FRINDEX & USB_FRINDEX_FRINDEX_MASK;
    const std::uint32_t now = timer::Timer::timestamp_quarter_us();

    // Guarded by if constexpr rather than left to the optimizer: these are
    // volatile reads of a peripheral, so the compiler must emit them even when
    // the consumer is an empty inline function.
    std::uint32_t frame_again = frame;
    if constexpr (sof_probe::kEnabled)
        frame_again = usb->FRINDEX & USB_FRINDEX_FRINDEX_MASK;

    // Write-one-to-clear, and only this bit.
    usb->USBSTS = USB_USBSTS_SRI_MASK;

    timebase::note_sof(frame, now);
    if constexpr (sof_probe::kEnabled)
        sof_probe::note_sof(frame, frame_again, now, status, usb->PORTSC1);
}

void sof_init() {
    if constexpr (sof_probe::kEnabled || timebase::kEnabled)
        HPM_USB0->USBINTR |= USB_USBINTR_SRE_MASK;
}

void sof_rearm() { sof_init(); }

} // namespace libhcs::firmware::sync
