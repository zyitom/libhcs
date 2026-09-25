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

    // 尽可能早地读, 先于应答、先于任何杂务。读晚了会让控制器完成状态位
    // 置起之后才开始的递增 -- 这恰是探针要检测的竞态, 而晚读会把它掩盖。
    // 第二次读是探针对该竞态的判据; 探针被编译剔除时, 编译器也会一并去掉
    // 它。
    const std::uint32_t frame = usb->FRINDEX & USB_FRINDEX_FRINDEX_MASK;
    const std::uint32_t now = timer::Timer::timestamp_quarter_us();

    // 用 if constexpr 把关而非交给优化器: 这些是对外设的 volatile 读,
    // 即使消费者是空的 inline 函数, 编译器也必须发射它们。
    if constexpr (sof_probe::kEnabled) {
        const std::uint32_t frame_again = usb->FRINDEX & USB_FRINDEX_FRINDEX_MASK;

        // 写一清零, 且只动这一位。
        usb->USBSTS = USB_USBSTS_SRI_MASK;

        timebase::note_sof(frame, now);
        sof_probe::note_sof(frame, frame_again, now, status, usb->PORTSC1);
    } else {
        // 写一清零, 且只动这一位。
        usb->USBSTS = USB_USBSTS_SRI_MASK;

        timebase::note_sof(frame, now);
    }
}

void sof_init() {
    if constexpr (sof_probe::kEnabled || timebase::kEnabled)
        HPM_USB0->USBINTR |= USB_USBINTR_SRE_MASK;
}

void sof_rearm() { sof_init(); }

} // namespace libhcs::firmware::sync
