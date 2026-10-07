#include "firmware/hpm_board/app/src/sync/sof.hpp"

#include <atomic>
#include <cstdint>

#include <hpm_soc.h>
#include <hpm_usb_regs.h>

#include "firmware/hpm_board/app/src/sync/sof_capture.hpp"
#include "firmware/hpm_board/app/src/sync/sof_probe.hpp"
#include "firmware/hpm_board/app/src/sync/timebase.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"

namespace libhcs::firmware::sync {

bool time_sync_start() {
    if (time_sync_on())
        return true;

    // 先复位、接通捕获, 再置位、开中断: ISR 看到"开着"时, 它要碰的状态都已就绪。
    // 此刻 SOF 中断是关着的(探针构建除外, 那时 ISR 被开关挡在时间基准之外)。
    timebase::reset();
    sof_capture::start();
    internal::g_time_sync_on.store(true, std::memory_order_release);
    HPM_USB0->USBINTR |= USB_USBINTR_SRE_MASK;
    return true;
}

void time_sync_stop() {
    if (!time_sync_on())
        return;

    internal::g_time_sync_on.store(false, std::memory_order_release);
    // 探针构建要的是一直开着的 SOF 中断; 别的构建关掉它, 不再为时间基准付每秒 8000 次中断。
    if constexpr (!sof_probe::kEnabled)
        HPM_USB0->USBINTR &= ~USB_USBINTR_SRE_MASK;
}

void sof_isr_entry() {
    // 本函数在每个 USB 中断里跑: 两者都关时只付这一次内存读。
    const bool time_sync = time_sync_on();
    if (!sof_probe::kEnabled && !time_sync)
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

    // 硬件 SOF 锁存值入环, 紧跟在上面两次读之后: 它要判断"锁存值是不是本次 SOF
    // 的", 读得越早判据越宽裕。
    if (time_sync)
        sof_capture::note_sof(frame);

    // 用 if constexpr 把关而非交给优化器: 这些是对外设的 volatile 读,
    // 即使消费者是空的 inline 函数, 编译器也必须发射它们。
    if constexpr (sof_probe::kEnabled) {
        const std::uint32_t frame_again = usb->FRINDEX & USB_FRINDEX_FRINDEX_MASK;

        // 写一清零, 且只动这一位。
        usb->USBSTS = USB_USBSTS_SRI_MASK;

        if (time_sync)
            timebase::note_sof(frame, now);
        sof_probe::note_sof(frame, frame_again, now, status, usb->PORTSC1);
    } else {
        // 写一清零, 且只动这一位。
        usb->USBSTS = USB_USBSTS_SRI_MASK;

        timebase::note_sof(frame, now);
    }
}

void sof_init() {
    if constexpr (sof_probe::kEnabled)
        HPM_USB0->USBINTR |= USB_USBINTR_SRE_MASK;
}

void sof_rearm() {
    if (sof_probe::kEnabled || time_sync_on())
        HPM_USB0->USBINTR |= USB_USBINTR_SRE_MASK;
}

} // namespace libhcs::firmware::sync
