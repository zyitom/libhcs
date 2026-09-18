#include "firmware/hpm_board/app/src/usb/vendor.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <common/tusb_types.h>
#include <device/usbd.h>
#include <hpm_clock_drv.h>
#include <hpm_interrupt.h>
#include <hpm_mchtmr_drv.h>
#include <hpm_soc.h>

#include "core/src/protocol/serializer.hpp"
#include "firmware/hpm_board/app/src/diag/can_diag.hpp"
#include "firmware/hpm_board/app/src/diag/latency.hpp"
#include "firmware/hpm_board/app/src/link/uplink.hpp"
#include "firmware/hpm_board/app/src/sync/sof.hpp"
#include "firmware/hpm_board/app/src/utility/boot_mailbox.hpp"

namespace {

constexpr uint32_t kDfuRuntimeResetDelayMs = 50U;

// bulk 端点尺寸, 挂载时刷新。下面的 tud_vendor_rx_cb() 需要它判断包是否为短包
// (短包即结束传输); 该回调每个下行包跑一次, 逐次调 tud_speed_get() 就是在最热的
// 路径上给单字节加载外包一圈 jal/ret。它在 usbd.c 里, 调用无法被内联掉。回调从
// tud_task() 在主循环运行, 不在 USB 中断里: vendor 类驱动没有注册 xfer_isr,
// xfer_cb 总是被推迟 -- can.hpp 依赖这一点。
//
// 刷新点必须是 mount 而非会话激活: 会话打开包本身就是经 tud_vendor_rx_cb 到达的,
// 若在激活时刷新, 分类那个包时读到的仍是默认值。tud_mount_cb 在 SET_CONFIGURATION
// 时运行, 早于端点存在、也就早于任何包能到达它, 且速率已被刚完成的枚举固定。
std::size_t g_packet_size = 64;

volatile bool g_dfu_runtime_reboot_requested = false;
volatile uint32_t g_dfu_runtime_reboot_requested_ms = 0U;

uint32_t runtime_ms() {
    const uint64_t ticks_per_ms = static_cast<uint64_t>(clock_get_frequency(clock_mchtmr0)) / 1000U;
    return static_cast<uint32_t>(mchtmr_get_count(HPM_MCHTMR) / ticks_per_ms);
}

} // namespace

// 三个 link:: 上行钩子不在此定义: 它们位于 link/uplink_usb.cpp -- "哪个传输拥有
// 数据面"是应用的选择, 不是本驱动的属性。见该文件。

namespace libhcs::firmware::usb {

void poll_dfu_runtime_reboot() {
    if (!g_dfu_runtime_reboot_requested)
        return;

    if ((runtime_ms() - g_dfu_runtime_reboot_requested_ms) < kDfuRuntimeResetDelayMs)
        return;

    boot::BootMailbox::reboot_to_bootloader();
}

// TinyUSB 设备回调
extern "C" {

// USB0 中断向量。HPM SDK 把具体 ISR 留在示例 family.c 里(本项目不编译它), 因此
// 在应用代码这里绑定, 两个第三方子模块保持原封不动。dcd_int_handler 是 TinyUSB
// 的设备 ISR 入口。
SDK_DECLARE_EXT_ISR_M(IRQn_USB0, hcs_usb0_isr)
void hcs_usb0_isr(void) {
    // 先于设备协议栈执行: SOF 探测的全部意义就在于在软件能及的最早瞬间读
    // FRINDEX。未定义 libhcs_APP_SOF_DIAG 时函数体为空, 但它 out-of-line 定义在
    // sof.cpp, 无 LTO 时调用本身仍在 -- 对着一个空 ret 的 jal。
    sync::sof_isr_entry();
    dcd_int_handler(0);
}

// size 必须与 TinyUSB 0.21 的声明同为 uint32_t: 两份 extern "C" 声明类型不一致时 GCC 不报,
// 但调用方按 uint32_t 传参。
void tud_vendor_rx_cb(uint8_t itf, const uint8_t* buffer, uint32_t size) {
    const bool finished = size < g_packet_size;
    const auto payload_size =
        static_cast<uint16_t>(std::min<uint32_t>(size, CFG_TUD_VENDOR_EPSIZE));

    if (itf != 0) [[unlikely]]
        return;

    // 在任何处理之前打时间戳, 这里打开的 turnaround 才能覆盖整个设备侧路径。
    // 未定义 libhcs_APP_CAN_DIAG 时编译消失。
    diag::note_usb_out_complete();
    diag::latency::open_downlink();

    // 先处理, 后挂端点: 类驱动交给本回调的指针指向端点自己的 DMA 缓冲, 类驱动在本
    // 回调返回后才重挂端点, 否则控制器会开始覆写同一块缓冲。因此端点保持未挂载 --
    // 设备对主机 NAK -- 贯穿下面的全部处理。
    //
    // 先把包拷出去可以解除该约束(turnaround 实测 2.22 -> 1.27 us), 2026-09-05 前
    // 试过。在这里没有收益: 本主机对每个设备每 125 us 微帧恰好调度 8 个 bulk
    // 事务, 下一个到来时设备早已空闲等待, 省下的 turnaround 只是挪进那段空闲 --
    // 包率有拷贝时 63999/63996/64002, 无拷贝时 63988/63988/63991。仅当主机每微帧
    // 调度超过 8 个、设备成为瓶颈时再重估。
    usb::vendor->handle_downlink(
        {reinterpret_cast<const std::byte*>(buffer), payload_size}, finished);

    diag::note_usb_out_armed();
}

void tud_dfu_runtime_reboot_to_dfu_cb() {
    boot::BootMailbox::request_enter_dfu();
    g_dfu_runtime_reboot_requested_ms = runtime_ms();
    g_dfu_runtime_reboot_requested = true;
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    usb::vendor->deactivate_session();
    usb::vendor->finish_downlink_transfer();
    // 新主机必须自己完成 EP0 握手。
    usb::vendor->set_ep0_handshake_done(false);
}

void tud_resume_cb() {}

void tud_mount_cb() {
    g_packet_size = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512U : 64U;
    // 新主机必须自己完成 EP0 握手。
    usb::vendor->set_ep0_handshake_done(false);
}

void tud_umount_cb() {
    usb::vendor->deactivate_session();
    usb::vendor->finish_downlink_transfer();
    // 新主机必须自己完成 EP0 握手。
    usb::vendor->set_ep0_handshake_done(false);
}

} // extern "C"

} // namespace libhcs::firmware::usb
