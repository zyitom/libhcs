#include "firmware/hpm_board/app/src/usb/vendor.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <common/tusb_types.h>
#include <device/usbd.h>
#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_interrupt.h>
#include <hpm_mchtmr_drv.h>
#include <hpm_soc.h>

#include "core/src/protocol/serializer.hpp"
#include "firmware/hpm_board/app/src/diag/can_diag.hpp"
#include "firmware/hpm_board/app/src/diag/latency.hpp"
#include "firmware/hpm_board/app/src/link/uplink.hpp"
#include "firmware/hpm_board/app/src/sync/sof.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"
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

void Vendor::begin_claim() { ownership_.begin_claim(); }

void Vendor::commit_claim(uint64_t now) { ownership_.commit_claim(now); }

void Vendor::abort_claim() { ownership_.abort_claim(); }

void Vendor::drop_host() {
    deactivate_session();
    finish_downlink_transfer();
}

void Vendor::poll_ownership() { ownership_.poll(&timer::Timer::timestamp64_quarter_us); }

void Vendor::release_ownership() { ownership_.release(); }

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
//
// 放 ILM: 下行每个包都从这里进入 deserializer, 它的被调方 (deserializer 协程、
// memcpy、分发回调、Can::handle_downlink) 全部在 ILM, 自成闭环, 不是
// USB_OPTIMIZATION_LOG.md 3.2 那种"入口进 ILM、被调方散在 flash"的倒退形态。留在
// flash 时它的行会不会在两帧之间被挤出 I-cache 取决于无关代码的布局
// (同文件第 14 节)。
ATTR_PLACE_AT(".fast")
void tud_vendor_rx_cb(uint8_t itf, const uint8_t* buffer, uint32_t size) {
    const bool finished = size < g_packet_size;
    const auto payload_size =
        static_cast<uint16_t>(std::min<uint32_t>(size, CFG_TUD_VENDOR_EPSIZE));

    if (itf != 0) [[unlikely]]
        return;

    // 在任何处理之前打时间戳, 这里打开的 turnaround 才能覆盖整个设备侧路径。
    // 未定义 libhcs_APP_CAN_DIAG 时编译消失。
    diag::note_usb_out_complete();
    // 先挂后处理(tusb_config.h 的 CFG_TUD_VENDOR_RX_ARM_FIRST): 类驱动在调本回调之前
    // 已把另一块 OUT 缓冲挂上, 下面处理这一包期间端点照常收下一包, 不对主机 NAK。
    diag::note_usb_out_armed();
    diag::latency::open_downlink();

    // buffer 指向刚收完的那块 DMA 缓冲, 在本回调返回之前有效: 下一次完成要等本回调返回、
    // 回到 tud_task() 才处理, 那时才会把这块重新挂上。所以这里不能留它的指针 -- 解析器
    // 把跨包的半条记录拷进自己的缓存(core/src/protocol/deserializer.hpp), CAN / UART 的
    // 下行也都是当场拷走。
    //
    // 2026-09-05 曾以"拷贝后再挂"的形式试过并否掉(当时板子几乎不处理载荷, 测不出差别);
    // 2026-10-03 用真实 CAN 帧洪泛重测, 包率 57.2k -> 62.8k/s(+10%), 1 kHz 往返在噪声内,
    // 见 USB_OPTIMIZATION_LOG.md 16.3。
    usb::vendor->handle_downlink(
        {reinterpret_cast<const std::byte*>(buffer), payload_size}, finished);
}

void tud_dfu_runtime_reboot_to_dfu_cb() {
    boot::BootMailbox::request_enter_dfu();
    g_dfu_runtime_reboot_requested_ms = runtime_ms();
    g_dfu_runtime_reboot_requested = true;
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    usb::vendor->drop_host();
}

void tud_resume_cb() {}

// 重新枚举: 不论上一次枚举时开着什么会话, 那个主机的连接都已不在。
void tud_mount_cb() {
    g_packet_size = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512U : 64U;
    usb::vendor->drop_host();
}

void tud_umount_cb() { usb::vendor->drop_host(); }

} // extern "C"

} // namespace libhcs::firmware::usb
