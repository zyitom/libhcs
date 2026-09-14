#include "firmware/mc02/app/src/usb/vendor.hpp"

#include <cstddef>
#include <cstdint>

#include <main.h>

#include "core/src/protocol/serializer.hpp"
#include "firmware/mc02/app/src/diag/usb_rx_hist.hpp"
#include "firmware/mc02/app/src/utility/boot_mailbox.hpp"

namespace {

constexpr uint32_t kDfuRuntimeResetDelayMs = 50U;

volatile bool g_dfu_runtime_reboot_requested = false;
volatile uint32_t g_dfu_runtime_reboot_requested_tick = 0U;

[[noreturn]] void reset_system() {
    __DSB();
    __ISB();
    NVIC_SystemReset();
    while (true) {}
}

} // namespace

namespace libhcs::firmware::usb {

core::protocol::Serializer& get_serializer() { return vendor->serializer(); }

bool uplink_session_active() { return vendor->session_established(); }

// tud_dfu_runtime_reboot_to_dfu_cb() 的延迟半边: 等请求 DFU 的那次控制传输有
// 时间完成后再复位。由主循环轮询, 延迟期间 CAN/UART 转发照常运行。
void poll_dfu_runtime_reboot() {
    if (!g_dfu_runtime_reboot_requested)
        return;

    if ((HAL_GetTick() - g_dfu_runtime_reboot_requested_tick) < kDfuRuntimeResetDelayMs)
        return;

    reset_system();
}

// TinyUSB 设备回调
extern "C" {

void tud_vendor_rx_cb(uint8_t itf, const uint8_t* buffer, uint32_t size) {
    if (itf != 0) [[unlikely]]
        return;

    diag::usb_rx_hist::note();

    // "主机的传输在此结束"判定的是短传输, 而非短包: DWC2 结束一次 OUT 传输的条件
    // 是请求的 rx_xfer_len 已收满, 或收到短于 wMaxPacketSize 的包。因此长度小于
    // 请求值即意味着后者。若改与端点的 64 字节比较, 则 rx_xfer_len 一旦超过 64,
    // 每个包之后传输都会被判结束, 协议帧会被拆散。
    usb::vendor->handle_downlink(
        {reinterpret_cast<const std::byte*>(buffer), size}, size < CFG_TUD_VENDOR_RX_EPSIZE);

    // 处理完立即重挂端点(节流时不挂, 由主循环在水位回落后再挂)。
    usb::vendor->poll_downlink_arm();
}

void tud_dfu_runtime_reboot_to_dfu_cb() {
    utility::boot_mailbox.request_enter_dfu();
    __DSB();
    __ISB();
    // TinyUSB 在 CONTROL_STAGE_SETUP 调用本回调, 紧接在 tud_control_status() 把
    // ZLP 排队之后、其真正发出之前。在 STM32H723 上此处直接 NVIC_SystemReset() 会
    // 在复位传播前的窗口内损坏 mailbox 写入(STM32F4 直接复位可行, 本芯片不同)。
    // 把复位推迟到 poll_dfu_runtime_reboot(), 让 ZLP 先完成。
    g_dfu_runtime_reboot_requested_tick = HAL_GetTick();
    g_dfu_runtime_reboot_requested = true;
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    usb::vendor->deactivate_session();
    usb::vendor->finish_downlink_transfer();
    // 恢复不会重新枚举, 下面的 tud_mount_cb 不会因此运行。
    usb::vendor->reset_downlink_arm();
    usb::vendor->set_vendor_mounted(false);
    // 新主机必须自己完成 EP0 握手。
    usb::vendor->set_ep0_handshake_done(false);
}

void tud_resume_cb() {}

void tud_mount_cb() {
    // SET_CONFIGURATION 会(重)建端点, 硬件原来持有的挂载随之消失。此刻端点尚不
    // 存在 -- 无妨, 这里只是记下欠账, 主循环会重试到 transfer 被接受为止。
    usb::vendor->reset_downlink_arm();
    usb::vendor->set_vendor_mounted(true);
    // 新主机必须自己完成 EP0 握手。
    usb::vendor->set_ep0_handshake_done(false);
}

void tud_umount_cb() {
    usb::vendor->deactivate_session();
    usb::vendor->finish_downlink_transfer();
    usb::vendor->reset_downlink_arm();
    usb::vendor->set_vendor_mounted(false);
    // 新主机必须自己完成 EP0 握手。
    usb::vendor->set_ep0_handshake_done(false);
}

} // extern "C"

} // namespace libhcs::firmware::usb
