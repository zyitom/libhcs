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

// Deferred half of tud_dfu_runtime_reboot_to_dfu_cb(): resets once the control
// transfer that requested DFU has had time to complete. Polled from the main
// loop, so CAN/UART forwarding keeps running during the delay.
void poll_dfu_runtime_reboot() {
    if (!g_dfu_runtime_reboot_requested)
        return;

    if ((HAL_GetTick() - g_dfu_runtime_reboot_requested_tick) < kDfuRuntimeResetDelayMs)
        return;

    reset_system();
}

// TinyUSB device callbacks
extern "C" {

void tud_vendor_rx_cb(uint8_t itf, const uint8_t* buffer, uint32_t size) {
    if (itf != 0) [[unlikely]]
        return;

    diag::usb_rx_hist::note();

    // "The host's transfer ended here" is a short transfer, not a short packet:
    // DWC2 completes an OUT transfer either when the requested rx_xfer_len is
    // full or when a packet shorter than wMaxPacketSize arrives. Anything below
    // the requested length therefore means the second case. Comparing against
    // the endpoint's 64 bytes instead would end the transfer after every packet
    // once rx_xfer_len exceeds 64, splitting protocol frames.
    usb::vendor->handle_downlink(
        {reinterpret_cast<const std::byte*>(buffer), size}, size < CFG_TUD_VENDOR_RX_EPSIZE);
}

void tud_dfu_runtime_reboot_to_dfu_cb() {
    utility::boot_mailbox.request_enter_dfu();
    __DSB();
    __ISB();
    // TinyUSB calls this callback from CONTROL_STAGE_SETUP, immediately after
    // tud_control_status() queues the ZLP, before it is physically transmitted.
    // On STM32H723, calling NVIC_SystemReset() here corrupts the mailbox write
    // in the window before reset propagates (unlike STM32F4 where direct reset
    // works). Defer the reset to poll_dfu_runtime_reboot() so the ZLP completes
    // first.
    g_dfu_runtime_reboot_requested_tick = HAL_GetTick();
    g_dfu_runtime_reboot_requested = true;
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    usb::vendor->deactivate_session();
    usb::vendor->finish_downlink_transfer();
}

void tud_resume_cb() {}

void tud_mount_cb() {}

void tud_umount_cb() {
    usb::vendor->deactivate_session();
    usb::vendor->finish_downlink_transfer();
}

} // extern "C"

} // namespace libhcs::firmware::usb
