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

// Bulk endpoint size, refreshed at mount. tud_vendor_rx_cb() below needs it to
// decide whether a packet is short (and therefore ends the transfer), and that
// callback runs once per downlink packet, which made tud_speed_get() a jal/ret
// around one byte load on the hottest path there is. It lives in usbd.c, so the
// call could not be inlined away. The callback runs from tud_task() on the main
// loop, not in the USB interrupt: the vendor class driver registers no xfer_isr
// in usbd.c, so its xfer_cb is always deferred. can.hpp relies on that.
//
// Mount is the right place and session activation is NOT: the session-open
// packet arrives through tud_vendor_rx_cb itself, so a value refreshed on
// activation would still be the default while that packet was being classified.
// tud_mount_cb runs on SET_CONFIGURATION, before the endpoint exists and
// therefore before any packet can arrive on it, and the speed is fixed by the
// enumeration that just finished.
std::size_t g_packet_size = 64;

volatile bool g_dfu_runtime_reboot_requested = false;
volatile uint32_t g_dfu_runtime_reboot_requested_ms = 0U;

uint32_t runtime_ms() {
    const uint64_t ticks_per_ms = static_cast<uint64_t>(clock_get_frequency(clock_mchtmr0)) / 1000U;
    return static_cast<uint32_t>(mchtmr_get_count(HPM_MCHTMR) / ticks_per_ms);
}

} // namespace

// The three link:: uplink hooks are NOT defined here: they live in
// link/uplink_usb.cpp, because "which transport owns the data plane" is an
// application choice, not a property of this driver. See that file.

namespace libhcs::firmware::usb {

void poll_dfu_runtime_reboot() {
    if (!g_dfu_runtime_reboot_requested)
        return;

    if ((runtime_ms() - g_dfu_runtime_reboot_requested_ms) < kDfuRuntimeResetDelayMs)
        return;

    boot::BootMailbox::reboot_to_bootloader();
}

// TinyUSB device callbacks
extern "C" {

// USB0 interrupt vector. The HPM SDK leaves the concrete ISR in the example
// family.c (which this project does not build), so bind it here in app code and
// keep both third-party submodules pristine. dcd_int_handler is TinyUSB's device
// ISR entry.
SDK_DECLARE_EXT_ISR_M(IRQn_USB0, hcs_usb0_isr)
void hcs_usb0_isr(void) {
    // Ahead of the device stack, because the whole point of the SOF probe is to
    // read FRINDEX at the earliest instant software can. Its body is empty
    // unless libhcs_APP_SOF_DIAG is set, but it is defined out of line in
    // sof.cpp, so without LTO the call itself remains -- a jal to a bare ret.
    sync::sof_isr_entry();
    dcd_int_handler(0);
}

void tud_vendor_rx_cb(uint8_t itf, const uint8_t* buffer, uint16_t size) {
    const bool finished = size < g_packet_size;
    const uint16_t payload_size = std::min<uint16_t>(size, CFG_TUD_VENDOR_EPSIZE);

    if (itf != 0) [[unlikely]]
        return;

    // Timestamp before any work, so the turnaround this opens covers the whole
    // device-side path. Compiled out unless libhcs_APP_CAN_DIAG.
    diag::note_usb_out_complete();
    diag::latency::open_downlink();

    // Process first, arm after: the class driver hands this callback a pointer
    // into the endpoint's own DMA buffer, and arming the endpoint again lets the
    // controller start writing that same buffer. So the endpoint stays un-armed
    // -- and the device NAKs the host -- for the whole of the processing below.
    //
    // Copying the packet out first would lift that constraint (turnaround
    // 2.22 -> 1.27 us, measured) and was tried through 2026-09-05. It buys
    // nothing here: this host schedules exactly 8 bulk transactions per 125 us
    // microframe per device, and the device is already idle waiting when the next
    // one arrives, so the saved turnaround just moves into that idle wait --
    // packet rate 63999/63996/64002 with the copy against 63988/63988/63991
    // without. Revisit only on a host that schedules more than 8 per microframe,
    // where the device would become the constraint.
    usb::vendor->handle_downlink(
        {reinterpret_cast<const std::byte*>(buffer), payload_size}, finished);

    usb::vendor->poll_downlink_arm();
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
    // Resume does not re-enumerate, so tud_mount_cb below never runs for it.
    usb::vendor->reset_downlink_arm();
    // A new host must perform the EP0 handshake for itself.
    usb::vendor->set_ep0_handshake_done(false);
}

void tud_resume_cb() {}

void tud_mount_cb() {
    g_packet_size = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512U : 64U;
    // SET_CONFIGURATION (re)creates the endpoints, so whatever arm the hardware
    // was holding is gone. The endpoint does not exist yet at this point -- that
    // is fine, this only records the debt and the main loop retries until the
    // transfer is accepted.
    usb::vendor->reset_downlink_arm();
    // A new host must perform the EP0 handshake for itself.
    usb::vendor->set_ep0_handshake_done(false);
}

void tud_umount_cb() {
    usb::vendor->deactivate_session();
    usb::vendor->finish_downlink_transfer();
    usb::vendor->reset_downlink_arm();
    // A new host must perform the EP0 handshake for itself.
    usb::vendor->set_ep0_handshake_done(false);
}

} // extern "C"

} // namespace libhcs::firmware::usb
