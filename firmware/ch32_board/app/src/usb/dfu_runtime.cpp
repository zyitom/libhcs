// DFU run-time interface for the application image.
//
// The application does not implement firmware download -- that is the V3F
// bootloader's job. It only advertises a DFU run-time interface so the host can
// say "detach and come back in DFU mode": on DFU_DETACH the app records the
// request in the boot mailbox and resets, and the bootloader reads the mailbox
// on the way up and enters DFU instead of waking this core again.
//
// Mirrors mc02's tud_dfu_runtime_reboot_to_dfu_cb() path; the reset is deferred
// to the main loop so the control transfer's status stage completes first --
// resetting inside the ISR leaves the host waiting for an ack that never comes,
// and it reports the detach as failed.

#include <cstdint>
#include <cstring>

extern "C" {
#include "ch32h417.h"
#include "ch32h417_usbss_device.h"
}

#include "firmware/ch32_board/app/src/usb/dfu_runtime.hpp"
#include "firmware/ch32_board/app/src/utility/interrupt_lock.hpp"
#include "firmware/ch32_board/boot/src/mailbox.hpp"
#include "firmware/ch32_board/boot/src/utility/boot_mailbox.hpp"

namespace libhcs::firmware::usb {
namespace {

constexpr uint8_t kRequestTypeMask = 0x60;
constexpr uint8_t kRequestTypeClass = 0x20;
constexpr uint8_t kRequestRecipientMask = 0x1F;
constexpr uint8_t kRequestRecipientInterface = 0x01;
constexpr uint8_t kRequestDirectionIn = 0x80;

constexpr uint8_t kStall = 0xFF;
constexpr uint8_t kAccept = 0x00;

// DFU 1.1 request codes and the two run-time states (spec table 3.2 / A.1).
constexpr uint8_t kDfuRequestDetach = 0;
constexpr uint8_t kDfuRequestGetStatus = 3;
constexpr uint8_t kDfuRequestClearStatus = 4;
constexpr uint8_t kDfuRequestGetState = 5;
constexpr uint8_t kDfuStateAppIdle = 0;
constexpr uint8_t kDfuStatusOk = 0;

// Interface number carrying the DFU run-time descriptor. Interface 0 is the
// libhcs vendor interface, so DFU run-time sits on interface 1.
constexpr uint16_t kDfuRuntimeInterfaceNumber = 1;

volatile bool detach_requested = false;

} // namespace

// Called from the forwarding loop, off the ISR, once per iteration. Does not
// return once the host has asked for a detach: this core parks and the boot core
// resets the chip out from under it.
void poll_dfu_runtime_reboot() {
    if (!detach_requested)
        return;

    // No further USB traffic matters from here on; the host is already waiting
    // for the device to disappear. Drop the SuperSpeed link explicitly so it
    // sees a disconnect now rather than a stalled device until the reset lands.
    USBSS_Device_Init(DISABLE);

    const utility::InterruptLockGuard guard;
    firmware::utility::boot_mailbox().request_enter_dfu();

    // The reset itself belongs to the boot core: this core's reset vector is
    // flash 0x0, which holds the V3F image, so resetting here would run the boot
    // core's code on this one. Ask V3F instead (see SharedBlock::reset_request)
    // and park -- either V3F's reset covers the whole chip, in which case this
    // loop is short-lived, or it does not and this core stays quiescent while
    // the bootloader re-initialises the USB device.
    boot::shared().reset_request = boot::kResetRequestMagic;
    while (true)
        __WFI();
}

} // namespace libhcs::firmware::usb

extern "C" {

uint8_t usb_ss_class_setup(void) {
    namespace usb = libhcs::firmware::usb;

    if ((USBSS_SetupReqType & usb::kRequestTypeMask) != usb::kRequestTypeClass)
        return usb::kStall;
    if ((USBSS_SetupReqType & usb::kRequestRecipientMask) != usb::kRequestRecipientInterface)
        return usb::kStall;
    if ((USBSS_SetupReqIndex & 0xFF) != usb::kDfuRuntimeInterfaceNumber)
        return usb::kStall;

    switch (USBSS_SetupReqCode) {
    case usb::kDfuRequestDetach:
        // bitWillDetach = 1, so the device -- not the host -- drops off the bus.
        // Ack now; poll_dfu_runtime_reboot() does the reset after the status
        // stage.
        usb::detach_requested = true;
        return usb::kAccept;

    case usb::kDfuRequestGetStatus: {
        if (!(USBSS_SetupReqType & usb::kRequestDirectionIn))
            return usb::kStall;
        // bStatus, bwPollTimeout[3], bState, iString -- always appIDLE here.
        const uint8_t response[6] = {usb::kDfuStatusOk, 0, 0, 0, usb::kDfuStateAppIdle, 0};
        if (USBSS_SetupReqLen > sizeof(response))
            USBSS_SetupReqLen = sizeof(response);
        std::memcpy(USBSS_EP0_Buf, response, USBSS_SetupReqLen);
        return usb::kAccept;
    }

    case usb::kDfuRequestGetState: {
        if (!(USBSS_SetupReqType & usb::kRequestDirectionIn))
            return usb::kStall;
        const uint8_t state = usb::kDfuStateAppIdle;
        if (USBSS_SetupReqLen > sizeof(state))
            USBSS_SetupReqLen = sizeof(state);
        std::memcpy(USBSS_EP0_Buf, &state, USBSS_SetupReqLen);
        return usb::kAccept;
    }

    case usb::kDfuRequestClearStatus:
        // Run-time mode never enters dfuERROR, so this is a no-op ack.
        return usb::kAccept;

    default: return usb::kStall;
    }
}

// The run-time interface has no control-OUT data stage: DETACH carries none and
// download belongs to the bootloader.
void usb_ss_class_ep0_out(void) {}

} // extern "C"
