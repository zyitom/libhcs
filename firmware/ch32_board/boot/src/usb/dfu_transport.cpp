// DFU class requests on the USBSS control endpoint (bootloader / DFU mode).
//
// The WCH stack stalls every non-standard request; the libhcs LOCAL PATCH in
// bsp/usb/ch32h417_usbss_it.c routes them here instead. All six DFU 1.1 requests
// live on EP0, which is why DFU needs nothing SuperSpeed-specific -- the only
// difference from a full-speed device is bMaxPacketSize0.
//
// Transfer sizing: the DFU functional descriptor advertises wTransferSize equal
// to the SuperSpeed EP0 packet size (512), so every DNLOAD block arrives as a
// single control-OUT packet and the payload length is exactly the SETUP packet's
// wLength. That avoids depending on an EP0 RX length register whose semantics
// are not documented in the reference manual.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

extern "C" {
#include "ch32h417_usbss_device.h"
}

#include "firmware/ch32_board/boot/src/usb/dfu.hpp"

namespace libhcs::firmware::usb {
namespace {

// bmRequestType fields, from the USB 2.0 spec table 9-2.
constexpr uint8_t kRequestTypeMask = 0x60;
constexpr uint8_t kRequestTypeClass = 0x20;
constexpr uint8_t kRequestRecipientMask = 0x1F;
constexpr uint8_t kRequestRecipientInterface = 0x01;
constexpr uint8_t kRequestDirectionIn = 0x80;

constexpr uint8_t kStall = 0xFF;
constexpr uint8_t kAccept = 0x00;

// The interface the DFU functional descriptor is attached to. Requests aimed at
// any other interface are not ours.
constexpr uint16_t kDfuInterfaceNumber = 0;

// Set by the SETUP stage of a DNLOAD so the data stage knows which block the
// bytes about to arrive belong to.
uint16_t pending_download_block = 0;
uint16_t pending_download_length = 0;
bool download_data_pending = false;

// Stage an IN response into the control buffer and tell the stack how much of it
// to send. Truncating to the host's wLength is required by the spec.
uint8_t reply(const void* data, uint16_t size) {
    if (USBSS_SetupReqLen > size)
        USBSS_SetupReqLen = size;
    std::memcpy(USBSS_EP0_Buf, data, USBSS_SetupReqLen);
    return kAccept;
}

} // namespace
} // namespace libhcs::firmware::usb

extern "C" {

uint8_t usb_ss_class_setup(void) {
    namespace usb = libhcs::firmware::usb;

    if ((USBSS_SetupReqType & usb::kRequestTypeMask) != usb::kRequestTypeClass)
        return usb::kStall;
    if ((USBSS_SetupReqType & usb::kRequestRecipientMask) != usb::kRequestRecipientInterface)
        return usb::kStall;
    if ((USBSS_SetupReqIndex & 0xFF) != usb::kDfuInterfaceNumber)
        return usb::kStall;

    usb::download_data_pending = false;

    switch (static_cast<usb::DfuRequest>(USBSS_SetupReqCode)) {
    case usb::DfuRequest::kDownload: {
        if (USBSS_SetupReqType & usb::kRequestDirectionIn)
            return usb::kStall;

        // A zero-length DNLOAD is the end-of-transfer marker and carries no data
        // stage, so it is completed here rather than in usb_ss_class_ep0_out().
        if (USBSS_SetupReqLen == 0)
            return usb::dfu.download(USBSS_SetupReqValue, {}) ? usb::kAccept : usb::kStall;

        if (USBSS_SetupReqLen > DEF_USBSSD_UEP0_SIZE)
            return usb::kStall;

        usb::pending_download_block = USBSS_SetupReqValue;
        usb::pending_download_length = USBSS_SetupReqLen;
        usb::download_data_pending = true;
        return usb::kAccept;
    }

    case usb::DfuRequest::kGetStatus: {
        if (!(USBSS_SetupReqType & usb::kRequestDirectionIn))
            return usb::kStall;
        const auto status = usb::dfu.get_status();
        return usb::reply(&status, sizeof(status));
    }

    case usb::DfuRequest::kGetState: {
        if (!(USBSS_SetupReqType & usb::kRequestDirectionIn))
            return usb::kStall;
        const auto state = static_cast<uint8_t>(usb::dfu.state());
        return usb::reply(&state, sizeof(state));
    }

    case usb::DfuRequest::kClearStatus: return usb::dfu.clear_status() ? usb::kAccept : usb::kStall;

    case usb::DfuRequest::kAbort: return usb::dfu.abort() ? usb::kAccept : usb::kStall;

    case usb::DfuRequest::kDetach:
        // bitWillDetach = 1: acknowledge, then the main loop resets once the
        // status stage has been delivered.
        usb::dfu.detach();
        return usb::kAccept;

    case usb::DfuRequest::kUpload:
        // bitCanUpload = 0. Readback of the application slot is deliberately not
        // offered; answering with zero bytes ends the transfer cleanly.
        if (!(USBSS_SetupReqType & usb::kRequestDirectionIn))
            return usb::kStall;
        USBSS_SetupReqLen = 0;
        return usb::kAccept;

    default: return usb::kStall;
    }
}

// The bootloader speaks DFU on EP0 only; the bulk pipe belongs to the
// application. These satisfy the patched ISR's references without pulling the
// libhcs protocol stack into the boot image.
void usb_ss_ep1_in_complete(void) {}
void usb_ss_ep1_out_complete(void) {}

void usb_ss_class_ep0_out(void) {
    namespace usb = libhcs::firmware::usb;

    if (!usb::download_data_pending)
        return;
    usb::download_data_pending = false;

    (void)usb::dfu.download(
        usb::pending_download_block,
        std::span<const uint8_t>{USBSS_EP0_Buf, usb::pending_download_length});
}

} // extern "C"
