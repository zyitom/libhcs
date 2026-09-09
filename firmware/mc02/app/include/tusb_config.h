#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
# error CFG_TUSB_MCU must be defined
#endif

// STM32H723 has only USB_OTG_HS; TinyUSB remaps it to rhport 0 by aliasing
// USB_OTG_FS_PERIPH_BASE -> USB1_OTG_HS_PERIPH_BASE when USB2_OTG_FS is absent.
#ifndef BOARD_DEVICE_RHPORT_NUM
# define BOARD_DEVICE_RHPORT_NUM 0
#endif

#ifndef BOARD_DEVICE_RHPORT_SPEED
# define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_FULL_SPEED
#endif

#if BOARD_DEVICE_RHPORT_NUM == 0
# define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#else
# error "Incorrect RHPort configuration"
#endif

#define CFG_TUSB_OS OPT_OS_NONE

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
# define CFG_TUD_ENDPOINT0_SIZE 64
#endif

#define CFG_TUD_CDC         0
#define CFG_TUD_MSC         0
#define CFG_TUD_HID         0
#define CFG_TUD_MIDI        0
#define CFG_TUD_VENDOR      1
#define CFG_TUD_DFU_RUNTIME 1
#define CFG_TUD_DFU         0

// How many bytes one OUT transfer requests. This is NOT the endpoint's
// wMaxPacketSize: that stays 64 in the descriptor (usb_descriptors.hpp), which is
// the USB-spec maximum for a Full-Speed bulk endpoint and cannot be raised.
//
// The two are separable because vendor_device.c only ever uses
// CFG_TUD_VENDOR_RX_EPSIZE for the endpoint buffer and, when
// CFG_TUD_VENDOR_RX_NEED_ZLP is set, for rx_xfer_len -- never for the
// descriptor. Asking for N x 64 bytes lets DWC2 accept N back-to-back packets
// per transfer, so the device stops NAKing between packets while it re-arms.
//
// At 64 the NEED_ZLP branch is deliberately left off, so rx_xfer_len falls back
// to tu_edpt_packet_size() and the build is bit-for-bit the old behaviour --
// that is the A/B baseline, not an approximation of it.
#ifndef libhcs_APP_USB_RX_XFER_SIZE
# define libhcs_APP_USB_RX_XFER_SIZE 64
#endif

#define CFG_TUD_VENDOR_RX_EPSIZE libhcs_APP_USB_RX_XFER_SIZE
#if libhcs_APP_USB_RX_XFER_SIZE > 64
# define CFG_TUD_VENDOR_RX_NEED_ZLP 1
#endif

#define CFG_TUD_VENDOR_TX_EPSIZE 64

// Direct mode to match the existing per-packet framing behavior: zeroing both FIFO
// sizes makes TinyUSB derive CFG_TUD_VENDOR_TXRX_BUFFERED == 0, so tud_vendor_n_write()
// submits each packet straight to the endpoint and a zero-length write becomes a ZLP.
// Note this also compiles out the FIFO-only API (tud_vendor_n_read/_write_flush/...).
#define CFG_TUD_VENDOR_RX_BUFSIZE 0
#define CFG_TUD_VENDOR_TX_BUFSIZE 0

#ifdef __cplusplus
}
#endif
