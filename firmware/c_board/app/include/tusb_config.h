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

#ifndef BOARD_DEVICE_RHPORT_NUM
# define BOARD_DEVICE_RHPORT_NUM 0
#endif

#ifndef BOARD_DEVICE_RHPORT_SPEED
# define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_FULL_SPEED
#endif

#if BOARD_DEVICE_RHPORT_NUM == 0
# define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#elif BOARD_DEVICE_RHPORT_NUM == 1
# define CFG_TUSB_RHPORT1_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
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

#define CFG_TUD_VENDOR_RX_EPSIZE 64
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
