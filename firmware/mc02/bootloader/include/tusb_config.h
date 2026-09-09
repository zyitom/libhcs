#pragma once

#ifdef __cplusplus
extern "C" {
#endif

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
#elif BOARD_DEVICE_RHPORT_NUM == 1
# define CFG_TUSB_RHPORT1_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#else
# error "Incorrect RHPort configuration"
#endif

#define CFG_TUSB_OS OPT_OS_NONE

#ifndef CFG_TUD_ENDPOINT0_SIZE
# define CFG_TUD_ENDPOINT0_SIZE 64
#endif

#define CFG_TUD_CDC         0
#define CFG_TUD_MSC         0
#define CFG_TUD_HID         0
#define CFG_TUD_MIDI        0
#define CFG_TUD_VENDOR      0
#define CFG_TUD_DFU_RUNTIME 0
#define CFG_TUD_DFU         1

#define CFG_TUD_DFU_XFER_BUFSIZE 1024

#ifdef __cplusplus
}
#endif
