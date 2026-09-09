#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// defined by board.mk
#ifndef CFG_TUSB_MCU
# error CFG_TUSB_MCU must be defined
#endif

// RHPort number used for device can be defined by board.mk, default to port 0
#ifndef BOARD_DEVICE_RHPORT_NUM
# define BOARD_DEVICE_RHPORT_NUM 0
#endif

// RHPort max operational speed can defined by board.mk
// Default to Highspeed for MCU with internal HighSpeed PHY (can be port specific), otherwise
// FullSpeed
#ifndef BOARD_DEVICE_RHPORT_SPEED
# define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_HIGH_SPEED
#endif

// Device mode with rhport and speed defined by board.mk
#if BOARD_DEVICE_RHPORT_NUM == 0
# define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#elif BOARD_DEVICE_RHPORT_NUM == 1
# define CFG_TUSB_RHPORT1_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#else
# error "Incorrect RHPort configuration"
#endif

// This example doesn't use an RTOS
#define CFG_TUSB_OS OPT_OS_NONE

// CFG_TUSB_DEBUG is defined by compiler in DEBUG build
// #define CFG_TUSB_DEBUG           0

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
# define CFG_TUSB_MEM_SECTION __attribute__((section(".noncacheable.non_init")))
#endif

#ifndef CFG_TUSB_MEM_ALIGN
# define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
# define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
// Give CAN its own bulk endpoint pair, on a second vendor interface, so a UART
// packet can no longer sit ahead of a CAN packet in the host controller's
// per-endpoint queue. Measured cost of sharing one pipe: a CAN frame that
// collides with UART data waits ~40 us extra, taking p99 from 104.5 to 143.9 us
// at UART line rate -- and only 25 kB/s (27% of line rate) is needed to inflict
// nine tenths of that. See hpm_board/AGENTS.md.
//
// Deliberately NOT the interrupt endpoint pair TinyUSB 0.21 also offers: an
// interrupt endpoint is polled once per bInterval, so at high speed it would
// quantize the uplink to a 125 us microframe. A second BULK pair keeps the
// existing continuous-poll behaviour and only stops sharing the queue.

#define CFG_TUD_CDC 0
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
// Two vendor interfaces: the main bulk pipe, plus a second pair that carries
// CAN uplink. Whether the second pair is USED is negotiated at run time over
// EP0 (kSetEndpointMode), so this is no longer a build-time choice -- the pipe
// costs nothing while the host declines to post URBs on it, and having it
// always present is what lets one image serve both workloads.
#define CFG_TUD_VENDOR 1
#define CFG_TUD_DFU_RUNTIME 1
#define CFG_TUD_DFU         0

#define CFG_TUD_VENDOR_EPSIZE 512

// Vendor FIFO size of TX and RX (0 means direct mode)
// https://docs.tinyusb.org/en/latest/reference/usb_concepts.html#class-driver-types
#define CFG_TUD_VENDOR_RX_BUFSIZE 0
#define CFG_TUD_VENDOR_TX_BUFSIZE 0

// Do not let the class driver re-arm the bulk OUT endpoint on its own. The
// application re-arms it from the main loop, and withholds that while the CAN
// software transmit queues are close to full, so an overloaded board answers the
// host with NAK instead of accepting frames it can only drop. See the arming
// policy and its bounded-stall escape in app/src/usb/vendor.hpp.
#define CFG_TUD_VENDOR_RX_MANUAL_XFER 1

#ifdef __cplusplus
}
#endif
