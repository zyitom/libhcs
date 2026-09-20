#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// 通用配置
//--------------------------------------------------------------------

// 由 board.mk 定义
#ifndef CFG_TUSB_MCU
# error CFG_TUSB_MCU must be defined
#endif

// 设备使用的 RHPort 编号, 可由 board.mk 定义, 默认端口 0
#ifndef BOARD_DEVICE_RHPORT_NUM
# define BOARD_DEVICE_RHPORT_NUM 0
#endif

// RHPort 最高运行速率, 可由 board.mk 定义。
// 默认: 带内部高速 PHY(可能因端口而异)的 MCU 用 Highspeed, 否则用 FullSpeed
#ifndef BOARD_DEVICE_RHPORT_SPEED
# define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_HIGH_SPEED
#endif

// 设备模式, rhport 与速率由 board.mk 定义
#if BOARD_DEVICE_RHPORT_NUM == 0
# define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#elif BOARD_DEVICE_RHPORT_NUM == 1
# define CFG_TUSB_RHPORT1_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#else
# error "Incorrect RHPort configuration"
#endif

// 不使用 RTOS
#define CFG_TUSB_OS OPT_OS_NONE

// DEBUG 构建中 CFG_TUSB_DEBUG 由编译器定义
// #define CFG_TUSB_DEBUG           0

/* 部分 MCU 的 USB DMA 只能访问特定 SRAM 区域, 且有对齐限制。
 * TinyUSB 用下列宏声明传输内存, 使其可放进这些特定 section, 例如
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
// 设备配置
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
# define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- 类驱动 -------------//
// libhcs 仍是唯一的 vendor 类实例(实例 0), 让到接口 3 / 0x04 / 0x84
// (libhcs/protocol/usb_identity.hpp)。DMTool 仿真的接口 0-2(端点 0x01-0x03 /
// 0x81-0x83, 由 DMTool 写死)由 dmtool 的应用类驱动承接(usbd_app_driver_get_cb),
// 不占 vendor 实例: libhcs 的 bulk 回调与发送路径因此与没有 DMTool 时逐条相同。
// CDC = 板上 UART 的 USB 串口桥。多出的端点对 libhcs 零成本: bulk 端点只在主机
// 挂着传输时才被调度, 见 usb_descriptors.hpp。也刻意不用 TinyUSB 0.21 的
// interrupt 端点对做数据 -- interrupt 端点每 bInterval 才被轮询一次, 高速下会把
// 上行量化到 125 us 微帧。

#define CFG_TUD_CDC         1
#define CFG_TUD_MSC         0
#define CFG_TUD_HID         0
#define CFG_TUD_MIDI        0
#define CFG_TUD_VENDOR      1
#define CFG_TUD_DFU_RUNTIME 1
#define CFG_TUD_DFU         0

#define CFG_TUD_VENDOR_EPSIZE 512

// vendor TX/RX FIFO 大小(0 表示 direct mode)
// https://docs.tinyusb.org/en/latest/reference/usb_concepts.html#class-driver-types
#define CFG_TUD_VENDOR_RX_BUFSIZE 0
#define CFG_TUD_VENDOR_TX_BUFSIZE 0

// CDC 串口桥: FIFO 与端点缓冲都按高速 bulk 包长 512 取。
#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 512
#define CFG_TUD_CDC_EP_BUFSIZE 512

#ifdef __cplusplus
}
#endif
