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
// 只有主 vendor 接口一对 bulk 端点。曾为 CAN 单开一对(2026-08-07 加,
// 2026-09-05 移除): 空闲 IN 端点被主机持续轮询, 包率损失四分之一, 超过它消除的
// 队头阻塞, 详见 usb_descriptors.hpp。也刻意不用 TinyUSB 0.21 的 interrupt 端点
// 对 -- interrupt 端点每 bInterval 才被轮询一次, 高速下会把上行量化到 125 us
// 微帧, 而 bulk 保持连续轮询, 只是不再共享队列。

#define CFG_TUD_CDC  0
#define CFG_TUD_MSC  0
#define CFG_TUD_HID  0
#define CFG_TUD_MIDI 0
// 一个 vendor 接口: 主 bulk 管道。2026-08-07 曾在此为 CAN 加第二个接口、经 EP0
// 的 kSetEndpointMode(0x46, 已废弃)运行时启用, 因空闲 IN 轮询的代价于 2026-09-05
// 移除, 见 usb_descriptors.hpp。
#define CFG_TUD_VENDOR      1
#define CFG_TUD_DFU_RUNTIME 1
#define CFG_TUD_DFU         0

#define CFG_TUD_VENDOR_EPSIZE 512

// vendor TX/RX FIFO 大小(0 表示 direct mode)
// https://docs.tinyusb.org/en/latest/reference/usb_concepts.html#class-driver-types
#define CFG_TUD_VENDOR_RX_BUFSIZE 0
#define CFG_TUD_VENDOR_TX_BUFSIZE 0

#ifdef __cplusplus
}
#endif
