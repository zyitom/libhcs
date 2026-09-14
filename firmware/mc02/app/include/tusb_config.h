#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// 通用配置
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
# error CFG_TUSB_MCU must be defined
#endif

// STM32H723 只有 USB_OTG_HS; 当 USB2_OTG_FS 不存在时, TinyUSB 把
// USB_OTG_FS_PERIPH_BASE 别名到 USB1_OTG_HS_PERIPH_BASE, 将其重映射为 rhport 0。
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
// 设备配置
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

// 一次 OUT 传输请求的字节数。这不是端点的 wMaxPacketSize: 后者在描述符中
// (usb_descriptors.hpp)保持 64 -- USB 规范对 Full-Speed bulk 端点的上限, 调不高。
//
// 两者可以分离, 是因为 vendor_device.c 只把 CFG_TUD_VENDOR_RX_EPSIZE 用作端点
// 缓冲, 以及在设置 CFG_TUD_VENDOR_RX_NEED_ZLP 时用作 rx_xfer_len -- 从不用于
// 描述符。请求 N x 64 字节能让 DWC2 在一次传输内接收 N 个背靠背包, 设备重挂端点
// 期间不再逐包 NAK。
//
// 取 64 时刻意不开 NEED_ZLP 分支, rx_xfer_len 便回退为 tu_edpt_packet_size(),
// 构建与旧有行为逐位一致 -- 这是 A/B 对照的基线, 不是对它的近似。
#ifndef libhcs_APP_USB_RX_XFER_SIZE
# define libhcs_APP_USB_RX_XFER_SIZE 64
#endif

#define CFG_TUD_VENDOR_RX_EPSIZE libhcs_APP_USB_RX_XFER_SIZE
#if libhcs_APP_USB_RX_XFER_SIZE > 64
# define CFG_TUD_VENDOR_RX_NEED_ZLP 1

// 不让类驱动自行重挂 bulk OUT 端点。由应用在主循环重挂, 并在 CAN 软件发送队列
// 接近满时扣住不放: 过载的板子对主机回 NAK, 而不是收下只能丢弃的帧。挂载策略、
// 迟滞水位与 20ms 逃生阀见 app/src/usb/vendor.hpp 的下行流控一节。
// (此开关来自本仓库 TinyUSB fork; 置 0 时类驱动自行重挂, 整个流控编译消失。)
# define CFG_TUD_VENDOR_RX_MANUAL_XFER 0
#endif

#define CFG_TUD_VENDOR_TX_EPSIZE 64

// 直连模式, 匹配既有的逐包组帧行为: 两个 FIFO 尺寸置 0 使 TinyUSB 推导出
// CFG_TUD_VENDOR_TXRX_BUFFERED == 0, tud_vendor_n_write() 把每个包直接提交给
// 端点, 零长写即成 ZLP。注意这同时编译掉了 FIFO 专用 API, 如
// tud_vendor_n_read/_write_flush 等。
#define CFG_TUD_VENDOR_RX_BUFSIZE 0
#define CFG_TUD_VENDOR_TX_BUFSIZE 0

#ifdef __cplusplus
}
#endif
