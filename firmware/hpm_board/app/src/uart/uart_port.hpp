#pragma once

#include <cstdint>

#include <hpm_dmamux_src.h> // 板级表使用的 HPM_DMA_SRC_UARTx_TX/RX 宏
#include <hpm_uart_drv.h>

#include "core/include/libhcs/data/datas.hpp"

namespace libhcs::firmware::board {

// 板卡对外暴露的一个 UART, 按逻辑顺序排列。DBUS 接收器只是 data_id ==
// kUartDbus、自带波特率/校验的普通条目; 公共 UART 层全靠本表构建, 因此
// 不存在逐端口的宏。
struct UartPort {
    uint32_t base;    // 寄存器基址, 即 HPM_UARTx_BASE
    uint32_t irq_num; // 中断号, 即 IRQn_UARTx
    uint32_t dma_src_tx;
    uint32_t dma_src_rx;
    data::DataId data_id;
    // 承载本端口运行时配置(波特率)的下行 id。与 data_id 配对, 其中
    // kUart0 -> kUart0Config, kUartDbus -> kUartDbusConfig。
    data::DataId config_data_id;
    uint32_t baudrate;
    parity_setting_t parity;
};

} // namespace libhcs::firmware::board
