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
    // 端口身份(丝印号), 取自板型 spec 的具名描述符; ports.hpp 的绑定核对它与 EP0 的
    // 寻址是同一个口。
    data::DataId data_id;
    uint32_t baudrate;
    parity_setting_t parity;
};

} // namespace libhcs::firmware::board
