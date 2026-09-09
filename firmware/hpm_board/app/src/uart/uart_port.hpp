#pragma once

#include <cstdint>

#include <hpm_dmamux_src.h> // HPM_DMA_SRC_UARTx_TX/RX for board tables
#include <hpm_uart_drv.h>

#include "core/include/libhcs/data/datas.hpp"

namespace libhcs::firmware::board {

// One UART exposed by a board, in logical order. The DBUS receiver is just an
// entry with data_id == kUartDbus and its own baudrate/parity; the shared UART
// layer builds everything from this table, so there are no per-port macros.
struct UartPort {
    uint32_t base;    // HPM_UARTx_BASE
    uint32_t irq_num; // IRQn_UARTx
    uint32_t dma_src_tx;
    uint32_t dma_src_rx;
    data::DataId data_id;
    // Downlink id that carries runtime configuration (baudrate) for this port.
    // Pairs with data_id: kUart0 -> kUart0Config, kUartDbus -> kUartDbusConfig.
    data::DataId config_data_id;
    uint32_t baudrate;
    parity_setting_t parity;
};

} // namespace libhcs::firmware::board
