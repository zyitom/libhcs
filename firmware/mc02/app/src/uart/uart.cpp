#include "firmware/mc02/app/src/uart/uart.hpp"

#include <usart.h>

#include "core/src/utility/assert.hpp"

namespace libhcs::firmware::uart {

namespace {

// USART2 / USART3 are UartRs485, a different type, and are reached through
// get_rs485_instance_from_dma() below for the same reason UART5 is reached
// through get_dbus_instance_from_dma().
Uart& get_uart_instance(UART_HandleTypeDef* hal_uart_handle) {
    if (hal_uart_handle == &huart1)
        return *uart1;

    if (hal_uart_handle == &huart7)
        return *uart7;

    if (hal_uart_handle == &huart10)
        return *uart10;

    core::utility::assert_failed_debug();
}

Uart& get_uart_instance_from_dma(DMA_HandleTypeDef* hal_dma_handle) {
    auto* hal_uart_handle = static_cast<UART_HandleTypeDef*>(hal_dma_handle->Parent);
    return get_uart_instance(hal_uart_handle);
}

UartRxOnly& get_dbus_instance_from_dma(DMA_HandleTypeDef* hal_dma_handle) {
    auto* hal_uart_handle = static_cast<UART_HandleTypeDef*>(hal_dma_handle->Parent);
    core::utility::assert_debug(hal_uart_handle == &huart5);
    return *uart_dbus;
}

#ifdef libhcs_APP_RS485_ENABLE
UartRs485& get_rs485_instance(UART_HandleTypeDef* hal_uart_handle) {
    if (hal_uart_handle == &huart2)
        return *uart2;

    core::utility::assert_debug(hal_uart_handle == &huart3);
    return *uart3;
}

UartRs485& get_rs485_instance_from_dma(DMA_HandleTypeDef* hal_dma_handle) {
    return get_rs485_instance(static_cast<UART_HandleTypeDef*>(hal_dma_handle->Parent));
}
#endif

} // namespace

void Uart::hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle) {
    get_uart_instance_from_dma(hal_dma_handle).rx_dma_error_callback();
}

void Uart::hal_tx_dma_complete_callback(DMA_HandleTypeDef* hal_dma_handle) {
    get_uart_instance_from_dma(hal_dma_handle).tx_complete_callback();
}

void Uart::hal_tx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle) {
    get_uart_instance_from_dma(hal_dma_handle).tx_dma_error_callback();
}

void UartRxOnly::hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle) {
    get_dbus_instance_from_dma(hal_dma_handle).rx_dma_error_callback();
}

#ifdef libhcs_APP_RS485_ENABLE
void UartRs485::hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle) {
    get_rs485_instance_from_dma(hal_dma_handle).rx_dma_error_callback();
}

void UartRs485::hal_tx_dma_complete_callback(DMA_HandleTypeDef* hal_dma_handle) {
    get_rs485_instance_from_dma(hal_dma_handle).tx_complete_callback();
}

void UartRs485::hal_tx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle) {
    get_rs485_instance_from_dma(hal_dma_handle).tx_dma_error_callback();
}
#endif

// NOLINTNEXTLINE(readability-inconsistent-declaration-parameter-name)
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* hal_uart_handle) {
    if (hal_uart_handle == &huart5) {
        uart_dbus->uart_error_callback();
        return;
    }

#ifdef libhcs_APP_RS485_ENABLE
    if (hal_uart_handle == &huart2 || hal_uart_handle == &huart3) {
        get_rs485_instance(hal_uart_handle).uart_error_callback();
        return;
    }
#endif

    get_uart_instance(hal_uart_handle).uart_error_callback();
}

// NOLINTNEXTLINE(readability-inconsistent-declaration-parameter-name)
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* hal_uart_handle, uint16_t size) {
    (void)size;

    // The HAL raises this for both IDLE and DMA transfer-complete events. The
    // stream is circular over the whole ring and wraps on its own, so a transfer
    // completion carries no information; only the IDLE event does.
    if (HAL_UARTEx_GetRxEventType(hal_uart_handle) != HAL_UART_RXEVENT_IDLE)
        return;

    if (hal_uart_handle == &huart5) {
        uart_dbus->rx_event_callback();
        return;
    }

#ifdef libhcs_APP_RS485_ENABLE
    if (hal_uart_handle == &huart2 || hal_uart_handle == &huart3) {
        get_rs485_instance(hal_uart_handle).rx_event_callback();
        return;
    }
#endif

    get_uart_instance(hal_uart_handle).rx_event_callback();
}

} // namespace libhcs::firmware::uart
