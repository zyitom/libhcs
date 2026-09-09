#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <usart.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/led/led.hpp"
#include "firmware/c_board/app/src/uart/rx_buffer.hpp"
#include "firmware/c_board/app/src/uart/tx_buffer.hpp"
#include "firmware/c_board/app/src/usb/helper.hpp"
#include "firmware/c_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::uart {

class Uart
    : private core::utility::Immovable
    , private TxBuffer
    , private RxBuffer<Uart> {
    friend class RxBuffer<Uart>;

public:
    using Lazy = utility::Lazy<Uart, data::DataId, UART_HandleTypeDef*>;

    Uart(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle)
        : TxBuffer(hal_uart_handle, &hal_tx_dma_complete_callback, &hal_tx_dma_error_callback)
        , RxBuffer(hal_uart_handle)
        , data_id_(data_id)
        , hal_uart_handle_(hal_uart_handle) {}

    // Runtime baudrate switch requested by the host. Writes the divisor register
    // directly instead of re-running HAL_UART_Init: that keeps every other
    // setting exactly as CubeMX generated it (this code must never reconfigure
    // the peripheral behind the .ioc) and avoids tearing down the running DMA.
    //
    // RX bytes arriving inside the switch window may be garbled; the host is
    // expected to quiesce the link first.
    bool handle_config(const data::UartConfigView& data) {
        if (!data.baudrate.has_value() || *data.baudrate == 0) [[unlikely]]
            return false;

        hal_uart_handle_->Init.BaudRate = *data.baudrate;
        hal_uart_handle_->Instance->BRR = compute_brr(*data.baudrate);
        return true;
    }

    void handle_downlink(const data::UartDataView& data) {
        if (!TxBuffer::try_enqueue(data))
            led::led->downlink_buffer_full();
    }

    void try_transmit() {
        RxBuffer::try_dequeue();
        TxBuffer::try_dequeue();
    }

    void tx_complete_callback() { TxBuffer::tx_complete_callback(); }

    void uart_error_callback() {
        constexpr uint32_t rx_error_mask =
            HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE;

        if ((hal_uart_handle_->ErrorCode & rx_error_mask) != 0U)
            RxBuffer::rx_error_callback();
    }

    void rx_dma_tc_callback() { RxBuffer::dma_tc_callback(); }

    void rx_dma_error_callback() {
        hal_uart_handle_->ErrorCode |= HAL_UART_ERROR_DMA;
        RxBuffer::rx_error_callback();
    }

    void tx_dma_error_callback() { TxBuffer::tx_error_callback(); }

    void rx_event_callback() { RxBuffer::uart_idle_event_callback(); }

private:
    static void hal_rx_dma_tc_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_complete_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    void handle_uplink(
        std::span<const std::byte> payload, std::span<const std::byte> payload2, bool is_idle) {
        auto& serializer = usb::get_serializer();
        core::utility::assert_always(
            serializer.write_uart(
                data_id_, {.uart_data = payload, .idle_delimited = is_idle}, payload2)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    [[nodiscard]] uint32_t compute_brr(uint32_t baudrate) const {
        const uint32_t peripheral_clock_hz_value = peripheral_clock_hz();
        if (hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8)
            return UART_BRR_SAMPLING8(peripheral_clock_hz_value, baudrate);
        return UART_BRR_SAMPLING16(peripheral_clock_hz_value, baudrate);
    }

    // STM32F407: USART1 and USART6 sit on APB2, every other U(S)ART on APB1.
    [[nodiscard]] uint32_t peripheral_clock_hz() const {
        if (hal_uart_handle_ == &huart1 || hal_uart_handle_ == &huart6)
            return HAL_RCC_GetPCLK2Freq();
        return HAL_RCC_GetPCLK1Freq();
    }

    data::DataId data_id_;
    UART_HandleTypeDef* hal_uart_handle_;
};

inline constinit Uart::Lazy uart1{data::DataId::kUart1, &huart6};
inline constinit Uart::Lazy uart2{data::DataId::kUart2, &huart1};
inline constinit Uart::Lazy uart_dbus{data::DataId::kUartDbus, &huart3};

} // namespace libhcs::firmware::uart
