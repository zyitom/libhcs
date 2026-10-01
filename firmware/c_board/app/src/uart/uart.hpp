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

    // ---- EP0 configuration channel: runtime rate + framing, and read-back ----
    //
    // Same contract as the other boards (core vendor_control.hpp): validate
    // everything before touching a register, so a control-transfer STALL means
    // the port is exactly as it was; and read back what the hardware actually
    // runs, never the last request. This board had neither half before -- only
    // a write-only handle_config(), which is the failure mode the migration
    // exists to remove.

    // Pure validation: no register is touched. Encoding matches
    // core::protocol::vendor_control's UartParity/UartStopBits (0 = skip;
    // parity 1=none 2=even 3=odd; stop bits 1=1 2=2); word length is the data
    // bit count (7/8; 9 is not offered -- the RX path is a byte-wide DMA ring
    // and a ninth bit would be silently truncated). 1.5 stop bits is not
    // offered: STM32 realizes it only for a 5-bit word, where it is
    // indistinguishable from 2 -- an offered alias would lie. RX polarity only
    // accepts 0 (skip) and 1 (normal): the F4 USART has no RX inversion bit, so
    // an inverted request must stall rather than pretend to apply.
    //
    // The F4 frame (M) is 8 or 9 bits INCLUDING the parity bit, so 7 data bits
    // exist only with parity (7E/7O = 8-bit frame); 7N1 would need a 7-bit
    // frame this USART does not have. Checked on the resolved pair (a zero
    // field stands for the port's current value), same as commit_framing().
    [[nodiscard]] bool check_framing(
        uint32_t word_length, uint32_t parity, uint32_t stop_bits, uint32_t rx_polarity) const {
        if (rx_polarity > 1U)
            return false;
        if (word_length != 0U && word_length != 7U && word_length != 8U)
            return false;
        if (parity != 0U && (parity < 1U || parity > 3U))
            return false;
        if (stop_bits != 0U && stop_bits != 1U && stop_bits != 2U)
            return false;
        const uint32_t data_bits = word_length != 0U ? word_length : this->word_length();
        const uint32_t parity_code = parity != 0U ? parity : this->parity();
        return data_bits != 7U || parity_code != 1U;
    }

    // Commits a framing that check_framing() already accepted -- so there is no
    // failure path here, which is what makes "STALL means nothing changed"
    // mechanical. M/PCE/PS/STOP are only writable with UE=0, so the write is
    // one down-then-up window; bytes arriving inside it are lost, and the host
    // is expected to quiesce the link first (same convention as the rate).
    //
    // M counts the whole frame (data + parity), so 8 data bits with even parity
    // is a 9-bit frame (M=1) -- the WORDLENGTH_9B + PARITY_EVEN CubeMX gives
    // DBUS. Word length and parity are therefore resolved together: a zero
    // field stands for the port's current value.
    void commit_framing(uint32_t word_length, uint32_t parity, uint32_t stop_bits) {
        const uint32_t data_bits = word_length != 0U ? word_length : this->word_length();
        const uint32_t parity_code = parity != 0U ? parity : this->parity();
        uint32_t parity_bits = 0;
        switch (parity_code) {
        case 2U: parity_bits = USART_CR1_PCE; break;                // even
        case 3U: parity_bits = USART_CR1_PCE | USART_CR1_PS; break; // odd
        default: break;                                             // none
        }
        const uint32_t frame_bits = data_bits + (parity_bits != 0U ? 1U : 0U);
        const uint32_t m_bits = frame_bits == 9U ? USART_CR1_M : 0U;
        uint32_t stop_reg = 0;
        if (stop_bits == 2U)
            stop_reg = USART_CR2_STOP_1;

        auto* instance = hal_uart_handle_->Instance;
        const uint32_t new_cr1 =
            (instance->CR1 & ~(USART_CR1_UE | USART_CR1_M | USART_CR1_PCE | USART_CR1_PS)) | m_bits
            | parity_bits;

        instance->CR1 = new_cr1 & ~USART_CR1_UE; // UE=0: framing fields writable
        if (stop_bits != 0U)
            instance->CR2 = (instance->CR2 & ~USART_CR2_STOP) | stop_reg;
        instance->CR1 = new_cr1; // UE restored
    }

    // ---- Read-back: decoded from the live registers, never the last request ----

    // Data bits = the frame M gives (8 or 9) minus the parity bit. DBUS's 9-bit
    // frame with even parity reads back as 8.
    [[nodiscard]] uint32_t word_length() const {
        const uint32_t cr1 = hal_uart_handle_->Instance->CR1;
        const uint32_t frame_bits = (cr1 & USART_CR1_M) != 0U ? 9U : 8U;
        return frame_bits - ((cr1 & USART_CR1_PCE) != 0U ? 1U : 0U);
    }

    [[nodiscard]] uint32_t parity() const {
        const uint32_t cr1 = hal_uart_handle_->Instance->CR1;
        if ((cr1 & USART_CR1_PCE) == 0U)
            return 1U;                               // none
        return (cr1 & USART_CR1_PS) != 0U ? 3U : 2U; // odd : even
    }

    [[nodiscard]] uint32_t stop_bits() const {
        return (hal_uart_handle_->Instance->CR2 & USART_CR2_STOP) == USART_CR2_STOP_1 ? 2U : 1U;
    }

    // No RX inversion hardware: always normal (1).
    [[nodiscard]] static uint32_t rx_polarity() { return 1U; }

    // The rate the port is ACTUALLY running, reconstructed from BRR rather than
    // from Init.BaudRate: the latter is only the last requested value, and a
    // rejected request leaves both unwritten. Reading BRR is what lets the host
    // tell "the switch landed" from "the switch was refused, still on the old
    // rate".
    [[nodiscard]] uint32_t effective_baudrate() const {
        return baudrate_for(hal_uart_handle_->Instance->BRR & 0xFFFFU);
    }

    // 给定 BRR 值在本口时钟下的速率; EP0 用它在写入前核对 solve_brr() 的结果。
    [[nodiscard]] uint32_t baudrate_for(uint32_t brr) const {
        const uint32_t kernel_clock_hz = peripheral_clock_hz();
        if (kernel_clock_hz == 0U || brr == 0U) [[unlikely]]
            return 0;
        if (hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8) {
            // 8x oversampling records BRR[3] as zero and shifts the low nibble
            // right by one; undo that before dividing.
            const uint32_t usartdiv = (brr & 0xFFF0U) | ((brr & 0x0007U) << 1U);
            return usartdiv ? (2U * kernel_clock_hz) / usartdiv : 0U;
        }
        return kernel_clock_hz / brr;
    }

    // The two integers the rate is made of, for the EP0 GET report and the SET
    // assertion. `divisor` is BRR itself, un-normalized -- the very integer
    // commit_brr()/compute_brr() wrote, so a read-back comparison needs no
    // conversion. See UartDivisor.
    [[nodiscard]] uint16_t divisor_u16() const {
        return static_cast<uint16_t>(hal_uart_handle_->Instance->BRR & 0xFFFFU);
    }

    [[nodiscard]] uint8_t oversample() const {
        return hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8 ? 8U : 16U;
    }

    // Solves the divisor without writing it: the validate half of the
    // validate-then-commit pair. compute_brr() is the same arithmetic, so the
    // value returned here is exactly what apply_baudrate() will program.
    [[nodiscard]] bool solve_brr(uint32_t baudrate, uint32_t& brr) const {
        if (baudrate == 0U) [[unlikely]]
            return false;
        if (peripheral_clock_hz() == 0U) [[unlikely]]
            return false;
        brr = compute_brr(baudrate);
        // Same bounds the other boards enforce: a divisor of 0 or one that
        // overflows the register means the rate is not representable here.
        return brr >= 0x10U && brr <= 0xFFFFU;
    }

    // The commit half. Writes the divisor register directly instead of
    // re-running HAL_UART_Init: that keeps every other setting exactly as
    // CubeMX generated it (this code must never reconfigure the peripheral
    // behind the .ioc) and avoids tearing down the running DMA.
    void commit_brr(uint32_t baudrate, uint32_t brr) {
        hal_uart_handle_->Init.BaudRate = baudrate;
        hal_uart_handle_->Instance->BRR = brr;
    }

    // Write-then-read-back: confirms the register took the programming. The
    // value compared is the divisor integer, not the baudrate -- the host's
    // kernel clock differs from this board's, so the rate is not recomputable
    // on the host side. See UartDivisor.
    [[nodiscard]] bool
        verify_baudrate(uint16_t expected_divisor, uint8_t expected_oversample) const {
        return divisor_u16() == expected_divisor && oversample() == expected_oversample;
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
