#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <main.h>
#include <usart.h>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/utility/assert.hpp"

namespace libhcs::firmware::uart {

// Continuous DMA reception into a power-of-two ring.
//
// One circular DMA transfer spans the whole ring and is never stopped or
// re-armed: USART_CR3_DMAR stays set for the entire life of the port and the
// stream wraps by itself. That is the whole point of this class. The previous
// mc02 implementation used HAL_UARTEx_ReceiveToIdle_DMA with a one-shot
// (DMA_NORMAL) stream, and this HAL clears DMAR and aborts the stream *before*
// invoking the RX event callback -- see UART_DMAReceiveCplt() and the IDLE
// branch of HAL_UART_IRQHandler in
// bsp/stm32h7xx-hal-driver/Src/stm32h7xx_hal_uart.c. Reception was therefore off
// from that abort until the application re-armed it, and because the one-shot
// buffer was 64 bytes, any stream longer than 64 bytes hit that window once per
// 64 bytes and lost whatever arrived inside it.
//
// The write position is not maintained by an interrupt. NDTR counts down across
// the ring and reloads on wrap, so the consumer derives the position from it
// directly in try_dequeue(). c_board instead runs the stream double-buffered
// over 32-byte banks and takes an interrupt per bank to re-point the idle bank
// and publish the position -- an interrupt per 32 bytes whose only product is a
// number the consumer could have read for itself. Deriving it here has three
// consequences beyond deleting the bookkeeping:
//
//   - the deadline for servicing the stream widens from one bank (32 byte times)
//     to a full lap of the ring (kBufferSize byte times), because no bank
//     address has to be advanced before the hardware reaches it;
//   - the RX interrupt rate drops to the idle-event rate, from roughly 9000/s
//     per port at 921600 baud;
//   - the only place where a UART interrupt masked the FDCAN interrupt goes
//     away. The bank bookkeeping read and wrote several stream registers, so it
//     ran under a global __disable_irq(); FDCAN sits at NVIC priority 1 and its
//     forwarding path is the one this board spends .itcm/.dtcm on.
//
// Reception errors are absorbed rather than recovered from. STM32H7 can leave the
// DMA request unblocked across a line error (CR3.DDRE clear) and can stop raising
// overruns altogether (CR3.OVRDIS); see configure_rx_error_policy(), and read the
// polarity note there before touching either bit. STM32F407 has neither, so
// c_board is forced to tear the stream down and restart it every time a glitch
// arrives. Restarting is still implemented here for genuine DMA controller
// faults, and the restart path uses two more registers c_board does not have:
// ICR to drop sticky flags without a destructive RDR read, and RQR.RXFRQ to
// empty the 16-entry RXFIFO.
//
// The consumer runs in the main loop (try_dequeue), so the interrupt path only
// bumps a counter and never touches USB.
// buffer_size is the ring, and what it has to cover is one thing only: the
// consumer must come back before the DMA laps it. The default suits a port
// carrying a continuous stream. A port that only ever sees short
// request/response transactions needs far less, and on this board the ring is
// charged against a 32 KB region -- see the sizes chosen at the bottom of
// uart.hpp for the two RS-485 ports.
template <typename T, size_t buffer_size = 2048>
class RxBuffer {
    friend T;

public:
    static constexpr size_t kBufferSize = buffer_size;
    static constexpr size_t kBufferMask = kBufferSize - 1;
    static_assert((kBufferSize & (kBufferSize - 1)) == 0);
    using IndexType = uint16_t;
    static_assert(kBufferSize <= std::numeric_limits<IndexType>::max());

    static constexpr size_t kMinFragmentSize = 32;
    static constexpr size_t kProtocolMaxPayloadSize =
        core::protocol::kProtocolBufferSize - sizeof(core::protocol::UartHeaderExtended);
    static_assert(0 < kMinFragmentSize && kMinFragmentSize <= kProtocolMaxPayloadSize);
    // Below this the ring could not even hold the bytes try_dequeue() waits for
    // before forwarding a non-idle fragment, so it would stall instead of
    // batching. Half the ring is also where sample_write_position()'s
    // fallen-behind assert sits, so leave clear room under it.
    static_assert(kMinFragmentSize * 4 <= kBufferSize);

    bool try_dequeue() {
        // Sample the idle counter BEFORE the write position. The IDLE interrupt
        // publishes the counter at the instant the line went quiet, so a counter
        // read first is always paired with a write position at or after that
        // instant, and the chunk published below is guaranteed to contain the
        // boundary. Reading NDTR first could pair a stale position with an idle
        // event that happened after it and report a boundary for bytes that are
        // not in the ring yet.
        const auto idle_count = idle_count_.load(std::memory_order::acquire);
        const auto in = sample_write_position();
        const auto out = out_;
        const auto readable = static_cast<size_t>(static_cast<IndexType>(in - out));

        if (readable > kBufferSize) [[unlikely]] {
            // Abnormal condition: The circular queue has wrapped around.
            // Fail-fast in debug builds to catch timing/interrupt issues early.
            core::utility::assert_debug_lazy([]() noexcept { return false; });

            // Release fallback: discard the accumulated bytes to resync the stream.
            out_ = in;
            consumed_idle_count_ = idle_count;
            return false;
        }

        const bool is_idle =
            (readable <= kProtocolMaxPayloadSize) ? (idle_count != consumed_idle_count_) : false;
        if (is_idle)
            consumed_idle_count_ = idle_count;
        else if (readable < kMinFragmentSize)
            return false;

        const auto size = std::min(readable, kProtocolMaxPayloadSize);
        const auto offset = out & kBufferMask;
        const auto first_size = std::min(size, kBufferSize - offset);
        const auto second_size = size - first_size;

        static_cast<T*>(this)->handle_uplink(
            {ring_.data() + offset, first_size}, {ring_.data(), second_size}, is_idle);

        out_ = static_cast<IndexType>(out + static_cast<IndexType>(size));
        return true;
    }

private:
    // Bounds the ISR.REACK poll in wait_receiver_ready(). RE is already
    // acknowledged in steady state, so the loop normally exits on its first
    // read; this only stops restart_rx_dma() from spinning forever inside the
    // DMA error interrupt if the receiver never comes back.
    static constexpr uint32_t kReceiverAckPollLimit = 1024;

    explicit RxBuffer(UART_HandleTypeDef* hal_uart_handle)
        : hal_uart_handle_(hal_uart_handle) {
        enable_fifo_mode();
        configure_rx_error_policy();
        bind_rx_dma_callbacks();
        start_rx_dma();
    }

    // Every port runs with the 16-entry RX and TX FIFOs on, decided here rather
    // than in the .ioc.
    //
    // CubeMX leaves the four ports inconsistent -- MX_UART5_Init and MX_UART7_Init
    // call HAL_UARTEx_EnableFifoMode, MX_USART1_UART_Init and MX_USART10_UART_Init
    // call HAL_UARTEx_DisableFifoMode -- with no reason behind the split. The
    // thresholds are already identical everywhere (RXFTCFG/TXFTCFG = 1/8 on all
    // four, and HAL_UARTEx_DisableFifoMode only clears CR1.FIFOEN, leaving CR3
    // alone), so the whole difference is one bit.
    //
    // Two reasons the bit belongs to this driver instead of to the .ioc:
    //
    //   - TxBuffer::try_dequeue() times its idle window from ISR.TC rather than
    //     from DMA completion *because* a TXFIFO can still hold 16 bytes when the
    //     DMA reports done -- 173 us at 921600 baud, the same order as the 300 us
    //     window itself. That reasoning only holds where the FIFO is on. Keeping
    //     the invariant next to the code that depends on it means a CubeMX
    //     regeneration cannot quietly invalidate it.
    //   - configure_rx_error_policy() sets CR3.OVRDIS, so an overrun is not
    //     reported at all; if the DMA were ever late enough to lose a byte it
    //     would be silent. The FIFO is 16 bytes of cushion against exactly that,
    //     which is worth having in a path that cannot detect its own failure.
    //
    // Not a throughput argument: at 921600 baud a byte takes 10.8 us and DMA
    // arbitration is orders of magnitude faster, so the cushion is never
    // approached in normal operation. It is there for the abnormal case.
    //
    // Uses the HAL entry point rather than a raw CR1 write so huart->FifoMode
    // stays truthful; it does its own UE-disable/restore around the bit, which is
    // required because FIFOEN is only writable while the USART is disabled.
    void enable_fifo_mode() const {
        core::utility::assert_always(HAL_UARTEx_EnableFifoMode(hal_uart_handle_) == HAL_OK);
    }

    // STM32H7 declares DMA_HandleTypeDef::Instance as void* (F4 types it as
    // DMA_Stream_TypeDef*), so every register access has to go through this cast.
    // UART RX streams are DMA1/DMA2 streams, never BDMA -- BDMA only serves
    // the D3 domain, and every UART on this board lives in D2.
    [[nodiscard]] DMA_Stream_TypeDef* rx_dma_stream() const {
        return static_cast<DMA_Stream_TypeDef*>(hal_uart_handle_->hdmarx->Instance);
    }

    // Ring offset the stream will write next, taken from the stream's own
    // counter, plus the lap bits that offset cannot carry.
    //
    // NDTR counts down from kBufferSize and is reloaded by the hardware on wrap,
    // so kBufferSize - NDTR is the offset. NDTR reads kBufferSize right after a
    // wrap and may read 0 in the instant before the reload; both mask to offset
    // 0, which is where the stream is. Nothing else has to be read, so unlike
    // the double-buffered version there is no pair of registers that could be
    // sampled either side of a hardware transition.
    IndexType sample_write_position() {
        const auto remaining = static_cast<size_t>(rx_dma_stream()->NDTR);
        core::utility::assert_debug(remaining <= kBufferSize);
        const auto offset = static_cast<IndexType>((kBufferSize - remaining) & kBufferMask);

        auto next = static_cast<IndexType>((in_ & ~kBufferMask) | offset);
        if (next < in_)
            next = static_cast<IndexType>(next + static_cast<IndexType>(kBufferSize));

        // A consumer that falls a full lap behind gets overwritten in place, and
        // the reconstruction above would under-report that rather than trip the
        // readable > kBufferSize check in try_dequeue(). Trap long before it: the
        // main loop polls every few microseconds, and half a ring is 11 ms at
        // 921600 baud.
        core::utility::assert_debug(
            static_cast<size_t>(static_cast<IndexType>(next - in_)) <= kBufferSize / 2);

        in_ = next;
        return next;
    }

    // Reception error policy, applied once before any DMA is armed.
    //
    //   CR3.DDRE = 0    "DMA Disable on Reception Error" stays OFF, so a
    //                   parity/framing/noise error does not block the DMA
    //                   request. The suspect byte lands in the ring like any
    //                   other and the protocol layer decides what to do with it.
    //   CR3.OVRDIS = 1  overrun detection off. start_rx_dma() deliberately
    //                   leaves CR3.EIE clear, so nothing would ever clear a
    //                   sticky ORE and a set ORE stalls reception. Continuous
    //                   DMA drains RDR fast enough that the flag carried no
    //                   information here anyway.
    //
    // Note the polarity. This code originally SET DDRE, with a comment claiming
    // that kept the DMA request alive across a line error; the bit does the
    // opposite -- its name states the behaviour it enables, exactly like OVRDIS
    // next to it. With DDRE set and CR3.EIE clear, one bad character killed the
    // port for good: the DMA request stayed blocked until the error flag was
    // cleared, no interrupt was raised to clear it, and rx_error_callback() only
    // ever fires for DMA controller faults, never for line errors. Measured on a
    // two-board rig -- one board caught a glitch during a baudrate switch and
    // from then on reported an IDLE event for every message the far end sent
    // while delivering zero bytes, permanently. Clearing DDRE fixed it outright;
    // see host/examples/uart_cross_test.cpp, "monitor" mode. Neither bit exists
    // on STM32F407, which is why c_board has no counterpart to this.
    //
    // Both are writable only while the USART is disabled, so UE is cycled around
    // the write. TxBuffer's constructor only binds callbacks, so nothing is
    // transmitting or receiving at this point.
    void configure_rx_error_policy() const {
        auto* instance = hal_uart_handle_->Instance;
        const bool was_enabled = (instance->CR1 & USART_CR1_UE) != 0U;

        ATOMIC_CLEAR_BIT(instance->CR1, USART_CR1_UE);
        ATOMIC_SET_BIT(instance->CR3, USART_CR3_OVRDIS);
        ATOMIC_CLEAR_BIT(instance->CR3, USART_CR3_DDRE);
        if (was_enabled)
            ATOMIC_SET_BIT(instance->CR1, USART_CR1_UE);
    }

    // Drop the sticky RX status through the dedicated clear register, which
    // leaves RDR untouched. On STM32F407 the only way to clear ORE is the "read
    // SR, then read DR" sequence, which would steal a byte from the DMA stream.
    void clear_rx_error_flags() const {
        WRITE_REG(
            hal_uart_handle_->Instance->ICR,
            USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF | USART_ICR_IDLECF);
    }

    // Discard whatever is still held in RDR and in the 16-entry RXFIFO (DS13313
    // Table 5 gives the depth). enable_fifo_mode() turns the FIFO on for every
    // port, so without this up to 16 bytes captured before an abort would survive
    // it and be written at ring offset 0 as if they had just arrived.
    // STM32F407 has no request register at all.
    void flush_rx_fifo() const { WRITE_REG(hal_uart_handle_->Instance->RQR, USART_RQR_RXFRQ); }

    // The receiver acknowledges RE asynchronously: it is only sampling the line
    // once ISR.REACK reads back as 1, and STM32F407 has no such handshake. RE is
    // never cleared once CubeMX has enabled it, so this normally exits on the
    // first read.
    void wait_receiver_ready() const {
        auto* instance = hal_uart_handle_->Instance;
        if ((instance->CR1 & USART_CR1_RE) == 0U)
            return;

        for (uint32_t i = 0; i < kReceiverAckPollLimit; ++i) {
            if ((instance->ISR & USART_ISR_REACK) != 0U)
                return;
        }
        core::utility::assert_debug_lazy([]() noexcept { return false; });
    }

    void bind_rx_dma_callbacks() {
        auto* hal_dma_handle = hal_uart_handle_->hdmarx;
        core::utility::assert_debug(hal_dma_handle != nullptr);

        // Transfer completion carries no information: the stream wraps on its own
        // and the write position is read out of NDTR by the consumer. The
        // interrupt is still enabled by HAL_DMA_Start_IT, but HAL_DMA_IRQHandler
        // null-checks every callback, so it just clears the flag. Once per lap
        // (22 ms at 921600 baud) that is not worth suppressing.
        hal_dma_handle->XferCpltCallback = nullptr;
        hal_dma_handle->XferM1CpltCallback = nullptr;
        hal_dma_handle->XferErrorCallback = &T::hal_rx_dma_error_callback;
        hal_dma_handle->XferHalfCpltCallback = nullptr;
        hal_dma_handle->XferM1HalfCpltCallback = nullptr;
        hal_dma_handle->XferAbortCallback = nullptr;
    }

    void start_rx_dma() {
        auto* hal_dma_handle = hal_uart_handle_->hdmarx;
        core::utility::assert_debug(hal_dma_handle->Init.Mode == DMA_CIRCULAR);

        // Single-buffer by construction: HAL_DMA_Init() masks DBM and CT out of
        // CR before writing it (see the register mask in
        // bsp/stm32h7xx-hal-driver/Src/stm32h7xx_hal_dma.c), and nothing in this
        // class sets them, so the stream has exactly one target address that
        // never needs re-pointing. HAL_DMA_Abort() in restart_rx_dma() leaves
        // both alone, so this holds across a restart too.
        core::utility::assert_debug((rx_dma_stream()->CR & (DMA_SxCR_DBM | DMA_SxCR_CT)) == 0U);

        in_ = 0;
        out_ = 0;
        idle_count_.store(0, std::memory_order::relaxed);
        consumed_idle_count_ = 0;

        const auto source =
            static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&hal_uart_handle_->Instance->RDR));
        const auto destination = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(ring_.data()));

        core::utility::assert_always(
            HAL_DMA_Start_IT(hal_dma_handle, source, destination, kBufferSize) == HAL_OK);

        // Present the port to the HAL as an in-progress circular ReceiveToIdle so
        // that HAL_UART_IRQHandler takes its DMA_CIRCULAR branch on IDLE: it then
        // reports the event through HAL_UARTEx_RxEventCallback without touching
        // DMAR or aborting the stream.
        hal_uart_handle_->RxState = HAL_UART_STATE_BUSY_RX;
        hal_uart_handle_->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

        // RxXferSize is deliberately one larger than the ring. The IDLE branch of
        // HAL_UART_IRQHandler reports the event when
        // `0 < __HAL_DMA_GET_COUNTER(hdmarx) < huart->RxXferSize`, and NDTR spans
        // 1..kBufferSize while the stream runs. Setting RxXferSize to exactly
        // kBufferSize would leave the just-wrapped case (NDTR == kBufferSize) to
        // the handler's separate `nb_remaining == RxXferSize` branch; one more
        // covers every NDTR the hardware can present through the single
        // comparison.
        //
        // The HAL reads RxXferSize nowhere else on this path: UART_DMAReceiveCplt
        // is never installed (bind_rx_dma_callbacks leaves XferCpltCallback
        // null), and RxXferCount is only written by that IDLE branch, never read
        // by us.
        hal_uart_handle_->RxXferSize = static_cast<uint16_t>(kBufferSize + 1);
        hal_uart_handle_->RxXferCount = static_cast<uint16_t>(kBufferSize);

        // Drop every sticky RX flag and every byte still held in RDR/RXFIFO, then
        // confirm the receiver is live, so DMA requests resume on a real byte
        // boundary instead of on leftovers from before the restart.
        clear_rx_error_flags();
        flush_rx_fifo();
        wait_receiver_ready();

        // CR3.EIE and CR1.PEIE stay clear on purpose, and that is what makes
        // clearing CR3.DDRE mandatory rather than merely preferable: with the
        // error interrupt off, nothing would ever clear an error flag, so a set
        // DDRE would block the DMA request forever. HAL_UART_IRQHandler() classifies *any*
        // RX error as blocking whenever DMAR is set -- the condition it tests is
        // `HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR) || ...` -- and
        // responds with UART_EndRxTransfer() followed by HAL_DMA_Abort_IT(). So
        // with the error interrupts enabled the hardware would keep the stream
        // running and the HAL would abort it anyway, reproducing the F407
        // behaviour on a part that does not need it.
        //
        // Line errors are absorbed by the DDRE/OVRDIS policy instead. Genuine DMA
        // controller faults still arrive through the stream's own error
        // interrupt (hal_rx_dma_error_callback -> rx_error_callback), which is
        // what restart_rx_dma() now exists for.
        ATOMIC_SET_BIT(hal_uart_handle_->Instance->CR3, USART_CR3_DMAR);
        ATOMIC_SET_BIT(hal_uart_handle_->Instance->CR1, USART_CR1_IDLEIE);
    }

    // Published for the half-duplex turnaround gate in TxBuffer, which uses a
    // change in this counter as "the peer has finished answering". Relaxed
    // rather than acquire because only the change matters here: nothing is
    // published through the counter, unlike try_dequeue() above, which acquires
    // it precisely to pair the boundary with ring contents.
    [[nodiscard]] uint16_t idle_count() const {
        return idle_count_.load(std::memory_order::relaxed);
    }

    void uart_idle_event_callback() {
        // Publishes a boundary and nothing else, so no lock and no register
        // write: the consumer pairs this counter with the write position it
        // samples immediately afterwards. The double-buffered version had to read
        // CR/NDTR and re-point M0AR/M1AR here, which is why it held a global
        // __disable_irq() -- at 921600 baud that masked the FDCAN interrupt
        // (NVIC priority 1) roughly 9000 times a second per port.
        idle_count_.fetch_add(1, std::memory_order::release);
    }

    void rx_error_callback() {
        // Reached for DMA controller faults (transfer/FIFO/direct-mode errors),
        // not for line errors: with CR3.EIE left clear, a framing/noise/parity
        // glitch no longer raises the UART interrupt at all and a clear DDRE
        // keeps the stream running through it. The HAL error callback in uart.cpp can still
        // route here if the HAL sets an RX ErrorCode by some other path.
        //
        // Deliberately no debug assert, unlike c_board: mc02 drives DBUS and
        // hot-pluggable ports, so the port must recover rather than trap the
        // debug build.
        restart_rx_dma();
    }

    void restart_rx_dma() {
        auto* hal_dma_handle = hal_uart_handle_->hdmarx;

        ATOMIC_CLEAR_BIT(hal_uart_handle_->Instance->CR1, USART_CR1_IDLEIE);
        ATOMIC_CLEAR_BIT(hal_uart_handle_->Instance->CR3, USART_CR3_DMAR);

        if ((rx_dma_stream()->CR & DMA_SxCR_EN) != 0U) {
            core::utility::assert_always(HAL_DMA_Abort(hal_dma_handle) == HAL_OK);
        }

        // The HAL's blocking-error path installs UART_DMAAbortOnError as the
        // stream's abort callback; re-bind so a later HAL_DMA_Abort cannot bounce
        // back into the HAL error machinery behind our back.
        bind_rx_dma_callbacks();
        start_rx_dma();
    }

    UART_HandleTypeDef* hal_uart_handle_;

    // Lives wherever the enclosing port object is placed, which uart.hpp puts in
    // .d2_sram -- D2 SRAM at 0x30000000, the same domain as the DMA1 streams that
    // write it. app.cpp maps that range non-cacheable through MPU region 1, so
    // DMA writes need no cache maintenance. Do not move the port objects into
    // .dtcm the way can.hpp places its objects: DMA1/DMA2 cannot reach DTCM, so
    // the stream would silently transfer nothing.
    alignas(uint32_t) std::array<std::byte, kBufferSize> ring_{};

    // Consumer-only. in_ carries the lap bits that the ring offset read out of
    // NDTR cannot; both are reset by start_rx_dma(), which also runs from the DMA
    // error interrupt -- a restart discards the consumer's position by design.
    IndexType in_{0};
    IndexType out_{0};

    // Published by the IDLE interrupt, consumed by try_dequeue(). A counter and
    // not a position: the consumer pairs it with the write position it samples
    // immediately afterwards, which is the same pairing the interrupt-published
    // version provided.
    std::atomic<uint16_t> idle_count_{0};
    uint16_t consumed_idle_count_{0};

    static_assert(std::atomic<uint16_t>::is_always_lock_free);
};

} // namespace libhcs::firmware::uart
