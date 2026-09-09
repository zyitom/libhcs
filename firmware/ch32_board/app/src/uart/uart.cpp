#include "firmware/ch32_board/app/src/uart/uart.hpp"

namespace libhcs::firmware::uart {

// Thin ISR shims routing each USART interrupt to its array element's
// irq_handler(). Plain __attribute__((interrupt)) so GCC emits mret.

static_assert(kUartCount == 2, "ISR shims below must cover every board::kUartPorts entry");

extern "C" void USART1_IRQHandler(void) __attribute__((interrupt()));
extern "C" void USART1_IRQHandler(void) { uart_array[0]->irq_handler(); }

// uart2 is USART3, not USART2: see the routing table in board_app.hpp. The
// vector must follow the peripheral, so this shim is named after the hardware
// while the array index stays logical.
extern "C" void USART3_IRQHandler(void) __attribute__((interrupt()));
extern "C" void USART3_IRQHandler(void) { uart_array[1]->irq_handler(); }

} // namespace libhcs::firmware::uart
