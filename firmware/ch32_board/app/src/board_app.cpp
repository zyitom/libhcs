#include "firmware/ch32_board/app/src/board_app.hpp"

#include <cstddef>
#include <cstdint>

#include "core/src/utility/assert.hpp"

namespace libhcs::firmware::board {
namespace {

// GPIOA..GPIOG sit at a fixed stride and RCC_HB2Periph_GPIOx are consecutive
// bits in the same order, so the enable bit is derived rather than switched --
// this is what the EVT reference does (EXAM/CAN/Networking/Common/hardware.c).
uint32_t gpio_clock_bit(GPIO_TypeDef* port) {
    const auto index = static_cast<size_t>(
        (reinterpret_cast<uintptr_t>(port) - reinterpret_cast<uintptr_t>(GPIOA))
        / sizeof(GPIO_TypeDef));
    return RCC_HB2Periph_GPIOA << index;
}

void init_pin(const PinConfig& config, GPIOMode_TypeDef mode) {
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | gpio_clock_bit(config.port), ENABLE);

    // Alternate-function select first: GPIO_Init() only programs direction and
    // drive, it does not route the peripheral to the pin.
    GPIO_PinAFConfig(config.port, config.pin_source, config.alternate);

    GPIO_InitTypeDef gpio = {};
    gpio.GPIO_Pin = static_cast<uint16_t>(1u << config.pin_source);
    gpio.GPIO_Mode = mode;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(config.port, &gpio);
}

} // namespace

uint32_t peripheral_clock() {
    RCC_ClocksTypeDef clocks = {};
    RCC_GetClocksFreq(&clocks);
    core::utility::assert_always(clocks.HCLK_Frequency != 0);
    return clocks.HCLK_Frequency;
}

uint32_t init_can(const CanPort& port) {
    // bxCAN is a master/slave pair: CAN2 (and CAN3) share CAN1's filter block,
    // so CAN1's clock must stay enabled even when only CAN2 is used. The EVT
    // reference ORs RCC_HB1Periph_CAN1 into every CAN enable for this reason.
    uint32_t can_clock_bits = RCC_HB1Periph_CAN1;
    if (port.base == CAN2_BASE)
        can_clock_bits |= RCC_HB1Periph_CAN2;
    else if (port.base == CAN3_BASE)
        can_clock_bits |= RCC_HB1Periph_CAN3;
    RCC_HB1PeriphClockCmd(can_clock_bits, ENABLE);

    init_pin(port.tx, GPIO_Mode_AF_PP);
    init_pin(port.rx, GPIO_Mode_IPU);

    return peripheral_clock();
}

uint32_t init_uart(const UartPort& port) {
    // USART1 hangs off the HB2 domain, the rest off HB1 -- the same split the
    // vendor headers encode as RCC_HB2Periph_USART1 vs RCC_HB1Periph_USARTx.
    if (port.base == USART1_BASE)
        RCC_HB2PeriphClockCmd(RCC_HB2Periph_USART1, ENABLE);
    else if (port.base == USART2_BASE)
        RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART2, ENABLE);
    else if (port.base == USART3_BASE)
        RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART3, ENABLE);
    else
        core::utility::assert_failed_always();

    init_pin(port.tx, GPIO_Mode_AF_PP);
    init_pin(port.rx, GPIO_Mode_IPU);

    return peripheral_clock();
}

} // namespace libhcs::firmware::board
