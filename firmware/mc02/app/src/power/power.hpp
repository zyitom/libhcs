#pragma once

#include <cstdint>

#include <gpio.h>
#include <main.h>

namespace libhcs::firmware::power {

// The three switched output rails on mc02's screw terminals.
//
// Pins, and the states they come up in, are fixed by mc02_slave.ioc and applied
// by MX_GPIO_Init() long before anything here runs:
//
//   Power_OUT1_EN  PC14  24 V rail 0  reset LOW   (off)
//   Power_OUT2_EN  PC13  24 V rail 1  reset LOW   (off)
//   Power_5V_EN    PC15   5 V rail    reset HIGH  (on)
//
// Deliberately free functions holding no state, and deliberately no
// initialization step: the 24 V rails feed whatever is wired to the terminals,
// so their power-on state is a hardware decision that belongs in the .ioc. A
// constructor that drove them would either duplicate that decision or silently
// override it, and either way the rails would glitch on every reset. The
// accessors read ODR back instead of caching, so they cannot disagree with the
// pin.
//
// Board-local only: nothing in core/ carries a power-rail concept, so these are
// not reachable from the host. Exposing them means adding a data view in
// core/include/libhcs/data/datas.hpp first.

namespace internal {

inline void write(GPIO_TypeDef* port, uint16_t pin, bool enabled) {
    HAL_GPIO_WritePin(port, pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

inline bool read(const GPIO_TypeDef* port, uint16_t pin) { return (port->ODR & pin) != 0; }

} // namespace internal

inline void set_output1(bool enabled) {
    internal::write(Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin, enabled);
}

inline void set_output2(bool enabled) {
    internal::write(Power_OUT2_EN_GPIO_Port, Power_OUT2_EN_Pin, enabled);
}

inline void set_5v(bool enabled) {
    internal::write(Power_5V_EN_GPIO_Port, Power_5V_EN_Pin, enabled);
}

[[nodiscard]] inline bool output1_enabled() {
    return internal::read(Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin);
}

[[nodiscard]] inline bool output2_enabled() {
    return internal::read(Power_OUT2_EN_GPIO_Port, Power_OUT2_EN_Pin);
}

[[nodiscard]] inline bool v5_enabled() {
    return internal::read(Power_5V_EN_GPIO_Port, Power_5V_EN_Pin);
}

} // namespace libhcs::firmware::power
