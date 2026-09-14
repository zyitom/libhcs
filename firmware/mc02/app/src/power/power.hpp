#pragma once

#include <cstdint>

#include <gpio.h>
#include <main.h>

namespace libhcs::firmware::power {

// mc02 接线端子上的三路开关输出电源轨。
//
// 引脚及其上电状态由 mc02_slave.ioc 固定、由 MX_GPIO_Init() 在此之前很久施加:
//
//   Power_OUT1_EN  PC14  24 V 轨 0  复位低(关)
//   Power_OUT2_EN  PC13  24 V 轨 1  复位低(关)
//   Power_5V_EN    PC15   5 V 轨    复位高(开)
//
// 刻意用无状态的自由函数, 也刻意不做初始化: 24 V 轨馈给端子上所接的东西, 其上电
// 状态是 .ioc 里的硬件决策。构造函数去驱动它, 要么重复该决策要么悄悄覆盖, 无论
// 哪种都会让电源轨在每次复位时毛刺。访问器回读 ODR 而非缓存, 故不可能与引脚
// 不一致。
//
// 仅限本板: core/ 不携带电源轨概念, 主机不可达。要暴露须先在
// core/include/libhcs/data/datas.hpp 增加数据视图。

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
