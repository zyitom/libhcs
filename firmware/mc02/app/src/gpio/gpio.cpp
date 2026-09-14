#include "firmware/mc02/app/src/gpio/gpio.hpp"

#include <cstdint>

#include <gpio.h>
#include <main.h>

#include "firmware/mc02/app/src/spi/bmi088/accel.hpp"
#include "firmware/mc02/app/src/spi/bmi088/gyro.hpp"
#include "firmware/mc02/app/src/timer/timer.hpp"

namespace libhcs::firmware::gpio {

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {
    if (gpio_pin == INT1_ACC_Pin) {
#ifdef libhcs_APP_IMU_ENABLE
        // 尽量贴近中断捕获数据就绪边沿的时刻; 取样的 SPI 读在之后的主循环进行。
        const uint32_t capture_timestamp_quarter_us =
            timer::timer->timepoint().time_since_epoch().count();
        if (spi::bmi088::accelerometer)
            spi::bmi088::accelerometer->data_ready_callback(capture_timestamp_quarter_us);
#endif
    } else if (gpio_pin == INT1_GYRO_Pin) {
#ifdef libhcs_APP_IMU_ENABLE
        const uint32_t capture_timestamp_quarter_us =
            timer::timer->timepoint().time_since_epoch().count();
        if (spi::bmi088::gyroscope)
            spi::bmi088::gyroscope->data_ready_callback(capture_timestamp_quarter_us);
#endif
    } else {
        gpio::gpio->handle_input_edge_interrupt(gpio_pin);
    }
}

// 通道边沿中断。四条 EXTI 线全部由 app 侧持有, 任何边沿中断都不依赖可能被
// CubeMX 重新生成删掉的 USER CODE 段。
//
// EXTI0/2/9_5 无需 CubeMX 参与: PA0/PA2/PE9 在 .ioc 里是 S_TIM*_CH* 引脚,
// CubeMX 不知道它们兼作输入, 也就不为这些线生成 handler。EXTI15_10 在 .ioc 中
// 已关闭 Generate IRQ handler(NVIC > Code generation), 只去掉生成 handler 一项;
// MX_GPIO_Init 仍会使能该线并设置优先级。
// NOLINTNEXTLINE(readability-identifier-naming): vector table symbol name.
extern "C" void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }
// NOLINTNEXTLINE(readability-identifier-naming): vector table symbol name.
extern "C" void EXTI2_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2); }
// NOLINTNEXTLINE(readability-identifier-naming): vector table symbol name.
extern "C" void EXTI9_5_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9); }

// PE10(INT1_ACC)、PE12(INT1_GYRO)与 PE13(PWM 通道 4)都在这条线上, 故它须
// 同时服务 IMU 与 GPIO 驱动。
// NOLINTNEXTLINE(readability-identifier-naming): vector table symbol name.
extern "C" void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(INT1_ACC_Pin);
    HAL_GPIO_EXTI_IRQHandler(INT1_GYRO_Pin);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

} // namespace libhcs::firmware::gpio
