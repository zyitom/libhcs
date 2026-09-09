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
        // Capture the data-ready edge time as close to the interrupt as possible;
        // the SPI read that fetches the sample happens later in the main loop.
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

// Channel edge interrupts. All four EXTI lines are owned app-side, so no edge
// interrupt depends on a USER CODE section that a CubeMX regeneration can drop.
//
// EXTI0/2/9_5 need nothing from CubeMX: PA0/PA2/PE9 are S_TIM*_CH* pins in the
// .ioc, so it never learns they are also inputs and generates no handler for
// those lines. EXTI15_10 used to be different -- CubeMX owned that handler
// because PE10/PE12 are GPXTI pins, and PE13 could only ride along through a
// USER CODE section, which is exactly what commit b972290 silently deleted.
// "Generate IRQ handler" is now off for EXTI15_10 in the .ioc (NVIC > Code
// generation), which removes only the handler; MX_GPIO_Init still enables the
// line and sets its priority.
extern "C" void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }
extern "C" void EXTI2_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2); }
extern "C" void EXTI9_5_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9); }

// PE10 (INT1_ACC), PE12 (INT1_GYRO) and PE13 (PWM channel 4) all sit on this
// line, so it must serve both the IMU and the GPIO driver.
extern "C" void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(INT1_ACC_Pin);
    HAL_GPIO_EXTI_IRQHandler(INT1_GYRO_Pin);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

} // namespace libhcs::firmware::gpio
