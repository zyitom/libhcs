#include <main.h>

// Fault recovery hook for the four fault handlers in the generated
// stm32h7xx_it.c, which is compiled into both the application and the
// bootloader. The application resets into DFU so a faulting image cannot brick
// the board (see app/src/utility/assert.cpp); the bootloader deliberately does
// not.
//
// Resetting here would be a downgrade. The bootloader is the last line of
// recovery, and a fault in it is almost certainly deterministic -- a reset would
// re-enter the same code and fault again, so the device would reboot in a loop
// and never stay enumerated long enough for a host to flash anything. Returning
// leaves the fault handler's own while(1) in place: the board stops with the
// faulting frame intact for a debugger, which is the only tool that can help at
// that point anyway.
extern "C" void libhcs_fault_recover(void) {}

// Stray-interrupt handler for the EXTI group the shared MX_GPIO_Init() arms.
//
// gpio.c is generated from the single .ioc that describes the application's
// pinout, and it is compiled into both images. Its tail configures the BMI088
// data-ready lines (PE10 INT1_ACC, PE12 INT1_GYRO) as rising-edge EXTI inputs
// and calls HAL_NVIC_EnableIRQ(EXTI15_10_IRQn) -- for the bootloader, which
// wants only KEY and USB, that is pure collateral damage.
//
// Only the application defines EXTI15_10_IRQHandler (app/src/gpio/gpio.cpp).
// Without this definition the bootloader falls back on the weak alias in
// startup_stm32h723vgtx.s, which points at Default_Handler -- an unconditional
// `b .`. That is not a fault and raises nothing a debugger reports: the
// interrupt dispatches successfully and then spins forever at preemption
// priority 4, starving SysTick and thread mode, so the bootloader stops before
// USB ever enumerates and the board looks bricked.
//
// It takes a warm reset out of a running application to reach that state. The
// BMI088 is an external chip and a CPU reset does not touch it, so it keeps
// free-running and its first data-ready pulse lands here. A cold power-on
// cannot reproduce it: the sensor loses power too and comes up in suspend.
//
// Clearing every line in the group, not just the two the .ioc uses today, so a
// regenerated gpio.c that arms another pin in 10..15 cannot resurrect the hang.
extern "C" void EXTI15_10_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(
        GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
}
