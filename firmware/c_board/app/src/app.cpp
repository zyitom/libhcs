#include "firmware/c_board/app/src/app.hpp"

#include <cstdint>

#include <can.h>
#include <device/usbd.h>
#include <dma.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <usart.h>
#include <usb_otg.h>

#include "firmware/c_board/app/src/can/can.hpp"
#include "firmware/c_board/app/src/gpio/gpio.hpp"
#include "firmware/c_board/app/src/led/led.hpp"
#include "firmware/c_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/c_board/app/src/spi/bmi088/gyro.hpp"
#include "firmware/c_board/app/src/spi/bmi088/service.hpp"
#include "firmware/c_board/app/src/spi/bmi088/temperature.hpp"
#include "firmware/c_board/app/src/spi/spi.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/c_board/app/src/uart/uart.hpp"
#include "firmware/c_board/app/src/usb/vendor.hpp"
#include "firmware/c_board/app/src/utility/boot_mailbox.hpp"
#include "firmware/c_board/app/src/utility/interrupt_lock.hpp"

int main() {
    SCB->VTOR = 0x08010000U;
    libhcs::firmware::app.init().run();
}

namespace libhcs::firmware {

// Linker-defined bounds of the .ccmram section: load image in FLASH (_siccmram)
// and run location in CCM (_sccmram.._eccmram). The F4 startup code only copies
// .data and clears .bss, so the App constructor stands in for the .ccmram copy.
extern "C" {
// NOLINTNEXTLINE(readability-identifier-naming)
extern uint32_t _siccmram, _sccmram, _eccmram;
}

App::App() {
    const utility::InterruptLockGuard guard;

    // Copy hot CPU-only forwarding state (serializer + USB batch buffers + CAN TX
    // rings) into zero-wait CCM before any of those globals are used. The M4 has
    // no data cache, so the writes land directly; interrupts are masked here.
    // The destination must remain writable; this clang-tidy check does not
    // account for the increment-and-store expression in the loop body.
    // NOLINTNEXTLINE(misc-const-correctness)
    for (uint32_t *dst = &_sccmram, *src = &_siccmram; dst < &_eccmram;)
        *dst++ = *src++;

    HAL_Init();
    SystemClock_Config();
    utility::boot_mailbox.clear();

    // TIM9 must be initialized before TIM2.
    MX_TIM9_Init();
    MX_TIM2_Init();
    timer::timer.init();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM8_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    MX_TIM5_Init();
    MX_USB_OTG_FS_PCD_Init();

    led::led.init();
    usb::vendor.init();
    can::can1.init();
    can::can2.init();
    uart::uart1.init();
    uart::uart2.init();
    uart::uart_dbus.init();
    gpio::gpio.init();
    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
    spi::bmi088::temperature.init();
}

// Non-static to ensure instantiation
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    while (true) {
        tud_task();
        usb::poll_dfu_runtime_reboot();

        // Publish the session state for the LED. The animation itself is driven
        // from HAL_IncTick() at 1 kHz; only this flag crosses from the main
        // loop, so the ISR never touches the USB stack. Reporting the session
        // rather than mere enumeration means steady green tells the operator
        // that data is actually being forwarded, not just that a cable is in.
        led::led->set_host_connected(usb::vendor->session_established());

        gpio::gpio->poll_periodic_input_samples();
        usb::vendor->try_transmit();
        can::can1->try_transmit();
        usb::vendor->try_transmit();
        can::can2->try_transmit();
        usb::vendor->try_transmit();
        spi::spi1->update();
        spi::bmi088::temperature->poll_pending_probe();
        spi::bmi088::service_pending_reads();
        usb::vendor->try_transmit();
        uart::uart1->try_transmit();
        usb::vendor->try_transmit();
        uart::uart2->try_transmit();
        usb::vendor->try_transmit();
        uart::uart_dbus->try_transmit();
    }
}

} // namespace libhcs::firmware
