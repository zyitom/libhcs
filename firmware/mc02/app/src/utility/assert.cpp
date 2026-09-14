#include "core/src/utility/assert.hpp"

#include <source_location>

#include <main.h>

#include "firmware/mc02/app/src/led/led.hpp"
#include "firmware/mc02/app/src/utility/boot_mailbox.hpp"
#include "firmware/mc02/app/src/utility/interrupt_lock.hpp"

// 生成代码 stm32h7xx_it.c 中四个 fault handler 共用的故障恢复。没有它时 fault 会
// 让 CPU 停在主机永远够不到的 while(1) 里: USB 不再应答, 只能靠调试器或复位时按住
// KEY 恢复。经 boot_mailbox 请求 DFU 后复位, 可保住烧录通道。
//
// 已接调试器时, 保留现场比自动恢复更有价值: 可以检查故障帧和下方
// assert_file/assert_line 全局变量, 故此时直接返回, 让外层 while(1) 维持状态。
extern "C" void libhcs_fault_recover(void) {
    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
        return;

    libhcs::firmware::utility::boot_mailbox.request_enter_dfu();
    __DSB();
    __ISB();
    NVIC_SystemReset();
}

namespace libhcs::core::utility {

const char* volatile assert_file = nullptr;
volatile unsigned int assert_line = 0;
const char* volatile assert_function = nullptr;

namespace {
inline void force_led_red() noexcept {
    // mc02 的状态 LED 是经 SPI6 驱动的 WS2812(不像 c_board 用 GPIO 模拟时序),
    // panic 指示走 Led 对象。LED 尚未构造(极早期 panic)时 try_get() 返回 nullptr,
    // 此时跳过指示。set_value() 轮询 SPI6 标志, 在此处关中断的状态下也能完成。
    if (auto* led = firmware::led::led.try_get())
        led->set_value(255, 0, 0);
}
} // namespace

[[noreturn]] void assert_func(const std::source_location& location) {
    firmware::utility::InterruptMutex::lock();

    assert_file = location.file_name();
    assert_line = location.line();
    assert_function = location.function_name();

    force_led_red();

    __builtin_trap();
}

} // namespace libhcs::core::utility
