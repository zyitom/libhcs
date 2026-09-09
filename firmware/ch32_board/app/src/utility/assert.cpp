#include "core/src/utility/assert.hpp"

#include <cstdint>
#include <source_location>

#include "firmware/ch32_board/app/src/led/led.hpp"
#include "firmware/ch32_board/app/src/utility/interrupt_lock.hpp"

namespace libhcs::core::utility {

const char* volatile assert_file = nullptr;
volatile unsigned int assert_line = 0;
const char* volatile assert_function = nullptr;

namespace {
inline void force_led_red() noexcept {
    // No-op on ch32_board (the Led is a stub, see led.hpp). Kept structurally
    // identical to mc02 so this file stays a near-verbatim copy; try_get()
    // returns nullptr until the LED is constructed.
    if (auto* led = firmware::led::led.try_get())
        led->set_value(255, 0, 0);
}
} // namespace

[[noreturn]] void assert_func(const std::source_location& location) {
    firmware::utility::InterruptMutex::lock();

    assert_file = location.file_name();
    assert_line = location.line();
    assert_function = location.function_name();

    // The globals above live in .bss, which the startup code re-zeroes -- and on
    // this board every debugger halt resets the core (PITFALLS.md 4.4), so by the
    // time they can be read they are already gone. Mirror them into the diag
    // window at 0x20170000, which no image links a section over and which
    // therefore survives a reset. Layout continues past the words the boot core
    // (diag[0..10]) and app (diag[11..25]) own:
    //   diag[26] marker, diag[27] line, diag[28] file, diag[29] function
    // Resolve the two pointers against the ELF, e.g.
    //   riscv32-wch-elf-objdump -s -j .rodata build/ch32_board_app.elf
    // TODO(usb-bringup): drop together with the rest of the diag instrumentation.
    {
        auto* diag = reinterpret_cast<volatile uint32_t*>(0x20170000U);
        diag[27] = location.line();
        diag[28] = reinterpret_cast<uint32_t>(location.file_name());
        diag[29] = reinterpret_cast<uint32_t>(location.function_name());
        diag[26] = 0xA55E'A55EU; // written last: a commit barrier for the three above
    }

    force_led_red();

    __builtin_trap();
}

} // namespace libhcs::core::utility
