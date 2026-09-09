#include <cstdint>

#include <device/usbd.h>
#include <gpio.h>
#include <main.h>
#include <tusb.h>

#include "firmware/mc02/bootloader/src/flash/layout.hpp"
#include "firmware/mc02/bootloader/src/flash/metadata.hpp"
#include "firmware/mc02/bootloader/src/flash/validation.hpp"
#include "firmware/mc02/bootloader/src/usb/dfu.hpp"
#include "firmware/mc02/bootloader/src/usb/usb_descriptors.hpp"
#include "firmware/mc02/bootloader/src/utility/assert.hpp"
#include "firmware/mc02/bootloader/src/utility/boot_mailbox.hpp"
#include "firmware/mc02/bootloader/src/utility/jump.hpp"

int main() {
    // The bootloader runs cache-less, mirroring the c_board (Cortex-M4) design.
    // With the D-cache enabled, every flash read in the DFU validate path
    // (CRC32 / SHA-256 / vector-table check) depends on manual cache
    // invalidation after each flash program; any gap leaves the validator
    // reading stale data, which makes a freshly-flashed image fail validation
    // and the device fall back into DFU. The bootloader only drives USB DFU and
    // flash programming, neither of which benefits from caching, so we leave the
    // caches and MPU off entirely to keep flash and RAM trivially coherent.
    HAL_Init();
    SystemClock_Config();

    RCC_PeriphCLKInitTypeDef usb_clk = {};
    usb_clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    usb_clk.UsbClockSelection    = RCC_USBCLKSOURCE_HSI48;
    libhcs::firmware::utility::assert_always(HAL_RCCEx_PeriphCLKConfig(&usb_clk) == HAL_OK);
    HAL_PWREx_EnableUSBVoltageDetector();
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

    MX_GPIO_Init();

    using namespace libhcs::firmware; // NOLINT(google-build-using-namespace)

    // Holding the user key at reset pins the board in DFU regardless of what the
    // mailbox says, so a bad image can always be recovered from.
    const bool force_stay = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET;

    // A single mailbox read covers both directions: the application asks for DFU
    // with "DFU0", and the DFU download path asks for the freshly flashed image
    // with "APP1". The latter wins over a stale DFU request so the reset that
    // follows manifestation lands in the application instead of back in DFU.
    const uint32_t boot_request = utility::boot_mailbox.consume_request();
    const bool force_dfu = boot_request == utility::BootMailbox::kMailboxRequestEnterDfu;
    const bool boot_app_once = boot_request == utility::BootMailbox::kMailboxRequestBootAppOnce;
    if (!force_stay && (boot_app_once || !force_dfu)) {
        if (flash::validate_app_image())
            utility::jump_to_app(flash::kAppStartAddress);
    }

    // Reaching here means DFU. Report why, in the order the decision above made
    // it: the key overrides everything, then an explicit host request, then a
    // half-written previous session, and otherwise the image simply is not
    // usable. Set before tusb_rhport_init() -- string descriptors are read
    // during enumeration and never refreshed afterwards.
    usb::get_usb_descriptors().set_entry_reason(
        force_stay ? usb::DfuEntryReason::kUserKey
        : force_dfu ? usb::DfuEntryReason::kHostRequest
        : flash::Metadata::get_instance().previous_session_interrupted()
            ? usb::DfuEntryReason::kInterrupted
            : usb::DfuEntryReason::kNoValidApp);

    utility::assert_always(tusb_rhport_init(0, nullptr));

    while (true) {
        tud_task();
        usb::Dfu::instance().poll();
    }
}
