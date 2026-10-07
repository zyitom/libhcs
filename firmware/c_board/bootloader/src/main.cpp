#include <cstdint>

#include <device/usbd.h>
#include <gpio.h>
#include <main.h>
#include <system_stm32f4xx.h>
#include <tusb.h>
#include <usb_otg.h>

#include "firmware/c_board/bootloader/src/flash/layout.hpp"
#include "firmware/c_board/bootloader/src/flash/validation.hpp"
#include "firmware/c_board/bootloader/src/usb/dfu.hpp"
#include "firmware/c_board/bootloader/src/utility/assert.hpp"
#include "firmware/c_board/bootloader/src/utility/boot_mailbox.hpp"
#include "firmware/c_board/bootloader/src/utility/jump.hpp"

namespace {

void bootloader_delay_us(uint32_t delay_us) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    const uint32_t start = DWT->CYCCNT;
    const uint32_t delay_cycles = (SystemCoreClock / 1000000U) * delay_us;
    while ((DWT->CYCCNT - start) < delay_cycles) {}
}

bool bootloader_check_bootloader_force_stay_requested() {
    for (uint32_t sample_index = 0; sample_index < 4; ++sample_index) {
        bootloader_delay_us(250);
        if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) != GPIO_PIN_RESET)
            return false;
    }

    return true;
}

} // namespace

int main() {
    HAL_Init();
    SystemClock_Config();

    // ART 加速器(FLASH_ACR 的预取与指令缓存位复位后为 0, CubeMX 只写了 LATENCY):
    // 168 MHz 下 5 WS 的取指等待全走缓存与预取, DFU 期间与镜像校验(SHA-256 全程读
    // FLASH)都受益。不开数据缓存: flash 编程后回读会命中 D-cache 里的旧行, 而本镜像
    // 没有 DMA 流量, 开它只有风险没有收益。跳转 app 前不需要清缓存 -- I-cache 里只有
    // bootloader 自己的地址(0x08000000 段), 与 app 的 0x08010000 段不相交; 数据侧
    // 本来就没缓存。app 侧自行再开缓存(app.cpp, 幂等)。
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();

    MX_GPIO_Init();

    const bool force_stay = bootloader_check_bootloader_force_stay_requested();

    MX_USB_OTG_FS_PCD_Init();

    using namespace libhcs::firmware; // NOLINT(google-build-using-namespace)

    const uint32_t boot_request = utility::boot_mailbox.consume_request();
    const bool force_dfu = boot_request == utility::BootMailbox::kMailboxRequestEnterDfu;
    const bool boot_app_once = boot_request == utility::BootMailbox::kMailboxRequestBootAppOnce;
    if (!force_stay && (boot_app_once || !force_dfu)) {
        if (flash::validate_app_image())
            utility::jump_to_app(flash::kAppStartAddress);
    }

    utility::assert_always(tusb_rhport_init(0, nullptr));

    while (true) {
        tud_task();
        usb::Dfu::instance().poll();
    }
}
