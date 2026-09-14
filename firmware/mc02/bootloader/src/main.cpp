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
    // bootloader 无缓存运行。若开 D-cache, DFU 校验路径(SHA-256 / 向量表检查)的
    // 每次 flash 读取都依赖编程后的手动缓存失效; 一旦遗漏就会读到旧数据, 使刚
    // 烧写的镜像校验失败、设备退回 DFU。bootloader 只驱动 USB DFU 与 flash 编程,
    // 均无缓存收益, 因此彻底关闭缓存与 MPU, 让 flash 与 RAM 天然保持一致。
    HAL_Init();
    SystemClock_Config();

    RCC_PeriphCLKInitTypeDef usb_clk = {};
    usb_clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    usb_clk.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    libhcs::firmware::utility::assert_always(HAL_RCCEx_PeriphCLKConfig(&usb_clk) == HAL_OK);
    HAL_PWREx_EnableUSBVoltageDetector();
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

    MX_GPIO_Init();

    using namespace libhcs::firmware; // NOLINT(google-build-using-namespace)

    // 复位时按住用户按键可无视 mailbox 内容强制停留在 DFU, 坏镜像因此总能救回。
    const bool force_stay = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET;

    // 一次 mailbox 读取覆盖两个方向: 应用以 "DFU0" 请求进入 DFU, DFU 下载路径
    // 以 "APP1" 请求启动刚烧写的镜像。后者优先于残留的 DFU 请求, 使 manifest 后
    // 的复位落在应用而不是回到 DFU。
    const uint32_t boot_request = utility::boot_mailbox.consume_request();
    const bool force_dfu = boot_request == utility::BootMailbox::kMailboxRequestEnterDfu;
    const bool boot_app_once = boot_request == utility::BootMailbox::kMailboxRequestBootAppOnce;
    if (!force_stay && (boot_app_once || !force_dfu)) {
        if (flash::validate_app_image())
            utility::jump_to_app(flash::kAppStartAddress);
    }

    // 走到这里即进入 DFU。按上方判定的优先级上报原因: 按键优先于一切, 其次是
    // 主机显式请求, 再次是上次半途而废的会话, 其余情况即镜像不可用。必须在
    // tusb_rhport_init() 之前设置 -- 字符串描述符只在枚举时读取, 此后不再刷新。
    const auto entry_reason = [&] {
        if (force_stay)
            return usb::DfuEntryReason::kUserKey;
        if (force_dfu)
            return usb::DfuEntryReason::kHostRequest;
        if (flash::Metadata::get_instance().previous_session_interrupted())
            return usb::DfuEntryReason::kInterrupted;
        return usb::DfuEntryReason::kNoValidApp;
    }();
    usb::get_usb_descriptors().set_entry_reason(entry_reason);

    utility::assert_always(tusb_rhport_init(0, nullptr));

    while (true) {
        tud_task();
        usb::Dfu::instance().poll();
    }
}
