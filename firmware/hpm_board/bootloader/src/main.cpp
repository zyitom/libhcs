#include <board.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <tusb.h>

#include "firmware/hpm_board/bootloader/src/flash/staging.hpp"
#include "firmware/hpm_board/bootloader/src/flash/validation.hpp"
#include "firmware/hpm_board/bootloader/src/usb/dfu.hpp"
#include "firmware/hpm_board/bootloader/src/usb/usb_descriptors.hpp"
#include "firmware/hpm_board/bootloader/src/utility/assert.hpp"
#include "firmware/hpm_board/bootloader/src/utility/boot_mailbox.hpp"
#include "firmware/hpm_board/bootloader/src/utility/jump.hpp"
#include "firmware/hpm_board/common/board_identity.hpp"

int main() {
    using namespace libhcs::firmware; // NOLINT(google-build-using-namespace)

    const bool force_stay = board_check_bootloader_force_stay_requested();

    // 任何动作之前先从 OTP 读板型。word 25 非两个已知值时无条件拒绝跳转 --
    // 即使 app 镜像校验完好: app 必须按板型之一配置 PA30/PA31, 不存在安全默认。
    // 此时设备落入 DFU 循环, 以哨兵 PID 枚举, product string 携带肇事的 word 25;
    // DFU 下载同样被拒, 唯一出路是人工介入。
    //
    // 注意: 这是刻意为之的硬停(明确要求过)。word 25 非 0 非 2 的芯片完全无法
    // 经 USB 恢复, 救援需将 PA07 拉到 GND 或使用 J-Link。这是绝不把收发器
    // 灌进 LED 网络的代价。
    const bool board_recognized = board::board_identity().recognized();

    // 安装 app 暂存的固件镜像(FoE 或 USB 自检路径)。放在跳转判断之前, 使下面
    // 校验并进入的是刚装好的镜像; 也放在 board_init() 之前, 与本路径其余步骤
    // 一致 -- ROM flash API 只在复位时钟下工作。
    //
    // 门控条件与跳转相同: 按住按键表示操作者要 DFU 而非安装; 未识别的板不能
    // 被交给新固件, 正如不能被交给控制权。这里不涉及 BootMailbox 请求 --
    // 已提交的 staging 记录本身就是请求, 安装中途掉电因此能在下次启动恢复,
    // 无需任何易失状态。
    if (board_recognized && !force_stay)
        (void)flash::install_staged_image_if_ready();

#if libhcs_BOOTLOADER_MODE_AUTO
    if (board_recognized && !force_stay && !boot::BootMailbox::consume_enter_dfu_request()
        && flash::validate_app_image())
#else
    if (board_recognized && !force_stay && boot::BootMailbox::consume_boot_app_once_request()
        && flash::validate_app_image())
#endif
        utility::jump_to_app();

    // 复位时钟已让 CPU0 运行在 360 MHz, board_init() 只是把它提到 480 MHz,
    // 却给直进 app 的路径平添可避免的启动延迟。
    board_init();
    board_init_usb();
    (void)usb::get_usb_descriptors();

    const tusb_rhport_init_t init_config{
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    utility::assert_always(tusb_rhport_init(0, &init_config));

    while (true) {
        tud_task();
        usb::Dfu::instance().poll();
    }
}
