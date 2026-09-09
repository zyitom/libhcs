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

    // Board identity from OTP, before anything can act on it. An unrecognized
    // value blocks the jump unconditionally -- even with a perfectly valid,
    // signed app image present -- because the app configures PA30/PA31 for one
    // variant or the other and there is no safe default. The device instead falls
    // through into the DFU loop and enumerates under the sentinel PID with the
    // offending word 25 in its product string; DFU downloads are refused there
    // too, so the only way out is deliberate human action.
    //
    // NOTE: this is a hard stop by design (requested explicitly). A chip whose
    // word 25 is neither 0 nor 2 cannot be recovered over USB at all. Rescue
    // needs PA07 pulled to GND or a J-Link. That is the accepted trade for never
    // mis-driving a transceiver against an LED network.
    const bool board_recognized = board::board_identity().recognized();

    // Install a firmware image the app staged for us (FoE, or the USB self-test
    // path). Runs before the jump decision so the freshly installed image is the
    // one validated and entered below -- and before board_init(), like the rest
    // of this path, because the ROM flash API works at reset-time clocks.
    //
    // Gated on the same two conditions as the jump: a held key means the operator
    // wants DFU, not an install, and an unrecognized board must not be handed new
    // firmware any more than it may be handed control. No BootMailbox request is
    // involved -- a committed staging record IS the request, which is what lets a
    // power loss mid-install resume on the next boot with no volatile state.
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

    // Reset-time clocks already run CPU0 at 360 MHz, so board_init() would only bump it to
    // 480 MHz while adding avoidable startup latency on the direct-to-app path.
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
