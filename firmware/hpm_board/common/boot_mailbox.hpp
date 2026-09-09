#pragma once

#include <array>
#include <cstdint>

#include <board.h>
#include <hpm_common.h>
#include <hpm_ppor_drv.h>
#include <hpm_soc.h>

#if defined(BOARD_BGPR)
#include <hpm_bgpr_drv.h>
#endif

#if defined(HPM_PDGO_BASE)
#include <hpm_pdgo_drv.h>
#endif

namespace libhcs::firmware::boot {

class BootMailbox {
public:
    static void clear() { write_pair(0U, 0U); }

    static void request_enter_dfu() { write_pair(kMailboxMagic, kMailboxRequestEnterDfu); }

    static void request_boot_app_once() {
        write_pair(kMailboxMagic, kMailboxRequestBootAppOnce);
    }

    static bool consume_enter_dfu_request() { return consume_request(kMailboxRequestEnterDfu); }

    static bool consume_boot_app_once_request() {
        return consume_request(kMailboxRequestBootAppOnce);
    }

    [[noreturn]] static void reboot_to_bootloader() {
        request_enter_dfu();
        reboot();
    }

    [[noreturn]] static void reboot_to_app_once() {
        request_boot_app_once();
        reboot();
    }

    [[noreturn]] static void reboot() {
        ppor_reset_mask_set_source_enable(HPM_PPOR, ppor_reset_software);
        // HPM6E dual-core DFU must reset CPU1; a hot reset only restarts CPU0.
        ppor_reset_set_cold_reset_enable(HPM_PPOR, ppor_reset_software);
        ppor_sw_reset(HPM_PPOR, 10U);
        while (true) {}
    }

private:
    static constexpr uint32_t kMailboxMagic = 0x48435331U;              // "HCS1"
    static constexpr uint32_t kMailboxRequestEnterDfu = 0x44465530U;    // "DFU0"
    static constexpr uint32_t kMailboxRequestBootAppOnce = 0x41505031U; // "APP1"
    static constexpr uint8_t kMagicGprIndex = 2U;
    static constexpr uint8_t kRequestGprIndex = 3U;

    static bool consume_request(uint32_t request) {
        const auto values = read_pair();
        clear();
        return values[0] == kMailboxMagic && values[1] == request;
    }

    static std::array<uint32_t, 2> read_pair() {
        std::array<uint32_t, 2> values{};

#if defined(BOARD_BGPR)
        uint32_t magic = 0U;
        uint32_t request = 0U;
        if (bgpr_read32(BOARD_BGPR, kMagicGprIndex, &magic) == status_success
            && bgpr_read32(BOARD_BGPR, kRequestGprIndex, &request) == status_success
            && magic == kMailboxMagic) {
            return {magic, request};
        }
#endif

#if defined(HPM_PDGO_BASE)
        if (pdgo_is_retention_mode_enabled(HPM_PDGO)) {
            const uint32_t magic = pdgo_read_gpr(HPM_PDGO, kMagicGprIndex);
            const uint32_t request = pdgo_read_gpr(HPM_PDGO, kRequestGprIndex);
            if (magic == kMailboxMagic)
                return {magic, request};
        }
#endif

        return values;
    }

    static void write_pair(uint32_t magic, uint32_t request) {
#if defined(BOARD_BGPR)
        (void)bgpr_write32(BOARD_BGPR, kRequestGprIndex, request);
        (void)bgpr_write32(BOARD_BGPR, kMagicGprIndex, magic);
#endif

#if defined(HPM_PDGO_BASE)
        if ((magic != 0U || request != 0U) && !pdgo_is_retention_mode_enabled(HPM_PDGO))
            pdgo_enable_retention_mode(HPM_PDGO);
        if (pdgo_is_retention_mode_enabled(HPM_PDGO)) {
            pdgo_write_gpr(HPM_PDGO, kRequestGprIndex, request);
            pdgo_write_gpr(HPM_PDGO, kMagicGprIndex, magic);
        }
#endif
    }
};

} // namespace libhcs::firmware::boot
