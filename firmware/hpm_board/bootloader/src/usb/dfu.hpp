#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <class/dfu/dfu.h>
#include <hpm_clock_drv.h>
#include <hpm_mchtmr_drv.h>
#include <hpm_soc.h>

#include "firmware/hpm_board/bootloader/src/flash/layout.hpp"
#include "firmware/hpm_board/bootloader/src/flash/metadata.hpp"
#include "firmware/hpm_board/bootloader/src/flash/validation.hpp"
#include "firmware/hpm_board/bootloader/src/flash/writer.hpp"
#include "firmware/hpm_board/bootloader/src/utility/boot_mailbox.hpp"
#include "firmware/hpm_board/common/board_identity.hpp"

namespace libhcs::firmware::usb {

class Dfu {
public:
    static Dfu& instance() {
        static Dfu dfu;
        return dfu;
    }

    uint32_t get_timeout_ms(uint8_t alt, uint8_t state) const {
        if (alt != kDfuAltFlash)
            return 0U;

        if (state == DFU_DNBUSY) {
            // TinyUSB invokes download() after acknowledging GETSTATUS. Most
            // blocks only fill the RAM sector buffer, but every fourth block
            // synchronously erases and programs a 4 KiB flash sector. Give the
            // host an honest poll interval for those operations so it does not
            // start another control transfer while the ROM flash call is busy.
            if (!session_started_)
                return kSectorCommitTimeoutMs;

            const uint32_t sector_offset = downloaded_size_ % flash::kFlashSectorSize;
            if (sector_offset + flash::Writer::kTransferBlockSize >= flash::kFlashSectorSize)
                return kSectorCommitTimeoutMs;

            return kBufferedBlockTimeoutMs;
        }

        if (state == DFU_MANIFEST)
            return kManifestTimeoutMs;

        return 0U;
    }

    uint8_t download(uint8_t alt, uint16_t block_num, const uint8_t* data, uint16_t length) {
        if (alt != kDfuAltFlash)
            return DFU_STATUS_ERR_TARGET;

        // Refuse every write on a board whose OTP identity is not one of the two
        // known values. Writing an app that then configures PA30/PA31 for the
        // wrong variant is the failure this exists to prevent, and there is no
        // way to pick a safe default from here -- so nothing gets flashed until
        // someone reads the reported word 25 and decides what the board is.
        // ERR_TARGET is the honest status: the device cannot accept an image for
        // this target. dfu-util surfaces it as a failed download rather than a
        // silent success.
        if (!board::board_identity().recognized())
            return DFU_STATUS_ERR_TARGET;

        if (length == 0U)
            return DFU_STATUS_ERR_NOTDONE;

        if (session_started_ && block_num == 0U)
            reset_transfer_state();

        if (!session_started_) {
            if (block_num != 0U)
                return DFU_STATUS_ERR_ADDRESS;

            flash_writer_.begin_session();

            // Reported rather than fatal: a metadata sector that will not erase
            // used to trap here, which took the device off the bus with no way
            // for the host to tell what happened.
            if (!flash::Metadata::get_instance().begin_flashing())
                return fail(DFU_STATUS_ERR_ERASE);

            session_started_ = true;
            expected_block_ = 0U;
            downloaded_size_ = 0U;
            reset_requested_ = false;
        }

        if (block_num != expected_block_)
            return fail(DFU_STATUS_ERR_ADDRESS);

        const uint64_t write_address_64 = static_cast<uint64_t>(flash::kAppStartAddress)
                                        + static_cast<uint64_t>(downloaded_size_);
        const uint64_t write_end_64 = write_address_64 + static_cast<uint64_t>(length);
        const uint64_t downloaded_size_64 =
            static_cast<uint64_t>(downloaded_size_) + static_cast<uint64_t>(length);

        if (write_address_64 >= static_cast<uint64_t>(flash::kAppEndAddress))
            return fail(DFU_STATUS_ERR_ADDRESS);
        if (write_end_64 > static_cast<uint64_t>(flash::kAppEndAddress))
            return fail(DFU_STATUS_ERR_ADDRESS);
        if (downloaded_size_64 > static_cast<uint64_t>(flash::kAppMaxImageSize))
            return fail(DFU_STATUS_ERR_ADDRESS);

        if (data == nullptr)
            return fail(DFU_STATUS_ERR_UNKNOWN);

        const auto payload = std::span<const std::byte>(
            reinterpret_cast<const std::byte*>(data), static_cast<size_t>(length));
        if (!flash_writer_.write(static_cast<uintptr_t>(write_address_64), payload))
            return fail(DFU_STATUS_ERR_PROG);

        downloaded_size_ = static_cast<uint32_t>(downloaded_size_64);
        expected_block_ = static_cast<uint16_t>(expected_block_ + 1U);
        return DFU_STATUS_OK;
    }

    uint8_t manifest(uint8_t alt) {
        if (alt != kDfuAltFlash)
            return DFU_STATUS_ERR_TARGET;

        if (!session_started_ || downloaded_size_ == 0U)
            return DFU_STATUS_ERR_NOTDONE;

        if (!flash_writer_.finish_session())
            return fail(DFU_STATUS_ERR_PROG);

        if (!flash::validate_candidate_image(downloaded_size_))
            return fail(DFU_STATUS_ERR_FIRMWARE);

        auto& metadata = flash::Metadata::get_instance();
        if (!metadata.finish_flashing(downloaded_size_))
            return fail(DFU_STATUS_ERR_WRITE);

        if (!metadata.is_ready() || metadata.image_size() != downloaded_size_)
            return fail(DFU_STATUS_ERR_VERIFY);

        reset_transfer_state();
        reset_requested_ = true;
        reset_requested_tick_ = mchtmr_get_count(HPM_MCHTMR);
        return DFU_STATUS_OK;
    }

    void abort(uint8_t alt) {
        if (alt != kDfuAltFlash)
            return;

        reset_transfer_state();
    }

    [[noreturn]] static void detach() { boot::BootMailbox::reboot_to_app_once(); }

    void poll() const {
        if (!reset_requested_)
            return;

        const uint64_t elapsed = mchtmr_get_count(HPM_MCHTMR) - reset_requested_tick_;
        const uint64_t reset_delay_ticks =
            (static_cast<uint64_t>(clock_get_frequency(clock_mchtmr0)) * kResetDelayMs) / 1000U;
        if (elapsed < reset_delay_ticks)
            return;

        detach();
    }

private:
    static constexpr uint8_t kDfuAltFlash = 0U;
    static constexpr uint32_t kResetDelayMs = 1500U;
    static constexpr uint32_t kBufferedBlockTimeoutMs = 1U;
    static constexpr uint32_t kSectorCommitTimeoutMs = 100U;
    static constexpr uint32_t kManifestTimeoutMs = 250U;
    static_assert(flash::Writer::kTransferBlockSize <= flash::kFlashSectorSize);

    Dfu() = default;

    uint8_t fail(uint8_t status) {
        reset_transfer_state();
        return status;
    }

    void reset_transfer_state() {
        flash_writer_.abort_session();
        expected_block_ = 0U;
        downloaded_size_ = 0U;
        session_started_ = false;
    }

    flash::Writer flash_writer_{};
    uint16_t expected_block_ = 0U;
    uint32_t downloaded_size_ = 0U;
    bool session_started_ = false;
    bool reset_requested_ = false;
    uint64_t reset_requested_tick_ = 0U;
};

} // namespace libhcs::firmware::usb
