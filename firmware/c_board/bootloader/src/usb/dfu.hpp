#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <class/dfu/dfu.h>
#include <main.h>

#include "firmware/c_board/bootloader/src/flash/layout.hpp"
#include "firmware/c_board/bootloader/src/flash/metadata.hpp"
#include "firmware/c_board/bootloader/src/flash/validation.hpp"
#include "firmware/c_board/bootloader/src/flash/writer.hpp"
#include "firmware/c_board/bootloader/src/utility/boot_mailbox.hpp"

namespace libhcs::firmware::usb {

class Dfu {
public:
    static Dfu& instance() {
        static Dfu dfu;
        return dfu;
    }

    static uint32_t get_timeout_ms(uint8_t alt, uint8_t state) {
        if (alt != kDfuAltFlash)
            return 0U;

        if (state == DFU_DNBUSY)
            return 1U;

        if (state == DFU_MANIFEST)
            return 100U;

        return 0U;
    }

    uint8_t download(uint8_t alt, uint16_t block_num, const uint8_t* data, uint16_t length) {
        if (alt != kDfuAltFlash)
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
            reset_requested_tick_ = 0U;
        }

        if (block_num != expected_block_)
            return fail(DFU_STATUS_ERR_ADDRESS);

        const uint64_t write_address_64 = static_cast<uint64_t>(flash::kAppStartAddress)
                                        + static_cast<uint64_t>(downloaded_size_);
        if (write_address_64 >= static_cast<uint64_t>(flash::kAppEndAddress))
            return fail(DFU_STATUS_ERR_ADDRESS);

        const uint64_t write_end_64 = write_address_64 + static_cast<uint64_t>(length);
        if (write_end_64 > static_cast<uint64_t>(flash::kAppEndAddress))
            return fail(DFU_STATUS_ERR_ADDRESS);

        const uint64_t downloaded_size_64 =
            static_cast<uint64_t>(downloaded_size_) + static_cast<uint64_t>(length);
        if (downloaded_size_64 > static_cast<uint64_t>(flash::kAppMaxImageSize))
            return fail(DFU_STATUS_ERR_ADDRESS);

        if (data == nullptr)
            return fail(DFU_STATUS_ERR_UNKNOWN);

        const auto payload = std::span<const std::byte>(
            reinterpret_cast<const std::byte*>(data), static_cast<size_t>(length));
        if (!flash_writer_.write(static_cast<uint32_t>(write_address_64), payload))
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

        utility::boot_mailbox.request_boot_app_once();
        reset_transfer_state();
        reset_requested_ = true;
        reset_requested_tick_ = HAL_GetTick();

        return DFU_STATUS_OK;
    }

    void abort(uint8_t alt) {
        if (alt != kDfuAltFlash)
            return;

        reset_transfer_state();
    }

    static void detach() {
        __DSB();
        __ISB();
        NVIC_SystemReset();
    }

    void poll() const {
        if (!reset_requested_)
            return;

        if ((HAL_GetTick() - reset_requested_tick_) < kResetDelayMs)
            return;

        __DSB();
        __ISB();
        NVIC_SystemReset();
    }

private:
    static constexpr uint8_t kDfuAltFlash = 0U;
    static constexpr uint32_t kResetDelayMs = 1500U;

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
    uint32_t reset_requested_tick_ = 0U;
};

} // namespace libhcs::firmware::usb
