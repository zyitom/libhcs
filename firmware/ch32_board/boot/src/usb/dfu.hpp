#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "firmware/ch32_board/boot/src/crypto/sha256.hpp"
#include "firmware/ch32_board/boot/src/flash/layout.hpp"
#include "firmware/ch32_board/boot/src/flash/metadata.hpp"
#include "firmware/ch32_board/boot/src/flash/validation.hpp"
#include "firmware/ch32_board/boot/src/flash/writer.hpp"
#include "firmware/ch32_board/boot/src/utility/boot_mailbox.hpp"

namespace libhcs::firmware::usb {

// DFU 1.1 constants (USB Device Firmware Upgrade spec, table 3.2 / A.1). Spelled
// out here rather than pulled from TinyUSB's class/dfu/dfu.h: TinyUSB has no USB
// 3.0 support at all, so it is not in this board's build, and these are just
// spec-defined enum values.
enum class DfuRequest : uint8_t {
    kDetach = 0,
    kDownload = 1,
    kUpload = 2,
    kGetStatus = 3,
    kClearStatus = 4,
    kGetState = 5,
    kAbort = 6,
};

enum class DfuState : uint8_t {
    kAppIdle = 0,
    kAppDetach = 1,
    kDfuIdle = 2,
    kDfuDownloadSync = 3,
    kDfuDownloadBusy = 4,
    kDfuDownloadIdle = 5,
    kDfuManifestSync = 6,
    kDfuManifest = 7,
    kDfuManifestWaitReset = 8,
    kDfuUploadIdle = 9,
    kDfuError = 10,
};

enum class DfuStatus : uint8_t {
    kOk = 0x00,
    kErrTarget = 0x01,
    kErrFile = 0x02,
    kErrWrite = 0x03,
    kErrErase = 0x04,
    kErrProgram = 0x06,
    kErrVerify = 0x07,
    kErrAddress = 0x08,
    kErrNotDone = 0x09,
    kErrFirmware = 0x0A,
    kErrUnknown = 0x0E,
    kErrStalledPkt = 0x0F,
};

// The six-byte payload of DFU_GETSTATUS.
struct DfuStatusResponse {
    uint8_t status;
    uint8_t poll_timeout[3]; // little-endian milliseconds
    uint8_t state;
    uint8_t string_index;
};
static_assert(sizeof(DfuStatusResponse) == 6);

// Transport-independent DFU download state machine. The USB layer feeds it the
// six class requests and it drives the flash writer, the metadata record and the
// hash verification; it never touches an endpoint itself, which is what lets the
// same logic sit on the WCH USBSS EP0 here and on TinyUSB elsewhere.
//
// Only DNLOAD is implemented: UPLOAD returns zero bytes (readback is refused on
// purpose -- the app slot is the customer's firmware) and the device advertises
// bitCanUpload = 0.
class Dfu {
public:
    // Reported in DFU_GETSTATUS so the host paces itself. Programming happens
    // synchronously inside download(), so the only real wait is manifestation.
    static constexpr uint32_t kDownloadPollTimeoutMs = 1;
    static constexpr uint32_t kManifestPollTimeoutMs = 100;

    [[nodiscard]] DfuState state() const { return state_; }

    void reset() {
        state_ = DfuState::kDfuIdle;
        status_ = DfuStatus::kOk;
        session_started_ = false;
        expected_block_ = 0;
        downloaded_size_ = 0;
        detach_requested_ = false;
    }

    // DFU_DNLOAD. An empty payload is the host saying "that was the last block";
    // anything else is firmware data for `block_number`. Returns false to stall
    // the control transfer.
    bool download(uint16_t block_number, std::span<const uint8_t> data) {
        if (state_ == DfuState::kDfuError)
            return false;

        if (data.empty())
            return finish_download();

        if (!session_started_) {
            // A session must start at block 0; anything else means the host and
            // device disagree about where the image begins.
            if (block_number != 0)
                return fail(DfuStatus::kErrAddress);

            if (!flash::MetadataStore::begin_flashing())
                return fail(DfuStatus::kErrErase);

            writer_.begin_session();
            session_started_ = true;
            expected_block_ = 0;
            downloaded_size_ = 0;
        }

        // DFU block numbers are a 16-bit wrapping counter; only sequential
        // delivery is supported, and a gap means a lost packet rather than
        // something to paper over.
        if (block_number != expected_block_)
            return fail(DfuStatus::kErrAddress);

        if (downloaded_size_ + data.size() > flash::kAppMaxImageSize)
            return fail(DfuStatus::kErrAddress);

        if (!writer_.write(data))
            return fail(DfuStatus::kErrProgram);

        downloaded_size_ += static_cast<uint32_t>(data.size());
        expected_block_ = static_cast<uint16_t>(expected_block_ + 1);
        state_ = DfuState::kDfuDownloadIdle;
        return true;
    }

    // DFU_GETSTATUS. Advances the sync states, which is where the spec expects
    // manifestation to actually run.
    DfuStatusResponse get_status() {
        if (state_ == DfuState::kDfuManifestSync)
            manifest();

        uint32_t poll_timeout_ms = 0;
        if (state_ == DfuState::kDfuDownloadBusy)
            poll_timeout_ms = kDownloadPollTimeoutMs;
        else if (state_ == DfuState::kDfuManifest)
            poll_timeout_ms = kManifestPollTimeoutMs;

        DfuStatusResponse response = {};
        response.status = static_cast<uint8_t>(status_);
        response.poll_timeout[0] = static_cast<uint8_t>(poll_timeout_ms & 0xFFU);
        response.poll_timeout[1] = static_cast<uint8_t>((poll_timeout_ms >> 8) & 0xFFU);
        response.poll_timeout[2] = static_cast<uint8_t>((poll_timeout_ms >> 16) & 0xFFU);
        response.state = static_cast<uint8_t>(state_);
        response.string_index = 0;
        return response;
    }

    // DFU_CLRSTATUS: only legal from dfuERROR, and it drops any partial image.
    bool clear_status() {
        if (state_ != DfuState::kDfuError)
            return false;
        reset();
        return true;
    }

    // DFU_ABORT: back to idle, partial download discarded.
    bool abort() {
        if (state_ == DfuState::kDfuError)
            return false;
        session_started_ = false;
        expected_block_ = 0;
        downloaded_size_ = 0;
        state_ = DfuState::kDfuIdle;
        return true;
    }

    // DFU_DETACH. In DFU mode (bitWillDetach = 1) the host follows with a bus
    // reset; the caller reboots once the status stage has completed.
    void detach() { detach_requested_ = true; }

    [[nodiscard]] bool detach_requested() const { return detach_requested_; }

    // True once a downloaded image has been committed and the device should
    // reset into it.
    [[nodiscard]] bool manifestation_complete() const {
        return state_ == DfuState::kDfuManifestWaitReset;
    }

private:
    bool finish_download() {
        if (!session_started_ || downloaded_size_ == 0)
            return fail(DfuStatus::kErrNotDone);

        if (!writer_.finish())
            return fail(DfuStatus::kErrProgram);

        state_ = DfuState::kDfuManifestSync;
        return true;
    }

    // Hash what actually landed in flash and commit the record. Reading back
    // through the alias means this verifies the flash contents, not a RAM copy
    // of what we intended to write.
    void manifest() {
        state_ = DfuState::kDfuManifest;

        uint8_t digest[crypto::kSha256DigestSize];
        flash::hash_region(flash::kAppStartAddress, downloaded_size_, digest);

        if (!flash::MetadataStore::commit(downloaded_size_, digest)) {
            fail(DfuStatus::kErrWrite);
            return;
        }

        if (!flash::app_image_is_valid()) {
            fail(DfuStatus::kErrVerify);
            return;
        }

        // Tell the next boot to launch the app rather than re-entering DFU.
        utility::boot_mailbox().request_boot_app_once();
        state_ = DfuState::kDfuManifestWaitReset;
    }

    bool fail(DfuStatus status) {
        status_ = status;
        state_ = DfuState::kDfuError;
        session_started_ = false;
        return false;
    }

    flash::Writer writer_;
    DfuState state_ = DfuState::kDfuIdle;
    DfuStatus status_ = DfuStatus::kOk;
    uint32_t downloaded_size_ = 0;
    uint16_t expected_block_ = 0;
    bool session_started_ = false;
    bool detach_requested_ = false;
};

inline constinit Dfu dfu;

} // namespace libhcs::firmware::usb
