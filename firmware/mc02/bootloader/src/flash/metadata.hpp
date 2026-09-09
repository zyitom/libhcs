#pragma once

#include <cstdint>
#include <cstring>

#include <main.h>

#include "firmware/mc02/bootloader/src/flash/layout.hpp"
#include "firmware/mc02/bootloader/src/flash/unlock_guard.hpp"

namespace libhcs::firmware::flash {

// STM32H7 flash word = 256 bits = 32 bytes; each DataSlot occupies exactly one flash word.
//
// Every mutating entry point returns false instead of trapping on a flash
// failure. The bootloader is the last line of recovery: a metadata sector that
// has become unwritable must leave the device sitting in DFU reporting an error
// status, not HardFault into a state no host can talk to.
class Metadata {
public:
    static Metadata& get_instance() {
        static Metadata image_metadata;
        return image_metadata;
    }

    bool is_ready()    const { return latest_valid_slot_state_ == DataSlotState::kReady; }
    bool is_flashing() const { return latest_valid_slot_state_ == DataSlotState::kFlashing; }

    // Latched at construction, before any session can overwrite the state: a
    // flashing marker with no ready record after it means the previous download
    // lost power between the two writes. is_flashing() cannot answer this on its
    // own, because begin_flashing() puts the live session into the same state.
    bool previous_session_interrupted() const { return previous_session_interrupted_; }

    uint32_t image_size() const { return latest_valid_slot_->image_size; }

    bool begin_flashing() {
        if (!latest_valid_slot_ || latest_valid_slot_state_ == DataSlotState::kFatal) {
            if (!erase_and_rescan())
                return false;
        } else if (latest_valid_slot_state_ == DataSlotState::kEmpty) {
            // Use current empty slot -- already positioned
        } else {
            // kFlashing or kReady: advance to the next slot. Running off the end
            // is caught by the room check below, which every branch shares.
            latest_valid_slot_ = reinterpret_cast<DataSlot*>(
                reinterpret_cast<uintptr_t>(latest_valid_slot_) + sizeof(DataSlot));
        }

        // A session consumes two slots: the flashing marker written just below,
        // and the ready record finish_flashing() places in the slot after it.
        // Reserving only one let begin_flashing() settle on the very last slot
        // of the sector, leaving finish_flashing() nowhere to put the ready
        // record -- the download would run to completion and only then fail at
        // manifestation.
        //
        // An uninterrupted sequence never reaches that state, because sessions
        // land on every other slot and the sector fills exactly. It takes a
        // session that lost power between the marker and the ready record: the
        // recovery shifts every later session by one slot, and the last one
        // before the sector wraps then starts on the final slot. Checked here,
        // after the branches, so it holds however the slot was chosen.
        if (!has_room_for_session(latest_valid_slot_)) {
            if (!erase_and_rescan())
                return false;
        }

        if (!latest_valid_slot_->enter_flashing_state())
            return false;

        latest_valid_slot_state_ = DataSlotState::kFlashing;
        return true;
    }

    // On H7 we cannot modify a written flash word, so the ready record goes in
    // the NEXT slot after the flashing-marker slot.
    bool finish_flashing(uint32_t size) {
        if (latest_valid_slot_state_ != DataSlotState::kFlashing)
            return false;

        // begin_flashing() reserved this slot, so the check should never fire;
        // it stays as a real bounds check rather than an assertion because the
        // ready record must never be allowed to land one slot past the end of
        // the metadata sector, which is the first flash word of the application
        // image.
        const auto next_addr = reinterpret_cast<uintptr_t>(latest_valid_slot_) + sizeof(DataSlot);
        if (next_addr + sizeof(DataSlot) > kMetadataEndAddress)
            return false;

        auto* ready_slot = reinterpret_cast<DataSlot*>(next_addr);
        if (!ready_slot->enter_ready_state(size))
            return false;

        latest_valid_slot_ = ready_slot;
        latest_valid_slot_state_ = DataSlotState::kReady;
        return true;
    }

private:
    Metadata() {
        scan_latest_valid_slot();
        previous_session_interrupted_ = latest_valid_slot_state_ == DataSlotState::kFlashing;
    }

    static constexpr uintptr_t kMetadataStartAddress = 0x08020000U; // Sector 1
    static constexpr uintptr_t kMetadataEndAddress   = 0x08040000U;

    static constexpr uint32_t kFlashWordErased    = 0xFFFFFFFF;
    static constexpr uint32_t kImageMetadataMagic = 0x48435331; // "HCS1"
    static constexpr uint32_t kImageStateReady    = 0x494D5244; // "IMRD"

    enum class DataSlotState : uint8_t { kFatal, kEmpty, kFlashing, kReady };

    // Exactly 32 bytes -- one STM32H7 flash word.
    struct [[gnu::aligned(32)]] DataSlot {
        uint32_t magic;
        uint32_t image_state;
        uint32_t image_size;
        // Retired CRC32 word. The image is covered by the SHA-256 suffix, which
        // subsumes a CRC entirely; the word is kept so the on-flash slot layout
        // and stride are unchanged and sectors written by earlier bootloaders
        // still read back correctly. It is left erased.
        uint32_t reserved;
        uint8_t  pad[16];

        DataSlotState read_state() const {
            switch (magic) {
            case kFlashWordErased:
                if (image_state == kFlashWordErased && image_size == kFlashWordErased
                    && reserved == kFlashWordErased)
                    return DataSlotState::kEmpty;
                return DataSlotState::kFatal;
            case kImageMetadataMagic:
                if (image_state == kFlashWordErased)
                    return DataSlotState::kFlashing;
                if (image_state == kImageStateReady && image_size <= kAppMaxImageSize)
                    return DataSlotState::kReady;
                return DataSlotState::kFatal;
            default:
                return DataSlotState::kFatal;
            }
        }

        bool enter_flashing_state() {
            if (read_state() != DataSlotState::kEmpty)
                return false;
            if (!write_flash_word(kImageMetadataMagic, kFlashWordErased, kFlashWordErased))
                return false;
            return read_state() == DataSlotState::kFlashing;
        }

        bool enter_ready_state(uint32_t size) {
            if (read_state() != DataSlotState::kEmpty)
                return false;
            if (!write_flash_word(kImageMetadataMagic, kImageStateReady, size))
                return false;
            return read_state() == DataSlotState::kReady;
        }

    private:
        bool write_flash_word(uint32_t magic_val, uint32_t state_val, uint32_t size_val) {
            alignas(32) DataSlot buf{};
            buf.magic       = magic_val;
            buf.image_state = state_val;
            buf.image_size  = size_val;
            buf.reserved    = kFlashWordErased;
            std::memset(buf.pad, 0xFF, sizeof(buf.pad));

            const auto guard = UnlockGuard();
            if (!guard.ok())
                return false;

            // Bootloader runs cache-less (see main.cpp): flash reads are always
            // coherent, and issuing D-cache ops with the cache disabled faults.
            return HAL_FLASH_Program(
                       FLASH_TYPEPROGRAM_FLASHWORD, reinterpret_cast<uintptr_t>(this),
                       reinterpret_cast<uint32_t>(&buf))
                == HAL_OK;
        }
    };
    static_assert(sizeof(DataSlot) == 32);

    // A flashing session needs two consecutive slots: the flashing marker and
    // the ready record that follows it.
    static bool has_room_for_session(const DataSlot* slot) {
        return reinterpret_cast<uintptr_t>(slot) + 2U * sizeof(DataSlot) <= kMetadataEndAddress;
    }

    void scan_latest_valid_slot() {
        latest_valid_slot_ =
            reinterpret_cast<DataSlot*>(kMetadataStartAddress);
        latest_valid_slot_state_ = DataSlotState::kEmpty;

        for (uintptr_t addr = kMetadataStartAddress; addr < kMetadataEndAddress;
             addr += sizeof(DataSlot)) {
            auto* slot = reinterpret_cast<DataSlot*>(addr);
            const auto state = slot->read_state();

            if (state == DataSlotState::kFatal) {
                latest_valid_slot_ = nullptr;
                latest_valid_slot_state_ = DataSlotState::kFatal;
                return;
            }
            if (state == DataSlotState::kEmpty) {
                // First gap: everything after here is unwritten
                if (latest_valid_slot_state_ == DataSlotState::kEmpty)
                    latest_valid_slot_ = slot; // point at first empty
                return;
            }
            latest_valid_slot_ = slot;
            latest_valid_slot_state_ = state;
        }

        // Metadata sector is completely full -- treat as fatal to force erase
        latest_valid_slot_ = nullptr;
        latest_valid_slot_state_ = DataSlotState::kFatal;
    }

    bool erase_and_rescan() {
        FLASH_EraseInitTypeDef erase{};
        erase.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase.Banks     = FLASH_BANK_1;
        erase.Sector    = FLASH_SECTOR_1;
        erase.NbSectors = 1;

        uint32_t sector_error = 0U;
        {
            const auto guard = UnlockGuard();
            if (!guard.ok())
                return false;
            if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
                return false;
        }
        // Bootloader runs cache-less (see main.cpp): no D-cache maintenance needed.

        scan_latest_valid_slot();
        return latest_valid_slot_ != nullptr
            && latest_valid_slot_state_ == DataSlotState::kEmpty;
    }

    DataSlot*     latest_valid_slot_       = nullptr;
    DataSlotState latest_valid_slot_state_ = DataSlotState::kFatal;
    bool          previous_session_interrupted_ = false;
};

} // namespace libhcs::firmware::flash
