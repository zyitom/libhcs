#pragma once

#include <cstdint>

#include "firmware/hpm_board/bootloader/src/flash/layout.hpp"
#include "firmware/hpm_board/bootloader/src/flash/xpi_nor.hpp"

namespace libhcs::firmware::flash {

// Every mutating entry point returns false instead of trapping on a flash
// failure. The bootloader is the last line of recovery: a metadata sector that
// has become unwritable must leave the device sitting in DFU reporting an error
// status, not fault into a state no host can talk to.
class Metadata {
public:
    static Metadata& get_instance() {
        static Metadata image_metadata;
        return image_metadata;
    }

    bool is_ready() const { return latest_valid_slot_state_ == DataSlotState::kReady; }
    bool is_flashing() const { return latest_valid_slot_state_ == DataSlotState::kFlashing; }

    uint32_t image_size() const { return latest_valid_slot_->image_size; }

    bool begin_flashing() {
        if (!latest_valid_slot_ || latest_valid_slot_state_ == DataSlotState::kFatal) {
            if (!erase_and_rescan())
                return false;
        } else if (latest_valid_slot_state_ == DataSlotState::kEmpty) {

        } else if (latest_valid_slot_state_ == DataSlotState::kFlashing) {
            // Reuse the marker an interrupted session already left behind: the
            // ready record goes into this same slot, word by word.
            return true;
        } else if (latest_valid_slot_state_ == DataSlotState::kReady) {
            const auto next_addr =
                reinterpret_cast<uintptr_t>(latest_valid_slot_) + sizeof(DataSlot);
            if (next_addr >= kMetadataEndAddress) {
                if (!erase_and_rescan())
                    return false;
            } else {
                latest_valid_slot_ = reinterpret_cast<DataSlot*>(next_addr);
            }
        }

        if (!latest_valid_slot_->enter_flashing_state())
            return false;

        latest_valid_slot_state_ = DataSlotState::kFlashing;
        return true;
    }

    bool finish_flashing(uint32_t size) {
        if (latest_valid_slot_state_ != DataSlotState::kFlashing)
            return false;

        if (!latest_valid_slot_->enter_ready_state(size))
            return false;

        latest_valid_slot_state_ = DataSlotState::kReady;
        return true;
    }

private:
    Metadata() { scan_latest_valid_slot(); }

    static constexpr uint32_t kFlashWordErased = 0xFFFFFFFFU;
    static constexpr uint32_t kImageMetadataMagic = 0x48435331U; // "HCS1"
    static constexpr uint32_t kImageStateReady = 0x494D5244U;    // "IMRD"

    enum class DataSlotState : uint8_t {
        kFatal,
        kEmpty,
        kFlashing,
        kReady,
    };

    struct DataSlot {
        volatile uint32_t magic;
        volatile uint32_t image_state;
        volatile uint32_t image_size;
        // Retired CRC32 word. The image is covered by the SHA-256 suffix, which
        // subsumes a CRC entirely; the word is kept so the on-flash slot layout
        // and stride are unchanged and sectors written by earlier bootloaders
        // still read back correctly. It is left erased.
        volatile uint32_t reserved;

        DataSlotState read_state() const {
            switch (magic) {
            case kFlashWordErased:
                return (image_state == kFlashWordErased && image_size == kFlashWordErased
                        && reserved == kFlashWordErased)
                         ? DataSlotState::kEmpty
                         : DataSlotState::kFatal;

            case kImageMetadataMagic:
                switch (image_state) {
                case kFlashWordErased:
                    return (image_size == kFlashWordErased && reserved == kFlashWordErased)
                             ? DataSlotState::kFlashing
                             : DataSlotState::kFatal;
                case kImageStateReady:
                    return (image_size <= kAppMaxImageSize) ? DataSlotState::kReady
                                                            : DataSlotState::kFatal;
                default: return DataSlotState::kFatal;
                }

            default: return DataSlotState::kFatal;
            }
        }

        bool enter_flashing_state() {
            if (read_state() != DataSlotState::kEmpty)
                return false;

            if (!XpiNor::instance().program_word(
                    reinterpret_cast<uintptr_t>(&magic), kImageMetadataMagic))
                return false;

            return read_state() == DataSlotState::kFlashing;
        }

        bool enter_ready_state(uint32_t size) {
            if (read_state() != DataSlotState::kFlashing)
                return false;

            auto& xpi_nor = XpiNor::instance();
            // Size first, state last: the state word is the commit barrier, so a
            // power loss between the two leaves the slot unrecognizable rather
            // than valid-looking with a missing size.
            if (!xpi_nor.program_word(reinterpret_cast<uintptr_t>(&image_size), size))
                return false;
            if (!xpi_nor.program_word(reinterpret_cast<uintptr_t>(&image_state), kImageStateReady))
                return false;

            return read_state() == DataSlotState::kReady;
        }
    };

    void scan_latest_valid_slot() {
        static_assert(kMetadataStartAddress % sizeof(DataSlot) == 0U);
        static_assert(kMetadataEndAddress % sizeof(DataSlot) == 0U);

        latest_valid_slot_ = nullptr;
        latest_valid_slot_state_ = DataSlotState::kFatal;

        for (uintptr_t addr = kMetadataStartAddress; addr < kMetadataEndAddress;
             addr += sizeof(DataSlot)) {
            auto& slot = *reinterpret_cast<DataSlot*>(addr);
            switch (const auto state = slot.read_state()) {
            case DataSlotState::kFatal:
                latest_valid_slot_ = nullptr;
                latest_valid_slot_state_ = DataSlotState::kFatal;
                return;

            case DataSlotState::kEmpty:
                if (!latest_valid_slot_) {
                    latest_valid_slot_ = &slot;
                    latest_valid_slot_state_ = DataSlotState::kEmpty;
                }
                break;

            case DataSlotState::kFlashing:
            case DataSlotState::kReady:
                if (!latest_valid_slot_
                    || (reinterpret_cast<uintptr_t>(latest_valid_slot_) + sizeof(DataSlot)
                            == reinterpret_cast<uintptr_t>(&slot)
                        && latest_valid_slot_state_ == DataSlotState::kReady)) {
                    latest_valid_slot_ = &slot;
                    latest_valid_slot_state_ = state;
                } else {
                    latest_valid_slot_ = nullptr;
                    latest_valid_slot_state_ = DataSlotState::kFatal;
                    return;
                }
                break;
            }
        }
    }

    bool erase_and_rescan() {
        if (!XpiNor::instance().erase_sector(kMetadataStartAddress))
            return false;

        scan_latest_valid_slot();
        return latest_valid_slot_ != nullptr && latest_valid_slot_state_ == DataSlotState::kEmpty;
    }

    DataSlot* latest_valid_slot_ = nullptr;
    DataSlotState latest_valid_slot_state_ = DataSlotState::kFatal;
};

} // namespace libhcs::firmware::flash
