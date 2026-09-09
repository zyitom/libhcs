#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

extern "C" {
#include "ch32h417.h"
}

#include "firmware/ch32_board/boot/src/crypto/sha256.hpp"
#include "firmware/ch32_board/boot/src/flash/layout.hpp"
#include "firmware/ch32_board/boot/src/flash/unlock_guard.hpp"
#include "firmware/ch32_board/boot/src/utility/assert.hpp"

namespace libhcs::firmware::flash {

// One record in the metadata block describing the image currently in the
// application slot. Written last, after the image itself has been programmed
// and read back, so a power loss mid-download leaves the record absent (or
// still marked in-progress) and the bootloader refuses to launch a torn image.
struct Metadata {
    static constexpr uint32_t kMagicValid = 0x314D4752;    // "RGM1"
    static constexpr uint32_t kMagicFlashing = 0x30474C46; // "FLG0"

    uint32_t magic;
    uint32_t image_size;
    uint8_t sha256[crypto::kSha256DigestSize];

    [[nodiscard]] bool is_valid_record() const {
        return magic == kMagicValid && image_size > 0 && image_size <= kAppMaxImageSize;
    }
};

static_assert(sizeof(Metadata) % sizeof(uint32_t) == 0, "programmed word by word");
static_assert(sizeof(Metadata) <= kMetadataEndAddress - kMetadataStartAddress);

class MetadataStore {
public:
    static const Metadata& stored() {
        return *reinterpret_cast<const Metadata*>(kMetadataStartAddress);
    }

    // Erase the block and mark a download in progress, so an interrupted
    // session cannot leave the previous (now partially overwritten) image
    // looking valid.
    static bool begin_flashing() {
        const UnlockGuard guard;
        if (FLASH_ErasePage(to_physical(kMetadataStartAddress)) != FLASH_COMPLETE)
            return false;

        Metadata record = {};
        record.magic = Metadata::kMagicFlashing;
        return program_record(record);
    }

    // Commit the record for a fully downloaded, hash-verified image.
    static bool commit(uint32_t image_size, const uint8_t (&digest)[crypto::kSha256DigestSize]) {
        Metadata record = {};
        record.magic = Metadata::kMagicValid;
        record.image_size = image_size;
        std::memcpy(record.sha256, digest, sizeof(record.sha256));

        const UnlockGuard guard;
        // The in-progress marker left the rest of the block erased, but the
        // magic word itself was already programmed, so the block has to be
        // erased again before the final record goes down.
        if (FLASH_ErasePage(to_physical(kMetadataStartAddress)) != FLASH_COMPLETE)
            return false;
        return program_record(record);
    }

private:
    static bool program_record(const Metadata& record) {
        // Program the magic LAST so a power loss part-way leaves the record
        // unrecognizable rather than valid-looking with a stale size/digest.
        const auto* words = reinterpret_cast<const uint32_t*>(&record);
        constexpr size_t kWordCount = sizeof(Metadata) / sizeof(uint32_t);

        for (size_t i = 1; i < kWordCount; ++i) {
            const uintptr_t address = kMetadataStartAddress + (i * sizeof(uint32_t));
            if (FLASH_ProgramWord(to_physical(address), words[i]) != FLASH_COMPLETE)
                return false;
        }
        return FLASH_ProgramWord(to_physical(kMetadataStartAddress), words[0]) == FLASH_COMPLETE;
    }
};

} // namespace libhcs::firmware::flash
