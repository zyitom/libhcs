#pragma once

#include <cstddef>
#include <cstdint>

#include <board.h>

// Cross-image contract for the FoE staging region.
//
// WHO WRITES WHAT, and why it is split this way:
//
//   * The RUNNING APP receives a firmware image (over FoE in BOOT state, or over
//     USB on the self-test path) and writes it into the staging region. It can
//     never write the app slot directly: core0 executes from that slot over XIP,
//     so the erase would remove the code performing it.
//
//   * The BOOTLOADER installs. After a cold reset it finds a Ready record here,
//     re-validates the staged image, and copies it into the app slot -- which it
//     may do safely because the bootloader lives in its own 124 KiB below the app
//     and never executes from the region it is erasing.
//
// CRASH SAFETY. The record is committed by programming `state` LAST, so every
// interruption lands on a state that is either "ignore" or "redo":
//
//   power loss while writing the image      -> state never set  -> ignored
//   power loss between `image_size`/`state` -> state never set  -> ignored
//   power loss after commit, before install -> Ready            -> installed next boot
//   power loss DURING install               -> still Ready      -> reinstalled next boot
//
// The staging record is cleared only after the app metadata has committed, which
// is what makes the last line hold. Do not reorder that.
//
// Unlike the app metadata sector this is a single fixed record rather than an
// append-only slot array: a staging session always rewrites the whole region, so
// there is nothing to append and one sector erase per session is already paid
// for by the image erase that follows it.

namespace libhcs::firmware::foe {

#if defined(BOARD_FOE_STAGING_ADDR)

inline constexpr bool kStagingSupported = true;

inline constexpr std::uintptr_t kStagingMetadataStart =
    BOARD_FLASH_BASE_ADDRESS + BOARD_FOE_STAGING_ADDR;
inline constexpr std::uintptr_t kStagingMetadataEnd =
    kStagingMetadataStart + BOARD_FOE_STAGING_METADATA_SIZE;
inline constexpr std::uintptr_t kStagingImageStart = kStagingMetadataEnd;
inline constexpr std::uintptr_t kStagingImageEnd =
    BOARD_FLASH_BASE_ADDRESS + BOARD_FOE_STAGING_END_OFFSET;

// Largest image the staging region accepts. Capped by what the APP SLOT holds,
// not by what staging holds -- staging is the larger of the two, and taking an
// image that fits here but not there would only defer the rejection to after the
// app slot has already been erased for it. BOARD_APP_FLASH_END_OFFSET bounds the
// app slot; 0x20000 is the bootloader + metadata reservation below it
// (kAppStartAddress in the bootloader's layout.hpp, which cannot be included
// from here because the app must not depend on bootloader headers).
inline constexpr std::size_t kStagingImageCapacity = kStagingImageEnd - kStagingImageStart;
inline constexpr std::size_t kAppSlotCapacity =
    (BOARD_FLASH_BASE_ADDRESS + BOARD_APP_FLASH_END_OFFSET) - (BOARD_FLASH_BASE_ADDRESS + 0x20000U);
inline constexpr std::uint32_t kStagingMaxImageSize = static_cast<std::uint32_t>(
    kStagingImageCapacity < kAppSlotCapacity ? kStagingImageCapacity : kAppSlotCapacity);

inline constexpr std::uint32_t kStagingMagic = 0x54534D52U;      // "RMST"
inline constexpr std::uint32_t kStagingStateReady = 0x53544452U; // "RDTS"
inline constexpr std::uint32_t kFlashWordErased = 0xFFFFFFFFU;

// Sits at kStagingMetadataStart. Word order is the commit order in reverse:
// `magic` is programmed when a session opens, `image_size` when it closes, and
// `state` last as the barrier.
struct StagingRecord {
    volatile std::uint32_t magic;
    volatile std::uint32_t state;
    volatile std::uint32_t image_size;
    volatile std::uint32_t reserved; // left erased; keeps the record 16 bytes
};

static_assert(sizeof(StagingRecord) == 16U);

inline const StagingRecord* staging_record() {
    return reinterpret_cast<const StagingRecord*>(kStagingMetadataStart);
}

// True only for a fully committed record. Deliberately strict about `reserved`:
// an unexpected value there means something other than this code wrote the
// sector, and installing on that basis is worse than refusing to.
inline bool staging_record_is_ready(std::uint32_t max_image_size) {
    const auto* record = staging_record();
    return record->magic == kStagingMagic && record->state == kStagingStateReady
        && record->reserved == kFlashWordErased && record->image_size != 0U
        && record->image_size <= max_image_size;
}

#else

inline constexpr bool kStagingSupported = false;

#endif

} // namespace libhcs::firmware::foe
