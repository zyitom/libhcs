#pragma once

#include <cstddef>
#include <cstdint>

#include <board.h>

#include "firmware/hpm_board/common/foe_staging.hpp"

namespace libhcs::firmware::flash {

// Boards can reserve flash above the app image (e.g. the EtherCAT bridge's
// flash-emulated ESC EEPROM at offset 2 MiB) by capping the accepted image
// region with BOARD_APP_FLASH_END_OFFSET; the default is the whole flash.
#ifndef BOARD_APP_FLASH_END_OFFSET
# define BOARD_APP_FLASH_END_OFFSET BOARD_FLASH_SIZE
#endif

inline constexpr uintptr_t kFlashBaseAddress = BOARD_FLASH_BASE_ADDRESS;
inline constexpr uintptr_t kMetadataStartAddress = 0x8001F000U;
inline constexpr uintptr_t kMetadataEndAddress = 0x80020000U;
inline constexpr uintptr_t kAppStartAddress = 0x80020000U;
inline constexpr uintptr_t kAppEntryAddress = kAppStartAddress + sizeof(uint32_t);
inline constexpr uintptr_t kAppEndAddress = BOARD_FLASH_BASE_ADDRESS + BOARD_APP_FLASH_END_OFFSET;
inline constexpr size_t kAppMaxImageSize = kAppEndAddress - kAppStartAddress;
inline constexpr uint32_t kFlashSectorSize = 4096U;

static_assert(kMetadataEndAddress == kAppStartAddress);
static_assert((kMetadataStartAddress % kFlashSectorSize) == 0U);
static_assert((kAppStartAddress % kFlashSectorSize) == 0U);
static_assert((kAppEndAddress % kFlashSectorSize) == 0U);

// FoE staging region. Addresses and the on-flash record live in common/, because
// the RUNNING APP writes them and the bootloader reads them -- see
// common/foe_staging.hpp for the ownership split and the crash-safety argument.
#if defined(BOARD_FOE_STAGING_ADDR)

// The cap itself lives in common/foe_staging.hpp, because the app enforces it
// while receiving and the bootloader enforces it while installing; a cap that
// differed between the two would let the app stage an image the bootloader then
// refuses, which is the one failure this whole design is meant to avoid.
inline constexpr size_t kStagingMaxImageSize = foe::kStagingMaxImageSize;

// foe_staging.hpp has to spell the app slot out from board.h alone (the app must
// not include bootloader headers). This is where the two derivations meet, so
// assert they agree rather than trusting that they do.
static_assert(
    foe::kAppSlotCapacity == kAppMaxImageSize,
    "foe_staging.hpp's app-slot arithmetic drifted from layout.hpp's kAppStartAddress");

static_assert((foe::kStagingMetadataStart % kFlashSectorSize) == 0U);
static_assert((foe::kStagingImageStart % kFlashSectorSize) == 0U);
static_assert((foe::kStagingImageEnd % kFlashSectorSize) == 0U);
static_assert(foe::kStagingMetadataStart >= kAppEndAddress, "staging overlaps the app slot");

#endif

} // namespace libhcs::firmware::flash
