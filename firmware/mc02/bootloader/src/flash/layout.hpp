#pragma once

#include <cstddef>
#include <cstdint>

#include <main.h>

namespace libhcs::firmware::flash {

// STM32H723 single-bank flash: 8 sectors of 128 KB each.
// Sector 0: Bootloader | Sector 1: Metadata | Sectors 2-7: App
inline constexpr uintptr_t kAppStartAddress  = 0x08040000U;
inline constexpr uintptr_t kAppEndAddress    = 0x08100000U; // exclusive
inline constexpr size_t    kAppMaxImageSize  = kAppEndAddress - kAppStartAddress;

struct SectorRange {
    uint32_t start;
    uint32_t end; // exclusive
    uint32_t sector;
};

inline constexpr size_t kAppSectorCount = 6U;
inline constexpr SectorRange kAppSectors[kAppSectorCount] = {
    {.start = 0x08040000U, .end = 0x08060000U, .sector = FLASH_SECTOR_2},
    {.start = 0x08060000U, .end = 0x08080000U, .sector = FLASH_SECTOR_3},
    {.start = 0x08080000U, .end = 0x080A0000U, .sector = FLASH_SECTOR_4},
    {.start = 0x080A0000U, .end = 0x080C0000U, .sector = FLASH_SECTOR_5},
    {.start = 0x080C0000U, .end = 0x080E0000U, .sector = FLASH_SECTOR_6},
    {.start = 0x080E0000U, .end = 0x08100000U, .sector = FLASH_SECTOR_7},
};

} // namespace libhcs::firmware::flash
