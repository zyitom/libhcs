#pragma once

#include <cstddef>
#include <cstdint>

#include <main.h>

namespace libhcs::firmware::flash {

// STM32H723 单 bank flash: 8 个扇区, 每扇区 128 KB。
// 扇区 0: Bootloader | 扇区 1: Metadata | 扇区 2-7: App
inline constexpr uintptr_t kAppStartAddress = 0x08040000U;
inline constexpr uintptr_t kAppEndAddress = 0x08100000U; // 不含
inline constexpr size_t kAppMaxImageSize = kAppEndAddress - kAppStartAddress;

struct SectorRange {
    uint32_t start;
    uint32_t end; // 不含
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
