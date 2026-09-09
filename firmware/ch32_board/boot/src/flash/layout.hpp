#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {
#include "ch32h417.h"
}

namespace libhcs::firmware::flash {

// CH32H417 flash map, from the reference manual chapter 46 ("闪存组织").
//
// This part carries the 480K-user-area variant: main memory is pages 0..1919 of
// 256 bytes at 0x0800_0000-0x0807_7FFF, which is what openocd reports as
// "flash size = 512kbytes". 0x0000_0000 is the boot alias of the same array --
// code links and executes against the alias, the flash controller programs
// against the 0x0800_0000 physical addresses, so keep both and convert once.
inline constexpr uintptr_t kFlashAliasBase = 0x00000000U;
inline constexpr uintptr_t kFlashPhysicalBase = 0x08000000U;
inline constexpr size_t kUserAreaSize = 480U * 1024U;

// Erase granularity. FLASH_ErasePage() masks the address to 8 KB when the
// dual-bank bit is set and 4 KB otherwise (see ch32h417_flash.c) -- despite the
// name it is NOT a 256-byte page erase. Programming is by 32-bit word.
inline constexpr uint32_t kFlashConfigDualBankBit = 1U << 28;
inline constexpr size_t kEraseBlockSizeSingleBank = 4U * 1024U;
inline constexpr size_t kEraseBlockSizeDualBank = 8U * 1024U;

inline size_t erase_block_size() {
    const auto config = *reinterpret_cast<volatile uint32_t*>(FLASH_CFGR0_BASE);
    return (config & kFlashConfigDualBankBit) ? kEraseBlockSizeDualBank : kEraseBlockSizeSingleBank;
}

// Partitioning, in alias addresses.
//
//   0x00000 .. 0x10000  bootloader: the V3F image (boot core + DFU)
//   0x10000 .. 0x70000  application: the V5F image, the only DFU-writable region
//   0x70000 .. 0x72000  metadata: one erase block holding the image record
//   0x72000 .. 0x78000  spare
//
// The bootloader base matches the V3F link address and the application base
// matches Core_V5F_StartAddr (0x10000), so no address is duplicated between
// here and the linker scripts by accident -- the static_asserts below tie them.
inline constexpr uintptr_t kBootloaderStartAddress = 0x00000000U;
inline constexpr uintptr_t kAppStartAddress = 0x00010000U;
inline constexpr uintptr_t kAppEndAddress = 0x00070000U;      // exclusive
inline constexpr uintptr_t kMetadataStartAddress = 0x00070000U;
inline constexpr uintptr_t kMetadataEndAddress = 0x00072000U; // exclusive

inline constexpr size_t kAppMaxImageSize = kAppEndAddress - kAppStartAddress;

static_assert(kAppStartAddress == 0x00010000U, "must match Core_V5F_StartAddr");
static_assert(kAppStartAddress % kEraseBlockSizeDualBank == 0);
static_assert(kAppEndAddress % kEraseBlockSizeDualBank == 0);
static_assert(kMetadataStartAddress % kEraseBlockSizeDualBank == 0);
static_assert(kMetadataEndAddress <= kUserAreaSize);

inline constexpr uintptr_t to_physical(uintptr_t alias_address) {
    return kFlashPhysicalBase + (alias_address - kFlashAliasBase);
}

// True when [address, address + size) lies wholly inside the application slot.
// Every DFU write is gated on this: it is the one thing standing between a bad
// image and an overwritten bootloader, so it must never be compiled out.
inline constexpr bool is_within_app_region(uintptr_t address, size_t size) {
    if (address < kAppStartAddress || size > kAppMaxImageSize)
        return false;
    return address <= kAppEndAddress - size;
}

} // namespace libhcs::firmware::flash
