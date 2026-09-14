#pragma once

#include <cstddef>
#include <cstdint>

#include <board.h>

#include "firmware/hpm_board/common/foe_staging.hpp"

namespace libhcs::firmware::flash {

// 板卡可在 app 镜像之上保留 flash(如 EtherCAT 桥用 flash 模拟的 ESC EEPROM,
// 偏移 2 MiB), 做法是用 BOARD_APP_FLASH_END_OFFSET 封顶可接受的镜像区域;
// 默认为整片 flash。
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

// FoE staging 区域。地址与 flash 上的记录格式都放在 common/: 写入方是运行中的
// app, 读取方是 bootloader -- 归属划分与崩溃安全论证见 common/foe_staging.hpp。
#if defined(BOARD_FOE_STAGING_ADDR)

// 容量上限本身定义在 common/foe_staging.hpp: app 在接收时执行它, bootloader 在
// 安装时执行它; 两边不一致会让 app 暂存一个 bootloader 随后拒绝的镜像 --
// 这正是整套设计要避免的唯一失败。
inline constexpr size_t kStagingMaxImageSize = foe::kStagingMaxImageSize;

// foe_staging.hpp 只能凭 board.h 推导 app 槽(app 不得包含 bootloader 的头文件)。
// 两条推导在此汇合, 用断言确认一致, 而非默认它们一致。
static_assert(
    foe::kAppSlotCapacity == kAppMaxImageSize,
    "foe_staging.hpp's app-slot arithmetic drifted from layout.hpp's kAppStartAddress");

static_assert((foe::kStagingMetadataStart % kFlashSectorSize) == 0U);
static_assert((foe::kStagingImageStart % kFlashSectorSize) == 0U);
static_assert((foe::kStagingImageEnd % kFlashSectorSize) == 0U);
static_assert(foe::kStagingMetadataStart >= kAppEndAddress, "staging overlaps the app slot");

#endif

} // namespace libhcs::firmware::flash
