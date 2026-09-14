#pragma once

#include <cstddef>
#include <cstdint>

#include <board.h>

// FoE staging 区域的跨镜像契约。
//
// 谁写什么, 以及为何这样分工:
//
//   * 运行中的 APP 接收固件镜像(BOOT 态经 FoE, 或自检路径经 USB)并写入
//     staging 区域。它永远不能直接写 app 槽: core0 正经 XIP 从该槽执行代码,
//     擦除会把执行擦除的代码本身擦掉。
//
//   * BOOTLOADER 负责安装。冷复位后在这里发现 Ready 记录, 重新校验 staged
//     镜像, 再复制进 app 槽 -- 这是安全的, 因为 bootloader 位于 app 下方自有
//     的 124 KiB 内, 从不在被擦除的区域上执行。
//
// 崩溃安全。记录以最后编程 `state` 的方式提交, 故任何中断都落在"忽略"或
// "重做"两种状态之一:
//
//   写镜像时掉电              -> state 未写  -> 忽略
//   image_size/state 之间掉电 -> state 未写  -> 忽略
//   提交后、安装前掉电        -> Ready       -> 下次启动安装
//   安装过程中掉电            -> 仍为 Ready  -> 下次启动重装
//
// staging 记录只在 app metadata 提交之后才清除, 最后一行靠这一点成立, 不可
// 重排该顺序。
//
// 与 app metadata 扇区不同, 这里是单条固定记录而非 append-only 槽位数组:
// 一次 staging 会话总会重写整个区域, 无可追加, 且每次会话的一次扇区擦除已由
// 随后的镜像擦除付账。

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

// staging 区域可接受的最大镜像。上限由 APP 槽的容量决定, 而非 staging 自己的
// 容量 -- staging 更大, 若接受一个放得进这里却放不进 app 槽的镜像, 只会把拒绝
// 推迟到 app 槽已为它擦除之后。BOARD_APP_FLASH_END_OFFSET 限定 app 槽;
// 0x20000 是其下方 bootloader + metadata 的预留(kAppStartAddress, 见 bootloader
// 的 layout.hpp; 此处不能 include 它, 因为 app 不得依赖 bootloader 头文件)。
inline constexpr std::size_t kStagingImageCapacity = kStagingImageEnd - kStagingImageStart;
inline constexpr std::size_t kAppSlotCapacity =
    (BOARD_FLASH_BASE_ADDRESS + BOARD_APP_FLASH_END_OFFSET) - (BOARD_FLASH_BASE_ADDRESS + 0x20000U);
inline constexpr std::uint32_t kStagingMaxImageSize = static_cast<std::uint32_t>(
    kStagingImageCapacity < kAppSlotCapacity ? kStagingImageCapacity : kAppSlotCapacity);

inline constexpr std::uint32_t kStagingMagic = 0x54534D52U;      // "RMST"
inline constexpr std::uint32_t kStagingStateReady = 0x53544452U; // "RDTS"
inline constexpr std::uint32_t kFlashWordErased = 0xFFFFFFFFU;

// 位于 kStagingMetadataStart。字的顺序是提交顺序的逆序: `magic` 在会话开启
// 时编程, `image_size` 在会话收尾时编程, `state` 最后作为屏障。
struct StagingRecord {
    volatile std::uint32_t magic;
    volatile std::uint32_t state;
    volatile std::uint32_t image_size;
    volatile std::uint32_t reserved; // 保持擦除; 使记录凑足 16 字节
};

static_assert(sizeof(StagingRecord) == 16U);

inline const StagingRecord* staging_record() {
    return reinterpret_cast<const StagingRecord*>(kStagingMetadataStart);
}

// 仅对完整提交的记录为真。对 `reserved` 刻意严格: 该字出现意外值说明写这个
// 扇区的是别的代码, 以此为据安装不如拒绝。
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
