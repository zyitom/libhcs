#pragma once

#include <cstdint>
#include <cstring>

#include <main.h>

#include "firmware/mc02/bootloader/src/flash/layout.hpp"
#include "firmware/mc02/bootloader/src/flash/unlock_guard.hpp"

namespace libhcs::firmware::flash {

// STM32H7 flash word = 256 bit = 32 字节, 每个 DataSlot 恰好占一个 flash word。
//
// 所有会改动 flash 的入口在失败时返回 false 而不是 trap: bootloader 是恢复的
// 最后防线, metadata 扇区不可写时设备必须停在 DFU 并上报错误状态, 让主机还能
// 通信, 而不是 HardFault 到失联。
class Metadata {
public:
    static Metadata& get_instance() {
        static Metadata image_metadata;
        return image_metadata;
    }

    bool is_ready() const { return latest_valid_slot_state_ == DataSlotState::kReady; }
    bool is_flashing() const { return latest_valid_slot_state_ == DataSlotState::kFlashing; }

    // 构造时锁存, 先于任何可能改写状态的操作: 只有 flashing 标记而没有其后的
    // ready 记录, 说明上一次下载在两次写入之间掉电。is_flashing() 自身无法回答,
    // 因为 begin_flashing() 会把进行中的会话置为同一状态。
    bool previous_session_interrupted() const { return previous_session_interrupted_; }

    uint32_t image_size() const { return latest_valid_slot_->image_size; }

    bool begin_flashing() {
        if (!latest_valid_slot_ || latest_valid_slot_state_ == DataSlotState::kFatal) {
            if (!erase_and_rescan())
                return false;
        } else if (latest_valid_slot_state_ == DataSlotState::kEmpty) {
            // 直接使用当前空槽 -- 已指向该处
        } else {
            // kFlashing 或 kReady: 前进到下一个槽。越过末尾由下方各分支共享的
            // 容量检查兜底。
            latest_valid_slot_ = reinterpret_cast<DataSlot*>(
                reinterpret_cast<uintptr_t>(latest_valid_slot_) + sizeof(DataSlot));
        }

        // 一个会话占两个槽: 下方写入的 flashing 标记, 加上 finish_flashing() 写在
        // 其后一槽的 ready 记录。只预留一槽会让 begin_flashing() 停在扇区最后一个
        // 槽, ready 记录无处可写, 下载到 manifest 阶段才失败; 只有在标记与 ready
        // 记录之间掉电的会话会把后续会话后移一槽, 恰好走到该状态。检查放在所有
        // 分支之后, 无论槽如何选中都成立。
        if (!has_room_for_session(latest_valid_slot_)) {
            if (!erase_and_rescan())
                return false;
        }

        if (!latest_valid_slot_->enter_flashing_state())
            return false;

        latest_valid_slot_state_ = DataSlotState::kFlashing;
        return true;
    }

    // H7 的 flash word 一旦写入便不可再改, 因此 ready 记录放在 flashing 标记槽
    // 的下一槽。
    bool finish_flashing(uint32_t size) {
        if (latest_valid_slot_state_ != DataSlotState::kFlashing)
            return false;

        // 该槽已由 begin_flashing() 预留, 正常不会触发; 仍保留真实边界检查而非
        // 断言: ready 记录绝不能落到 metadata 扇区末尾之外, 那里已是 App 镜像的
        // 第一个 flash word。
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

    static constexpr uintptr_t kMetadataStartAddress = 0x08020000U; // 扇区 1
    static constexpr uintptr_t kMetadataEndAddress = 0x08040000U;

    static constexpr uint32_t kFlashWordErased = 0xFFFFFFFF;
    static constexpr uint32_t kImageMetadataMagic = 0x48435331; // "HCS1"
    static constexpr uint32_t kImageStateReady = 0x494D5244;    // "IMRD"

    enum class DataSlotState : uint8_t { kFatal, kEmpty, kFlashing, kReady };

    // 恰好 32 字节 -- 一个 STM32H7 flash word。
    struct [[gnu::aligned(32)]] DataSlot {
        uint32_t magic;
        uint32_t image_state;
        uint32_t image_size;
        // 已废弃的 CRC32 字段。镜像完整性由 SHA-256 后缀完全覆盖; 保留该字段是为
        // 了维持 flash 上槽布局与步长不变, 旧版 bootloader 写出的扇区仍能正确读回。
        // 该字段保持擦除态。
        uint32_t reserved;
        uint8_t pad[16];

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
            default: return DataSlotState::kFatal;
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
            buf.magic = magic_val;
            buf.image_state = state_val;
            buf.image_size = size_val;
            buf.reserved = kFlashWordErased;
            std::memset(buf.pad, 0xFF, sizeof(buf.pad));

            const auto guard = UnlockGuard();
            if (!guard.ok())
                return false;

            // bootloader 无缓存运行(见 main.cpp): flash 读取天然一致, 且在缓存
            // 关闭时执行 D-cache 维护指令会触发 fault。
            return HAL_FLASH_Program(
                       FLASH_TYPEPROGRAM_FLASHWORD, reinterpret_cast<uintptr_t>(this),
                       reinterpret_cast<uint32_t>(&buf))
                == HAL_OK;
        }
    };
    static_assert(sizeof(DataSlot) == 32);

    // 一次 flashing 会话需要两个连续槽: flashing 标记及其后的 ready 记录。
    static bool has_room_for_session(const DataSlot* slot) {
        return reinterpret_cast<uintptr_t>(slot) + 2U * sizeof(DataSlot) <= kMetadataEndAddress;
    }

    void scan_latest_valid_slot() {
        latest_valid_slot_ = reinterpret_cast<DataSlot*>(kMetadataStartAddress);
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
                // 第一个空洞: 此后全部未写入
                if (latest_valid_slot_state_ == DataSlotState::kEmpty)
                    latest_valid_slot_ = slot; // 指向第一个空槽
                return;
            }
            latest_valid_slot_ = slot;
            latest_valid_slot_state_ = state;
        }

        // metadata 扇区已完全填满 -- 置为 fatal 以强制擦除
        latest_valid_slot_ = nullptr;
        latest_valid_slot_state_ = DataSlotState::kFatal;
    }

    bool erase_and_rescan() {
        FLASH_EraseInitTypeDef erase{};
        erase.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase.Banks = FLASH_BANK_1;
        erase.Sector = FLASH_SECTOR_1;
        erase.NbSectors = 1;

        uint32_t sector_error = 0U;
        {
            const auto guard = UnlockGuard();
            if (!guard.ok())
                return false;
            if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
                return false;
        }
        // bootloader 无缓存运行(见 main.cpp), 无需 D-cache 维护。

        scan_latest_valid_slot();
        return latest_valid_slot_ != nullptr && latest_valid_slot_state_ == DataSlotState::kEmpty;
    }

    DataSlot* latest_valid_slot_ = nullptr;
    DataSlotState latest_valid_slot_state_ = DataSlotState::kFatal;
    bool previous_session_interrupted_ = false;
};

} // namespace libhcs::firmware::flash
