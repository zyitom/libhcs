#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <main.h>

#include "firmware/mc02/bootloader/include/tusb_config.h"
#include "firmware/mc02/bootloader/src/flash/layout.hpp"
#include "firmware/mc02/bootloader/src/flash/unlock_guard.hpp"

namespace libhcs::firmware::flash {

// App 槽的按扇区缓冲写入器, 由 DFU 下载路径喂数据。
//
// 每一次擦除/编程失败都返回给调用方, 由其转为主机可见的 DFU 状态
// (errERASE / errPROG)。这条路径上任何失败都不 trap: 磨损或锁死的扇区不能让
// bootloader 变得不可达。
//
// 下方的边界检查是真实检查而非断言: 它们是畸形下载与覆盖 bootloader/metadata
// 扇区之间的最后一道防线, 绝不能被编译剔除。
class Writer {
public:
    static constexpr uint32_t kTransferBlockSize = CFG_TUD_DFU_XFER_BUFSIZE;

    void begin_session() { clear_active_sector(); }

    void abort_session() { clear_active_sector(); }

    bool finish_session() {
        if (!has_active_sector_)
            return true;

        const bool committed = commit_active_sector_if_needed();
        clear_active_sector();
        return committed;
    }

    bool write(uint32_t address, std::span<const std::byte> data) {
        if (data.empty())
            return false;
        if (address < kAppStartAddress || address >= kAppEndAddress)
            return false;

        const uint64_t end64 = static_cast<uint64_t>(address) + data.size();
        if (end64 > static_cast<uint64_t>(kAppEndAddress))
            return false;

        size_t input_offset = 0U;
        while (input_offset < data.size()) {
            const uint32_t write_address = address + static_cast<uint32_t>(input_offset);
            size_t sector_index = 0U;
            if (!find_sector_index(write_address, sector_index))
                return false;
            if (!activate_sector(sector_index))
                return false;

            const auto& sector = kAppSectors[sector_index];
            const uint32_t sector_size = get_sector_size(sector);
            const uint32_t offset_in_sector = write_address - sector.start;
            const size_t writable = static_cast<size_t>(sector_size - offset_in_sector);
            const size_t chunk_size =
                (data.size() - input_offset < writable) ? (data.size() - input_offset) : writable;

            // DFU 不会回退, 传入偏移必须与缓冲区已有内容对齐; 不一致说明传输
            // 失步, 镜像会被静默写坏。
            if (offset_in_sector != buffered_size_)
                return false;

            std::memcpy(
                sector_buffer_.data() + offset_in_sector, data.data() + input_offset, chunk_size);
            advance_buffer(offset_in_sector, chunk_size);

            input_offset += chunk_size;
            if ((offset_in_sector + static_cast<uint32_t>(chunk_size)) == sector_size) {
                if (!commit_active_sector_if_needed())
                    return false;
                clear_active_sector();
            }
        }

        return true;
    }

private:
    static constexpr uint32_t kSectorBufferCapacity = 128U * 1024U;

    static constexpr uint32_t get_sector_size(const SectorRange& sector) {
        return sector.end - sector.start;
    }

    static bool find_sector_index(uint32_t address, size_t& sector_index) {
        for (size_t i = 0; i < kAppSectorCount; ++i) {
            const auto& sector = kAppSectors[i];
            if (address >= sector.start && address < sector.end) {
                sector_index = i;
                return true;
            }
        }

        return false;
    }

    bool activate_sector(size_t sector_index) {
        if (has_active_sector_ && active_sector_index_ == sector_index)
            return true;

        if (has_active_sector_) {
            if (!commit_active_sector_if_needed())
                return false;
            clear_active_sector();
        }

        if (get_sector_size(kAppSectors[sector_index]) > kSectorBufferCapacity)
            return false;

        has_active_sector_ = true;
        active_sector_index_ = sector_index;
        buffered_size_ = 0U;
        return true;
    }

    void advance_buffer(uint32_t offset, size_t size) {
        const uint32_t dirty_end = offset + static_cast<uint32_t>(size);
        buffered_size_ = std::max(dirty_end, buffered_size_);
    }

    bool has_buffered_data() const { return has_active_sector_ && buffered_size_ > 0U; }

    void clear_active_sector() {
        has_active_sector_ = false;
        active_sector_index_ = 0U;
        buffered_size_ = 0U;
    }

    bool commit_active_sector_if_needed() {
        if (!has_buffered_data())
            return true;

        const auto& sector = kAppSectors[active_sector_index_];
        const size_t dirty_size = static_cast<size_t>(buffered_size_);
        const auto* flash_ptr = reinterpret_cast<const void*>(static_cast<uintptr_t>(sector.start));
        const bool is_same = std::memcmp(flash_ptr, sector_buffer_.data(), dirty_size) == 0;

        if (!is_same) {
            const auto guard = UnlockGuard();
            if (!guard.ok())
                return false;
            if (!erase_sector(active_sector_index_))
                return false;
            if (!program_bytes(
                    sector.start, std::span<const std::byte>(sector_buffer_.data(), dirty_size)))
                return false;
        }

        buffered_size_ = 0U;
        return true;
    }

    static bool erase_sector(size_t index) {
        FLASH_EraseInitTypeDef erase{};
        erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
        erase.Banks        = FLASH_BANK_1;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        erase.Sector       = kAppSectors[index].sector;
        erase.NbSectors    = 1;

        uint32_t sector_error = 0U;
        // bootloader 无缓存运行(见 main.cpp), 此处既不需要也不允许 D-cache
        // 维护 -- 在 D-cache 关闭的 Cortex-M7 上执行缓存维护指令会 fault。
        return HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;
    }

    // STM32H7 flash 编程: 256 bit (32 字节) flashword, 地址必须 32 字节对齐。
    static bool program_bytes(uint32_t address, std::span<const std::byte> data) {
        if ((address & 0x1FU) != 0U)
            return false;

        size_t offset = 0U;
        alignas(32) std::array<std::byte, 32> buf{};

        while (offset < data.size()) {
            const size_t chunk = std::min(data.size() - offset, size_t{32});
            std::memcpy(buf.data(), data.data() + offset, chunk);
            if (chunk < 32)
                std::memset(buf.data() + chunk, 0xFF, 32 - chunk);

            if (HAL_FLASH_Program(
                    FLASH_TYPEPROGRAM_FLASHWORD, address + static_cast<uint32_t>(offset),
                    reinterpret_cast<uint32_t>(buf.data()))
                != HAL_OK)
                return false;

            offset += 32;
        }

        return true;
    }

    static inline std::array<std::byte, kSectorBufferCapacity> sector_buffer_
        __attribute__((section(".flash_sector_cache"), aligned(32), used));

    bool has_active_sector_ = false;
    size_t active_sector_index_ = 0U;
    uint32_t buffered_size_ = 0U;
};

} // namespace libhcs::firmware::flash
