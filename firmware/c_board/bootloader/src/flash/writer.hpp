#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <main.h>

#include "firmware/c_board/bootloader/include/tusb_config.h"
#include "firmware/c_board/bootloader/src/flash/layout.hpp"
#include "firmware/c_board/bootloader/src/flash/unlock_guard.hpp"

namespace libhcs::firmware::flash {

// Sector-buffered writer for the application slot, fed by the DFU download path.
//
// Every erase/program failure is returned to the caller, which turns it into a
// DFU status the host can see (errERASE / errPROG). Nothing on this path traps:
// a worn or locked sector must not be able to make the bootloader unreachable.
//
// The bounds checks below are real checks rather than assertions. They are the
// last thing standing between a malformed download and an overwritten
// bootloader or metadata sector, so they must never be compiled out.
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

            // DFU never rewinds, so the incoming offset must line up with what
            // the buffer already holds. A mismatch means the transfer went out
            // of step and the image would be silently corrupted.
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
        erase.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        erase.Sector = kAppSectors[index].sector;
        erase.NbSectors = 1;

        uint32_t sector_error = 0U;
        return HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;
    }

    static bool program_bytes(uint32_t address, std::span<const std::byte> data) {
        if ((address & 0x3U) != 0U)
            return false;

        size_t offset = 0U;

        while (offset + sizeof(uint32_t) <= data.size()) {
            uint32_t word = 0U;
            std::memcpy(&word, data.data() + offset, sizeof(word));
            if (HAL_FLASH_Program(
                    FLASH_TYPEPROGRAM_WORD, address + static_cast<uint32_t>(offset), word)
                != HAL_OK)
                return false;
            offset += sizeof(uint32_t);
        }

        if (offset < data.size()) {
            uint32_t tail_word = 0xFFFFFFFFU;
            std::memcpy(&tail_word, data.data() + offset, data.size() - offset);
            if (HAL_FLASH_Program(
                    FLASH_TYPEPROGRAM_WORD, address + static_cast<uint32_t>(offset), tail_word)
                != HAL_OK)
                return false;
        }

        return true;
    }

    static inline std::array<std::byte, kSectorBufferCapacity> sector_buffer_
        __attribute__((section(".flash_sector_cache"), aligned(4), used));

    bool has_active_sector_ = false;
    size_t active_sector_index_ = 0U;
    uint32_t buffered_size_ = 0U;
};

} // namespace libhcs::firmware::flash
