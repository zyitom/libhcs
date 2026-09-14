#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "firmware/hpm_board/bootloader/include/tusb_config.h"
#include "firmware/hpm_board/bootloader/src/flash/layout.hpp"
#include "firmware/hpm_board/bootloader/src/flash/xpi_nor.hpp"

namespace libhcs::firmware::flash {

// app 槽的按扇区缓冲写入器, 由 DFU 下载路径喂数据。
//
// 擦除/编程失败一律返回给调用方, 由调用方转成主机可见的 DFU 状态(errERASE /
// errPROG)。此路径上没有任何 trap: 磨损或锁死的扇区不能让 bootloader 失联。
//
// 下面的边界检查是真实检查而非断言: 它们是畸形下载与覆写 bootloader 或
// metadata 扇区之间的最后屏障, 绝不能被编译掉。
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

    bool write(uintptr_t address, std::span<const std::byte> data) {
        if (data.empty())
            return false;
        if (address < kAppStartAddress || address >= kAppEndAddress)
            return false;
        if (static_cast<uint64_t>(address) + data.size() > static_cast<uint64_t>(kAppEndAddress))
            return false;

        const uint32_t sector_size = XpiNor::instance().sector_size();
        if (sector_size == 0U || sector_size > kSectorBufferCapacity)
            return false;

        size_t input_offset = 0U;
        while (input_offset < data.size()) {
            const uintptr_t write_address = address + input_offset;
            const uintptr_t sector_address = write_address - (write_address % sector_size);
            if (!activate_sector(sector_address))
                return false;

            const uintptr_t sector_offset = write_address - active_sector_address_;
            const size_t writable = static_cast<size_t>(sector_size - sector_offset);
            const size_t chunk_size = std::min(writable, data.size() - input_offset);

            // DFU 从不回退, 进来的偏移必须与缓冲已有内容衔接。不匹配说明传输
            // 失序, 镜像将被静默写坏。
            if (sector_offset != buffered_size_)
                return false;

            std::memcpy(
                writable_sector_buffer_bytes().data() + sector_offset, data.data() + input_offset,
                chunk_size);
            advance_buffer(sector_offset, chunk_size);

            input_offset += chunk_size;
            if ((sector_offset + chunk_size) == sector_size) {
                if (!commit_active_sector_if_needed())
                    return false;
                clear_active_sector();
            }
        }

        return true;
    }

private:
    static constexpr size_t kSectorBufferCapacity = kFlashSectorSize;
    static constexpr size_t kSectorBufferWordCount = kSectorBufferCapacity / sizeof(uint32_t);
    static constexpr uintptr_t kInvalidAddress = ~static_cast<uintptr_t>(0U);

    static std::span<std::byte> writable_sector_buffer_bytes() {
        return std::as_writable_bytes(std::span<uint32_t>(sector_buffer_));
    }

    static std::span<const std::byte> sector_buffer_bytes() {
        return std::as_bytes(std::span<const uint32_t>(sector_buffer_));
    }

    bool activate_sector(uintptr_t sector_address) {
        if (has_active_sector_ && active_sector_address_ == sector_address)
            return true;

        if (has_active_sector_) {
            if (!commit_active_sector_if_needed())
                return false;
            clear_active_sector();
        }

        if ((sector_address % kSectorBufferCapacity) != 0U)
            return false;

        has_active_sector_ = true;
        active_sector_address_ = sector_address;
        buffered_size_ = 0U;
        return true;
    }

    void advance_buffer(uintptr_t offset, size_t size) {
        buffered_size_ = std::max(buffered_size_, static_cast<size_t>(offset) + size);
    }

    bool has_buffered_data() const { return has_active_sector_ && buffered_size_ > 0U; }

    void clear_active_sector() {
        has_active_sector_ = false;
        active_sector_address_ = kInvalidAddress;
        buffered_size_ = 0U;
    }

    bool commit_active_sector_if_needed() {
        if (!has_buffered_data())
            return true;

        const auto* flash_ptr = reinterpret_cast<const void*>(active_sector_address_);
        const bool is_same =
            std::memcmp(flash_ptr, sector_buffer_bytes().data(), buffered_size_) == 0;
        if (!is_same) {
            if (!XpiNor::instance().erase_sector(active_sector_address_))
                return false;
            if (!program_buffer(active_sector_address_, buffered_size_))
                return false;
        }

        buffered_size_ = 0U;
        return true;
    }

    static bool program_buffer(uintptr_t address, size_t size) {
        if ((address & 0x3U) != 0U)
            return false;

        const size_t full_word_count = size / sizeof(uint32_t);
        if (full_word_count > 0U) {
            if (!XpiNor::instance().program_words(
                    address, std::span<const uint32_t>(sector_buffer_.data(), full_word_count)))
                return false;
        }

        const size_t full_word_bytes = full_word_count * sizeof(uint32_t);
        const size_t tail_size = size - full_word_bytes;
        if (tail_size == 0U)
            return true;

        uint32_t tail_word = 0xFFFFFFFFU;
        std::memcpy(&tail_word, sector_buffer_bytes().data() + full_word_bytes, tail_size);
        return XpiNor::instance().program_word(address + full_word_bytes, tail_word);
    }

    static_assert((kSectorBufferCapacity % sizeof(uint32_t)) == 0U);
    alignas(4) static inline std::array<uint32_t, kSectorBufferWordCount> sector_buffer_{};

    bool has_active_sector_ = false;
    uintptr_t active_sector_address_ = kInvalidAddress;
    size_t buffered_size_ = 0U;
};

} // namespace libhcs::firmware::flash
