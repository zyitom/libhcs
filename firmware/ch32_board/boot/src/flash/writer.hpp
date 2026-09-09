#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

extern "C" {
#include "ch32h417.h"
}

#include "firmware/ch32_board/boot/src/flash/layout.hpp"
#include "firmware/ch32_board/boot/src/flash/unlock_guard.hpp"
#include "firmware/ch32_board/boot/src/utility/assert.hpp"

namespace libhcs::firmware::flash {

// Sequential writer for the application slot, fed by the DFU download path.
//
// DFU hands over arbitrarily sized chunks at monotonically increasing offsets
// while the flash erases in 4 KB / 8 KB blocks and programs in 32-bit words, so
// this buffers a word at a time and erases each block lazily on first touch.
// Buffering a whole erase block instead would need 8 KB of RAM for no gain: DFU
// never rewinds, so a block is finished before the next one starts.
class Writer {
public:
    void begin_session() {
        write_cursor_ = kAppStartAddress;
        pending_size_ = 0;
        pending_word_ = 0xFFFFFFFFU;
        erased_upto_ = kAppStartAddress;
        block_size_ = erase_block_size();
        utility::assert_always(block_size_ != 0);
    }

    // Append `data` at the current cursor. Returns false on any bounds or
    // hardware failure; the caller turns that into a DFU error status.
    bool write(std::span<const uint8_t> data) {
        // Hard bound, never assert_debug: this is what keeps a malformed image
        // out of the bootloader and the metadata block.
        if (!is_within_app_region(write_cursor_, data.size()))
            return false;

        const UnlockGuard guard;

        for (const uint8_t byte : data) {
            pending_word_ &= ~(0xFFU << (8U * pending_size_));
            pending_word_ |= static_cast<uint32_t>(byte) << (8U * pending_size_);
            if (++pending_size_ < sizeof(uint32_t))
                continue;

            if (!program_pending_word())
                return false;
        }
        return true;
    }

    // Flush a trailing partial word. The unwritten high bytes stay 0xFF, which
    // is the erased value, so a re-flash of the same slot still verifies.
    bool finish() {
        if (pending_size_ == 0)
            return true;

        const UnlockGuard guard;
        return program_pending_word();
    }

    [[nodiscard]] size_t written_size() const { return write_cursor_ - kAppStartAddress; }

private:
    bool program_pending_word() {
        if (!ensure_erased(write_cursor_ + sizeof(uint32_t)))
            return false;

        if (FLASH_ProgramWord(to_physical(write_cursor_), pending_word_) != FLASH_COMPLETE)
            return false;

        // Read back through the alias: a silently failed program is worse than
        // a reported one, and the verify pass would only catch it much later.
        if (*reinterpret_cast<const volatile uint32_t*>(write_cursor_) != pending_word_)
            return false;

        write_cursor_ += sizeof(uint32_t);
        pending_size_ = 0;
        pending_word_ = 0xFFFFFFFFU;
        return true;
    }

    // Erase every block the cursor has reached but not yet cleared. Called with
    // the end of the word about to be programmed so a word straddling a block
    // boundary (it cannot, given 4-byte alignment, but the arithmetic is cheap)
    // still erases ahead of the write.
    bool ensure_erased(uintptr_t required_upto) {
        while (erased_upto_ < required_upto) {
            if (erased_upto_ >= kAppEndAddress)
                return false;
            if (FLASH_ErasePage(to_physical(erased_upto_)) != FLASH_COMPLETE)
                return false;
            erased_upto_ += block_size_;
        }
        return true;
    }

    uintptr_t write_cursor_ = kAppStartAddress;
    uintptr_t erased_upto_ = kAppStartAddress;
    size_t block_size_ = kEraseBlockSizeDualBank;
    uint32_t pending_word_ = 0xFFFFFFFFU;
    uint8_t pending_size_ = 0;
};

} // namespace libhcs::firmware::flash
