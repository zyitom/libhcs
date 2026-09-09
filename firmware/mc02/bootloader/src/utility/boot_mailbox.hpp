#pragma once

#include <cstdint>
#include <type_traits>

namespace libhcs::firmware::utility {

struct BootMailbox {
    static constexpr uint32_t kMailboxMagic = 0x48435331;              // "HCS1"
    static constexpr uint32_t kMailboxRequestEnterDfu = 0x44465530;    // "DFU0"
    static constexpr uint32_t kMailboxRequestBootAppOnce = 0x41505031; // "APP1"

    volatile uint32_t magic;
    volatile uint32_t request;

    void clear() {
        magic = 0;
        request = 0;
    }

    // Written by the DFU download path once a candidate image has passed
    // validation, so the reset that follows manifestation boots the new
    // application instead of re-entering DFU. The magic is stored last as a
    // commit barrier (see the app-side mailbox).
    void request_boot_app_once() {
        request = kMailboxRequestBootAppOnce;
        magic = kMailboxMagic;
    }

    uint32_t consume_request() {
        const uint32_t consumed_request = (magic == kMailboxMagic) ? request : 0U;
        clear();
        return consumed_request;
    }
};

inline BootMailbox boot_mailbox __attribute__((section(".boot_mailbox"), aligned(4), used));

static_assert(std::is_standard_layout_v<BootMailbox> && sizeof(BootMailbox) <= 64);

} // namespace libhcs::firmware::utility
