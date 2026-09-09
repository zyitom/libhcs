#pragma once

#include <cstdint>
#include <type_traits>

namespace libhcs::firmware::utility {

// Cross-reset handshake between the application and the bootloader, held in
// shared SRAM that neither image's .bss initializer touches (it is placed by
// address, not by section, for the same reason boot/src/mailbox.hpp is).
//
// Ported from mc02's BootMailbox; the semantics and magics are identical so the
// host-side flow is the same across boards.
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

    // Written by the application when the host asks it to detach, so the reset
    // that follows lands in DFU instead of relaunching the app. The magic goes
    // last as a commit barrier.
    void request_enter_dfu() {
        request = kMailboxRequestEnterDfu;
        magic = kMailboxMagic;
    }

    // Written by the DFU download path once a candidate image has passed
    // validation, so the reset that follows manifestation boots the new
    // application instead of re-entering DFU.
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

static_assert(std::is_standard_layout_v<BootMailbox> && sizeof(BootMailbox) <= 64);

// Fixed placement in the shared SRAM region, just below the cross-core mailbox
// (boot/src/mailbox.hpp reserves 0x2017_8000). Kept out of both images' .bss so
// its contents survive the reset that carries the request.
inline constexpr uintptr_t kBootMailboxAddress = 0x2017'7F00U;

// Spelled as a reference-returning accessor rather than mc02's `.boot_mailbox`
// section object: this board's linker scripts are vendored, and adding a section
// to them would be one more local patch to re-apply on every EVT re-vendor.
inline BootMailbox& boot_mailbox() { return *reinterpret_cast<BootMailbox*>(kBootMailboxAddress); }

} // namespace libhcs::firmware::utility
