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

    // 由 DFU 下载路径在候选镜像通过校验后写入, 使 manifest 后的复位启动新应用
    // 而不是重回 DFU。magic 最后写入, 作为提交屏障(见应用侧 mailbox)。
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
