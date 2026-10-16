#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "core/include/libhcs/protocol/can_dlc.hpp"
#include "firmware/hpm_board/app/src/pcan/pcan_protocol.hpp"

namespace libhcs::firmware::pcan {

// 只由主循环访问。CAN ISR 只回传硬件完成的 marker, 不读取这里的 USB 元数据。
// 槽位直到确认记录已装进 USB IN 缓冲后才释放, 避免覆盖尚未确认的 tag/client。
template <std::size_t Capacity = 128>
class TxEchoes {
    static_assert(Capacity > 0 && Capacity <= 256);

    struct Entry {
        bool used = false;
        uint32_t can_id = 0;
        uint32_t tag_low = 0;
        uint32_t tag_high = 0;
        uint8_t client = 0;
        uint8_t channel_dlc = 0;
        uint16_t flags = 0;
        std::array<uint8_t, 64> data{};
    };

public:
    static constexpr std::size_t kCapacity = Capacity;

    static bool requested(const protocol::TxFrame& frame) {
        return (frame.flags & (protocol::kTxFlagSelfReceive | protocol::kTxFlagLoopedBack)) != 0;
    }

    std::optional<uint8_t> reserve(uint8_t channel, const protocol::TxFrame& frame) {
        for (std::size_t count = 0; count < Capacity; ++count) {
            const std::size_t index = next_;
            next_ = (next_ + 1U) % Capacity;
            auto& entry = entries_[index];
            if (entry.used)
                continue;
            entry = {};
            entry.used = true;
            entry.can_id = frame.can_id;
            entry.tag_low = frame.tag_low;
            entry.tag_high = frame.tag_high;
            entry.client = frame.client;
            entry.channel_dlc = static_cast<uint8_t>((frame.channel_dlc & 0xF0U) | channel);
            entry.flags = (frame.flags
                           & (protocol::kTxFlagExtId | protocol::kTxFlagRtr
                              | protocol::kTxFlagSingleShot | protocol::kTxFlagSelfReceive))
                        | protocol::kRxFlagLoopedBack;
            std::copy_n(
                frame.data.begin(), std::min(frame.data.size(), entry.data.size()),
                entry.data.begin());
            return static_cast<uint8_t>(index);
        }
        return std::nullopt;
    }

    std::optional<protocol::CanRxRecord>
        completed(uint8_t marker, uint64_t timestamp_us, uint16_t wire_flags) const {
        if (marker >= Capacity || !entries_[marker].used)
            return std::nullopt;
        const auto& entry = entries_[marker];
        const bool remote = (entry.flags & protocol::kRxFlagRtr) != 0;
        const bool fd = (wire_flags & protocol::kRxFlagExtDataLen) != 0;
        const std::size_t length = remote ? 0
                                 : fd     ? core::protocol::payload_length(entry.channel_dlc >> 4)
                                          : std::min<unsigned>(entry.channel_dlc >> 4, 8);
        return protocol::CanRxRecord{
            .timestamp_us = timestamp_us,
            .can_id = entry.can_id,
            .data = std::span{entry.data}.first(length),
            .channel_dlc = entry.channel_dlc,
            .flags = static_cast<uint16_t>(entry.flags | wire_flags),
            .tag_low = entry.tag_low,
            .tag_high = entry.tag_high,
            .client = entry.client,
        };
    }

    void release(uint8_t marker) {
        if (marker < Capacity)
            entries_[marker].used = false;
    }

    void clear() {
        for (auto& entry : entries_)
            entry.used = false;
        next_ = 0;
    }

private:
    std::array<Entry, Capacity> entries_{};
    std::size_t next_ = 0;
};

} // namespace libhcs::firmware::pcan
