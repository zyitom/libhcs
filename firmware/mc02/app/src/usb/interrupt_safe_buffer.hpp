#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cstddef>
#include <span>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/led/led.hpp"
#include "firmware/mc02/app/src/usb/helper.hpp"

namespace libhcs::firmware::usb {

class InterruptSafeBuffer final
    : public core::protocol::SerializeBuffer
    , private core::utility::Immovable {
public:
    static constexpr size_t kBatchCount = 8;
    static_assert(std::has_single_bit(kBatchCount), "Batch count must be a power of 2");

    static constexpr size_t kMask = kBatchCount - 1;

    constexpr InterruptSafeBuffer() = default;

    std::span<std::byte> allocate(size_t size) noexcept override {
        core::utility::assert_debug(size <= core::protocol::kProtocolBufferSize);
        if (is_locked_.test(std::memory_order::relaxed))
            return {};

        auto out = out_.load(std::memory_order::relaxed);

        while (true) {
            auto in = in_.load(std::memory_order::relaxed);

            auto readable = in - out;
            if (readable) {
                if (auto* result = batches_[(in - 1) & kMask].allocate(size))
                    return {result, size};
            }

            auto writeable = kBatchCount - readable - 1;
            if (!writeable) {
                // 只有主机正在排空时才可能是故障。无会话时 try_transmit() 从不弹出
                // batch, 环形队列填满一次后就一直保持满, 直到 activate_session()
                // 清空 -- 若此时上报, 空闲的健康板会无限闪烁缓冲满灯型, 因为此后
                // 每次 allocate() 都落到这里并重新触发 5 s 窗口。
                if (uplink_session_active())
                    led::led->uplink_buffer_full();
                return {};
            }

            in_.compare_exchange_weak(in, in + 1, std::memory_order::relaxed);
        }
    }

    class Batch {
    public:
        bool empty() const { return written_size_.load(std::memory_order::relaxed) == 0; }

        std::span<const std::byte> data() const {
            return {data_, written_size_.load(std::memory_order::relaxed)};
        }

        std::byte* allocate(size_t size) {
            size_t written_size_local;

            do {
                written_size_local = written_size_.load(std::memory_order::relaxed);
                if (core::protocol::kProtocolBufferSize - written_size_local < size)
                    return nullptr;
            } while (!written_size_.compare_exchange_weak(
                written_size_local, written_size_local + size, std::memory_order::relaxed));

            return data_ + written_size_local;
        }

        void reset() { written_size_.store(0, std::memory_order::relaxed); }

    private:
        std::atomic<size_t> written_size_ = 0;
        // 对齐到 Cortex-M7 D-cache 行(32 B), batch 不与相邻状态共享缓存行
        // -- USB 路径上的 cache clean/invalidate 因此没有伪共享。
        alignas(32) std::byte data_[core::protocol::kProtocolBufferSize]{};
    };

    const Batch* pop_batch() {
        auto in = in_.load(std::memory_order::relaxed);
        auto out = out_.load(std::memory_order::relaxed);

        auto readable = in - out;
        if (!readable)
            return nullptr;
        auto& batch = batches_[out & kMask];
        if (batch.empty())
            return nullptr;

        std::atomic_signal_fence(std::memory_order::release);
        out_.store(out + 1, std::memory_order::relaxed);

        return &batch;
    }

    static void release_batch(const Batch* batch) {
        const_cast<Batch*>(batch)->reset(); // NOLINT(cppcoreguidelines-pro-type-const-cast):
                                            // 为维持封装所做的妥协。
    }

    void clear() {
        const bool was_locked = is_locked_.test_and_set(std::memory_order::relaxed);
        core::utility::assert_debug(!was_locked);

        auto in = in_.load(std::memory_order::relaxed);
        auto out = out_.load(std::memory_order::relaxed);

        auto readable = in - out;
        if (readable) {
            auto offset = out & kMask;
            auto slice = std::min(readable, kBatchCount - offset);

            for (size_t i = 0; i < slice; i++)
                batches_[offset + i].reset();
            for (size_t i = 0; i < readable - slice; i++)
                batches_[i].reset();

            std::atomic_signal_fence(std::memory_order::release);
            out_.store(in, std::memory_order::relaxed);
        }

        is_locked_.clear(std::memory_order::relaxed);
    }

private:
    std::atomic_flag is_locked_;
    std::atomic<size_t> in_{0}, out_{0};
    static_assert(std::atomic<size_t>::is_always_lock_free);
    Batch batches_[kBatchCount];
};

} // namespace libhcs::firmware::usb
