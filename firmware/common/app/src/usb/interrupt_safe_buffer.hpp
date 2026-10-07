#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cstddef>
#include <span>
#include <utility>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"

// 上行批缓冲: 中断(ISR)侧分配 batch、写入序列化字节, 主循环侧整批弹出交给传输
// 层。原先在 mc02、c_board、hpm 各写一份, 已漂移(batch 对齐、上行满时是否点 LED),
// 2026-10-05 收拢成这一个模板。
//
// ---- 与板级的分工 ----
//
//   - kBatchAlign: batch 数据的对齐, 板级的缓存事实 -- 有缓存的芯片(M7、HPM)取缓存行 32(不与
//     相邻状态共享缓存行, USB 路径上的 cache clean/invalidate 没有伪共享), M4 无
//     缓存, 对齐到 alignof(size_t) 即可; HPM 传 HPM_L1C_CACHELINE_SIZE。
//   - AllocFailed: 上行满(环满且无可弹)时板级要做的报告 -- LED 何时点、要不要记
//     诊断计数, 是各板的UI与遥测决定。无状态, 只有一个静态 report()。
//
// ---- 命名空间就是链接脚本的匹配键 ----
//
// mc02 与 hpm5321 的手维护链接脚本按修饰名前缀把本模板的成员收进 ITCM / ILM
// (`_ZN6libhcs8firmware3usb19InterruptSafeBuffer*`), 并有链接期 ASSERT 盯着
// allocate。改命名空间或类名会让规则悄悄失配, 所以模板必须住在这里; 各板用
// using 别名在自己的命名空间里取名, 别名不改修饰名。
//
// 只在单核上使用: is_locked_ / in_ / out_ 与 written_size_ 的原子操作在这几个
// 工具链上既是优化器约束也是多核屏障, 这里只依赖前者 -- 顺序取自原 mc02 实现,
// 未改。

namespace libhcs::firmware::usb {

// 上行满时的板级报告动作。
template <typename T>
concept AllocFailedHook = requires {
    { T::report() } -> std::same_as<void>;
};

template <size_t kBatchAlign, AllocFailedHook AllocFailed>
class InterruptSafeBuffer
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
                // 满不一定是故障, 所以报不报、怎么报归板级。公共注释留一句判据:
                // 无会话时主循环从不弹出 batch, 环形队列填满一次后就一直保持满,
                // 直到新会话清空 -- 此时若无条件报警, 空闲的健康板会无限闪烁
                // 缓冲满灯型, 因为此后每次 allocate() 都落到这里并重新触发 5 s
                // 窗口。
                AllocFailed::report();
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
        // 对齐由板级给出(kBatchAlign): 有缓存的芯片取缓存行, batch 不与相邻状态
        // 共享缓存行, USB 路径上的 cache clean/invalidate 因此没有伪共享。
        alignas(kBatchAlign) std::byte data_[core::protocol::kProtocolBufferSize]{};
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
