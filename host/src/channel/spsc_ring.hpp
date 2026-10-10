#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>

#include "core/src/utility/assert.hpp"

namespace libhcs::host::channel {

// 拉取式通道的接收环: SPSC, 满了丢最旧并计数。
//
// 生产者是 SDK 的 IO 线程(板子上来的每条记录 push 一次), 消费者是调用
// Board::receive() 的使用者线程。它和 utility::RingBuffer 的差别就一条: 那个
// 满了拒绝写入(调用方自己决定丢什么), 这个满了覆盖最旧的 -- 对接收路径来说
// "新数据比旧数据有价值"是使用者的默认期望, 丢的次数进 dropped 计数, 由状态
// 路径报出去。
//
// 索引用两个自由增长的 uint64: 生产者只写 head_, 消费者只写 tail_。覆盖最旧
// 不需要动对方的索引: 逻辑上的起点是 max(tail_, head_ - N), 消费者按它跳过已
// 被覆盖的槽。内存序: head_ 用 release 写 / acquire 读(槽内容对消费者可见),
// tail_ 用 release 写 / acquire 读(生产者的溢出判断不漏读消费者的进度)。
//
// 使用前提(文档化, 不做运行期防御): 消费者一次 drain 的耗时内生产者推进不满
// 整圈。这是任何按拍 drain 的控制环的自然不变量 -- 一拍收到的记录数远小于环
// 容量, 而 drain N 条的时间比生产者再生产 N 条(至少一个 USB 传输的到达间隔)
// 短几个量级。违反它的用法会把撕读当成数据交出去。
template <typename T>
class DropOldestRing {
    static_assert(std::is_trivially_copyable_v<T>, "the ring copies slots by value");

public:
    explicit DropOldestRing(std::size_t capacity)
        : mask_(size_of(capacity) - 1)
        , storage_(std::make_unique<T[]>(mask_ + 1)) {}

    DropOldestRing(const DropOldestRing&) = delete;
    DropOldestRing& operator=(const DropOldestRing&) = delete;
    DropOldestRing(DropOldestRing&&) = delete;
    DropOldestRing& operator=(DropOldestRing&&) = delete;
    ~DropOldestRing() = default;

    [[nodiscard]] std::size_t capacity() const noexcept { return mask_ + 1; }

    // ---- 生产者(IO 线程) ----
    // 满时覆盖最旧的槽并累计 dropped()。
    void push(const T& value) noexcept {
        const std::uint64_t head = head_.load(std::memory_order_relaxed);
        const std::uint64_t tail = tail_.load(std::memory_order_acquire);
        // 一次 push 至多覆盖一个槽（head - capacity 那个），也至多丢一条。
        if (head - tail >= capacity()) [[unlikely]]
            dropped_.fetch_add(1, std::memory_order_relaxed);
        storage_[head & mask_] = value;
        head_.store(head + 1, std::memory_order_release);
    }

    [[nodiscard]] std::uint64_t dropped() const noexcept {
        return dropped_.load(std::memory_order_relaxed);
    }

    // ---- 消费者(使用者线程) ----
    // 环里现在可读的条数。
    [[nodiscard]] std::size_t available() const noexcept {
        const std::uint64_t head = head_.load(std::memory_order_acquire);
        const std::uint64_t tail = tail_.load(std::memory_order_relaxed);
        return static_cast<std::size_t>(head - std::max(tail, head - capacity()));
    }

    // 取走全部可读记录(被覆盖的自动跳过)。fn 接 const T&; 逐条调用, 不抛。
    template <typename Fn>
    requires std::is_nothrow_invocable_v<Fn&, const T&> void drain(Fn&& fn) noexcept {
        const std::uint64_t head = head_.load(std::memory_order_acquire);
        std::uint64_t tail = tail_.load(std::memory_order_relaxed);
        const std::uint64_t oldest = head > capacity() ? head - capacity() : 0;
        if (tail < oldest)
            tail = oldest;
        for (; tail != head; ++tail)
            fn(storage_[tail & mask_]);
        tail_.store(head, std::memory_order_release);
    }

    // 消费者清空(不逐条回调)。
    void consume_all() noexcept {
        tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
    }

private:
    // 至少 2, 向上取整到 2 的幂(取模换成按位与)。
    static std::size_t size_of(std::size_t requested) noexcept {
        if (requested <= 2)
            return 2;
        return std::bit_ceil(requested);
    }

    std::size_t mask_;
    std::unique_ptr<T[]> storage_;

    std::atomic<std::uint64_t> head_{0};    // 生产者写
    std::atomic<std::uint64_t> tail_{0};    // 消费者写
    std::atomic<std::uint64_t> dropped_{0}; // 生产者写(溢出计数), 状态路径读
};

} // namespace libhcs::host::channel
