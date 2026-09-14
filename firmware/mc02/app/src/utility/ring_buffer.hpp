#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <limits>
#include <memory>
#include <new>
#include <utility>

namespace libhcs::firmware::utility {

// 无锁单生产者/单消费者(SPSC)环形队列, 思路借鉴 Linux kfifo。
//
// memory order 取舍: 本头文件只被固件的 GCC 交叉工具链(Cortex-M7、RV32)编译,
// 所有队列都在单核上使用(host SDK 另有自己的实现)。在这些平台上 memory order
// 同时是两件事:
//
//   - 对其他核的屏障指令(DMB / FENCE)。单核下它什么都不同步, 只让每处需要它的
//     load/store 多付一条指令。
//   - 对优化器的约束 -- 这才是本队列需要的部分。生产者与消费者可能是中断与被它
//     抢占的代码, 生成的代码必须保持算法依赖的顺序:
//       * 触碰对方索引守护的槽位之前先加载对方索引 -- acquire,
//         其后的访问不得上提到该加载之前;
//       * 发布本侧索引之前完成槽位的写入(生产者)/读取(消费者) -- release,
//         其前的访问不得下沉到该存储之后。
//     各侧索引只有本侧一个写者, 对它的加载用 relaxed 即可。
//     GCC -O3 确实会利用 relaxed 给予的自由度: 在两块板上都实测到它删掉了一处
//     中断可能观察到的写入。
//
// std::atomic_signal_fence() 能在不生成指令的情况下携带存储侧约束, 但标准并未
// 文档化独立 fence 能钉住 relaxed load, 所以两侧都保留逐操作的 memory order,
// 各付一条屏障。
//
// 索引类型是 size_t(本机字宽), 自由增长并按 2^32 回绕 -- 它是任何 2 的幂容量的
// 公倍数, 因此 `in - out` 恒为当前填充量。更窄的索引每个队列只省几个字节, 但
// 减法会提升为 int, 每次运算后都要零扩展; 而且在这些工具链上 uint32_t 是
// `unsigned long` 而 size_t 是 `unsigned int`, 容量达到 65536 及以上时
// std::min() 会编译失败。
template <typename T, size_t max_size>
class RingBuffer {
public:
    using IndexType = size_t;

    static_assert(max_size >= 2, "RingBuffer size must be at least 2");
    static_assert((max_size & (max_size - 1)) == 0, "RingBuffer size must be a power of two");

    static constexpr size_t kMaxSize = max_size;
    static constexpr size_t kMask = max_size - 1;

    constexpr RingBuffer() = default;

    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;
    RingBuffer(RingBuffer&&) = delete;
    RingBuffer& operator=(RingBuffer&&) = delete;

    ~RingBuffer() { clear(); }

    // 消费者侧: 可读元素数。acquire 读 in_, 保证看到生产者已构造完成的元素。
    size_t readable() const {
        const auto in = in_.load(std::memory_order::acquire);
        const auto out = out_.load(std::memory_order::relaxed);

        return in - out;
    }

    // 生产者侧: 可写槽位数。acquire 读 out_, 避免覆写消费者尚未取走的槽位。
    size_t writable() const {
        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);

        return kMaxSize - (in - out);
    }

    // 仅消费者侧调用。返回指针在被弹出或覆写前保持有效。
    T* peek_front() {
        const auto out = out_.load(std::memory_order::relaxed);

        if (out == in_.load(std::memory_order::acquire))
            return nullptr;

        return std::launder(reinterpret_cast<T*>(storage_[out & kMask].data));
    }

    // 仅消费者侧调用。返回指针在被弹出或覆写前保持有效。
    T* peek_back() {
        const auto in = in_.load(std::memory_order::acquire);

        if (in == out_.load(std::memory_order::relaxed))
            return nullptr;

        return std::launder(reinterpret_cast<T*>(storage_[(in - 1) & kMask].data));
    }

    // 生产者侧: 在尾部批量原地构造。F 以 `void(std::byte*)` 签名经 placement-new
    // 构造 T; count 为上限(缺省尽量多构造), fail_fast 为 true 时空间不足则一个都
    // 不构造并返回 0。返回实际构造数; 发布 in_ 用 release。
    template <typename F>
    requires requires(F& f, std::byte* storage) {
        { f(storage) } noexcept;
    }
    size_t emplace_back_n(
        F construct_functor, size_t count = std::numeric_limits<size_t>::max(),
        bool fail_fast = false) {

        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);

        const auto writable = kMaxSize - (in - out);

        if (count > writable)
            count = fail_fast ? 0 : writable;
        if (!count)
            return 0;

        const auto offset = in & kMask;
        const auto slice = std::min(count, kMaxSize - offset);

        for (size_t i = 0; i < slice; i++)
            construct_functor(storage_[offset + i].data);
        for (size_t i = 0; i < count - slice; i++)
            construct_functor(storage_[i].data);

        in_.store(in + count, std::memory_order::release);

        return count;
    }

    template <typename... Args>
    bool emplace_back(Args&&... args) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{std::forward<Args>(args)...})) {
                new (storage) T{std::forward<Args>(args)...};
            },
            1);
    }

    template <typename F>
    requires requires(F& f) {
        { f() } noexcept;
        { T{f()} } noexcept;
    }
    size_t push_back_n(
        F generator, size_t count = std::numeric_limits<size_t>::max(), bool fail_fast = false) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{generator()})) {
                new (storage) T{generator()};
            },
            count, fail_fast);
    }

    bool push_back(const T& value) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{value})) { new (storage) T{value}; }, 1);
    }
    bool push_back(T&& value) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{std::move(value)})) {
                new (storage) T{std::move(value)};
            },
            1);
    }

    // 消费者侧: 从头部批量弹出并销毁, F 以 `void(T)` 签名接收移出的元素;
    // count 缺省为全部可读元素。发布 out_ 用 release。
    template <typename F>
    requires requires(F& f, T& t) {
        { f(std::move(t)) } noexcept;
    } size_t pop_front_n(F callback_functor, size_t count = std::numeric_limits<size_t>::max()) {
        const auto in = in_.load(std::memory_order::acquire);
        const auto out = out_.load(std::memory_order::relaxed);

        const auto readable = in - out;
        count = std::min(count, readable);
        if (!count)
            return 0;

        const auto offset = out & kMask;
        const auto slice = std::min(count, kMaxSize - offset);

        auto process = [&callback_functor](std::byte* storage) {
            auto& element = *std::launder(reinterpret_cast<T*>(storage));
            callback_functor(std::move(element));
            std::destroy_at(&element);
        };
        for (size_t i = 0; i < slice; i++)
            process(storage_[offset + i].data);
        for (size_t i = 0; i < count - slice; i++)
            process(storage_[i].data);

        out_.store(out + count, std::memory_order::release);

        return count;
    }

    template <typename F>
    requires requires(F& f, T& t) {
        { f(std::move(t)) } noexcept;
    } bool pop_front(F&& callback_functor) {
        return pop_front_n(std::forward<F>(callback_functor), 1);
    }

    size_t clear() {
        return pop_front_n([](const T&) noexcept {});
    }

private:
    struct {
        alignas(T) std::byte data[sizeof(T)];
    } storage_[max_size];

    std::atomic<IndexType> in_{0}, out_{0};
    static_assert(std::atomic<IndexType>::is_always_lock_free);
};

} // namespace libhcs::firmware::utility
