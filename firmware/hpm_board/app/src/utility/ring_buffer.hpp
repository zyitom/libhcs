#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <limits>
#include <memory>
#include <new>
#include <utility>

namespace libhcs::firmware::utility {

// Lock-free Single-Producer/Single-Consumer (SPSC) ring buffer
// Inspired by Linux kfifo.
//
// Memory orders. This header is built only by the firmware's GCC cross toolchains
// (Cortex-M7, RV32), and every queue is used from a single core; the host SDK has its
// own class. On these parts a memory order is two things at once:
//
//   - A barrier instruction for other cores (DMB / FENCE). With one core it
//     synchronizes nothing and costs one instruction per load or store that asks
//     for it.
//   - A constraint on the optimizer, and that is the part this queue needs. The
//     producer and the consumer may be an interrupt and the code it preempted,
//     so the emitted code must keep the order the algorithm relies on:
//       * load the other side's index before touching the slots it guards --
//         acquire, so nothing after the load is hoisted in front of it;
//       * finish the slot writes (producer) or reads (consumer) before
//         publishing this side's index -- release, so nothing before the store
//         is sunk behind it.
//     Each side is the only writer of its own index, so that load is relaxed.
//     GCC -O3 does use the freedom a relaxed order grants: on both boards it
//     deletes a write that an interrupt could observe between two stores.
//
// std::atomic_signal_fence() could carry the store-side constraint without the
// instruction, but a separate fence is not documented to pin a relaxed load in
// place, so both sides keep the per-operation orders and pay the two barriers.
//
// Indices are size_t, the native word. They run free and wrap modulo 2^32, a
// multiple of every power-of-two capacity, so `in - out` is always the fill level.
// A narrower index saves a few bytes per queue but promotes to int on subtraction
// and needs a zero-extension after every one; and a uint32_t index is
// `unsigned long` on these toolchains while size_t is `unsigned int`, which made
// std::min() fail to compile for capacities of 65536 and above.
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

    /*!
     * @brief Destructor
     * Destroys all elements remaining in the buffer.
     */
    ~RingBuffer() { clear(); }

    /*!
     * @brief Number of elements currently readable
     * @return Count of elements available to the consumer
     * @note Uses acquire on producer index and relaxed on consumer index to
     *       ensure visibility of constructed elements to the consumer.
     */
    size_t readable() const {
        const auto in = in_.load(std::memory_order::acquire);
        const auto out = out_.load(std::memory_order::relaxed);

        return in - out;
    }

    /*!
     * @brief Number of free slots for producer
     * @return Count of slots available to write
     * @note Uses relaxed on producer index and acquire on consumer index to
     *       avoid overrun while allowing the producer to run without contention.
     */
    size_t writable() const {
        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);

        return kMaxSize - (in - out);
    }

    /*!
     * @brief Peek the first element (consumer side)
     * @return Pointer to the first element, or nullptr if empty
     * @warning Do not call from producer thread. The pointer remains valid
     *          until the element is popped or overwritten.
     */
    T* peek_front() {
        const auto out = out_.load(std::memory_order::relaxed);

        if (out == in_.load(std::memory_order::acquire))
            return nullptr;

        return std::launder(reinterpret_cast<T*>(storage_[out & kMask].data));
    }

    /*!
     * @brief Peek the last produced element (consumer side)
     * @return Pointer to the last element, or nullptr if empty
     * @warning Do not call from producer thread. The pointer remains valid
     *          until the element is popped or overwritten.
     */
    T* peek_back() {
        const auto in = in_.load(std::memory_order::acquire);

        if (in == out_.load(std::memory_order::relaxed))
            return nullptr;

        return std::launder(reinterpret_cast<T*>(storage_[(in - 1) & kMask].data));
    }

    /*!
     * @brief Batch-construct elements at the tail (producer)
     * @tparam F Functor with signature `void(std::byte* storage)` that constructs
     *         a `T` in-place via placement-new.
     * @param count Maximum number of elements to construct (defaults to as many as fit)
     * @param fail_fast If true, this function constructs nothing and returns 0
     *        unless there is enough space for @p count elements. If false, it
     *        constructs as many elements as currently fit (up to @p count).
     * @return Number of elements actually constructed
     * @note Producer-only. Publishes with release semantics.
     */
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

    /*!
     * @brief Construct one element in-place at the tail (producer)
     * @return true if pushed, false if buffer is full
     */
    template <typename... Args>
    bool emplace_back(Args&&... args) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{std::forward<Args>(args)...})) {
                new (storage) T{std::forward<Args>(args)...};
            },
            1);
    }

    /*!
     * @brief Batch-push using a generator (producer)
     * @tparam F Functor returning a `T` to be stored
     * @param count Maximum number to generate/push
     * @param fail_fast If true, this function pushes nothing and returns 0
     *        unless there is enough space for @p count elements. If false, it
     *        pushes as many elements as currently fit (up to @p count).
     * @return Number of elements actually pushed
     */
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

    /*!
     * @brief Push a copy of value (producer)
     * @return true if pushed, false if buffer is full
     */
    bool push_back(const T& value) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{value})) { new (storage) T{value}; }, 1);
    }
    /*!
     * @brief Push by moving value (producer)
     * @return true if pushed, false if buffer is full
     */
    bool push_back(T&& value) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{std::move(value)})) {
                new (storage) T{std::move(value)};
            },
            1);
    }

    /*!
     * @brief Batch-pop elements from the head (consumer)
     * @tparam F Functor with signature `void(T)` receiving moved-out elements
     * @param count Maximum number of elements to pop (defaults to all available)
     * @return Number of elements actually popped
     * @note Consumer-only. Consumes with release on `out_` and destroys elements.
     */
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

    /*!
     * @brief Pop one element (consumer)
     * @return true if an element was popped, false if empty
     */
    template <typename F>
    requires requires(F& f, T& t) {
        { f(std::move(t)) } noexcept;
    } bool pop_front(F&& callback_functor) {
        return pop_front_n(std::forward<F>(callback_functor), 1);
    }

    /*!
     * @brief Clear the buffer by consuming all elements
     * @return Number of elements that were erased
     */
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
