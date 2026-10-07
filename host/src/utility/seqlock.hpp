#pragma once

#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace libhcs::host::utility {

// 单写者、多读者的小值快照(seqlock)。写者从不等待, 读者撞上写到一半的值就重读一遍;
// 两侧都不加锁, 读者在控制环里调用也不会被接收线程挡住。
//
// 值按 64 位字存成原子变量(relaxed), 序号的 release/acquire 加一道栅栏给出顺序 --
// 这样读写两侧都没有标准意义上的数据竞争, 不靠"对齐的读写恰好是原子的"。
template <typename T>
requires std::is_trivially_copyable_v<T> class Seqlock {
public:
    // 只许一个线程调用。
    void store(const T& value) noexcept {
        Words words{};
        std::memcpy(words.data(), &value, sizeof(T));

        const std::uint32_t sequence = sequence_.load(std::memory_order_relaxed);
        sequence_.store(sequence + 1U, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        for (std::size_t i = 0; i < kWords; ++i)
            words_[i].store(words[i], std::memory_order_relaxed);
        sequence_.store(sequence + 2U, std::memory_order_release);
    }

    // 任意线程。
    [[nodiscard]] T load() const noexcept {
        Words words{};
        std::uint32_t before = 0;
        std::uint32_t after = 0;
        do {
            before = sequence_.load(std::memory_order_acquire);
            for (std::size_t i = 0; i < kWords; ++i)
                words[i] = words_[i].load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            after = sequence_.load(std::memory_order_relaxed);
        } while (before != after || (before & 1U) != 0U);

        std::array<std::byte, sizeof(T)> bytes{};
        std::memcpy(bytes.data(), words.data(), sizeof(T));
        return std::bit_cast<T>(bytes);
    }

private:
    static constexpr std::size_t kWords =
        (sizeof(T) + sizeof(std::uint64_t) - 1) / sizeof(std::uint64_t);
    using Words = std::array<std::uint64_t, kWords>;

    std::atomic<std::uint32_t> sequence_{0};
    std::array<std::atomic<std::uint64_t>, kWords> words_{};
};

} // namespace libhcs::host::utility
