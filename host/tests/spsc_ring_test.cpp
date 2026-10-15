#include <array>
#include <atomic>
#include <cstdint>
#include <latch>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "host/src/channel/spsc_ring.hpp"

namespace {
using libhcs::host::channel::SpscRing;

TEST(SpscRing, ExactCapacityAndEmptyAvailable) {
    SpscRing<int> ring{3};
    EXPECT_EQ(ring.available(), 0U);
    EXPECT_EQ(ring.capacity(), 3U);
    for (int i = 0; i < 3; ++i)
        ASSERT_TRUE(ring.push(i));
    EXPECT_FALSE(ring.push(3));
    std::vector<int> read;
    ring.drain([&](const int& value) noexcept { read.push_back(value); });
    EXPECT_EQ(read, (std::vector<int>{0, 1, 2}));
    EXPECT_EQ(ring.dropped(), 1U);
    EXPECT_EQ(ring.available(), 0U);
    EXPECT_TRUE(ring.push(4));
}

TEST(SpscRing, FullProducerCannotOverwriteActiveConsumer) {
    SpscRing<int> ring{2};
    ASSERT_TRUE(ring.push(1));
    ASSERT_TRUE(ring.push(2));
    std::latch reading{1};
    std::latch attempted{1};
    int observed = 0;
    std::thread consumer{[&] {
        ring.drain([&](const int& value) noexcept {
            if (value == 1) {
                reading.count_down();
                attempted.wait();
                observed = value;
            }
        });
    }};
    reading.wait();
    EXPECT_FALSE(ring.push(3));
    attempted.count_down();
    consumer.join();
    EXPECT_EQ(observed, 1);
    EXPECT_EQ(ring.available(), 0U);
}

TEST(SpscRing, DrainTakesOnlyEntrySnapshot) {
    SpscRing<int> ring{4};
    ASSERT_TRUE(ring.push(1));
    unsigned count = 0;
    ring.drain([&](const int&) noexcept {
        ++count;
        (void)ring.push(2);
    });
    EXPECT_EQ(count, 1U);
    EXPECT_EQ(ring.available(), 1U);
}

TEST(SpscRing, BatchIsAcceptedOrRejectedInFull) {
    SpscRing<int> ring{3};
    const std::array first{1, 2};
    const std::array rejected{3, 4};
    ASSERT_TRUE(ring.push_batch(first));
    EXPECT_FALSE(ring.push_batch(rejected));
    EXPECT_EQ(ring.available(), 2U);
    EXPECT_EQ(ring.dropped(), 2U);
    std::vector<int> read;
    ring.drain([&](const int& value) noexcept { read.push_back(value); });
    EXPECT_EQ(read, (std::vector<int>{1, 2}));
}

TEST(SpscRing, ConcurrentWraparoundPreservesEveryAcceptedRecord) {
    struct Value {
        std::uint64_t sequence;
        std::uint64_t complement;
    };
    SpscRing<Value> ring{31};
    constexpr std::uint64_t count = 100000;
    std::atomic<bool> done{false};
    std::thread producer{[&] {
        for (std::uint64_t i = 0; i < count; ++i)
            while (!ring.push({i, ~i}))
                std::this_thread::yield();
        done.store(true, std::memory_order_release);
    }};
    std::uint64_t expected = 0;
    bool intact = true;
    while (!done.load(std::memory_order_acquire) || ring.available() != 0)
        ring.drain([&](const Value& value) noexcept {
            intact &= value.sequence == expected && value.complement == ~expected;
            ++expected;
        });
    producer.join();
    EXPECT_TRUE(intact);
    EXPECT_EQ(expected, count);
}
} // namespace
