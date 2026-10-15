#include <array>
#include <vector>

#include <gtest/gtest.h>

#include "host/src/session/writer.hpp"

namespace {
class RecordingTransport final : public libhcs::Transport {
public:
    bool pooled = true;
    bool exhausted = false;
    bool borrowed = false;
    unsigned copies = 0, acquires = 0, returns = 0;
    std::array<std::byte, 32> storage{};
    std::vector<std::vector<std::byte>> sent;

    void transmit(std::span<const std::byte> bytes) override {
        ++copies;
        sent.emplace_back(bytes.begin(), bytes.end());
    }
    std::size_t transmit_buffer_capacity() const noexcept override {
        return pooled ? storage.size() : 0;
    }
    std::span<std::byte> acquire_transmit_buffer() noexcept override {
        ++acquires;
        if (exhausted)
            return {};
        EXPECT_FALSE(borrowed);
        borrowed = true;
        return storage;
    }
    void submit_transmit_buffer(std::size_t size) noexcept override {
        EXPECT_TRUE(borrowed);
        borrowed = false;
        ++returns;
        if (size != 0)
            sent.emplace_back(storage.begin(), storage.begin() + size);
    }
    void receive(std::function<void(std::span<const std::byte>)>) override {}
    ControlResult
        vendor_control(std::uint8_t, std::uint8_t, std::uint16_t, std::span<std::byte>) override {
        return ControlResult::kUnsupported;
    }
    bool link_faulted() const noexcept override { return false; }
    std::string_view serial() const noexcept override { return {}; }
    int usb_bus_number() const noexcept override { return -1; }
};

TEST(StreamWriter, WritesDirectlyIntoBorrowedBuffersAndReturnsOnFlush) {
    RecordingTransport transport;
    {
        libhcs::host::session::StreamWriter writer{transport};
        libhcs::core::protocol::SerializeBuffer& buffer = writer;
        auto first = buffer.allocate(24);
        ASSERT_EQ(first.data(), transport.storage.data());
        first[0] = std::byte{0x42};
        auto second = buffer.allocate(16); // flush first, then borrow again
        ASSERT_EQ(second.data(), transport.storage.data());
        second[0] = std::byte{0x53};
    }
    EXPECT_FALSE(transport.borrowed);
    EXPECT_EQ(transport.copies, 0U);
    EXPECT_EQ(transport.acquires, 2U);
    EXPECT_EQ(transport.returns, 2U);
    ASSERT_EQ(transport.sent.size(), 2U);
    EXPECT_EQ(transport.sent[0].size(), 24U);
    EXPECT_EQ(transport.sent[0][0], std::byte{0x42});
    EXPECT_EQ(transport.sent[1][0], std::byte{0x53});
}

TEST(StreamWriter, PoolExhaustionDoesNotFallBackToCopyOrLeakBorrow) {
    RecordingTransport transport;
    transport.exhausted = true;
    {
        libhcs::host::session::StreamWriter writer{transport};
        libhcs::core::protocol::SerializeBuffer& buffer = writer;
        EXPECT_TRUE(buffer.allocate(4).empty());
    }
    EXPECT_EQ(transport.copies, 0U);
    EXPECT_EQ(transport.returns, 0U);
}

TEST(StreamWriter, PlainTransportStillWorksWithoutPool) {
    RecordingTransport transport;
    transport.pooled = false;
    {
        libhcs::host::session::StreamWriter writer{transport};
        libhcs::core::protocol::SerializeBuffer& buffer = writer;
        auto bytes = buffer.allocate(4);
        ASSERT_EQ(bytes.size(), 4U);
        bytes[0] = std::byte{0x21};
    }
    EXPECT_EQ(transport.acquires, 0U);
    EXPECT_EQ(transport.copies, 1U);
    ASSERT_EQ(transport.sent.size(), 1U);
    EXPECT_EQ(transport.sent[0][0], std::byte{0x21});
}
} // namespace
