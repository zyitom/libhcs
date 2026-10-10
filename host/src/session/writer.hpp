#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <libhcs/transport.hpp>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"

namespace libhcs::host::session {

// 引擎对外的下行流写出口：一块定长缓冲 + 满了就先发出去（transport 侧再按
// USB 包大小切）。Serializer 只认 SerializeBuffer，这里是它的 Transport 适配。
//
// v3 的 StreamBuffer 从传输层按块借缓冲（libusb 的零拷贝路径）；v4 的 Transport
// 接口是"拷进来的字节"（FakeTransport 与将来的 usbfs 都好实现），所以这里换
// 成自有缓冲，多一次 memcpy——发生在 IO 线程上，不在 RT 路径上。
//
// 单线程使用（构造它的那次调用），不抛。
class StreamWriter final : public core::protocol::SerializeBuffer {
public:
    static constexpr std::size_t kCapacity = 4096;

    explicit StreamWriter(Transport& transport) noexcept
        : transport_(transport) {}

    StreamWriter(const StreamWriter&) = delete;
    StreamWriter& operator=(const StreamWriter&) = delete;
    StreamWriter(StreamWriter&&) = delete;
    StreamWriter& operator=(StreamWriter&&) = delete;

    ~StreamWriter() override { flush(); }

    /// 把已写的内容发出去并复位。空缓冲无事可做。
    void flush() noexcept {
        if (used_ == 0)
            return;
        transport_.transmit(std::span<const std::byte>{storage_.data(), used_});
        used_ = 0;
    }

    /// 已写但未 flush 的字节数（断言"这条记录真的进去了"用）。
    [[nodiscard]] std::size_t used() const noexcept { return used_; }

private:
    std::span<std::byte> allocate(std::size_t size) noexcept override {
        core::utility::assert_debug(size != 0);
        if (size > kCapacity) [[unlikely]]
            return {}; // 单条记录就超缓冲：kBadAlloc，调用方按丢帧计数
        if (kCapacity - used_ < size)
            flush();
        const std::span<std::byte> out{storage_.data() + used_, size};
        used_ += size;
        return out;
    }

    Transport& transport_;
    std::array<std::byte, kCapacity> storage_{};
    std::size_t used_ = 0;
};

} // namespace libhcs::host::session
