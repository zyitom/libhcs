#include "firmware/hpm_board/app/src/dmtool/dm_protocol.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace libhcs::firmware::dmtool::protocol {

namespace {

// CRC-16/ARC 的标准校验值: 编码端与 DMTool 对不上时在编译期就暴露。
constexpr std::array<uint8_t, 9> kCrcCheckInput{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
static_assert(crc16(kCrcCheckInput) == 0xBB3D);

constexpr uint32_t read_le32(std::span<const uint8_t, 4> bytes) {
    return uint32_t{bytes[0]} | (uint32_t{bytes[1]} << 8U) | (uint32_t{bytes[2]} << 16U)
         | (uint32_t{bytes[3]} << 24U);
}

constexpr void write_le16(std::span<uint8_t, 2> out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8U);
}

constexpr void write_le32(std::span<uint8_t, 4> out, uint32_t value) {
    for (std::size_t i = 0; i < out.size(); ++i)
        out[i] = static_cast<uint8_t>(value >> (8U * i));
}

constexpr uint32_t kIdMask = 0x1FFF'FFFFU;
constexpr uint32_t kRecordExtendedBit = 1U << 30U;
constexpr uint32_t kRecordRemoteBit = 1U << 31U;

} // namespace

std::span<const uint8_t> build_ack(
    std::span<uint8_t> out, uint8_t command, AckStatus status, std::span<const uint8_t> payload) {
    const std::size_t total = payload.size() + kAckOverhead;
    if (out.size() < total || payload.size() > UINT16_MAX)
        return {};

    out[0] = kFrameHead;
    out[1] = command;
    out[2] = static_cast<uint8_t>(status);
    write_le16(out.subspan<3, 2>(), static_cast<uint16_t>(payload.size()));
    std::ranges::copy(payload, out.begin() + 5);

    // CMD 起至负载末尾: CMD + STATUS + LEN16 + 负载。
    const uint16_t crc = crc16(out.subspan(1, payload.size() + 4));
    write_le16(out.subspan(5 + payload.size()).first<2>(), crc);
    out[total - 1] = kFrameTail;
    return out.first(total);
}

std::optional<CommandParser::Frame> CommandParser::push(uint8_t byte) {
    switch (state_) {
    case State::kHead:
        // 逐字节找帧头: 失步后从下一个 A5 恢复。
        if (byte == kFrameHead)
            state_ = State::kCommand;
        return std::nullopt;

    case State::kCommand:
        command_ = byte;
        crc_ = crc16_update(0, byte);
        state_ = State::kLengthLow;
        return std::nullopt;

    case State::kLengthLow:
        length_ = byte;
        crc_ = crc16_update(crc_, byte);
        state_ = State::kLengthHigh;
        return std::nullopt;

    case State::kLengthHigh:
        length_ = static_cast<uint16_t>(length_ | (uint16_t{byte} << 8U));
        crc_ = crc16_update(crc_, byte);
        received_ = 0;
        state_ = length_ != 0 ? State::kPayload : State::kCrcLow;
        return std::nullopt;

    case State::kPayload:
        if (received_ < payload_.size())
            payload_[received_] = byte;
        crc_ = crc16_update(crc_, byte);
        if (++received_ == length_)
            state_ = State::kCrcLow;
        return std::nullopt;

    case State::kCrcLow:
        crc_received_ = byte;
        state_ = State::kCrcHigh;
        return std::nullopt;

    case State::kCrcHigh:
        crc_received_ = static_cast<uint16_t>(crc_received_ | (uint16_t{byte} << 8U));
        state_ = State::kTail;
        return std::nullopt;

    case State::kTail: {
        const bool valid = byte == kFrameTail && crc_received_ == crc_;
        reset();
        if (!valid) [[unlikely]]
            return std::nullopt;
        const bool complete = length_ <= payload_.size();
        return Frame{
            .command = command_,
            .declared_length = length_,
            .payload_complete = complete,
            .payload = complete ? std::span<const uint8_t>{payload_.data(), length_}
                                : std::span<const uint8_t>{},
        };
    }
    }
    reset();
    return std::nullopt;
}

std::optional<TransmitRequest> parse_transmit_request(std::span<const uint8_t> bytes) {
    if (bytes.size() < kTransmitHeaderSize)
        return std::nullopt;

    const uint8_t dlc = bytes[4] >> 4U;
    const std::size_t length = payload_length(dlc);
    if (bytes.size() < kTransmitHeaderSize + length)
        return std::nullopt;

    const CanFrameFlags flags{
        .extended = (bytes[3] & 0x40U) != 0,
        .remote = (bytes[3] & 0x80U) != 0,
        .fd = (bytes[4] & 0x01U) != 0,
        .bitrate_switch = (bytes[4] & 0x02U) != 0,
    };
    return TransmitRequest{
        .id = read_le32(bytes.first<4>()) & kIdMask,
        .flags = flags,
        .dlc = dlc,
        .channel = bytes[5],
        .repeat_count = read_le32(bytes.subspan<0x12, 4>()),
        .payload = bytes.subspan(kTransmitHeaderSize, length),
        .encoded_size = kTransmitHeaderSize + length,
    };
}

std::size_t encode_record(const CanRecord& record, std::span<uint8_t> out) {
    const std::size_t size = record_size(record.dlc);
    if (out.size() < size)
        return 0;

    uint32_t id_word = record.id & kIdMask;
    if (record.flags.extended)
        id_word |= kRecordExtendedBit;
    if (record.flags.remote)
        id_word |= kRecordRemoteBit;
    write_le32(out.first<4>(), id_word);
    write_le32(out.subspan<4, 4>(), static_cast<uint32_t>(record.timestamp_ns));
    write_le32(out.subspan<8, 4>(), static_cast<uint32_t>(record.timestamp_ns >> 32U));

    uint8_t flags = static_cast<uint8_t>((record.dlc & 0x0FU) << 4U);
    if (record.flags.fd)
        flags |= 0x01U;
    if (record.direction == Direction::kTransmit)
        flags |= 0x02U;
    if (record.flags.bitrate_switch)
        flags |= 0x04U;
    if (record.delivered)
        flags |= 0x08U;
    out[0xC] = 0;
    out[0xD] = flags;
    // [0x0E] 状态字节。DMTool 的显示判定走 can_sent_unpack_thread_func ->
    // insert_item: 发送方向在 [0xE]==0 且 [0xD] 位 3 置位时显示"发送成功",
    // 否则"发送失败"(DMTOOL_PROTOCOL.md 3.2)。handleCANFIFO @0xce5f9 查表
    // 0x18b0f0 的 0x12/0x02 是原始适配器固件侧的状态枚举, 不属于记录流 --
    // 成功回显填 0x12 的版本实测把每一行都显示成"发送失败"。因此:
    //   成功: [0xE]=0, 成功语义由 [0xD] 位 3(delivered)承载;
    //   失败: [0xE]=0x02(发送失败), 位 3 不置位;
    //   接收记录 0x00; 0xFF = 错误帧(进错误队列), 本板不产生。
    // 最早恒写 0 且无位 3 的版本显示"心跳失败", 是状态 0 且位 3 未置位的组合。
    out[0xE] = record.direction == Direction::kTransmit && !record.delivered ? 0x02U : 0x00U;
    out[0xF] = record.channel;

    const auto payload = out.subspan(kRecordHeaderSize, size - kRecordHeaderSize);
    const auto copied = std::min(record.data.size(), payload.size());
    std::ranges::copy(record.data.first(copied), payload.begin());
    std::ranges::fill(payload.subspan(copied), uint8_t{0});
    return size;
}

} // namespace libhcs::firmware::dmtool::protocol
