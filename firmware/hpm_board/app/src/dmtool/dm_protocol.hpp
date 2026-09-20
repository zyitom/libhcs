#pragma once

// 达妙 USB2FDCAN 适配器私有协议的纯协议层: 帧格式、校验、编解码。不碰 USB 与
// 外设 -- 那些在 dm_adapter.cpp。字段依据(DMTool 2.1.6.7 反汇编, 函数地址)与
// 端点语义见 firmware/hpm_board/DMTOOL_PROTOCOL.md。
//
// 线上字节统一用 uint8_t 而非 std::byte: 两端都是 TinyUSB 的 uint8_t 缓冲,
// 协议本身按字节算术(CRC、位域), 转来转去只会多出一层 cast。

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "core/include/libhcs/protocol/can_dlc.hpp"

namespace libhcs::firmware::dmtool::protocol {

// ---- 校验与 DLC ----

// CRC-16/ARC: 反射多项式 0xA001, 初值 0, 无终值异或(DMTool 的
// fdcan_protocol::CRC16_IBM)。注意不是 Modbus -- 后者初值 0xFFFF。
constexpr uint16_t crc16_update(uint16_t crc, uint8_t byte) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit)
        crc = (crc & 1U) != 0U ? static_cast<uint16_t>((crc >> 1U) ^ 0xA001U)
                               : static_cast<uint16_t>(crc >> 1U);
    return crc;
}

constexpr uint16_t crc16(std::span<const uint8_t> bytes, uint16_t crc = 0) {
    for (const uint8_t byte : bytes)
        crc = crc16_update(crc, byte);
    return crc;
}

// 线上 4 位 DLC 到负载字节数(DMTool 的 GetLenFromDlc), 以及反向映射。表收敛
// 到 core 一处 (core/include/libhcs/protocol/can_dlc.hpp): DMTool 桥与 libhcs
// 记录流必须用同一张表, 否则两条记录流会对同一帧定界出不同长度。
using core::protocol::dlc_from_payload_len;
using core::protocol::payload_length;

// ---- 命令通道(EP 0x02 OUT / 0x82 IN) ----
//
// 命令: A5 | CMD | LEN(LE16) | PAYLOAD | CRC16(LE) | 5A
// 应答: A5 | CMD | STATUS | LEN(LE16) | PAYLOAD | CRC16(LE) | 5A
// CRC 覆盖 CMD 起至负载末尾(应答含 STATUS 与 LEN)。主机要求 5A 恰为一次 IN
// 传输的最后一个字节, 故一条应答独占一次传输。

inline constexpr uint8_t kFrameHead = 0xA5;
inline constexpr uint8_t kFrameTail = 0x5A;

// usb_contorl_transfer 的命令字, 由各 USB_CMD_* 包装函数的立即数还原。
enum class Command : uint8_t {
    kStartCapture = 0x00,     // 负载 1 字节通道号
    kStopCapture = 0x01,      // 负载 1 字节通道号
    kIapPacket = 0x02,        // 固件升级数据包(4100 字节负载)
    kIapEnd = 0x03,           // 固件升级结束
    kStartTest = 0x04,        // 设备端自测
    kSetupBaudrate = 0x05,    // 负载 10 字节, 见 BaudrateConfig
    kReadVersion = 0x06,      // 应答负载: NUL 结尾 ASCII 版本串
    kGetUuid = 0x09,          // 应答负载: 16 字节
    kWriteSerial = 0x0A,      // 写 SN
    kGetSerial = 0x0B,        // 读 SN
    kJumpOutLoader = 0x0C,    // 退出 bootloader
    kGetBaudrate = 0x0D,      // 负载 1 字节通道号, 应答 10 字节同 kSetupBaudrate
    kRecoveryFactory = 0x0E,  // 恢复出厂
    kStopPeriodicSend = 0x0F, // 负载 1 字节通道号
    kSaveParameters = 0x10,   // 保存参数(心跳帧同码, 但走 EP 0x01, 互不相干)
};

enum class AckStatus : uint8_t {
    kOk = 0,
    kFailed = 1, // 非 0 即失败, DMTool 只区分 0 与非 0
};

inline constexpr std::size_t kAckOverhead = 8; // A5 CMD STATUS LEN16 ... CRC16 5A

// 写入 out 并返回已用前缀; out 放不下时返回空 span。
std::span<const uint8_t> build_ack(
    std::span<uint8_t> out, uint8_t command, AckStatus status, std::span<const uint8_t> payload);

// 命令帧流式解析器。bulk OUT 按包交付, 帧可能跨包(固件包 4 KB 以上), 所以逐字节
// 喂; 但一帧绝不跨传输 -- DMTool 一条命令一次 libusb_bulk_transfer -- 于是传输
// 结束(短包)时丢弃半帧, 一次失步最多污染一条命令。
class CommandParser {
public:
    // 负载上限。实际要处理的命令负载最长 15 字节(kRecoveryFactory); 超长帧
    // (固件包)仍按长度消费并校验, 只是不留存负载, 回调里 payload 为空、
    // payload_complete 为 false。
    static constexpr std::size_t kMaxStoredPayload = 64;

    struct Frame {
        uint8_t command;
        uint16_t declared_length;
        bool payload_complete;
        std::span<const uint8_t> payload; // 仅在回调期间有效
    };

    // 喂一段传输数据, 每解出一条 CRC 与帧尾都正确的命令就调用一次 on_frame。
    template <typename OnFrame>
    requires std::invocable<OnFrame&, const Frame&>
    void feed(std::span<const uint8_t> bytes, bool end_of_transfer, OnFrame&& on_frame) {
        for (const uint8_t byte : bytes) {
            if (const auto frame = push(byte))
                on_frame(*frame);
        }
        if (end_of_transfer)
            reset();
    }

    void reset() {
        state_ = State::kHead;
        received_ = 0;
    }

private:
    enum class State : uint8_t {
        kHead,
        kCommand,
        kLengthLow,
        kLengthHigh,
        kPayload,
        kCrcLow,
        kCrcHigh,
        kTail
    };

    std::optional<Frame> push(uint8_t byte);

    State state_ = State::kHead;
    uint8_t command_ = 0;
    uint16_t length_ = 0;
    uint16_t received_ = 0;
    uint16_t crc_ = 0;
    uint16_t crc_received_ = 0;
    std::array<uint8_t, kMaxStoredPayload> payload_{};
};

// kSetupBaudrate 负载 / kGetBaudrate 应答负载, 10 字节(USB_CMD_SETUP_BUARD
// 的参数装配顺序):
//   [0] 通道 [1] 帧类型(0 = CAN, 非 0 = CANFD)
//   [2] 仲裁 seg1 [3] 仲裁 seg2 [4] 仲裁 sjw [5] 仲裁分频
//   [6] 数据 seg1 [7] 数据 seg2 [8] 数据 sjw [9] 数据分频
// seg1 = 传播段 + 相位段 1, 位长 = seg1 + seg2 + 1 个 tq; DMTool 按 80 MHz
// 位时钟换算: 速率 = 80 MHz / 分频 / (seg1 + seg2 + 1)。
struct PhaseTiming {
    uint8_t seg1 = 0;
    uint8_t seg2 = 0;
    uint8_t sjw = 0;
    uint8_t prescaler = 0;

    // 0 表示字段不构成合法位时序。
    [[nodiscard]] constexpr uint32_t bitrate(uint32_t clock_hz) const {
        const uint32_t quanta = uint32_t{seg1} + seg2 + 1U;
        if (prescaler == 0 || seg1 == 0 || seg2 == 0)
            return 0;
        return clock_hz / prescaler / quanta;
    }
};

struct BaudrateConfig {
    static constexpr std::size_t kSize = 10;
    static constexpr uint32_t kClockHz = 80'000'000;

    uint8_t channel = 0;
    bool fd = false;
    PhaseTiming nominal;
    PhaseTiming data;

    static constexpr std::optional<BaudrateConfig> decode(std::span<const uint8_t> bytes) {
        if (bytes.size() != kSize)
            return std::nullopt;
        return BaudrateConfig{
            .channel = bytes[0],
            .fd = bytes[1] != 0,
            .nominal = {.seg1 = bytes[2], .seg2 = bytes[3], .sjw = bytes[4], .prescaler = bytes[5]},
            .data = {.seg1 = bytes[6], .seg2 = bytes[7], .sjw = bytes[8], .prescaler = bytes[9]},
        };
    }

    [[nodiscard]] constexpr std::array<uint8_t, kSize> encode() const {
        return {channel,      static_cast<uint8_t>(fd ? 1U : 0U),
                nominal.seg1, nominal.seg2,
                nominal.sjw,  nominal.prescaler,
                data.seg1,    data.seg2,
                data.sjw,     data.prescaler};
    }
};

// ---- CAN 数据面 ----

struct CanFrameFlags {
    bool extended       : 1 = false;
    bool remote         : 1 = false;
    bool fd             : 1 = false;
    bool bitrate_switch : 1 = false;
};

// EP 0x03 OUT: 22 字节头 + payload_length(dlc) 字节负载, 一次传输一帧
// (CustomCDC::fillFDCANFrame 装配, FdcanDeviceSend 发送):
//   [0..3]   CAN ID(LE, 低 29 位); [3] 位 6 = 扩展帧, 位 7 = 远程帧
//   [4]      高 4 位 DLC; 位 0 = FD, 位 1 = BRS, 位 2 = ID 自增, 位 3 = 数据自增
//   [5]      通道
//   [6..0xD] 主机侧状态, 设备不用
//   [0xE..0x11] 重复发送间隔(LE32)  [0x12..0x15] 发送次数(LE32)
//   [0x16..] 负载
inline constexpr std::size_t kTransmitHeaderSize = 22;

struct TransmitRequest {
    uint32_t id;
    CanFrameFlags flags;
    uint8_t dlc;
    uint8_t channel;
    uint32_t repeat_count;
    std::span<const uint8_t> payload; // payload_length(dlc) 字节, 指向传入缓冲
    std::size_t encoded_size;         // 本帧在传输中占的字节数
};

// 解析 bytes 开头的一帧。头不完整或负载不足时返回空。
std::optional<TransmitRequest> parse_transmit_request(std::span<const uint8_t> bytes);

// EP 0x81(接收)/ 0x83(发送回显)IN: 记录流, 16 字节头 + payload_length(dlc)
// 字节负载, 一次传输可连续放多条(can_rec_unpack_thread_func 逐条推进):
//   [0..3]  CAN ID(LE, 低 29 位); 位 30 = 扩展帧, 位 31 = 远程帧
//   [4..0xB] 64 位纳秒时间戳(LE, DMTool 除以 1e9 显示为秒)
//   [0xC]   0
//   [0xD]   高 4 位 DLC; 位 0 = FD, 位 1 = 方向(0 接收 / 1 发送),
//           位 2 = BRS, 位 3 = 发送成功(仅发送方向有意义)
//   [0xE]   0 = 正常, 0xFF = 错误帧, 其余 = 失败
//   [0xF]   通道(DMTool 不读, 按真适配器语义填)
inline constexpr std::size_t kRecordHeaderSize = 16;

enum class Direction : uint8_t { kReceive, kTransmit };

struct CanRecord {
    uint32_t id = 0;
    CanFrameFlags flags;
    Direction direction = Direction::kReceive;
    bool delivered = false; // 仅发送方向: 帧已交给 CAN 控制器
    uint8_t dlc = 0;
    uint8_t channel = 0;
    uint64_t timestamp_ns = 0;
    std::span<const uint8_t> data; // 不足 payload_length(dlc) 的部分补 0
};

[[nodiscard]] constexpr std::size_t record_size(uint8_t dlc) {
    return kRecordHeaderSize + payload_length(dlc);
}

// 写入 out, 返回写入字节数; 放不下时返回 0 且不写。
std::size_t encode_record(const CanRecord& record, std::span<uint8_t> out);

} // namespace libhcs::firmware::dmtool::protocol
