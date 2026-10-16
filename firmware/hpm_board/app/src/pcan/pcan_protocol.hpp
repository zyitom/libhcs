#pragma once

// PEAK PCAN-USB FD / PCAN-USB Pro FD 私有 uCAN 线协议的纯协议层: EP0 记录、命令
// 流、TX 帧与 RX 记录的编解码。不碰 USB 与外设 -- 那些在 pcan_adapter.cpp /
// pcan_control.cpp; 非 constexpr 的编解码实现见 pcan_protocol.cpp(与 dmtool 层
// 同一分法)。
//
// 字段依据: Linux 内核 peak_usb 驱动(drivers/net/can/usb/peak_usb/ 的
// pcan_usb_fd.c / pcan_usb_pro.{c,h} / pcan_usb_core.{c,h}, 及
// include/linux/can/dev/peak_canfd.h)。该驱动由 PEAK 员工编写并随内核发布, 是
// 本协议唯一公开的行为规范; 按仓库的 GPL 纪律只把它当规范读, 实现全部自研。
// 逐字段的核对记录见 firmware/hpm_board/PCAN_PROTOCOL.md。
//
// 线上字节统一 uint8_t 而非 std::byte: 两端都是 TinyUSB 的 uint8_t 缓冲, 协议
// 按小端字节算术, 转来转去只会多一层 cast(与 dm_protocol.hpp 同一取舍)。

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <utility>

namespace libhcs::firmware::pcan::protocol {

// ---- 小端存取(RV32 小端, memcpy 走窄类型别名规则) ----

inline uint16_t load16(const uint8_t* p) {
    uint16_t value = 0;
    std::memcpy(&value, p, sizeof(value));
    return value;
}

inline uint32_t load32(const uint8_t* p) {
    uint32_t value = 0;
    std::memcpy(&value, p, sizeof(value));
    return value;
}

inline void store16(uint8_t* p, uint16_t value) { std::memcpy(p, &value, sizeof(value)); }

inline void store32(uint8_t* p, uint32_t value) { std::memcpy(p, &value, sizeof(value)); }

inline void store64(uint8_t* p, uint64_t value) { std::memcpy(p, &value, sizeof(value)); }

// ---- EP0 vendor 请求(bRequest / wValue) ----

inline constexpr uint8_t kReqInfo = 0;       // REQ_INFO: 读设备信息
inline constexpr uint8_t kReqFct = 2;        // REQ_FCT: 功能设置
inline constexpr uint16_t kInfoBl = 0;       // REQ_INFO wValue: bootloader 信息(blinfo)
inline constexpr uint16_t kInfoFw = 1;       // REQ_INFO wValue: 固件信息(fw_info)
inline constexpr uint16_t kFctDrvld = 5;     // REQ_FCT wValue: 驱动已加载

inline constexpr uint16_t kDrvldLength = 16; // DRVLD 的 OUT 数据段长度(驱动只写前 2 字节)

// Pro 系默认端点(PCAN-USB FD 单通道不用 0x03)。fw_info 的 EXT 端点表与此一致;
// 描述符(pcan_descriptors.hpp)与适配器(pcan_adapter.cpp)从这里取值。
inline constexpr uint8_t kEpCmdOut = 0x01;   // bulk OUT: uCAN 命令流
inline constexpr uint8_t kEpCmdIn = 0x81;    // bulk IN: Windows 设备初始化应答
inline constexpr uint8_t kEpDataOut0 = 0x02; // bulk OUT: CAN0 发送
inline constexpr uint8_t kEpDataOut1 = 0x03; // bulk OUT: CAN1 发送(双通道板)
inline constexpr uint8_t kEpDataIn = 0x82;   // bulk IN: 共享 RX 记录流
inline constexpr uint8_t kEpUnusedIn = 0x83; // Pro FD 保留的第三个 IN 管道, 不传数据

// ---- 本板报告的设备事实 ----

// 主机按 fw_version[0] >= 2 判断"固件支持 ISO/non-ISO 切换"(内核 pcan_usb_fd_init);
// 本板用 MCAN CCCR.NISO 真实实现。版本号对齐真实 Pro FD 的当前固件 3.5.4
// (PEAK 固件信息页, 2026-08-27)。这是兼容身份, 不表示本实现具备真卡全部特性。
// 早期 Code 10 修复同时改了版本、描述符与 EP0, 不能单独归因于版本号。
inline constexpr std::array<uint8_t, 3> kFwVersion{3, 5, 4};
inline constexpr uint8_t kHwVersion = 1;
inline constexpr uint8_t kHwType = 1;

// fw_info(STD 部分 28 字节 + EXT 端点表 8 字节, 共 36)。type=2(EXT) 时内核改用
// 记录里的端点号; 本板端点与 Pro 系默认值一致, 两处同值。
struct FwInfo {
    static constexpr std::size_t kSize = 36;

    std::array<uint32_t, 2> device_ids; // 每通道 32 位 device id(DEVID_SET 只改 RAM)
    uint32_t serial_number = 0;

    [[nodiscard]] std::array<uint8_t, kSize> encode() const;
};

// blinfo(28 字节)。Linux 的 FD 路径不读它, 按 Pro 系形状填齐备 Windows 之问。
struct BlInfo {
    static constexpr std::size_t kSize = 28;
    uint32_t serial_number = 0;

    [[nodiscard]] std::array<uint8_t, kSize> encode() const;
};

// ---- uCAN 命令流(bulk OUT cmd 端点, 8 字节记录) ----

// opcode_channel 的低 10 位。uCAN 命令(0x001..0x010)与 FD 扩展命令(0x080..0x086)
// 同处一个编码空间; 0x3ff(以及 0xff 填充)是流结尾。
enum class Opcode : uint16_t {
    kResetMode = 0x001,
    kNormalMode = 0x002,
    kListenOnlyMode = 0x003,
    kTimingSlow = 0x004,
    kTimingFast = 0x005,
    kSetStdFilter = 0x006,
    kFilterStd = 0x008,
    kTxAbort = 0x009,
    kWrErrCnt = 0x00a,
    kSetEnOption = 0x00b,
    kClrDisOption = 0x00c,
    kRxBarrier = 0x010,
    kEndOfCollection = 0x3ff,
    kClockSet = 0x080,
    kDeviceIdSet = 0x081,
    kLedSet = 0x086,
};

inline constexpr uint16_t kOpcodeChannelBits = 10;
inline constexpr uint16_t kChannelShift = 12;

constexpr uint16_t opcode_of(uint16_t opcode_channel) {
    return opcode_channel & ((1U << kOpcodeChannelBits) - 1U);
}

constexpr uint8_t channel_of(uint16_t opcode_channel) {
    return static_cast<uint8_t>(opcode_channel >> kChannelShift);
}

// 命令流的 8 字节记录视图。opcode_channel 是小端 u16; args 指向头之后的 6 字节
// (指向调用方缓冲, 仅回调期间有效)。
struct Command {
    uint16_t opcode_channel = 0;
    std::span<const uint8_t> args;
};

// 遍历一段传输里的命令记录, 每条调一次 on_command; 遇流结尾(0x3ff / 0xff 填充)
// 停止。命令记录不跨传输(内核一次 bulk_msg 发完整命令表, FS 下按 64 字节分片,
// 每片仍是整条 8 字节记录的序列)。
template <typename OnCommand>
requires std::invocable<OnCommand&, const Command&>
void for_each_command(std::span<const uint8_t> bytes, OnCommand&& on_command) {
    for (std::size_t offset = 0; offset + 8 <= bytes.size(); offset += 8) {
        const uint16_t opcode_channel = load16(bytes.data() + offset);
        if (opcode_of(opcode_channel) >= std::to_underlying(Opcode::kEndOfCollection))
            return; // 0x3ff 或 0xffff 填充: 流结尾
        std::forward<OnCommand>(on_command)(
            Command{.opcode_channel = opcode_channel, .args = bytes.subspan(offset + 2, 6)});
    }
}

// 位时序(主机按 80 MHz 位时钟算好发来, 字段值为"原始值 - 1")。语义与
// Can::PhaseTiming 对齐: seg1 = 传播段 + 相位段 1, 位长 = seg1 + seg2 + 1 个 tq。
struct PhaseTiming {
    uint16_t prescaler = 0; // 10 位字段; MCAN NBRP/DBRP 只有 9 位, 超界由适配器拒收
    uint16_t seg1 = 0;
    uint8_t seg2 = 0;
    uint8_t sjw = 0;
};

// TIMING_SLOW 的 6 字节参数 {ewl, sjw_t, tseg2, tseg1, brp_le16}。sjw_t 位 7 是
// 三采样使能 -- MCAN 无此功能, 忽略(PCAN_PROTOCOL.md 记录); ewl 是错误警告限,
// MCAN 固定 96, 与内核发送的默认值相同, 只存不用。
struct TimingSlow {
    PhaseTiming timing;
    uint8_t ewl = 96;
};

// TIMING_FAST 的 6 字节参数 {unused, sjw, tseg2, tseg1, brp_le16}。
struct TimingFast {
    PhaseTiming timing;
};

TimingSlow decode_timing_slow(std::span<const uint8_t> args);
TimingFast decode_timing_fast(std::span<const uint8_t> args);

// FILTER_STD / SET_STD_FILTER: 64 行 x 32 位的标准 ID 接收位图, 位 = 1 收。
struct FilterMask {
    uint16_t row = 0; // 行号; > 63 表示全部行(内核 set_filter_std(-1) 的形态)
    uint32_t mask = 0;
};

FilterMask decode_filter_std(std::span<const uint8_t> args);
FilterMask decode_set_std_filter(std::span<const uint8_t> args);

// WR_ERR_CNT 的 {sel_mask_le16, tx, rx}。TE=0x4000 / RE=0x8000。
struct ErrCounterWrite {
    uint16_t select = 0;
    uint8_t tx = 0;
    uint8_t rx = 0;
};

ErrCounterWrite decode_err_counter(std::span<const uint8_t> args);

// SET_EN_OPTION / CLR_DIS_OPTION 的 {ucan_mask_le16, unused_le16, usb_mask_le16}。
struct Options {
    uint16_t ucan = 0;
    uint16_t usb = 0;
};

Options decode_options(std::span<const uint8_t> args);

inline constexpr uint16_t kOptionError = 0x0001;          // uCAN: 发送错误计数记录
inline constexpr uint16_t kOptionBusload = 0x0002;        // uCAN: 总线负载记录(不实现)
inline constexpr uint16_t kOptionCanFdIso = 0x0004;       // uCAN: ISO CAN-FD 模式
inline constexpr uint16_t kUsbOptionCalibration = 0x8000; // USB: 周期时间戳记录

// ---- TX 帧(bulk OUT data 端点) ----

// pucan_tx_msg 的 20 字节头(不含负载)。size 含头与负载并按 4 字节对齐; 记录流以
// size == 0 结尾。tag 是主机侧回显标记, Linux 不使用。
struct TxFrame {
    static constexpr std::size_t kHeaderSize = 20;

    uint32_t tag_low = 0;
    uint32_t tag_high = 0;
    uint32_t can_id = 0;
    std::span<const uint8_t> data{}; // payload_length(dlc) 字节, 指向传入缓冲
    uint8_t channel_dlc = 0;         // 低 4 位通道, 高 4 位线上 DLC
    uint8_t client = 0;
    uint16_t flags = 0;
    std::size_t encoded_size = 0; // 本帧(含对齐填充)在传输中占的字节数
};

inline constexpr uint16_t kTxFlagRtr = 0x01;
inline constexpr uint16_t kTxFlagExtId = 0x02;
inline constexpr uint16_t kTxFlagLoopedBack = 0x04;
inline constexpr uint16_t kTxFlagSingleShot = 0x08;
inline constexpr uint16_t kTxFlagExtDataLen = 0x10; // FD 帧
inline constexpr uint16_t kTxFlagBitrateSwitch = 0x20;
inline constexpr uint16_t kTxFlagErrorStateInd = 0x40;
inline constexpr uint16_t kTxFlagSelfReceive = 0x80;

// 解析 bytes 开头的一帧。头不完整或长度字段非法时返回空; size == 0 是流结尾,
// 同样返回空。
std::optional<TxFrame> parse_tx_frame(std::span<const uint8_t> bytes);

// ---- RX 记录(bulk IN data 端点, 4 字节对齐的记录流) ----

inline constexpr uint16_t kMsgCanRx = 0x0001;
inline constexpr uint16_t kMsgError = 0x0002;
inline constexpr uint16_t kMsgStatus = 0x0003;
inline constexpr uint16_t kMsgCalibration = 0x0100;
inline constexpr uint16_t kMsgOverrun = 0x0101;
inline constexpr uint16_t kMsgCanTx = 0x1000;

inline constexpr uint16_t kRxFlagRtr = 0x01;
inline constexpr uint16_t kRxFlagExtId = 0x02;
inline constexpr uint16_t kRxFlagLoopedBack = 0x04;
inline constexpr uint16_t kRxFlagSingleShot = 0x08;
inline constexpr uint16_t kRxFlagExtDataLen = 0x10;
inline constexpr uint16_t kRxFlagBitrateSwitch = 0x20;
inline constexpr uint16_t kRxFlagErrorStateInd = 0x40;
inline constexpr uint16_t kRxFlagSelfReceive = 0x80;

inline constexpr uint8_t kStatusRxBarrier = 0x10;
inline constexpr uint8_t kStatusPassive = 0x20;
inline constexpr uint8_t kStatusWarning = 0x40;
inline constexpr uint8_t kStatusBusOff = 0x80;

// CALIBRATION / ERROR / STATUS / OVERRUN 均为 16 字节定长。
inline constexpr std::size_t kFixedRecordSize = 16;

// CAN_RX 记录(28 字节头 + 负载; 负载 = payload_length(dlc) 字节, 尾部对齐补 0)。
// 头 = size(2) type(2) ts_low(4) ts_high(4) tag_low(4) tag_high(4) channel_dlc(1)
// client(1) flags(2) can_id(4)。时间戳是设备 64 位 us 自由计数器。
struct CanRxRecord {
    uint64_t timestamp_us = 0;
    uint32_t can_id = 0;
    std::span<const uint8_t> data;
    uint8_t channel_dlc = 0;
    uint16_t flags = 0;
    uint32_t tag_low = 0;
    uint32_t tag_high = 0;
    uint8_t client = 0;
};

struct CalibrationRecord {
    uint64_t timestamp_us = 0;
    uint16_t usb_frame_index = 0;
};

struct ErrorRecord {
    uint64_t timestamp_us = 0;
    uint8_t channel = 0;
    uint8_t tx_err_cnt = 0;
    uint8_t rx_err_cnt = 0;
};

struct StatusRecord {
    uint64_t timestamp_us = 0;
    uint8_t channel_bits = 0; // 低 4 位通道, 高位为 kStatus* 状态位
};

struct OverrunRecord {
    uint64_t timestamp_us = 0;
    uint8_t channel = 0;
};

// 编码进 out 的开头, 返回写入字节数; 放不下时返回 0 且不写。
std::size_t encode_can_rx(const CanRxRecord& record, std::span<uint8_t> out);
std::size_t encode_calibration(const CalibrationRecord& record, std::span<uint8_t> out);
std::size_t encode_error(const ErrorRecord& record, std::span<uint8_t> out);
std::size_t encode_status(const StatusRecord& record, std::span<uint8_t> out);
std::size_t encode_overrun(const OverrunRecord& record, std::span<uint8_t> out);

} // namespace libhcs::firmware::pcan::protocol
