#include "firmware/hpm_board/app/src/pcan/pcan_protocol.hpp"

#include <cstring>

// 非 constexpr 的编解码实现; 常量与结构见 pcan_protocol.hpp 的注释。字段偏移
// 依据内核 peak_usb 驱动的 __packed 结构(见该头), 对齐规则: 记录 4 字节对齐,
// CAN_RX 的 size = 28 + align4(负载长)。

namespace libhcs::firmware::pcan::protocol {

namespace {

constexpr std::size_t kAlign = 4;

constexpr std::size_t align_up(std::size_t value) { return (value + kAlign - 1) & ~(kAlign - 1); }

// 定长记录的公共头: size / type / ts_low / ts_high。返回写入字节数(恒 12)。
std::size_t store_record_head(uint8_t* out, uint16_t type, uint64_t timestamp_us) {
    store16(out, static_cast<uint16_t>(kFixedRecordSize));
    store16(out + 2, type);
    store32(out + 4, static_cast<uint32_t>(timestamp_us));
    store32(out + 8, static_cast<uint32_t>(timestamp_us >> 32));
    return 12;
}

} // namespace

std::array<uint8_t, FwInfo::kSize> FwInfo::encode() const {
    std::array<uint8_t, kSize> out{};
    store16(out.data(), static_cast<uint16_t>(kSize));
    store16(out.data() + 2, 2); // type = 2 (EXT): 端点表随记录下发
    out[4] = kHwType;
    out[5] = 0;                 // bl_version[3]: bootloader 1.0.0(内核不读, 报个形状)
    out[6] = 1;
    out[7] = 0;
    out[8] = kHwVersion;
    out[9] = kFwVersion[0];
    out[10] = kFwVersion[1];
    out[11] = kFwVersion[2];
    store32(out.data() + 12, device_ids[0]);
    store32(out.data() + 16, device_ids[1]);
    store32(out.data() + 20, serial_number);
    store32(out.data() + 24, 0); // flags: 无特殊功能
    out[28] = kEpCmdOut;
    out[29] = kEpCmdIn;
    out[30] = kEpDataOut0;
    out[31] = kEpDataOut1;
    out[32] = kEpDataIn;
    out[33] = 0;
    out[34] = 0;
    out[35] = 0;
    return out;
}

std::array<uint8_t, BlInfo::kSize> BlInfo::encode() const {
    std::array<uint8_t, kSize> out{};
    store32(out.data(), 2);                            // ctrl_type(内核 FD 路径不读)
    out[4] = 1;                                        // version[4] = 1.0.0.0
    out[8] = 1;                                        // day
    out[9] = 1;                                        // month
    out[10] = 26;                                      // year(自 2000)
    out[11] = 0;
    store32(out.data() + 12, serial_number >> 16);     // serial_num_hi
    store32(out.data() + 16, serial_number & 0xFFFFU); // serial_num_lo
    store32(out.data() + 20, kHwType);
    store32(out.data() + 24, kHwVersion);              // hw_rev
    return out;
}

TimingSlow decode_timing_slow(std::span<const uint8_t> args) {
    // {ewl, sjw_t, tseg2, tseg1, brp_le16}; 位 7 of sjw_t = 三采样(MCAN 无, 忽略)。
    return {
        .timing =
            {
                     .prescaler = static_cast<uint16_t>(load16(args.data() + 4) + 1U),
                     .seg1 = static_cast<uint16_t>(args[3] + 1U),
                     .seg2 = static_cast<uint8_t>(args[2] + 1U),
                     .sjw = static_cast<uint8_t>((args[1] & 0x7FU) + 1U),
                     },
        .ewl = args[0],
    };
}

TimingFast decode_timing_fast(std::span<const uint8_t> args) {
    // {unused, sjw, tseg2, tseg1, brp_le16}
    return {
        .timing =
            {
                     .prescaler = static_cast<uint16_t>(load16(args.data() + 4) + 1U),
                     .seg1 = static_cast<uint16_t>(args[3] + 1U),
                     .seg2 = static_cast<uint8_t>(args[2] + 1U),
                     .sjw = static_cast<uint8_t>(args[1] + 1U),
                     },
    };
}

FilterMask decode_filter_std(std::span<const uint8_t> args) {
    return {
        .row = load16(args.data()),
        .mask = load32(args.data() + 2),
    };
}

FilterMask decode_set_std_filter(std::span<const uint8_t> args) {
    return {
        .row = args[1],
        .mask = load32(args.data() + 2),
    };
}

ErrCounterWrite decode_err_counter(std::span<const uint8_t> args) {
    return {
        .select = load16(args.data()),
        .tx = args[2],
        .rx = args[3],
    };
}

Options decode_options(std::span<const uint8_t> args) {
    return {
        .ucan = load16(args.data()),
        .usb = load16(args.data() + 4),
    };
}

std::optional<TxFrame> parse_tx_frame(std::span<const uint8_t> bytes) {
    if (bytes.size() < TxFrame::kHeaderSize)
        return std::nullopt;
    const uint16_t size = load16(bytes.data());
    // size == 0 是记录流结尾; 非法长度(未对齐 / 超出本次传输)整段停析。
    if (size < TxFrame::kHeaderSize || size > bytes.size() || (size % kAlign) != 0)
        return std::nullopt;
    if (load16(bytes.data() + 2) != kMsgCanTx)
        return std::nullopt;
    return TxFrame{
        .tag_low = load32(bytes.data() + 4),
        .tag_high = load32(bytes.data() + 8),
        .can_id = load32(bytes.data() + 16),
        .data = bytes.subspan(TxFrame::kHeaderSize, size - TxFrame::kHeaderSize),
        .channel_dlc = bytes[12],
        .client = bytes[13],
        .flags = load16(bytes.data() + 14),
        .encoded_size = size,
    };
}

std::size_t encode_can_rx(const CanRxRecord& record, std::span<uint8_t> out) {
    const std::size_t payload = align_up(record.data.size());
    const std::size_t total = 28 + payload;
    if (out.size() < total)
        return 0;
    uint8_t* p = out.data();
    store16(p, static_cast<uint16_t>(total));
    store16(p + 2, kMsgCanRx);
    store32(p + 4, static_cast<uint32_t>(record.timestamp_us));
    store32(p + 8, static_cast<uint32_t>(record.timestamp_us >> 32));
    store32(p + 12, record.tag_low);
    store32(p + 16, record.tag_high);
    p[20] = record.channel_dlc;
    p[21] = record.client;
    store16(p + 22, record.flags);
    store32(p + 24, record.can_id);
    std::memset(p + 28, 0, payload); // 负载不足对齐界的尾部补 0
    if (!record.data.empty())
        std::memcpy(p + 28, record.data.data(), record.data.size());
    return total;
}

std::size_t encode_calibration(const CalibrationRecord& record, std::span<uint8_t> out) {
    if (out.size() < kFixedRecordSize)
        return 0;
    uint8_t* p = out.data();
    p += store_record_head(p, kMsgCalibration, record.timestamp_us);
    store16(p, record.usb_frame_index);
    store16(p + 2, 0);
    return kFixedRecordSize;
}

std::size_t encode_error(const ErrorRecord& record, std::span<uint8_t> out) {
    if (out.size() < kFixedRecordSize)
        return 0;
    uint8_t* p = out.data();
    p += store_record_head(p, kMsgError, record.timestamp_us);
    p[0] = record.channel & 0x0FU; // channel_type_d: 高 4 位错误类别, 内核不读
    p[1] = 0;                      // code_g: 内核不读
    p[2] = record.tx_err_cnt;
    p[3] = record.rx_err_cnt;
    return kFixedRecordSize;
}

std::size_t encode_status(const StatusRecord& record, std::span<uint8_t> out) {
    if (out.size() < kFixedRecordSize)
        return 0;
    uint8_t* p = out.data();
    p += store_record_head(p, kMsgStatus, record.timestamp_us);
    p[0] = record.channel_bits;
    p[1] = 0;
    p[2] = 0;
    p[3] = 0;
    return kFixedRecordSize;
}

std::size_t encode_overrun(const OverrunRecord& record, std::span<uint8_t> out) {
    if (out.size() < kFixedRecordSize)
        return 0;
    uint8_t* p = out.data();
    p += store_record_head(p, kMsgOverrun, record.timestamp_us);
    p[0] = record.channel & 0x0FU;
    p[1] = 0;
    p[2] = 0;
    p[3] = 0;
    return kFixedRecordSize;
}

} // namespace libhcs::firmware::pcan::protocol
