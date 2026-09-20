#pragma once

#include <cstdint>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/utility/bitfield.hpp"

namespace libhcs::core::protocol {

using FieldId = data::DataId;

namespace layouts {

using utility::BitfieldMember;

struct FieldHeaderLayout {
    using Id = utility::BitfieldMember<0, 4, FieldId>;
};

struct FieldHeaderExtendedLayout {
    using IdExtended = utility::BitfieldMember<4, 8, FieldId>;
};

struct CanHeaderLayout {
    // Bit 3 was previously HasTimestamp but conflicts with FieldHeader.Id bit 3.
    // HasTimestamp has been moved to CanHeaderStandardLayout / CanHeaderExtendedLayout.
    // Bit 3 is reserved.
    //
    // Bit 4 is IsLongFrame (since 2026-09-20): the payload is one of the CAN-FD
    // long lengths (12-64 bytes) and the DataLengthCode field below holds
    // wireDLC - 9 instead of bytes-1. It reuses the slot of IsFdCan, retired
    // 2026-09-12: classic-vs-FD stays a property of the BUS, fixed by the
    // firmware's port table and read over the EP0 configuration channel
    // (kGetInterface.can_fd_mask / kGetCanConfig -- see
    // libhcs/protocol/vendor_control.hpp). IsLongFrame says nothing about the
    // frame type -- short frames encode identically on classic and FD buses --
    // it only switches how the 3-bit length field decodes, which the per-bus
    // model has no room to express. Peers agree on the encoding through the
    // EP0 wire-layout fingerprint plus kCapCanFdLongFrames; a peer that never
    // negotiated it neither sends nor receives long frames, so the retired
    // bit's old rule (a receiver may ignore bit 4) is gone -- ignoring it now
    // misreads the length field. Invalid encoding, reserved on both
    // directions: IsLongFrame with IsRemoteTransmission (ISO CAN-FD has no
    // remote frames), and a long-form DataLengthCode of 7.
    using IsLongFrame = BitfieldMember<4, 1>;
    using IsExtendedCanId = BitfieldMember<5, 1>;
    using IsRemoteTransmission = BitfieldMember<6, 1>;
    using HasCanData = BitfieldMember<7, 1>;
};

// DataLengthCode semantics, both layouts, keyed on IsLongFrame:
//   short (IsLongFrame = 0): payload bytes - 1, 0-7 == 1-8 bytes;
//   long  (IsLongFrame = 1): wire DLC - 9, 0-6 == DLC 9-15 == 12-64 bytes via
//          the table in libhcs/protocol/can_dlc.hpp; 7 is reserved.
// Short frames on classic and FD buses encode identically, so a receiver that
// does not track the bus mode decodes them correctly either way; per-frame
// FDF/BRS fidelity is deliberately not carried here (the DMTool record
// stream, firmware/hpm_board/DMTOOL_PROTOCOL.md, reports it for analysis).
struct CanHeaderStandardLayout {
    using CanId = BitfieldMember<8, 11>;
    // HasTimestamp sits in the 2-bit gap [19:21) between CanId and
    // DataLengthCode -- avoiding overlap with DataLengthCode bit 1.
    using HasTimestamp = BitfieldMember<19, 1>;
    // Bit 20 is the last free bit of the standard header. Reserved -- spent
    // now so a future per-frame flag (BRS, ESI) does not reshuffle the layout
    // again; a receiver must ignore it.
    using DataLengthCode = BitfieldMember<21, 3>;
};

struct CanHeaderExtendedLayout {
    using CanId = BitfieldMember<8, 29>;
    // Long frames use the same DataLengthCode encoding as the standard header
    // (one decoder, one table) rather than the 7 free bits below: those stay
    // reserved so the extended header never grows a second length encoding.
    using DataLengthCode = BitfieldMember<8 + 29, 3>;
    using HasTimestamp = BitfieldMember<40, 1>;
    // Bits 41-47 are free. Reserved; a receiver must ignore them.
};

struct UartHeaderLayout {
    using IdleDelimited = BitfieldMember<4, 1>;
    using IsExtendedLength = BitfieldMember<5, 1>;
    using DataLength = BitfieldMember<6, 2>;
};

struct UartHeaderExtendedLayout {
    using DataLengthExtended = BitfieldMember<6, 10>;
};

// UART configuration payload. Bits [0,4) are unused: a config field always
// carries an extended field header, whose id nibble already occupies them.
struct UartConfigPayloadLayout {
    using Baudrate = BitfieldMember<4, 32, uint32_t>;
};

struct SessionHeaderLayout {
    using Type = BitfieldMember<4, 4, data::SessionType>;
    using Nonce = BitfieldMember<8, 32, uint32_t>;
};

// The CAN hardware timestamp rides after the CAN payload as a plain 32-bit
// word. Specified little-endian like everything else in this file: it is the
// one field that used to be moved with a native memcpy, which made it the
// only wire field whose layout depended on the CPU endianness.
struct CanTimestampLayout {
    using TimestampUs = BitfieldMember<0, 32, uint32_t>;
};

} // namespace layouts

struct FieldHeader
    : utility::Bitfield<1>
    , layouts::FieldHeaderLayout {};

struct FieldHeaderExtended
    : utility::Bitfield<2>
    , layouts::FieldHeaderLayout
    , layouts::FieldHeaderExtendedLayout {};

struct CanHeader
    : utility::Bitfield<1>
    , layouts::CanHeaderLayout {};

struct CanHeaderStandard
    : utility::Bitfield<3>
    , layouts::CanHeaderLayout
    , layouts::CanHeaderStandardLayout {};

struct CanHeaderExtended
    : utility::Bitfield<6>
    , layouts::CanHeaderLayout
    , layouts::CanHeaderExtendedLayout {};

struct UartHeader
    : utility::Bitfield<1>
    , layouts::UartHeaderLayout {};

struct UartHeaderExtended
    : utility::Bitfield<2>
    , layouts::UartHeaderLayout
    , layouts::UartHeaderExtendedLayout {};

struct UartConfigPayload
    : utility::Bitfield<5>
    , layouts::UartConfigPayloadLayout {};

struct SessionHeader
    : utility::Bitfield<5>
    , layouts::SessionHeaderLayout {};

// Payloads that follow a SessionHeader whose Type says so. Both are plain
// byte-aligned blocks rather than packed bitfields: nothing here is bandwidth
// critical (one exchange per keepalive period) and a readable layout is worth
// more than four saved bytes.
struct TimeAnchorPayload : utility::Bitfield<8> {
    using Microframe = utility::BitfieldMember<0, 64, uint64_t>;
};

// 30 bytes since 2026-09-13: the PTPC diagnostics block (48 bytes that only
// the retired SOF->PTPC measurement path consumed) left with that path, and
// the whole payload now fits one 64-byte full-speed bulk packet. data::
// kSessionWireVersion guards the layout.
struct TimeStatusPayload : utility::Bitfield<30> {
    using Microframe = utility::BitfieldMember<0, 64, uint64_t>;
    using TimestampQuarterUs = utility::BitfieldMember<64, 32, uint32_t>;
    using TicksPerMicroframeQ16 = utility::BitfieldMember<96, 32, uint32_t>;
    using State = utility::BitfieldMember<128, 8, data::TimeState>;
    using AnomalyCount = utility::BitfieldMember<136, 24, uint32_t>;
    // Out-of-sample prediction error of the board's own fit, in Q16 timer ticks.
    // This is the quantity that becomes cross-board skew; see the accumulator in
    // sync/timebase.cpp for why the mean and the extremum say different things.
    using ResidualMeanQ16 = utility::BitfieldMember<160, 32, int32_t>;
    using ResidualAbsMaxQ16 = utility::BitfieldMember<192, 32, uint32_t>;
    using ResidualCount = utility::BitfieldMember<224, 16, uint16_t>;
};

struct PulseSchedulePayload : utility::Bitfield<8> {
    using Microframe = utility::BitfieldMember<0, 64, uint64_t>;
};

struct PulseReportPayload : utility::Bitfield<21> {
    using ScheduledMicroframe = utility::BitfieldMember<0, 64, uint64_t>;
    using CapturedMicroframeQ16 = utility::BitfieldMember<64, 64, uint64_t>;
    using TicksPerMicroframeQ16 = utility::BitfieldMember<128, 32, uint32_t>;
    // data::PulseReportFlags. A report is emitted for every schedule, so the
    // host can tell "the board refused to arm" from "the board armed and heard
    // nothing" -- two failures that look identical when only captures report.
    using Flags = utility::BitfieldMember<160, 8, uint8_t>;
};

// ---- Session-payload fingerprint: pinned into the EP0 version gate ----
//
// The EP0 fingerprint (libhcs/protocol/vendor_control.hpp) cannot see the
// payloads below -- they are defined here, after its header in the include
// graph. data::kSessionWireVersion (datas.hpp) therefore carries the session
// layout identity on the version check's behalf, and the static_assert below
// PINS it to the value computed here: change any session payload and the
// build fails until the constant is updated -- "forgot to bump" cannot
// compile. Folded: every payload size (added/removed/retyped fields), the
// offsets of the trailing fields (same-size reordering that sizeof alone
// cannot see), and the session type values a receiver's framing depends on.
constexpr uint16_t session_layout_fingerprint() {
    uint32_t h = 2166136261U; // FNV-1a 32-bit offset basis
    auto fold = [&h](uint32_t value) noexcept { h = (h ^ value) * 16777619U; };

    fold(sizeof(SessionHeader));
    fold(sizeof(TimeAnchorPayload));
    fold(sizeof(TimeStatusPayload));
    fold(sizeof(PulseSchedulePayload));
    fold(sizeof(PulseReportPayload));

    // Field positions via the BitfieldMember index constants (offsetof cannot
    // see through the member aliases).
    fold(TimeStatusPayload::AnomalyCount::kIndex);
    fold(TimeStatusPayload::AnomalyCount::kBitWidth);
    fold(TimeStatusPayload::ResidualMeanQ16::kIndex);
    fold(TimeStatusPayload::ResidualCount::kIndex);
    fold(TimeStatusPayload::ResidualCount::kBitWidth);
    fold(PulseReportPayload::Flags::kIndex);
    fold(TimeAnchorPayload::Microframe::kBitWidth);
    fold(TimeStatusPayload::TicksPerMicroframeQ16::kIndex);

    fold(static_cast<uint32_t>(data::SessionType::kStart));
    fold(static_cast<uint32_t>(data::SessionType::kStartAck));
    fold(static_cast<uint32_t>(data::SessionType::kKeepalive));
    fold(static_cast<uint32_t>(data::SessionType::kKeepaliveAck));
    fold(static_cast<uint32_t>(data::SessionType::kTimeAnchor));
    fold(static_cast<uint32_t>(data::SessionType::kTimeStatus));
    fold(static_cast<uint32_t>(data::SessionType::kPulseSchedule));
    fold(static_cast<uint32_t>(data::SessionType::kPulseReport));

    return static_cast<uint16_t>((h >> 16) ^ (h & 0xFFFFU));
}

static_assert(
    data::kSessionWireVersion == session_layout_fingerprint(),
    "session payload layout changed: update data::kSessionWireVersion "
    "(core/include/libhcs/data/datas.hpp) to the new session_layout_fingerprint() value");

struct GpioHeader : utility::Bitfield<2> {
    enum class PayloadEnum : uint8_t {
        kDigitalLow = 0b0000,
        kDigitalHigh = 0b0001,
        kAnalog = 0b0010,
        kDigitalReadConfig = 0b0100,
        kAnalogReadConfig = 0b0110,
    };

    using PayloadType = utility::BitfieldMember<4, 4, PayloadEnum>;
    using ChannelIndex = utility::BitfieldMember<8, 6>;
    using Timestamped = utility::BitfieldMember<15, 1>;
};

struct GpioReadConfigPayload : utility::Bitfield<3> {
    using Asap = utility::BitfieldMember<0, 1>;
    using RisingEdge = utility::BitfieldMember<1, 1>;
    using FallingEdge = utility::BitfieldMember<2, 1>;
    using Pull = utility::BitfieldMember<4, 2, data::GpioPull>;
    using PeriodMs = utility::BitfieldMember<8, 16, uint16_t>;
};

struct GpioDigitalReadTimestampPayload : utility::Bitfield<4> {
    using TimestampQuarterUs = utility::BitfieldMember<0, 32, uint32_t>;
};

struct GpioAnalogPayload : utility::Bitfield<2> {
    using Value = utility::BitfieldMember<0, 16, uint16_t>;
};

struct ImuHeader : utility::Bitfield<1> {
    enum class PayloadEnum : uint8_t {
        kAccelerometer = 0,
        kGyroscope = 1,
        kTemperature = 2,
    };
    using PayloadType = utility::BitfieldMember<4, 4, PayloadEnum>;
};

struct ImuAccelerometerPayload : utility::Bitfield<10> {
    using X = utility::BitfieldMember<0, 16, int16_t>;
    using Y = utility::BitfieldMember<16, 16, int16_t>;
    using Z = utility::BitfieldMember<32, 16, int16_t>;
    using TimestampQuarterUs = utility::BitfieldMember<48, 32, uint32_t>;
};

struct ImuGyroscopePayload : utility::Bitfield<10> {
    using X = utility::BitfieldMember<0, 16, int16_t>;
    using Y = utility::BitfieldMember<16, 16, int16_t>;
    using Z = utility::BitfieldMember<32, 16, int16_t>;
    using TimestampQuarterUs = utility::BitfieldMember<48, 32, uint32_t>;
};

struct ImuTemperaturePayload : utility::Bitfield<6> {
    using Temperature = utility::BitfieldMember<0, 16, uint16_t>;
    using TimestampQuarterUs = utility::BitfieldMember<16, 32, uint32_t>;
};

} // namespace libhcs::core::protocol
