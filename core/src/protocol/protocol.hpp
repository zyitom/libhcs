#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <variant>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/spec/port.hpp"
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
    // (kPortFd in kGetPortList, kGetPortConfig with wIndex = DataId::kCanN -- see
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

struct SessionHeaderLayout {
    using Type = BitfieldMember<4, 4, data::SessionType>;
    using Nonce = BitfieldMember<8, 32, uint32_t>;
};

// The CAN frame's place on the shared microframe axis rides after the CAN
// payload as three bytes: a libhcs::time::SofStamp, little-endian like
// everything else in this file. Until 2026-10-03 this slot was a 32-bit
// microsecond count of a board-local clock; see sof_stamp.hpp for why the
// stamp is both shorter and worth more.
struct CanStampLayout {
    using Ticks = BitfieldMember<0, libhcs::time::SofStamp::kBits, uint32_t>;
};
inline constexpr std::size_t kCanStampBytes = libhcs::time::SofStamp::kBits / 8;

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

// 34 bytes; with the session header it fits one 64-byte full-speed bulk
// packet. data::kSessionWireVersion guards the layout.
//
// The first eight bytes are the board's position on the microframe axis as one
// little-endian Q48.16 number -- the same eight bytes that used to hold a
// whole microframe count, now with the fraction the host needs to use the
// report as a time (data::TimeStatusView::microframe_fraction_q16).
struct TimeStatusPayload : utility::Bitfield<34> {
    using MicroframeFractionQ16 = utility::BitfieldMember<0, 16, uint16_t>;
    using Microframe = utility::BitfieldMember<16, 48, uint64_t>;
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
    // Hardware SOF captures accepted / skipped as stale since the last report.
    using CaptureFreshCount = utility::BitfieldMember<240, 16, uint16_t>;
    using CaptureStaleCount = utility::BitfieldMember<256, 16, uint16_t>;
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

// Port status record (data::SessionType::kPortStatus): one header byte, then a
// body whose layout the port's kind defines. The header says which port and how
// long the body is, so the whole payload is at most 16 bytes and a receiver
// can step over a body it does not know (or the tail of a longer one: fields
// are only ever appended).
struct PortStatusHeader : utility::Bitfield<1> {
    using Port = utility::BitfieldMember<0, 4, data::DataId>;
    using BodyLength = utility::BitfieldMember<4, 4, uint8_t>;
};
inline constexpr std::size_t kMaxPortStatusBody = 15;

// CAN body, 11 bytes: the error state packed into three, then the four
// 16-bit counts.
struct CanStatusBody : utility::Bitfield<11> {
    using Tec = utility::BitfieldMember<0, 8, uint8_t>;
    using Rec = utility::BitfieldMember<8, 7, uint8_t>;
    using Flags = utility::BitfieldMember<15, 3, uint8_t>; // data::CanStateFlags
    using LastError = utility::BitfieldMember<18, 3, data::CanLastError>;
    using DataLastError = utility::BitfieldMember<21, 3, data::CanLastError>;
    using TxCancelled = utility::BitfieldMember<24, 16, uint16_t>;
    using TxDropped = utility::BitfieldMember<40, 16, uint16_t>;
    using RxDropped = utility::BitfieldMember<56, 16, uint16_t>;
    using RxLost = utility::BitfieldMember<72, 16, uint16_t>;
};

// Link body (DataId::kSession), 6 bytes: the downlink error count and the most
// recent one.
struct LinkStatusBody : utility::Bitfield<6> {
    using DownlinkErrors = utility::BitfieldMember<0, 16, uint16_t>;
    using LastField = utility::BitfieldMember<16, 8, data::DataId>;
    using LastReason = utility::BitfieldMember<24, 8, data::DownlinkError>;
    using LastTransfer = utility::BitfieldMember<32, 16, uint16_t>;
};

// UART body, 12 bytes: six 16-bit counts.
struct UartStatusBody : utility::Bitfield<14> {
    using Overrun = utility::BitfieldMember<0, 16, uint16_t>;
    using Parity = utility::BitfieldMember<16, 16, uint16_t>;
    using Framing = utility::BitfieldMember<32, 16, uint16_t>;
    using Noise = utility::BitfieldMember<48, 16, uint16_t>;
    using Unattributed = utility::BitfieldMember<64, 16, uint16_t>;
    using TxDropped = utility::BitfieldMember<80, 16, uint16_t>;
    using RxDropped = utility::BitfieldMember<96, 16, uint16_t>;
};

// The one place that ties a status view (data::PortStatusVariant's
// alternatives) to its body: the body layout and the field-by-field mapping
// both ways. Which ports a view is for is the view's own is_for(). Serializer
// and Deserializer are generic over it (write_port_status / the kPortStatus
// case), and so is everything downstream of them.
template <typename View>
struct PortStatusRecord;

template <>
struct PortStatusRecord<data::LinkStatusView> {
    using Body = LinkStatusBody;

    static void encode(Body::Ref body, const data::LinkStatusView& view) noexcept {
        body.set<Body::DownlinkErrors>(view.downlink_errors);
        body.set<Body::LastField>(view.last_field);
        body.set<Body::LastReason>(view.last_reason);
        body.set<Body::LastTransfer>(view.last_transfer);
    }

    static data::LinkStatusView decode(Body::CRef body) noexcept {
        return {
            .downlink_errors = body.get<Body::DownlinkErrors>(),
            .last_field = body.get<Body::LastField>(),
            .last_reason = body.get<Body::LastReason>(),
            .last_transfer = body.get<Body::LastTransfer>(),
        };
    }
};

template <>
struct PortStatusRecord<data::CanStatusView> {
    using Body = CanStatusBody;

    static void encode(Body::Ref body, const data::CanStatusView& view) noexcept {
        body.set<Body::Tec>(view.tec);
        body.set<Body::Rec>(view.rec);
        body.set<Body::Flags>(view.flags);
        body.set<Body::LastError>(view.last_error);
        body.set<Body::DataLastError>(view.data_last_error);
        body.set<Body::TxCancelled>(view.tx_cancelled);
        body.set<Body::TxDropped>(view.tx_dropped);
        body.set<Body::RxDropped>(view.rx_dropped);
        body.set<Body::RxLost>(view.rx_lost);
    }

    static data::CanStatusView decode(Body::CRef body) noexcept {
        return {
            .tec = body.get<Body::Tec>(),
            .rec = body.get<Body::Rec>(),
            .last_error = body.get<Body::LastError>(),
            .data_last_error = body.get<Body::DataLastError>(),
            .flags = body.get<Body::Flags>(),
            .tx_cancelled = body.get<Body::TxCancelled>(),
            .tx_dropped = body.get<Body::TxDropped>(),
            .rx_dropped = body.get<Body::RxDropped>(),
            .rx_lost = body.get<Body::RxLost>(),
        };
    }
};

template <>
struct PortStatusRecord<data::UartStatusView> {
    using Body = UartStatusBody;

    static void encode(Body::Ref body, const data::UartStatusView& view) noexcept {
        body.set<Body::Overrun>(view.overrun);
        body.set<Body::Parity>(view.parity);
        body.set<Body::Framing>(view.framing);
        body.set<Body::Noise>(view.noise);
        body.set<Body::Unattributed>(view.unattributed);
        body.set<Body::TxDropped>(view.tx_dropped);
        body.set<Body::RxDropped>(view.rx_dropped);
    }

    static data::UartStatusView decode(Body::CRef body) noexcept {
        return {
            .overrun = body.get<Body::Overrun>(),
            .parity = body.get<Body::Parity>(),
            .framing = body.get<Body::Framing>(),
            .noise = body.get<Body::Noise>(),
            .unattributed = body.get<Body::Unattributed>(),
            .tx_dropped = body.get<Body::TxDropped>(),
            .rx_dropped = body.get<Body::RxDropped>(),
        };
    }
};

// A kind of port status: a view that says which ports it is for, with a record
// whose body fits the 4-bit length.
template <typename View>
concept PortStatusKind = requires(data::DataId port) {
    { View::is_for(port) } -> std::same_as<bool>;
    typename PortStatusRecord<View>::Body;
} && sizeof(typename PortStatusRecord<View>::Body) <= kMaxPortStatusBody;

// Calls f.template operator()<View>() for each kind in data::PortStatusVariant
// (std::monostate skipped), in order, until one returns true; returns whether
// one did. Compile-time unrolled: the code is the hand-written chain of ifs.
template <typename F>
constexpr bool any_port_status_kind(F&& f) {
    return [&]<std::size_t... kIndex>(std::index_sequence<kIndex...>) {
        const auto one = [&]<typename View>() {
            if constexpr (std::same_as<View, std::monostate>)
                return false;
            else
                return f.template operator()<View>();
        };
        return (
            one.template operator()<std::variant_alternative_t<kIndex, data::PortStatusVariant>>()
            || ...);
    }(std::make_index_sequence<std::variant_size_v<data::PortStatusVariant>>{});
}

// Every listed kind has a record: forgetting step 3 of datas.hpp's list is a
// compile error here, not a silently dropped status.
static_assert([]<std::size_t... kIndex>(std::index_sequence<kIndex...>) {
    return (
        (std::same_as<std::variant_alternative_t<kIndex, data::PortStatusVariant>, std::monostate>
         || PortStatusKind<std::variant_alternative_t<kIndex, data::PortStatusVariant>>)
        && ...);
}(std::make_index_sequence<std::variant_size_v<data::PortStatusVariant>>{}));

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
    fold(sizeof(PortStatusHeader));
    fold(sizeof(CanStatusBody));
    fold(sizeof(UartStatusBody));

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
    fold(TimeStatusPayload::Microframe::kIndex);
    fold(TimeStatusPayload::Microframe::kBitWidth);
    fold(TimeStatusPayload::MicroframeFractionQ16::kBitWidth);
    fold(TimeStatusPayload::CaptureFreshCount::kIndex);
    fold(TimeStatusPayload::CaptureStaleCount::kIndex);
    fold(sizeof(LinkStatusBody));
    fold(LinkStatusBody::LastField::kIndex);
    fold(LinkStatusBody::LastReason::kBitWidth);
    fold(LinkStatusBody::LastTransfer::kIndex);
    fold(PortStatusHeader::BodyLength::kIndex);
    fold(CanStatusBody::Flags::kIndex);
    fold(CanStatusBody::LastError::kIndex);
    fold(CanStatusBody::DataLastError::kIndex);
    fold(CanStatusBody::TxCancelled::kIndex);
    fold(CanStatusBody::RxDropped::kIndex);
    fold(CanStatusBody::RxLost::kIndex);
    fold(static_cast<uint32_t>(std::variant_size_v<data::PortStatusVariant>));
    fold(UartStatusBody::Parity::kIndex);
    fold(UartStatusBody::RxDropped::kIndex);

    fold(static_cast<uint32_t>(data::SessionType::kStart));
    fold(static_cast<uint32_t>(data::SessionType::kStartAck));
    fold(static_cast<uint32_t>(data::SessionType::kKeepalive));
    fold(static_cast<uint32_t>(data::SessionType::kKeepaliveAck));
    fold(static_cast<uint32_t>(data::SessionType::kTimeAnchor));
    fold(static_cast<uint32_t>(data::SessionType::kTimeStatus));
    fold(static_cast<uint32_t>(data::SessionType::kPulseSchedule));
    fold(static_cast<uint32_t>(data::SessionType::kPulseReport));
    fold(static_cast<uint32_t>(data::SessionType::kPortStatus));
    // The DataId ranges the views' is_for() pick a body by.
    fold(static_cast<uint32_t>(data::DataId::kCan3));
    fold(static_cast<uint32_t>(data::DataId::kUartDbus));
    fold(static_cast<uint32_t>(data::DownlinkError::kMalformed));
    fold(static_cast<uint32_t>(data::DownlinkError::kUnknownField));
    fold(static_cast<uint32_t>(data::DownlinkError::kRefused));
    fold(static_cast<uint32_t>(data::CanLastError::kCrc));
    fold(static_cast<uint32_t>(data::CanLastError::kNoChange));
    fold(static_cast<uint32_t>(data::kCanErrorPassive));
    fold(static_cast<uint32_t>(data::kCanWarning));
    fold(static_cast<uint32_t>(data::kCanBusOff));

    return static_cast<uint16_t>((h >> 16) ^ (h & 0xFFFFU));
}

static_assert(
    data::kSessionWireVersion == session_layout_fingerprint(),
    "session payload layout changed: update data::kSessionWireVersion "
    "(core/include/libhcs/data/datas.hpp) to the new session_layout_fingerprint() value");

// GPIO records: the board's GPIO port (data::DataId::kGpio), one line at a time.
// The record type and the timestamp bit fill the spare nibble of the field
// header's byte; the line number takes the second byte. Every GPIO record header
// is two bytes, whichever way it travels.
//
// How a line samples (direction, pull, period, edges, timestamps) is not in the
// stream: it is the line's EP0 declaration, which the board can refuse with a
// reason. What is left here is data, plus kRead, the one request a declared input
// takes at run time. A line the board does not have is dropped by the board
// (kGpioRecordSemantics in libhcs/protocol/vendor_control.hpp).
struct GpioHeader : utility::Bitfield<2> {
    enum class PayloadEnum : uint8_t {
        kDigitalLow = 0,  // level low: a write (downlink) or a sample (uplink)
        kDigitalHigh = 1, // level high
        kAnalog = 2,      // GpioAnalogPayload follows: PWM duty (downlink)
        kRead = 3,        // downlink, no payload: sample this input line once, now
    };

    using PayloadType = utility::BitfieldMember<4, 3, PayloadEnum>;
    // A GpioDigitalReadTimestampPayload follows (uplink level samples only).
    using Timestamped = utility::BitfieldMember<7, 1>;
    using Line = utility::BitfieldMember<8, 8, uint8_t>;
};

struct GpioDigitalReadTimestampPayload : utility::Bitfield<4> {
    using TimestampQuarterUs = utility::BitfieldMember<0, 32, uint32_t>;
};

struct GpioAnalogPayload : utility::Bitfield<2> {
    using Value = utility::BitfieldMember<0, 16, uint16_t>;
};

// One buzzer record, downlink only: the tone to play from now on. The header
// nibble is reserved and must be zero.
struct BuzzerHeader : utility::Bitfield<1> {
    using Reserved = utility::BitfieldMember<4, 4, uint8_t>;
};

struct BuzzerTonePayload : utility::Bitfield<3> {
    using FrequencyHz = utility::BitfieldMember<0, 16, uint16_t>;
    using Loudness = utility::BitfieldMember<16, 8, uint8_t>;
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
