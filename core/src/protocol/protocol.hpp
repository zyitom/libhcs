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
    using IsFdCan = BitfieldMember<4, 1>; // Currently invalid, reserved only
    using IsExtendedCanId = BitfieldMember<5, 1>;
    using IsRemoteTransmission = BitfieldMember<6, 1>;
    using HasCanData = BitfieldMember<7, 1>;
};

struct CanHeaderStandardLayout {
    using CanId = BitfieldMember<8, 11>;
    // HasTimestamp sits in the 2-bit gap [19:21) between CanId and
    // DataLengthCode -- avoiding overlap with DataLengthCode bit 1.
    using HasTimestamp = BitfieldMember<19, 1>;
    using DataLengthCode = BitfieldMember<21, 3>;
};

struct CanHeaderExtendedLayout {
    using CanId = BitfieldMember<8, 29>;
    using DataLengthCode = BitfieldMember<8 + 29, 3>;
    using HasTimestamp = BitfieldMember<40, 1>;
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

struct TimeStatusPayload : utility::Bitfield<78> {
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
    // Fitted PTPC units per microframe. Nominally 120000; published because the
    // conversion from a CAN hardware capture to the shared axis rides on it, and
    // a wrong slope there shows up as drift that looks exactly like skew.
    using PtpcUnitsPerMicroframe = utility::BitfieldMember<240, 32, uint32_t>;
    // The fitted line's anchor point, reported raw so the host can check the
    // pair against the slope itself. A reference whose units and microframe do
    // not advance in step with the published slope is the signature of a fit
    // that is computed correctly and stored inconsistently -- which no
    // cross-board comparison can distinguish from real skew.
    using PtpcReferenceUnits = utility::BitfieldMember<272, 64, uint64_t>;
    using PtpcReferenceMicroframe = utility::BitfieldMember<336, 64, uint64_t>;
    using PtpcResidualMean = utility::BitfieldMember<400, 32, int32_t>;
    using PtpcResidualAbsMax = utility::BitfieldMember<432, 32, uint32_t>;
    using PtpcStepMin = utility::BitfieldMember<464, 32, uint32_t>;
    using PtpcStepMax = utility::BitfieldMember<496, 32, uint32_t>;
    // The most recent RAW sample pair, straight out of the interrupt with no fit
    // applied. This is what lets the host build its own conversion instead of
    // trusting the board's -- the only way to tell a bad fit from bad samples.
    using PtpcRawNs = utility::BitfieldMember<528, 32, uint32_t>;
    using PtpcRawMicroframe = utility::BitfieldMember<560, 64, uint64_t>;
};

struct SyncSamplePayload : utility::Bitfield<18> {
    using Tag = utility::BitfieldMember<0, 32, uint32_t>;
    using MicroframeQ16 = utility::BitfieldMember<32, 64, uint64_t>;
    using Bus = utility::BitfieldMember<96, 8, uint8_t>;
    // The capture as the hardware reported it, before the board converted it.
    using PtpcNs = utility::BitfieldMember<104, 32, uint32_t>;
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
