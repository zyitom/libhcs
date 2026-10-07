// The bulk-stream wire format, checked end to end: whatever the serializer
// writes, the deserializer must hand back unchanged.
//
// Both halves live in core/ and are compiled into the host SDK AND into every
// board's firmware, so this is the one contract the two sides of the cable
// share. It needs no hardware: the test plays both ends.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/serializer.hpp"

namespace {

using libhcs::core::protocol::Deserializer;
using libhcs::core::protocol::FieldId;
using libhcs::core::protocol::Serializer;
using libhcs::data::DataId;
namespace data = libhcs::data;

using Result = Serializer::SerializeResult;

std::vector<std::byte> bytes(std::size_t size, std::uint8_t seed = 1) {
    std::vector<std::byte> out(size);
    for (std::size_t i = 0; i < size; ++i)
        out[i] = static_cast<std::byte>(seed + i * 7);
    return out;
}

// One transfer's worth of serialized fields, laid end to end.
class Wire final : public libhcs::core::protocol::SerializeBuffer {
public:
    Wire() { storage_.reserve(16 * 1024); } // spans handed out must stay valid

    std::span<std::byte> allocate(std::size_t size) noexcept override {
        if (storage_.size() + size > storage_.capacity())
            return {};
        const auto offset = storage_.size();
        storage_.resize(offset + size);
        return {storage_.data() + offset, size};
    }

    // Hand-assembled bytes in between serialized ones: reserved encodings the
    // serializer refuses to write are exactly what the receiver must survive.
    void append_raw(std::span<const std::byte> raw) {
        storage_.insert(storage_.end(), raw.begin(), raw.end());
    }

    [[nodiscard]] std::span<const std::byte> data() const { return storage_; }

private:
    std::vector<std::byte> storage_;
};

struct CanFrame {
    FieldId id;
    std::uint32_t can_id;
    std::vector<std::byte> payload;
    bool extended;
    bool remote;
    std::optional<libhcs::time::SofStamp> stamp;
};

struct UartChunk {
    FieldId id;
    std::vector<std::byte> payload;
    bool idle_delimited;
};

// One GPIO record, by the line of the GPIO port: a level (with its timestamp
// when sampled with one), an analog value, or a read request ('L' / 'A' / 'R').
struct GpioRecord {
    std::uint8_t line;
    char type;
    bool high = false;
    std::optional<std::uint32_t> timestamp = std::nullopt;
    std::uint16_t value = 0;

    friend bool operator==(const GpioRecord&, const GpioRecord&) = default;
};

// Copies everything out: the views the deserializer passes are only valid for
// the duration of the callback.
class Recorder final : public libhcs::core::protocol::DeserializeCallback {
public:
    bool can_deserialized_callback(FieldId id, const data::CanDataView& view) override {
        // 模拟"本板没有这个口": 名单里的字段号照单拒收, 其余照常交付。
        if (std::ranges::find(refuse_can, id) != refuse_can.end())
            return false;
        can.push_back({
            id,
            view.can_id,
            {view.can_data.begin(), view.can_data.end()},
            view.is_extended_can_id,
            view.is_remote_transmission,
            view.sof_stamp
        });
        order.push_back('C');
        return true;
    }
    bool uart_deserialized_callback(FieldId id, const data::UartDataView& view) override {
        uart.push_back({
            id, {view.uart_data.begin(), view.uart_data.end()},
             view.idle_delimited
        });
        order.push_back('U');
        return true;
    }
    bool gpio_digital_data_deserialized_callback(
        std::uint8_t line, const data::GpioDigitalDataView& view) override {
        gpio.push_back(
            {.line = line, .type = 'L', .high = view.high, .timestamp = view.timestamp_quarter_us});
        order.push_back('G');
        return true;
    }
    bool gpio_analog_data_deserialized_callback(
        std::uint8_t line, const data::GpioAnalogDataView& view) override {
        gpio.push_back({.line = line, .type = 'A', .value = view.value});
        order.push_back('G');
        return true;
    }
    bool gpio_read_deserialized_callback(std::uint8_t line) override {
        gpio.push_back({.line = line, .type = 'R'});
        order.push_back('G');
        return true;
    }
    bool buzzer_tone_deserialized_callback(const data::BuzzerToneDataView& view) override {
        tones.push_back({view.frequency_hz, view.loudness});
        order.push_back('B');
        return true;
    }
    void accelerometer_deserialized_callback(const data::ImuAccelerometerDataView&) override {}
    void gyroscope_deserialized_callback(const data::ImuGyroscopeDataView&) override {}
    void temperature_deserialized_callback(const data::ImuTemperatureDataView&) override {}
    void session_control_deserialized_callback(const data::SessionControlView& view) override {
        sessions.push_back(view);
        order.push_back('S');
    }
    void time_status_deserialized_callback(const data::TimeStatusView& view) override {
        time_status.push_back(view);
        order.push_back('T');
    }
    void port_status_deserialized_callback(
        std::uint32_t nonce, DataId port, const data::PortStatusVariant& status) override {
        if (const auto* link = std::get_if<data::LinkStatusView>(&status)) {
            link_status.push_back({nonce, port, *link});
            order.push_back('l');
        } else if (const auto* can = std::get_if<data::CanStatusView>(&status)) {
            can_status.push_back({nonce, port, *can});
            order.push_back('c');
        } else if (const auto* uart = std::get_if<data::UartStatusView>(&status)) {
            uart_status.push_back({nonce, port, *uart});
            order.push_back('u');
        }
    }
    void error_callback(FieldId field, data::DownlinkError reason) override {
        ++errors;
        error_fields.push_back(field);
        error_reasons.push_back(reason);
    }

    // can_deserialized_callback 对这些字段号返回 false(板子拒收)。
    std::vector<FieldId> refuse_can;
    std::vector<CanFrame> can;
    std::vector<UartChunk> uart;
    std::vector<GpioRecord> gpio;
    std::vector<std::pair<std::uint16_t, std::uint8_t>> tones;
    std::vector<data::SessionControlView> sessions;
    std::vector<data::TimeStatusView> time_status;
    template <typename View>
    struct PortStatus {
        std::uint32_t nonce;
        DataId port;
        View status;
    };
    std::vector<PortStatus<data::LinkStatusView>> link_status;
    std::vector<PortStatus<data::CanStatusView>> can_status;
    std::vector<PortStatus<data::UartStatusView>> uart_status;
    std::vector<char> order;
    int errors = 0;
    std::vector<FieldId> error_fields;
    std::vector<data::DownlinkError> error_reasons;
};

// One USB transfer: everything in one piece, then the end-of-transfer mark.
void deliver(Recorder& recorder, std::span<const std::byte> wire) {
    Deserializer deserializer{recorder};
    deserializer.feed(wire);
    deserializer.finish_transfer();
}

TEST(WireProtocol, CanFrameSurvivesTheRoundTrip) {
    Wire wire;
    Serializer serializer{wire};
    const auto payload = bytes(8);
    ASSERT_EQ(
        serializer.write_can(DataId::kCan1, {.can_id = 0x145, .can_data = payload}),
        Result::kSuccess);
    ASSERT_EQ(
        serializer.write_can(
            DataId::kCan2, {.can_id = 0x1ABCDEF0,
                            .can_data = std::span{payload}.first(3),
                            .is_extended_can_id = true,
                            .sof_stamp = libhcs::time::SofStamp{0xABCDEF}}),
        Result::kSuccess);
    // A zero-length data frame (DLC 0) and a remote frame: both carry no bytes
    // and both are real frames that must arrive, not be mistaken for garbage.
    ASSERT_EQ(
        serializer.write_can(DataId::kCan3, {.can_id = 0x7FF, .can_data = {}}), Result::kSuccess);
    ASSERT_EQ(
        serializer.write_can(
            DataId::kCan1, {.can_id = 0x10, .can_data = {}, .is_remote_transmission = true}),
        Result::kSuccess);

    Recorder recorder;
    deliver(recorder, wire.data());
    EXPECT_EQ(recorder.errors, 0);
    ASSERT_EQ(recorder.can.size(), 4U);

    EXPECT_EQ(recorder.can[0].id, DataId::kCan1);
    EXPECT_EQ(recorder.can[0].can_id, 0x145U);
    EXPECT_EQ(recorder.can[0].payload, payload);
    EXPECT_FALSE(recorder.can[0].extended);
    EXPECT_FALSE(recorder.can[0].stamp.has_value());

    EXPECT_EQ(recorder.can[1].id, DataId::kCan2);
    EXPECT_EQ(recorder.can[1].can_id, 0x1ABCDEF0U);
    EXPECT_EQ(recorder.can[1].payload, bytes(3));
    EXPECT_TRUE(recorder.can[1].extended);
    EXPECT_EQ(recorder.can[1].stamp, libhcs::time::SofStamp{0xABCDEF});

    EXPECT_EQ(recorder.can[2].can_id, 0x7FFU);
    EXPECT_TRUE(recorder.can[2].payload.empty());
    EXPECT_FALSE(recorder.can[2].remote);

    EXPECT_EQ(recorder.can[3].can_id, 0x10U);
    EXPECT_TRUE(recorder.can[3].payload.empty());
    EXPECT_TRUE(recorder.can[3].remote);
}

// The UART header has a short form (under 4 bytes) and an extended one; both
// boundaries and the largest field are exercised.
// The stamp is three bytes after the payload on both header forms, and every
// one of its 24 bits comes back. A frame without one pays nothing for the
// feature.
TEST(WireProtocol, CanStampCostsThreeBytesAndKeepsAllItsBits) {
    const auto payload = bytes(8);
    const auto size_of = [&](bool extended, std::optional<libhcs::time::SofStamp> stamp) {
        Wire wire;
        Serializer serializer{wire};
        EXPECT_EQ(
            serializer.write_can(
                DataId::kCan1, {.can_id = 0x123,
                                .can_data = payload,
                                .is_extended_can_id = extended,
                                .sof_stamp = stamp}),
            Result::kSuccess);
        return wire.data().size();
    };
    const libhcs::time::SofStamp any{0x000001};
    EXPECT_EQ(size_of(false, any), size_of(false, std::nullopt) + 3);
    EXPECT_EQ(size_of(true, any), size_of(true, std::nullopt) + 3);

    for (const std::uint32_t ticks : {0x000000U, 0x000001U, 0x800000U, 0xFFFFFFU, 0x5A5A5AU}) {
        for (const bool extended : {false, true}) {
            Wire wire;
            Serializer serializer{wire};
            ASSERT_EQ(
                serializer.write_can(
                    DataId::kCan2, {.can_id = 0x7FF,
                                    .can_data = payload,
                                    .is_extended_can_id = extended,
                                    .sof_stamp = libhcs::time::SofStamp{ticks}}),
                Result::kSuccess);
            Recorder recorder;
            deliver(recorder, wire.data());
            EXPECT_EQ(recorder.errors, 0);
            ASSERT_EQ(recorder.can.size(), 1U);
            EXPECT_EQ(recorder.can[0].payload, payload);
            ASSERT_TRUE(recorder.can[0].stamp.has_value());
            EXPECT_EQ(recorder.can[0].stamp->ticks, ticks);
        }
    }
}

// The time status report: the microframe position travels as Q48.16, and the
// capture counters ride at the end.
TEST(WireProtocol, TimeStatusSurvivesTheRoundTrip) {
    Wire wire;
    Serializer serializer{wire};
    const data::TimeStatusView sent{
        .nonce = 0xA5A55A5AU,
        .microframe = 0x0000'8765'4321'0FEDULL,
        .microframe_fraction_q16 = 0xBEEF,
        .timestamp_quarter_us = 0x12345678U,
        .ticks_per_microframe_q16 = 500U << 16U,
        .state = data::TimeState::kValid,
        .anomaly_count = 0x00ABCDEFU,
        .residual_mean_q16 = -12345,
        .residual_abs_max_q16 = 67890U,
        .residual_count = 4321,
        .capture_fresh_count = 2000,
        .capture_stale_count = 3,
    };
    ASSERT_EQ(serializer.write_time_status(sent), Result::kSuccess);

    Recorder recorder;
    deliver(recorder, wire.data());
    EXPECT_EQ(recorder.errors, 0);
    ASSERT_EQ(recorder.time_status.size(), 1U);
    const auto& got = recorder.time_status[0];
    EXPECT_EQ(got.nonce, sent.nonce);
    EXPECT_EQ(got.microframe, sent.microframe);
    EXPECT_EQ(got.microframe_fraction_q16, sent.microframe_fraction_q16);
    EXPECT_EQ(got.timestamp_quarter_us, sent.timestamp_quarter_us);
    EXPECT_EQ(got.ticks_per_microframe_q16, sent.ticks_per_microframe_q16);
    EXPECT_EQ(got.state, sent.state);
    EXPECT_EQ(got.anomaly_count, sent.anomaly_count);
    EXPECT_EQ(got.residual_mean_q16, sent.residual_mean_q16);
    EXPECT_EQ(got.residual_abs_max_q16, sent.residual_abs_max_q16);
    EXPECT_EQ(got.residual_count, sent.residual_count);
    EXPECT_EQ(got.capture_fresh_count, sent.capture_fresh_count);
    EXPECT_EQ(got.capture_stale_count, sent.capture_stale_count);
}

TEST(WireProtocol, UartPayloadSurvivesTheRoundTripAtEveryHeaderForm) {
    for (const std::size_t size :
         {std::size_t{1}, std::size_t{3}, std::size_t{4}, std::size_t{64}, std::size_t{300},
          std::size_t{1000}}) {
        Wire wire;
        Serializer serializer{wire};
        const auto payload = bytes(size, static_cast<std::uint8_t>(size));
        ASSERT_EQ(
            serializer.write_uart(
                DataId::kUart7, {.uart_data = payload, .idle_delimited = size % 2 == 0}),
            Result::kSuccess)
            << size;

        Recorder recorder;
        deliver(recorder, wire.data());
        EXPECT_EQ(recorder.errors, 0) << size;
        ASSERT_EQ(recorder.uart.size(), 1U) << size;
        EXPECT_EQ(recorder.uart[0].id, DataId::kUart7);
        EXPECT_EQ(recorder.uart[0].payload, payload) << size;
        EXPECT_EQ(recorder.uart[0].idle_delimited, size % 2 == 0) << size;
    }
}

// A ring that wrapped is written as two pieces; the reader must see one field.
TEST(WireProtocol, UartSuffixIsJoinedIntoOneField) {
    Wire wire;
    Serializer serializer{wire};
    const auto payload = bytes(40);
    ASSERT_EQ(
        serializer.write_uart(
            DataId::kUart0, {.uart_data = std::span{payload}.first(25)},
            std::span{payload}.subspan(25)),
        Result::kSuccess);

    Recorder recorder;
    deliver(recorder, wire.data());
    ASSERT_EQ(recorder.uart.size(), 1U);
    EXPECT_EQ(recorder.uart[0].payload, payload);
}

TEST(WireProtocol, FieldsOfDifferentKindsKeepTheirOrder) {
    Wire wire;
    Serializer serializer{wire};
    const auto payload = bytes(8);
    ASSERT_EQ(
        serializer.write_session_control(
            {.type = data::SessionType::kKeepalive, .nonce = 0xC0FFEE}),
        Result::kSuccess);
    ASSERT_EQ(
        serializer.write_can(DataId::kCan1, {.can_id = 1, .can_data = payload}), Result::kSuccess);
    ASSERT_EQ(serializer.write_uart(DataId::kUartDbus, {.uart_data = payload}), Result::kSuccess);
    ASSERT_EQ(
        serializer.write_can(DataId::kCan1, {.can_id = 2, .can_data = payload}), Result::kSuccess);

    Recorder recorder;
    deliver(recorder, wire.data());
    EXPECT_EQ(recorder.errors, 0);
    EXPECT_EQ(recorder.order, (std::vector<char>{'S', 'C', 'U', 'C'}));
    ASSERT_EQ(recorder.sessions.size(), 1U);
    EXPECT_EQ(recorder.sessions[0].type, data::SessionType::kKeepalive);
    EXPECT_EQ(recorder.sessions[0].nonce, 0xC0FFEEU);
    EXPECT_EQ(recorder.can[1].can_id, 2U);
}

// A GPIO record names its line of the GPIO port in its two-byte header: writes,
// requests and samples, every record type and value survive, in either direction.
TEST(WireProtocol, GpioRecordsSurviveTheRoundTripOnEveryLine) {
    Wire wire;
    Serializer serializer{wire};
    ASSERT_EQ(serializer.write_gpio_digital_value(0, {.high = true}), Result::kSuccess);
    EXPECT_EQ(wire.data().size(), 2U);
    ASSERT_EQ(serializer.write_gpio_digital_value(7, {.high = false}), Result::kSuccess);
    ASSERT_EQ(
        serializer.write_gpio_digital_value(2, {.high = true, .timestamp_quarter_us = 0xDEADBEEF}),
        Result::kSuccess);
    EXPECT_EQ(wire.data().size(), 10U);
    ASSERT_EQ(serializer.write_gpio_analog_value(1, {.value = 0xABCD}), Result::kSuccess);
    EXPECT_EQ(wire.data().size(), 14U);
    ASSERT_EQ(serializer.write_gpio_read(3), Result::kSuccess);
    EXPECT_EQ(wire.data().size(), 16U);

    Recorder recorder;
    deliver(recorder, wire.data());
    EXPECT_EQ(recorder.errors, 0);
    EXPECT_EQ(
        recorder.gpio, (std::vector<GpioRecord>{
                           {.line = 0, .type = 'L', .high = true},
                           {.line = 7, .type = 'L', .high = false},
                           {.line = 2, .type = 'L', .high = true, .timestamp = 0xDEADBEEF},
                           {.line = 1, .type = 'A', .value = 0xABCD},
                           {.line = 3, .type = 'R'},
    }));
}

// A buzzer record is four bytes: the compact field header and the tone.
TEST(WireProtocol, BuzzerTonesSurviveTheRoundTrip) {
    Wire wire;
    Serializer serializer{wire};
    ASSERT_EQ(
        serializer.write_buzzer_tone({.frequency_hz = 2093, .loudness = 128}), Result::kSuccess);
    EXPECT_EQ(wire.data().size(), 4U);
    ASSERT_EQ(serializer.write_buzzer_tone({.frequency_hz = 0, .loudness = 0}), Result::kSuccess);

    Recorder recorder;
    deliver(recorder, wire.data());
    EXPECT_EQ(recorder.errors, 0);
    EXPECT_EQ(
        recorder.tones, (std::vector<std::pair<std::uint16_t, std::uint8_t>>{
                            {2093, 128},
                            {   0,   0}
    }));
}

// A GPIO port has at most eight lines (spec::kMaxGpioLines).
TEST(WireProtocol, AGpioLineBeyondTheHeaderIsRefused) {
    Wire wire;
    Serializer serializer{wire};
    EXPECT_EQ(serializer.write_gpio_digital_value(8, {.high = true}), Result::kInvalidArgument);
    EXPECT_EQ(serializer.write_gpio_analog_value(8, {.value = 1}), Result::kInvalidArgument);
    EXPECT_EQ(serializer.write_gpio_read(255), Result::kInvalidArgument);
    EXPECT_TRUE(wire.data().empty());
}

// USB hands the stream over in packets that cut fields anywhere. Fed one byte
// at a time, the result must be the same as fed whole.
TEST(WireProtocol, AStreamCutAtEveryByteDecodesTheSame) {
    Wire wire;
    Serializer serializer{wire};
    const auto payload = bytes(8);
    const auto long_payload = bytes(200);
    ASSERT_EQ(
        serializer.write_can(DataId::kCan1, {.can_id = 0x201, .can_data = payload}),
        Result::kSuccess);
    ASSERT_EQ(serializer.write_uart(DataId::kUart7, {.uart_data = long_payload}), Result::kSuccess);
    ASSERT_EQ(
        serializer.write_can(
            DataId::kCan2, {.can_id = 0x12345, .can_data = payload, .is_extended_can_id = true}),
        Result::kSuccess);

    Recorder recorder;
    {
        Deserializer deserializer{recorder};
        for (const std::byte& byte : wire.data())
            deserializer.feed({&byte, 1});
        deserializer.finish_transfer();
    }
    EXPECT_EQ(recorder.errors, 0);
    ASSERT_EQ(recorder.can.size(), 2U);
    ASSERT_EQ(recorder.uart.size(), 1U);
    EXPECT_EQ(recorder.can[0].payload, payload);
    EXPECT_EQ(recorder.uart[0].payload, long_payload);
    EXPECT_EQ(recorder.can[1].can_id, 0x12345U);
}

// What cannot be put on the wire is refused at the sender, with nothing written.
TEST(WireProtocol, FieldsThatHaveNoEncodingAreRefused) {
    Wire wire;
    Serializer serializer{wire};
    const auto payload = bytes(2000);

    // An 11-bit id above 0x7FF, a 29-bit id above its range.
    EXPECT_EQ(
        serializer.write_can(DataId::kCan1, {.can_id = 0x800, .can_data = {}}),
        Result::kInvalidArgument);
    EXPECT_EQ(
        serializer.write_can(
            DataId::kCan1, {.can_id = 0x20000000, .can_data = {}, .is_extended_can_id = true}),
        Result::kInvalidArgument);
    // 9 bytes is not a CAN-FD length; a remote frame carries no data.
    EXPECT_EQ(
        serializer.write_can(DataId::kCan1, {.can_id = 1, .can_data = std::span{payload}.first(9)}),
        Result::kInvalidArgument);
    EXPECT_EQ(
        serializer.write_can(
            DataId::kCan1,
            {.can_id = 1, .can_data = std::span{payload}.first(1), .is_remote_transmission = true}),
        Result::kInvalidArgument);
    // More UART bytes than one field holds.
    EXPECT_EQ(
        serializer.write_uart(DataId::kUart7, {.uart_data = payload}), Result::kInvalidArgument);

    EXPECT_TRUE(wire.data().empty());
}

// A transfer that ends in the middle of a field is a truncated field: reported,
// dropped, and the next transfer starts clean.
TEST(WireProtocol, ATruncatedTransferIsDroppedAndTheNextOneDecodes) {
    Wire wire;
    Serializer serializer{wire};
    const auto payload = bytes(8);
    ASSERT_EQ(
        serializer.write_can(DataId::kCan1, {.can_id = 0x10, .can_data = payload}),
        Result::kSuccess);
    const auto whole = wire.data();

    Recorder recorder;
    Deserializer deserializer{recorder};
    deserializer.feed(whole.first(whole.size() - 3));
    deserializer.finish_transfer();
    EXPECT_TRUE(recorder.can.empty());
    ASSERT_EQ(recorder.errors, 1);
    EXPECT_EQ(recorder.error_fields.back(), FieldId::kCan1);
    EXPECT_EQ(recorder.error_reasons.back(), data::DownlinkError::kMalformed);

    deserializer.feed(whole);
    deserializer.finish_transfer();
    ASSERT_EQ(recorder.can.size(), 1U);
    EXPECT_EQ(recorder.can[0].can_id, 0x10U);
    EXPECT_EQ(recorder.can[0].payload, payload);
}

// A record the receiver refuses (no such port on this board) costs itself
// alone: the records after it in the same batch are delivered, and the error
// callback names the field and the reason.
TEST(WireProtocol, ARefusedRecordIsSkippedAndTheRestIsDelivered) {
    Wire wire;
    Serializer serializer{wire};
    const auto payload = bytes(8);
    ASSERT_EQ(
        serializer.write_can(DataId::kCan1, {.can_id = 0x11, .can_data = payload}),
        Result::kSuccess);
    // kCan3 is not on this "board": can_deserialized_callback refuses it.
    ASSERT_EQ(
        serializer.write_can(DataId::kCan3, {.can_id = 0x22, .can_data = payload}),
        Result::kSuccess);
    ASSERT_EQ(serializer.write_uart(DataId::kUart7, {.uart_data = payload}), Result::kSuccess);
    ASSERT_EQ(
        serializer.write_can(DataId::kCan1, {.can_id = 0x33, .can_data = payload}),
        Result::kSuccess);

    Recorder recorder;
    recorder.refuse_can.push_back(FieldId::kCan3);
    deliver(recorder, wire.data());

    EXPECT_EQ(recorder.order, (std::vector<char>{'C', 'U', 'C'}))
        << "the refused record alone was skipped, its neighbours delivered";
    EXPECT_EQ(recorder.can[0].can_id, 0x11U);
    EXPECT_EQ(recorder.can[1].can_id, 0x33U);
    ASSERT_EQ(recorder.errors, 1);
    EXPECT_EQ(recorder.error_fields.back(), FieldId::kCan3);
    EXPECT_EQ(recorder.error_reasons.back(), data::DownlinkError::kRefused);
}

// A reserved encoding loses the framing: the rest of the transfer is discarded,
// and the next transfer starts clean.
TEST(WireProtocol, AMalformedRecordDiscardsTheRestOfTheTransfer) {
    const auto payload = bytes(8);

    // A reserved encoding, hand-assembled: IsLongFrame on a remote frame (ISO
    // CAN-FD has no remote frames). Field id kCan1, no parser can size it.
    const std::array<std::byte, 3> malformed{
        std::byte{0x03 | 0x10 | 0x40}, // id=kCan1, IsLongFrame, IsRemoteTransmission
        std::byte{0xFF}, std::byte{0xFF}};
    Wire batch;
    Serializer batch_serializer{batch};
    ASSERT_EQ(
        batch_serializer.write_can(DataId::kCan2, {.can_id = 0x22, .can_data = payload}),
        Result::kSuccess);
    batch.append_raw(malformed);
    ASSERT_EQ(
        batch_serializer.write_can(DataId::kCan2, {.can_id = 0x44, .can_data = payload}),
        Result::kSuccess);

    Recorder recorder;
    deliver(recorder, batch.data());
    EXPECT_EQ(recorder.order, (std::vector<char>{'C'}))
        << "only the record before the malformed one was delivered";
    ASSERT_EQ(recorder.errors, 1);
    EXPECT_EQ(recorder.error_fields.back(), FieldId::kCan1);
    EXPECT_EQ(recorder.error_reasons.back(), data::DownlinkError::kMalformed);
}

// A field id no parser knows cannot be sized, so it takes the rest of the
// transfer with it -- reported as kUnknownField.
TEST(WireProtocol, AnUnknownFieldIdDiscardsTheRestOfTheTransfer) {
    // An extended field header naming an id outside the DataId range. The
    // extended id rides bits 4-11 (it shares the first byte with the compact
    // field header), so 0x2A = byte0 0xA0 + byte1 0x02.
    const std::array<std::byte, 2> unknown{std::byte{0xA0}, std::byte{0x02}};
    Wire batch;
    Serializer batch_serializer{batch};
    ASSERT_EQ(
        batch_serializer.write_uart(DataId::kUart7, {.uart_data = bytes(8)}), Result::kSuccess);
    batch.append_raw(unknown);
    ASSERT_EQ(
        batch_serializer.write_can(DataId::kCan2, {.can_id = 0x44, .can_data = bytes(8)}),
        Result::kSuccess);

    Recorder recorder;
    deliver(recorder, batch.data());
    EXPECT_EQ(recorder.order, (std::vector<char>{'U'}));
    ASSERT_EQ(recorder.errors, 1);
    EXPECT_EQ(recorder.error_fields.back(), static_cast<FieldId>(0x2A));
    EXPECT_EQ(recorder.error_reasons.back(), data::DownlinkError::kUnknownField);
}

// Port status records: every field survives, the port and the nonce ride along,
// and two records back to back (what a keepalive round with two changed ports
// writes) stay framed. One record kind for every port; the body is the port
// kind's, and the whole payload stays within 16 bytes.
TEST(WireProtocol, PortStatusSurvivesTheRoundTrip) {
    Wire wire;
    Serializer serializer{wire};
    const data::CanStatusView can{
        .tec = 255,
        .rec = 127,
        .last_error = data::CanLastError::kAck,
        .data_last_error = data::CanLastError::kBit1,
        .flags = data::kCanErrorPassive | data::kCanWarning | data::kCanBusOff,
        .tx_cancelled = 0xBEEF,
        .tx_dropped = 5,
        .rx_dropped = 0xFFFF,
        .rx_lost = 0x1234,
    };
    const data::UartStatusView uart{
        .overrun = 1,
        .parity = 2,
        .framing = 0xFFFF,
        .noise = 4,
        .unattributed = 7,
        .tx_dropped = 5,
        .rx_dropped = 6};
    const data::LinkStatusView link{
        .downlink_errors = 42,
        .last_field = DataId::kCan3,
        .last_reason = data::DownlinkError::kRefused,
        .last_transfer = 7,
    };
    ASSERT_EQ(serializer.write_port_status(0x11223344U, DataId::kSession, link), Result::kSuccess);
    ASSERT_EQ(serializer.write_port_status(0x11223344U, DataId::kCan3, can), Result::kSuccess);
    ASSERT_EQ(serializer.write_port_status(0x11223344U, DataId::kUart7, uart), Result::kSuccess);
    // Each: 5 bytes of session header + 1 byte port/length + the body (link 6, CAN 11, UART 14).
    EXPECT_EQ(wire.data().size(), (5U + 1U + 6U) + (5U + 1U + 11U) + (5U + 1U + 14U));
    // A view is only for its own ports.
    EXPECT_EQ(serializer.write_port_status(1, DataId::kUart7, can), Result::kInvalidArgument);

    Recorder recorder;
    deliver(recorder, wire.data());
    EXPECT_EQ(recorder.errors, 0);
    EXPECT_EQ(recorder.order, (std::vector<char>{'l', 'c', 'u'}));
    ASSERT_EQ(recorder.link_status.size(), 1U);
    EXPECT_EQ(recorder.link_status[0].port, DataId::kSession);
    EXPECT_EQ(recorder.link_status[0].status, link);
    ASSERT_EQ(recorder.can_status.size(), 1U);
    EXPECT_EQ(recorder.can_status[0].nonce, 0x11223344U);
    EXPECT_EQ(recorder.can_status[0].port, DataId::kCan3);
    EXPECT_EQ(recorder.can_status[0].status, can);
    ASSERT_EQ(recorder.uart_status.size(), 1U);
    EXPECT_EQ(recorder.uart_status[0].port, DataId::kUart7);
    EXPECT_EQ(recorder.uart_status[0].status, uart);
}

// A body the receiver does not know -- a port kind with no status layout, or a
// longer body from a newer board -- is stepped over by its length: the record
// after it still arrives, and a longer body still yields the fields it knows.
TEST(WireProtocol, APortStatusBodyIsSkippableByItsLength) {
    // Laid out by hand: field id kSession (14) in the low nibble, the session type in the
    // high nibble, a 32-bit little-endian nonce, then the port (low nibble) and the body
    // length (high nibble).
    const auto record = [](DataId port, std::span<const std::byte> body) {
        std::vector<std::byte> bytes{
            static_cast<std::byte>(
                std::to_underlying(DataId::kSession)
                | (std::to_underlying(data::SessionType::kPortStatus) << 4U)),
            std::byte{7},
            std::byte{0},
            std::byte{0},
            std::byte{0},
            static_cast<std::byte>(std::to_underlying(port) | (body.size() << 4U))};
        bytes.insert(bytes.end(), body.begin(), body.end());
        return bytes;
    };
    std::array<std::byte, 15> longer{};
    longer[0] = std::byte{42}; // tec
    const std::array<std::byte, 3> imu{std::byte{1}, std::byte{2}, std::byte{3}};

    std::vector<std::byte> wire = record(DataId::kImu, imu);
    const auto can = record(DataId::kCan1, longer);
    wire.insert(wire.end(), can.begin(), can.end());

    Recorder recorder;
    deliver(recorder, wire);
    EXPECT_EQ(recorder.errors, 0);
    ASSERT_EQ(recorder.can_status.size(), 1U);
    EXPECT_EQ(recorder.can_status[0].port, DataId::kCan1);
    EXPECT_EQ(recorder.can_status[0].status.tec, 42);
}

} // namespace
