#include "deserializer.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/protocol/can_dlc.hpp"
#include "core/src/coroutine/lifo.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/utility/assert.hpp"

namespace libhcs::core::protocol {

namespace {
// The 3-bit DataLengthCode field to payload bytes, keyed on IsLongFrame
// (semantics in core/src/protocol/protocol.hpp). Returns 0 for the reserved
// long-form code 7; the caller turns a zero-length data frame into a parse
// failure so a reserved encoding can never deliver a frame that was never
// sent on any bus.
uint8_t resolve_can_length(uint8_t code, bool is_long_frame) {
    if (!is_long_frame)
        return static_cast<uint8_t>(code + 1);
    if (code > 6) [[unlikely]]
        return 0;
    return static_cast<uint8_t>(payload_length(static_cast<uint8_t>(code + kCanFdLongDlcBase)));
}
} // namespace

coroutine::LifoTask<void> Deserializer::process_stream() {
    while (true) {
        utility::assert_debug(pending_bytes_ == 0 && requested_bytes_ == 0);

        FieldId id;
        {
            awaiting_field_first_byte_ = true;
            const auto* header_bytes = co_await peek_bytes(sizeof(FieldHeader));
            // Logically impossible; stack unwinding is invalid here.
            utility::assert_debug(header_bytes);
            auto header = FieldHeader::CRef{header_bytes};
            id = header.get<FieldHeader::Id>();
            awaiting_field_first_byte_ = false;
        }
        if (id == FieldId::kExtend) {
            const auto* header_bytes = co_await peek_bytes(sizeof(FieldHeaderExtended));
            if (!header_bytes) [[unlikely]] {
                // The transfer ended inside the extended field header: the
                // record is truncated, and only its first byte says kExtend.
                callback_.error_callback(FieldId::kExtend, data::DownlinkError::kMalformed);
                enter_discard_mode();
                continue;
            }
            auto header = FieldHeaderExtended::CRef{header_bytes};
            id = header.get<FieldHeaderExtended::IdExtended>();
            consume_peeked_partial(sizeof(FieldHeader));
        }

        RecordStatus status = RecordStatus::kDelivered;
        switch (id) {
        case FieldId::kCan0:
        case FieldId::kCan1:
        case FieldId::kCan2:
        case FieldId::kCan3: status = co_await process_can_field(id); break;
        case FieldId::kUartDbus:
        case FieldId::kUart0:
        case FieldId::kUart1:
        case FieldId::kUart2:
        case FieldId::kUart3:
        case FieldId::kUart7:
        case FieldId::kUart10: status = co_await process_uart_field(id); break;
        case FieldId::kGpio: status = co_await process_gpio_field(); break;
        case FieldId::kBuzzer: status = co_await process_buzzer_field(); break;
        case FieldId::kImu: status = co_await process_imu_field(id); break;
        case FieldId::kSession: status = co_await process_session_field(id); break;
        default: status = RecordStatus::kUnknownField; break;
        }
        if (status != RecordStatus::kDelivered) [[unlikely]] {
            if (status == RecordStatus::kRefused) {
                // The record is complete and its bytes consumed, so the next
                // record starts on a known boundary: skip this one and keep
                // delivering the rest of the batch.
                callback_.error_callback(id, data::DownlinkError::kRefused);
                continue;
            }
            // Truncated, structurally invalid, or of an unknown field id: the
            // next record's boundary is unknowable, so the rest of this
            // transfer is dropped. Re-entering discard mode from the
            // finish_transfer() unwind (discard_mode_ already set) is
            // harmless -- and is what reports the truncation exactly once.
            callback_.error_callback(
                id, status == RecordStatus::kUnknownField ? data::DownlinkError::kUnknownField
                                                          : data::DownlinkError::kMalformed);
            enter_discard_mode();
        }
    }
}

auto Deserializer::process_can_field(FieldId field_id) -> coroutine::LifoTask<RecordStatus> {
    data::CanDataView data_view;
    uint8_t can_data_length = 0;
    bool has_can_data = false;
    bool is_long_frame = false;
    bool has_timestamp = false;
    {
        const auto* header_bytes = co_await peek_bytes(sizeof(CanHeader));
        if (!header_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        auto header = CanHeader::CRef{header_bytes};

        data_view.is_extended_can_id = header.get<CanHeader::IsExtendedCanId>();
        data_view.is_remote_transmission = header.get<CanHeader::IsRemoteTransmission>();
        can_data_length = static_cast<uint8_t>(header.get<CanHeader::HasCanData>());
        has_can_data = can_data_length != 0;
        is_long_frame = header.get<CanHeader::IsLongFrame>();
        // Reserved combination: ISO CAN-FD has no remote frames, so no peer
        // can ever have encoded IsLongFrame on one.
        if (is_long_frame && data_view.is_remote_transmission) [[unlikely]]
            co_return RecordStatus::kMalformed;
    }

    if (data_view.is_extended_can_id) {
        const auto* header_ext_bytes = co_await peek_bytes(sizeof(CanHeaderExtended));
        if (!header_ext_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        auto header = CanHeaderExtended::CRef{header_ext_bytes};

        data_view.can_id = header.get<CanHeaderExtended::CanId>();
        can_data_length =
            can_data_length
                ? resolve_can_length(header.get<CanHeaderExtended::DataLengthCode>(), is_long_frame)
                : 0;
        has_timestamp = header.get<CanHeaderExtended::HasTimestamp>();
    } else {
        const auto* header_std_bytes = co_await peek_bytes(sizeof(CanHeaderStandard));
        if (!header_std_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        auto header = CanHeaderStandard::CRef{header_std_bytes};

        data_view.can_id = header.get<CanHeaderStandard::CanId>();
        can_data_length =
            can_data_length
                ? resolve_can_length(header.get<CanHeaderStandard::DataLengthCode>(), is_long_frame)
                : 0;
        has_timestamp = header.get<CanHeaderStandard::HasTimestamp>();
    }
    // resolve_can_length returns 0 for a reserved encoding; the record is
    // structurally undecodable, so the stream goes to discard mode rather
    // than delivering a frame that was never sent.
    //
    // Only a record that CLAIMS data can be undecodable this way. A record
    // with HasCanData clear is a zero-length frame -- a legal CAN data frame
    // (DLC 0) that the serializer writes exactly like this, remote or not.
    // Testing the length alone used to reject it and, through discard mode,
    // every field after it in the same transfer (host/tests/wire_protocol_test.cpp).
    if (has_can_data && can_data_length == 0)
        co_return RecordStatus::kMalformed;
    consume_peeked();

    // A later peek may reuse the pending cache, so keep the payload and its
    // timestamp in one window until the callback has consumed can_data.
    const size_t tail_size = can_data_length + (has_timestamp ? layouts::kCanStampBytes : 0);
    if (tail_size) {
        const auto* tail_bytes = co_await peek_bytes(tail_size);
        if (!tail_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;

        data_view.can_data = std::span<const std::byte>{tail_bytes, can_data_length};
        if (has_timestamp) {
            const auto* stamp_bytes = tail_bytes + can_data_length;
            data_view.sof_stamp =
                libhcs::time::SofStamp{utility::Bitfield<layouts::kCanStampBytes>::CRef{stamp_bytes}
                                           .get<layouts::CanStampLayout::Ticks>()};
        }
        consume_peeked();
    } else {
        data_view.can_data = std::span<const std::byte>{};
    }

    co_return callback_.can_deserialized_callback(field_id, data_view) ? RecordStatus::kDelivered
                                                                       : RecordStatus::kRefused;
}

auto Deserializer::process_uart_field(FieldId field_id) -> coroutine::LifoTask<RecordStatus> {
    data::UartDataView data_view;
    uint16_t uart_data_length;
    {
        const auto* header_bytes = co_await peek_bytes(sizeof(UartHeader));
        if (!header_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        auto header = UartHeader::CRef{header_bytes};
        data_view.idle_delimited = header.get<UartHeader::IdleDelimited>();

        if (!header.get<UartHeader::IsExtendedLength>()) {
            uart_data_length = header.get<UartHeader::DataLength>();
        } else {
            const auto* header_ext_bytes = co_await peek_bytes(sizeof(UartHeaderExtended));
            if (!header_ext_bytes) [[unlikely]]
                co_return RecordStatus::kMalformed;
            auto header_ext = UartHeaderExtended::CRef{header_ext_bytes};
            uart_data_length = header_ext.get<UartHeaderExtended::DataLengthExtended>();
            if (uart_data_length > sizeof(pending_bytes_buffer_)) [[unlikely]]
                co_return RecordStatus::kMalformed;
        }
    }
    consume_peeked();

    if (uart_data_length) {
        const auto* uart_data_bytes = co_await peek_bytes(uart_data_length);
        if (!uart_data_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        data_view.uart_data = std::span<const std::byte>{uart_data_bytes, uart_data_length};
        consume_peeked();
    } else {
        data_view.uart_data = std::span<const std::byte>{};
    }

    co_return callback_.uart_deserialized_callback(field_id, data_view) ? RecordStatus::kDelivered
                                                                        : RecordStatus::kRefused;
}

auto Deserializer::process_gpio_field() -> coroutine::LifoTask<RecordStatus> {
    GpioHeader::PayloadEnum payload_type;
    bool timestamped = false;
    uint8_t line = 0;
    {
        const auto* header_bytes = co_await peek_bytes(sizeof(GpioHeader));
        if (!header_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;

        auto header = GpioHeader::CRef{header_bytes};
        payload_type = header.get<GpioHeader::PayloadType>();
        timestamped = header.get<GpioHeader::Timestamped>();
        line = header.get<GpioHeader::Line>();
        consume_peeked();
    }

    switch (payload_type) {
    case GpioHeader::PayloadEnum::kDigitalLow:
    case GpioHeader::PayloadEnum::kDigitalHigh: {
        data::GpioDigitalDataView data_view{};
        data_view.high = payload_type == GpioHeader::PayloadEnum::kDigitalHigh;
        if (timestamped) {
            const auto* payload_bytes =
                co_await peek_bytes(sizeof(GpioDigitalReadTimestampPayload));
            if (!payload_bytes) [[unlikely]]
                co_return RecordStatus::kMalformed;
            auto payload = GpioDigitalReadTimestampPayload::CRef{payload_bytes};
            data_view.timestamp_quarter_us =
                payload.get<GpioDigitalReadTimestampPayload::TimestampQuarterUs>();
            consume_peeked();
        }
        co_return callback_.gpio_digital_data_deserialized_callback(line, data_view)
            ? RecordStatus::kDelivered
            : RecordStatus::kRefused;
    }
    case GpioHeader::PayloadEnum::kAnalog: {
        if (timestamped) [[unlikely]]
            co_return RecordStatus::kMalformed;
        const auto* payload_bytes = co_await peek_bytes(sizeof(GpioAnalogPayload));
        if (!payload_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;

        auto payload = GpioAnalogPayload::CRef{payload_bytes};
        data::GpioAnalogDataView data_view{};
        data_view.value = payload.get<GpioAnalogPayload::Value>();
        consume_peeked();

        co_return callback_.gpio_analog_data_deserialized_callback(line, data_view)
            ? RecordStatus::kDelivered
            : RecordStatus::kRefused;
    }
    case GpioHeader::PayloadEnum::kRead:
        if (timestamped) [[unlikely]]
            co_return RecordStatus::kMalformed;
        co_return callback_.gpio_read_deserialized_callback(line) ? RecordStatus::kDelivered
                                                                  : RecordStatus::kRefused;
    default: co_return RecordStatus::kMalformed;
    }
}

auto Deserializer::process_buzzer_field() -> coroutine::LifoTask<RecordStatus> {
    {
        const auto* header_bytes = co_await peek_bytes(sizeof(BuzzerHeader));
        if (!header_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        if (BuzzerHeader::CRef{header_bytes}.get<BuzzerHeader::Reserved>() != 0) [[unlikely]]
            co_return RecordStatus::kMalformed;
        consume_peeked();
    }
    const auto* payload_bytes = co_await peek_bytes(sizeof(BuzzerTonePayload));
    if (!payload_bytes) [[unlikely]]
        co_return RecordStatus::kMalformed;
    auto payload = BuzzerTonePayload::CRef{payload_bytes};
    const data::BuzzerToneDataView tone{
        .frequency_hz = payload.get<BuzzerTonePayload::FrequencyHz>(),
        .loudness = payload.get<BuzzerTonePayload::Loudness>(),
    };
    consume_peeked();
    co_return callback_.buzzer_tone_deserialized_callback(tone) ? RecordStatus::kDelivered
                                                                : RecordStatus::kRefused;
}

auto Deserializer::process_imu_field(FieldId) -> coroutine::LifoTask<RecordStatus> {
    ImuHeader::PayloadEnum payload_type;
    {
        const auto* header_bytes = co_await peek_bytes(sizeof(ImuHeader));
        if (!header_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;

        auto header = ImuHeader::CRef{header_bytes};
        payload_type = header.get<ImuHeader::PayloadType>();
        consume_peeked();
    }

    switch (payload_type) {
    case ImuHeader::PayloadEnum::kAccelerometer: {
        data::ImuAccelerometerDataView data_view{};
        const auto* payload_bytes = co_await peek_bytes(sizeof(ImuAccelerometerPayload));
        if (!payload_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        auto payload = ImuAccelerometerPayload::CRef{payload_bytes};
        data_view.x = payload.get<ImuAccelerometerPayload::X>();
        data_view.y = payload.get<ImuAccelerometerPayload::Y>();
        data_view.z = payload.get<ImuAccelerometerPayload::Z>();
        data_view.timestamp_quarter_us = payload.get<ImuAccelerometerPayload::TimestampQuarterUs>();
        consume_peeked();
        callback_.accelerometer_deserialized_callback(data_view);
        break;
    }
    case ImuHeader::PayloadEnum::kGyroscope: {
        data::ImuGyroscopeDataView data_view{};
        const auto* payload_bytes = co_await peek_bytes(sizeof(ImuGyroscopePayload));
        if (!payload_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        auto payload = ImuGyroscopePayload::CRef{payload_bytes};
        data_view.x = payload.get<ImuGyroscopePayload::X>();
        data_view.y = payload.get<ImuGyroscopePayload::Y>();
        data_view.z = payload.get<ImuGyroscopePayload::Z>();
        data_view.timestamp_quarter_us = payload.get<ImuGyroscopePayload::TimestampQuarterUs>();
        consume_peeked();
        callback_.gyroscope_deserialized_callback(data_view);
        break;
    }
    case ImuHeader::PayloadEnum::kTemperature: {
        data::ImuTemperatureDataView data_view{};
        const auto* payload_bytes = co_await peek_bytes(sizeof(ImuTemperaturePayload));
        if (!payload_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        auto payload = ImuTemperaturePayload::CRef{payload_bytes};
        data_view.raw_register_value = payload.get<ImuTemperaturePayload::Temperature>();
        data_view.timestamp_quarter_us = payload.get<ImuTemperaturePayload::TimestampQuarterUs>();
        consume_peeked();
        callback_.temperature_deserialized_callback(data_view);
        break;
    }
    default: co_return RecordStatus::kMalformed;
    }
    co_return RecordStatus::kDelivered;
}

auto Deserializer::process_session_field(FieldId) -> coroutine::LifoTask<RecordStatus> {
    const auto* header_bytes = co_await peek_bytes(sizeof(SessionHeader));
    if (!header_bytes) [[unlikely]]
        co_return RecordStatus::kMalformed;

    auto header = SessionHeader::CRef{header_bytes};
    data::SessionControlView data_view{};
    data_view.type = header.get<SessionHeader::Type>();
    data_view.nonce = header.get<SessionHeader::Nonce>();
    consume_peeked();

    // The four original session types are header-only; the types after them
    // carry a payload whose length is implied by the type. A receiver that does
    // not recognise a type therefore cannot skip it -- which is why the sender
    // only emits these to a peer known to support them (the EP0 fingerprint
    // gate for the host, the session gate for the boards).
    switch (data_view.type) {
    case data::SessionType::kTimeAnchor: {
        const auto* payload_bytes = co_await peek_bytes(sizeof(TimeAnchorPayload));
        if (!payload_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        auto payload = TimeAnchorPayload::CRef{payload_bytes};
        const data::TimeAnchorView anchor{
            .nonce = data_view.nonce,
            .microframe = payload.get<TimeAnchorPayload::Microframe>(),
        };
        consume_peeked();
        callback_.time_anchor_deserialized_callback(anchor);
        co_return RecordStatus::kDelivered;
    }
    case data::SessionType::kTimeStatus: {
        const auto* payload_bytes = co_await peek_bytes(sizeof(TimeStatusPayload));
        if (!payload_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        auto payload = TimeStatusPayload::CRef{payload_bytes};
        const data::TimeStatusView status{
            .nonce = data_view.nonce,
            .microframe = payload.get<TimeStatusPayload::Microframe>(),
            .microframe_fraction_q16 = payload.get<TimeStatusPayload::MicroframeFractionQ16>(),
            .timestamp_quarter_us = payload.get<TimeStatusPayload::TimestampQuarterUs>(),
            .ticks_per_microframe_q16 = payload.get<TimeStatusPayload::TicksPerMicroframeQ16>(),
            .state = payload.get<TimeStatusPayload::State>(),
            .anomaly_count = payload.get<TimeStatusPayload::AnomalyCount>(),
            .residual_mean_q16 = payload.get<TimeStatusPayload::ResidualMeanQ16>(),
            .residual_abs_max_q16 = payload.get<TimeStatusPayload::ResidualAbsMaxQ16>(),
            .residual_count = payload.get<TimeStatusPayload::ResidualCount>(),
            .capture_fresh_count = payload.get<TimeStatusPayload::CaptureFreshCount>(),
            .capture_stale_count = payload.get<TimeStatusPayload::CaptureStaleCount>(),
        };
        consume_peeked();
        callback_.time_status_deserialized_callback(status);
        co_return RecordStatus::kDelivered;
    }
    case data::SessionType::kPulseSchedule: {
        const auto* payload_bytes = co_await peek_bytes(sizeof(PulseSchedulePayload));
        if (!payload_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        auto payload = PulseSchedulePayload::CRef{payload_bytes};
        const data::PulseScheduleView schedule{
            .nonce = data_view.nonce,
            .microframe = payload.get<PulseSchedulePayload::Microframe>(),
        };
        consume_peeked();
        callback_.pulse_schedule_deserialized_callback(schedule);
        co_return RecordStatus::kDelivered;
    }
    case data::SessionType::kPulseReport: {
        const auto* payload_bytes = co_await peek_bytes(sizeof(PulseReportPayload));
        if (!payload_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        auto payload = PulseReportPayload::CRef{payload_bytes};
        const data::PulseReportView report{
            .nonce = data_view.nonce,
            .scheduled_microframe = payload.get<PulseReportPayload::ScheduledMicroframe>(),
            .captured_microframe_q16 = payload.get<PulseReportPayload::CapturedMicroframeQ16>(),
            .ticks_per_microframe_q16 = payload.get<PulseReportPayload::TicksPerMicroframeQ16>(),
            .flags = payload.get<PulseReportPayload::Flags>(),
        };
        consume_peeked();
        callback_.pulse_report_deserialized_callback(report);
        co_return RecordStatus::kDelivered;
    }
    case data::SessionType::kPortStatus: {
        const auto* header_bytes = co_await peek_bytes(sizeof(PortStatusHeader));
        if (!header_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        const auto header = PortStatusHeader::CRef{header_bytes};
        const data::DataId port = header.get<PortStatusHeader::Port>();
        const std::size_t length = header.get<PortStatusHeader::BodyLength>();
        consume_peeked();
        if (length == 0)
            co_return RecordStatus::kDelivered;

        const auto* body_bytes = co_await peek_bytes(length);
        if (!body_bytes) [[unlikely]]
            co_return RecordStatus::kMalformed;
        // 认识的口种按自己的正文解前缀(字段只追加); 不认识的、或比已知正文短的, 按长度
        // 跳过 -- 定界不丢。
        deliver_port_status(data_view.nonce, port, body_bytes, length);
        consume_peeked();
        co_return RecordStatus::kDelivered;
    }
    default: break;
    }

    callback_.session_control_deserialized_callback(data_view);

    co_return RecordStatus::kDelivered;
}

} // namespace libhcs::core::protocol
