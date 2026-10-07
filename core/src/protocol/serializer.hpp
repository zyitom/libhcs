#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <utility>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/protocol/can_dlc.hpp"
#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/verify.hpp"

namespace libhcs::core::protocol {

class SerializeBuffer {
public:
    SerializeBuffer() = default;
    SerializeBuffer(const SerializeBuffer&) = delete;
    SerializeBuffer& operator=(const SerializeBuffer&) = delete;
    SerializeBuffer(SerializeBuffer&&) = delete;
    SerializeBuffer& operator=(SerializeBuffer&&) = delete;
    virtual ~SerializeBuffer() noexcept = default;

    virtual std::span<std::byte> allocate(std::size_t size) noexcept = 0;
};

class Serializer {
public:
    enum class SerializeResult : std::uint8_t { kSuccess = 0, kBadAlloc = 1, kInvalidArgument = 2 };

    explicit Serializer(SerializeBuffer& buffer) noexcept
        : buffer_(buffer) {}

    [[nodiscard]] SerializeResult
        write_can(FieldId field_id, const data::CanDataView& view) noexcept {
        return emit(required_can_size(field_id, view), [&](Cursor& cursor) {
            write_field_header(cursor, field_id);

            const std::size_t can_data_length = view.can_data.size();
            const bool has_data = can_data_length != 0;
            const bool is_long_frame = can_data_length > kCanClassicMaxPayload;
            const bool has_timestamp = view.sof_stamp.has_value();
            std::uint8_t data_length_code = 0;
            if (has_data)
                // Long form stores the wire DLC offset by kCanFdLongDlcBase; the
                // length was already validated against the table by
                // required_can_size(), so the subtraction cannot underflow.
                data_length_code =
                    is_long_frame ? static_cast<std::uint8_t>(
                                        dlc_from_payload_len(can_data_length) - kCanFdLongDlcBase)
                                  : static_cast<std::uint8_t>(can_data_length - 1);

            // The standard and extended headers name the same fields and differ
            // only in layout (3 vs 5 bytes), so the field writes are spelled
            // once; each instantiation is inlined into its own branch and the
            // two can no longer drift apart.
            const auto write_header = [&]<typename Header>() {
                auto header = cursor.emplace<Header>();
                header.template set<typename Header::IsLongFrame>(is_long_frame);
                header.template set<typename Header::IsExtendedCanId>(view.is_extended_can_id);
                header.template set<typename Header::IsRemoteTransmission>(
                    view.is_remote_transmission);
                header.template set<typename Header::HasTimestamp>(has_timestamp);
                header.template set<typename Header::HasCanData>(has_data);
                header.template set<typename Header::CanId>(view.can_id);
                header.template set<typename Header::DataLengthCode>(data_length_code);
            };
            if (view.is_extended_can_id)
                write_header.template operator()<CanHeaderExtended>();
            else
                write_header.template operator()<CanHeaderStandard>();

            cursor.copy(view.can_data);

            if (has_timestamp) {
                cursor.emplace<utility::Bitfield<layouts::kCanStampBytes>>()
                    .set<layouts::CanStampLayout::Ticks>(view.sof_stamp->ticks);
            }
        });
    }

    [[nodiscard]] SerializeResult write_uart(
        FieldId field_id, const data::UartDataView& view,
        std::span<const std::byte> suffix_data = {}) noexcept {
        return emit(required_uart_size(field_id, view, suffix_data), [&](Cursor& cursor) {
            write_field_header(cursor, field_id);

            const std::size_t uart_data_length = view.uart_data.size() + suffix_data.size();
            if (uart_data_length >= 4) {
                auto header = cursor.emplace<UartHeaderExtended>();
                header.set<UartHeaderExtended::IdleDelimited>(view.idle_delimited);
                header.set<UartHeaderExtended::IsExtendedLength>(true);
                header.set<UartHeaderExtended::DataLengthExtended>(
                    static_cast<std::uint16_t>(uart_data_length));
            } else {
                auto header = cursor.emplace<UartHeader>();
                header.set<UartHeader::IdleDelimited>(view.idle_delimited);
                header.set<UartHeader::IsExtendedLength>(false);
                header.set<UartHeader::DataLength>(static_cast<std::uint8_t>(uart_data_length));
            }

            cursor.copy(view.uart_data);
            cursor.copy(suffix_data);
        });
    }

    // A level of a line of the GPIO port: a write to an output line (downlink) or
    // a sample of an input line (uplink, optionally timestamped).
    [[nodiscard]] SerializeResult write_gpio_digital_value(
        std::uint8_t line, const data::GpioDigitalDataView& view) noexcept {
        libhcs_VERIFY_LIKELY(line < spec::kMaxGpioLines, SerializeResult::kInvalidArgument);
        const auto payload_type = view.high ? GpioHeader::PayloadEnum::kDigitalHigh
                                            : GpioHeader::PayloadEnum::kDigitalLow;
        const bool timestamped = view.timestamp_quarter_us.has_value();
        const std::size_t size =
            sizeof(GpioHeader) + (timestamped ? sizeof(GpioDigitalReadTimestampPayload) : 0U);
        return emit(size, [&](Cursor& cursor) {
            write_gpio_header(cursor, line, payload_type, timestamped);
            if (timestamped) {
                cursor.emplace<GpioDigitalReadTimestampPayload>()
                    .set<GpioDigitalReadTimestampPayload::TimestampQuarterUs>(
                        *view.timestamp_quarter_us);
            }
        });
    }

    // A PWM duty for an output line.
    [[nodiscard]] SerializeResult
        write_gpio_analog_value(std::uint8_t line, const data::GpioAnalogDataView& view) noexcept {
        libhcs_VERIFY_LIKELY(line < spec::kMaxGpioLines, SerializeResult::kInvalidArgument);
        return emit(sizeof(GpioHeader) + sizeof(GpioAnalogPayload), [&](Cursor& cursor) {
            write_gpio_header(cursor, line, GpioHeader::PayloadEnum::kAnalog, false);
            cursor.emplace<GpioAnalogPayload>().set<GpioAnalogPayload::Value>(view.value);
        });
    }

    // Asks an input line for one sample, now.
    [[nodiscard]] SerializeResult write_gpio_read(std::uint8_t line) noexcept {
        libhcs_VERIFY_LIKELY(line < spec::kMaxGpioLines, SerializeResult::kInvalidArgument);
        return emit(sizeof(GpioHeader), [&](Cursor& cursor) {
            write_gpio_header(cursor, line, GpioHeader::PayloadEnum::kRead, false);
        });
    }

    // The tone the buzzer plays from now on (0 Hz or loudness 0 = silence).
    [[nodiscard]] SerializeResult write_buzzer_tone(const data::BuzzerToneDataView& view) noexcept {
        return emit(sizeof(BuzzerHeader) + sizeof(BuzzerTonePayload), [&](Cursor& cursor) {
            write_field_header(cursor, FieldId::kBuzzer);
            cursor.emplace<BuzzerHeader>().set<BuzzerHeader::Reserved>(0);
            auto payload = cursor.emplace<BuzzerTonePayload>();
            payload.set<BuzzerTonePayload::FrequencyHz>(view.frequency_hz);
            payload.set<BuzzerTonePayload::Loudness>(view.loudness);
        });
    }

    [[nodiscard]] SerializeResult
        write_imu_accelerometer(const data::ImuAccelerometerDataView& view) noexcept {
        return write_imu<ImuHeader::PayloadEnum::kAccelerometer, ImuAccelerometerPayload>(
            [&](auto payload) {
                payload.template set<ImuAccelerometerPayload::X>(view.x);
                payload.template set<ImuAccelerometerPayload::Y>(view.y);
                payload.template set<ImuAccelerometerPayload::Z>(view.z);
                payload.template set<ImuAccelerometerPayload::TimestampQuarterUs>(
                    view.timestamp_quarter_us);
            });
    }

    [[nodiscard]] SerializeResult
        write_imu_gyroscope(const data::ImuGyroscopeDataView& view) noexcept {
        return write_imu<ImuHeader::PayloadEnum::kGyroscope, ImuGyroscopePayload>(
            [&](auto payload) {
                payload.template set<ImuGyroscopePayload::X>(view.x);
                payload.template set<ImuGyroscopePayload::Y>(view.y);
                payload.template set<ImuGyroscopePayload::Z>(view.z);
                payload.template set<ImuGyroscopePayload::TimestampQuarterUs>(
                    view.timestamp_quarter_us);
            });
    }

    [[nodiscard]] SerializeResult
        write_imu_temperature(const data::ImuTemperatureDataView& view) noexcept {
        return write_imu<ImuHeader::PayloadEnum::kTemperature, ImuTemperaturePayload>(
            [&](auto payload) {
                payload.template set<ImuTemperaturePayload::Temperature>(view.raw_register_value);
                payload.template set<ImuTemperaturePayload::TimestampQuarterUs>(
                    view.timestamp_quarter_us);
            });
    }

    [[nodiscard]] SerializeResult
        write_session_control(const data::SessionControlView& view) noexcept {
        return emit(required_session_size(), [&](Cursor& cursor) {
            write_session_header(cursor, view.type, view.nonce);
        });
    }

    // Session field carrying a kTimeAnchor payload. Kept separate from
    // write_session_control() rather than folded into it, because the two have
    // different sizes and the fixed-size path is on the keepalive hot path of
    // every board, including the ones that know nothing about time sync.
    [[nodiscard]] SerializeResult write_time_anchor(const data::TimeAnchorView& view) noexcept {
        return emit(required_session_size() + sizeof(TimeAnchorPayload), [&](Cursor& cursor) {
            write_session_header(cursor, data::SessionType::kTimeAnchor, view.nonce);
            cursor.emplace<TimeAnchorPayload>().set<TimeAnchorPayload::Microframe>(view.microframe);
        });
    }

    [[nodiscard]] SerializeResult write_time_status(const data::TimeStatusView& view) noexcept {
        return emit(required_session_size() + sizeof(TimeStatusPayload), [&](Cursor& cursor) {
            write_session_header(cursor, data::SessionType::kTimeStatus, view.nonce);

            auto payload = cursor.emplace<TimeStatusPayload>();
            payload.set<TimeStatusPayload::Microframe>(view.microframe);
            payload.set<TimeStatusPayload::MicroframeFractionQ16>(view.microframe_fraction_q16);
            payload.set<TimeStatusPayload::TimestampQuarterUs>(view.timestamp_quarter_us);
            payload.set<TimeStatusPayload::TicksPerMicroframeQ16>(view.ticks_per_microframe_q16);
            payload.set<TimeStatusPayload::State>(view.state);
            payload.set<TimeStatusPayload::AnomalyCount>(view.anomaly_count);
            payload.set<TimeStatusPayload::ResidualMeanQ16>(view.residual_mean_q16);
            payload.set<TimeStatusPayload::ResidualAbsMaxQ16>(view.residual_abs_max_q16);
            payload.set<TimeStatusPayload::ResidualCount>(view.residual_count);
            payload.set<TimeStatusPayload::CaptureFreshCount>(view.capture_fresh_count);
            payload.set<TimeStatusPayload::CaptureStaleCount>(view.capture_stale_count);
        });
    }

    [[nodiscard]] SerializeResult
        write_pulse_schedule(const data::PulseScheduleView& view) noexcept {
        return emit(required_session_size() + sizeof(PulseSchedulePayload), [&](Cursor& cursor) {
            write_session_header(cursor, data::SessionType::kPulseSchedule, view.nonce);
            cursor.emplace<PulseSchedulePayload>().set<PulseSchedulePayload::Microframe>(
                view.microframe);
        });
    }

    [[nodiscard]] SerializeResult write_pulse_report(const data::PulseReportView& view) noexcept {
        return emit(required_session_size() + sizeof(PulseReportPayload), [&](Cursor& cursor) {
            write_session_header(cursor, data::SessionType::kPulseReport, view.nonce);

            auto payload = cursor.emplace<PulseReportPayload>();
            payload.set<PulseReportPayload::ScheduledMicroframe>(view.scheduled_microframe);
            payload.set<PulseReportPayload::CapturedMicroframeQ16>(view.captured_microframe_q16);
            payload.set<PulseReportPayload::TicksPerMicroframeQ16>(view.ticks_per_microframe_q16);
            payload.set<PulseReportPayload::Flags>(view.flags);
        });
    }

    // One port's runtime status (data::SessionType::kPortStatus), appended
    // after a keepalive ack. Uplink only. The body layout is
    // PortStatusRecord<View>'s; this writer only frames it.
    template <PortStatusKind View>
    [[nodiscard]] SerializeResult
        write_port_status(uint32_t nonce, data::DataId port, const View& view) noexcept {
        using Record = PortStatusRecord<View>;
        using Body = typename Record::Body;
        libhcs_VERIFY_LIKELY(View::is_for(port), SerializeResult::kInvalidArgument);
        return emit(
            required_session_size() + sizeof(PortStatusHeader) + sizeof(Body), [&](Cursor& cursor) {
                write_session_header(cursor, data::SessionType::kPortStatus, nonce);

                auto header = cursor.emplace<PortStatusHeader>();
                header.set<PortStatusHeader::Port>(port);
                header.set<PortStatusHeader::BodyLength>(static_cast<uint8_t>(sizeof(Body)));
                Record::encode(cursor.template emplace<Body>(), view);
            });
    }

private:
    // Write position into the allocated field. Turns the hand-written pair
    // `auto x = Layout::Ref(cursor); cursor += sizeof(Layout);` into one call:
    // forgetting the second half compiled fine and silently made the next
    // field overwrite this one.
    class Cursor {
    public:
        constexpr explicit Cursor(std::byte* position) noexcept
            : position_(position) {}

        // `advance` defaults to the layout size; the field header, which shares
        // its last byte with the header after it, passes a smaller value.
        template <typename Layout>
        typename Layout::Ref emplace(std::size_t advance = sizeof(Layout)) noexcept {
            typename Layout::Ref ref{position_};
            position_ += advance;
            return ref;
        }

        void copy(std::span<const std::byte> bytes) noexcept {
            if (bytes.empty())
                return;
            std::memcpy(position_, bytes.data(), bytes.size());
            position_ += bytes.size();
        }

        [[nodiscard]] constexpr const std::byte* position() const noexcept { return position_; }

    private:
        std::byte* position_;
    };

    // Shared skeleton of every write_*: validated size -> allocate -> let
    // `write` fill it -> check it was filled exactly. This used to be copied
    // into each writer by hand, and two of the copies had already lost the
    // size assertion. `write` is a lambda and is always inlined, so the
    // generated code matches the hand-expanded form.
    template <typename Write>
    SerializeResult emit(std::size_t required, Write&& write) noexcept {
        libhcs_VERIFY_LIKELY(required, SerializeResult::kInvalidArgument);

        auto dst = buffer_.allocate(required);
        libhcs_VERIFY_LIKELY(!dst.empty(), SerializeResult::kBadAlloc);
        utility::assert_debug(dst.size() == required);

        Cursor cursor{dst.data()};
        std::forward<Write>(write)(cursor);

        utility::assert_debug(cursor.position() == dst.data() + dst.size());
        return SerializeResult::kSuccess;
    }

    // Header shared by every kSession payload.
    static void
        write_session_header(Cursor& cursor, data::SessionType type, uint32_t nonce) noexcept {
        write_field_header(cursor, FieldId::kSession);
        auto header = cursor.emplace<SessionHeader>();
        header.set<SessionHeader::Type>(type);
        header.set<SessionHeader::Nonce>(nonce);
    }

    static void write_gpio_header(
        Cursor& cursor, std::uint8_t line, GpioHeader::PayloadEnum payload_type,
        bool timestamped) noexcept {
        write_field_header(cursor, FieldId::kGpio);
        auto header = cursor.emplace<GpioHeader>();
        header.set<GpioHeader::PayloadType>(payload_type);
        header.set<GpioHeader::Timestamped>(timestamped);
        header.set<GpioHeader::Line>(line);
    }

    template <ImuHeader::PayloadEnum payload, typename Payload, typename WriteFields>
    SerializeResult write_imu(WriteFields&& write_fields) noexcept {
        return emit(required_imu_size(FieldId::kImu, payload), [&](Cursor& cursor) {
            write_field_header(cursor, FieldId::kImu);
            cursor.emplace<ImuHeader>().set<ImuHeader::PayloadType>(payload);
            std::forward<WriteFields>(write_fields)(cursor.emplace<Payload>());
        });
    }

    static constexpr bool use_extended_field_header(FieldId field_id) {
        utility::assert_debug(field_id != FieldId::kExtend);
        return static_cast<std::uint8_t>(field_id) > 0xF;
    }

    static constexpr std::size_t required_field_header_size(FieldId field_id) {
        return use_extended_field_header(field_id) ? sizeof(FieldHeaderExtended)
                                                   : sizeof(FieldHeader);
    }

    // The field header shares its last byte with the header that follows it
    // (the size computations subtract that byte back out), so it advances by
    // less than its sizeof.
    static void write_field_header(Cursor& cursor, FieldId field_id) noexcept {
        if (use_extended_field_header(field_id)) {
            static_assert(sizeof(FieldHeaderExtended) == sizeof(FieldHeader) + 1);
            auto header = cursor.emplace<FieldHeaderExtended>(1);
            header.set<FieldHeaderExtended::Id>(FieldId::kExtend);
            header.set<FieldHeaderExtended::IdExtended>(field_id);
        } else {
            auto header = cursor.emplace<FieldHeader>(0);
            header.set<FieldHeader::Id>(field_id);
        }
    }

    static std::size_t required_can_size(FieldId field_id, const data::CanDataView& view) noexcept {
        // A remote frame carries no data; this also rules out the reserved
        // IsLongFrame + remote combination, which needs data to be long.
        libhcs_VERIFY_LIKELY(!view.is_remote_transmission || view.can_data.empty(), 0);
        libhcs_VERIFY_LIKELY(view.can_data.size() <= kCanMaxPayload, 0);
        // Long payloads must be exact CAN-FD table entries: a length that is
        // not on the table (9-11, 13-15, ...) has no wire DLC to encode it.
        if (view.can_data.size() > kCanClassicMaxPayload)
            libhcs_VERIFY_LIKELY(dlc_from_payload_len(view.can_data.size()) != kDlcInvalid, 0);
        if (view.is_extended_can_id)
            libhcs_VERIFY_LIKELY(view.can_id <= 0x1FFFFFFF, 0);
        else
            libhcs_VERIFY_LIKELY(view.can_id <= 0x7FF, 0);

        const std::size_t field_header_bytes = required_field_header_size(field_id);
        const std::size_t can_header_bytes =
            view.is_extended_can_id ? sizeof(CanHeaderExtended) : sizeof(CanHeaderStandard);
        const std::size_t timestamp_bytes =
            view.sof_stamp.has_value() ? layouts::kCanStampBytes : 0;
        const std::size_t total =
            (field_header_bytes + can_header_bytes - 1) + view.can_data.size() + timestamp_bytes;
        utility::assert_debug(total <= kProtocolBufferSize);

        return total;
    }

    static std::size_t required_uart_size(
        FieldId field_id, const data::UartDataView& view,
        std::span<const std::byte> suffix_data) noexcept {
        const std::size_t field_header_bytes = required_field_header_size(field_id);

        const std::size_t uart_data_length = view.uart_data.size() + suffix_data.size();
        libhcs_VERIFY_LIKELY(uart_data_length <= kProtocolBufferSize, 0);

        const bool use_extended_length = uart_data_length >= 4;
        const std::size_t uart_header_bytes =
            use_extended_length ? sizeof(UartHeaderExtended) : sizeof(UartHeader);

        const std::size_t total = (field_header_bytes + uart_header_bytes - 1) + uart_data_length;
        libhcs_VERIFY_LIKELY(total <= kProtocolBufferSize, 0);

        return total;
    }

    static std::size_t
        required_imu_size(FieldId field_id, ImuHeader::PayloadEnum payload) noexcept {
        const std::size_t field_header_bytes = required_field_header_size(field_id);
        const std::size_t imu_header_bytes = sizeof(ImuHeader);
        std::size_t payload_bytes = 0;
        switch (payload) {
        case ImuHeader::PayloadEnum::kAccelerometer:
            payload_bytes = sizeof(ImuAccelerometerPayload);
            break;
        case ImuHeader::PayloadEnum::kGyroscope: payload_bytes = sizeof(ImuGyroscopePayload); break;
        case ImuHeader::PayloadEnum::kTemperature:
            payload_bytes = sizeof(ImuTemperaturePayload);
            break;
        default: return 0;
        }

        const std::size_t total = (field_header_bytes + imu_header_bytes - 1) + payload_bytes;
        utility::assert_debug(total <= kProtocolBufferSize);

        return total;
    }

    static constexpr std::size_t required_session_size() {
        constexpr std::size_t total = required_field_header_size(FieldId::kSession)
                                    + sizeof(SessionHeader) - sizeof(FieldHeader);
        utility::assert_debug(total <= kProtocolBufferSize);
        return total;
    }

    SerializeBuffer& buffer_;
};

} // namespace libhcs::core::protocol
