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
            const bool has_timestamp = view.timestamp_us.has_value();
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
                // Explicit little-endian to match every other wire field -- this
                // used to be a native memcpy, which made it the one field whose
                // layout depended on the CPU endianness. On a little-endian host
                // the bitfield store compiles to the same plain store.
                cursor.emplace<utility::Bitfield<4>>()
                    .set<layouts::CanTimestampLayout::TimestampUs>(*view.timestamp_us);
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

    // Sparse patch semantics: a view with nothing set writes no bytes at all and
    // reports success, so callers can pass a partially filled config through
    // unconditionally.
    [[nodiscard]] SerializeResult
        write_uart_config(FieldId field_id, const data::UartConfigView& view) noexcept {
        if (!view.baudrate.has_value())
            return SerializeResult::kSuccess;

        return emit(required_uart_config_size(field_id, *view.baudrate), [&](Cursor& cursor) {
            write_field_header(cursor, field_id);
            cursor.emplace<UartConfigPayload>().set<UartConfigPayload::Baudrate>(*view.baudrate);
        });
    }

    [[nodiscard]] SerializeResult write_gpio_digital_value(
        uint8_t channel_index, const data::GpioDigitalDataView& view) noexcept {
        utility::assert_debug(channel_index < (1U << GpioHeader::ChannelIndex::kBitWidth));
        const auto payload_type = view.high ? GpioHeader::PayloadEnum::kDigitalHigh
                                            : GpioHeader::PayloadEnum::kDigitalLow;
        const bool timestamped = view.timestamp_quarter_us.has_value();

        return emit(
            required_gpio_size(FieldId::kGpio, payload_type, timestamped), [&](Cursor& cursor) {
                write_field_header(cursor, FieldId::kGpio);

                auto header = cursor.emplace<GpioHeader>();
                header.set<GpioHeader::PayloadType>(payload_type);
                header.set<GpioHeader::ChannelIndex>(channel_index);
                header.set<GpioHeader::Timestamped>(timestamped);

                if (timestamped) {
                    cursor.emplace<GpioDigitalReadTimestampPayload>()
                        .set<GpioDigitalReadTimestampPayload::TimestampQuarterUs>(
                            *view.timestamp_quarter_us);
                }
            });
    }

    [[nodiscard]] SerializeResult write_gpio_digital_read_config(
        uint8_t channel_index, const data::GpioReadConfigView& view) noexcept {
        utility::assert_debug(channel_index < (1U << GpioHeader::ChannelIndex::kBitWidth));
        return emit(
            required_gpio_size(FieldId::kGpio, GpioHeader::PayloadEnum::kDigitalReadConfig),
            [&](Cursor& cursor) {
                write_field_header(cursor, FieldId::kGpio);

                auto header = cursor.emplace<GpioHeader>();
                header.set<GpioHeader::PayloadType>(GpioHeader::PayloadEnum::kDigitalReadConfig);
                header.set<GpioHeader::ChannelIndex>(channel_index);
                header.set<GpioHeader::Timestamped>(view.capture_timestamp);

                write_gpio_read_config_payload(cursor, view, view.rising_edge, view.falling_edge);
            });
    }

    [[nodiscard]] SerializeResult write_gpio_analog_value(
        uint8_t channel_index, const data::GpioAnalogDataView& view) noexcept {
        utility::assert_debug(channel_index < (1U << GpioHeader::ChannelIndex::kBitWidth));
        return emit(
            required_gpio_size(FieldId::kGpio, GpioHeader::PayloadEnum::kAnalog),
            [&](Cursor& cursor) {
                write_field_header(cursor, FieldId::kGpio);

                auto header = cursor.emplace<GpioHeader>();
                header.set<GpioHeader::PayloadType>(GpioHeader::PayloadEnum::kAnalog);
                header.set<GpioHeader::ChannelIndex>(channel_index);
                header.set<GpioHeader::Timestamped>(false);

                cursor.emplace<GpioAnalogPayload>().set<GpioAnalogPayload::Value>(view.value);
            });
    }

    [[nodiscard]] SerializeResult write_gpio_analog_read_config(
        uint8_t channel_index, const data::GpioReadConfigView& view) noexcept {
        utility::assert_debug(channel_index < (1U << GpioHeader::ChannelIndex::kBitWidth));
        libhcs_VERIFY_LIKELY(
            !view.falling_edge && !view.rising_edge, SerializeResult::kInvalidArgument);
        libhcs_VERIFY_LIKELY(!view.capture_timestamp, SerializeResult::kInvalidArgument);

        return emit(
            required_gpio_size(FieldId::kGpio, GpioHeader::PayloadEnum::kAnalogReadConfig),
            [&](Cursor& cursor) {
                write_field_header(cursor, FieldId::kGpio);

                auto header = cursor.emplace<GpioHeader>();
                header.set<GpioHeader::PayloadType>(GpioHeader::PayloadEnum::kAnalogReadConfig);
                header.set<GpioHeader::ChannelIndex>(channel_index);
                header.set<GpioHeader::Timestamped>(false);

                // The analog form has no edge semantics; the guards above make
                // that an argument error rather than a silently dropped bit.
                write_gpio_read_config_payload(cursor, view, false, false);
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
            payload.set<TimeStatusPayload::TimestampQuarterUs>(view.timestamp_quarter_us);
            payload.set<TimeStatusPayload::TicksPerMicroframeQ16>(view.ticks_per_microframe_q16);
            payload.set<TimeStatusPayload::State>(view.state);
            payload.set<TimeStatusPayload::AnomalyCount>(view.anomaly_count);
            payload.set<TimeStatusPayload::ResidualMeanQ16>(view.residual_mean_q16);
            payload.set<TimeStatusPayload::ResidualAbsMaxQ16>(view.residual_abs_max_q16);
            payload.set<TimeStatusPayload::ResidualCount>(view.residual_count);
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

    // Digital and analog read configs share this payload; only the edge bits
    // differ (analog has no edge semantics, so its caller passes false).
    static void write_gpio_read_config_payload(
        Cursor& cursor, const data::GpioReadConfigView& view, bool rising_edge,
        bool falling_edge) noexcept {
        auto payload = cursor.emplace<GpioReadConfigPayload>();
        payload.set<GpioReadConfigPayload::Asap>(view.asap);
        payload.set<GpioReadConfigPayload::RisingEdge>(rising_edge);
        payload.set<GpioReadConfigPayload::FallingEdge>(falling_edge);
        payload.set<GpioReadConfigPayload::Pull>(view.pull);
        payload.set<GpioReadConfigPayload::PeriodMs>(view.period_ms);
    }

    // The three IMU payloads share one skeleton (field header + ImuHeader +
    // fixed-size payload) and differ only in layout and field names.
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
        const std::size_t timestamp_bytes = view.timestamp_us.has_value() ? sizeof(uint32_t) : 0;
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

    static constexpr bool is_uart_config_field_id(FieldId field_id) {
        switch (field_id) {
        case FieldId::kUartDbusConfig:
        case FieldId::kUart0Config:
        case FieldId::kUart1Config:
        case FieldId::kUart2Config:
        case FieldId::kUart3Config:
        case FieldId::kUart7Config:
        case FieldId::kUart10Config: return true;
        default: return false;
        }
    }

    static std::size_t required_uart_config_size(FieldId field_id, uint32_t baudrate) noexcept {
        libhcs_VERIFY_LIKELY(is_uart_config_field_id(field_id), 0);
        libhcs_VERIFY_LIKELY(baudrate != 0, 0);

        // The payload's low nibble shares a byte with the field header's id, so
        // one FieldHeader worth of overlap comes back out of the total.
        const std::size_t total =
            required_field_header_size(field_id) + sizeof(UartConfigPayload) - sizeof(FieldHeader);
        utility::assert_debug(total <= kProtocolBufferSize);

        return total;
    }

    static std::size_t required_gpio_size(
        FieldId field_id, GpioHeader::PayloadEnum payload, bool timestamped = false) noexcept {
        const std::size_t field_header_bytes = required_field_header_size(field_id);
        const std::size_t gpio_header_bytes = sizeof(GpioHeader);
        std::size_t payload_bytes = 0;
        switch (payload) {
        case GpioHeader::PayloadEnum::kDigitalLow:
        case GpioHeader::PayloadEnum::kDigitalHigh:
            payload_bytes = timestamped ? sizeof(GpioDigitalReadTimestampPayload) : 0;
            break;
        case GpioHeader::PayloadEnum::kDigitalReadConfig:
        case GpioHeader::PayloadEnum::kAnalogReadConfig:
            payload_bytes = sizeof(GpioReadConfigPayload);
            break;
        case GpioHeader::PayloadEnum::kAnalog: payload_bytes = sizeof(GpioAnalogPayload); break;
        default: return 0;
        }

        const std::size_t total = (field_header_bytes + gpio_header_bytes - 1) + payload_bytes;
        utility::assert_debug(total <= kProtocolBufferSize);

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
