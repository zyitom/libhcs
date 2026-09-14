#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include <libhcs/spec/gpio.hpp>

namespace libhcs::data {

enum class DataId : uint8_t {
    kExtend = 0,

    kGpio = 1,

    // CAN field ids match each board's silkscreen number. hpm6e8y prints
    // CAN0..CAN3, so its four buses occupy kCan0..kCan3. 5321 / mc02 /
    // c_board / ch32_board print CAN1.. and stay on kCan1..
    kCan0 = 2,
    kCan1 = 3,
    kCan2 = 4,
    kCan3 = 5,

    // UART field ids match silkscreen numbers. HPM boards print UART0;
    // mc02 prints UART1/2/3/7/10 plus DBUS. Compact ids 6-12 are consecutive
    // so every UART stream uses a 1-byte field header.

    // Primary UART on the HPM boards, which number that port from 0.
    // Unused by mc02's ordinary ports (those are kUart1/2/3/7/10 plus DBUS);
    // mc02 diagnostic builds still emit on this id so they do not collide with
    // a silkscreen UART. Kept because the numeric value is wire format.
    kUart0 = 6,
    kUart1 = 7,
    kUart2 = 8,
    kUart3 = 9,
    kUart7 = 10,
    kUart10 = 11,

    kUartDbus = 12,
    kImu = 13,

    kSession = 14,

    // Downlink configuration channels. These ride the same byte stream as the
    // data fields above and are told apart by the field header alone; ids > 15
    // need the extended (2-byte) field header. Config traffic is rare (UART baud
    // and CAN mode now go over EP0), so the extra byte is acceptable.
    kCan0Config = 17,
    kCan1Config = 18,
    kCan2Config = 19,
    kCan3Config = 20,

    kUart0Config = 21,
    kUart1Config = 22,
    kUart2Config = 23,
    kUart3Config = 24,
    kUart7Config = 25,
    kUart10Config = 26,
    kUartDbusConfig = 27,
};

enum class SessionType : uint8_t {
    kStart = 0,
    kStartAck = 1,
    kKeepalive = 2,
    kKeepaliveAck = 3,

    // Shared time base (firmware/hpm_board/SOF_TIMEBASE.md). These ride the
    // session field rather than a channel of their own because the session field
    // is already the one periodic, low-rate, data-plane-independent exchange the
    // link has -- and because time sync must keep working exactly as long as the
    // session does, which is what sharing the nonce buys.
    //
    // Unlike the four types above, these carry a payload after the session
    // header, so a receiver that does not know them cannot skip them: it would
    // read the payload as the next field and lose framing. The host therefore
    // sends kTimeAnchor only to a board that was opened with time sync enabled.
    kTimeAnchor = 4,
    kTimeStatus = 5,

    // kSyncSample = 6 was the PTPC capture report, retired 2026-09-13 together
    // with the SOF->PTPC measurement path it served (SOF_TIMEBASE.md 5.4: the
    // capture-ownership premise does not hold). The number is deliberately not
    // reused; kPulseSchedule/Report keep their values for the test builds.

    // Hardware pulse exchange over the GPTMR compare/capture pins (see
    // firmware/hpm_board/app/src/sync/pulse.hpp). Host asks both boards to fire
    // at the SAME microframe; each reports where it captured the other's pulse.
    kPulseSchedule = 7,
    kPulseReport = 8,
};

// Wire-layout version of the session payloads that follow a SessionHeader.
// The EP0 fingerprint in libhcs/protocol/vendor_control.hpp folds this in, so
// a host and a board that disagree about the session payload layout fail the
// EP0 version check instead of corrupting stream framing.
//
// NOT hand-bumped: the value is the generated session_layout_fingerprint()
// from core/src/protocol/protocol.hpp, where a static_assert pins this
// constant to the computed value -- edit a session payload and the build
// fails until this constant is updated to the fingerprint it prints.
// History: 1 = pre-2026-09-13 (TimeStatus carried the PTPC diagnostics block,
// 78 B); 2 = PTPC block removed (30 B), kSyncSample retired.
inline constexpr uint16_t kSessionWireVersion = 20862; // session_layout_fingerprint(), 2026-09-14

enum class TimeState : uint8_t {
    // No usable timeline: nothing may be scheduled against it. This is the state
    // after boot, after a re-enumeration, and after any anomaly that could have
    // desynchronized the microframe counter.
    kInvalid = 0,
    // The microframe counter is running and the fit is converging, but the
    // absolute (wrap-resolved) origin is not known yet -- one kTimeAnchor
    // supplies it.
    kWaitingAnchor = 1,
    kValid = 2,
};

struct SessionControlView {
    SessionType type;
    uint32_t nonce;
};

// Host -> board. The host's estimate of the microframe number current at the
// moment the board receives this. Only the WRAP has to be right: the board keeps
// the low 14 bits from its own FRINDEX, which is hardware-exact and identical on
// every board of the same host controller, and uses this value solely to pick
// which multiple of 16384 those bits belong to. The estimate may therefore be
// off by up to +-1.024 s and still produce a bit-identical result on every
// board -- which is the entire reason the timestamp cannot degrade sync
// accuracy. The host must compute it ONCE per round and send the same value to
// every board.
struct TimeAnchorView {
    uint32_t nonce;
    uint64_t microframe;
};

// Host -> board: fire a hardware pulse at this absolute microframe. The host
// sends the identical value to every board, which is what makes the two-way
// difference below meaningful.
struct PulseScheduleView {
    uint32_t nonce;
    uint64_t microframe;
};

// Board -> host: one completed pulse exchange.
//
// Both boards fired at `scheduled`, so each board's capture offset from it
// carries the path delay plus the skew, with OPPOSITE sign for the skew:
//
//     a_offset = (d + skew) / 125us      b_offset = (d - skew) / 125us
//
// so skew = (a_offset - b_offset) / 2 and the path delay drops out. That is why
// the raw offsets are reported rather than a skew computed on the board -- no
// board can see both halves.
// Flags of PulseReportView. A board answers every schedule with a report, so
// silence means the link is broken rather than the pulse being missed.
enum PulseReportFlags : uint8_t {
    // The board accepted the target and wrote the comparator. Clear means the
    // target was unusable (no timeline yet, or too near/far), so no pulse was
    // emitted and nothing should be expected on the other board either.
    kPulseArmed = 1U << 0U,
    // This report carries a capture; without it the report is the immediate
    // acknowledgement of a schedule and captured_microframe_q16 is meaningless.
    kPulseCaptured = 1U << 1U,
};

struct PulseReportView {
    uint32_t nonce;
    uint64_t scheduled_microframe;
    // Where the incoming pulse landed on this board's axis, Q16 microframes.
    uint64_t captured_microframe_q16;
    // Measured GPTMR ticks per microframe, Q16. Exactly 3000<<16 = 196608000
    // confirms the crystal-derived clock tree; anything else invalidates the
    // exact-multiplication scheduling.
    uint32_t ticks_per_microframe_q16;
    // PulseReportFlags.
    uint8_t flags;
};

// Board -> host, answering a kTimeAnchor.
//
// 30 bytes on the wire since 2026-09-13: the PTPC diagnostics block (48 bytes
// of fields that only the retired SOF->PTPC measurement path consumed) moved
// out together with that path. Fits one 64-byte full-speed bulk packet.
struct TimeStatusView {
    uint32_t nonce;
    // Absolute microframe once anchored; the board's own origin before that.
    uint64_t microframe;
    // Local machine-timer reading paired with `microframe`, in quarter
    // microseconds. The pair is what lets the host fit its own clock to the
    // microframe axis.
    uint32_t timestamp_quarter_us;
    // Fitted local timer ticks per microframe, Q16. Nominally 500 * 65536 on a
    // 4 MHz timer; the deviation is the board crystal's offset from the host's
    // USB clock. Zero while the fit has not converged.
    uint32_t ticks_per_microframe_q16;
    TimeState state;
    // Free-running count of microframe deltas that were not exactly the
    // expected step. Any increase invalidates the timeline; the host watches it
    // to tell a clean re-anchor from a recurring hazard.
    uint32_t anomaly_count;

    // How far the board's own fit was wrong, measured against SOF samples the
    // fit had not seen yet, since the previous report. Q16 timer ticks; one tick
    // is 0.25 us.
    //
    // The MEAN is the number that matters. Interrupt entry jitter is zero-mean
    // and averages out of it, leaving the systematic error of the fitted line --
    // and since two boards run identical code on identical hardware, whatever is
    // common to both cancels, so the difference of their means is the expected
    // cross-board skew. The EXTREMUM is dominated by that same interrupt jitter
    // and therefore overstates the error of a scheduled action, which fires off
    // a timer comparison rather than out of the SOF handler.
    int32_t residual_mean_q16;
    uint32_t residual_abs_max_q16;
    uint16_t residual_count;
};

struct CanDataView {
    uint32_t can_id;
    std::span<const std::byte> can_data;
    // No frame-type flag here, by design: classic-vs-FD is a property of the BUS
    // the frame lives on, fixed by the firmware at init and read over the EP0
    // configuration channel (libhcs/protocol/vendor_control.hpp). The per-frame
    // header bit that used to carry it was retired 2026-09-12 -- see
    // CanHeaderLayout in core/src/protocol/protocol.hpp.
    bool is_extended_can_id = false;
    bool is_remote_transmission = false;
    // Hardware TSU timestamp in microseconds (1 tick = 1 us, wraps ~71.6 min).
    // std::nullopt if unsupported.
    std::optional<uint32_t> timestamp_us = std::nullopt;
};

struct UartDataView {
    std::span<const std::byte> uart_data;
    bool idle_delimited = false;
};

// Sparse patch: an unset field leaves the corresponding setting untouched. An
// entirely empty view is a deliberate no-op rather than an error.
struct UartConfigView {
    std::optional<uint32_t> baudrate = std::nullopt;
};

struct GpioDigitalDataView {
    bool high;
    std::optional<uint32_t> timestamp_quarter_us = std::nullopt;
};

struct GpioAnalogDataView {
    uint16_t value;
};

enum class GpioPull : uint8_t {
    kNone = 0,
    kUp = 1,
    kDown = 2,
};

struct GpioReadConfigView {
    uint16_t period_ms = 0;
    bool asap = false;
    bool rising_edge = false;
    bool falling_edge = false;
    bool capture_timestamp = false;
    GpioPull pull = GpioPull::kNone;

    [[nodiscard]] constexpr bool supported(const spec::GpioDescriptor& gpio) const noexcept {
        return (!asap || gpio.supports(spec::GpioCapability::kDigitalReadOnce))
            && (!period_ms || gpio.supports(spec::GpioCapability::kDigitalReadPeriodic))
            && ((!rising_edge && !falling_edge)
                || gpio.supports(spec::GpioCapability::kDigitalReadInterrupt))
            && (pull != GpioPull::kUp || gpio.supports(spec::GpioCapability::kPullUp))
            && (pull != GpioPull::kDown || gpio.supports(spec::GpioCapability::kPullDown))
            && (!capture_timestamp || gpio.supports(spec::GpioCapability::kTimestampedDigitalRead));
    }
};

struct ImuAccelerometerDataView {
    int16_t x;
    int16_t y;
    int16_t z;
    uint32_t timestamp_quarter_us;
};

struct ImuGyroscopeDataView {
    int16_t x;
    int16_t y;
    int16_t z;
    uint32_t timestamp_quarter_us;
};

struct ImuTemperatureDataView {
    uint16_t raw_register_value;
    uint32_t timestamp_quarter_us;
};

/**
 * @brief Interface for consuming deserialized uplink data.
 *
 * This interface is invoked after the protocol layer has already identified the payload type and
 * decoded its contents. For callback families that are further multiplexed by a sub-identifier,
 * such as `DataId` or a GPIO `channel_index`, the callback returns `bool` to report whether that
 * sub-identifier is valid for the concrete implementation.
 *
 * Return `true` when the sub-identifier is recognized and the payload has been dispatched.
 * Return `false` when deserialization succeeded but the `DataId` or `channel_index` is unexpected,
 * so the caller can propagate that routing error to upper layers.
 *
 * IMU callbacks return `void` because each payload type maps to a single callback and requires no
 * additional route validation.
 */
class DataCallback {
public:
    DataCallback() = default;
    DataCallback(const DataCallback&) = delete;
    DataCallback& operator=(const DataCallback&) = delete;
    DataCallback(DataCallback&&) = delete;
    DataCallback& operator=(DataCallback&&) = delete;
    virtual ~DataCallback() = default;

    [[nodiscard]] virtual bool can_receive_callback(DataId id, const CanDataView& data) = 0;

    [[nodiscard]] virtual bool uart_receive_callback(DataId id, const UartDataView& data) = 0;

    [[nodiscard]] virtual bool gpio_digital_read_result_callback(
        uint8_t channel_index, const GpioDigitalDataView& data) = 0;
    [[nodiscard]] virtual bool
        gpio_analog_read_result_callback(uint8_t channel_index, const GpioAnalogDataView& data) = 0;

    virtual void accelerometer_receive_callback(const ImuAccelerometerDataView& data) = 0;
    virtual void gyroscope_receive_callback(const ImuGyroscopeDataView& data) = 0;
    virtual void temperature_receive_callback(const ImuTemperatureDataView& data) = 0;

    // Shared time base status, one per keepalive period on a link with time sync
    // enabled. Non-pure and defaulted: every existing implementor predates it,
    // and an application that only wants a synchronized clock never has to see
    // the individual reports -- libhcs::host::time::timeline() is fed
    // regardless of whether this is overridden.
    virtual void time_status_callback(const TimeStatusView& data) { (void)data; }

    // One completed hardware pulse exchange. See PulseReportView.
    virtual void pulse_report_callback(const PulseReportView& data) { (void)data; }
};

} // namespace libhcs::data
