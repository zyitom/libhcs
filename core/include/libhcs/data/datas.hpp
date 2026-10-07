#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>

#include <libhcs/time/sof_stamp.hpp>

namespace libhcs::data {

enum class DataId : uint8_t {
    kExtend = 0,

    // Every id below is a port (EP0 addresses it, a manifest declares it) and the
    // field id of that port's records, except kSession (the link's own field).
    // Names come from the connectors: CAN/UART ids match the silkscreen number.

    // The board's GPIO port: a group of lines (the PWM header), declared one line
    // at a time. A line is not a port of its own -- like a motor id on a CAN bus,
    // it is an address within the port, carried as the line number in the record
    // header (GpioHeader in core/src/protocol/protocol.hpp) and in the high byte
    // of wIndex on EP0.
    kGpio = 1,

    // CAN field ids match each board's silkscreen number. hpm6e8y prints
    // CAN0..CAN3, so its four buses occupy kCan0..kCan3. 5321 / mc02 /
    // c_board print CAN1.. and stay on kCan1..
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

    // The on-board buzzer (mc02: a passive buzzer on a timer channel). Declared, it
    // starts silent; each downlink record sets the tone it plays until the next
    // one. The melody's timing is the host's. Every id now uses the compact field
    // header; kExtend stays as the escape for ids above 15.
    kBuzzer = 15,
};

static_assert(sizeof(DataId) == 1); // 类型层:底层类型必须还是单字节(uint8_t)

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

    // kStreamError = 9 was the board's downlink error report (v15), folded into
    // kPortStatus as the link's own status (data::LinkStatusView) in v16. The
    // number is not reused.

    // Board -> host: one port's runtime status, appended AFTER the
    // kKeepaliveAck (whose own format is untouched). Like the time-base types
    // above it carries a payload, so only a peer that agreed on this session
    // layout over EP0 can skip it. One record per running port whose
    // status changed since the previous report; the first round after a
    // kStart carries every running port, which is the host's baseline. The
    // payload is one byte -- the port's DataId (4 bits) and the length of what
    // follows (4 bits) -- then a body whose layout the port's kind defines
    // (data::PortStatusVariant): at most 16 bytes in all,
    // and a receiver can skip a body it does not know. This is the only
    // runtime status path: EP0 is the configuration plane and carries no
    // per-round traffic (kGetPortStatus was retired in v16).
    kPortStatus = 10,
};

// Why a downlink record did not reach a port (data::LinkStatusView carries the
// most recent one to the host). The two discard reasons lose the framing and
// take the rest of the USB transfer with them; kRefused is one skipped record.
enum class DownlinkError : uint8_t {
    // 格式坏: 记录截断、头部非法或保留编码 -- 已无法对齐下一条记录的边界, 本次
    // 传输剩余的全部字节只能丢弃。
    kMalformed = 0,
    // 字段号不认识: 记录的长度无从得知, 同样只能整批丢弃。
    kUnknownField = 1,
    // 字节完整但板子拒收(本板没有这个口、方向不符): 只丢这一条, 同一批里排在
    // 后面的记录照常处理。
    kRefused = 2,
};

constexpr const char* downlink_error_name(DownlinkError reason) noexcept {
    switch (reason) {
    case DownlinkError::kMalformed: return "malformed";
    case DownlinkError::kUnknownField: return "unknown field";
    case DownlinkError::kRefused: return "refused";
    }
    return "unknown reason";
}

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
// 78 B); 2 = PTPC block removed (30 B), kSyncSample retired; 20862 = first
// computed value (2026-09-14); 60804 = TimeStatus reports a fractional
// microframe and the hardware-capture counters (34 B); 18007 = kStreamError
// (10 B payload) answers the keepalive ack (2026-10-06); then the kPortStatus
// record = 11758 (2026-10-06, v16), which also took over kStreamError's report;
// 26076 = the UART status body gains `unattributed` (14 B, 2026-10-06, v17).
inline constexpr uint16_t kSessionWireVersion = 26076; // session_layout_fingerprint(), 2026-10-06

// PSR.LEC / PSR.DLEC coding, straight from the M_CAN register [RM]; bxCAN's
// ESR.LEC uses the same order.
enum class CanLastError : uint8_t {
    kNone = 0,
    kStuff = 1,
    kForm = 2,
    kAck = 3,
    kBit1 = 4,
    kBit0 = 5,
    kCrc = 6,
    kNoChange = 7, // no new error since the register was last read
};

constexpr const char* last_error_name(CanLastError code) noexcept {
    switch (code) {
    case CanLastError::kNone: return "none";
    case CanLastError::kStuff: return "STUFF";
    case CanLastError::kForm: return "FORM";
    case CanLastError::kAck: return "ACK";
    case CanLastError::kBit1: return "BIT1";
    case CanLastError::kBit0: return "BIT0";
    case CanLastError::kCrc: return "CRC";
    default: return "no-change";
    }
}

// An error code that names an actual bus error (kStuff..kCrc), as opposed to
// "none" or "nothing new since the last read".
constexpr bool is_bus_error(CanLastError code) noexcept {
    return code != CanLastError::kNone && code != CanLastError::kNoChange;
}

enum CanStateFlags : uint8_t {
    kCanErrorPassive = 1U << 0, // PSR.EP
    kCanWarning = 1U << 1,      // PSR.EW
    kCanBusOff = 1U << 2,       // PSR.BO
};

// One CAN controller's runtime status (data::SessionType::kPortStatus).
//
// Why a controller's error registers matter: when a bus delivers nothing,
// "the wire is bad" and "the firmware never transmitted" look identical from
// the host -- both are rx=0. The controller knows which it is. Read it in the
// order that narrows fastest:
//   last_error == kAck, tec climbing   transmitted, nobody acknowledged --
//                                      the far end is not listening
//   last_error == kBit0                drove dominant, read back recessive:
//                                      the bus cannot be pulled low at all
//   stuff / form / crc / bit1          bits arrive corrupted -- bit timing,
//                                      termination, or noise
//
// The board reads the registers once per keepalive round. PSR.LEC clears on
// read (and resets to kNone after a clean transfer), so the DRIVER latches
// last_error / data_last_error: the most recent real bus error (is_bus_error)
// since it started, kNone until there is one. A repeat of the same error does
// not change them -- TEC/REC do.
struct CanStatusView {
    // Which ports carry this status: the DataId numbering groups the kinds
    // (CAN ids 2..5), so a status record needs no kind byte of its own.
    static constexpr bool is_for(DataId id) noexcept {
        return id >= DataId::kCan0 && id <= DataId::kCan3;
    }

    uint8_t tec = 0; // ECR transmit error counter
    uint8_t rec = 0; // ECR receive error counter (0..127; the RP bit is not carried)
    CanLastError last_error = CanLastError::kNone;      // arbitration phase
    CanLastError data_last_error = CanLastError::kNone; // CAN-FD data phase
    uint8_t flags = 0;                                  // CanStateFlags
    // The counts below are free-running since boot and wrap at 2^16: the host
    // takes the difference of two consecutive snapshots modulo 2^16. A port
    // reports at least every round its counts move, and no count can move by
    // 65536 in one 250 ms round, so the difference is never ambiguous.
    //
    // Transmissions abandoned (single-shot: every lost arbitration or bus
    // error drops the frame; TEC/REC do not see lost arbitration). Every
    // board runs single-shot and counts it when a transmit slot (mailbox) is
    // reused, so the outcome of the last few transmissions (up to the FIFO /
    // mailbox depth) shows up one slot reuse later.
    uint16_t tx_cancelled = 0;
    // Frames the board itself dropped: downlink frames that found the transmit
    // queue full, and received frames that found the uplink buffer full. "The
    // bus had frames but the board lost them" must not look like "the bus
    // delivered nothing" -- the first fork of every dead-bus hunt.
    uint16_t tx_dropped = 0;
    uint16_t rx_dropped = 0;
    // Receive FIFO overflows: frames lost inside the controller before the
    // board could take them (M_CAN IR.RF0L, bxCAN RF0R.FOVR0). One count is at
    // least one lost frame -- the hardware flags the event, it does not count
    // frames. hpm_board / c_board count every interrupt that sees the flag;
    // mc02 checks it once a keepalive round.
    uint16_t rx_lost = 0;

    friend constexpr bool operator==(const CanStatusView&, const CanStatusView&) = default;
};

// One UART's runtime errors since boot (data::SessionType::kPortStatus).
// Free-running counts that wrap at 2^16, differenced like CanStatusView's. One receive-error
// count is one keepalive round in which that error was seen (each kind at most +1 a round), on
// every board but c_board, which counts one per HAL error callback. Zero means none happened.
// unattributed is the HPM UART's "an error happened but its kind is gone": in FIFO mode its
// LSR error bits describe only the byte at the RXFIFO head, and RX DMA takes that byte before
// the line-status interrupt can read them; a persistent error (wrong rate) still shows its kind
// in the once-a-round LSR read. noise is STM32 only (HPM has no noise flag).
struct UartStatusView {
    static constexpr bool is_for(DataId id) noexcept { // UART ids 6..12
        return id >= DataId::kUart0 && id <= DataId::kUartDbus;
    }

    uint16_t overrun = 0;      // a byte arrived before the previous one was taken
    uint16_t parity = 0;
    uint16_t framing = 0;      // includes a line break on HPM
    uint16_t noise = 0;
    uint16_t unattributed = 0; // a receive error of unknown kind (HPM only, see above)
    // Records the board itself dropped: downlink records that found the
    // transmit ring full, and received chunks that found the uplink buffer full.
    uint16_t tx_dropped = 0;
    uint16_t rx_dropped = 0;

    friend constexpr bool operator==(const UartStatusView&, const UartStatusView&) = default;
};

// The link's own status (DataId::kSession, the field the session records ride):
// the downlink records the board could not deliver. Why they are worth a
// report, and the two ways a record fails, are in core/DOWNLINK_ERRORS.md.
struct LinkStatusView {
    static constexpr bool is_for(DataId id) noexcept { return id == DataId::kSession; }

    // Downlink records the board could not deliver since boot, refused or
    // malformed alike. Free-running, wraps at 2^16 like every status count.
    uint16_t downlink_errors = 0;
    // The most recent one (meaningful once downlink_errors moved): which field
    // it was on, why it was dropped, and the ordinal (1-based, wraps at 2^16)
    // of the downlink transfer it arrived in, counted from the start of the
    // current session -- so the host can line it up with what it sent.
    DataId last_field = DataId::kExtend;
    DownlinkError last_reason = DownlinkError::kMalformed;
    uint16_t last_transfer = 0;

    friend constexpr bool operator==(const LinkStatusView&, const LinkStatusView&) = default;
};

// Every kind of port status there is -- THE list. Each port has at most one
// kind (its view's is_for() says which), so one variant per port holds it;
// std::monostate is "this port has no status". Everything that handles port
// status is generic over this list: the board's ledger, the record's decoder
// and its callback, the host's snapshot store and Handler::status<View>().
// Adding a kind is:
//   1. a view here, with is_for() and a defaulted operator==;
//   2. its alternative below;
//   3. a PortStatusRecord<View> specialization (body layout, core/src/protocol/protocol.hpp);
//   4. read_status() on the drivers of that kind (core/src/link/port_ops.hpp).
using PortStatusVariant =
    std::variant<std::monostate, LinkStatusView, CanStatusView, UartStatusView>;

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
    // Where the board was on the microframe axis when it wrote this report:
    // absolute once anchored, the board's own origin before that. 48 bits of
    // whole microframes on the wire.
    uint64_t microframe;
    // The part of a microframe past `microframe`, in 1/65536.
    //
    // This is what makes the report usable as a time: the host pairs the value
    // with the round trip that carried it, and a whole number alone is the
    // count at the last Start-of-Frame -- up to one microframe old, uniformly,
    // which is the entire margin the host has for telling its own counter's
    // integer offset from the next one over. Zero from a board whose fit has
    // not converged, where `microframe` is the latched count.
    uint16_t microframe_fraction_q16 = 0;
    // Local machine-timer reading paired with the position above, in quarter
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

    // Hardware SOF captures since the previous report: how many Start-of-Frame
    // interrupts found a fresh latch of the timestamp counter (and fed the
    // ring that places records on the axis, see libhcs/time/sof_stamp.hpp),
    // and how many found the latch older than half a microframe and skipped
    // it. Both zero on a board without the capture path. Their ratio is the
    // direct reading of how the trigger behaves on this silicon -- all fresh
    // means one latch per SOF; half stale means it latches every other one.
    // Saturating.
    uint16_t capture_fresh_count = 0;
    uint16_t capture_stale_count = 0;
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
    // When the frame started on the bus, on the shared USB microframe axis
    // (libhcs/time/sof_stamp.hpp). Latched by hardware at the frame's first
    // edge; uplink only.
    //
    // Empty whenever the board cannot place the frame on that axis: firmware
    // built without the time base, a board with no hardware capture path, or
    // the capture ring unable to bracket this particular frame. It is never a
    // reading of some board-local clock -- a consumer either gets a time it
    // can compare across boards and convert to host time, or nothing.
    std::optional<time::SofStamp> sof_stamp = std::nullopt;
};

struct UartDataView {
    std::span<const std::byte> uart_data;
    bool idle_delimited = false;
};

// One GPIO line's level. Downlink it is a write to an output line; uplink it is a
// sample of an input line, with the board's quarter-microsecond timestamp when
// the line was declared with kGpioInputTimestamp.
struct GpioDigitalDataView {
    bool high;
    std::optional<uint32_t> timestamp_quarter_us = std::nullopt;
};

// One GPIO line's analog value, 0..65535 of full scale. Downlink it is the PWM
// duty of an output line.
struct GpioAnalogDataView {
    uint16_t value;
};

// A tone for the buzzer, downlink: frequency in Hz and loudness 0..255 (255 is
// the loudest a passive buzzer gets, a 50% duty). Zero in either is silence.
struct BuzzerToneDataView {
    uint16_t frequency_hz;
    uint8_t loudness;
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

// The uplink callback interface lives on the host side: libhcs/data/callback.hpp.
// This header keeps the wire-layout views firmware and host share.

} // namespace libhcs::data
