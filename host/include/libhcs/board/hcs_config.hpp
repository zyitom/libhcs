#pragma once

#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <string>
#include <stdexcept>
#include <utility>

#include <libhcs/protocol/handler.hpp>
#include <libhcs/protocol/vendor_control.hpp>


// Construction-time channel configuration for the hpm_board family, over EP0.
//
// The rule these helpers exist to enforce: a constructed board object is a board
// whose configuration is KNOWN, not assumed. Every setting is written and then
// read back from the hardware before the constructor returns, and a disagreement
// throws rather than being logged -- a link running at a baudrate the caller did
// not ask for is indistinguishable, from the caller's side, from a wiring fault.
//
// This replaces two write-only paths. UART baudrate used to be an in-band
// kUart*Config field whose acceptance the board had no way to report, and the
// CAN frame type used to be a per-frame header bit that the board silently
// downgraded on a classic bus. Both are now settled once, here, with an answer.
namespace libhcs::board::hcs {

namespace vc = core::protocol::vendor_control;

// What the board says it has. Read once during construction; every other check
// in this header is against these numbers rather than against a constant, so a
// board variant that carries fewer channels than its image can serve (the
// single-CAN hpm5321 shares an image with the dual-CAN one) is handled by
// asking rather than by guessing.
//
// CAN bus numbering in this header's errors and parameters is the EP0 wire
// index (0-based, == wIndex on the wire); the exceptions below print it +1,
// which is the silkscreen number on every board whose CAN ports start at
// CAN1 (mc02, hpm5321). hpm6e8y silkscreens CAN0.. and its wrappers pass the
// raw index, so its messages read one higher than the silk -- cosmetic, and
// the board classes are the silk-aware layer.
struct Interface {
    uint8_t can_count = 0;
    uint8_t uart_count = 0;
    uint8_t can_fd_mask = 0;
    // Whether the board accepts host-driven TX frame type changes at
    // kSetCanConfig (see CanCapabilities::kCapCanModeSettable). False turns
    // every can_fd() entry below into a pure assertion.
    bool can_mode_settable = false;

    [[nodiscard]] bool can_fd(std::size_t bus) const {
        return bus < can_count && ((can_fd_mask >> bus) & 1U) != 0U;
    }
};

// Desired configuration, all optional. An unset field is left alone: the board
// keeps whatever its firmware brought the channel up with, which is the right
// default for a caller that does not care.
struct Configuration {
    // Baudrate per UART port, indexed by the board's own port numbering.
    std::optional<uint32_t> uart_baudrate[8];
    // CAN-FD mode per bus, as a REQUEST: on a board advertising
    // kCapCanModeSettable (mc02, where applying the mode is a per-Tx-element
    // flag switch) the bus is reconfigured and the mode read back before the
    // constructor returns; on a board without that capability (hpm_board,
    // where the mode is a controller-init property) it degrades to an
    // assertion and construction fails when the board disagrees. Either way a
    // constructed board runs the requested mode or the constructor threw.
    std::optional<bool> can_fd[8];
};

inline Interface read_interface(host::protocol::Handler& handler) {
    vc::InterfacePayload payload{};
    if (!handler.vendor_control_in(
            std::to_underlying(vc::Request::kGetInterface), 0, &payload, sizeof(payload))) {
        throw std::runtime_error{
            "Board rejected the EP0 interface query. Its firmware predates the EP0 configuration "
            "channel; reflash it, or open the board with an SDK of the matching version."};
    }
    if (payload.version != vc::kVersion) {
        throw std::runtime_error{std::format(
            "EP0 configuration version mismatch: board speaks v{}, this SDK speaks v{}. "
            "Reflash the board.",
            payload.version, vc::kVersion)};
    }
    return {
        .can_count = payload.can_count,
        .uart_count = payload.uart_count,
        .can_fd_mask = payload.can_fd_mask,
        .can_mode_settable = (payload.caps & vc::kCapCanModeSettable) != 0U,
    };
}

// Reads back what the port is REALLY running, reconstructed on the board from
// the divisor actually programmed. Never the value that was requested: that
// distinction is the entire point, because a rate the 80 MHz clock cannot
// represent leaves the divisor untouched and the port on its old rate.
// Human-readable name for a ConfigErrorReason, for logs and exceptions.
inline const char* config_error_name(uint8_t reason) {
    switch (static_cast<vc::ConfigErrorReason>(reason)) {
    case vc::ConfigErrorReason::kConfigErrorNone: return "none";
    case vc::ConfigErrorReason::kConfigErrorBadRequest: return "bad request";
    case vc::ConfigErrorReason::kConfigErrorBadIndex: return "no such channel";
    case vc::ConfigErrorReason::kConfigErrorUnsupportedMode: return "mode unsupported on that bus";
    case vc::ConfigErrorReason::kConfigErrorModeFixed: return "mode is fixed by the firmware";
    case vc::ConfigErrorReason::kConfigErrorRateUnrepresentable: return "rate unrepresentable";
    case vc::ConfigErrorReason::kConfigErrorFramingUnsupported: return "framing unsupported";
    default: return "unknown";
    }
}

// Reads WHY the board most recently STALLed a configuration request. Best
// effort: a transport-level failure while reading (device gone mid-error
// handling) surfaces as a zeroed latch -- the caller's original exception
// matters more than this read.
inline vc::LastConfigErrorPayload read_last_config_error(host::protocol::Handler& handler) {
    vc::LastConfigErrorPayload payload{};
    try {
        (void)handler.vendor_control_in(
            std::to_underlying(vc::Request::kGetLastConfigError), 0, &payload, sizeof(payload));
    } catch (const std::exception&) {
        payload = {};
    }
    return payload;
}

// Suffix for configuration-rejection exceptions: what the board latched. The
// latch is sticky, so a "none" here may mean the rejection predates this
// exchange's latch -- hence the soft wording.
inline std::string config_error_suffix(host::protocol::Handler& handler) {
    const auto latch = read_last_config_error(handler);
    if (latch.reason == std::to_underlying(vc::ConfigErrorReason::kConfigErrorNone))
        return " (the board latched no rejection reason)";
    return std::format(
        " (board latched: request 0x{:02x} index {} -- {})", latch.request, latch.index,
        config_error_name(latch.reason));
}

// One UART port's setting, in the EP0 payload's coding: 0 means "leave
// unchanged" on a request, and a read-back always fills every field with what
// the hardware actually runs (never a zero, never the last request).
struct UartSetting {
    uint32_t baudrate = 0;
    vc::UartWordLength word_length = static_cast<vc::UartWordLength>(0);
    vc::UartParity parity = static_cast<vc::UartParity>(0);
    vc::UartStopBits stop_bits = static_cast<vc::UartStopBits>(0);
};

// Reads what the port is REALLY running: the effective baudrate reconstructed
// on the board from the divisor actually programmed, plus the framing decoded
// from the live registers. Never the values that were requested -- that
// distinction is the entire point, because a rate the clock cannot represent
// leaves the divisor untouched and the port on its old setting.
inline UartSetting read_uart_setting(host::protocol::Handler& handler, std::size_t port) {
    vc::UartConfigPayload payload{};
    if (!handler.vendor_control_in(
            std::to_underlying(vc::Request::kGetUartConfig), static_cast<uint16_t>(port), &payload,
            sizeof(payload))) {
        throw std::runtime_error{
            std::format("Board rejected the EP0 configuration read-back for UART{}.", port)};
    }
    return {
        .baudrate = payload.baudrate,
        .word_length = static_cast<vc::UartWordLength>(payload.word_length),
        .parity = static_cast<vc::UartParity>(payload.parity),
        .stop_bits = static_cast<vc::UartStopBits>(payload.stop_bits),
    };
}

// Rate-only convenience over the full read-back, for board classes that expose
// just the baudrate.
inline uint32_t read_uart_baudrate(host::protocol::Handler& handler, std::size_t port) {
    return read_uart_setting(handler, port).baudrate;
}

// Applies a setting and confirms it landed. Every non-zero field is applied;
// zero fields leave the port untouched. Throws on rejection rather than
// returning a status, because every caller of this is a constructor or an
// explicit reconfiguration request: there is no sensible way to continue with
// a port running at a configuration nobody chose. The board guarantees a
// STALL leaves it exactly as it was (validate-then-commit on its side), and
// the read-back below is what turns that guarantee into knowledge.
//
// The rate tolerance is the board's own: its divisor solver accepts a rate
// within 3%, so 3000000 comes back as 3076923 and is correct. Comparing for
// equality here would reject a switch that actually worked.
inline void configure_uart(
    host::protocol::Handler& handler, std::size_t port, const UartSetting& setting) {
    const vc::UartConfigPayload payload{
        .baudrate = setting.baudrate,
        .word_length = std::to_underlying(setting.word_length),
        .parity = std::to_underlying(setting.parity),
        .stop_bits = std::to_underlying(setting.stop_bits),
        .control = vc::kUartConfigApply};
    if (!handler.vendor_control_out(
            std::to_underlying(vc::Request::kSetUartConfig), static_cast<uint16_t>(port), &payload,
            sizeof(payload))) {
        throw std::runtime_error{std::format(
            "Board rejected the UART{} configuration request: {} ({}). The port keeps its "
            "current configuration{}.",
            port,
            [&] {
                const auto latch = read_last_config_error(handler);
                return latch.reason == std::to_underlying(vc::ConfigErrorReason::kConfigErrorNone)
                         ? std::string{"reason not latched"}
                         : config_error_name(latch.reason);
            }(),
            static_cast<unsigned>(read_last_config_error(handler).request),
            config_error_suffix(handler))};
    }

    const UartSetting effective = read_uart_setting(handler, port);
    if (setting.baudrate != 0) {
        const uint64_t error = effective.baudrate > setting.baudrate
                                 ? effective.baudrate - setting.baudrate
                                 : setting.baudrate - effective.baudrate;
        if (error * 100U > static_cast<uint64_t>(setting.baudrate) * 5U) {
            throw std::runtime_error{std::format(
                "UART{} accepted {} baud but reads back {}. The board and the host disagree "
                "about what was programmed; do not trust this link.",
                port, setting.baudrate, effective.baudrate)};
        }
    }
    if (setting.word_length != static_cast<vc::UartWordLength>(0)
        && effective.word_length != setting.word_length) {
        throw std::runtime_error{std::format(
            "UART{}: word length was not applied (reads back {}).", port,
            std::to_underlying(effective.word_length))};
    }
    if (setting.parity != static_cast<vc::UartParity>(0) && effective.parity != setting.parity) {
        throw std::runtime_error{std::format(
            "UART{}: parity was not applied (reads back {}).", port,
            std::to_underlying(effective.parity))};
    }
    if (setting.stop_bits != static_cast<vc::UartStopBits>(0)
        && effective.stop_bits != setting.stop_bits) {
        throw std::runtime_error{std::format(
            "UART{}: stop bits were not applied (reads back {}).", port,
            std::to_underlying(effective.stop_bits))};
    }
}

// Rate-only form, the shape every board class exposed before framing joined
// the payload.
inline void configure_uart(host::protocol::Handler& handler, std::size_t port, uint32_t baudrate) {
    configure_uart(handler, port, UartSetting{.baudrate = baudrate});
}

// Reads one CAN bus's full timing identity as the board reported it: the TX
// mode in force, the rates and sample points the controller is actually timed
// for, and the capability bits. Read-only -- applying a mode goes through
// request_can_mode(), and the timing fields can never be applied (they are
// board-measured facts constrained by the bus peers' hardware).
inline vc::CanConfigPayload read_can_config(host::protocol::Handler& handler, std::size_t bus) {
    vc::CanConfigPayload payload{};
    if (!handler.vendor_control_in(
            std::to_underlying(vc::Request::kGetCanConfig), static_cast<uint16_t>(bus), &payload,
            sizeof(payload))) {
        throw std::runtime_error{
            std::format("Board rejected the EP0 configuration read-back for CAN{}.", bus + 1)};
    }
    return payload;
}

// Requests a bus's TX frame type and confirms the board agrees. The exchange is
// deliberately GET -> SET -> GET:
//
//   1. GET captures the bus's current timing identity (rates, sample points).
//   2. The SET asks for `want_fd` AND echoes that captured timing back in the
//      assertion fields. This is what makes it impossible to forget the data
//      phase: the caller only says want_fd, and the SDK asserts the whole
//      electrical identity along with the mode -- a mode switch that somehow
//      disturbed the timing (no re-init is involved, so it cannot) would be
//      caught instead of silently accepted. A caller working with raw
//      Handler::vendor_control calls can still skip the timing fields (0 =
//      skip); the helper exists so nobody HAS to remember.
//   3. The final GET re-reads the mode, so the decision rests on what the
//      board says it runs, not on the absence of a stall.
//
// The `settable` flag (the board's own capability report) selects between the
// two failure stories:
//   settable    the board applies the mode; a stall or a disagreeing read-back
//               means the board refused, and the caller's expectation is wrong
//               for a bus someone else may have configured.
//   !settable   the mode is a compile-time property of the firmware's port
//               table (hpm_board: a controller initialized without CAN-FD
//               cannot receive FD frames at all -- measured 0/50 -- so mode
//               application is deliberately not offered). A mismatch is the
//               caller's expectation being wrong, and it is fatal here rather
//               than silently producing frames of the other type on the wire.
inline void request_can_mode(
    host::protocol::Handler& handler, std::size_t bus, bool want_fd, bool settable) {
    const vc::CanConfigPayload before = read_can_config(handler, bus);
    const vc::CanConfigPayload payload{
        .mode = std::to_underlying(want_fd ? vc::CanMode::kCanFd : vc::CanMode::kClassic),
        .control = vc::kCanConfigApply,
        .reserved0 = 0,
        .arbitration_baudrate = before.arbitration_baudrate,
        .data_baudrate = before.data_baudrate,
        .nominal_sample_point = before.nominal_sample_point,
        .data_sample_point = before.data_sample_point,
        .reserved1 = 0};
    if (!handler.vendor_control_out(
            std::to_underlying(vc::Request::kSetCanConfig), static_cast<uint16_t>(bus), &payload,
            sizeof(payload))) {
        if (settable)
            throw std::runtime_error{std::format(
                "CAN{}: the board refused the requested {} ({}). The bus keeps its current mode.",
                bus + 1, want_fd ? "CAN-FD" : "classic CAN",
                config_error_name(read_last_config_error(handler).reason))};
        throw std::runtime_error{std::format(
            "CAN{} runs {} on this board, but was opened expecting {}. The bus mode is fixed "
            "by the firmware's port table; change the expectation or the firmware.",
            bus + 1, want_fd ? "classic CAN" : "CAN-FD", want_fd ? "CAN-FD" : "classic CAN")};
    }

    const vc::CanConfigPayload after = read_can_config(handler, bus);
    if (after.mode != payload.mode) {
        const char* actual_mode =
            after.mode == std::to_underlying(vc::CanMode::kCanFd) ? "CAN-FD" : "classic CAN";
        throw std::runtime_error{std::format(
            "CAN{} was asked to run {} but reads back {}. The board and the host disagree "
            "about what was configured; do not trust this bus.",
            bus + 1, want_fd ? "CAN-FD" : "classic CAN", actual_mode)};
    }
    // The timing fields were echoed from the board itself, so any drift here
    // means the mode switch disturbed the bit timing -- impossible by
    // construction (mode application never re-initializes the controller), and
    // cheap to prove rather than assume.
    if (after.arbitration_baudrate != before.arbitration_baudrate
        || after.data_baudrate != before.data_baudrate
        || after.nominal_sample_point != before.nominal_sample_point
        || after.data_sample_point != before.data_sample_point) {
        throw std::runtime_error{std::format(
            "CAN{}: the mode switch changed the reported timing ({} -> {} baud, {} -> {} per "
            "mille). The controller was re-initialized behind this exchange; do not trust "
            "this bus.",
            bus + 1, before.arbitration_baudrate, after.arbitration_baudrate,
            before.nominal_sample_point, after.nominal_sample_point)};
    }
}

// Reads a CAN controller's own error state. Available on the shipping image --
// no diagnostic build, and it does not touch the kUart0 telemetry channel that
// the CAN_DIAG record rides.
//
// Interpreting the result, in the order that narrows fastest:
//   last_error == kAck, tec high         transmitted, nobody acknowledged: the
//                                        far end is not listening
//   last_error == kBit0                  the bus cannot be pulled low at all
//   stuff / form / crc                   bits arrive corrupted
//   tx_occurred == 0, tx_cancelled != 0  nothing ever reached the wire
inline vc::CanStatusPayload read_can_status(host::protocol::Handler& handler, std::size_t bus) {
    vc::CanStatusPayload payload{};
    if (!handler.vendor_control_in(
            std::to_underlying(vc::Request::kGetCanStatus), static_cast<uint16_t>(bus), &payload,
            sizeof(payload))) {
        throw std::runtime_error{
            std::format("Board rejected the EP0 status query for CAN{}.", bus + 1)};
    }
    return payload;
}

inline const char* last_error_name(uint8_t code) {
    switch (static_cast<vc::LastErrorCode>(code)) {
    case vc::LastErrorCode::kNone: return "none";
    case vc::LastErrorCode::kStuff: return "STUFF";
    case vc::LastErrorCode::kForm: return "FORM";
    case vc::LastErrorCode::kAck: return "ACK";
    case vc::LastErrorCode::kBit1: return "BIT1";
    case vc::LastErrorCode::kBit0: return "BIT0";
    case vc::LastErrorCode::kCrc: return "CRC";
    default: return "no-change";
    }
}

// Reads how much of a CAN round trip the board itself accounts for. Available
// on the shipping image; pass reset=true to clear the accumulators after
// reading, so a caller can bracket a measurement.
inline vc::LatencyBreakdownPayload
    read_latency_breakdown(host::protocol::Handler& handler, bool reset) {
    vc::LatencyBreakdownPayload payload{};
    if (!handler.vendor_control_in(
            std::to_underlying(vc::Request::kGetLatencyBreakdown), reset ? 1 : 0, &payload,
            sizeof(payload))) {
        throw std::runtime_error{"Board rejected the EP0 latency-breakdown query."};
    }
    return payload;
}

// One place for the whole construction-time exchange, so every board class in
// this directory performs it identically and in the same order: learn what the
// board is, assert the CAN modes, then apply the UART rates.
inline Interface apply(host::protocol::Handler& handler, const Configuration& configuration) {
    const Interface interface = read_interface(handler);

    for (std::size_t bus = 0; bus < std::size(configuration.can_fd); ++bus) {
        if (!configuration.can_fd[bus].has_value())
            continue;
        if (bus >= interface.can_count) {
            throw std::runtime_error{std::format(
                "CAN{} was configured but this board reports only {} CAN bus(es).", bus + 1,
                interface.can_count)};
        }
        request_can_mode(handler, bus, *configuration.can_fd[bus], interface.can_mode_settable);
    }

    for (std::size_t port = 0; port < std::size(configuration.uart_baudrate); ++port) {
        if (!configuration.uart_baudrate[port].has_value())
            continue;
        if (port >= interface.uart_count) {
            throw std::runtime_error{std::format(
                "UART{} was configured but this board reports only {} UART port(s).", port,
                interface.uart_count)};
        }
        configure_uart(handler, port, *configuration.uart_baudrate[port]);
    }

    return interface;
}

} // namespace libhcs::board::hcs
