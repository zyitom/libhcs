#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <format>
#include <optional>
#include <stdexcept>

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
struct Interface {
    uint8_t can_count = 0;
    uint8_t uart_count = 0;
    uint8_t can_fd_mask = 0;

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
    // Expected CAN-FD mode per bus. The board's mode is fixed by its firmware,
    // so this is an assertion, not a request -- construction fails when the
    // board disagrees, which is the check that used to be impossible.
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
        throw std::runtime_error{
            std::format(
                "EP0 configuration version mismatch: board speaks v{}, this SDK speaks v{}. "
                "Reflash the board.",
                payload.version, vc::kVersion)};
    }
    return {
        .can_count = payload.can_count,
        .uart_count = payload.uart_count,
        .can_fd_mask = payload.can_fd_mask,
    };
}

// Reads back what the port is REALLY running, reconstructed on the board from
// the divisor actually programmed. Never the value that was requested: that
// distinction is the entire point, because a rate the 80 MHz clock cannot
// represent leaves the divisor untouched and the port on its old rate.
inline uint32_t read_uart_baudrate(host::protocol::Handler& handler, std::size_t port) {
    vc::UartConfigPayload payload{};
    if (!handler.vendor_control_in(
            std::to_underlying(vc::Request::kGetUartConfig), static_cast<uint16_t>(port),
            &payload, sizeof(payload))) {
        throw std::runtime_error{
            std::format("Board rejected the EP0 baudrate read-back for UART{}.", port)};
    }
    return payload.baudrate;
}

// Applies a baudrate and confirms it landed. Throws on rejection rather than
// returning a status, because every caller of this is a constructor or an
// explicit reconfiguration request: there is no sensible way to continue with a
// port running at a rate nobody chose.
//
// The tolerance is the board's own: its divisor solver accepts a rate within 3%,
// so 3000000 comes back as 3076923 and is correct. Comparing for equality here
// would reject a switch that actually worked.
inline void configure_uart(
    host::protocol::Handler& handler, std::size_t port, uint32_t baudrate) {
    const vc::UartConfigPayload payload{.baudrate = baudrate, .reserved = 0};
    if (!handler.vendor_control_out(
            std::to_underlying(vc::Request::kSetUartConfig), static_cast<uint16_t>(port),
            &payload, sizeof(payload))) {
        throw std::runtime_error{
            std::format(
                "Board rejected {} baud on UART{}: its divisor solver found no setting within "
                "3% of that rate. The port is untouched and still running at {} baud.",
                baudrate, port, read_uart_baudrate(handler, port))};
    }

    const uint32_t effective = read_uart_baudrate(handler, port);
    const uint64_t error = effective > baudrate ? effective - baudrate : baudrate - effective;
    if (error * 100U > static_cast<uint64_t>(baudrate) * 5U) {
        throw std::runtime_error{
            std::format(
                "UART{} accepted {} baud but reads back {} baud. The board and the host disagree "
                "about what was programmed; do not trust this link.",
                port, baudrate, effective)};
    }
}

// Confirms the board agrees about a bus's frame type. The board never
// reconfigures its controller for this -- bringing an MCAN up carries a pinned
// sample point, transmitter delay compensation and an external PTPC timebase
// feeding the timestamp unit, none of which survive a casual re-init -- so a
// mismatch is the caller's expectation being wrong, and it is fatal here rather
// than silently producing frames of the other type on the wire.
inline void verify_can_mode(host::protocol::Handler& handler, std::size_t bus, bool expect_fd) {
    const vc::CanConfigPayload payload{
        .mode = std::to_underlying(expect_fd ? vc::CanMode::kCanFd : vc::CanMode::kClassic),
        .reserved = {}};
    if (!handler.vendor_control_out(
            std::to_underlying(vc::Request::kSetCanConfig), static_cast<uint16_t>(bus), &payload,
            sizeof(payload))) {
        throw std::runtime_error{
            std::format(
                "CAN{} runs {} on this board, but was opened expecting {}. The bus mode is fixed "
                "by the firmware's port table; change the expectation or the firmware.",
                bus, expect_fd ? "classic CAN" : "CAN-FD", expect_fd ? "CAN-FD" : "classic CAN")};
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
inline vc::LatencyBreakdownPayload read_latency_breakdown(
    host::protocol::Handler& handler, bool reset) {
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
            throw std::runtime_error{
                std::format(
                    "CAN{} was configured but this board reports only {} CAN bus(es).", bus,
                    interface.can_count)};
        }
        verify_can_mode(handler, bus, *configuration.can_fd[bus]);
    }

    for (std::size_t port = 0; port < std::size(configuration.uart_baudrate); ++port) {
        if (!configuration.uart_baudrate[port].has_value())
            continue;
        if (port >= interface.uart_count) {
            throw std::runtime_error{
                std::format(
                    "UART{} was configured but this board reports only {} UART port(s).", port,
                    interface.uart_count)};
        }
        configure_uart(handler, port, *configuration.uart_baudrate[port]);
    }

    return interface;
}

} // namespace libhcs::board::hcs
