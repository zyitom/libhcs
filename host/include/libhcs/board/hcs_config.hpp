#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <libhcs/data/datas.hpp>
#include <libhcs/protocol/handler.hpp>
#include <libhcs/protocol/vendor_control.hpp>
#include <libhcs/spec/port.hpp>

// Construction-time port declaration for every board with an EP0 configuration
// channel.
//
// The rule these helpers exist to enforce: a constructed board object is a board
// whose configuration is KNOWN, not assumed. The host declares the ports it will
// use -- and only those -- in one kApplyManifest; the board validates the whole
// declaration (no hardware touched), applies it, verifies each port by read-back
// and reports per-port results. A fully applied manifest starts exactly the
// declared ports, with every other port off, and moves board ownership to libhcs;
// the constructor returns only then, so "constructed" still means "verified" --
// a link running at a baudrate the caller did not ask for is indistinguishable,
// from the caller's side, from a wiring fault, and a disagreement throws rather
// than being logged.
//
// Ports are addressed by DataId -- the same identity the bulk stream uses and
// the number the connector prints (CAN1 is DataId::kCan1). Nothing in this
// header, in the wire format, or in an error message deals in positions: there
// is no "bus #1" that means CAN2 on one board and CAN1 on another.
namespace libhcs::board::hcs {

namespace vc = core::protocol::vendor_control;
using spec::channel_name;

// What every helper in this header needs from the thing it talks through: the
// two EP0 vendor transfers, each answering "did the board ACK". The real one is
// host::protocol::Handler; naming only this much is what lets the whole
// declaration below run against a simulated board in host/tests, with no USB.
template <class T>
concept Ep0Channel = requires(
    T& channel, uint8_t request, uint16_t index, void* in, const void* out, std::size_t size) {
    { channel.vendor_control_in(request, index, in, size) } -> std::convertible_to<bool>;
    { channel.vendor_control_out(request, index, out, size) } -> std::convertible_to<bool>;
};
static_assert(Ep0Channel<host::protocol::Handler>);

// The link's state as the transmit path sees it: which ports this host declared
// (the board runs exactly those, everything else is off), which lines of the
// GPIO port were declared and which of them as outputs, which CAN buses are
// running CAN-FD right now, and which carry CAN-FD long frames at all -- the
// last one a capability the BOARD reported in its port list, not something the
// host assumed. The port masks are bit-per-DataId (the port's IDENTITY is its
// bit, so there is no per-board numbering to keep in step); the GPIO masks are
// bit-per-line.
//
// The board classes keep the learned snapshot in a std::atomic: the before-
// session hook that writes it re-runs on every reconnect from the keepalive
// thread, while application transmit paths read it concurrently. An oversized or
// non-lock-free layout would put an implicit lock on the transmit path -- refuse
// to compile instead. The masks fit one 64-bit word because each is only as
// wide as the ids it can hold: every DataId is below 16, every CAN id below 8
// and every GPIO line below 8.
struct Interface {
    std::uint64_t declared_mask   : 16 = 0;
    std::uint64_t fd_mask         : 8 = 0; // CAN buses
    std::uint64_t long_frame_mask : 8 = 0; // CAN buses
    std::uint64_t line_mask       : 8 = 0; // GPIO lines declared
    std::uint64_t output_mask     : 8 = 0; // GPIO lines declared as outputs

    [[nodiscard]] static constexpr bool has(std::uint64_t mask, data::DataId port) noexcept {
        return ((mask >> std::to_underlying(port)) & 1U) != 0U;
    }
    [[nodiscard]] static constexpr std::uint64_t
        with(std::uint64_t mask, data::DataId port, bool on) noexcept {
        const std::uint64_t bit = std::uint64_t{1} << std::to_underlying(port);
        return on ? mask | bit : mask & ~bit;
    }

    [[nodiscard]] constexpr bool declared(data::DataId port) const noexcept {
        return has(declared_mask, port);
    }
    // The GPIO line was declared, and declared as an output (vc::kGpioModeOutput).
    [[nodiscard]] constexpr bool line_declared(std::uint8_t line) const noexcept {
        return line < 8U && ((line_mask >> line) & 1U) != 0U;
    }
    [[nodiscard]] constexpr bool output(std::uint8_t line) const noexcept {
        return line < 8U && ((output_mask >> line) & 1U) != 0U;
    }
    [[nodiscard]] constexpr bool can_fd(data::DataId port) const noexcept {
        return has(fd_mask, port);
    }
    // The board reported this CAN port as carrying 12-64 byte payloads
    // (spec::kCanCapFdLongFrames in its port list).
    [[nodiscard]] constexpr bool long_frames(data::DataId port) const noexcept {
        return has(long_frame_mask, port);
    }

    // The board accepted this port's declaration.
    constexpr void declare(data::DataId port) noexcept {
        declared_mask = with(declared_mask, port, true);
    }
    constexpr void declare_line(std::uint8_t line, bool output) noexcept {
        declare(data::DataId::kGpio);
        const std::uint64_t bit = std::uint64_t{1} << line;
        line_mask |= bit;
        output_mask = output ? (output_mask | bit) : (output_mask & ~bit);
    }
    constexpr void set_can_fd(data::DataId port, bool fd) noexcept {
        fd_mask = with(fd_mask, port, fd);
    }
    constexpr void set_long_frames(data::DataId port, bool capable) noexcept {
        long_frame_mask = with(long_frame_mask, port, capable);
    }
};
static_assert(sizeof(Interface) == 8);
static_assert(std::atomic<Interface>::is_always_lock_free);
// Every identity fits its mask; with() relies on it (a wider id would be cut off
// silently by the bit-field).
static_assert(std::to_underlying(data::DataId::kBuzzer) < 16U);
static_assert(std::to_underlying(data::DataId::kCan3) < 8U);
static_assert(spec::kMaxGpioLines <= 8U);

// One UART port's setting, in the EP0 payload's coding. A declaration states the
// whole of it -- baudrate, word_length, parity, stop_bits and rx_polarity; a zero
// in any of them is refused before anything is sent (and by the board too),
// because "whatever the firmware has" may be a placeholder or a CDC rewrite the
// host never sees. A read-back always fills every field with what the hardware
// actually runs (never a zero, never the last request).
//
// `divisor`/`oversample` are the port's rate in its integer form, and they are
// what a rate agreement is actually checked against -- see vc::UartDivisor for
// why the baudrate number cannot be. On a declaration they are an ASSERTION, not
// an instruction: leave them zero and the board only checks that it programmed
// something; set them (from a prior read_uart_setting) and the board must land on
// exactly those integers.
struct UartSetting {
    uint32_t baudrate = 0;
    uint16_t divisor = 0;
    uint8_t oversample = 0;
    vc::UartWordLength word_length = static_cast<vc::UartWordLength>(0);
    vc::UartParity parity = static_cast<vc::UartParity>(0);
    vc::UartStopBits stop_bits = static_cast<vc::UartStopBits>(0);
    // Polarity at the MCU pin (see vc::UartRxPolarity). mc02 only: on its DBUS
    // port, kUartRxPolarityInverted is what lets an iBUS receiver through the
    // on-board inverter; every other board refuses it.
    vc::UartRxPolarity rx_polarity = vc::kUartRxPolaritySkip;

    friend constexpr bool operator==(const UartSetting&, const UartSetting&) = default;
};

// The common case written out: 8 data bits, no parity, 1 stop bit, RX not
// inverted, at `baudrate`.
[[nodiscard]] constexpr UartSetting uart_8n1(uint32_t baudrate) noexcept {
    return {
        .baudrate = baudrate,
        .word_length = vc::kUartWordLength8,
        .parity = vc::kUartParityNone,
        .stop_bits = vc::kUartStopBits1,
        .rx_polarity = vc::kUartRxPolarityNormal,
    };
}

// 一路 CAN 总线的设置(vc::CanSetting): 帧型与两段速率, 接线事实, 写死在 host 代码里经 EP0
// 清单声明下发。速率可设的板(hpm_board)按它解位时序, 解不出即构造失败; 速率写死的板
// (mc02、c_board)把非零速率当核对, 不符即构造失败。
using CanSetting = vc::CanSetting;

// 本项目实际存在的两种总线。
inline constexpr CanSetting kClassic1M{.fd = false, .arbitration_baudrate = 1'000'000};
inline constexpr CanSetting kFd1M5M{
    .fd = true, .arbitration_baudrate = 1'000'000, .data_baudrate = 5'000'000};

// One GPIO line's setting, in the EP0 payload's coding (vc::GpioConfigPayload).
// The direction is the declaration: an output takes level and PWM-duty writes
// and starts low; an input is sampled on request (PacketBuilder::gpio_read),
// every `period_ms`, and/or on the edges in input_flags, as the line's
// capabilities allow -- a sampling the line cannot do is refused at
// construction, with the line's name.
struct GpioSetting {
    vc::GpioMode mode = vc::kGpioModeOutput;
    vc::GpioPull pull = vc::kGpioPullNone;
    uint8_t input_flags = 0; // vc::GpioInputFlags
    uint16_t period_ms = 0;

    friend constexpr bool operator==(const GpioSetting&, const GpioSetting&) = default;
};

inline constexpr GpioSetting kGpioOutput{};

// The on-board IMU's setting, in the EP0 payload's coding (vc::ImuConfigPayload):
// physical units, every field required. It is stated here, by the host, because
// the host is the one that scales the raw samples -- the ranges are its numbers,
// and a declared IMU runs exactly these or construction throws.
struct ImuSetting {
    uint8_t accelerometer_range_g = 0;
    uint16_t accelerometer_rate_hz = 0;
    uint16_t gyroscope_range_dps = 0;
    uint16_t gyroscope_rate_hz = 0;
    uint16_t gyroscope_bandwidth_hz = 0;

    friend constexpr bool operator==(const ImuSetting&, const ImuSetting&) = default;
};

// The ports this host uses, and how each one is set.
//
// CONFIGURATION IS DECLARATION, ONE TRANSACTION (core vendor_control.hpp has the
// wire side): a port with an entry here is named in the kApplyManifest and thereby
// started; a port with no entry is one this host does not use, and the board keeps
// it off for the whole session -- a CAN controller that is not on the bus, a UART
// that forwards nothing. There is no "works with whatever the firmware has"
// default: an unused connector must not be able to inject events, and a host that
// forgot a port now hears silence on it instead of data it never asked for.
// Transmitting on an undeclared port throws UndeclaredChannel from the board
// class (see require_declared()); a host that bypasses the board classes is still
// stopped by the board, which drops the data.
//
// The declaration is KEYED BY DataId, not by position: there is no can[8]/uart[8]
// whose slot i means "the i-th bus of whatever board this turns out to be". The
// container is a fixed-capacity array (the largest board's port count) so a
// Configuration is trivially copyable, has no heap, and its size is a compile-
// time fact -- the board classes copy it for their reconnect hook.
struct Configuration {
    struct Entry {
        data::DataId data_id = data::DataId::kExtend; // kExtend = empty slot
        spec::PortKind kind = spec::PortKind::kCan;
        std::uint8_t line = 0; // the GPIO port's line; 0 for every other kind
        // The setting of the port's kind; the others are ignored. Kept side by
        // side (not a union) so Entry stays trivially copyable and each kind's
        // declare_* is the only way to write its own field.
        CanSetting can{};
        UartSetting uart{};
        ImuSetting imu{};
        GpioSetting gpio{};
    };

    static constexpr std::size_t kCapacity = spec::kMaxManifestEntries;

    std::array<Entry, kCapacity> entries{};
    std::size_t count = 0;

    // The shared USB-SOF time base: CAN frames carry a SofStamp on the shared
    // microframe axis, and the link exchanges kTimeAnchor / kTimeStatus each
    // keepalive (host/src/time/usb_frame_axis.hpp). Not a port: it rides the manifest
    // header (vc::kManifestFlagTimeSync) and is refused before anything is sent
    // when the board does not report vc::kBoardCapTimeSync. Off, the board does
    // not even take the SOF interrupt.
    bool time_sync = false;

    constexpr void enable_time_sync(bool on = true) noexcept { time_sync = on; }

    [[nodiscard]] constexpr const Entry*
        find(data::DataId port, std::uint8_t line = 0) const noexcept {
        for (std::size_t i = 0; i < count; ++i)
            if (entries[i].data_id == port && entries[i].line == line)
                return &entries[i];
        return nullptr;
    }

    constexpr Entry& slot_for(data::DataId port, std::uint8_t line = 0) {
        if (auto* existing = const_cast<Configuration*>(this)->find_mutable(port, line))
            return *existing;
        if (count >= kCapacity)
            throw std::invalid_argument{std::format("more than {} declarations", kCapacity)};
        return entries[count++];
    }

    // Declares one CAN bus: its frame type and rates are applied over EP0 and
    // verified by the board's own read-back before the constructor returns.
    constexpr void declare_can(data::DataId port, const CanSetting& setting) {
        Entry& entry = slot_for(port);
        entry.data_id = port;
        entry.kind = spec::PortKind::kCan;
        entry.can = setting;
    }

    // Declares one UART port. The setting must be complete (see UartSetting);
    // apply() refuses one that is not, naming the missing fields.
    constexpr void declare_uart(data::DataId port, const UartSetting& setting) {
        Entry& entry = slot_for(port);
        entry.data_id = port;
        entry.kind = spec::PortKind::kUart;
        entry.uart = setting;
    }

    // Declares the on-board IMU (DataId::kImu).
    constexpr void declare_imu(const ImuSetting& setting) {
        Entry& entry = slot_for(data::DataId::kImu);
        entry.data_id = data::DataId::kImu;
        entry.kind = spec::PortKind::kImu;
        entry.imu = setting;
    }

    // Declares one line of the GPIO port (DataId::kGpio) as an output or an input,
    // by number or by the spec's name for it (Spec::kGpios.kPwm3).
    constexpr void declare_gpio(std::uint8_t line, const GpioSetting& setting) {
        Entry& entry = slot_for(data::DataId::kGpio, line);
        entry.data_id = data::DataId::kGpio;
        entry.kind = spec::PortKind::kGpio;
        entry.line = line;
        entry.gpio = setting;
    }
    template <typename Tag>
    constexpr void declare_gpio(spec::GpioLineDescriptor<Tag> line, const GpioSetting& setting) {
        declare_gpio(line.line, setting);
    }

    // Declares the on-board buzzer (DataId::kBuzzer). It has no setting: declared,
    // it starts silent and plays the tones the host sends.
    constexpr void declare_buzzer() {
        Entry& entry = slot_for(data::DataId::kBuzzer);
        entry.data_id = data::DataId::kBuzzer;
        entry.kind = spec::PortKind::kBuzzer;
    }

private:
    [[nodiscard]] constexpr Entry* find_mutable(data::DataId port, std::uint8_t line) noexcept {
        for (std::size_t i = 0; i < count; ++i)
            if (entries[i].data_id == port && entries[i].line == line)
                return &entries[i];
        return nullptr;
    }
};

// What the board says it has: one row per PRESENT port, from kGetPortList. The
// spec describes the image's capacity; this is the runtime truth (the single-CAN
// hpm5321 reports one bus), read before declaring so a wiring that names a port
// the PCB lacks fails with the port's name instead of a board-side stall.
struct PortList {
    struct Port {
        data::DataId data_id;
        spec::PortKind kind;
        uint8_t capabilities;
        uint8_t status; // spec::PortStatusFlags
    };

    std::array<Port, spec::kMaxPorts> ports{};
    std::size_t count = 0;
    std::uint8_t board_caps = 0; // vc::BoardCapability

    [[nodiscard]] constexpr const Port* find(data::DataId id) const noexcept {
        for (std::size_t i = 0; i < count; ++i)
            if (ports[i].data_id == id)
                return &ports[i];
        return nullptr;
    }

    [[nodiscard]] constexpr bool running(data::DataId id) const noexcept {
        const auto* port = find(id);
        return port != nullptr && (port->status & spec::kPortRunning) != 0U;
    }
};

// Reads the port list. PURE READ on the board: no ownership change, no port
// stops -- the declaration round starts at kApplyManifest, not here.
inline PortList read_port_list(Ep0Channel auto& handler) {
    vc::PortListPayload payload{};
    if (!handler.vendor_control_in(
            std::to_underlying(vc::Request::kGetPortList), 0, &payload, sizeof(payload))) {
        throw std::runtime_error{
            "Board rejected the EP0 port-list query. Its firmware predates the EP0 configuration "
            "channel; reflash it, or open the board with an SDK of the matching version."};
    }
    if (payload.version != vc::kVersion) {
        throw std::runtime_error{std::format(
            "EP0 configuration version mismatch: board speaks v{}, this SDK speaks v{}. "
            "Reflash the board.",
            payload.version, vc::kVersion)};
    }
    PortList list;
    for (std::size_t i = 0; i < payload.port_count && i < std::size(list.ports); ++i) {
        list.ports[i] = {
            .data_id = static_cast<data::DataId>(payload.ports[i].data_id),
            .kind = static_cast<spec::PortKind>(payload.ports[i].kind),
            .capabilities = payload.ports[i].capabilities,
            .status = payload.ports[i].status,
        };
    }
    list.count = std::min<std::size_t>(payload.port_count, std::size(list.ports));
    list.board_caps = payload.board_caps;
    return list;
}

inline const char* port_kind_name(spec::PortKind kind) {
    switch (kind) {
    case spec::PortKind::kCan: return "CAN bus";
    case spec::PortKind::kUart: return "UART port";
    case spec::PortKind::kImu: return "IMU";
    case spec::PortKind::kGpio: return "GPIO port";
    case spec::PortKind::kBuzzer: return "buzzer";
    default: return "port of unknown kind";
    }
}

// A transmit on a port this host never declared.
//
// That is a mistake in the calling program -- its wiring and its code disagree
// -- not a condition of the link, hence a logic_error. It is a type of its own
// so a caller, and the tests, can tell it from a port the board does not have
// (std::out_of_range) and from a payload that cannot be encoded
// (std::invalid_argument).
class UndeclaredChannel : public std::logic_error {
public:
    using std::logic_error::logic_error;
};

// What a declaration is called in a message: the port's name, or for a GPIO line
// the line's (PWM3).
[[nodiscard]] inline std::string_view declaration_name(data::DataId port, std::uint8_t line) {
    return port == data::DataId::kGpio ? spec::gpio_line_name(line) : channel_name(port);
}
[[nodiscard]] inline std::string_view declaration_name(const Configuration::Entry& entry) {
    return declaration_name(entry.data_id, entry.line);
}

[[noreturn]] inline void throw_undeclared(data::DataId port) {
    throw UndeclaredChannel{std::format(
        "{0} transmission failed: {0} is not declared. A port the host's Configuration does "
        "not name stays off on the board, which drops whatever is sent to it. Give it an entry "
        "in the Configuration the board is constructed with, or redeclare it at run time.",
        channel_name(port))};
}

// The gate every transmit entry point passes through, beside the port-range and
// payload checks: a frame for a port that is off has no way to reach the wire,
// and the board cannot say so -- the data stream carries no negative reply -- so
// the host says it here, at the call that is wrong.
//
// `interface` is the PacketBuilder's snapshot, taken once per packet, so the
// check costs a bit test. Like the other transmit-path throws it refuses one
// field, not the packet: whatever the PacketBuilder was given before it still
// goes out. The board's own drop stays as the backstop for a host that does
// not come through these classes.
inline void require_declared(const Interface& interface, data::DataId port) {
    if (!interface.declared(port)) [[unlikely]]
        throw_undeclared(port);
}

// The GPIO half of the same gate: a write needs a line declared as an output, a
// read request one declared as an input. The board drops an undeclared line and
// the other direction as silently as an undeclared port, so this is the same
// kind of mistake.
inline void require_gpio_direction(const Interface& interface, std::uint8_t line, bool output) {
    const std::string_view name = spec::gpio_line_name(line);
    if (!interface.line_declared(line)) [[unlikely]]
        throw UndeclaredChannel{std::format(
            "{0} transmission failed: {0} is not declared. A GPIO line the host's Configuration "
            "does not name stays off on the board, which drops whatever is sent to it. Give it an "
            "entry in the Configuration the board is constructed with, or redeclare it at run "
            "time.",
            name)};
    if (interface.output(line) != output) [[unlikely]]
        throw UndeclaredChannel{std::format(
            "{0} transmission failed: {0} is declared as an {1}, not as an {2}. Declare it as an "
            "{2} in the Configuration the board is constructed with, or redeclare it at run time.",
            name, output ? "input" : "output", output ? "output" : "input")};
}

// Human-readable name for a ConfigErrorReason, for logs and exceptions.
inline const char* config_error_name(uint8_t reason) {
    switch (static_cast<vc::ConfigErrorReason>(reason)) {
    case vc::ConfigErrorReason::kConfigErrorNone: return "none";
    case vc::ConfigErrorReason::kConfigErrorBadRequest: return "bad request";
    case vc::ConfigErrorReason::kConfigErrorBadIndex: return "no such port";
    case vc::ConfigErrorReason::kConfigErrorUnsupportedMode: return "mode unsupported on that bus";
    case vc::ConfigErrorReason::kConfigErrorModeFixed: return "mode is fixed by the firmware";
    case vc::ConfigErrorReason::kConfigErrorRateUnrepresentable: return "rate unrepresentable";
    case vc::ConfigErrorReason::kConfigErrorFramingUnsupported: return "framing unsupported";
    case vc::ConfigErrorReason::kConfigErrorKindMismatch:
        return "the port is not of the expected kind";
    case vc::ConfigErrorReason::kConfigErrorVerifyFailed:
        return "read-back disagrees with what was programmed";
    case vc::ConfigErrorReason::kConfigErrorIncomplete:
        return "the declaration leaves a required field unset";
    default: return "unknown";
    }
}

// Reads WHY the board most recently STALLed a configuration request. Best
// effort: a transport-level failure while reading (device gone mid-error
// handling) surfaces as a zeroed latch -- the caller's original exception
// matters more than this read.
inline vc::LastConfigErrorPayload read_last_config_error(Ep0Channel auto& handler) {
    vc::LastConfigErrorPayload payload{};
    try {
        (void)handler.vendor_control_in(
            std::to_underlying(vc::Request::kGetLastConfigError), 0, &payload, sizeof(payload));
    } catch (const std::exception&) {
        payload = {};
    }
    return payload;
}

// SET 被 STALL 后的唯一一次追问: 读锁存取原因并抛出。锁存粘滞, 请求码对不上说明是
// 更早那次拒绝留下的, 不能当作本次原因。
[[noreturn]] inline void
    throw_rejected(Ep0Channel auto& handler, vc::Request request, std::string_view what) {
    const auto latch = read_last_config_error(handler);
    const bool ours = latch.request == std::to_underlying(request)
                   && latch.reason != std::to_underlying(vc::ConfigErrorReason::kConfigErrorNone);
    // 只有 VerifyFailed 可能发生在写入之后, 其余原因都保证寄存器没动。
    const bool maybe_written =
        !ours
        || latch.reason == std::to_underlying(vc::ConfigErrorReason::kConfigErrorVerifyFailed);
    throw std::runtime_error{std::format(
        "Board rejected {}: {}. {}", what,
        ours ? config_error_name(latch.reason) : "the board latched no reason for this request",
        maybe_written ? "Read the port back before trusting it."
                      : "The port keeps its previous configuration.")};
}

namespace detail {

// 一个声明项的线上形状: 清单项的设置段就是该口类型的载荷, 一比一拷贝。
[[nodiscard]] inline vc::CanConfigPayload
    to_can_setting(const Configuration::Entry& entry, bool rate_settable) {
    if (!entry.can.fd && entry.can.data_baudrate != 0U) {
        throw std::invalid_argument{std::format(
            "{}: a classic bus has no data phase, but data_baudrate = {} was given.",
            channel_name(entry.data_id), entry.can.data_baudrate)};
    }
    uint8_t control = vc::kCanConfigApply;
    if (rate_settable)
        control |= vc::kCanConfigApplyTiming;
    return {
        .mode = std::to_underlying(entry.can.fd ? vc::CanMode::kCanFd : vc::CanMode::kClassic),
        .control = control,
        .reserved0 = 0,
        .arbitration_baudrate = entry.can.arbitration_baudrate,
        .data_baudrate = entry.can.data_baudrate,
        .nominal_sample_point = 0,
        .data_sample_point = 0,
        .reserved1 = 0};
}

[[nodiscard]] inline vc::UartConfigPayload to_uart_setting(const Configuration::Entry& entry) {
    return {
        .baudrate = entry.uart.baudrate,
        .divisor = entry.uart.divisor,
        .oversample = entry.uart.oversample,
        .word_length = std::to_underlying(entry.uart.word_length),
        .parity = std::to_underlying(entry.uart.parity),
        .stop_bits = std::to_underlying(entry.uart.stop_bits),
        .control = vc::kUartConfigApply,
        .rx_polarity = std::to_underlying(entry.uart.rx_polarity)};
}

[[nodiscard]] inline vc::GpioConfigPayload to_gpio_setting(const Configuration::Entry& entry) {
    return {
        .mode = std::to_underlying(entry.gpio.mode),
        .pull = std::to_underlying(entry.gpio.pull),
        .input_flags = entry.gpio.input_flags,
        .reserved0 = 0,
        .period_ms = entry.gpio.period_ms,
        .reserved1 = 0};
}

[[nodiscard]] inline vc::ImuConfigPayload to_imu_setting(const Configuration::Entry& entry) {
    return {
        .accelerometer_rate_hz = entry.imu.accelerometer_rate_hz,
        .gyroscope_rate_hz = entry.imu.gyroscope_rate_hz,
        .gyroscope_bandwidth_hz = entry.imu.gyroscope_bandwidth_hz,
        .gyroscope_range_dps = entry.imu.gyroscope_range_dps,
        .accelerometer_range_g = entry.imu.accelerometer_range_g,
        .reserved = {}};
}

// 声明是完整的(vendor_control.hpp, v10 -> v11): 板子也会拒绝缺项的声明, 但在发出之前
// 就按字段名报出来, 比板子的"某个偏移处缺一项"好读得多。
inline void require_complete(const Configuration::Entry& entry) {
    std::string missing;
    const auto need = [&missing](bool given, std::string_view name) {
        if (given)
            return;
        if (!missing.empty())
            missing += ", ";
        missing += name;
    };
    switch (entry.kind) {
    case spec::PortKind::kCan:
        need(entry.can.arbitration_baudrate != 0U, "arbitration_baudrate");
        if (entry.can.fd)
            need(entry.can.data_baudrate != 0U, "data_baudrate");
        break;
    case spec::PortKind::kUart:
        need(entry.uart.baudrate != 0U, "baudrate");
        need(std::to_underlying(entry.uart.word_length) != 0U, "word_length");
        need(std::to_underlying(entry.uart.parity) != 0U, "parity");
        need(std::to_underlying(entry.uart.stop_bits) != 0U, "stop_bits");
        need(std::to_underlying(entry.uart.rx_polarity) != 0U, "rx_polarity");
        break;
    case spec::PortKind::kImu:
        need(entry.imu.accelerometer_range_g != 0U, "accelerometer_range_g");
        need(entry.imu.accelerometer_rate_hz != 0U, "accelerometer_rate_hz");
        need(entry.imu.gyroscope_range_dps != 0U, "gyroscope_range_dps");
        need(entry.imu.gyroscope_rate_hz != 0U, "gyroscope_rate_hz");
        need(entry.imu.gyroscope_bandwidth_hz != 0U, "gyroscope_bandwidth_hz");
        break;
    case spec::PortKind::kGpio: need(entry.gpio.mode != vc::kGpioModeOff, "mode"); break;
    case spec::PortKind::kBuzzer: break; // nothing to state
    }
    if (!missing.empty()) {
        throw std::invalid_argument{std::format(
            "{}: a declaration states every setting of the port, nothing is left to the "
            "firmware's current value; missing: {}",
            declaration_name(entry), missing)};
    }
}

// 清单没被完整应用: 按逐口结果给口名、原因、值, 以及板子此刻的样子 -- 校验被拒时
// 板子原样, 应用中途回滚时所有口都已挂起。结果读不到时退回锁存。
[[noreturn]] inline void throw_manifest_failure(
    Ep0Channel auto& handler, bool accepted, const vc::ManifestResultPayload* result) {
    if (result != nullptr) {
        const char* aftermath = static_cast<vc::ManifestOutcome>(result->outcome)
                                     == vc::ManifestOutcome::kManifestRolledBack
                                  ? "The board rolled the whole declaration back: every port is "
                                    "off for libhcs until a "
                                    "declaration is accepted."
                                  : "Nothing was changed on the board.";
        for (std::size_t i = 0; i < result->entry_count && i < std::size(result->entries); ++i) {
            const auto& entry = result->entries[i];
            if (entry.reason != std::to_underlying(vc::ConfigErrorReason::kConfigErrorNone)) {
                throw std::runtime_error{std::format(
                    "Board rejected the declaration of {}: {} (value {}). {}",
                    declaration_name(static_cast<data::DataId>(entry.data_id), entry.line),
                    config_error_name(entry.reason), entry.value, aftermath)};
            }
        }
    }
    if (accepted) {
        throw std::runtime_error{
            result == nullptr ? "Board rejected the EP0 manifest result read-back."
                              : "Board acknowledged the declaration but reports it unapplied."};
    }
    throw_rejected(handler, vc::Request::kApplyManifest, "the port declaration");
}

// 按口寻址的一次 IN 读: wIndex = 口的 DataId(GPIO 口另在高字节带线号), 应答就是
// Payload。被拒即抛, 报错带口名(线名)。
template <typename Payload>
[[nodiscard]] Payload read_port(
    Ep0Channel auto& handler, vc::Request request, data::DataId port, std::string_view what,
    std::uint8_t line = 0) {
    Payload payload{};
    const auto index = static_cast<std::uint16_t>(std::to_underlying(port) | (line << 8U));
    if (!handler.vendor_control_in(std::to_underlying(request), index, &payload, sizeof(payload))) {
        throw std::runtime_error{
            std::format("Board rejected the EP0 {} for {}.", what, declaration_name(port, line))};
    }
    return payload;
}

} // namespace detail

// One place for the whole construction-time exchange, so every board class
// performs it identically: learn what the board is, declare every port of the
// wiring in ONE manifest, read the per-port results.
//
// The board classes run it again from the keepalive thread before every
// re-opened session, under their reconfigure lock, with their stored
// configuration: the constructor's, plus every run-time redeclaration written
// back by reconfigure_uart()/reconfigure_can().
//
// Every failure throws before the constructor returns, so "constructed" means
// "the board runs exactly this declaration": a wiring that names a port the PCB
// lacks, a port of the wrong kind, or a setting the board refused (with the
// latched reason and the port's name).
//
// What the board has and what each port can do come from the board itself --
// the port list its firmware builds from the drivers it initialized at power-up
// -- never from a host-side copy that could disagree with it.
inline Interface apply(Ep0Channel auto& handler, const Configuration& configuration) {
    const PortList list = read_port_list(handler);

    // 每个声明的口都必须在板上实际存在, 类型一致, 能力与设置相容 -- 在发送清单
    // 之前就核对, 报错用口名。
    for (std::size_t i = 0; i < configuration.count; ++i) {
        const auto& entry = configuration.entries[i];
        const auto* port = list.find(entry.data_id);
        if (port == nullptr) {
            throw std::runtime_error{std::format(
                "wiring uses {} but this board has no {}", declaration_name(entry),
                channel_name(entry.data_id))};
        }
        if (port->kind != entry.kind) {
            throw std::runtime_error{std::format(
                "wiring uses {} as a {} but the board reports it as a {}", declaration_name(entry),
                port_kind_name(entry.kind), port_kind_name(port->kind))};
        }
        // GPIO 口的能力字节是它的线数。
        if (entry.kind == spec::PortKind::kGpio && entry.line >= port->capabilities) {
            throw std::runtime_error{std::format(
                "wiring uses {} but this board's GPIO port has {} lines", declaration_name(entry),
                port->capabilities)};
        }
        detail::require_complete(entry);
        if (entry.kind == spec::PortKind::kCan
            && (port->capabilities & spec::kCanCapRateSettable) == 0U) {
            // 速率不可设的板把速率当核对; 但经典数据段速率在这类板上没有意义,
            // 与 hpm 的规则一致地提前拒绝(报错里有口名, 而不是板端 stall 的笼统原因)。
            if (!entry.can.fd && entry.can.data_baudrate != 0U)
                throw std::invalid_argument{std::format(
                    "{}: a classic bus has no data phase, but data_baudrate = {} was given.",
                    channel_name(entry.data_id), entry.can.data_baudrate)};
        }
    }

    // 整板请求同样先在主机这边核对: 板子没报这项能力就不发。
    if (configuration.time_sync && (list.board_caps & vc::kBoardCapTimeSync) == 0U) {
        throw std::runtime_error{
            "time_sync was declared but this board reports no shared time base"};
    }

    // 一次清单 = 一次声明。板端两阶段提交: 全部校验通过才应用, 应用全部成功才
    // 切归属并 ACK。
    vc::ManifestPayload manifest{};
    manifest.version = vc::kVersion;
    manifest.entry_count = static_cast<uint8_t>(configuration.count);
    manifest.flags =
        configuration.time_sync ? std::uint8_t{vc::kManifestFlagTimeSync} : std::uint8_t{0};
    for (std::size_t i = 0; i < configuration.count; ++i) {
        const auto& entry = configuration.entries[i];
        const auto* port = list.find(entry.data_id);
        auto& out = manifest.entries[i];
        out.data_id = std::to_underlying(entry.data_id);
        out.kind = std::to_underlying(entry.kind);
        out.line = entry.line;
        if (entry.kind == spec::PortKind::kCan) {
            const auto can = detail::to_can_setting(
                entry, (port->capabilities & spec::kCanCapRateSettable) != 0U);
            std::memcpy(out.setting, &can, sizeof(can));
        } else if (entry.kind == spec::PortKind::kUart) {
            const auto uart = detail::to_uart_setting(entry);
            std::memcpy(out.setting, &uart, sizeof(uart));
        } else if (entry.kind == spec::PortKind::kImu) {
            const auto imu = detail::to_imu_setting(entry);
            std::memcpy(out.setting, &imu, sizeof(imu));
        } else if (entry.kind == spec::PortKind::kGpio) {
            const auto gpio = detail::to_gpio_setting(entry);
            std::memcpy(out.setting, &gpio, sizeof(gpio));
        } // the buzzer's setting is all zero
    }
    const auto manifest_size =
        offsetof(vc::ManifestPayload, entries) + manifest.entry_count * sizeof(vc::ManifestEntry);
    // 逐口结果在清单被拒(STALL)与被接受之后都要读: 板端对任何失败都记录"哪个口、
    // 为什么、什么值", 以及这次是校验被拒(板子原样)还是应用中途回滚(口全部挂起),
    // 抛错时给口名和板子此刻的样子。
    const bool accepted = handler.vendor_control_out(
        std::to_underlying(vc::Request::kApplyManifest), 0, &manifest, manifest_size);
    vc::ManifestResultPayload result{};
    const bool have_result = handler.vendor_control_in(
        std::to_underlying(vc::Request::kGetManifestResult), 0, &result, sizeof(result));
    if (!accepted || !have_result
        || static_cast<vc::ManifestOutcome>(result.outcome)
               != vc::ManifestOutcome::kManifestApplied)
        detail::throw_manifest_failure(handler, accepted, have_result ? &result : nullptr);

    // 主机自己的快照: 口的位号 = DataId, GPIO 的位号 = 线号。FD 位以声明为准(板端 ACK 已
    // 确认它现在跑的就是声明的帧型); 清单里未声明 CAN 口的 FD 位随板报, 供诊断读取。
    Interface interface{};
    for (std::size_t i = 0; i < list.count; ++i) {
        const auto& port = list.ports[i];
        if (port.kind != spec::PortKind::kCan)
            continue;
        interface.set_can_fd(port.data_id, (port.status & spec::kPortFd) != 0U);
        interface.set_long_frames(
            port.data_id, (port.capabilities & spec::kCanCapFdLongFrames) != 0U);
    }
    for (std::size_t i = 0; i < configuration.count; ++i) {
        const auto& entry = configuration.entries[i];
        interface.declare(entry.data_id);
        if (entry.kind == spec::PortKind::kCan)
            interface.set_can_fd(entry.data_id, entry.can.fd);
        if (entry.kind == spec::PortKind::kGpio)
            interface.declare_line(entry.line, entry.gpio.mode == vc::kGpioModeOutput);
    }
    return interface;
}

// Serializes run-time redeclaration against the reconnect hook.
//
// WHY THIS IS SHARED RATHER THAN PER-BOARD. The hazard is identical on every
// board and does not depend on what a given chip's ports are: the keepalive
// thread re-runs apply() (and re-reads the port list) before every re-opened
// session, while an application thread can redeclare a port at any moment. Two
// writers, one EP0 channel, no ordering between them -- under one lock, whichever
// runs second defines both.
//
// Readers never take this lock: the board classes keep the learned interface
// in a std::atomic that writers publish to, so transmit-path checks stay
// wait-free (see the static_assert on atomic<Interface>).
class Reconfigurable {
public:
    Reconfigurable() = default;
    Reconfigurable(const Reconfigurable&) = delete;
    Reconfigurable& operator=(const Reconfigurable&) = delete;
    Reconfigurable(Reconfigurable&&) = delete;
    Reconfigurable& operator=(Reconfigurable&&) = delete;
    ~Reconfigurable() = default;

protected:
    std::mutex reconfigure_mutex_;
};

// 运行期改一路 UART: 合并非零字段(同旧协议的稀疏补丁), 然后把整份存储的声明作为一次
// 新清单重放 -- 清单就是"声明的全部", 只发一项会把其余口挂起。被拒即抛, 存储的
// Configuration 不含这次改动(重连不重放被拒的设置)。调用方须持重配锁(见 Reconfigurable)。
inline void reconfigure_uart(
    Ep0Channel auto& handler, Configuration& configuration, std::atomic<Interface>& interface,
    data::DataId port, const UartSetting& setting) {
    Configuration candidate = configuration;
    auto* existing = candidate.find(port);
    UartSetting stored = existing != nullptr ? existing->uart : UartSetting{};
    // 分频器断言只跟它所属的速率走: 换速率即作废旧断言, 无速率的纯断言不记。
    if (setting.baudrate != 0) {
        stored.baudrate = setting.baudrate;
        stored.divisor = setting.divisor;
        stored.oversample = setting.oversample;
    }
    if (std::to_underlying(setting.word_length) != 0)
        stored.word_length = setting.word_length;
    if (std::to_underlying(setting.parity) != 0)
        stored.parity = setting.parity;
    if (std::to_underlying(setting.stop_bits) != 0)
        stored.stop_bits = setting.stop_bits;
    if (std::to_underlying(setting.rx_polarity) != 0)
        stored.rx_polarity = setting.rx_polarity;
    candidate.declare_uart(port, stored);

    Interface fresh = apply(handler, candidate); // 被拒即抛, candidate 丢弃
    configuration = candidate;
    interface.store(fresh, std::memory_order_release);
}

// 运行期改一路 CAN 的设置: 同上, 整份声明重放。
inline void reconfigure_can(
    Ep0Channel auto& handler, Configuration& configuration, std::atomic<Interface>& interface,
    data::DataId port, const CanSetting& setting) {
    Configuration candidate = configuration;
    candidate.declare_can(port, setting);

    Interface fresh = apply(handler, candidate);
    configuration = candidate;
    interface.store(fresh, std::memory_order_release);
}

// 运行期改一根 GPIO 线的方向或采样: 同上, 整份声明重放(其余线设置不变, 输出值不受影响)。
inline void reconfigure_gpio(
    Ep0Channel auto& handler, Configuration& configuration, std::atomic<Interface>& interface,
    std::uint8_t line, const GpioSetting& setting) {
    Configuration candidate = configuration;
    candidate.declare_gpio(line, setting);

    Interface fresh = apply(handler, candidate);
    configuration = candidate;
    interface.store(fresh, std::memory_order_release);
}

// Reads what a UART port is REALLY running: the effective baudrate reconstructed
// on the board from the divisor actually programmed, plus the framing decoded
// from the live registers. Never the values that were requested -- that
// distinction is the entire point, because a rate the clock cannot represent
// leaves the divisor untouched and the port on its old setting.
inline UartSetting read_uart_setting(Ep0Channel auto& handler, data::DataId port) {
    const auto payload = detail::read_port<vc::UartConfigPayload>(
        handler, vc::Request::kGetPortConfig, port, "configuration read-back");
    return {
        .baudrate = payload.baudrate,
        .divisor = payload.divisor,
        .oversample = payload.oversample,
        .word_length = static_cast<vc::UartWordLength>(payload.word_length),
        .parity = static_cast<vc::UartParity>(payload.parity),
        .stop_bits = static_cast<vc::UartStopBits>(payload.stop_bits),
        .rx_polarity = static_cast<vc::UartRxPolarity>(payload.rx_polarity),
    };
}

// Rate-only convenience over the full read-back.
inline uint32_t read_uart_baudrate(Ep0Channel auto& handler, data::DataId port) {
    return read_uart_setting(handler, port).baudrate;
}

// Reads what a GPIO line runs: its direction and sampling as the board's
// registers and driver hold them (vc::kGpioModeOff for an undeclared line).
inline GpioSetting read_gpio_setting(Ep0Channel auto& handler, std::uint8_t line) {
    const auto payload = detail::read_port<vc::GpioConfigPayload>(
        handler, vc::Request::kGetPortConfig, data::DataId::kGpio, "configuration read-back", line);
    return {
        .mode = static_cast<vc::GpioMode>(payload.mode),
        .pull = static_cast<vc::GpioPull>(payload.pull),
        .input_flags = payload.input_flags,
        .period_ms = payload.period_ms,
    };
}

// Reads what the buzzer plays right now, reconstructed from its timer registers
// (0 Hz = silent).
inline vc::BuzzerConfigPayload read_buzzer_state(Ep0Channel auto& handler) {
    return detail::read_port<vc::BuzzerConfigPayload>(
        handler, vc::Request::kGetPortConfig, data::DataId::kBuzzer, "state read-back");
}

// Reads one CAN bus's full timing identity as the board reported it: the TX
// mode in force and the rates and sample points the controller is actually
// timed for. Read-only -- redeclaring goes through the manifest.
inline vc::CanConfigPayload read_can_config(Ep0Channel auto& handler, data::DataId port) {
    return detail::read_port<vc::CanConfigPayload>(
        handler, vc::Request::kGetPortConfig, port, "configuration read-back");
}

using data::last_error_name;

// Reads how much of a CAN round trip the board itself accounts for. Available
// on the shipping image; pass reset=true to clear the accumulators after
// reading, so a caller can bracket a measurement.
inline vc::LatencyBreakdownPayload read_latency_breakdown(Ep0Channel auto& handler, bool reset) {
    vc::LatencyBreakdownPayload payload{};
    if (!handler.vendor_control_in(
            std::to_underlying(vc::Request::kGetLatencyBreakdown), reset ? 1 : 0, &payload,
            sizeof(payload))) {
        throw std::runtime_error{"Board rejected the EP0 latency-breakdown query."};
    }
    return payload;
}

} // namespace libhcs::board::hcs
