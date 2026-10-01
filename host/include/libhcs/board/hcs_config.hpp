#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <format>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <libhcs/data/datas.hpp>
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
    // Whether the board carries CAN-FD long frames (12-64 byte payloads) on
    // FD buses (see CanCapabilities::kCapCanFdLongFrames). The board classes
    // gate can_transmit() on this plus can_fd(bus): a long frame to a bus
    // that is not running FD has no on-wire encoding either.
    bool can_fd_long_frames = false;
    // 板子是否按 host 下发的速率解位时序(kCapCanRateSettable)。否则 CanSetting 的速率只作核对。
    bool can_rate_settable = false;
    // Explicit padding, NOT an afterthought: the atomic<Interface> below must
    // stay is_always_lock_free, and on x86-64 that means 1/2/4/8 bytes -- a
    // 6-byte Interface would cross the boundary and put an implicit lock on
    // every transmit path.
    std::array<uint8_t, 2> reserved = {};

    [[nodiscard]] bool can_fd(std::size_t bus) const {
        return bus < can_count && ((can_fd_mask >> bus) & 1U) != 0U;
    }

    [[nodiscard]] bool can_long_frames(std::size_t bus) const {
        return can_fd_long_frames && can_fd(bus);
    }

    // 板子确认 bus 已切到 fd 后同步掩码, 不必再读一次 kGetInterface。
    void set_can_fd(std::size_t bus, bool fd) {
        const auto bit = static_cast<uint8_t>(1U << bus);
        can_fd_mask =
            fd ? static_cast<uint8_t>(can_fd_mask | bit) : static_cast<uint8_t>(can_fd_mask & ~bit);
    }
};

// The board classes keep the learned interface in a std::atomic: the
// before-session hook that writes it re-runs on every reconnect from the
// keepalive thread, while application transmit paths read it concurrently.
// An oversized or non-lock-free layout would put an implicit lock on the
// transmit path -- refuse to compile instead.
static_assert(std::atomic<Interface>::is_always_lock_free);

// One UART port's setting, in the EP0 payload's coding: 0 means "leave
// unchanged" on a request, and a read-back always fills every field with what
// the hardware actually runs (never a zero, never the last request).
//
// `divisor`/`oversample` are the port's rate in its integer form, and they are
// what a rate agreement is actually checked against -- see vc::UartDivisor for
// why the baudrate number cannot be. On a request they are an ASSERTION, not an
// instruction: leave them zero and the apply path only checks that the board
// programmed something; set them (from a prior read_uart_setting) and the
// board must land on exactly those integers.
//
// Declared above Configuration because Configuration stores the whole setting
// per port -- see its comment for why there is one array and not two.
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
};

// 一路 CAN 总线的设置: 帧型与两段速率都是接线事实(由总线对端电机决定), 写死在 host 代码里,
// 经 EP0 握手下发。速率 0 = 沿用板子当前值。采样点(87.5%)与 TDC 是板端实测策略, 不在此。
//
// 速率可设的板(hpm_board)按它解位时序, 解不出即构造失败; 速率写死的板(mc02)把非零速率
// 当核对, 不符即构造失败。c_board 同为核对(速率从 bxCAN BTR 反推), 经典总线无数据段,
// data_baudrate 须为 0。
struct CanSetting {
    bool fd = false;
    uint32_t arbitration_baudrate = 0;
    uint32_t data_baudrate = 0; // 仅 FD; 经典总线没有数据段, 须为 0

    friend constexpr bool operator==(const CanSetting&, const CanSetting&) = default;
};

// 本项目实际存在的两种总线。
inline constexpr CanSetting kClassic1M{.fd = false, .arbitration_baudrate = 1'000'000};
inline constexpr CanSetting kFd1M5M{
    .fd = true, .arbitration_baudrate = 1'000'000, .data_baudrate = 5'000'000};

// Serializes run-time reconfiguration against the reconnect hook.
//
// WHY THIS IS SHARED RATHER THAN PER-BOARD. The hazard is identical on every
// board and does not depend on what a given chip's channels are: the keepalive
// thread re-runs apply() (and re-learns the interface) before every re-opened
// session, while an application thread can call configure_uartN()/
// configure_canN() at any moment. Two writers, one EP0 channel, no ordering
// between them -- so whichever finishes last decides the board's state, and a
// configure_*() that lands between the hook's request and the hook's mirror
// leaves the cached interface describing the hook's request while the board
// runs this call's. Under one lock, whichever runs second defines both.
//
// mc02 held a mutex of its own before this, but only configure_can() took it:
// its configure_uart*() were unguarded, so the same interleaving was reachable
// through the UART path. Boards that expose only one kind of mutator inherit
// the guard anyway -- the cost is an uncontended mutex, and the alternative is
// per-board reasoning about which mutators happen to exist today.
//
// Readers never take this lock: the board classes keep the learned interface
// in a std::atomic that writers publish to, so canN_is_fd()/interface() stay
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

// Desired configuration, all optional. An unset field is left alone: the board
// keeps whatever its firmware brought the channel up with, which is the right
// default for a caller that does not care.
struct Configuration {
    // Per-UART-port setting, indexed by the board's own port numbering. An
    // unset entry is left alone ("don't touch"); a set entry is applied in
    // full, and a field left zero inside it means "set the rate, keep the
    // framing the firmware came up with".
    //
    // One array of the full setting, NOT a baudrate array beside a framing
    // one: two parallel arrays would give "is this port configured?" a second
    // source of truth, and the two answers could disagree. This also fixes a
    // silent regression -- with only a baudrate replayable, a 7E2 port fell
    // back to its CubeMX frame format on the first reconnect, because nothing
    // carried the framing across.
    std::optional<UartSetting> uart[8];
    // Per-bus setting (frame type + rates), as a REQUEST: on a board advertising
    // kCapCanModeSettable / kCapCanRateSettable the bus is reconfigured and the
    // board verifies its own read-back before ACKing (mc02 flips the Tx-element
    // FDF/BRS flags; hpm_board re-initializes the controller -- with FD off for
    // classic, so a CAN 2.0 bus never sees an FD bit -- at the requested
    // rates); what a board cannot apply degrades to an assertion and
    // construction fails when the board disagrees. Either way a constructed
    // board runs the requested setting or the constructor threw.
    std::optional<CanSetting> can[8];
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
        .can_fd_long_frames = (payload.caps & vc::kCapCanFdLongFrames) != 0U,
        .can_rate_settable = (payload.caps & vc::kCapCanRateSettable) != 0U,
    };
}

// Gate for boards that cannot carry a CAN-FD long payload at all: bxCAN-class
// controllers (c_board, ch32_board), or boards whose firmware has not widened
// its RX elements yet (mc02). The core serializer WOULD encode the frame, so
// the board classes must call this before write_can() -- the firmware side has
// only an 8-byte TX element, and a long record arriving there would overflow
// it on a debugless build.
inline void reject_long_payload(std::span<const std::byte> payload, std::string_view port) {
    if (payload.size() > 8)
        throw std::invalid_argument{std::format(
            "{} transmission failed: payloads over 8 bytes need a board advertising "
            "kCapCanFdLongFrames on a CAN-FD bus",
            port)};
}

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
    case vc::ConfigErrorReason::kConfigErrorVerifyFailed:
        return "read-back disagrees with what was programmed";
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

// SET 被 STALL 后的唯一一次追问: 读锁存取原因并抛出。锁存粘滞, 请求码或下标对不上
// 说明是更早那次拒绝留下的, 不能当作本次原因。
[[noreturn]] inline void throw_rejected(
    host::protocol::Handler& handler, vc::Request request, std::size_t index,
    std::string_view what) {
    const auto latch = read_last_config_error(handler);
    const bool ours = latch.request == std::to_underlying(request) && latch.index == index
                   && latch.reason != std::to_underlying(vc::ConfigErrorReason::kConfigErrorNone);
    // 只有 VerifyFailed 可能发生在写入之后, 其余原因都保证寄存器没动。
    const bool maybe_written =
        !ours
        || latch.reason == std::to_underlying(vc::ConfigErrorReason::kConfigErrorVerifyFailed);
    throw std::runtime_error{std::format(
        "Board rejected {}: {}. {}", what,
        ours ? config_error_name(latch.reason) : "the board latched no reason for this request",
        maybe_written ? "Read the channel back before trusting it."
                      : "The channel keeps its previous configuration.")};
}

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
        .divisor = payload.divisor,
        .oversample = payload.oversample,
        .word_length = static_cast<vc::UartWordLength>(payload.word_length),
        .parity = static_cast<vc::UartParity>(payload.parity),
        .stop_bits = static_cast<vc::UartStopBits>(payload.stop_bits),
        .rx_polarity = static_cast<vc::UartRxPolarity>(payload.rx_polarity),
    };
}

// Rate-only convenience over the full read-back, for board classes that expose
// just the baudrate.
inline uint32_t read_uart_baudrate(host::protocol::Handler& handler, std::size_t port) {
    return read_uart_setting(handler, port).baudrate;
}

// Applies a setting: ONE control transfer. Every non-zero field is applied,
// zero fields leave the port untouched, and `divisor`/`oversample` (if set) are
// assertions the board checks against its own solution before writing.
//
// 成功路径只有这一次 SET, 不再回读: 板端先全量校验(求解、10% 宽松兜底、回显断言)再写,
// 写后自己回读分频器与帧格式, 不符即 STALL(kConfigErrorVerifyFailed)。所以 ACK 就等于
// "已生效", 主机再读一遍只是重复核对。STALL 时才多读一次锁存取原因, 然后抛出。
// 实际生效的值(如 921600 -> 909090)要看就显式调 read_uart_setting()。
inline void
    configure_uart(host::protocol::Handler& handler, std::size_t port, const UartSetting& setting) {
    const vc::UartConfigPayload payload{
        .baudrate = setting.baudrate,
        .divisor = setting.divisor,
        .oversample = setting.oversample,
        .word_length = std::to_underlying(setting.word_length),
        .parity = std::to_underlying(setting.parity),
        .stop_bits = std::to_underlying(setting.stop_bits),
        .control = vc::kUartConfigApply,
        .rx_polarity = std::to_underlying(setting.rx_polarity)};
    if (!handler.vendor_control_out(
            std::to_underlying(vc::Request::kSetUartConfig), static_cast<uint16_t>(port), &payload,
            sizeof(payload)))
        throw_rejected(
            handler, vc::Request::kSetUartConfig, port,
            setting.baudrate != 0
                ? std::format("the UART{} configuration ({} baud)", port, setting.baudrate)
                : std::format("the UART{} configuration", port));
}

// Rate-only form, the shape every board class exposed before framing joined
// the payload.
inline void configure_uart(host::protocol::Handler& handler, std::size_t port, uint32_t baudrate) {
    configure_uart(handler, port, UartSetting{.baudrate = baudrate});
}

// 运行期改口: 应用后并入 configuration, 重连时 apply() 重放它而非回退; 调用方须持重配锁。
inline void reconfigure_uart(
    host::protocol::Handler& handler, Configuration& configuration, std::size_t port,
    const UartSetting& setting) {
    configure_uart(handler, port, setting);
    UartSetting stored = configuration.uart[port].value_or(UartSetting{});
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
    configuration.uart[port] = stored;
}

// Reads one CAN bus's full timing identity as the board reported it: the TX
// mode in force and the rates and sample points the controller is actually
// timed for. Read-only -- applying a setting goes through request_can_setting().
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

// Requests a bus's setting: ONE control transfer.
//
// 成功路径只有这一次 SET: 速率可设的板(hpm_board)带 kCanConfigApplyTiming, 按速率重初始化后
// 自己回读帧型/速率/采样点, 不符即 STALL(kConfigErrorVerifyFailed), 解不出即 STALL
// (kConfigErrorRateUnrepresentable); 其余板速率字段是核对, 帧型按 kCapCanModeSettable 切或
// 核对。所以 ACK 就等于"总线已是这份设置"。STALL 时读一次锁存取原因并抛出。要看速率与采样点
// 就显式调 read_can_config()。
inline void request_can_setting(
    host::protocol::Handler& handler, std::size_t bus, const CanSetting& setting,
    const Interface& interface) {
    if (!setting.fd && setting.data_baudrate != 0U) {
        throw std::invalid_argument{std::format(
            "CAN{}: a classic bus has no data phase, but data_baudrate = {} was given.", bus + 1,
            setting.data_baudrate)};
    }
    uint8_t control = vc::kCanConfigApply;
    if (interface.can_rate_settable)
        control |= vc::kCanConfigApplyTiming;
    const vc::CanConfigPayload payload{
        .mode = std::to_underlying(setting.fd ? vc::CanMode::kCanFd : vc::CanMode::kClassic),
        .control = control,
        .reserved0 = 0,
        .arbitration_baudrate = setting.arbitration_baudrate,
        .data_baudrate = setting.data_baudrate,
        .nominal_sample_point = 0,
        .data_sample_point = 0,
        .reserved1 = 0};
    if (!handler.vendor_control_out(
            std::to_underlying(vc::Request::kSetCanConfig), static_cast<uint16_t>(bus), &payload,
            sizeof(payload)))
        throw_rejected(
            handler, vc::Request::kSetCanConfig, bus,
            setting.fd
                ? std::format(
                      "CAN{} as CAN-FD ({} / {} baud)", bus + 1, setting.arbitration_baudrate,
                      setting.data_baudrate)
                : std::format(
                      "CAN{} as classic CAN ({} baud)", bus + 1, setting.arbitration_baudrate));
}

// 运行期改总线设置: 成功后同步 FD 掩码并写回 configuration, 重连时 apply() 重放本次设置而非
// 回退。调用方须持重配锁(见 Reconfigurable)。
inline void reconfigure_can(
    host::protocol::Handler& handler, Configuration& configuration,
    std::atomic<Interface>& interface, std::size_t bus, const CanSetting& setting) {
    Interface current = interface.load(std::memory_order_relaxed);
    if (bus >= current.can_count || bus >= std::size(configuration.can)) {
        throw std::out_of_range{std::format(
            "CAN{} does not exist: this board reports {} CAN bus(es).", bus + 1,
            current.can_count)};
    }
    request_can_setting(handler, bus, setting, current);
    current.set_can_fd(bus, setting.fd);
    interface.store(current, std::memory_order_release);
    configuration.can[bus] = setting;
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
// board is, apply the CAN settings, then apply the UART rates.
//
// The board classes run it again from the keepalive thread before every
// re-opened session, under their reconfigure lock, with their stored
// configuration: the constructor's, plus every run-time UART change written
// back by reconfigure_uart(). Channels it leaves unset keep what the board runs.
inline Interface apply(host::protocol::Handler& handler, const Configuration& configuration) {
    Interface interface = read_interface(handler);

    for (std::size_t bus = 0; bus < std::size(configuration.can); ++bus) {
        const auto& setting = configuration.can[bus];
        if (!setting.has_value())
            continue;
        if (bus >= interface.can_count) {
            throw std::runtime_error{std::format(
                "CAN bus #{} (index in the board's CAN table, not a silkscreen number) was "
                "configured but this board reports only {} CAN bus(es).",
                bus, interface.can_count)};
        }
        request_can_setting(handler, bus, *setting, interface);
        // The mask above was read before this request. On a settable board
        // (mc02, hpm_board) the request can change the mode, and the board's
        // ACK has just confirmed it now runs setting->fd -- without this,
        // canN_is_fd() would report the firmware's mode instead of the
        // configured one.
        interface.set_can_fd(bus, setting->fd);
    }

    for (std::size_t port = 0; port < std::size(configuration.uart); ++port) {
        const auto& setting = configuration.uart[port];
        if (!setting.has_value())
            continue;
        if (port >= interface.uart_count) {
            throw std::runtime_error{std::format(
                "UART port #{} (index in the board's UART table, not a silkscreen number) was "
                "configured but this board reports only {} UART port(s).",
                port, interface.uart_count)};
        }
        configure_uart(handler, port, *setting);
    }

    return interface;
}

} // namespace libhcs::board::hcs
