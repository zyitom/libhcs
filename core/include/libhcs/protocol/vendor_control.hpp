#pragma once

#include <cstdint>

#include <libhcs/data/datas.hpp>

// EP0 (control endpoint) configuration channel.
//
// WHY THIS EXISTS. Channel configuration used to ride the same bulk byte stream
// as the data itself: a UART baudrate went out as a kUart*Config field, and the
// CAN frame type as a per-frame IsFdCan header bit. Both were write-only. The
// deserializer callback's bool means "this field id was recognized", not "the
// operation succeeded", so a baudrate the board's divisor solver could not
// represent was rejected on the board and reported to the host as success --
// the failure mode that makes a peer rate mismatch look like a wiring fault.
// (firmware/hpm_board/AGENTS.md recorded this as an open gap: "configuration
// rejected, the host cannot see it".) Echoing the field back does not work
// either: config is a downlink-only channel by contract and an uplink one fails
// the host deserializer outright.
//
// A control transfer has the acknowledgement the bulk stream lacks: the device
// stalls the status stage to reject, and every setting can be read back. So
// configuration moves here, is applied ONCE while the host board object is
// being constructed, and is verified by reading back what the hardware is
// actually running before the constructor returns.
//
// Not gated on the session handshake. Configuration is transport state, not
// data-plane state -- the host applies it before the first keepalive has opened
// a session, and a board must answer these while its data plane is idle.
namespace libhcs::core::protocol::vendor_control {

// The wire-layout version the host and board compare at kGetInterface. NOT a
// hand-maintained number: it is a compile-time FINGERPRINT folded from the
// definitions in this file -- payload sizes, field offsets, request codes, and
// the coding values of every configuration enum. Change any of those and the
// fingerprint changes with them; there is nothing to bump by hand and no way
// to forget. See layout_fingerprint() for the covered inputs and the one
// maintenance rule it does have.
//
//   v1 -> v2 (2026-09-12, the last hand-bumped value): CanConfigPayload grew
//   from a bare mode byte to the current struct (apply control + rate and
//   sample-point verification fields), UartConfigPayload gained the framing
//   fields, and InterfacePayload repurposed a reserved byte as `caps`. A v1
//   host and a v2 board (or the reverse) failed the version check at
//   kGetInterface and refused cleanly -- exactly what the fingerprint
//   continues to do, without the manual step.
//
//   v2 -> v3 (2026-09-20, automatic): CanCapabilities gained
//   kCapCanFdLongFrames, and the CAN record header's retired IsFdCan bit was
//   repurposed as IsLongFrame (core/src/protocol/protocol.hpp). The data
//   stream itself is what changed; the fingerprint moves because a peer that
//   cannot parse long frames must not be handed them, and refusing at
//   kGetInterface is the mechanism both directions already share.
//
//   v3 -> v4 (2026-09-30, automatic): UartConfigPayload grew from 8 to 12
//   bytes with `divisor` and `oversample`, and ConfigErrorReason gained
//   kConfigErrorVerifyFailed = 7. This is the UART-configuration unification:
//   the rate agreement is now checked on the divisor integer the board
//   actually programmed instead of on a percentage of the requested rate.
//   Both ends must move together -- a v3 host against a v4 board (or the
//   reverse) refuses at kGetInterface, which is the intended outcome, since a
//   v3 host would read the old 8-byte layout out of a 12-byte payload.
//
//   v4 -> v5 (2026-09-30, automatic): CanConfigControl gained
//   kCanConfigApplyTiming and CanCapabilities gained kCapCanRateSettable --
//   the host now DECLARES a bus's rates (they are a wiring fact the host code
//   knows, like a UART baudrate) and a board that advertises the capability
//   solves its bit timing for them. Both are new enum values no payload
//   layout observes, so they are folded in by hand (see the maintenance rule).
//
//   v5 -> v6 (2026-09-30, automatic): UartConfigPayload's trailing reserved
//   byte became `rx_polarity` (UartRxPolarity), so one DBUS port can take DBUS,
//   SBUS and iBUS receivers: mc02 flips STM32H7 RXINV to cancel its on-board
//   inverter for iBUS. Same payload size; the fingerprint moves on the new
//   field's offset and the enum codings.
//
// The fingerprint itself lives at the bottom of this file, after every payload
// it folds over.

// bRequest codes. Sent as vendor requests with the DEVICE recipient: TinyUSB
// routes every vendor-type request to tud_vendor_control_xfer_cb regardless of
// recipient, so nothing is gained by addressing an interface, and a device
// recipient keeps wIndex free for the channel index.
enum class Request : uint8_t {
    // IN, wIndex = 0 -> InterfacePayload. What the board has and what it can do.
    kGetInterface = 0x40,
    // IN, wIndex = CAN bus index -> CanConfigPayload. The mode in force.
    kGetCanConfig = 0x41,
    // OUT, wIndex = CAN bus index <- CanConfigPayload. Stalls unless the mode
    // matches what the controller was brought up in; see CanConfigPayload.
    kSetCanConfig = 0x42,
    // IN, wIndex = UART index -> UartConfigPayload. Hardware truth: the
    // EFFECTIVE baudrate reconstructed from the divisor actually programmed,
    // plus the framing decoded from the live registers.
    kGetUartConfig = 0x43,
    // OUT, wIndex = UART index <- UartConfigPayload. Applies the non-zero
    // fields (kUartConfigApply) or asserts them; a STALL means every field was
    // invalid-or-disagreeing and NOTHING changed. See UartConfigPayload.
    kSetUartConfig = 0x44,
    // IN, wIndex = CAN bus index -> CanStatusPayload. The controller's own
    // error registers.
    kGetCanStatus = 0x45,
    // 0x46 was kSetEndpointMode, which chose whether CAN uplink used a second
    // bulk pipe. That pipe was removed 2026-09-05 (see
    // firmware/hpm_board/AGENTS.md); the code is left unused rather than
    // recycled so an old host asking for it gets a stall, not a silent
    // reinterpretation as some later request.
    // IN -> LatencyBreakdownPayload. Board-side share of the round trip; a
    // non-zero wIndex also resets the accumulators after reading.
    kGetLatencyBreakdown = 0x47,
    // IN, wIndex = 0 -> LastConfigErrorPayload. WHY the most recent STALLed
    // config request was refused. A STALL cannot carry data (the status stage
    // is zero-length by USB definition), so the reason travels here instead:
    // the board latches it, and the host reads it back after any failure --
    // the same latch-and-read pattern as kGetCanStatus's error registers.
    kGetLastConfigError = 0x48,
};

// Direction bits of bmRequestType for the requests above.
inline constexpr uint8_t kRequestTypeIn = 0xC0;  // Device-to-host | Vendor | Device
inline constexpr uint8_t kRequestTypeOut = 0x40; // Host-to-device | Vendor | Device

// All payloads are little-endian and byte-packed. USB is little-endian on the
// wire and every host and board in this project is too, so the structs go out
// as-is with no serialization step; the static_asserts pin the layout.
struct InterfacePayload {
    uint16_t version;    // kVersion
    uint8_t can_count;   // CAN buses this PCB actually has
    uint8_t uart_count;  // UART ports this image serves
    uint8_t can_fd_mask; // bit i: bus i currently transmits CAN-FD
    uint8_t caps;        // CanCapabilities
    uint8_t reserved[2];
};
static_assert(sizeof(InterfacePayload) == 8);

// What a board lets the host do to its buses. A capability the board does not
// advertise is not a softer setting -- requesting it stalls.
enum CanCapabilities : uint8_t {
    // The host may switch a bus's TX frame type at kSetCanConfig. How is the
    // board's business: mc02 keeps the controller FD-capable and flips the
    // Tx-element FDF/BRS flags; hpm_board re-initializes the controller with
    // FD off for classic, so a CAN 2.0 bus cannot see an FD bit (the price:
    // a classic-mode controller receives no FD frames -- measured 0/50 --
    // which on a 2.0-only bus is exactly right). Clear means the mode is a
    // compile-time property of the firmware (c_board: bxCAN has no FD).
    kCapCanModeSettable = 1U << 0,
    // The board carries CAN-FD long frames (payloads 12-64 bytes) in BOTH
    // directions on an FD bus: its RX elements and TX buffers are sized for
    // 64 bytes, read_uplink forwards them instead of dropping, and the record
    // stream speaks the IsLongFrame encoding (core/src/protocol/protocol.hpp).
    // A board that clears this bit also rejects long frames downlink -- the
    // host SDK enforces the same gate on its side, so neither end has to
    // guess. Boards whose CAN controller cannot receive FD frames at all
    // (c_board: bxCAN) clear it trivially; mc02 clears it until its RX FIFO
    // elements are widened to match its already FD-capable TX path.
    kCapCanFdLongFrames = 1U << 1,
    // The host may SET a bus's arbitration/data rates (kCanConfigApplyTiming).
    // The board keeps its own sample-point policy and solves prescaler and
    // segments for the requested rate at that point. Clear means the rates are
    // a compile-time property of the firmware and the rate fields stay pure
    // assertions. hpm_board sets it; mc02 and c_board clear it.
    kCapCanRateSettable = 1U << 2,
};

enum class CanMode : uint8_t {
    kClassic = 0, // CAN 2.0
    kCanFd = 1,   // CAN-FD with bitrate switching
};

// What the host asks of one CAN bus, and what the board answers about it.
//
// Two kinds of fields, and the difference is load-bearing:
//
//   mode + control   the REQUEST. control bit kCanConfigApply asks the board
//                    to switch the bus's TX frame type to `mode`. Boards that
//                    advertise kCapCanModeSettable (mc02, hpm_board) apply it;
//                    boards that do not (c_board) stall unless `mode` already
//                    equals the mode they run. A SET without
//                    kCanConfigApply is a pure assertion on every board: it
//                    ACKs only if the bus already runs `mode`.
//
//   rate fields       arbitration_baudrate / data_baudrate. The rates are a
//                    wiring fact -- fixed by the bus peers' hardware, which
//                    the HOST code knows and the firmware does not. With
//                    kCanConfigApplyTiming (boards advertising
//                    kCapCanRateSettable) they are the setting: non-zero is
//                    the new rate, zero keeps the current one. Without it
//                    they are an assertion: zero skips, non-zero must equal
//                    the board's. A classic bus has no data phase: applying
//                    a non-zero data_baudrate with mode = kClassic stalls.
//
//   sample points     always the ASSERTION. The sample point is a measured,
//                    board-specific policy (pinned to 87.5 per mille, so every
//                    node on the bus samples at the same place), never set by
//                    the host; zero means "no opinion, skip the check".
//
// GET returns hardware truth, not the last request: mode is what the bus
// transmits right now, the rates are what the controller is actually timed
// for (data_baudrate is the FD data phase the receiver runs at: mc02 keeps
// its controller FD-capable while transmitting classic and reports it
// regardless of `mode`; hpm_board turns FD off for classic and reports 0,
// as does c_board -- a timing assertion on a data phase the bus does not
// have is a wrong expectation), the sample points are the achieved
// per-mille positions of both phases, and control is zero.
struct CanConfigPayload {
    uint8_t mode;                  // CanMode
    uint8_t control;               // CanConfigControl, SET only; GET returns 0
    uint16_t reserved0;
    uint32_t arbitration_baudrate; // 0 = skip check; else must equal the board's
    uint32_t data_baudrate;        // 0 = skip check; else must equal the board's
    uint16_t nominal_sample_point; // per-mille; 0 = skip; else must equal the board's
    uint16_t data_sample_point;    // per-mille; 0 = skip; else must equal the board's
    uint32_t reserved1;
};
static_assert(sizeof(CanConfigPayload) == 20);

enum CanConfigControl : uint8_t {
    // Apply `mode` to the bus if the board advertises kCapCanModeSettable.
    // Without this bit the SET only asserts that the bus already runs `mode`.
    kCanConfigApply = 1U << 0,
    // Apply the rate fields instead of asserting them, on boards advertising
    // kCapCanRateSettable (see the rate fields above). A board that cannot
    // solve the rate at its pinned sample point stalls with
    // kConfigErrorRateUnrepresentable and leaves the bus untouched.
    kCanConfigApplyTiming = 1U << 1,
};

// UART framing codings. Zero means "no opinion": the field is skipped on SET
// and never returned on GET.
enum UartWordLength : uint8_t {
    kUartWordLength7 = 7, // 7 data bits
    kUartWordLength8 = 8, // 8 data bits
    // 9 data bits is deliberately absent: the RX path on every board is a
    // byte-wide DMA ring, so a ninth data bit would be silently truncated.
};
enum UartParity : uint8_t {
    kUartParitySkip = 0,
    kUartParityNone = 1,
    kUartParityEven = 2,
    kUartParityOdd = 3,
};
enum UartStopBits : uint8_t {
    kUartStopBitsSkip = 0,
    kUartStopBits1 = 1,
    kUartStopBits2 = 2,
    // 1.5 stop bits is deliberately absent: the 16550-style controllers in
    // this repo only realize it for 5-bit words, where the wire looks like
    // what kUartStopBits2 means everywhere else -- an offered alias would lie.
};
// RX line polarity AT THE MCU PIN, not at the connector: a port with an
// on-board inverter (mc02's DBUS) receives an inverted protocol (DBUS, SBUS)
// with kUartRxPolarityNormal, and a non-inverted one (iBUS) only with
// kUartRxPolarityInverted, which cancels the inverter. Only controllers with
// an RX-inversion bit accept kUartRxPolarityInverted (mc02's STM32H7 RXINV);
// hpm_board and c_board have none and refuse it with
// kConfigErrorFramingUnsupported.
enum UartRxPolarity : uint8_t {
    kUartRxPolaritySkip = 0,
    kUartRxPolarityNormal = 1,
    kUartRxPolarityInverted = 2,
};
enum UartConfigControl : uint8_t {
    // Apply every non-zero field of the payload. Without this bit the SET is
    // a pure assertion: it ACKs only if the port already runs exactly what
    // the non-zero fields say (the divisor integers must match exactly; the
    // framing must match exactly). With it, non-zero fields are applied and
    // zero fields are left untouched -- a sparse patch, per setting rather
    // than per bus.
    kUartConfigApply = 1U << 0,
};

// The divisor a port is actually programmed with, and the oversampling it is
// programmed at. These are the integers the rate is made of, and the ONLY
// thing a rate agreement can be checked against:
//
// WHY NOT THE BAUDRATE. The host cannot reconstruct the board's divisor: it
// does not know the board's UART kernel clock (hpm 80 MHz; mc02 STM32H7,
// resolved at run time from PLL2Q/PLL3Q/HSI by UART_GETCLOCKSOURCE; c_board
// 42 or 84 MHz off APB1/APB2; ch32 100 MHz). And the requested rate is not a
// fixed point of the solver: 921600 -- the rate this project uses most --
// comes out 909090 on hpm (80 MHz, 1.36%), 923076 / 933333 on c_board
// (84 / 42 MHz, 0.16% / 1.27%), and each board's error has its own shape. So
// both "compare the requested rate" and "compare within a percentage" are
// unsound, and the only sound comparison is the divisor integer the board
// confirms.
//
// The divisor is the raw register, deliberately un-normalized; the oversample
// is the ratio (8..32), not its register encoding:
//   hpm      divisor = DLM:DLL (16-bit); oversample = OSCR decoded (field 0 -> 32)
//   mc02     divisor = BRR[15:0];        oversample = 8 or 16
//   c_board  divisor = BRR[15:0];        oversample = 8 or 16
// A board reports what its own registers hold, so the host never has to know
// which family it is talking to -- it only has to compare two integers it got
// back from the same board.
//
// SET treats both as an ASSERTION (never as an instruction): apply the
// requested rate, then require the resulting divisor to equal the one the
// caller echoed. A caller that first GETs and then SETs the same values is
// asserting the whole electrical identity of the port in one transfer -- the
// same property kSetCanConfig's timing fields have. Zero means "no opinion,
// skip the check".
struct UartDivisor {
    uint16_t divisor;
    uint8_t oversample;
};

// One UART port's configuration. SET applies what is non-zero (with
// kUartConfigApply) or asserts it (without); a STALL leaves the port exactly
// as it was, because every field is validated before the first register is
// touched. GET returns hardware truth, never the last request: the EFFECTIVE
// baudrate reconstructed from the divisor actually programmed, the framing
// decoded from the live registers, and control = 0.
//
// Flow control (RTS/CTS) has no field here on purpose: it is pinned by the
// pins the .ioc routes at boot, a hardware deployment fact no run-time
// register write can change. The kernel clock source is likewise an init-time
// fact -- it defines what the divisor means, and the board's read-back already
// reflects it. The OVERsampling is not in that category: it is reported, as
// UartDivisor::oversample, because it is half of what turns a divisor into a
// rate.
struct UartConfigPayload {
    uint32_t baudrate;   // SET: requested rate, 0 = leave unchanged; GET: effective rate
    uint16_t divisor;    // UartDivisor::divisor -- GET: programmed; SET: asserted
    uint8_t oversample;  // UartDivisor::oversample -- GET: programmed; SET: asserted
    uint8_t word_length; // UartWordLength
    uint8_t parity;      // UartParity
    uint8_t stop_bits;   // UartStopBits
    uint8_t control;     // UartConfigControl, SET only; GET returns 0
    uint8_t rx_polarity; // UartRxPolarity
};
static_assert(sizeof(UartConfigPayload) == 12);

// Why a refused configuration request deserves a request of its own: a STALL
// cannot carry data (the status stage is zero-length by USB definition), so
// "the board refused" is all the wire ever says. The board therefore LATCHES
// why -- request code, channel, a reason from the enum below, and the
// offending value -- and the host reads this after any failure, which turns
// "rate outside the solver's reach OR framing unsupported" guesses into an
// exact answer. Sticky until the next refusal overwrites it; cleared only by
// a reboot. The same latch-and-read pattern as the boards' own CAN error
// registers (PSR.LEC + kGetCanStatus).
enum ConfigErrorReason : uint8_t {
    kConfigErrorNone = 0,
    kConfigErrorBadRequest = 1,          // unknown bRequest or control bits
    kConfigErrorBadIndex = 2,            // wIndex names a channel this board lacks
    kConfigErrorUnsupportedMode = 3,     // requested TX frame type unsupported on that bus
    kConfigErrorModeFixed = 4,           // the board does not let the host change the mode
    kConfigErrorRateUnrepresentable = 5, // the divisor solver cannot produce this rate
    kConfigErrorFramingUnsupported = 6,  // word length / parity / stop bits unsupported
    // The rate WAS representable and the port was programmed -- but reading
    // the divisor back gave a different integer than the one that was just
    // written, or than the one the caller echoed. Distinct from
    // kConfigErrorRateUnrepresentable on purpose: that one means the solver
    // refused and nothing was touched, this one means something was written
    // and the hardware does not agree with what it was given. It is the only
    // reason that reports a port whose register file no longer matches its
    // programming, so it must not be collapsed into the other.
    kConfigErrorVerifyFailed = 7,
};

struct LastConfigErrorPayload {
    uint8_t request; // bRequest of the refused request
    uint8_t reason;  // ConfigErrorReason
    uint16_t index;  // wIndex of the refused request
    uint32_t value;  // offending rate for timing rejections; 0 otherwise
    uint32_t reserved;
};
static_assert(sizeof(LastConfigErrorPayload) == 12);

// Why a controller's error registers are worth a request of their own: when a
// bus delivers nothing, "the wire is bad" and "the firmware never transmitted"
// look identical from the host -- both are rx=0. The controller knows which it
// is, and says so in registers no data path exposes.
//
// Reading them used to mean building and flashing a libhcs_CAN_DIAG image,
// whose telemetry then rides DataId::kUart0 and corrupts anything measuring
// that channel. On 2026-09-04 that cost six reflashes and one false UART
// regression. Here they cost one control transfer against the shipping image.
//
// How to read the answer, in the order that narrows fastest:
//   last_error == kAck, tec climbing      transmitted, nobody acknowledged --
//                                         the far end is not listening
//   last_error == kBit0                   drove dominant, read back recessive:
//                                         the bus cannot be pulled low at all
//   stuff / form / crc / bit1             bits arrive corrupted -- bit timing,
//                                         termination, or noise
//   tx_occurred == 0 && tx_cancelled != 0 nothing ever reached the wire
struct CanStatusPayload {
    uint8_t tec;             // ECR transmit error counter
    uint8_t rec;             // ECR receive error counter
    uint8_t last_error;      // PSR.LEC  -- arbitration phase, see LastErrorCode
    uint8_t data_last_error; // PSR.DLEC -- CAN-FD data phase, same coding
    uint8_t flags;           // CanStatusFlags
    uint8_t reserved[3];
    uint32_t tx_occurred;    // TXBTO: transmissions that actually completed
    uint32_t tx_cancelled;   // TXBCF: transmissions abandoned (this driver runs
                             // with automatic retransmission disabled)
    uint32_t rx_frames;      // frames forwarded to the host since boot
    uint32_t rx_fifo_level;  // RXF0S fill level, non-zero means a backlog
};
static_assert(sizeof(CanStatusPayload) == 24);

// How much of a CAN round trip the board is responsible for.
//
// Two segments, both entirely inside the board and both measured in core clock
// cycles (divide by cpu_hz):
//
//   downlink  bulk OUT completion -> frame in the MCAN TX FIFO
//   uplink    CAN receive interrupt -> frame serialized into the uplink batch
//
// Everything else in the round trip is elsewhere: the CAN wire itself (about
// 50 us for 8 bytes at 1M/5M FD), USB microframe quantization, and the host's
// submit/wakeup path. Sizing the board's share against those is the point --
// without it, choosing which segment to optimise is guesswork.
struct LatencyBreakdownPayload {
    uint32_t downlink_count;
    uint32_t downlink_min_cycles;
    uint32_t downlink_max_cycles;
    uint64_t downlink_sum_cycles;
    uint32_t uplink_count;
    uint32_t uplink_min_cycles;
    uint32_t uplink_max_cycles;
    uint64_t uplink_sum_cycles;
    uint32_t cpu_hz;
    uint32_t reserved;
};
static_assert(sizeof(LatencyBreakdownPayload) == 56);

// ---- Wire-layout fingerprint: where kVersion comes from ----
//
// kVersion used to be a hand-bumped ordinal (v1, v2, ...) with one failure
// mode: whoever changed a payload forgot to bump it, and an old host happily
// misparsed the new layout. This fingerprint removes the human from the loop.
// It is a compile-time FNV-1a fold over everything that defines the wire
// format:
//
//   - sizeof of every payload: catches added/removed/retyped fields;
//   - offsetof of the position-bearing fields: catches reordering and
//     same-size repurposing that sizeof alone cannot see;
//   - every bRequest code and bmRequestType constant: catches renumbering;
//   - the coding values of the configuration enums (CanMode, UartParity, ...):
//     catches re-codings that leave the byte layout untouched.
//
// Maintenance rule, and the only one: when you change wire semantics in a way
// no listed input observes (in practice: adding a brand-new enum without
// touching any payload), fold its values in right here -- a new FIELD is
// always caught by its size or offset anyway.
constexpr uint16_t layout_fingerprint() {
    uint32_t h = 2166136261U; // FNV-1a 32-bit offset basis
    auto fold = [&h](uint32_t value) noexcept { h = (h ^ value) * 16777619U; };

    fold(sizeof(InterfacePayload));
    fold(sizeof(CanConfigPayload));
    fold(sizeof(UartConfigPayload));
    fold(sizeof(CanStatusPayload));
    fold(sizeof(LatencyBreakdownPayload));
    fold(sizeof(LastConfigErrorPayload));

    fold(__builtin_offsetof(InterfacePayload, can_fd_mask));
    fold(__builtin_offsetof(InterfacePayload, caps));
    fold(__builtin_offsetof(CanConfigPayload, control));
    fold(__builtin_offsetof(CanConfigPayload, arbitration_baudrate));
    fold(__builtin_offsetof(CanConfigPayload, data_baudrate));
    fold(__builtin_offsetof(CanConfigPayload, nominal_sample_point));
    fold(__builtin_offsetof(CanConfigPayload, data_sample_point));
    fold(__builtin_offsetof(UartConfigPayload, word_length));
    fold(__builtin_offsetof(UartConfigPayload, parity));
    fold(__builtin_offsetof(UartConfigPayload, stop_bits));
    fold(__builtin_offsetof(UartConfigPayload, control));
    fold(__builtin_offsetof(UartConfigPayload, divisor));
    fold(__builtin_offsetof(UartConfigPayload, oversample));
    fold(__builtin_offsetof(UartConfigPayload, rx_polarity));
    fold(__builtin_offsetof(CanStatusPayload, tx_occurred));
    fold(__builtin_offsetof(LatencyBreakdownPayload, cpu_hz));

    fold(static_cast<uint32_t>(Request::kGetInterface));
    fold(static_cast<uint32_t>(Request::kGetCanConfig));
    fold(static_cast<uint32_t>(Request::kSetCanConfig));
    fold(static_cast<uint32_t>(Request::kGetUartConfig));
    fold(static_cast<uint32_t>(Request::kSetUartConfig));
    fold(static_cast<uint32_t>(Request::kGetCanStatus));
    fold(static_cast<uint32_t>(Request::kGetLatencyBreakdown));
    fold(static_cast<uint32_t>(Request::kGetLastConfigError));
    fold(kRequestTypeIn);
    fold(kRequestTypeOut);

    fold(static_cast<uint32_t>(CanMode::kClassic));
    fold(static_cast<uint32_t>(CanMode::kCanFd));
    fold(kCanConfigApply);
    fold(kCanConfigApplyTiming);
    fold(kCapCanModeSettable);
    fold(kCapCanFdLongFrames);
    fold(kCapCanRateSettable);
    fold(kUartConfigApply);
    fold(static_cast<uint32_t>(UartWordLength::kUartWordLength7));
    fold(static_cast<uint32_t>(UartWordLength::kUartWordLength8));
    fold(static_cast<uint32_t>(UartParity::kUartParityNone));
    fold(static_cast<uint32_t>(UartParity::kUartParityEven));
    fold(static_cast<uint32_t>(UartParity::kUartParityOdd));
    fold(static_cast<uint32_t>(UartStopBits::kUartStopBits1));
    fold(static_cast<uint32_t>(UartStopBits::kUartStopBits2));
    fold(static_cast<uint32_t>(UartRxPolarity::kUartRxPolarityNormal));
    fold(static_cast<uint32_t>(UartRxPolarity::kUartRxPolarityInverted));
    fold(static_cast<uint32_t>(ConfigErrorReason::kConfigErrorFramingUnsupported));
    fold(static_cast<uint32_t>(ConfigErrorReason::kConfigErrorVerifyFailed));
    // UartDivisor is folded whole, not through UartConfigPayload: it is
    // embedded there as loose fields, so a change to its own layout would slip
    // past the payload's size and offsets. Same reasoning as the enum codings
    // above -- this is the "brand-new type no listed input observes" case the
    // maintenance rule names.
    fold(sizeof(UartDivisor));
    fold(__builtin_offsetof(UartDivisor, divisor));
    fold(__builtin_offsetof(UartDivisor, oversample));

    // The EP0 payloads above cannot see a change to the SESSION payloads (they
    // are defined in core/src/protocol/protocol.hpp, below this header in the
    // include graph), so the session wire layout carries its own version
    // constant -- PINNED by a static_assert in protocol.hpp to the value
    // session_layout_fingerprint() computes there, which turns "forgot to bump"
    // into a compile error -- and it rides the same gate: an old host against a
    // slimmed-TimeStatus board fails here instead of corrupting stream framing.
    fold(static_cast<uint32_t>(data::kSessionWireVersion));

    return static_cast<uint16_t>((h >> 16) ^ (h & 0xFFFFU));
}

// Same value on host and every board -- they compile this same header -- and
// different the moment the wire format moves. The 16-bit width is deliberate:
// the fingerprint only needs to separate the handful of layouts that will ever
// coexist in the wild, and it rides in the version field InterfacePayload
// already had. Build identity (which git commit produced this binary) is a
// DIFFERENT question, answered by the USB product string at discovery time.
inline constexpr uint16_t kVersion = layout_fingerprint();
static_assert(kVersion != 0, "the fingerprint folded to zero -- treated as absent");

// PSR.LEC / PSR.DLEC coding, straight from the M_CAN register [RM].
enum class LastErrorCode : uint8_t {
    kNone = 0,
    kStuff = 1,
    kForm = 2,
    kAck = 3,
    kBit1 = 4,
    kBit0 = 5,
    kCrc = 6,
    kNoChange = 7, // no new error since the register was last read
};

enum CanStatusFlags : uint8_t {
    kCanErrorPassive = 1U << 0, // PSR.EP
    kCanWarning = 1U << 1,      // PSR.EW
    kCanBusOff = 1U << 2,       // PSR.BO
};

} // namespace libhcs::core::protocol::vendor_control
