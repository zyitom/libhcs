#pragma once

#include <cstdint>

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

// Bumped whenever a payload below changes shape. The host reads it first and
// refuses a board it cannot speak to, rather than misparsing a struct.
inline constexpr uint16_t kVersion = 1;

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
    // IN, wIndex = UART index -> UartConfigPayload. The EFFECTIVE baudrate,
    // reconstructed from the divisor actually programmed.
    kGetUartConfig = 0x43,
    // OUT, wIndex = UART index <- UartConfigPayload. Stalls when the board's
    // divisor solver rejects the rate; the port keeps running at the old one.
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
};

// Direction bits of bmRequestType for the requests above.
inline constexpr uint8_t kRequestTypeIn = 0xC0;  // Device-to-host | Vendor | Device
inline constexpr uint8_t kRequestTypeOut = 0x40; // Host-to-device | Vendor | Device

enum class CanMode : uint8_t {
    kClassic = 0, // CAN 2.0
    kCanFd = 1,   // CAN-FD with bitrate switching
};

// All payloads are little-endian and byte-packed. USB is little-endian on the
// wire and every host and board in this project is too, so the structs go out
// as-is with no serialization step; the static_asserts pin the layout.
struct InterfacePayload {
    uint16_t version;    // kVersion
    uint8_t can_count;   // CAN buses this PCB actually has
    uint8_t uart_count;  // UART ports this image serves
    uint8_t can_fd_mask; // bit i: bus i runs CAN-FD
    uint8_t reserved[3];
};
static_assert(sizeof(InterfacePayload) == 8);

// The mode is reported and validated, never applied: bringing a controller up
// costs a heavily tuned mcan_init (sample point pinned to 87.5%, transmitter
// delay compensation, an external PTPC timebase feeding the TSU, sync filters),
// and re-running it live would take the bus down and put every one of those
// back at risk. So the board keeps its compile-time CanPort::mode and stalls a
// SET that disagrees with it -- which is the check the host needs, because the
// only thing a host can do wrong here is assume the wrong mode.
struct CanConfigPayload {
    uint8_t mode; // CanMode
    uint8_t reserved[3];
};
static_assert(sizeof(CanConfigPayload) == 4);

struct UartConfigPayload {
    uint32_t baudrate; // requested on SET, effective on GET
    uint32_t reserved;
};
static_assert(sizeof(UartConfigPayload) == 8);

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
    uint32_t tx_occurred;   // TXBTO: transmissions that actually completed
    uint32_t tx_cancelled;  // TXBCF: transmissions abandoned (this driver runs
                            // with automatic retransmission disabled)
    uint32_t rx_frames;     // frames forwarded to the host since boot
    uint32_t rx_fifo_level; // RXF0S fill level, non-zero means a backlog
};
static_assert(sizeof(CanStatusPayload) == 24);

// Which pipe the board sends CAN uplink on.
//
// The second bulk pipe is not free: the host has to post idle IN URBs on it,
// and those are polled and NAKed continuously, taking scheduling slots from the
// data pipe. Measured on hpm5321: 49466 packets/s with the pipe, 63333 without
// -- 28%, against a per-device ceiling of about 64000.
//
// Whether that is worth paying depends on the load, and it is a genuine
// crossover rather than a strict win: the second pipe keeps a CAN frame from
// queueing behind a UART batch, worth about 21 us of p99 at UART line rate, but
// costs about 11 us of p99 when UART is idle (firmware/hpm_board/AGENTS.md).
// So it is a runtime choice, not a build-time one.
//
// BOTH SIDES MUST AGREE. A host that stops listening on the second pipe while
// the board still transmits there loses every CAN frame, silently -- measured,
// 8000 of 8000 round trips timed out. This request is what keeps them in step.
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
