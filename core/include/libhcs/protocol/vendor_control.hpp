#pragma once

#include <cstdint>
#include <initializer_list>

#include <libhcs/data/datas.hpp>
#include <libhcs/spec/port.hpp>

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
// Not gated on a session. Configuration is transport state, not data-plane
// state -- the host applies it before the first keepalive has opened a session,
// and a board must answer these while its data plane is idle.
//
// CONFIGURATION IS DECLARATION, ONE TRANSACTION. A port the host never declared
// is a port the host does not use, and the board keeps it off: the host sends
// the ports it will use -- and only those -- in one kApplyManifest. The board
// first validates every entry (no hardware touched), then applies them; a
// fully applied manifest starts exactly the declared ports, with every other
// port left off, and moves board ownership to libhcs. A manifest that fails
// validation changes nothing; one that fails at apply time leaves every port
// suspended. On the two-phase commit and the per-kind payloads, see the v9 ->
// v10 history and the payload comments below.
//
// Data a host sends to a stopped channel is dropped by the board; the data
// stream has no way to say so. The libhcs board classes therefore refuse the
// transmit themselves (hcs::UndeclaredChannel, host/include/libhcs/board/
// hcs_config.hpp), and the board's drop is the backstop for any other host.
//
// Per board (the difference is one policy parameter in each firmware's EP0
// glue, not three copies of the logic):
//   mc02        ports are off from power-up and only a manifest entry starts
//               one; a stopped UART has no DMA stream and no interrupt armed,
//               a stopped FDCAN stays in INIT.
//   hpm_board   ports run while no libhcs host owns the board (the board is
//               then a DMTool adapter and CDC serial bridge, which do not
//               speak EP0); a fully applied manifest takes ownership, starts
//               exactly the declared ports and leaves the rest suspended.
//   c_board     ports always run (the board has no extras to serve): manifest
//               entries are assertions, undeclared ports are not stopped.
//
// The on-board IMU is a port like the others (DataId::kImu): a manifest entry
// for it is its declaration. On mc02 the sensor is not even initialised until
// a manifest that names it is applied -- no data-ready interrupt, no SPI read,
// nothing polled -- and the entry carries the ranges and rates the host will
// scale the samples with, so the two ends cannot disagree about them. c_board's
// IMU always runs (not converted) and accepts the entry only as an assertion
// of the setting it was built with.
//
// The GPIO header is a port too (DataId::kGpio), declared one LINE at a time:
// each line's manifest entry says whether it is an output or an input and how
// the input is sampled (GpioConfigPayload). A line is an address within the
// port, not a port of its own -- the same relation as a motor id on a CAN bus.
// An undeclared line is not touched -- no timer channel started, no EXTI line
// armed, no sampling in the main loop -- on every board, c_board included:
// unlike its CAN/UART, a line has no running state to keep for anyone. A
// suspended output is held low (compare 0), not floated, so an ESC on it sees
// the signal stop rather than noise.
namespace libhcs::core::protocol::vendor_control {

// The wire-layout version the host and board compare: the board reports it in
// PortListPayload (kGetPortList), the host carries it in ManifestPayload
// (kApplyManifest), and either side refuses a mismatch. Before v10 the check
// was at kGetInterface, which the history below still names. NOT a
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
//   v6 -> v7 (2026-10-02, by hand): no layout moved. kSetCanConfig and
//   kSetUartConfig became the per-channel handshake -- a channel now runs only
//   after the host configured it, and kGetInterface stops every channel first
//   (see "CONFIGURATION IS DECLARATION" above). A v6 host that relied on
//   unconfigured channels working would open a session and hear nothing, so
//   kChannelDeclarationSemantics is folded in to make it refuse at
//   kGetInterface instead.
//
//   v7 -> v8 (2026-10-03, automatic): the on-board IMU joined the declaration.
//   InterfacePayload's first reserved byte became `imu_count`, and
//   kSetImuConfig / ImuConfigPayload are new. The mc02 compile switch
//   libhcs_APP_IMU_ENABLE is gone with it: whether the IMU runs is the host's
//   declaration, not a property of the image.
//
//   v8 -> v9 (2026-10-03, by hand + automatic): timestamps moved onto the
//   shared USB microframe axis. The CAN record's trailing timestamp shrank
//   from a 32-bit microsecond count of a board-local clock to a 24-bit
//   libhcs::time::SofStamp (kCanStampSemantics, folded by hand: the record
//   stream is defined below this header), and kTimeStatus now reports a
//   fractional microframe plus the hardware-capture counters (the session
//   fingerprint moves by itself). An old host would read three stamp bytes as
//   four and lose framing on the first timestamped frame.
//
//   v9 -> v10 (2026-10-04, the port-architecture upgrade): ports are addressed
//   by DataId, described by one list, and declared by one transaction.
//   Per-type requests (kGet/SetCanConfig, kGet/SetUartConfig, kGetCanStatus,
//   kSetImuConfig) are retired; their wIndex used to be a position ("the i-th
//   CAN bus") that the host re-offset to print silkscreen names, which was
//   already wrong on hpm6e8y (silk starts at CAN0, so its errors printed one
//   high). Their replacements take wIndex = the DataId itself -- the same
//   identity the bulk stream already uses -- so no port has two names again:
//
//     kGetPortList (IN, wIndex 0)   what this PCB actually has, per port
//                                   {DataId, kind, capabilities, status}.
//                                   PURE READ: the board-ownership switch and
//                                   the declaration round it used to trigger
//                                   moved to kApplyManifest.
//     kGetPortConfig / kGetPortStatus (IN, wIndex = DataId)
//                                   the per-kind payloads below, addressed by
//                                   identity. Unknown DataId, or a kind other
//                                   than the request's, stalls with the reason
//                                   latched.
//     kApplyManifest (OUT)          the whole declaration in one transfer:
//                                   {DataId, kind, setting} per port the host
//                                   will use. Two-phase commit -- every entry
//                                   is validated (no hardware touched) before
//                                   anything is applied; one bad entry leaves
//                                   ALL ports and the ownership as they were.
//                                   Validation covers every foreseeable refusal
//                                   (CAN bit-timing solving included), so what
//                                   can still fail while applying is a hardware
//                                   read-back: then every port is suspended
//                                   and the board goes back to its previous
//                                   owner (the DMTool/CDC extras get their
//                                   ports back). libhcs takes the board from
//                                   the extras before touching a port, so the
//                                   two never see each other's half-state, and
//                                   keeps it once the manifest is fully
//                                   applied -- the handshake-to-declaration
//                                   window in which CDC's SET_LINE_CODING could
//                                   rewrite a just-declared UART is gone too.
//     kGetManifestResult (IN)       per-entry outcome of the last manifest
//                                   (reason + offending value) and which of
//                                   refused / applied / rolled back it was, so
//                                   a refused declaration is diagnosable
//                                   without a second transfer per port.
//
//   Also folded since v10: the ports' DataId values (they are wIndex now) and
//   ManifestOutcome.
//
//   The old per-type payloads (CanConfigPayload, UartConfigPayload,
//   ImuConfigPayload) are unchanged: they are the per-kind settings, now
//   carried inside manifest entries and read back through kGetPortConfig.
//   kChannelDeclarationSemantics is superseded by kManifestSemantics (both
//   folded): a v9 host's kSetCanConfig sequence would declare nothing on a v10
//   board, and a v10 manifest is unreadable to a v9 board.
//
//   v10 -> v11 (2026-10-04, GPIO joins the port model): every GPIO pin is a
//   port with a DataId of its own (kPwm1..kPwm7), declared in the manifest
//   (PortKind::kGpio, GpioConfigPayload) and off until then. Before, the pins
//   were the one thing outside the v10 model: addressed by a 6-bit channel
//   index (a position), configured by write-only read-config records in the
//   bulk stream that a board could only refuse silently, and started at
//   power-up whether anyone used them or not. The record stream changed with
//   it (kGpioRecordSemantics): a GPIO record's field id is the pin's DataId,
//   its header lost the channel index and the read-config payloads, and
//   kRead (read once now) is all that is left of the read side downlink.
//   kMaxPorts went 10 -> 14 (mc02 3 CAN + 6 UART + IMU + 4 pins; c_board 13).
//
//   Also in v11, every declaration is COMPLETE (kCompleteDeclarationSemantics):
//   a manifest entry states everything about how its port runs, and a zero in
//   a required field is refused with kConfigErrorIncomplete (value = the
//   field's byte offset in the setting) instead of meaning "keep the firmware's
//   current value". What the firmware currently has may be a .ioc placeholder,
//   a previous host's choice, or a CDC rewrite made outside any session (the
//   host OS has set a UART to 9596 baud) -- none of which the host can see.
//   Required: CAN kCanConfigApply, arbitration_baudrate, and data_baudrate on
//   an FD bus; UART kUartConfigApply, baudrate, word_length, parity, stop_bits
//   and rx_polarity; all five IMU fields; GPIO mode (already). Assertion-only
//   values stay optional: the CAN sample points and the UART divisor/oversample.
//
//   v11 -> v12 (2026-10-05, the GPIO header is ONE port): v11 gave every pin
//   a DataId of its own (kPwm1..kPwm7), which made the DataId enum the union of
//   every board's pin names and put kPwm2.. behind the two-byte field header.
//   Now the header is one port, DataId::kGpio, and a pin is a line of it -- an
//   address within the port, like a motor id on a CAN bus:
//     - stream: one compact field id, kGpio = 1, with the record type and the
//       timestamp bit in the header nibble and the line in the second byte
//       (kGpioRecordSemantics 2): two bytes for a level on every line, read
//       the same way in both directions;
//     - EP0: wIndex = DataId | line << 8 (the high byte is 0 for every other
//       kind); a manifest entry and a result entry carry the line; one entry
//       per declared line; the port list has one GPIO row whose capability
//       byte is the line count (per-line capabilities stay on the board, which
//       refuses with the missing bits);
//     - limits: kMaxPorts 14 -> 11 (port list rows), kMaxManifestEntries = 14
//       (declarations: mc02 10 ports + 4 lines), kMaxGpioLines = 8.
//   Also in v12, the on-board buzzer is a port (DataId::kBuzzer = 15,
//   PortKind::kBuzzer, BuzzerConfigPayload): declared with no parameters, it
//   starts silent; a downlink record (one header byte + 2-byte frequency +
//   loudness, kBuzzerRecordSemantics) sets the tone. kMaxPorts 12,
//   kMaxManifestEntries 15. Every DataId fits the compact field header.
//
//   v12 -> v13 (2026-10-05, by hand): the CAN record's 24-bit SofStamp keeps
//   its width but moves its binary point: 10 microframe bits + 14 fraction
//   bits (7.6 ns, 128 ms span) instead of 14 + 10 (122 ns, 2.048 s). The
//   10-bit fraction's quantisation was the largest error on the whole
//   timestamp path. (An 11 + 13 split was written first the same day and
//   never flashed; v13 means 10 + 14.) No layout moved, so kCanStampSemantics is bumped by hand:
//   a v12 host would read every stamp eight times too coarse and wrap it at
//   the wrong span.
//
//   v13 -> v14 (2026-10-05, by hand): the shared USB-SOF time base is
//   declared, not compiled in. Every image carries it; kGetPortList's
//   board_caps says whether this board has one (kBoardCapTimeSync) and the
//   manifest header's flags byte (was reserved) asks for it
//   (kManifestFlagTimeSync). The board starts it inside the manifest
//   transaction -- before the ports, so the first frame after acceptance can
//   already be stamped -- stops it on rollback, and stops it again when the
//   session ends; undeclared, it costs nothing (no SOF interrupt). It used to
//   be the build switch libhcs_TIME_SYNC plus a matching host option that had
//   to be kept in step by hand. No layout moved: kTimeSyncDeclarationSemantics.
//
//   v14 -> v15 (2026-10-06, by hand): a new session record, kStreamError, rides
//   after a keepalive ack whenever the board recorded a downlink error since
//   the previous report (data::StreamErrorView: the running count plus the
//   field, reason and transfer of the most recent one). The session payload
//   layout moved, so kSessionWireVersion changed and this fingerprint moved
//   with it; the pin at the end of host/tests/ep0_declaration_test.cpp names
//   the reflash.
//
//   v15 -> v16 (2026-10-06): runtime port status leaves EP0. kGetPortStatus
//   (0x4C, CanStatusPayload) is retired; the board pushes each running CAN /
//   UART port's status as a session record after the keepalive ack instead
//   (data::SessionType::kPortStatus, core/src/link/port_status.hpp): one per
//   port whose status changed, a port/length byte plus a kind-defined body,
//   at most 16 bytes. EP0 is the configuration plane only:
//   a control transfer is scheduled ahead of bulk and is a synchronous round
//   trip on the host, so polling it at run time is a jitter source that grows
//   with the poll rate and the port count. UART receive errors are reported for
//   the first time (they had no status at all). v15's kStreamError folds into
//   the same record as the link's own status (data::LinkStatusView, port
//   DataId::kSession). The session payload layout moved, so both fingerprints
//   moved with it.
//
// The fingerprint itself lives at the bottom of this file, after every payload
// it folds over.

// bRequest codes. Sent as vendor requests with the DEVICE recipient: TinyUSB
// routes every vendor-type request to tud_vendor_control_xfer_cb regardless of
// recipient, so nothing is gained by addressing an interface, and a device
// recipient keeps wIndex free for the port's DataId.
//
// Every port-addressed request takes wIndex = the port's DataId (data::DataId's
// numeric value) in the low byte and, for the GPIO port, the line in the high
// byte (wIndex = DataId::kGpio | line << 8); every other kind takes 0 there. A
// wIndex that names no port (or line) on this board -- or one whose kind does
// not match the request -- stalls, with the reason latched for
// kGetLastConfigError (whose index field carries the same wIndex).
enum class Request : uint8_t {
    // The per-type requests below were retired by the v10 upgrade (2026-10-04,
    // see the history). The codes are not recycled: a stale host must stall,
    // not have its requests silently reinterpreted as later ones.
    //
    //   0x40 kGetInterface  -- superseded by kGetPortList + kApplyManifest;
    //                          it was also the ownership switch and the start
    //                          of a declaration round, both of which moved to
    //                          kApplyManifest.
    //   0x41 kGetCanConfig  -- kGetPortConfig, wIndex = DataId::kCanN.
    //   0x42 kSetCanConfig  -- folded into kApplyManifest.
    //   0x43 kGetUartConfig -- kGetPortConfig, wIndex = DataId::kUartN.
    //   0x44 kSetUartConfig -- folded into kApplyManifest.
    //   0x45 kGetCanStatus  -- kGetPortStatus (itself retired in v16, below).
    //   0x49 kSetImuConfig  -- folded into kApplyManifest.

    // IN, wIndex = 0 -> PortListPayload. What this PCB actually has: one entry
    // per present port, {DataId, kind, capabilities, current status}. PURE
    // READ: no ownership change, no channel stops, no latched state moves.
    // The host reads it before declaring, so a wiring that names a port the
    // PCB lacks fails with the port's name instead of a board-side stall.
    kGetPortList = 0x4A,
    // IN, wIndex = DataId (| line << 8 for GPIO) -> the port's kind's config
    // payload (CAN / UART / IMU / one GPIO line, see the payload structs
    // below). Hardware truth, never the last request.
    kGetPortConfig = 0x4B,
    // 0x4C was kGetPortStatus (CAN error registers over EP0), retired in v16:
    // runtime status rides the stream (data::SessionType::kPortStatus). Not
    // recycled, for the same reason as the codes above.
    // OUT <- ManifestPayload. The whole declaration, one transaction. See the
    // v9 -> v10 history and ManifestPayload for the two-phase commit; board
    // ownership moves to libhcs exactly when the manifest is fully applied.
    kApplyManifest = 0x4D,
    // IN, wIndex = 0 -> ManifestResultPayload. The per-port outcome of the
    // most recent kApplyManifest, in the manifest's own order.
    kGetManifestResult = 0x4E,
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
    // the same latch-and-read pattern as a CAN controller's PSR.LEC.
    // The `index` field carries the refused request's wIndex: a DataId for the
    // port-addressed requests, 0 for the board-wide ones.
    kGetLastConfigError = 0x48,
};

// Direction bits of bmRequestType for the requests above.
inline constexpr uint8_t kRequestTypeIn = 0xC0;  // Device-to-host | Vendor | Device
inline constexpr uint8_t kRequestTypeOut = 0x40; // Host-to-device | Vendor | Device

// All payloads are little-endian and byte-packed. USB is little-endian on the
// wire and every host and board in this project is too, so the structs go out
// as-is with no serialization step; the static_asserts pin the layout.

// A manifest entry carries the largest per-kind setting, so one entry size
// serves all kinds and the board can bounds-check the payload per entry.
// (The static_asserts that pin the four settings under this bound sit with
// GpioConfigPayload, the last of them.)
inline constexpr std::size_t kManifestSettingCapacity = 20;

// ---- kGetPortList: what this PCB actually has ----
//
// One entry per PRESENT port, in the board spec's order. The spec describes the
// image's capacity (the hpm5321 image serves both the single- and dual-CAN
// PCBs); the list is the runtime truth, built from each port's own presence
// query. Pure read, no side effects.
struct PortListEntry {
    uint8_t data_id;      // data::DataId -- the port's only identity
    uint8_t kind;         // spec::PortKind
    uint8_t capabilities; // static bits for this kind (spec::CanCapability, ...);
                          // for the GPIO port, its line count
    uint8_t status;       // spec::PortStatusFlags -- how the port is RIGHT NOW
};
static_assert(sizeof(PortListEntry) == 4);

// Board-level abilities, kGetPortList's board_caps: what the board can do as a
// whole rather than on one port. A bit the board does not report cannot be
// asked for (the manifest that does is refused).
enum BoardCapability : uint8_t {
    // The shared USB-SOF time base: CAN frames carry a SofStamp and the board
    // answers kTimeAnchor (firmware/hpm_board/SOF_TIMEBASE.md). Asked for with
    // kManifestFlagTimeSync.
    kBoardCapTimeSync = 1U << 0,
};

struct PortListPayload {
    uint16_t version;                     // kVersion
    uint8_t port_count;                   // entries that follow
    uint8_t board_caps;                   // BoardCapability bits
    uint16_t reserved;
    PortListEntry ports[spec::kMaxPorts]; // only port_count entries are sent
};
static_assert(sizeof(PortListPayload) == 6 + 4 * spec::kMaxPorts);

// What a board lets the host do to its buses is per port, in kGetPortList's
// `capabilities` byte: spec::CanCapability (spec/port.hpp) is the one encoding.
// A capability the board does not advertise is not a softer setting --
// requesting it stalls. In short: kCanCapModeSettable = the host may switch a
// bus's TX frame type (mc02 flips the Tx-element FDF/BRS flags, hpm_board
// re-initializes the controller with FD off for classic); kCanCapFdLongFrames =
// the board carries 12-64 byte payloads both ways on an FD bus (the host SDK
// gates long frames on it); kCanCapRateSettable = the host may SET the rates
// (kCanConfigApplyTiming) and the board solves its bit timing at its own
// sample-point policy, otherwise the rate fields are pure assertions.

enum class CanMode : uint8_t {
    kClassic = 0, // CAN 2.0
    kCanFd = 1,   // CAN-FD with bitrate switching
};

// One CAN bus's setting: frame type plus both rates. It is a wiring fact (the
// motors on the bus decide it), written in host code and declared through the
// manifest; the board's CAN drivers take the same type. A declaration gives the
// arbitration rate, and the data rate on an FD bus (a classic bus has no data
// phase: data_baudrate must be 0). Sample point and TDC are the board's own
// policy and not part of it.
struct CanSetting {
    bool fd = false;
    uint32_t arbitration_baudrate = 0;
    uint32_t data_baudrate = 0;

    friend constexpr bool operator==(const CanSetting&, const CanSetting&) = default;
};

// What the host asks of one CAN bus, and what the board answers about it.
// This is the CAN port's setting: it rides a manifest entry down (as the
// declaration) and comes back up through kGetPortConfig (as hardware truth).
//
// Two kinds of fields, and the difference is load-bearing:
//
//   mode + control   the REQUEST. control bit kCanConfigApply asks the board
//                    to switch the bus's TX frame type to `mode`. Boards that
//                    carry spec::kCanCapModeSettable (mc02, hpm_board) apply it;
//                    boards that do not (c_board) accept only `mode` equal to
//                    the mode they run. A manifest entry without
//                    kCanConfigApply is refused: a declaration is a setting.
//
//   rate fields       arbitration_baudrate / data_baudrate. The rates are a
//                    wiring fact -- fixed by the bus peers' hardware, which
//                    the HOST code knows and the firmware does not. With
//                    kCanConfigApplyTiming (boards carrying
//                    kCanCapRateSettable) they are the setting. Without it
//                    they are an assertion that must equal the board's. Either
//                    way they are required: arbitration_baudrate always,
//                    data_baudrate on an FD bus (kConfigErrorIncomplete
//                    otherwise). A classic bus has no data phase: declaring a
//                    non-zero data_baudrate with mode = kClassic is refused.
//
//   sample points     always the ASSERTION. The sample point is a measured,
//                    board-specific policy (pinned to 87.5 per mille, so every
//                    node on the bus samples at the same place), never set by
//                    the host; zero means "no opinion, skip the check".
//
// kGetPortConfig returns hardware truth, not the last request: mode is what
// the bus transmits right now, the rates are what the controller is actually
// timed for (data_baudrate is the FD data phase the receiver runs at: mc02 keeps
// its controller FD-capable while transmitting classic and reports it
// regardless of `mode`; hpm_board turns FD off for classic and reports 0,
// as does c_board -- a timing assertion on a data phase the bus does not
// have is a wrong expectation), the sample points are the achieved
// per-mille positions of both phases, and control is zero.
struct CanConfigPayload {
    uint8_t mode;                  // CanMode
    uint8_t control;               // CanConfigControl, SET only; GET returns 0
    uint16_t reserved0;
    uint32_t arbitration_baudrate; // required in a declaration
    uint32_t data_baudrate;        // required on an FD bus, 0 on a classic one
    uint16_t nominal_sample_point; // per-mille; 0 = skip; else must equal the board's
    uint16_t data_sample_point;    // per-mille; 0 = skip; else must equal the board's
    uint32_t reserved1;
};
static_assert(sizeof(CanConfigPayload) == 20);

enum CanConfigControl : uint8_t {
    // Apply `mode` to the bus if the board advertises spec::kCanCapModeSettable.
    // Without this bit the SET only asserts that the bus already runs `mode`.
    kCanConfigApply = 1U << 0,
    // Apply the rate fields instead of asserting them, on boards advertising
    // spec::kCanCapRateSettable (see the rate fields above). A board that cannot
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
    // Apply the payload. Required in a manifest entry: a declaration is a
    // setting, not an assertion about what the port happens to run.
    kUartConfigApply = 1U << 0,
};

// The divisor a port is actually programmed with, and the oversampling it is
// programmed at. These are the integers the rate is made of, and the ONLY
// thing a rate agreement can be checked against:
//
// WHY NOT THE BAUDRATE. The host cannot reconstruct the board's divisor: it
// does not know the board's UART kernel clock (hpm 80 MHz; mc02 STM32H7,
// resolved at run time from PLL2Q/PLL3Q/HSI by UART_GETCLOCKSOURCE; c_board
// 42 or 84 MHz off APB1/APB2). And the requested rate is not a
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

// One UART port's configuration -- the UART port's setting, the same way
// CanConfigPayload is the CAN bus's. It rides a manifest entry down and comes
// back through kGetPortConfig. A manifest entry carries kUartConfigApply and
// gives the rate and the whole framing -- baudrate, word_length, parity,
// stop_bits, rx_polarity -- so the port runs exactly what the host said, never
// a leftover. divisor/oversample stay optional assertions (0 = no opinion).
// kGetPortConfig returns hardware truth, never the last
// request: the EFFECTIVE baudrate reconstructed from the divisor actually
// programmed, the framing decoded from the live registers, and control = 0.
//
// Flow control (RTS/CTS) has no field here on purpose: it is pinned by the
// pins the .ioc routes at boot, a hardware deployment fact no run-time
// register write can change. The kernel clock source is likewise an init-time
// fact -- it defines what the divisor means, and the board's read-back already
// reflects it. The OVERsampling is not in that category: it is reported, as
// UartDivisor::oversample, because it is half of what turns a divisor into a
// rate.
struct UartConfigPayload {
    uint32_t baudrate;   // SET: requested rate (required); GET: effective rate
    uint16_t divisor;    // UartDivisor::divisor -- GET: programmed; SET: asserted
    uint8_t oversample;  // UartDivisor::oversample -- GET: programmed; SET: asserted
    uint8_t word_length; // UartWordLength
    uint8_t parity;      // UartParity
    uint8_t stop_bits;   // UartStopBits
    uint8_t control;     // UartConfigControl, SET only; GET returns 0
    uint8_t rx_polarity; // UartRxPolarity
};
static_assert(sizeof(UartConfigPayload) == 12);

// The on-board IMU's setting, in physical units rather than one sensor's
// register codes: the host needs the ranges to scale the raw samples, so they
// are stated the way the host uses them. Every field is required (zero is
// refused with kConfigErrorIncomplete) and must be a value the sensor offers,
// else the request stalls with kConfigErrorRateUnrepresentable and the IMU
// stays stopped.
//
//   BMI088 accelerometer   range 3 / 6 / 12 / 24 g
//                          rate  12 / 25 / 50 / 100 / 200 / 400 / 800 / 1600 Hz
//   BMI088 gyroscope       range 125 / 250 / 500 / 1000 / 2000 deg/s
//                          rate / filter bandwidth, ONE setting given as a
//                          pair: 2000/532, 2000/230, 1000/116, 400/47, 200/23,
//                          100/12, 200/64, 100/32 Hz
//
// There is no apply bit and no read-back request: the request IS the
// declaration, the board verifies each sensor register before it ACKs, and an
// ACK therefore means "running exactly this".
struct ImuConfigPayload {
    uint16_t accelerometer_rate_hz;
    uint16_t gyroscope_rate_hz;
    uint16_t gyroscope_bandwidth_hz;
    uint16_t gyroscope_range_dps;
    uint8_t accelerometer_range_g;
    uint8_t reserved[3];
};
static_assert(sizeof(ImuConfigPayload) == 12);

// One GPIO line's setting. The direction is the declaration; everything else
// describes how an input is sampled and must be zero on an output.
enum GpioMode : uint8_t {
    // GET only: the line is not declared and is idle (an output held low, or an
    // input with nothing armed). A manifest entry with it is refused.
    kGpioModeOff = 0,
    // Timer-driven output: kGpioCapDigitalWrite takes high/low records,
    // kGpioCapAnalogWrite takes PWM duty records. Starts low.
    kGpioModeOutput = 1,
    // Input. Sampled on request (kRead records), every `period_ms`, and/or on
    // the edges in input_flags -- each one needing its capability bit.
    kGpioModeInput = 2,
};
enum GpioPull : uint8_t {
    kGpioPullNone = 0,
    kGpioPullUp = 1,
    kGpioPullDown = 2,
};
enum GpioInputFlags : uint8_t {
    kGpioInputRisingEdge = 1U << 0,  // sample on a rising edge (kGpioCapReadInterrupt)
    kGpioInputFallingEdge = 1U << 1, // sample on a falling edge (kGpioCapReadInterrupt)
    kGpioInputTimestamp = 1U << 2,   // samples carry the board's timestamp
};

// Like ImuConfigPayload, there is no apply bit: the entry IS the declaration,
// the board checks every field against the line's capabilities (kGpioCap*, a
// wiring fact of the board) before touching anything, verifies the line's mode
// and pull registers after writing them, and kGetPortConfig returns what the
// line runs. A line the manifest redeclares with the same setting keeps its
// output value:
// a runtime redeclaration of some other port replays the whole manifest, and
// that must not glitch a running PWM.
struct GpioConfigPayload {
    uint8_t mode;        // GpioMode
    uint8_t pull;        // GpioPull; input only
    uint8_t input_flags; // GpioInputFlags; input only
    uint8_t reserved0;
    uint16_t period_ms;  // periodic sampling period, 0 = none; input only
    uint16_t reserved1;
};
static_assert(sizeof(GpioConfigPayload) == 8);

// The buzzer (DataId::kBuzzer). The declaration carries nothing -- every field
// must be zero: a declared buzzer starts silent and plays what the record stream
// says (BuzzerTonePayload, core/src/protocol/protocol.hpp). kGetPortConfig
// reports the tone it plays right now, reconstructed from the timer registers.
struct BuzzerConfigPayload {
    uint16_t frequency_hz; // GET: the tone playing now, 0 = silent
    uint8_t loudness;      // GET: 0..255, 255 = 50% duty
    uint8_t reserved;
};
static_assert(sizeof(BuzzerConfigPayload) == 4);
static_assert(sizeof(CanConfigPayload) <= kManifestSettingCapacity);
static_assert(sizeof(UartConfigPayload) <= kManifestSettingCapacity);
static_assert(sizeof(ImuConfigPayload) <= kManifestSettingCapacity);
static_assert(sizeof(GpioConfigPayload) <= kManifestSettingCapacity);
static_assert(sizeof(BuzzerConfigPayload) <= kManifestSettingCapacity);

// ---- kApplyManifest: the whole declaration, one transaction ----
//
// The host names every port it will use -- and only those -- as {DataId, kind,
// setting} entries. The board's two-phase commit:
//
//   phase 1, validate  every entry is checked (port exists and kind matches,
//                      capability bits satisfied, setting representable) with
//                      NO hardware touched. Any failure latches the reason and
//                      stalls: the board, its ports and its ownership are all
//                      exactly as they were.
//   phase 2, apply     every present port is driven to its manifest state --
//                      a declared one is configured, verified by read-back and
//                      started; an undeclared one is left off. A port whose
//                      apply or read-back fails rolls the whole manifest back:
//                      all ports suspended, reason latched, ownership NOT
//                      moved.
//   accepted           ownership moves to libhcs and the session gate opens.
//                      This is the single switch -- there is no other
//                      handshake, so no window exists between "the board
//                      serves libhcs" and "the declared settings are in force".
//
// Entries are fixed-length (the largest per-kind setting's size) so the
// manifest is bounds-checkable per entry; the payload length is 4 + n * entry.
// A manifest for the largest board (mc02, 14 ports) is 340 bytes: the data
// stage spans multiple EP0 packets, which TinyUSB's control machinery handles
// by chunking into CFG_TUD_ENDPOINT0_BUFSIZE-sized transactions and
// accumulating into the one staging buffer the board hands it.
struct ManifestEntry {
    uint8_t data_id; // data::DataId
    uint8_t kind;    // spec::PortKind -- must match the port's own
    uint8_t line;    // the GPIO port's line this entry declares; 0 for every other kind
    uint8_t reserved;
    // The per-kind setting, laid out exactly as the kind's config payload
    // (CanConfigPayload for a CAN port, and so on). Unused trailing bytes are
    // zero.
    uint8_t setting[kManifestSettingCapacity];
};
static_assert(sizeof(ManifestEntry) == 4 + kManifestSettingCapacity);

// Board-wide requests riding the manifest header -- the counterpart of
// kGetPortList's board_caps. They are not ports (no DataId, no record stream
// of their own), so they are not entries. Same transaction as the entries:
// started before the ports are applied, stopped if the manifest rolls back;
// a manifest without the bit stops it. An unknown bit, or one whose ability
// the board does not report, refuses the whole manifest (kConfigErrorBadRequest).
enum ManifestFlag : uint8_t {
    kManifestFlagTimeSync = 1U << 0, // needs kBoardCapTimeSync
};

struct ManifestPayload {
    uint16_t version;                                 // kVersion -- checked before anything else
    uint8_t entry_count;
    uint8_t flags;                                    // ManifestFlag bits
    ManifestEntry entries[spec::kMaxManifestEntries]; // only entry_count entries arrive
};
static_assert(sizeof(ManifestPayload) == 4 + sizeof(ManifestEntry) * spec::kMaxManifestEntries);

// ---- kGetManifestResult: per-port outcome of the last manifest ----
//
// Entries in the manifest's own order; one per entry the manifest got through
// to. The outcome byte says which of the transaction's three endings this was,
// because what the board's ports look like afterwards differs between them:
enum class ManifestOutcome : uint8_t {
    // Refused while validating (or not parsed): no hardware was touched, every
    // port and the board's ownership are exactly as before the request. The
    // failing entry carries the reason; earlier entries read kConfigErrorNone
    // and later ones are absent.
    kManifestRefused = 0,
    // Every entry took effect; undeclared ports are off; the board is libhcs's.
    kManifestApplied = 1,
    // Validation passed but applying an entry failed (a hardware read-back
    // that disagreed). Every port is suspended and the board went back to
    // whoever had it before the manifest: the extras (DMTool, CDC bridge) get
    // their ports back, a board already serving libhcs stays with libhcs with
    // nothing on the bus until the next accepted manifest. The failing entry
    // carries the reason.
    kManifestRolledBack = 2,
};

struct ManifestResultEntry {
    uint8_t data_id; // data::DataId, echoed from the manifest entry
    uint8_t reason;  // ConfigErrorReason
    uint8_t line;    // echoed from the manifest entry
    uint8_t reserved;
    uint32_t value;  // offending value, same convention as LastConfigErrorPayload
};
static_assert(sizeof(ManifestResultEntry) == 8);

struct ManifestResultPayload {
    uint8_t entry_count; // entries reported
    uint8_t outcome;     // ManifestOutcome
    uint16_t reserved;
    ManifestResultEntry entries[spec::kMaxManifestEntries];
};
static_assert(
    sizeof(ManifestResultPayload) == 4 + sizeof(ManifestResultEntry) * spec::kMaxManifestEntries);

// Why a refused configuration request deserves a request of its own: a STALL
// cannot carry data (the status stage is zero-length by USB definition), so
// "the board refused" is all the wire ever says. The board therefore LATCHES
// why -- request code, port (DataId), a reason from the enum below, and the
// offending value -- and the host reads this after any failure, which turns
// "rate outside the solver's reach OR framing unsupported" guesses into an
// exact answer. Sticky until the next refusal overwrites it; cleared only by
// a reboot. The same latch-and-read pattern as the boards' own CAN error
// registers (PSR.LEC).
enum ConfigErrorReason : uint8_t {
    kConfigErrorNone = 0,
    kConfigErrorBadRequest = 1, // unknown bRequest, control bits or manifest encoding
    kConfigErrorBadIndex = 2,   // wIndex names no port on this board
    // The port cannot do what was asked: a CAN TX frame type the bus does not
    // support, or a GPIO direction/pull/sampling the pin has no capability
    // for (value = the missing kGpioCap* bits).
    kConfigErrorUnsupportedMode = 3,
    kConfigErrorModeFixed = 4,           // the board does not let the host change the mode
    kConfigErrorRateUnrepresentable = 5, // the divisor solver cannot produce this rate
    kConfigErrorFramingUnsupported = 6,  // word length / parity / stop bits unsupported
    kConfigErrorKindMismatch = 8,        // wIndex names a port, but not of the requested kind
    // A declaration left a required field zero (value = the field's byte
    // offset in the setting): declarations are complete, see v10 -> v11.
    kConfigErrorIncomplete = 9,
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
    uint16_t index;  // wIndex of the refused request: a DataId for the
                     // port-addressed requests, 0 for the board-wide ones
    uint32_t value;  // offending rate for timing rejections; 0 otherwise
    uint32_t reserved;
};
static_assert(sizeof(LastConfigErrorPayload) == 12);

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
//
// kChannelDeclarationSemantics is that rule applied to a change with no new
// symbol at all: what an accepted kSet*Config MEANT (see "CONFIGURATION IS
// DECLARATION" at the top of this file). v10 superseded it with the manifest
// transaction; both stay folded so the fingerprints of every era differ.
inline constexpr uint32_t kChannelDeclarationSemantics = 1;

// The v10 rule case: what a declaration IS (one kApplyManifest transaction, a
// board-wide atomic, ownership moving at acceptance) and what wIndex MEANS (a
// DataId, not a position). Both are semantics no payload layout observes.
inline constexpr uint32_t kManifestSemantics = 1;

// The same rule for the record stream, which this header cannot see: what the
// bytes after a CAN payload are when HasTimestamp is set. 1 = a 24-bit
// libhcs::time::SofStamp (core/src/protocol/protocol.hpp, CanStampLayout)
// with 14 microframe bits and a 10-bit fraction; 2 = the same 24 bits split
// 10 + 14 (7.6 ns instead of 122 ns). Bump it when that field's width or
// meaning changes.
inline constexpr uint32_t kCanStampSemantics = 2;

// And for the GPIO record (protocol.hpp, GpioHeader): 1 = the field id was the
// pin's DataId, the header carried the payload type and the timestamp bit.
// 2 = the field id is the GPIO port's (kGpio), the line rides in the header's
// second byte. Bump it when the GPIO record
// changes in a way this header cannot see.
inline constexpr uint32_t kGpioRecordSemantics = 2;

// The buzzer record (protocol.hpp, BuzzerHeader + BuzzerTonePayload): 1 = a
// reserved header nibble, then frequency_hz (16 bits) and loudness (8 bits);
// downlink only.
inline constexpr uint32_t kBuzzerRecordSemantics = 1;

// What a zero in a manifest setting MEANS: since v11 a required field may not
// be zero (it used to mean "keep the firmware's current value"). No layout
// observes that, so it is folded by hand.
inline constexpr uint32_t kCompleteDeclarationSemantics = 1;

// What the manifest header's last byte and kGetPortList's board_caps MEAN
// (v14): the time base is asked for there, and a board only runs it while a
// manifest that asked for it is in force. No layout observes that.
inline constexpr uint32_t kTimeSyncDeclarationSemantics = 1;

constexpr uint16_t layout_fingerprint() {
    uint32_t h = 2166136261U; // FNV-1a 32-bit offset basis
    auto fold = [&h](uint32_t value) noexcept { h = (h ^ value) * 16777619U; };

    fold(sizeof(PortListPayload));
    fold(sizeof(PortListEntry));
    fold(sizeof(ManifestPayload));
    fold(sizeof(ManifestEntry));
    fold(sizeof(ManifestResultPayload));
    fold(sizeof(ManifestResultEntry));
    fold(sizeof(CanConfigPayload));
    fold(sizeof(UartConfigPayload));
    fold(sizeof(LatencyBreakdownPayload));
    fold(sizeof(LastConfigErrorPayload));
    fold(sizeof(ImuConfigPayload));
    fold(sizeof(GpioConfigPayload));
    fold(sizeof(BuzzerConfigPayload));

    fold(__builtin_offsetof(PortListPayload, port_count));
    fold(__builtin_offsetof(PortListPayload, board_caps));
    fold(__builtin_offsetof(PortListPayload, ports));
    fold(__builtin_offsetof(ManifestPayload, entry_count));
    fold(__builtin_offsetof(ManifestPayload, flags));
    fold(__builtin_offsetof(ManifestPayload, entries));
    fold(__builtin_offsetof(ManifestEntry, setting));
    fold(__builtin_offsetof(ManifestResultPayload, outcome));
    fold(__builtin_offsetof(ManifestResultPayload, entries));
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
    fold(__builtin_offsetof(LatencyBreakdownPayload, cpu_hz));
    fold(__builtin_offsetof(ImuConfigPayload, gyroscope_rate_hz));
    fold(__builtin_offsetof(ImuConfigPayload, gyroscope_bandwidth_hz));
    fold(__builtin_offsetof(ImuConfigPayload, gyroscope_range_dps));
    fold(__builtin_offsetof(ImuConfigPayload, accelerometer_range_g));
    fold(__builtin_offsetof(GpioConfigPayload, pull));
    fold(__builtin_offsetof(GpioConfigPayload, input_flags));
    fold(__builtin_offsetof(GpioConfigPayload, period_ms));

    fold(static_cast<uint32_t>(Request::kGetPortList));
    fold(static_cast<uint32_t>(Request::kGetPortConfig));
    fold(static_cast<uint32_t>(Request::kApplyManifest));
    fold(static_cast<uint32_t>(Request::kGetManifestResult));
    fold(static_cast<uint32_t>(Request::kGetLatencyBreakdown));
    fold(static_cast<uint32_t>(Request::kGetLastConfigError));
    fold(kRequestTypeIn);
    fold(kRequestTypeOut);

    fold(static_cast<uint32_t>(spec::PortKind::kCan));
    fold(static_cast<uint32_t>(spec::PortKind::kUart));
    fold(static_cast<uint32_t>(spec::PortKind::kImu));
    fold(static_cast<uint32_t>(spec::PortKind::kGpio));
    fold(static_cast<uint32_t>(spec::PortKind::kBuzzer));
    fold(static_cast<uint32_t>(spec::kMaxPorts));
    fold(static_cast<uint32_t>(spec::kMaxManifestEntries));
    fold(static_cast<uint32_t>(spec::kMaxGpioLines));
    fold(__builtin_offsetof(ManifestEntry, line));
    fold(__builtin_offsetof(ManifestResultEntry, line));
    fold(static_cast<uint32_t>(ManifestOutcome::kManifestRefused));
    fold(static_cast<uint32_t>(ManifestOutcome::kManifestApplied));
    fold(static_cast<uint32_t>(ManifestOutcome::kManifestRolledBack));
    // Port identities are wire values too: wIndex = DataId on every port-addressed
    // request, and the same numbers tag the bulk stream's fields. Renumbering a
    // port must refuse at the version check, not address the neighbour.
    for (const data::DataId port :
         {data::DataId::kCan0, data::DataId::kCan1, data::DataId::kCan2, data::DataId::kCan3,
          data::DataId::kUart0, data::DataId::kUart1, data::DataId::kUart2, data::DataId::kUart3,
          data::DataId::kUart7, data::DataId::kUart10, data::DataId::kUartDbus, data::DataId::kImu,
          data::DataId::kGpio, data::DataId::kBuzzer})
        fold(static_cast<uint32_t>(port));
    fold(static_cast<uint32_t>(spec::kPortRunning));
    fold(static_cast<uint32_t>(spec::kPortFd));
    fold(static_cast<uint32_t>(spec::kCanCapModeSettable));
    fold(static_cast<uint32_t>(spec::kCanCapFdLongFrames));
    fold(static_cast<uint32_t>(spec::kCanCapRateSettable));
    fold(static_cast<uint32_t>(spec::kUartCapRxPolaritySettable));
    for (const uint8_t capability :
         {spec::kGpioCapDigitalWrite, spec::kGpioCapAnalogWrite, spec::kGpioCapReadOnce,
          spec::kGpioCapReadPeriodic, spec::kGpioCapReadInterrupt, spec::kGpioCapPullUp,
          spec::kGpioCapPullDown, spec::kGpioCapReadTimestamp})
        fold(capability);
    fold(kGpioModeOff);
    fold(kGpioModeOutput);
    fold(kGpioModeInput);
    fold(kGpioPullNone);
    fold(kGpioPullUp);
    fold(kGpioPullDown);
    fold(kGpioInputRisingEdge);
    fold(kGpioInputFallingEdge);
    fold(kGpioInputTimestamp);
    fold(static_cast<uint32_t>(CanMode::kClassic));
    fold(static_cast<uint32_t>(CanMode::kCanFd));
    fold(kCanConfigApply);
    fold(kCanConfigApplyTiming);
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
    fold(static_cast<uint32_t>(ConfigErrorReason::kConfigErrorKindMismatch));
    fold(static_cast<uint32_t>(ConfigErrorReason::kConfigErrorIncomplete));
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

    fold(kChannelDeclarationSemantics);
    fold(kManifestSemantics);
    fold(kCanStampSemantics);
    fold(kGpioRecordSemantics);
    fold(kBuzzerRecordSemantics);
    fold(kCompleteDeclarationSemantics);
    fold(kTimeSyncDeclarationSemantics);
    fold(kBoardCapTimeSync);
    fold(kManifestFlagTimeSync);

    return static_cast<uint16_t>((h >> 16) ^ (h & 0xFFFFU));
}

// Same value on host and every board -- they compile this same header -- and
// different the moment the wire format moves. The 16-bit width is deliberate:
// the fingerprint only needs to separate the handful of layouts that will ever
// coexist in the wild, and it rides in the version field PortListPayload and
// ManifestPayload already carry. Build identity (which git commit produced this
// binary) is a DIFFERENT question, answered by the USB product string at
// discovery time.
inline constexpr uint16_t kVersion = layout_fingerprint();
static_assert(kVersion != 0, "the fingerprint folded to zero -- treated as absent");

} // namespace libhcs::core::protocol::vendor_control
