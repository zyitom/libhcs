#pragma once

// CAN telemetry for mc02, modelled on hpm_board's app/src/diag/can_diag.hpp.
//
// This board had NO way to report CAN controller state: no PSR/ECR/TXFQS readout,
// no counters, nothing. Debugging a suspected forwarding fault in August 2026
// stalled on exactly that -- the only observable was what the host did or did not
// receive, so "the frame never reached the wire" and "the frame was sent and then
// lost" were indistinguishable from the host side. hpm_board has had this channel
// since its own CAN latch-up investigation; mc02 now gets the equivalent.
//
// Rides DataId::kUart0, which is not a silkscreen UART on this board (those are
// kUart1/2/3/7/10 plus DBUS). Diagnostic builds therefore no longer collide with
// the enclosure RS-485 ports. hpm_board's diagnostic uses the same id, so a
// host-side decoder can share the header; the record version distinguishes them.
//
// Compiled out unless libhcs_APP_CAN_DIAG is set, so the forwarding hot path
// carries nothing in production builds.

#include <cstddef>
#include <cstdint>

namespace libhcs::firmware::diag {

#if defined(libhcs_APP_CAN_DIAG) && libhcs_APP_CAN_DIAG

inline constexpr bool kEnabled = true;

// Wire format of the kUart0 uplink payload. Little endian, fixed layout so the
// host decoder needs no length negotiation. Version 64+ marks the mc02 variant:
// its per-controller block is FDCAN (PSR/ECR/TXFQS), not MCAN, so a decoder must
// not treat it as hpm_board's record.
inline constexpr std::uint8_t kRecordMagic = 0xD1U;
inline constexpr std::uint8_t kRecordVersion = 64U;

// Hot-path notifications: a single relaxed atomic add each. The RX ISR calls the
// first two per interrupt and per forwarded frame respectively.
void note_isr_entry(std::size_t can_index);
void note_frame(std::size_t can_index);
// Frames the board was asked to send but could not queue (software ring full).
void note_tx_fail(std::size_t can_index);
// Received frames dropped because the uplink batch pool was full. This is the
// silent-drop path that went uncounted until 2026-08-05.
void note_uplink_drop(std::size_t can_index);

void note_main_loop();

// Main-loop sampler. Emits one record every kEmitPeriodMs; a no-op when the
// session is not up.
void poll();

#else

inline constexpr bool kEnabled = false;

inline void note_isr_entry(std::size_t) {}
inline void note_frame(std::size_t) {}
inline void note_tx_fail(std::size_t) {}
inline void note_uplink_drop(std::size_t) {}
inline void note_main_loop() {}
inline void poll() {}

#endif

} // namespace libhcs::firmware::diag
