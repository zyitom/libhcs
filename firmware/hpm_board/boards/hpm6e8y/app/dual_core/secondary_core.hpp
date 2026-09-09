#pragma once

#include <cstdint>

namespace libhcs::firmware::ecat {
struct XcoreChannel;
}

namespace libhcs::firmware::xcore {

// Core0 side of the core-swap layout (ecat/CORE_SWAP_MIGRATION.md step 0): this
// image stays the boot core and owns USB + CAN + UART + the protocol stack,
// while the EtherCAT stack runs from a secondary-core image loaded into core1's
// ILM.
//
// Three responsibilities, in the order the hardware forces:
//   1. publish_channel() -- construct the SHARE_RAM channel. core1 spins on its
//      magic word, so the store must happen before the core is released.
//   2. publish_channel() also opens the MBX0 clock gate. MBX0A (core0) and
//      MBX0B (core1) are one full-duplex pair behind a single gate, and core1
//      pokes its half as soon as it has an uplink, so the gate must be open
//      before release. The doorbell ISR is deliberately NOT armed: its direction
//      reverses in migration step 2 and step 0 has no consumer on core0.
//   3. release_core1() -- copy the image into core1's ILM and start it.
//
// (1) and (2) run inside App's interrupt lock; (3) deliberately does not (see
// App::App). Nothing here touches SHARE_RAM before board_init_pmp() has mapped
// it non-cacheable with AMO, which is what the lock-free rings rely on.
//
// Free functions rather than a class, matching ecat/core0/src/pd_glue.cpp: the
// channel pointer is core0-local state with exactly one owner, and with
// libhcs_APP_RELEASE_CORE1 off these become inline no-ops that leave the image
// byte-identical to the plain single-core application.

#if defined(libhcs_APP_RELEASE_CORE1) && libhcs_APP_RELEASE_CORE1

// Call with interrupts masked, after board_init(), before release_core1().
void publish_channel();

// The published channel, for the data plane (xcore/pd_link.cpp). Null before
// publish_channel(). The channel itself is read-only from core0's perspective as
// a structure -- core0 mutates only the ring indices it owns as producer/consumer
// and the arbitration fields it is the authority for; the layout contract lives
// in ecat/common/xcore_channel.hpp and is not this image's to change.
ecat::XcoreChannel* channel();

// Call with interrupts ENABLED, and BEFORE USB and the CAN/UART drivers are
// brought up (but after the flash RPC server is armed).
//
// Interrupts must be on because core1 issues a cross-core request within
// microseconds of release and a masked core0 would turn that into an unbounded,
// silent start-up stall. USB and the CAN controllers must be off because the
// first request can be a full SII rewrite, and servicing one costs core0 tens of
// milliseconds of masked interrupts per erased sector -- long enough to NAK an
// enumerated USB host out of its session and to overrun a live MCAN receiver.
// Pair it with wait_for_core1_eeprom() below, which is what makes the ordering
// mean anything.
void release_core1();

// Block until core1 reports that its boot-path EEPROM work is done, or until
// `timeout_ms` elapses. Returns false on timeout, in which case start-up
// continues anyway: a core1 that never answers must not brick the USB firmware,
// and the only cost of proceeding is the disturbance this ordering avoids.
//
// A no-op returning true in single-core builds.
bool wait_for_core1_eeprom(std::uint32_t timeout_ms);

// Drains core1's diagnostic ring to the core0 console. core1 must never printf
// (the reasoning is in ecat/common/xcore_diag.hpp); this is its only log path,
// so it has to be pumped from the main loop.
//
// A two-load no-op when the ring is empty. When it is not, note that
// console_send_byte() busy-waits on the 115200-baud UART for ~87 us per byte,
// hence the small per-pass budget in the implementation: a chatty core1 must not
// be able to monopolize the loop that also runs tud_task() and the CAN/UART
// pumps.
void poll_diagnostics();

// Poke core1 to republish the ESC input image now, because a batch was just
// pushed into the up ring. Without it the reply waits for core1's next MainLoop
// pass: in SM-synchron mode the SSC maps inputs only inside the SM2-event ISR,
// which runs before this core has produced the reply to the chunk consumed in
// that very ISR, so every request/response exchange loses a full poll cycle.
//
// Call ONLY after a successful push. The mailbox holds a single word, so a poke
// that finds one pending is dropped -- harmless, since the up ring (not the
// mailbox word) is the source of truth the handler re-reads.
void ring_uplink_doorbell();

#else

inline void publish_channel() {}
inline void release_core1() {}
inline bool wait_for_core1_eeprom(std::uint32_t) { return true; }
inline void poll_diagnostics() {}
inline void ring_uplink_doorbell() {}
inline ecat::XcoreChannel* channel() { return nullptr; }

#endif

} // namespace libhcs::firmware::xcore
