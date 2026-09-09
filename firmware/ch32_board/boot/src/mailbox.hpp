#pragma once

#include <cstddef>
#include <cstdint>
#include <new>

#include "firmware/ch32_board/app/src/utility/ring_buffer.hpp"

namespace libhcs::firmware::boot {

// Cross-core telemetry record: produced by the V3F offload core, consumed by
// the V5F forwarding core. This is deliberately OFF the forwarding hot path --
// V3F publishes non-forwarding work (diagnostics now; IMU/filtering later) here
// so the V5F CAN/UART/USB interrupt path is never interrupted by it.
struct TelemetryRecord {
    uint32_t timestamp; // V5F stamps this on drain (V3F owns no timer instance)
    uint32_t sequence;  // monotonically increasing per record on the V3F side
    uint32_t value;     // opaque diagnostic payload
};

// Shared control block, placed at a fixed address inside the 512 KB shared SRAM
// region (0x2010_0000..0x2018_0000). Both cores access this region with zero
// wait states (CH32H417RM 1591). The address sits below the V3F stack (the top
// 2 KB of that region, per Link_v3f.ld) and far above the V3F .data/.bss, so it
// collides with neither image; V5F runs entirely out of its own TCM and only
// reaches in here for the mailbox.
//
// NOTE(cross-core coherency): RingBuffer publishes with C++ atomic
// acquire/release, which is sufficient on a coherent zero-wait shared-SRAM
// view. If the V5F access path turns out to be cached on target, add explicit
// cache maintenance around this block -- verify on hardware (WCH-Link).
struct SharedBlock {
    // Startup handshake: the V5F core writes 1 once its forwarding bring-up is
    // complete; the V3F boot core spins on this before entering its own loop.
    // Keep this first: it is the documented debugger read-back at kSharedBlockAddr
    // (see PITFALLS.md 4.4).
    volatile uint32_t v5f_ready;

    // Reboot request from the application core, written with kResetRequestMagic.
    // V5F must not reset itself: its reset vector is flash 0x0, i.e. the V3F
    // image, so a self-reset would run the boot core's code on the app core (the
    // mtvec = 0x2010_0003 symptom in PITFALLS.md). The boot core owns the reset,
    // so V5F parks itself and asks V3F to pull it. Used by the DFU detach path
    // (app/src/usb/dfu_runtime.cpp), which pairs it with the boot mailbox.
    volatile uint32_t reset_request;

    utility::RingBuffer<TelemetryRecord, 64> telemetry;
};

inline constexpr uint32_t kResetRequestMagic = 0x52535451; // "RSTQ"

// Fixed placement inside the shared region. Reserve a 8 KB window here.
inline constexpr uintptr_t kSharedBlockAddr = 0x20178000u;
inline constexpr size_t kSharedBlockWindow = 0x2000u;
static_assert(
    sizeof(SharedBlock) <= kSharedBlockWindow, "SharedBlock must fit its reserved window");

inline SharedBlock& shared() { return *reinterpret_cast<SharedBlock*>(kSharedBlockAddr); }

// Boot core (V3F) ONLY, called exactly once BEFORE waking V5F: construct the
// shared block so the consumer never observes an uninitialised RingBuffer. The
// V5F core must never call this (it would wipe the mailbox); it only reads
// v5f_ready and drains telemetry.
inline void init_shared() { new (&shared()) SharedBlock{}; }

} // namespace libhcs::firmware::boot
