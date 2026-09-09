#pragma once

#include <cstdint>

#include <hpm_csr_drv.h>

// Board-side latency breakdown, readable over EP0 on the shipping image.
//
// WHY IT EXISTS. A CAN round trip on this link measures around 118 us, and
// until now there was no way to say how much of that the board is responsible
// for. The only instrument was a libhcs_CAN_DIAG build, whose telemetry rides
// DataId::kUart0 and therefore corrupts anything measuring that channel -- on
// 2026-09-04 that cost six reflashes and one false UART regression. Without the
// split, every latency decision is a guess about which segment to attack.
//
// Two segments are timed, both entirely inside the board:
//
//   downlink  the bulk OUT completion callback is entered, until the CAN frame
//             is in the controller's TX FIFO. Covers deserialization, dispatch
//             and the MCAN write.
//   uplink    the CAN receive interrupt is entered, until the frame has been
//             serialized into the uplink batch. Covers the FIFO read,
//             normalization and serialization.
//
// What is deliberately NOT covered: the wire time of the CAN frame itself
// (about 50 us for 8 bytes at 1M/5M FD), the USB microframe quantization, and
// the host's own submit/wakeup path. Those are the other terms of the 118 us,
// and the point of this header is to size the board's share against them.
//
// CSR_MCYCLE is the clock source: one instruction, core-clock resolution, and
// no side effects -- unlike reading a peripheral register, it cannot perturb
// what it measures. That property is why can_diag uses it too, and it matters
// here because both call sites are on the hot path.
namespace libhcs::firmware::diag::latency {

struct Segment {
    uint32_t count;
    uint32_t min_cycles;
    uint32_t max_cycles;
    uint64_t sum_cycles;

    void note(uint32_t cycles) {
        if (count == 0 || cycles < min_cycles)
            min_cycles = cycles;
        if (cycles > max_cycles)
            max_cycles = cycles;
        sum_cycles += cycles;
        ++count;
    }
};

// Written from the USB callback and the CAN receive interrupt, read from the
// main loop's EP0 handler. Plain 32-bit words: aligned loads and stores are
// atomic on RV32, and a torn 64-bit sum would only skew an average that is
// already a diagnostic, never a control input.
inline Segment downlink{};
inline Segment uplink{};

// Opened by the bulk OUT callback, closed once the frame reaches the TX FIFO.
// Zero means "no packet in flight", which is the state between transfers and
// also what a CAN frame arriving from the EtherCAT side would see.
inline uint32_t downlink_opened_at = 0;

// Two CSR_MCYCLE reads land in tud_vendor_rx_cb and two in the CAN receive ISR.
// Work on the rx_cb path converts to packet rate at 1.2-1.4x, so keep this at
// exactly this size; anything heavier belongs behind libhcs_APP_CAN_DIAG.
inline uint32_t now() { return static_cast<uint32_t>(hpm_csr_get_core_mcycle()); }

inline void open_downlink() { downlink_opened_at = now(); }

inline void close_downlink() {
    if (downlink_opened_at == 0)
        return;
    downlink.note(now() - downlink_opened_at);
}

inline void close_uplink(uint32_t opened_at) { uplink.note(now() - opened_at); }

inline void reset() {
    downlink = {};
    uplink = {};
}

} // namespace libhcs::firmware::diag::latency
