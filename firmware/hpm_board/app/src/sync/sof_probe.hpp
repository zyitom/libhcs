#pragma once

// Step 1 of the USB-SOF cross-board time base: prove that FRINDEX really does
// advance by exactly one per SOF interrupt, before any of the timeline algorithm
// is written.
//
// WHY THIS EXISTS AS ITS OWN FIRMWARE STEP. The whole scheme rests on the device
// controller's frame index being a passive, hardware-updated copy of the host's
// microframe counter -- identical on every board hanging off the same host
// controller, because every board is clocked by the same SOF edges. If the
// controller instead sets the SOF-received status flag slightly BEFORE it
// increments FRINDEX, an interrupt handler that reads the register at entry sees
// the previous value, and the observed delta sequence degenerates into an
// alternating 0, 2, 0, 2 pattern. Both cases produce a working-looking counter,
// and they differ by exactly one microframe of systematic offset -- which is 125
// us, two orders of magnitude past the ~1 us the design targets. Debugging that
// after the wrap-count state machine and the host protocol exist would mean
// separating an algorithm bug from a silicon race with both in flight, so it is
// answered here with nothing else running.
//
// The record therefore carries three independent discriminators:
//   * a full histogram of consecutive FRINDEX deltas (the primary evidence);
//   * for every anomalous delta, the USBSTS the interrupt was taken on, PORTSC1
//     (port speed, suspend, PHY low-power), and the millisecond tick -- enough
//     to tell an enumeration artifact from something happening in steady state;
//   * a second FRINDEX read taken immediately after the first, inside the same
//     interrupt -- a nonzero "advanced within ISR" count is the direct signature
//     of sampling right on the increment edge;
//   * ISR-to-ISR local timer intervals, which must center on 125 us. A delta of
//     1 paired with a 250 us interval means interrupts are being missed and the
//     counter is silently wrong, which the delta histogram alone cannot see.
//
// The register reads and the SRI acknowledge live in sync/sof.cpp, which is the
// single SOF hook this and sync::timebase share; the probe only receives the
// values.
//
// Compiled out entirely unless libhcs_APP_SOF_DIAG, so a production image
// carries neither the counters nor the 8 kHz interrupt.

#include <cstdint>

namespace libhcs::firmware::sync::sof_probe {

#if defined(libhcs_APP_SOF_DIAG) && libhcs_APP_SOF_DIAG

inline constexpr bool kEnabled = true;

// Wire format of the UART0 uplink payload. Little endian, fixed layout except
// for the anomaly tail, whose entry count is carried in the record.
inline constexpr std::uint8_t kRecordMagic = 0xD2U;
inline constexpr std::uint8_t kRecordVersion = 2U;

// Delta buckets 0..8 plus one catch-all for 9 and above.
inline constexpr std::uint32_t kDeltaBuckets = 10U;

// Anomalies kept per record. Drained on every emit, so a run shows anomalies as
// they happen instead of only the first few after boot -- which is what version
// 1 of this record effectively did, because 8 slots per 100 ms filled up during
// enumeration and everything after went unseen. Sized well above the ~3 per
// record actually observed; the host cross-checks the stored count against the
// free-running total, so an overflow cannot pass silently.
inline constexpr std::uint32_t kAnomalyCapacity = 12U;

// ISR path, called from sync::sof_isr_entry() with the values it read.
void note_sof(
    std::uint32_t frindex, std::uint32_t frindex_again, std::uint32_t now_quarter_us,
    std::uint32_t usbsts, std::uint32_t portsc1);

// Main-loop sampler; emits one record every kEmitPeriodMs of the 1 kHz tick.
void poll(std::uint32_t tick);

#else

inline constexpr bool kEnabled = false;

inline void note_sof(
    std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t) {}
inline void poll(std::uint32_t) {}

#endif

} // namespace libhcs::firmware::sync::sof_probe
