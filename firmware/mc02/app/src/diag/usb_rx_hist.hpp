#pragma once

#include <cstdint>

namespace libhcs::firmware::diag::usb_rx_hist {

// Distribution of the interval between consecutive bulk OUT transfer
// completions, measured on the board's own DWT cycle counter.
//
// The question this was built to settle: the DWC2 controller's internal DMA cost
// about 1 us per packet of achieved rate while making the board's CPU work
// strictly smaller (the switch is gone now; see firmware/mc02/AGENTS.md). An
// average is compatible with two very different mechanisms, and they call for
// opposite fixes:
//
//   uniform shift -- every packet really is ~1 us slower, so the cost is in the
//       controller's own per-transfer work (DMA arbitration, endpoint re-arm).
//   bimodal       -- almost every packet costs the same as before, and a small
//       fraction misses the host's next transaction window and waits a whole
//       125 us microframe. Then the fix is anything that shortens the path from
//       packet arrival to the endpoint being re-armed, not the copy itself.
//
// Sampling point is tud_vendor_rx_cb, i.e. usbd task context in the main loop,
// not the interrupt. That adds up to one loop pass (about 8 us here) of jitter
// to each sample -- irrelevant for telling a 1 us shift from a 125 us step, and
// it keeps the instrument out of bsp/ code.
//
// Occupies DataId::kUart0 like the other diagnostic channels, so it is mutually
// exclusive with them. Compiled out entirely by default.

#if defined(libhcs_APP_USB_RX_HIST) && libhcs_APP_USB_RX_HIST

inline constexpr bool kEnabled = true;

// One completed bulk OUT transfer. Emits a record every 500 ms.
void note();

#else

inline constexpr bool kEnabled = false;

inline void note() {}

#endif

} // namespace libhcs::firmware::diag::usb_rx_hist
