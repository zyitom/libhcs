#pragma once

// The USB Start-of-Frame hook: one entry point, two consumers.
//
// Everything the shared time base is built on enters here. The device
// controller raises SRI once per 125 us microframe and passively updates
// FRINDEX from the host's SOF packets, so this handler -- and nothing else --
// is where a board learns what microframe it is in. Both consumers need the
// register read to happen at the same instant, and the read must be the FIRST
// thing that happens in the vector, so they share one hook rather than each
// installing their own:
//
//   * sync::timebase  -- the production time base (counter, fit, anchoring).
//   * sync::sof_probe -- the validation instrument (delta histogram, port
//                        state), see SOF_TIMEBASE.md.
//
// Either, both, or neither may be compiled in; with neither this is a no-op the
// compiler removes from the USB vector entirely.
//
// The SRI status bit is consumed here, so TinyUSB's device ISR never sees SOF
// and never queues a DCD_EVENT_SOF. A board with the time base enabled
// therefore presents the same USB behaviour to the class drivers as one
// without it.

namespace libhcs::firmware::sync {

// First statement of the USB0 vector, ahead of dcd_int_handler().
void sof_isr_entry();

// Arms the SOF interrupt. Must run after tud_init(), whose dcd_init() assigns
// USBINTR wholesale.
void sof_init();

// Re-arms the SOF enable. Cheap enough to call from the main loop's periodic
// work, and what makes the hook survive a controller that was reinitialized.
void sof_rearm();

} // namespace libhcs::firmware::sync
