#pragma once

// The USB Start-of-Frame hook on mc02's DWC2 controller.
//
// Same role as hpm_board's sync/sof.hpp -- the single place a board learns what
// microframe it is in -- but the hardware underneath is a different USB IP, so
// every register in the path differs:
//
//   concept                 HPM (ChipIdea/EHCI)   STM32H723 (Synopsys DWC2)
//   -------                 -------------------   -------------------------
//   SOF status flag         USBSTS.SRI            GINTSTS.SOF
//   SOF interrupt enable    USBINTR.SRE           GINTMSK.SOFM
//   frame counter           FRINDEX (microframes) DSTS.FNSOF (frames at FS)
//   port speed              PORTSC1.PSPD          DSTS.ENUMSPD
//
// The frame counter is the difference that propagates: DWC2's FNSOF holds a
// MICROFRAME number only at high speed. At full speed -- which is all mc02 can
// do, it has no ULPI PHY -- it holds a FRAME number, so it steps by one per
// interrupt where an EHCI FRINDEX would step by eight. sync::timebase does the
// scaling; see the comment on frame_scale() there.
//
// HOW IT GETS INTO THE VECTOR, and why it looks unusual. hpm_board owns its USB
// vector and calls the hook as the first statement. On mc02 the vector lives in
// bsp/cubemx/Core/Src/stm32h7xx_it.c, which is CubeMX output that this repo
// forbids editing (see the CubeMX BSP discipline section of AGENTS.md), and it
// does nothing but call TinyUSB's dcd_int_handler(). The hook is therefore
// interposed at LINK time
// instead: -Wl,--wrap=dcd_int_handler routes the vector's call to
// __wrap_dcd_int_handler() below, which timestamps and then chains to the real
// one. Nothing generated is touched, and with libhcs_APP_TIME_SYNC off the wrap
// flag is not passed at all, so the default build links exactly as before.
//
// The cost of arriving one call later than hpm_board's hook does is the call
// itself: a handful of cycles, tens of nanoseconds at 550 MHz, and constant.
//
// The SOF status bit is consumed here, so dcd_int_handler() never sees it and
// never queues a DCD_EVENT_SOF. A board with the time base enabled therefore
// presents the same USB behaviour to the class drivers as one without it.

#include <cstdint>

namespace libhcs::firmware::sync {

// Reads the timestamp and the frame counter, then acknowledges SOF. Called from
// __wrap_dcd_int_handler() ahead of the real handler; a no-op the compiler
// removes entirely when the time base is compiled out.
void sof_isr_entry();

// Arms GINTMSK.SOFM. Must run after tusb_rhport_init(), whose dcd_init()
// assigns GINTMSK wholesale.
void sof_init();

// True when the hardware SOF capture (TIM2 ITR5) is fine enough to be used, and
// the number of CPU cycles one TIM2 tick is worth. Both are decided at
// sof_init() from TIM2's prescaler, so they follow the .ioc without a code
// change; see the comment block in sof.cpp for why a coarse capture is refused.
bool sof_capture_active();
std::uint32_t sof_capture_cycles_per_tick();

// Re-arms the SOF enable. Cheap enough for the main loop's periodic work, and
// what makes the hook survive a controller that was reinitialized behind us --
// dcd_int_handler() clears SOFM itself whenever it sees a SOF it did not expect.
void sof_rearm();

} // namespace libhcs::firmware::sync
