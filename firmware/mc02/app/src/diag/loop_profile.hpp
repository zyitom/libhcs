#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace libhcs::firmware::diag::profile {

// Where the main loop's time actually goes, measured rather than argued about.
//
// The question this exists to settle: past roughly 780 KB/s of aggregate UART
// forwarding the board starts losing data, and delivered throughput *falls* as
// offered load rises -- congestion collapse, not link saturation. Turning the
// IMU on costs another 18.5% at that point, which proves the CPU is the
// constraint there (a CPU waiting on a saturated USB link would have absorbed
// that extra work for free). What it does not say is which part of the loop is
// expensive: "main loop work" is a category, not a function.
//
// Instrumentation is one DWT->CYCCNT read per section boundary, attributing the
// delta to the section just left. DWT is a free-running 32-bit cycle counter at
// 550 MHz already enabled in app.cpp and until now never read by anything.
// Twelve boundaries per pass at roughly 5 cycles each is about 4% of a 2.6 us
// pass -- enough to shift the absolute numbers slightly, not enough to reorder
// them, which is what this is for. Compiled out entirely by default.

#if defined(libhcs_APP_LOOP_PROFILE) && libhcs_APP_LOOP_PROFILE

inline constexpr bool kEnabled = true;

enum class Section : std::uint8_t {
    kTudTask, // TinyUSB device task: the USB stack's own processing
    kUsb,     // usb::vendor->try_transmit(), all seven interleaved calls
    kCan,     // Can::drain_pending_transmits(): all three CAN queues
    kUart,    // uart1/2/3/dbus try_transmit(): RX dequeue plus TX dequeue
    kImu,     // BMI088 probe and pending-read servicing
    kLed,     // session state plus WS2812 refresh
    kGpio,    // periodic input sampling
    kOther,   // everything not attributed above, including the loop itself
    kCount,
};

// Attribute the cycles since the last mark to the section being left, then start
// timing the named one. Cheaper than an RAII scope per section: one counter read
// per boundary instead of two.
void mark(Section section);

// Closes the pass, counts it, and emits a record every kEmitPeriodMs.
void end_pass();

#else

inline constexpr bool kEnabled = false;

enum class Section : std::uint8_t {
    kTudTask,
    kUsb,
    kCan,
    kUart,
    kImu,
    kLed,
    kGpio,
    kOther,
    kCount,
};

inline void mark(Section) {}
inline void end_pass() {}

#endif

} // namespace libhcs::firmware::diag::profile
