#pragma once

// Host-side access to the xHCI microframe counter, as a time source.
//
// WHY THIS EXISTS. Every board on one host controller derives its time base
// from the SOF packets that controller emits, so all of them share an axis
// exactly (measured: cross-board delta == 1 at 100.00000%). What no board can
// supply is the mapping from that shared axis to this machine's own clock --
// today the Timeline estimates it from the arrival time of a USB round trip,
// which imports USB scheduling jitter, kernel wakeup and URB reap latency into
// a number none of those things have anything to do with.
//
// MFINDEX is the same counter, readable directly. It is the microframe index
// the xHC increments every 125 us, and the controller emits exactly one SOF per
// microframe, so it is not "another clock that happens to agree" -- it is the
// board's own clock, read at the other end of the cable.
//
// WHAT THE HOST CANNOT DO, AND WHY POLLING IS NOT A WORKAROUND. The host does
// not receive SOF, it generates it, so there is no per-SOF event to wait on --
// no interrupt, no fd, nothing to poll(). xHCI offers exactly one frame-related
// interrupt, the MFINDEX Wrap Event (USBCMD bit 10, CMD_EWE), and it fires once
// per wrap, i.e. every 2.048 s; Linux neither enables it nor has a handler for
// TRB_MFINDEX_WRAP. Sampling the counter is therefore not second best, it is
// the only shape available -- and at 0.8 us per read it is three orders under
// the 44 us USB round trip it replaces.
//
// TWO USES, VERY DIFFERENT COSTS.
//
//   RATE (ppm). Plain sample() at any convenient rate. Quantization averages
//   out over a fit, no busy-waiting. This is nearly free and is what kills
//   long-term drift between the board axis and host time.
//
//   PHASE (where an edge falls). sample_edge() has to catch the counter
//   changing, which means busy-polling, because nothing will announce it. The
//   cost is bounded by predicting the next edge and opening a narrow window
//   around it -- see sample_edge().
//
// CLOCKS. Every reading carries both a steady_clock stamp (CLOCK_MONOTONIC, to
// interoperate with Timeline) and a CLOCK_MONOTONIC_RAW stamp. Use the raw one
// for anything measured in ppm: this kernel was caught slewing CLOCK_MONOTONIC
// at a saturated -500 ppm, which is 600x the disagreement such a measurement is
// trying to resolve. [MEASURED 2026-09-07, host_mfindex_readable_and_ntp_slew]
//
// FAILURE IS ALWAYS A FALLBACK, NEVER A WRONG ANSWER. Construction validates
// the controller by behaviour, not by register layout: it checks that the thing
// it found actually advances at 8 kHz. A suspended controller reads all-ones, a
// non-xHCI BAR reads nonsense, an ARM SoC has no PCI resource to map at all --
// each is rejected before use, and every rejection means "fall back to the USB
// round trip", which is what the Timeline already does.
//
// REQUIRES root or CAP_SYS_RAWIO: the counter lives in the controller's PCI
// BAR, which reaches userspace only through sysfs. The mapping is PROT_READ and
// the only register ever read outside construction is MFINDEX, a free-running
// counter with no side effects. PORTABILITY: x86 with a PCI xHCI. An ARM SoC
// exposes xHCI as a platform device with no resource0, and is rejected at
// for_usb_bus().

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <libhcs/export.hpp>

namespace libhcs::host::time {

class libhcs_API MicroframeSource {
public:
    // Deliberately the same clock as Timeline::Clock, so a Sample can be handed
    // to Timeline::observe() without a conversion that could hide a domain mix.
    using Clock = std::chrono::steady_clock;

    static constexpr std::uint32_t kCounterBits = 14;
    static constexpr std::uint32_t kCounterModulus = std::uint32_t{1} << kCounterBits;
    static constexpr std::chrono::nanoseconds kMicroframePeriod{125'000};

    // 2.048 s. Two samples further apart than half of this cannot resolve the
    // wrap from the counter alone; see the gap policy on sample().
    static constexpr std::chrono::nanoseconds kWrapPeriod = kMicroframePeriod * kCounterModulus;

    // Longest gap between sample() calls from which the axis can still be
    // continued. The limit is not the 2.048 s wrap: every sample is checked for
    // consistency against elapsed host time, which catches a counter that
    // stopped (the suspended-controller case, where host time keeps running and
    // believing it would manufacture a plausible wrong answer). What this bound
    // protects is the wrap COUNT -- resolving it needs host time good to
    // +-1.024 s over the gap, and 60 s leaves three orders of margin even for a
    // clock slewing at the -500 ppm this kernel was caught using.
    static constexpr std::chrono::seconds kMaxGap{60};

    enum class Error : std::uint8_t {
        kNoPciDevice,      // bus is not behind a PCI device (ARM SoC, bad bus number)
        kPermissionDenied, // needs root or CAP_SYS_RAWIO
        kMapFailed,
        kNotXhci,          // CAPLENGTH/HCIVERSION/RTSOFF implausible, or D3 all-ones
        kCounterStopped,   // found the register, but it does not advance at 8 kHz
    };

    [[nodiscard]] static std::string_view describe(Error error) noexcept;

    enum class State : std::uint8_t {
        kValid,
        kLostSync,       // a gap too long to resolve even with revalidation
        kCounterStopped, // controller suspended or reset under us
    };

    [[nodiscard]] static std::string_view describe(State state) noexcept;

    struct Sample {
        // 64-bit extension of the 14-bit register. Same axis as the board's
        // microframe counter up to a constant integer offset, which is
        // established once per link and does not change while it is up.
        std::uint64_t microframe;

        // Host time of the READ, not of the microframe boundary. A plain
        // sample localises the edge only to within one microframe; that is
        // exactly why phase needs sample_edge() and rate does not.
        Clock::time_point at;
        std::int64_t raw_ns; // CLOCK_MONOTONIC_RAW, for ppm work
    };

    struct Edge {
        // The value the counter took AT this edge.
        std::uint64_t microframe;

        // The edge happened inside [not_before, not_after]: the last read that
        // still saw the old value, and the first that saw the new one. The
        // width is the honest uncertainty of one edge -- it is a bracket, not
        // an estimate, so it cannot be optimistic.
        Clock::time_point not_before;
        Clock::time_point not_after;
        std::int64_t raw_not_before_ns;
        std::int64_t raw_not_after_ns;

        [[nodiscard]] std::chrono::nanoseconds uncertainty() const noexcept {
            return not_after - not_before;
        }
        [[nodiscard]] Clock::time_point midpoint() const noexcept {
            return not_before + (not_after - not_before) / 2;
        }
        [[nodiscard]] std::int64_t raw_midpoint_ns() const noexcept {
            return raw_not_before_ns + (raw_not_after_ns - raw_not_before_ns) / 2;
        }
    };

    // What edge hunting has actually cost, so the caller can report measured
    // overhead instead of a prediction.
    struct EdgeStats {
        std::uint64_t attempts;
        std::uint64_t hits;
        std::uint64_t misses;           // window opened too late, or edge never seen
        std::uint64_t cold_hunts;       // had to poll a whole period
        std::uint64_t reads;            // MFINDEX reads spent in poll loops
        std::chrono::nanoseconds busy;  // total time spent busy-polling
        std::chrono::nanoseconds guard; // current lead on the predicted edge

        // How late the sleep has been returning (decaying max), and the total
        // signed wake error, so a caller can tell "the guard is wide because
        // the kernel wakes late" from "the guard is wide for some other
        // reason". Sizing the guard from anything but this is a loop with no
        // signal in it, which is a mistake this had to make once to find.
        std::chrono::nanoseconds wake_error_high;
        std::chrono::nanoseconds wake_error_total;
        std::uint64_t wake_samples;

        // Where the predicted edge sat when polling actually began. If the
        // predictor is right this equals the poll duration; if it does not,
        // the guard is being asked to cover a prediction error rather than a
        // sleep error, and no amount of guard tuning will help.
        std::chrono::nanoseconds predicted_lead_total;

        // Times the window opened after the target edge and had to wait out a
        // whole period for the next one. This is what the guard is tuned
        // against, so it should be a small fraction of hits.
        std::uint64_t overshoots;
    };

    // Finds the controller backing a USB bus number (the "Bus 003" of lsusb)
    // and maps it. This is the entry point to prefer: it ties the time source
    // to the bus the board is actually on, and two controllers in one machine
    // are two crystals, not one clock.
    [[nodiscard]] static std::expected<MicroframeSource, Error> for_usb_bus(int bus_number);

    [[nodiscard]] static std::expected<MicroframeSource, Error>
        for_pci_device(std::string_view pci_device);

    // NOLINTNEXTLINE(performance-trivially-destructible) -- clang-tidy suggests
    // defaulting this on its first declaration, which cannot compile: the
    // destructor of unique_ptr<Impl> needs Impl complete, and Impl is opaque here.
    ~MicroframeSource();
    MicroframeSource(MicroframeSource&&) noexcept;
    MicroframeSource& operator=(MicroframeSource&&) noexcept;
    MicroframeSource(const MicroframeSource&) = delete;
    MicroframeSource& operator=(const MicroframeSource&) = delete;

    // One read. Cheap enough to call in a completion callback: measured at
    // 0.79-0.88 us on Tiger Lake, against 44 us for the USB round trip it
    // replaces. Returns nullopt once the source has gone invalid, permanently.
    [[nodiscard]] std::optional<Sample> sample();

    // Brackets one microframe boundary by busy-polling.
    //
    // COST, AND WHY IT IS NOT ONE MICROFRAME. Polling blindly would burn a core
    // for up to a whole 125 us period per edge. It does that only once: after
    // the first edge the period is known to better than a ppm, so the next edge
    // can be predicted and the window opened just before it. The lead adapts --
    // it shrinks while the edge keeps landing early in the window and doubles
    // on a miss -- so the steady-state busy time per edge converges to what
    // this machine's sleep granularity actually delivers rather than to a
    // number guessed here. Read it back from edge_stats().
    //
    // Returns nullopt on a miss (the window closed with no transition seen),
    // which is not an error: the lead widens and the next call retries.
    [[nodiscard]] std::optional<Edge> sample_edge();

    [[nodiscard]] State state() const noexcept;
    [[nodiscard]] bool valid() const noexcept { return state() == State::kValid; }

    [[nodiscard]] const std::string& pci_device() const noexcept;

    // Median cost of one MFINDEX read, measured during construction.
    [[nodiscard]] std::chrono::nanoseconds read_cost() const noexcept;

    // Microframes per second seen during the construction-time behavioural
    // check, against CLOCK_MONOTONIC_RAW. Nominally 8000.
    [[nodiscard]] double validated_rate_hz() const noexcept;

    [[nodiscard]] EdgeStats edge_stats() const noexcept;

private:
    struct Impl;
    explicit MicroframeSource(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

// The PCI device backing a USB bus number, e.g. 3 -> "0000:00:14.0". Empty if
// the bus does not exist or is not on PCI -- the ARM SoC case, where xHCI is a
// platform device and there is no BAR in sysfs to map.
[[nodiscard]] libhcs_API std::string pci_device_for_usb_bus(int bus_number);

// USB bus numbers backed by a given PCI device, for diagnostics that want to
// report which boards share a crystal.
[[nodiscard]] libhcs_API std::vector<int> usb_buses_of_pci_device(std::string_view pci_device);

// Every PCI device with a USB controller class code, whether or not it is xHCI.
[[nodiscard]] libhcs_API std::vector<std::string> usb_controller_pci_devices();

} // namespace libhcs::host::time
