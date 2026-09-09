#pragma once

// Maps the boards' shared microframe axis onto this host's clock by reading the
// controller's own counter, instead of inferring it from a USB round trip.
//
// WHAT THIS REPLACES. Timeline fits (microframe -> host time) from the arrival
// of a board's kTimeStatus, timestamped at the midpoint of the round trip that
// produced it. That fit is the weak link in the whole time base: the boards
// agree with each other to 20 ns and each board's own tick-to-microframe fit has
// a 0.03 us residual, but the hop to host time carries USB scheduling jitter,
// kernel wakeup and reap latency, and lands at single-digit to tens of
// microseconds. MicroframeSource reads the same axis directly at a measured
// 21 ns, so this class exists to put that number where the 19 us one was.
//
// THE ONLY THING THAT HAS TO BE LEARNED IS AN INTEGER. MFINDEX and a board's
// counter both count the SOF packets this controller emits, so they are one
// clock and differ by a whole number of microframes -- nothing to fit, nothing
// that drifts. The offset is estimated by median over a window rather than by
// average, because each estimate carries the round-trip asymmetry (about 0.3 of
// a microframe) and the median of that lands on the right integer while a mean
// need not.
//
// AND ITS CONSTANCY IS THE SAFETY CHECK. If the offset walks, the two counters
// are NOT one clock -- the board is on a different controller, or one of the
// two extensions slipped a wrap. That is exactly the case where using this
// would produce a confident wrong timestamp, so a spread wider than a couple of
// microframes refuses to lock, and a lock that starts drifting is dropped.
// Unlocked means Timeline keeps using the round-trip fit it uses today.
//
// A CONSEQUENCE WORTH NAMING. Once this is locked, kTimeStatus stops being a
// timing-sensitive packet: its ARRIVAL TIME is no longer used for anything. It
// only has to carry which microframe it refers to, and may be late, jittery or
// batched without costing accuracy. That is the protocol change discussed as
// "de-time-sensitising the session packet", obtained without touching the wire
// format, the firmware, or the protocol version -- so the 50 ms session refresh
// interval, which exists to buy sqrt(N) against round-trip noise, can go back
// up once this path is trusted.

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

#include <libhcs/export.hpp>
#include <libhcs/time/microframe_source.hpp>

namespace libhcs::host::time {

class libhcs_API MicroframeTimebase {
public:
    using Clock = MicroframeSource::Clock;

    struct Options {
        // Edges per second. Phase drifts at the rate error between the two
        // clocks -- under a ppm once fitted, so about 0.1 us/s -- and each
        // re-anchor costs a measured 24 us of busy-poll. 12 Hz holds phase far
        // inside the 21 ns the fit itself is worth, for 0.03% of one core.
        double edge_hertz = 12.0;

        // Edges kept for the fit. 256 at 12 Hz is a 21 s window: long enough
        // that the per-edge 360 ns averages down to about 22 ns, short enough
        // to follow a real rate change.
        std::size_t edge_capacity = 256;

        // Board observations kept for the offset median.
        std::size_t offset_capacity = 128;

        // Refuse to lock until this many observations agree, and drop the lock
        // if their spread exceeds the tolerance. Two microframes is generous
        // for a round-trip asymmetry of about a third of one, and far tighter
        // than any real drift would stay inside.
        std::size_t offset_minimum = 32;

        // Scatter tolerance, as a MEDIAN ABSOLUTE DEVIATION -- not a min-to-max
        // span, which grows with sample count and would drift into the limit on
        // a perfectly healthy link. In FRACTIONAL microframes: the estimates are
        // kept unrounded, so this is a real measure of round-trip jitter rather
        // than of where rounding happened to land.
        double offset_tolerance_microframes = 1.0;

        // Drift tolerance: how far the median of the newer half of the window
        // may sit from the older half. This, not the scatter, is what says the
        // two counters are one clock -- scatter is just the round trip. A
        // quarter of a microframe is 31 us; two independent crystals separate
        // by that in seconds.
        double offset_drift_tolerance_microframes = 0.25;

        // Pin the edge-hunting thread. -1 leaves it wherever the scheduler puts
        // it. Busy-polling next to the transport's IO thread is the one avoidable
        // way this could cost throughput.
        int core = -1;
    };

    struct Status {
        bool attached;
        bool locked;
        std::int64_t offset_microframes; // board_microframe - host microframe
        double offset_spread;            // microframes, median absolute deviation
        double offset_drift;             // microframes, newer half median vs older half
        std::size_t edges;
        std::size_t observations;
        double fitted_period_ns;  // against CLOCK_MONOTONIC_RAW
        double residual_sigma_ns; // robust, per edge
        double fitted_phase_ns;   // residual / sqrt(edges)
        std::uint64_t edge_misses;
        MicroframeSource::State source_state;
    };

    // Takes ownership of a validated source. Starts the edge thread immediately;
    // the object is usable (unlocked) from the moment it returns.
    MicroframeTimebase(MicroframeSource source, const Options& options);
    ~MicroframeTimebase();

    MicroframeTimebase(const MicroframeTimebase&) = delete;
    MicroframeTimebase& operator=(const MicroframeTimebase&) = delete;

    // Convenience: open the controller behind a USB bus and wrap it. nullptr if
    // the source is unavailable for any reason, which is a fallback, not an
    // error -- the caller carries on with the round-trip fit.
    [[nodiscard]] static std::unique_ptr<MicroframeTimebase> open_for_usb_bus(int bus_number);
    [[nodiscard]] static std::unique_ptr<MicroframeTimebase>
        open_for_usb_bus(int bus_number, const Options& options);

    // One board report: the microframe it named, and when the round trip that
    // carried it was centred. Only used to learn the integer offset -- once
    // locked, the timestamp is not used for anything, which is the whole point.
    void observe_board(std::uint64_t board_microframe, Clock::time_point sampled_at);

    // Host time of a board microframe. nullopt until locked, and after the
    // source or the offset stops being trustworthy.
    [[nodiscard]] std::optional<Clock::time_point>
        host_time_of(std::uint64_t board_microframe) const;

    [[nodiscard]] bool locked() const;
    [[nodiscard]] Status status() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace libhcs::host::time
