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
// microseconds. MicroframeSource reads the same axis directly, so this class
// exists to put that number where the round-trip one was.
//
// THE ONLY THING THAT HAS TO BE LEARNED IS AN INTEGER. MFINDEX and a board's
// counter both count the SOF packets this controller emits, so they are one
// clock and differ by a whole number of microframes -- nothing to fit, nothing
// that drifts. The integer is found by CAUSALITY, not by assuming the round
// trip is symmetric: each exchange bounds it from both sides, and on a
// high-speed link the bounds close on a single value (detail::OffsetBracket in
// host/src/time/axis_fit.hpp has the argument and the history).
//
// AND ITS CONSTANCY IS THE SAFETY CHECK. If a new exchange contradicts the
// bounds, the two counters are NOT one clock any more -- the board is on a
// different controller, or it re-anchored onto another wrap. That is exactly
// the case where using this would produce a confident wrong timestamp, so the
// lock is dropped on the spot and has to be earned again. Unlocked means
// Timeline keeps using the round-trip fit.
//
// ONLY AN EXACT INTEGER LOCKS. A link whose round trips are all longer than a
// microframe -- full speed, one frame per millisecond -- cannot pin the
// integer, and a guess that is off by one is 125 us of error wearing a
// sub-microsecond label. Such a setup stays on the round-trip fit unless a
// high-speed board shares the controller: the axis is common, so one board's
// exchanges settle it for all.
//
// A CONSEQUENCE WORTH NAMING. Once this is locked, kTimeStatus stops being a
// timing-sensitive packet: its ARRIVAL TIME is no longer used for anything
// but the bounds above. It only has to carry which microframe it refers to,
// and may be late, jittery or batched without costing accuracy.

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <libhcs/export.hpp>

#include "host/src/time/axis_map.hpp"
#include "host/src/time/microframe_source.hpp"

namespace libhcs::host::time {

class MicroframeTimebase {
public:
    using Clock = MicroframeSource::Clock;

    struct Options {
        // Edges per second. Each costs a measured 24 us of busy-poll; 12 Hz is
        // 0.03% of one core, and it is also the rate at which the published
        // map follows CLOCK_MONOTONIC being re-steered.
        double edge_hertz = 12.0;

        // Edges kept for the long fit against CLOCK_MONOTONIC_RAW. 256 at
        // 12 Hz is a 21 s window: the per-edge scatter averages down by 16.
        std::size_t edge_capacity = 256;

        // Edges, newest first, that define CLOCK_MONOTONIC against
        // CLOCK_MONOTONIC_RAW. One second's worth: that pair is noiseless, so
        // this sets only how fast an NTP rate change is followed.
        std::size_t recent_edges = 12;

        // Exchanges kept for the integer offset's bounds.
        std::size_t offset_capacity = 128;

        // Exchanges required before the offset is believed, even if the very
        // first one already leaves a single candidate.
        std::size_t offset_minimum = 4;

        // How much each causality bound is widened, in microframes, for the
        // error of the two lines it is computed from. 0.05 is 6 us -- an
        // order above what either line is worth, and far under the gap to
        // the neighbouring integer.
        double offset_slack_microframes = 0.05;

        // Pin the edge-hunting thread. -1 leaves it wherever the scheduler puts
        // it. Busy-polling next to the transport's IO thread is the one avoidable
        // way this could cost throughput.
        int core = -1;
    };

    struct Status {
        bool locked;
        std::int64_t offset_microframes; // board_microframe - host microframe; valid when locked
        double offset_lower;             // the causality bounds, in microframes
        double offset_upper;
        std::size_t edges;
        std::size_t edges_used;   // after dropping wide and outlying ones
        std::size_t observations;
        double raw_period_ns;     // against CLOCK_MONOTONIC_RAW: the ppm figure
        double mono_period_ns;    // against CLOCK_MONOTONIC: what the map uses
        double residual_sigma_ns; // robust, per edge
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

    // One exchange with a board: the position it reported (fractional
    // microframes on the boards' axis), and the host times between which it
    // must have written that -- request sent, reply seen.
    void observe_board(
        double board_microframe, Clock::time_point sent_at, Clock::time_point received_at);

    // The boards' axis on steady_clock. Never waits: safe on a real-time
    // thread. Not valid() until locked, and again after the source or the
    // offset stops being trustworthy.
    [[nodiscard]] AxisMap axis_map() const noexcept libhcs_NONBLOCKING;

    [[nodiscard]] bool locked() const;
    [[nodiscard]] Status status() const;

    // PCI address of the controller being read, e.g. "0000:00:14.0".
    [[nodiscard]] const std::string& pci_device() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace libhcs::host::time
