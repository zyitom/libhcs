#pragma once

// Host side of the shared USB-SOF time base (firmware/hpm_board/SOF_TIMEBASE.md).
//
// One process-wide axis, deliberately. Every board resolves the wrap of its own
// hardware microframe counter against the anchor this object produces, so two
// boards agree in ABSOLUTE terms only if both anchors came from the same origin.
// A per-board axis would leave each board internally consistent and mutually
// offset by whole seconds -- the failure that is hardest to notice, because
// every board would report a plausible timeline.
//
// Note what the anchor does NOT have to be: accurate. A board keeps the low 14
// bits from FRINDEX and takes only the wrap from the host, so the anchor may be
// off by up to +-1.024 s without changing the result on any board. Host clock
// error therefore cannot degrade cross-board synchronisation. It can only make
// the mapping to wall-clock time wrong, which is a separate and much weaker
// requirement.
//
// UNIX TIME. observe() builds a least-squares fit of (microframe -> host
// steady_clock) from the boards' own reports, timestamped at the midpoint of the
// anchor/status round trip so the USB transit bias mostly cancels. Wall-clock
// conversion then adds a single steady->system offset sampled ONCE, at
// construction: tracking it live would import every NTP step and slew straight
// into the timeline. Consequences worth stating plainly:
//
//   * alignment to THIS MACHINE's Unix clock as it was at startup: microseconds;
//   * alignment to true UTC: whatever this machine's own clock discipline is
//     worth, which for plain NTP is milliseconds. A microsecond-accurate UTC
//     mapping needs PTP or a PPS reference, and no amount of work here supplies
//     it.

#include <chrono>
#include <cstdint>
#include <mutex>

#include <libhcs/export.hpp>

namespace libhcs::host::time {

class MicroframeTimebase;

class libhcs_API Timeline {
public:
    using Clock = std::chrono::steady_clock;

    static constexpr std::chrono::nanoseconds kMicroframePeriod{125'000};

    // Samples kept for the fit.
    //
    // The residual against this fit is not "USB round-trip jitter" in any vague
    // sense -- on a full-speed link it is EXACTLY the 1 ms frame quantization,
    // and that was measured rather than assumed: the residual histogram is flat,
    // spans one frame (-486..+534 us), and its sigma is 288.0 us against the
    // 1000/sqrt(12) = 288.7 us a uniform distribution predicts. [2026-09-07, mc02]
    //
    // Being uniform, zero-mean and independent, it averages down as exactly
    // sqrt(N), which makes this constant a direct accuracy knob. Measured on
    // mc02 at the 50 ms refresh interval below:
    //
    //     256 samples  -> fitted phase 18.5 us
    //    1024 samples  -> fitted phase  8.4 us
    //
    // 1024 is chosen with the 50 ms period, not the old 250 ms one: that keeps
    // the window at ~51 s per board -- no longer than the 64 s this used to
    // cover, so it still follows the crystals apart -- while giving four times
    // the samples inside it.
    static constexpr std::size_t kSampleCapacity = 1024;

    static Timeline& instance();

    // The anchor to send to every board this round. Open loop against the
    // process origin until the fit has samples, fitted afterwards -- the
    // handover matters because an open-loop axis drifts against the real
    // microframe rate by the host crystal's error, which would eventually walk
    // past the +-1.024 s the wrap resolution tolerates.
    uint64_t anchor_for(Clock::time_point when) const;
    uint64_t anchor_now() const { return anchor_for(Clock::now()); }

    // One board's kTimeStatus, timestamped at the midpoint of the round trip
    // that produced it.
    void observe(uint64_t microframe, Clock::time_point sampled_at);

    bool locked() const;
    std::size_t sample_count() const;

    // Fitted host time of a microframe, and the inverse. Both fall back to the
    // open-loop origin before the fit has converged.
    Clock::time_point host_time_of(uint64_t microframe) const;

    // The same, for a microframe with a fraction -- which is what every caller
    // that converts a board's LOCAL timer reading actually has, since a record
    // lands between two microframes rather than on one.
    //
    // This overload exists because three separate call sites independently wrote
    // host_time_of(static_cast<uint64_t>(microframe)) and so quantised their
    // answer to the 125 us grid: a uniform 0..125 us error with a 62.5 us mean,
    // larger than every other term in the conversion put together. It hid for as
    // long as the axis came from the USB round trip, whose own scatter is the
    // same order; against the host microframe counter (26 ns) it is the entire
    // error budget. Prefer this overload whenever the microframe was computed
    // rather than reported.
    Clock::time_point host_time_of(double microframe) const;
    std::chrono::system_clock::time_point unix_time_of(uint64_t microframe) const;
    uint64_t microframe_at_unix(std::chrono::system_clock::time_point when) const;

    // Nanoseconds per microframe as measured against this host's steady clock;
    // 125000 exactly would mean the two crystals agree. Zero before lock.
    double measured_period_ns() const;

    // Optional host-side microframe source (libhcs/time/microframe_timebase.hpp).
    //
    // When one is attached AND locked, host_time_of() answers from the
    // controller's own counter -- measured 21 ns -- instead of from the fit
    // through the USB round trip below, which is the same quantity known to
    // single-digit microseconds at best. Everything else here is unchanged: the
    // round-trip fit keeps running, keeps being what anchor_for() publishes, and
    // is what host_time_of() falls back to the moment the source stops being
    // trustworthy. Attaching is therefore never a commitment -- the worst case
    // is today's behaviour.
    //
    // The Timeline does not own it; the caller must detach (pass nullptr) before
    // destroying it.
    void attach_microframe_timebase(MicroframeTimebase* timebase);
    bool microframe_timebase_locked() const;

private:
    Timeline();

    struct Sample {
        uint64_t microframe;
        int64_t host_ns;
    };

    void refit_locked();

    mutable std::mutex mutex_;
    Clock::time_point origin_;
    std::chrono::system_clock::time_point unix_origin_;

    Sample samples_[kSampleCapacity];
    std::size_t sample_head_ = 0;
    std::size_t sample_count_ = 0;

    bool fitted_ = false;
    uint64_t fit_reference_microframe_ = 0;
    double fit_reference_ns_ = 0.0;
    double fit_period_ns_ = 0.0;

    MicroframeTimebase* microframe_timebase_ = nullptr;
};

// Shorthand for Timeline::instance().
libhcs_API Timeline& timeline();

} // namespace libhcs::host::time
