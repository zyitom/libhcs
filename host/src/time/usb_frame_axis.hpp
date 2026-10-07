#pragma once

// Host side of the shared USB-SOF time base (firmware/hpm_board/SOF_TIMEBASE.md).
//
// One axis per HOST CONTROLLER, deliberately. Every board resolves the wrap of
// its own hardware microframe counter against the anchor its axis produces, so
// boards agree in ABSOLUTE terms only if their anchors came from the same
// origin -- and what defines the origin is the controller: its frame counter is
// the real, process-wide physical resource the boards' counters follow. Boards
// on two controllers are not on one axis at all (today the only guard is a log
// line telling you to move them onto one controller), so the registry keys on
// the controller's PCI address and hands every bus of one controller the same
// object. A per-board axis would leave each board internally consistent and
// mutually offset by whole seconds -- the failure that is hardest to notice,
// because every board would report a plausible timeline.
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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>

#include <libhcs/export.hpp>

#include "host/src/time/affine.hpp"
#include "host/src/time/axis_map.hpp"
#include "host/src/time/frame_clock.hpp"
#include "host/src/time/microframe_timebase.hpp"

namespace libhcs::host::time {

class UsbFrameAxis {
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

    // The axis of the host controller behind a USB bus (the "Bus 003" of
    // lsusb). Boards reached through buses of the same controller -- same PCI
    // address; buses the PCI lookup cannot place share one unbound axis --
    // get the same object, and the registry holds it for the life of the
    // process. The first call on a fresh axis also starts reading that
    // controller's own microframe counter
    // (microframe_timebase.hpp); when it locks, map() answers from the
    // counter instead of from the round-trip fit.
    [[nodiscard]] static std::shared_ptr<UsbFrameAxis> for_usb_bus(int bus_number);

    // The anchor to send to every board this round. Open loop against the
    // process origin until the fit has samples, fitted afterwards -- the
    // handover matters because an open-loop axis drifts against the real
    // microframe rate by the host crystal's error, which would eventually walk
    // past the +-1.024 s the wrap resolution tolerates.
    uint64_t anchor_for(Clock::time_point when) const;
    uint64_t anchor_now() const { return anchor_for(Clock::now()); }

    // One board's kTimeStatus: the position on the axis it reported
    // (fractional microframes), and the host times between which it must have
    // written that -- the anchor that asked for it sent, this reply seen.
    //
    // Both ends of the round trip, not its midpoint: the midpoint is all the
    // round-trip fit below can use, but the controller source turns the two
    // ends into hard bounds on its integer offset and needs no assumption
    // about where between them the board sampled.
    void observe(double microframe, Clock::time_point sent_at, Clock::time_point received_at);

    bool locked() const;
    std::size_t sample_count() const;

    // The axis on this machine's steady_clock, as a value to convert with.
    //
    // THIS is the call for anything that turns a board timestamp into host
    // time: it never waits (no lock, no syscall), so it is safe on a real-time
    // thread, and the copy it returns converts any number of timestamps
    // against one consistent line. Not valid() until a source has locked --
    // check, because before that there is no relation to report.
    //
    // It answers from the host controller's own counter when that source is
    // attached and locked, and from the round-trip fit otherwise.
    [[nodiscard]] AxisMap axis_map() const noexcept libhcs_NONBLOCKING;

    // The same line in Affine form -- what the sample timer converts with.
    // Invalid (rate 0) until a source has locked.
    [[nodiscard]] Affine<FrameClock, HostClock> map() const noexcept libhcs_NONBLOCKING {
        return affine_of(axis_map());
    }

    // Fitted host time of a microframe. Unlike map() these fall back to
    // the open-loop origin before any fit exists, so they always answer and
    // the early answers mean nothing; prefer map() in new code.
    Clock::time_point host_time_of(uint64_t microframe) const;
    Clock::time_point host_time_of(double microframe) const;
    std::chrono::system_clock::time_point unix_time_of(uint64_t microframe) const;
    uint64_t microframe_at_unix(std::chrono::system_clock::time_point when) const;

    // Nanoseconds per microframe as measured against this host's steady clock;
    // 125000 exactly would mean the two crystals agree. Zero before lock.
    double measured_period_ns() const;

    // Starts reading the microframe counter of the host controller behind a USB
    // bus (microframe_timebase.hpp), and keeps doing so for the life of the
    // process. for_usb_bus() does this already; it stays public for tests.
    //
    // When it is attached AND locked, map() answers from the controller's
    // own counter instead of from the fit through the USB round trip, which is
    // the same quantity known to microseconds at best. Everything else here is
    // unchanged: the round-trip fit keeps running, keeps being what
    // anchor_for() publishes, and is what map() falls back to the moment
    // the source stops being trustworthy. Calling this is therefore never a
    // commitment -- the worst case is the round-trip fit alone.
    //
    // Returns whether the axis is now read from that bus's controller. False
    // when the counter cannot be read (no permission, not a PCI xHCI: see
    // microframe_source.hpp), and false when a source is already attached to a
    // DIFFERENT controller -- the axis is one controller's Start-of-Frame
    // stream, and boards on two controllers are not on one axis at all.
    // Blocks for about a third of a second the first time, while the counter
    // is validated.
    bool use_controller_of_usb_bus(int bus_number);

    // The attached controller source's state, or nothing if none is attached.
    [[nodiscard]] std::optional<MicroframeTimebase::Status> controller_status() const;

    ~UsbFrameAxis();
    UsbFrameAxis(const UsbFrameAxis&) = delete;
    UsbFrameAxis& operator=(const UsbFrameAxis&) = delete;

private:
    UsbFrameAxis();

    struct Sample {
        double microframe;
        int64_t host_ns;
    };

    void refit_locked();
    void publish_locked() noexcept;

    mutable std::mutex mutex_;
    Clock::time_point origin_;
    std::chrono::system_clock::time_point unix_origin_;

    Sample samples_[kSampleCapacity];
    std::size_t sample_head_ = 0;
    std::size_t sample_count_ = 0;

    bool fitted_ = false;
    double fit_reference_microframe_ = 0.0;
    double fit_reference_ns_ = 0.0;
    double fit_period_ns_ = 0.0;

    // The round-trip fit, in the form axis_map() hands out. Behind a pointer so
    // this header does not have to show how a map is published.
    struct Published;
    std::unique_ptr<Published> published_;

    // Owned, created at most once, never replaced: axis_map() reads the raw
    // pointer without the mutex, which is only sound because the object it
    // points at lives until the UsbFrameAxis itself goes.
    std::unique_ptr<MicroframeTimebase> controller_;
    std::atomic<MicroframeTimebase*> controller_view_{nullptr};
};

} // namespace libhcs::host::time
