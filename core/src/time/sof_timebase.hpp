#pragma once

#include <cstdint>
#include <optional>

#include "core/include/libhcs/data/datas.hpp"

namespace libhcs::core::time {

// The shared time base's arithmetic, one copy for every board: a 64-bit microframe counter
// fed by the USB SOF, a sliding least-squares fit of a local clock against it, the host
// anchor that resolves the counter's 2.048 s wraps, and the state machine that refuses to
// answer while the counter is suspect. The design and its measurements are in
// firmware/hpm_board/SOF_TIMEBASE.md; each board's sync/timebase.hpp says what is
// chip-specific about it.
//
// What is NOT here is what differs by chip, and it stays in the board's sync/timebase.cpp:
// reading the SOF frame number and the port speed, which local clock is sampled and the
// corrections applied to it before it reaches note_time() (SOF packet length, full-speed
// bit stuffing), the interrupt lock, and turning fit ticks into the quarter-us the board's
// other records carry (the STM32 boards fit on the cycle counter and bridge to a timer).
//
// Locking is the caller's: note_*() run in the SOF interrupt, everything else holds the
// board's interrupt lock, except compute_fit(), which is pure and runs unlocked between
// copy_window() and publish_fit().
//
// Policy is a type with
//   static constexpr std::uint32_t kNominalTicksPerMicroframe;  // fit clock, 125 us
//   static constexpr std::uint64_t kResidualAbsMaxLimit;        // larger magnitudes are
//                                                                // left out of the maximum
//   static std::int64_t to_quarter_us(std::int64_t fit_ticks);  // for the status record
inline constexpr std::uint32_t kSampleDecimation = 64U; // one fit sample per 64 microframes
inline constexpr std::uint32_t kSampleCount = 128U;     // 8192 microframes = 1.024 s window
inline constexpr std::uint32_t kFitPeriodMs = 20U;

// The microframe axis is 14 bits on every board, whatever the hardware counter looks
// like (EHCI FRINDEX counts microframes, DWC2 at full speed 11-bit frames times 8), so one
// host anchor serves them all.
inline constexpr std::uint32_t kMicroframeMask = 0x3FFFU;
inline constexpr std::uint64_t kMicroframeModulus = 0x4000U;

struct TimebaseSnapshot {
    data::TimeState state;
    // Absolute microframe when kValid; the board's own origin otherwise.
    std::uint64_t microframe;
    // Filled in by the board: the local time paired with that microframe.
    std::uint64_t timestamp_quarter_us;
    // Fitted local ticks per microframe, Q16 quarter-us; 0 until the fit converges.
    std::uint32_t ticks_per_microframe_q16;
    std::uint32_t anomaly_count;
    // Out-of-sample prediction error since the last report(), Q16 quarter-us.
    std::int32_t residual_mean_q16;
    std::uint32_t residual_abs_max_q16;
    std::uint32_t residual_count;
};

// Rounding toward negative infinity. C++ truncates toward zero, which would make the wrap
// resolution asymmetric about 0: two boards straddling the origin could pick different
// wraps.
constexpr std::int64_t floor_div(std::int64_t numerator, std::int64_t denominator) {
    const std::int64_t quotient = numerator / denominator;
    const std::int64_t remainder = numerator % denominator;
    return (remainder != 0 && ((remainder < 0) != (denominator < 0))) ? quotient - 1 : quotient;
}

template <typename Policy>
class SofTimebase {
public:
    // A copy of the sample ring, taken under the lock, fitted outside it.
    struct Window {
        std::uint32_t sample[kSampleCount];
        std::uint32_t count = 0;
        std::uint64_t oldest_microframe = 0;
        std::uint64_t now = 0;
    };

    struct Fit {
        std::uint64_t reference_microframe;
        std::uint64_t reference_time;
        std::uint64_t ticks_per_microframe_q16;
    };

    // ---- the SOF interrupt ----

    // Extend the 32-bit local clock to 64 bits, before anything uses the reading. The
    // clock must wrap no faster than every few SOFs for "the low word went down" to mean
    // a wrap (550 MHz: 7.8 s, 4 MHz: 1073 s, one SOF per ms at most).
    void note_time(std::uint32_t now) {
        if (!time_seeded_) {
            time_seeded_ = true;
        } else if (now < previous_time_) {
            time_high_++;
        }
        previous_time_ = now;
    }

    // Advance the counter to the hardware reading `index`, already on the 14-bit
    // microframe axis; `step` is the microframes one SOF interrupt should advance (8 at full
    // speed, 1 at high speed). True when it did exactly that and the SOF goes on to
    // note_sample(); false when it seeded, recovered a gap or invalidated.
    bool note_microframe(std::uint32_t index, std::uint32_t step) {
        if (!counter_seeded_) {
            counter_seeded_ = true;
            previous_index_ = index;
            // Seeded with the reading, not 0: the counter's low 14 bits must stay equal
            // to the hardware's for an anchor to be a plain multiple of 16384.
            counter_ = index;
            state_ = data::TimeState::kWaitingAnchor;
            reset_fit_window();
            ring_oldest_microframe_ = counter_;
            return false;
        }

        const std::uint32_t delta = (index - previous_index_) & kMicroframeMask;
        previous_index_ = index;
        counter_ += delta;

        if (delta != step) [[unlikely]] {
            anomaly_count_++;
            if (delta > step && delta < step * 8U) {
                // A few missed interrupts. The counter is still right -- it moved by the
                // true delta -- so the timeline survives; only the fit window goes, since
                // samples either side of the gap would tilt the line.
                reset_fit_window();
                ring_oldest_microframe_ = counter_;
            } else {
                // 0 (bus not running: the enumeration window looks like this) or a whole
                // frame or more unaccounted for. The counter is no longer trustworthy.
                invalidate();
            }
            return false;
        }

        if (state_ == data::TimeState::kInvalid) {
            state_ = data::TimeState::kWaitingAnchor;
            reset_fit_window();
            ring_oldest_microframe_ = counter_;
        }
        return true;
    }

    // Take a fit sample on every kSampleDecimation-th microframe: first score it against
    // the fit published before it existed (a true forecast error, the one a timed action
    // suffers), then put it in the window. `now` is the note_time() reading.
    void note_sample(std::uint32_t now) {
        if (counter_ % kSampleDecimation != 0U)
            return;

        if (fit_valid_) {
            // Both sides relative to the fit's reference before the Q16 shift: shifting
            // absolute times overflows int64 after about six days at 550 MHz. The
            // difference is bounded by the fit window, so this form has no such limit.
            const auto distance = static_cast<std::int64_t>(counter_ - fit_reference_microframe_);
            const std::int64_t predicted_q16 =
                distance * static_cast<std::int64_t>(fit_ticks_per_microframe_q16_);
            const std::int64_t actual_q16 =
                static_cast<std::int64_t>(extended(now) - fit_reference_time_) << 16U;
            const std::int64_t residual_q16 = predicted_q16 - actual_q16;

            residual_sum_q16_ += residual_q16;
            residual_count_++;
            const auto magnitude =
                static_cast<std::uint64_t>(residual_q16 < 0 ? -residual_q16 : residual_q16);
            if (magnitude > residual_abs_max_q16_ && magnitude <= Policy::kResidualAbsMaxLimit)
                residual_abs_max_q16_ = magnitude;
        }

        if (ring_count_ < kSampleCount) {
            if (ring_count_ == 0U)
                ring_oldest_microframe_ = counter_;
            sample_[(ring_head_ + ring_count_) % kSampleCount] = now;
            ring_count_++;
            return;
        }
        sample_[ring_head_] = now;
        ring_head_ = (ring_head_ + 1U) % kSampleCount;
        ring_oldest_microframe_ += kSampleDecimation;
    }

    // ---- the main loop ----

    // True once every kFitPeriodMs: the fit is recomputed that often.
    bool fit_due(std::uint32_t tick_ms) {
        if (tick_ms - last_fit_tick_ < kFitPeriodMs)
            return false;
        last_fit_tick_ = tick_ms;
        return true;
    }

    // Under the lock: the ring and its origin, consistent with each other.
    void copy_window(Window& window) const {
        window.count = ring_count_;
        window.oldest_microframe = ring_oldest_microframe_;
        window.now = extended(previous_time_);
        for (std::uint32_t index = 0; index < window.count; index++)
            window.sample[index] = sample_[(ring_head_ + index) % kSampleCount];
    }

    // Unlocked, pure: the least-squares line through a full window, or nothing when the
    // window is not full or its slope is not a crystal's.
    static std::optional<Fit> compute_fit(const Window& window) {
        const std::uint32_t count = window.count;
        if (count < kSampleCount)
            return std::nullopt;

        // Least squares over equally spaced x = 0..N-1, y relative to the first sample so
        // the arithmetic stays in 32 bits until it widens. Sxy accumulates as
        // sum((2x - (N-1)) * y), keeping the half-integer mean out of integer arithmetic.
        // Widths at 550 MHz (the largest clock): y spans 5.6e8 over a window, sum_xy2 peaks
        // near 9e12 and its Q16 shift near 6e17, all inside int64 with room to spare.
        const std::uint32_t base = window.sample[0];
        std::int64_t sum_y = 0;
        std::int64_t sum_xy2 = 0;
        for (std::uint32_t index = 0; index < count; index++) {
            const auto y =
                static_cast<std::int64_t>(static_cast<std::int32_t>(window.sample[index] - base));
            sum_y += y;
            sum_xy2 += ((2LL * index) - static_cast<std::int64_t>(count - 1U)) * y;
        }

        // Ticks per decimated sample, then per microframe, both Q16.
        const std::int64_t slope_q16 = (sum_xy2 << 16U) / (2 * kSxx);
        const std::int64_t per_microframe_q16 = slope_q16 / kSampleDecimation;

        // More than about 3% (1/32) off nominal is not crystal drift but a corrupt window;
        // refuse it so a bad fit is never published.
        constexpr std::int64_t nominal_q16 =
            static_cast<std::int64_t>(Policy::kNominalTicksPerMicroframe) << 16U;
        if (per_microframe_q16 < nominal_q16 - (nominal_q16 / 32)
            || per_microframe_q16 > nominal_q16 + (nominal_q16 / 32))
            return std::nullopt;

        // The ring holds the low 32 bits only. Rebuild the oldest sample's full value from
        // the current time: it is at most one window (1.024 s) older, so at most one wrap.
        const std::uint64_t now = window.now;
        std::uint64_t base_extended = (now & ~static_cast<std::uint64_t>(0xFFFFFFFFU)) | base;
        if (base_extended > now)
            base_extended -= static_cast<std::uint64_t>(1) << 32U;

        // The line is evaluated at the newest sample, not the centroid: every query
        // extrapolates forward from now, and anchoring at the leading edge keeps that arm
        // shortest.
        const auto newest_index = static_cast<std::int64_t>(count - 1U);
        const std::int64_t mean_y_q16 = (sum_y << 16U) / count;
        const std::int64_t offset_q16 = mean_y_q16 + ((slope_q16 * newest_index) / 2);

        return Fit{
            .reference_microframe = window.oldest_microframe
                                  + (static_cast<std::uint64_t>(newest_index) * kSampleDecimation),
            .reference_time =
                base_extended + static_cast<std::uint64_t>((offset_q16 + 32768) >> 16U),
            .ticks_per_microframe_q16 = static_cast<std::uint64_t>(per_microframe_q16),
        };
    }

    // Under the lock.
    void publish_fit(const Fit& fit) {
        fit_reference_microframe_ = fit.reference_microframe;
        fit_reference_time_ = fit.reference_time;
        fit_ticks_per_microframe_q16_ = fit.ticks_per_microframe_q16;
        fit_valid_ = true;
        if (state_ == data::TimeState::kWaitingAnchor && anchored_)
            state_ = data::TimeState::kValid;
    }

    // Under the lock. Idempotent while the resolved wrap stays the same; a change on a live
    // timeline is a fault, not something to accept quietly: the counter or the host's
    // estimate moved by more than a second, and everything timed since the last anchor was
    // timed in the wrong second.
    void apply_anchor(std::uint64_t host_microframe) {
        if (state_ == data::TimeState::kInvalid && !counter_seeded_)
            return;

        // The multiple of 16384 that puts the counter nearest the host's estimate; rounded,
        // not truncated, so the decision does not depend on which side of it the counter is.
        const auto difference = static_cast<std::int64_t>(host_microframe - counter_);
        const std::int64_t wraps = floor_div(
            difference + static_cast<std::int64_t>(kMicroframeModulus / 2),
            static_cast<std::int64_t>(kMicroframeModulus));
        const std::int64_t offset = wraps * static_cast<std::int64_t>(kMicroframeModulus);

        if (!anchored_) {
            anchor_offset_ = offset;
            anchored_ = true;
        } else if (offset != anchor_offset_) {
            anomaly_count_++;
            invalidate();
            return;
        }

        if (fit_valid_)
            state_ = data::TimeState::kValid;
        else if (state_ == data::TimeState::kInvalid)
            state_ = data::TimeState::kWaitingAnchor;
    }

    // Under the lock. timestamp_quarter_us is left 0 for the board to fill.
    [[nodiscard]] TimebaseSnapshot snapshot() const {
        const std::int64_t mean_q16 =
            residual_count_ == 0 ? 0
                                 : residual_sum_q16_ / static_cast<std::int64_t>(residual_count_);
        return {
            .state = state_,
            .microframe = current_microframe(),
            .timestamp_quarter_us = 0,
            // Published in the protocol's quarter-us so every board reports the same
            // nominal 500 * 65536 and the host never scales by board.
            .ticks_per_microframe_q16 = static_cast<std::uint32_t>(
                Policy::to_quarter_us(static_cast<std::int64_t>(fit_ticks_per_microframe_q16_))),
            // 24-bit wire field, clamped rather than truncated: a silent wrap would pass off
            // a steady stream of anomalies as a small count.
            .anomaly_count = anomaly_count_ > 0xFFFFFFU ? 0xFFFFFFU : anomaly_count_,
            .residual_mean_q16 = static_cast<std::int32_t>(Policy::to_quarter_us(mean_q16)),
            .residual_abs_max_q16 = static_cast<std::uint32_t>(
                Policy::to_quarter_us(static_cast<std::int64_t>(residual_abs_max_q16_))),
            // Clamped likewise (16-bit field): a host a minute without anchoring must not
            // look like a plausible small count.
            .residual_count = residual_count_ > 0xFFFFU ? 0xFFFFU : residual_count_,
        };
    }

    // Under the lock, right after snapshot() for the periodic report: the mean only means
    // something over a bounded window, and a sum across a re-anchor would mix two fits.
    void clear_residuals() {
        residual_sum_q16_ = 0;
        residual_count_ = 0;
        residual_abs_max_q16_ = 0;
    }

    // Back to power-on. Only while the SOF interrupt is off.
    void reset() {
        invalidate();
        counter_ = 0;
        previous_index_ = 0;
        previous_time_ = 0;
        time_high_ = 0;
        time_seeded_ = false;
        anomaly_count_ = 0;
        anchor_offset_ = 0;
        ring_oldest_microframe_ = 0;
        fit_reference_microframe_ = 0;
        fit_reference_time_ = 0;
    }

    // ---- queries along the fitted line, in fit ticks; under the lock ----

    // The timeline answers only when it is valid and fitted; a caller must not schedule
    // against a dead clock.
    [[nodiscard]] bool answers() const { return state_ == data::TimeState::kValid && fit_valid_; }

    // The counter on the absolute axis once anchored, on the board's own origin before.
    [[nodiscard]] std::uint64_t current_microframe() const {
        return anchored_
                 ? static_cast<std::uint64_t>(static_cast<std::int64_t>(counter_) + anchor_offset_)
                 : counter_;
    }

    // A 32-bit reading of the fit clock, extended with the last SOF's high word.
    [[nodiscard]] std::uint64_t extended(std::uint32_t low) const {
        return (static_cast<std::uint64_t>(time_high_) << 32U) | low;
    }

    // The last SOF's local time, extended.
    [[nodiscard]] std::uint64_t latest_time() const { return extended(previous_time_); }

    // A reading taken after the last SOF: the clock only goes up, so a low word below the
    // last SOF's means it wrapped since, before the next SOF could carry it.
    [[nodiscard]] std::uint64_t extended_after_latest(std::uint32_t low) const {
        std::uint64_t now = extended(low);
        if (low < previous_time_)
            now += static_cast<std::uint64_t>(1) << 32U;
        return now;
    }

    // Fit-clock time of an absolute microframe.
    [[nodiscard]] std::int64_t time_of(std::uint64_t microframe) const {
        const auto local_microframe =
            static_cast<std::uint64_t>(static_cast<std::int64_t>(microframe) - anchor_offset_);
        const auto distance =
            static_cast<std::int64_t>(local_microframe - fit_reference_microframe_);
        const std::int64_t ticks =
            ((distance * static_cast<std::int64_t>(fit_ticks_per_microframe_q16_)) + 32768) >> 16U;
        return static_cast<std::int64_t>(fit_reference_time_) + ticks;
    }

    // Absolute microframe at a fit-clock time (whole microframes, toward zero).
    [[nodiscard]] std::uint64_t microframe_at(std::int64_t time) const {
        const std::int64_t distance = time - static_cast<std::int64_t>(fit_reference_time_);
        const std::int64_t microframes =
            (distance << 16U) / static_cast<std::int64_t>(fit_ticks_per_microframe_q16_);
        return static_cast<std::uint64_t>(
            static_cast<std::int64_t>(fit_reference_microframe_) + microframes + anchor_offset_);
    }

    // Absolute microframe at a fit-clock time with its fraction (1/65536). The Q16 quotient
    // shifts the distance left by 32: beyond 2^30 ticks from the fit's leading edge the fit
    // has long stopped updating, and there is no answer.
    [[nodiscard]] bool microframe_q16_at(
        std::int64_t time, std::uint64_t& out_microframe, std::uint16_t& out_fraction_q16) const {
        const std::int64_t distance = time - static_cast<std::int64_t>(fit_reference_time_);
        constexpr std::int64_t limit = std::int64_t{1} << 30U;
        if (distance <= -limit || distance >= limit)
            return false;

        // Arithmetic shift for the whole part = toward negative infinity, matching the
        // fraction in the low 16 bits.
        const std::int64_t microframes_q16 =
            (distance << 32U) / static_cast<std::int64_t>(fit_ticks_per_microframe_q16_);
        out_microframe = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(fit_reference_microframe_) + (microframes_q16 >> 16U)
            + anchor_offset_);
        out_fraction_q16 = static_cast<std::uint16_t>(microframes_q16 & 0xFFFF);
        return true;
    }

private:
    // sum((x - mean)^2) over equally spaced x = 0..N-1, a constant: N(N^2-1)/12.
    static constexpr std::int64_t kSxx =
        static_cast<std::int64_t>(kSampleCount)
        * ((static_cast<std::int64_t>(kSampleCount) * kSampleCount) - 1) / 12;

    void reset_fit_window() {
        ring_head_ = 0;
        ring_count_ = 0;
        fit_valid_ = false;
        fit_ticks_per_microframe_q16_ = 0;
        residual_sum_q16_ = 0;
        residual_count_ = 0;
        residual_abs_max_q16_ = 0;
    }

    void invalidate() {
        state_ = data::TimeState::kInvalid;
        anchored_ = false;
        counter_seeded_ = false;
        reset_fit_window();
    }

    // ==== written only by the SOF interrupt (note_*), read under the lock ====
    // Plain scalars, not atomics: every reader holds the lock, and these must be read as
    // one consistent set, which per-variable atomicity cannot give.

    // Microframes since power-on, seeded with the first reading so that
    // (counter mod 16384) always equals the hardware's contribution.
    std::uint64_t counter_ = 0;
    bool counter_seeded_ = false;
    std::uint32_t previous_index_ = 0;

    std::uint32_t previous_time_ = 0;
    std::uint32_t time_high_ = 0;
    bool time_seeded_ = false;

    data::TimeState state_ = data::TimeState::kInvalid;
    std::uint32_t anomaly_count_ = 0;
    std::int64_t anchor_offset_ = 0;
    bool anchored_ = false;

    // Fit samples: sample_[i] is the fit clock's low 32 bits at microframe
    // (ring_oldest_microframe_ + i * kSampleDecimation), in insertion order from ring_head_.
    std::uint32_t sample_[kSampleCount]{};
    std::uint32_t ring_head_ = 0;
    std::uint32_t ring_count_ = 0;
    std::uint64_t ring_oldest_microframe_ = 0;

    // ==== the fit: written only by publish_fit(), every reader holds the lock ====
    // The reference point in whole ticks, not Q16: averaging 128 samples is what puts the
    // phase below a single sample's jitter, and rounding to a tick is far below that. The
    // slope stays Q16 -- parts per million of it accumulate along the extrapolation. 64 bits:
    // 68750 cycles per microframe in Q16 is 4.5e9.
    bool fit_valid_ = false;
    std::uint64_t fit_reference_microframe_ = 0;
    std::uint64_t fit_reference_time_ = 0;
    std::uint64_t fit_ticks_per_microframe_q16_ = 0;

    // Out-of-sample prediction error between reports, Q16 fit ticks.
    std::int64_t residual_sum_q16_ = 0;
    std::uint32_t residual_count_ = 0;
    std::uint64_t residual_abs_max_q16_ = 0;

    std::uint32_t last_fit_tick_ = 0;
};

} // namespace libhcs::core::time
