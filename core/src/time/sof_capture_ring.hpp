#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "core/include/libhcs/time/sof_stamp.hpp"

namespace libhcs::core::time {

// Turns "the counter read N when this happened" into a position on the shared
// USB microframe axis, for a counter that the hardware also latches at
// Start-of-Frame.
//
// THE IDEA. Two events latched into the SAME free-running counter can be
// compared by subtraction, with no clock conversion between them. So when the
// counter that timestamps a record (a CAN frame's start, say) is also latched
// by SOF edges, the record's place on the microframe axis is
//
//     frame k  +  (record - capture[k]) / (capture[k+1] - capture[k])
//
// -- a ratio of differences of one counter. Its rate does not have to be
// known, stable, or even close to nominal: the denominator measures it over
// the very interval the numerator lies in.
//
// WHY LOCAL, AND NOT A LINE THROUGH THE WHOLE RING. The rate really is not
// stable. On the HPM5321 the SOF captures, read against the PTPC that latches
// them, wander by hundreds of nanoseconds over a few milliseconds, while two
// neighbouring captures are only some 25 ns apart from a straight line
// [measured 2026-10-05, ThreeBoardBench and an on-board diagnostic]. A
// version of this class fitted one least-squares line through 31 captures
// (3.9 ms) and carried it up to a millisecond forward: its prediction of the
// next SOF was off by 100-140 ns RMS, and two boards stamping the same CAN
// frame disagreed by 160 ns RMS. Reading the rate off the captures next to
// the record leaves the wander out.
//
// So a record is placed by the two captures around it when the later one is
// in; when it is not yet -- the usual case, a CAN frame's interrupt comes a
// frame length after its first edge -- by the newest capture and the rate
// over the last kRateBaselineMicroframes, carried forward at most a
// microframe or two.
//
// WHO CALLS WHAT. push() from the SOF interrupt, once per SOF whose capture
// the caller has confirmed belongs to it: it stores it, a handful of
// instructions (on both boards that interrupt is the USB interrupt). locate()
// from wherever the record's timestamp is in hand -- typically a
// HIGHER-priority interrupt, which is why the reader takes no lock: it works
// on the entries that were complete when it looked, and retries in the cases
// where the writer lapped it or started the ring over.
//
// NOT EVERY SOF NEEDS A CAPTURE. The frame-index difference between captures
// is part of the arithmetic. A full-speed port (one SOF per eight
// microframes), a trigger that only latches every other SOF, or a capture the
// caller rejected as stale all just make the bracket wider.
//
// ARITHMETIC. 32-bit divisions only: both ends of the cable run this, one of
// them an RV32 core without a 64-bit divider, in an interrupt.
template <std::size_t capacity>
class SofCaptureRing {
    static_assert(capacity >= 4 && (capacity & (capacity - 1)) == 0, "power of two, at least 4");

public:
    struct Counter {
        // Value at which the counter wraps to zero; 0 means it uses all 32 bits.
        std::uint32_t modulus;
        // Counts per microframe if the counter ran at its nominal rate. Only a
        // plausibility reference: measured rates are accepted within
        // kRateToleranceShift of it.
        std::uint32_t nominal_per_microframe;
    };

    // A rate further than 1/32 (3%) from nominal is not a slow or fast
    // counter, it is captures that are not the frames they claim to be.
    static constexpr std::uint32_t kRateToleranceShift = 5;

    // Widest step between two neighbouring captures, in microframes. A
    // full-speed port spans 8 per SOF; 16 leaves room for one missed capture
    // there. A wider step starts the ring over.
    static constexpr std::uint32_t kMaxBracketMicroframes = 16;

    // How far back the rate is taken for a record past the newest capture.
    // Long enough to average a few captures' jitter out of the rate, short
    // enough that the counter's wander over it stays small. One bracket at
    // least, whatever its width (a full-speed port's is 8).
    static constexpr std::uint32_t kRateBaselineMicroframes = 4;

    // How far past the newest capture a record may lie before the answer is
    // refused. Past it, the SOF side has stopped feeding the ring (suspend, a
    // stalled interrupt) and a confident number would be a guess.
    static constexpr std::uint32_t kMaxExtrapolationMicroframes = 16;

    // The fixed-point rate (Q8 counts per microframe) must stay under 2^25
    // for the division in to_ticks(): nominal plus tolerance under 2^17
    // counts per microframe. HPM5321 PTPC 120000, HPM6E8Y 125000, mc02 TIM5
    // 34375.
    [[nodiscard]] static constexpr bool supports(Counter counter) noexcept {
        return counter.nominal_per_microframe > 0
            && counter.nominal_per_microframe
                       + (counter.nominal_per_microframe >> kRateToleranceShift)
                   < (std::uint32_t{1} << 17);
    }

    constexpr explicit SofCaptureRing(Counter counter) noexcept
        : counter_(counter) {}

    // For a counter only known at run time. Call before the SOF interrupt and
    // the readers start; forgets everything.
    void configure(Counter counter) noexcept {
        counter_ = counter;
        clear();
    }

    // Forget everything. From push()'s context (or with it excluded).
    void clear() noexcept { start_over(); }

    // `frame_index` is the device controller's frame index (in microframes)
    // at the SOF whose edge latched `capture`. A repeat of the newest frame
    // index is dropped: it would make a zero-width bracket.
    //
    // A capture that does not continue the ring -- further from the newest
    // one than kMaxBracketMicroframes -- starts it over. The counter wraps
    // (the HPM PTPC about once a second), so a capture from before a gap (a
    // suspend, a re-enumeration) would sooner or later sit "just before" some
    // unrelated event, and nothing in its value says it is stale.
    void push(std::uint32_t frame_index, std::uint32_t capture) noexcept {
        frame_index &= kFrameMask;
        std::uint32_t held = held_.load(std::memory_order_relaxed);
        if (held != 0) {
            const std::uint32_t step =
                (frame_index - entries_[(held - 1U) & kIndexMask].frame_index) & kFrameMask;
            if (step == 0)
                return;
            if (step > kMaxBracketMicroframes) {
                start_over();
                held = 0;
            }
        }
        entries_[held & kIndexMask] = Entry{.frame_index = frame_index, .capture = capture};
        // Published only after the slot is whole.
        held_.store(held + 1U, std::memory_order_release);
    }

    // Captures pushed since the ring last started over (diagnostics, tests).
    [[nodiscard]] std::uint32_t pushed() const noexcept {
        return held_.load(std::memory_order_relaxed);
    }

    // Where `event` (a reading of the same counter) lies on the microframe
    // axis, or nothing if the ring cannot say: fewer than two captures, the
    // event older than every capture kept, the bracket or the rate not
    // believable, or the newest capture too far behind the event.
    [[nodiscard]] std::optional<libhcs::time::SofStamp> locate(std::uint32_t event) const noexcept {
        for (int attempt = 0; attempt < 3; ++attempt) {
            const std::uint32_t generation = generation_.load(std::memory_order_acquire);
            const std::uint32_t held = held_.load(std::memory_order_acquire);
            const auto result = locate_in(event, held);
            // Keeps the reads above from sinking below the checks. One core,
            // interrupt contexts: a compiler barrier is the whole need.
            std::atomic_signal_fence(std::memory_order_acq_rel);
            // One push during the walk reuses the oldest slot, which the walk
            // never reads. Two means an entry it did read may be half-written;
            // a start-over means none of them belong to the ring any more.
            if (generation_.load(std::memory_order_relaxed) == generation
                && held_.load(std::memory_order_relaxed) - held <= 1U)
                return result;
        }
        return std::nullopt;
    }

private:
    struct Entry {
        std::uint32_t frame_index;
        std::uint32_t capture;
    };

    static constexpr std::uint32_t kIndexMask = capacity - 1;
    static constexpr std::uint32_t kFrameMask =
        (std::uint32_t{1} << libhcs::time::SofStamp::kMicroframeBits) - 1U;

    void start_over() noexcept {
        held_.store(0, std::memory_order_relaxed);
        generation_.store(
            generation_.load(std::memory_order_relaxed) + 1U, std::memory_order_release);
    }

    // Counts from `from` forward to `to`, around the counter's wrap.
    [[nodiscard]] constexpr std::uint32_t
        forward(std::uint32_t from, std::uint32_t to) const noexcept {
        if (counter_.modulus == 0 || to >= from)
            return to - from;
        return to + (counter_.modulus - from);
    }

    [[nodiscard]] constexpr std::uint32_t half_span() const noexcept {
        return counter_.modulus == 0 ? 0x8000'0000U : counter_.modulus / 2U;
    }

    // Counts per microframe between two captures, Q8, or 0 if they do not
    // form a believable span.
    [[nodiscard]] constexpr std::uint32_t
        rate_q8(const Entry& older, const Entry& newer) const noexcept {
        const std::uint32_t microframes = (newer.frame_index - older.frame_index) & kFrameMask;
        if (microframes == 0 || microframes > kMaxBracketMicroframes)
            return 0;
        const std::uint32_t counts = forward(older.capture, newer.capture);
        if (counts >= (std::uint32_t{1} << 24U))
            return 0;
        const std::uint32_t rate = (counts << 8U) / microframes;
        const std::uint32_t nominal_q8 = counter_.nominal_per_microframe << 8U;
        const std::uint32_t tolerance_q8 = nominal_q8 >> kRateToleranceShift;
        if (rate + tolerance_q8 < nominal_q8 || rate > nominal_q8 + tolerance_q8)
            return 0;
        return rate;
    }

    // round(numerator_q8 * 2^kFractionBits / rate_q8) by long division, six
    // bits at a time so that every partial value stays in 32 bits: the
    // remainder is below rate_q8 < 2^25, so shifted by 6 it is below 2^31,
    // with room left for the rounding half.
    [[nodiscard]] static constexpr std::uint32_t
        to_ticks(std::uint32_t numerator_q8, std::uint32_t rate) noexcept {
        std::uint32_t quotient = numerator_q8 / rate;
        std::uint32_t remainder = numerator_q8 % rate;
        std::uint32_t bits = libhcs::time::SofStamp::kFractionBits;
        while (bits > 6U) {
            const std::uint32_t shifted = remainder << 6U;
            quotient = (quotient << 6U) + (shifted / rate);
            remainder = shifted % rate;
            bits -= 6U;
        }
        return (quotient << bits) + (((remainder << bits) + (rate >> 1U)) / rate);
    }

    [[nodiscard]] constexpr std::optional<libhcs::time::SofStamp>
        locate_in(std::uint32_t event, std::uint32_t held) const noexcept {
        if (!supports(counter_))
            return std::nullopt;
        // The slot the writer would fill next holds the oldest entry once the
        // ring is full; leave it out, so a push in flight cannot be observed.
        const std::uint32_t usable =
            held < capacity ? held : static_cast<std::uint32_t>(capacity - 1);
        if (usable < 2)
            return std::nullopt;

        const Entry* newer = nullptr;
        for (std::uint32_t age = 0; age < usable; ++age) {
            const Entry& entry = entries_[(held - 1U - age) & kIndexMask];
            const std::uint32_t ahead = forward(entry.capture, event);
            if (ahead >= half_span()) {
                // This capture is after the event; keep looking back.
                newer = &entry;
                continue;
            }

            // `entry` is the latest capture at or before the event.
            std::uint32_t rate = 0;
            if (newer != nullptr) {
                // Inside a bracket: its own rate, exactly.
                rate = rate_q8(entry, *newer);
            } else {
                // Past the newest capture: the rate over the last few
                // microframes, at least one bracket.
                const Entry* base = nullptr;
                for (std::uint32_t back = age + 1U; back < usable; ++back) {
                    const Entry& older = entries_[(held - 1U - back) & kIndexMask];
                    const std::uint32_t span = (entry.frame_index - older.frame_index) & kFrameMask;
                    if (base != nullptr && span > kRateBaselineMicroframes)
                        break;
                    base = &older;
                }
                if (base == nullptr)
                    return std::nullopt;
                rate = rate_q8(*base, entry);
                if (rate != 0 && ahead >= kMaxExtrapolationMicroframes * (rate >> 8U))
                    return std::nullopt;
            }
            if (rate == 0 || ahead >= (std::uint32_t{1} << 24U))
                return std::nullopt;

            const std::uint32_t ticks = to_ticks(ahead << 8U, rate);
            return libhcs::time::SofStamp{
                (libhcs::time::SofStamp::from(entry.frame_index, 0).ticks + ticks)
                & libhcs::time::SofStamp::kMask};
        }
        return std::nullopt;
    }

    Counter counter_;
    // Written by push() only.
    std::array<Entry, capacity> entries_{};
    // Captures pushed since the last start-over; entry n lives in slot n mod
    // capacity.
    std::atomic<std::uint32_t> held_{0};
    // Bumped by every start-over; a reader that sees it move discards what it
    // read.
    std::atomic<std::uint32_t> generation_{0};
};

} // namespace libhcs::core::time
