#pragma once

#include <cstdint>

namespace libhcs::time {

// A point on the shared USB microframe axis, in the form it travels next to a
// record on the wire.
//
// WHAT THE AXIS IS. Every board on one host controller counts the same
// Start-of-Frame packets, so "microframe k" names the same instant on all of
// them (firmware/hpm_board/SOF_TIMEBASE.md). A stamp is a position on that
// axis: the low 10 bits of the microframe number -- the low bits of the device
// controller's frame index register itself, identical on every board --
// followed by a 14-bit binary fraction of a microframe.
//
// WHY THIS SPLIT OF 24 BITS. The fraction resolves 125 us / 16384 = 7.6 ns,
// as fine as the counters that produce it: an HPM board latches both edges
// into a 6.25 ns counter, the mc02 into a 7.3 ns one. The first version of
// this field had 10 fraction bits (122 ns), and that quantisation alone
// (35 ns RMS) outweighed everything else on the path; a 13-bit version
// (15 ns, 4.4 ns RMS) was still the largest single term, ahead of the CAN
// controllers' own 12.5 ns edge synchronisation (3.6 ns RMS).
//
// The integer part only has to be wide enough for the receiver to resolve the
// rest from its own idea of "now" (see unwrap()): 10 bits span 128 ms, against
// a record that is a fraction of a millisecond old when it is resolved. A
// board still produces it without knowing the absolute microframe -- it is the
// hardware counter, masked: no anchor, no state machine, nothing that can be
// wrong by a whole second.
//
// It replaced a 32-bit microsecond count of a free-running board clock: one
// byte shorter, 131 times finer, and on an axis the host can place on its own
// clock -- which the old field never was.
struct SofStamp {
    static constexpr std::uint32_t kFractionBits = 14;
    static constexpr std::uint32_t kMicroframeBits = 10;
    static constexpr std::uint32_t kBits = kMicroframeBits + kFractionBits;
    static constexpr std::uint32_t kTicksPerMicroframe = std::uint32_t{1} << kFractionBits;
    static constexpr std::uint32_t kModulus = std::uint32_t{1} << kBits;
    static constexpr std::uint32_t kMask = kModulus - 1U;

    // Position modulo kModulus, in units of 1/16384 microframe.
    std::uint32_t ticks = 0;

    // `fraction_ticks` may exceed one microframe: a board that interpolates
    // across a gap in its captures lands past the reference frame, and the
    // carry into the frame index is exactly what the addition does.
    [[nodiscard]] static constexpr SofStamp
        from(std::uint32_t frame_index, std::uint32_t fraction_ticks) noexcept {
        return SofStamp{((frame_index << kFractionBits) + fraction_ticks) & kMask};
    }

    [[nodiscard]] constexpr std::uint32_t frame_index() const noexcept {
        return ticks >> kFractionBits;
    }
    [[nodiscard]] constexpr std::uint32_t fraction() const noexcept {
        return ticks & (kTicksPerMicroframe - 1U);
    }

    friend constexpr bool operator==(SofStamp, SofStamp) noexcept = default;
};

// The position on the unbounded axis that `stamp` stands for, given a rough
// idea of when it was taken. Both in stamp ticks (1/16384 microframe).
//
// The answer is the one value congruent to the stamp inside
//     [around - 3/4 span, around + 1/4 span)
// -- lopsided on purpose. A stamp describes something that already happened,
// so most of the window lies behind `around`; the quarter span ahead (32 ms)
// is there because `around` is somebody's estimate of the present, and a
// control loop that passes "the start of this cycle" routinely handles records
// that arrived a little after that.
//
// What it cannot do: tell a record that is more than 96 ms old from a fresh
// one. A live session resolves a record when it arrives, well under a
// millisecond after it was stamped; a caller replaying a log, or holding
// records back, must supply `around` from when the record was received.
[[nodiscard]] constexpr std::int64_t unwrap(SofStamp stamp, std::int64_t around) noexcept {
    constexpr auto modulus = static_cast<std::int64_t>(SofStamp::kModulus);
    const std::int64_t window_start = around - ((modulus / 4) * 3);
    std::int64_t offset = (static_cast<std::int64_t>(stamp.ticks) - window_start) % modulus;
    if (offset < 0)
        offset += modulus;
    return window_start + offset;
}

} // namespace libhcs::time
