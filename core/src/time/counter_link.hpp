#pragma once

#include <cstdint>
#include <optional>

namespace libhcs::core::time {

// A 16-bit counter that ticks once every 2^shift ticks of a 32-bit counter
// running off the same clock, the 32-bit one wrapping at 2^32:
//
//     narrow = ((wide - offset) >> shift) & 0xFFFF       for a constant offset
//
// That holds for as long as nobody reloads either counter, because 2^32 is a
// whole number of narrow wraps (2^(16 + shift)). The mc02 is the case in
// point: its FDCAN timestamps frames with TIM3 (16 bits), the shared time
// axis lives on TIM5 (32 bits), both on the APB1 timer clock.
//
// What it buys: a reading of the narrow counter latched by hardware (a CAN
// frame's start) becomes an exact reading of the wide one, by splicing the
// narrow value into a wide reading taken afterwards. No second counter is
// read at that moment, so there is no read-to-read skew and no quantisation
// of "now" in the result -- only the narrow counter's own tick.
//
// Pure arithmetic, shared by the firmware and the host tests.
struct CounterLink {
    std::uint32_t shift = 0;
    std::uint32_t offset = 0;

    static constexpr std::uint32_t kNarrowMask = 0xFFFFU;

    [[nodiscard]] constexpr std::uint16_t narrow_at(std::uint32_t wide) const noexcept {
        return static_cast<std::uint16_t>(((wide - offset) >> shift) & kNarrowMask);
    }

    // The wide reading at the middle of the narrow tick that read `stamp`,
    // given a wide reading `now` taken after it. Nothing if `stamp` is
    // `window` narrow ticks old or more: past a full narrow wrap the stamp is
    // ambiguous, and the caller sets `window` below that with margin.
    [[nodiscard]] constexpr std::optional<std::uint32_t>
        widen(std::uint16_t stamp, std::uint32_t now, std::uint32_t window) const noexcept {
        const std::uint32_t relative = now - offset;
        const std::uint32_t narrow_now = (relative >> shift) & kNarrowMask;
        const std::uint32_t age = (narrow_now - stamp) & kNarrowMask;
        if (age >= window)
            return std::nullopt;
        const std::uint32_t tick = std::uint32_t{1} << shift;
        const std::uint32_t tick_start = (relative & ~(tick - 1U)) - (age << shift);
        return tick_start + offset + (tick >> 1U);
    }

    // Whether a narrow reading taken between two wide readings fits the link,
    // allowing `slack` wide ticks either side (the offset may be off by the
    // few clocks of the path it was measured through). False means somebody
    // reloaded one of the counters, or the offset was never right.
    [[nodiscard]] constexpr bool agrees(
        std::uint32_t before, std::uint16_t narrow, std::uint32_t after,
        std::uint32_t slack) const noexcept {
        const std::uint16_t low = narrow_at(before - slack);
        const std::uint16_t high = narrow_at(after + slack);
        return static_cast<std::uint16_t>(narrow - low) <= static_cast<std::uint16_t>(high - low);
    }
};

} // namespace libhcs::core::time
