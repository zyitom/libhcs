#pragma once

#include <cstdint>

namespace libhcs::core::time {

// How many bits USB bit stuffing inserts into a full-speed SOF packet, given
// its 11-bit frame number.
//
// WHY IT MATTERS. A device raises its SOF event after the whole packet has
// arrived, and on a full-speed port one bit is 83.3 ns. The packet is
// SYNC(8) PID(8) frame(11) CRC5(5) EOP(3) = 35 bit times -- plus one stuffed
// zero after every six consecutive ones in the data, which depends on the
// frame number and its CRC. About one SOF in nine carries one stuffed bit,
// one in 250 two [computed over all 2048 frame numbers], so their edges come
// 83 or 167 ns late. A board that interpolates between SOF captures turns
// that into an error on everything it timestamps; subtracting it per frame
// number takes it out. (At high speed one bit is 2.1 ns and this is noise.)
//
// The rules [USB 2.0 7.1.9, 8.3.5]: stuffing runs from the SYNC pattern
// (whose last bit is a one) through the CRC; fields go least significant bit
// first, except the CRC, which goes most significant bit first; CRC5 is the
// polynomial x^5 + x^2 + 1 over the 11 frame bits, seeded with all ones and
// inverted.
[[nodiscard]] constexpr std::uint32_t usb_token_crc5(std::uint32_t field11) noexcept {
    std::uint32_t remainder = 0x1FU;
    for (std::uint32_t bit = 0; bit < 11U; ++bit) {
        const std::uint32_t in = (field11 >> bit) & 1U;
        const std::uint32_t top = (remainder >> 4U) & 1U;
        remainder = (remainder << 1U) & 0x1FU;
        if ((top ^ in) != 0U)
            remainder ^= 0x05U;
    }
    return ~remainder & 0x1FU;
}

[[nodiscard]] constexpr std::uint32_t full_speed_sof_stuffed_bits(std::uint32_t frame) noexcept {
    frame &= 0x7FFU;
    std::uint32_t run = 1; // the SYNC pattern ends in a one
    std::uint32_t stuffed = 0;
    const auto feed = [&](std::uint32_t bit) {
        if (bit == 0U) {
            run = 0;
            return;
        }
        if (++run == 6U) {
            stuffed++;
            run = 0;
        }
    };
    constexpr std::uint32_t sof_pid = 0xA5U;
    for (std::uint32_t bit = 0; bit < 8U; ++bit)
        feed((sof_pid >> bit) & 1U);
    for (std::uint32_t bit = 0; bit < 11U; ++bit)
        feed((frame >> bit) & 1U);
    const std::uint32_t crc = usb_token_crc5(frame);
    for (std::uint32_t bit = 0; bit < 5U; ++bit)
        feed((crc >> (4U - bit)) & 1U);
    return stuffed;
}

} // namespace libhcs::core::time
