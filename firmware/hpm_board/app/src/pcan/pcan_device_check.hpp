#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "firmware/hpm_board/app/src/pcan/pcan_protocol.hpp"

namespace libhcs::firmware::pcan::device_check {

using Block = std::array<uint8_t, 16>;

// AES-128, FIPS 197: independent byte-oriented implementation. This is used
// only during device initialization, never on the CAN forwarding hot path.
constexpr uint8_t multiply(uint8_t a, uint8_t b) {
    uint8_t value = 0;
    for (unsigned i = 0; i < 8; ++i) {
        if (b & 1U)
            value ^= a;
        a = static_cast<uint8_t>((a << 1) ^ ((a & 0x80U) ? 0x1BU : 0));
        b >>= 1;
    }
    return value;
}

inline constexpr auto kSbox = []() consteval {
    std::array<uint8_t, 256> table{};
    for (unsigned i = 0; i < table.size(); ++i) {
        uint8_t inverse = 1;
        uint8_t power = static_cast<uint8_t>(i);
        for (unsigned exponent = 254; exponent != 0; exponent >>= 1) {
            if (exponent & 1U)
                inverse = multiply(inverse, power);
            power = multiply(power, power);
        }
        uint8_t value = inverse;
        for (unsigned shift = 1; shift < 5; ++shift)
            value ^= static_cast<uint8_t>((inverse << shift) | (inverse >> (8 - shift)));
        table[i] = value ^ 0x63U;
    }
    return table;
}();

inline Block encrypt(Block state, Block key) {
    for (std::size_t i = 0; i < state.size(); ++i)
        state[i] ^= key[i];
    uint8_t rcon = 1;
    for (unsigned round = 1; round <= 10; ++round) {
        Block shifted{};
        for (std::size_t column = 0; column < 4; ++column) {
            for (std::size_t row = 0; row < 4; ++row)
                shifted[4 * column + row] = kSbox[state[4 * ((column + row) % 4) + row]];
        }
        state = shifted;
        if (round != 10) {
            for (std::size_t column = 0; column < 4; ++column) {
                const std::size_t i = 4 * column;
                const uint8_t a = state[i], b = state[i + 1], c = state[i + 2], d = state[i + 3];
                const uint8_t all = a ^ b ^ c ^ d;
                state[i] = a ^ all ^ multiply(a ^ b, 2);
                state[i + 1] = b ^ all ^ multiply(b ^ c, 2);
                state[i + 2] = c ^ all ^ multiply(c ^ d, 2);
                state[i + 3] = d ^ all ^ multiply(d ^ a, 2);
            }
        }
        const std::array<uint8_t, 4> last = {
            static_cast<uint8_t>(kSbox[key[13]] ^ rcon), kSbox[key[14]], kSbox[key[15]],
            kSbox[key[12]]};
        for (std::size_t i = 0; i < 4; ++i)
            key[i] ^= last[i];
        for (std::size_t i = 4; i < key.size(); ++i)
            key[i] ^= key[i - 4];
        for (std::size_t i = 0; i < state.size(); ++i)
            state[i] ^= key[i];
        rcon = multiply(rcon, 2);
    }
    return state;
}

// Device-selected slot 0 of the driver-compatible initialization exchange.
// The block cipher is implemented independently; the host driver is unmodified.
inline constexpr Block kSlot0 = {0x45, 0x28, 0x48, 0x2B, 0x4D, 0x62, 0x51, 0x65,
                                 0x54, 0x68, 0x57, 0x6D, 0x5A, 0x71, 0x34, 0x74};

class Exchange {
public:
    // These are variable-length device records, not arrays of 8-byte CAN
    // configuration commands. Consume the packet so its data never configures CAN.
    [[nodiscard]] bool receive(std::span<const uint8_t> packet) {
        if (packet.size() < 2)
            return false;
        const uint16_t opcode = protocol::load16(packet.data());
        if (opcode != 0xA0 && opcode != 0xD0)
            return false;
        if (packet.size() < 8)
            return true;
        if (opcode == 0xA0) {
            valid_ = false;
            pending_ = false;
            if (protocol::load32(packet.data() + 2) != 16 || packet.size() < 24
                || protocol::load16(packet.data() + 6) != 1)
                return true;
            for (std::size_t i = 0; i < challenge_.size(); ++i)
                challenge_[i] = packet[i + 8];
            valid_ = true;
        } else if (valid_ && protocol::load16(packet.data() + 6) == 1) {
            // Generate for the current request, not for a recorded USB session.
            const auto result = encrypt(challenge_, kSlot0);
            reply_ = {};
            // Command-IN uses the device-record envelope, not the CAN data
            // record's size/type header. The driver reads a 64-byte prefix
            // from this 72-byte D1 record before checking its 17-byte payload.
            protocol::store16(reply_.data(), 0xD1);
            protocol::store32(reply_.data() + 2, 17);
            protocol::store16(reply_.data() + 6, 1);
            reply_[8] = 0;
            for (std::size_t i = 0; i < result.size(); ++i)
                reply_[i + 9] = result[i];
            pending_ = true;
        }
        return true;
    }

    [[nodiscard]] bool pending() const { return pending_; }
    [[nodiscard]] const std::array<uint8_t, 72>& reply() const { return reply_; }
    void completed() { pending_ = false; }
    void reset() { *this = {}; }

private:
    Block challenge_{};
    std::array<uint8_t, 72> reply_{};
    bool valid_ = false;
    bool pending_ = false;
};

} // namespace libhcs::firmware::pcan::device_check
