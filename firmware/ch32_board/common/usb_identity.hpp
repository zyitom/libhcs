#pragma once

// USB string-descriptor construction shared by both images on this board.
//
// The application and the bootloader present the same manufacturer and the same
// UID-derived serial number, and differ only in the product string -- which is
// how the host tells a running application apart from a device sitting in DFU
// mode. Header-only and outside app/ and boot/ so neither image's source tree
// depends on the other's.

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace libhcs::firmware::usb {

// USB string descriptors are bLength + bDescriptorType + UTF-16LE payload, and
// bLength is one byte, so a descriptor can carry at most 126 characters. 64 is
// comfortably above every string used here (the longest is the product string,
// a prefix plus the project version).
inline constexpr size_t kMaxStringLength = 64;
inline constexpr size_t kStringDescriptorSize = 2 + (2 * kMaxStringLength);

inline constexpr uint8_t kDescriptorTypeString = 0x03;

inline constexpr std::string_view kManufacturerString = "Helios";

// CH32H417 signature area. The first word is the chip id openocd also reports;
// the second is per-die. Both are folded into the serial: the mixer below
// spreads whatever entropy is there over the whole string, so a partially
// constant source degrades gracefully instead of producing near-identical
// serials. Confirm against a second board before relying on uniqueness.
inline constexpr uintptr_t kSignatureUniqueIdAddress = 0x1FFFF7E8u;

// "D4" prefix mirrors the board-type PID nibble, as on c_board/mc02.
inline constexpr std::string_view kSerialPrefix = "D4";
inline constexpr size_t kSerialWordCount = 2;
// "D4" + per 32-bit word: "-XXXX-XXXX".
inline constexpr size_t kSerialLength = kSerialPrefix.size() + (kSerialWordCount * 10);

inline void write_string_descriptor(uint8_t* descriptor, std::string_view string) {
    const size_t length = string.size() < kMaxStringLength ? string.size() : kMaxStringLength;

    descriptor[0] = static_cast<uint8_t>(2 + (2 * length));
    descriptor[1] = kDescriptorTypeString;
    for (size_t i = 0; i < length; ++i) {
        descriptor[2 + (2 * i)] = static_cast<uint8_t>(string[i]);
        descriptor[3 + (2 * i)] = 0;
    }
}

// Identical to c_board's: the raw words are highly correlated between dies, so
// avalanche them before they become visible digits.
constexpr void mix_uid_entropy(std::array<uint32_t, kSerialWordCount>& uid) {
    auto& [a, b] = uid;

    const auto mix_step = [](uint32_t v) {
        v *= 0x9E3779B9;
        return v ^ (v >> 16);
    };

    a ^= mix_step(b);
    b ^= mix_step(a);
    a ^= mix_step(b >> 5);
    b ^= mix_step(a << 5);
    a += mix_step(b);
    b += mix_step(a);
}

inline char* write_hex_u16(uint16_t value, char* buffer) {
    static constexpr char hex_lut[] = "0123456789ABCDEF";

    for (int shift = 12; shift >= 0; shift -= 4)
        *buffer++ = hex_lut[(value >> shift) & 0xF];
    return buffer;
}

// Build the UID-derived serial string. Both images produce the same value for a
// given die, so a host can follow one board across a DFU cycle.
inline void write_serial_descriptor(uint8_t* descriptor) {
    std::array<uint32_t, kSerialWordCount> uid{};
    const auto* signature = reinterpret_cast<const volatile uint32_t*>(kSignatureUniqueIdAddress);
    for (size_t i = 0; i < uid.size(); ++i)
        uid[i] = signature[i];
    mix_uid_entropy(uid);

    std::array<char, kSerialLength> serial{};
    auto* cursor = serial.data();
    for (const char character : kSerialPrefix)
        *cursor++ = character;
    for (const auto& word : uid) {
        *cursor++ = '-';
        cursor = write_hex_u16(static_cast<uint16_t>(word >> 16), cursor);
        *cursor++ = '-';
        cursor = write_hex_u16(static_cast<uint16_t>(word), cursor);
    }

    write_string_descriptor(descriptor, {serial.data(), serial.size()});
}

} // namespace libhcs::firmware::usb
