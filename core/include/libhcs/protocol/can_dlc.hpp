#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace libhcs::core::protocol {

// CAN payload-length codec: the one table mapping the 4-bit wire DLC to a
// payload byte count, and back. Classic CAN (DLC 0-8) and CAN-FD (DLC 9-15)
// share it; the FD entries are the standard nonlinear step
// 12/16/20/24/32/48/64. Both record-stream directions on every board and the
// host SDK, plus the DMTool bridge, must resolve lengths through these two
// functions -- a length that is not a table entry cannot exist on the wire,
// and encoders must reject it rather than guess.
//
// The record header carries wire DLC in a 3-bit field whose meaning switches
// with the IsLongFrame bit (core/src/protocol/protocol.hpp): short frames
// store bytes-1, long frames store wireDLC - kCanFdLongDlcBase. That split
// exists because the FD lengths 12-64 do not fit "bytes-1" into 3 bits, while
// wireDLC-9 does with one code (7) left reserved.

inline constexpr std::size_t kCanClassicMaxPayload = 8;
inline constexpr std::size_t kCanMaxPayload = 64;
// Wire DLC of the shortest long frame (12 bytes): 9. The record header's
// 3-bit long-form field stores wireDLC minus this base.
inline constexpr uint8_t kCanFdLongDlcBase = 9;
// dlc_from_payload_len() result for a byte count that is not a legal frame
// length on the wire (9-11, 13-15, 17-19, ... -- the gaps between FD steps).
inline constexpr uint8_t kDlcInvalid = 0xFF;

// The normative mapping (ISO 11898-1:2015, Table 5), indexed by wire DLC. The two
// functions below do not read it at run time -- it is the specification they are
// proven against at compile time (see the static_assert at the end of this file).
inline constexpr std::array<uint8_t, 16> kDlcPayloadTable{0, 1,  2,  3,  4,  5,  6,  7,
                                                          8, 12, 16, 20, 24, 32, 48, 64};

// Wire DLC -> payload bytes. The DLC field is 4 bits wide: only the low nibble is
// meaningful, so a wider argument is masked instead of indexing past the table
// (the mask costs nothing where the caller already holds a 4-bit field).
//
// This sits on the per-frame receive path of every board. It used to index a local
// constexpr table, which GCC rebuilt on the stack from immediates on every call --
// about ten instructions per received frame. Classic lengths are the identity and
// now cost one compare; the seven FD codes are two arithmetic runs:
// 9-12 -> 12/16/20/24 (step 4) and 13-15 -> 32/48/64 (step 16).
[[nodiscard]] constexpr std::size_t payload_length(uint8_t dlc) noexcept {
    dlc &= 0x0FU;
    if (dlc <= kCanClassicMaxPayload) [[likely]]
        return dlc;
    return dlc <= 12U ? (dlc - 6U) * 4U : (dlc - 11U) * 16U;
}

// Payload bytes -> wire DLC, or kDlcInvalid for a length with no wire encoding
// (9-11, 13-15, 17-19, ... -- the gaps between FD steps). The exact inverse of
// payload_length() over the table; closed form instead of the linear search this
// used to be.
[[nodiscard]] constexpr uint8_t dlc_from_payload_len(std::size_t bytes) noexcept {
    if (bytes <= kCanClassicMaxPayload)
        return static_cast<uint8_t>(bytes);
    // Past 8, the multiples of 4 up to 24 are exactly 12/16/20/24.
    if (bytes <= 24U)
        return (bytes % 4U == 0U) ? static_cast<uint8_t>((bytes / 4U) + 6U) : kDlcInvalid;
    if (bytes <= kCanMaxPayload)
        return (bytes % 16U == 0U) ? static_cast<uint8_t>((bytes / 16U) + 11U) : kDlcInvalid;
    return kDlcInvalid;
}

namespace detail {

// Proof that the fast forms above implement the normative table exactly: every DLC
// maps to its table length and back, and a byte count has a DLC if and only if it
// is a table entry -- checked past the 64-byte maximum and on DLC arguments with
// stray high bits.
consteval bool can_dlc_codec_matches_table() {
    for (std::size_t dlc = 0U; dlc < kDlcPayloadTable.size(); ++dlc) {
        const auto dlc_byte = static_cast<uint8_t>(dlc);
        const auto bytes = kDlcPayloadTable[dlc_byte];
        if (payload_length(dlc_byte) != bytes || payload_length(dlc_byte | 0xF0U) != bytes
            || dlc_from_payload_len(bytes) != dlc_byte)
            return false;
    }
    for (std::size_t bytes = 0U; bytes <= 2U * kCanMaxPayload; ++bytes) {
        bool on_table = false;
        for (const auto entry : kDlcPayloadTable)
            on_table = on_table || entry == bytes;
        if (on_table != (dlc_from_payload_len(bytes) != kDlcInvalid))
            return false;
    }
    return true;
}

} // namespace detail

static_assert(detail::can_dlc_codec_matches_table());

} // namespace libhcs::core::protocol
