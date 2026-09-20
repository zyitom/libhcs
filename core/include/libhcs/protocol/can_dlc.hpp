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

constexpr std::size_t payload_length(uint8_t dlc) {
    constexpr std::array<uint8_t, 16> kTable{0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return kTable[dlc];
}

constexpr uint8_t dlc_from_payload_len(std::size_t bytes) {
    for (uint8_t dlc = 0U; dlc <= 15U; dlc++) {
        if (payload_length(dlc) == bytes)
            return dlc;
    }
    return kDlcInvalid;
}

} // namespace libhcs::core::protocol
