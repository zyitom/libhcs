#pragma once

#include <cstdint>

// USB identity and data-pipe layout shared by the host SDK and the firmware.
//
// WHY THIS EXISTS. The HPM5321 image also emulates a DM (Damiao) USB2FDCAN
// adapter so that DMTool can drive the board directly
// (firmware/hpm_board/DMTOOL_PROTOCOL.md). DMTool accepts a device only by an
// exact VID:PID pair out of a table compiled into its binary, and addresses the
// adapter by hard-coded endpoint numbers 0x01-0x03 / 0x81-0x83. Two
// consequences for libhcs, both decided here once instead of as magic numbers
// in three files:
//
//   - The board enumerates under DM's identity, not under 0xA511. The product
//     string ("HCS Agent v<version>") stays the libhcs discriminator: a real DM
//     adapter never reports it (host/src/transport/usb/device_scanner.hpp).
//   - The libhcs bulk pipe moves out of DMTool's way, to interface 3 with
//     endpoints 0x04 / 0x84. Every other board keeps interface 0, 0x01 / 0x81.
//
// Moving the pipe costs the libhcs stream nothing: a bulk endpoint is only
// scheduled by the host controller while a transfer is queued on it, so the
// idle DMTool endpoints are never polled while libhcs runs (and vice versa).
namespace libhcs::core::protocol::usb_identity {

// DM USB2FDCAN ids from DMTool's FDCAN_DEVICE_ID table, one per HPM5321 PCB.
// DMTool treats every entry of that table the same; which PCB answered is also
// reported over EP0 (kGetInterface can_count), the PID just makes it visible
// before the device is opened (dfu-util, udev, lsusb).
inline constexpr uint16_t kDmtoolVendorId = 0x34B7;
inline constexpr uint16_t kHpm5321SingleCanProductId = 0x6877;
inline constexpr uint16_t kHpm5321DualCanProductId = 0x6632;

constexpr bool is_dmtool_identity(uint16_t vendor_id, uint16_t product_id) {
    return vendor_id == kDmtoolVendorId
        && (product_id == kHpm5321SingleCanProductId || product_id == kHpm5321DualCanProductId);
}

// Interface and endpoint pair carrying the libhcs byte stream.
struct DataPipe {
    uint8_t interface_number;
    uint8_t out_endpoint;
    uint8_t in_endpoint;
};

inline constexpr DataPipe kDefaultDataPipe{
    .interface_number = 0x00,
    .out_endpoint = 0x01,
    .in_endpoint = 0x81,
};

// Layout of a board that also answers to DMTool (DMTool owns interfaces 0-2).
inline constexpr DataPipe kDmtoolCoexistDataPipe{
    .interface_number = 0x03,
    .out_endpoint = 0x04,
    .in_endpoint = 0x84,
};

// The identity a device enumerated with decides where its libhcs pipe lives.
constexpr DataPipe data_pipe_for(uint16_t vendor_id, uint16_t product_id) {
    return is_dmtool_identity(vendor_id, product_id) ? kDmtoolCoexistDataPipe : kDefaultDataPipe;
}

} // namespace libhcs::core::protocol::usb_identity
