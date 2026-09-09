// Which transport owns the data plane: the USB vendor class.
//
// This file is the seam. link/uplink.hpp only DECLARES these; whoever owns the
// data plane defines them, and that choice is a property of the APPLICATION,
// not of the board. Selecting a different owner therefore means linking a
// different translation unit -- never an #ifdef.
//
// Concretely: a dual-core application would ship its own uplink_xcore.cpp
// binding the same three names to the cross-core process-data link, and simply
// not link this file. That is the whole mechanism. An earlier revision did it
// with libhcs_APP_RELEASE_CORE1 instead, which spread 20 #if sites across nine
// files, because two translation units defining the same symbols is a duplicate
// -- so one had to be compiled out. Splitting the file removes the conflict at
// its source.
//
// NOT here, deliberately: tud_vendor_rx_cb (the bulk OUT downlink) still lives
// in usb/vendor.cpp. It reads a file-local packet size that exists precisely to
// keep a call off the USB ISR's hottest path, so relocating it would add a
// cross-TU call there. Move it only with a measurement in hand.

#include "firmware/hpm_board/app/src/link/uplink.hpp"

#include "core/src/protocol/serializer.hpp"
#include "firmware/hpm_board/app/src/usb/vendor.hpp"

namespace libhcs::firmware::link {

core::protocol::Serializer& uplink_serializer() { return usb::vendor->serializer(); }
bool uplink_enabled() { return usb::vendor->session_established(); }

// CAN shares the one bulk pipe with UART. A second pair existed 2026-08-07 to
// 2026-09-05 and was removed: see firmware/hpm_board/AGENTS.md for why the
// head-of-line blocking it avoided is cheaper than the endpoint it cost.
core::protocol::Serializer& can_uplink_serializer() { return usb::vendor->serializer(); }

} // namespace libhcs::firmware::link
