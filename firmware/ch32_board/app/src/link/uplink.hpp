#pragma once

#include "core/src/protocol/serializer.hpp"

namespace libhcs::firmware::link {

// Uplink serializer of the active host transport (USB SuperSpeed vendor class on
// this board). Declared transport-neutrally, as on hpm_board, so the CAN/UART
// drivers never include the USB layer: the application that owns the transport
// defines these (see usb/vendor.cpp).
core::protocol::Serializer& uplink_serializer();

// True once the host has acked kStart: telemetry may enter the uplink stream.
// Distinct from mere transport connectivity (USB enumeration) -- serializing
// before the session is up would fill the batch ring with frames no one reads.
bool uplink_enabled();

} // namespace libhcs::firmware::link
