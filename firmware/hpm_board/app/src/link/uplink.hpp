#pragma once

#include "core/src/protocol/serializer.hpp"

namespace libhcs::firmware::link {

// Uplink serializer of the active host transport (the USB vendor class).
// Declared transport-neutrally so the CAN/UART drivers can be reused by any
// transport application; the application that owns the transport defines it.
core::protocol::Serializer& uplink_serializer();
bool uplink_enabled();

// Uplink serializer for CAN specifically. Identical to uplink_serializer() on
// every transport except a USB build with libhcs_SPLIT_CAN_ENDPOINT, where CAN
// gets its own batch pool and its own bulk endpoint so a UART batch cannot
// occupy the pipe ahead of it. Declared separately rather than switched inside
// uplink_serializer() because UART and the diagnostics channel must keep using
// the bulk pipe -- only CAN moves.
core::protocol::Serializer& can_uplink_serializer();

} // namespace libhcs::firmware::link
