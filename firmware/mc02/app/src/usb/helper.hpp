#pragma once

#include "core/src/protocol/serializer.hpp"

namespace libhcs::firmware::usb {

core::protocol::Serializer& get_serializer();

// Whether a host has completed the nonce handshake and is holding the keepalive
// lease. Declared here rather than reached through `vendor` so that
// interrupt_safe_buffer.hpp, which vendor.hpp itself includes, can ask.
//
// Uplink producers need this because the consumer already has it: try_transmit()
// refuses to drain the uplink ring without a session, and activate_session()
// clears the ring outright when one arrives. Everything written while
// disconnected is therefore discarded by construction, so a full ring in that
// state is the expected resting condition and not a fault to report.
bool uplink_session_active();

} // namespace libhcs::firmware::usb
