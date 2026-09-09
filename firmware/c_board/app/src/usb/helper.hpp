#pragma once

#include "core/src/protocol/serializer.hpp"

namespace libhcs::firmware::usb {

core::protocol::Serializer& get_serializer();

} // namespace libhcs::firmware::usb
