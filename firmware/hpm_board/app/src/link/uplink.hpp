#pragma once

#include "core/src/protocol/serializer.hpp"

namespace libhcs::firmware::link {

// 当前主机传输层 (USB vendor class) 的上行 serializer。以传输层中立方式
// 声明, 使 CAN/UART 驱动可被任何传输应用复用; 拥有传输层的应用负责定义。
core::protocol::Serializer& uplink_serializer();
bool uplink_enabled();

} // namespace libhcs::firmware::link
