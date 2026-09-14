#pragma once

#include "core/src/protocol/serializer.hpp"

namespace libhcs::firmware::usb {

core::protocol::Serializer& get_serializer();

// 主机是否已完成 nonce 握手并持有 keepalive 租约。声明在此而非经 `vendor` 获取,
// 是因为 interrupt_safe_buffer.hpp(vendor.hpp 自己也包含它)需要查询。
//
// 上行生产者需要它, 而消费侧已经持有: try_transmit() 无会话时拒绝排空上行环,
// activate_session() 在会话建立时直接清环。断连期间写入的一切按构造即被丢弃,
// 因此该状态下环满是预期稳态, 不是需要上报的故障。
bool uplink_session_active();

} // namespace libhcs::firmware::usb
