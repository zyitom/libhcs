#pragma once

#include "core/include/libhcs/spec/hpm6e8y/ports.hpp"
#include "core/src/link/registry.hpp"
#include "firmware/hpm_board/app/src/ports_binding.hpp"

namespace libhcs::firmware::ports {

// 口表(spec/hpm6e8y/ports.hpp)到驱动的绑定: 身份与类型取自 spec 的具名描述符, 能力取自
// 驱动类型, 本文件只说"第几个硬件槽位"。只有四路 CAN: 板上的 UART1 是调试焊盘, 生产
// 镜像不带它的驱动(board_app.hpp 的 kUartPorts 为空)。
using Spec = spec::hpm6e8y::Spec;
using Can0 = CanSlot<0, Spec::Cans::kCan0>;
using Can1 = CanSlot<1, Spec::Cans::kCan1>;
using Can2 = CanSlot<2, Spec::Cans::kCan2>;
using Can3 = CanSlot<3, Spec::Cans::kCan3>;

using Registry = core::link::PortRegistry<Can0, Can1, Can2, Can3>;
static_assert(Registry::matches(Spec::kPorts));

} // namespace libhcs::firmware::ports
