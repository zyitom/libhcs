#pragma once

#include "core/include/libhcs/spec/hpm5321/ports.hpp"
#include "core/src/link/registry.hpp"
#include "firmware/hpm_board/app/src/ports_binding.hpp"

namespace libhcs::firmware::ports {

// 口表(spec/hpm5321/ports.hpp)到驱动的绑定: 身份与类型取自 spec 的具名描述符, 能力取自
// 驱动类型, 本文件只说"第几个硬件槽位"。Registry::matches() 的 static_assert 保证与口表
// 一一对应 -- 漏绑、多绑、绑错类型都是编译错误。
//
// 镜像按双 CAN 板定容量; 单 CAN 板上 CAN2 的驱动不初始化, 这个口就不存在(CanSlot)。
using Spec = spec::hpm5321::Spec;
using Can1 = CanSlot<0, Spec::Cans::kCan1>;
using Can2 = CanSlot<1, Spec::Cans::kCan2>;
using Uart0 = UartSlot<0, Spec::Uarts::kUart0>;

using Registry = core::link::PortRegistry<Can1, Can2, Uart0>;
static_assert(Registry::matches(Spec::kPorts));

} // namespace libhcs::firmware::ports
