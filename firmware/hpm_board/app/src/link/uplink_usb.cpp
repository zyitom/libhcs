// 数据平面归哪个传输层所有: USB vendor class。
//
// 本文件是接缝。link/uplink.hpp 只声明这些名字; 谁拥有数据平面谁定义, 这个
// 选择是应用层的属性, 不是板级的。换归属者因此意味着链接另一个编译单元 --
// 永远不用 #ifdef。
//
// 具体地: 双核应用会提供自己的 uplink_xcore.cpp, 把同样的两个名字绑到跨核
// 过程数据链路, 并不链接本文件。机制全貌即此。更早的版本用
// libhcs_APP_RELEASE_CORE1 做, 在九个文件里撒了 20 处 #if, 因为两个编译单元
// 定义同一符号即是重定义, 只能编译掉其一; 拆文件从源头消除冲突。
//
// 刻意不搬: tud_vendor_rx_cb (bulk OUT 下行) 仍在 usb/vendor.cpp。它读一个
// 文件局部包长, 该变量存在的意义就是让一次调用离开 USB ISR 最热的路径, 搬走
// 会引入跨编译单元调用。除非带着测量数据, 否则不动。

#include <hpm_common.h>

#include "core/src/protocol/serializer.hpp"
#include "firmware/hpm_board/app/src/link/uplink.hpp"
#include "firmware/hpm_board/app/src/usb/vendor.hpp"

namespace libhcs::firmware::link {

// 两者都放 ILM: CAN 与 UART 接收 ISR (均在 ILM) 每帧各调一次, 跨编译单元无法内联。
// 留在 flash 时, ISR 是否为它们付一次 XIP 行填充取决于无关代码的布局
// (USB_OPTIMIZATION_LOG.md 第 14 节)。
ATTR_PLACE_AT(".fast")
core::protocol::Serializer& uplink_serializer() { return usb::vendor->serializer(); }
ATTR_PLACE_AT(".fast")
bool uplink_enabled() { return usb::vendor->session_established(); }

} // namespace libhcs::firmware::link
