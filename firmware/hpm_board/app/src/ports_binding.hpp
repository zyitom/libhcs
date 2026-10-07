#pragma once

#include <cstddef>

#include "board_app.hpp"
#include "core/src/link/registry.hpp"
#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/uart/uart.hpp"

// hpm 两块板共用的绑定形状: spec 的一个具名描述符 -> 板级硬件表(board::kCanPorts /
// kUartPorts)的一个槽位 -> 该槽位构造出的驱动。
//
// 驱动的身份(数据流按它打标)来自硬件表项, EP0 的寻址来自 spec 描述符; 硬件表项的
// data_id 本身就引用 spec 常量, 这里的 static_assert 再钉死"第 index 个槽位就是这个
// 描述符" -- 两张表的行序一旦与绑定对不上就是编译错误, 而不是 EP0 配了 CAN1、数据流
// 却标成 CAN2。
namespace libhcs::firmware::ports {

template <std::size_t index, auto kDescriptor>
struct CanSlot : core::link::PortOf<kDescriptor> {
    static_assert(index < std::size(board::kCanPorts));
    static_assert(
        board::kCanPorts[index].data_id == kDescriptor.data_id,
        "the CAN hardware table row and its spec descriptor disagree");

    // 单 CAN 的 hpm5321 上 App 只初始化槽位 0(第二个槽位的焊盘实为 LED 阴极), 槽位 1
    // 的驱动从未构造, try_get() 为空: 它不出现在 kGetPortList, 清单里声明它也会被拒。
    static can::Can* instance() { return can::can_array[index].try_get(); }
};

template <std::size_t index, auto kDescriptor>
struct UartSlot : core::link::PortOf<kDescriptor> {
    static_assert(index < std::size(board::kUartPorts));
    static_assert(
        board::kUartPorts[index].data_id == kDescriptor.data_id,
        "the UART hardware table row and its spec descriptor disagree");

    static uart::Uart* instance() { return uart::uart_array[index].try_get(); }
};

} // namespace libhcs::firmware::ports
