#pragma once

#include <cstdint>

#include "core/include/libhcs/data/datas.hpp"

namespace libhcs::firmware::board {

enum class CanMode : uint8_t {
    kClassic, // 经典 CAN 2.0, 1 Mbps
    kCanFd,   // CAN-FD, 仲裁 1 Mbps / 数据段 5 Mbps (BRS 开)
};

// 一块板暴露的一个物理 CAN 控制器, 按端口序排列。data_id 是该连接器的丝印
// 编号: CAN0 -> DataId::kCan0 (hpm6e8y), CAN1 -> DataId::kCan1 (5321 及
// 其余)。板在 board_app.hpp 里列出端口; 共享 CAN 层的一切由该表构建, 没有
// 逐端口宏。
struct CanPort {
    uint32_t base;    // 外设基址 HPM_MCANx_BASE
    uint32_t irq_num; // 中断号 IRQn_MCANx
    CanMode mode;
    data::DataId data_id;
};

} // namespace libhcs::firmware::board
