#pragma once

#include <cstdint>

#include "core/include/libhcs/data/datas.hpp"

namespace libhcs::firmware::board {

enum class CanMode : uint8_t {
    kClassic, // Classic CAN 2.0, 1Mbps
    kCanFd,   // CAN-FD, 1Mbps arbitration / 5Mbps data phase (BRS on)
};

// One physical CAN controller exposed by a board, in port order. data_id is
// the silkscreen number of that connector: CAN0 -> DataId::kCan0 (hpm6e8y),
// CAN1 -> DataId::kCan1 (5321 / the rest). A board lists its ports in
// board_app.hpp; the shared CAN layer builds everything from that table, so
// there are no per-port macros.
struct CanPort {
    uint32_t base;    // HPM_MCANx_BASE
    uint32_t irq_num; // IRQn_MCANx
    CanMode mode;
    data::DataId data_id;
};

} // namespace libhcs::firmware::board
