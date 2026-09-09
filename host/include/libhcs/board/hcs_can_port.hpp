#pragma once

#include <cstdint>

// CAN port naming for the hpm_board family.
//
// The number is the silkscreen on THAT board: CanPort::kCanN is DataId::kCanN
// is the connector printed CANN.
//
//   5321 / mc02 / c_board / ch32_board print CAN1.. so they start at kCan1.
//   hpm6e8y prints CAN0..CAN3, so its four buses are kCan0..kCan3.
//
// hpm_board used to be the exception: its methods were can0_transmit()/
// can1_transmit() and they wrote DataId::kCan0/kCan1, so the socket printed
// CAN1 answered to "CAN0" in code. That cost a long hardware investigation on
// 2026-09-04. The old can0_transmit()/can1_transmit() methods stay deleted:
// can1_transmit() existed in both schemes with opposite meanings, so keeping
// it would still compile and silently drive the other bus.
//
// kCan4 is gone. hpm6e8y's fourth bus is silk CAN3 / DataId::kCan3, not a
// fifth id. Host and this firmware must ship together.

namespace libhcs::board::hcs {

enum class CanPort : uint8_t {
    kCan0 = 0, // silk "CAN0" -- hpm6e8y first controller, DataId::kCan0
    kCan1 = 1, // socket "CAN1" -- DataId::kCan1 (5321 first controller)
    kCan2 = 2,
    kCan3 = 3,
};

} // namespace libhcs::board::hcs
