#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {
#include "ch32h417.h"
}

#include "core/include/libhcs/data/datas.hpp"

// Board-hardware layer for the single CH32H417EVT variant, mirroring upstream
// hpm_board's boards/<variant>/board_app split (kept as one file here since
// this board has no pro/lite variants). Everything hardware-specific -- which
// GPIO pins a peripheral uses, its alternate function, its kernel clock -- lives
// HERE, not in the board-agnostic drivers under can/ and uart/. The drivers are
// built from the port tables below, so adding a bus means adding a table entry.
//
// Target board: Petros CH32H417M Alef Breakout Rev 1V0 (U1 = CH32H417MEU6,
// QFN88), schematic petros_ch32h417m_alef_breakout.kicad_sch. Which MCU pin sits
// on which connector pin was read off that schematic; the tables below only use
// pins that are actually broken out.
//
//   J2  RPi-Pico-compatible 2.54 mm 40-pin header -- the practical wiring
//       surface. Signal pins, by header pin number:
//        1 PD3   2 PD4   4 PD5   5 PE0   6 PE1   7 PF14  9 PE3  10 PE4
//       11 PE5  12 PE6  14 PE14 15 PE13 16 PE12 17 PE11 19 PE15 20 PD10
//       21 PD11 22 PF0  24 PF1  25 PF2  26 PA9  27 PA10 29 PA11 30 NRST
//       31 PA12 32 PA13 34 PA14 35 PA15 37 PC11
//       (36 = 3V3 out, 39 = VSYS, 40 = VBUS, the rest GND)
//   J3  DF12NB(3.0)-40DP-0.5V board-to-board -- needs a mating carrier, not
//       hand-wirable. Carries the rest: PB0/PB1/PB10..PB14, PC0..PC3, PC6..PC10,
//       PC12, PD0/PD1/PD2/PD6/PD7/PD12..PD15, PE10, PF3/PF4/PF5, PA0/PA4/PA5.
//   J4  1x06 2.54 mm: 1 = 3V3, 2 = GND, 3 = PB9/SWDIO, 4 = PB8/SWCLK,
//       5 = PB4/USART8_TX, 6 = PB3/USART8_RX. The board's intended console --
//       both cores' printf go here (see DEBUG=DEBUG_UART8 in CMakeLists.txt),
//       which is why USART8 is deliberately absent from the table below.
namespace libhcs::firmware::board {

// One pin's routing. CH32H417 selects alternate functions per pin (like STM32's
// AFR), so GPIO_PinAFConfig() must run before GPIO_Init() -- omitting it leaves
// the pin on its default function and the peripheral silently unconnected.
struct PinConfig {
    GPIO_TypeDef* port;
    uint16_t pin_source; // GPIO_PinSourceN (an index, NOT the GPIO_Pin_N bitmask)
    uint8_t alternate;   // GPIO_AFn
};

// One physical CAN controller exposed by the board, in logical order (the first
// entry is can1, the second can2, ...).
//
// Valid pin routings for this part, from the EVT reference
// (EXAM/CAN/Networking/Common/hardware.c), annotated with what the Petros
// CH32H417M Alef Breakout (MEU6, QFN88) actually exposes -- see the connector
// map above:
//   CAN1 TX: PA12(AF9)=J2.31  PA14(AF5)=J2.34  PD1(AF9)=J3.35  PB7(AF3),PB9(AF9)
//   CAN1 RX: PA11(AF9)=J2.29  PA13(AF5)=J2.32  PD0(AF9)=J3.5   PB6(AF3),PB8(AF9)
//   CAN2 TX: PB13(AF9)=J3.28  PB6(AF9)
//   CAN2 RX: PB12(AF9)=J3.13  PB5(AF9)
//   CAN3 TX: PD13(AF5)=J3.11  PF3(AF2)=J3.4    PF7(AF2), PC5(AF6)
//   CAN3 RX: PD12(AF5)=J3.29  PF4(AF2)=J3.37   PF6(AF2), PC4(AF6)
// PB5/PB6/PB7 and PC4/PC5/PF6/PF7 are not bonded out on the MEU6 package, so
// CAN2 has exactly one legal routing here. PB8/PB9 are SWCLK/SWDIO -- never
// route CAN there, it costs the debug interface (see PITFALLS.md).
//
// Consequence for wiring: CAN1 is the only bus reachable from the 2.54 mm
// header. CAN2 (and the unused CAN3) live on J3, which needs a DF12 mating
// carrier. Neither connector carries a transceiver: this is a bare breakout, so
// every CAN pin needs an external CAN PHY before it sees a bus.
struct CanPort {
    uint32_t base; // CANx_BASE
    IRQn_Type irq_num;
    PinConfig tx;
    PinConfig rx;
    data::DataId data_id;
    uint32_t bitrate;
};

// One USART exposed by the board, in logical order (the first entry is uart1,
// the second uart2, ...). The logical index is what the host addresses; it does
// NOT have to match the peripheral number, and here it deliberately does not:
// uart2 is USART3, because USART2's only routing (PD5/PD6, AF7) straddles J2
// and J3 while USART3 has a pair entirely on J2.
//
// Legal routings whose BOTH pins land on the 2.54 mm header (datasheet V1.8
// table 2-1-1):
//   USART1 TX PA9(AF7)=J2.26   RX PA10(AF7)=J2.27
//   USART3 TX PA13(AF4)=J2.32  RX PA14(AF4)=J2.34
// Split or J3-only alternatives, for a carrier board:
//   USART2 TX PD5(AF7)=J2.4    RX PD6(AF7)=J3.34
//   USART3 TX PB10(AF7)=J3.14  RX PB11(AF7)=J3.27   (or RX PC11(AF7)=J2.37)
//   USART8 TX PB4(AF11)=J4.5   RX PB3(AF11)=J4.6    -- reserved for the console
struct UartPort {
    uint32_t base; // USARTx_BASE
    IRQn_Type irq_num;
    PinConfig tx;
    PinConfig rx;
    data::DataId data_id;
    // Downlink field the host addresses runtime reconfiguration to, kept
    // distinct from data_id so a config patch cannot be confused with payload.
    // Must match the config_data_id in core/include/libhcs/spec/ch32_board/uart.hpp.
    data::DataId config_data_id;
    uint32_t baudrate; // power-on rate; the host may change it at runtime
    uint16_t max_receive_size;
};

// Pin choices below are confirmed against the Petros breakout schematic; the
// alternate-function numbers come from the EVT CAN pin table and the CH32H417
// datasheet (V1.8) pin-definition table, cross-checked against the vendor
// drivers that use them (Debug/debug.c for USART1, EXAM/USART for USART2/3).
inline constexpr CanPort kCanPorts[] = {
    {
     // J2.31 (TX) / J2.29 (RX). These double as OTG_FS DP/DM, so using them
     // here forecloses the "fly-wire a second USB socket off J2" escape hatch
     // PITFALLS.md 1.2 mentions; move CAN1 to PA14/PA13 (AF5, J2.34/J2.32) if
     // that port is ever wanted.
     .base = CAN1_BASE,
     .irq_num = CAN1_RX0_IRQn,
     .tx = {GPIOA, GPIO_PinSource12, GPIO_AF9},
     .rx = {GPIOA, GPIO_PinSource11, GPIO_AF9},
     .data_id = data::DataId::kCan1,
     .bitrate = 1'000'000,
     },
    {
     // J3.28 (TX) / J3.13 (RX), the only routing this package allows for CAN2.
     // Reachable only through the DF12 carrier -- see the note on CanPort.
     .base = CAN2_BASE,
     .irq_num = CAN2_RX0_IRQn,
     .tx = {GPIOB, GPIO_PinSource13, GPIO_AF9},
     .rx = {GPIOB, GPIO_PinSource12, GPIO_AF9},
     .data_id = data::DataId::kCan2,
     .bitrate = 1'000'000,
     },
};
inline constexpr size_t kCanPortCount = sizeof(kCanPorts) / sizeof(kCanPorts[0]);

inline constexpr UartPort kUartPorts[] = {
    {
     // J2.26 (TX) / J2.27 (RX). Free only because the V3F boot core's printf was
     // moved off its default USART1 onto the console USART8 (CMakeLists.txt);
     // otherwise the bootloader banner would be shifted out at 921600 baud onto
     // this port on every reset.
     .base = USART1_BASE,
     .irq_num = USART1_IRQn,
     .tx = {GPIOA, GPIO_PinSource9, GPIO_AF7},
     .rx = {GPIOA, GPIO_PinSource10, GPIO_AF7},
     .data_id = data::DataId::kUart1,
     .config_data_id = data::DataId::kUart1Config,
     .baudrate = 115'200,
     .max_receive_size = 64,
     },
    {
     // USART3 on J2.32 (TX) / J2.34 (RX). Mutually exclusive with routing CAN1
     // to its AF5 alternative, which claims the same two pins.
     .base = USART3_BASE,
     .irq_num = USART3_IRQn,
     .tx = {GPIOA, GPIO_PinSource13, GPIO_AF4},
     .rx = {GPIOA, GPIO_PinSource14, GPIO_AF4},
     .data_id = data::DataId::kUart2,
     .config_data_id = data::DataId::kUart2Config,
     .baudrate = 115'200,
     .max_receive_size = 64,
     },
};
inline constexpr size_t kUartPortCount = sizeof(kUartPorts) / sizeof(kUartPorts[0]);

// Kernel clock shared by CAN, USART and the timers on this part. NOT
// SystemCoreClock: on the V5F core that is the 400 MHz core clock, while
// peripherals hang off the ~100 MHz bus clock (SystemClock >> HPRE >> FPRE).
// The vendor's own USB_Timer_Init() derives from HCLK_Frequency the same way,
// which is the ground truth here -- using SystemCoreClock puts every prescaler
// out by the FPRE factor (4x on the stock 400 MHz clock tree).
uint32_t peripheral_clock();

// Pinmux + peripheral clock gating. Both return the peripheral kernel clock in
// Hz so the driver can compute its own prescaler.
uint32_t init_can(const CanPort& port);
uint32_t init_uart(const UartPort& port);

} // namespace libhcs::firmware::board
