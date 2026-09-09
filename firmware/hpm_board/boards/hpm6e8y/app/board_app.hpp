#pragma once

#include <array>
#include <cstddef>

#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_gpiom_soc_drv.h>
#include <hpm_iomux.h>
#include <hpm_mcan_regs.h>
#include <hpm_mcan_soc.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>
#include <hpm_uart_regs.h>

#include "firmware/hpm_board/app/src/can/can_port.hpp"
#include "firmware/hpm_board/app/src/gpio/gpio_pin.hpp"
#include "firmware/hpm_board/app/src/uart/uart_port.hpp"

namespace libhcs::firmware::board {

// Fieldbus (core1) application layer of the EtherCAT bridge. This board
// exposes all four physical CAN ports (CAN0..CAN3 = MCAN0..MCAN3; see kCanPorts
// for pin routing) and one test UART (UART1 on PY06/PY07), plus a plain GPIO RGB
// LED. The EtherCAT side (ESC, core0) is configured in ../board.c and does not
// appear here.

// USB0 uses the HPM6E80 high-speed device controller and PHY.
bool usb_use_high_speed();

// CAN ports in logical order, mapped to the four physical silk ports CAN0..CAN3
// (= MCAN0..MCAN3). Pin routing recovered by the CAN pin scanner and recorded in
// CAN_PIN_REVERSE_ENGINEERING.md:
//   CAN0 = MCAN0  TX PC00 / RX PC01
//   CAN1 = MCAN1  TX PB05 / RX PB04
//   CAN2 = MCAN2  TX PD08 / RX PD09
//   CAN3 = MCAN3  TX PD15 / RX PD14
// All four run CAN-FD (1 Mbps arbitration / 5 Mbps data, BRS on). FD mode is a
// strict superset -- classic frames still work -- and is required by the CANFD
// loopback stress test that jumpers CAN0<->CAN1 and CAN2<->CAN3.
constexpr CanPort kCanPorts[] = {
    {.base = HPM_MCAN0_BASE,
     .irq_num = IRQn_MCAN0,
     .mode = CanMode::kCanFd,
     .data_id = data::DataId::kCan0},
    {.base = HPM_MCAN1_BASE,
     .irq_num = IRQn_MCAN1,
     .mode = CanMode::kCanFd,
     .data_id = data::DataId::kCan1},
    {.base = HPM_MCAN2_BASE,
     .irq_num = IRQn_MCAN2,
     .mode = CanMode::kCanFd,
     .data_id = data::DataId::kCan2},
    {.base = HPM_MCAN3_BASE,
     .irq_num = IRQn_MCAN3,
     .mode = CanMode::kCanFd,
     .data_id = data::DataId::kCan3},
};

// Table capacity and the number of controllers actually populated on this board.
// They differ only on boards whose directory serves more than one PCB
// (boards/hpm5321), where the table is sized for the larger variant and this
// count comes from the runtime identity. Here they are the same.
constexpr size_t kCanPortCapacity = std::size(kCanPorts);
constexpr size_t can_port_count() { return kCanPortCapacity; }
constexpr CanPort can_port(size_t index) { return kCanPorts[index]; }

uint32_t init_can(MCAN_Type* ptr);
void can_irq_handler(size_t board_can_index);

// MCAN message RAM on HPM6E80 must live in the 32 KiB AHB RAM at 0xF0200000.
// The core1 linker script exposes no .ahb_sram output section, so the board
// hands out fixed slices of that (otherwise unused) region instead of a
// section-placed array.
mcan_msg_buf_attr_t can_message_ram(size_t can_index);

// PTPC (the shared CAN timestamp timebase) runs on AHB0, pinned to 200 MHz in
// board.c: reported-nanosecond step is 5 ns, so true microseconds = reported
// nanoseconds / (200 * 5). The CAN driver asserts this against the clock tree
// at init -- if board.c changes the AHB0 divider, update this constant.
constexpr uint32_t kCanTimestampNsPerUs = 1000;

// UART ports in logical order: one test data UART (UART1, PY06/PY07 header).
constexpr UartPort kUartPorts[] = {
    {.base = HPM_UART1_BASE,
     .irq_num = IRQn_UART1,
     .dma_src_tx = HPM_DMA_SRC_UART1_TX,
     .dma_src_rx = HPM_DMA_SRC_UART1_RX,
     .data_id = data::DataId::kUart0,
     .config_data_id = data::DataId::kUart0Config,
     .baudrate = 921600,
     .parity = parity_none},
};

uint32_t init_uart(UART_Type* ptr);
void uart_irq_handler(size_t board_uart_index);

// Machine timer of whichever core runs this application layer. Each core sees
// only its own MCHTMR through the same HPM_MCHTMR_BASE window, so the clock name
// must follow the running core: MCHTMR1 for the current EtherCAT bridge (the
// app layer lives on core1), MCHTMR0 for the single-core USB image and for the
// core-swap layout that moves the protocol stack back onto core0. board.c
// clocks BOTH dividers at the 4 MHz the shared Timer driver asserts, so no
// clock-tree change is needed when this flips.
#if defined(BOARD_RUNNING_CORE) && BOARD_RUNNING_CORE == HPM_CORE1
constexpr clock_name_t kMchtmrClockName = clock_mchtmr1;
#else
constexpr clock_name_t kMchtmrClockName = clock_mchtmr0;
#endif

// DMA ring storage section for the shared UART driver. Core1 has no AHB SRAM
// section; the AXI SRAM non-cacheable region serves the same purpose (DMA
// coherent without manual cache maintenance).
#define libhcs_DMA_BUFFER_SECTION ".noncacheable.non_init"

// Main RGB LED, active-LOW (common-anode: drive the pad low to light it). Pads
// verified by the GPIO LED scan (see GPIO_LED_REVERSE_ENGINEERING.md): red=PE05,
// green=PE04, blue=PE03.
//
// These three pads DO carry an ESC0_CTR alt function (PE03=CTR_1, PE04=CTR_2,
// PE05=CTR_3), which the old EVK-derived pinmux used to select. That is no
// longer the case: board.c init_esc_pins() now uses the HPM6E*Y* on-die PHY
// mapping and routes the four CTR signals to PA25 (CTR_0), PA28 (CTR_1),
// PC20 (CTR_2) and PC21 (CTR_3) -- it never writes PE03/PE04/PE05. So the RGB
// LED is exclusively owned by this app layer, independent of which core runs
// init_esc_pins() first. Do not re-add ESC0_CTR on these pads.
constexpr GpioPin kLedRedPin = make_gpio_pin<gpiom_soc_gpio0, 'E', 5, false>();
constexpr GpioPin kLedGreenPin = make_gpio_pin<gpiom_soc_gpio0, 'E', 4, false>();
constexpr GpioPin kLedBluePin = make_gpio_pin<gpiom_soc_gpio0, 'E', 3, false>();

// Accessor form of the three constants above. The shared LED driver calls these
// rather than reading the constants, because a board directory that serves more
// than one PCB has to pick its pads from the runtime identity
// (boards/hpm5321/app/board_app.hpp does). This board has one pinout, so these
// are constant folds.
constexpr GpioPin led_red_pin() { return kLedRedPin; }
constexpr GpioPin led_green_pin() { return kLedGreenPin; }
constexpr GpioPin led_blue_pin() { return kLedBluePin; }

// This board DOES have per-CAN indicator LEDs (green+blue per port); the GPIO LED
// scan confirmed CAN0 green=PC26, CAN1 blue=PE02, CAN2 green=PA09/blue=PB00,
// CAN3 green=PB02/blue=PB03. The full green+blue-per-port mapping is still being
// scanned, so they are not wired up as indicators yet.
constexpr std::array<GpioPin, 0> kCanIndicatorPins{};
constexpr size_t can_indicator_count() { return kCanIndicatorPins.size(); }

void init_led_pins();
void init_can_indicator_pins();

} // namespace libhcs::firmware::board
