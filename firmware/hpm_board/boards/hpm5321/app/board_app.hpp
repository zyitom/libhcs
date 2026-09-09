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
#include "firmware/hpm_board/common/board_identity.hpp"

namespace libhcs::firmware::board {

// ONE IMAGE, TWO PCBs
//
// This board_app serves both HPM5321 variants -- the single-CAN board and the
// dual-CAN board -- from a single binary. They were separate `boards/`
// directories with ~700 duplicated lines carrying about fifteen lines of real
// difference: how many CAN ports exist, the RGB LED pins, and the per-CAN
// indicator pins. Everything else (board.c, the .yaml, the UART, the clock
// tree, USB) was already identical.
//
// The variant is decided at run time from OTP shadow word 25 -- see
// common/board_identity.hpp for what that word is, what the evidence for it
// actually supports, and why an unrecognized value stops the boot rather than
// picking a default. The bootloader refuses to jump to this app at all unless
// the word is one of the two known values, so by the time any code here runs the
// identity is already known-good.
//
// WHY THE TABLES ARE RUNTIME AND NOT COMPILE-TIME
//
// PA30 and PA31 are the green and red LED cathodes on the single-CAN board and
// MCAN3 RXD/TXD on the dual board. The two pinouts are mutually exclusive on the
// same pads, so this is not a superset that could be configured unconditionally:
// each pad gets exactly one FUNC_CTL, chosen once, from the identity.
//
// The tables are therefore sized for the maximum (two CAN controllers) and
// populated for the maximum, with a runtime count gating which entries are
// actually brought up. That costs one extra MCAN message-RAM slice in AHB SRAM
// (MCAN_MSG_BUF_SIZE_IN_WORDS = 640 words, about 2.5 KiB of the 32 KiB) on the
// single-CAN board, where it goes unused. Making it dynamic instead would buy
// back 2.5 KiB in exchange for losing the static placement the SoC requires --
// not a good trade at 8% of AHB SRAM.

// The HPM5321 USB always runs at high speed.
bool usb_use_high_speed();

// Maximum CAN controllers across both variants. The single-CAN board uses only
// entry 0; kCanPorts is sized and populated for the dual board, and
// can_port_count() reports how many are live on this board.
constexpr size_t kCanPortCapacity = 2;

// CAN ports in table order. Silkscreen CAN1 = kCanPorts[0] = DataId::kCan1;
// silkscreen CAN2 = kCanPorts[1] = DataId::kCan2.
//
// Both variants run CAN-FD, so the table is the same for both PCBs and the
// single-CAN board simply brings up one fewer port. FD is a strict superset of
// classic CAN 2.0: an FD-enabled M_CAN sends and receives classic frames too,
// with the format chosen per element from the FDF/BRS bits, which can.cpp
// drives from the host's per-frame is_fdcan flag. Nothing switches mode at run
// time -- the controller is configured once and stays FD-capable.
//
// This replaces an earlier classic-only MCAN0 on the single-CAN board, which
// existed only to keep that board byte-identical to what shipped. Classic mode
// is strictly weaker with no upside: it cannot receive FD frames at all
// (measured 0/50 from an FD peer, versus 50/50 classic). What still needs
// on-target confirmation is the single-CAN PCB's transceiver at the 5 Mbit data
// phase -- see README.md.
constexpr CanPort kCanPorts[] = {
    {.base = HPM_MCAN0_BASE,
     .irq_num = IRQn_MCAN0,
     .mode = CanMode::kCanFd,
     .data_id = data::DataId::kCan1},
    {.base = HPM_MCAN3_BASE,
     .irq_num = IRQn_MCAN3,
     .mode = CanMode::kCanFd,
     .data_id = data::DataId::kCan2},
};
static_assert(std::size(kCanPorts) == kCanPortCapacity);

// Number of CAN controllers actually present: 2 on the dual board, 1 on the
// single-CAN board. Every loop over the CAN array must bound on this rather than
// on std::size(kCanPorts), or it will bring up an MCAN3 that has no transceiver
// and whose pads are LED cathodes.
inline size_t can_port_count() { return board_identity().dual_can() ? 2U : 1U; }

// The port at `index`. Entries at or above can_port_count() are not valid on
// this board.
constexpr CanPort can_port(size_t index) { return kCanPorts[index]; }

uint32_t init_can(MCAN_Type* ptr);
void can_irq_handler(size_t board_can_index);

// MCAN message RAM region for the CAN controller at the given logical index
// (an .ahb_sram-placed array on this SoC, see board_app.cpp).
mcan_msg_buf_attr_t can_message_ram(size_t can_index);

// PTPC (the shared CAN timestamp timebase) runs on the 160 MHz AHB clock:
// reported-nanosecond step is 6 ns, so true microseconds = reported
// nanoseconds / (160 * 6). The CAN driver asserts this against the clock tree
// at init.
constexpr uint32_t kCanTimestampNsPerUs = 960;

// The application runs on core0: its machine timer is MCHTMR0, clocked at the
// 4 MHz the shared Timer driver expects (board.c).
constexpr clock_name_t kMchtmrClockName = clock_mchtmr0;

// DMA ring storage section for the shared UART driver: AHB SRAM is naturally
// non-cached on this SoC.
#define libhcs_DMA_BUFFER_SECTION ".ahb_sram"

// UART ports in logical order. Both variants have a single data UART (UART2) and
// no DBUS receiver.
constexpr UartPort kUartPorts[] = {
    {
        .base = HPM_UART2_BASE,
        .irq_num = IRQn_UART2,
        .dma_src_tx = HPM_DMA_SRC_UART2_TX,
        .dma_src_rx = HPM_DMA_SRC_UART2_RX,
        .data_id = data::DataId::kUart0,
        .config_data_id = data::DataId::kUart0Config,
        .baudrate = 921600,
        .parity = parity_none,
    },
};

uint32_t init_uart(UART_Type* ptr);
void uart_irq_handler(size_t board_uart_index);

// Plain GPIO RGB LED, active-low (common-anode: drive the pad LOW to light the
// channel). The last template argument is active_high = false.
//
// The two variants wire the LED to different pads, and on the single-CAN board
// two of them (PA30, PA31) are the very pads the dual board gives to MCAN3 --
// which is why these cannot be a single table. led_red_pin() and friends pick the
// set at run time.
constexpr GpioPin kSingleCanLedBluePin = make_gpio_pin<gpiom_soc_gpio0, 'A', 29, false>();
constexpr GpioPin kSingleCanLedGreenPin = make_gpio_pin<gpiom_soc_gpio0, 'A', 30, false>();
constexpr GpioPin kSingleCanLedRedPin = make_gpio_pin<gpiom_soc_gpio0, 'A', 31, false>();

constexpr GpioPin kDualCanLedBluePin = make_gpio_pin<gpiom_soc_gpio0, 'A', 26, false>();
constexpr GpioPin kDualCanLedGreenPin = make_gpio_pin<gpiom_soc_gpio0, 'A', 27, false>();
constexpr GpioPin kDualCanLedRedPin = make_gpio_pin<gpiom_soc_gpio0, 'A', 28, false>();

inline GpioPin led_red_pin() {
    return board_identity().dual_can() ? kDualCanLedRedPin : kSingleCanLedRedPin;
}

inline GpioPin led_green_pin() {
    return board_identity().dual_can() ? kDualCanLedGreenPin : kSingleCanLedGreenPin;
}

inline GpioPin led_blue_pin() {
    return board_identity().dual_can() ? kDualCanLedBluePin : kSingleCanLedBluePin;
}

// Per-CAN indicator LEDs: two on the dual board (active-high), none on the
// single-CAN board. Sized for the maximum; can_indicator_count() reports how many
// exist on this board, and the shared Led driver skips the indicator logic
// entirely when that is zero.
constexpr std::array<GpioPin, 2> kCanIndicatorPins{
    make_gpio_pin<gpiom_soc_gpio0, 'B', 14, true>(),
    make_gpio_pin<gpiom_soc_gpio0, 'B', 15, true>(),
};

inline size_t can_indicator_count() { return board_identity().dual_can() ? 2U : 0U; }

void init_led_pins();
void init_can_indicator_pins();

} // namespace libhcs::firmware::board
