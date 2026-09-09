#include "board_app.hpp"

#include <cstdint>
#include <iterator>

#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_ioc_regs.h>
#include <hpm_iomux.h>
#include <hpm_mcan_regs.h>
#include <hpm_mcan_soc.h>
#include <hpm_soc.h>
#include <hpm_soc_feature.h>
#include <hpm_soc_irq.h>
#include <hpm_uart_regs.h>

#include "core/src/utility/assert.hpp"

namespace libhcs::firmware::board {
namespace {

// MCAN message RAM must live in AHB SRAM on this SoC. Sized for the maximum
// across both variants (see kCanPortCapacity in board_app.hpp): the single-CAN
// board leaves the second slice unused, which costs 2.5 KiB of the 32 KiB AHB
// SRAM and buys a single binary for both PCBs.
static_assert(MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1);
ATTR_PLACE_AT(".ahb_sram")
constinit uint32_t can_msg_buffer[std::size(kCanPorts)][MCAN_MSG_BUF_SIZE_IN_WORDS]{};

uint32_t init_can_clock(MCAN_Type* ptr) {
    if (ptr == HPM_MCAN0) {
        clock_add_to_group(clock_can0, 0);
        clock_set_source_divider(clock_can0, clk_src_pll1_clk0, 10);
        return clock_get_frequency(clock_can0);
    }
    // MCAN3 exists only on the dual-CAN variant. Reached only through init_can(),
    // which the CAN layer calls once per port it actually brings up -- bounded by
    // can_port_count(), so this branch cannot run on the single-CAN board.
    if (ptr == HPM_MCAN3) {
        clock_add_to_group(clock_can3, 0);
        clock_set_source_divider(clock_can3, clk_src_pll1_clk0, 10);
        return clock_get_frequency(clock_can3);
    }
    return 0;
}

uint32_t init_uart_clock(UART_Type* ptr) {
    if (ptr == HPM_UART2) {
        clock_add_to_group(clock_uart2, 0);
        return clock_get_frequency(clock_uart2);
    }
    return 0;
}

} // namespace

uint32_t init_can(MCAN_Type* ptr) {
    if (ptr == HPM_MCAN0) {
        HPM_IOC->PAD[IOC_PAD_PA01].FUNC_CTL = IOC_PA01_FUNC_CTL_MCAN0_RXD;
        HPM_IOC->PAD[IOC_PAD_PA00].FUNC_CTL = IOC_PA00_FUNC_CTL_MCAN0_TXD;
    } else if (ptr == HPM_MCAN3) {
        // PA30/PA31 are the MCAN3 pair on the dual board and the green/red LED
        // cathodes on the single-CAN board. Claiming them for MCAN3 on a
        // single-CAN board would drive a transceiver's TXD output into an LED
        // network, so this assignment is gated by identity twice over: the CAN
        // layer never constructs port 1 unless can_port_count() says 2, and
        // init_led_pins() takes the same pads for the LED on the other variant.
        // Assert rather than silently branch -- reaching here on a single-CAN
        // board means the identity plumbing broke, and that is worth trapping in
        // a debug build instead of energizing the pad.
        core::utility::assert_debug(board_identity().dual_can());
        HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_MCAN3_RXD;
        HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_MCAN3_TXD;
    }
    return init_can_clock(ptr);
}

mcan_msg_buf_attr_t can_message_ram(size_t can_index) {
    return {
        .ram_base = reinterpret_cast<uintptr_t>(&can_msg_buffer[can_index]),
        .ram_size = sizeof(can_msg_buffer[can_index]),
    };
}

uint32_t init_uart(UART_Type* ptr) {
    constexpr uint32_t tx_pad = IOC_PAD_PAD_CTL_PE_SET(1) | // Pull enable
                                IOC_PAD_PAD_CTL_PS_SET(1);  // Pull select - Pull up
    constexpr uint32_t rx_pad = IOC_PAD_PAD_CTL_PE_SET(1) | // Pull enable
                                IOC_PAD_PAD_CTL_PS_SET(1) | // Pull select - Pull up
                                IOC_PAD_PAD_CTL_HYS_SET(1); // Enable Schmitt trigger
    if (ptr == HPM_UART2) {
        HPM_IOC->PAD[IOC_PAD_PB09].FUNC_CTL = IOC_PB09_FUNC_CTL_UART2_RXD;
        HPM_IOC->PAD[IOC_PAD_PB09].PAD_CTL = rx_pad;

        HPM_IOC->PAD[IOC_PAD_PB08].FUNC_CTL = IOC_PB08_FUNC_CTL_UART2_TXD;
        HPM_IOC->PAD[IOC_PAD_PB08].PAD_CTL = tx_pad;
    }
    return init_uart_clock(ptr);
}

void init_led_pins() {
    for (const auto& pin : {led_red_pin(), led_green_pin(), led_blue_pin()}) {
        pin.configure_controller();
        pin.configure_ioc_function();
        pin.configure_pad_control(0);
        pin.set_active(false);
        pin.configure_as_output();
    }
}

void init_can_indicator_pins() {
    // Zero on the single-CAN board, which has no indicator LEDs; two on the dual
    // board. Pull-down enabled so the pad does not float while it is being
    // configured as an output.
    const size_t count = can_indicator_count();
    for (size_t i = 0; i < count; ++i) {
        const auto& pin = kCanIndicatorPins[i];
        pin.configure_controller();
        pin.configure_ioc_function();
        pin.configure_pad_control(IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0));
        pin.set_active(false);
        pin.configure_as_output();
    }
}

bool usb_use_high_speed() {
#if defined(libhcs_APP_USB_FULL_SPEED) && libhcs_APP_USB_FULL_SPEED
    // Test builds only (-Dlibhcs_USB_FULL_SPEED=ON). The PHY is high-speed
    // capable; this forces the slower enumeration so a mixed HS/FS pair can be
    // put on one host controller and the shared SOF time base measured.
    return false;
#else
    return true;
#endif
}

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, can0_isr)
void can0_isr() { can_irq_handler(0); }

// Present in both images. On the single-CAN board MCAN3 is never clocked, never
// configured and its IRQ is never enabled (Can's constructor is what calls
// intc_m_enable_irq_with_priority, and it only runs for ports below
// can_port_count()), so this vector is installed but unreachable. Registering it
// unconditionally is what lets one binary serve both boards; can_irq_handler
// bounds-checks the index besides.
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN3, can1_isr)
void can1_isr() { can_irq_handler(1); }

SDK_DECLARE_EXT_ISR_M(IRQn_UART2, uart0_isr)
void uart0_isr() { uart_irq_handler(0); }

} // namespace libhcs::firmware::board
