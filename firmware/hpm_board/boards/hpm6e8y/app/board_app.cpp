#include "board_app.hpp"

#include <cstddef>
#include <cstdint>
#include <iterator>

#include <hpm_clock_drv.h>
#include <hpm_gpio_drv.h>
#include <hpm_ioc_regs.h>
#include <hpm_iomux.h>
#include <hpm_mcan_soc.h>
#include <hpm_pmic_iomux.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>
#include <hpm_uart_regs.h>

namespace libhcs::firmware::board {
namespace {

uint32_t init_can_clock(MCAN_Type* ptr) {
    // Group membership (group1, the core1 domain) is set centrally in board.c
    // before core1 is released; only the divider is chosen here. 80 MHz from
    // PLL1CLK0 (800 MHz / 10), same as the EVK reference, for every controller.
    if (ptr == HPM_MCAN0) {
        clock_set_source_divider(clock_can0, clk_src_pll1_clk0, 10);
        return clock_get_frequency(clock_can0);
    }
    if (ptr == HPM_MCAN1) {
        clock_set_source_divider(clock_can1, clk_src_pll1_clk0, 10);
        return clock_get_frequency(clock_can1);
    }
    if (ptr == HPM_MCAN2) {
        clock_set_source_divider(clock_can2, clk_src_pll1_clk0, 10);
        return clock_get_frequency(clock_can2);
    }
    if (ptr == HPM_MCAN3) {
        clock_set_source_divider(clock_can3, clk_src_pll1_clk0, 10);
        return clock_get_frequency(clock_can3);
    }
    return 0;
}

uint32_t init_uart_clock(UART_Type* ptr) {
    if (ptr == HPM_UART1) {
        // Clocked from group1 (see board.c); default 24 MHz OSC source.
        return clock_get_frequency(clock_uart1);
    }
    return 0;
}

} // namespace

bool usb_use_high_speed() { return true; }

uint32_t init_can(MCAN_Type* ptr) {
    // RX pads: pull-up enabled + Schmitt trigger, so an idle/unconnected bus
    // reads recessive cleanly. TX pads take the IOC default drive.
    constexpr uint32_t rx_pad =
        IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_HYS_SET(1);
    if (ptr == HPM_MCAN0) {
        HPM_IOC->PAD[IOC_PAD_PC00].FUNC_CTL = IOC_PC00_FUNC_CTL_MCAN0_TXD;
        HPM_IOC->PAD[IOC_PAD_PC01].FUNC_CTL = IOC_PC01_FUNC_CTL_MCAN0_RXD;
        HPM_IOC->PAD[IOC_PAD_PC01].PAD_CTL = rx_pad;
    } else if (ptr == HPM_MCAN1) {
        HPM_IOC->PAD[IOC_PAD_PB05].FUNC_CTL = IOC_PB05_FUNC_CTL_MCAN1_TXD;
        HPM_IOC->PAD[IOC_PAD_PB04].FUNC_CTL = IOC_PB04_FUNC_CTL_MCAN1_RXD;
        HPM_IOC->PAD[IOC_PAD_PB04].PAD_CTL = rx_pad;
    } else if (ptr == HPM_MCAN2) {
        HPM_IOC->PAD[IOC_PAD_PD08].FUNC_CTL = IOC_PD08_FUNC_CTL_MCAN2_TXD;
        HPM_IOC->PAD[IOC_PAD_PD09].FUNC_CTL = IOC_PD09_FUNC_CTL_MCAN2_RXD;
        HPM_IOC->PAD[IOC_PAD_PD09].PAD_CTL = rx_pad;
    } else if (ptr == HPM_MCAN3) {
        HPM_IOC->PAD[IOC_PAD_PD15].FUNC_CTL = IOC_PD15_FUNC_CTL_MCAN3_TXD;
        HPM_IOC->PAD[IOC_PAD_PD14].FUNC_CTL = IOC_PD14_FUNC_CTL_MCAN3_RXD;
        HPM_IOC->PAD[IOC_PAD_PD14].PAD_CTL = rx_pad;
    }
    return init_can_clock(ptr);
}

mcan_msg_buf_attr_t can_message_ram(size_t can_index) {
    // Fixed slices of the 32 KiB AHB RAM (0xF0200000), which nothing else in
    // this firmware uses; see the declaration for why this is not a
    // section-placed array. One default-sized message buffer per controller.
    constexpr uint32_t slice_size = MCAN_MSG_BUF_SIZE_IN_WORDS * sizeof(uint32_t);
    static_assert(
        std::size(kCanPorts) * slice_size
        <= MCAN_MSG_BUF_BASE_VALID_END - MCAN_MSG_BUF_BASE_VALID_START);
    return {
        .ram_base = MCAN_MSG_BUF_BASE_VALID_START + (can_index * slice_size),
        .ram_size = slice_size,
    };
}

uint32_t init_uart(UART_Type* ptr) {
    constexpr uint32_t tx_pad = IOC_PAD_PAD_CTL_PE_SET(1) | // Pull enable
                                IOC_PAD_PAD_CTL_PS_SET(1);  // Pull select - Pull up
    constexpr uint32_t rx_pad = IOC_PAD_PAD_CTL_PE_SET(1) | // Pull enable
                                IOC_PAD_PAD_CTL_PS_SET(1) | // Pull select - Pull up
                                IOC_PAD_PAD_CTL_HYS_SET(1); // Enable Schmitt trigger
    if (ptr == HPM_UART1) {
        // PY pads: route through PIOC to the SoC domain in addition to IOC.
        HPM_IOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_UART1_TXD;
        HPM_PIOC->PAD[IOC_PAD_PY07].FUNC_CTL = PIOC_PY07_FUNC_CTL_SOC_PY_07;
        HPM_IOC->PAD[IOC_PAD_PY07].PAD_CTL = tx_pad;

        HPM_IOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY06_FUNC_CTL_UART1_RXD;
        HPM_PIOC->PAD[IOC_PAD_PY06].FUNC_CTL = PIOC_PY06_FUNC_CTL_SOC_PY_06;
        HPM_IOC->PAD[IOC_PAD_PY06].PAD_CTL = rx_pad;
    }
    return init_uart_clock(ptr);
}

void init_led_pins() {
    for (const auto& pin : {kLedRedPin, kLedGreenPin, kLedBluePin}) {
        pin.configure_controller();
        pin.configure_ioc_function();
        pin.configure_pad_control(0);
        pin.set_active(false);
        pin.configure_as_output();
    }
}

void init_can_indicator_pins() {
    // No per-CAN indicator LEDs on this board.
}

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, can0_isr)
void can0_isr() { can_irq_handler(0); }

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN1, can1_isr)
void can1_isr() { can_irq_handler(1); }

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN2, can2_isr)
void can2_isr() { can_irq_handler(2); }

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN3, can3_isr)
void can3_isr() { can_irq_handler(3); }

SDK_DECLARE_EXT_ISR_M(IRQn_UART1, uart0_isr)
void uart0_isr() { uart_irq_handler(0); }

} // namespace libhcs::firmware::board
