#include "board.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include <hpm_batt_iomux.h>
#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_debug_console.h>
#include <hpm_enet_drv.h>
#include <hpm_uart_drv.h> /* board_console_try_send_byte: non-blocking TX check */
#include <hpm_esc_drv.h>
#include <hpm_gpio_drv.h>
#include <hpm_gpiom_drv.h>
#include <hpm_gpiom_soc_drv.h>
#include <hpm_ioc_regs.h>
#include <hpm_iomux.h>
#include <hpm_pcfg_drv.h>
#include <hpm_pllctlv2_drv.h>
#include <hpm_pmic_iomux.h>
#include <hpm_pmp_drv.h>
#include <hpm_soc.h>
#include <hpm_sysctl_drv.h>
#include <hpm_usb_drv.h>

/* Pin-level configuration is based on board reverse engineering. The HPM6E*Y*
 * package uses on-die 100M PHYs for the two EtherCAT ports. */

#if defined(FLASH_XIP) && FLASH_XIP
__attribute__((section(".nor_cfg_option"), used))
const uint32_t kOption[4] = {0xfcf90002, 0x00000007, 0x00001000, 0x0};
#endif

#if defined(FLASH_UF2) && FLASH_UF2
ATTR_PLACE_AT(".uf2_signature")
__attribute__((used)) const uint32_t kUf2Signature = BOARD_UF2_SIGNATURE;
#endif

static inline void board_init_clock(void);
static inline void init_esc_pins(void);
static inline void configure_esc_internal_phy_interface(ESC_Type* ptr);
static inline void configure_internal_phy_mdio_as_enet(void);
static inline void configure_internal_phy_mdio_as_esc(void);
static bool configure_internal_phy_mii_mode(uint8_t phy_addr, uint16_t* rmsr_p7);
static void board_ecat_apply_internal_phy_link_status(const board_ecat_phy_status_t* status);

#define BOARD_ECAT_PHY_REG_BMCR           (0U)
#define BOARD_ECAT_PHY_REG_BMSR           (1U)
#define BOARD_ECAT_PHY_REG_ID1            (2U)
#define BOARD_ECAT_PHY_REG_ID2            (3U)
#define BOARD_ECAT_PHY_REG_RMSR_P7        (16U)
#define BOARD_ECAT_PHY_REG_PAGESEL        (31U)
#define BOARD_ECAT_PHY_PAGE_RMSR          (7U)
#define BOARD_ECAT_PHY_BMSR_LINK_MASK     (0x0004U)
#define BOARD_ECAT_PHY_RMSR_MII_MODE_MASK (0x0008U)

/* Console: UART0 on PA00/PA01, routed to the on-board FT2232 (DEBUGUART0). */
#define BOARD_CONSOLE_UART_BASE     HPM_UART0
#define BOARD_CONSOLE_UART_CLK_NAME clock_uart0
#define BOARD_CONSOLE_BAUDRATE      (115200UL)

void board_init_console(void) {
    console_config_t cfg;

    /* Configure the pin function before enabling the clock, otherwise the rx
     * level change during muxing can be latched as a spurious byte. */
    HPM_IOC->PAD[IOC_PAD_PA00].FUNC_CTL = IOC_PA00_FUNC_CTL_UART0_TXD;
    HPM_IOC->PAD[IOC_PAD_PA01].FUNC_CTL = IOC_PA01_FUNC_CTL_UART0_RXD;

    clock_add_to_group(BOARD_CONSOLE_UART_CLK_NAME, 0);

    cfg.type = CONSOLE_TYPE_UART;
    cfg.base = (uint32_t)BOARD_CONSOLE_UART_BASE;
    cfg.src_freq_in_hz = clock_get_frequency(BOARD_CONSOLE_UART_CLK_NAME);
    cfg.baudrate = BOARD_CONSOLE_BAUDRATE;

    if (console_init(&cfg) != status_success) {
        while (1) {}
    }
}

bool board_console_try_send_byte(uint8_t byte) {
    if (!uart_check_status(BOARD_CONSOLE_UART_BASE, uart_stat_tx_slot_avail))
        return false;
    uart_write_byte(BOARD_CONSOLE_UART_BASE, byte);
    return true;
}

static void board_park_leds_off(void) {
    /* Drive safe board LEDs to their OFF state at boot so nothing glows before
     * the fieldbus core takes ownership. PA25/PA28 were first found by the LED
     * scan, but the HPM6E*Y* datasheet identifies them as internal PHY LED/strap
     * pins, so normal firmware must leave them to the PHY.
     *
     * These must be real push-pull outputs: a weak internal pull cannot hold an
     * LED line. This runs before board_init_clock(), so the GPIO peripheral clock
     * is not up yet -- enable it here or the output writes silently no-op. */
    clock_add_to_group(clock_gpio, 0);

    /* {GPIO port/bank index, pin, off level}. Bank index: A=0, B=1, C=2, E=4. */
    static const struct {
        uint8_t bank;
        uint8_t pin;
        uint8_t off_level;
    } leds[] = {
        {GPIO_DO_GPIOE,  5, 1},
        {GPIO_DO_GPIOE,  4, 1},
        {GPIO_DO_GPIOE,  3, 1}, /* main RGB R/G/B */
        {GPIO_DO_GPIOC, 26, 0},
        {GPIO_DO_GPIOC, 27, 0}, /* CAN0 green/blue */
        {GPIO_DO_GPIOE,  0, 0},
        {GPIO_DO_GPIOE,  2, 0}, /* CAN1 green/blue */
        {GPIO_DO_GPIOA,  9, 0},
        {GPIO_DO_GPIOB,  0, 0}, /* CAN2 green/blue */
        {GPIO_DO_GPIOB,  2, 0},
        {GPIO_DO_GPIOB,  3, 0}, /* CAN3 green/blue */
        {GPIO_DO_GPIOC, 20, 0},
        {GPIO_DO_GPIOC, 21, 0}, /* EtherCAT mid grn/red */
    };

    for (uint32_t i = 0; i < sizeof(leds) / sizeof(leds[0]); ++i) {
        const uint32_t pad = ((uint32_t)leds[i].bank * 32U) + leds[i].pin;
        HPM_IOC->PAD[pad].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0); /* GPIO */
        HPM_IOC->PAD[pad].PAD_CTL = 0;
        gpiom_set_pin_controller(HPM_GPIOM, leds[i].bank, leds[i].pin, gpiom_soc_gpio0);
        gpio_set_pin_output_with_initial(HPM_GPIO0, leds[i].bank, leds[i].pin, leds[i].off_level);
    }
}

bool board_check_bootloader_force_stay_requested(void) {
    /* Bootloader force-stay input = PB24, asserted low, internal pull-up.
     *
     * The name "user key" comes from the HPM6E00EVK reference design, where PB24
     * is KEYA. THIS BOARD HAS NO BUTTON ON IT [confirmed with the hardware,
     * 2026-09-08] -- the pad is only brought out, so asserting it means shorting
     * PB24 to GND through the reset. Do not document this as "hold the key".
     *
     * The sampling window is 4 x 250 us (~1 ms) right after reset. Unlike the
     * hpm5321, PB24 carries no other function here, so it is simply left as a
     * GPIO input afterwards (the hpm5321 must restore PA07 to JTAG_TMS). */
    const uint32_t pad_ctl =
        IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_HYS_SET(1);

    clock_add_to_group(clock_gpio, 0);

    HPM_IOC->PAD[IOC_PAD_PB24].FUNC_CTL = IOC_PB24_FUNC_CTL_GPIO_B_24;
    HPM_IOC->PAD[IOC_PAD_PB24].PAD_CTL = pad_ctl;

    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOB, 24, gpiom_soc_gpio0);
    gpio_set_pin_input(HPM_GPIO0, GPIO_DI_GPIOB, 24);

    bool pressed = true;
    for (uint32_t sample_index = 0; sample_index < 4; ++sample_index) {
        board_delay_us(250);
        if (gpio_read_pin(HPM_GPIO0, GPIO_DI_GPIOB, 24) != 0U) {
            pressed = false;
            break;
        }
    }
    return pressed;
}

void board_init(void) {
    board_park_leds_off();
    board_init_clock();
    board_init_console();
    board_init_pmp();
}

void board_init_core1(void) {
    clock_update_core_clock();
    /* No console on core1: UART1 (the SDK's core1 console) is this board's
     * fieldbus data UART. The fieldbus application must not printf. */
    board_init_pmp();
}

void board_init_pmp(void) {
    uint32_t start_addr;
    uint32_t end_addr;
    uint32_t length;
    pmp_entry_t pmp_entry[16] = {0};
    uint8_t index = 0;

    pmp_entry[index].pmp_addr = 0xFFFFFFFF;
    pmp_entry[index].pmp_cfg.val =
        PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
    index++;

    /* Non-cacheable data region (DMA buffers). Linker-script symbol names.
     * NOLINTBEGIN(bugprone-reserved-identifier, readability-identifier-naming) */
    extern uint32_t __noncacheable_start__[];
    extern uint32_t __noncacheable_end__[];
    /* NOLINTEND(bugprone-reserved-identifier, readability-identifier-naming) */
    start_addr = (uint32_t)__noncacheable_start__;
    end_addr = (uint32_t)__noncacheable_end__;
    length = end_addr - start_addr;
    if (length > 0) {
        assert((length & (length - 1U)) == 0U);
        assert((start_addr & (length - 1U)) == 0U);
        pmp_entry[index].pmp_addr = PMP_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pmp_cfg.val =
            PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
        pmp_entry[index].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pma_cfg.val =
            PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
        index++;
    }

    /* SHARE_RAM: non-cacheable + AMO on BOTH cores -- the cross-core rings of
     * the EtherCAT bridge (ecat/common/xcore_ring.hpp) rely on this.
     * NOLINTBEGIN(bugprone-reserved-identifier, readability-identifier-naming) */
    extern uint32_t __share_mem_start__[];
    extern uint32_t __share_mem_end__[];
    /* NOLINTEND(bugprone-reserved-identifier, readability-identifier-naming) */
    start_addr = (uint32_t)__share_mem_start__;
    end_addr = (uint32_t)__share_mem_end__;
    length = end_addr - start_addr;
    if (length > 0) {
        assert((length & (length - 1U)) == 0U);
        assert((start_addr & (length - 1U)) == 0U);
        pmp_entry[index].pmp_addr = PMP_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pmp_cfg.val =
            PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
        pmp_entry[index].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pma_cfg.val =
            PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
        index++;
    }

    pmp_config(&pmp_entry[0], index);
}

static inline void board_init_clock(void) {
    const uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);
    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);

        /* Select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, 2);
    }

    /* Group 0: core0 domain (fabric, flash, DMA, EtherCAT-side resources). */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_ahb0, 0);
    clock_add_to_group(clock_axif, 0);
    clock_add_to_group(clock_axis, 0);
    clock_add_to_group(clock_axic, 0);
    clock_add_to_group(clock_axin, 0);
    clock_add_to_group(clock_rom0, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_lmm1, 0);
    clock_add_to_group(clock_ram0, 0);
    clock_add_to_group(clock_ram1, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xdma, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_ptpc, 0);
    /* A clock group answers "which CPU requests this peripheral to stay on", not
     * "which CPU may access it" -- both cores reach every peripheral over AXI.
     * A peripheral is gated when the CPU its group is bound to is not running.
     * The two switches below are therefore ORTHOGONAL and must stay that way:
     *
     *   BOARD_FIELDBUS_ON_CORE0   CAN0..CAN3 + UART1 belong to group 0
     *   BOARD_SECONDARY_CORE_USED core1 is released, so group 1 is meaningful
     *
     * The core-swap layout (EtherCAT on core1, USB+CAN on core0) needs BOTH set:
     * the fieldbus peripherals must be clocked from the always-running core0
     * while core1 still needs its CPU clock and machine timer. */
#if defined(BOARD_FIELDBUS_ON_CORE0) && BOARD_FIELDBUS_ON_CORE0
    /* Fieldbus peripherals driven from core0. Leaving them in group 1 while
     * core1 is never released gates them forever -- that was the 2026-07-31
     * bug where MCAN silently neither transmitted nor received (bring-up notes
     * pitfall 12). */
    clock_add_to_group(clock_can0, 0);
    clock_add_to_group(clock_can1, 0);
    clock_add_to_group(clock_can2, 0);
    clock_add_to_group(clock_can3, 0);
    clock_add_to_group(clock_uart1, 0);
#endif
    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

#if defined(BOARD_SECONDARY_CORE_USED) && BOARD_SECONDARY_CORE_USED
    /* Group 1: core1 domain. Always carries core1's own CPU clock and machine
     * timer; the fieldbus peripherals only join when they are not already in
     * group 0. */
    clock_add_to_group(clock_cpu1, 1);
    clock_add_to_group(clock_mchtmr1, 1);
#if !defined(BOARD_FIELDBUS_ON_CORE0) || !BOARD_FIELDBUS_ON_CORE0
    clock_add_to_group(clock_can0, 1);
    clock_add_to_group(clock_can1, 1);
    clock_add_to_group(clock_can2, 1);
    clock_add_to_group(clock_can3, 1);
    clock_add_to_group(clock_uart1, 1);
#endif
    /* Connect Group1 to CPU1 */
    clock_connect_group_to_cpu(1, 1);
#endif

    /* Bump up DCDC voltage to 1275mv */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1275);
    pcfg_dcdc_switch_to_dcm_mode(HPM_PCFG);

    /* Set CPU clock to 600MHz */
    clock_set_source_divider(clock_cpu0, clk_src_pll0_clk0, 1);
    clock_set_source_divider(clock_cpu1, clk_src_pll0_clk0, 1);

    /* Pin AHB0 to a known 200 MHz (PLL1CLK0 800 MHz / 4). PTPC -- the shared
     * CAN timestamp timebase -- runs on AHB0; the app-layer CAN driver bakes
     * the resulting 5 ns PTPC step into a compile-time divisor
     * (board::kCanTimestampNsPerUs = 1000, see app/board_app.hpp) and asserts
     * the relationship at init. Keep these three places consistent. */
    clock_set_source_divider(clock_ahb0, clk_src_pll1_clk0, 4);

    /* Both machine timers run at 4 MHz (24 MHz OSC / 6): the shared app-layer
     * Timer (app/src/timer) and the protocol session lease constants assume
     * 0.25 us ticks on every board. */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 6);
    clock_set_source_divider(clock_mchtmr1, clk_src_osc24m, 6);

    clock_update_core_clock();
}

void board_init_usb(void) {
    /* USB0_ID / USB0_OC / USB0_PWR */
    HPM_IOC->PAD[IOC_PAD_PF22].FUNC_CTL = IOC_PF22_FUNC_CTL_USB0_ID;
    HPM_IOC->PAD[IOC_PAD_PF23].FUNC_CTL = IOC_PF23_FUNC_CTL_USB0_OC;
    HPM_IOC->PAD[IOC_PAD_PF19].FUNC_CTL = IOC_PF19_FUNC_CTL_USB0_PWR;

    clock_add_to_group(clock_usb0, 0);

    usb_hcd_set_power_ctrl_polarity(HPM_USB0, true);
    /* Wait for the USB_PWR pin controlled vbus power to stabilize. */
    board_delay_ms(100);
}

void board_init_ethercat(ESC_Type* ptr) {
    /* Keep the ESC in the clock group of whichever core owns it, same as
     * clock_eth0 below. Purely tidiness: a clock group only says which CPU
     * requests the peripheral to stay on, and core0 is always running in every
     * layout, so leaving the ESC in group 0 would work even when core1 drives
     * it. Nothing here is fragile. */
#if defined(BOARD_RUNNING_CORE) && BOARD_RUNNING_CORE == HPM_CORE1
    clock_add_to_group(clock_esc0, 1);
#else
    clock_add_to_group(clock_esc0, 0);
#endif
    esc_core_enable_clock(ptr, true);
    esc_phy_enable_clock(ptr, true);
    configure_esc_internal_phy_interface(ptr);

    init_esc_pins();
    /* Keep the on-die ECAT PHY resets asserted; the port layer releases them. */
    gpio_set_pin_output_with_initial(
        BOARD_ECAT_PHY0_RESET_GPIO, BOARD_ECAT_PHY0_RESET_GPIO_PORT_INDEX,
        BOARD_ECAT_PHY0_RESET_PIN_INDEX, BOARD_ECAT_PHY_RESET_LEVEL);
    gpio_set_pin_output_with_initial(
        BOARD_ECAT_PHY1_RESET_GPIO, BOARD_ECAT_PHY1_RESET_GPIO_PORT_INDEX,
        BOARD_ECAT_PHY1_RESET_PIN_INDEX, BOARD_ECAT_PHY_RESET_LEVEL);
}

static inline void configure_esc_internal_phy_interface(ESC_Type* ptr) {
    uint32_t phy_cfg0 = ptr->PHY_CFG0;
    phy_cfg0 |= ESC_PHY_CFG0_MAC_SPEED_MASK;
    phy_cfg0 &=
        ~(ESC_PHY_CFG0_PORT0_RMII_EN_MASK | ESC_PHY_CFG0_PORT1_RMII_EN_MASK
          | ESC_PHY_CFG0_PORT2_RMII_EN_MASK);
    ptr->PHY_CFG0 = phy_cfg0;

    ptr->PHY_CFG1 |= ESC_PHY_CFG1_REFCK_25M_OE_MASK;
}

void board_ecat_set_internal_phy_link(bool port0_up, bool port1_up) {
    /* The two on-die PHYs do NOT route a dedicated LINK pin to an ESC CTR input
     * the way the EVK's external PHYs do. The SDK default therefore leaves
     * NMII_LINK sourced from pads that are not real link inputs.
     *
     * Switch the NMII_LINK source for port0/port1 to the GPR register and drive
     * the value from software. The GPR bit is "link invalid" when set, so
     * clearing it marks the link valid. Do not blindly force both ports up: with
     * one cable connected, a false-up empty port prevents the ESC from closing
     * that loop and the master may never receive a frame back. */
    uint32_t gpr = HPM_ESC->GPR_CFG2;
    gpr &= ~(ESC_GPR_CFG2_NMII_LINK0_FROM_IO_MASK | ESC_GPR_CFG2_NMII_LINK1_FROM_IO_MASK);
    if (port0_up) {
        gpr &= ~ESC_GPR_CFG2_NMII_LINK0_GPR_MASK;
    } else {
        gpr |= ESC_GPR_CFG2_NMII_LINK0_GPR_MASK;
    }
    if (port1_up) {
        gpr &= ~ESC_GPR_CFG2_NMII_LINK1_GPR_MASK;
    } else {
        gpr |= ESC_GPR_CFG2_NMII_LINK1_GPR_MASK;
    }
    HPM_ESC->GPR_CFG2 = gpr;
}

bool board_ecat_get_internal_phy_status(board_ecat_phy_status_t* status) {
    if (status == NULL) {
        return false;
    }

    *status = (board_ecat_phy_status_t){0};
    configure_internal_phy_mdio_as_enet();

    uint16_t value = 0;
    status->port0_read_ok =
        enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT0_PHY_ADDR, BOARD_ECAT_PHY_REG_BMCR, &value)
        == status_success;
    status->port0_bmcr = value;
    if (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT0_PHY_ADDR, BOARD_ECAT_PHY_REG_BMSR, &value)
        != status_success) {
        status->port0_read_ok = false;
    }
    if (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT0_PHY_ADDR, BOARD_ECAT_PHY_REG_BMSR, &value)
        != status_success) {
        status->port0_read_ok = false;
    }
    status->port0_bmsr = value;
    status->port0_link_up =
        status->port0_read_ok && ((status->port0_bmsr & BOARD_ECAT_PHY_BMSR_LINK_MASK) != 0U);
    if (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT0_PHY_ADDR, BOARD_ECAT_PHY_REG_ID1, &value)
        == status_success) {
        status->port0_id1 = value;
    }
    if (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT0_PHY_ADDR, BOARD_ECAT_PHY_REG_ID2, &value)
        == status_success) {
        status->port0_id2 = value;
    }
    (void)enet_write_phy(
        HPM_ENET0, BOARD_ECAT_PORT0_PHY_ADDR, BOARD_ECAT_PHY_REG_PAGESEL, BOARD_ECAT_PHY_PAGE_RMSR);
    if (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT0_PHY_ADDR, BOARD_ECAT_PHY_REG_RMSR_P7, &value)
        == status_success) {
        status->port0_rmsr_p7 = value;
    }
    (void)enet_write_phy(HPM_ENET0, BOARD_ECAT_PORT0_PHY_ADDR, BOARD_ECAT_PHY_REG_PAGESEL, 0U);

    status->port1_read_ok =
        enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT1_PHY_ADDR, BOARD_ECAT_PHY_REG_BMCR, &value)
        == status_success;
    status->port1_bmcr = value;
    if (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT1_PHY_ADDR, BOARD_ECAT_PHY_REG_BMSR, &value)
        != status_success) {
        status->port1_read_ok = false;
    }
    if (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT1_PHY_ADDR, BOARD_ECAT_PHY_REG_BMSR, &value)
        != status_success) {
        status->port1_read_ok = false;
    }
    status->port1_bmsr = value;
    status->port1_link_up =
        status->port1_read_ok && ((status->port1_bmsr & BOARD_ECAT_PHY_BMSR_LINK_MASK) != 0U);
    if (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT1_PHY_ADDR, BOARD_ECAT_PHY_REG_ID1, &value)
        == status_success) {
        status->port1_id1 = value;
    }
    if (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT1_PHY_ADDR, BOARD_ECAT_PHY_REG_ID2, &value)
        == status_success) {
        status->port1_id2 = value;
    }
    (void)enet_write_phy(
        HPM_ENET0, BOARD_ECAT_PORT1_PHY_ADDR, BOARD_ECAT_PHY_REG_PAGESEL, BOARD_ECAT_PHY_PAGE_RMSR);
    if (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT1_PHY_ADDR, BOARD_ECAT_PHY_REG_RMSR_P7, &value)
        == status_success) {
        status->port1_rmsr_p7 = value;
    }
    (void)enet_write_phy(HPM_ENET0, BOARD_ECAT_PORT1_PHY_ADDR, BOARD_ECAT_PHY_REG_PAGESEL, 0U);

    configure_internal_phy_mdio_as_esc();

    return status->port0_read_ok || status->port1_read_ok;
}

bool board_ecat_configure_internal_phy_mii_mode(void) {
    configure_internal_phy_mdio_as_enet();
    const bool port0_ok = configure_internal_phy_mii_mode(BOARD_ECAT_PORT0_PHY_ADDR, NULL);
    const bool port1_ok = configure_internal_phy_mii_mode(BOARD_ECAT_PORT1_PHY_ADDR, NULL);
    configure_internal_phy_mdio_as_esc();
    return port0_ok || port1_ok;
}

bool board_ecat_refresh_internal_phy_link(void) {
    board_ecat_phy_status_t status = {0};
    configure_internal_phy_mdio_as_enet();

    uint16_t value = 0;
    status.port0_read_ok =
        enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT0_PHY_ADDR, BOARD_ECAT_PHY_REG_BMSR, &value)
        == status_success;
    status.port0_read_ok =
        (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT0_PHY_ADDR, BOARD_ECAT_PHY_REG_BMSR, &value)
         == status_success)
        && status.port0_read_ok;
    status.port0_bmsr = value;
    status.port0_link_up =
        status.port0_read_ok && ((status.port0_bmsr & BOARD_ECAT_PHY_BMSR_LINK_MASK) != 0U);

    status.port1_read_ok =
        enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT1_PHY_ADDR, BOARD_ECAT_PHY_REG_BMSR, &value)
        == status_success;
    status.port1_read_ok =
        (enet_read_phy(HPM_ENET0, BOARD_ECAT_PORT1_PHY_ADDR, BOARD_ECAT_PHY_REG_BMSR, &value)
         == status_success)
        && status.port1_read_ok;
    status.port1_bmsr = value;
    status.port1_link_up =
        status.port1_read_ok && ((status.port1_bmsr & BOARD_ECAT_PHY_BMSR_LINK_MASK) != 0U);

    configure_internal_phy_mdio_as_esc();
    if (!status.port0_read_ok && !status.port1_read_ok) {
        return false;
    }
    board_ecat_apply_internal_phy_link_status(&status);
    return true;
}

bool board_ecat_wait_internal_phy_link(uint32_t timeout_ms) {
    const uint32_t poll_interval_ms = 20U;
    const uint32_t attempts = (timeout_ms + poll_interval_ms - 1U) / poll_interval_ms;

    for (uint32_t i = 0; i <= attempts; ++i) {
        board_ecat_phy_status_t status = {0};
        if (board_ecat_get_internal_phy_status(&status)) {
            board_ecat_apply_internal_phy_link_status(&status);
            if (status.port0_link_up || status.port1_link_up) {
                return true;
            }
        }
        board_delay_ms(poll_interval_ms);
    }

    return false;
}

static bool configure_internal_phy_mii_mode(uint8_t phy_addr, uint16_t* rmsr_p7) {
    uint16_t value = 0;
    bool ok =
        enet_write_phy(HPM_ENET0, phy_addr, BOARD_ECAT_PHY_REG_PAGESEL, BOARD_ECAT_PHY_PAGE_RMSR)
        == status_success;
    if (ok) {
        ok = enet_read_phy(HPM_ENET0, phy_addr, BOARD_ECAT_PHY_REG_RMSR_P7, &value)
          == status_success;
    }
    if (ok) {
        value &= ~BOARD_ECAT_PHY_RMSR_MII_MODE_MASK;
        ok = enet_write_phy(HPM_ENET0, phy_addr, BOARD_ECAT_PHY_REG_RMSR_P7, value)
          == status_success;
    }
    if (rmsr_p7 != NULL) {
        *rmsr_p7 = value;
    }
    (void)enet_write_phy(HPM_ENET0, phy_addr, BOARD_ECAT_PHY_REG_PAGESEL, 0U);
    return ok;
}

static void board_ecat_apply_internal_phy_link_status(const board_ecat_phy_status_t* status) {
#if defined(BOARD_ECAT_SWAP_PHY_LINK_TO_ESC_PORT) && BOARD_ECAT_SWAP_PHY_LINK_TO_ESC_PORT
    board_ecat_set_internal_phy_link(status->port1_link_up, status->port0_link_up);
#else
    board_ecat_set_internal_phy_link(status->port0_link_up, status->port1_link_up);
#endif
}

static inline void configure_internal_phy_mdio_as_enet(void) {
    const uint32_t mdio_pad_ctl =
        IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_HYS_SET(1);

#if defined(BOARD_RUNNING_CORE) && BOARD_RUNNING_CORE == HPM_CORE1
    clock_add_to_group(clock_eth0, 1);
#else
    clock_add_to_group(clock_eth0, 0);
#endif

    HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_ETH0_MDIO;
    HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_ETH0_MDC;
    HPM_IOC->PAD[IOC_PAD_PA30].PAD_CTL = mdio_pad_ctl;
}

static inline void configure_internal_phy_mdio_as_esc(void) {
    const uint32_t mdio_pad_ctl =
        IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_HYS_SET(1);

    HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_ESC0_MDIO;
    HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_ESC0_MDC;
    HPM_IOC->PAD[IOC_PAD_PA30].PAD_CTL = mdio_pad_ctl;
}

static inline void init_esc_pins(void) {
    const uint32_t strap_pullup_ctl = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);

    static const uint32_t analog_pads[] = {
        IOC_PAD_PA16, IOC_PAD_PA17, IOC_PAD_PA18, IOC_PAD_PA19, IOC_PAD_PA20,
        IOC_PAD_PA21, IOC_PAD_PA22, IOC_PAD_PA23, IOC_PAD_PA24, IOC_PAD_PA26,
        IOC_PAD_PA27, IOC_PAD_PA29, IOC_PAD_PW16, IOC_PAD_PW17,
    };

    for (uint32_t i = 0; i < sizeof(analog_pads) / sizeof(analog_pads[0]); ++i) {
        HPM_IOC->PAD[analog_pads[i]].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
        HPM_IOC->PAD[analog_pads[i]].PAD_CTL = 0;
    }

    configure_internal_phy_mdio_as_esc();

    HPM_IOC->PAD[IOC_PAD_PV12].FUNC_CTL = IOC_PV12_FUNC_CTL_GPIO_V_12;
    HPM_IOC->PAD[IOC_PAD_PW12].FUNC_CTL = IOC_PW12_FUNC_CTL_GPIO_W_12;
    HPM_IOC->PAD[IOC_PAD_PV12].PAD_CTL = 0;
    HPM_IOC->PAD[IOC_PAD_PW12].PAD_CTL = 0;
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOV, 12, gpiom_soc_gpio0);
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOW, 12, gpiom_soc_gpio0);

    HPM_IOC->PAD[IOC_PAD_PW20].FUNC_CTL = IOC_PW20_FUNC_CTL_ESC0_REFCK;
    HPM_IOC->PAD[IOC_PAD_PW21].FUNC_CTL = IOC_PW21_FUNC_CTL_ESC0_REFCK;

    /* On-die PHY LED/address-strap pads double as ESC link-source controls. */
    HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PA25_FUNC_CTL_ESC0_CTR_0;
    HPM_IOC->PAD[IOC_PAD_PA25].PAD_CTL = strap_pullup_ctl;
    HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_ESC0_CTR_1;
    HPM_IOC->PAD[IOC_PAD_PA28].PAD_CTL = strap_pullup_ctl;

    /* EtherCAT RUN/ERROR status LEDs: the GPIO LED scan found PC20 (green) and
     * PC21 (red) as the "EtherCAT middle" indicators, and both carry an ESC0_CTR
     * alt function (PC20 = CTR_2, PC21 = CTR_3), so the ESC drives them directly
     * from the AL state machine. CTR_2/CTR_3 are free (CTR_0/CTR_1 are the
     * NMII_LINK sources above). The port layer (hpm_ecat_hw.c) binds LED_RUN to
     * BOARD_ECAT_LED_RUN_CTRL_INDEX (2) and LED_ERROR to
     * BOARD_ECAT_LED_ERROR_CTRL_INDEX (3); board_park_leds_off() drives these OFF
     * before the ESC takes them. The LEDs are active-high and the SDK drives the
     * CTR LED signal non-inverted -- if they read inverted on hardware, that
     * polarity is the thing to flip. */
    HPM_IOC->PAD[IOC_PAD_PC20].FUNC_CTL = IOC_PC20_FUNC_CTL_ESC0_CTR_2;
    HPM_IOC->PAD[IOC_PAD_PC21].FUNC_CTL = IOC_PC21_FUNC_CTL_ESC0_CTR_3;

    HPM_IOC->PAD[IOC_PAD_PV00].FUNC_CTL = IOC_PV00_FUNC_CTL_ESC0_P0_RXDV;
    HPM_IOC->PAD[IOC_PAD_PV01].FUNC_CTL = IOC_PV01_FUNC_CTL_ESC0_P0_RXD_0;
    HPM_IOC->PAD[IOC_PAD_PV02].FUNC_CTL = IOC_PV02_FUNC_CTL_ESC0_P0_RXD_1;
    HPM_IOC->PAD[IOC_PAD_PV03].FUNC_CTL = IOC_PV03_FUNC_CTL_ESC0_P0_RXD_2;
    HPM_IOC->PAD[IOC_PAD_PV04].FUNC_CTL = IOC_PV04_FUNC_CTL_ESC0_P0_RXD_3;
    HPM_IOC->PAD[IOC_PAD_PV05].FUNC_CTL = IOC_PV05_FUNC_CTL_ESC0_P0_RXCK;
    HPM_IOC->PAD[IOC_PAD_PV06].FUNC_CTL = IOC_PV06_FUNC_CTL_ESC0_P0_TXCK;
    HPM_IOC->PAD[IOC_PAD_PV07].FUNC_CTL = IOC_PV07_FUNC_CTL_ESC0_P0_TXD_0;
    HPM_IOC->PAD[IOC_PAD_PV08].FUNC_CTL = IOC_PV08_FUNC_CTL_ESC0_P0_TXD_1;
    HPM_IOC->PAD[IOC_PAD_PV09].FUNC_CTL = IOC_PV09_FUNC_CTL_ESC0_P0_TXD_2;
    HPM_IOC->PAD[IOC_PAD_PV10].FUNC_CTL = IOC_PV10_FUNC_CTL_ESC0_P0_TXD_3;
    HPM_IOC->PAD[IOC_PAD_PV11].FUNC_CTL = IOC_PV11_FUNC_CTL_ESC0_P0_TXEN;
    HPM_IOC->PAD[IOC_PAD_PV15].FUNC_CTL = IOC_PV15_FUNC_CTL_ESC0_P0_RXER;

    HPM_IOC->PAD[IOC_PAD_PW00].FUNC_CTL = IOC_PW00_FUNC_CTL_ESC0_P1_RXDV;
    HPM_IOC->PAD[IOC_PAD_PW01].FUNC_CTL = IOC_PW01_FUNC_CTL_ESC0_P1_RXD_0;
    HPM_IOC->PAD[IOC_PAD_PW02].FUNC_CTL = IOC_PW02_FUNC_CTL_ESC0_P1_RXD_1;
    HPM_IOC->PAD[IOC_PAD_PW03].FUNC_CTL = IOC_PW03_FUNC_CTL_ESC0_P1_RXD_2;
    HPM_IOC->PAD[IOC_PAD_PW04].FUNC_CTL = IOC_PW04_FUNC_CTL_ESC0_P1_RXD_3;
    HPM_IOC->PAD[IOC_PAD_PW05].FUNC_CTL = IOC_PW05_FUNC_CTL_ESC0_P1_RXCK;
    HPM_IOC->PAD[IOC_PAD_PW06].FUNC_CTL = IOC_PW06_FUNC_CTL_ESC0_P1_TXCK;
    HPM_IOC->PAD[IOC_PAD_PW07].FUNC_CTL = IOC_PW07_FUNC_CTL_ESC0_P1_TXD_0;
    HPM_IOC->PAD[IOC_PAD_PW08].FUNC_CTL = IOC_PW08_FUNC_CTL_ESC0_P1_TXD_1;
    HPM_IOC->PAD[IOC_PAD_PW09].FUNC_CTL = IOC_PW09_FUNC_CTL_ESC0_P1_TXD_2;
    HPM_IOC->PAD[IOC_PAD_PW10].FUNC_CTL = IOC_PW10_FUNC_CTL_ESC0_P1_TXD_3;
    HPM_IOC->PAD[IOC_PAD_PW11].FUNC_CTL = IOC_PW11_FUNC_CTL_ESC0_P1_TXEN;
    HPM_IOC->PAD[IOC_PAD_PW15].FUNC_CTL = IOC_PW15_FUNC_CTL_ESC0_P1_RXER;
}

void board_delay_us(uint32_t us) { clock_cpu_delay_us(us); }

void board_delay_ms(uint32_t ms) { clock_cpu_delay_ms(ms); }
