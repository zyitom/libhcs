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

// 本 SoC 上 MCAN message RAM 必须放在 AHB SRAM。按两种板型的上限定尺寸
// (见 board_app.hpp 的 kCanPortCapacity): 单 CAN 板闲置第二片, 付出 32 KiB
// AHB SRAM 中的 2.5 KiB, 换来两块 PCB 共用一个二进制。
static_assert(MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1);
ATTR_PLACE_AT(".ahb_sram")
constinit uint32_t can_msg_buffer[std::size(kCanPorts)][MCAN_MSG_BUF_SIZE_IN_WORDS]{};

uint32_t init_can_clock(MCAN_Type* ptr) {
    if (ptr == HPM_MCAN0) {
        clock_add_to_group(clock_can0, 0);
        clock_set_source_divider(clock_can0, clk_src_pll1_clk0, 10);
        return clock_get_frequency(clock_can0);
    }
    // MCAN3 只存在于双 CAN 板。仅经 init_can() 到达, CAN 层对实际使能的每个
    // 端口各调一次 -- 以 can_port_count() 为界, 单 CAN 板走不到此分支。
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
        // PA30/PA31 在双 CAN 板上是 MCAN3 对, 在单 CAN 板上是绿/红 LED 阴极。
        // 若在单 CAN 板上把它们让给 MCAN3, 收发器 TXD 输出会灌进 LED 网络, 故
        // 此赋值受板型双重把关: CAN 层仅在 can_port_count() 为 2 时才构造端口
        // 1, 另一板型的 init_led_pins() 会把同样的焊盘拿去做 LED。用 assert
        // 而非静默分支 -- 单 CAN 板走到这里说明板型链路已坏, 值得在 debug 构建
        // 中陷住, 而不是给焊盘上电。
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
    constexpr uint32_t tx_pad = IOC_PAD_PAD_CTL_PE_SET(1) | // 上拉使能
                                IOC_PAD_PAD_CTL_PS_SET(1);  // 上拉选择: 上拉
    constexpr uint32_t rx_pad = IOC_PAD_PAD_CTL_PE_SET(1) | // 上拉使能
                                IOC_PAD_PAD_CTL_PS_SET(1) | // 上拉选择: 上拉
                                IOC_PAD_PAD_CTL_HYS_SET(1); // 使能施密特触发器
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
    // 单 CAN 板没有指示灯, 此处为 0; 双 CAN 板为 2。配置为输出期间使能下拉,
    // 避免焊盘悬空。
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
    // 板上收发器与 PHY 均支持高速, 始终以 480 Mbit 枚举。曾有强制全速的测试开关
    // (PORTSC1.PFSC) 用于混速 SOF 时间基实验, 结论见 SOF_TIMEBASE.md 5.7: 除非
    // 硬件只能跑全速, 没有理由混速; 实验结束, 开关随结论一并移除。
    return true;
}

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, can0_isr)
void can0_isr() { can_irq_handler(0); }

// 两种镜像都存在此 ISR。单 CAN 板上 MCAN3 从不上时钟、从不配置、IRQ 从不
// 使能(调用 intc_m_enable_irq_with_priority 的是 Can 构造函数, 它只对
// can_port_count() 以内的端口运行), 故该向量已安装但不可达。无条件注册正是
// 一个二进制服务两块板的关键; can_irq_handler 另有下标边界检查。
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN3, can1_isr)
void can1_isr() { can_irq_handler(1); }

SDK_DECLARE_EXT_ISR_M(IRQn_UART2, uart0_isr)
void uart0_isr() { uart_irq_handler(0); }

} // namespace libhcs::firmware::board
