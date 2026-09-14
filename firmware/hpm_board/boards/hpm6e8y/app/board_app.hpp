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

// EtherCAT 桥的现场总线(core1)应用层。本板引出全部四个物理 CAN 口
// (CAN0..CAN3 = MCAN0..MCAN3, 引脚走线见 kCanPorts)和一个测试 UART(UART1 于
// PY06/PY07), 另有一盏纯 GPIO RGB LED。EtherCAT 侧(ESC, core0)在 ../board.c
// 配置, 不出现在这里。

// USB0 使用 HPM6E80 的高速设备控制器与 PHY。
bool usb_use_high_speed();

// CAN 口按逻辑序排列, 映射到四个物理丝印口 CAN0..CAN3(= MCAN0..MCAN3)。
// 引脚走线由 CAN 引脚扫描恢复, 记录于 CAN_PIN_REVERSE_ENGINEERING.md:
//   CAN0 = MCAN0  TX PC00 / RX PC01
//   CAN1 = MCAN1  TX PB05 / RX PB04
//   CAN2 = MCAN2  TX PD08 / RX PD09
//   CAN3 = MCAN3  TX PD15 / RX PD14
// 四路都跑 CAN-FD(仲裁 1 Mbps / 数据 5 Mbps, BRS 开)。接收是严格超集 -- 对端
// 的经典帧照常解码; 而本板发送的每帧都是 FD: 帧类型跟随总线, 且是 CANFD 回环
// 压力测试(跳线 CAN0<->CAN1、CAN2<->CAN3)所要求的。
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

// 表容量与本板实际存在的控制器数。两者只在服务多块 PCB 的板目录
// (boards/hpm5321)上不同: 表按更大的变体分配尺寸, 数量则来自运行时身份。
// 本板两者相同。
constexpr size_t kCanPortCapacity = std::size(kCanPorts);
constexpr size_t can_port_count() { return kCanPortCapacity; }
constexpr CanPort can_port(size_t index) { return kCanPorts[index]; }

uint32_t init_can(MCAN_Type* ptr);
void can_irq_handler(size_t board_can_index);

// HPM6E80 的 MCAN message RAM 必须位于 0xF0200000 处的 32 KiB AHB RAM。
// core1 链接脚本没有暴露 .ahb_sram 输出段, 板级改为发放该区域(本固件中无其他
// 用途)的固定切片, 而不是用 section 放置的数组。
mcan_msg_buf_attr_t can_message_ram(size_t can_index);

// PTPC(共享的 CAN 时间戳时基)挂在 AHB0 上, 在 board.c 中钉为 200 MHz: 上报的
// 纳秒步进是 5 ns, 故真实微秒 = 上报纳秒 / (200 * 5)。CAN 驱动在 init 时对照
// 时钟树断言该值 -- board.c 若改 AHB0 分频, 这里同步更新。
constexpr uint32_t kCanTimestampNsPerUs = 1000;

// UART 口按逻辑序: 一个测试数据 UART(UART1, PY06/PY07 排针)。
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

// 运行本应用层的那个核的机器定时器。每个核经同一 HPM_MCHTMR_BASE 窗口只能
// 看到自己的 MCHTMR, 时钟名必须跟随运行中的核: 当前 EtherCAT 桥(应用层在
// core1)用 MCHTMR1; 单核 USB 镜像与把协议栈挪回 core0 的核交换布局用
// MCHTMR0。board.c 把两个分频都配成共享 Timer 驱动所断言的 4 MHz, 此开关翻转
// 时无需改时钟树。
#if defined(BOARD_RUNNING_CORE) && BOARD_RUNNING_CORE == HPM_CORE1
constexpr clock_name_t kMchtmrClockName = clock_mchtmr1;
#else
constexpr clock_name_t kMchtmrClockName = clock_mchtmr0;
#endif

// 共享 UART 驱动的 DMA 环形存储段。core1 没有 AHB SRAM 段, 用 AXI SRAM 的
// 非 cache 区达到同样目的(DMA 一致, 无需手动 cache 维护)。
#define libhcs_DMA_BUFFER_SECTION ".noncacheable.non_init"

// 主 RGB LED, 低有效(共阳: 拉低焊盘点亮)。焊盘经 GPIO LED 扫描核实
// (见 GPIO_LED_REVERSE_ENGINEERING.md): 红=PE05, 绿=PE04, 蓝=PE03。
//
// 这三个焊盘确实带 ESC0_CTR 复用功能(PE03=CTR_1, PE04=CTR_2, PE05=CTR_3),
// 旧 EVK 派生的 pinmux 选的正是它。现在不再如此: board.c 的 init_esc_pins()
// 已改用 HPM6E*Y* 片内 PHY 映射, 把四个 CTR 信号引到 PA25(CTR_0)、PA28(CTR_1)、
// PC20(CTR_2)、PC21(CTR_3), 从不写 PE03/PE04/PE05。因此 RGB LED 由本应用层
// 独占, 与哪个核先跑 init_esc_pins() 无关。不要在这三个焊盘上重新加 ESC0_CTR。
constexpr GpioPin kLedRedPin = make_gpio_pin<gpiom_soc_gpio0, 'E', 5, false>();
constexpr GpioPin kLedGreenPin = make_gpio_pin<gpiom_soc_gpio0, 'E', 4, false>();
constexpr GpioPin kLedBluePin = make_gpio_pin<gpiom_soc_gpio0, 'E', 3, false>();

// 上面三个常量的访问器形式。共享 LED 驱动调用它们而非直接读常量, 因为服务
// 多块 PCB 的板目录必须按运行时身份选焊盘(boards/hpm5321/app/board_app.hpp
// 就是如此)。本板只有一种引脚布局, 这里会被常量折叠。
constexpr GpioPin led_red_pin() { return kLedRedPin; }
constexpr GpioPin led_green_pin() { return kLedGreenPin; }
constexpr GpioPin led_blue_pin() { return kLedBluePin; }

// 本板确有每路 CAN 的指示灯(每口绿+蓝); GPIO LED 扫描已确认 CAN0 绿=PC26、
// CAN1 蓝=PE02、CAN2 绿=PA09/蓝=PB00、CAN3 绿=PB02/蓝=PB03。每口绿+蓝的完整
// 映射仍在扫描, 故尚未接成指示灯。
constexpr std::array<GpioPin, 0> kCanIndicatorPins{};
constexpr size_t can_indicator_count() { return kCanIndicatorPins.size(); }

void init_led_pins();
void init_can_indicator_pins();

} // namespace libhcs::firmware::board
