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

// 一份镜像服务两块 PCB: 单 CAN 板与双 CAN 板, 仅 CAN 端口数、RGB LED 引脚与
// 每 CAN 指示灯引脚不同, 其余(board.c、.yaml、UART、时钟树、USB)完全一致。
//
// 板型在上电时由 OTP shadow word 25 判定, 该字的证据强度与未识别值为何直接
// 拒绝启动见 common/board_identity.hpp。bootloader 在 word 25 非两个已知值时
// 拒绝跳进本 app, 故此处任何代码运行时板型必已确认。
//
// 表为何是运行时而非编译期: PA30/PA31 在单 CAN 板上是绿/红 LED 阴极, 在双 CAN
// 板上是 MCAN3 RXD/TXD, 两种引脚分配在同一组焊盘上互斥, 不存在可无条件配置的
// 超集; 每个焊盘只能按板型一次性选定一个 FUNC_CTL。
//
// 表按上限(两个 CAN 控制器)定尺寸并填充, 运行时以端口数决定实际启用哪些条目。
// 代价是单 CAN 板多占一片闲置的 MCAN message RAM(即 MCAN_MSG_BUF_SIZE_IN_WORDS
// = 640 words, 32 KiB AHB SRAM 中约 2.5 KiB); 改为动态布局只能省回这 2.5 KiB,
// 却失去 SoC 要求的静态放置, 不划算。

// HPM5321 的 USB 始终以高速运行。
bool usb_use_high_speed();

// 两种板型下 CAN 控制器数的上限。单 CAN 板只用条目 0; kCanPorts 按双 CAN 板
// 定尺寸并填充, can_port_count() 报告本板实际存在的端口数。
constexpr size_t kCanPortCapacity = 2;

// CAN 端口按表序排列: 丝印 CAN1 = kCanPorts[0] = DataId::kCan1, 丝印
// CAN2 = kCanPorts[1] = DataId::kCan2。
//
// 两种板型都跑 CAN-FD, 表对两块 PCB 相同, 单 CAN 板只是少使能一路。接收方向
// FD 是经典 CAN 2.0 的严格超集: 开启 FD 的 M_CAN 仍能解码对端的经典帧。发送
// 方向不再逐帧选择 -- 帧类型跟随总线, 本板发出的每一帧都是 FD(见 can.cpp 与
// core/src/protocol/protocol.hpp 中已废弃的 IsFdCan 头部位)。运行时不切换
// 模式: 控制器配置一次, 保持 FD 能力。经典模式完全收不到 FD 帧(实测 FD 对端
// 0/50, 经典帧 50/50); 单 CAN 板收发器在 5 Mbit 数据段的时序仍待上板确认,
// 见 README.md。
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

// 实际存在的 CAN 控制器数: 双 CAN 板为 2, 单 CAN 板为 1。遍历 CAN 数组的
// 循环必须以它而非 std::size(kCanPorts) 为界, 否则会初始化一路没有收发器、
// 焊盘实为 LED 阴极的 MCAN3。
inline size_t can_port_count() { return board_identity().dual_can() ? 2U : 1U; }

// 下标 index 处的端口。>= can_port_count() 的条目在本板上无效。
constexpr CanPort can_port(size_t index) { return kCanPorts[index]; }

uint32_t init_can(MCAN_Type* ptr);
void can_irq_handler(size_t board_can_index);

// 逻辑下标 can_index 对应 CAN 控制器的 MCAN message RAM 区域
// (本 SoC 上为 .ahb_sram 段中的数组, 见 board_app.cpp)。
mcan_msg_buf_attr_t can_message_ram(size_t can_index);

// PTPC(共享的 CAN 时间戳时基)运行在 160 MHz AHB 时钟: 报告的纳秒步进为
// 6 ns, 故真实微秒 = 报告纳秒 / (160 * 6)。CAN 驱动在 init 时会对照时钟树
// 断言这一点。
constexpr uint32_t kCanTimestampNsPerUs = 960;

// 应用运行在 core0: 其 machine timer 为 MCHTMR0, 时钟为共享 Timer 驱动所
// 要求的 4 MHz(见 board.c)。
constexpr clock_name_t kMchtmrClockName = clock_mchtmr0;

// 共享 UART 驱动的 DMA 环形缓冲所在 section: AHB SRAM 在本 SoC 上天然非缓存。
#define libhcs_DMA_BUFFER_SECTION ".ahb_sram"

// UART 端口按逻辑序。两种板型都只有一个数据 UART(UART2), 无 DBUS 接收器。
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

// 普通 GPIO RGB LED, 低电平有效(共阳: 拉低焊盘点亮通道)。模板末参为
// active_high = false。
//
// 两种板型的 LED 接在不同焊盘, 且单 CAN 板上的 PA30/PA31 正是双 CAN 板给
// MCAN3 的焊盘, 因此不能合并成一张表。led_red_pin() 等在运行时选择引脚组。
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

// 每 CAN 指示灯: 双 CAN 板两颗(高电平有效), 单 CAN 板没有。按上限定尺寸;
// can_indicator_count() 报告本板实际数量, 为 0 时共享 Led 驱动完全跳过指示灯
// 逻辑。
constexpr std::array<GpioPin, 2> kCanIndicatorPins{
    make_gpio_pin<gpiom_soc_gpio0, 'B', 14, true>(),
    make_gpio_pin<gpiom_soc_gpio0, 'B', 15, true>(),
};

inline size_t can_indicator_count() { return board_identity().dual_can() ? 2U : 0U; }

void init_led_pins();
void init_can_indicator_pins();

} // namespace libhcs::firmware::board
