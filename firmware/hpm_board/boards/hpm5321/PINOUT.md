# hpm5321 —— 硬件引脚分配

> **文档类型**：硬件参考（引脚表）
> **适用范围**：`firmware/hpm_board/boards/hpm5321/`
> **状态**：现行有效
> **相关文档**：[README.md](README.md)（板级说明）

## 摘要

本文件是 hpm5321 两种 PCB 变体的**引脚速查表**。每一项都直接取自固件源码，"来源"列
标明该引脚在哪里被配置，**便于随时回源核对**——发现表与代码不一致时，以代码为准并回来
更正本表。两种 PCB 使用同一份镜像，OTP shadow word 25 在上电时选择对应表。

SDK 使用的 SoC 模型是 **HPM5361**；板上实际器件为 **HPM5321IEG1**，J-Link 设备族名为
`HPM5321xEGx`（QFN48；`openocd-soc: hpm5300`）。注意 `hpm5321` 是仓库板卡名，
不是 SDK 的 SoC 目录名；SDK 的 HPM5361 模型用于这颗兼容芯片。

## CAN（MCAN）

**共用端口：**

| 接口 | 硬件 | TXD | RXD | 来源 |
| ------- | -------- | ---- | ---- | ------------------------ |
| CAN1 | MCAN0 | PA00 | PA01 | `app/board_app.cpp` 的 `init_can()` |

**单 CAN 变体（OTP25 = `0x00000000`）：**

| 接口 | 模式 | 说明 |
| ------- | ---- | ------- |
| CAN1 | CAN-FD | 帧类型由总线模式决定（`is_fdcan` 已移到 EP0）；收发器 5 Mbit 能力未实测 |

**双 CAN-FD 变体（OTP25 = `0x00000002`）：**

| 接口 | 硬件 | TXD | RXD | 模式 |
| ------- | -------- | ---- | ---- | ------- |
| CAN1 | MCAN0 | PA00 | PA01 | CAN-FD |
| CAN2 | MCAN3 | PA31 | PA30 | CAN-FD |

端口表在 `app/board_app.hpp`（`kCanPorts`、`can_port()`）；两个变体的模式相同，变体只
决定**启用几路**（`can_port_count()`）。CAN1 是 DM（达妙）电机总线。两个变体的控制器都跑
CAN-FD，经典帧照收照发，见 [README.md](README.md)；单 CAN 变体的收发器 5 Mbit 能力尚未
实测。

### CAN 编号：全仓库统一按机壳的 1 起始 [2026-09-04 起]

**机壳、主机 API、协议 `DataId`、测试工具输出，现在是同一个数字。**

| 机壳标注 | `CanPort` / `DataId` | 硬件 | TXD | RXD |
| ------- | ------------------- | -------- | ---- | ---- |
| **CAN1** | `kCan1` | MCAN0 | PA00 | PA01 |
| **CAN2** | `kCan2` | MCAN3 | PA31 | PA30 |

**改这一处之前，hpm_board 是全仓库唯一从 0 开始的板**：它的 `can0_transmit()` 写
`DataId::kCan0`，而 mc02 / c_board / ch32_board 的 `can1_transmit()` 一直写的是
`DataId::kCan1`。也就是说别的板早就和机壳对齐了，只有本板差 1。这个差异在 2026-09-04
造成过一次代价很大的误诊：一整套物理层排查（一直做到采样接收引脚电平）跑在了一个**根本
没插线的口**上，因为对话双方都说「CAN1」而指的是不同的接口。

**怎么自己复核**（不用拆机，不用调试器）：CAN 指示灯是**故障灯**，不是活动灯，
PB14 对应 CAN1、PB15 对应 CAN2（灯语见下一节）。只压一条总线、并让它没有对端，
那一路就会以约 1 Hz 慢闪（NO-ACK）：

```bash
./host/build/examples/dual_board_test stress 5000 45 1   # 只压 CAN1
```

跑的时候看板子，慢闪的那个灯挨着的接口就是 CAN1，即上表。
`[实测 2026-09-04，靠 CAN 指示灯确认]`

**主机 SDK 侧的入口**：hpm_board 系列板类不再有 `can0_transmit()` / `can1_transmit()`，
改为带端口枚举的单一入口：

```cpp
builder.can_transmit(libhcs::board::hcs::CanPort::kCan1, data);  // 机壳 CAN1
board.can_receive(CanPort port, const CanDataView& data);          // 一个回调带端口
```

旧的 `canN_transmit` **删除而不是弃用**，因为 `can1_transmit` 在新旧两套编号里都存在
且含义相反——留着它，老调用点会照常编译却打到另一条总线上。删掉之后每一处旧调用都变成
编译错误。定义在 `host/include/libhcs/board/hcs_can_port.hpp`。
**mc02 / c_board / ch32_board 不受影响**——它们的 `canN_transmit()` 本来就是 1 起始、
本来就写 `DataId::kCanN`，这次改动只是让 hpm_board 追上它们。

顺带：`DataId::kCan0` 现在由 hpm6e8y 使用（丝印 CAN0）。5321 / mc02 / c_board /
ch32_board 的机壳从 CAN1 起，不用这个 id。数值仍是线格式的一部分。

## UART

| 逻辑名 | 硬件 | TXD | RXD | 波特率 | 校验 | 来源 |
| ------- | -------- | ---- | ---- | ------ | ------ | ------------------------- |
| UART0 | UART2 | PB08 | PB09 | 921600 | 无 | `app/board_app.cpp` 的 `init_uart()` |

端口表在 `app/board_app.hpp`（`kUartPorts`）。两个引脚都启用了内部上拉；RX 额外开了
施密特触发器。收发均由 DMA 驱动（`HPM_DMA_SRC_UART2_TX/RX`），环形缓冲放在
`.ahb_sram`（在这颗 SoC 上天然是非缓存的）。

## LED（普通 GPIO 驱动的 RGB，低电平有效）

共阳极 RGB 灯：把引脚**拉低**才点亮对应通道（`make_gpio_pin<..., false>`）。定义在
`app/board_app.hpp`，由 `app/board_app.cpp` 的 `init_led_pins()` 完成引脚复用。

**单 CAN 变体（OTP25 = `0x00000000`）：**

| 颜色 | 引脚 |
| ----- | ---- |
| 蓝 | PA29 |
| 绿 | PA30 |
| 红 | PA31 |

**双 CAN-FD 变体（OTP25 = `0x00000002`）：**

| 颜色 | 引脚 |
| ----- | ---- |
| 蓝 | PA26 |
| 绿 | PA27 |
| 红 | PA28 |

## CAN 指示灯

单 CAN 变体没有每路 CAN 的独立指示灯（`kCanIndicatorPins` 为空）。双 CAN-FD 变体有两路
高电平有效指示灯：CAN1 为 PB14，CAN2 为 PB15；定义在 `app/board_app.hpp`，由
`app/board_app.cpp` 的 `init_can_indicator_pins()` 配置。

**这是故障灯，不是活动灯**——健康的总线灯是灭的，跑满流量也不亮。灯语在
`app/src/led/led.hpp` 的 `update()` 里，故障在最后一次错误约 5 s 后衰减回灭：

| 灯 | 含义 |
| --- | ---- |
| 灭 | 正常，无错误 |
| 慢闪 约 1 Hz | **NO-ACK**：没有节点应答——总线上只有自己、对端没上电、或 TX 线断 |
| 快闪 约 5 Hz | **接线故障（Bit0）**：总线拉不成显性——CAN_H/L 短路、接反或开路 |
| 双闪（闪两下停一下） | **信号错误**：位被破坏——缺 120Ω 终端、波特率不符或干扰，三者无法区分 |
| 常亮 | **bus-off**：错误过多，控制器已离线/正在恢复 |

排查一条不通的总线时，这四种状态就是第一手判据：慢闪指向"对端不在"，快闪指向
"这一段的电气接法错了"，双闪指向"接通了但信号质量不行"。

## USB（设备模式，高速）

`usb_use_high_speed()` 返回 true。引脚设为模拟功能，并且在 `board.c` 的
`board_init_usb_dp_dm_pins()` 里断开了 DP/DM 的 45 欧下拉；VBUS 走内部。

| 功能 | 引脚 | 说明 |
| -------- | ---- | ------ |
| USB0 DM | PA24 | 模拟 |
| USB0 DP | PA25 | 模拟 |

## Bootloader 强制常驻引脚 / 调试

> **这里的「触发」不是按键。** `board.c` 采的是**与 JTAG_TMS 复用的引脚**，函数名里的
> `button` 只是沿用叫法。**HPM5321 的两个变体都没有任何实体按键**
> `[实测 2026-08-03 双 CAN-FD / 2026-09-09 全部确认]`——触发只能靠复位时把该引脚短接到
> GND，或改用 `-DBOARD_BOOTLOADER_MODE=manual` 的 bootloader（见
> [../hpm6e8y/README.md](../hpm6e8y/README.md#bootloader-常驻没有按键靠短接或-manual-模式)）。

| 功能 | 引脚 | 说明 |
| ----------------- | ---- | ------------------------------------------------------------ |
| Bootloader 强制常驻 | PA07 | 与 JTAG_TMS 复用。复位后由 `board.c` 的 `board_check_bootloader_force_stay_requested()` 采样：内部上拉，**需拉到 GND 才算触发**；窗口只有 4×250us≈1ms，采样完毕后**恢复为 JTAG_TMS**，以便调试器仍能连接 |
| JTAG TCK | PA04 | 默认 JTAG 功能，app 不做复用 |
| JTAG TDO | PA05 | 默认 JTAG 功能，app 不做复用 |
| JTAG TDI | PA06 | 默认 JTAG 功能，app 不做复用 |

## PY 域

| 引脚 | 功能 | 说明 |
| ---- | ------------- | ------------------------------------------------------- |
| PY00 | SOC_GPIO_Y_00 | 在 `board.c` 里切到 SoC GPIO 域，不接外设 |
| PY01 | SOC_GPIO_Y_01 | 在 `board.c` 里切到 SoC GPIO 域，不接外设 |

## 时钟 / 时基（与固件相关的常量）

| 项目 | 取值 | 来源 |
| ------------------- | -------------------- | ----------------------------------- |
| MCHTMR0（app 定时器） | 4 MHz（0.25 us 一个 tick） | `board.c`；`kMchtmrClockName` |
| PTPC（CAN 时间戳） | 160 MHz AHB，6 ns 步进（`kCanTimestampNsPerUs = 960`） | `app/board_app.hpp` |
| Flash | 1 MiB XPI NOR | `board.h`（`BOARD_FLASH_SIZE`） |
