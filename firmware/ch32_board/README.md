# ch32_board（WCH CH32H417，Qingke V5F）—— USB 3.0 SuperSpeed 设备

> **文档类型**：背景说明（设计动机与移植决策）
> **适用范围**：`firmware/ch32_board/`
> **状态**：现行有效（部分章节含 bring-up 期间的阶段性描述，已就地标注最新结论）
> **相关文档**：[AGENTS.md](AGENTS.md)（**命令以那份为准**） · [PITFALLS.md](PITFALLS.md)（上板前必读） · [PROGRESS.md](PROGRESS.md)（工作日志） · [仓库根 AGENTS.md](../../AGENTS.md)

## 摘要

本文件讲 **ch32_board 为什么长这样**：为什么选这颗芯片、双核怎么分工、USB SuperSpeed
数据通路怎么搭、移植时做了哪些取舍。日常要用的构建/烧录/调试命令请看
[AGENTS.md](AGENTS.md)——两边命令若有出入，**以 AGENTS.md 为准**。

一句话说清选型动机：CH32H417 带 **USB 3.0 SuperSpeed（5 Gbps）** 设备控制器，正面解决
mc02、c_board 这些 FS/HS 板卡在"板子 -> 主机"方向撞到的转发天花板。

## 本文导航

| 章节 | 内容 |
|---|---|
| [当前状态](#当前状态) | 上板验证到哪一步了 |
| [App 布局](#app-布局libhcs-c-层) | V5F 上跑的 C++ 转发层目录结构 |
| [USB SS 数据通路](#usb-ss-数据通路) | 为什么要改厂商 demo 的 DMA 回环 |
| [目录结构](#目录结构) | 整个 `firmware/ch32_board/` 的组织 |
| [构建](#构建) | 编译命令（详版在 AGENTS.md） |
| [关键移植决策](#关键移植决策) | 不用 `xw`、USB 放 V5F、坚持严格 C11 |
| [USB SS bulk 实测天花板](#usb-ss-bulk-实测天花板2026-08-06) | 441/462 MB/s、13 µs 往返，以及怎么测的 |
| [作为转发板的接线](#作为转发板的接线引脚映射的由来) | CAN/UART 引脚怎么定的、座子上接哪几脚 |
| [已解决：中断返回 `ret` vs `mret`](#已解决中断返回-ret-vs-mret) | 曾经让 USB 彻底死掉的那个 bug |
| [烧录](#烧录wch-link) | WCH-Link 烧录与注意事项 |
| [Bootloader / DFU](#bootloader--dfu) | 双核架构下 bootloader 的特殊形态 |
| [尚未完成](#尚未完成) | 待上板确认的清单 |

## 当前状态

**已在目标板上枚举为 USB 3.0 SuperSpeed 设备（5 Gbps）**。2026-07-25 用 WCH-LinkE 实测：
设备身份是干净的 `A511:D403`（不带 `1a86`）、`bcdUSB 3.00`、bulk 端点 `0x01` / `0x81`
包长 1024 B、`bMaxBurst 15`，反复复位均稳定。两个核都在跑：V3F 启动、拉起时钟树并唤醒
V5F，V5F 运行 libhcs 转发 app。

**USB 2.0 回退是刻意关掉的**（`libhcs_USBSS_HS_FALLBACK=0`）。原厂协议栈在 SuperSpeed
训练失败约 1.5 秒后会切到 USB 2.0，而 USB 2.0 的 D+/D- 就是 **PB8/PB9——和 SWCLK/SWDIO
是同一对引脚**，所以一旦回退，调试口就跟着没了，只能等下次断电才能恢复。关闭后已上板
验证：把核 halt 住 5 秒（主机因此判定 USB 设备掉线）之后，调试器仍然能重新连上。

**bulk 数据通路已端到端验证**（2026-07-26）：11/11 个下行包经 EP1 解析并回 ack，零丢包。
主机侧 SDK 通过 `libhcs::board::Ch32Board`（PID `0xD403`）访问这块板，因此所有 example
看它和看别的板没有区别。仍未收尾的是 CAN/UART 引脚映射，还是占位值，见[尚未完成](#尚未完成)。

## App 布局（libhcs C++ 层）

```
app/src/
  app.cpp            # main(): WCH 时钟 + USBSS 初始化 + mc02 风格的转发循环
  can/               # WCH 经典 bxCAN（中断模式），can1=CAN1 can2=CAN2
  uart/              # WCH USART（RXNE+IDLE 组帧，TXE 双缓冲发送）
  usb/vendor.*       # 从 mc02 移植的 Vendor：session/serializer/deserialize；WCH SS 传输
  timer/             # TIM10（32 位），1 MHz，四分之一微秒时间戳
  led/               # 空实现的状态 LED stub
  utility/           # lazy / ring_buffer（移植）+ WCH 版 interrupt_lock、assert
```

`app.cpp` 提供 `main()`，替代 WCH demo 的 `app/User/main.c`（后者已从构建中排除）。

## USB SS 数据通路

CH372 demo 把数据当成**纯硬件 DMA 回环**转发：EP1-OUT 直接链到 EP2-IN，CPU 全程不碰
负载。而 libhcs 需要 CPU 把 OUT 方向的字节喂给 deserializer、再把 serializer 攒的批次
写进 IN 方向，所以 EP1 被改作 libhcs 自己的 bulk 管道：

- `usb/vendor.cpp` 的 `ss::tx_write` 通过 `UEP_TX_DMA / CHAIN_LEN / CHAIN_EXP_NUMP`
  装填 EP1 IN，并用 `tx_in_flight` 标志做门控，该标志由 EP1 IN 完成回调清除。
- `bsp/usb/ch32h417_usbss_it.c` 打了本地补丁（标记 `libhcs LOCAL PATCH`，登记在
  `bsp/PROVENANCE.md`），让 `DEF_UEP1` 的 IN/OUT 分支去调用
  `usb_ss_ep1_in_complete()` / `usb_ss_ep1_out_complete()`，而不是跑 demo 的回环逻辑。

> **状态更新**：本节原文写于枚举刚跑通、数据面尚未验证的阶段，当时的结论是"通路已写好
> 但未经验证，第一件要上板确认的事是 EP1-OUT 的收长语义"。该确认已于 2026-07-26 完成：
> 收长语义按芯片手册 RM 27.2.4 修正（`UEP_RX_DMA` 自增），bulk 通路端到端跑通、零丢包。
> 原文保留于此，因为它记录了当时的风险判断和验证思路。

原文风险描述（保留）：枚举过程只用到 EP0，所以这条通路**写好了但还没被证明**。上板后第一件
要确认的事是 EP1-OUT 的收长：回调读的是 `UEP_RX_CHAIN_LEN`，负载从 RX 缓冲区基址取
（单缓冲，无乒乓），多包突发的行为未经验证。

## 为什么用 WCH 裸驱动，而不是开源 USB 栈

**结论：截至 2026-07 不存在支持 USB 3.x SuperSpeed 设备侧的开源 MCU USB 栈。**
`bsp/usb/ch32h417_usbss_device.c` 这条厂商裸驱动路线是唯一选项，不是将就 —— 不必再花
时间去找"更好的 SS 库"。

- **CherryUSB**：device 端 SS 明确标为未实现，其 README 原文为"支持 USB2.0 全速和高速
  设备（USB3.0 超高速 TODO）"。仓库里的 `USB_SPEED_SUPER` / `USB_SPEED_SUPER_PLUS`
  常量只出现在 **host 侧** hub 枚举路径（`class/hub/usbh_hub.c`、`core/usbh_core.c`），
  那是"主机去枚举别人的 SS 设备"，与设备侧无关。另外其 `version.rst` 写明 **ch32 的
  porting 冻结在 `<= v0.10.2`，后续不再支持**。
- **TinyUSB**：速度枚举 `TUSB_SPEED_*` 中根本没有 SS 取值，是纯 USB 2.0 栈。
- **根因**：USB 3 的链路层（LTSSM、link training、包头 CRC、credit-based 流控）几乎全部
  固化在硅里，而各家 SS 控制器的寄存器模型互不兼容——不像 USB 2.0 有 dwc2 / ChipIdea /
  EHCI 这类事实标准 IP 可以共用一个 port 层。"可移植 SS 栈"的投入产出比过低，因此没有
  项目在做。[推断]
- 实际能跑 SS device 的只有三条路：**(1) 厂商裸驱动**（本板所选）；(2) 跑 Linux 的 SoC +
  dwc3 + gadget/configfs，MCU 不适用；(3) Cypress/Infineon FX3 自带 SS SDK，同样厂商专用。

### 值得对照的参考实现：HydraUSB3

[hydrausb3/hydrausb3_fw](https://github.com/hydrausb3/hydrausb3_fw) 面向 **WCH CH569**
（同为 WCH 的 USB3 SS RISC-V MCU），是目前最接近"开源 SS 参考实现"的东西：bare-metal SS
栈、Device Bulk + Burst，项目实测 **> 330 MB/s**；用 WinUSB WCID 描述符做到 Windows 免驱，
并给了 libusb 主机侧示例。它不是可直接链入的库，但有三处值得逐项对照本板实现：

1. **burst / NUMP 的配置方式** —— 决定能否吃满 SS 带宽，是 SS 相对 HS 的主要增益来源。
   本板当前用 `CHAIN_LEN` / `CHAIN_EXP_NUMP`（见上节）。**若 SS 实测吞吐远低于预期，
   先查这里，而不是怀疑协议层。**
2. **WCID 描述符** —— 若将来要 Windows 免驱可直接借鉴。
3. **libusb 主机侧代码** —— 与 host SDK 的 transport 层对照。

> [推断，未比对手册] CH569 与 CH32H417 的 SS 控制器同为 WCH 自研，寄存器组织可能同源，
> 但尚未逐项比对两颗芯片的手册确认。参考量级：WCH 官方对 CH32H417 宣称 USB3.0 带宽
> 450 MB/s。官方 SDK 见 [openwch/ch569](https://github.com/openwch/ch569)。

## 目录结构

```
firmware/ch32_board/
  cmake/toolchain-wch-riscv.cmake  # 裸机 RV32IMAC/ilp32 软浮点，不启用 F、不启用 WCH 的 'xw'
  cmake/merge_hex.cmake            # V3F@0x0 + V5F@0x10000 -> 合成单个 .hex
  bsp/ch32h417-evt/                # submodule: WCH 标准外设库（零改动，见 bsp/PROVENANCE.md）
  bsp/usb/                         # vendored CH372Device USBSS 设备栈（有本地 patch）
  bsp/syscalls.c                   # newlib stub（_write/_sbrk 由 debug.c 提供）
  app/User/                        # V5F app 核的 system_、ch32h417_it、conf
  app/src/                         # libhcs C++ 转发 app（V5F）
  boot/User/                       # V3F 启动核的 system_（拥有 SystemInit/HSE）
  boot/src/                        # V3F 启动 + offload 核，共享 SRAM mailbox
```

## 构建

```bash
export WCH_TOOLCHAIN_PATH="$HOME/3rd_party/MRS_Toolchain_Linux_X64_V240/Toolchain/RISC-V Embedded GCC15"
export WCH_TOOLCHAIN_PREFIX=riscv32-wch-elf-
cmake --preset debug -S firmware/ch32_board
cmake --build firmware/ch32_board/build
# -> build/ch32_board_app.elf  （V5F，链接地址 0x10000）
# -> build/ch32_board_boot.elf （V3F，链接地址 0x0）
# -> build/ch32_board_merged.hex  <- 烧这个
```

target：`ch32_board_app`、`ch32_board_boot`、`ch32_board_merged`（默认全部构建）。
`ch32_board_boot` 既是 V3F 启动核镜像，**同时也是** DFU bootloader——两者是同一个程序，
原因见下面的 [Bootloader / DFU](#bootloader--dfu)。

> 正式构建固定使用 MounRiver GCC15；旧 HPM 构建仅保留作历史尺寸对照。工具链约束与
> FLASH 占用实测数据见 [AGENTS.md 芯片与工具链](AGENTS.md#芯片与工具链)。当前机器已
> 安装上述路径；`libhcs-ci` / `libhcs-develop` 则固定使用
> `/opt/wch-gcc15`。完整约束见
> [仓库根 AGENTS.md](../../AGENTS.md#开发机环境路径约定重要先读这条再看任何路径)。

## 关键移植决策

- **使用 WCH GCC15，但不启用 `xw`，也不启用 F。** Qingke V5F 实现的是 RV32IMAFC 加上 WCH 私有的
  `xw` 压缩扩展。我们按 `-march=rv32imac_zicsr_zifencei -mabi=ilp32` 构建：不依赖私有 ISA，
  也不给 `__attribute__((interrupt))` 硬浮点 ABI——否则每个 ISR 都会在软件里再存一遍
  caller-saved FP 寄存器，而 Qingke HPE（`intsyscr=0x0F`）已经压过 GPR。固件里没有浮点运算。
  编译器来源与 ISA 选择是两件独立的事。
- **双核，USB 放在 V5F 上。** `-DRun_Core=Run_Core_V3FandV5F`；两个镜像合并成一个 flash
  产物（`ch32_board_merged.hex`，V3F 在 `0x0`、V5F 在 `0x10000`）。V3F 是启动核：它拥有
  时钟树（它的 `SystemInit` 会使能 HSE，而 V5F 那份 `system_ch32h417.c` **不会**），
  负责唤醒 V5F，随后通过共享 SRAM mailbox 跑非转发类的 offload 工作。V5F 跑转发快路径，
  USB 协议栈也在这一侧。
  注意这和 WCH 自己的双核 CH372 demo 相反——它把 USB 栈放在 V3F、让 V5F 闲着；我们坚持
  USB 留在 V5F，是为了让转发热路径不必跨核跳转。实测可行：USB DMA 访问 V5F 的 DTCM
  缓冲区没有问题。
- **保持严格 C11。** WCH 的头文件用了 GNU 的 `asm` / `typeof` 关键字；我们用
  `-Dasm=__asm__ -Dtypeof=__typeof__` 做别名，而不是把标准放宽到 gnu11，这样仓库
  "禁用 GNU 扩展"的规定对我们自己的代码仍然成立。

## USB SS bulk 实测天花板（2026-08-06）

**结论先给**：这颗芯片的 bulk 管道能跑到 **上行 441 MB/s / 下行 462 MB/s**，回环往返
**13 µs（min）/ 26 µs（p50）**。对得上 WCH 官方宣称的 450 MB/s，也把 README 里
[HydraUSB3 那节](#值得对照的参考实现hydrausb3) 留的问号（"吞吐远低于预期先查 NUMP"）
回答掉了。[实测]

测法：`-Dlibhcs_CH32_USB_BENCH=ON` 的专用固件（`app/src/usb/bench.cpp`）+ 纯 libusb 的
主机程序，**协议层、序列化器、串口全部不在路径上**。之所以必须专门做一版固件：不接外设
时上行没有数据源，而下行每包都要过反序列化器——625 MB/s 的逐字节扫描会先撞 CPU 上限，
测出来的是解析器不是管道。

| 方向 | 吞吐 | 占 500 MB/s 净荷上限 |
|---|---|---|
| EP1 IN（device→host） | 441.2 MB/s（3.53 Gbit/s） | 88.2% |
| EP1 OUT（host→device） | 462.5 MB/s（3.70 Gbit/s） | 92.5% |

（500 MB/s = 5 Gbit/s 经 8b/10b 编码后的净荷上限。）

| 往返延迟（64 B 净荷） | min | p50 | p99 |
|---|---|---|---|
| EP1 回环，IN 提前挂好 | 13.1 µs | 26.0 µs | 48.9 µs |
| 同上，同步写法（先等 OUT 完成再挂 IN） | 66.7 µs | 80.0 µs | — |
| EP0 控制传输（同步，不含应用代码） | 27.7 µs | 40.1 µs | — |

**后两行不能拿来相减**：主机路径的形状不同（同步提交-等唤醒 vs 预挂）。列在这里只是为了
说明同一根线上测法能差 5 倍——引用任何 USB 延迟数字之前先问清楚 IN 是不是预挂的。

两条实测约束，都值得记住：

1. **主机发给这块板的 bulk OUT 长度不能是 1024 的整数倍。** 设备的 RX chain 装填成
   `MAX_NUMP = 16`，**收满 16 包或收到一个短包**才触发完成中断；正好 1024 字节两个条件
   都不满足，包就卡在控制器里直到超时。第一次测 RTT 全是超时就是撞了这条。协议本身不会
   踩到（批次上限 1023 字节，天然是短包），但任何绕过协议直接发 bulk 的工具都要注意。
2. **生产路径离这个天花板有 16 倍的包数差距**，且这是设计使然：`tx_write()` 每次只装
   1 个 ≤1023 字节的包（`EXP_NUMP = 1`），bench 装的是 16×1024 的满突发。要吃满突发
   得先抬高 `core/` 的协议批次上限并同步改主机 SDK——**是协议改动，不是固件改动**。
   顺带补一条 bench 才用到的寄存器语义：多包突发必须设 `UEP_TX_DMA_OFS`（包间步长），
   `UEP_TX_CHAIN_LEN` 只是**最后一包**的长度，`EXP_NUMP` 最后写才启动传输。

## 作为转发板的接线（引脚映射的由来）

**结论先给**：引脚不再是占位值。CAN/UART 的最终选法记在
`app/src/board_app.hpp` 的 `kCanPorts` / `kUartPorts`（**那里是唯一权威**，本节只讲
为什么这么选）。一句话：**2.54 排针 `J2` 上只能引出 1 路 CAN + 2 路 UART，第 2 路
CAN 必须走 0.5 mm 的 `J3`。** [原理图 + 数据手册 V1.8]

| 通道 | 外设 | TX / RX | 座子引脚 |
|---|---|---|---|
| can1 | CAN1 | PA12(AF9) / PA11(AF9) | `J2.31` / `J2.29` |
| can2 | CAN2 | PB13(AF9) / PB12(AF9) | `J3.28` / `J3.13` |
| uart1 | USART1 | PA9(AF7) / PA10(AF7) | `J2.26` / `J2.27` |
| uart2 | **USART3** | PA13(AF4) / PA14(AF4) | `J2.32` / `J2.34` |
| 控制台 | USART8 | PB4(AF11) / PB3(AF11) | `J4.5` / `J4.6` |

四条约束把选择压到几乎唯一：

1. **封装先砍一刀。** MEU6 是 QFN88，`PB5/PB6/PB7`、`PC4/PC5`、`PF6/PF7`、`PA2/PA3`
   根本没引出来。原先占位的 `uart2 = USART2 (PA2/PA3)` 因此是**物理上不存在的引脚**
   ——配上去 `GPIO_Init` 静默无效，永远收不到一个字节。这是本轮改动里唯一的实质 bug。
2. **CAN2 没得选。** 去掉未封装的 `PB5/PB6`，CAN2 只剩 `PB13/PB12` 一种走法，而这两
   个脚只出现在 `J3`（DF12 板对板，0.5 mm，飞不了线，得配对插座）。所以**只用 2.54
   排针时这块板是单 CAN 板**。CAN3（`PD13/PD12` 或 `PF3/PF4`）同样只在 `J3`。
3. **uart2 换成 USART3 是为了不跨座子。** USART2 唯一走法 `PD5/PD6` 一半在 `J2.4`、
   一半在 `J3.34`；USART3 的 `PA13/PA14` 两个脚都在 `J2`，所以逻辑通道 2 挂到
   USART3。**逻辑编号与外设编号本来就不必一致**，`kUartPorts` 的顺序才是主机看到的
   顺序，主机侧无需任何改动。
4. **控制台必须让出 PA9。** 厂商 `Debug/debug.c` 的 V3F 默认控制台是 USART1/PA9，正好
   是 uart1 的 TX——每次复位都会往转发口上打一串 921600 baud 的 banner。改法是给
   `ch32_board_boot` 定义 `DEBUG=DEBUG_UART8`，让两个核共用 USART8。这不是将就：原理图
   把 `J4` 的 5/6 脚直接标成 `TX8`/`RX8`，板子本来就是这么设计的。

还有两件硬件事实，选完引脚也绕不开：

- **板上没有任何收发器。** 这是裸 breakout，CAN 引脚必须外挂 CAN PHY（`SN65HVD230`
  一类）才看得到总线；`J1` 的 VBUS 经 `D1` 到 LDO，**没有保险丝**。
- **没有可用的状态灯。** 全板只有 `D2`（红色电源指示，`R2` 1.5K 直连 3V3），不受
  GPIO 控制，所以 `app/src/led/led.hpp` 保持空实现。要现场指示得自己从 `J2` 的空脚
  （`PE0`/`PE1`/`PF0` 等）外接一颗。

**CAN 位时序已由手册确认，不再是推断** [RM]：BTIMR 的 `Tq =（BRP[9:0]+1）× tHCLK`，
即 CAN 的时间份额直接由 HCLK 分频，没有额外的外设分频器；驱动写入的是
`CAN_Prescaler - 1`。因此 `board_app.hpp` 里 `peripheral_clock()` 返回 `HCLK_Frequency`
是对的：100 MHz / 5 = 20 MHz，20 tq/bit 得 1 Mbit/s，采样点 16/20 = 80%。同一份 HCLK
也是 USART 波特率（`USART_Init` 直接读 `HCLK_Frequency`）和 TIM 预分频的基准——这颗芯片
的 `RCC_ClocksTypeDef` 里根本没有 PCLK1/PCLK2 之分。

## 已解决：中断返回 `ret` vs `mret`

**结论**：这就是当初让 USB 彻底死掉的那个 bug。`[实测]`

WCH 的 ISR 声明为 `__attribute__((interrupt("WCH-Interrupt-fast")))`。主线 GCC
**不接受这个参数，并且会把整条 attribute 丢掉**：

```
warning: argument to 'interrupt' attribute is not '"user"', '"supervisor"', or '"machine"'
```

于是 `TIM12_IRQHandler`、`USBSS_IRQHandler`、`USBSS_LINK_IRQHandler`、`USBHS_IRQHandler`
全都被当成普通函数编译，收尾指令是 `ret`。而启动代码把 `intsyscr`（CSR `0x804`）设成
`0x0F`（HPE 硬件压栈 + 中断嵌套），不走 `mret` 返回就既不会恢复 `mstatus.MIE`、也不会
弹出硬件栈：**第一个厂商 ISR 触发之后，中断就被永久关掉了**，USB 链路状态机随之停摆，
而 app 还在主循环里若无其事地空转——症状极具迷惑性。

修法在 `CMakeLists.txt` 里，思路和已有的 `-Dasm=__asm__` 一致：

```cmake
-Dinterrupt\(x\)=interrupt
```

这个函数式宏只在 `interrupt` 后面紧跟 `(` 时才展开，因此它把 WCH 的
`interrupt("WCH-Interrupt-fast")` 改写成不带参数的 `__attribute__((interrupt))`
（GCC 会为它生成 `mret` 以及寄存器保存/恢复），同时不影响我们自己写的
`__attribute__((interrupt))` 处理函数。**换工具链之后务必复验**：

```bash
riscv32-wch-elf-objdump -d --disassemble=USBSS_LINK_IRQHandler \
    build/ch32_board_app.elf | tail -3    # 结尾必须是 mret
```

## 烧录（WCH-Link）

> **命令以 [AGENTS.md 烧录（WCH-Link）](AGENTS.md#烧录wch-link) 为准。** 本节保留的是
> bring-up 时期使用的 `~/3rd_party/wch-openocd` 写法（`[前机路径]`）；前机后来改用了
> MounRiver 包内自带的 OpenOCD，路径不同但**命令序列和下面所有注意事项完全一致**。
> 两份路径当前机器都未安装，属正常，见
> [仓库根 AGENTS.md](../../AGENTS.md#开发机环境路径约定重要先读这条再看任何路径)。

```bash
OCD=~/3rd_party/wch-openocd      # [前机路径]
$OCD/bin/openocd -f $OCD/bin/wch-riscv.cfg \
    -c "init" -c "wch_riscv unfreeze" -c "halt" \
    -c "program firmware/ch32_board/build/ch32_board_merged.hex verify" \
    -c "reset run" -c "exit"
```

- **`wch_riscv unfreeze` 必须夹在 `init` 和 `halt` 之间。** 挪到 `halt` 后面它会静默
  失效；此时 `program` 会报 `Read-Protect Status Currently Enabled` 或
  `error writing to flash at address 0x00000000`，而 `flash erase_sector` 却仍然报成功，
  回读全是 `e339e339`（那是读保护状态下的假数据——真正擦干净的 flash 读出来是
  `ffffffff`）。MounRiver Studio 传的正是 `-c init -c "wch_riscv unfreeze"`；它捆绑的
  openocd 二进制和 `wch-riscv.cfg` 与 `~/3rd_party` 下那份**逐字节相同**，所以当 IDE
  能烧、命令行烧不进时，该怀疑的是命令序列，不是工具。
- **不要加 `flash erase_sector wch_riscv.flash 0 last`。** 这个范围会碰到 option byte。
  `program` 自己会擦它要写的区域；而且在 `unfreeze` 和 `program` 之间多插一条 flash
  命令，还会把 unfreeze 的效果抵消掉。
- **让板子跑起来要用 `wch-riscv.cfg`（单 target），不要用 `wch-dual-core.cfg`。**
  双核配置的 `reset run` 会同时放行**两个** hart，于是 V5F 不等
  `NVIC_WakeUp_V5F(0x10000)`，直接从 flash `0x0` 开始执行 V3F 镜像——症状是 V5F 的
  `mtvec = 0x20100003`。需要**同时观察**两个核时，双核配置仍然是对的选择
  （`targets wch_riscv.cpu.0` / `.1`）。
- **`halt` 之后读到的 PC 值在这颗芯片上不可信**：WCH 的 openocd 在 halt 的同时会顺带
  复位芯片，因此反复读到的都是复位期间 loadcode 拷贝循环里的地址（`0x00004020`）。
  判断"固件是不是在跑"要看 **USB 枚举**，不要看 PC，也不要看 `v5f_ready`（V3F 每次启动
  都会把它清零）。
- halt 会让主机判定 USB 设备掉线，而且 **`resume` 拉不回来——只有 `reset run` 可以。**

## Bootloader / DFU

在这颗芯片上，bootloader 不是一个独立分区：V3F 先复位出来，拥有时钟树，并且决定要不要
唤醒 V5F——这本来就是 bootloader 干的活。所以 `ch32_board_boot` 一身二任。"启动 app"
的含义是 `NVIC_WakeUp_V5F()` 而非跳转，而且 V3F 之后**继续常驻**，跑它的 offload 循环。

Flash 映射（`boot/src/flash/layout.hpp`）：

```
0x00000 .. 0x10000  bootloader   V3F 镜像，DFU 永不写入此处
0x10000 .. 0x70000  application  V5F 镜像，DFU 唯一可写区域
0x70000 .. 0x72000  metadata     一个擦除块：magic + image_size + sha256
```

启动判定逻辑在 `decide_boot_mode()`：

1. boot mailbox 里是 `DFU0`（app 被要求 detach）-> 进 DFU 模式；
2. boot mailbox 里是 `APP1`（刚下载完成并已校验哈希）-> 直接启动，不重复校验；
3. 其余情况：对 app 区做哈希，与 metadata 记录比对。镜像损坏或缺失一律进 DFU 模式，
   绝不启动。

用调试器烧录会绕过 DFU，所以构建流程自己补上 metadata 记录：`cmake/gen_metadata_hex.py`
对 `ch32_board_app.bin` 取哈希，把记录生成到 `0x70000` 并并入 `ch32_board_merged.hex`。
少了这一步，调试器烧过的板子每次冷启动都会掉进 DFU。

通过 DFU 烧 app（不接调试器，所以下面的 SWD/USB 引脚冲突不适用）：

```bash
./flash-ch32.sh            # 在仓库根执行；PRESET=debug 可得到 -O0 镜像
# 等价于：
dfu-util -d 0xa511:0xd403 -a 0 -D firmware/ch32_board/build/ch32_board_app.dfu
```

`ch32_board_app.dfu` 是裸 `.bin` 加一个 DFU 后缀——**刻意不用** mc02/c_board 那种带
`ImageHash` 后缀的形式，因为这块板的 bootloader 是对**自己烧进去的内容**取哈希，而不是
信任后缀。两个镜像枚举出来都是 `A511:D403`；区分方法：DFU 模式那个的接口
`bInterfaceProtocol` 为 `0x02`，产品字符串是 `HCS Bootloader v<版本号>`。

进入 DFU 模式的两条路：上电时没有有效镜像，或者用 `dfu-util` 让正在运行的 app detach
（app 在接口 1 上带了 DFU 运行时接口，见 `app/src/usb/dfu_runtime.cpp`）。detach 时
app 会断开 SS 链路、写 boot mailbox 然后 park；**真正执行复位的是 boot 核**，因为 V5F
的复位向量指向 flash `0x0`——那是 V3F 镜像——它自己复位会跑到另一个核的代码上去
（`SharedBlock::reset_request`）。

这块板**没有恢复按键**，所以如果 app 能枚举但不响应 DFU_DETACH，就仍然只能靠 WCH-Link 救。

## 尚未完成

- **DFU 的上板实跑。** 下载通路已经构建、描述符也在 ELF 里核对过，但还没有在真实硬件上
  经 DFU 推过一次镜像。首先要盯的两点：擦写动作发生在 USBSS 中断里；V3F 镜像是从 RAM
  执行的（`Link_v3f.ld` 会把它拷到 `0x20100000`），这正是自烧录安全的前提——两点都需
  上板确认。
- ~~**CAN 与 USART 的 GPIO 引脚映射**在 `app/src/board_app.cpp` 里还是占位值。~~
  > **2026-08-06 完成**：已按 Petros breakout 原理图定稿，见
  > [作为转发板的接线](#作为转发板的接线引脚映射的由来)。仍未做的是**通电实测**：
  > 挂上 CAN PHY 和 USB-TTL 之后，确认 can1/uart1/uart2 真能收发。
- ~~**CAN 位时序与定时器分频**都是以 `SystemCoreClock` 为基准推出来的~~
  > **2026-08-06 完成**：`Tq =（BRP+1）× tHCLK` 已在 RM 中查到，USART/TIM 同样以 HCLK
  > 为基准，代码里的 `peripheral_clock()` 是对的。见上一节末尾。[RM]
- SS 速度测试（WCH 随 CH372 demo 提供了一个主机侧速度测试工具）。
- **只有 USB 3.0 口能用**：`libhcs_USBSS_HS_FALLBACK=0` 关掉了回退，插到 USB 2.0 口或
  用只有 USB2 线芯的线缆时**根本不会枚举**。部署到机器人上时这是一条硬约束。
