# ch32_board 移植工作日志

> **文档类型**：过程记录（按时间推进的工作日志）
> **适用范围**：`firmware/ch32_board/`
> **状态**：历史记录 + 现行待办混合。**"已完成"各节记录的是当时的阶段性结论，其中若干条已被后续进展超越**，超越之处已就地标注；文末"尚未完成"清单仍然有效。
> **相关文档**：[AGENTS.md](AGENTS.md)（现状以那份为准） · [README.md](README.md)（设计动机） · [PITFALLS.md](PITFALLS.md)

## 摘要

本文件是 ch32_board 从零移植过程的**流水账**：做了什么、为什么这么做、当时确认到哪一步。
它的价值在于保留推进顺序和判断依据；**要查"现在是什么状态"请看
[AGENTS.md 的"现状"一节](AGENTS.md#现状改代码前须知)**，那里是唯一权威。

移植目标：`firmware/ch32_board/`，libhcs 的第 4 个固件目标，芯片是 WCH **CH32H417**
（双核 RISC-V，Qingke V5F），选它是因为片上带 **USB 3.0 SuperSpeed** 设备控制器。
素材来源：EVT 包，位于 `~/Downloads/CH32H417EVT`（`[前机路径]`）。

> **原文有一句已过时的总述，保留并更正**：日志开写时的表述是"以下全部工作**仅通过编译
> 验证，尚未在硬件上跑过**（需要 WCH-Link）"。该状态已被推翻——2026-07-25 完成 USB SS
> 枚举实测，2026-07-26 完成 bulk 数据面与启动链的上板验证。详见 [AGENTS.md](AGENTS.md)。

## 已完成

### 板级 bring-up 与构建系统

- 把 WCH 标准外设库 vendored 进 `bsp/wch/`（Core、Peripheral、Debug、Startup、Ld），
  把 CH372Device USBSS 协议栈 vendored 进 `bsp/usb/`。
- CMake 构建 + `cmake/toolchain-wch-riscv.cmake`，复用 hpm_board 已经在用的**标准
  `riscv32-unknown-elf` GCC**（`GNURISCV_TOOLCHAIN_PATH=~/3rd_party/hpm`，`[前机路径]`）。
  按 `-march=rv32imafc_zicsr_zifencei -mabi=ilp32f` 构建，**不带** WCH 私有的 `xw` 扩展，
  因此不需要 MounRiver GCC。当时是 V5F 单核（`-DCore_V5F -DRun_Core=Run_Core_V5F`）。

  > **后续变更**：现已改为双核构建 `-DRun_Core=Run_Core_V3FandV5F`，V3F 作启动核兼
  > bootloader、V5F 跑转发。见 [README.md 关键移植决策](README.md#关键移植决策)。

- 保持严格 C11：厂商代码里的 GNU 关键字用 `-Dasm=__asm__ -Dtypeof=__typeof__` 做别名
  （不切到 gnu11）。`bsp/syscalls.c` 提供 newlib stub（`_write`/`_sbrk` 归 `debug.c` 管，
  **不要**加 `nosys.specs`——会冲突）。
- 构建方式：`cmake --preset debug -S firmware/ch32_board && cmake --build
  firmware/ch32_board/build` -> `ch32_board_app.elf` / `.bin`。占 128K flash 的约 82%。

### 版本管理（WCH 没有 git 形式的 SDK）

- WCH 只按芯片发 EVT zip 包（本次这份日期为 2026.04）。解决办法：vendored 进仓库 +
  在 `bsp/PROVENANCE.md` 里钉住来源（源文件路径、逐文件版本、重新 vendor 的流程、
  以及本地补丁清单）。

### 完整转发 app（CAN + UART + USB SS），采用规范写法

对齐的是**上游 hpm_board**（作为参考的 RISC-V 板），而不是 mc02：

- `app/src/board_app.hpp/.cpp` —— 单板的 `board::` 层，负责 GPIO 引脚与时钟；
  `init_can` / `init_uart` 返回外设时钟。
- `app/src/can/` —— 经典 bxCAN。`handle_downlink` **直接**写硬件发送邮箱
  （没有软件发送环形缓冲，**没有 `try_transmit`**）；`can_array[]` + `HardwareConfig`
  + `irq_handler()` 方法 + 轻量 ISR 转接。
- `app/src/uart/` —— WCH USART，中断模式（RXNE 收字节 + IDLE 作帧分隔，TXE 双缓冲发送）；
  `uart_array[]` + `HardwareConfig` + `irq_handler()`；`try_transmit()` 负责排空下行缓冲。
- `app/src/usb/vendor.*` —— libhcs 的会话/保活 + 序列化/反序列化逻辑（与板卡无关，
  直接复用共享的 `core/`），传输层换成 **WCH USBSS bulk EP1**（主机侧没有 TinyUSB-SS
  这种对应物）。
- `app/src/timer/` —— TIM10（32 位 `CNT_32`）跑 1 MHz -> 四分之一微秒时间戳。
- `app/src/led/` —— 空实现的状态 LED stub。`app/src/utility/` —— `lazy`、`ring_buffer`、
  `interrupt_safe_buffer`（共享），`interrupt_lock`（WCH 的 `__disable_irq`），`assert`。
- `app/src/app.cpp` —— 提供 `main()`（替换 demo 的 `app/User/main.c`，后者已排除出构建）；
  WCH 时钟 + `USB_Timer_Init` + `USBSS_Device_Init` + 用 range-for 初始化各外设；
  主循环与上游一致（先 usb 的 `try_transmit`，再
  `for (auto& u : uart_array) u->try_transmit()`）。

### USB SS 数据通路（真实实现，不是 stub）

- `vendor.cpp` 的 `ss::tx_write` 装填 EP1 IN（`UEP_TX_DMA/CHAIN_LEN/CHAIN_EXP_NUMP`，
  照搬 demo 里真实的 EP3 装填写法），配一个 `tx_in_flight` 标志；
  `usb_ss_ep1_in_complete()` / `usb_ss_ep1_out_complete()` 分别处理上行/下行。
- **本地补丁**打在 vendored 的 `bsp/usb/ch32h417_usbss_it.c` 上：原本 `DEF_UEP1` 的
  IN + OUT 分支是一个硬件 EP1<->EP2 DMA 回环（CH372 demo 的 echo 功能），替换为调用上述
  两个钩子。补丁标记为 `libhcs LOCAL PATCH`，登记在 `bsp/PROVENANCE.md`（**任何一次
  重新 vendor 之后都要重新打**）。EP1 即 bulk 管道，OUT `0x01` / IN `0x81`（与主机 SDK
  的端点一致）。

### 主机侧 SDK（USB 3.0 SS 调优）

- `host/src/transport/usb/usb.cpp` 本来就用异步 libusb（64 个在途 TX 传输，
  `LIBUSB_TRANSFER_FREE_BUFFER`）。把 `kReceiveTransferCount` 从 4 提到 16——只有 4 层深
  的接收池喂不满 5 Gbit 的管道。主机侧构建通过。

## 关键发现与决策

- **CH32H417 的 CAN 是经典 2.0B，不是 CAN-FD**——所以转发速度完全押在 USB SS 上。
- **ITCM/DTCM 是白送的**：链接脚本让全部代码从 ITCM 执行（`RAM_CODE=0x200A0000`），
  并把所有 `.data`/`.bss` 放进 DTCM（`RAM=0x200C0000`），因此 mc02 那种手工挑热路径
  往 ITCM 搬的做法在这里没有必要。
- 工具链坑：WCH 的 `CANx/USARTx/TIMx` 宏本质是 `reinterpret_cast`（不是常量表达式）
  ——要把整数形式的 `*_BASE` 传给 `Lazy` 的 consteval 构造函数，在运行时再转换。
- 我们自己写的 ISR 用朴素的 `__attribute__((interrupt))`（GCC 会生成 `mret`），从而绕开
  "主线 GCC 丢弃 WCH `interrupt("WCH-Interrupt-fast")`"的风险。

  > **后续补充**：厂商代码里的那些 ISR 并没有绕开这个问题，它正是让 USB 彻底死掉的
  > 真凶，最终用 `-Dinterrupt\(x\)=interrupt` 宏解决。完整分析见
  > [README.md 已解决：中断返回 ret vs mret](README.md#已解决中断返回-ret-vs-mret)。

### 主机侧板卡接口（2026-07-26）

- `core/include/libhcs/spec/ch32_board/{can,uart}.hpp` —— 通道描述表（CAN1/CAN2、
  UART1/UART2），形状与其他板一致。顺序与 `app/src/board_app.hpp` 里的 `kCanPorts` /
  `kUartPorts` 对应。
- `host/include/libhcs/board/ch32_board.hpp` —— `libhcs::board::Ch32Board`，
  `0xA511:0xD403`。没有 IMU / DBUS / GPIO 回调（这块板没有这些外设），也**没有**
  `uartN_config`：固件的 `uart_config_deserialized_callback` 是空实现，暴露出来只会
  静默地什么都不做。
- `host/examples/common/multi_board.hpp` —— 新增 `Ch32BoardSession`，并在
  `connect_any()` 里加了一项，因此 `rx_monitor` 等所有 example 都能直接驱动这块板。

### Bootloader / DFU（2026-07-26）

V3F 镜像本身就是 bootloader（它本来就掌管复位、时钟树，以及"要不要唤醒 V5F"这个决定）。
Flash 布局、metadata 记录、SHA-256 校验、顺序写 flash 的 writer、DFU 1.1 状态机，这些
原先就已到位；本轮补齐的是缺失的部分：

- **DFU 模式描述符**：`bsp/usb/usb_desc.c` 按 `libhcs_DFU_DEVICE` 在应用配置和 DFU 模式
  配置之间二选一（app 为 0，boot 为 1）——DFU 那份是单接口、无端点、
  `bInterfaceProtocol` 为 `0x02`、`wTransferSize` 为 512（等于 `DEF_USBSSD_UEP0_SIZE`，
  于是一个 DNLOAD 块恰好是一个控制 OUT 包）。已登记在 `bsp/PROVENANCE.md`。
- **DFU 模式下的 USB bring-up**：`boot/src/main.cpp` 的 `run_dfu_mode()` 复刻了 app 的
  初始化顺序（`Chip`、`libhcs_usb_init_descriptors`、`USB_Timer_Init`、
  `USBSS_Device_Init`），随后轮询 manifest/detach，最后拆链路并复位。这个轮询带一条
  **显式的内存屏障**——`Dfu` 没有 volatile 成员且只被 ISR 改写，否则 -O2 会把读提到循环
  外面变成死转。
- **detach 路径做成双核安全**：V5F 不能自己复位（它的复位向量是 flash `0x0`，即 V3F
  镜像），所以 `poll_dfu_runtime_reboot()` 的做法是断开 SS 链路、写 boot mailbox、置
  `SharedBlock::reset_request` 然后 park；真正执行 `NVIC_SystemReset()` 的是 V3F 的
  offload 循环。
- **主机侧产物**：`ch32_board_app.dfu`（裸 `.bin` + DFU 后缀，**不带** `ImageHash` 后缀
  ——这个 bootloader 是对自己烧进去的内容取哈希），以及仓库根的 `flash-ch32.sh`。

自烧录之所以安全，是因为 `Link_v3f.ld` 让 V3F 镜像从 RAM（`0x20100000`）执行，所以擦写
app 区不会卡住取指。

> **状态更新**：原文此处记为"尚未在硬件上验证"。启动链已于 2026-07-26 上板复验通过
> （flash 内容与 `app.bin` 一致、sha256 匹配、不进 DFU、成功唤醒 V5F）；但**经 DFU 实际
> 推一次镜像**仍未做，见下面第 6 条与 [README.md 尚未完成](README.md#尚未完成)。

### 转发板化：引脚定稿 + 链路复位修复（2026-08-06）

目标从"USB SS 能通"转向"这块板真的当转发板用"，做了三件事：

1. **修掉一个会让上行永久哑掉的 bug。** `usb::ss::tx_in_flight` 只由 EP1 IN 完成中断
   清零，而链路掉线/热复位/暖复位都会走 `USBSS_Reset_Init(ENABLE)` →
   `USBSS_Device_Endp_Init()` → `USBSS_USB_CLR_ALL`，把已装填的 IN 链直接丢掉，那个
   完成中断永远不来。于是拔插一次 USB 线之后 `tx_ready()` 恒假：下行还能收，kStart 的
   ack 却发不出去，主机永远建不起 session，看起来就是"板子死了"。修法是主循环里的
   `usb::ss::poll_link_reset()`，按 `USBSS_DevEnumStatus` 的下降沿清标志——**用沿不用
   电平**，因为裸 `SET_CONFIGURATION(0)` 也会清枚举状态却不拆端点，那时清标志会让下一
   个批次覆盖控制器仍在读的缓冲区。[代码分析，未上板复现]
2. **引脚按原理图定稿**，选型过程与座子对应表见
   [README.md 作为转发板的接线](README.md#作为转发板的接线引脚映射的由来)。过程中发现
   原先占位的 `uart2 = USART2 (PA2/PA3)` 在 MEU6（QFN88）上**根本没有引出**，属于会静默
   失效的配置；改为 USART3 `PA13/PA14`，两个脚都落在 2.54 排针 `J2` 上。
3. **把 V3F 的控制台从 USART1/PA9 挪到 USART8**（`ch32_board_boot` 定义
   `DEBUG=DEBUG_UART8`），否则 bootloader banner 会以 921600 baud 打进 uart1 的 TX 线。

顺带用手册确认了时钟基准：`Tq =（BRP+1）× tHCLK`，USART 波特率直接读 `HCLK_Frequency`，
这颗芯片没有 PCLK1/PCLK2 之分——`peripheral_clock()` 返回 HCLK 是对的。[RM]

Debug 构建 FLASH 占用升到 **96.08%**（125940 B / 128 KB），逼近上限，后续加代码前要么
先清 diag 仪表，要么改用 Release。

**当天稍晚上板复验，Petros breakout 首次跑通 USB 全流程**：SS 枚举（`usb 4-2: new
SuperSpeed USB device`，5000 Mbps）+ `rx_monitor` 的 `Connected: Ch32Board`，固件计数
下行/上行/主机收走各 88 包零丢包。两条排查经验：

- **插上线主机毫无反应，先复位板子，不要怀疑代码。** 现象是 `lsusb` 什么都没有、四个
  USB3 端口全读 `not attached`、内核日志一条 USB 事件都没有。因为 `HS_FALLBACK=0` 时
  板子永远不拉 D+ 上拉，SS 不通就是彻底静默，没有任何中间状态可看。修法就是
  [PITFALLS 1.3](PITFALLS.md#13-幻影低速设备会拖垮主机端口进而挡掉热插拔-实测) 那条：
  线插着复位一次。判据是固件自己记的 `diag[11]` —— RXDET→POLLING 的跃迁纯发生在 PHY
  层，早于任何固件逻辑，读到 `0x1a1`（U0+RXDET+POLLING+RECOVERY）就说明链路训练成功、
  代码无嫌疑。
- **openocd 每次 `halt` 都会让 USB 掉线**，所以"读一次 diag 再跑一次主机程序"这个顺序
  会让主机程序找不到设备（`No relevant devices discovered`）。读完必须重新
  `reset run` 再跑主机侧。

随后用 `J2.26`↔`J2.27`（PA9/PA10）一根跳线做 USART1 自环，跑通了**零外设的全链路测试**：
主机 → USB SS 下行 → 反序列化 → USART1 TX → 跳线 → USART1 RX → 序列化 → USB SS 上行 →
主机。结果：

| 波特率 | TX | RX | rx/tx | 说明 |
|---|---|---|---|---|
| 115200（上电默认） | 1000 msg/s，32000 B/s | 180 msg/s，**11520 B/s** | 18% | 11520 = 115200/10，**线跑满**，每帧 64 B 是 `max_receive_size` 满缓冲 flush |
| 921600（运行时切换） | 1000 msg/s，32000 B/s | 1000 msg/s，32000 B/s | **100%** | 一个字节不丢，msg 数 1:1，IDLE 组帧正确 |

第一行那个 18% 极具误导性，**判据是先算线速再判丢包**。第二行同时把"运行时改波特率"
从"已实现未上板"变成了实测通过——切换是在一个独立进程里发一条 `uart1_config` 完成的，
新波特率在会话结束后仍然保持（固件只在复位时回到 `kUartPorts` 里的上电值）。

**发热问题查证结论：不是超频。** 数据手册表 3-2 给的 FCORE1（V5F）默认模式上限就是
400 MHz，我们跑的正是 400 MHz；V3F/HCLK 100 MHz，上限 160 MHz。热源是三项叠加：两个核
都 100% 忙轮询（V5F 主循环是刻意的忙等，V3F 的 `Delay_Ms` 在 WCH 库里也是忙等）、SS PHY
因 `HS_FALLBACK=0` 永不进低功耗、以及 `VBUS → D1 → CJA1117B-3.3 LDO` 这条线性供电链把
压差全烧成热。可选的降温手段（未实施）：让 V3F 真正 WFI、把 V5F 降到 240 MHz
（`SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE`，HCLK 120 MHz 仍能整除出 1 Mbit/s CAN）。

## 尚未完成 / 上板 bring-up 待办（以下均已编译正确，但取值未经确认）

1. **USB SS EP1-OUT 的接收长度**：`vendor.cpp` 读 `UEP_RX_CHAIN_LEN`，并从 RX 缓冲区
   基址取负载（单缓冲）。需在目标板上确认链式 DMA 的接收长度/偏移语义，尤其是多包突发
   的情况。

   > **已完成**：2026-07-26 按芯片手册 RM 27.2.4 修正（`UEP_RX_DMA` 自增）并实测确认。

2. **GPIO 引脚**（CAN、USART）在 `board_app.cpp` 里是占位值——需按原理图确定。

   > **2026-08-05 更新**：目标板已换成 Petros CH32H417M Alef Breakout，要对的是那块板的
   > 原理图而非 EVT 包里的 `CH32H417SCH.pdf`。详见 [README.md 尚未完成](README.md#尚未完成)。
   >
   > **2026-08-06 已完成**：见上一节。剩下的只是通电实测收发。
3. **CAN 位时序与定时器分频**都以 `SystemCoreClock` 为基准——需确认 CAN/TIM 的内核时钟
   分频系数。

   > **2026-08-06 已完成**：手册确认基准是 HCLK，代码原本就是对的。[RM]
4. **UART 目前是中断模式**；DMA + 空闲线检测属于推迟的优化项（需要 DMAMUX 的请求映射）。
5. **烧录**：EVT 用的是 WCH-Link（wlink / openocd-wch），不是这台机器上的 HPM openocd。
6. 上板 SS 枚举 + 速度测试；bootloader / DFU。

   > **进展**：SS 枚举已完成（2026-07-25），bulk 数据面已完成（2026-07-26）；**速度测试**
   > 与 **DFU 上板实跑**仍未做。
