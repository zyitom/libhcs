# hpm_board 固件指南

> **文档类型**：现行规范（板级）
> **适用范围**：`firmware/hpm_board/`，HPMicro HPM6E8Y / HPM5321（Andes RISC-V）
> **状态**：现行有效
> **相关文档**：[仓库根 AGENTS.md](../../AGENTS.md) · [PITFALLS.md](PITFALLS.md)（选型与踩坑实录） · [USB_OPTIMIZATION_LOG.md](USB_OPTIMIZATION_LOG.md)（USB 调优实录） · [BUILD_ENVIRONMENT.md](BUILD_ENVIRONMENT.md)（完整环境搭建） · [SOF_TIMEBASE.md](SOF_TIMEBASE.md) · [CONTROL_TIMING.md](CONTROL_TIMING.md)

> 本目录专属指南，叠加在仓库根 `AGENTS.md` 之上。完整编译环境（依赖清单、工具链下载、
> 烧录）见 `BUILD_ENVIRONMENT.md`；实测过程与踩坑见 `PITFALLS.md` 与
> `USB_OPTIMIZATION_LOG.md`，此处只列**现在该怎么做**。

## 摘要

hpm_board 与其他三块板最大的不同：它是 **RISC-V（Andes 核）**、用 **HPM SDK 超级构建**。
工具链是仓库外的预编译二进制，装在哪由环境变量决定——本机实际安装状态见
[仓库根 AGENTS.md 开发机环境路径约定](../../AGENTS.md#开发机环境路径约定重要先读这条再看任何路径)。

## 芯片与工具链
- MCU：**HPM6E8Y / HPM5321**（HPMicro，**Andes RISC-V 核，不是 ARM**）；HPM6E8Y 双核。
- ISA/工具链：RISC-V，用 HPMicro GNU 工具链 `rv32imac_zicsr_zifencei_multilib_b_ext`
  （带 B 扩展 multilib），需 `riscv32-unknown-elf-gcc`。**不要**用 arm-none-eabi-gcc
  或 WCH 工具链。工具链留在仓库外，不入库、不做 submodule；下载与安装见
  `BUILD_ENVIRONMENT.md`。
- HPM SDK **v1.12.0 随仓库自带**（`bsp/hpm_sdk`，submodule），无需另装。submodule 指向
  fork `zyitom/hpm_sdk`，当前提交 `v1.12.0-3-ge4347411`，分支 `migrate-v1.12.0`。
- 所有 hpm_board 镜像（app、bootloader）统一编译共享的
  `firmware/c_board/bsp/tinyusb` **v0.21.0**，接入点是 `cmake/current_tinyusb.cmake`；
  HPM SDK 自带的 TinyUSB v0.20.0 及 fork 内旧补丁不再进入镜像。构建前必须初始化该
  TinyUSB submodule；HPM SDK 仍提供 SoC、USB PHY 与寄存器驱动。

## 固件镜像

`firmware/hpm_board/`（超级构建，默认）出 USB 数据固件（单核，USB vendor bulk）：
**USB 延迟最低（p50 100us）**，core1 不释放。

> 选型依据（5321 vs 6E8Y、多板方案、EtherCAT 为何被否）见
> [PITFALLS.md](PITFALLS.md) 第 1-2 节。EtherCAT 桥固件已于 2026-09-08 移出本仓库，
> 归档在 `~/Desktop/ethercat-archive-2026-09-08/`。

## 主机侧延迟调优（每次重启都要重做）

```bash
sudo ./host-tuning.sh          # 应用：governor=performance、RT 限流关闭、USB autosuspend 等
sudo ./host-tuning.sh --check  # 只报告不改（还会打印本机的控制器、IRQ、P/E 核归属）
sudo ./host-tuning.sh --pmqos  # 另开一个终端，测量期间持住（1 kHz 控制环下值 p99.9 约 9us）
```

**除内核 cmdline 外全部不持久化。** 逐项状态、证据等级、以及被实测否掉的做法见
[../../HOST_TUNING.md](../../HOST_TUNING.md)（现行权威）第 1-3 节。测 USB 延迟前
**必须**先把主机弄到高频状态；但**别拿压测（gap=0 紧凑 ping-pong）的结论去判断
1 kHz 控制环的主机调优**——两种工况结论方向相反，判据见 `HOST_TUNING.md` 1.1 与
`USB_OPTIMIZATION_LOG.md` 第 5 节。

## 烧录

App 走 USB DFU：`./tools/flash.sh hpm5321`（默认 release；末尾加 `debug`）。
手工 `dfu-util` 见本文「构建」一节。`flash-ecat.sh` / `flash-ecat-swap.sh`
两个脚本随 EtherCAT 桥一并移出仓库（见归档）。

## CAN 采样点：钉死 87.5%，不要动 [实测 2026-08-03]

`can.hpp` 把仲裁域和数据域的采样点都钉死在 87.5%，**不要改回 SDK 默认（实测恒为
75.0%），也不要照厂商推荐表改**：

```cpp
config.can20_samplepoint_min = 875U;   config.can20_samplepoint_max = 875U;
config.canfd_samplepoint_min = 875U;   config.canfd_samplepoint_max = 875U;
```

症状签名：采样点不一致时 **classic 双向通、FD 单向不通**（`PSR.DLEC = ACK error`）。
真实电机（DJI、达妙 MIT、瓴控）全是 87.5%。为什么必须对齐总线而不是推荐表、
判读与自查方法见 [PITFALLS.md](PITFALLS.md) 第 3 节。

**更一般的硬约束**：本板 CAN 的协议（classic / FD）、仲裁与数据段速率、采样点，上限
全部由总线对端的**电机硬件**决定（DJI、达妙 MIT、瓴控的电机固件在仓库之外，无法
修改）`[硬件事实，用户确认 2026-09-12]`。吞吐/延迟优化**不要**以「升级 CAN-FD /
提高波特率 / 改采样点」为建议方向——总线参数没有可动的余地；可行杠杆在主机侧协议、
成帧与软件路径，见 [USB_OPTIMIZATION_LOG.md](USB_OPTIMIZATION_LOG.md) 与
[HOST_TUNING.md](../../HOST_TUNING.md)。

## UART 运行时改波特率：快照，不回读 [实测 2026-08-05]

读 `DLL`/`DLM` **必须在 TX DMA 停稳之后**（DLAB=1 时 `DLL` 与 `THR` 共址，回读动作本身
会把数据字节写进除数锁存器，端口从此不出声）。现行实现：`snapshot_divisor()` 只在
init 和 `abort_transmit()` 之后采样，遥测只读快照；`uart_set_baudrate()` 求解失败时
**不清 DLAB**，调用方必须无条件自己清一次。症状签名与修法见
[PITFALLS.md](PITFALLS.md) 第 4 节。配置被拒要让主机看见，走下一节的 EP0，不走带内通道。

## EP0 配置通道

**结论先行：每帧的、周期性的、要和数据定序的 → EP1 bulk。构造期问一次的、失败必须让
主机看见的 → EP0。** 协议定义在 `core/include/libhcs/protocol/vendor_control.hpp`
（主机与固件共用），板端实现在 `app/src/usb/vendor_control.cpp`，主机端封装在
`host/include/libhcs/board/hcs_config.hpp`。

| bRequest | 方向 | wIndex | 载荷 | 语义 |
|---|---|---|---|---|
| `0x40 kGetInterface` | IN | 0 | `InterfacePayload` | 版本、CAN/UART 路数、哪几路是 CAN-FD |
| `0x41 kGetCanConfig` | IN | 总线号 | `CanConfigPayload` | 该总线实际模式 |
| `0x42 kSetCanConfig` | OUT | 总线号 | `CanConfigPayload` | **只校验不重配**，与固件不符即 STALL |
| `0x43 kGetUartConfig` | IN | 端口号 | `UartConfigPayload` | **实际生效**的波特率（由分频器反推） |
| `0x44 kSetUartConfig` | OUT | 端口号 | `UartConfigPayload` | 求解失败即 STALL，寄存器不动 |
| `0x45 kGetCanStatus` | IN | 丝印编号 | — | 控制器错误寄存器回读，判读表见 [PITFALLS.md](PITFALLS.md) 第 5 节 |
| `0x47 kGetLatencyBreakdown` | IN | — | — | 延迟拆解埋点，恒开（见下） |

> 曾用于分端点协商的 `0x46 kSetEndpointMode` 已随分端点拆除（2026-09-05），
> **编号空出不复用**——老主机来问会拿到 STALL，而不是被当成某个后加的请求重新解释。

三条必须知道的约束：

1. **`kSetCanConfig` 不会重配控制器。** `mcan_init()` 那一整块（87.5% 采样点、TDC、
   外部 PTPC 时基喂 TSU、sync 滤波器）是逐条实测调出来的，运行时重跑等于把它们全部
   重新置于风险中，还要断总线。所以板端保留编译期的 `CanPort::mode`，`SET` 只做
   "主机的预期和我一致吗"这一件事。**推论：`CanDataView::is_fdcan` 在 hpm_board 上
   已被忽略**，FD 总线一律发 FD 帧；要知道某条总线是什么模式，读 `canN_is_fd()`。
2. **回读永远不等于请求值。** `effective_baudrate()` 由实际写进去的分频器反推，
   115200 读回 114942，921600 读回 909090。**用容差比，不要用相等比**——相等比会在
   几乎所有 80 MHz 除不尽的速率上误报失败。
3. **EP0 通道不受 session 门控。** 主机在板对象构造期就下发，那时 keepalive 线程还
   没开出 session；session 掉线后回读也必须继续可用。

**没做 EP0 握手的主机开不了 session。** 板端记住"这个主机读过我的接口描述符没有"
（`kGetInterface` 即握手），没读过就不应答 `kStart`，主机在自己的 ack 超时后约 1 秒抛
`Timed out waiting for SESSION_ACK`。它挡的是**旧版 SDK 连新固件**；正常路径下版本串
校验已经把旧 SDK 挡住了，所以门控只在 `dangerously_skip_version_checks=true` 时才真正
生效——而仓库自己的全部测量工具都开着它，正是最容易混用镜像的那一群。落地时的坑
（握手时序、旧主机蹭握手）见 [PITFALLS.md](PITFALLS.md) 第 5 节。

**`kSession` 不搬 EP0**，三条理由，按硬度排：

1. **keepalive 要证明的是数据管道活着。** EP0 健康不等于 bulk 健康——本板出现过 bulk OUT
   在总线复位后没重新 arm、发得出收不到、而 EP0 一路正常应答的故障。
2. **控制传输是带外的，没有定序。** 现在"session 关了"和"这一帧数据"的先后由字节流免费
   保证；搬走之后要么接受竞态，要么自己补一套 quiesce/drain。
3. **同步控制传输把掉线检测拖慢。** bulk + 条件变量超时 200 ms；EP0 路超时 1000 ms，
   且会占住 keepalive 线程。

**延迟拆解埋点恒开**（`libhcs_LATENCY_PROBE` 开关 2026-09-05 已删除）：`diag/latency.hpp`
的四个时间戳实测至多 0.5% 包率（`USB_OPTIMIZATION_LOG.md` 第 11 节），落在够不到的余量
上，而替代方案 `-Dlibhcs_CAN_DIAG=ON` 会污染 `kUart0` 上行流（见
[PITFALLS.md](PITFALLS.md) 第 8 节）。EP0 并发请求对 bulk 包率免费到 kHz 量级
（`USB_OPTIMIZATION_LOG.md` 第 11 节）。

## USB 现行约束（机制与全部数据见 [USB_OPTIMIZATION_LOG.md](USB_OPTIMIZATION_LOG.md)）

- **OUT 背压**：CAN 过载时板端不再 arm OUT 端点、让控制器 NAK 把主机压回 CAN 线速，
  不静默丢帧。开关 `CFG_TUD_VENDOR_RX_MANUAL_XFER`，`usb::Vendor::poll_downlink_arm()`
  门控看 `can::max_transmit_queue_depth()`：48/64 停 arm、16/64 恢复（迟滞）。**必须有
  逃生门**：扣满 20 ms 就放弃背压（session lease 1000 ms），宁可丢帧也不丢会话。它限的
  是稳态速率，不是无损保证；代价 −4.4% 峰值包率（`USB_OPTIMIZATION_LOG.md` 10.1）。
- **主循环里 CAN 要排在 bulk 前面。** 一趟主循环内的调用顺序是板端唯一的优先级机制：
  92 kB/s 下 CAN 优先 p99 123.7，bulk 优先 128.4。
- **UART 会顶掉 CAN 的尾部延迟**（head-of-line blocking，p99 +40us 量级）：分端点方案
  已于 2026-09-05 拆除，此代价当前无条件存在。机制、定价与还剩的杠杆见
  `USB_OPTIMIZATION_LOG.md` 第 12-13 节。
- **调不动的旋钮别再 A/B**（全部实测否掉，见 `USB_OPTIMIZATION_LOG.md` 第 3 节）：
  `CFG_TUD_TASK_EVENTS_PER_RUN`（惰性）、host transfer 池深度（无关）、multi-qTD
  （turnaround 不在关键路径；`libhcs_COPY_THEN_ARM` 开关已删，恒为"处理完再 arm"）、
  xHCI IMOD（非瓶颈）。
- **不要把 TinyUSB 热路径放进 ILM**：`TU_ATTR_FAST_FUNC` 加在 `tud_task_ext` 一类
  "入口在热路径、被调方散落在 flash"的函数上是净倒退（包率 +0.11%、主循环 −4.0%）。
  `.fast` 只适合叶子函数或自成闭环的调用子图。机制见 `USB_OPTIMIZATION_LOG.md` 3.2。

同批落在共享 `firmware/c_board/bsp/tinyusb` 里的改动（**动它之前先读**）：

| 改动 | 位置 | 结论 |
|---|---|---|
| 恢复 ChipIdea SETUP tripwire（SUTW 信号量重试环） | `dcd_ci_hs.c` | **保留**，正确性回归修复（见下） |
| `CFG_TUD_MEM_DCACHE_ENABLE=0` 时把 `dcd_dcache_*` 编译掉 | `dcd.h` / `usbd.c` | 保留，与下一项合计 +1.24% 包率 |
| ISR 端点扫描改为只遍历 `ENDPTCOMPLETE` 的置位 | `dcd_ci_hs.c` | 保留，同上（两者缺一都更差） |
| ~~USB 热路径加 `TU_ATTR_FAST_FUNC` 进 ILM~~ | — | **已撤销**：包率 +0.11%，主循环 −4.0% |

tripwire 那项是**真回归修复，不是上游老毛病**：HPM SDK 自带的 `dcd_hpm.c` 实现了
`set_sutw`/`get_sutw` 重试环，而 0.21 通用的 `dcd_ci_hs.c` 清完 `ENDPTSETUPSTAT` 就直接
把 qhd 指针交出去——`USBCMD_SETUP_TRIPWIRE`（`ci_hs_type.h`，HPM 上是 `USBCMD` bit 13
[RM]）在该文件里定义了却没人用。删掉它，背靠背 SETUP 会让 usbd 拿到撕裂的 8 字节。

## 主机侧板类：一块芯片一个类 [2026-09-05]

`host/include/libhcs/board/hpm5321.hpp` 里的 `Hpm5321` **同时服务两块 PCB**（单 CAN
`0x5321` 与双 CAN `0x5322`），**类名不编码总线数**——它是运行期事实，来自 EP0
`kGetInterface` 的 `can_count`（与固件读 OTP 第 25 字判板型是同一个事实来源）。
三条实现约定：

1. **描述符表现在是"镜像的容量"，不是"某块板的配置"**。`spec::hpm5321`
   固定两条，真实条数读 `interface().can_count`。
2. **越界在运行期拦，不在编译期**。`can_transmit()` 用 `interface().can_count` 兜底，
   在单 CAN 板上发 CAN2 会抛出并指明这块板只有 CAN1。
3. **一个设备可以有多个 PID**。`DeviceScanner::select_device` / `create_transport` /
   `Handler` 收 `std::span<const uint16_t>`；单 PID 的板走保留的便利重载。

来龙去脉见 [PITFALLS.md](PITFALLS.md) 第 7 节。

## 硬件中断与数据路径归属（6E8Y / 5321 共用同一份 app 代码）

**延迟相关的事件全部是中断驱动的，没有该用中断却在轮询的地方。**

| 事件 | 上下文 | 优先级 | 备注 |
|---|---|---|---|
| CAN RX（MCAN0..3） | **ISR** | 3（最高） | 进 ISR 就排空 FIFO + 序列化，不甩给主循环 |
| USB（USB0） | **ISR** | 2 | `dcd_int_handler` 在 ISR 里处理硬件；TinyUSB 的回调（`tud_vendor_rx_cb`）按其设计延到主循环的 `tud_task()`——**代价 < 0.85us，见下** |
| UART RX/TX | **ISR** | 1 | |
| 1 kHz tick | **ISR**（MTIP） | 绕过 PLIC | 故意只做一个计数器自增，LED 等工作甩到主循环，避免抢占 CAN ISR |
| 跨核上行门铃 | **ISR**（MBX0B） | 低于 PDI | 核对调布局 |
| 跨核 flash RPC | **ISR**（MBX1A） | — | 不依赖主循环，见 `xcore/flash_server.hpp` |
| ESC PDI（core1） | **ISR** | 4 | |
| CAN TX 完成 | **不启用中断** | — | 发完无事可做，启用只是白加中断 |
| DMA | **数据路径未用** | — | `dma_mgr_init()` 调了但没接数据面；MCAN 确实是 DMAMUX 源（`HPM_DMA_SRC_MCAN0..5`） |

主循环周期实测 0.72us（空闲）/ 0.85us（满载），"等下一趟主循环"最多值 0.85us；对照
RTT p50 124.8us，板端整条路径不到 3%。完整论证与"哪些板端优化因此不值得做"见
[../../HOST_TUNING.md](../../HOST_TUNING.md) 第 8 节。

## 板上没有调试器时怎么看现场

本板的调试口（FT2232：串口 + JTAG）**在实际使用的板子上只留了通孔焊盘，没有插座，
不是对外接口**——它是调试预留，不要当数据/日志通道用（详见
[boards/hpm6e8y/README.md](boards/hpm6e8y/README.md)「串口：只有调试通孔，不是接口」）。
本机 OpenOCD 状态见根 `AGENTS.md` 环境节。带内诊断通道已入库，都走 USB vendor 端点、
编成 UART0 上行帧：

| 想看什么 | 固件开关 | 主机工具 |
|---|---|---|
| CAN 转发是否在走：ISR 进入计数、MCAN `IR`/`RXF0S`/`PSR`/`ECR`、PLIC pending/enable/trigger | `-Dlibhcs_CAN_DIAG=ON` | `host/examples/can_stall_probe.cpp`（边压测边解码，转发停摆时打印前后快照） |
| 主循环周期（板端 CPU 余量的直接读数） | `-Dlibhcs_CAN_DIAG=ON` | `host/examples/hpm5321_loop_probe.cpp` |
| USB SOF 时间轴是否可信：相邻 FRINDEX 差值直方图、ISR 间隔、端口状态、跨板一致性 | `-Dlibhcs_SOF_DIAG=ON` | `host/examples/sof_probe.cpp`（见 [SOF_TIMEBASE.md](SOF_TIMEBASE.md)） |
| 跨板共享时间轴本身：各板状态、拟合出的晶振偏差、绝对微帧是否一致、到 Unix 时间的映射 | `-Dlibhcs_TIME_SYNC=ON` | `host/examples/time_sync_test.cpp`（主机侧还要 `AdvancedOptions::set_enable_time_sync(true)`） |
| 跨板 skew 直接实测：两块板在同一微帧各发一个硬件脉冲，互相硬件捕获 | `-Dlibhcs_PULSE_TEST=ON`（**要和 `TIME_SYNC` 一起开**） | `host/examples/pulse_skew_test.cpp`（见 [SOF_TIMEBASE.md](SOF_TIMEBASE.md) 5.5 / 7.1） |

> `can_stall_probe` **在 HPM5321 上跑不了**：它绑死 HPM6E8Y 的 PID `0x6E84`，并按
> 单板自环驱动 CAN0->CAN1 与 CAN2->CAN3，假设四路总线。5321 只有两路，双板 rig 又是
> 交叉对接而非自环。`hpm5321_loop_probe` 补的就是这个缺口——只解码遥测里的主循环
> 计数，负载自带（两块板双向对发），因为压测工具会独占两块板，遥测读端再也开不进去。

诊断开关的使用纪律（`CAN_DIAG`/`SOF_DIAG` 共占 `kUart0` 不得同开、`CAN_DIAG` 会让
`dual_board_test uart` 必然 FAIL、A/B 前先对齐 CMakeCache、`PULSE_TEST` 借走 UART0
引脚且外设初始化刻意推迟）见 [PITFALLS.md](PITFALLS.md) 第 8 节。

## 构建
```bash
export PATH=~/3rd_party/hpm/bin:$PATH        # [前机路径] 确保 riscv32-unknown-elf-gcc 可见
# USB 数据固件（超级构建，含 app + bootloader）
cmake --preset debug -S firmware/hpm_board
cmake --build firmware/hpm_board/build       # target: hpm_board_app / hpm_board_bootloader
# 两块 HPM5321 板（单 CAN / 双 CAN-FD）共用 -DBOARD=hpm5321 这一个镜像，上电自己判板型
cmake --preset release -S firmware/hpm_board -B <build> -DBOARD=hpm5321
# 烧录：同一个 .dfu，只有 -d 的 PID 按板子填（单 CAN 5321 / 双 CAN-FD 5322）
# dfu-util -d 0xa511:0x5322 -a 0 -D <build>/app/output/hpm_board_app_hpm5321.dfu
```
- preset：`debug` / `debug-outside` / `release`（注意本板 `CMAKE_BUILD_TYPE` 用小写 `debug`/`release`）。
- 构建需 Python 3 + `PyYAML`、`jinja2`（HPM SDK 代码生成用）。
- 本机工具链的实际位置以根 `AGENTS.md`「开发机环境路径约定」/ `ENV.md` 为准。

## 目录结构
- `app/`、`bootloader/`：USB 固件两半。`app/src/xcore/`：core0 的跨核部分（环主机侧、
  次核装载、flash RPC 服务端），默认单核构建下整体编译为空——见本文末「备注」。
- `boards/`、`common/`：板级配置与共享代码。`bsp/hpm_sdk`：HPM SDK submodule（第三方，只读）。
- 无 CubeMX，不适用 CubeMX 纪律。

## 备注
- 缺工具链的机器（如 build server）只能编 host SDK；固件需在装了 RISC-V 工具链的 PC 上编/烧。详见 `BUILD_ENVIRONMENT.md`。
- **遗留**：`app/src/xcore/` 与 `libhcs_APP_RELEASE_CORE1` 是 EtherCAT-on-core1 的
  残留，默认 OFF 时全部编译为空，不进镜像。EtherCAT 已于 2026-09-08 移出仓库
  （归档 `~/Desktop/ethercat-archive-2026-09-08/`），这套开关待单独清理。
