# mc02 固件指南

> **文档类型**：现行规范（板级）
> **适用范围**：`firmware/mc02/`，DM-MC02 / CtrBoard-H7（STM32H723VGT6）
> **状态**：现行有效
> **相关文档**：[仓库根 AGENTS.md](../../AGENTS.md) · [本目录 README.md](README.md)（外设与低延迟设计） · [UART_RING_LOG.md](UART_RING_LOG.md)（UART 实录与实测） · [仓库根 README.md](../../README.md)（烧录流程）

> 本目录专属指南，叠加在仓库根 `AGENTS.md` 之上。深入的外设/时钟/低延迟设计见本目录 `README.md`，此处只列 agent 关键点。

## 摘要

mc02 是 Cortex-M7 @ 550 MHz 的高性能板，特点是 **CAN-FD 常驻 FD+BRS** 与 **热路径代码
放 `.itcm`**；USB 受封装限制只能跑 Full-Speed，**瓶颈在 USB 还是在 CAN 取决于开几条
总线**（分界线见下方「关键特性」）。改这块板之前必须知道两件事：外设配置回 CubeMX 改，以及**每次 CubeMX 重新 Generate 之后要手工
复原一批改动**（清单在 [README.md](README.md)）。

## 芯片与工具链
- MCU：**STM32H723VGT6**（DM-MC02 / CtrBoard-H7），Cortex-M7 @ 550 MHz，LQFP100。
- ISA/工具链：ARM，`cmake/gcc-arm-none-eabi.cmake`，需 `arm-none-eabi-gcc`。

## 构建
```bash
cmake --preset debug -S firmware/mc02
cmake --build firmware/mc02/build --target mc02_app mc02_bootloader
```
- preset：`debug` / `release`。target：`mc02_app`、`mc02_bootloader`。

### build 目录已存在时，`option()` 的默认值不生效 [实测 2026-08-12]

`cmake --preset debug` 在**已有** `firmware/mc02/build/` 上运行时**沿用旧 cache**，
`app/CMakeLists.txt` 里 `option(... ON)` 的默认值不会被应用。曾经因此整整一轮测试都跑在
一个自以为是 ON、实际是 OFF 的配置上，连带做出错误结论。

改开关必须显式传：

```bash
cmake -Dlibhcs_APP_IMU_ENABLE=ON firmware/mc02/build && cmake --build firmware/mc02/build --target mc02_app
```

核对当前生效值：

```bash
grep libhcs_APP firmware/mc02/build/CMakeCache.txt
arm-none-eabi-nm firmware/mc02/build/app/mc02_app.elf | grep -c bmi088   # IMU: OFF=3, ON=57
```

## 编译开关

全部定义在 `app/CMakeLists.txt`，默认值即下表。**带 * 的三个都占用 `DataId::kUart0`，
互斥。** kUart0 不是这块板的丝印 UART；丝印口是 UART1 / UART2 / UART3 / UART7 / UART10 加 DBUS
（UART2 / UART3 就是 USART2 / USART3 的 RS-485，默认开，可用 `libhcs_APP_RS485_ENABLE=OFF` 整段关掉）。

| 开关 | 默认 | 作用 |
|---|---|---|
| `libhcs_APP_IMU_ENABLE` | ON | BMI088 初始化与采样 |
| `libhcs_APP_RS485_ENABLE` | ON | USART2 / USART3（丝印 UART2 / UART3）初始化、D2 对象与主循环 poll。OFF 时不调 `MX_USARTx_UART_Init`，省约 1.8 KB D2 SRAM 和每圈两次 NDTR 读。DataId 仍是 `kUart2` / `kUart3` |
| `libhcs_APP_USB_RX_XFER_SIZE` | 1024 | 一次 bulk OUT 传输请求的字节数（64 的倍数）。1024 是实测拐点，比旧的 64 高 59% |
| `libhcs_APP_LOOP_BALLAST_CYCLES` | 0 | 每圈主循环插入的纯忙等周期数，**只是测量仪器**，出厂镜像永远是 0 |
| `libhcs_APP_USB_RX_HIST` * | OFF | 相邻两次 bulk OUT 完成的间隔直方图（DWT）在 kUart0 上输出 |
| `libhcs_APP_LOOP_PROFILE` * | OFF | 主循环分段耗时（DWT）在 kUart0 上以 ASCII 输出 |
| `libhcs_APP_CAN_DIAG` * | OFF | CAN 遥测记录在 kUart0 上输出 |

下面实测段里写「两臂都是 `libhcs_APP_RS485_ENABLE=ON`」的，就是今天默认镜像的条件。
关掉时 DataId 不变——诊断仍走 `kUart0`，RS-485 关了只是固件不认 `kUart2` / `kUart3`。

### DWC2 内部 DMA：测过更慢，开关已删除 [实测 2026-08-28]

**结论：小包包率降约 3%，大包吞吐一点没动，板上主循环省下的时间可以忽略。没有任何一档
测下来是赢的，所以 `libhcs_APP_USB_DWC2_DMA` 开关连同 `tusb_config.h` 里的
`CFG_TUD_DWC2_DMA_ENABLE` / `CFG_DWC2_MEM_UNCACHED_REGIONS` 一并删掉了，slave 模式是
唯一支持的配置**（删除前后默认镜像 bit 级相同，已 `cmp` 验证）。下面这份记录留着，
是为了别人不用再走一遍——**要重新加回这个开关，先把下面的测量重做一遍。**

A/B 方式：同一块板，`release` 预设，交替 DFU 烧录两个镜像，每档 3 次 5 s，
工具 `host/examples/mc02_packet_rate.cpp`（下行泛洪，记录发给没接线的 UART1，
固件把多余的丢掉，测的就是 USB + 反序列化这条路）。主机未跑 `host-tuning.sh`
（governor 仍是 powersave），两臂条件相同。

| 档位（秒 / 载荷 / 每包记录数） | OFF | ON | 差 |
|---|---|---|---|
| `5 8 1`（一包一条 8 B 记录） | 26650 / 26604 / 26628，第二轮 26623 / 26631 / 26563 包/s | 25934 / 25904 / 25912，第二轮 25808 / 25818 / 25921 包/s | **-2.9%** |
| `5 64 4`（约 300 B 传输） | 884.5 KB/s | 882.9 KB/s | -0.2% |
| `5 256 8`（约 2 KB 传输） | 1010.0 KB/s | 1009.0 KB/s | 0（都撞在 Full-Speed 线速上） |

为什么省下的拷贝换不回吞吐（**省的是 CPU 周期，付的是墙钟延迟，卡住的是后者**）：

- **拷贝本来就不在瓶颈上。** `libhcs_APP_LOOP_PROFILE=ON` 再测一轮，主循环
  avg 从 4782 降到 4737 cycles（550 MHz 下 -0.08 us/pass），`usb` 段 821 -> 805 cycles。
  可是泛洪时主循环跑到 114-125 kHz，而包只有 25 kHz——**每来一个包主循环空转四五圈**，
  省 45 cycles 换不到任何东西。**CPU 更闲了包率却更低**，这本身就说明瓶颈不在 CPU。
- **真正卡住的是"包到 -> 端点重新 arm"这段墙钟时间**，因为主机是背靠背发的。用
  `libhcs_APP_USB_RX_HIST=ON` 量相邻两次 OUT 完成的间隔（板上 DWT，采样点在
  `tud_vendor_rx_cb`），两臂各约 12900 样本/500 ms：

  | 间隔档 | slave（OFF） | DMA（ON，按样本数归一） |
  |---|---|---|
  | <20 us（同一串背靠背） | 2081 | **1830** |
  | 20-30 us | 3739 | 3739 |
  | 34-38 us | 743 | 803 |
  | 55-70 us | 3298 | 3443 |
  | >=90 us（尾部） | 223 | 224 |

  **少掉的全在 <20 us 那一档，尾部一个没多。** 所以不是"偶尔漏掉一整个 125 us 微帧"，
  而是**每串背靠背里少接住一个包**：DWC2 内部 DMA 要先把包从 RX FIFO 经 AHB 搬进
  AXI SRAM 才拉 `XFRC`，比 slave 模式"包一进 RX FIFO 就 `RXFLVL`、CPU 立刻开搬"晚，
  重新 arm 还多写一个 `doepdma`。晚这一点，就赶不上主机约 17 us 后的下一个 OUT 令牌。
  `[实测 2026-08-28，间隔直方图]`
- 缓存维护不是原因：TinyUSB 的传输缓冲区（`_vendord_epbuf` 0x240009a0、`_dcd_usbbuf`、
  `_ctrl_epbuf`）全在 AXI SRAM，且已由 `CFG_DWC2_MEM_UNCACHED_REGIONS` 声明为非缓存
  （`tusb_config.h`），DMA 臂上没有额外的 clean/invalidate。
- **这 3% 不是相位假象。** 见下一节：包率对主循环周期极其敏感（±35%），只在一个工作点
  上比较不可信。在**两个**相差 35% 的工作点上各测三次，DMA 的差都是 -3.0%：

  | | ballast 0 | ballast 825 cycles（1.5 us） |
  |---|---|---|
  | slave | 26637 / 26766 / 26736 | 36108 / 36050 / 36075 |
  | DMA | 25934 / 25903 / 25933 | 34918 / 35093 / 34987 |
  | 差 | -3.0% | -3.0% |

当时 DMA 确实生效了，不是没开：`CFG_TUD_DWC2_DMA_ENABLE=1` 会让
`CFG_TUD_DWC2_SLAVE_ENABLE` 默认变成 0（`tusb_option.h`），FIFO 拷贝那条路径整个编不进去
——`dcd_int_handler` 从 0x990 字节缩到 0x788——而板子照常枚举、照常跑 1 MB/s，
说明走的就是 DMA 路径。

**和 5321 没有可比性**：`hpm_board` 的 HPM5321 用的是 `dcd_hpm.c`（ChipIdea/EHCI 式的
QHD/QTD 描述符控制器），本来就只有 DMA 一种工作方式，根本没有"FIFO 拷贝模式"可省，
也没有对应的编译开关。这个开关是 Synopsys DWC2 独有的，只对 `c_board` / `mc02` 有意义。

**但 5321 那边量出来的斜率可以用来验算这 3%**：[hpm_board/AGENTS.md](../hpm_board/AGENTS.md)
「因果实验：`cycle = turnaround x 1.2~1.4 + 约 13.5 us`」实测**每包周期对 turnaround
的斜率是 1.2~1.4**。把这条套到 mc02：DMA 让 turnaround 长约 1 us，预期包率损失
1 x 1.3 / 37.6 us ≈ **3.5%**，实测 3.0%。两块板、两种控制器、独立测出来的两个数对得上，
"DMA 的代价落在 turnaround 上"这个解释因此不只是本板的自圆其说。`[推断，基于两项实测]`

### RS-485 收环改到 IDLE ISR 里排空：测过更差，开关已删除 [实测 2026-08-31]

**结论：`libhcs_APP_UART_RX_IN_ISR` 确实省下了它承诺的东西——两个 RS-485 口每趟主循环
的 NDTR 读，主循环 5410 -> 5133 cycles/圈（-277 cycles，0.50 us），101 -> 107 kHz——但
这些时间本来就是富余的，换不到任何包率；而 200 字节的 RS-485 往返因此慢了 26%。开关连同
`UartRs485::try_transmit()` / `rx_event_callback()` 里的两处 `#if` 一并删掉，主循环排空
是唯一支持的配置**（删除前后 `.text` 只差 8 字节，是 `.dirty` 版本串本身，`App::run` 大小
和地址都不变，已核对）。**要重新加回这个开关，先把下面的测量重做一遍。**

测法：两块 mc02，USART2 交叉接 A-A / B-B（`HCS_RS485_PORT=1`），`release` 预设，
两臂都是 `libhcs_APP_RS485_ENABLE=ON`，交替 DFU 烧录。

**1. 正确性：两臂都满分。** `host/examples/rs485_cross_test` 全套（尺寸 1~200 B、
波特率扫到 4800000、半双工换向、200 轮 ping-pong、冲突恢复、静默总线、通道隔离）
两臂都是 `ALL PASSED (0 failures)`。ISR 排空不是错的，只是不划算。

**2. 包率：ISR 臂反而低，而且这个比较本身不可信。** `mc02_packet_rate 5 8 1`，
每臂三次，三种镜像交替烧录跑四轮（**交替**而不是分块，因为整台机器几十分钟里会漂
约 1%）：

| 轮次 | RS-485 关 | RS-485 开 + 主循环排空 | RS-485 开 + ISR 排空 |
|---|---|---|---|
| 1 | 26511 | 26942 | 26378 |
| 2 | 26907 | 27065 | 26359 |
| 3 | 26940 | 26973 | 26342 |
| 4 | 26551 | 27261 | 26200 |

顺序稳定得可疑：ISR 臂每一轮都最低（-2.7%）。但这是相位，不是代价——把
`libhcs_APP_LOOP_BALLAST_CYCLES` 当相位旋钮扫一遍，名次直接翻过来：

| ballast | 0 | 275 | 550 | 825 |
|---|---|---|---|---|
| 主循环排空 | 27060 | 26807 | 26230 | 26994 |
| ISR 排空 | 26320 | 26747 | **27844** | 27136 |

550 那一档 ISR 臂反超 6.2%。四点平均 26773 vs 27012，差别整个埋在摆幅里。
**结论是"包率无法区分这两臂"，不是"ISR 排空慢 2.7%"**——见上一节，本板的包率是
主循环周期的强非单调函数，而 ISR 排空恰恰改的就是主循环周期。

**3. 主循环：省下的 277 cycles 是真的，也是真的没人要。** `libhcs_APP_LOOP_PROFILE=ON`
（当时 RS-485 与诊断共用 kUart0；现在 RS-485 是 kUart2/kUart3，诊断独占 kUart0）：

| | 每圈 cycles | 其中 uart 段 | 主循环 | 该臂包率 |
|---|---|---|---|---|
| 主循环排空 | 5410 | 2059（38.0%） | 101 kHz | 23955 / 24068 |
| ISR 排空 | 5133 | 1773（34.5%） | 107 kHz | 24512 / 24634 |

两个口的 NDTR 读合计约 277 cycles（每个口约 0.25 us，D2 外设读的典型代价）。
但泛洪时包只有约 24 kHz 而主循环 101 kHz——**每来一个包主循环已经空转四圈**，
省下的第五圈没有买主。（注意这一档里 ISR 臂又"快"2.4%：插桩本身改了主循环周期，
于是相位又换了一个位置。同一个改动在三种插桩下给出 -2.7% / 0 / +2.4%，这就是为什么
单点包率不能用来判决。）

**4. 判决性证据是延迟，不是包率。** `rs485_cross_test latency`（4800000 baud，p50，
每臂三次）：

| | 1 B | 200 B |
|---|---|---|
| 主循环排空 | 182 / 196 / 202 us | 1521 / 1525 / 1532 us |
| ISR 排空 | 165 / 171 / 171 us | **1925 / 1932 / 1992 us** |

小包快约 25 us（少等一趟主循环），**大包慢约 400 us（+26%）**。机制是确定的：
`RxBuffer::try_dequeue()` 每攒够 `kMinFragmentSize`（32 B）就发一块上行，**不等线路空闲**，
所以主循环排空时报文还在收、前几块已经在 USB 上走了；改到 IDLE ISR 之后，
整条报文只有在线路空闲后才一次性上行，收包与上行的流水线没了。400 us 与 200 B 在
4.8 Mbaud 上的线时（417 us）同量级。这条总线上真实的应答是 78 字节，早就越过 32 B
门限，**吃的就是这条**。

顺带说一条与旧说法的出入：`uart.hpp` 曾写"两个 RS-485 口的每趟 NDTR 读吃掉 2.09% 的
USB 包率"（2026-08-25，那次是用一个"照常 `init()`、但主循环不 pump"的等布局镜像单独
隔出来的，设计比本轮严）。**本轮的整开关 ON/OFF 对比给出了相反的方向**：上表第一列
vs 第二列，开 RS-485 反而稳定高约 1.5%，四轮无一例外。两者不是同一个实验——本轮换掉的
不止是轮询，还有镜像布局和主循环周期，因而同样吃相位。**能同时成立的读法只有一个：
每趟 0.5 us / 277 cycles 的代价是真的（第 3 节直接量到），但"这 2% 包率"在今天的工作点
上重现不出来，任何拿它做前提的优化都得先重测。** 要判决那 2.09%，得重跑
2026-08-25 那个等布局的 pump / 不 pump 对比，本轮没有重跑。`[实测有，与旧结论冲突未合并]`

### 下行包率是主循环周期的强非单调函数，摆幅 ±35% [实测 2026-08-28]

**结论：给主循环每圈加 1.5 us 纯忙等，小包下行包率从 26700 涨到 36100（+35%）；加 3 us
反而掉回 26000，加 6 us 掉到 23900。这不是"CPU 太忙"——加的是纯浪费的忙等。任何
mc02 的 USB 性能 A/B，如果只在一个工作点上测，都可能测的是相位而不是改动本身。**

`libhcs_APP_LOOP_BALLAST_CYCLES=N` 扫描，`mc02_packet_rate 4 8 1`，各三次取中位，
IMU=ON、无诊断开关（DMA=OFF 臂）：

| ballast | 0 | 825（1.5 us） | 1650（3 us） | 2200（4 us） | 3300（6 us） | 4400（8 us） |
|---|---|---|---|---|---|---|
| 包/s | 26700 | **36100** | 26100 | 25500 | 23900 | 27300 |

为什么会这样：整条链路是**闭环**——板子 arm 端点、主机才发下一个包、包到了板子再
arm。所以"包到 -> 重新 arm"的相位是自锁的，而 arm 只发生在 `tud_task()` 里，
即每个主循环周期一次。主循环周期与主机的事务节奏（微帧 125 us，串内背靠背约 17 us）
形成拍频：**相位好时一串能接住两个包，相位差时只接住一个**。直方图看得很清楚——
1.5 us ballast 那一臂 `<20 us` 档从 2081 涨到 5167，尾部（>=90 us）完全没动。

**这不是可以直接拿来用的优化**，两条理由：

- **最优点跟着主循环的组成变**。把 IMU 关掉（每圈少一次 BMI088 SPI 服务），同样的
  1.5 us ballast 从 +35% 变成 -7%：IMU=OFF 时 ballast 0 是 29100、ballast 825 是 27100。
  也就是说甜点位置由整圈的时序决定，任何一处改动都会挪走它。
- 它只对"主机背靠背泛洪小包"这一种流量成立。控制回路那种 1 kHz 的往返上面没测过。

**要用的话必须重测**，而且要连着扫几个 ballast 值确认自己不在悬崖边上。测量工具是
`libhcs_APP_USB_RX_HIST=ON`（间隔直方图）加 `libhcs_APP_LOOP_BALLAST_CYCLES`（相位旋钮）。

**5321 上没有这个现象，别照搬。** [hpm_board/AGENTS.md](../hpm_board/AGENTS.md)
2026-08-07 做过同一类实验（在 `rx_cb` 里 re-arm 之前注入忙等），六个点**单调下降**、
没有任何甜点：0 -> 54.3k、600 cy -> 52.5k、1200 cy -> 47.1k、1800 cy -> 40.8k。
两处的注入点不完全相同（5321 注在 `rx_cb` 里，本板注在主循环开头，因而还改了
`tud_task()` 的相位），但结论方向是清楚的：**High-Speed 的 5321 每包周期由约 13.5 us
的主机侧地板 + 1.2~1.4 倍斜率决定，是单调的；本板这个甜点是 Full-Speed 下主循环周期与
主机事务节奏打拍出来的，属于 mc02 特有。**

本板这个峰**还没有被完整解释**。已知的线索是它跟上行有关：关掉 IMU（上行几乎没数据了）
ballast 0 反而涨到 29100，说明上行 IN 事务本身在跟下行抢 Full-Speed 总线；而加 ballast
会把上行攒成更少更大的包。但这解释不了全部 35%。`[实测有，机制未定论]`

## 目录结构
- `app/`、`bootloader/`：两套独立镜像。`app/src/app.cpp` 提供自己的 `main()`，直接驱动生成的 `*_Config()` / `MX_*_Init()`。
- `bsp/cubemx/`：CubeMX 生成产物。`bsp/linker/`：手维护链接脚本（如 `STM32H723VGTx_APP.ld`，含 `.itcm` 热路径段）。
- `bsp/`：`cmsis-device-h7`、`stm32h7xx-hal-driver` 等第三方，视为只读。

## 关键特性（改代码前须知）
- USB：OTG_HS 跑 **Full-Speed**（LQFP100 无 HS PHY 引出）。**"瓶颈是 USB 还是 CAN"没有
  统一答案，按跑满的总线条数分界**：上行每帧 15 B（`1 + 3 - 1 + 8 + 4`，见
  [HOST_TUNING.md](../../HOST_TUNING.md) 9.3），CAN-FD 每条总线上限 19870 帧/s，而本板
  USB 聚合上限约 800 KB/s。于是 **1-2 条总线跑满时卡在 CAN 线速**（298 / 596 KB/s），
  **3 条一起跑满时才卡在 USB**（894 KB/s > 800）。分界点约 2.7 条总线，即聚合 53000 帧/s。
  `[推断，基于 800 KB/s 与 19870 帧/s 两项实测]`
- CAN：FDCAN1/2/3 常驻 FD+BRS，逐帧按 host `is_fdcan` 切换，不做 INIT 重配。
- **下行 CAN 帧直写硬件 FIFO，只有 FIFO 满了才进队列** [2026-08-24 修复]。
  `handle_downlink` 由 `tud_vendor_rx_cb` 在 `tud_task()` 里调用，与 `try_transmit()`
  同线程，所以直写是安全的（队列非空时必须让路，否则会插队）。
  **改之前是无条件入队**，于是每一帧都要等到主循环末尾的 `canN->try_transmit()` 才
  进硬件，中间隔着 DFU poll、GPIO 采样、一次 BMI088 SPI 读和 LED poll；同时那个
  16 深的环（继承自 c_board，那块板 bxCAN 只有 3 个发送邮箱，16 是净赚）架在 32 条
  FDCAN FIFO **前面**，把单包突发上限从 32 砍到了 16。队列深度现在是 64
  （`kTransmitQueueSize`，每路 1 KB DTCM），与 hpm_board 一致。
  **实测效果**（交替烧录 A/B，三轮各 4000 帧，已跑 `host-tuning.sh`；
  5321 -> mc02 方向作对照组，三轮 p50 131.2/131.1/131.1 -> 131.6/131.0/131.4，确认未动）：

  | mc02 -> 5321 | 改前（三轮） | 改后（三轮） |
  |---|---|---|
  | CAN-FD min | 94.6 / 95.0 / 94.7 us | **90.9 / 92.9 / 91.8 us** |
  | CAN-FD p50 | 124.8 / 124.7 / 124.7 us | **123.6 / 123.7 / 123.7 us** |
  | CAN-FD avg | 125.4 / 125.9 / 125.3 us | **121.7 / 122.8 / 122.5 us** |
  | classic p50 | 180.5 / 180.5 / 180.5 us | **179.7 / 179.8 / 179.5 us** |
  | classic p90 | 209.9 / 207.6 / 209.4 us | **206.7 / 206.7 / 206.2 us** |
  | 单包突发 17/24/32/40/64 帧 | 丢 5.9/33/50/60/75% | **全部 0%** |

  **改后 avg 落到 p50 之下**（122.3 vs 123.7；改前是正常右偏的 125.4 vs 124.7）：分布
  变成双峰，一部分帧真的走了直通路径（min 掉到 91 us），把均值拉到中位数以下。这也是
  为什么均值改善 3.2 us 而中位数只有 1.1 us。

  **p99 / max 没有可重现的改善**，且调优后 max 在**所有臂包括对照组**仍是 630-950 us。
  那是主机侧的：`mixed_board_test` 经 `multi_board.hpp` 构造 session，**没有传
  `thread_setup`，事件线程没绑核**，而这是 [HOST_TUNING.md](../../HOST_TUNING.md) 1.3
  记的尾部最差一档。**要评估板级抖动，得先给测量工具加上绑核能力**，否则测的是主机调度。
  `[实测 2026-08-24，mc02 <-> 5321，已调优主机、事件线程未绑核]`
- **未做**：hpm_board 那套下行流控（`transmit_queue_depth()` 决定是否再 arm OUT 包）
  没有移植。所以队列真被打满时，mc02 仍然是静默丢弃 + 点 LED，主机无感——
  `diag::note_tx_fail()` 在默认构建下是空实现（`libhcs_APP_CAN_DIAG` 默认 OFF）。
- 热路径 `Can::handle_uplink/handle_downlink/try_transmit` 等放 `.itcm`，启动时从 FLASH 拷入。
- UART：六个口（USART1 / USART2 / USART3 / UART7 / USART10 / UART5-DBUS）各一条**永不停的整环 circular DMA**，写指针由主循环读 `NDTR` 推导，不由中断维护。端口对象（含 DMA 环）放 `.d2_sram`，启动时从 FLASH 拷入，MPU region 1 在 `app.cpp` 里设为非缓存。**不要给 UART 的 DMA 开 FIFO/burst**——`NDTR` 只统计到 DMA FIFO，写指针会算错。DataId 就是丝印号：UART1/2/3/7/10 加 DBUS。USART2 / USART3 受 `libhcs_APP_RS485_ENABLE` 控制（默认 ON）。
- UART 错误策略：`CR3.OVRDIS=1`、**`CR3.DDRE=0`**、`CR3.EIE=0`。`DDRE` 是"出错时禁用 DMA"，**置 1 会让一个坏字符永久杀死端口**——细节见 [UART_RING_LOG.md](UART_RING_LOG.md) 第 1 章。
- 实测吞吐天花板约 **800 KB/s 聚合**（USB Full-Speed 决定），781 KB/s 时零丢失；主循环在满过载下仍有约 10 倍余量。

## 不要用 HAL_RCCEx_GetPeriphCLKFreq() 取 UART 内核时钟 [实测 2026-08-05]

**结论先行：本版 HAL 的 `HAL_RCCEx_GetPeriphCLKFreq()` 对两个 UART 组都返回 0。**
它的 if/else 链只覆盖 SAI / SPI / ADC / SDMMC / SPI6 / FDCAN，
`RCC_PERIPHCLK_USART16910` 和 `RCC_PERIPHCLK_USART234578` **一个分支都没有**，
直接掉到末尾 `else { frequency = 0; }`（`bsp/stm32h7xx-hal-driver/Src/stm32h7xx_hal_rcc_ex.c`）。
那两个宏本身是存在的，所以编译期没有任何提示。

后果：`uart.hpp` 的 `handle_config()` 拿到 0 之后撞上自己的
`kernel_clock_hz == 0` 提前返回，**`BRR` 一次都没写过**，主机发来的运行时波特率
请求被静默忽略，端口永远停在 CubeMX 的 115200。

**正确做法**（已改成这样）：用 HAL 自己的 `UART_GETCLOCKSOURCE(handle, src)` 宏——
它按外设实例分派，正是 `UART_SetConfig()` 在 init 时算 `BRR` 用的同一个宏——
再按 `UART_CLOCKSOURCE_*` 取 `HAL_RCC_GetPCLK1Freq()` / `PCLK2` / HSI（**要按
`RCC_FLAG_HSIDIV` 右移**）/ CSI / LSE / PLL2Q / PLL3Q。不要自己手写"哪个口属于
哪个时钟组"的判断，那是在重复这个宏已经做对的事。

### 为什么 c_board 两行就够、这块板不行

c_board 同样的功能只用两行（`HAL_RCC_GetPCLKxFreq()`，见
`firmware/c_board/app/src/uart/uart.hpp` 的 `peripheral_clock_hz()`），**那在 F407
上是对的**：STM32F407 的 UART 直接挂在自己的 APB 总线上，总线频率就是内核时钟，
连 `Init.ClockPrescaler` 字段都不存在。STM32H7 在中间插了**每组可选时钟源 + 预分频器**，
所以必须先解析时钟源，频率才有意义。**别看着 c_board 简单就照抄过来。**

**为什么很难发现**：`UART7 <-> UART10` 自环测试在 115200 到 2000000 **全部 PASS**。
自环把两端同时设成目标值，配置没生效时两端**一起**留在 115200，于是波特率一致、
通信正常、测试通过。**自环只能验证两端一致，不能验证两端等于你要的值。**
要验证切换真的生效，必须**一端切、另一端不切**并确认通信变坏，或者跨板对打。
另见 [hpm_board/AGENTS.md](../hpm_board/AGENTS.md) 里同一次排查的 5321 侧 DLAB 坑。

## CubeMX 纪律（本板适用）
- 配置改在 CubeMX（`.ioc`），人工 Generate；**禁止**直接改 `bsp/cubemx/Core/` 生成代码。
- **每次 CubeMX “Generate Code” 后需手工复原**（会被覆盖），清单见 `README.md` 末节，例如删除 `Core/Src/main.c` 里重新生成的 `int main(void)`、把 `static void MPU_Config` 改回 `void MPU_Config`。
