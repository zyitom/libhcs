# mc02 固件指南

> **文档类型**：现行规范（板级）
> **适用范围**：`firmware/mc02/`，DM-MC02 / CtrBoard-H7（STM32H723VGT6）
> **状态**：现行有效
> **相关文档**：[仓库根 AGENTS.md](../../AGENTS.md) · [本目录 README.md](README.md)（外设与低延迟设计） · [UART_RING_LOG.md](UART_RING_LOG.md)（UART 实录与实测） · [PACKET_RATE_LOG.md](PACKET_RATE_LOG.md)（USB 包率实测与已删开关） · [仓库根 README.md](../../README.md)（烧录流程）

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
| `libhcs_APP_DEBUG_KNOBS` | OFF | 暴露调试器可写的调优旋钮（`diag/knobs`），配合 Ozone / J-Link 在线改参 |
| `libhcs_APP_TIME_SYNC` | OFF | 运行共享 USB-SOF 时间基准做跨板对时，机制与实测见 [../hpm_board/SOF_TIMEBASE.md](../hpm_board/SOF_TIMEBASE.md) |
| `libhcs_APP_USB_RX_HIST` * | OFF | 相邻两次 bulk OUT 完成的间隔直方图（DWT）在 kUart0 上输出 |
| `libhcs_APP_LOOP_PROFILE` * | OFF | 主循环分段耗时（DWT）在 kUart0 上以 ASCII 输出 |
| `libhcs_APP_CAN_DIAG` * | OFF | CAN 遥测记录在 kUart0 上输出 |

**已删除、不要重新加回的开关**（都经过完整实测，判决与数据见
[PACKET_RATE_LOG.md](PACKET_RATE_LOG.md)，要加回必须先重做那里的测量）：

- ~~`libhcs_APP_USB_DWC2_DMA`~~（2026-08-28）：小包包率 −3%、大包无收益，slave 模式是唯一支持的配置。
- ~~`libhcs_APP_UART_RX_IN_ISR`~~（2026-08-31）：主循环省 0.5 us 但换不到包率，200 字节 RS-485 往返慢 26%。

本板包率对主循环周期**强非单调**（±35% 相位摆幅），任何 USB 性能 A/B 只在一个工作点
测都不可信——判据与工具见 [PACKET_RATE_LOG.md](PACKET_RATE_LOG.md) 第 1 节。

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
- **CAN 的协议与速率由电机硬件决定，不是可调参数**：仲裁段 1 Mbit/s / 数据段
  5 Mbit/s 的上限、能否上 FD，都由总线对端电机固件决定，本仓库**无法修改**
  `[硬件事实，用户确认 2026-09-12]`。吞吐/延迟优化**不要**以「升级 CAN-FD /
  提高波特率 / 改采样点」为建议方向；可行杠杆在成帧、软件路径与主机侧（见上方
  「瓶颈」分析与 [PACKET_RATE_LOG.md](PACKET_RATE_LOG.md)）。
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

  **改后 avg 落到 p50 之下**（122.3 vs 123.7）：分布变成双峰，一部分帧真的走了直通路径
  （min 掉到 91 us），把均值拉到中位数以下。这也是为什么均值改善 3.2 us 而中位数只有
  1.1 us。

  **p99 / max 没有可重现的改善**，且调优后 max 在**所有臂包括对照组**仍是 630-950 us。
  那是主机侧的：`mixed_board_test` 经 `multi_board.hpp` 构造 session，**没有传
  `thread_setup`，事件线程没绑核**，而这是 [HOST_TUNING.md](../../HOST_TUNING.md) 1.3
  记的尾部最差一档。**要评估板级抖动，得先给测量工具加上绑核能力**，否则测的是主机调度。
  `[实测 2026-08-24，mc02 <-> 5321，已调优主机、事件线程未绑核]`
- **未做**：hpm_board 那套下行流控（`transmit_queue_depth()` 决定是否再 arm OUT 包）
  没有移植。所以队列真被打满时，mc02 仍然是静默丢弃 + 点 LED，主机无感——
  `diag::note_tx_fail()` 在默认构建下是空实现（`libhcs_APP_CAN_DIAG` 默认 OFF）。
- 热路径 `Can::handle_uplink/handle_downlink` 与排空发送队列的
  `drain_transmit_queue`/`drain_pending_transmits_slow` 等放 `.itcm`，启动时从 FLASH 拷入；
  `try_transmit()` 已改为头文件内联的空队列快测，主循环入口是 `drain_pending_transmits()`。
  `[代码核对 main 1022f3e，2026-09-12]`
- UART：六个口（USART1 / USART2 / USART3 / UART7 / USART10 / UART5-DBUS）各一条**永不停的整环 circular DMA**，写指针由主循环读 `NDTR` 推导，不由中断维护。端口对象（含 DMA 环）放 `.d2_sram`，启动时从 FLASH 拷入，MPU region 1 在 `app.cpp` 里设为非缓存。**不要给 UART 的 DMA 开 FIFO/burst**——`NDTR` 只统计到 DMA FIFO，写指针会算错。DataId 就是丝印号：UART1/2/3/7/10 加 DBUS。USART2 / USART3 受 `libhcs_APP_RS485_ENABLE` 控制（默认 ON）。
- UART 错误策略：`CR3.OVRDIS=1`、**`CR3.DDRE=0`**、`CR3.EIE=0`。`DDRE` 是"出错时禁用 DMA"，**置 1 会让一个坏字符永久杀死端口**——细节见 [UART_RING_LOG.md](UART_RING_LOG.md) 第 1 章。
- 实测吞吐天花板约 **800 KB/s 聚合**（USB Full-Speed 决定），781 KB/s 时零丢失；主循环在满过载下仍有约 10 倍余量。

## 不要用 HAL_RCCEx_GetPeriphCLKFreq() 取 UART 内核时钟 [实测 2026-08-05]

**结论先行：本版 HAL 的 `HAL_RCCEx_GetPeriphCLKFreq()` 对两个 UART 组都返回 0**
（if/else 链只覆盖 SAI / SPI / ADC / SDMMC / SPI6 / FDCAN，`RCC_PERIPHCLK_USART16910`
和 `RCC_PERIPHCLK_USART234578` 一个分支都没有；那两个宏本身存在，所以编译期没有任何
提示）。后果是 `handle_config()` 拿到 0 后提前返回，**`BRR` 一次都没写过**，运行时
波特率请求被静默忽略，端口永远停在 CubeMX 的 115200。

**正确做法**（已改成这样）：用 HAL 自己的 `UART_GETCLOCKSOURCE(handle, src)` 宏——
它按外设实例分派，正是 `UART_SetConfig()` 在 init 时算 `BRR` 用的同一个宏——
再按 `UART_CLOCKSOURCE_*` 取 `HAL_RCC_GetPCLK1Freq()` / `PCLK2` / HSI（**要按
`RCC_FLAG_HSIDIV` 右移**）/ CSI / LSE / PLL2Q / PLL3Q。不要自己手写"哪个口属于
哪个时钟组"的判断。**别看着 c_board 的两行实现（F407 上是对的）就照抄过来**——
H7 在中间插了每组可选时钟源 + 预分频器。

**为什么很难发现**：`UART7 <-> UART10` 自环测试在 115200 到 2000000 **全部 PASS**——
自环只能验证两端一致，不能验证两端等于你要的值（详见
[hpm_board PITFALLS.md 第 6 节](../hpm_board/PITFALLS.md)，同一次排查的 5321 侧
DLAB 坑也在那里）。

## CubeMX 纪律（本板适用）
- 配置改在 CubeMX（`.ioc`），人工 Generate；**禁止**直接改 `bsp/cubemx/Core/` 生成代码。
- **每次 CubeMX “Generate Code” 后需手工复原**（会被覆盖），清单见 `README.md` 末节，例如删除 `Core/Src/main.c` 里重新生成的 `int main(void)`、把 `static void MPU_Config` 改回 `void MPU_Config`。
