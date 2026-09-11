# hpm_board 踩坑与实测记录

> **文档类型**：过程记录
> **适用范围**：`firmware/hpm_board/`，HPMicro HPM6E8Y / HPM5321
> **状态**：现行有效（结论被推翻的不删除，原地标注"历史记录"）
> **相关文档**：[AGENTS.md](AGENTS.md)（现行命令与约束） · [USB_OPTIMIZATION_LOG.md](USB_OPTIMIZATION_LOG.md)（USB 调优实录） · [SOF_TIMEBASE.md](SOF_TIMEBASE.md) · [BUILD_ENVIRONMENT.md](BUILD_ENVIRONMENT.md)

## 摘要

本文收录 hpm_board 上板过程中的选型实测与踩坑记录：两块芯片的延迟对比、CAN 采样点
为什么必须钉 87.5%、UART 运行时改波特率的 DLAB 坑、EP0 配置通道落地过程中的坑，以及
几条测量方法论教训。**现行约束以 [AGENTS.md](AGENTS.md) 为准**，这里讲的是"为什么"和
"当时怎么发现的"。

## 本文导航

- 第 1-2 节：选型实测（5321 vs 6E8Y；多板方案与 EtherCAT 否决）
- 第 3 节：CAN 采样点（对齐总线，不是对齐推荐表）
- 第 4 节：UART DLAB 坑（遥测回读毁掉波特率）
- 第 5 节：EP0 落地坑（握手时序、旧主机蹭握手、CAN 状态判读表）
- 第 6 节：自环测试对共模错误是瞎的
- 第 7 节：主机侧板类合一的来龙去脉
- 第 8 节：诊断开关的使用坑

---

## 1. 选型实测：HPM5321 DualCan vs HPM6E8Y [实测 2026-08-01，各 3 轮]

两块板**刷同一份固件**（`g3ce3bf6`，含当天的 MCAN ISR 修复）、**同一个测量二进制**
（`bridge_can_loopback_latency`，新增 `usb-5321` 模式就是为了这个）、同一台主机、
同样 `chrt -f 80` 绑核 7/6、同样锁死深 C-state。都是 CAN0 -> CAN1 回环、CAN-FD 8 字节。

| | HPM5321 DualCan（单核） | HPM6E8Y 单核镜像 | HPM6E8Y 核对调镜像 |
|---|---|---|---|
| p50 | 99.8 / 99.8 / 99.9 | 99.8 / 99.9 / 99.8 | 124.5 / 124.4 / 124.6 |
| p90 | 104.5 / 104.3 / 104.8 | 104.7 / 107.3 / 104.4 | 126.2 / 126.2 / 126.5 |
| p99 | 125.4 / 125.0 / 125.7 | 125.5 / 125.8 / 125.6 | 159.2 / 154.4 / 159.9 |
| max | 131.8 / 140.2 / 160.7 | 129.5 / 128.5 / 142.0 | 197.2 / 172.9 / 198.5 |

**结论（顺序很重要，别只看第一行）：**

1. **5321 和 6E8Y 单核镜像在每一个分位上都一样**，差值全在噪声内（p50 都是 99.8，
   p99 都是约 125.5）。**这条链路的延迟不由芯片决定。**
2. **原因**：延迟由 USB HS 微帧（125us）+ CAN-FD 线上时间（约 30us）主导，这两项
   两块芯片完全相同。6E8Y 的 600 MHz 双核、200 MHz AHB（5321 是 160 MHz，由固件里
   `kCanTimestampNsPerUs` 960 vs 1000 反推）、4 路 CAN，在这个指标上**买不到任何东西**。
3. **5321 看起来比 6E8Y 快 25us，是拿它跟核对调镜像比出来的假象**——那 25us 是
   6E8Y 释放 core1 之后的双核争用代价，不是芯片差距。

**选型含义**：如果需求就是"USB 转几路 CAN"，**5321 DualCan 在延迟上和 6E8Y 打平**，
6E8Y 的价值在于 **4 路 CAN、双核**这两件功能，而不是更低的延迟。
为延迟去选 6E8Y 是选错了理由。

复现命令：

```bash
sudo ./host-tuning.sh --pmqos     # 另开终端
sudo chrt -f 80 ./host/build/examples/bridge_can_loopback_latency usb-5321 3000 7 6
sudo chrt -f 80 ./host/build/examples/bridge_can_loopback_latency usb      3000 7 6
```

## 2. 多板方案选型：N 块 USB 板 [实测 2026-08-01]

| | 1 块 USB 板 | 2 块 USB 板（同一控制器） |
|---|---|---|
| p50 | **99.8** | **124.7**（丢一个微帧） |
| max（调好 IRQ 后） | 130 | 180 |
| max（**没调 IRQ**） | 130 | **~1000** |
| 扩到更多块 | — | 同控制器越多越差；控制器数量有限（本机 2 个） |
| 布线 | 1 根 | **N 根都要回主机** |
| 距离 | ~5 m | ~5 m |
| **故障隔离** | — | **每块板独立，坏一块不影响其他** |
| 跨板同步 | 无 | **有：USB SOF 共享时间轴**（见 [SOF_TIMEBASE.md](SOF_TIMEBASE.md)） |
| 主机开销 | 1 个事件线程 | N 个事件线程 + 共享控制器中断 |

**怎么选：**

- **1 块板 → 用 USB 单核镜像。** 99.8us。
- **2-3 块板 → 多块 USB 板仍然合理**，代价是 p50 退到约 125us，且**必须把 xHCI
  中断线程提到 FIFO 90**，否则 max 约 1 ms（见
  [../../HOST_TUNING.md](../../HOST_TUNING.md) 第 9 节）。

> **为什么不用 EtherCAT 级联**：同口径实测下它在任何板数下都不比 USB 快，而单从站
> 场景付出了它全部的固定成本。完整对比、解冻条件与全部实测数据见归档
> `~/Desktop/ethercat-archive-2026-09-08/ARCHIVE_NOTES.md`。

## 3. CAN 采样点：必须对齐总线，不是对齐推荐表 [实测 2026-08-03]

**结论先行：`can.hpp` 把仲裁域和数据域的采样点都钉死在 87.5%，不要改回 SDK 默认，
也不要照厂商推荐表改。** 改动是 `Can` 构造函数里这四行：

```cpp
config.can20_samplepoint_min = 875U;   config.can20_samplepoint_max = 875U;
config.canfd_samplepoint_min = 875U;   config.canfd_samplepoint_max = 875U;
```

### 为什么必须显式设

HPM SDK 的默认窗口是 `[750, 875]`（千分比），而它的求解器**爬过下界就停**：

```c
while ((num_seg1 * 1000U) / num_tq < samplepoint_min) { ++num_seg1; --num_seg2; }
```

所以结果**永远是 75.0%**，那个 `max = 875` 是死代码。`bsp/hpm_sdk/samples/drivers/mcan/`
里一处都没设过这两个字段，**HPM 全部官方示例都跑 75%**。

### 症状与判据

CubeMX 板（mc02、c_board）两个域都是 `tseg1/tseg2 = 13/2 = 87.5%`。5321 在 75% 时：

| | 结果 |
|---|---|
| classic CAN 双向 | **正常**（1 Mbit 容差有 12.5 个百分点余量） |
| `mc02 -> 5321` FD | **正常** |
| `5321 -> mc02` FD | **一帧不到**，`PSR.DLEC = ACK error`，TEC 爬到 136 进 error-passive |

5 Mbit 一个位只有 200 ns，75% 与 87.5% 差 25 ns，接收方取到的已是下一位。
**这种"classic 通、FD 单向不通"就是采样点不一致的特征签名。**
另见 [USB_OPTIMIZATION_LOG.md](USB_OPTIMIZATION_LOG.md) 8.5：两块板固件版本偏斜也会
产生一模一样的现象，多板 rig 上先查版本再读遥测。

> **2026-08-24 在实板上复验：修复仍然生效。** mc02 <-> 5321，CAN0<->CAN0 / CAN1<->CAN1，
> 两个方向 x 两条总线 x classic/FD 共 8 组，每组 2000 帧，**全部 0 超时、0 损坏**；
> 上表那一行"`5321 -> mc02` FD 一帧不到"现在是 2000/2000 通，p50 128.0 us。
> `[实测 2026-08-24，mixed_board_test latency]`

### 关键陷阱：不要照推荐表改回去

厂商推荐表（">800 kbps 用 75%"、"仲裁域与数据域不要求一致"）会让你认为 1 Mbit
仲裁域应该是 75%。**照着做会坏。** 实测：仲裁域留 75%、只钉数据域 87.5% →
`PSR.DLEC = bit1 error`，双总线 **0/40000**。

原因是 mc02 的仲裁域**不是** 75%：

| | 仲裁域配置 | TQ 数 | 波特率 | 采样点 |
|---|---|---|---|---|
| 典型参考驱动 `CAN_BR_1M` | brp=1, seg1=59, seg2=20 | 80 | 1 Mbit | 75.0% |
| **mc02 实际（CubeMX）** | brp=5, seg1=13, seg2=2 | 16 | 1 Mbit | **87.5%** |

两者都是 1 Mbit，采样点差 12.5 个百分点。**推荐表是"没有其他约束时选什么"，
一旦总线上已有节点，"所有节点采样点一致"这条压倒一切。**

### 影响范围

`can.hpp` 由**全部 hpm_board 镜像共用**（hpm5321 / hpm6e8y）。此前"FD 没问题"的印象来自只测 hpm_board 对 hpm_board——
两端错得一样所以互通。**真实电机（DJI、达妙 MIT、瓴控）全是 87.5%**，所以这个修复
是让 5321 能跟电机跑 CAN-FD，不只是为了跟 mc02 说话。

### 怎么自查

`-Dlibhcs_CAN_DIAG=ON` 的遥测（记录版本 5）现在带 `NBTP`/`DBTP`，
`host/examples/can_stall_probe` 会直接打印解码后的采样点：

```
timing: nominal brp=1 tseg1=69 tseg2=10 sp=87.5% | data brp=1 tseg1=13 tseg2=2 sp=87.5% tdc=0
```

## 4. UART 运行时改波特率：DLAB 会把 THR 变成除数锁存器 [实测 2026-08-05]

**结论先行：读 `DLL`/`DLM` 必须在 TX DMA 停稳之后，否则读的动作本身会毁掉波特率。**
`DLL` 与 `THR` 共用偏移 `0x20`、`DLM` 与 `IER` 共用 `0x24`，由 `LCR.DLAB` 选择
（`soc/HPM5300/ip/hpm_uart_regs.h`）。而 TX DMA 的目的地址在 init 时就固定成
`&uart_base_->THR`，**它不认识 DLAB**。所以只要 DLAB=1 期间 DMA 送出一个字节，
那个字节就写进了除数锁存器，波特率当场被数据覆盖，端口从此不出声。

### 症状签名（很反直觉，值得记住）

| 现象 | 说明 |
|---|---|
| 遥测报告除数**完全正确** | `DLM` 在 `DLL` 被覆盖**之前**读到，所以快照是好的 |
| 但那个口**一个字节都不发** | 硬件里的除数已经被数据字节改掉 |
| 与波特率精度**无关** | 1000000 / 2000000 除数精确（div=10 / div=5）照样失败 |
| 重刷两块板也没用 | 不是状态污染，每次 100 ms 遥测都会重新踩一次 |

**"算对了、写对了、然后不发"= 观测者把被观测对象改坏了。** 这次的观测者就是
`-Dlibhcs_CAN_DIAG=ON` 里那段读除数的遥测代码，它跑在主循环、和 TX DMA 并发。

### 修法

- `Uart::snapshot_divisor()` 只在 **init** 和 **`handle_config()` 里 `abort_transmit()` 之后**
  采样一次，存进 `uart_divisor_`；遥测从快照读，**永不按需读寄存器**。
- `abort_transmit()` 先 `dma_abort_channel()` 再 `dma_mgr_disable_channel()`：
  `CHABORT` 的寄存器手册明确写"写入对未使能的通道会被忽略"[RM]，顺序颠倒等于没中止。
- 切换后补 `uart_reset_tx_fifo()`，丢掉旧波特率下已经进 FIFO 的字节。

### 上游怎么做的（对照过 SDK）

HPM SDK **全库没有任何读回 `DLL`/`DLM` 的代码**，也没有 `uart_get_baudrate()`。
唯一改运行时波特率的样例（`samples/drivers/uart/uart_lin/slave_baudrate_adaptive`）
是纯"只写不读"：`uart_set_baudrate()` 之后 `uart_reset_rx_fifo()` 再重开 RX。
**上游根本不回读，所以它碰不到这个坑。** 我们要回读是为了遥测，那就必须用快照。

另外 `uart_set_baudrate()` 自己有个缺陷值得知道：它**先**置 DLAB，求解失败时
**带着 DLAB=1 直接 return**，不清标志位（`drivers/src/hpm_uart_drv.c:222-227`）。
所以调用方必须无条件自己清一次 DLAB，不能只在成功分支清。

`uart_set_baudrate()` **是有返回值的**（`status_success` /
`status_uart_no_suitable_baudrate_parameter_found`），上游样例也确实检查它，而且
**失败时提前 return、跳过后面的 FIFO 复位**：

```c
hpm_stat_t stat = uart_set_baudrate(TEST_UART, lin_baudrate, uart_source_clk);
if (status_success == stat) { ... } else { printf("not supports"); return; }
uart_reset_rx_fifo(TEST_UART);   /* 只有成功才走到这里 */
```

我们照这个形状做：被拒时分频器没动、旧波特率仍然有效、队列里的字节对它仍然合法，
所以**跳过 FIFO 复位**（不为一次什么都没改的请求丢数据），直接返回 false。
这也和仓库内 `ch32_board` 的既成惯例一致——它对 BRR 范围做检查后同样返回 false，
注释讲得最直白：*"this is host-supplied data, so a bad value must be reported,
never asserted on"*。

### 已知缺口：配置被拒绝，主机看不到（**已关闭 2026-09-04，改走 EP0**）

> **状态**：历史记录。缺口本身已由 EP0 配置通道填上，但下面这段"为什么
> 带内通道传不了失败"的推理仍然成立，是选择 EP0 的直接理由，保留备查。

`handle_config()`（现已改名 `set_baudrate()`）返回求解器的真实结果（80 MHz 表示不
出的波特率会被拒，此时**分频器保持不动、端口继续用旧波特率**，不会损坏）。但
`link/host_session.hpp` 的 `uart_config_deserialized_callback` 丢弃了这个返回值并
无条件 `return true`——因为该回调的 `bool` 在协议层的含义是"这个 field 认不认识"，
不是"操作成不成功"，不能拿来传失败。**要让主机知道切换被拒，需要一条独立的应答
通道**——最终选的不是"协议层加一条 config ack"，而是把配置整个搬到 EP0，因为控制
传输的 status stage 本来就是这条应答通道。

实测：请求 6000000（求解器无解）板端正确拒绝、寄存器未变、随后 115200 仍 PASS；
但主机侧不会打印 rejected。而 3000000 **会被接受**（`osc=26 div=1 -> 3076923`，
误差 2.56%，在 SDK 的 3% 容差内），别以为它非法。**这两条实测在 EP0 通道上原样
复现，只是现在 6000000 会抛异常而不是静默成功。**`[实测 2026-09-04]`

### 顺带修掉的独立 bug

`abort_transmit()` 漏清 `in_flight_`，导致下一次 `try_dequeue()` 把 `out_` 推过
DMA 实际没发出的字节，环缓冲永久错位。**这个与上面的除数覆盖无关**：它只在切换
之后发作，且表现为"数据错乱"而不是"完全不发"。

## 5. EP0 配置通道落地时的坑 [2026-09-04]

现行协议（bRequest 表、三条约束、EP0/bulk 判据）见 [AGENTS.md](AGENTS.md)
「EP0 配置通道」。本节记的是落地时踩的坑。

### 握手时序：主机侧握手必须早于第一个 kStart

`Handler` 构造是**同步等 SESSION_ACK** 的，而板类原本在成员初始化列表里
`handler_(...)` 之后才做 EP0 握手——那样板子永远等不到握手、主机永远等不到 ack。
现在 `Handler` 多了一个 `BeforeSession` 钩子，在传输已建立、session 未开的窗口里跑；
`Impl::start()` 也因此从构造函数里拆了出来。

**拆出来会引入一个泄漏**：构造函数**体**抛异常不会调用本对象析构，`impl_` 连同已 claim
的 libusb 接口一起泄漏，下次打开同一块板得到 `ERROR_BUSY`（`ep0_config_test` 第 5 项
当场抓到）。所以那段必须 try/catch + `delete impl_`。

### 旧主机会蹭到上一个主机留下的握手

**板端要在 session 结束时忘掉握手，主机要在重连时重做。** `tud_mount_cb` 只在**重新
枚举**时触发，而换个主机程序不重插线是不会重新枚举的——实测旧主机就这样**直接蹭到了
前一个主机留下的握手**走了进来。现在 `deactivate_session()` 会清除它，主机的 keepalive
线程在重建 session 前重跑一次钩子。钩子和 `Configuration` 都必须**按值**存起来：它们
原本是构造函数参数，重连时早已悬空。

### kGetCanStatus 的判读表

EP0 `0x45 kGetCanStatus`（wIndex = 丝印编号）回读
`TEC`/`REC`/`PSR.LEC`/`PSR.DLEC`/`TXBTO`/`TXBCF`/转发帧数。**正式镜像上就能用**，
不需要 `-Dlibhcs_CAN_DIAG=ON`，也不占用 `kUart0` 上行通道。判据：

| 读数 | 含义 |
|---|---|
| `LEC=ACK`、`TEC` 高 | 帧发出去了，没有节点应答——对端没在听 |
| `LEC=BIT0` | 总线拉不成显性——CAN_H/L 开路、接反或收发器没使能 |
| `STUFF`/`FORM`/`CRC` | 位被破坏——位时序、终端电阻或干扰 |
| `tx_ok=0` 且 `tx_cancel!=0` | 一帧都没上过总线（驱动关了自动重传，失败即取消） |

健康总线读 `tx_ok=0xffffffff / tx_cancel=0x00000000`，帧到不了对端的总线读**正好相反**。
`[实测 2026-09-04，两个方向都验证过]` 工具：`host/examples/can_bus_diag.cpp`。
配套验证工具：`host/examples/ep0_config_test.cpp`（一块 0x5322 板，不需要 CAN/UART 接线），
六项全 PASS，含"6000000 被拒且端口仍在旧速率"和"把 FD 总线当 classic 打开会在构造
期抛异常"这两条旧通道表达不出来的结果。

## 6. 教训：自环测试对"共模错误"是瞎的 [实测 2026-08-05]

**结论先行：同一块板两个口互接的自环测试，无法证明波特率切换真的生效。**
排查 UART DLAB 坑时，mc02 的 `UART7 <-> UART10` 自环在 115200 到 2000000 全部 PASS，
于是"mc02 被洗清、问题在 5321"——**这个推论是错的**。

自环会把两端**同时**设成目标波特率。如果配置代码根本没生效，两端就**一起**留在
115200，波特率相同、通信照样正常、测试照样 PASS。**自环只能验证两端一致，不能验证
两端等于你要的值。** 共模错误对它完全不可见。

实际上 mc02 这边还藏着第二个独立 bug：`HAL_RCCEx_GetPeriphCLKFreq()` 在本版 HAL 里
的 if/else 链只覆盖 SAI/SPI/ADC/SDMMC/FDCAN，**两个 UART 组一个分支都没有**，
直接掉到末尾 `else { frequency = 0; }` 返回 **0**（`stm32h7xx_hal_rcc_ex.c`）。
于是 `handle_config()` 撞上自己的 `kernel_clock_hz == 0` 提前返回，**`BRR` 一次都没写过**，
mc02 永远停在 CubeMX 的 115200。修法与判据见
[mc02 AGENTS.md](../mc02/AGENTS.md)「不要用 HAL_RCCEx_GetPeriphCLKFreq()」。

**判据**：要证明"切换生效"，必须让**一端切、另一端不切**，然后确认通信**变坏**；
或者跨板对打，两端各自独立配置。只看自环 PASS 等于没测。

## 7. 主机侧板类合一：一块芯片一个类 [2026-09-05]

`host/include/libhcs/board/hpm5321.hpp` 里的 `Hpm5321` **同时服务两块 PCB**（单 CAN
`0x5321` 与双 CAN `0x5322`）。`core/include/libhcs/spec/hpm5321_dual_can/` 与兼容别名头
`hpm_board_hpm5321_dual_can.hpp` 均已删除：**类名不再编码总线数**，因为它是运行期事实。

**为什么以前是两个**：板卡有几条 CAN 总线，过去只能由编译期的描述符表回答，于是
"几条总线"变成了类型的一部分。**EP0 的 `kGetInterface` 把这个前提废掉了**——它在第一个
session 之前就报出 `can_count`，所以运行期知道得比编译期更准，而且和固件（自己读 OTP
第 25 字判板型）用的是同一个事实来源。

> **同样的形状还在别处**：`board/*.hpp` 这一族 7 个头文件两两之间只差 50~90 行，
> `lite`/`pro`、`hpm5321`/`hpm6e8y` 都是复制粘贴关系。彻底的做法是一个按 board traits
> 参数化的类模板，**但那会动全部 7 块板的公开 API，本次没有做**。

## 8. 诊断开关的使用坑

- **`CAN_DIAG` 与 `SOF_DIAG` 都占用 `DataId::kUart0`**（本板的 UART1 数据口），把记录
  当 UART 上行帧发出去（`diag/can_diag.cpp` 的 `write_uart(kUart0, ...)`）。**两者共用
  这一条上行通道，不要同时打开**；`SOF_DIAG` 还会额外开 8 kHz 的 SOF 中断。
  `TIME_SYNC` 走 session 字段，跟另外三个不冲突。
- **`CAN_DIAG=ON` 会让 `dual_board_test uart` 必然 FAIL** `[实测 2026-09-04]`。遥测
  记录混进 UART0 的上行流，接收侧按"字节流必须是伪随机序列的前缀"校验，多出来的
  字节把计数器一次性错位，**之后每个字节都算 mismatch**：实测 `received
  34120/32000 bytes, mismatches=32628`，而同一份代码 `CAN_DIAG=OFF` 是
  `32000/32000, mismatches=0`。这看起来完全像一个 UART 回归——2026-09-04 为此烧了
  六次板、追了一个并不存在的 UART 回归。
- **教训：A/B 两个镜像时，先 `grep libhcs_ .../CMakeCache.txt` 对齐编译选项。**
  `cmake --preset` 不会重置已缓存的 `-D` 开关，所以复用旧 build 目录和新建 build
  目录**默认就不是同一份配置**。
- `PULSE_TEST` 会**借走 UART0 的两个引脚**（PB08/PB09 改成 GPTMR0 的比较/捕获），
  所以这个 build 下 UART0 不工作；它也是唯一一个自己开外设的诊断开关，硬件初始化被
  刻意推迟到收到主机第一条 schedule 之后——**理由是 boot 期卡死会连 DFU 一起带走，
  实测代价是两块板**，见 [SOF_TIMEBASE.md](SOF_TIMEBASE.md) 5.5「教训」一段。

2026-08-01 的 CAN 闩死定位、core1 时基确认，以及 2026-08-19 的跨板 SOF 时间轴验证，
全部靠这几条带内诊断通道完成，没有用到任何调试器。
