# mc02 UART 环形缓冲实录：修了什么、测出了什么、我错在哪

> **文档类型**：过程记录
> **适用范围**：`firmware/mc02/`，当时四个 UART 口（USART1 / UART7 / USART10 / UART5-DBUS）的 DMA 收发路径。USART2 / USART3（丝印 UART2 / UART3）后来常开，不在这一轮重构范围内。
> **状态**：现行有效（2026-08-12 完成的一轮重构、修复与实测）
> **相关文档**：[AGENTS.md](AGENTS.md)（现行命令与约束） · [README.md](README.md)（外设与低延迟设计） · [../hpm_board/USB_OPTIMIZATION_LOG.md](../hpm_board/USB_OPTIMIZATION_LOG.md)（同类实录，USB 侧）

## 摘要

这份文档记录 2026-08-12 一轮 UART 工作：**两个真 bug 的修复**（其中一个会让整个端口
永久失聪）、**一次接收路径重构**、**把 DMA 环搬进 D2 SRAM**、以及**第一次把这块板压到
出错为止的吞吐实测**。

最重要的一条：**`CR3.DDRE` 的极性一直是反的**，配合刻意关闭的 `CR3.EIE`，一个坏字符就
能让端口永久停止接收——USART 仍然报告 IDLE 事件，但零字节进入内存。这个 bug 从
`2d83b99` 引入，只有跨板测试能发现。

第二重要的一条：**瓶颈不在固件**。实测容量 781 KB/s 聚合零丢失，天花板约 800 KB/s，
由 USB Full-Speed 决定；主循环在满过载时仍有 10 倍余量。

## 本文导航

| 章 | 内容 |
|---|---|
| 1 | 修掉的两个真 bug（DDRE 极性、TXFIFO 未冲刷） |
| 2 | 接收路径重构：双 bank → 整环单缓冲 |
| 3 | DMA 环搬进 D2 SRAM |
| 4 | UART FIFO 全部打开 |
| 5 | 实测：容量、天花板、主循环剖析 |
| 6 | **我在过程中写错又更正的五处结论** |
| 7 | 测试工具与复现方法 |
| 8 | 未做的与待办 |

---

## 1. 修掉的两个真 bug

### 1.1 `CR3.DDRE` 极性写反——一个坏字符永久杀死端口 [实测]

**结论**：`DDRE` 是 "DMA **Disable** on Reception Error"，**置 1 = 出错时禁用 DMA**。
命名规则和旁边的 `OVRDIS` 一致：位名描述的是置 1 时启用的行为。原代码
`ATOMIC_SET_BIT(CR3, USART_CR3_DDRE | USART_CR3_OVRDIS)` 配的注释却说"这样出错时 DMA
就不会被禁用"，**做的事和注释说的正好相反**。

**为什么后果是永久的**：`CR3.EIE` 是**刻意留空**的（不让 HAL 的错误分支去
`UART_EndRxTransfer` + `HAL_DMA_Abort_IT`）。于是没有任何东西会去清那个错误标志，而
`rx_error_callback` 只在 DMA 控制器故障时触发、从不管线路错误。一次 framing/noise
glitch → DMA 请求被永久阻断 → 重启前不再恢复。

**现场特征**（值得记住，这是识别它的指纹）：

```
silent 3s     : A chunks=0 bytes=0 | B chunks=0 bytes=0
A sent 10x64B : B chunks=10 bytes=0        <-- 精确 10 次事件,0 字节
B sent 10x64B : A chunks=30 bytes=640      <-- 反方向正常
```

发 10 次，对端**精确报告 10 次 chunk 但零字节**。线是通的、USART 收到了字符并检测到随后
的空闲（所以 IDLE 中断照常触发、`idle_count` 照常递增、消费者照常产出零长度的
idle-delimited 上行），**只是 DMA 请求被挡住了**。这个组合能把"接线问题"和"DMA 被阻断"
区分开。

**为什么以前没发现**：同板自环测不出来——两端都是同一颗芯片、同一次 glitch 或都没
glitch。只有两块独立的板子，其中一块在启动或第一次波特率切换时吃到坏字符，才会呈现
单向失聪。

**修法**：置 `OVRDIS`、**清** `DDRE`。见 `app/src/uart/rx_buffer.hpp`
的 `configure_rx_error_policy()`，那里写了完整的极性说明。

### 1.2 TX 错误路径没有冲刷 TXFIFO [推断，未上板]

`tx_error_callback()` 只清了 `CR3.DMAT`，没有清 TXFIFO。TX DMA 中途出错时，被中止的那个
包最多还有 16 字节留在 TXFIFO 里，会抢在下一个包前面发出去——对靠 idle 分帧的下游就是
一个被污染的帧，而 `TxBuffer` 的 checkpoint 机制**恰恰是为了让这些帧分开而存在的**。

修法：`RQR.TXFRQ`，与接收侧的 `flush_rx_fifo()`（`RQR.RXFRQ`）对称。F407 没有请求寄存器，
所以这是从 c_board 移植时不可能带过来的东西。

**注意这是第 4 章的配套义务**：打开全部 FIFO 之前它只影响 UART5/UART7，之后覆盖四个口。

**无法在现有台架验证**——需要 DMA 控制器故障才能触发。改动是一次寄存器写、只在故障路径
执行，回归风险为零，但正确性是推理出来的。

---

## 2. 接收路径重构：双 bank → 整环单缓冲

**改之前**（从 c_board 移植而来）：DMA 双缓冲模式，两个 32 字节 bank 轮流；每填满一个就
中断一次，中断里读 `CR.CT`、读 `NDTR`、改另一个 bank 的 `M0AR/M1AR`，全程
`InterruptLockGuard`（全局 `__disable_irq()`）。

**改之后**：一条 circular DMA 盖住整个 2048 字节环，启动后**永不停、永不重装、不分块**。
写指针由消费者在主循环里从 `NDTR` 推导（`2048 - NDTR` 是环内偏移，高位 lap 由上次的
`in_` 补）。IDLE 中断缩成一条 `idle_count_.fetch_add(1)`。

| | 双 bank | 单环 |
|---|---|---|
| ISR 必须多久跑到 | 32 字节时间（921600 下 347 us） | **2048 字节时间（22 ms）** |
| RX 中断率/口 | ~9000/s @921600 | **只剩 IDLE 事件** |
| 中断里关全局中断 | 每次 TC 都关 | **不再有** |
| `CT`/`NDTR` 读撕裂 | 存在（实测不可达） | **结构上不存在** |
| 代码量 | — | 少约 80 行 |

关键实现细节，都写在 `rx_buffer.hpp` 的注释里：

- `try_dequeue()` **先读 idle 计数，再读 NDTR**。顺序反了会把边界贴到还没到达的字节上。
- `RxXferSize` 设成 `kBufferSize + 1`。HAL 的 IDLE 分支判据是
  `0 < NDTR < RxXferSize`，而 circular 模式下 NDTR 取值 1..kBufferSize，多一才能用单条
  比较覆盖全部。
- `bind_rx_dma_callbacks()` 把 `XferCpltCallback` 设为 `nullptr`。传输完成不携带信息，
  `HAL_DMA_IRQHandler` 会 null 检查后只清标志 [RM]。
- `in_`/`out_` 变成**消费者独占的普通变量**，整个类里唯一的原子量是 `idle_count_`。

**为什么不用"半满/全满"（HT+TC 中断）**：那是循环 DMA 的教科书写法，但对 2048 字节的环
粒度是 1024 字节 —— 921600 下 11 ms。而主循环 12 µs 就轮一次 NDTR，分辨率是 0.24 字节，
**细约 4000 倍**。教科书那么写是因为大多数设计没有一个高频主循环可以轮询。

**为什么 DMA FIFO 必须保持关闭**：`NDTR` 统计的是"从外设搬进 DMA FIFO"的数量，不是"落到
内存"的数量。开了 FIFO 之后，IDLE 时残留在 DMA FIFO 里的字节不在内存里，写指针必然算错。
重构之后这条比以前更致命，因为写指针**完全**依赖 NDTR。

---

## 3. DMA 环搬进 D2 SRAM

四个 UART 端口对象整体移到 `.d2_sram`（0x30000000）。DMA1 是 **D2 域主设备**，之前每个
字节都要跨 D2→D1 互联去写 AXI SRAM，并在那里和 M7 自己的访问抢总线。

**意外收获比这个更实在**：这些对象**本来就在 `.data` 里**（共 18.7 KB，`Lazy` 的 union
里存了构造参数，所以整块要 FLASH 装载镜像）。搬走之后：

| 区域 | 之前 | 之后 |
|---|---|---|
| RAM（AXI） | 42544 B | **23408 B** |
| RAM_D2 | 0 B | **19136 B**（58.4%） |
| FLASH | 117788 B | 117972 B（+184，MPU 配置代码） |

`_ebss` 从 26160 降到 7024，链接脚本尾部那条 32 KB 非缓存 MPU 窗口的 ASSERT 从只剩
6.5 KB 余量变成 25 KB。

配套的三件必要工作：

1. **`__HAL_RCC_D2SRAM1/2_CLK_ENABLE()` 必须在拷贝之前**。H723 只有 SRAM1/SRAM2 两个
   16 KB 银行，**没有 SRAM3**——`__HAL_RCC_D2SRAM3_CLK_ENABLE` 在这颗片子上不存在，
   写了编译不过 [实测]。
2. **MPU region 1** 把 0x30000000 设成非缓存。0x30000000 落在 M7 默认映射的 SRAM 区，
   是 write-back write-allocate **可缓存**的；不改的话 DMA 环就藏在 D-cache 后面。写在
   `app.cpp` 的 `configure_d2_sram_mpu_region()` 而不是 CubeMX 生成的 `MPU_Config()` 里，
   **所以下次 Generate 不会被覆盖、也不需要手工复原**。
3. 链接脚本加 `.d2_sram >RAM_D2 AT> FLASH` + 启动拷贝循环 + 一条 32 KB 上限 ASSERT。

**不要把端口对象移进 `.dtcm`**（像 `can.hpp` 那样）：DTCM 只有内核够得到，DMA 流指向它会
静默地什么也不搬。

---

## 4. UART FIFO 全部打开

CubeMX 留下的状态是不一致的：UART5/UART7 调 `EnableFifoMode`，USART1/USART10 调
`DisableFifoMode`，**没有理由**。而 `HAL_UARTEx_DisableFifoMode` 只清 `CR1.FIFOEN`，
CubeMX 给每个口都设过的 `CR3.RXFTCFG/TXFTCFG`（全是 1/8）原封不动留着——**四个口的差异
就是一个 bit** [实测]。

统一到全开，实现在 `RxBuffer::enable_fifo_mode()`，**不在 `.ioc` 里**。两个理由：

- `TxBuffer::try_dequeue()` 用 `ISR.TC` 而不是 DMA 完成来判线路排空，**正是因为 TXFIFO
  存在**（16 字节在 921600 下 173 µs，和 300 µs 空闲窗口同量级）。把不变量和依赖它的代码
  放在一起，CubeMX 重新 Generate 就不可能悄悄破坏它。
- `configure_rx_error_policy()` 设了 `CR3.OVRDIS`，溢出**不上报**。真发生溢出就是静默丢
  数据，而 FIFO 那 16 字节正是对这件事的廉价保险。

**收益不是吞吐**：921600 下一字节 10.8 µs，DMA 仲裁快几个数量级，缓冲永远用不上。它是给
异常路径兜底的。

**后果**：`.ioc` 里 USART1/USART10 的 FIFO 复选框现在是装饰性的，app 代码无条件打开。

---

## 5. 实测

台架：两块 mc02，USART1↔USART1（port 0）与 UART7↔UART7（port 1）交叉直连，
两块板各挂在 Bus 003 的不同根端口（**各有独立的 12 Mbit/s FS 总线**，`/sys/.../speed=12`）。

### 5.1 容量与天花板 [实测]

| 配置 | 主机发出 | 送达 | 丢失 |
|---|---|---|---|
| 921600 × 4 腿 | 360 KB/s | 360 KB/s | **0.00%** |
| 2 Mbaud × 4 腿 | 781 KB/s | 781 KB/s | **0.00%** |
| 4 Mbaud × 2 腿（单口全双工） | 781 KB/s | 781 KB/s | **0.00%** |
| 2.2857 Mbaud × 4 腿 | ~893 | 728 | 16.8% |
| 2.56 Mbaud × 4 腿 | ~965 | ~693 | 22~36% |
| 3 Mbaud × 4 腿 | ~1009 | 632 | 35.9% |

两条结论：

- **4 Mbaud 单口全双工 0% 丢失**。UART 跑到 HSI 时钟允许的最高速率（64 MHz / 16）、
  双向同时满载，一个字节不丢。**串口路径不是瓶颈。**
- **送达量先升后降**（781 → 728 → 693 → 632）。喂得更多拿到更少，是**拥塞崩溃**不是链路
  饱和：TX 环满 `try_enqueue` 退回、uplink 池满收到的帧被丢，白干的部分越来越多。

**测量局限**：超过 ~1 MB/s 之后各条腿的 offered 开始发散（同一次里 1280000 / 960000 /
1280000 / 1234688），**主机侧发送器自己也到顶了**，不再是受控输入。**过载区的任何对比都
不可信**，包括本文档 6.3 撤回的那条。

### 5.2 主循环剖析 [实测]

用一直闲置的 `DWT->CYCCNT`（`app.cpp` 早就使能、从没人读）给主循环分段，
`libhcs_APP_LOOP_PROFILE=ON`：

| | 空载 | 2 Mbaud（0% 丢失） | 2.56 Mbaud（42.6% 丢失） |
|---|---|---|---|
| 循环频率 | 85 kHz | 82 kHz | **80 kHz** |
| 每圈周期 | 6443 | 6637 | 6823 |
| **usb（7 次 try_transmit）** | **2570（40%）** | **2544（38%）** | **2610（38%）** |
| uart（4 口） | 1773（27%） | 1685 | 1772 |
| tud_task | 484 | 739 | 775 |
| can（3 路） | 729 | 752 | 750 |

**`usb::vendor->try_transmit()` × 7 确实是主循环里最大的一块（约 40%，每次 367 周期），
但它不是瓶颈**：从空载到 42% 丢包，每圈耗时只涨 6%，时间分布几乎不动。循环在丢包最狠时
仍跑 80 kHz——每 12.5 µs 轮遍所有源，比它伺候的 1 ms USB 帧细 80 倍。**CPU 有约 10 倍
余量。**

补充证据：打开 IMU（每圈多 816 周期，循环掉到 69 kHz），**容量点仍然 781.2 KB/s、0% 丢失**。

### 5.3 `try_transmit` 到底做什么 [RM + 源码]

它**不搬数据**。`dcd_dwc2.c:632` 的 `dcd_edpt_xfer()` 注释写得很直白：

```c
// Schedule packets to be sent within interrupt
edpt_schedule_packets(rhport, epnum, dir);
```

只记下缓冲区指针和长度、使能 TX-FIFO-空中断就返回。真正把字节写进 USB 外设 FIFO 的是
**USB 中断里的 `handle_epin_slave()`**（`dcd_dwc2.c:970`，`CFG_TUD_VENDOR_TX_BUFSIZE = 0`
的非缓冲模式）。

完整数据流三段，CPU 只管中间一段：

```
① 线上字节 ──DMA(硬件)──> ring              CPU 不参与
② ring ──主循环──> serializer ──> 批次       CPU 唯一搬字节的地方
③ 批次 ──USB 中断──> 外设 FIFO ──硬件──> 线上  主循环不管
```

`try_transmit` 的 367 周期主要花在 `refresh_session_state()` + 端点空闲查询上。

**为什么不改成完成回调驱动**（`tud_vendor_tx_cb` 是存在的）：有 8 个生产者，回调只说
"端点空了"不说"该发谁的"；主循环里 7 次交错**本身就是延迟优化**（CAN1 刚填满就推走，
不等 UART1 被看一眼），而完成回调只能在上一次传输结束时触发；轮询 12.5 µs 的劣势相对
1 ms 的帧粒度可以忽略；放进 USB 中断（优先级 2）会挤 FDCAN（优先级 1）。

---

## 6. 我在过程中写错又更正的五处结论

留在这里是因为其中几条会重复发生。

### 6.1 「DDRE 让 DMA 在出错时继续跑」——极性反了

见 1.1。原注释、原代码、我最初的复述**全都是反的**。教训：`OVRDIS` 就在旁边，同样的命名
规则，本可以互相印证。

### 6.2 「781 KB/s 是硬天花板」——是我测试参数的巧合

4 Mbaud × 2 腿和 2 Mbaud × 4 腿恰好都等于 781 KB/s，**两次都是 0% 丢失全部送达**，所以
781 是**下限不是上限**。我一度把两个相同的数字读成"共享资源到顶"。

### 6.3 「IMU 让吞吐掉 18.5%」——撤回

那组数字全在**过载区**测，而过载区 offered 会漂 ±20%（同配置 IMU OFF 拿到过 665、685、
691、702、719，跨度 8%）。**在容量点及以下 IMU 开关完全无差别**（都是 781.2、0%）。

### 6.4 「主循环 380 kHz」——实测 69~85 kHz

`led.hpp` 注释里的 380 kHz 差 4.5 倍。基于它做的推算（例如 IMU 关中断占 2.9% 时间）也
偏大，实际约 0.65%。LED 的节拍逻辑不受影响。**该注释仍需更正。**

### 6.5 IMU 的 A/B 测试第一次是无效的——build 目录 cache 陷阱

`firmware/mc02/build/` 在会话开始前就存在，`cmake --preset debug` **沿用旧 cache**，所以
`option(libhcs_APP_IMU_ENABLE ... ON)` 的默认值从未生效——cache 里它是 OFF。我设 OFF 做
A/B 时是个空操作，两边都是 OFF。

**这条最值得记住**：在已有 build 目录上，`option()` 的默认值不生效，必须显式
`-Dlibhcs_APP_IMU_ENABLE=ON`。识别方法：`grep libhcs_APP_IMU_ENABLE build/CMakeCache.txt`，
或看 `arm-none-eabi-nm build/app/mc02_app.elf | grep -c bmi088`（OFF=3，ON=57）。

### 6.6 附：一个当场撤销的改动

会话中一度加了 RAMECC（SRAM ECC 错误上报）。**它第一次上板就误报**：H7 上读取从未写过的
ECC 内存会报假的双位错，而 `.d3_sram`（WS2812 缓冲）和 `._user_heap_stack` 都是 `NOLOAD`
从不初始化，`memset` 一个没写过的 `txbuf` 正好命中 `RAMECC_SR_DEBWDF`（字节写触发的双位
错）。表现是 WS2812 常亮红白快闪。**已整个撤销**——功能无法在台架验证、真实发生率接近零、
且第一次接触硬件就误报。要正确地做，前提是先在启动时把所有 NOLOAD 的 ECC 内存清零。

---

## 7. 测试工具

`host/examples/uart_cross_test.cpp`（新增）。**存在的理由**：同板自环结构上无法发现本文
第 1 章那类 bug——两端一起忽略同一个配置请求，自环照样通过。

四个模式：

```bash
# 全套 15 项回归（默认 port 4 = UART10，用 HCS_UART_PORT 选口）
# 口序号：0=UART1、1=UART2 RS-485、2=UART3 RS-485、3=UART7、4=UART10
HCS_UART_PORT=3 ./host/build/examples/uart_cross_test

# 定位单向失聪：静默 3 秒 + 各方向探测，区分"没发"和"发了但没进内存"
./host/build/examples/uart_cross_test monitor

# 多口并发（默认 0 和 3：UART1 与 UART7）
HCS_UART_BYTES=32768 ./host/build/examples/uart_cross_test concurrent 0 3

# 加压到出错，报告 offered/delivered/丢失率/KB/s
HCS_UART_BAUD=2000000 HCS_UART_PERCENT=100 ./host/build/examples/uart_cross_test saturate 0 3

# 打印固件诊断通道（kUart0，配合 libhcs_APP_LOOP_PROFILE）
HCS_UART_DIAG=1 ... 
```

覆盖：链路双向、1/31/32/64/128/256 字节帧（32 整倍数专门覆盖"收尾 idle chunk 为空"）、
idle 分帧不被拼接、4096/32768 字节流逐字节、**一端切波特率另一端不切必须变坏**（自环测不
出的那条）。

**测试脚本自己也踩过坑**：限速发流时块间间隔远超 300 µs，接收端每块后都正确上报 idle，
而最初的重组逻辑把"第一个 idle"当消息结束，只比较了第一块。流测试必须忽略帧边界、比较
字节序列。

---

## 8. 未做的与待办

**已知缺口**

- `kMinFragmentSize = 32` 是**个数不是时间**，低波特率下代价爆炸（9600 时 33 ms）。
  加一条时间兜底触发（`timer::check_expired`，`TxBuffer` 已在用）就能把延迟钉死。
  纯软件、不动协议。**这是串口路径上唯一还能明显挪动延迟的旋钮。**
- UART 内核时钟选的是 **HSI 64 MHz**（`RCC.USART16CLockSelection` /
  `USART234578CLockSelection`，在各口的 MspInit 里生效）。改成 D2PCLK1/2（137.5 MHz，
  由 24 MHz 晶振经 PLL1 得到）能把精度从 RC 的百分级提到 ppm 级——**只有精度价值，没有
  速度价值**（5.1 已证明 4 Mbaud 就远超 USB 能搬走的量）。必须走 CubeMX：每个
  `HAL_UART_MspInit` 都会把时钟源写回 HSI，app 代码改不动。
- `uart.hpp:49-50` 的 `kBrrMin`/`kBrrMax` 触发 clang-tidy 的
  `readability-identifier-naming`。**`firmware/mc02/` 整个不在 `.scripts/lint-targets.yml`
  里**，所以门禁从未覆盖过这块板；要加进去还得先补一份 `firmware/mc02/.clangd`。

**明确不要做**

| | 理由 |
|---|---|
| DMA FIFO / burst | 破坏 NDTR 派生写指针，见第 2 章 |
| 把 RX 改回中断驱动 | 刚从反方向优化过来 |
| USB High-Speed | H723 无 `USBPHYC`，须外挂 ULPI PHY，硬件死路 [实测] |
| 硬件 SHA256 / CRC / RNG | H723 无 HASH/CRYP；协议无校验和；nonce 由主机下发 |
| FLASH 预取 | H7 的 `FLASH_ACR` 只有 `LATENCY` 和 `WRHIGHFREQ`，没有预取位 |

**RS-485**：USART2 / USART3 是丝印 UART2 / UART3，DataId 是 `kUart2` / `kUart3`，由 `libhcs_APP_RS485_ENABLE` 控制（默认 ON，和 IMU 同一类开关）。这一轮重构测量不包括它们；半双工换向与 SRAM 代价见 [README.md](README.md) 「USART2 / USART3」一节。
