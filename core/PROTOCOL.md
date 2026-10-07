# libhcs 线协议规格：CAN 记录流与 EP0 配置通道

> **文档类型**：现行规范（全仓库共享线协议）
> **适用范围**：core 记录流（上行/下行 USB 批量流）+ EP0 配置通道，全部板卡与主机 SDK
> **状态**：现行有效
> **相关文档**：[README.md](../README.md)（上手） · [DMTOOL_PROTOCOL.md](../firmware/hpm_board/DMTOOL_PROTOCOL.md)（DMTool 桥私有协议，独立记录流） · 代码即规范的落点：[protocol.hpp](src/protocol/protocol.hpp) · [vendor_control.hpp](include/libhcs/protocol/vendor_control.hpp) · [can_dlc.hpp](include/libhcs/protocol/can_dlc.hpp)

## 摘要

本文档给出 CAN 帧在记录流中的位布局（含 2026-09-20 引入的 CAN-FD 长帧编码）、
DLC↔字节数的唯一映射表、主机/固件如何经 EP0 协商能力，以及 bxCAN（无 FD）板卡
的部署约束。改线协议前先读第 2、3 节；上新板或混布总线前读第 5 节。

## 1. 记录流概览

一条 USB 批量流承载定界记录；每条记录以 1 字节字段头开始（位 0-3 是字段 ID，
`kCan0..kCan3` 等高 4 位字段 ID 走两字节扩展字段头）。CAN 记录 = 字段头 +
CAN 帧头（标准 3 字节 / 扩展 6 字节）+ 负载 + 可选 3 字节时间戳（小端，见 2.5）。
编码与解码的**代码即规范**：`core/src/protocol/serializer.hpp`（上行编码）、
`core/src/protocol/deserializer.cpp`（下行解码）、`core/src/protocol/protocol.hpp`
（位定义）。

### 1.1 GPIO 记录（v12，2026-10-05）

GPIO 排针是**一路口**（`DataId::kGpio`），引脚是这路口上的**线**（线号 0..7，丝印 PWM(n+1)），
与 CAN 总线上的电机 id 是同一种关系：子地址，不是口。逐线声明（清单一线一项），EP0 按
`wIndex = kGpio | 线号 << 8` 寻址一根线；记录流里一条 GPIO 记录的头是 2 字节，上下行同一种
读法：

```
字节0: [位0-3 字段ID kGpio = 1][位4-6 记录类型][位7 带时间戳]
字节1: [线号]
```

| 记录类型 | 方向 | 含义 | 负载 | 总长 |
|---|---|---|---|---|
| `kDigitalLow` / `kDigitalHigh` | 下行 | 写输出电平 | 无 | 2 |
| `kDigitalLow` / `kDigitalHigh` | 上行 | 输入样本；带时间戳位时跟 4 字节 quarter-us 时间戳 | 0 / 4 字节 | 2 / 6 |
| `kAnalog` | 下行 | 写 PWM 占空比 | 2 字节，0..65535 | 4 |
| `kRead` | 下行 | 让输入线立刻采一次 | 无 | 2 |

板上没有的线号由板子丢弃。v12 当天曾用过 `kGpio` + `kGpioOther` 两个字段 ID 把电平记录压到
1 字节，代价是多占一个字段 ID、反序列化器要按方向解读；用户 2026-10-05 选了更简单的这一版，
字段 ID 15 留给以后的口。

EP0 一侧：口清单里 GPIO 口只有一行，能力字节是线数；逐线能力（`kGpioCap*`）是板子的接线
事实，只在板上校验，做不到即拒绝并报出缺的位。清单项与清单结果项都带 `line` 字段（别的
类型恒为 0）。

线怎么工作（输出 / 输入、上下拉、周期采样、边沿、时间戳）**不在记录流里**：它是 EP0
清单里这根线的声明（`GpioConfigPayload`），板子按这根线的能力校验，做不到即带原因拒绝。
没声明的线不启动；写给没声明（或方向不符）的线的记录由板子丢弃，
主机板类在发送入口先抛 `UndeclaredChannel`。

### 1.2 蜂鸣器记录（v12，2026-10-05）

板载蜂鸣器是一路口（`DataId::kBuzzer = 15`，`PortKind::kBuzzer`），目前只有 mc02 有（PB15，
TIM12 CH2）。清单项不带参数（设置全零），声明了才启动定时器通道且从静音开始；没声明不碰。
记录只有下行一种，4 字节：

```
字节0: [位0-3 字段ID kBuzzer = 15][位4-7 保留，必须为 0]
字节1-2: 频率 Hz（小端）      字节3: 音量 0..255（255 = 50% 占空比）
```

频率或音量为 0 即静音；低于 100 Hz 也静音。旋律的节拍由主机掌握，每个音符一条记录，板上
只写定时器寄存器。`kGetPortConfig`（wIndex = `kBuzzer`）回读此刻在放的音（从 ARR/CCR 重建）。
会话结束、清单里去掉它、新会话接手都静音。

### 1.3 会话记录与运行时状态 kPortStatus（v16，2026-10-06）

`kSession`（字段号 14）是链路自己的字段：一条会话记录 = 5 字节头（紧凑字段头 +
`SessionHeader`：4 位类型 + 32 位 nonce，`core/src/protocol/protocol.hpp`）。类型
（`data::SessionType`）与方向：

| 类型 | 方向 | 载荷 | 用途 |
|---|---|---|---|
| `kStart` / `kStartAck` / `kKeepalive` / `kKeepaliveAck` | 双向 | 无 | 会话握手与租约（`core/src/link/session.hpp`）|
| `kTimeAnchor` | 下行 | 8 字节 | 共享时间基准（`firmware/hpm_board/SOF_TIMEBASE.md`）|
| `kTimeStatus` | 上行 | 34 字节 | 对时状态回报 |
| `kPulseSchedule` | 下行 | 8 字节 | 硬件脉冲交换（`app/src/sync/pulse.hpp`）|
| `kPulseReport` | 上行 | 21 字节 | 脉冲捕获回报 |
| `kPortStatus` | 仅上行 | 1 + 正文 ≤ 16 字节 | 一路口（含链路本身）的运行时状态（v16）|

编号 9 是 v15 的 `kStreamError`（下行流错误汇报），v16 并入 `kPortStatus`（链路这个"口"的
状态），编号不复用。

带载荷的类型**只有指纹同意的对端才能安全发出**：接收方不认识的类型没法跳过（载荷长度
不可知），会把载荷当下一条记录解开。这是 EP0 `kVersion` 指纹把关的线格式之一
（`data::kSessionWireVersion`）。

下行错误的**两类结局**（`Deserializer`，编进 `error_callback(field, reason)`）：
字节完整但板子拒收（本板没有这个口、方向不符）——**只丢那一条**，同一批里排在后面的
记录照常交付；格式坏（截断、头部非法、保留编码）或字段号不认识——定界已失，本次传输
剩余的全部字节丢弃，到传输边界恢复。两类都记进同一份账本（`core/src/link/downlink_errors.hpp`，
三块板共用），取舍与实测见 [DOWNLINK_ERRORS.md](DOWNLINK_ERRORS.md)。账本就是链路本身
（`DataId::kSession`）的运行时状态来源，随下面的 `kPortStatus` 上报。

**端口运行时状态只走数据流，不走 EP0**（v16 起 `kGetPortStatus` 退役）：EP0 是配置面
（握手、声明、查清单）。`kPortStatus` 挂在 keepalive 应答之后（应答本身格式不动），先是链路
本身（`kSession`），再是每路**在跑（被声明）**的 CAN / 串口，一口一条，**只发状态变了的口**；每个 kStart 之后的第一轮
每个在跑的口都发一条，作主机的基线（主机从不清快照）。每条都是全量快照，不是增量。

载荷 = 1 字节头 + 正文，合计不超过 16 字节（`core/src/protocol/protocol.hpp`）：

| 头字节 位 | 宽度 | 字段 |
|---|---|---|
| 0 | 4 | 口的 `DataId`（紧凑字段号）|
| 4 | 4 | 正文长度（0..15 字节）|

正文的布局由口的种类决定，种类由 `DataId` 给出（视图的 `is_for()`：链路 14 / CAN 2..5 /
UART 6..12；全部种类登记在 `data::PortStatusVariant`）。字段只追加：接收方按自己认识的前缀解，比已知正文长的部分、
不认识的种类按长度跳过，定界不丢。计数都是上电以来的自由计数，**按 2^16 回绕**，主机取相邻
两次之差（模 2^16；板子每轮计数一动就报，一轮里动不了 65536 次）。语义见
`data::LinkStatusView` / `data::CanStatusView` / `data::UartStatusView`。

| 链路正文（6 字节）位 | 宽度 | 字段 |
|---|---|---|
| 0 | 16 | `downlink_errors`：上电以来没能交付的下行记录 |
| 16 | 8 | `last_field`：最近一次错误的字段号 |
| 24 | 8 | `last_reason`：`data::DownlinkError`——0 格式坏 / 1 字段号不认识 / 2 拒收 |
| 32 | 16 | `last_transfer`：最近一次错误发生在本会话第几次下行传输（1 起）|

| CAN 正文（11 字节）位 | 宽度 | 字段 |
|---|---|---|
| 0 | 8 | `tec`：ECR 发送错误计数 |
| 8 | 7 | `rec`：ECR 接收错误计数 |
| 15 | 3 | `flags`：`data::CanStateFlags`——error-passive / warning / bus-off |
| 18 | 3 | `last_error`：`data::CanLastError`，最近一次**真实**总线错误（驱动锁存，见下）|
| 21 | 3 | `data_last_error`：同上，CAN-FD 数据相位 |
| 24 | 16 | `tx_cancelled`：单发作废的帧（三块板都单发，发送槽复用时清点）|
| 40 | 16 | `tx_dropped`：板子丢的下行帧（发送队列满）|
| 56 | 16 | `rx_dropped`：板子丢的上行帧（上行缓冲满）|
| 72 | 16 | `rx_lost`：接收 FIFO 溢出次数（帧在控制器里就丢了，一次至少一帧）|

| 串口正文（14 字节）位 | 宽度 | 字段 |
|---|---|---|
| 0 / 16 / 32 / 48 | 16 ×4 | `overrun` / `parity` / `framing` / `noise`：接收错误 |
| 64 | 16 | `unattributed`：种类未知的接收错误（只有 hpm，见 [PORT_STATUS.md](PORT_STATUS.md) 第 3 节）|
| 80 | 16 | `tx_dropped`：板子丢的下行记录（发送环满）|
| 96 | 16 | `rx_dropped`：板子丢的上行块（上行缓冲满）|

一条记录连会话头合计：链路 12 字节、CAN 17 字节、串口 20 字节。

`PSR.LEC` 读后自清（bxCAN 的 `ESR.LEC` 成功收发后复位），所以驱动把读到的真实错误
（`kStuff`..`kCrc`）锁存（`firmware/common/app/src/utility/latched_bus_error.hpp`），报的是
最近一次；同一种错误重复出现不算变化，那时变的是 TEC/REC。串口
错误一个计数是什么因板而异：STM32（c_board、mc02）每次 HAL 错误回调记一次；HPM 不开线路
状态中断，每轮读一次 `LSR`，见到错误位记一次（`LBREAK` 计入 framing，没有 noise）。
主机侧无锁快照：`Handler::status<View>(port)` / `port_status(port)`（板卡类的口句柄
`status()`）。取舍见 [PORT_STATUS.md](PORT_STATUS.md)。

## 2. CAN 帧头位布局

### 2.1 标准帧头（3 字节，经典 ID）

| 位 | 字段 | 说明 |
|---|---|---|
| 0-3 | `FieldHeader.Id` | 字段 ID，不是空闲位（历史上 HasTimestamp 曾放位 3，因与 ID 冲突搬走） |
| 4 | `IsLongFrame` | 1 = 负载为 CAN-FD 长帧（12-64 字节），DLC 字段切换语义（见 2.3） |
| 5 | `IsExtendedCanId` | 本布局恒 0 |
| 6 | `IsRemoteTransmission` | 远程帧；远程帧无负载 |
| 7 | `HasCanData` | 0 = 无负载（远程帧） |
| 8-18 | `CanId`（11 位） | |
| 19 | `HasTimestamp` | 负载后跟 3 字节时间戳（`SofStamp`，见 2.5） |
| 20 | 预留 | 标准帧头最后一个空闲位，**留作未来 per-frame 标志（BRS/ESI）**，接收方忽略 |
| 21-23 | `DataLengthCode` | 语义随 `IsLongFrame` 切换，见 2.3 |

### 2.2 扩展帧头（6 字节，29 位 ID）

与 2.1 共用位 0-7；位 8-36 为 29 位 `CanId`，位 37-39 为 `DataLengthCode`，
位 40 为 `HasTimestamp`，**位 41-47 预留**。长帧在扩展头里使用与标准头相同的
DLC 编码——不另用 7 个空闲位存字节数，保持一份解码器、一张表。

### 2.3 `DataLengthCode` 的双语义（长帧扩展，2026-09-20）

| `IsLongFrame` | DLC 字段语义 | 取值 |
|---|---|---|
| 0（短帧） | 负载字节数 − 1 | 0-7 → 1-8 字节；经典帧与 FD 短帧编码相同 |
| 1（长帧） | 线上 DLC − 9 | 0-6 → DLC 9-15 → 12/16/20/24/32/48/64 字节；**7 保留** |

设计动机：FD 长度 12-64 塞不进"字节数−1"的 3 位，而 DLC−9 恰好放下且剩一个
保留码。该位复用的是 2026-09-12 退役的 `IsFdCan` 槽位——经典 vs FD 仍是**总线
属性**（EP0 端口清单 `kGetPortList` 的逐口状态位；v9 及更早是 `kGetInterface.can_fd_mask`），
per-frame 只表达"长度字段怎么解"。注意：
**位 4 退役时"接收方可忽略"的规则随之作废**——忽略位 4 会把长帧长度解错。

### 2.4 无效编码（两个方向都定义为保留，收到即丢弃并进 discard 模式）

- `IsLongFrame = 1` 且 `IsRemoteTransmission = 1`（ISO CAN-FD 无远程帧）；
- 长帧 DLC 字段 = 7；
- 长帧负载字节数不在 DLC 表内（9-11、13-15、17-19 等，见第 3 节）。

经典帧 DLC 9-15 仍按 CAN 规范钳为 8 字节（板端归一化）。

### 2.5 时间戳：共享微帧轴上的位置（2026-10-03；2026-10-05 改 10 + 14 位）

`HasTimestamp` 置位时，负载之后是 3 字节小端的 `libhcs::time::SofStamp`
（`core/include/libhcs/time/sof_stamp.hpp`）：这一帧在总线上开始的时刻，表示成共享
USB 微帧轴上的位置。

| 位 | 含义 |
|---|---|
| 0-13 | 微帧内的小数，1/16384 微帧（7.6 ns） |
| 14-23 | 微帧号的低 10 位，即设备控制器帧号寄存器（FRINDEX）的低 10 位 |

- **只有这一种含义。** 给不出这条轴上位置的板（固件没开时间基准、没有硬件捕获通路、
  这一帧定不了位）不带时间戳，而不是带一个板上本地时钟的读数。
- **跨度 128 ms，由接收方还原。** 主机用自己对"现在"的估计补全高位
  （`libhcs::time::unwrap()`，窗口是"现在"之前 96 ms 到之后 32 ms）；换成本机时钟用
  `libhcs::host::time::AxisMap::time_of()`。回放日志时"现在"要取这一帧到达的时刻。
- **仅上行。** 下行帧不带。哪些板会带：主机的清单要了时间基准（`kManifestFlagTimeSync`，
  第 4 节；v14 起不是编译开关）的 HPM5321 / HPM6E8Y 与 mc02（另要 .ioc 的 TIM3 配置，见
  `firmware/mc02/app/src/sync/sof_capture.hpp`）。c_board 不报这项能力。
- 历史：2026-10-03 之前这里是 4 字节微秒数，取自板上自由运行的 PTPC，主机无法把它放到
  任何可比的时间轴上。2026-10-03 至 10-05 是同样 3 字节、14 位微帧号 + 10 位小数
  （122 ns）：那一档量化本身（35 ns RMS）比硬件锁存的误差大一个数量级，于是把小数点
  左移 4 位（同日先写过 11 + 13 位、没刷过板，量化 4.4 ns RMS 仍是最大的一项）。每次改动都不兼容，由 EP0 指纹里的 `kCanStampSemantics` 把关（第 4 节）。

## 3. DLC ↔ 字节数映射（唯一权威表）

`core/include/libhcs/protocol/can_dlc.hpp` 的 `payload_length()` /
`dlc_from_payload_len()`。记录流两个方向、全部板卡、主机 SDK 与 DMTool 桥
（原表自 dm_protocol.hpp 收敛至此）共用：

| 线上 DLC | 0-8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 |
|---|---|---|---|---|---|---|---|---|
| 字节数 | 0-8 | 12 | 16 | 20 | 24 | 32 | 48 | 64 |

不在表内的字节数没有线上表达，编码端必须拒绝（`kDlcInvalid`）。

## 4. 能力协商与兼容矩阵

协商机制 = EP0 端口清单（`kGetPortList`，纯读）的**线格式指纹** `kVersion`（编译期折叠，
改线格式自动变化，无手工版本号）+ 逐口能力位（CAN 口的 `kCanCapFdLongFrames` 等，
`spec/port.hpp` 编码）。**wIndex 的语义是 DataId（丝印号）**，声明是**一次事务**
（`kApplyManifest` 两阶段提交：校验判完一切可预见的拒绝、不碰硬件；通过后先从 DMTool/CDC
手里接过板子再动端口，全部生效才落定归 libhcs，应用中途失败则所有口挂起、板子还给接手前的
主人；`kGetManifestResult` 的 `outcome` 区分这三种结局）——这是 v10（2026-10-04）的形状，
v11（同日）把 GPIO 引脚也纳入其中（`PortKind::kGpio`，见 1.1 节，`kMaxPorts` 10 → 14）；
v14（2026-10-05）把共享时间基准也纳入其中：它是整板的事、不是口，板子在 `kGetPortList` 的
`board_caps` 报 `kBoardCapTimeSync`，主机在清单头的 `flags` 要 `kManifestFlagTimeSync`，板子在
同一个事务里、端口应用之前打开（开不起来整份回滚），没要它的清单把它关掉；
v9 及更早的按类型请求（`kGetInterface`/`kGet/SetCanConfig`/`kGet/SetUartConfig`/
`kGetCanStatus`/`kSetImuConfig`，wIndex 是"第几路"）已退役，编号不复用，老对端拿
STALL。板子有哪些口、每个口能做什么只有一处事实：固件按上电时初始化了哪些驱动报告的口清单
（能力位出自驱动类型的 `kPortCapabilities`）；`spec/<board>/ports.hpp` 只给口起名字、定
类型，主机接线表与固件绑定共用这份名字。逐请求的语义在
[vendor_control.hpp](include/libhcs/protocol/vendor_control.hpp) 的版本历史与
[payload 注释](include/libhcs/protocol/vendor_control.hpp)里，此处只留兼容矩阵。规则：

- **固件**：只在总线处于 FD 模式时编码/接收长帧（总线模式它自己知道）；
  下行长帧遇经典总线丢弃（DMTool 可在会话中切模式，存在竞态窗口）。
- **主机 SDK**：板类在 `can_transmit()` 门禁——负载 > 8 字节要求该口的能力位有
  `kCanCapFdLongFrames`（板子在口清单里报的能力，能力出自固件的驱动类型）**且**目标总线此刻在 FD（运行期快照）。
  不满足即抛异常，而不是发出板子无法承载的记录。

| 组合 | 行为 |
|---|---|
| 新主机 + 新固件（广告位） | FD 总线上长帧双向可用 |
| 新主机 + 老固件 | 指纹不匹配，构造时抛异常要求重烧 [实测，v1→v2 先例] |
| 老主机 + 新固件 | 老 SDK 指纹校验失败，连接阶段拒绝 [实测，v1→v2 先例] |

结论：老对端**不会静默失步**（那会把记录流撕裂），而是连接时干净拒绝——
代价是本改动上线后**全部板卡需重烧固件**才能配新 SDK。

## 5. 板卡支持矩阵与 bxCAN 约束

| 板卡 | 控制器 | 硬件能力 | `kCanCapFdLongFrames`（驱动类型的 `kPortCapabilities`，经口清单上报） | 说明 |
|---|---|---|---|---|
| `hpm_board`（6E8Y/5321） | MCAN | RX 元素与 TX 缓冲已配 64 字节 | ✓ 广告 | FD 模式由端口表初定，DMTool 可运行时切换 [实测] |
| `mc02`（H723 FDCAN） | FDCAN | TX 侧可 64 字节，RX 元素仍 8 字节 | ✗ | RX 扩容前不广告，避免"能发不能收"的不对称能力 |
| `c_board`（F407） | bxCAN | 无 CAN-FD | ✗（物理不可能） | 永远无长帧 |

### 5.1 bxCAN 与 FD 总线不能共存（部署硬约束）

bxCAN 在普通模式下收到 FD 帧会把 FDF 位当作保留位的显性电平 → form error，
回发错误帧；BRS 的变速段它无法跟踪 [RM：STM32 bxCAN 参考手册]。即一块 bxCAN
节点足以拖垮整条 FD 总线的通信。因此：

1. **FD 帧只允许出现在全部节点都 FD-capable 的总线段**。给 c_board 所在总线
   上 FD 通信不是软件能解决的，是拓扑问题。
2. 混布系统的推荐做法与本仓库架构一致：**板子是纯桥，路由在主机**——c_board
   挂经典段，mc02/hpm 挂 FD 段，两块板各自对主机，跨段转发由主机 SDK 完成。
   硬件 CAN-FD 网关是替代项。
3. 若 c_board 只需**旁听**混布总线：bxCAN 静默模式（listen-only）不 ACK、不发
   错误帧，不会破坏总线 [RM]；但它只能正确收到经典帧，FD 帧不可收，且错误
   计数会累积。仅作诊断用途，不作控制用途。
4. FD 总线上线核查单：ISO CAN-FD 与非 ISO 全总线一致二选一、数据相位波特率
   与采样点、TDC（收发器延迟补偿）。见 `firmware/hpm_board/AGENTS.md`。

## 6. 测试矩阵

已有：`host/examples/can_frame_type_test.cpp`（std 8B / ext 29bit / RTR 字段
验证）、`host/examples/usb_canfd_stress.cpp`（FD 总线 8 字节负载压力）。
长帧上板需补的用例 [推断，未上板]：

- 标准头长帧 12/64 字节、扩展头长帧 64 字节、长帧 + 时间戳；
- 经典帧 DLC 9-15 钳 8 回归；FD 总线上 ≤8 字节短帧回归（编码不变）；
- 下行长帧；总线运行中被 DMTool 切回经典后在途长帧被丢弃（竞态窗口）；
- 保留组合（长帧+远程、长帧 DLC=7）进 discard 模式；
- 指纹不匹配的老固件拒绝连接。

---

改动记录：2026-09-20 位 4 复用为 `IsLongFrame`（原退役 `IsFdCan` 槽位）、
DLC 字段双语义、`kCapCanFdLongFrames` 能力位、DLC 表收敛至
[can_dlc.hpp](include/libhcs/protocol/can_dlc.hpp)。2026-10-06 新增 1.3 节：
会话记录一览与 `kStreamError`（下行流错误的拒收/丢弃纪律与回报）；同日 v16 把端口运行时
状态从 EP0 `kGetPortStatus` 移到会话记录 `kPortStatus`，`kStreamError` 并入其中作链路的状态；
同日 v17（EP0 `0x5bec`）串口正文加 `unattributed`，12 → 14 字节。
