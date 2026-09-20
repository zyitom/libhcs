# DMTool 兼容：达妙 USB2FDCAN 协议与本板实现

> **文档类型**：硬件参考
> **适用范围**：`firmware/hpm_board/` 的 HPM5321 应用（`app/src/dmtool/`）；host SDK 的 USB 身份与数据管道布局（`core/include/libhcs/protocol/usb_identity.hpp`）
> **状态**：现行有效（协议逆向完成，未上板；待办见第 7 节）
> **相关文档**：[AGENTS.md](AGENTS.md)（命令与约束） · [USB_OPTIMIZATION_LOG.md](USB_OPTIMIZATION_LOG.md) 4.5 节（空闲端点的代价） · [仓库根 AGENTS.md](../../AGENTS.md)「架构边界」

## 摘要

HPM5321 应用在 libhcs 之外还扮演一块达妙 USB2FDCAN 适配器：不跑 HCS 时，用达妙官方上位机
DMTool 2.1.6.7 可以直接打开本板收发 CAN、看硬件时间戳、经 CDC 串口读写板上 UART。本文记录
从 DMTool 二进制逆向出的线协议（每条带函数地址作出处）、板端实现的取舍、对 libhcs 转发路径
的逐指令核对，以及尚待真适配器 / 上板确认的项。命令与约束以 [AGENTS.md](AGENTS.md) 为准。

## 本文导航

- 第 1 节：DMTool 怎么认设备、怎么打开；本板的 USB 接口与端点布局。
- 第 2 节：命令通道（帧格式、命令字、本板对每条命令的应答）。
- 第 3 节：CAN 发送帧、接收 / 回显记录流，以及记录流在 USB 上的两条硬规则。
- 第 4 节：板端取舍——哪些只转发、哪些如实回失败、libhcs 如何优先。
- 第 5 节：对 libhcs 的代价，对照无 DMTool 支持镜像的反汇编。
- 第 6 节：与此前一版逆向清单的出入（那版有几处会让实现直接跑不通）。
- 第 7 节：待真适配器对拍与上板确认的项。
- 第 8 节：主机侧配置（udev）与怎么复现逆向。

## 1. 设备识别与 USB 布局

**DMTool 只按 VID:PID 认设备，不看任何字符串。** `fdcan_protocol::IsFdcanDevice`（@0x143ac0）
逐对比对静态表 `FDCAN_DEVICE_ID`（@0x2c1d60，4 对）：`34B7:6877`、`34B7:6632`、`34B7:4543`、
`1D50:606F`。本板用第一对 `0x34B7:0x6877`。`OpenFdcanDevice`（@0x148d60）依次 claim 接口
0、1、2，随后发 `START_CAP(通道)`；主窗口再起两个接收线程并读版本。`[逆向 DMTool 2.1.6.7]`

| 接口 | 端点 | 方向 | 用途 | DMTool 侧行为（出处） |
|---|---|---|---|---|
| 0 | `0x01` | OUT | 心跳 | `USB_CMD_HEART` @0x143950：24 字节，超时 10 ms，设备不回应 |
| 0 | `0x81` | IN | CAN 接收记录流 | `can_rec_thread_func` @0x14b0c0：32 KB 同步读，超时 1 s |
| 1 | `0x02` | OUT | 命令 | `usb_control_send` @0x1455f0：**先 `clear_halt(0x02)`**，超时 100 ms |
| 1 | `0x82` | IN | 命令应答 | `usb_control_ack` @0x145de0：**先 `clear_halt(0x82)`**，10 KB 读，超时 500 ms |
| 2 | `0x03` | OUT | CAN 发送 | `FdcanDeviceSend` @0x144000：22 字节头 + 负载，超时 10 ms |
| 2 | `0x83` | IN | CAN 发送回显记录流 | `can_sent_thread_func` @0x14b210：同 `0x81` |
| 3 | `0x04` / `0x84` | OUT / IN | libhcs 字节流 | —（DMTool 不碰） |
| 4、5 | `0x86` / `0x05`、`0x85` | IN / OUT、IN | CDC 串口桥（通知 / 数据） | DMTool 串口助手按普通 CDC 打开 |
| 6 | — | — | DFU runtime | — |

- 接口与端点的分组 DMTool 不依赖：libusb 只要求端点属于已 claim 的接口，0/1/2 全被 claim。
- libhcs 主机靠产品串 `HCS Agent v<版本>` 认板。真达妙适配器与本板 VID:PID 相同，扫描器
  对这个身份即使开了 `dangerously_skip_version_checks` 也要求产品串以 `HCS Agent v` 开头
  （`host/src/transport/usb/device_scanner.hpp`），测量工具不会误开真适配器。
- DMTool 每条命令前后的 `clear_halt` 不会打乱 data toggle：本仓库 TinyUSB 的
  `usbd_edpt_clear_stall` 无论端点是否 stall 都调 `dcd_edpt_clear_stall` 复位 toggle，与
  主机侧的复位对得上。`[源码]`

## 2. 命令通道（`0x02` / `0x82`）

### 2.1 帧格式

```text
命令  A5 | CMD | LEN(LE16) | PAYLOAD | CRC16(LE) | 5A
应答  A5 | CMD | STATUS | LEN(LE16) | PAYLOAD | CRC16(LE) | 5A
```

- CRC 是 **CRC-16/ARC**（反射多项式 0xA001、初值 0、无终值异或；`CRC16_IBM` @0x143890），
  覆盖 CMD 起至负载末尾，应答还含 STATUS 与 LEN。不是 Modbus（初值 0xFFFF）。
- 应答校验（@0x145fb3 起）：首字节 A5、**本次传输最后一个字节** 5A、CRC、CMD 回显一致；
  STATUS 为 0 即成功，非 0 报错。所以一条应答必须独占一次 IN 传输，且要以短包结束。
- 心跳走 `0x01`：`A5 10 11 00 | FF×17 | CRC | 5A`，与 `0x10`（保存参数）同码但不同端点。

### 2.2 命令字与本板应答

命令字取自各 `USB_CMD_*` 包装函数传给 `usb_contorl_transfer` 的立即数。

| CMD | 名称 | 负载 | 本板应答 |
|---|---|---|---|
| `0x00` | START_CAP | 1 字节通道 | 通道存在且没有 libhcs 会话时开始采集，成功；否则失败 |
| `0x01` | STOP_CAP | 1 字节通道 | 停止采集，成功（DMTool 切通道时先停另一路，单 CAN 板上停通道 1 也回成功） |
| `0x02` | IAP_PACK | 4100 字节 | **失败**（不做固件升级；负载超解析器上限，只校验不留存） |
| `0x03` | IAP_END | 0 | **失败** |
| `0x04` | START_TEST | 0 | **失败**（设备端自测不做） |
| `0x05` | SETUP_BUARD | 10 字节，见 2.3 | **按命令参数重配控制器位时序并切换 FD/经典模式**（2026-09-19 上板实测，见 6.1） |
| `0x06` | 读版本 | 0 | `HCS Agent v<版本>` + NUL（`CheckIsBoot` @0x146f10 对它 strlen 后找 "boot"） |
| `0x09` | GetUUID | 0 | OTP UUID 16 字节 |
| `0x0A` | 写 SN | 11 字节 | **失败**（序列号来自 OTP，只读） |
| `0x0B` | 读 SN | — | USB 序列号串 + NUL（DMTool 界面的读 SN 按钮是空函数） |
| `0x0C` | 退出 bootloader | 13 字节 | 成功（应用本就不在 bootloader） |
| `0x0D` | GetBaudRate | 1 字节通道 | 10 字节，同 2.3 布局，**回报控制器里实际的位时序** |
| `0x0E` | 恢复出厂 | 15 字节 | 成功（没有可恢复的参数，现状即出厂状态） |
| `0x0F` | 停止周期发送 | 1 字节通道 | 成功（设备端从不重复发送） |
| `0x10` | 保存参数 | — | 成功（总线参数是编译期事实，无可保存） |
| 其余 | — | — | 失败 |

### 2.3 SETUP_BUARD / GetBaudRate 负载

`USB_CMD_SETUP_BUARD` @0x147500 的参数装配（@0x1476ce 起）与 `on_fdcanGetBaudRate_clicked`
@0x90df0 的解析一致：

| 偏移 | 含义 |
|---|---|
| 0 | 通道 |
| 1 | 帧类型：0 = CAN，非 0 = CANFD（GetBaudRate 应答里据此 `setCANType`） |
| 2 / 3 / 4 / 5 | 仲裁段 seg1 / seg2 / sjw / 分频 |
| 6 / 7 / 8 / 9 | 数据段 seg1 / seg2 / sjw / 分频 |

速率 = 80 MHz / 分频 / (seg1 + seg2 + 1)（DMTool 日志按 `0x4C4B400` 换算）。sjw 两字节的
含义为推断 `[推断]`。本板 CAN 位时钟同为 80 MHz（PLL1 / 10），所以 GetBaudRate 可以把
`NBTP` / `DBTP` 原样折成 DMTool 的字段；位时钟不是整 80 MHz 或字段超出一字节时回失败。

## 3. CAN 数据面

### 3.1 发送帧（`0x03` OUT）

`CustomCDC::fillFDCANFrame` @0x137840 装配 22 字节头，`FdcanDeviceSend` 发
`22 + GetLenFromDlc(DLC)` 字节，一帧一次传输：

| 偏移 | 含义 |
|---|---|
| 0..3 | CAN ID（LE，低 29 位）；字节 3 位 6 = 扩展帧，位 7 = 远程帧 |
| 4 | 高 4 位 DLC；位 0 = FD，位 1 = BRS，位 2 = ID 自增，位 3 = 数据自增 |
| 5 | 通道（界面的"通道1/通道2"是 bool，即 0 / 1） |
| 6..0xD | 主机侧状态，设备不用 |
| 0xE..0x11 | 重复发送间隔（LE32） |
| 0x12..0x15 | 发送次数（LE32；普通发送为 1） |
| 0x16.. | 负载 |

### 3.2 接收 / 回显记录流（`0x81` / `0x83` IN）

两条流格式相同（`can_rec_unpack_thread_func` @0x14c930、`can_sent_unpack_thread_func`
@0x14bf80），一次传输里可以连续放多条记录，DMTool 按 DLC 逐条推进；显示在
`MainWindow::insert_item` @0xd0d60。

| 偏移 | 含义 |
|---|---|
| 0..3 | CAN ID（LE，低 29 位）；位 30 = 扩展帧，位 31 = 远程帧 |
| 4..0xB | 64 位纳秒时间戳（LE；DMTool 除以 1e9 显示为 9 位小数的秒） |
| 0xC | 0 |
| 0xD | 高 4 位 DLC；位 0 = FD，位 1 = 方向（0 接收 / 1 发送），位 2 = BRS，**位 3 = 发送成功**（发送方向的成败就靠这一位） |
| 0xE | 状态字节：接收记录 0x00；发送回显**成功 = 0x00（成败由 [0xD] 位 3 表达）、失败 = 0x02**；0xFF = 错误帧（进错误队列）。`handleCANFIFO` @0xce5f9 查表 0x18b0f0 的 0x12/0x02 是原始适配器固件侧的状态枚举，**不是**记录流该填的值——成功回显填 0x12 的版本，板端实测每一行都显示成"发送失败" [实测]。更早恒写 0 且不置位 3 的版本显示"心跳失败"（状态 0 且位 3 未置位的组合） |
| 0xF | 本板填通道号（发送表格的通道列是否读它待真机核对） |
| 0x10.. | 负载，按 DLC 表补齐（DLC 9..15 → 12/16/20/24/32/48/64） |

接收方向一律显示"接收成功"；发送方向在 [0xE] 为 0 且位 3 置位时显示"发送成功"，否则
"发送失败"。DMTool 表格里的发送行只来自 `0x83` 回显，本地不自己添行。

### 3.3 记录流在 USB 上的两条硬规则

1. **不能发零长度传输。** 接收线程在 `libusb_bulk_transfer` 成功但长度为 0 时置
   `usb_err_happen`，当作 USB 故障处理（@0x14b10e）。
2. **每次传输必须以短包结束。** 主机是 32 KB 的同步读：以满包结尾的传输要等下一个包或 1 s
   超时，而超时返回的数据 DMTool 直接丢弃。本板每次传输最多 `包长 - 1` 字节（高速 511），
   8 字节帧的记录 24 字节，一次最多 21 条。

## 4. 板端实现取舍

板子是纯转发桥（[仓库根 AGENTS.md](../../AGENTS.md)「架构边界」）。DMTool 里凡是要板子
自己做事或改持久状态的功能，一律不做并如实回失败，只保留转发那一部分：

- **设备端重复发送、ID / 数据自增不实现。** 发送帧的"次数 / 间隔"字段被忽略，每个请求只发
  一次；DMTool 自己的主机定时发送（`onFDCANPeriodicSendTimerTimeout`）不受影响。
- **SETUP_BUARD 按命令重配控制器。** 最初的实现只核对（任何非编译期速率都回失败，
  上板实测 DMTool 报配置失败），已改为真正应用：走 SDK 低级位时序路径
  （`use_lowlevel_timing_setting`，DMTool 的 seg1/seg2/分频本就是 TQ 语义，照抄
  保证界面显示的采样点与硬件一致），fd=0 切经典模式、fd=1 切 FD。数据段 TDC:
  分频 1 走 SDK 自动 TDCO（seg1+1 mtq，实测可用），分频 2 显式
  `ssp_offset = 分频 × (seg1 + 1 + seg2/2)`（5M 预设下 = 14 mtq，与自动值相同；
  采样点位置公式 12 mtq 实测早于收发器环路延迟，帧全部 bit error 丢弃）。
  参数非法时 SDK 拒绝，恢复编译期时序并回失败。libhcs 的默认配置不受影响 --
  那是本仿真模块接管期间的行为。
- **会话启动自动套用 1M/5M 预设。** 复位后控制器跑 SDK 求解解(分频1/seg1 69/seg2 10,
  87.5%), 不在 DMTool 预设表里 -- "更新配置"按表回查查不到就毫无反应。因此每次会话
  首次 START_CAP 前(即 DMTool 打开设备时)自动把两路套用预设(分频2/29/10 与 2/5/2),
  之后"更新配置"回读值与表项逐一命中; 用户改速率后回读的是新时序, 同样按表命中对应
  预设。libhcs 会话建立时反向还原编译期时序(87.5%), 两种模式各自自洽。
- **保存配置(0x10)只 ACK 不持久化。** 本板没有为 DMTool 提供参数存储: 设置在会话内
  保持, 断电/重启后回到编译期默认。真适配器写 flash 的行为不复刻。
- **帧类型跟总线走。** 与 libhcs 下行同一规则（`Can::handle_downlink`）：FD 端口一律发
  FD + BRS，请求里的 FD/BRS 位只是界面选择；回显报的是实际上线的帧型。负载超过 64 字节
  或 ID 越界的帧不发，回显为发送失败（MCAN RX/TX 元素已扩到 64 字节，长帧 DLC 9-15
  直传线上 DLC；经典模式下此类请求按经典帧 DLC 9-15 发出，接收方按规范只取 8 字节）。
- **时间戳**：接收帧用 MCAN TSU 在 SOF 处捕获的 PTPC0 时间戳，回显用发送时刻读 PTPC0，
  同一时基。PTPC 数字模式纳秒计数偏慢，按 `×1000 / kCanTimestampNsPerUs` 还原成真实纳秒
- **通道指示灯**: DMTool 选中采集通道(START_CAP/STOP_CAP)时, 对应 CAN 指示灯常亮
  (led.hpp `set_channel_active`); 故障灯语优先 -- 采集期间无应答(约 1 Hz 闪)/接线故障
  (约 5 Hz)/信号错误(双闪)/BUS-OFF(常亮)照常显示, 最后一次错误约 5 s 后衰减回
  "通道常亮"。
- **总线占用率是上位机软件算的**，设备端无事可做：DMTool 点占用率测量按钮时从
  界面两个波特率下拉框 `parseBaudrate` 取参考速率（@0xaae30），测量窗内按收到记录的
  DLC 累加位数（FD 连接态每帧 `DLC×8+47` bit、未连接 `+8`，TX 回显 `+66`，
  @0xc3d40 `calculateOccupancy`），除以「窗口×波特率」总容量 ×100。前提三条：
  测量按钮点过、下拉框波特率非 0、记录流里有帧（采集已开）——缺一就一直是 0.0。
  记录头里的 DLC 正确即可，无需设备端额外字段
  （与 libhcs 的微秒换算同一修正），64 位除法放在主循环做。
- **通道映射**：DMTool 通道 0 / 1 = 丝印 CAN1 / CAN2；单 CAN 板上通道 1 不存在，开始采集
  回失败。
- **libhcs 优先。** libhcs 会话一建立，采集、CDC 桥全部关闭，队列清空；会话期间
  START_CAP 回失败、DMTool 发来的 CAN 帧不上总线。HCS 退出后 DMTool 需要重新打开设备。
- **CDC 串口桥** 只在三件事同时成立时接通：串口已打开（DTR）、主机设的波特率与板上 UART
  实际速率差在 3% 以内、没有 libhcs 会话。不按主机的设置改 UART 波特率——那是 libhcs 经
  EP0 管的配置，串口工具关掉后 libhcs 必须看到原样；这条互锁也顺带把 ModemManager 一类按
  9600 / 115200 探测的程序挡在 UART 之外。DMTool 的"USB2CAN 串口款"协议
  （`CustomCDC`、0x2E88:0x4603、`55 AA` 帧）本板**不**实现，CDC 只做 UART 透明桥。
- **心跳不解析**，也不据此判断 DMTool 是否在线：会话边界由 START_CAP / STOP_CAP、总线复位
  与 libhcs 会话决定。

## 5. 对 libhcs 的代价 `[实测，2026-09-19]`

**结论：libhcs 的 CAN 收发 ISR 路径逐条指令不变；每个 bulk OUT 包多 2 条 ALU 指令，主循环
每轮多 3 条指令，均为纳秒级。USB 总线上，DMTool 不开时它的端点一次事务都没有。**

总线侧：bulk 端点只在主机挂着传输时才被主机控制器调度。2026-09-05 的分端点实验实测过这个
二值性——第二管道 0 个 URB 时包率 63.1k（无损），挂 1~16 个都是 49.2k
（[USB_OPTIMIZATION_LOG.md](USB_OPTIMIZATION_LOG.md) 4.5 节）；那次的损失来自 host SDK
常驻挂着第二管道，而 DMTool 的端点只有 DMTool 在跑时才有 URB。CDC 通知端点由 Linux
`cdc_acm` 在串口打开时（`acm_port_activate`）才提交 URB
`[推断，未在本机核对内核源码]`，且 `bInterval` 已从 `TUD_CDC_DESCRIPTOR` 写死的 1 改为
16（高速约 4 s）。

固件侧：同一版本号（`-Dlibhcs_PROJECT_VERSION=3.3.0`）分别编 HEAD（d635dd7）与本实现的
release 镜像，逐函数比对反汇编：

| 函数 | 变化 |
|---|---|
| `Can::irq_handler`、`board::can_irq_handler` | 逐条相同 |
| `Can::handle_interrupt_flags`（ILM） | libhcs 分支逐条相同（仅寄存器编号）；无会话分支由内联丢弃循环换成调用 `drain_without_session`（FLASH） |
| `Can::handle_downlink`、`drain_transmit_queue`（ILM） | 相同，仅数据地址 |
| `board::uart_irq_handler`（ILM） | 栈帧同为 64 B，libhcs 分支相同；新增指令全在无会话分支 |
| `tud_vendor_rx_cb` | libhcs 包 +2 条 ALU（实例号 0→3 要先装常数、一次寄存器 mv） |
| `App::run` | 每轮 +`lui/lbu/bnez`（`dmtool::poll()` 的闸）；1 kHz LED 块多一个判断 |
| `dcd_event_handler`（ILM） | SOF 事件遍历的类驱动 2→3 个，只在开了 SOF 中断的构建（TIME_SYNC 等）里每 125 us 多一次迭代 |
| ILM 总量 | 15728 → 15792 B |

为了做到上表，实现里有三处刻意的写法，改动时要保住：无会话分支都是对冷函数的调用
（`Can::drain_without_session`、`dmtool::uart_rx_without_session`，后者传标量而非 span，
否则编译器会把空 span 的栈实体提到会话判断之前）；`tud_vendor_rx_cb` 把 DMTool 分派放在
最前面；新增的 CAN 时钟字段以 MHz 存进 `Can` 已有的对齐空洞，`sizeof(Can)` 不变。

尚未做的：上板包率 / RTT 回归（第 7 节）。

## 6. 与此前一版逆向清单的出入

此前清单（2026-09-19 前的会话产物，未入库）有几处与二进制不符，按它实现会直接跑不通：

- **发送帧"首字节 0xFF 是发送头标记"不成立。** 那是 `FillFdcanFrame` @0x1439f0 构造的
  ID `0x7FF`（达妙电机参数寄存器 ID）的低字节；[0..3] 就是 CAN ID。按"标记"解析会丢掉
  几乎所有帧。
- **`0x83` 是发送回显流，不是 USB 错误事件流。** 读它的是 `can_sent_thread_func`，进
  `fdcan_raw_tx_queue`；错误帧在两条流里都以 [0xE] = 0xFF 标记。
- **版本应答是 NUL 结尾 ASCII 串**，不是 2 字节 LE 版本号。
- **CRC 是 CRC-16/ARC（初值 0）**，不是 Modbus。
- **心跳负载是 17 个 0xFF**，不是 16 个 0 加 0xFF。
- 记录头 [0xD] 低 4 位、[0xE]、发送头 [4] 低 4 位与 [0xE..0x15] 的含义见第 3 节；此前均
  标为"待抓包"。
- 命令 `0x02` / `0x03` 是 IAP（固件升级），此前的实现对未知命令一律回成功，会让 DMTool
  以为升级成功。
- 零长度传输被 DMTool 当成 USB 故障；满包结尾的传输会拖到超时并被丢弃（3.3 节）。

## 6.1 上板验证记录（2026-09-19, CAN1↔CAN2 回环线）

- **DLC 9-15 长帧(12-64 字节)已支持**：消息 RAM 元素扩到 64B(RX FIFO0 20 元素/TX FIFO
  14 元素/过滤器 1+1/RXBUF 与 RX FIFO1 关闭, 总计 615/640 词), DLC 9-15 逐档回环验证
  回显+接收全对。三个曾踩的坑: ①数据域 64B 但元素数沿用经典预设(32+32+过滤器 96 词)
  消息 RAM 924 词溢出; ②RXBUF(64 词)与 RX FIFO1(64 词)是经典预设残留, 必须显式关闭;
  ③`mcan_read_rxfifo` 的 `rx.dlc` 是线上 DLC 码而非字节数, 记录头直接用它, 不要再按
  字节数"反查"。`handle_downlink` 的 DLC 必须用线上 DLC 码(12 直写), 字节数 16 会被
  4 位字段截断成 0 -- 线上帧 DLC=0, 接收方全丢。

- **字段正确性**：标准帧/扩展帧(29bit)/DLC3/RTR 逐字段回环比对全对（ID、DLC、负载、
  通道、回显状态 0x12）；标准 ID >0x7FF 如实回发送失败(status=0x02)且不产生接收记录。
- **吞吐**：DM 路径双向各 ~11.9k 帧/s（发送+回显+接收三流并发）零丢失；此时瓶颈是
  主机侧 Python 发送循环，非板子。libhcs 路径回环 RTT p50 115.6 us 与 HEAD 基线一致。
- **时间戳精度**：300 帧 @1ms 主机节拍，设备时间戳间隔 avg 1.132ms/min 1.093/max 1.547
  忠实跟随主机实际节拍(含 Python sleep 抖动)，单调递增，PTPC 粒度 6ns。

## 7. 待确认

真适配器对拍（用户手上有）：

- 真适配器在记录头 [0xC]、[0xF] 实际填什么；错误帧记录的负载格式（本板不产生错误帧）。
- DMTool 打开设备时是否自动发 SETUP_BUARD、默认参数是什么（若默认是经典 CAN，打开时会报
  一次失败，需在界面选 CANFD 1M / 5M）。
- 心跳周期。

上板：

- `lsusb -v` 核对描述符（7 个接口、CDC 通知端点 `bInterval` 16）。
- DMTool 全流程：打开 → 版本 → 读波特率 → 采集 → 收发 → 时间戳排序 → 关闭。
- libhcs 回归：`HcsLinkProbe` / 延迟拆解（`kGetLatencyBreakdown`）与 HEAD 镜像对照，
  确认第 5 节的指令级结论在包率与 RTT 上无可见差异。

## 8. 主机侧配置与复现

udev：应用换了身份，原来按 `a511` 放行的规则管不到它。以下规则放行本板（也放行真达妙
适配器，DMTool 同样需要），并让 ModemManager 不去探测它的 CDC 口：

```text
# /etc/udev/rules.d/99-libhcs.rules（追加；0x6877 单 CAN、0x6632 双 CAN）
SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6632", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="tty", ATTRS{idVendor}=="34b7", ATTRS{idProduct}=="6877", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="34b7", ATTRS{idProduct}=="6632", ENV{ID_MM_DEVICE_IGNORE}="1"
```

复现逆向：`DMTool-v2.1.6.7-x86_64.AppImage --appimage-extract` 解出 `squashfs-root/`，
主二进制 `serial-port-assistant` 未剥离符号；`nm -C` 找函数地址，
`objdump -d -C -M intel --no-show-raw-insn` 反汇编。本文所有地址都指这个二进制。
