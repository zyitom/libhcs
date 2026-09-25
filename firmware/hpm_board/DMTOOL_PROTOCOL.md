# DMTool 兼容：达妙 USB2FDCAN 协议与本板实现

> **文档类型**：硬件参考
> **适用范围**：`firmware/hpm_board/` 的 HPM5321 应用（`app/src/dmtool/`）；host SDK 的 USB 身份与数据管道布局（`core/include/libhcs/protocol/usb_identity.hpp`）
> **状态**：现行有效（2026-09-21 上板回归见 6.4；命令通道 clear_halt 失步待修，见第 7 节）
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
- 第 6 节：与此前一版逆向清单的出入（那版有几处会让实现直接跑不通）、IAP 分包格式
  与 2.1.6/2.1.9 两个版本的实测差异，电机固件升级（CAN 与串口两条路）的线协议，以及
  2026-09-21 的上板回归（性能 A/B、DMTool 功能、命令通道 clear_halt 失步）。
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
- **在 Linux 上，DMTool 每条命令前后的 `clear_halt` 会打乱 data toggle——命令通道只有
  第一条能成功** `[实测 2026-09-21，见 6.4]`。根因不在板子一侧：板子按 USB 2.0 9.4.5 在
  CLEAR_FEATURE(ENDPOINT_HALT) 时**总是**把 toggle 复位成 DATA0（`usbd_edpt_clear_stall`
  无论是否 stall 都调 `dcd_edpt_clear_stall`），而这台 Linux 主机（6.8 xhci）对**没有
  halt 的端点**做 `libusb_clear_halt` 时**不复位主机侧 toggle**——证据：绕过 libusb 直接发
  裸 CLEAR_FEATURE（主机 toggle 必然不动）与 `clear_halt` 的失败模式逐条相同。Windows 的
  `WinUsb_ResetPipe` 按文档会复位主机侧 toggle `[文档，未实测]`，所以同一块板在 Windows 上
  应当正常。2026-09-19 的 gdb 验证只看了打开设备后的第一条 START_CAP，所以没暴露。CAN
  数据面（`0x03`/`0x81`/`0x83`）DMTool 不做 clear_halt，不受影响。

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
| `0x02` | IAP_PACK | 4100 字节：LE32 分包序号 + 4096 字节镜像数据，见 6.2 | **失败**（不做固件升级；负载超解析器上限，只校验不留存） |
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
| `0x10` | 保存参数 | 0 | 成功；两路当前位时序写 flash 末扇区，掉电保持，下次会话首条命令套用（见第 4 节） |
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

### 2.4 波特率预设表（界面下拉框 ↔ 线上字段）`[逆向 DMTool 2.1.9.3]`

下拉框的每一档对应一组写死的 `{分频, seg1, seg2}`，表里**没有 sjw**；"更新配置"按
GetBaudRate 读回的这三个字段回查表项，查不到就没有任何界面反应。两张表都是
`fdcan_protocol` 的全局数组，索引越界的那一项（仲裁 6 / 数据 11）是"自定义"对话框。

| 仲裁段 `can_seg_table` @0x3ec720 | 分频 | seg1 | seg2 | 数据段 `can_fd_seg_table` @0x3ec6c0 | 分频 | seg1 | seg2 |
|---|---|---|---|---|---|---|---|
| 1 M | 2 | 29 | 10 | 5 M | 2 | 5 | 2 |
| 500 k | 2 | 69 | 10 | 4 M | 2 | 6 | 3 |
| 250 k | 4 | 69 | 10 | 3.2 M | 1 | 16 | 8 |
| 200 k | 8 | 43 | 6 | 2.5 M | 2 | 9 | 6 |
| 125 k | 8 | 69 | 10 | 2 M | 2 | 14 | 5 |
| 100 k | 8 | 87 | 12 | 1 M | 2 | 29 | 10 |
| | | | | 500 k | 8 | 15 | 4 |
| | | | | 250 k | 16 | 16 | 3 |
| | | | | 200 k | 16 | 21 | 3 |
| | | | | 125 k | 20 | 27 | 4 |
| | | | | 100 k | 25 | 27 | 4 |

本板会话预设用的正是两张表的第 0 项（1 M / 5 M，采样点 75%），所以读回值一定命中。
2.1.5.3 / 2.1.6.7 的这两张表与 2.1.9.3 一致，SETUP_BUARD 的负载布局三版也一致
（2.1.5.3 @0x190e17、2.1.9.3 @0x18f634 起的装包序列逐字段相同）。

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
- **会话的第一条命令就套用时序**（`ensure_session_timing()`，2026-09-21 起；此前是
  会话首次 START_CAP）。复位后控制器跑 SDK 求解解(分频1/seg1 69/seg2 10, 87.5%),
  不在 DMTool 预设表里 -- "更新配置"按表回查查不到就毫无反应。flash 里有保存配置就
  套用它, 否则套用两张表的第 0 项(分频2/29/10 与 2/5/2, 见 2.4)。触发点提前到首条
  命令, 是因为 DMTool 的打开顺序随版本而异: 2.1.6.7 打开即 START_CAP, 更早的版本
  (2.1.5.3)先读版本/读波特率 -- 连接前先读一次波特率同样要读到表里有的值。libhcs
  会话建立时反向还原编译期时序(87.5%), 两种模式各自自洽。
- **保存配置(0x10)写 flash, 掉电保持。** 命令负载长度 0(`USB_CMD_SaveParam`
  @0x18f390), 只要一个 ACK。本板先回 ACK(避开 DMTool 500 ms 的同步读超时), 再在
  主循环里擦写最后一个 4KB 扇区(`dm_persist.hpp`); 下次会话的 `ensure_session_timing()`
  优先套用它。
- **帧类型按请求逐帧走**（`Can::handle_downlink_as`，2026-09-21 起；此前是一律跟端口
  走、FD 端口恒发 FD+BRS）。端口模式仍是上限：经典模式下请求 FD 会降级成经典帧，
  BRS 只在实际发 FD 帧时才置位；回显报的是实际上线的帧型。**改这条的原因是达妙电机
  的 bootloader 说经典 CAN**：升级流程逐帧带的就是"非 FD"，按端口模式一律发 FD 会让
  这些帧进不了电机(见 6.3)。libhcs 下行的语义与代码路径不受影响 -- 那条路仍是
  `handle_downlink`，两个入口共用同一段装配/入队代码，ILM 里的指令流逐条未变。
  负载超过 64 字节或 ID 越界的帧不发，回显为发送失败（MCAN RX/TX 元素已扩到 64 字节，
  长帧 DLC 9-15 直传线上 DLC；经典帧的此类请求按经典 DLC 9-15 发出，接收方按规范只取
  8 字节）。
- **DMTool 会话开自动重传**（`reconfigure_timing` 里 `disable_auto_retransmission
  = false`，2026-09-21 起）。libhcs 不重传是刻意的(过期的控制指令重发不如丢掉)，但
  DMTool 是通用适配器：电机固件升级一块镜像要连发 1025 帧、全部成功才等到一个 "OK"，
  丢一帧整块作废，而丢帧对上位机不可见。libhcs 会话建立时 `restore_default_timing()`
  连同时序一起把"不重传"恢复回去。代价：总线上没有应答者时，首帧会一直重试到进
  error-passive / BUS-OFF，而不是像以前那样发一次就丢 -- 这正是真适配器的行为。
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
- **libhcs 优先，而且握手后 DMTool 零开销。** libhcs 会话一建立，采集、CDC 桥全部关闭，
  队列清空，**DMTool 的 6 个端点全部 STALL**（`Adapter::isolate()`，2026-09-21 起）。
  STALL 由 USB 控制器硬件直接应答，不产生中断和事件；主机侧还开着的 DMTool 拿到
  `LIBUSB_ERROR_PIPE`，报 USB 故障并按 100 ms 退避（2.1.9.3 `can_rec_thread_func`），总线
  上只剩零星 STALL 握手。不 STALL 的代价实测见 6.4：DMTool 开着时它挂在 0x81/0x83 上的读
  请求让主机控制器持续 NAK 轮询，libhcs 的 RTT 整体右移约 7 µs。隔离期间 DMTool 的
  `clear_halt` 会被立即重新 stall（`on_clear_halt`）。会话结束（租约 **4 s** 到期、挂起、
  拔线）时 `release()` 解除 STALL、重新挂 OUT；已打开的 DMTool 若已报 USB 故障，重新打开
  设备即可。
- **记录流只在采集打开时才泵送**（`poll()` 的 `kServiceCapture` = `capture_mask_ != 0`）。
  没 START_CAP 时：DMTool 发来的 CAN 帧照常上总线（`transmit()` 的 `delivered` 不看
  capture），但总线的回应——电机 ACK、参数应答——只进 `rx_queues_`，**永远不会到主机**。
  因此一切"发一帧等一个应答"的功能都会超时失败：DMTool 的电机固件升级
  （`dm::flow::BuildFirmwareTaskFlow`：读参数 → 读版本 → 固件校验 → 升级检测 →
  编码器校验，全是请求/应答步骤）、读波特率后的"更新配置"回查、占用率测量（记录流里
  没帧恒为 0.0）。真适配器上 DMTool "打开设备"即 START_CAP，所以这一步隐含在打开
  流程里；本板上它对应 FDCAN 连接开关（`sw_btn_FDCAN`）——**灯不亮 = 记录流没开 =
  电机升级必败**，两者是同一个根因。
- **CDC 串口桥** 只在三件事同时成立时接通：串口已打开（DTR）、主机设的线路编码与板上
  UART 实际速率对得上（容差比较，`effective_baudrate()` 是分频器反推值）、没有 libhcs
  会话。**主机设的波特率/字长/校验/停止位会真下发到板上 UART**（`on_cdc_line_coding`，
  仅在无 libhcs 会话时；求解失败则保持原值、桥因失配保持关闭）——所以 DMTool 串口助手
  选什么速率，板子就跟到什么速率，电机的 UART 调试口可以直接这样接。DMTool 的
  "USB2CAN 串口款"协议（`CustomCDC`、0x2E88:0x4603、`55 AA` 帧）本板**不**实现，CDC
  只做 UART 透明桥；但 DMTool 走串口的**电机**固件升级（6.3）因此是通的。
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

上板包率 / RTT 回归（2026-09-21 完成）：HEAD 源码镜像与含 DMTool 仿真的工作树镜像各 3 轮交错，
RTT 双方均为 110/131/133–134 µs，本节的指令级结论在系统级成立。随后出现的 RTT 回归经查与 DMTool
无关，是镜像布局把每帧 flash 依赖挤出 XIP cache，定位与修复见
[USB_OPTIMIZATION_LOG.md](USB_OPTIMIZATION_LOG.md) 第 14 节。

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

  > **更正（2026-09-21）**：上面"20/14/过滤器 1+1, 615/640 词"是当时的设想，**代码从引入
  > 64 字节元素的那次提交（f120599）起就不是这样写的**：实际是 RX 8 / TX 8 / 过滤器 16+16，
  > 外加 SDK 默认、从没被覆盖的 32 项 TX event FIFO，合计 1600/2560 字节。同一次还发现
  > SETUP_BUARD 失败时的救援路径用的正是这套 20/14/1+1，连同那 32 项 TX event FIFO 合计
  > 2716 字节、超出 2560，`mcan_init` 会失败而返回值被丢弃——救援把端口救死。现在四处配置
  > （构造、DMTool 重配、救援、还原）统一走 `Can::apply_message_ram_layout()`：
  > RX 16 / TX 12 / TX event 12 / 过滤器 16+16 = **2304/2560 字节**，预算在 `can.hpp`
  > 里 `static_assert` 编译期核对 `[源码，未上板]`。

- **字段正确性**：标准帧/扩展帧(29bit)/DLC3/RTR 逐字段回环比对全对（ID、DLC、负载、
  通道、回显状态 0x12）；标准 ID >0x7FF 如实回发送失败(status=0x02)且不产生接收记录。
- **吞吐**：DM 路径双向各 ~11.9k 帧/s（发送+回显+接收三流并发）零丢失；此时瓶颈是
  主机侧 Python 发送循环，非板子。libhcs 路径回环 RTT p50 115.6 us 与 HEAD 基线一致。
- **时间戳精度**：300 帧 @1ms 主机节拍，设备时间戳间隔 avg 1.132ms/min 1.093/max 1.547
  忠实跟随主机实际节拍(含 Python sleep 抖动)，单调递增，PTPC 粒度 6ns。

## 6.2 IAP 分包与两个 DMTool 版本的实测差异（2026-09-21）

对 `DMTool-v2.1.6.7` 与 `DMTool-v2.1.9.3` 两个 AppImage 解包反汇编 + 对本板
（`34b7:6632`，已插在开发机上）用 libusb/gdb 插桩对照，结论如下。

**IAP 只发一包是本板"按设计拒绝"的必然结果，不是协议 bug。** `USB_CMD_IAP_PACK`
（2.1.6 @0x1470a0 / 2.1.9 @0x18f010）先把 4096 字节全局缓冲清零，偏移 0 写 LE32
分包序号，再把主机侧 4096 字节数据搬到 +4，以 `usb_contorl_transfer(0x02, 0x1004)`
发出；两版的升级循环（`ConnectGroupBox::on_firmwareUpgradeClicked` 的
QtConcurrent lambda）逐字节相同：`packets = ceil(size/4096)`，逐包
`USB_CMD_IAP_PACK(i, buf + i*4096)`，**任一包返回失败即记录错误、关文件、整体返回，
连 `USB_CMD_IAP_END` 都不发**；全部成功才在末尾发 IAP_END。本板对 `0x02` 一律回
失败（§2.2），于是 DMTool 恰好发出第 0 包就停手。要让 DMTool 的"固件升级
（USB2FDCAN 驱动）"真正可用，需要在板端实现 IAP 语义（分包进 flash staging、
IAP_END 校验写片重启）——那是功能决策，本文只记录协议事实。

**两版的设备识别与连接序列完全一致，差别全在上位机怎么把设备摆到用户眼前：**

| | 2.1.6.7 | 2.1.9.3 |
|---|---|---|
| VID:PID 表 | 4 对：`34b7:6877`、`34b7:6632`、`1d50:606f`、`34b7:4543` | 3 对（前三个）+ `DeviceType` 映射表 |
| 启动时设备枚举 | 2 次（`smartPointerInit` + 刷新槽） | 1 次（`DeviceManager::FindLibusbDevices`） |
| 设备列表来源 | `ConnectGroupBox::on_refresh_usb_btn_clicked` 填 combo | `DeviceManager::matchDevices` → `DeviceListModel`，启动即自动填 |
| 设备在列表里的名字 | `device:<bus>,<addr>`（如 `device:3,6`），由 `QString::arg()` 拼，不用 USB 字符串 | 设备信息结构直接展示 |
| USB2CAN 页 | `tab_Hidden`，启动时不在 6 个可见 tab 里；设备 combo、`sw_btn_FDCAN` 连接开关、`refresh_usb_btn` 都在该页内 | 设备列表直接可见可点 |
| 串口枚举 | 启动时**不**调 `QSerialPortInfo::availablePorts`（点"刷新串口"才调） | 随设备管理一起刷新 |
| 打开序列 | `libusb_open` → claim 0/1/2 → `START_CAP` | 同左 |
| 功能入口 | 全靠 `shortCutKeysInit()` 建的 QShortcut：**F9 = `on_fdcanupdateDialogShow`（USB2FDCAN 固件升级对话框）**、**F8 = `on_firmwareUpgradeClicked`**、F2 = `configDevice()`、F3 = `comDataList()`、F4 = `specialMotorSelection()`、PgUp/PgDn = 波形翻页。Qt 键码 0x01000031..0x01000038 即 F2..F9，没有 Ctrl/Alt 组合 | 图形化设备列表，点就用 |

对本板的插桩实测（2.1.6，`OpenFdcanDevice(0)`）：`claim_interface 0/1/2` 全部
`LIBUSB_SUCCESS`，EP 0x02 发出 `a5 00 01 00 00 51 c0 5a 00`（START_CAP 通道 0），
EP 0x82 收回 `a5 00 00 00 00 00 00 5a`（STATUS=0 成功）。**即本板的 DMTool 协议
实现对 2.1.6 完全正常**，"2.1.6 点不亮"不是固件或协议问题：2.1.6 默认停在"关于"页、
USB2CAN 页默认不可见，设备的 CDC 口（`/dev/ttyACM0`）又在串口页里，看着就像"被当成
普通 usb2tty"。2.1.9 用新的 DeviceManager 把设备直接列出来，所以一扫就能连上。
实测按 **F9** 能直接唤出 USB2FDCAN 固件升级对话框（`FirmwareDialog`：选固件按钮 +
版本标签 + 进度条 + 升级/取消），但设备没连上时对话框里的按钮不响应——所以 2.1.6 的
正确顺序是：先连上 FDCAN 设备（`sw_btn_FDCAN`），再 F9 升级。

## 6.3 电机固件升级（`[逆向 DMTool 2.1.9.3]`，2026-09-21）

**别和 6.2 的适配器 IAP 混淆**：DMTool 里有两个"固件升级"。F9 的 `FirmwareDialog` 升级
**适配器自己**（USB 命令 0x02/0x03，本板按设计回失败）；这一节说的是升级**电机**，走
CAN 或串口，板子只是转发桥，不需要任何设备端功能。

任务按连接方式二选一，两个类都注册在 `dm::core::RegisterAllTasks`，任务 id 字符串是
`firmware_upgrade_can` / `firmware_upgrade_uart`（和 `read_param_can` 一样的命名套路）。

### CAN：`dm::core::FirmwareUpgradeCanTask::Run()` @0x245ae0

| 步骤 | 线上内容 | 超时/节奏 |
|---|---|---|
| ① 进 bootloader | TX **ID 0x7FF**, DLC 8: `55 01 02 AA <id_lo> <id_hi> AA AA`（@0x245b3c 起装包，[4..5] 是电机 ID） | — |
| ② 等应答 | RX **ID 0x7FE**，负载里 `find("Aupgrade")`（@0x245c24） | 500 ms，失败报 `can enter bootloader timeout` |
| ③ 传镜像 | 每 8 KB 一块，缓冲 0x2006 = 8198 字节：`23 <块号> 23 00 20` + 8192 字节数据(不足补 0) + 1 字节尾；切成 **8 字节一帧**发到**电机 ID**（不是 0x7FF） | 帧间 **600 µs**（构造函数 @0x245a9b 写死 0x258，`SetCanParams` 只改 ID 不改它）→ 1025 帧/块 ≈ 615 ms |
| ④ 等块 ACK | RX ID 0x7FE，负载里 `find("OK")`（@0x246172） | 500 ms，失败报 `can packet ack timeout` |

600 µs 的节奏远低于经典 CAN 1 Mbit 的 ~9k 帧/s，本板 32 级 TX FIFO + 64 级软件队列不会
被冲爆；**真正会卡住升级的是帧型与丢帧**，两条都已在第 4 节处理（按请求发经典帧、
DMTool 会话开自动重传）。此外前置步骤（`BuildFirmwareTaskFlow`：读参数 → 读版本 →
固件校验 → 升级检测 → 编码器校验）全是请求/应答，**采集必须是开的**，否则应答只进
`rx_queues_` 到不了主机（见第 4 节"记录流只在采集打开时才泵送"）。

### 串口：`dm::core::FirmwareUpgradeUartTask::Run()` @0x2440c0

先发 **1 个字节 `'X'`**（`kX` @0x37cf72 = 0x58），然后轮询 `DataCenter::TryPullUart()`，
判据是累积缓冲第 0x10..0x18 字节全为 `'!'`(0x21) 的横幅（@0x24432c-0x244494），握上手才
进数据阶段。这条路不碰 DMTool 的 USB2FDCAN 协议，走的是本板的 CDC 透明桥 → 板载 UART0：
串口助手设什么速率板子就跟到什么速率（第 4 节 CDC 那条），电机的 UART 调试口直接接
UART0 即可。

## 6.4 上板回归（2026-09-21，HPM5321 双 CAN `AF-90A7`，CAN1↔CAN2 回环）

**结论：今天的三处 DMTool 改动在真板上行为正确；libhcs 的 p50–p99.9 与改前逐项相同；
另发现命令通道的 clear_halt 失步 bug（改前就有）。** 两个镜像从等长路径的源码树、相同
CMakeCache（`TIME_SYNC=ON`）构建，版本串用等长的 `3.3.1-ab.base` / `3.3.1-ab.newt`
以便每轮从产品串确认板上镜像；base = HEAD f120599，newt = 当天工作区。

**libhcs 性能 A/B** `[实测]`：HCS `HcsLinkProbe`（`hcs_link.yaml` 的 link_5321_a 配置，
1 kHz），每镜像 6 轮 × 60 s 交错（A-B-A-B-A-B-B-A-B-A-B-A），每轮约 5.5 万样本。主机
**未做** `host-tuning.sh`（调速器 powersave），尾部噪声偏大，两臂同条件。

| | rtt p50 | p99 | p99.9 | 每轮 max | ≥1 ms | >500 µs 慢回包 | tx / cmd p50 |
|---|---|---|---|---|---|---|---|
| base ×6 | 110 | 130–131 | 133–134 | 614–1014（均值 800） | 1 | 32 | 6 / 10 |
| newt ×6 | 110 | 130–131 | 133–134 | 922–1073（均值 1020） | 4 | 32 | 6 / 10 |

慢回包两臂都是"连续 seq 每个 +~98 µs"的递增簇，且两臂都出现约 25.6 s 周期的簇——主机侧
卡顿的形态。**未定项**：簇的次数相同，但起步就在 830–1073 µs 的"高位簇" newt 8 次、
base 3 次，使每轮 max 的秩和检验 p≈0.004（按次数 8:3 则 p≈0.11，不显著）；没有找到能
产生百 µs 级主机可见延迟的固件机制（热路径只多约 4 条指令，见第 7 节）。要定性需在主机
调优后按改动二分（base+serializer、base+消息 RAM 各一个镜像）。

**DMTool 功能验证** `[实测]`（pyusb 按本文协议直接操作，无 clear_halt）：

| 项 | base | newt |
|---|---|---|
| T1 会话刚开、START_CAP 之前读波特率 | `(1,69,10)/(1,13,2)`，**不在表里** | `(2,29,10)/(2,5,2)` 命中 1M/5M |
| T2 请求经典帧 → CAN2 收到 | FD+BRS（错） | 经典帧 |
| T2 请求 FD+BRS → | FD+BRS | FD+BRS |
| T2 请求 FD 不带 BRS → | FD+BRS（错） | FD、无 BRS |
| T3 SETUP_BUARD 切经典后请求 FD → | 经典 | 经典 |

数据、回显 `tx_ok` 全部正确。

**命令通道 clear_halt 失步** `[实测，base 与 newt 相同]`：同一串命令（读波特率 →
START_CAP×2 → 读波特率 → STOP×2），主机侧三种做法：

| 每条命令 | 结果 |
|---|---|
| 不 clear_halt | 6/6 成功 |
| 只在写 OUT 前 `clear_halt(0x02)` | 成功/超时**交替** |
| DMTool 的做法：写前 `clear_halt(0x02)`、读前 `clear_halt(0x82)` | **仅第一条成功**，之后全超时 |

按 toggle 规则逐拍推演，"交替"只可能是**板子复位、主机不复位**（主机复位、板子不复位
的话第二条以后会全部失败）。再用裸 CLEAR_FEATURE 代替 `clear_halt`（主机 toggle 必然
不动）复跑"只在写前"那一行，结果逐条相同，坐实了主机侧 `clear_halt` 没有复位 toggle。
一次"先 stall 冲掉在途传输、再 clear、再重新挂上"的板端重同步也试过，结果不变，已撤回。
修法见第 7 节。

**libhcs 握手后隔离 DMTool** `[实测]`：DMTool 开着（按 2.1.9.3 接收线程的方式持续读
0x81/0x83）的同时跑 HCS `HcsLinkProbe`（1 kHz，每轮 30 s）：

| 场景 | rtt p50 | p99 | p99.9 |
|---|---|---|---|
| 没开 DMTool | 110 | 131 | 134 |
| DMTool 开着，无隔离的镜像 ×2 | 117–118 | 138 | 143–144 |
| DMTool 开着，隔离的镜像 ×3 | 110–111 | 131 | 135–138 |

隔离镜像上读者在 HCS 建立会话后 0.5 s 内开始收到 STALL；HCS 退出后租约（4 s）一到期，
同一个读者不做任何操作即恢复正常读取。libhcs 的热路径函数（`handle_downlink`、
`handle_uplink`、`serialize_uplink`、`handle_interrupt_flags`、`tud_vendor_rx_cb`、
`dcd_event_handler`、`can_deserialized_callback`）与改动前逐条相同（只差数据地址）。
仍剩的一处开销：`TIME_SYNC` 构建里 USB 中断每个 SOF 遍历全部类驱动（`usbd.c`
`dcd_event_handler`），DMTool 类驱动加 CDC 让遍历从 2 个变 4 个，每 125 µs 多约 24 条
指令；要去掉只能改 TinyUSB（bsp）。

## 7. 待确认

真适配器对拍（用户手上有）：

- 真适配器在记录头 [0xC]、[0xF] 实际填什么；错误帧记录的负载格式（本板不产生错误帧）。
- DMTool 打开设备时是否自动发 SETUP_BUARD、默认参数是什么（若默认是经典 CAN，打开时会报
  一次失败，需在界面选 CANFD 1M / 5M）。
- 心跳周期。

上板：

- **命令通道 clear_halt 失步（第 1 节、6.4）—— 已按方案②处理（2026-09-22）**：bsp
  `usbd.c` 以"主机是否取过 MS OS 2.0 集"识别 Windows（应用侧经
  `usbd_note_ms_os_20_fetch()` 置位，总线复位清零），`usbd_edpt_clear_stall` 只在端点
  真 stall 或主机是 Windows 时才调 dcd 复位 toggle。A/B：门控前后各跑一轮每命令
  clear_halt 序列（GetBaud / 启停采集 ×2 通道），均 6/6 ok——失步在当前内核 + 本固件上
  **未复现**（2026-09-21 的现象疑与当时的会话恢复场景绑定）；门控保留为防御性修复，
  Windows 语义不变。仍未做：真 Windows 主机实机回归（推断式识别的最终验证）；
  `usb.cpp` `try_recover_link()` 的注释更正。
- 2026-09-21 三处改动的剩余未验项：电机真机上的固件升级全流程（本次只有 CAN1↔CAN2
  回环，没有电机）；自动重传的副作用（总线无应答者时首帧一直重试到 error-passive /
  BUS-OFF）在 LED 与 `kGetCanStatus` 上的表现。
- `lsusb -v` 核对描述符（7 个接口、CDC 通知端点 `bInterval` 16）。
- DMTool 全流程：打开 → 版本 → 读波特率 → 采集 → 收发 → 时间戳排序 → 关闭。
- libhcs 回归：`HcsLinkProbe` / 延迟拆解（`kGetLatencyBreakdown`）与 HEAD 镜像对照，
  确认第 5 节的指令级结论在包率与 RTT 上无可见差异。2026-09-21 起这次回归还要覆盖
  两处改动 `[未上板]`：①消息 RAM 深度 RX 8→16、TX 8→12（6.1 的更正），看 `burst`
  过载下的丢帧门槛；②`serializer.hpp` 重构后 CAN 上行 ISR 的内联形状变了——线上
  字节已用 300 组向量逐字节对拍（改前/改后、`-O0`/`-O2` 全同），但 GCC 把 `write_can`
  内联进了 `Can::serialize_uplink`、不再把后者内联进 `handle_uplink`，每帧多存取 2 个
  callee-saved 寄存器（约 +4 条指令，全部仍在 ILM，ILM 总量反而 −160 字节）。
  `handle_downlink` 仍逐条相同。
  **RTT / 延迟拆解对照 2026-09-21 已上板**（4 镜像 × 2 轮，见
  [USB_OPTIMIZATION_LOG.md](USB_OPTIMIZATION_LOG.md) 第 14 节）：①②③ 合并上板曾稳定复现
  RTT 118–120/147–152 的回归，定位为镜像布局把每帧 flash 依赖挤出 XIP cache、与改动本身无关；
  该组依赖收进 ILM 后 RTT 恢复 110/131/133，①②③ 净收益为下行段 −165 ns、上行段 −144 ns
  （各约 −6%），板端单帧最大 23.4 µs → 3.2 µs。① 的 `burst` 过载丢帧门槛已测
  （2026-09-22，burst=30 全程过载 30 s）：丢帧 240398/744720 = 32.3%，与第 2 节记录的
  约 32% 一致——丢帧边界仍在软件发送队列 + CAN 线速，消息 RAM 加深未引入新丢帧模式
  （miss/stray/tx_exc 全 0，过载全程会话存活）。

- **电机真机 IAP 升级失败已定位（2026-09-22 首次真机实测，DM6006）**：步骤①②正常
  （0x7FF → 电机回 0x7FE "Aupgrade"，透传链路通），失败在步骤④块 0——按 650 µs 逐帧发
  1025 帧后无 "OK"，电机全程静默；发送回显仅 17/1025 帧成功，其余被 **64 深下行软件队列
  溢出丢弃**（青灯 = 下行缓冲满，与用户目击一致）。机制：电机 bootloader 收到 ~17 帧
  （≈136 B）后停止接收（RX 背压/缓冲边界），auto-retransmission 把未 ACK 帧钉死在
  TX FIFO（12 深），软件队列 ~45 ms 灌满，其余帧全部丢弃——**丢帧对上位机不可见**，
  块永不完整，"OK" 永不来，电机停在半块状态（重发 entry 也不应答，需断电恢复）。
  真适配器能过此流程，说明其下行在设备侧缓冲或对主机施加背压（NAK bulk OUT），
  而本板的"下行不背压"设计（2026-09-14，libhcs 控制路径的正确选择）对 IAP 突发是
  致命的。修复选项（需决策，触及 2026-09-14 决策与 RAM 预算）：a) DMTool 会话期间
  加深 CAN TX 软件队列至 ≥1100 帧（~79 KB/路，RAM 预算待查）；b) 队列满时对 0x03
  施加背压（NAK），需验证对 keepalive/UART 下行的影响；c) IAP 块级 8.3 KB 缓冲。
  未验证：真 Windows 主机回归；块号 0/1 起始与尾字节取值（0x00 假设未验证）。

- **双速率实测拆出两层叠加根因（2026-09-22）**：①板端下行无缓冲——600 µs/帧时电机
  bootloader 仅吸收前 ~17 帧（≈136 B，块头触发其 flash 擦除/处理窗口）即停止 ACK，
  auto-retransmission 钉死硬件 FIFO，64 深软件队列 ~45 ms 灌满，其后帧全部丢弃
  （青灯）；放慢到 1.9 ms/帧则 **1025/1025 帧全部被电机接收**。②即便全帧送达，
  bootloader 对块 0 仍零应答（0x7FE 无 OK 也无错误）——块格式细节（块序号基址/
  尾字节语义/前置命令）与 bootloader 预期不符，需对齐官方《USB 转 CANFD 模块
  使用说明书》的 CAN 通道升级流程。板端修复方向：vendor RX 切缓冲模式
  （CFG_TUD_VENDOR_RX_BUFSIZE ≈ 16-32 KB，FIFO 满时 TinyUSB 自动 NAK = USB 背压），
  使 DMTool 的发送节奏被电机实际接收率自适应拉长——这是真适配器的行为，触及
  2026-09-14 "下行不背压"决策的实现层，libhcs 路径的丢弃行为可保持不变
  （两种会话互斥）。

- **板端背压 hold 实验定案（2026-09-22，已实现→已回退）**：曾实现"CAN TX 队列满时
  帧留在 EP 0x03 DMA 缓冲、端点不重挂（主机 NAK 自适应）、队列疏干后断点续帧"
  （dm_adapter hold/resume），推演与实测发现致命缺陷：电机 CAN 不应答（块头触发
  处理/擦除窗口）时 M_CAN 12 深硬件 FIFO 被 auto-retransmission **永久钉死**，软件
  队列永不疏干，hold 永不释放，EP 0x03 整体卡死——比丢弃语义更糟。已回退为
  2026-09-14 丢弃语义（队列满即丢帧+青灯），保留 transmit→bool 返回值链供将来接回。
  修复方向仅剩：a) 队列加深到能装下整个突发窗口（@600 µs 需千帧级 ×72 B，RAM 受限）；
  b) vendor RX 切缓冲模式（CFG_TUD_VENDOR_RX_BUFSIZE ≈ 16-32 KB，FIFO 满时 TinyUSB
  自动 NAK = 真 USB 背压；DMTool 10 ms 写超时风险仍在）。

- **最终定性（2026-09-22，用户指正）：这颗 DM6006 的 bootloader 大概率只在 UART 上
  实现了固件数据阶段**——CAN 上 entry 能进（0x7FF → 0x7FE "Aupgrade" ✓）、数据阶段
  全帧 ACK 但 0x7FE 零应答（所有节奏/格式变体一致）。与官方手册"确保电机固件版本
  支持该功能后才可以 CAN 模式升级"一致：该电机 bootloader 版本的 CAN 数据阶段未实现
  或需特定前置。**电机升级走 UART 路径**：板子 UART0（CDC 桥 ttyACM0）@921600，
  DMTool 串口模式或达妙助手，按《bootloader & 固件 更新操作说明.pdf》流程操作。

- **立即可用的替代路径 [实测 2026-09-22]**：电机 bootloader 同时服务 UART——板子
  UART0（CDC 桥 ttyACM0）以 **921600** 发 `'X'`(0x58)，电机回 "Enter Bootloader!"
  并进入 bootloader 等固件。DMTool 串口模式/达妙助手经此路径可立即完成升级，
  无需等 CAN IAP 修复。注意 bootloader 的 UART 速率为固定 921600（扫描
  4.8M/3M/2M/115200 均无应答）。

- **UART IAP 完整流程实测成功（2026-09-22，正版适配器 USB 抓包）**：'X' 握手 →
  数据阶段 → 电机每 ~300 ms 回状态包 `[b5 da][seq][13B]":OK\r\n"` → 完成回
  `\r\n\r\n   Update firmware complete`（抓包原文）。主机侧在 EP 0x01 上有
  10 ms 周期帧（A5 10 11 00 FF×15 0A 1C 5A，24 B——UART 通道心跳/流控）。
  **板子的 CDC 桥是透明透传：DMTool 串口模式经本板升级与经原版适配器完全等价**，
  未来电机升级直接用串口模式即可。CAN IAP 数据阶段在此电机 bootloader 上不可用。

- **V6217_04 复测定案（2026-09-22，电机升级完成后）**：电机经 UART 路径刷入
  V6217_04 后 CAN IAP 复测（`iap_ack_check.py`）——entry ✓ "Aupgrade"（0x7FF TX
  记录 flags=0x8a，bit3 发送成功），随后 120 帧 IAP 块数据 @1.9 ms **120/120 全帧
  被电机 ACK**（TX 记录 120 条、bit3 全置位、状态字节全 0x00），0x7FE 仍**零应答**。
  帧确认全部到达电机、bootloader 收下后不做任何回应——**CAN 数据阶段的块确认机制
  在该 bootloader 上不存在**（不是节奏/格式问题、不是丢帧问题；同批命令通道 6/6
  PASS、1.9 ms 节奏零丢帧，板子回退后固件工作正常）。CAN IAP 对本 DM6006 定案：
  不可用，UART 为唯一路径。

- **DMTool 串口升级失败根因与修复（2026-09-22，usbmon 实锤）**：DMTool v2.1.9.3
  串口模式经本板升级报 "packet ack timeout at part 0"。抓包时间线：'X' → 电机回
  "Enter Bootloader!" ✓ → 8198B 整块一次 write 到 CDC，USB 层完整送达 → 电机
  12 秒静默 → 超时重启回正常模式（banner），全程无 ":OK"。根因：**板子 UART TX
  环形缓冲 kBufferSize=2048**——USB 高速侧 8198B 突发毫秒级涌入，UART0 921600
  排空需 89 ms，2KB 装不下，try_enqueue 溢出丢弃 ~6KB（青灯）→ 电机收残块 CRC
  不过 → 静默。正版适配器设备侧缓冲整块故能过。**修复：kBufferSize 2048→16384**
  （DLM 78 KB/130 KB，60%）。修复后端到端裸测：**7/7 块全部 ":OK"**（52524 B
  镜像 2.0 s 发完），电机重启回 V6217_04 —— 板子串口桥升级链路完整打通。

- 2.1.6 的 USB2CAN 页（`tab_Hidden`，见 6.2）究竟靠什么唤出：它不在启动时的 6 个可见
  tab 里，`actFDCAN_Capture` / `actFDCAN_Config` 两个 QAction 在 2.1.6 里也没有对应槽
  （`connectSlotsByName` 会为此报警告）。用户是在哪个入口进到该页并点到连接开关的，
  需要实机确认——这决定了"2.1.6 点不亮"是纯粹的入口问题还是另有分支。已知 F9 能唤出
  `FirmwareDialog`（USB2FDCAN 固件升级），但设备未连接时其按钮无响应。

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
