# 端口运行时状态（kPortStatus）设计记录

> **文档类型**：背景说明（设计取舍）
> **适用范围**：三块板固件与主机 SDK 的端口运行时状态路径（`core/src/link/port_status.hpp`、
> `data::SessionType::kPortStatus`）
> **状态**：现行有效
> **相关文档**：[PROTOCOL.md](PROTOCOL.md) 1.3（线格式的现行规范，本文不重复） ·
> [DOWNLINK_ERRORS.md](DOWNLINK_ERRORS.md)（同一挂点上的下行流错误）

## 摘要

本文回答三个问题：运行时状态为什么从 EP0 搬到数据流、"变了才发"的账本怎么写、各板
能报出什么。v16（2026-10-06）起 `kGetPortStatus` 退役，CAN 控制器的错误状态、串口的
接收错误、以及板子自己因缓冲满丢掉的帧，统一挂在 keepalive 应答后面推给主机。

## 1. 为什么不走 EP0

EP0 留给配置面（握手、声明、查口清单），运行时一条控制传输都不发。理由：

- **控制传输排在 bulk 前面**：按 USB 2.0 规范，控制传输在 microframe 里先于 bulk 调度，
  会推迟同一 microframe 的 bulk OUT [手册]。
- **主机侧是一次同步往返**：`libusb_control_transfer` 是阻塞的 ioctl，单次在 100 µs 量级，
  还要拿 `control_mutex_`（和 keepalive 断线重连时的 EP0 共用）[推断，未测]。
- **代价随轮询频率和口数线性增长**：N 个口就是 N 次往返。
- **轮询会漏掉中间态**：两次查询之间进了 bus-off 又恢复，主机看不见。

旧做法（HCS 每秒对每路声明的 CAN 发一次 `kGetPortStatus`）在 1 Hz 下总量很小，没有测出
抖动；换掉是为了把"运行时 = 数据流、配置 = EP0"定成一条不需要权衡的规则，而且推送的
增量开销接近零。EP0 抖动的定量数据**没有测**。

## 2. 设计

### 2.1 一种机制：计数器 + 变化才报

- **板端**：每路口的驱动提供 `read_status()` 原语（`CanPortDriver` / `UartPortDriver`），
  返回 `data::CanStatusView` / `data::UartStatusView`。计数都是上电以来的自由计数
  （`firmware/common/app/src/utility/event_counter.hpp` 的 `EventCounter`：单写者，relaxed
  的读 + 写，编出来就是一条 ldr/str，只在出错分支上累加）。
- **账本**（`core::link::PortStatusLedger`）：按 DataId 存"上次发出的"快照；`changed()`
  并进读数后比较（C++20 默认 `operator==`），`commit()` 只在真写进上行流之后调用——上行
  缓冲满丢掉的那条，下一轮照样再发。
- **遍历**（`core::link::offer_port_status<Registry>`）：编译期展开注册表，只取在跑、且驱动提供
  `read_status()` 的口，不按种类列举；读出的视图必须是这个口的（`View::is_for`），接错是编译
  错误。各板的 `report_port_status()` 覆盖就是这一行。
- **线格式：一种记录，正文按口的种类自定义**。头 1 字节 = `DataId`（4 位）+ 正文长度（4 位），
  整个载荷不超过 16 字节；种类由 `DataId` 的编号区间给出，不另占字节。计数用 16 位、按 2^16
  回绕（主机取差），CAN 记录连会话头 17 字节、串口 18 字节——mc02 是全速 USB（64 字节一包），
  一轮里多个口一起报也省包。正文字段只追加，接收方按长度跳过不认识的部分。
- **主机**：接收线程把快照写进按 DataId 索引的 `Seqlock<data::PortStatusVariant>`
  （`host/src/utility/seqlock.hpp`），`Handler::status<View>(port)`（或 `port_status(port)` 拿
  variant；板卡类的口句柄 `status()`）任意线程无锁读，控制环里调也不会被挡住。总线状态跳变
  （error-passive / bus-off）由 SDK 打一行；计数的增量由应用按自己的节奏看（HCS 的 1 Hz
  `report_port_health()`）。

### 2.2 种类只登记一处：`data::PortStatusVariant`

一个口的状态种类由它的 DataId 定死（视图的 `is_for()`：链路 14、CAN 2..5、UART 6..12），所以
"有哪几种状态"只需要一个列表：`data::PortStatusVariant = std::variant<std::monostate,
LinkStatusView, CanStatusView, UartStatusView>`。所有处理状态的地方都对它泛型，不按种类各写一份，
也没有任何一种的特例：

| 环节 | 泛型的做法 |
|---|---|
| 账本 | 每口一个 variant（`std::get_if<View>` 取上次发出的），只做比较：读数就是状态 |
| 编码 | `Serializer::write_port_status<View>`，正文布局取 `PortStatusRecord<View>` |
| 解码 | `protocol::any_port_status_kind()` 编译期展开 variant 的各种类，`is_for(port)` 的那一种解码 |
| 回调 | 一个 `port_status_deserialized_callback(nonce, port, const PortStatusVariant&)` |
| 主机 | 每口一个 `Seqlock<PortStatusVariant>`；`status<View>()` 取其中一种；少见又要紧的变化按种类重载 `note_change()` 打日志（`std::visit` 分派，没有重载的种类不打）|

新增一种状态（例如 IMU）只有四步，漏了第 3 步是编译错误（protocol.hpp 的 static_assert）：

1. 在 `datas.hpp` 写视图：字段、`static constexpr bool is_for(DataId)`、默认 `operator==`；
2. 加进 `PortStatusVariant`；
3. 在 `protocol.hpp` 加 `PortStatusRecord<View>` 特化（正文 ≤ 15 字节，`Bitfield` 布局 + 编解码）；
4. 该类驱动提供 `read_status()`（并把它加进 `port_ops.hpp` 该类的 concept）。

**链路本身也是一个口**（`DataId::kSession`，`data::LinkStatusView`）：v15 的 `kStreamError`
（下行流错误）原来自成一套——自己的载荷、序列化、回调、主机快照与 HCS 报告——v16 并进来，
只剩 `core/src/link/downlink_errors.hpp` 这个状态来源，由会话代码在每轮先 offer。错误计数因此
也改成上电以来的自由计数（与别的计数同一口径）；传输序号仍从本会话起算。

改了布局，两份指纹会自己变（`kSessionWireVersion` 的 static_assert 会提示新值），按惯例全部
板子同批重刷。

### 2.3 基线

板子在每个 kStart 处（`kOpened` 与 `kRenewed` 都算）清账本，之后第一轮**每个在跑的口**都发
一条，干净的也发；主机从不清快照，靠这一轮覆盖。这样应用取的计数差不会因为清零跳变（16 位
回绕的差分辨不出"清零"）。kStart 只在连接 / 重连时出现，不在轮次里，所以不增加稳态流量。

### 2.4 CAN 错误码由驱动锁存

`PSR.LEC` / `DLEC` 读后自清为 `kNoChange`，成功收发后是 `kNone`；bxCAN 的 `ESR.LEC` 成功收发后
也复位。所以"最近一次真实错误"只能在**读寄存器的地方**留住：三块板的驱动共用
`LatchedBusError`（`firmware/common/app/src/utility/latched_bus_error.hpp`），谁读错误码寄存器谁
`note()`，上报用 `latest()`。这样错误码本身不会每轮翻转、触发无意义的上报；重复的同一种错误由
TEC/REC 的变化带出。账本因此不需要认识 CAN（v16 初版在账本里有一个 CAN 特化，与驱动的锁存
重复了一遍，已删）。

**错误码必须在中断里锁存，不能靠主循环轮询**：LEC/DLEC 不光读后自清，下一帧成功收发后硬件
也会把它写回 none。总线忙、偶尔出错时，250 ms 读一次几乎永远看不到错误码——TEC/REC 涨了，却
说不出是哪种错误。所以：

- **hpm、mc02**（同一个 M_CAN IP）：开协议错误中断（hpm `MCAN_INT_PROTOCOL_ERR_IN_ARB/DATA_PHASE`，
  mc02 `IR.PEA` / `IR.PED`，在 `line0_isr` 里直接处理、不经 HAL），中断里读一次 `PSR` 锁存，中断是
  唯一的写者；`read_status()` 只读。单发模式下每个出错的帧至多进一次中断，中断数以帧率为上限。
- **c_board**（bxCAN）：错误码中断走单独的 `CAN1_SCE` / `CAN2_SCE` 向量（2026-10-06 在 .ioc 里开了
  `NVIC.CAN1_SCE_IRQn` / `NVIC.CAN2_SCE_IRQn`，优先级同 RX0，已 Generate）。驱动开
  `CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR`，HAL 把 `ESR.LEC` 译成 `ErrorCode` 的位并清掉 LEC，
  `HAL_CAN_ErrorCallback` 按位还原后锁存，再 `HAL_CAN_ResetError()`（HAL 只往上或）。顺带修掉一个旧 bug：ISR 原先先查 bus-off、
再读 LEC、再读 DLEC，三次读 `PSR`，后两次永远是"无变化"——LED 的错误分类一直是坏的。现在只读
一次 [推断，未上板]。

## 3. 各板能报什么

| 项 | hpm_board | mc02 | c_board |
|---|---|---|---|
| CAN tec / rec / flags / last_error | ✓（PSR/ECR）| ✓（HAL）| ✓（ESR）|
| CAN data_last_error | ✓ | ✓ | 恒 `kNone`（无 FD）|
| CAN tx_cancelled（单发作废）| ✓（发送槽复用时清点）| ✓（同法；含 ES0491 卡死请求被取消的帧）| ✓（同法，看邮箱的 RQCP/TXOK）|
| CAN tx_dropped | 队列满 | 队列满 | 发送环满 |
| CAN rx_dropped | 上行缓冲满 | 上行缓冲满 | 上行缓冲满 |
| CAN rx_lost（接收 FIFO 溢出）| 每次 ISR 见到 IR.RF0L | 每轮看一次 IR.RF0L 并清 | 每次接收中断见到 RF0R.FOVR0 |
| UART parity / framing | 单发 ELSI + 每轮读一次 LSR，按轮计 | 每轮读一次 ISR 粘滞位并写 ICR 清，按轮计 | 每次 HAL 错误回调 |
| UART overrun | 同上 | 恒 0（`OVRDIS`，接收环满记在 `rx_dropped`）| 每次 HAL 错误回调 |
| UART noise | 0（IP 无此位）| ✓（同 parity）| ✓ |
| UART unattributed（种类未知）| ✓ | 恒 0 | 恒 0 |
| UART tx_dropped / rx_dropped | ✓ | ✓ | ✓ |

串口接收错误按轮计（c_board 除外）：一个计数 = "这一轮里出现过这种错误"，每种每轮至多 +1。

- **mc02**：RX 路径有意不开错误中断（`CR3.EIE` / `CR1.PEIE`，见 `rx_buffer.hpp`），但 `ISR` 的
  PE/FE/NE 是粘滞位，DMA 取走字节不清、HAL 也不碰，FIFO 模式下照样留着 [实测]，所以
  `read_status()` 每轮读一次、记下、写 `ICR` 清掉，不需要中断。
- **hpm**：HPM5300 的 UART 没有粘滞的错误记录：FIFO 模式下 `LSR` 的 PE/FE/LBREAK 只描述 RXFIFO
  队首那个字节，DMA 一取走就没了 [手册 + 实测]。按种类的粘滞位（`IIR2.LSR_*_STS`）是
  `HPM_IP_FEATURE_UART_RX_LINE_ERROR_DETECT`，hpm_sdk 里只有 HPM5E00 有，上游 main 的 5300 也没有。
  所以用单发 ELSI：第一个坏字节进一次中断、记下后关 ELSI，`read_status()` 每轮再开，每口每轮至多
  多一次中断，不会有逐字节的中断风暴挤 CAN 转发。中断进来时坏字节几乎总已被 DMA 取走（台架 5/5），
  这时只知道出过错，记 `unattributed`；持续的错误（速率不符）在每轮那一读里仍能读到种类。
- 想在 hpm 上也分清每一次的种类，只有 SDK 例程 `uart_rx_line_status` 的做法：RX FIFO 的 DMA 门限
  设成大于 1 字节，让坏字节在 FIFO 里停到中断读它。代价是每段消息尾部要等字符超时（约 4 个字符时间）
  才被 DMA 取走，且比按 10 bit 空闲切消息边界还晚，要改切包逻辑；不值得，没做。

`rx_lost` 一个计数是"溢出过一次"，至少丢一帧：硬件只给标志，不数帧。mc02 的作废清点在发送
热路径上每帧多读一次 TXBCF（D2 域外设读），与 hpm 同价；bus-off 恢复后至多一个 FIFO 深度的
槽可能被多计作废（恢复在中断里，不去与主循环抢写槽位图）。

计数为什么只要 16 位：板子每轮计数一动就报，一轮 250 ms 里任何一项都动不了 65536 次（2 Mbaud
串口满速收垃圾约 5 万字节/轮，hpm 的串口错误还是按轮计），主机取模 2^16 的差就没有歧义；板子
重启必然让链路先报 kFaulted、换一个 SDK 对象，同一个基准不会跨过重启。

随这次改动退役的旧字段：`tx_occurred`（TXBTO 位图快照）、`rx_frames`（转发帧数，每轮都变，
推送时会每轮触发一条；"总线有帧但板子丢了"由 `rx_dropped` 直接给出）、`rx_fifo_level`（瞬时
电平）。

## 4. 验证状态

[实测 2026-10-06] 主机侧：libhcs `ctest` 136 条通过（wire 往返含链路、按长度跳过正文、账本与
基线、遍历、上行满重发、链路只报新错误、退役请求 STALL），HCS `tools/check.sh --firmware` 552 条 0 失败、
四块板固件编过（线格式 v16 = `0x761f`）。

[实测 2026-10-06 21:40] 台架 Mc02Bench（5321 AF-958F + mc02，两块都刷本树 v16 镜像）：
`Mc02Bench` 25 条里 21 条 ×3 轮通过，含 CAN 状态推送（`AnUndeclaredBusIsOffTheWire` 读到推送的
TEC > 0 与 ACK 错误码）。没跑的：`Bench`（只接了一块 5321）、`ThreeBoardBench`（同）、c_board 与
hpm6e8y（不在台架上，只编过）。没过的 4 条：

- **FD 帧的 ACK 错误记在 DLEC**：FD + BRS 帧没人应答时，hpm 与 mc02 都把 ACK 记在
  `data_last_error`，`last_error` 是 `kNone`。两块板的驱动各自读 `PSR`（hpm SDK 宏、mc02 HAL），
  位域无误，是 M_CAN 本身的行为。`Mc02Bench.AnUndeclaredBusIsOffTheWire` 原先期望 `last_error`，
  已改为看 `data_last_error`；`Bench` 版跑经典 CAN，不受影响 [实测]。
- **mc02 串口错误永远是 0**：`note_rx_errors()` 挂在 `HAL_UART_ErrorCallback` 上，但
  `rx_buffer.hpp` 有意不开 `CR3.EIE` / `CR1.PEIE`（且 `OVRDIS = 1`），线路错误不进中断，回调
  从不因 PE/NE/FE 触发。v16 的 mc02 串口错误计数实际不工作 [实测 + 读代码]。
- **hpm 的 `LSR` 每轮读一次只看得到"此刻 FIFO 里的错误"**：mc02 按 115200 发 256 字节
  （约 22 ms）给按 1 M 收的 5321，计数为 0；改发 8 KB（约 0.7 s，跨过几轮）计数就涨。所以 `LSR`
  错误位随 DMA 取走字节就没了，不保持到被读，第 3 节"这一轮里出现过"的口径不成立：
  两次读之间的一阵短错误整个漏掉 [实测]。推送链路本身是通的（同一用例里长突发能报上来）。
- **CAN2 两边都收不到**：`DeclaredBusesCarryFramesBothWays`、`ConfiguringABusAtRunTimeDeclaresIt`、
  `ANewHandshakeReplacesTheOldDeclaration` 在 CAN2 上失败，CAN1 正常。两边发送方都只记 ACK 错误
  （能回读自己的位，收发器与 RX 引脚是好的），两边 `rec` 都是 0（谁也没看到对方的任何一位）；
  M_CAN 在正常模式下对任何合法帧都会应答，与过滤器无关。两个正常模式的控制器互相完全看不见，
  更像 CAN2 那对线没接上，待核对接线 [推断]。

[实测 2026-10-06 22:50] 串口错误改按轮计后复测（第 3 节的做法；当时是实验版，线格式不变）：mc02
256 字节乱码每次记 framing + noise 各 1 次；hpm 5 轮里中断每次都进来了，但每次 `LSR` 已空，种类
一次也没读到。CAN2 接上后 `Mc02Bench` 25 条 ×2 全过。随后定为 v17：加 `unattributed`。

[实测 2026-10-07] c_board 第一次上板（台架 CBoardBench：5321 AF-90A7 + mc02 + c_board，三块板 CAN1 同一条
总线，c_board 丝印 UART1 对 mc02 UART1）。通过：三板 CAN1 经典 1M 互发、c_board 单独在线时推送的 TEC 与
`kAck`（CAN_SCE 错误中断锁存，经典帧记在 `last_error`）、UART1 双向、速率不符时推送接收错误、之后干净字节
无错误（8/8）。途中修掉三处固件问题：

- **bootloader 拒绝本树的 app**：app 的栈在 CCM 顶（`_estack = 0x1000FFC0`），bootloader 的向量表检查只认
  主 SRAM，DFU 之后报"固件损坏"。两边自首次提交起就不一致，c_board 此前从未能启动本树 app。bootloader 改为
  也接受 CCM（需用 J-Link 重刷 bootloader）。
- **`read_status()` 的 ESR 位域取错**：按 TEC[7:0]、LEC[18:16]、REC[14:8] 读，TEC/REC 一直是错的
  （RM0090：REC[31:24]、TEC[23:16]、LEC[6:4]）。
- **串口出错重启接收后把整个环（2048 字节）重发一遍**：F4 的 HAL 遇接收错误先中止 DMA，重启时双缓冲
  `CR.CT` 保留、HAL 不清，DMA 先写第二块，写指针被误判为回绕。重启前清 `CT`；同时重启不再在中断里改主循环
  独占的读指针。

随后补上"没声明的口不工作"（原先 c_board 的 CAN / 串口 / IMU "常开"，没声明的 CAN1 照样在总线上应答、
发往它的帧照发、IMU 照样出样本）：CAN 没声明停在 INIT 模式；IMU 的 `apply`
接受即上线。没声明的串口关接收器、停接收 DMA 与中断，IMU 屏蔽数据就绪线、不发起 SPI 读（不只是不上报）。
CBoardBench 9 条 ×3 通过。三板同挂 CAN1 时 5321↔mc02 的 CAN-FD 即使 c_board 离线也过不去，错误
全在 5 Mbit 数据段（BIT1 / STUFF，仲裁段干净），推断是线上的问题（三个终端并联、分支），未查 [推断]。

`can_rtt_probe` 要一个 0x145 的电机挂在 CAN1、UART0 接 HI91 IMU，台架上没有，往返延迟没测。
