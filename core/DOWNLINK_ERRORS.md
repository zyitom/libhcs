# 下行流错误的处理与回报（kStreamError）设计记录

> **文档类型**：背景说明（设计取舍与实测）
> **适用范围**：core 记录流（`core/src/protocol/deserializer`）、三块板固件与主机 SDK 的
> 错误回报路径（`core/src/link/downlink_errors.hpp`、`data::SessionType::kStreamError`）
> **状态**：第 2.1 节（两类结局）现行有效；回报路径（2.2–2.4）已被 v16 接替——下行流错误
> 并入端口运行时状态，作链路（`DataId::kSession`）的 `kPortStatus`，见 [PORT_STATUS.md](PORT_STATUS.md)。
> 2.2 "挂在 keepalive 应答上、变了才报" 的结论仍成立；2.3 的"从会话起算"改为上电以来的自由计数。保留备查。
> **相关文档**：[PROTOCOL.md](PROTOCOL.md) 1.3（线格式的现行规范，本文不重复） ·
> [AGENTS.md](AGENTS.md)（构建与测试命令）

## 摘要

本文回答三个问题：下行错误为什么要分两类、错误为什么挂在 keepalive 应答上、
线格式为什么为此动了一次（v15）。改动前的行为与改动后的实测在最后两节。

## 1. 改动前的问题

2026-10-06 之前，`Deserializer` 遇到任何解析失败都走同一条路：无参的
`error_callback()`（板子上是空的 TODO，hpm 上只有注释），然后 `enter_discard_mode()`
丢掉**本次 USB 传输剩余的全部字节**，到 `finish_transfer()` 才恢复。两类后果完全不同
的情况被一视同仁：

1. **格式坏**（记录截断、头部非法、保留编码、字段号不认识）：定界已失，除了丢到
   传输边界别无选择——这类丢整批是对的。
2. **字节完整但板子拒收**（`*_deserialized_callback` 返回 false：本 PCB 没有这个口、
   下行 GPIO 记录带时间戳、逆着声明方向）：记录的每个字节都已消费，下一条记录的
   边界完好。丢整批是错的——同一批里排在后面的好记录（例如电机帧）被连坐。

主机侧对这一切完全不可见：`error_callback()` 无参，主机连"出过错"都不知道。

## 2. 设计与取舍

### 2.1 两类结局分开（第 1 部分，不动线格式）

`process_*_field` 的返回值从 `bool` 换成四值的 `RecordStatus`：
`kDelivered` / `kRefused`（拒收，只跳一条）/ `kMalformed`（结构坏或截断，丢整批）/
`kUnknownField`（字段号不认识，主循环的 default 分支直接给——长度不可知，跳不了）。
主循环里只有出错分支多花比较；成功路径上没有新增工作，出错分支全部 `[[unlikely]]`。

`error_callback(FieldId field, data::DownlinkError reason)` 带上字段号与原因
（`kMalformed` / `kUnknownField` / `kRefused`）。报告点收拢在主循环一处，
`enter_discard_mode()` 只管机制不再上报——截断（`finish_transfer()` 的恢复路径）
与结构坏因此共用同一份报告代码，恰好各报一次。

主机 Handler 同步实现新回调（上行方向），把"Unexpected can field id"那类逐口日志
删了并入这一处（字段号 + 原因，按 1st/2nd/4th… 限频），少一半重复日志。

### 2.2 为什么挂在 keepalive 应答上（第 2 部分，v15）

板子要"主动上报"错误，候选有三个挂点：专门的诊断字段、EP0 中断、keepalive 应答。
选 keepalive 应答后追加一条 `kStreamError`，理由：

- **天然限流**：每轮（250 ms）至多一条，计数没变不追加。错误风暴打不爆上行。
- **零新增往返**：错误最多延迟一个轮次到达，对诊断足够。
- **格式不动 `kKeepaliveAck`**：不认识 `kStreamError` 的旧解析器根本到不了这里——
  指纹门（`kSessionWireVersion` → EP0 `kVersion`，v15 = 0xb1c4）保证对端要么认识、
  要么在连接时就被拒。带载荷的会话类型本来就不能发给指纹不符的对端（PROTOCOL.md 1.3）。

### 2.3 计数从会话起算

`core::link::DownlinkErrors`（三块板共用一份，各持一实例）记：累计错误数、最近一次的
字段号 / 原因 / 传输序号。**会话打开时清零**：主机把 `last_transfer` 对着自己在这个
会话里发出的下行传输数，一一对应；上一会话的错误不报给新主机。32 位计数自由回绕，
主机只比较相邻两次回报的变化，不解释绝对值。

传输序号由 `finish_downlink_transfer()` 递增（错误记在"尚未结束"的那次上）；
`note()` 在 `error_callback()` 里跑，每字节热路径不经过账本。

### 2.4 主机侧出口

- SDK：`Handler::downlink_errors()`（`DownlinkErrorSummary` 快照）——收到
  `kStreamError` 时更新两个原子并打一条 error 日志（计数没变不重复打）。
- HCS 板层：接在现成的尽力域出口上——`BoardCore::report()`（1 Hz）新增
  `report_downlink_errors()`，计数比上次读到的**高**才打一条 warn 汇总（逐条日志
  SDK 已打；计数变小说明换了会话，不是好转，不报）。

## 3. 实测

[实测 2026-10-06，三板台架：两块 5321（USB 3-1 / 3-3）+ mc02（3-4），三块 CAN1 同一条总线，
固件从本树编出（v15 = 0xb1c4）。`tools/check.sh --all`：545 条测试 0 失败（32 条跳过 =
台架形态不符），其中 `ThreeBoardBench` 全部用例真板上通过。]

- `ThreeBoardBench.ARefusedRecordIsSkippedAndReported`（0.57 s）：向 5321 A 一批发两条
  记录——先一帧它没有的 CAN3（不经板卡类检查），再一帧正常 CAN1。对端 5321 B 收到
  CAN1 帧；A 的 `downlink_errors()` 报 `total == 1`、`last_field == CAN3`、
  `last_reason == kRefused`；此后 300 ms 内计数不再增长。三条结论各自成立：
  拒收只跳一条、回报恰好一条、计数不带连坐。
- 跨板时间戳用例（`TwoHpmBoardsStampTheSameFrameAlike`、`TheMc02StampsTheSameFrameAsA5321`）
  在新固件上原样通过——错误处理路径的改动没有碰热路径的时序。
- `c_board`、`hpm6e8y` 不在台架上，只编译（`--all` 的固件步），未上板跑。
- 已知环境噪音：`hcs_link` 的多进程通道压测（`test_channel`）与 USB 台架用例并发跑时
  出现过一次丢值失败，单独重跑 14/14 通过——与本改动无关（该包不依赖 libhcs）。

## 4. 影响

- 线格式：`kSessionWireVersion` 60804 → 18007，EP0 `kVersion` 0x4257 → 0xb1c4。
  台架上所有板子与主机 SDK 必须同批更新（ep0_declaration_test.cpp 末尾的版本钉）。
- 板端开销：成功路径零新增；错误路径多一次虚调用与一次记账。hpm 每帧热路径
  保持在 ILM（链接期 ASSERT 未动过）。
- `c_board`、`hpm6e8y` 不在台架上，只编译验证，未上板跑。
