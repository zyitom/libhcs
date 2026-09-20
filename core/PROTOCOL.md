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
CAN 帧头（标准 3 字节 / 扩展 6 字节）+ 负载 + 可选 4 字节时间戳（小端）。
编码与解码的**代码即规范**：`core/src/protocol/serializer.hpp`（上行编码）、
`core/src/protocol/deserializer.cpp`（下行解码）、`core/src/protocol/protocol.hpp`
（位定义）。

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
| 19 | `HasTimestamp` | 负载后跟 4 字节微秒时间戳 |
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
属性**（`kGetInterface.can_fd_mask`），per-frame 只表达"长度字段怎么解"。注意：
**位 4 退役时"接收方可忽略"的规则随之作废**——忽略位 4 会把长帧长度解错。

### 2.4 无效编码（两个方向都定义为保留，收到即丢弃并进 discard 模式）

- `IsLongFrame = 1` 且 `IsRemoteTransmission = 1`（ISO CAN-FD 无远程帧）；
- 长帧 DLC 字段 = 7；
- 长帧负载字节数不在 DLC 表内（9-11、13-15、17-19 等，见第 3 节）。

经典帧 DLC 9-15 仍按 CAN 规范钳为 8 字节（板端归一化）。

## 3. DLC ↔ 字节数映射（唯一权威表）

`core/include/libhcs/protocol/can_dlc.hpp` 的 `payload_length()` /
`dlc_from_payload_len()`。记录流两个方向、全部板卡、主机 SDK 与 DMTool 桥
（原表自 dm_protocol.hpp 收敛至此）共用：

| 线上 DLC | 0-8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 |
|---|---|---|---|---|---|---|---|---|
| 字节数 | 0-8 | 12 | 16 | 20 | 24 | 32 | 48 | 64 |

不在表内的字节数没有线上表达，编码端必须拒绝（`kDlcInvalid`）。

## 4. 能力协商与兼容矩阵

协商机制 = EP0 `kGetInterface` 的**线格式指纹** `kVersion`（编译期折叠，改线
格式自动变化，无手工版本号）+ `caps` 里的 `kCapCanFdLongFrames` 位。规则：

- **固件**：只在总线处于 FD 模式时编码/接收长帧（总线模式它自己知道）；
  下行长帧遇经典总线丢弃（DMTool 可在会话中切模式，存在竞态窗口）。
- **主机 SDK**：板类在 `can_transmit()` 门禁——负载 > 8 字节要求板子广告
  `kCapCanFdLongFrames` **且**目标总线是 FD（`hcs::Interface::can_long_frames()`）。
  不满足即抛异常，而不是发出板子无法承载的记录。

| 组合 | 行为 |
|---|---|
| 新主机 + 新固件（广告位） | FD 总线上长帧双向可用 |
| 新主机 + 老固件 | 指纹不匹配，构造时抛异常要求重烧 [实测，v1→v2 先例] |
| 老主机 + 新固件 | 老 SDK 指纹校验失败，连接阶段拒绝 [实测，v1→v2 先例] |

结论：老对端**不会静默失步**（那会把记录流撕裂），而是连接时干净拒绝——
代价是本改动上线后**全部板卡需重烧固件**才能配新 SDK。

## 5. 板卡支持矩阵与 bxCAN 约束

| 板卡 | 控制器 | 硬件能力 | `kCapCanFdLongFrames` | 说明 |
|---|---|---|---|---|
| `hpm_board`（6E8Y/5321） | MCAN | RX 元素与 TX 缓冲已配 64 字节 | ✓ 广告 | FD 模式由端口表初定，DMTool 可运行时切换 [实测] |
| `mc02`（H723 FDCAN） | FDCAN | TX 侧可 64 字节，RX 元素仍 8 字节 | ✗ | RX 扩容前不广告，避免"能发不能收"的不对称能力 |
| `c_board`（F407） | bxCAN | 无 CAN-FD | ✗（物理不可能） | 永远无长帧 |
| `ch32_board` | bxCAN 类 | 无 CAN-FD | ✗ | 同上 |

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
[can_dlc.hpp](include/libhcs/protocol/can_dlc.hpp)。
