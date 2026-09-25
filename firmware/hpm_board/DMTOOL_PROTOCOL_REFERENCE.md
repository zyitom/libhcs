# 达妙 USB2FDCAN + DMTool 完整协议参考

> **文档类型**：线协议参考（逆向 + 正版适配器 USB 抓包逐字节验证）
> **验证方式**：2026-09-22 正版 DM-USB2FDCAN 适配器 + DMTool 2.1.9.3 + DM6006 真电机，
> usbmon 全程抓包（含一次**完整成功的固件升级**）；DMTool 2.1.9.3 反汇编
> （`FirmwareUpgradeCanTask::Run` @0x245ae0、装包 @0x245f63-0x246069、
> `FirmwareUpgradeUartTask::Run` @0x2440c0）。
> **适用**：任何要仿真这块适配器（HPM5321 的 DMTool 兼容层）或直接对接 DM 电机的人。
> **相关**：[DMTOOL_PROTOCOL.md](DMTOOL_PROTOCOL.md)（板端实现与取舍）·
> [AGENTS.md](AGENTS.md)（约束）

---

## 0. 三个通信域总览

```text
┌────────┐  USB (bulk+CDC)   ┌──────────────┐   CAN 1M/5M   ┌────────┐
│ DMTool │ ←──────────────→ │  USB2FDCAN   │ ←───────────→ │ DM 电机 │
│ (x86)  │  本参考第 1-5 节  │    适配器     │  第 6 节       │         │
└────────┘                   └──────┬───────┘               └───┬────┘
                                    │ UART (CDC 桥)              │
                                    │ 第 7 节（921600）           │
                                    └── 电机 UART 调试/IAP ───────┘
```

DMTool 只认 VID:PID（`34B7:6877` / `34B7:6632` / `34B7:4543` / `1D50:606F`），
不看任何字符串。正版适配器与本板仿真 **VID:PID 完全相同**（`34b7:6632`），
产品串不同（正版 `DaMiao-Tech DM-USB2FDCAN`，本板 `HCS Agent v<版本>`）。

---

## 1. USB 端点布局（正版实测 + 本板对齐）

| 接口 | 端点 | 方向 | 用途 | 备注 |
|---|---|---|---|---|
| 0 | `0x01` | OUT | **心跳** | 24 B、10 ms 周期、设备不回应 |
| 0 | `0x81` | IN | CAN 接收记录流 | 32 KB 同步读（DMTool 侧），超时 1 s |
| 1 | `0x02` | OUT | 命令 | 正版每条命令前对 0x02/0x82 各 clear_halt |
| 1 | `0x82` | IN | 命令应答 | 一条应答独占一次 IN 传输、短包结束 |
| 2 | `0x03` | OUT | CAN 发送 | 22 字节头 + 负载，一帧一次传输 |
| 2 | `0x83` | IN | CAN 发送回显流 | 同 0x81 |
| 3 | `0x04` / `0x84` | OUT / IN | 保留字节流 | DMTool 不碰 |
| 4/5 | `0x86`；`0x05`/`0x85` | IN；OUT/IN | CDC 串口桥 | **电机 UART IAP / 调试走这里**（第 7 节） |
| 6 | — | — | DFU runtime | — |

正版配置描述符实测（152 B，5 接口）：EP `0x81` bulk 512 B（接口 0）……
完整映射见抓包 `usbmon_3u.bin` 设备 023 的 GET_DESCRIPTOR(CONFIG) 记录。

---

## 2. 命令通道（`0x02` / `0x82`）

### 2.1 帧格式

```text
命令  A5 | CMD | LEN(LE16) | PAYLOAD | CRC16(LE) | 5A
应答  A5 | CMD | STATUS | LEN(LE16) | PAYLOAD | CRC16(LE) | 5A
```

- CRC = **CRC-16/ARC**（反射多项式 0xA001、初值 0、无异或；`CRC16_IBM` @0x143890），
  覆盖 CMD 至负载末尾（应答含 STATUS 与 LEN）。
- 应答校验：首字节 A5、本次传输最后字节 5A、CRC、CMD 回显；STATUS=0 成功。
- **正版每条命令带一个尾部通道字节**（`... 5A 02` = 通道 2）：设备端可忽略。
- 正版每条命令前对 0x02/0x82 各做 `clear_halt`：Linux 上对健康端点 clear_halt
  不会复位主机侧 toggle，设备侧复位会失步——**仿真端必须在 CLEAR_FEATURE(HALT)
  时按"端点是否真的 halt 过"门控 toggle 复位**（详见 DMTOOL_PROTOCOL.md §1）。

### 2.2 命令字全表

| CMD | 名称 | 负载 | 语义 |
|---|---|---|---|
| `0x00` | START_CAP | 1 B 通道 | 开始采集 |
| `0x01` | STOP_CAP | 1 B 通道 | 停止采集 |
| `0x02` | IAP_PACK | 4+4096 B | 适配器自身固件分包（LE32 序号 + 数据） |
| `0x03` | IAP_END | 0 | 适配器固件结束 |
| `0x04` | START_TEST | 0 | 自测 |
| `0x05` | SETUP_BUARD | 10 B | 重配 CAN 位时序（seg1/seg2/sjw/分频 + FD），见 2.3 |
| `0x06` | 读版本 | 0 | 版本串 + NUL |
| `0x09` | GetUUID | 0 | OTP UUID 16 B |
| `0x0A` | 写 SN | 11 B | （正版设备写 OTP） |
| `0x0B` | 读 SN | 0 | SN 串 + NUL |
| `0x0C` | 退出 bootloader | 13 B | — |
| `0x0D` | GetBaudRate | 1 B 通道 | 回 10 B 实际位时序 |
| `0x0E` | 恢复出厂 | 15 B | — |
| `0x0F` | 停止周期发送 | 1 B 通道 | — |
| `0x10` | 保存参数 | 0 | 位时序写 flash，掉电保持 |

### 2.3 SETUP_BUARD / GetBaudRate 负载（10 B）

```text
[0]    通道
[1]    帧类型：0 = classic，非 0 = CANFD（GetBaud 应答据此 setCANType）
[2/3/4/5] 仲裁段 seg1 / seg2 / sjw / 分频
[6..9] 数据段同构（FD 时有效）
```

DMTool 预设第 0 项 = 仲裁 1M（分频 2/seg1 29/seg2 10，87.5%）+ 数据 5M
（分频 2/seg1 5/seg2 2）；第 1 项 = 1M + 数据 2M。电机升级流程（第 7 节）
实测 921600 UART + CAN 1M/5M 组合可用。

---

## 3. CAN 数据面（`0x03` OUT / `0x81`、`0x83` IN）

### 3.1 发送帧（22 字节头 + 负载，一帧一次传输）

```text
[0..3]   CAN ID（LE，低 29 位）；字节 3 位 6 = 扩展帧，位 7 = 远程帧
[4]      高 4 位 DLC；位 0 = FD，位 1 = BRS，位 2 = ID 自增，位 3 = 数据自增
[5]      通道（0 / 1）
[6..0xD] 主机侧状态（设备不用）
[0xE..0x11] 重复间隔 LE32
[0x12..0x15] 发送次数 LE32（普通 = 1）
[0x16..] 负载（GetLenFromDlc(DLC) 字节）
```

### 3.2 接收 / 回显记录流（两条流格式相同）

```text
[0..3]   CAN ID（LE；位 30 = 扩展，位 31 = 远程）
[4..0xB] 64 位纳秒时间戳（PTPC0 @SOF 捕获）
[0xC]    0
[0xD]    高 4 位 DLC；位 0 = FD，位 1 = 方向（0 收 / 1 发），位 2 = BRS，
         位 3 = 发送成功
[0xE]    状态：接收 0x00；发送成功 0x00（成败看 [0xD] 位 3）、失败 0x02；
         0xFF = 错误帧
[0xF]    通道
[0x10..] 负载（DLC 表补齐：9..15 → 12/16/20/24/32/48/64）
```

### 3.3 两条硬规则

1. **不能发零长度传输**（DMTool 视作 USB 故障）。
2. **每次传输必须以短包结束**（主机 32 KB 同步读；满包结尾要等超时，超时数据被丢）。
   高速下每次传输最多 511 B：8 字节帧的记录 24 B → 一次最多 21 条。

---

## 4. 心跳（`0x01` OUT）

`A5 10 11 00 | FF×15 + 0A 1C | CRC16 | 5A`（24 B），**10 ms 周期**，设备不回应。
正版抓包逐字节确认。作用：会话保活（设备端监测心跳，超时会话降级）。
与保存参数（0x10）同码但不同端点——设备按端点区分语义。

---

## 5. 电机 CAN 应用层（DM 电机协议摘要）

标准帧、1 Mbit（数据段 FD 时 5M）。常用帧（完整表见达妙手册/例程）：

| 用途 | 帧格式 |
|---|---|
| 使能 | `FF FF ID ID 01 ...` |
| 寄存器读 | 按 DM_Reg 枚举的读取命令（达妙手册/驱动 `read_motor_param`） |
| 寄存器写 | 按 DM_Reg 的写入命令 |
| 运动控制 | MIT 模式 / 位置速度模式帧 |

DMTool 的"读参数/读版本"任务即上述寄存器读取的打包：请求/应答都经适配器
记录流送到上位机，**采集（START_CAP）必须是开的**。

---

## 6. 适配器自身固件 IAP（`0x02` / `0x03` 命令）

```
packets = ceil(size/4096)
逐包: IAP_PACK = A5 02 1004 | LE32(分包序号) + 4096 B 数据 | CRC | 5A
全部成功后: IAP_END = A5 03 0000 | CRC | 5A
任一包失败: 记错误、关文件、整体返回（IAP_END 不发）
```

这是**升级适配器自己**的流程（DMTool F9 对话框）。对电机升级不适用。

---

## 7. 电机固件升级（两条路，均实测）

### 7.1 CAN 路（`FirmwareUpgradeCanTask::Run` @0x245ae0）——对 DM6006 无效

```text
① 进 bootloader:  TX 0x7FF, DLC8: 55 01 02 AA <id_le16> AA AA
② 等应答:          RX 0x7FE 含 "Aupgrade"（500 ms 超时）
③ 分块传输:        每块 8 KB，缓冲 0x2006 = 8198 B:
                   [0]=0x23  [1]=块序号  [2]=0x23  [3..4]=本块长度 LE16
                   [5..5+n]=数据(不足补 0)  [5+n]=尾字节
                   切成 8 字节一帧发到电机 ID，帧间 600 µs
④ 等块确认:        RX 0x7FE 含 "OK"（500 ms 超时）
最后一块:           块序号 = 0，[3..4] = 实际剩余长度，尾 = CRC-8(实际数据)
```

- **块序号 = 倒计数**：`(剩余字节-1) >> 13`（例：52524 B 镜像 → 首块 6，
  依次 5、4、3、2、1，末块 0）。正向 0 起的序列会被静默拒收 `[实测]`。
- **尾字节 = CRC-8**：多项式 0x8C（反射 0x31）、初值 0、仅数据区
  （`xor r15b,[rdx]` 后每位移位判 LSB 异或 0x8C——反汇编 246460 起）。固定 0x00
  会被静默拒收 `[实测]`。
- **对 DM6006 的实测结论**：①② 通（"Aupgrade" ✓），③④ 的块传输全帧 ACK 但
  0x7FE 零应答（所有节奏/变体）——**该电机 bootloader 版本未实现 CAN 数据阶段**
  （官方手册：需电机固件版本支持）。UART 路径（7.2）为正解。
- **V6217_04 复测定案（2026-09-22）**：电机经 UART 路径升级到 V6217_04 后复测——
  entry ✓，120 帧 IAP 块数据 @1.9 ms **120/120 全帧 ACK**（TX 记录 bit3 发送成功
  全置位、状态 0x00），0x7FE 仍零应答。帧全部到达、bootloader 收下不回应：CAN
  块确认机制在该 bootloader 上不存在，**CAN IAP 对本 DM6006 定案不可用**。
- **串口升级链路修复定案（2026-09-22）**：板子 UART TX 缓冲 2048→16384 后，
  DMTool 串口模式所需的全块突发（8198 B 一次 write）完整送达；裸测 7/7 块 OK、
  2.0 s 发完 52524 B，电机重启 V6217_04。板子串口桥升级 = 正版等价。

### 7.2 UART 路（`FirmwareUpgradeUartTask::Run` @0x2440c0）——✅ 完整实测成功

```text
① 握手:   TX 'X' (0x58) @921600
          RX "Enter Bootloader!\r\n"
② 数据:   每块 ONE 传输: [0x23][块序号][0x23][长度 LE16][数据][CRC-8 尾]
          （与 7.1 的缓冲同构！块序号倒计数同 7.1）
          经适配器 UART 通道 → 电机 UART（921600 固定）
③ 应答:   电机每 ~300 ms 回状态包:
          [b5 da][seq][13B 状态]":OK\r\n"
④ 完成:   "\r\n\r\n   Update firmware complete"
```

- **整块一次传输**（8198 B / 末块 3378 B），适配器设备侧缓冲并自己做 CAN/UART
  节奏——**主机不需要逐帧 600 µs**。
- 电机 bootloader 的 UART 速率固定 **921600**（4.8M/3M/2M/115200 扫描无应答）。
- 块间隔 ~300 ms（每块含 flash 擦写 + "OK"）。
- 状态包 `[b5 da][seq]...` 的 13 B 为加密/打包状态（不透明），尾部明文 ":OK"。
- **流控**：适配器 RX 满时 NAK USB OUT，DMTool 的写阻塞自适应——零丢帧。
  仿真端等价物：vendor RX 缓冲模式（CFG_TUD_VENDOR_RX_BUFSIZE）或重挂门控。

### 7.3 UART IAP 块格式实例（抓包原文，7 块完整序列）

```text
块6: 23 06 23 00 20 | cc02 64d5 15d7 ...（8192 B）| CRC  → 8198 B 一次传输
块5: 23 05 23 00 20 | 1cac fae4 b7e9 ...                 → 8198 B
块4: 23 04 23 00 20 | 33dc 85d3 e56f ...                 → 8198 B
块3: 23 03 23 00 20 | 7719 a7d4 7fd6 ...                 → 8198 B
块2: 23 02 23 00 20 | 826b ef5b 7c0a ...                 → 8198 B
块1: 23 01 23 00 20 | d22a a3c4 2e21 ...                 → 8198 B
块0: 23 00 23 2c 0d | 42be a9de e25a ...（3372 B）| CRC   → 3378 B
块间隔 ~300 ms；完成后 "\r\n   Update firmware complete"
```

（数据为镜像原文——APP_DM6006(V3) 固件为加密/打包格式，bootloader 侧解密。）

---

## 8. 时序与节奏要求汇总

| 参数 | 值 | 来源 |
|---|---|---|
| 心跳周期 | 10 ms | EP 0x01 抓包 |
| CAN IAP 帧间（CAN 路） | 600 µs | DMTool 反汇编 0x258 |
| CAN IAP 块确认超时 | 500 ms | DMTool 反汇编 |
| UART IAP 块间隔 | ~300 ms | 抓包（含电机擦写） |
| UART IAP 波特率 | 921600 固定 | 扫描实测 |
| 命令应答超时 | 500 ms（同步读） | DMTool 反汇编 |
| 心跳超时 | 10 ms | DMTool 反汇编 |

---

## 9. 仿真端的已验证状态与缺口（HPM5321，2026-09-22）

**已验证**：命令通道（含 clear_halt 门控修复）6/6、CAN 发送/回显/接收记录、
帧类型按请求逐帧、SETUP_BUARD、保存参数、心跳透传、CDC 串口桥（921600 +
'X' 握手 + 透明透传）、libhcs 并存与优先级、RTT 与吞吐与 HEAD 基线一致。

**缺口**：CAN IAP 数据阶段依赖电机 bootloader 版本（本 DM6006 不支持，走 UART）；
下行突发背压（vendor RX 缓冲模式）待实现——对 DMTool CAN 大流量操作有意义，
对 UART IAP 不必需（CDC 桥自带流控）。

---

## 10. 调试工具箱（2026-09-22 会话产物，/tmp/handoff-sp/ab/）

| 脚本 | 用途 |
|---|---|
| `dmtool_chalt.py` | 命令通道 clear_halt A/B（每命令 clear_halt 序列 6 条） |
| `iap_full_test.py` | CAN IAP 全流程（旧块格式） |
| `iap_diag_test.py` | CAN IAP 分辨性测试（逐帧回显统计 + entry 复测） |
| `iap_fixed_test.py` | CAN IAP（倒计数块号 + CRC-8 尾） |
| `iap_stall_measure.py` | 电机背压窗口测量（探针帧法） |
| `iap_entry_test.py` | CAN entry + 0x7FE 应答监听 |
| `usbmon_3u.bin` | 正版适配器完整 USB 抓包（10+ MB，含成功升级全程） |

抓包方法：`printf ' \n' | sudo -S bash -c 'modprobe usbmon; cat
/sys/kernel/debug/usb/usbmon/3u > capture.txt'`（debugfs 文本流，
按 `:设备号:端点` 过滤，`C` 行带数据 `= hex`）。
