# 崩溃记录与场下复现构思

> **文档类型**：背景说明（设计提案）
> **适用范围**：`firmware/mc02/`（机制可推广到其余板卡）与 host SDK
> **状态**：**提案，尚未实施**。本文只记录构思与权衡，不描述现有行为
> **相关文档**：[firmware/mc02/AGENTS.md](firmware/mc02/AGENTS.md) · [firmware/mc02/README.md](firmware/mc02/README.md) · [仓库根 AGENTS.md](AGENTS.md)

## 摘要

板子上场崩溃后，现场信息目前会完全丢失，导致赛后无法复现。本文提出一套
「崩溃时写入不清零 SRAM → 复位 → 重连后上报 → 上位机确认后才清除」的机制，
并说明为什么需要板子与上位机两侧数据拼接才能复现。核心约束是：正常运行路径
零开销、崩溃当下不碰 flash 与 USB、上报可靠性由应用层 ack 保证。

本文是评估用的设计稿，**不构成实施承诺**。

## 本文导航

- 第 1 节：当前行为的两个缺口（为什么现在复现不了）
- 第 2 节：核心机制与整体时序
- 第 3 节：板子侧数据结构与生命周期
- 第 4 节：协议改动
- 第 5 节：上位机侧职责
- 第 6 节：场下复现流程与前置条件
- 第 7 节：分阶段实施顺序
- 第 8 节：未决权衡

---

## 1. 当前行为的两个缺口

结论先行：**现在板子上场崩溃，信息 100% 丢失，且板子会停在 DFU 不再转发。**

### 1.1 崩溃信息复位即丢

`app/src/utility/assert.cpp` 中的 `assert_func()` 已经记录了崩溃位置：

```cpp
const char* volatile assert_file = nullptr;
volatile unsigned int assert_line = 0;
const char* volatile assert_function = nullptr;
```

但这三个是普通全局变量，位于 `.bss`。而 `libhcs_fault_recover()` 在没有调试器时
会 `NVIC_SystemReset()`，复位后启动代码将 `.bss` 清零，**三个值全部丢失**。

因此现状是：接了调试器才能看到崩溃现场；上场跑的时候崩了，只剩下一块自己重启的
板子，没有任何线索。[实测，读代码确认]

### 1.2 崩溃后停在 DFU

`libhcs_fault_recover()` 的恢复路径是请求 DFU 后复位：

```cpp
libhcs::firmware::utility::boot_mailbox.request_enter_dfu();
__DSB();
__ISB();
NVIC_SystemReset();
```

bootloader 消费该请求后停在 DFU 模式，**不跳转 app**。对比赛场景意味着：一次偶发
HardFault 导致 CAN 转发永久停止，所有电机失联，只能靠人工物理断电重上电恢复
（断电后 DTCM 掉电、mailbox 清空，才会重新跳 app）。[实测，读代码确认]

该设计对**开发场景**是合理的：崩溃说明有 bug，停在 DFU 保证还能刷新固件。但比赛
场景的诉求相反——需要尽快自愈，而非等待被刷机。

> 本文其余部分只讨论 1.1 的记录与上报机制。1.2 是独立问题，可单独修复，见第 7 节 P0。

---

## 2. 核心机制与整体时序

### 2.1 机制一句话

崩溃时把现场写进一块**复位不清零的 SRAM**，复位后由 app 读出，等 USB 会话建立后
上报给上位机，**收到上位机确认才清除**。

关键在于「复位不清零」：必须是 `NOLOAD` 段，启动代码既不清零也不初始化它。仓库里
已有同类机制——`.boot_mailbox` 段（`0x2001FFC0`，DTCM 顶部 MAILBOX 区域），app 与
bootloader 靠它传递 DFU 请求。新的崩溃记录照此实现。[实测]

崩溃当下**只写内存**，不碰 flash、不碰 USB：

- flash：H7 上一次擦写是毫秒级，且崩溃时系统状态已不可信，很可能写坏
- USB：fault 本身可能就是 USB 栈引起的；且 bulk 传输要等主机下一个 1 ms frame 来取，
  期间 CAN 转发完全停摆。**崩溃现场不具备可靠通信的条件**

### 2.2 整体时序

```
【正常运行】
   T0   app 转发中，crash_record 为空 (magic = 0)
        |
【崩溃】
   T1   HardFault / assert 触发
   T2   fault handler 写 .crash_record            <- 几微秒，纯内存写
        |  不碰 flash、不碰 USB
   T3   NVIC_SystemReset()
        |
【恢复】
   T4   bootloader 跳 app（不进 DFU，见 P0）
   T5   app 启动，检查 .crash_record
        |  magic 有效 -> 标记「有待上报」
        |  同时检查连续崩溃计数 -> 决定是否改为进 DFU
   T6   USB 重新枚举                               ~几十 ms [推断，未上板]
        |
【上报】
   T7   上位机发 kStart，板子回 kStartAck
   T8   板子发 kCrashRecord                        <- 会话建立后第一时间
   T9   上位机落盘（record + 命令历史）
   T10  上位机回 ack
   T11  板子清除 .crash_record (magic = 0)         <- 只有到这一步才清
        |
【场下】
   T12  addr2line 解析 PC -> 源码行
   T13  重放命令序列复现
```

### 2.3 为什么上报要等到 T7 之后

不在枚举完成（T6）就发，是因为那时上位机可能尚未打开设备、尚未开始读。数据要么堆在
USB 缓冲区，要么被当作未知字段丢弃。`kStartAck` 之后才是明确的「两侧都已就绪」。

### 2.4 开销

正常运行路径**零开销**：T2 的代码只在崩溃时执行，正常运行一条指令都不跑；T8 一次
上电最多发一次，约几十字节。

明确**不做**以下设计，它们才是会持续增加延迟的：周期性健康上报、运行时日志
ring buffer、热路径插桩计数。崩溃记录的本质是「平时是死的，出事才活一次」。

---

## 3. 板子侧数据结构与生命周期

### 3.1 数据结构

```c
struct CrashRecord {
    uint32_t magic;          // 0 = 空, "HCS1" = 有待上报。最后写，兼作提交屏障
    uint32_t boot_id;        // 单调递增，上位机据此去重
    uint32_t crash_count;    // 连续崩溃次数，崩溃循环保护用
    uint32_t uptime_ms;      // 崩溃前运行时长，区分「一上电就崩」与「跑了十分钟才崩」

    uint32_t pc, lr, psr;              // 异常栈帧
    uint32_t cfsr, hfsr, bfar, mmfar;  // fault status 寄存器，崩溃时直接读

    const char* assert_file;           // 存指针而非字符串
    const char* assert_function;
    uint32_t    assert_line;

    uint32_t fw_version;     // 校验上面两个指针属于哪个固件
    uint32_t crc;            // 完整性，防止半写状态被误读
};
```

约 60 字节。

**两个实现细节**：

- `magic` **最后写**，作为提交屏障。写到一半又崩，读出来是无效记录，而不是有效
  magic 配上半截脏数据。现有 `BootMailbox` 就是这个思路，注释已写明。[实测]
- `assert_file` / `assert_function` 存**指针**不存字符串内容。它们指向 flash 中的
  `.rodata`，复位后地址不变，解引用仍然有效，几个字节就存下完整文件名与函数名。
  代价是重新烧录固件后旧指针会指向错误位置，因此需要 `fw_version` 校验，对不上就
  只采信寄存器部分。

`.crash_record` 需要单独的 NOLOAD 段。注意现有 MAILBOX 区域只有 64 字节且已被
`BootMailbox` 占用，需要另划空间（DTCM 当前仅用 8.8%，空间充裕）。[实测]

### 3.2 生命周期

```
    空 ──[崩溃，fault handler 写入]──> 待上报
    ^                                    |
    |                                    | [会话建立，发出]
    |                                    v
    └────[收到上位机 ack]──────────  已发送（未确认）
                                         |
                                         | [未等到 ack，下次会话]
                                         └──> 重发
```

**「已发送」与「空」必须是两个不同状态。** 发出去不等于安全落地：

USB bulk 自身可靠（有 ACK 与重传），但那只保证字节到达主机的 USB 栈，**不保证上位机
进程拿到了，更不保证已写入文件**。上位机可能正在重启、可能读到后落盘前自己崩了、
可能该版本软件尚未实现此字段。发完即清，这些情况下记录永久蒸发——而这恰恰是比赛
现场最可能发生的。

因此清除必须由**应用层 ack** 驱动。未收到 ack 就在下次会话建立时重发，反复发送直到
被接走。上位机用 `boot_id` 去重，重复记录丢弃但照常 ack。

由此得到的保证是：**只要板子不断电，崩溃信息不会丢失。**

---

## 4. 协议改动

只需新增两项：

- `DataId::kCrashRecord = 30`，接在现有 `kUart3Config = 29` 之后 [实测：现有枚举
  最大值为 29]
- 下行方向的 ack。可复用同一 id 以 payload 区分方向，或按现有 `kXxxConfig` 模式
  新增一个配置类 id

id >= 15 需走扩展 2 字节字段头。一次上电仅发一次，该开销可忽略。

---

## 5. 上位机侧职责

控制逻辑全部运行在 x86 上位机，板子只是转发管道。因此**触发崩溃的输入序列只存在于
上位机侧**，板子不留存。上位机需要：

1. **常驻 ring buffer**：滚动保存最近 3~5 秒发出的全部 CAN 命令及单调时钟时间戳。
   x86 上无需节省，内存开销可忽略
2. **监听 USB 断开事件**并打时间戳
3. **重连建立会话后**接收 `kCrashRecord`
4. **落盘为自包含文件**：crash record + 断连时刻前后的 ring buffer dump + 固件版本
   + 墙上时间。写进同一个文件，避免赛后跨多个日志对时间戳
5. **落盘成功后**才回 ack

---

## 6. 场下复现流程与前置条件

拿到落盘文件后：

```bash
# 1. 定位崩溃指令（需要当时那一版固件的 ELF）
arm-none-eabi-addr2line -f -e mc02_app.elf 0x0804xxxx

# 2. 解析 fault 类型
#    CFSR 各位域直接对应 STM32H7 参考手册的 fault 原因 [RM]

# 3. 重放文件中的命令序列复现
```

### 6.1 前置条件：必须存档 ELF

**每次比赛所用固件的 ELF 必须存档。** `addr2line` 依赖 ELF 中的调试信息才能把地址
翻译成源码行。没有存档 ELF，crash record 里的 PC 就只是一串无意义的数字，整套机制
的价值归零。

相关的既有条件：本仓库 mc02 的 Release 构建已从 `-g0` 改为 `-g`
（`firmware/mc02/cmake/gcc-arm-none-eabi.cmake`）。烧录进 flash 的字节与此前逐字节
相同（`.debug_*` 非 PT_LOAD 段），但主机侧 ELF 因此保留了完整 DWARF，可用于地址解析。
[实测：`.bin` 与 `.text` md5 一致，指令条数 29433 相同]

建议 `fw_version` 中带上 git commit hash，落盘文件名同样带上，以便直接 checkout 到
对应版本。

---

## 7. 分阶段实施顺序

| 阶段 | 内容 | 依赖 | 备注 |
|---|---|---|---|
| **P0** | `libhcs_fault_recover()` 改为正常复位，仅连续崩溃时进 DFU | 无 | 独立，比赛安全收益最大 |
| **P1** | `.crash_record` NOLOAD 段 + fault 时填充 | 需修改 linker script | 见下方纪律说明 |
| **P2** | `DataId::kCrashRecord` + 会话后上报 + ack 清除 | P1 | |
| **P3** | 上位机 ring buffer + 落盘 + ack | P2 | |
| **P4** | ELF 存档流程 + 解析脚本 | P3 | 不可跳过，见 6.1 |

**P0 独立于其余全部阶段**，可单独实施。

P1 需修改 `firmware/mc02/bsp/linker/STM32H723VGTx_APP.ld`。按仓库纪律，链接脚本
仅在用户明确要求时方可由 AI 编辑。

fault handler 本身位于 CubeMX 生成的 `bsp/cubemx/Core/Src/stm32h7xx_it.c`，禁止直接
修改；但它已调用 `libhcs_fault_recover()`，该函数定义在 `app/src/utility/assert.cpp`
（非生成代码），**全部改动可在此完成，无需触碰生成文件**。[实测]

---

## 8. 未决权衡

### 8.1 断电窗口

NOLOAD SRAM 断电即失效。风险窗口是「复位 → 会话建立」这几十至几百毫秒。

比赛中板子自行重启，人不会在该窗口内拔电，因此 SRAM 方案基本够用。要彻底防断电需
持久化到 flash，但崩溃当下写 flash 不安全（状态不可信 + 毫秒级耗时），只能在恢复后
系统健康时补写，且需绕开 bootloader 的 SHA-256 校验区域，复杂度上一个台阶。

**倾向**：先做 SRAM 版本运行一个赛季，若实际发生过丢失再加 flash 持久化。
[推断，未上板]

### 8.2 是否新增 USB 端点

**倾向不新增**，复用现有 vendor 通道加 `DataId`。理由：

STM32H723 的 OTG_FS 仅有 1.25 KB 专用 FIFO RAM，需分配给一个共享 RX FIFO 与每个 IN
端点各一个独立 TX FIFO。[RM] 而 TX FIFO 大小直接影响 bulk IN 吞吐，新增端点只能从
主数据端点分走 FIFO——USB Full-Speed 的 12 Mbps 本就是该板的系统瓶颈（LQFP100 未引出
HS PHY）。为一条一次上电仅发一次的记录切走最稀缺资源，性价比极低。

此外 bulk IN 端点需主机持续 poll，空端点的每次 NAK 都是真实的总线事务。

新增端点合理的场景是：需要不同传输类型（如 interrupt 端点换确定性延迟）、需要独立
流控避免 head-of-line blocking、或数据量大且持续。崩溃记录均不符合。

### 8.3 记录条数

当前构思只保留最近 1 条 + 累计计数。若需诊断「连续多次崩溃，每次原因不同」的场景，
可扩展为保存最近 N 条的 ring buffer。DTCM 空间充裕，成本主要在协议与上位机侧复杂度。

暂**倾向单条**，待实际需求出现再扩展。
