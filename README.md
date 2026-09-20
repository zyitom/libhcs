# libhcs

> **文档类型**：背景说明 + 上手指引
> **适用范围**：整个仓库（host SDK 与全部四块板）
> **状态**：现行有效
> **相关文档**：[AGENTS.md](AGENTS.md)（开发约束与文档规范） · [ENV.md](ENV.md)（外部工具下载入口） · [HOST_TUNING.md](HOST_TUNING.md)（延迟/吞吐实测与主机调优）

## 摘要

libhcs 是 [无下位机控制系统 HCS](https://github.com/zyitom/libhcs) 的核心通讯部分。
本文档面向第一次接触本仓库的人，讲清四件事：**装什么依赖**、**怎么把 host SDK 和固件编出来**、
**怎么把固件烧进板子**、**它实测能跑多快**。芯片内部的外设设计、踩坑记录、烧录细节在各板
自己的文档里，见文末索引；延迟与吞吐的完整测量在 [HOST_TUNING.md](HOST_TUNING.md)。

## 本文导航

| 你想做什么 | 看哪一节 |
|---|---|
| 了解版本状态 | [libhcs v3](#libhcs-v3) |
| **看它能跑多快**（延迟 / 吞吐实测） | **[性能实测速览](#性能实测速览)** |
| 在 PC 上编 SDK | [Host SDK 编译](#host-sdk-编译) |
| 编固件 | [固件编译与烧录](#固件编译与烧录) |
| 第一次烧板子 | [烧录 Bootloader](#烧录-bootloader首次或更新引导) |
| 日常更新固件 | [烧录 App（USB DFU）](#烧录-appusb-dfu) |
| 认板子 / 查 PID | [各板 USB 标识](#各板-usb-标识) |
| 找某块板的深入文档 | [各板文档索引](#各板文档索引) |
| **调低 USB 延迟** | **[HOST_TUNING.md](HOST_TUNING.md)**（主机侧设置，每次重启要重做） |

## libhcs v3

libhcs v3 正在开发！目前处于测试阶段，无良好的文档/教程。

## 性能实测速览

下面是这套链路在**本仓库自带工具**下的实测数字，用来回答"它到底能跑多快、我该期待什么"。
测量条件、对照数据、每条结论的证据等级全部在 **[HOST_TUNING.md](HOST_TUNING.md)**，
这里只放结果。**测量条件**：2026-08-04，两块 HPM5321 双 CAN-FD 硬件，`BOARD=hpm5321`
的 `release` 固件，主机已跑过
`host-tuning.sh` `[实测]`。

**延迟**（1 kHz 控制环占空比，即两次事务之间核会真正空闲）：

| 测什么 | 工具 | min | p50 | p99 | p99.9 |
|---|---|---|---|---|---|
| 板级 **CAN-FD** 单向（A.CAN0 -> B.CAN0，过 USB + 固件 + CAN 线） | `dual_board_test latency` | 77 us | **99 us** | 122 us | 129 us |
| 纯主机路径 RTT（提交 -> xHCI -> 设备 EP0 -> 中断 -> 唤醒） | `usb_ep0_rtt` | 49 us | **68 us** | 77 us | 83 us |

> **第一行是 CAN-FD，不是 classic CAN。** `dual_board_test` 的 `use_fdcan()` 默认返回
> true，只有 `HCS_CAN_CLASSIC=1` 才发经典帧——这一点此前没有写在任何地方，而 99 us
> 其实**低于下面吞吐表里 classic CAN 自己的帧时 116.8 us**，物理上不可能是 classic 的
> 单向延迟。换 classic 会慢 60-75 us，见下表。`[实测 2026-08-24]`

**换一套异构 rig 的对照**（mc02 <-> HPM5321 DualCan，CAN0<->CAN0 / CAN1<->CAN1，
`mixed_board_test latency`，每格 4000 帧、0 超时 0 损坏）：

> **测量条件与上表不同，不要直接比。** 两者都跑过 `host-tuning.sh`，但上表把事件线程
> 绑到了 P 核，下表**没有绑**（`mixed_board_test` 不支持），而不绑核是 HOST_TUNING 1.3
> 记的尾部最差一档——所以下表只有 min / p50 / avg 反映板子，p99 以上是主机调度。
> `[实测 2026-08-24]`

| 方向 | classic p50 | CAN-FD p50 |
|---|---|---|
| 5321 -> mc02 | 201.2 us | 131.2 us |
| mc02 -> 5321 | **179.5 us** | **123.7 us** |

两条结论：**classic 比 FD 慢 55-75 us**（就是两者线上帧时之差），以及**异构 rig 比
两块 5321 慢约 25 us**（mc02 是 USB Full-Speed，一次 bulk 事务的线上时间比 High-Speed
长几十 us）。拿本表任何数字去推 classic CAN 或推异构链路之前，先看清楚测的是哪一种、
以及主机调没调优。

**吞吐**：

| 测什么 | 上限 | 卡在哪 |
|---|---|---|
| CAN-FD 单总线（1M 仲裁 / 5M 数据，8 字节） | **19870 帧/s** | **CAN 线速**（每帧 50.3 us），主机和固件都动不了 |
| Classic CAN 单总线（1M，8 字节） | **8560 帧/s** | 同上（每帧 116.8 us） |
| 一块双 CAN 板合计 | 约 39700 帧/s | 两条总线各自独立到顶 |
| USB bulk 包率，单板 | 约 **56000 包/s** | USB 路径（主机 CPU 只用了 23%） |
| USB bulk 包率，双板同挂一个控制器 | 约 **105000 包/s** | 上限是**每设备**的，加板子能叠加 |

**动手调之前，先按收益排序看这三条**（细节见 HOST_TUNING.md）：

1. **固件必须是 `release`（`-O3`）构建。** 换成 `debug`（`-Og`）会让板级 p50 从 99 us 涨到
   120 us——**比全部主机侧调优加起来还大**，而且版本字符串看不出区别。见 8.4。
2. **主机 `governor=performance`。** 在 1 kHz 占空比下值 16-18 us 的板级 p50；
   紧凑压测下则完全为零——**别拿压测结论去配置控制环**。见 1.1 / 1.2。
3. **把事件线程绑到某个 P 核。** 不绑核是最差的一档（p99.9 176 us / max 458 us）。
   注意绑到**隔离核**相比绑到普通 P 核并无可测收益，`isolcpus` 未必值得占掉一个物理核。见 1.3。

> **除内核 cmdline 外，主机侧设置全部不持久化**，每次重启都要重跑 `sudo ./host-tuning.sh`。

## Host SDK 编译

### 依赖

见 [ENV.md](ENV.md)。

### 编译

```bash
cmake --preset linux-debug -S host
cmake --build host/build
```

如需同时构建示例程序（`rx_monitor`、`uart_stress` 等），加上 `-DBUILD_EXAMPLES=ON`：

```bash
cmake --preset linux-debug -S host -DBUILD_EXAMPLES=ON
cmake --build host/build --target rx_monitor
```

如果系统默认 GCC 版本低于 14，需手动指定编译器：

```bash
cmake --preset linux-debug -S host -DBUILD_EXAMPLES=ON \
    -DCMAKE_CXX_COMPILER=g++-14 -DCMAKE_C_COMPILER=gcc-14
```

## 固件编译与烧录

### 依赖

见 [ENV.md](ENV.md)。

> STM32 板（`c_board`、`mc02`）用 ARM GCC。RISC-V 板（`ch32_board`、`hpm_board`）
> 用另一套工具链，见各自章节。

### 编译

以 mc02（STM32H723VG）为例，在仓库根目录执行：

```bash
cmake --preset debug -S firmware/mc02
cmake --build firmware/mc02/build --target mc02_app mc02_bootloader
```

c_board（STM32F407VG）同理，把路径换成 `firmware/c_board`、目标换成 `c_board_app c_board_bootloader` 即可。

产物：

- App：`firmware/<board>/build/app/<board>_app.elf` / `.bin` / `.dfu`
- Bootloader：`firmware/<board>/build/bootloader/<board>_bootloader.elf`

`debug` 可替换为 `release`。

### 烧录 Bootloader（首次或更新引导）

Bootloader 位于 Flash 起始地址 `0x08000000`，需要用调试器（**只用 J-Link**，不用
ST-Link / OpenOCD，见 [ENV.md](ENV.md)「宿主机调试工具」）烧录一次，例如：

App 之后即可通过下面的 DFU 流程烧录，无需调试器。

### 烧录 App（USB DFU）

日常入口是 `./tools/flash.sh <target>`：编对应 preset（默认 release），再 `dfu-util`。

```bash
./tools/flash.sh mc02            # 默认 release
./tools/flash.sh mc02 debug      # 只有末尾是 debug 才换成 debug
./tools/flash.sh c_board
./tools/flash.sh hpm5321
```

App 镜像 `*.dfu` 已带好镜像哈希与 DFU 后缀。下面是脚本做的手工步骤。

1. **让设备进入 DFU 模式**，任选其一：
   - 由上位机软件发起 DFU 重启请求（App 运行时触发）——最常用；
   - Flash 中没有有效 App 时，Bootloader 会自动停在 DFU 模式；
   - 复位时按住 **KEY** 键（mc02 = PA15，低电平有效）。

> ⚠️ **KEY 这一条只对 mc02 / c_board 成立。HPM 板（`hpm5321` 的两个变体、`hpm6e8y`）
> 上没有任何实体按键** `[实测 2026-09-09]`：force-stay 引脚只是引出的焊盘，要复位时
> 短接到 GND（窗口约 1 ms），或改用 `-DBOARD_BOOTLOADER_MODE=manual` 的 bootloader。
> 见 [firmware/hpm_board/boards/hpm6e8y/README.md](firmware/hpm_board/boards/hpm6e8y/README.md)。
> `ch32_board` 也没有恢复按键，见其 README。

2. **确认设备已枚举**（应能看到 `[a511:0723]`，接口为 alt 0 `Internal Flash`）：

   ```bash
   dfu-util -l
   ```

3. **烧录**（`-a 0` 选择 Internal Flash 接口）：

   ```bash
   dfu-util -a 0 -D firmware/mc02/build/app/mc02_app.dfu
   ```

   若同时接了多个 DFU 设备，加 `-d 0xa511:0x0723` 指定目标。

烧录完成后 Bootloader 会校验镜像并自动复位跳转到 App。此时 `dfu-util` 可能打印设备掉线/状态读取失败之类的提示，属于正常现象（设备自行复位了）。

### 各板 USB 标识

| 板型       | 芯片         | App PID  | Bootloader 产品名          |
| ---------- | ------------ | -------- | -------------------------- |
| c_board    | STM32F407VG  | `0xF407` | `HCS DFU Bootloader`      |
| mc02       | STM32H723VG  | `0x0723` | `HCS DFU Bootloader`      |
| ch32_board | WCH CH32H417 | `0xD403` | `HCS Bootloader v<版本号>` |
| hpm_board HPM5321 单 CAN 版 | HPM5321 | `0x6877`（VID `0x34B7`，见下）；bootloader `0x5321` | `HCS Agent v<版本号>` |
| hpm_board HPM5321 双 CAN-FD 版 | HPM5321 | `0x6877`（VID `0x34B7`，见下）；bootloader `0x5322` | 同上 |
| hpm_board `hpm6e8y` | HPM6E8Y | `0x6E84` | 同上 |

VID 除 HPM5321 应用外均为 `0xA511`。

> **HPM5321 应用借用达妙 USB2FDCAN 的身份 `0x34B7:0x6877`**，好让达妙上位机 DMTool 直接
> 打开本板；host SDK 靠产品串认板，两块板靠 EP0 报告的 CAN 路数区分。DFU bootloader 仍是
> `0xA511`，PID 由硬件决定：上电读 OTP 第 25 个字判断板型，报 `0x5321` 或 `0x5322`，
> `-DBOARD=hpm5321` 出的单个镜像同时服务两块板。见
> [firmware/hpm_board/AGENTS.md](firmware/hpm_board/AGENTS.md)「DMTool 兼容」与
> [firmware/hpm_board/boards/hpm5321/README.md](firmware/hpm_board/boards/hpm5321/README.md)。

> **`hpm_board` 烧录前必须核对板级变体。** `BOARD` 的默认值是 `hpm5321`。
> 不带 `-DBOARD=<变体>` 编出来的镜像烧到 6E8Y 系列板上会改掉 PID 并配错引脚
> （HPM5321 的两块板之间不受此影响，同一镜像通用）。
> **版本字符串区分不出构建类型和变体**——它来自 `git describe`，`debug` 和 `release` 完全一样。
> 多块同型号板同时在线时用 `dfu-util -S <序列号>` 逐块烧（`dfu-util -l` 列序列号）。

### ch32_board 的差异

RISC-V 双核，工具链是 `riscv32-unknown-elf-gcc`（不是 `arm-none-eabi-gcc`），
编译前需 `export GNURISCV_TOOLCHAIN_PATH=~/3rd_party/hpm`（`[前机路径]`，含义见
[AGENTS.md 开发机环境路径约定](AGENTS.md#开发机环境路径约定重要先读这条再看任何路径)）：

```bash
cmake --preset debug -S firmware/ch32_board && cmake --build firmware/ch32_board/build
```

- **首次烧录**（含 bootloader）用 WCH-Link 烧 `build/ch32_board_merged.hex`，
  之后 App 可走 DFU：`./flash-ch32.sh`。
- Bootloader 就是 V3F 启动核镜像本身（`ch32_board_boot`），"启动 App" 是唤醒 V5F
  而非跳转。
- App 的 `.dfu` 只带 DFU 后缀、**不带** `ImageHash` 后缀：这块板的 bootloader 会
  自己哈希烧进去的内容并写进独立的 metadata 记录。

细节见 `firmware/ch32_board/README.md`。

## 各板文档索引

| 板子 | 入口文档 | 深入阅读 |
|---|---|---|
| `c_board`（STM32F407） | [firmware/c_board/AGENTS.md](firmware/c_board/AGENTS.md) | — |
| `mc02`（STM32H723） | [firmware/mc02/AGENTS.md](firmware/mc02/AGENTS.md) | [README.md](firmware/mc02/README.md)（外设与低延迟设计） · [PACKET_RATE_LOG.md](firmware/mc02/PACKET_RATE_LOG.md)（包率实测） |
| `ch32_board`（CH32H417） | [firmware/ch32_board/AGENTS.md](firmware/ch32_board/AGENTS.md) | [README.md](firmware/ch32_board/README.md) · [PITFALLS.md](firmware/ch32_board/PITFALLS.md)（上板前必读） · [PROGRESS.md](firmware/ch32_board/PROGRESS.md) |
| `hpm_board`（HPM6E8Y/5321） | [firmware/hpm_board/AGENTS.md](firmware/hpm_board/AGENTS.md) | [BUILD_ENVIRONMENT.md](firmware/hpm_board/BUILD_ENVIRONMENT.md) · [PITFALLS.md](firmware/hpm_board/PITFALLS.md)（选型与踩坑） · [USB_OPTIMIZATION_LOG.md](firmware/hpm_board/USB_OPTIMIZATION_LOG.md)（USB 调优） |

完整文档清单见上表与各板 `AGENTS.md` 的「相关文档」一节。线协议（CAN 记录流
位布局、DLC 表、能力协商、bxCAN 部署约束）见 [core/PROTOCOL.md](core/PROTOCOL.md)。
