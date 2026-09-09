# hpm6e8y 板子适配完整记录

> **文档类型**：过程记录（适配全过程）+ 现行操作步骤（烧录）
> **适用范围**：`firmware/hpm_board/boards/hpm6e8y/`，HPM6E00 芯片 / hpm6e00evk 等效硬件
> **状态**：现行有效（烧录步骤仍是当前做法）
> **相关文档**：[../../AGENTS.md](../../AGENTS.md)（构建与烧录） · [../../BUILD_ENVIRONMENT.md](../../BUILD_ENVIRONMENT.md)（工具链）

## 摘要

本文件是把这块板**从零适配起来**的完整记录，重点解决一个问题：**怎么把固件正确烧进去
并让它自启动**。当初卡住最久的两个坑是 **FCFG 选错 option** 和 **bootloader 烧录地址
偏移写错**，两者都会让芯片启动失败、回退到 Boot ROM ISP 模式（`lsusb` 显示 `34b7:0006`）。

**赶时间就直接看这三节**：[验证 FCFG 的方法](#验证-fcfg-的方法)、
[正确烧录步骤](#正确烧录步骤)、[完整避坑清单](#完整避坑清单)。

基本前提：HPM6E80 固件运行在 HPM6E00 芯片上，hpm6e00evk 等效硬件，
XPI NOR Flash 使用 FCFG option 1（`0xfcf90002`）。

> ✅ **已核实（2026-09-08）**：本文件原先写着"normal CAN0 实际为 **MCAN4/PZ00-PZ01**"，
> 那是**错的**。以代码为准——`boards/hpm6e8y/app/board_app.hpp` 的 `kCanPorts[]` 明确是
> **CAN0 = HPM_MCAN0，TX PC00 / RX PC01**，与
> 下方「板级对外接口现状」的 CAN 引脚表一致。矛盾消除。`[实测]`

> **路径说明**：本文出现的 `/home/zyi/3rd_party/...`、`/home/zyi/Desktop/libhcs/...`
> 等绝对路径均为 `[前机路径]`，记录的是上一台开发机上真实可用的位置；当前机器未安装
> 这些工具属正常现象，含义见
> [仓库根 AGENTS.md 的开发机环境路径约定](../../../../AGENTS.md#开发机环境路径约定重要先读这条再看任何路径)。

---

## 目录

1. [背景](#背景)
2. [OTP 分析](#otp-分析)
3. [FCFG：Flash 配置的命门](#fcfgflash-配置的命门)
4. [Linker Script 与 Flash 内存布局](#linker-script-与-flash-内存布局)
5. [烧录地址偏移陷阱（第一次失败根因）](#烧录地址偏移陷阱第一次失败根因)
6. [正确烧录步骤](#正确烧录步骤)
7. [HPM6E80 vs HPM6E00](#hpm6e80-vs-hpm6e00)
8. [安全启动与密钥](#安全启动与密钥)
9. [调试与救砖](#调试与救砖)
10. [完整避坑清单](#完整避坑清单)

---

## 背景

- **芯片**: HPM6E00（USB PID `34b7:0006`，Boot ROM 模式）
- **板子**: hpm6e00evk 等效硬件，使用**外部 XPI NOR Flash**（4MB）
- **固件**: 来自 `firmware/hpm_board/`（超级构建）的 USB 数据固件
- **原始 board**: `hpm6e80ivm1`（FCFG `0xfcf90001`，option 0）
- **新 board**: `hpm6e8y`（FCFG `0xfcf90002`，option 1）

---

## OTP 分析

### OTP 是什么

OTP（One-Time Programmable）是芯片内部的一次性可编程存储器。物理上每个 bit
出厂时全是 `1`，编程时把指定 bit 烧成 `0`，**永远无法从 `0` 恢复为 `1`**。
所以 OTP 只能「追加」烧录，已有内容无法擦除或修改。

### 本芯片 OTP 内容

使用 `hpm_manufacturing_cmd` 读取（memory_id=`0x20000`）：

```
OTP 大小: 512 bytes (128 words × 4 bytes)
USB VID:PID: 34b7:0006 → HPM6E00
ROM Version: V1.2.0
SoC LifeCycle: 0x04 (已配置，非空白)
```

关键 OTP 字段：

| Word | 值 | 含义 |
|------|-----|------|
| 0 | `0x30400016` | Device Config: boot_mode=6 (Primary Boot + ISP fallback) |
| 1 | `0x10000001` | 扩展配置 |
| 8-11 | 16 bytes | UUID/Unique ID |
| 21 | `0x03A008CC` | Flash 配置相关 |
| 25 | `0x00000006` | 编程计数/配置标志 |
| 32-54 | 92 bytes | **SRK Hash**（Super Root Key，安全启动验签用） |
| 69-75 | ASCII | `"usb_en_ec2canfd by damiao"` — 开发者注释 |
| 77 | `0x00000001` | **OTP_PROGRAM_COMPLETE**（OTP 编程完成标志） |
| 88-91 | 16 bytes | **EXIP KEK**（外部镜像加密密钥） |

### 安全状态详解

```
Word 0 = 0x30400016 = 0b_0011_0000_0100_0000_0000_0000_0001_0110

Bit[3:0]  BOOT_MODE         = 0x6  → Primary Boot (失败回退 ISP)
Bit[4]    DBG_AUTH          = 1    → 调试认证使能（但不锁）
Bit[5]    DBG_PORT_LOCK     = 0    → JTAG/SWD 未锁定 ✓
Bit[8]    SECURE_BOOT_EN    = 0    → 安全启动未开启 ✓
Bit[9]    ENCRYPTED_BOOT_EN = 0    → 加密启动未开启 ✓
```

> **关键结论**: 芯片虽然 OTP 已编程（有 SRK Hash、EXIP KEK），但 **两个
> 安全开关都没打开**（SECURE_BOOT_EN=0, DBG_PORT_LOCK=0）。这意味着：
> - J-Link 可以正常连接调试
> - 可以不签名烧录任意固件
> - USB ISP 烧录不受限制
>
> 形象比喻：锁和钥匙都装好了，但门没锁。

---

## FCFG：Flash 配置的命门

### 什么是 FCFG

FCFG（Flash Configuration）是一个 3-4 word 的配置表，告诉 ROM 如何初始化
外部 XPI NOR Flash：

```
Word 0: OPT_HDR  — 选项头 (0xfcf9XXXX)
Word 1: OPT_OPT0 — 引脚组选择 (0x00000007 = group 7)
Word 2: OPT_OPT1 — 时序参数   (0x00001000 或 0x00000000)
```

`OPT_HDR` 的末位数字决定使用哪个「选项集」：
- `0xfcf90001` → option 0
- `0xfcf90002` → option 1

不同选项集对应不同的 Flash 引脚映射和时序配置。
**同一款 SoC、同一块板子、同一颗 Flash 芯片，如果引脚布线不同，
就需要不同的 option。**

### 本板子的 FCFG 发现过程

1. `board.h`（来自 `hpm6e80ivm1`）定义了 `BOARD_APP_XPI_NOR_CFG_OPT_HDR = 0xfcf90001`
2. 但执行 `config-memory 0x10000 0x200` 时 **失败**：
   ```
   [Critical] ROM response failed! status: 1001006
              message: configuring memory failed
   ```
3. 换成 `0xfcf90002` 后 **成功** → 本板子需要 option 1

### 验证 FCFG 的方法

**不要直接烧录！先验证 FCFG 是否匹配：**

```bash
# 1. 写 FCFG 到 ILM
sudo ./hpm_manufacturing_cmd -u -f "HPM6E00,0" \
  -r "write-memory 0x0 0x200 [[0xfcf90002,0x00000007,0x00001000,0x0]]"

# 2. 尝试配置 XPI NOR（这一步会暴露 FCFG 是否正确）
sudo ./hpm_manufacturing_cmd -u -f "HPM6E00,0" \
  -r "write-memory 0x0 0x200 [[0xfcf90002,0x00000007,0x00001000,0x0]]" \
  -r "config-memory 0x10000 0x200"

# 如果 config-memory 返回 OK → FCFG 正确
# 如果返回 configuring memory failed → FCFG 不匹配，需要换 option
```

---

## Linker Script 与 Flash 内存布局

### 两个 Linker Script

这个项目有两个 linker script，分别用于 bootloader 和 application：

**Bootloader: `flash_xip.ld`**
```
__nor_cfg_option_load_addr__ = 0x80000000 + 0x400   = 0x80000400
__boot_header_load_addr__    = 0x80000000 + 0x1000  = 0x80001000
__app_load_addr__            = 0x80000000 + 0x3000  = 0x80003000
```

Flash 物理布局：
```
0x80000000 ┌──────────────────────────┐
           │ (空 0x400 bytes)          │  ← ROM 不读这里
0x80000400 ├──────────────────────────┤
           │ .nor_cfg_option           │  ← FCFG 表，ROM 从这里读
           │ (kOption[4])              │
0x80001000 ├──────────────────────────┤
           │ .boot_header              │  ← 启动头 + 签名信息
0x80003000 ├──────────────────────────┤
           │ .start / _start           │  ← 第一条指令
           │ .vectors (LMA, 复制到ILM) │
           │ .text                     │
           │ .data (LMA)               │
           │ .fast (LMA)               │
0x80015xxx ├──────────────────────────┤  ← bootloader 结束 (~86KB)
           │ (padding 到 128KB)        │
0x80020000 ├──────────────────────────┤
           │ UF2 Bootloader 保留区     │
           │ (128KB, 存放 bootloader)  │
           └──────────────────────────┘
```

**Application: `flash_uf2.ld`**
```
FLASH ORIGIN = 0x80000000 + UF2_BOOTLOADER_RESERVED_LENGTH
             = 0x80000000 + 0x20000
             = 0x80020000
```

Application Flash 布局（在 0x80020000 之后）：
```
0x80020000 ┌──────────────────────────┐
           │ UF2 Signature (4 bytes)   │  ← "HPM\n" (0x0A4D5048)
0x80020004 ├──────────────────────────┤
           │ _start / .start           │  ← 入口点
           │ .vectors (LMA, 复制到ILM) │
           │ .text                     │
           │ .data (LMA)               │
0x80043xxx ├──────────────────────────┤  ← app 结束 (~166KB)
           │ (free space)              │
0x80200000 ├──────────────────────────┤
           │ (2MB 保留)                │  ← 历史上由 EtherCAT 的 Flash 模拟
           │                           │     EEPROM 占用，现为空闲
           └──────────────────────────┘
```

### objcopy -O binary 的行为

`riscv32-unknown-elf-objcopy -O binary` 从**最低的 Flash VMA** 开始输出：

- Bootloader: 最低 Flash LOAD 段在 `0x80000400` → **binary 偏移 0 = Flash 地址 0x80000400**
- Application: 最低 Flash LOAD 段在 `0x80020000` → **binary 偏移 0 = Flash 地址 0x80020000**

---

## 烧录地址偏移陷阱（第一次失败根因）

### 错误操作

```bash
# ✗ 错误！bootloader binary 被写到 0x80000000
write-memory 0x10000 0x80000000 bootloader.bin
```

### 为什么失败

Bootloader binary 从偏移 0 开始就是 FCFG 表（`.nor_cfg_option` 的内容）。
这个 FCFG 表在 flash 里的**正确位置是 `0x80000400`**。

写到 `0x80000000` 导致：
```
实际:  0x80000000: [FCFG 表]  ← ROM 不读这里
       0x80000400: [boot_header 的后半段]  ← ROM 来这里找 FCFG，读到垃圾
       0x80000C90: [.start 代码]  ← ROM 在错误位置找启动头
```

ROM 启动流程：
1. 用 OTP 或默认引脚配置尝试读取 Flash
2. 去 `0x80000400` 找 FCFG 表 → **读到的不是 FCFG（偏移错了）**
3. 无法正确配置 Flash → 回退到 **ISP 模式**
4. USB 设备显示 `34b7:0006`（Boot ROM Open）

### 正确操作

```bash
# ✓ 正确！bootloader binary 写到 0x80000400
write-memory 0x10000 0x80000400 bootloader.bin

# ✓ Application 写到 0x80020000
write-memory 0x10000 0x80020000 demo.bin
```

### 如何确认自己有没有踩这个坑

芯片重新上电后，用 `lsusb` 检查：
- `34b7:0006` → Boot ROM 模式，**启动失败**，回退 ISP
- `a511:6e84`（或你在 CMakeLists.txt 设的 PID）→ **启动成功**，bootloader 在运行

---

## 正确烧录步骤

### 环境

```bash
# 以下三行均为 [前机路径]，换机器后改成自己的安装位置
cd /home/zyi/3rd_party/hpm/HPMicro_Manufacturing_Tool_v0.6.0
export GNURISCV_TOOLCHAIN_PATH="/home/zyi/3rd_party/hpm/rv32imac_zicsr_zifencei_multilib_b_ext-linux"
export PATH="${GNURISCV_TOOLCHAIN_PATH}/bin:$PATH"
```

### 编译

```bash
cd <仓库根>

# 配置（只需一次）
cmake --preset release -S firmware/hpm_board -B firmware/hpm_board/build_hpm6e8y \
  -DBOARD=hpm6e8y

# 编译
cmake --build firmware/hpm_board/build_hpm6e8y --target hpm_board_app hpm_board_bootloader
```

产物：
```
build_hpm6e8y/bootloader/output/hpm_board_bootloader_hpm6e8y.bin
build_hpm6e8y/app/output/hpm_board_app_hpm6e8y.dfu
```

### 验证 FCFG（烧录前必做）

```bash
# 读 bootloader 的前 4 字节确认 FCFG
python3 -c "
with open('build_hpm6e8y/bootloader/output/hpm_board_bootloader_hpm6e8y.bin', 'rb') as f:
    opt = int.from_bytes(f.read(4), 'little')
print(f'FCFG OPT_HDR: 0x{opt:08X}')
assert opt == 0xfcf90002, 'FCFG 错误！'
print('✓ FCFG 正确')
"
```

### 烧录

```bash
# sudo 密码是一个空格
echo ' ' | sudo -S ./hpm_manufacturing_cmd -u -f "HPM6E00,0" \
  -r "write-memory 0x0 0x200 [[0xfcf90002,0x00000007,0x00001000,0x0]]" \
  -r "config-memory 0x10000 0x200" \
  -r "write-memory 0x10000 0x80000400 /path/to/bootloader.bin" \
  -r "write-memory 0x10000 0x80020000 /path/to/demo.bin"
```

命令解释：
| 命令 | 作用 |
|------|------|
| `-u -f "HPM6E00,0"` | USB 连接 + 加载未签名 blfw 到 RAM |
| `write-memory 0x0 0x200 [[...]]` | 写 FCFG 到 ILM `0x200` |
| `config-memory 0x10000 0x200` | 用 ILM `0x200` 的 FCFG 初始化 XPI NOR |
| `write-memory 0x10000 0x80000400` | **写 bootloader 到正确偏移** |
| `write-memory 0x10000 0x80020000` | 写 application |

### 验证烧录

```bash
echo ' ' | sudo -S ./hpm_manufacturing_cmd -u -f "HPM6E00,0" \
  -r "write-memory 0x0 0x200 [[0xfcf90002,0x00000007,0x00001000,0x0]]" \
  -r "config-memory 0x10000 0x200" \
  -r "read-memory 0x10000 0x80000400 256 /tmp/verify.bin"

# 对比
diff <(xxd /tmp/verify.bin) \
     <(xxd -l 256 /path/to/bootloader.bin) && echo "✓ 验证通过"
```

### 测试启动

重新上电后检查 USB 设备：
```bash
lsusb | grep -Ei '34b7:0006|a511:6e84'
# 期望: a511:6e84 (bootloader DFU 模式)
# 失败: 34b7:0006 (Boot ROM，说明启动失败回退 ISP)
```

---

## HPM6E80 vs HPM6E00

### 本质差异

| | HPM6E80 | HPM6E00 |
|---|---|---|
| **RISC-V 核心** | 相同 | 相同 |
| **外设地址** | 相同 | 相同 |
| **引脚电气** | 相同（同封装） | 相同 |
| **内置 Flash** | 4MB (XPI0 QSPI) | 4MB (XPI0 QSPI) |
| **SDK 路径** | `soc/HPM6E00/HPM6E80/` | `soc/HPM6E00/HPM6E80/` |

> SDK 对 HPM6E00 和 HPM6E80 使用**同一套头文件和 linker script**，
> 目录名是 `HPM6E00/HPM6E80`。两者核心+外设完全一致。

### ⚠️ HPM6E00 的 Flash 对 J-Link 来说是 QSPI

虽然 Flash 是内置的（封装在芯片内部），但它走的是 **XPI0 → QSPI 协议**，
不是传统 MCU 那种挂在 AHB 总线上的 embedded Flash。对 J-Link 来说，
它看起来就像一片外部 QSPI NOR Flash，需要专门的编程算法。

### 为什么 HPM6E80 固件能跑在 HPM6E00 上

1. 两种芯片都通过 XPI0 接口访问内置 Flash，CPU 地址都是 `0x80000000`
2. 外设寄存器地址相同
3. 引脚 mux 相同
4. 中断向量表布局相同

**编译时 target 为 HPM6E80、实际芯片为 HPM6E00 不影响运行**，
因为两者从 XPI0 的角度完全一样。

---

## 安全启动与密钥

### 当前状态

```
SECURE_BOOT_EN    = 0  → 不验证固件签名，任意固件可启动
DBG_PORT_LOCK     = 0  → J-Link 可连接
DBG_AUTH          = 1  → 调试认证使能（未锁，实际不影响）
LifeCycle         = 0x04 → 芯片已配置

SRK Hash  已写入 OTP (Words 32-54, 92 bytes)
EXIP KEK  已写入 OTP (Words 88-91, 16 bytes)
```

### 潜在风险

如果将来有人把 `SECURE_BOOT_EN` 烧成 1：
- ROM 会验证固件签名
- 未签名固件无法启动
- 必须用对应私钥签名才能烧录

如果同时把 `DBG_PORT_LOCK` 也烧成 1：
- J-Link 完全断开
- 只能用 **USB-HID ISP**（Boot ROM 模式）烧录
- 但仍然可以烧录未签名固件（只要 SECURE_BOOT_EN 没开）

### 救砖底线

无论 OTP 怎么配置，只要芯片物理上没坏，**把 BOOT_MODE 引脚
（PA02/PA03）强制设为 ISP 模式（PA02=1, PA03=0），上电后芯片
必定进入 Boot ROM ISP 模式**。这是硬件级后门，无法被任何 OTP
设置关闭。

---

## 板级对外接口现状 [2026-09-08 核实]

**这块板的全部引脚已逆向完毕并在此收口。** 三份逆向过程文档
（`CAN_PIN_` / `GPIO_LED_` / `ETHERNET_PIN_REVERSE_ENGINEERING.md`）已于 2026-09-08 归档到
`~/Desktop/ethercat-archive-2026-09-08/`；**结论全部搬到本节，过程不再随仓库分发**。
注意"有引脚"不等于"有接口"，也不等于固件在用。

### 概览

| 接口 | 固件是否使用 | 说明 |
|---|---|---|
| CAN0..CAN3 | **在用** | 见下方引脚表与 `app/board_app.hpp` 的 `kCanPorts[]` |
| USB0 | **在用** | 数据面与 DFU |
| LED × 15 | **在用** | 见下方引脚表；**极性不统一** |
| 串口（UART0 控制台） | **不要用** | 只有通孔，见本节末 |
| 千兆以太网（Realtek） | **零引用** | **引脚已全部查实**；未定的只是 MDIO 设备地址，见下 |
| EtherCAT 进/出口 | 已移除 | 片内两颗 JL1111 级 100M PHY（ID `0x937c4024`），管理走 `PA30`/`PA31` 复用为 ENET0 SMI |

### CAN 引脚（四路，收发均实测确认）

| 逻辑 CAN | 板上丝印 | 外设 | TX | RX |
|---|---|---|---|---|
| `can0` | CAN0 | `HPM_MCAN0` | `PC00` | `PC01` |
| `can1` | CAN1 | `HPM_MCAN1` | `PB05` | `PB04` |
| `can2` | CAN2 | `HPM_MCAN2` | `PD08` | `PD09` |
| `can3` | CAN3 | `HPM_MCAN3` | `PD15` | `PD14` |

### LED 引脚（15 个已逐个闪灯确认，但**不是同一类东西**）

当年的 GPIO 扫描能点亮全部 15 个，只是因为 GPIO 是这些焊盘的**其中一个**复用功能。
按"正常运行时谁在驱动它"分，实际是四类：

| 分类 | 引脚 | 极性 | 谁驱动 | 当前固件 |
|---|---|---|---|---|
| **主 RGB** 红/绿/蓝 | PE05 / PE04 / PE03 | **低有效**（上电即亮） | GPIO，软件 | **在用**，`board_app.hpp` 的 `kLedRedPin` 等三个 |
| CAN0..CAN3 指示灯（绿/蓝） | PC26/PC27、PE00/PE02、PA09/PB00、PB02/PB03 | 高有效 | GPIO，软件 | 开机 park 成灭，**之后固件再没驱动过** |
| EtherCAT RUN / ERROR | PC20 / PC21 | 高有效 | **ESC 的 AL 状态机**（复用为 `ESC0_CTR_2` / `ESC0_CTR_3`） | ESC 复用随 EtherCAT 移除已不可达，park 成灭后**无人驱动** |
| PHY LED / 地址 strap | PA28 / PA25 | — | **片内 PHY 自己** | **刻意不驱动**：数据手册标明是 PHY 的 LED/strap 脚，`board_park_leds_off()` 的列表故意不含它们 |

**要点：以太网/EtherCAT 那几个灯不归软件管。** PA25/PA28 由片内 PHY 直接驱动（同时是地址
strap，软件驱动它会干扰 PHY 地址）；PC20/PC21 过去由 ESC 硬件从 AL 状态机驱动，不是 GPIO
输出。真正由固件点的只有主 RGB 三个。

`board_park_leds_off()` 在 `board_init()` 最开头把上表前三类共 13 个脚强制拉成 GPIO 推挽
输出并驱到灭（主 RGB 写 1，其余写 0），避免上电时乱亮；它跑在 `board_init_clock()` 之前，
所以自己先 `clock_add_to_group(clock_gpio, 0)`，否则写寄存器会静默失效。

### 千兆以太网（Realtek，固件未使用）

**引脚已全部查实**，没有遗留项：

| 项 | 值 | 来源 |
|---|---|---|
| 复位脚 | `PE01`，低电平有效 PHYRSTB | `[实测]` |
| 管理脚 | `PF00` = MDC，`PF01` = MDIO（走 ENET0 SMI） | `[实测]` |
| RGMII 数据总线 | `PF02..PF15` | `[实测]`，已用**实发数据包**证明 |
| PHY ID | `0x001cc916` | `[实测]` |

唯一没有最终定案的**不是引脚，是 MDIO 上的设备地址**：这颗 PHY 在地址 `0` 和 `1` 上
都应答。几乎可以肯定**真实地址是 `1`，`0` 是广播别名** `[推断]`——同一份逆向记录在另一条
MDIO 总线（片内 JL1111，走 `PA30`/`PA31`）上已经得出过同样形态的结论：地址 `0` 也返回
JL1111 的 ID `0x937c4024`，但不随任何 RJ45 建立链路，被判定为广播/别名响应。

要坐实它只需在地址 `1` 写一个广播不可见的寄存器再读回；但**固件从未初始化过这个口**，
当时用的探测镜像也已随 EtherCAT 归档，所以没有做的动力。真要用这个千兆口时再验。

### 三条别再踩的结论

逆向过程里被证伪、且容易重犯的判断：

1. **`PA25` / `PA28` 不是软件的 LED，是片内 PHY 自己驱动的 LED/地址-strap 脚**——早期
   按 GPIO LED 驱动过，会干扰 PHY 地址 strap。同理 `PC20` / `PC21` 是 ESC 硬件驱动的
   RUN/ERROR 灯，不是 GPIO 灯。
2. **一个引脚不可能既是 MII 又是 LED**：`PA09` 看着像 `REFCK`，实际是 CAN2-绿；
   `PE02` 看着像 `CTR_6`，实际是 CAN1-蓝。按数据手册的复用名去猜 LED 会错。
3. **极性不统一**：主 RGB 低有效（共阳），其余高有效。盲翻电平会得到反的结果。

另外 `PY06`/`PY07` 是 UART1、`PWDG_RSTN` 是看门狗复位，**盲翻这些引脚的电平不安全**。

### 串口：只有调试通孔，不是接口

**板上的串口只留了通孔焊盘供调试，没有插座，不是对外接口——请尽量不要用它。**
`board.c` 把 UART0（PA00/PA01）配成控制台并接到板载 FT2232，但那个调试头在实际使用的
板子上**并未装配**。因此：

- **不要把控制台当数据通道或日志通道。** 需要看板上发生了什么，走 USB 带内诊断
  （见 [../../AGENTS.md](../../AGENTS.md)「板上没有调试器时怎么看现场」），不要指望串口。
- `board_console_try_send_byte()` 之所以是非阻塞的，正是因为 SDK 的
  `console_send_byte()` 会在 THR-empty 上忙等约 87us/字节——在没接收端的板子上那是纯浪费。
- UART1（PY06/PY07）同属这一类调试引脚，**不要盲翻它们的电平**（同段的
  `PWDG_RSTN` 是看门狗复位）。

## 调试与救砖

### bootloader 常驻：没有按键，靠短接或 manual 模式

**这块板上没有任何实体按键** `[实测 2026-09-08]`。`board.c` 的
`board_check_bootloader_force_stay_requested()` 采的是 **PB24**（内部上拉，拉低算触发），
但那只是一个引出的焊盘——函数注释里的 "user key KEYA" 是从 HPM6E00EVK 参考设计继承的叫法，
**不代表板上焊了键**。5321 的情况一样（PA07，还与 JTAG_TMS 复用）。

于是强制 bootloader 常驻只有两条路：

| 手段 | 怎么做 | 代价 |
|---|---|---|
| 硬件短接 | 复位期间把 **PB24 拉到 GND**，采样窗口只有复位后 4×250us ≈ **1 ms** | 要碰板子；窗口很窄 |
| **`BOARD_BOOTLOADER_MODE=manual`** | 编一个 manual 版 bootloader 烧进去 | 之后每次上电**默认停在 DFU** |

**两种模式的语义是互为反相的**（`bootloader/src/main.cpp`）：

| 模式 | 上电行为 | 跳进 app 的条件 |
|---|---|---|
| `auto`（默认） | 默认进 app | 板型可识别 + 未触发 force-stay + **没有** DFU 请求 + app 镜像校验通过 |
| `manual` | **默认停在 DFU** | 同上，但要求 `BootMailbox` 里**有**一条 `boot_app_once` 请求 |

```bash
cmake --preset release -S firmware/hpm_board -B <build> -DBOARD=hpm6e8y \
      -DBOARD_BOOTLOADER_MODE=manual
```

> **什么时候必须用 manual**：要试的 app 镜像**有可能在 USB 枚举之前就跑飞**（典型是在
> boot 路径上做外设 bring-up——时钟门控下的寄存器访问会直接卡住）。`auto` 的 bootloader
> 只要看到一份**校验合法**的 app 就跳进去，坏 app 一旦不枚举 USB，**就再没有 DFU 窗口**，
> 板子等于砖，只能靠短接 PB24 或 J-Link 救。**先烧 manual bootloader，再试危险镜像**——
> 这个教训的代价是两块板。

另外两种"跳不进去"的情况不需要按键，是设计如此：OTP word 25 无法识别时 bootloader
**无条件拒绝跳转**（并以哨兵 PID 枚举、拒绝 DFU 下载），以及 app 镜像校验不过时自然留在 DFU。

### BOOT_MODE 引脚

| PA02 | PA03 | 启动模式 |
|------|------|---------|
| 0 | 0 | 按 OTP Word 0 配置 |
| 0 | 1 | Serial Boot (UART) |
| **1** | **0** | **ISP Boot (USB-HID)** ← 救命模式 |
| 1 | 1 | Primary Boot (XPI Flash) |

### 常见问题

| 症状 | 原因 | 解决 |
|------|------|------|
| USB 显示 `34b7:0006` | Flash 启动失败，回退 ISP | 检查 FCFG + 烧录地址 |
| `config-memory` 失败 | FCFG OPT_HDR 不匹配硬件 | 换 option（0 和 1 之间切换） |
| 烧录后芯片没反应 | bootloader 偏移错误 | 确认写到 `0x80000400` 不是 `0x80000000` |
| J-Link 连不上 | DBG_PORT_LOCK=1 | 用 BOOT_MODE 引脚进 ISP 救砖 |

### 读取 Flash 当前内容（排错用）

```bash
echo ' ' | sudo -S ./hpm_manufacturing_cmd -u -f "HPM6E00,0" \
  -r "write-memory 0x0 0x200 [[0xfcf90002,0x00000007,0x00001000,0x0]]" \
  -r "config-memory 0x10000 0x200" \
  -r "read-memory 0x10000 0x80000000 4096 /tmp/flash_dump.bin"
```

### 读取 OTP（排错用）

```bash
echo ' ' | sudo -S ./hpm_manufacturing_cmd -u \
  -r "query-rte 0" \
  -r "query-rte 4 0x20000" \
  -r "read-memory 0x20000 0x0 512 /tmp/otp_dump.bin"
```

---

## PMP 与 J-Link：为什么能烧录 ELF 但不能擦除 Flash

### 问题现象

```text
J-Link> erase
Erasing device...
J-Link: Flash download: Only internal flash banks will be erased.
To enable erasing of other flash banks like QSPI or CFI,
it needs to be enabled via "exec EnableEraseAllFlashBanks"

J-Link> exec EnableEraseAllFlashBanks   # 启用 QSPI 擦除
J-Link> erase
****** Error: Timeout while waiting for core to halt after reset
****** Error: Failed to preserve target RAM @ 0x00000000-0x0001FFFF.
Failed to prepare for programming.
ERROR: Erase returned with error code -1.
```

但烧录 ELF 却正常：

```text
J-Link> loadfile demo.elf
Downloading... 166072 bytes @ 0x80020000  ← OK
```

### 根因：两条不同的访问路径

```
┌─────────────────────────────────────────────────────────────────┐
│                        J-Link 调试探针                           │
├─────────────────────┬───────────────────────────────────────────┤
│  SBA 路径 (System   │  CPU 执行路径 (Flash Loader)               │
│  Bus Access)        │                                           │
│                     │                                           │
│  J-Link → SBA →     │  J-Link → ILM (0x00000000)               │
│  直接读写内存地址     │  下载 Flash Loader 代码                   │
│  ↓                  │  → 唤醒 CPU                               │
│  XPI0 控制器        │  → CPU 执行:                               │
│  ↓                  │    - 往 XPI0 命令寄存发 QSPI Erase 指令     │
│  Flash 读写          │    - 轮询 busy flag                       │
│                     │    - 等待擦除完成                          │
│                     │                                           │
│  不走 CPU            │  必须走 CPU                                │
│  不受 PMP 限制       │  受 PMP 限制!                              │
│  只能做简单读写       │  能做复杂时序 (Erase, Program)             │
└─────────────────────┴───────────────────────────────────────────┘
```

**烧录 ELF 走 SBA 路径**：
- SBA 直接往 `0x8002xxxx`（Flash 地址空间）写数据
- XPI0 控制器自动把 AHB 写事务转成 QSPI Page Program
- 全程不碰 ILM (`0x00000000`)，PMP 管不到

**擦除走 CPU 路径**：
- J-Link 需要把一段 Flash Loader 代码下载到 ILM
- 芯片的 `board_init_pmp()` 已经把 ILM 区域锁了
- J-Link 写 ILM 失败 → `Failed to preserve target RAM @ 0x00000000-0x0001FFFF`

### PMP 是你们自己代码配的

```c
// board.c:115 — 不是 HPM SDK 的问题，是你自己的 PMP 配置
void board_init_pmp(void) {
    pmp_entry_t pmp_entry[16] = {0};

    // 第一条: 全部放行 (NAPOT, 覆盖整个地址空间)
    pmp_entry[0].pmp_addr = 0xFFFFFFFF;
    pmp_entry[0].pmp_cfg.val =
        PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);

    // 后续: 锁定 non-cacheable 区域、SHARE_RAM 等
    ...
    pmp_config(&pmp_entry[0], index);
}
```

PMP 规则本身没问题（第一个 entry 全部放行），但 PMP 一旦使能，
J-Link 的 SBA 视角和 CPU 视角产生不一致，Flash Loader 下载就失败了。

### 解决方法

#### 方法 A：Reset + Halt（最简单）

```
J-Link> r                    # reset & halt（在第一条指令前停住）
J-Link> exec EnableEraseAllFlashBanks
J-Link> erase
```

`r` 让 CPU 停在 `_start` 的第一条指令，此时 `board_init()` 还没跑，
PMP 还没配置，ILM 完全开放，Flash Loader 可以正常下载。

#### 方法 B：用垃圾 ELF 覆盖 PMP 代码（不需要改接线）

核心原理：**利用 J-Link 能 SBA 写 Flash 的能力，把 bootloader 里
调用 PMP 的代码段用 0x00 或 NOP 覆盖掉。**

```
1. 编一个 "垃圾 ELF"：
   - 链接地址覆盖 bootloader 的 PMP 相关区域 (0x80003000-0x80004xxx)
   - 内容全 0x00 或非法指令
   
2. J-Link 烧这个 ELF → SBA 直接覆盖 Flash 对应扇区
3. 复位 → bootloader 被破坏，PMP 配不了 → ILM 开放
4. 此时 J-Link 可以 erase chip
5. 或者芯片掉回 Boot ROM (PID 34b7:0006) → USB 制造工具可用
6. 用制造工具重新烧录正确的 bootloader
```

实际操作更简单：直接新建一个只有 `.start` 段、链接到 `0x80000400`
的 ELF，内容全零，覆盖掉 FCFG 表 + boot header。

```c
// dummy_main.c — 最小 ELF，只用来破坏 PMP
__attribute__((section(".start"), used))
void _start(void) { while(1); }
```

链接脚本让它覆盖 `0x80000400` 开始的关键区域。烧进去后芯片复位，
bootloader 找不到有效代码 → BOOT ROM ISP 接管 → USB 制造工具可用。

#### 方法 C：BOOT_MODE 引脚进 Boot ROM（硬件方式）

PA02=1, PA03=0 → 上电 → 芯片不进你的固件 → PMP 不存在 → J-Link 随便擦。

---

## 完整避坑清单

### 坑 1：FCFG OPT_HDR 不匹配
- **现象**: `config-memory` 返回 `configuring memory failed`
- **根因**: `board.h` 里 `0xfcf90001` 不匹配板子的 Flash 引脚布线
- **解决**: 两个 option 都试一次（`0xfcf90001` 和 `0xfcf90002`），
  找到能通过 `config-memory` 的那个

### 坑 2：Bootloader 烧录偏移错误
- **现象**: 烧录后芯片仍显示 `34b7:0006`（Boot ROM），无法从 Flash 启动
- **根因**: `flash_xip.ld` 把 `.nor_cfg_option` 放在 `0x80000400`，
  binary 偏移 0 对应 Flash `0x80000400`，但被写到了 `0x80000000`
- **解决**: 永远用 `0x80000400` 作为 bootloader 的写入地址

### 坑 3：混淆 load-image 和 write-memory
- `load-image`：加载固件到 **RAM** 执行（ISP 模式，掉电丢失）
- `write-memory 0x10000`：写入 **XPI NOR Flash**（持久化，上电自启）
- 两个命令用途不同，不可混用

### 坑 4：忘记验证 FCFG 就烧录
- 应该先用 `config-memory` 验证 FCFG 能否初始化 Flash
- 验证通过后再 `write-memory` 写入
- 否则会在 Flash 上留下一份配错 FCFG 的 bootloader

### 坑 5：GTK 库缺失导致 GUI 启动失败
- `hpm_manufacturing_gui` 依赖 GTK 库，在无桌面的 Linux 上不可用
- 用 `hpm_manufacturing_cmd` 命令行工具替代

### 坑 6：sudo 权限
- Linux 下访问 USB-HID 设备需要 root 权限
- 自动化脚本：`echo ' ' | sudo -S ./hpm_manufacturing_cmd ...`
- 注意：`-S` 从 stdin 读密码，本机器的 sudo 密码是一个空格

### 坑 7：blfw 加载后设备重连
- `-f "HPM6E00,0"` 加载 blfw 后设备会断开再重连
- 连续多次调用工具时不需要每次 `-f`（设备已在 blfw 模式下）
- 但如果设备断电重来，就需要重新 `-f`

### 坑 8：OTP 烧录不可逆
- OTP 只能从 1→0，不能从 0→1
- 烧录前务必确认值正确
- 建议先用 `read-memory 0x20000` 读取当前 OTP，确认空位再写

### 坑 9：不要把 app binary 和 bootloader binary 搞混
- Bootloader（`flash_xip.ld`）：写到 `0x80000400`
- Application（`flash_uf2.ld`）：写到 `0x80020000`
- 两个 linker script 的 FLASH 基址不同！

### 坑 10：调试认证 (DBG_AUTH) 可能影响 J-Link
- 本芯片 `DBG_AUTH=1` 但 `DBG_PORT_LOCK=0`，J-Link 仍可用
- 如果某天 `DBG_PORT_LOCK` 被烧成 1，J-Link 完全不可用
- 这时候只能用 BOOT_MODE 引脚进入 ISP 来救砖

### 坑 11：PMP 使能后 J-Link 能烧不能擦
- **现象**: `loadfile demo.elf` 成功，`erase` 失败
- **根因**: 烧录走 SBA 直写 Flash 地址（绕过 PMP）；擦除需要
  下载 Flash Loader 到 ILM → 被 PMP 拦截
- **解决**: `J-Link> r`（reset+halt）在 PMP 配置前停住 CPU，
  然后执行 erase；或者用 BOOT_MODE 引脚进 ISP

---

## 参考文件路径

> 下列绝对路径全部是 `[前机路径]`，仅作参照；仓库内的相对位置才是稳定的。

```
固件:
  /home/zyi/Desktop/libhcs/firmware/hpm_board/

Board 文件夹 (hpm6e8y):
  /home/zyi/Desktop/libhcs/firmware/hpm_board/boards/hpm6e8y/
  ├── board.h      ← FCFG: 0xfcf90002
  ├── board.c      ← kOption = {0xfcf90002, 0x00000007, 0x00001000, 0x0}
  ├── CMakeLists.txt ← USB PID: 0x6E84
  └── hpm6e8y.yaml

Linker Scripts:
  boards/bsp/hpm_sdk/soc/HPM6E00/HPM6E80/toolchains/gcc/
  ├── flash_xip.ld   ← Bootloader (FCFG @ 0x400)
  └── flash_uf2.ld   ← Application (FLASH @ 0x20000)

烧录工具:
  /home/zyi/3rd_party/hpm/HPMicro_Manufacturing_Tool_v0.6.0/
  ├── hpm_manufacturing_cmd   ← 命令行工具
  ├── hpm_manufacturing_gui   ← GUI（需要 GTK）
  └── bl_fw/HPM6E00/         ← Boot Loader Firmware（ISP 辅助固件）
```
