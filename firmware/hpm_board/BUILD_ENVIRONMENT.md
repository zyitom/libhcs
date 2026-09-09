# hpm_board 固件编译环境

> **文档类型**：现行规范（环境搭建步骤）
> **适用范围**：`firmware/hpm_board/` 的 USB 数据固件
> **状态**：现行有效
> **相关文档**：[AGENTS.md](AGENTS.md)（日常构建命令） · [仓库根 AGENTS.md](../../AGENTS.md#开发机环境路径约定重要先读这条再看任何路径)（路径约定）

## 摘要

本文件是 hpm_board 的**从零搭环境**指南：装什么、装到哪、设哪些环境变量、怎么验证装
对了。日常只是编译的话看 [AGENTS.md](AGENTS.md) 就够，本文件是换机器或首次配置时用的。

**最关键的一条**：这块板用 **RISC-V（HPMicro GNU 工具链）**，不是 ARM；工具链是仓库外
的预编译二进制，需要单独下载安装。文中出现的具体安装路径（如 `~/3rd_party/hpm`）都是
`[前机路径]`——记录的是曾经真实可用的位置，当前机器上不存在属正常，含义见
[仓库根 AGENTS.md 的开发机环境路径约定](../../AGENTS.md#开发机环境路径约定重要先读这条再看任何路径)。

本文档说明编译 `firmware/hpm_board` 下 USB 数据固件(`firmware/hpm_board/` 超级构建,
USB vendor 类 bulk 传输)所需的环境。

---

## 1. 依赖清单

| 组件 | 版本 / 型号 | 是否随仓库 | TL101 现状 |
|---|---|---|---|
| CMake | >= 3.13(实测 3.28.3 可用) | 否(系统装) | 已装 |
| Ninja | 任意近版(实测 1.11.1) | 否 | 已装 |
| Python | 3.x(实测 3.12.3) | 否 | 已装 |
| Python 包 | `PyYAML`、`jinja2` | 否(pip 装) | 需确认 |
| HPM SDK | v1.12.0(fork `zyitom/hpm_sdk`,`v1.12.0-3-ge4347411`) | **是**(`bsp/hpm_sdk`) | 随仓库 |
| TinyUSB | v0.21.0 | **是**(`../c_board/bsp/tinyusb` submodule) | 随仓库，需初始化 |
| RISC-V 工具链 | `rv32imac_zicsr_zifencei_multilib_b_ext`(HPMicro GNU) | **否** | 见下方状态更新 |
| 烧录工具 | HPMicro Manufacturing Tool v0.6.0(DFU) | 否 | 见 §5 |

> **状态更新(2026-07-27,实测):当前开发机已装 RISC-V 工具链**,下面这段"缺工具链"的
> 结论只适用于 TL101 那台机器,不要照搬到当前机器。本机实测:
> `/home/zyi/3rd_party/hpm/rv32imac_zicsr_zifencei_multilib_b_ext-linux/bin/riscv32-unknown-elf-gcc`
> 存在。`~/3rd_party/hpm` 下还有 `hpm_sdk`、`openocd-linux-x86_64`、
> `HPMicro_Manufacturing_Tool_v0.6.0`。**断言"本机没工具链"之前先 `ls` 确认。**

**(历史结论,针对 TL101)TL101 上目前缺 RISC-V 工具链**(`~/3rd_party/hpm` 不存在,
PATH 里无 `riscv32-unknown-elf-gcc`),所以固件的实际编译/烧录需在**已装工具链的开发 PC**
上完成。Host 端 SDK / examples(纯 x86)可以在 TL101 上编。

---

## 2. 一次性安装

### 2.1 RISC-V 工具链(不是 ARM)

> **HPM6E8Y / HPM5321 是 RISC-V 芯片(Andes 核),不是 ARM。** 因此用
> `riscv32-unknown-elf-gcc`,**不要用 `arm-none-eabi-gcc`**。arm-gcc 只在编
> STM32 的 `firmware/c_board` 时才需要。

下载带 B 扩展 multilib 的 HPMicro GNU RISC-V 工具链(版本 2023.10.18):

```
https://github.com/hpmicro/riscv-gnu-toolchain/releases/download/2023.10.18/rv32imac_zicsr_zifencei_multilib_b_ext-linux.tar.gz
```

**这是预编译二进制工具,不是源码——不要放进仓库或 git submodule。** 解压到
仓库外的固定路径(沿用现有约定 `~/3rd_party/hpm/`):

```bash
mkdir -p ~/3rd_party/hpm && tar -xzf rv32imac_zicsr_zifencei_multilib_b_ext-linux.tar.gz -C ~/3rd_party/hpm
```

解压后应得到 `~/3rd_party/hpm/rv32imac_zicsr_zifencei_multilib_b_ext-linux/bin/riscv32-unknown-elf-gcc` 等。

> 对比:`bsp/hpm_sdk` 能随仓库是因为它是**源码**(SDK);工具链是**二进制工具**,
> 留在仓库外,仅用 `GNURISCV_TOOLCHAIN_PATH` 引用。

### 2.2 Python 包

```bash
pip3 install PyYAML jinja2
```

### 2.3 HPM SDK

无需单独安装——已随仓库 vendored 在 `firmware/hpm_board/bsp/hpm_sdk`。当前固件只使用
它的 SoC、USB PHY 和寄存器驱动；TinyUSB device stack 来自共享的
`firmware/c_board/bsp/tinyusb` v0.21.0。clone 后需初始化两项 submodule：

```bash
git submodule update --init firmware/hpm_board/bsp/hpm_sdk firmware/c_board/bsp/tinyusb
```

submodule 指向 **fork `zyitom/hpm_sdk`**(不是上游),当前
`v1.12.0-3-ge4347411`,分支 `migrate-v1.12.0`。fork 里的本地补丁:tinyusb 构建
vendor class 并去掉示例 BSP、vendor ZLP 显式写 API、`rh_init` 按速度强制 full speed,
外加 HPM5300 的 `__cpluscplus` typo 修复。前三项 TinyUSB 补丁仅保留在 SDK fork 的历史
代码中，当前 hpm_board 镜像不再编译它们。

#### v1.11.0 -> v1.12.0 迁移评估(2026-07-27,已完成迁移)

只有一项改动对本仓库有实际价值,其余与我们用到的代码无关。记录于此以免重复调研:

- **历史价值:USB setup 包的 SUTW(setup tripwire)修复**,落在
  `middleware/tinyusb/src/portable/hpm/dcd_hpm.c`。v1.11 直接读
  `qhd0->setup_request`,无 tripwire 保护;主机在读取途中又发来 setup 包时会读到撕裂
  或过期的 8 字节,表现为**枚举偶发失败、控制传输卡死**。v1.12 改为
  `set_sutw -> memcpy -> 检查 sutw` 的重读循环(ChipIdea/EHCI 手册要求的协议),并给
  `bus_reset` 分支补了 `return`。该结论适用于迁移前的 SDK TinyUSB v0.20.0 路径；
  2026-08-06 起所有镜像改用共享 v0.21.0 的通用 ChipIdea DCD
  `portable/chipidea/ci_hs/dcd_ci_hs.c`，`dcd_hpm.c` 已不在编译路径。[实测构建]
- **无关:cherryusb 升到 v1.6.1**。本仓库用 **TinyUSB**,不编 cherryusb,该改动零影响。
- **暂不采用:MCAN 新增 queue 模式与 `*_unchecked` 高吞吐 API**
  (`mcan_read_rxfifo_unchecked` 等)。相比 checked 版省掉 NULL/边界/空 FIFO 检查,且只
  拷 DLC 对应字数;但 unchecked 版**不判空**,要配 `mcan_get_rxfifo_fill_level()` 改写
  排空循环。省下的是每帧几十个周期,而链路瓶颈在线速与 echo 往返上,改了测不出来还要
  重验 ISR 排空语义。`mcan_transmit_via_txqueue_nonblocking`(按 ID 优先级而非时序发送)
  仅在出现"某些 CAN ID 必须插队"的需求时才有意义。
- **无关:HPM5E00 SoC 文件重新生成**(本项目用 HPM6E8Y / HPM5321)、**boards 宏重命名**
  (`BOARD_APP_HDMA` -> `BOARD_APP_DMA0`;全仓库零引用,`boards/*/board.c` 是手写的)、
  `hpm_l1c_drv.c` 的 assert typo(仅影响开 assert 的构建)、
  `usb_phyctrl1_select_utmi_clk_src`(门在 `HPM_IP_FEATURE_USB_CLK_SELECT` 后)。
- **`hpm_interrupt.h` 重构**:纯 SEGGER SysView 埋点,未定义 `CONFIG_SEGGER_SYSVIEW`
  时展开为空,生成代码无变化;可作为 ISR 级延迟剖析的现成入口。
- **v1.12.1 是空版本**:改动全在 `samples/*/app.yaml`(加 DFU 构建排除标记),与我们
  使用的代码零关系。

---

## 3. 环境变量(每个新终端都要设)

```bash
cd <repo-root>
export GNURISCV_TOOLCHAIN_PATH="/home/<user>/3rd_party/hpm/rv32imac_zicsr_zifencei_multilib_b_ext-linux"
export PATH="${GNURISCV_TOOLCHAIN_PATH}/bin:$PATH"
export HPM_SDK_BASE="$(pwd)/firmware/hpm_board/bsp/hpm_sdk"
```

验证:

```bash
riscv32-unknown-elf-gcc --version   # 应打印工具链版本
echo "$HPM_SDK_BASE"                 # 应指向 bsp/hpm_sdk
```

---

## 4. 编译:USB 数据固件(超级构建,BOARD 可选 6e8y)

```bash
# 配置:BOARD 决定目标板与 USB PID(6e8y=0x6E84,见 boards/hpm6e8y/CMakeLists.txt)
cmake --preset debug -S firmware/hpm_board -DBOARD=hpm6e8y
cmake --build firmware/hpm_board/build
```

产物 ELF/BIN/DFU 名字里带 BOARD(如 `hpm_board_app_hpm6e8y.*`)。

> 提示:`BOARD` 是自由字符串,`BOARD_SEARCH_PATH` 默认指向 `boards/`,只要
> `boards/hpm6e8y/` 存在即可用 `-DBOARD=hpm6e8y`。超级构建 `CMakeLists.txt` 的
> `STRINGS` 下拉列表目前只列了 `hpm5321`,那只是 GUI 提示,
> 不影响命令行;把 `hpm6e8y` 加进该列表纯属可读性优化。

> **HPM5321 只有一个 BOARD 值要记:`hpm5321`。** 它出的单个镜像同时服务单 CAN 版和
> 双 CAN-FD 版两块板,上电时读 OTP 第 25 个字判断跑在哪块上,并据此报 `0x5321` 或
> `0x5322`——所以 `dfu-util -d` 的 PID 按板子填,和合并前一样。`.dfu` 容器后缀里的
> PID 是通配 `0xFFFF`,同一个文件对两个 PID 都能过校验。原理、实测数据、以及"读到
> 未知值就拒绝启动"的行为见 [boards/hpm5321/README.md](boards/hpm5321/README.md)。
> HPM5321 两块 PCB 共用 `-DBOARD=hpm5321` 这一份镜像；OTP 第 25 字决定 CAN/LED/PID。

---

## 5. 烧录(简述)

用 HPMicro Manufacturing Tool(DFU 方式),把板子的 USB0 接开发 PC,上电进
bootloader(`lsusb` 见 `a511:6e84` 表示 bootloader 在跑),再用 `.dfu` 包烧录。
FCFG / 详细步骤见 `boards/hpm6e8y/README.md`。

---

## 6. Host 端(x86,可在 TL101 上编)

固件之外的上位机 SDK 与 examples 不需要 RISC-V 工具链:

```bash
cmake --preset linux-debug -S host
cmake --build host/build
```

USB 回环对比测试用现成的 `host/build/examples/can_loopback_latency`(走
`common/multi_board` 的 USB API)。

---

## 附:最小检查清单

- [ ] `riscv32-unknown-elf-gcc --version` 能跑
- [ ] `$HPM_SDK_BASE` 指向 `bsp/hpm_sdk`
- [ ] `pip3 show PyYAML jinja2` 均已安装
- [ ] BOARD 选对(`hpm6e8y`),USB PID 对应 `0x6E84`
