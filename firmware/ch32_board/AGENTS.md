# ch32_board 固件指南

> **文档类型**：现行规范（板级）
> **适用范围**：`firmware/ch32_board/`，WCH CH32H417（Qingke V3F + V5F 双核 RISC-V）
> **状态**：现行有效
> **相关文档**：[仓库根 AGENTS.md](../../AGENTS.md) · [README.md](README.md)（设计与移植决策） · [PITFALLS.md](PITFALLS.md)（**上板前必读**） · [PROGRESS.md](PROGRESS.md)（工作日志）

> 本目录专属指南，叠加在仓库根 `AGENTS.md` 之上。项目动机、app 布局、bring-up 缺口见本目录 `README.md` 与 `PROGRESS.md`，此处只列 agent 关键点。
>
> **改代码或上板之前先读 `PITFALLS.md`** —— 首次 bring-up 踩过的坑（调试口被 USB2 抢占、
> ISR `mret`、读保护解除要两会话 + POR、`e339e339` 假数据等）都在那里，附判据和修法。

## 摘要

ch32_board 是四块板里**最容易把自己弄进死胡同**的一块，原因有三：

1. **双核异构**——V3F 先复位、拥有时钟树、兼任 DFU bootloader，V5F 才是转发快路径；
   "启动 App" 是唤醒 V5F 而不是跳转，V5F 也**不能自己复位**。
2. **调试口和 USB2 抢引脚**——PB8/PB9 既是 USB2 的 D+/D-，也是 SWCLK/SWDIO，一旦回退到
   USB 2.0 就丢调试口，只能断电恢复（所以仓库里把 USB2 回退关掉了）。
3. **烧录链路坑多**——读保护、`e339e339` 假数据、`unfreeze` 的位置、halt 顺带复位导致
   读到的 PC 不可信，全部记在 `PITFALLS.md`。

本板的正式构建必须使用 MounRiver 随附的 WCH GCC15，并通过独立的
`WCH_TOOLCHAIN_PATH` 选择；不得复用 `hpm_board` 的 HPM 工具链。

本文件给的是**现在该怎么做**（工具链、构建、烧录、调试、当前进度）；**为什么这么做**
在 `README.md`，**踩过什么坑**在 `PITFALLS.md`。

## 本文导航

| 章节 | 内容 |
|---|---|
| [芯片与工具链](#芯片与工具链) | WCH GCC15 隔离规则、版本和 Docker 状态 |
| [构建](#构建) | app、boot 和 merged 固件的构建命令 |
| [烧录](#烧录wch-link) | WCH OpenOCD 的烧录顺序与保护态陷阱 |
| [GDB 调试](#gdb-调试) | WCH GDB 选择和调试连接方式 |
| [目录结构](#目录结构) | 双核源码和厂商 BSP 的职责边界 |
| [现状](#现状改代码前须知) | 已验证能力、硬件约束和剩余缺口 |

## 芯片与工具链
- MCU：**WCH CH32H417**，RISC-V **双核**（V3F boot/offload + V5F 转发快路径）；
  卖点是片上 **USB 3.0 SuperSpeed（5 Gbps）** 设备控制器。
- ISA/工具链：RISC-V bare-metal，`cmake/toolchain-wch-riscv.cmake`（RV32IMAC/ilp32 软浮点，**不启用** F 也**不启用** WCH 私有 `xw` 扩展）。正式构建使用 MounRiver `MRS_Toolchain_*` 随附的
  **RISC-V Embedded GCC15**（不是 ARM，也不是 HPM GCC）。
- **工具链隔离是硬约束**：`ch32_board` 只读取独立的 `WCH_TOOLCHAIN_PATH`，不得把
  `GNURISCV_TOOLCHAIN_PATH`、`~/3rd_party/hpm` 或 `/opt/riscv32-none-elf` 当作本板的正式
  编译器。`toolchain-wch-riscv.cmake` 只接受 `riscv32-wch-elf-`（或显式指定的 WCH
  前缀），不再静默回退到 `riscv32-unknown-elf-`。
- MounRiver 的 `Toolchain/` 下并排放着三套 GCC，`WCH_TOOLCHAIN_PATH` 必须指向
  **具体某一套**（含 `bin/` 的那层），不是 `Toolchain/` 本身：

  | 目录 | 前缀 | 版本 | 可用性 |
  |---|---|---|---|
  | `RISC-V Embedded GCC` | `riscv-none-embed-` | 8.2.0 | **不可用**，编不了 C++23 |
  | `RISC-V Embedded GCC12` | `riscv-wch-elf-` | 12.2.0 | 仅保留 GDB 备用，不用于正式编译 |
  | `RISC-V Embedded GCC15` | `riscv32-wch-elf-` | 15.2.0 | **正式编译器** |

  ```bash
  # 当前机器的安装位置；换机器后改成自己的 MRS 解压目录即可
  export WCH_TOOLCHAIN_PATH="$HOME/3rd_party/MRS_Toolchain_Linux_X64_V240/Toolchain/RISC-V Embedded GCC15"
  export WCH_TOOLCHAIN_PREFIX=riscv32-wch-elf-
  ```
- **代码尺寸约束**：MRS GCC15 的 Debug app 在 2026-08-05 测过 125332 B / FLASH **95.62%**
  （128 KB 槽；当时 Debug 是 CMake 默认的 `-g`、没有 `-O`）；Release 49124 B / **37.48%**。
  早期 HPM GCC13 Debug 为 113364 B / 86.5%。现在 Debug 显式 `-Og -g3`，占用会低于那次
  95.62%，但每次构建仍必须看链接器的 FLASH 占用，避免静默越界。[Docker 实测 2026-08-05；
  `-Og` 后的新数字尚未重测]
- **Docker / CI 现状**：`libhcs-ci` 已把 MounRiver V240 的 GCC15 安装到独立目录
  `/opt/wch-gcc15`，并设置 `WCH_TOOLCHAIN_PATH` / `WCH_TOOLCHAIN_PREFIX`。镜像固定为
  `linux/amd64`；Docker 构建从官网 API 取得临时签名地址，并校验固定的资源 ID、文件大小
  和 SHA-256。Lint 和 Release 都会实际构建 `ch32_board`，由 HPM 编译器生成的产物仍不
  作为正式结果。CH32 当前纳入 clang-format，但 clang-tidy 因 WCH 厂商宏误报而在
  `.scripts/lint-targets.yml` 中显式禁用。[官网与 Docker 实测 2026-08-05]

## 构建
```bash
export WCH_TOOLCHAIN_PATH="$HOME/3rd_party/MRS_Toolchain_Linux_X64_V240/Toolchain/RISC-V Embedded GCC15"
export WCH_TOOLCHAIN_PREFIX=riscv32-wch-elf-
cmake --preset debug -S firmware/ch32_board
cmake --build firmware/ch32_board/build
# -> build/ch32_board_app.elf   (V5F @0x10000)
# -> build/ch32_board_boot.elf  (V3F @0x0)
# -> build/ch32_board_merged.hex  <- 烧这个
```
- 在 `libhcs-ci` / `libhcs-develop` 容器内不需要手动 `export`，镜像已经设置上述变量。
- preset：`debug` / `release`。target：`ch32_board_app` / `ch32_board_boot` /
  `ch32_board_merged`（默认全建）。**`boot` 既是 V3F 启动核镜像，也是 DFU
  bootloader**——本芯片 V3F 先复位、拥有时钟树、决定要不要唤醒 V5F，这本来就是
  bootloader 的活，所以两者合一，"启动 App" = `NVIC_WakeUp_V5F()` 而非跳转。
- App 还会产出 `ch32_board_app.dfu`（裸 `.bin` + DFU suffix，**不带** mc02/c_board
  那个 `ImageHash` 后缀：本 bootloader 自己哈希烧进去的内容）。烧法见 `README.md`
  的 "Bootloader / DFU" 一节，或仓库根 `./flash-ch32.sh`。

## 烧录（WCH-Link）

> **路径说明**：本节的 OpenOCD 取自 MounRiver 工具链包。前机上曾经单独放过一份
> `~/3rd_party/wch-openocd`（`README.md` 里保留的就是那个版本的写法），后来改为直接用
> MRS 包内自带的，所以以 **本节路径为准**。当前机器已安装本节的 MRS 路径；旧的
> `~/3rd_party/wch-openocd` 仍不存在。换机器后只需把 `OCD=` 指向新装的 MRS 包即可，
> 命令序列不变。OpenOCD、Ozone 和 J-Link 在宿主机运行，不由通用开发容器访问 USB。

本板按当前官方配置和仓库实测必须使用 **WCH-LinkE**。MounRiver 的 `wch-riscv.cfg` 与
`wch-dual-core.cfg` 都固定为 `adapter driver wlinke`、`transport select sdi`，还依赖
`wch_riscv unfreeze` 等 WCH 扩展；它不是通用的 J-Link SWD/JTAG 链路。仓库的 J-Link / Ozone
配置因此不包含 `ch32_board`，当前安装的 J-Link V9.48 设备库也没有 CH32H417 条目。
[MounRiver 配置与本机实测 2026-08-05]

> **这份 OpenOCD 是 WCH 从主线 0.11 分叉的私有版本，主线和发行版的 openocd 一律不能
> 替代**——`apt install openocd` 装到的东西第一行 `adapter driver wlinke` 就报错，因为
> 主线 0.12 的 72 个 adapter 驱动里没有 wlink/sdi 任何一个。而它的**源码未公开**
> （`openwch/openocd_wch` 只有 2023 年的 Windows 二进制，早于本芯片；源码要发邮件到
> support@mounriver.com 索取），Linux 版**唯一来源**就是 MounRiver 工具链包。
>
> 因为不可替代且无法重建，**已归档入库**：`tools/openocd-wch/`（对仓库根"工具链留在
> 仓库外"通则的有意例外，理由见该目录 README，**不要按通则清掉**）。x86-64 Linux
> only，依赖宿主 `libusb-1.0.so.0`；其他平台装 MounRiver 对应包。
> [本机实测 2026-08-06]

用入库的那份（推荐，路径稳定）：
```bash
OCD=firmware/ch32_board/tools/openocd-wch
$OCD/bin/openocd -f $OCD/bin/wch-riscv.cfg \
    -c "init" -c "wch_riscv unfreeze" -c "halt" \
    -c "program firmware/ch32_board/build/ch32_board_merged.hex verify" \
    -c "reset run" -c "exit"
```

或用 MRS 包内的原始位置。注意是 **`OpenOCD/OpenOCD/bin`** 两层同名目录，
`wch-riscv.cfg` 等 cfg 就和 `openocd` 放在同一个 `bin/` 里：
```bash
OCD=~/3rd_party/MRS_Toolchain_Linux_X64_V240/OpenOCD/OpenOCD
```
- 两处是同一份 `0.11.0+dev-snapshot (2026-02-28)`，三个 cfg（`wch-riscv` /
  `wch-dual-core` / `wch-arm`）与旧版同名同用法，下面所有注意事项照旧适用。
- **`wch_riscv unfreeze` 必须在 `init` 之后、`halt` 之前**。夹到 `halt` 后面它会静默失效，
  症状是 `program` 报 `Read-Protect Status Currently Enabled` 或
  `error writing to flash at address 0x00000000`，而 `flash erase_sector` 却"成功"、
  回读全是 `e339e339`（那是保护态下的假数据，擦净应该是 `ffffffff`）。这一条是
  MounRiver 的调用方式，可用 `-c init -c "wch_riscv unfreeze"` 在其扩展里搜到。
- **不要 `flash erase_sector wch_riscv.flash 0 last`**：会擦到 option byte。`program` 自己
  会擦它要写的扇区，不需要额外擦除；多加一条 flash 命令反而会打掉 unfreeze 的效果。
- **跑板子用 `wch-riscv.cfg`（单 target），不要用 `wch-dual-core.cfg`**：后者的 `reset run`
  会同时放出两个 hart，V5F 就从 flash `0x0` 跑起 V3F 镜像（症状：V5F `mtvec=0x20100003`）。
  只在需要同时观察两个核时用 dual-core cfg（`targets wch_riscv.cpu.0` / `.1`）。
- **halt 之后读到的 PC 不可信**：WCH 的 openocd 在 halt 时会顺带复位，几次都读到复位后
  拷贝代码的循环里（`0x00004020`）。判断固件是否正常运行看 **USB 枚举**，不要看 PC，
  也不要看 `v5f_ready`（每次启动 V3F 都会重新清零）。
- halt 会让主机判 USB 掉线，且 **`resume` 不会让它回来，必须 `reset run`**。
- MounRiver Studio 自带的 openocd 与命令行这份 **二进制和 cfg 完全相同**，
  差别只在调用序列——它能烧而命令行烧不进时，先怀疑序列，不要怀疑工具。

## GDB 调试
先让 openocd 挂着当 gdb server（把上面 `program`/`exit` 换成常驻）：
```bash
OCD=firmware/ch32_board/tools/openocd-wch
$OCD/bin/openocd -f $OCD/bin/wch-riscv.cfg -c "init" -c "wch_riscv unfreeze" -c "halt"
# 另开一个终端
~/3rd_party/MRS_Toolchain_Linux_X64_V240/Toolchain/RISC-V\ Embedded\ GCC12/bin/riscv-wch-elf-gdb \
    firmware/ch32_board/build/ch32_board_app.elf -ex "target extended-remote :3333"
```
- **不要用 `RISC-V Embedded GCC15` 里的 `riscv32-wch-elf-gdb`**：它链的是
  `libpython3.8.so.1.0`，在只装了 python 3.12 的系统上直接 `error while loading
  shared libraries` 起不来。
- 使用 MRS `RISC-V Embedded GCC12` 里的 `riscv-wch-elf-gdb` **12.1** 作为调试器。
  GDB 不需要与编译器同版本，但必须留在 WCH 独立工具包内，不再借用 HPM 工具链。
- 调试同样受"调试口与 USB 抢线"的限制，见 `PITFALLS.md`：halt 会让主机判 USB 掉线，
  且 `resume` 拉不回来，必须 `reset run`。

## 目录结构
- `app/src/`：V5F 上的 C++ libhcs 转发层（`app.cpp` 提供 `main()`，替换被排除的 WCH demo `app/User/main.c`）；`can/ uart/ usb/ timer/ led/ utility/`。
- `boot/src/`：V3F 启动/卸载核。它 **拥有整棵时钟树**（`SystemInit` 开 HSE，V5F 那份 `system_ch32h417.c` 不开），唤醒 V5F，然后跑共享 SRAM mailbox 的 offload 循环。
- `bsp/ch32h417-evt/`：WCH 标准外设库 **submodule**（[`zyitom/ch32h417-evt`](https://github.com/zyitom/ch32h417-evt)，
  上游 `openwch/ch32h417` V1.5 的精简子集），零本地改动、只读。CMake 只认
  `CH32_WCH_ROOT` 一个变量，未 init 时直接 `FATAL_ERROR` 提示 `git submodule update`。
  `wch/Peripheral/` 是**完整 38 个外设驱动**，所以上新外设（sdio / eth / i2c…）不用动
  submodule，include 头文件即可。`examples/` 是官方示例源码（不参与构建），上新外设前
  先看那里——WCH 的 demo 常常是 RM 之外唯一的可运行参考。芯片手册按上游做法只给链接
  不入库，链接在 submodule 的 README。
- `bsp/usb/`：vendored CH372Device USBSS 设备栈，**有本地 patch 所以留在树内**（那些文件
  调用 `app/src/usb/vendor.cpp` 的符号）。submodule 里的 `usb/` 是纯净基线，
  `diff -r bsp/usb bsp/ch32h417-evt/usb` 的每个 hunk 都必须对应一个
  `libhcs LOCAL PATCH` 标记。`bsp/syscalls.c`：newlib stub。
- `bsp/` 与 `app/User/` 下厂商代码视为第三方（只读）；已有的 local patch 一律标 `libhcs LOCAL PATCH` 并登记在 `bsp/PROVENANCE.md`，改动前先读那份清单。
- 无 CubeMX，不适用 CubeMX 纪律。

## 现状（改代码前须知）
- **已上板：USB 3.0 SuperSpeed 枚举成功**（2026-07-25，WCH-LinkE 实测，**官方 EVB**）。
  `A511:D403`、`bcdUSB 3.00`、bulk EP `0x01`/`0x81` 1024B/`bMaxBurst 15`，5 Gbps，
  多次复位稳定。
- **USB SS bulk 天花板已实测**（2026-08-06）：**上行 441 MB/s、下行 462 MB/s、回环往返
  13 µs(min)/26 µs(p50)**，占 500 MB/s 净荷上限的 88%/92%。用 `-Dlibhcs_CH32_USB_BENCH=ON`
  的专用固件（`app/src/usb/bench.cpp`，**默认关，生产镜像不受影响**）+ 纯 libusb 主机程序
  测的。数据、测法和两条硬约束见
  [README.md USB SS bulk 实测天花板](README.md#usb-ss-bulk-实测天花板2026-08-06)。
  其中一条务必记住：**主机发给这块板的 bulk OUT 长度不能是 1024 的整数倍**，否则设备的
  RX chain 既没收满 16 包也没遇到短包，完成中断不触发，包直接卡死到超时。
- **Petros breakout 上首次跑通全流程**（2026-08-06 实测）：`usb 4-2: new SuperSpeed USB
  device`、`/sys/bus/usb/devices/4-2/speed` = 5000，`rx_monitor` 打印
  `Connected: Ch32Board (2 CAN, 2 UART)`。同一次运行的固件计数：下行 88 包、上行装填
  88 包、主机收走 88 包（`diag[15]`/`[23]`/`[25]`），零丢包。**在此之前这块板只验过
  启动链，SS 从未在它上面通过电**——所以旧文档里的 SS 结论都要读作"官方 EVB 上的"。
- **bulk 数据通路已端到端验证**（2026-07-26）：11/11 下行包解析并回 ack，0 丢包。
  EP1-OUT 的链式 DMA 收长语义按 RM 27.2.4 修正过（`UEP_RX_DMA` 自增，实测确认）。
- **主机侧接口已就位**：`libhcs::board::Ch32Board`（`host/include/libhcs/board/ch32_board.hpp`，
  PID `0xD403`，CAN1/CAN2 + UART1/UART2），通道表在
  `core/include/libhcs/spec/ch32_board/`，`host/examples/common/multi_board.hpp`
  里有 `Ch32BoardSession`，所以 `rx_monitor` 等 example 都能直接连这块板。
- **UART 数据面已上板验证**（2026-08-06，`J2.26`↔`J2.27` 短接做 PA9/PA10 自环）：
  `uart_stress` 在 921600 baud 下 **TX 1000 msg/s / 32000 B/s，RX 1000 msg/s / 32000 B/s，
  rx/tx = 100%**。同一条链路在默认 115200 下 RX 稳定在 **11520 B/s = 该波特率的理论上限**、
  每帧 64 B（`max_receive_size` 满缓冲 flush），即线跑满而非丢包——**看到 rx/tx 只有 18%
  先算线速，不要当成丢包**。这条同时证明了 USB SS 双向、USART1 收发、IDLE 组帧都正确。
- **UART 运行时改波特率已上板验证**（2026-08-06，用上面那条自环测的 115200 → 921600
  切换，切换后立即 100% 回环）：下行 `kUartNConfig` 走
  `link::HostSession::uart_config_deserialized_callback` → `Uart::handle_config`，
  主机侧是 `Ch32Board::PacketBuilder::uart1_config` / `uart2_config`。切换时先清
  UE 再 `USART_Init`（`CTLR1_CLEAR_Mask=0x29F3` 保住 UE 和中断使能位，所以 TXE 要
  显式关掉），两个方向在途的字节一律丢弃。BRR 是 12.4 定点，只接受
  `HCLK/65536 < baud <= HCLK/16` 的值，越界**返回 false 而不是断言**（这是主机来的
  数据，坏包不该 panic 固件）。注意可表示 ≠ 精确：量化误差在范围顶端能到 ~4%，
  标准速率不受影响（100 MHz HCLK 下 115200 误差 0.006%、3 MBaud 1.01%）。
- **启动链已上板复验**（2026-07-26，无 USB 线，只接 WCH-Link）：flash 内容 == `app.bin`、
  metadata 里的 sha256 == sha256(flash) → `app_image_is_valid()` 为真、不进 DFU、唤醒 V5F；
  V5F 走完整个 `App()`（`diag[30] == 12`）、无断言、主循环 **release 219 ns/圈（4.6 MHz）、
  debug 3.25 µs/圈**。没插线时 LTSSM 只在 DISABLE/RXDET 之间循环（`diag[11] == 0x30`）、
  `USBSS_DevEnumStatus == 0`，这是预期行为不是故障（见 `PITFALLS.md` 4.5）。
- **看 V5F 死活要按 `PITFALLS.md` 4.7 的姿势**：调试器 attach 期间 V5F 不跑，同一个
  openocd 会话里 `reset run; sleep; halt` 读到的全是残留；必须先 `exit` 断开、自由跑、
  再 attach 读。判据用 `diag[30]`，**不要用 `v5f_ready`**（attach 触发的复位会把它清零）。
- **不要碰 `-Dinterrupt\(x\)=interrupt`**（`CMakeLists.txt`）：WCH 的
  `interrupt("WCH-Interrupt-fast")` 会被主线 GCC 整条丢掉，ISR 收尾变成 `ret` 而
  不是 `mret`，第一个 USB/TIM12 中断进去就永久关中断——USB 死掉的真凶。改工具链后
  用 `objdump -d --disassemble=USBSS_LINK_IRQHandler` 复验必须是 `mret`。
- **初始化顺序有硬约束**：`USBSS_Device_Init(ENABLE)` 里就 `NVIC_EnableIRQ`，所以
  它必须排在 `usb::vendor.init()` / `can` / `uart` **之后**，否则第一个 USB 中断会
  去解引用还没构造的 `Lazy<Vendor>`。
- USB 设备身份（VID/PID + 字符串描述符）在 `app/src/usb/descriptors.cpp`，
  `libhcs_usb_init_descriptors()` 必须在 `USBSS_Device_Init()` 之前调用。
- **USB 2.0 回退已关闭**（`libhcs_USBSS_HS_FALLBACK=0`，`CMakeLists.txt`）。厂商栈在 SS
  训练超时 1.5 秒后会切到 USB 2.0，而 USB2 的 D+/D- 是 **PB8/PB9，同时也是 SWCLK/SWDIO**，
  一切过去调试口就没了、只能断电恢复。关掉之后 SS 失败就一直重试 SS，引脚永不被占。
  已实测：halt 核 5 秒（主机判 USB 掉线）后调试器仍能连上，旧固件此处必断。
  设成 1 可恢复原厂行为并支持 USB 2.0 主机，代价是回退时丢调试口。
- **CAN/USART 引脚已按 Petros breakout 原理图定稿**（2026-08-06），权威表在
  `app/src/board_app.hpp` 的 `kCanPorts` / `kUartPorts`，选型理由见
  [README.md 作为转发板的接线](README.md#作为转发板的接线引脚映射的由来)。要点：
  can1=PA12/PA11(`J2.31/29`)、can2=PB13/PB12(**`J3` 才有**)、uart1=USART1 PA9/PA10
  (`J2.26/27`)、**uart2=USART3 PA13/PA14**(`J2.32/34`)。改这四条之前先读 README 那节，
  封装没引出的脚（`PA2/PA3`、`PB5/PB6/PB7`、`PC4/PC5`、`PF6/PF7`）配上去是静默失效。
  **仍未通电实测收发。**
- **两个核的 printf 都走 USART8**（`J4.5/J4.6`）：`ch32_board_boot` 显式定义
  `DEBUG=DEBUG_UART8`，否则厂商 `debug.c` 会把 V3F 控制台放到 USART1/PA9，那是 uart1
  的 TX。删掉这个定义等于让 bootloader banner 打进转发口。
- **`usb::ss::poll_link_reset()` 必须留在主循环里**（`app/src/usb/vendor.cpp`）：链路掉线
  时厂商栈走 `USBSS_Reset_Init` → `USB_CLR_ALL` 把已装填的 EP1 IN 链丢掉，完成中断永远
  不来，`tx_in_flight` 会永久卡住——症状是**拔插一次 USB 线之后上行彻底哑掉**（还能收
  下行，但 kStart 的 ack 发不出去，主机永远建不起 session）。[代码分析，未上板复现]
- **目标板是 Petros CH32H417M Alef Breakout**（不是官方 EVB），座子用 J 编号：
  `J1`=USB3.0-A、`J2`=Pico 40 脚排针、`J3`=DF12 板对板、`J4`=1x06（调试+串口）。
  **板上没有第二个 USB 座**，USB 数据面和 SWD 共用 PB8/PB9，所以**烧录必须拔 USB 线**，
  没有"走另一个口"的退路。详见 [PITFALLS.md 1.4](PITFALLS.md#14-usb3-座子里有两套线但不能同时用)。
- **DFU 相关的两条硬约束**：
  1. 描述符按 `libhcs_DFU_DEVICE` 在 `bsp/usb/usb_desc.c` 里二选一（app=0、
     boot=1），boot 那份是 DFU-mode（单接口、无端点、`bInterfaceProtocol=0x02`、
     `wTransferSize=512`=EP0 包长）。改端点或接口号要同步改
     `boot/src/usb/dfu_transport.cpp` 的 `kDfuInterfaceNumber`。
  2. **V5F 不能自己复位**：它的复位向量是 flash `0x0`（V3F 镜像），自复位会让 app
     核跑 boot 核的代码（`mtvec=0x2010_0003` 那个症状）。所以 DFU detach 时 V5F 只
     写 `boot_mailbox` + `SharedBlock::reset_request` 然后 park，由 V3F 执行
     `NVIC_SystemReset()`。
- **boot 侧轮询 `usb::dfu` 必须带 memory barrier**：`Dfu` 没有 volatile 成员，全靠
  USBSS 中断改，-O2 下会把读提到循环外变成死转（`boot/src/main.cpp` 的
  `run_dfu_mode()` 里有一条 `__asm volatile("" ::: "memory")`，别删）。
