# hpm_board 固件指南

> **文档类型**：现行规范（板级）
> **适用范围**：`firmware/hpm_board/`，HPMicro HPM6E8Y / HPM5321（Andes RISC-V）
> **状态**：现行有效（2026-10-06 精简：只留命令与约束，实测与来龙去脉在下列 L3 文档）
> **相关文档**：[仓库根 AGENTS.md](../../AGENTS.md) · [PITFALLS.md](PITFALLS.md) · [USB_OPTIMIZATION_LOG.md](USB_OPTIMIZATION_LOG.md) · [BUILD_ENVIRONMENT.md](BUILD_ENVIRONMENT.md) · [SOF_TIMEBASE.md](SOF_TIMEBASE.md) · [CONTROL_TIMING.md](CONTROL_TIMING.md) · [DMTOOL_PROTOCOL.md](DMTOOL_PROTOCOL.md) · [../../HOST_TUNING.md](../../HOST_TUNING.md)

## 摘要

RISC-V（Andes 核）+ HPM SDK 超级构建，单核 USB vendor bulk 数据固件。下面每条是"现在该怎么做 /
不要做什么"，括号里是理由所在的 L3 文档。本机工具链位置以 [ENV.md](../../ENV.md) 为准。

## 1. 芯片、工具链、构建

- **RISC-V，不是 ARM**：HPMicro GNU 工具链 `rv32imac_zicsr_zifencei_multilib_b_ext`
  （`riscv32-unknown-elf-gcc`），**不要**用 arm-none-eabi-gcc。工具链在仓库外，见 `BUILD_ENVIRONMENT.md`。
- HPM SDK v1.12.0 随仓库（`bsp/hpm_sdk`，fork `zyitom/hpm_sdk`，分支 `migrate-v1.12.0`），只读。
- TinyUSB 用共享的 `firmware/common/bsp/tinyusb` v0.21.0（`cmake/current_tinyusb.cmake`），构建前初始化该 submodule。
- 构建需 Python 3 + `PyYAML`、`jinja2`。preset `debug` / `debug-outside` / `release`（小写）。

```bash
export PATH=~/3rd_party/hpm/bin:$PATH        # [前机路径]
cmake --preset release -S firmware/hpm_board -B <build> -DBOARD=hpm5321   # 单 CAN / 双 CAN 共用一个镜像
cmake --build <build> --target hpm_board_app                              # 或 hpm_board_bootloader
```

- 无 CubeMX。`app/src/xcore/` 与 `libhcs_APP_RELEASE_CORE1` 是 EtherCAT 遗留，默认编译为空，待清理。

## 2. 烧录

在仓库根 `./tools/flash.sh hpm5321`（默认 release，末尾加 `debug`）。手工 DFU 要写两段身份：

```bash
dfu-util -d 0x34b7:0x6632,0xa511:* -a 0 -D <build>/app/output/hpm_board_app_hpm5321.dfu   # 多块板加 -p <USB 路径>
```

USB 身份（唯一来源 `core/include/libhcs/protocol/usb_identity.hpp`）：应用 `0x34B7:0x6632`（双 CAN）/
`0x6877`（单 CAN）；bootloader `0xA511:0x5321/0x5322`。libhcs 按产品串 `HCS Agent v<版本>` 认板。

## 3. 主机侧调优（每次重启重做）

在仓库根 `sudo ./host-tuning.sh`（`--check` 只报告，`--pmqos` 测量期间另开终端持住）。除内核 cmdline 外不持久化。
测 USB 延迟前必须先调；压测（gap=0）与 1 kHz 控制环结论方向相反，别混用（`HOST_TUNING.md` 1.1）。

## 4. CAN

- **采样点钉死 87.5%**（仲裁与数据段，`can.hpp` 的 `*_samplepoint_min/max = 875`），不要改回 SDK
  默认 75%，也不要照厂商推荐表改。症状：classic 双向通、FD 单向不通（`PITFALLS.md` 第 3 节）。
- **协议、速率、采样点由对端电机决定**，不是可调参数：优化不要往"升 FD / 提波特率 / 改采样点"走。
- 帧型与速率由主机 EP0 声明下发（`hcs::kClassic1M` / `hcs::kFd1M5M`）；经典模式让控制器
  `CCCR.FDOE=0`，经典总线上收不到 FD 帧是有意的。设置存在 `Can::libhcs_setting_`，挂起/恢复按它重建。
- libhcs 路径的 `mcan_config_t` 只有一份：`Can::libhcs_config()`，不要另抄；DMTool 的
  `reconfigure_timing` 是另一条路，保持原样。
- **消息 RAM 只在 `Can::apply_message_ram_layout()` 改**（超 640 词 `mcan_init` 静默失败；见 `DMTOOL_PROTOCOL.md` 6.1）。
- **CAN 单发**：libhcs 关自动重传，作废帧由 `send_to_fifo()` 按 `TXBCF` 计数（端口状态 `kPortStatus` 的 `tx_cancelled`）；
  主机按 CAN ID 从大到小发（`PITFALLS.md` 第 9 节）。保持单发是用户决定。

## 5. UART

- 运行时改波特率：`DLL`/`DLM` 只在 TX DMA 停稳后读（DLAB=1 时与 `THR` 共址）。`snapshot_divisor()`
  只在 init 与 `abort_transmit()` 之后采样；`uart_set_baudrate()` 求解失败时不清 DLAB，调用方必须自己清（`PITFALLS.md` 第 4 节）。
- 速率一致性比分频器整数（`divisor`/`oversample`），不比波特率；百分比只作 10% 兜底。

## 6. EP0 配置通道（协议在 `core/include/libhcs/protocol/vendor_control.hpp`）

- **分工**：每帧的、要和数据定序的走 bulk（`0x04`/`0x84`）；构造期问一次、失败必须让主机看见的走 EP0。
  `kSession`（keepalive）不搬 EP0：它证明的是 bulk 管道活着。
- 逻辑在 core：`core/src/link/ep0.hpp`（分发 + 清单两阶段提交）、`session.hpp`（nonce 握手 + 4 s 租约）、
  `ownership.hpp`（归属）。本板只给注册表（`boards/<board>/app/ports.hpp`，绑错是编译错误）与板级上下文
  （`app/src/usb/vendor_control.cpp`：归属交接、时延分解、时间基准开关）。
- **会话胶水在 common（2026-10-06）**：kStart/kKeepalive 应答、租约、断联下线、kTimeAnchor 应答与批量缓冲宿主
  在 `firmware/common/app/src/link/host_session.hpp` 的 `link::HostSession` 模板（三板同一份；原
  `app/src/link/host_session.hpp` 已删）。本板 Vendor 只剩传输形态、归属交接、脉冲交换与 TimeSync 策略；
  `UplinkAllocFailed` 必须留在 `link` 命名空间（hpm5321 链接脚本的 ILM tripwire 按修饰名点名它）。
- 请求：`0x4A kGetPortList`（纯读，含线格式指纹 `kVersion` 与 `board_caps`）、`0x4B kGetPortConfig`、
  `0x4D kApplyManifest`、`0x4E kGetManifestResult`、`0x47 kGetLatencyBreakdown`、
  `0x48 kGetLastConfigError`。wIndex = DataId（丝印号）。退役编号（`0x40`–`0x46`、`0x49`、`0x4C`）不复用。
- **运行时状态不走 EP0**：CAN/串口的错误与丢帧随 keepalive 应答推送（`kPortStatus`，
  [PROTOCOL.md](../../core/PROTOCOL.md) 1.3）；驱动只提供 `read_status()`，串口的 `LSR` 只在那里读。
- **配置即声明，一次事务**：校验判完一切可预见的拒绝且不碰硬件 → 先从 DMTool/CDC 接手（端口全挂起）
  → 应用 → 全部生效才落定；中途失败所有口挂起、板子还给原主人。没声明的口不工作。
- **ACK = 已生效，STALL = 拒绝**：每条 STALL 路径先记锁存（`0x48` 粘滞）再返回。
- EP0 不受会话门控；清单没被接受的主机开不了会话（`PITFALLS.md` 第 5 节）。

## 7. USB 现行约束（数据见 `USB_OPTIMIZATION_LOG.md`）

- **下行不背压**：`CFG_TUD_VENDOR_RX_MANUAL_XFER` 不设；手动重挂、水位、逃生阀已删，不要加回。
- **主循环里不加无条件的工作**：周期性检查放 1 kHz tick 块，"声明了才有"的挂在掩码后；每趟路径先用内存
  状态判断有没有活，再碰外设寄存器（一次寄存器读约 30 周期，整趟只有 150–220）。
- 主循环里 CAN 排在 bulk 前面；UART 与 CAN 共用一条 bulk 管道。
- 测包率前 `lsof /dev/ttyACM*`：挂着任何 IN（开着的串口也算）包率上限就从 11 降到 8 个/微帧。
- 调不动的旋钮别再 A/B：`CFG_TUD_TASK_EVENTS_PER_RUN`、host transfer 池深、`CFG_TUD_VENDOR_RX_NEED_ZLP`、multi-qTD。
  `CFG_TUD_VENDOR_RX_ARM_FIRST`（先挂下一块 OUT 再回调）只本板打开，保留。
- **不要把 TinyUSB 热路径放 ILM**（`TU_ATTR_FAST_FUNC` 加在 `tud_task_ext` 一类上是净倒退）。
- **每帧热路径不许碰 flash**：每帧执行的代码与只读表必须在 ILM，新增每帧调用要在
  `boards/hpm5321/linker/app_flash_uf2.ld` 补规则（失配即链接期 `ASSERT` 失败）。
- 共享 TinyUSB 里本板依赖的改动（动之前先读 `USB_OPTIMIZATION_LOG.md`）：ChipIdea SETUP tripwire（正确性
  修复，必须保留）、`CFG_TUD_MEM_DCACHE_ENABLE=0` 编掉 dcache 调用、ISR 只扫 `ENDPTCOMPLETE` 置位、
  无 sof 驱动时跳过 SOF 遍历、`CFG_TUD_VENDOR_RX_ARM_FIRST`。CLEAR_FEATURE(HALT) 每次都复位 toggle，不要门控。

## 8. DMTool 兼容（HPM5321，代码在 `app/src/dmtool/`，协议见 `DMTOOL_PROTOCOL.md`）

- 本板同时是一块达妙 USB2FDCAN：接口 0-2 / 端点 `0x01-0x03`、`0x81-0x83` 归 DMTool（写死在其二进制里），
  libhcs 在接口 3（`0x04`/`0x84`），CDC 在接口 4-5，DFU runtime 在接口 6。
- **libhcs 优先，唯一入口是清单声明**：交接步骤是 `usb/vendor.hpp` 的 `BoardOwnership`
  （`dmtool::Handoff` → `PortHandoff` → `sync::TimeSyncHandoff`），交还时倒序。交接是冷路径，留在 `vendor.cpp`。
- 接手后 DMTool 的 6 个端点与 CDC bulk IN、通知端点**关使能位不应答**（`Adapter::isolate()`）：
  不要改成 STALL；**CDC bulk OUT 不要关**；会话结束时 CDC bulk IN 的 halt 留给主机清；交还时按主机最近一次
  的线路编码重写 UART（`Adapter::release()`）。（`DMTOOL_PROTOCOL.md` 第 4 节、5.1、6.4）
- **CDC 桥不看 DTR**：DMTool 从不调 `setDataTerminalReady`（2.1.9.3 / 2.1.9.4 导入表核对），Windows 上
  没人拉 DTR。桥在主机线路编码与 UART 实际速率一致、且板子不归 libhcs 时接通。LED 只在桥上最近 1 s
  真有字节时才算会话（`dmtool::session_established(tick)`）：Linux 的 cdc_acm 枚举时就发
  SET_LINE_CODING，只看桥会让闲置的板常亮绿。
- **热路径不为 DMTool 付代价**：CAN / UART RX ISR 的 libhcs 分支与没有 DMTool 时逐条相同；改接入点后
  对照旧镜像比反汇编（`DMTOOL_PROTOCOL.md` 第 5 节）。
- 只转发：重复发送、自测、IAP、写 SN 一律回失败；总线参数由 DMTool 说了算（只影响仿真路径）。
- udev（同时让 ModemManager 不探测 CDC）：

  ```text
  SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", MODE="0666", TAG+="uaccess"
  SUBSYSTEM=="tty", ATTRS{idVendor}=="34b7", ENV{ID_MM_DEVICE_IGNORE}="1", MODE="0666"
  ```

## 9. CAN 帧时间戳与共享时间基准（机制与实测见 `SOF_TIMEBASE.md` 第 8 节）

- **时间基准是声明，不是编译开关**（v14）：主机 `hcs::Configuration::enable_time_sync()`，本板在
  `board_caps` 报 `kBoardCapTimeSync`。开着时每个收到的 CAN 帧带 3 字节 `SofStamp`；关着时 SOF 中断不开，
  USB / CAN 中断只多读一次 `sync::time_sync_on()`（DLM）。`libhcs_TIME_SYNC` 已删，不要加回。
- PTPC0 的输入捕获被占用（TRGM `USB0_SOF` → `MCAN_PTPC0_CAP`，双沿）：不要另作他用，CAN TSU 不换时基。
- 捕获接通不在 boot 路径：`sof_capture::start()` 只由 `sync::time_sync_start()`（清单应用时）调（`PITFALLS.md` 第 8 节）。
- 帧只用它附近的 SOF 锁存值定位，**不要改回长窗口拟合**（游走，8.7.8）；`note_sof()` 的两道归属检查不要删。
- `stamp_of()` 在 `.fast`，环模板由链接脚本按名收进 ILM；改 `sof_capture_ring.hpp` 后看反汇编，只许 32 位 `divu`。

## 10. 主机侧板类

`Hpm5321` 同时服务单 CAN 与双 CAN 两块 PCB：spec 口表是镜像容量，实有哪些口读 `kGetPortList`；接线表用了
板上没有的口在构造时抛；一个设备可有多个 PID（`std::span<const uint16_t>`）。（`PITFALLS.md` 第 7 节）

## 11. 中断与诊断

- 延迟相关事件全部中断驱动：CAN RX 优先级 3（ISR 内排空 + 序列化）> USB 2 > UART 1；1 kHz tick（MTIP，绕过
  PLIC）只自增计数，LED 等工作在主循环。CAN TX 完成中断不开。（`HOST_TUNING.md` 第 8 节）
- 板上调试口只有通孔焊盘，不当数据/日志通道；探针只用 J-Link。
- 带内诊断（都编成 UART0 上行帧）：`-Dlibhcs_CAN_DIAG=ON`（CAN 转发计数、主循环周期）、`-Dlibhcs_SOF_DIAG=ON`
  （FRINDEX 直方图）、`-Dlibhcs_PULSE_TEST=ON`（跨板脉冲，借走 UART0）。`CAN_DIAG` 与 `SOF_DIAG` 不得同开，
  A/B 前先对齐 CMakeCache（`PITFALLS.md` 第 8 节）。对应主机工具已随 `host/examples/` 删除。
