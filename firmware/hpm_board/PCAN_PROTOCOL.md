# PCAN-USB FD 人格协议参考

> **文档类型**：硬件参考
> **适用范围**：`firmware/hpm_board/app/src/pcan/`（`-Dlibhcs_APP_PCAN_PERSONALITY=ON` 的独立镜像）
> **状态**：现行有效（2026-10-15 首版并完成 Linux 实测：双 CAN 板识别为 PCAN-USB Pro FD，经典 + FD 长帧双向回环通；Windows 侧待实测）
> **相关文档**：[AGENTS.md](AGENTS.md) · [DMTOOL_PROTOCOL.md](DMTOOL_PROTOCOL.md)（同一模式的先例） · [SOF_TIMEBASE.md](SOF_TIMEBASE.md)（本镜像不用的共享时间基）

## 摘要

本文记录 PCAN 人格镜像的协议事实与板端取舍：整块 5321 枚举为 PEAK PCAN-USB FD（单 CAN
PCB）/ PCAN-USB Pro FD（双 CAN PCB），让只认 PCAN 适配器的上位机（Linux SocketCAN
`peak_usb`、Windows PCAN-Basic）直接使用本板。读完可以：核对每条 EP0 请求 / uCAN 命令 /
记录的板端行为、知道哪些 PCAN 特性没实现及为什么、按第 7 节在 Linux 台架上验收与刷回。

协议规范来源是 Linux 内核 `peak_usb` 驱动（`drivers/net/can/usb/peak_usb/`，PEAK 员工
参与编写并随内核发布，GPL-2.0-only）。按仓库纪律它**只作规范读**，实现全部自研——与
DMTOOL_PROTOCOL.md 对 DMTool 逆向的处理相同。

## 本文导航

1. USB 身份与接口布局
2. EP0 vendor 请求
3. uCAN 命令流（bulk OUT 0x01）
4. TX 帧（bulk OUT 0x02/0x03）与 RX 记录流（bulk IN 0x82）
5. 板端取舍与未实现项
6. 符号所有权与构建
7. 主机侧用法：验收、烧录、刷回

## 1. USB 身份与接口布局

- VID `0x0C72`（PEAK）；PID 运行时按 OTP 板型选择：单 CAN `0x0012`（PCAN-USB FD，
  1 通道）/ 双 CAN `0x0011`（PCAN-USB Pro FD，2 通道）。常量在
  `core/include/libhcs/protocol/usb_identity.hpp`（与 DM 身份同一张表）。
- bcdUSB `0x0200`，设备类 0（类由接口定义）。**无 BOS / MS OS 2.0 WCID**：Windows 上
  WCID 会让 WinUSB 抢绑 vendor 接口，反而挡住 PEAK 自己的驱动。
- 接口 0（vendor 0xFF）的端点表——端点号是协议身份的一部分，与 fw_info 的 EXT 端点表
  一致（`pcan_protocol.hpp` 的 `kEp*`）：

| 端点 | 方向 | 用途 |
|---|---|---|
| `0x01` | OUT | uCAN 命令流（8 字节记录序列） |
| `0x81` | IN | 命令应答（Linux 不读，仅声明） |
| `0x02` | OUT | CAN0 发送（`pucan_tx_msg` 记录流） |
| `0x03` | OUT | CAN1 发送（仅双 CAN 板描述符里存在） |
| `0x82` | IN | 共享 RX 记录流（CAN/校准/错误/状态/溢出） |

- 接口 1：DFU runtime（与默认镜像同一份 `TUD_DFU_RT_DESCRIPTOR`），刷回默认镜像的
  通道。内核 `pcan_usb_fd_probe` 只绑接口 0，DFU 接口在 Linux 上保持未认领，dfu-util
  经 libusb 可直接用。[内核驱动行为]
- 高速 bulk 512（FS 64 变体保留；内核 FS 下自行按 64 字节分片命令——`pcan_usb_fd_send_cmd`）。
- 字符串：iManufacturer "PEAK-System"、iProduct 按型号、iSerialNumber 是 fw_info
  `ser_no` 的十进制串（OTP UUID 折叠派生，两处同源）。

## 2. EP0 vendor 请求

bmRequestType = vendor | recipient **OTHER**（内核 `pcan_usb_pro_send_req`），wIndex 恒
0；`pcan_control.cpp` 应答，别的 vendor 请求一律 STALL。wLength 精确匹配，不截断。

| bRequest | wValue | 方向 | 长度 | 板端应答 |
|---|---|---|---|---|
| 0 REQ_INFO | 1 INFO_FW | IN | 36 | `fw_info`：size_of=36、type=2（EXT）、hw_type=1、bl 1.0.0、hw_version=1、**fw 2.0.0**（内核按 fw≥2 启用 ISO/non-ISO 切换，板端用 CCCR.NISO 真实实现）、dev_id[2]（DEVID_SET 只改 RAM）、ser_no、flags=0、端点表（同上节） |
| 0 REQ_INFO | 0 INFO_BL | IN | 28 | `blinfo`（Linux FD 路径不读，按 Pro 系形状填） |
| 2 REQ_FCT | 5 DRVLD | OUT | 16 | 主机写 `[0]=0,[1]=1/0`；板端记录"驱动已加载"，只影响 LED 判据 |

## 3. uCAN 命令流（bulk OUT 0x01）

8 字节记录，`opcode_channel = (channel<<12)|opcode`（10 位），0xff 填充 / `0x3ff` 结尾。
解码在 `pcan_protocol.hpp::for_each_command`，分派在 `pcan_adapter.cpp::handle_command`。

| opcode | 名 | 板端行为 |
|---|---|---|
| 0x001 | RESET_MODE | `Can::set_bus_off()`：控制器进 init 模式撤下总线，**不**置 libhcs 的 suspended 标志 |
| 0x002 | NORMAL_MODE | 清 CCCR.MON 后上线（`set_bus_on`） |
| 0x003 | LISTEN_ONLY_MODE | 置 CCCR.MON 上线（不应答） |
| 0x004 | TIMING_SLOW | 存 `{ewl, sjw_t, tseg2, tseg1, brp_le16}`（均 -1 编码）→ NBTP；sjw_t 位 7 是三采样，MCAN 无此功能忽略；ewl 只存（MCAN 固定 96=内核默认） |
| 0x005 | TIMING_FAST | 存 → DBTP；**fd 与否由"是否收到过本命令"决定**（内核经典模式不设数据段时序） |
| 0x006 | SET_STD_FILTER | `{u8 idx, le32 mask}` → 64×32 位接收位图 |
| 0x008 | FILTER_STD | `{le16 idx(>63=全行), le32 mask}` → 同上 |
| 0x009 | TX_ABORT | 软件队列每主循环轮次已排空，空转（见 §5） |
| 0x00a | WR_ERR_CNT | `Can::clear_error_counters()` 空转（见 §5） |
| 0x00b | SET_EN_OPTION | `{le16 ucan, u16, le16 usb}`：ERROR=发错误记录；BUSLOAD=不实现；CANDFDISO=ISO 模式（CCCR.NISO=0）；usb 0x8000=CALIBRATION 周期记录 |
| 0x00c | CLR_DIS_OPTION | 同上清除（清 CANDFDISO = non-ISO，NISO=1） |
| 0x010 | RX_BARRIER | 回一条带 0x10 位的 STATUS 记录 |
| 0x080 | CLK_SET | 只存；本板恒 80 MHz（与声明一致，MCAN 时钟即 80 MHz，时序字段近直映射） |
| 0x081 | DEVID_SET | `dev_id[ch]` 只写 RAM，**不写 OTP** |
| 0x086 | LED_SET | 0 设备态 / 1 快闪 / 2 慢闪 / 3 常亮 / 4 灭 → RGB LED 强制图案（`led::ForcedPattern`） |

时序应用规则（`Adapter::apply_bus_state`）：TIMING 到达时若通道在线则立即
`reconfigure_timing(fd, nominal, data)`（复用 DMTool 的"主机 TQ 直写"路径，自动重传开），
再按通道意图摆 MON/NISO；若在 reset 模式则先整表重配再撤下。brp 是 10 位字段而 MCAN
NBRP/DBRP 只有 9 位——80 MHz 下低于 ~5 kbps 才可能超界，超界拒收该组时序保旧配置。

## 4. TX 帧与 RX 记录流

### 4.1 TX（bulk OUT 0x02 / 0x03）

`pucan_tx_msg`：`{le16 size(4 对齐), le16 type=0x1000, le32 tag_lo, le32 tag_hi, u8
channel_dlc(低 4 位通道/高 4 位 DLC), u8 client, le16 flags, le32 can_id, data[]}`，
size=0 结尾。flags：EXT_ID 0x02 / RTR 0x01 / SINGLE_SHOT 0x08 / EXT_DATA_LEN 0x10（FD）/
BITRATE_SWITCH 0x20 / ESI 0x40 / SELF_RECEIVE 0x80。板端经 `Can::handle_downlink_as`
逐帧转发（wire DLC 直传，经典端口上 FD 请求降级——与 DMTool 同语义）；队列满丢弃并点
LED。通道号越界（单 CAN 板的 ch=1）丢弃。

### 4.2 RX 记录（bulk IN 0x82）

记录 4 字节对齐；一次 IN 传输严格小于端点包长（短包结尾；内核对满包结尾的传输要等下一
包），**记录绝不跨传输**（内核遇跨 URB 记录报错丢弃）。泵在
`Adapter::poll`，顺序：CALIBRATION → STATUS/BARRIER → ERROR → CAN_RX → OVERRUN。

| 类型 | 长度 | 布局（头 12 字节：size/type/ts_low/ts_high） |
|---|---|---|
| 0x0001 CAN_RX | 28+align4(len) | tag×2（Linux 不用）、channel_dlc、client=0、flags（EXT_ID/RTR/EXT_DATA_LEN/BRS/ESI）、can_id、负载 |
| 0x0100 CALIBRATION | 16 | usb_frame_index=0（内核 `decode_ts` 不读）；CALIBRATION 选项开且至少一路在线时每 ~100 ms 一条（1 kHz tick 块排期）。内核丢弃前 5 条再同步（`cm_ignore_count`），任意节奏可容 |
| 0x0002 ERROR | 16 | `ch&0xf`、code_g=0（内核不读）、tx_err_cnt、rx_err_cnt；ERROR 选项开的通道错计数变化时发（内核 `get_berr_counter` 依赖） |
| 0x0003 STATUS | 16 | `ch | RX_BARRIER 0x10 | PASSIVE 0x20 | WARNING 0x40 | BUSOFF 0x80`；warning/passive/busoff 变化时发（源自 `read_status()` 的 PSR 标志） |
| 0x0101 OVERRUN | 16 | `ch`；ISR 侧接收环形队列（32 深/通道）满丢帧时按计数差补发 |

时间戳：64 位 µs 自由计数器 = PTPC0 原始 {秒,纳秒} ÷ `kCanTimestampNsPerUs`（960）校正
÷ 1000（换算在主循环，与 DMTool 记录流同一取舍）。本镜像**不碰** SOF 捕获与共享时间基
（`sync::time_sync_*` 在 pcan 构建里恒关）。

### 4.3 主机期望序列（验收基准）[内核驱动行为]

probe：INFO_FW → DRVLD(1) → CLK_SET(80MHz) → LED_SET(0)。dmesg 应出
`PEAK-System PCAN-USB FD v1 fw v2.0.0 (1 channels)`。
`ip link set can0 up type can bitrate ... [dbitrate ... fd on]`：FILTER_STD(全收) →
SET_EN_OPTION{ERROR+usb CALIBRATION} → TIMING_SLOW[/FAST] → WR_ERR_CNT(0) → NORMAL_MODE。
down / rmmod：选项关 + RESET_MODE [+ LED_SET(4) + DRVLD(0)]。

## 5. 板端取舍与未实现项

- **SINGLE_SHOT 逐帧标志不跟随**：MCAN 的自动重传（CCCR.DAR）是控制器级开关，本路径
  沿用 `reconfigure_timing` 的自动重传开（与真实适配器默认及 DMTool 路径一致）。
  [设计取舍]
- **WR_ERR_CNT 空转**：本 SoC 的 MCAN ECR 只读（无 CCCR.CEC 类清零位）。命令到来时紧
  接着的时序重配（整表 mcan_init）已把计数归零，内核不核对结果。[RM + 推断，未上板]
- **TX_ABORT 空转**：软件发送队列每个主循环轮次排空；作废语义由 TXBCF 计数承担。[推断]
- **BUSLOAD 不实现**：内核从不置该选项；选项位收下不生效。
- **三采样位忽略**：MCAN 无 SJA1000 式三采样。
- **SELF_RECEIVE/LOOPED_BACK 回显默认关**：Linux 的 TX 回显来自 URB 完成而非设备。若
  Windows 上位机等 TX 确认，按 `pcan_protocol.hpp` 加 ~15 行软件回显（tag 透传）。
  [Windows 验证项]
- **扩展 ID 绕过过滤位图**：64×32 位图只覆盖标准 ID 空间，与真实硬件一致。
- **主机拔出/总线复位**：协议状态全清（含过滤位图回全收），控制器的电气状态保持——
  真实适配器在意外拔出后同样保持，新主机的 ifup 序列会整体重配。
- **上电即 reset 模式**：所有通道 `set_bus_off()` 等主机命令（真卡语义；默认镜像那种
  "上电即 DMTool 适配器"的在线默认会让错误速率的监听节点往真总线上打错误帧）。

## 6. 符号所有权与构建

默认镜像与 pcan 镜像各自持有互斥的一组单定义符号（一个构建只能一份）：

| 符号 | 默认镜像 | pcan 镜像 |
|---|---|---|
| `usbd_app_driver_get_cb` | dm_adapter.cpp | pcan_adapter.cpp |
| `tud_vendor_control_xfer_cb` | vendor_control.cpp | pcan_control.cpp |
| `tud_descriptor_{device,configuration,string}_cb` | usb_descriptors.cpp | pcan_descriptors.cpp |
| `hcs_usb0_isr` / USB IRQ 优先级 | vendor.cpp | pcan_control.cpp |
| DFU 重启 cb + 延时复位 | vendor.cpp | pcan_control.cpp |
| `link::uplink_enabled/uplink_serializer` | uplink_usb.cpp | pcan_control.cpp（`.fast` 桩，恒 false / 空缓冲） |

构建（详见 [AGENTS.md](AGENTS.md) 第 1 节）：超建默认产出两个 app 子构建
（`hpm_board_app` / `hpm_board_app_pcan`，binary 目录 `app/` / `app_pcan/`）；源互斥在
`app/CMakeLists.txt` 的 glob 过滤里（pcan 构建排除 dmtool/uart/vendor/vendor_control/
usb_descriptors/uplink_usb/can_diag，默认构建排除 pcan/）。CAN_DIAG / PULSE_TEST /
SOF_DIAG 借 UART0，与人格互斥（CMake FATAL_ERROR 兜底）。

hpm5321 链接脚本的 ILM tripwire 按修饰名点名 libhcs 上行缓冲模板；pcan 构建不实例化它，
binutils 对表达式里的未定义符号报错，故 pcan 构建把该名 `--defsym` 成 0（断言平凡通过），
memcpy/mcan 那条 tripwire 仍对两镜像生效。

**tud_mount_cb 不能清适配器状态**（2026-10-15 实测踩坑）：mount 回调在
SET_CONFIGURATION 成功后触发，此时 `open()` 刚把 bulk OUT 挂上；把 `on_usb_reset()`
接到 mount 上等于每次配置成功后自毁——现象是 EP0 全通（fw_info/DRVLD 应答完美）而
bulk OUT 全程 NAK（内核 `-110` 超时）。总线复位的清理走 `driver_reset`，umount 才清
协议状态，挂起/恢复保持（pcan_adapter.cpp 的回调区有此约束的注释）。[实测 2026-10-15]

**默认镜像回归判据**（2026-10-15 实测）：改动前后默认镜像符号表、map 与全部代码数据
一致；整个 .bin 仅符号间填充区 1 字节链接器噪音差异（改任何被 .data 符文所在编译单元
引用的头文件都会扰动填充字节，逐字节相同在实践上不可达；DMTool 人格当年用的也是反汇编
级对照）。[实测]

## 7. 主机侧用法

### 7.1 Linux 验收（首选，peak_usb 即规范）[实测 2026-10-15 全流程通过]

```bash
# 构建(超建 ALL 同时产出默认镜像; 产物在 <build>/app_pcan/output/)
cmake --preset release -S firmware/hpm_board -B <build> -DBOARD=hpm5321
cmake --build <build> --target hpm_board_app_pcan

# 烧录(经 DFU; 0x0c72 没有仓库的 udev 放行规则, 本机要 sudo)
sudo dfu-util -d 0x0c72:0x0011,0xa511:* -a 0 -D <build>/app_pcan/output/hpm_board_app_hpm5321_pcan.dfu
dmesg | tail    # 应见 PEAK-System PCAN-USB Pro FD v1 fw v2.0.0 (2 channels)
sudo ip link set can3 up type can bitrate 1000000 dbitrate 5000000 fd on sample-point 0.875 dsample-point 0.875
candump can4        # 对端: 电机本体, 或同一块板的另一路(实测用的就是双路自回环)
cansend can3 123#DEADBEEF
```

注意（实测）：
- 接口名按主机已有 CAN 设备顺延（本机 can0-2 被一块 gs_usb 卡占用，本板得到
  can3/can4）；`dmesg | grep "attached to PCAN"` 能看到确定的接口名。
- **重刷要先把驱动摘掉**（`sudo modprobe -r peak_usb`）：绑定成功后 peak_usb 占着
  接口，dfu-util 会 `LIBUSB_ERROR_BUSY`；驱动绑定失败（早期调试阶段）反而没这问题。
- 命令行 `bitrate/dbitrate/sample-point/dsample-point` 全部按声明被接受（87.5% 采样
  点由主机算出 TQ 直写 NBTP/DBTP）。

### 7.2 刷回默认镜像

DFU runtime 接口随人格保留：`dfu-util -d 0x0c72:0x0012 -a 0 -D
.../hpm_board_app_hpm5321.dfu`（detach 后设备以 bootloader `0xA511:0x5321` 继续下载）。
兜底：PA07 拉住地强制驻留 bootloader（见 AGENTS.md 第 2 节），再 `dfu-util -d 0xa511:*`。

### 7.3 Windows（待实测）

装 PEAK 驱动包（PCAN-Basic / Device Driver）后用 PCAN-View 冒烟，再上电机调试工具。
已知待验项：驱动对字符串/hw_type 的核验、TX 是否需要 SELF_RECEIVE 回显（§5）。
若拒认，调整集中在 `pcan_protocol.hpp` 的 `kHwType/kFwVersion` 与
`pcan_descriptors.cpp` 的字符串常量。
