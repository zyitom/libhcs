# PCAN-USB FD 人格协议参考

> **文档类型**：硬件参考
> **适用范围**：`firmware/hpm_board/app/src/pcan/`（`-Dlibhcs_APP_PCAN_PERSONALITY=ON` 的独立镜像）
> **状态**：现行有效（2026-10-16 原版 Windows 驱动初始化修复与持续收发验收见 7.13）
> **相关文档**：[AGENTS.md](AGENTS.md) · [DMTOOL_PROTOCOL.md](DMTOOL_PROTOCOL.md)（同一模式的先例） · [SOF_TIMEBASE.md](SOF_TIMEBASE.md)（本镜像不用的共享时间基）

## 摘要

本文记录 PCAN 人格镜像的协议事实与板端取舍。原 PEAK 身份已补齐 Windows 设备初始化
交换，官方驱动 5.1.3.20113 与原版 x86 DLL 4.5.4.508 的持续/恢复测试见 7.13。
第 7 节保留早期停更与独立 WinUSB 方案的历史；通过版本的结果不能外推到所有 PCAN 功能。

原始人格的设计是：整块 5321 枚举为 PEAK PCAN-USB FD（单 CAN
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
| `0x01` | OUT | uCAN 命令流及变长设备初始化记录 |
| `0x81` | IN | Windows 设备初始化命令应答（独立于 CAN 数据 IN） |
| `0x02` | OUT | CAN0 发送（`pucan_tx_msg` 记录流） |
| `0x03` | OUT | CAN1 发送（仅双 CAN 板描述符里存在） |
| `0x82` | IN | 共享 RX 记录流（CAN/校准/错误/状态/溢出） |
| `0x83` | IN | Pro FD 保留管道（仅双 CAN 板描述符里存在，不传数据） |

双 CAN 接口保留六个 bulk 端点，按 `0x81/0x01/0x82/0x02/0x83/0x03` 排列；
`0x83` 对应内核协议常量 `PCAN_USBPRO_EP_UNUSED`。首版只声明五个端点，Linux 能枚举
不能证明 Windows 驱动接受该布局。[内核协议；Windows 影响待抓包确认]

- 接口 1：DFU runtime（与默认镜像同一份 `TUD_DFU_RT_DESCRIPTOR`），刷回默认镜像的
  通道。单 CAN 的 `pcan_usb_fd_probe` 按接口号筛选；Pro FD 使用
  `pcan_usb_pro_probe`，它按端点地址筛选，零端点 DFU 接口也会通过，因而双 CAN 板可能
  被重复注册成四路 SocketCAN（接口 1 的两路不是额外物理口）。重刷前需释放驱动，见
  第 7.1 节。[内核驱动；2026-10-15 本机枚举实测]
- 高速 bulk 512（FS 64 变体保留；内核 FS 下自行按 64 字节分片命令——`pcan_usb_fd_send_cmd`）。
- 字符串：iManufacturer "PEAK-System"、iProduct 按型号、iSerialNumber 是 fw_info
  `ser_no` 的十进制串（OTP UUID 折叠派生，两处同源）。

## 2. EP0 vendor 请求

bmRequestType = vendor | recipient **OTHER**（内核 `pcan_usb_pro_send_req`），wIndex 恒
0；`pcan_control.cpp` 应答，别的 vendor 请求一律 STALL。信息读取允许先读结构头、
读完整结构或提供更大的缓冲，实际 IN 长度为 `min(wLength, 结构长度)`；不能因为主机
请求的长度不是 36 / 28 就 STALL。DRVLD 仅接受 OUT，信息读取仅接受 IN。

| bRequest | wValue | 方向 | 长度 | 板端应答 |
|---|---|---|---|---|
| 0 REQ_INFO | 1 INFO_FW | IN | 最多 36 | `fw_info`：size_of=36、type=2（EXT）、hw_type=1、bl 0.1.0、hw_version=1、**fw 3.5.4**（内核按 fw≥2 启用 ISO/non-ISO 切换，板端用 CCCR.NISO 真实实现）、dev_id[2]（DEVID_SET 只改 RAM）、ser_no、flags=0、端点表（同上节） |
| 0 REQ_INFO | 0 INFO_BL | IN | 最多 28 | `blinfo`（Linux FD 路径不读，按 Pro 系形状填） |
| 2 REQ_FCT | 5 DRVLD | OUT | 2–16 | 主机写 `[0]=0,[1]=1/0`（Linux 用 16 字节）；板端记录"驱动已加载"，只影响 LED 判据 |

## 3. uCAN 命令流（bulk OUT 0x01）

8 字节记录，`opcode_channel = (channel<<12)|opcode`（10 位），0xff 填充 / `0x3ff` 结尾。
解码在 `pcan_protocol.hpp::for_each_command`，分派在 `pcan_adapter.cpp::handle_command`。

| opcode | 名 | 板端行为 |
|---|---|---|
| 0x001 | RESET_MODE | `Can::set_bus_off()`：控制器进 init 模式撤下总线，**不**置 libhcs 的 suspended 标志 |
| 0x002 | NORMAL_MODE | 已收到且成功应用时序才清 CCCR.MON 上线；失败保持离线并报 STATUS/BUSOFF |
| 0x003 | LISTEN_ONLY_MODE | 时序成功应用后置 CCCR.MON 上线（不应答）；失败同 NORMAL_MODE |
| 0x004 | TIMING_SLOW | 存 `{ewl, sjw_t, tseg2, tseg1, brp_le16}`（均 -1 编码）→ NBTP；sjw_t 位 7 是三采样，MCAN 无此功能忽略；ewl 只存（MCAN 固定 96=内核默认） |
| 0x005 | TIMING_FAST | 存 → DBTP；**fd 与否由"是否收到过本命令"决定**（内核经典模式不设数据段时序） |
| 0x006 | SET_STD_FILTER | `{u8 idx, le32 mask}` → 64×32 位接收位图 |
| 0x008 | FILTER_STD | `{le16 idx(>63=全行), le32 mask}` → 同上 |
| 0x009 | TX_ABORT | 撤下该路控制器，取消软件请求与未编码的发送确认，释放持有的 OUT 包；按已保存时序/模式重建（见 §5） |
| 0x00a | WR_ERR_CNT | `Can::clear_error_counters()` 空转（见 §5） |
| 0x00b | SET_EN_OPTION | `{le16 ucan, u16, le16 usb}`：ERROR=发错误记录；BUSLOAD=不实现；CANDFDISO=ISO 模式（CCCR.NISO=0）；usb 0x8000=CALIBRATION 周期记录 |
| 0x00c | CLR_DIS_OPTION | 同上清除（清 CANDFDISO = non-ISO，NISO=1） |
| 0x010 | RX_BARRIER | 回一条带 0x10 位的 STATUS 记录 |
| 0x080 | CLK_SET | 只存；本板恒 80 MHz（与声明一致，MCAN 时钟即 80 MHz，时序字段近直映射） |
| 0x081 | DEVID_SET | `dev_id[ch]` 只写 RAM，**不写 OTP** |
| 0x086 | LED_SET | 0 设备态 / 1 快闪 / 2 慢闪 / 3 常亮 / 4 灭 → RGB LED 强制图案（`led::ForcedPattern`） |

时序应用规则（`pcan_bus.hpp::BusState`，由 `Adapter::apply_bus_state` 调用）：reset 模式
只存时序、不上线；NORMAL/LISTEN 时应用 `reconfigure_timing(fd, nominal, data)`（复用
DMTool 的 TQ 直写路径，自动重传开），成功后才摆 MON/NISO、打开收发闸和通道指示灯。
字段越过 MCAN 上限或驱动返回失败时再次 `set_bus_off()`，避免失败救援使用的默认配置
留在总线上；保留主机意图供后续有效 TIMING 重试，ERROR 选项开启时用 STATUS/BUSOFF
报告通道不可用。这里的 BUSOFF 也可能表示配置被拒绝，并非只有物理总线错误。
nominal 的 seg1 用 16 位解码，避免线上的 255（实际 256 TQ）在加一后回绕成 0。
[代码 + 主机回归测试，配置拒绝未上板注入]

## 4. TX 帧与 RX 记录流

### 4.1 TX（bulk OUT 0x02 / 0x03）

`pucan_tx_msg`：`{le16 size(4 对齐), le16 type=0x1000, le32 tag_lo, le32 tag_hi, u8
channel_dlc(低 4 位通道/高 4 位 DLC), u8 client, le16 flags, le32 can_id, data[]}`，
size=0 结尾。flags：EXT_ID 0x02 / RTR 0x01 / SINGLE_SHOT 0x08 / EXT_DATA_LEN 0x10（FD）/
BITRATE_SWITCH 0x20 / ESI 0x40 / SELF_RECEIVE 0x80，LOOPED_BACK 0x04 也触发回显。
板端经 `Can::handle_downlink_as` 逐帧转发（wire DLC 直传，经典端口上 FD 请求降级——
与 DMTool 同语义）。CAN 软件队列或发送确认槽满时，保留当前记录及其 DMA OUT 缓冲，
暂停重挂该路数据 OUT，主循环有空位后重试；整包收下才重挂。命令 OUT 独立，RESET /
TX_ABORT 可释放积压。reset 模式或配置失败的通道丢弃，不进入 CAN 发送队列。

每通道 128 个发送确认槽保留原始 64 位 tag、client、ID 与负载，不覆盖尚未确认的请求。
只有 MCAN TX Event FIFO 的真实完成事件才能生成回显，USB 收到请求不等于线上发送
成功；FD/BRS/ESI 与时间戳取自硬件事件，回显记录编码到 IN 暂存包后释放槽位。
[代码 + 主机测试 + 第 7.6 节上板实测]

### 4.2 RX 记录（bulk IN 0x82）

记录 4 字节对齐；一次 IN 传输严格小于端点包长（短包结尾；内核对满包结尾的传输要等下一
包），**记录绝不跨传输**（内核遇跨 URB 记录报错丢弃）。泵在
`Adapter::poll`，顺序：CALIBRATION → STATUS/BARRIER → ERROR → TX 回显 → CAN_RX → OVERRUN。
已编码的短包保留到 USB IN 成功完成；端点 claim、提交或完成失败时重试原包，避免记录
出队后在 USB 提交失败路径上消失。[代码，故障重试分支未注入测试]

| 类型 | 长度 | 布局（头 12 字节：size/type/ts_low/ts_high） |
|---|---|---|
| 0x0001 CAN_RX | 28+align4(len) | tag×2、channel_dlc、client、flags、can_id、负载；外部接收的 tag/client=0；TX 回显透传请求的 tag/client，带 LOOPED_BACK 0x04 及请求的 SELF_RECEIVE 0x80 |
| 0x0100 CALIBRATION | 16 | 64 位 µs 时间戳与同一毫秒帧内采样的 USB 帧号（`FRINDEX>>3`，11 位回绕）；CALIBRATION 选项开且至少一路在线时每 ~100 ms 一条（1 kHz tick 块排期） |
| 0x0002 ERROR | 16 | `ch&0xf`、code_g=0（内核不读）、tx_err_cnt、rx_err_cnt；ERROR 选项开的通道错计数变化时发（内核 `get_berr_counter` 依赖） |
| 0x0003 STATUS | 16 | `ch | RX_BARRIER 0x10 | PASSIVE 0x20 | WARNING 0x40 | BUSOFF 0x80`；warning/passive/busoff 变化时发（源自 `read_status()` 的 PSR 标志） |
| 0x0101 OVERRUN | 16 | `ch`；ISR 侧接收环形队列（32 深/通道）满丢帧时按计数差补发 |

STATUS 的故障位须经 `encode_status_bits()` 显式转换：内部 `CanStateFlags` 在 bit0/1/2，
uCAN 却要求 bit5/6/7，低四位留给通道号。CAN0 的 passive + warning 应编码为 `0x60`，
不能直接发 `0x03`（会被主机理解成通道 3，掩盖真实故障）。[内核协议 + 主机回归测试]

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
- **TX_ABORT 冷重建通道**：当前不区分命令的 FLUSH 标志，均取消该路待发数据与未编码的
  回显，按已保存时序/模式重建控制器；硬件 RX FIFO 也随重建清空。已编码或在途的 IN 包
  保持不变，之后的 RX_BARRIER 标记用于主机排空旧记录。已测试 FLUSH=1 → RX_BARRIER
  恢复路径，见第 7.6 节。[代码 + 上板实测]
- **BUSLOAD 不实现**：内核从不置该选项；选项位收下不生效。
- **三采样位忽略**：MCAN 无 SJA1000 式三采样。
- **SELF_RECEIVE/LOOPED_BACK 回显按请求开启**：不额外生成总线报文，只把实际发送完成
  转成带原始 tag/client 的 CAN_RX 确认。未请求回显的帧不占确认槽。[代码 + 上板实测]
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

### 7.3 Windows 兼容验证

2026-10-15，用户反馈 Win11、PCAN 驱动 5.1.3，CAN 接口硬件 ID 为
`USB\VID_0C72&PID_0011&REV_0354&MI_00`，启动失败 Code 10。这说明报错来自接口 0，
单凭硬件 ID 不能判定是描述符校验还是初始化请求失败。当前修正包括补齐保留端点、
放宽 EP0 信息读取长度、接受两字节 DRVLD，并修复未知请求未 STALL 的分支。
修正版经 DFU 刷入后，用户反馈 Windows PCAN 已可使用，随后反馈功能正常、此前的
单路疑问与关闭后闪灯现象暂时无法复现。这是用户现场反馈，尚无 Windows USBPcap
抓包，不能据此认定上述哪一项修正解决了 Code 10。[用户反馈 2026-10-15]

装 PEAK 驱动包（PCAN-Basic / Device Driver）后用 PCAN-View 冒烟，再上电机调试工具。
2026-10-16 用户追加反馈：Windows 意优软件最初能连接，数包后显示旧数据，发送也无效，
关闭重开不恢复，必须拔插 USB。当前已补发送确认与 TX_ABORT，并完成第 7.6 节的 Linux
原始 uCAN 上板测试；仍需同一 Windows 软件与驱动 5.1.3 重跑停更场景，不能把 Linux
结果当作 Windows 端到端验收。[用户反馈 + 验证边界]
其他待验项：驱动对字符串/hw_type 的核验。
若拒认，调整集中在 `pcan_protocol.hpp` 的 `kHwType/kFwVersion` 与
`pcan_descriptors.cpp` 的字符串常量。

### 7.4 双路与关停回归（2026-10-15）

双 CAN 板两口 H/H、L/L 互接回环，Linux `peak_usb` 下使用接口 0 的 `can0/can1`；
接口 1 产生的重复 `can2/can3` 保持 DOWN。测试程序使用 Python 原生 SocketCAN，
不写入仓库。[实测]

| 检查 | 结果 |
|---|---|
| 经典 CAN，1 Mbit/s，两路同时互发 | 每方向 1000 帧，8 字节负载，序号和内容逐帧一致 |
| CAN FD，仲裁 1 Mbit/s、数据 5 Mbit/s、BRS，两路同时互发 | 每方向 1000 帧，64 字节负载，序号和内容逐帧一致 |
| 交替单独关闭、重新打开一路 | 经典 / FD 各 10 轮；关闭一口时另一口保持 UP；每轮恢复后每方向 10 帧校验通过 |
| 关停一口、从另一口提交一帧，再关闭剩余口 | 静置 6 秒后两口保持 STOPPED；重新开启后每方向 100 帧校验通过 |
| 真实 EP0 读取与拒绝恢复 | INFO_FW / INFO_BL 请求 4 / 28 / 36 / 64 / 1024 字节，返回长度均为结构与请求长度较小值；未知请求 STALL 后正常读取可恢复 |
| 关闭后的两颗 CAN 灯 | 测试结束两口 DOWN，静置超过 6 秒，用户确认均已熄灭，未复现持续闪烁 |

前述数据与启停检查是 Linux 上板结果，灯光是用户现场观察；不替代 Windows 应用关闭
时的 USB 命令序列验证。无应答场景的主机错误计数仍显示 0，未取得物理发送或板端错误
记录，故不把它计作故障上报验证通过。测试结束全部 SocketCAN 接口均为 DOWN。[实测边界]

### 7.5 错误状态修复与意优 RP 读取排查（2026-10-16）

**本节保留当时的排查结果；后续读通与持续读取结果见第 7.6 节。**当时错误状态上报
已上板修复，ID=1 的意优 RP 电机仍未读通。两项结论独立：能看见
错误不代表总线通信已经成功，也不能据此断言电机或 Windows 驱动有问题。[实测]

- 修复前 USB 抓包确认：仲裁 1 Mbit/s、数据 5 Mbit/s，扩展帧读取请求已到 bulk OUT，
  标志 `0x32`（EXT_ID + FD + BRS）；IN 只有 CALIBRATION 与 STATUS，没有 CAN_RX。
  三轮 STATUS 的通道/状态字节均为 `0x03`，对应代码把内部故障位直接拼进了通道号。
  临时解绑 DFU 接口上的重复 `peak_usb` 实例后仍未收到电机回复。[USB 抓包 + 代码]
- 修复包括：状态位转换；时序非法或应用失败时保持离线并关闭收发闸，后续有效配置
  可恢复；nominal seg1=256 的解码不再回绕。纯逻辑与编解码的 8 项回归测试在
  `host/tests/pcan_test.cpp`，不依赖硬件；PCAN 与默认 HPM5321 镜像均构建通过。[主机测试]
- 修正版经 DFU 刷入后，在物理 CAN1（本次枚举为 `can0`）对产品 ID=1、设备 ID=1
  读取设备 ID、型号、通信参数和软件版本；仲裁/数据采样点依次为 75%/75%、80%/75%、
  87.5%/87.5%，每组 8 帧，读取负载长度 0 或 8。未收到应用回复，但主机已能显示
  ERROR-PASSIVE，并曾收到 TEC=128、REC=0 的错误上报，证实故障可见性修复生效。
  测试未发送使能、运动、零位或参数写入，结束后通道 DOWN。[Linux 上板实测]
- 追加在 75%/75% 下比较 FD+BRS、FD 不切速、经典数据帧三种只读请求，每种对
  产品 ID=1/0、设备 ID=1 读厂家信息整组、型号和设备 ID（共 18 帧），仍无应用回复。
  不能以这些请求的 USB 发送成功认定线上已成功发送或得到 ACK。[Linux 上板实测]

意优 RP 手册总述写采样点 75%，通信参数表却写仲裁默认 80%、数据默认 75%；本次两组
都试过，不能把二者差异直接当作唯一原因。手册为用户提供的
`意优RP系列行星关节CAN-FD通信协议.pdf`（V1.0，2026-03-30）；寄存器读取使用扩展
ID，设备地址 1 不等于线上查询 ID=1。[用户手册 + 实测边界]

全量 host 验证另有现存阻塞：`spsc_ring_test`、`stream_writer_test` 引用旧接口而编译
失败；排除这两组后 203 项中 202 项通过，
`EngineLifecycle.StopQuiescesAnAdmittedReceiveBeforeRestart` 失败。本次未修改上述
主机实现与用例。全仓库 clang-format-check 仍有其他文件的格式差异；本次 C++ 修改已
用 clang-format 20.1.8 单独检查通过。[本次验证]

### 7.6 发送确认、积压恢复与持续读取（2026-10-16）

**修正版已刷入实板；CAN1 的意优 RP ID=1 连续 2048 次只读查询全部得到电机回复和
匹配的发送确认，时间戳持续递增；积压后可经 TX_ABORT → RX_BARRIER 恢复，无需拔插
USB。Windows 意优软件的原现场问题仍待用户复测。**[Linux 原始 uCAN 上板实测]

旧版对 SELF_RECEIVE 请求未回传确认，且 TX_ABORT 空转；主机若等待确认释放发送额度，
就可能在有限次发送后停住，重开后的取消流程也无法清理旧请求。这与用户症状相符，
但尚无 Windows 抓包证明其唯一根因。[代码事实 + 推断]

本次补齐硬件 TX 完成事件、tag/client 透传、有限确认槽与数据 OUT 背压、IN 暂存重试，
以及 TX_ABORT 的通道重建。仅 PCAN 人格启用 TX Event FIFO 中断；新增 ISR 调用驻留
ILM 并有链接断言。协议字段与取消顺序参考 PEAK 官方 Linux 驱动 9.3.1 的
`driver/src/pcanfd_core_user.c`（`canfd_encode_txmsg_flags`、`canfd_handle_rxmsg_flags`、
`canfd_reset_path`），只读作规范，实现自研。[代码；
[官方驱动下载](https://www.peak-system.com/fileadmin/media/linux/index.php)]

台架为物理 CAN1，原始 USB 协议直接访问接口 0，临时解绑重复的 DFU/peak_usb 实例。
主机声明 80 MHz、仲裁 1 Mbit/s、数据 5 Mbit/s、ISO FD、两段采样点 75%；扩展查询
ID=`0x01020101`，8 字节零负载，只读取设备 ID，未发送使能、运动、零位或参数写入。
[实测条件；查询格式见用户提供的意优 RP 协议]

| 检查 | 结果 |
|---|---|
| 主机 PCAN 专项回归 | 13/13 通过，覆盖状态位、时序拒绝/恢复、长帧 tag/client、额度耗尽与取消，以及 10000 次槽位循环 |
| 构建与格式 | PCAN、默认 HPM5321 镜像构建通过；本次 C++ 文件通过 clang-format 20.1.8；全量既有阻塞见第 7.5 节 |
| FD+BRS / FD 不切速 / 经典发送确认 | 每组 8/8 硬件完成回显，64 位 tag 与 client 保持一致；FD 两组可见电机回复 |
| 单请求等待回复，再发下一请求 | 2048 请求、2048 匹配回显、2048 电机回复；回复负载均为 `01 00 00 00` |
| 数据是否停更 | 2048 个不同且递增的回复时间戳，首末为 3924674 / 9054968 µs，持续约 5.13 s |
| 暂停读取 IN，继续发送 | 提交含 144 请求的 USB 包后遇到 OUT 背压；此时 EP0 仍可读取 |
| 不拔插恢复 | TX_ABORT(FLUSH=1) → RX_BARRIER，收到屏障记录，随后新带 tag 请求得到确认 |

前期以每包 16 请求突发，共 2048 请求时，发送确认全部匹配，但只有 1625 个匹配查询
ID 的电机回复；不能将其算作突发收发零丢失通过。验收按手册的一问一答模式进行，
得到上表 2048/2048 结果。该测试约 5 秒，不替代 Windows 长时间运行验证。测试程序
与原始日志留在本机 `/tmp`，不进仓库；结束后恢复 Linux 驱动，全部 CAN 接口 DOWN。
[实测边界]

### 7.7 意优安装包、Windows 驱动与校准字段复核（2026-10-16）

**用户明确反馈第 7.6 节版本在 Windows 仍严重停更，上一轮未解决现场问题。**本轮
确认校准帧号恒 0 是 Windows 协议兼容缺口，已改成硬件帧号并刷入实板；尚无 Windows
失败时的 USB 抓包，不能断言这是唯一根因，也没有证据支持盲目更换驱动或固件版本号。
[用户反馈 + 代码事实 + 验证边界]

用户提供 `EYouPlanet(RP Series)_Installer_V260904(1).exe`，SHA-256 为
`6bd458e6550b3132ff47281c8002baa43a8787a602eec3587bd5e79be64da54b`。只解包与检查
PE 信息，未修改其二进制。临时解包目录为 `/tmp/libhcs-eyou-inspect/`。[文件实查]

| 层次 | 核对结果 |
|---|---|
| 意优应用与 `eu_candrv.dll` | 均为 x86 / 32 位；PCAN 路径调用 `CAN_InitializeFD`、`CAN_ReadFD`、`CAN_WriteFD`、`CAN_Uninitialize`；没有导入 `CAN_Reset` 或 `CAN_SetValue` |
| 随包 `PCANBasic.dll` | 文件版本 **4.5.4.508**，x86；SHA-256=`2f1174891d6dec27c86c170f579f2e6868685bec3c1d9b440e29735581ee9222` |
| 应用的 1M/5M 参数 | `f_clock_mhz=80,nom_brp=2,nom_tseg1=31,nom_tseg2=8,nom_sjw=4,data_brp=1,data_tseg1=11,data_tseg2=4,data_sjw=4`；对应采样点 80% / 75% |
| 官方 Windows USB 驱动 | 从 PEAK 官方驱动包解出 x64 `PCAN_USB.SYS`，文件版本 **5.1.3.20113**；这是本轮分析的参考文件，尚未读取用户 Windows 上实际加载文件 |
| 板端版本身份 | 回报 **3.5.4** 是协议兼容身份，不能据此认为具备原厂固件全部特性；早期 Code 10 同时涉及多处修正，版本号单独作用未证明 |

Windows 驱动接收校准记录时，会取 offset 12 的 USB 帧号，随后以主机帧号与设备帧号
的差（掩码 `0x7ff`）校正时间映射。可复核位置为该 x64 文件 RVA `0x3cc02`、
`0x5c188`、`0x5c290` 至 `0x5c343`。旧固件的恒 0 会制造错误的帧号差；Linux 内核
`peak_usb` 的原验收没有覆盖 Windows 这条路径。此处只读参考二进制行为，实现自研。
[官方驱动二进制检查；实际停更的因果关系仍待 Windows 复测]

修正后在主循环每次生成校准记录时，用两次 USB `FRINDEX` 读数夹住板钟采样，确保
时间戳和 USB 毫秒帧号属于同一帧；不启用 SOF 中断，也不改变默认镜像。校准记录
携带帧号的编解码回归加入 `pcan_test`，覆盖 2047 → 0 回绕与完整 64 位时间戳。
[代码 + 主机测试]

按上表意优原始时序，物理 CAN1 只读设备 ID=1 的上板结果：[实测]

| 检查 | 结果 |
|---|---|
| 持续读取 | 4096/4096 匹配回显、4096/4096 电机回复，4096 个不同且递增的时间戳，持续约 43.26 s |
| USB 校准 | 收到 432 条记录，432 个不同帧号；相邻帧号差换算时间与校准时间差的最大误差 1060 µs，跨多轮 2048 ms 回绕 |
| 积压恢复 | OUT 背压时 EP0 活跃；TX_ABORT → RX_BARRIER 后，新请求同时取得匹配回显与电机回复，无 USB 拔插 |
| 主机回归与构建 | PCAN 专项 14/14 通过，PCAN 固件构建通过；改动文件格式通过，全仓库已有格式阻塞仍在 |

额外准备仓库外的 Windows x86 诊断程序，动态加载用户选择的意优原版
`PCANBasic.dll`；记录实际 DLL 路径/版本、API/通道版本、ReadFD/WriteFD/GetStatus
错误码和回复时间戳。三阶段各 15 秒：只读查询 → CAN_Reset 后查询 → 关闭重开后查询。
只发送寄存器设备 ID 读取，不发送使能、运动或参数写入。程序已交叉编译并核对 ABI 与
依赖，尚未在 Windows 运行；它用于取得现场错误，不算 Windows 验收通过。
[交付物与验证边界]

外部依据：[PEAK 官方驱动包](https://www.peak-system.com/quick/DrvSetup)、
[官方驱动版本表](https://www.peak-system.com/support/hardware-information/driver-information/)。

### 7.8 Windows 首份诊断日志：通道初始化被拒绝（2026-10-16）

**本次 Windows 测试没有进入收发阶段：固定句柄 `0x51` 在 `CAN_InitializeFD` 返回
`0x1400`（`PCAN_ERROR_ILLHW`）。**这证明当时该硬件通道不可用，不能据此判断总线
波特率、电机应答或此前校准修正的效果。[用户提供的原始日志]

日志确认实际加载路径为 `C:\Program Files (x86)\EYouPlanet(RP Series)\PCANBasic.dll`，
进程 32 位、DLL 4.5.4.508，驱动版本查询返回 `PCAN_USB 5.1.3.20113`（KMDF 1.15，
x64）。同一次运行中硬件名称查询也返回 `0x1400`。驱动版本可查询不等于硬件通道已
注册可用；尚不能区分句柄变化、设备启动失败或连接状态变化。[用户日志 + 验证边界]

首版诊断将物理 CAN1 固定等同于 `PCAN_USBBUS1=0x51`，未枚举已连接设备，诊断覆盖
不足。v2 改为先查询 `PCAN_ATTACHED_CHANNELS_COUNT/PCAN_ATTACHED_CHANNELS`，记录
句柄、controller、FD 特性、设备名称与通道条件；额外只读扫描 16 个 USB 句柄。
优先选可用的 USB FD controller 0，多个候选时由用户选择，避免误连其他适配器；没有
候选则保存日志并停止，不继续发送 CAN 请求。[工具修正]

v2 同时经 Windows SetupAPI/Configuration Manager 记录当前在场的 `VID_0C72` 与
`VID_A511` 设备实例、绑定服务、启动标志与问题码，用于区分 PCAN 通道编号和 PnP
启动故障；不自动重装驱动或重启设备。工具仍在仓库外，交叉编译通过（x86，C11，
`-Wall -Wextra -Werror`），尚待用户 Windows 实际运行。固件本轮未修改或重刷。
[交付物与验证边界]

错误定义与枚举方式见 [PEAK 官方 GetValue 说明](https://www.peak-system.com/documentation/API/PCAN-Basic.Net/html/f6856925-2109-619a-6b68-8b751ed1c216.htm)。

### 7.9 Windows v2 日志：已初始化、137 次查询无接收（2026-10-16）

**本轮日志确认 `0x51` 实际可用且初始化成功；第一阶段 137 次查询全部超时，未收到
任何 CAN 记录。**因此上轮的 `0x1400` 不能继续当作当前故障，也不能据此认定 CAN1
固定映射在这台 Windows 上有误。API 日志仍不足以定位 USB 命令、数据 OUT、CAN 线上
发送或数据 IN 的哪一段失效，需要 Windows USB 传输抓包。[用户日志 + 验证边界]

用户提供的 `/home/zyi/Desktop/error.md`，时间区间 `1083368281` 至 `1083383953`
（Windows 单调毫秒计数）：[原始日志]

| 证据 | 结果与边界 |
|---|---|
| CAN 接口 `MI_00` | 绑定 `PCAN_USB`，`started=1, problem=0`；父复合设备也已启动 |
| DFU 接口 `MI_01` | `started=0, problem=28`，描述为 `DFU Runtime`；这是独立接口未安装驱动，不能当作 CAN 接口启动失败 |
| 通道枚举 | 两路：`0x51/controller=0`、`0x52/controller=1`，FD 能力位存在，`condition=1` 可用 |
| API 与驱动 | x86 DLL 4.5.4.508；PCAN_USB 5.1.3.20113 / KMDF 1.15 / x64 |
| 初始化 | `CAN_InitializeFD(0x51)` 返回 0 |
| 第一阶段 | 137 次 `CAN_WriteFD` 均返回 0，137 次读取等待均超时；没有 `RX` 记录；13 次 `CAN_GetStatus` 返回 0 |
| 重置 | `CAN_Reset` 返回 0；文件在第二阶段起始时间戳处截断，无法证明之后恢复或卡死 |

`CAN_WriteFD` 成功只表示请求被 API 接受进入发送路径，不证明设备收到 USB OUT、
MCAN 发送成功或对端 ACK。`GetStatus=0` 同样不能替代板端原始 STATUS/ERROR 报文。
不根据这份日志改波特率、重装驱动或再次猜测固件版本。[接口语义 + 验证纪律]

下一步采集同一次 Windows 会话的 USB 命令 OUT `0x01`、数据 OUT `0x02/0x03`、
记录 IN `0x82` 及 EP0，先核对时序/ISO/上线命令，再核对查询实际负载与返回记录。
可单独安装 USBPcap，不要求安装 Wireshark；安装抓包驱动后需重启，再从重新连接前
开始记录枚举、初始化及故障前后传输。固件本轮未修改或重刷。[待采证]

外部依据：[Microsoft Code 28 说明](https://learn.microsoft.com/en-us/windows-hardware/drivers/install/cm-prob-failed-install)、
[USBPcap 官方采集步骤](https://desowin.org/usbpcap/tour.html)。

### 7.10 本机原生 Windows 复现与官方驱动边界（2026-10-16）

**停更已在本机原生 Windows + 实体 USB 板 + ID1 电机上复现；升级至驱动
5.1.3.20113 后仍复现，不能继续将 Linux 压测通过当作修复完成。**Windows 在停更后
不再提交 CAN 数据 OUT，但板子的校准 IN 仍继续，USB 与板端主循环没有整体卡死。
[本机 Windows + USBmon 实测]

测试用 QEMU/KVM 启动已有 Windows 的写时复制覆盖层，物理 Windows 分区作为只读
后端；USB 实体板直通。意优安装目录里的 x86 PCANBasic.dll 为 4.5.4.508，先测原驱动
5.0.0.20000，再安装官方签名 5.1.3.20113 并重启，API 再次查询确认实际版本。
仅发送 `0x01020101`、8 字节零的 FD+BRS 设备 ID 查询，不使能、不运动、不写参数。
[测试环境与安全边界]

| Windows 路径 | 第一阶段有效回复 | Reset 后 | Uninitialize/Initialize 后 |
|---|---:|---:|---:|
| 官方 DLL + 驱动 5.0 | 98 后停止 | 未恢复 | 未恢复 |
| 官方 DLL + 驱动 5.1.3 | 287 后停止 | 0 | 0 |

驱动 5.1.3 第一阶段提交 369 次 API 写请求，287 个回复有递增时间戳；第二、三阶段
各提交 138 次 API 写请求，均无回复。`CAN_WriteFD=0` 不能证明产生 USB OUT。
不同重插会话停在不同包数，不存在一个可通过补单个固定计数解决的板端阈值。
[同一次会话 API 日志 + USBmon]

只读检查官方驱动发现与设备验证关联的有限通信额度状态，数据收发经过该状态检查；
CAN_Reset 不重建设备启动状态。这与有限包数停更、仅重插恢复一致。[静态分析 + 实测对应]
当时未实现该初始化应答，选择了自研 DLL 与 Windows 自带 WinUSB；此处是历史方案，
用户后续要求保留原版软件，固件初始化修复与新版验收见 7.13。官方驱动始终未修改。
PEAK 也明确说明官方驱动/API 的硬件适用范围，见
[官方说明](https://www.peak-system.com/fr/know-how/blog/notice-about-counterfeit-products/)。

此外修复了独立存在的启动校准问题：主机启用 calibration 时，即使两路 CAN 尚在
reset 也要发校准；USB 时钟没有随 CAN reset 停止。旧固件 0.55 秒内 0 条，修正后
6 条校准且帧号推进。该修正改善初始化，不解决上述官方驱动停更。[Linux 原始 USB 实测]

### 7.11 独立 WinUSB 与兼容 DLL（2026-10-16）

**Windows 原生 x86 诊断已在新路径读到 ID1；Reset、关闭重连都继续收发，无需拔插
USB。**这是独立实现的 CAN-FD API 子集，不是对官方驱动做补丁。用户已确认接受配套
兼容 DLL。新增独立固件目标，不改变原始人格与默认 libhcs 镜像。[授权 + 上板实测]

新身份为 `A511:F531/F532`，产品串为 HCS CAN FD Bridge，MS OS 2.0 描述符包含
CAN 接口与 DFU runtime 的 WINUSB CompatibleID 及不同的 DeviceInterfaceGUIDs。
Windows 实际枚举两接口为 `WINUSB, started=1, problem=0`，无需手工安装 INF。
DLL 只按专属 GUID 与新 VID/PID 查找设备。[实现 + Windows PnP 实测]

CAN-FD 初始化、收发、Reset、关闭重连、状态与通道查询由自研 C11 DLL 实现，使用
WinUSB 读写原有 uCAN 转发端点；错误、回包与硬件时间戳来自板子。导出 14 个旧 ABI
名字与序号供意优链接，其中未实现 API 明确返回不支持。具体范围见
[兼容层说明](../../host/compat/pcan_basic/README.md)。不承诺完整 PCAN-Basic 或 PCAN-View。

首轮三阶段各 15 秒，DLL 为新构建的 x86 版本，速率为仲裁 1M、数据 5M，查询同 7.10：
[本机 Windows 实测]

| 阶段 | API 写成功 | 收到 ID1 | 递增时间戳 | 超过 100 ms 等待 |
|---|---:|---:|---:|---:|
| 初次连接 | 422 | 422 | 422 | 2 |
| CAN_Reset 后 | 129 | 129 | 129 | 30 |
| 关闭重连后 | 164 | 164 | 164 | 30 |

表中的等待超时与迟到回复可同时存在，所以不能用 `回复数 == 写成功数` 断言无延迟。
首轮同时遇到 Windows 启动程序负载；迟到原因尚不能只归因虚拟机。[验证边界]

第二轮第一阶段连续 120 秒，随后 Reset 15 秒、关闭重连 15 秒，全程未拔插 USB：
[本机 Windows 实测]

| 阶段 | API 写成功 | 收到 ID1 | 递增时间戳 | 超过 100 ms 等待 |
|---|---:|---:|---:|---:|
| 连续 120 秒 | 3694 | 3693 | 3693 | 38 |
| Reset 后 15 秒 | 477 | 477 | 477 | 0 |
| 关闭重连后 15 秒 | 477 | 477 | 477 | 0 |

总计 4647 个递增时间戳的 ID1 回复；第一阶段存在迟到及 1 条未计入的回复，不能宣称
全程零丢包。后两个阶段完整对应、零超时，已越过原官方驱动约百包至几百包即永久停止
的场景。此测试证明本次台架上的持续读取与恢复，不是所有速率/负载的性能上限验收。
[原始日志 `pcan-diagnose-20261016-124946.log`]

意优 V260904 GUI 的第一段持续 168.063 秒：10641 次周期发送、10641 条 ID1 状态
回复，10641 个硬件时间戳全部递增，负载有 123 种变化。随后在界面断开、关闭设备、
重新打开 PCAN/1M/5M，再连接 ID1，18.149 秒内 1140 发、1140 收，时间戳继续推进。
GUI 显示 ID1、软件 2.0.12、硬件 1.0.0、电压 24V、MOS 36℃、线圈 33℃、错误码 0。
[实体电机 + 原生意优 GUI + USBmon 实测]

为排除“USB 收到而界面只看旧缓存”，只读采样了**本项目 DLL** 在意优进程中的队列：
消费头每秒从 893→958→1025→1090，积压为 0/1/0/0，status=0；这证明 GUI 持续调用
ReadFD 消费新帧，不只是后台 USB 接收线程在工作。[本项目 DLL 只读内存诊断]

GUI 连接后会自动周期发送 `0x80`、64 字节的 MIT 中位帧，每 8 字节均为
`7F FF 7F F0 00 00 07 FF`。协议手册定义它为零位置/速度/扭矩及零 Kp/Kd。
本次没有点击使能、执行、参数写入、更新零点或修改 ID，GUI 使能状态始终为 false。
GUI 并非纯 ID 查询器，故不能把原生诊断工具的“仅设备 ID 读取”描述套用于 GUI。
[手册 4.1.2.1 + USB 抓包 + 操作记录]

意优整个进程退出后重新启动，再选择 PCAN/1M/5M 连接 ID1，也恢复状态读取，
期间 USB 未拔插。安装脚本在 Windows 临时测试目录实际完成安装、保留原 DLL 备份、
重复安装不覆盖备份及恢复原文件哈希的验证。真实 Windows 系统盘保持只读，DLL 替换与
安装测试都发生在写时复制覆盖层；实际 Windows 用户仍需运行配套安装脚本。
[原生 Windows 实测；`installer-result.txt`]

构建验证：新固件与默认固件编译通过，默认 DFU 的 SHA256 与新增 WinUSB 前完全一致；
15 项 PCAN 主机测试通过（含 MS OS 2.0 子集长度、注册表字符串与接口边界校验）。
修改的 13 份 C/C++ 文件格式检查通过；全仓检查仍有无关既有格式问题，不能声称全仓门禁通过。
[本机构建与测试]

### 7.12 原版 DLL/驱动的发布目标与剩余障碍（2026-10-16，历史阶段）

本节记录修复前的核查结论；设备初始化交换现已实现，本次台架验收见 7.13。

**用户进一步明确：必须保留原版 PCAN DLL/驱动，让现有 PCAN 软件直接使用。
第 7.11 节的 WinUSB 配套方案不满足这一目标；不能把意优 GUI 已通过的结果扩展成
原版驱动已兼容，或宣称所有 PCAN 软件可用。**此前交付的 WinUSB 固件本轮未修改。
本轮只读核查，没有重刷实体板；核查末尾本机 USB 列表未枚举到该板，未新增上板验收。[用户要求 + 当前验证边界]

重新核对同一份 Windows 5.1.3 USBmon 抓包与旧固件：[实测抓包 + 源码]

- 驱动读取设备信息时请求 84 字节，旧固件的 `FwInfo::encode()` 返回 36 字节。
  短包本身符合 USB 传输规则，不能单凭长度不同判为故障，也不能盲补零到 84 字节。
- 一条 vendor OUT 为 `bRequest=2, wValue=0xF8, wLength=36`，完成状态为
  `-EPIPE`；旧固件只接受 `kFctDrvld=5`，因而拒绝它。
- 启动 bulk 命令流还出现旧固件未实现的专有记录，含 `0xA0` 与 `0xD0`。
  这些传输不全是固定 8 字节 uCAN 命令；不能把后续负载按 8 字节拆分后当作一串
  独立未知操作码。上述数值仅用于描述抓包事实，不作为实现规范。

只读驱动分析确认：设备启动会执行独立的请求/响应验证并保存结果；收发路径检查
该结果，对部分未通过状态维护有限通信计数，计数阈值在设备启动时生成，CAN_Reset
不重新建立该验证状态。请求数据包含生成值及校验处理，不能仅把未实现请求改成
ACK 或固定成功应答，就声称完成了设备验证。[静态分析；与 7.10 实测现象一致]

因此，尚不能提供“纯 5321 固件 + 任意原版 PCAN 驱动/API”的发布承诺。普通 CAN/USB
转发缺陷可以独立修复，但现有资料与实测未证明已满足官方设备验证。即使某个版本
以后通过，也必须单独列出驱动、DLL、应用、位数及持续/恢复测试范围，不能外推到
全部软件与未来版本。[工程验收边界]

PEAK 官方说明驱动、软件和 API 面向其自身硬件；官方另有支持 CAN-FD 的
PCAN-Chip USB OEM 模块，明确提供与其 USB 驱动和软件的兼容性。这是需要正式原版
软件支持时可核实的硬件集成方向，不表示本板已采用该模块，也没有实施购买或改板。
[厂商资料]

来源：[官方硬件适用范围](https://www.peak-system.com/fr/know-how/blog/notice-about-counterfeit-products/)、
[PCAN-Chip USB & Eval](https://www.peak-system.com/de/produkte/hardware/chiploesungen/pcan-chip-usb-eval/)。


### 7.13 原版驱动初始化交换修复（2026-10-16）

**固件补齐设备初始化交换后，原版 DLL 4.5.4.508 + 官方驱动 5.1.3.20113 在同一块
HPM5321、CAN1 的 ID1 电机上通过连续读取、Reset 和关闭重连。**前述“几百包后停更”
并非单靠升级驱动解决；此前固件没有提供驱动设备启动时需要的完整交换。
[原生 Windows API + 实体板 USBmon 实测]

实现位于 `pcan_device_check.hpp` 与 `pcan_adapter.cpp`，仅编入 PCAN 人格：

- 将变长 `0xA0` / `0xD0` 设备记录与 8 字节 CAN 配置命令分开解析，防止负载误配置 CAN。
- 保存当次挑战，独立实现标准 AES-128 计算响应，不回放历史会话响应。
- 在命令 IN `0x81` 返回完整 `0xD1` 设备记录，72 字节，17 字节有效负载；主机校验其
  64 字节前缀。此封装不同于数据 IN `0x82` 的 size/type 包头。
- 命令 IN 有独立 DMA 缓冲、提交/完成与失败重试；两路 CAN 都在 reset 时也能应答。
  USB reset 清理挑战与待发状态，CAN_Reset 不破坏设备初始化结果。
- 不修改原版 DLL、内核驱动、签名或主机校验分支，不降低固件版本以跳过检查。
  EP0 `wValue=0xF8` 仍拒绝未实现请求，没有为消除日志而盲目 ACK。
[源代码 + 原始 USB 记录]

首轮候选实现虽有正确的计算结果，却误用数据记录包头并只返回 32 字节，真实驱动在
读取封装层拒绝后仍于 491 次回复后停更。最初离线验证把 USB 读取层替换为直接复制，
未覆盖该拒绝条件；因此离线结果不能代替真实 Windows 长测。修正为完整命令记录后，
实际 USB 命令 IN 为 72 字节，后续有效通信超过原有随机额度。[反例与修复实测]

正式原版 API 三阶段测试如下，全程未拔插 USB；仅发送 FD+BRS 设备 ID 查询，
速率为仲裁 1M / 数据 5M，x86 进程、64 位 Windows：
[原始日志 `pcan-diagnose-20261016-151104.log`]

| 阶段 | API 写成功 | 有效 ID1 回复 | 递增时间戳 | 100 ms 超时 | API 错误 |
|---|---:|---:|---:|---:|---:|
| 连续 120 秒 | 5518 | 5518 | 5518 | 0 | 0 |
| CAN_Reset 后 15 秒 | 663 | 663 | 663 | 0 | 0 |
| 关闭重连后 15 秒 | 708 | 708 | 708 | 0 | 0 |

总计 6889 次读请求与 6889 次有效回复；Reset、重新 InitializeFD 均返回成功。
USB 抓包也对应 6889 次数据 OUT。此结果证明本次驱动/DLL/台架组合消除了原故障，
不能据此承诺未来驱动、全部 PCAN API、其他速率或单 CAN PCB 已经过硬件验收。
[API 日志与 USBmon 交叉核对]

意优 V260904 原生 GUI 首段连续 127.418 秒，周期中位帧发送 22010 次、ID1 状态
回复 22102 条，22102 个时间戳全部递增，负载有 111 种变化；曲线横轴从启动时推进
到超过 120 秒，速度读数与曲线持续变化。界面显示连接 true、使能 false、24V、
MOS 37℃、线圈 34℃、错误码 0。[意优 GUI 截图 + USBmon]

断开电机连接、关闭设备并退出整个软件，再启动原 EXE、选择 PCAN/1M/5M 连接 ID1，
39.512 秒内周期中位帧发送 15034 次、状态回复 15045 条，全部时间戳递增，负载有
102 种变化。期间 USB 未拔插。GUI 还读取设备信息/参数，表中周期发送和状态回复并非
一一对应，不能由二者差额计算丢包率。周期帧负载与 7.11 相同，为零 Kp/Kd 和零输出的
中位帧；没有点击使能、执行、写入、零点或修改 ID。[GUI + USBmon 实测]

测试继续使用物理 Windows 盘的只读后端与临时写时复制覆盖层；原版 DLL 已在测试
覆盖层恢复，真实 Windows 安装未修改。此前若安装过本项目 WinUSB 兼容 DLL，用户切回
原版路径时仍需恢复安装目录中的原 DLL。板子本次保持原 PEAK 身份修复固件。
[测试环境与交付边界]

验证还包括 FIPS 197 公开向量、设备记录固定向量、乱序/截断/重置/新挑战用例，共
18 项 PCAN 测试通过。相关固件编译通过；默认 libhcs DFU 与本轮改动前 SHA256 完全一致。
修改的 C/C++ 文件格式检查通过；全仓格式检查仍有不相关既有错误，不能标为全仓通过。
[本机构建与测试]
