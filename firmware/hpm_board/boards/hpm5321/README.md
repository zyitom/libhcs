# HCS Slave HPM5321

> **文档类型**：硬件参考（板级说明）
> **适用范围**：`firmware/hpm_board/boards/hpm5321/`，**两块 HPM5321 板共用一份镜像**（单 CAN 版 + 双 CAN-FD 版）
> **状态**：现行有效
> **相关文档**：[PINOUT.md](PINOUT.md)（引脚分配） · [../../AGENTS.md](../../AGENTS.md) · [../../BUILD_ENVIRONMENT.md](../../BUILD_ENVIRONMENT.md)

## 摘要

最精简的 USB <-> 现场总线转发桥：**一路 CAN、一路数据 UART、一颗普通 GPIO 驱动的 RGB
状态灯**。没有 IMU、没有 DBUS 接收机、没有 GPIO 应用通道。

**这个目录现在同时服务两块板**：单 CAN 版（只有 MCAN0，RGB 灯在 PA29/30/31，无 CAN 指示
灯）和双 CAN 版（MCAN0 + MCAN3，RGB 灯在 PA26/27/28，两颗 CAN 指示灯在 PB14/PB15）。
两块板的 CAN 控制器**都跑 CAN-FD**，变体只决定启用几路。
`-DBOARD=hpm5321` 出的**同一个 `.dfu` 两块板都能烧**，固件上电时自己判断跑在哪块板上
（见下节）。这是 HPM5321 唯一的固件构建入口。

硬件引脚分配见 [PINOUT.md](PINOUT.md)，其中同时列出单 CAN 和双 CAN-FD 两套复用关系。

## 板型自识别：OTP 第 25 个字 [实测 2026-08-05，4 颗芯片]

两块板的 `.yaml` **逐字节相同**，`board.h` 只差一个 `BOARD_NAME`，PCB 上也没有任何
可读的板型标志位——没有空闲 GPIO 跳线、没有电阻编码、`DGO_GPR` 四个字里没有一个是空
的。唯一在出厂时就已经不同、且软件能读到的东西是 **OTP 影子寄存器第 25 个字**
（`0xF3050064`）：

| 板型 | OTP word 25 | 样本 |
|---|---|---|
| 单 CAN 版 | `0x00000000` | 2 颗（lot 378、380） |
| 双 CAN-FD 版 | `0x00000002` | 2 颗（lot 375、377） |
| 其他任何值 | — | **拒绝启动**，见下 |

固件（`common/board_identity.hpp`）在 bootloader 和 app 里各读一次这个字，据此选择
CAN 端口表、CAN 模式、LED 引脚和 USB PID。**上电只读一次**并缓存；读 OTP 是一条普通
的 MMIO load，不消耗写次数、不损伤熔丝阵列（编程走的是完全另一条路径：先开 2.5 V
LDO、写 `UNLOCK` 魔数、再写 `FUSE[]`）。而且序列号本来就要读 4 个 OTP 字，这只是第 5 个。

### 已知局限：板型与批次在样本里是共线的

四颗样本恰好是**按批次订的**（双 CAN 来自 375/377，单 CAN 来自 378/380），所以
"word 25 = 2 表示双 CAN" 和 "word 25 = 2 表示某个批次" 这两个解释在现有数据上**无法
区分**。要打破这个共线，需要**一块 375-377 批次的单 CAN 板**或**一块 ≥378 批次的双
CAN 板**。现已确认 OTP 第 25 字是板型标志，固件只保留运行时识别这一条路径；其他值按下
节说明拒绝启动。

### 读到未知值：拒绝启动，并在 lsusb 里报错

如果 word 25 既不是 0 也不是 2，bootloader **不跳 app、也不接受任何 DFU 下载**，并以
PID `0x53FF`（与单 CAN `0x5321`、双 CAN `0x5322` 同属 53xx，但不是已知板号）枚举，产品字符串写成
`HCS DFU UNKNOWN BOARD OTP25=0x<八位十六进制>`。所以一根 USB 线 + `lsusb -v` 就能看到
是哪个值不对，不需要调试器。

> **这是故意做成砖的**：拒绝下载意味着不能用 DFU 修回来。恢复手段是复位时把 **PA07 拉
> 到 GND** 强制 bootloader 常驻（**这块板没有实体按键**，PA07 只是引出的焊盘，见
> [PINOUT.md](PINOUT.md)），或者直接上 J-Link。这是明确要求的行为——宁可停下来让人查，也不要在认不出的硬件上乱跑。

其他板（`hpm6e8y`）**不做这个检查**：它们的 word 25 读出来是 `0x6`
（见 [../hpm6e8y/README.md](../hpm6e8y/README.md)），只有本目录通过
`libhcs_BOARD_OTP_IDENTITY=1` 打开它。

## 标识信息

| 项目 | 取值 |
| ----------- | --------------------------------- |
| 板名 | `HCS_Slave_HPM5321`（`BOARD_NAME`） |
| 实际器件 | HPM5321IEG1（QFN48），单 RV32 核 |
| SDK SoC 模型 | HPM5361（SDK 当前兼容模型） |
| USB PID | `0x5321`（VID `0xA511`） |
| USB 速度 | High speed（480 Mbps） |
| Flash | 1 MiB XPI NOR，app 位于共享 DFU bootloader 之后 |

## 主机侧看到什么

这块板跑的是 hpm_board 的共享 USB 应用（`app/`）：一个承载 libhcs 字节流的 USB
vendor 类设备，会话握手（kStart nonce + keepalive 租约）由共享的 `link::HostSession`
处理。

主机看到的 PID 取决于**芯片在哪块板上**，不取决于编译：单 CAN 版报 `0x5321`，双 CAN-FD
版报 `0x5322`。host SDK、udev 规则、`dfu-util -d` 都按这两个 PID 匹配。

**单 CAN 版：**

| 主机侧端点 | 板上资源 | 说明 |
| ------------- | -------------- | ------------------------------ |
| `DataId::kCan1` | MCAN0（CAN-FD，1M 仲裁 / 5M 数据） | DM（达妙）电机总线 |
| `DataId::kUart0` | UART2，921600-8N1 | 收发双向均由 DMA 驱动 |

**双 CAN-FD 版：**

| 主机侧端点 | 板上资源 | 说明 |
| ------------- | -------------- | ------------------------------ |
| `DataId::kCan1` | MCAN0（CAN-FD） | PA00/PA01 |
| `DataId::kCan2` | MCAN3（CAN-FD） | PA30/PA31 |
| `DataId::kUart0` | UART2，921600-8N1 | 同上 |

镜像里两路 CAN 的表和缓冲**都按 2 路的容量编译**（AHB SRAM 多占约 2.5 KiB），但在单 CAN
版上只有第 0 路被构造和启用。主机若给单 CAN 版发 `kCan2` 的帧，会被当作"本板不认识这个
field"拒掉，不会打到未初始化的对象上。

### 两个变体都跑 CAN-FD：帧格式是逐帧的，控制器模式不切换

**控制器在 init 时被配置成 CAN-FD 一次，之后永不重配。** 每一帧是 classic 还是 FD，
由那一帧帧头里的 FDF/BRS 位决定，两个方向都逐帧携带 `is_fdcan`：

| 方向 | 代码 | 含义 |
|---|---|---|
| 下行（主机 → 板） | `can.cpp:39-41`，`send_fd = canfd_ && data.is_fdcan` | 主机**请求**这帧用 FD 发 |
| 上行（板 → 主机） | `can.cpp:125`，`data.is_fdcan = rx.canfd_frame` | 硬件**报告**收到的是什么格式 |

FD 是经典 CAN 2.0 的**严格超集**：FD-enabled 的 M_CAN 同样收发 classic 帧。所以
"开 FD"不会让这块板收不到普通 CAN 帧——**没有"发来经典帧就切回经典模式"这个动作**，
INIT 模式重配、总线中断统统不存在。

#### 历史：单 CAN 版曾被配成仅经典模式 [2026-08-19 移除]

此前 `can_port()` 会在单 CAN 版上把 `.mode` 覆盖成 `kClassic`，理由只是"保持与出厂
固件逐字节一致"。这个配置是**单向亏**的——classic 不是超集，对端发来的 FD 帧一帧都
收不到。当时的实测（单 CAN 版与 mc02 的 FDCAN3 对接，mc02 常驻 FD+BRS）：

| 方向 | 格式 | 结果 |
|---|---|---|
| 本板 → mc02 | 主机请求 FD | **400/400 通**（实际降级成 classic 发出）|
| mc02 → 本板 | classic | **50/50 通** |
| mc02 → 本板 | **CAN-FD** | **0/50，全丢** |

`[实测 2026-08-05]`。**移除覆盖后这张表不再描述本仓库的行为**，但那个签名仍值得记住：
一个方向通、另一个方向零帧，看起来很像 [../../PITFALLS.md](../../PITFALLS.md) 第 3 节记的采样点
不一致签名，成因却完全不同（那是采样点，这是控制器不支持 FD 格式），且总线上不会有错误
计数。日后再遇到"classic 通、FD 单向不通、错误计数为 0"，先查**对端**的控制器模式。

## 构建与烧录

```bash
cmake --preset debug -S firmware/hpm_board -DBOARD=hpm5321
cmake --build firmware/hpm_board/build
```

构建产物是 `firmware/hpm_board/build/app/output/` 下的
`hpm_board_app_hpm5321.dfu`，**两块板都用这一个文件**。通过 USB DFU 烧录（**前提是芯片
上已经有共享的 HCS DFU bootloader**；首次烧 bootloader 需要调试器）：

```bash
# 单 CAN 版
dfu-util -d 0xa511:0x5321 -a 0 -D firmware/hpm_board/build/app/output/hpm_board_app_hpm5321.dfu
# 双 CAN-FD 版：同一个 .dfu，只有 -d 的 PID 不同
dfu-util -d 0xa511:0x5322 -a 0 -D firmware/hpm_board/build/app/output/hpm_board_app_hpm5321.dfu
```

`.dfu` 容器后缀里的 PID 写的是通配 `0xFFFF`（`dfu-suffix` 支持），所以同一个文件对两个
PID 都能通过校验；VID 和 CRC 仍然照常检查。**`-d` 后面的 PID 要按板子填**——它匹配的是
设备实际枚举出来的值，不是文件里的值。

正在运行的 app 会暴露 DFU 运行时接口，所以 `dfu-util` 能自动让它 detach 并重新枚举进
DFU 模式。若要强制 bootloader 常驻，复位时把 **PA07 拉到 GND**（与 JTAG_TMS 复用，
**没有实体按键**，见 [PINOUT.md](PINOUT.md)），或者在没有有效 app 的状态下上电。

> **要试有可能在 USB 枚举前就跑飞的 app 镜像，先烧 `-DBOARD_BOOTLOADER_MODE=manual`
> 的 bootloader。** 默认的 `auto` 只要看到一份校验合法的 app 就跳进去，坏 app 不枚举
> USB 就再没有 DFU 窗口；本板又没有实体键，PA07 的采样窗口只有复位后约 1 ms。两种模式
> 的完整语义见 [../hpm6e8y/README.md](../hpm6e8y/README.md#bootloader-常驻没有按键靠短接或-manual-模式)。

> app 与 bootloader 的 mailbox 版本不一致时，上面的自动 detach 会静默失效；此时按
> [../../AGENTS.md](../../AGENTS.md) 中的 bootloader 恢复流程处理。

## 状态灯语义

单颗 RGB 灯（单 CAN 版在 PA29/30/31，双 CAN-FD 版在 PA26/27/28，见
[PINOUT.md](PINOUT.md)），由共享的 Led 驱动
（`app/src/led/led.hpp`）驱动。黄色 = 红+绿，青色 = 绿+蓝；**优先级从上到下**：

| 灯效 | 含义 |
| --------------------- | --------------------------------------------------------- |
| 黄/青交替 | 双向都拥塞（上行和下行缓冲同时满） |
| 黄色闪烁（约 4 Hz） | 上行缓冲满（板 -> 主机）；主机没在取数据 |
| 青色闪烁（约 4 Hz） | 下行缓冲满（主机 -> 板）；CAN 发送积压 |
| 绿色常亮 | 主机会话已建立，正在转发数据 |
| 绿色慢闪 1 Hz | 存活，等待主机会话 |

"主机会话已建立"指的是 **libhcs 握手已完成**（kStart nonce 得到确认，且 keepalive
租约在 1 秒内被刷新），**不等于"USB 线插上了"**。设备枚举成功但主机侧没有应用程序在跑
时，灯会停在绿色慢闪。
