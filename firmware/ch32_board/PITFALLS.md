# ch32_board 踩坑记录

> **文档类型**：过程记录（实测踩坑档案）
> **适用范围**：`firmware/ch32_board/`，WCH CH32H417
> **状态**：现行有效。**改代码或上板之前必读。**
> **相关文档**：[AGENTS.md](AGENTS.md)（简版纪律） · [README.md](README.md)（设计背景） · [PROGRESS.md](PROGRESS.md)（工作日志）

## 摘要

本文件是 ch32_board 上**每一个真实踩过的坑**：现象、判据、原因、修法。2026-07-25 首次
上板 bring-up 起持续积累（07-26 补入 USB 数据通路打通期间的发现）。每条都是实测结论。

`AGENTS.md` 里那份是简版纪律，这里是完整来龙去脉——当板子行为和预期不符时，先来这里
按现象查，多数情况能省掉重新排查的几小时。

**标注约定**：**[RM]** = 已在参考手册 `~/Downloads/CH32H417RM.PDF`（`[前机路径]`）中找到
依据；**[EVT]** = 依据 EVT 例程/板级手册；**[原理图]** = 依据板卡原理图；
**[实测]** = 只有板上实验证据，手册未覆盖。

> **当前在用的板子（2026-08-05 起）**：**Petros CH32H417M Alef Breakout**
> （`petros_ch32h417m_alef_breakout.kicad_sch`，Rev 1V0，2026-04-21，XPU Labs &
> AnalogLamb），MCU 为 `U1 = CH32H417MEU6`。座子用 **J 编号**：`J1` = USB3.0-A、
> `J2` = 树莓派 Pico 40 脚兼容排针、`J3` = `DF12NB(3.0)-40DP-0.5V(51)` 板对板、
> `J4` = 1x06 排针（调试 + 串口复合）。
>
> **本文早期（2026-07-25/26）条目里的 `P1` / `P2` / `P10` 座子编号来自官方 EVB**
> （`CH32H417SCH.pdf`），与当前板子对不上，已就地更正为 J 编号并标注差异。**电气结论
> 本身不受影响**——两块板的 PB8/PB9 复用关系一致，实测结论继续有效。官方 EVB 的编号
> 作为历史记录保留在各条的括注里。

## 本文导航

| 章 | 主题 | 什么时候来看 |
|---|---|---|
| [1. 硬件层](#1-硬件层) | 引脚复用、USB3 座、VBUS | 调试器连不上、插线就出怪事 |
| [2. 工具链层](#2-工具链层) | 中断属性被丢、TinyUSB 不可用 | 换工具链之后、中断行为异常 |
| [3. 烧录层](#3-烧录层时间黑洞集中地) | 读保护、option byte、hex 分段、metadata | 烧不进去、烧进去不跑 |
| [4. 调试观测层](#4-调试观测层) | halt 会复位、什么值可信 | 读出来的值和预期对不上 |
| [5. 固件结构层](#5-固件结构层) | 初始化顺序、时钟树归属、双核分工 | 改初始化流程之前 |

**最容易浪费时间的三条**：[1.1（调试口被 USB2 抢走）](#11-pb8pb9-既是-usb2-的-dd也是-swclkswdio-实测--原理图)、
[3.10（插着 USB3 线就烧不进）](#310-usb3-线插着就烧不了与固件无关-实测)、
[4.7（调试器挂着时 V5F 根本不跑）](#47-调试器-attach-期间-v5f-根本不跑同会话的-reset-run-读到的全是残留-实测)。

---

## 1. 硬件层

### 1.1 PB8/PB9 既是 USB2 的 D+/D-，也是 SWCLK/SWDIO **[实测 + 原理图]**

**现象**：固件跑起来一两秒后调试器就连不上（`failed to connect with riscvchip`），
按 NRST 无效，只有断电才能恢复。

**原因**：原理图上这两个脚标的是 `PB8/USB2DP/SWCLK`、
`PB9/USB2DM/SWDIO`（官方 EVB `CH32H417SCH.pdf` 与当前 Petros breakout 一致）。
厂商 USB 栈在 SuperSpeed 链路训练超时约 1.5 秒后会自动回退到
USB 2.0（`TIM12_IRQHandler` → `USBSS_Device_Init(DISABLE)` + `USBHS_Device_Init(ENABLE)`），
一旦切过去这两个脚就归 USB2 PHY，调试接口当场消失。

**这不是崩溃**。判据：flash 被完全擦空、芯片里一行代码都没有时同样出现过失联——
没有代码在跑就谈不上跑死。

**证据强度注意**：引脚复用关系来自原理图网络名（`PB8/USB2DP/SWCLK`）加板上 A/B 实验
（USB 线插着=调试器一直在，拔掉=1.5 秒后断）。参考手册**没有**引脚分配表（那在数据手册里，
目前手上没有），所以这条尚未得到官方文档确认。

**修法**：`CMakeLists.txt` 里 `libhcs_USBSS_HS_FALLBACK=0`，在
`USB_Timer_Start()` 和 `USBHS_Device_Init()` 各加一处守卫（`libhcs LOCAL PATCH`，
见 `bsp/PROVENANCE.md`）。SS 失败就一直重试 SS，引脚永不被占。
**已验证**：halt 核 5 秒（主机判 USB 掉线）后调试器仍能连上，旧固件此处必断。

**连带影响**：任何一次调试器 halt 都会让主机判 USB 掉线，从而触发回退——所以旧固件
下"调试一次就失联一次"，是个自我强化的死循环。

**2026-07-25 补充依据（把这条从【实测】升级为【RM】+【原理图】）**：

- 参考手册 **25.1（USBHS 功能描述）** 最后一条功能：「支持 USART 串口或 I2C 引脚映射，
  **兼用于两线调试**」。官方对引脚共用只有这一句，埋在功能列表里，没有引脚表也没有警告。
- 参考手册 **9.3.2.1（AFIO_PCFR1，`SW_CFG[2:0]`，bit 26:24）**：「SWD（SDI）是访问内核的
  调试接口。**系统复位后总是作为 SWD 端口**」，`0xx` 启用、`100` 关闭作为 GPIO。
  即复位默认是 SWD，固件不主动放弃就不会丢。
- **原理图**（`U1 = CH32H417MEU6`）：MCU 82/83 脚的引脚名直接印着
  `PB8/USB2DP/SWCLK`、`PB9/USB2DM/SWDIO`，网络名分别是 `UD_P` / `UD_N`。
  `J1`（USB3.0-A 座）的 3 脚 `D+` / 2 脚 `D-` 就是这两个网络，`J4`（1x06 排针）的
  **4 脚 `UD_P` / 3 脚 `UD_N`** 也接在同一对网络上。**同一个电气节点，中间没有串阻、
  没有跳线、没有模拟开关。**

  > **2026-08-05 按当前 breakout 原理图更正 [原理图]**：早期条目写的是"`P1`（USB3.0-A）
  > 的 `Dp`/`Dn`、`P2`（DEBUG HEADER 6）的 2/3 脚、`P1` 的 VBUS 经 `F1`(2A 保险丝)"，
  > 那是官方 EVB 的编号。当前 Petros breakout 上：座子是 `J1`/`J4`，J4 上的调试线在
  > **3/4 脚**（不是 2/3），**没有保险丝**——`J1` 的 VBUS 经 `D1`（防倒灌二极管）到
  > `VSYS`，再经 `U2 = CJA1117B-3.3` LDO 出 `3V3`。**引脚复用这个核心结论两块板一致。**

- **`R2`(1.5K) 不是 D+ 上拉，别看错了** [原理图]。它一端接 `3V3`、另一端串
  `D2`(RED LED) 到地，是**电源指示灯的限流电阻**，和 USB 没有任何关系。这块板上
  `UD_P`/`UD_N` 从 J1 到 MCU 之间**什么元件都没有**，是纯粹的直连。

### 1.2 「SS 模式就安全」是错的表述

[1.4](#14-usb3-座子里有两套线但不能同时用) 那句「SS 模式下 D+/D- 完全闲置」只说了
**芯片这一侧**不再驱动它们，不等于这两个脚电气上干净——插着线时主机的 15 kΩ 下拉和
线缆电容一直挂在 PB8/PB9 上。

正确的说法是**两个独立机制**，别混为一谈：

| 机制 | 触发条件 | 后果 |
|---|---|---|
| 固件把 USB2 PHY 接过去 | SS 训练超时回退（`libhcs_USBSS_HS_FALLBACK=1`） | 引脚归 USB2 PHY，调试口必死 |
| 主机主动驱动 D+/D- | 插着线且 **SS 没枚举成功**，主机回头试 USB 2.0 | 主机推挽驱动怼在 SDI 两线上，握手失败 |

第二条能解释一个曾经互相矛盾的观测对：跑我们的 SS 固件时**插着线调试器一直在**（SS 枚举
成功，主机没兴趣碰 USB2 那一半），而跑官方那个纯 USART demo（一行 USB 代码都没有、SS 永远
不会建立）时**插着线连芯片型号都读不到**（报「设置芯片型号失败…两线通信状态」）。

**2026-07-26 机制二已实测证实**，不再是推测。证据有两条：

1. **内核日志直接拍到主机在驱动那两根线**。板子在跑 SS、WCH-Link 也插着时：
   ```
   usb 3-8: new low-speed USB device number 38 using xhci_hcd
   usb 3-8: Device not responding to setup address.
   usb usb3-port8: unable to enumerate USB device
   ```
   **低速是靠 D- 上拉判定的**，而 D- 就是 PB9/SWDIO——主机看到的这个"低速设备"，
   其实是 WCH-Link 驱动调试线的静态电平。主机会反复重试、还会给端口做 power cycle。

2. **插着线烧录必然失败**（见 [3.10](#310-usb3-线插着就烧不了与固件无关-实测)）。

**操作纪律**：拔线烧录 → 插线测试。

> **2026-08-05 更正 [原理图]**：早期这里写"要长期同时有数据口和调试口，走 Type-C
> （P10，`OTG_FS` → PA11/PA12，引脚不相交）"——**当前 Petros breakout 上没有这个座子**，
> 详见 [1.4](#14-usb3-座子里有两套线但不能同时用)。在这块板上"拔线烧录"不是可选纪律，
> 是**唯一选择**。真要第二个 USB 口，从 J2 的 29/31 脚（`GP_A11`/`GP_A12`）飞线自接。

自己画板时把 USB3 座的 USB2 D+/D- 串 0Ω 或不接（我们 `HS_FALLBACK=0` 本来就不用），
官方 EVB 和这块 breakout 都没留这个余地。

### 1.3 幻影低速设备会拖垮主机端口，进而挡掉热插拔 **[实测]**

上面那串 `unable to enumerate USB device` 不只是噪声。主机对该端口放弃之后，**连带不再
去轮询它的 SS 侧**，于是插上线也不枚举（`/sys/.../usb4-portN/state` 读作 `not attached`）。

**这不是板子的问题**：板子这侧一直在找对端（见
[4.5](#45-ss-的-rx_detect-会永远重试别信8-次就放弃-rm--实测)），是主机端口被拖住了。

**判据**：`cat /sys/devices/.../usb4-portN/state`。`not attached` 且板子确实在跑，
就是这条。**修法**：线插着的情况下复位板子（`openocd -c init -c "reset run"`），
或者换一个 USB3 口。实测复位这招每次都灵。

### 1.4 USB3 座子里有两套线，但不能同时用

一个 USB3.0-A 座（**J1**）同时接出：
- `D+`/`D-`（网络 `UD_P`/`UD_N`）→ PB8/PB9（USB 2.0）
- `SSRX+/-`、`SSTX+/-`（网络 `SSRX_P/N`、`SSTX_P/N`，经 `C23`/`C24` 100nF AC 耦合）
  → 专用 SerDes 脚 84-87（USB 3.0 SuperSpeed）

USB 3.0 规范规定设备要么 SS 模式要么 USB2 模式，主机只枚举一次。SS 枚举成功时厂商栈
会主动 `USBHS_Device_Init(DISABLE)`。**SS 模式下 D+/D- 完全闲置**，这正是能一边跑
5 Gbps 一边用调试器的原因。（这句话的适用边界见
[1.2](#12-ss-模式就安全是错的表述)。）

> **2026-08-05 更正 [原理图]**：早期条目把这个座子写作 `P2`，那是官方 EVB 编号；
> 当前 breakout 上是 `J1`。同时早期写的"板上另有 Type-C 座（P10）接 `OTG_FS_DP/DM`
> → PA11/PA12，真要同时有数据面 + 调试/DFU 接口就走这个口"——**当前这块 breakout 上
> 不成立，没有第三个 USB 座子**。全板只有 J1(USB3.0-A)、J2(Pico 排针)、J3(板对板)、
> J4(1x06)，`PA11`/`PA12` 只作为普通 GPIO 引到 J2 的 **29 脚(`GP_A11`) / 31 脚
> (`GP_A12`)**，没有接任何 USB 座。
>
> **后果**：在这块板上 USB 数据面和调试口共用同一组物理线，**没有退路**，
> [1.2](#12-ss-模式就安全是错的表述) 结尾的"走 Type-C"那条建议在此板不可用。要另开一路
> USB，只能从 J2 的 29/31 脚飞线自接 USB 座（PA11/PA12 = `OTG_FS_DP/DM`，Full Speed）。

### 1.5 VBUS 会阻止 WCH-Link 给板子断电

板子的 USB3 线插在电脑上时，VBUS 持续供电。此时拔插 WCH-Link **不等于给板子断电**，
所有依赖上电复位（POR）的恢复手段全部失效。

要真正 POR：两根线都拔掉，或只留 WCH-Link 供电（板子吃 Link 的 3V3，拔 Link 即断电）。

**当前 breakout 的供电链** [原理图]：`J1` VBUS → `D1`（防倒灌二极管）→ `VSYS`
→ `U2 = CJA1117B-3.3` LDO → `3V3`。**没有保险丝**（官方 EVB 那道 `F1` 2A 保险丝在这块
板上不存在）。`VBUS`/`VSYS`/`3V3` 三条轨同时引到 `J2` 的 40/39/36 脚，所以**从 J2 供电
也会让板子活着**——排查"以为断电了其实没断"时，J2 上挂的东西也要算进去。

---

## 2. 工具链层

### 2.1 WCH 的 `interrupt("WCH-Interrupt-fast")` 会被主线 GCC 整条丢弃

```
warning: argument to 'interrupt' attribute is not '"user"', '"supervisor"', or '"machine"'
```

后果：`TIM12_IRQHandler`、`USBSS_IRQHandler`、`USBSS_LINK_IRQHandler`、
`USBHS_IRQHandler` 全部编译成普通函数，收尾是 `ret` 而不是 `mret`。启动代码设了
`intsyscr(CSR 0x804) = 0x0F`（硬件压栈 + 中断嵌套），不用 `mret` 返回既不恢复
`mstatus.MIE` 也不弹硬件栈——**第一个 USB/TIM12 中断进去，全局中断就永久关闭**，
USB 链路状态机停摆，而主循环看起来一切正常。这是 USB 完全没反应的真凶。

**修法**：`CMakeLists.txt` 加 `-Dinterrupt\(x\)=interrupt`，沿用仓库既有的
`-Dasm=__asm__` 套路。函数式宏只在 `interrupt` 后跟 `(` 时展开，所以我们自己写的
`__attribute__((interrupt))` 不受影响。

**复验**（换工具链后必做）：
```bash
riscv32-unknown-elf-objdump -d --disassemble=USBSS_LINK_IRQHandler \
    build/ch32_board_app.elf | tail -3     # 必须是 mret
```

### 2.2 TinyUSB 在这颗芯片上用不了

`tusb_speed_t` 最高只到 `TUSB_SPEED_HIGH`，没有 SuperSpeed、没有 SS Endpoint
Companion 描述符、没有链路层。要 5 Gbps 就只能用 WCH 的 USBSS 栈。另外三块板
（c_board / mc02 / hpm_board）继续用 TinyUSB，不受影响。

---

## 3. 烧录层（时间黑洞集中地）

### 3.1 绝对不要 `flash erase_sector wch_riscv.flash 0 last` **[RM]**

该范围会擦到 **option byte**，`RDPR` 变成 `0xFF` 即"读保护开启"。今天所有烧录失败的
源头就是这一条。`program` 自己会擦它要写的扇区，不需要额外擦除。

手册 46.2.2「安全性-防止非法访问」把后果说得很直白：

> 系统引导代码区、**SWD 或 SDI 模式**、RAM 区域**都不可对主存储器进行擦除或编程**。
> 可擦除或编程用户选择字区域。如果试图解除读保护（编程用户字），芯片将**自动擦除整片用户区**。

也就是说读保护一旦置位，WCH-Link 走的 SDI 通道**在硬件层面就没有擦写主存储器的权限**，
只能改用户选择字。这解释了本节后面所有现象，也说明它和固件内容无关。

### 3.2 解除读保护要 **两个会话 + 中间一次 POR**

```
会话 A:  init -> wch_riscv unfreeze        把"解除"写进 option byte
         【断电重上电 POR】                 option byte 此时才重载
会话 B:  init -> halt -> program ... verify
```

三个必须注意的点：

- **`wch_riscv unfreeze` 必须紧跟 `init`，不能放在 `halt` 后面**，否则静默失效
  （不打印 `Success to Disable Read-Protect`）。这是 MounRiver 的调用方式，可在其扩展
  里搜到字面量 `-c init -c "wch_riscv unfreeze"`。
- **中间那次断电不可省**。NRST 复位不重载 option byte，只有 POR 会。这就是
  "按 reset 没用，全部断电只插 WCH-Link 就能烧"的原因。
- **`unfreeze` 和 `program` 之间不能夹任何别的 flash 命令**（比如 `erase_sector`），
  夹了就把解除效果用掉了。

### 3.3 `e339e339` 到底是什么 **[实测，结论已修正]**

保护态下回读 flash 得到固定的 `e339e339`。回读 `0x00000000` 是判断保护是否真的解除的
最快判据（正常固件首字应是 `7f90306f`，一条 `j` 跳转）。

**2026-07-25 修正**：本条原先断言「擦净的 flash 应该读出 `ffffffff`」，那是照搬其他芯片
的经验。实测反例：同一个 openocd 会话里（已 `unfreeze`，`0x00010000` 能读出真实指令
`6081806f`），`0x00070000` 仍然读出 `e339e339`——那块区域当时确实没被编程过。
所以 **`e339e339` 更像是这颗 flash 未编程区的实际读值**，不是（或不只是）读保护假数据。

判据因此要改：**不能靠 `e339e339` 判断是否处于读保护**，要靠一个**已知有内容的地址**
（比如 `0x00010000`）读出来对不对。只有一个数据点，未做进一步验证。

同样地，保护态下 `flash erase_sector` 会**报成功但什么都没擦**，只有写入会诚实报错。

### 3.4 `.hex` 是分段的

`objcopy` 产出的 merged hex 在 `0x410c` 之后跳到 `0x10000`，openocd 会打印
`Flash write discontinued at 0x0000410c`。构建同时产出平铺的
`ch32_board_merged.bin`（基址 `0x0`，空洞填 `0xFF`），给 MounRiver / WCH-LinkUtility 用。

### 3.5 烧录地址：链接地址 vs 编程地址 **[RM]**

手册 46.1「闪存组织」给出主存储器页 0 位于 **`0x08000000`**（用户区 960K + BOOT 区 56K），
即 flash 的物理基址是 `0x08000000`，`0x00000000` 是启动别名。
板级手册（`CH32H417 Evaluation Board Reference-EN.pdf`）里 MounRiver 的下载地址是
`0x08000000`（V3F）/ `0x08010000`（V5F），而链接脚本用的是别名地址 `0x0` / `0x10000`。
**但 openocd 的 flash bank 声明在 `0x00000000`**，所以命令行烧录要用 `0x0`；
传 `0x08000000` 会得到 `Warn: no flash bank found` 然后假成功。

### 3.6 MounRiver 自带的 openocd 和我们用的完全相同

`/usr/share/MRS2/.../components/WCH/OpenOCD/OpenOCD/bin/openocd` 与
`~/3rd_party/wch-openocd/bin/openocd`（均为 `[前机路径]`）**二进制和 `wch-riscv.cfg`
字节级相同**。IDE 能烧而命令行烧不进时，**先怀疑调用序列，不要怀疑工具**。

### 3.7 WCH-Link 适配器自身会卡死

症状：`unknow WCH-LINK` / `LIBUSB_ERROR_TIMEOUT` / `claim interface failed`
（最后一个是有残留 openocd 进程占着接口，`pkill -f wch-openocd` 即可）。
前两个要拔插适配器，判据是 `lsusb` 里的设备号必须变化。

### 3.8 救援镜像

`~/mounriver-studio-projects/CH32H417MEU`（`[前机路径]`）是个纯 USART2↔USART3 轮询 demo，
**一行 USB 代码都没有**，所以 PB8/PB9 永远不被占用，调试口始终可用。
调试口被占死时烧它可恢复到干净状态。建议长期保留。

### 3.9 烧 merged hex 必须带 metadata 记录，否则 V5F 永远不被唤醒 **[实测]**

**现象**：openocd 烧录 `** Verified OK **`，板子却什么都不干；调试器看 hart1（V5F）
`pc = 0x00000000`。

**原因**：V3F 启动时 `decide_boot_mode()`（`boot/src/main.cpp`）调
`flash::app_image_is_valid()`，要求 `0x00070000` 处有一条合法记录
（magic `RGM1` + image_size + 该镜像的 SHA-256，见 `boot/src/flash/metadata.hpp`）。
这条记录本来由 DFU 下载流程写，而**用调试器烧 merged hex 完全绕过了 DFU**，
记录不存在 → 门禁不过 → V3F 停在 DFU park 死循环，`init_shared()` 和
`NVIC_WakeUp_V5F()` 都不执行。而那个 park 循环里 DFU 传输还没接（`main.cpp` 有 TODO），
所以是个死胡同：**烧进去就再也起不来**。

**判据**（三条互相印证）：
- hart0 的 PC 反解到 `libhcs::firmware::usb::Dfu::detach_requested()`（park 循环里）
- `0x20170000` 的启动诊断 `diag[6]` 不是 `0x00010000`（`Core_V5F_StartAddr`），是垃圾
  ——说明 V3F 根本没走到唤醒块
- `0x00070000` 的 magic 不是 `RGM1`

**修法**：`cmake/gen_metadata_hex.py` 在构建时算出 app 镜像的 SHA-256 并生成记录，
`merge_hex.cmake` 把它作为第三个片段并进 merged hex。现在 `cmake --build` 出来的
hex 是自洽可冷启动的，openocd 烧完直接跑。地址常量在 `CMakeLists.txt` 里，
**必须和 `boot/src/flash/layout.hpp` 保持一致**。

烧录日志里应能看到第二次分段：`Flash write discontinued at 0x0002b914,
next section at 0x00070000`——没有这行就说明记录没进去。

### 3.10 USB3 线插着就烧不了，与固件无关 **[实测]**

**这是硬性的，不是保守建议。** 2026-07-26 在最有利的条件下做过实验：SS 已成功枚举
（`configured`、5 Gbps）、SDI 读取一切正常（`0x10000` 读出 `6081806f`、`0x70000` 读出
`RGM1`），然后烧录——

```
** Programming Started **
Info : Flash write discontinued at 0x00009ee0, next section at 0x00010000
Error: ** Programming Failed **
```

同一时刻的内核日志显示主机正在 `usb 3-8` 上反复枚举那个幻影低速设备。**链条**：
`program` 必须 halt 核 → SS 链路断 → 主机判掉线、立刻回头探 USB 2.0 的 D+/D- →
那就是 SWCLK/SWDIO → 撞死 SDI → 写 flash 失败。

所以「SS 枚举成功时插着线也许能烧」是错的，**任何固件、任何 SS 状态都一样**：烧录
本身就要 halt，halt 就必然把线让给主机。

运气好的话失败发生在切段边界、旧镜像没坏（那次就是），但不可依赖。**烧录前先拔线。**

---

## 4. 调试观测层

> 本章各条互相依赖，建议按顺序读。核心结论一句话：**halt 会顺带复位，所以外设寄存器
> 和 PC 都不可信，唯一可靠的观测手段是让固件把状态写进 RAM，再按
> [4.7](#47-调试器-attach-期间-v5f-根本不跑同会话的-reset-run-读到的全是残留-实测)
> 的姿势去读。**

### 4.1 halt 之后读到的 PC 不可信

WCH 的 openocd 在 halt 时会顺带触发复位，连续几次都读到复位后拷贝代码的循环里
（`0x00004020`）。判断固件是否正常运行**看 USB 枚举**，不要看 PC。

同理不要看 `v5f_ready`（每次启动 V3F 都会 `init_shared()` 重新清零）。

### 4.2 `resume` 不会让 USB 回来

halt 会让主机判 USB 掉线，`resume` 无法恢复枚举，**必须 `reset run`**。

### 4.3 跑板子别用 `wch-dual-core.cfg`

它的 `reset run` 会同时放出两个 hart，V5F 就从 flash `0x0` 跑起 V3F 镜像，
症状是 V5F `mtvec = 0x20100003`（V3F 的向量基址）卡在 trap 死循环。
用单 target 的 `wch-riscv.cfg` 跑，只在需要同时观察两个核时用 dual-core cfg
（`targets wch_riscv.cpu.0` / `.1`）。

**补充**：它的 `init` 也会污染观测——只 `init` 不 `reset run`，照样看到两个 hart 的 PC
都落在 V3F 的 `0x201xxxxx` 代码区、hart1 `mtvec = 0x20100003`。所以**用 dual-core cfg
读出来的"V5F 在跑 V3F 镜像"未必是真的固件状态**，可能只是这个 cfg 自己造成的。
判断 V5F 死活请用 [4.7](#47-调试器-attach-期间-v5f-根本不跑同会话的-reset-run-读到的全是残留-实测)
的方法（不是 4.4 的——那条对 V5F 无效）。

### 4.4 halt 会复位，但核冻结在 `handle_reset`——RAM 因此保留，这是唯一可靠的观测法 **[实测]**

4.1 说"halt 会顺带复位"。2026-07-25 把它坐实并补上了关键的后半句：halt 之后
hart0 的 PC 反解出来是 `bsp/wch/Startup/startup_ch32h417_v3f.S:534` 的 `handle_reset`，
也就是**核被冻结在复位向量、一行代码都还没执行**。

推论——**复位后 RAM 尚未被重新初始化，还保留着上一轮运行的内容**。于是：

- ❌ **halt 后读外设寄存器没有意义**：复位把它们清了。我两次读 TIM10 的 `CNT_32`
  都是 0，一度以为 V5F 没跑，其实只是被复位清零后核又被摁住不让重跑。
- ❌ **halt 后读 PC 只在核恰好没被复位时有效**，不可依赖（同一天两次读，一次读到
  DFU park 循环里、一次读到 `handle_reset`）。
- ✅ **读 RAM 里的软件痕迹是可靠的**。标准动作：

  ```
  -c "init" -c "reset run" -c "sleep 2000" -c "halt" -c "mdw <addr>" -c "reset run" -c "exit"
  ```

  让它自由跑一段，再 halt 去读那段时间写下的 RAM。已验证可用的两个观测点：
  - `0x20170000` 起的启动诊断 `diag[0..10]`（V3F 写，见 `boot/src/main.cpp`），
    其中 `diag[6] == 0x00010000` 表示 V3F 走到了唤醒块；
  - `0x20178000` 的 `shared().v5f_ready`（`boot/src/mailbox.hpp` 里 `SharedBlock`
    的首字段），**读到 1 就证明 V5F 跑完了整个 `App()`**。V3F 每次启动都会
    `init_shared()` 把它清零，所以 `reset run` 之后再读到 1，必定是本轮写的。

  4.1 说 `v5f_ready` 不可信——那只针对**读到 0** 的情形（可能只是还没写）。
  **读到 1 是强证据**。

> ⚠️ **2026-07-26 更正**：上面那条"标准动作"在 V5F 上**会骗人**，`v5f_ready` 这个观测点
> 也已作废。见 [4.7](#47-调试器-attach-期间-v5f-根本不跑同会话的-reset-run-读到的全是残留-实测)。
> 本条对 **V3F** 仍然成立，故完整保留。

### 4.5 SS 的 RX_DETECT 会永远重试（别信"8 次就放弃"） **[RM + 实测]**

手册 27.2.6 `LINK_GO_DISABLED`（复位值 1）说：清零它 → LTSSM 进 SS.RX_DETECT →
每 12 ms 检测一次共 8 次 → 没检测到 TERM 就进 SS.DISABLE。**读到这里很容易误以为
"检测窗口一次性用完就再也不找了"，从而把热插拔失败归到板子头上。那是错的。**

厂商的 `USBSS_LINK_Handle()` 在 `LINK_STATE_DISABLE` 分支里第一句就是
`USBSSHx->LINK_CTRL &= ~LINK_GO_DISABLED;`，等于立刻把 RX_DETECT 重新拉起来。
**实测**：拔掉 USB 线空跑 3 秒，`diag[13]` 记录到 **11 次状态跃迁**，`diag[11]` 的
位图只有 bit4(DISABLE) 和 bit5(RXDET)——LTSSM 在这两态之间约每 270 ms 循环一次，
永远等下去。

被我们 `HS_FALLBACK=0` 短路掉的 `USB_Timer_Start(ENABLE)` 只影响"超时后切 USB2.0"，
**不影响这个重试循环**。所以插上线不枚举时，先怀疑主机端口
（[1.3](#13-幻影低速设备会拖垮主机端口进而挡掉热插拔-实测)），不是板子。

### 4.6 USB 层的实时状态只能靠固件自己记

4.4 说 halt 会复位。**外设寄存器首当其冲**：LTSSM 状态、`USBSS_DevEnumStatus`、
EP 的 chain 寄存器，halt 之后读到的全是复位值，毫无意义（我在 TIM10 上栽过一次，
又在 USB 上差点栽第二次）。

可用的办法是让固件把关心的值写进 `0x20170000` 那片 RAM，再按 4.4 的姿势读。
`app/src/app.cpp` 的 `poll_usb_link_diagnostics()` 和 `app/src/usb/vendor.cpp` 里的
计数就是这么做的（都标了 `TODO(usb-bringup)`），布局：

| 字 | 内容 |
|---|---|
| `diag[11]` | 出现过的 LTSSM 态位图（bit N = LINK_STATE 值 N） |
| `diag[12]` | 最后一次 `LINK_STATUS` |
| `diag[13]` | 状态跃迁次数 |
| `diag[14]` | `USBSS_DevEnumStatus` |
| `diag[15]` | EP1 OUT 完成次数 |
| `diag[16..21]` | `NUMP` / `DMA_OFS` / `CHAIN_LEN` / 收后 `RX_DMA` / armed 基址 / 算出的长度 |
| `diag[22]` | 缓冲基址处的头 4 字节 |
| `diag[23..25]` | 上行 armed 次数 / 最后长度 / 完成次数 |

**实测确认**：`收后 RX_DMA - armed 基址` 恒等于 `NUMP × DMA_OFS`，即
**`UEP_RX_DMA` 确实会自增**——这正是 `usb_ss_ep1_out_complete()` 倒推起始地址的依据，
原先只是从厂商 EP2 OUT 代码反推、手册未明说，现在有板上证据。

### 4.7 调试器 attach 期间 V5F 根本不跑——同会话的 `reset run` 读到的全是残留 **[实测]**

> 本条推翻了 4.4 在 V5F 上的适用性，是**当前观测 V5F 的唯一正确方法**。

4.4 那条命令（同一个 openocd 会话里 `reset run` → `sleep` → `halt` → `mdw`）用来看
V3F 没问题，**用来看 V5F 是错的**：openocd 挂着的时候 V5F 不会被真正放出来，那段
`sleep` 里 app 一行都没跑，`mdw` 读到的是**上一次断开状态下自由运行**留下的残留。

判据（2026-07-26 实测，加了一个每圈自增的 `diag[31]` 才看出来）：

| 观测方式 | 自由跑 1 s | 自由跑 3 s / 6 s / 10 s |
|---|---|---|
| 同会话 `reset run; sleep; halt` | 计数器纹丝不动（还是上电垃圾值） | 同左 |
| 先 `exit` 断开、再 attach 读 | 5.6e5 | 1.8e6 / 3.1e6，**与时间成正比** |

**正确姿势**：让 openocd 先退出，在**无调试器**状态下自由跑，再重新 attach 读残留：

```bash
$OCD/bin/openocd -f $OCD/bin/wch-riscv.cfg -c "init" -c "wch_riscv unfreeze" \
    -c "reset run" -c "exit"          # 断开，让它真的跑
sleep 10                               # 这段时间没有调试器
$OCD/bin/openocd -f $OCD/bin/wch-riscv.cfg -c "init" -c "wch_riscv unfreeze" \
    -c "halt" -c "mdw 0x20170000 32" -c "exit"
```

连带的两个后果：

- **`v5f_ready` 在这个协议下永远读到 0**，不能再当判据：attach 会触发复位 → V3F 重跑 →
  `init_shared()` 把它清零，而 V5F 被摁住不会重跑去写 1。4.4 里"读到 1 是强证据"这句
  在实践中已经用不上了。
- **换用 `diag[30]`（V5F 自己写、V3F 从不碰、也不在任何 image 的 section 里）**：
  `app.cpp` 的 `trace_init_step()` 在 `App()` 每一步后写进度号，**读到 12 = 整个 `App()`
  跑完（含 `v5f_ready = 1`）**。卡在哪一步就读到几。配套的还有 `diag[26..29]`：
  `utility/assert.cpp` 把断言的行号/文件/函数指针镜像到那里（`diag[26] == 0xA55EA55E`
  才算有效），因为 `assert_file/line` 那几个全局变量在 `.bss` 里，复位就被清了。

被这条坑了半小时：先后误判成"V5F 没被唤醒"和"V5F 卡在 init 里"，实际固件一直是好的。

---

## 5. 固件结构层

### 5.1 USB 初始化必须排在最后

`USBSS_Device_Init(ENABLE)` 内部就 `NVIC_EnableIRQ`，第一个 LINK/EP 中断会立刻
进到 `usb::vendor`，再经 `handle_downlink` 摸到 can/uart。所以它必须排在
`usb::vendor.init()` / `can` / `uart` **之后**，否则解引用未构造的 `Lazy<Vendor>`。

### 5.2 V3F 拥有时钟树

V3F 的 `boot/User/system_ch32h417.c` 里 `SystemInit()` 才开 HSE 并配 PLL；
V5F 那份 `app/User/system_ch32h417.c` **只读时钟不配置**。所以 V5F 必须由 V3F 唤醒，
不能单独跑。晶振本身没问题（实测 HSEON/HSERDY 置位、主 PLL 锁定、SysClk 400 MHz）。

### 5.3 WCH 双核 USBSS demo 把 USB 放在 V3F

`EVT/EXAM/USBSS/DEVICE/CH372Device/` 的双核配置里 `Hardware()` 在 V3F 上调用，
V5F 只做 HSEM 握手然后空转。我们反过来（USB 在 V5F）是有意的——转发热路径要留在
V5F 且避免跨核跳转。已验证可行：USB DMA 能正常访问 V5F 的 DTCM 缓冲。
