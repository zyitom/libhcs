# UART 配置统一到 EP0 —— 移交与续作说明

> **文档类型**：过程记录
> **适用范围**：`firmware/*/`（EP0 配置通道）、`host/include/libhcs/board/`、`core/include/libhcs/protocol/vendor_control.hpp`
> **状态**：现行有效（阶段 0-5、7、8 完成，hpm5321 台架实测通过（3.5）；阶段 6 待 c_board 上板；mc02 / c_board 固件改动只有编译验证）
> **相关文档**：[AGENTS.md](AGENTS.md)（构建/提交纪律）· [firmware/hpm_board/AGENTS.md](firmware/hpm_board/AGENTS.md)（EP0 契约表）· [firmware/mc02/AGENTS.md](firmware/mc02/AGENTS.md)（mc02 EP0 与 UART 时钟）

## 摘要

把 UART 的运行时配置统一到 EP0 vendor control 一条路径：给 `c_board` 补上 EP0（它和
`ch32_board` 目前只有数据流 `DataId::kUartNConfig` 一条路），同时把"设置后验证"从
百分比容差收紧为分频器精确往返，并消除 hpm/mc02 之间的重复实现。

**接手时先读第 1 节的已定决策与第 2 节的实测结论**——它们推翻了几个看起来合理的直觉
（尤其"主机能复算分频器"和"精确比对请求波特率"这两条，已实测证伪）。阶段 1 已移动
`kVersion`，**主机与固件必须同批升级**：未刷新固件的板在新 SDK 下 `kGetInterface` 版本
不符即拒。台架两块 hpm5321 已刷新并实测通过（3.5）；剩余只有阶段 6（4.6）与 mc02 /
c_board 的上板。

## 本文导航

- **第 1 节** 已定决策：保留 `kUart0` 命名、取消 RS-485 开关、跳过 ch32、分频器往返 + 宽松兜底。
- **第 2 节** 实测结论：主机不能复算分频器；921600 处处非精确（2.2 含 2026-09-30 更正）；
  CDC 会在会话外改 UART；`kUart0` 实物逐板不同；`DataId` 不进指纹。
- **第 3 节** 已完成：阶段 0（3.1）、examples 清理（3.2）、台架（3.3）、阶段 1-5/7/8 与复查修复（3.4）、
  hpm5321 台架实测（3.5，含 D2 与重连重放）、一次 SET 即全部（3.6）。
- **第 4 节** 各阶段原始计划；只剩 4.6（阶段 6）未执行。
- **第 5 节** 构建、格式检查口径、台架实测命令。**第 6 节** 盲区与纪律。

## 1. 已定决策（用户拍板，不要再讨论）

| 议题 | 决定 | 理由 |
|---|---|---|
| HPM 唯一串口的命名 | **保留 `DataId::kUart0`**，不新增枚举、不新增 hpp | 用户明确要求；其实际外设逐板不同（详见 2.4） |
| 寻址类型 | **不新增** `UartPort` / `hcs_uart_port.hpp` | 同上 |
| `libhcs_APP_RS485_ENABLE` 开关 | **取消**（方案 B）：USART2/USART3 永远编入、永远可配置 | 代价是接受 **1.8 KB D2 SRAM 常驻**（`Lazy<T>` 用 union，ring 从链接时即占 `.d2_sram`，与是否 `init()` 无关） |
| `ch32_board` | **跳过**，保留其数据流路径 | 其工具链本机未装、不可编；数据流是它唯一通道 |
| 验证策略 | 分频器精确往返 **+ 保留一层宽松兜底**（抓"离谱到不可能"的值） | 见 2.2 |
| examples | 删陈旧的、修好台架要用的、其余保留 | 已执行，见 3.2 |

## 2. 实测结论（这些推翻了直觉，务必先读）

### 2.1 主机无法复算分频器——"板端确认分频器 + 主机对账"是唯一可行形状

主机**不知道板子的 UART 内核时钟**：hpm 80 MHz、mc02 STM32H7 内核时钟（运行期由
`UART_GETCLOCKSOURCE` + PLL2Q/PLL3Q/HSI 解出）、c_board 42 或 84 MHz（APB1/APB2）、
ch32 100 MHz。各板 `effective_baudrate()` 都从**实际写入的分频器**反推。

因此精确验证必须实现为：

1. 板子用**自己的求解器**为请求速率解出分频器；
2. 编程；
3. 回读**实际写入**的分频器，与第 1 步比对，不等即 STALL + 新 reason `kConfigErrorVerifyFailed`；
4. 主机对"SET 往返中板端确认过的分频器整数"做相等比较，**主机不碰波特率数值**。

### 2.2 "精确比对请求波特率"已实测证伪——921600 处处不是精确值

各板用**各自的**求解器实算（hpm = `hpm_uart_drv.c:66-99` 的整数分频 + 过采样搜索，
`SCALE=1000`、`OSC 8..32` 步长 2、容差 3%；c_board = F4 HAL `UART_BRR_SAMPLING16` 的
12.4 定点分数分频；ch32 = WCH `USART_Init` 同式的 12.4 定点）：

| 速率 | hpm 80MHz | c_board APB2 84MHz | c_board APB1 42MHz | ch32 100MHz |
|---|---|---|---|---|
| 115200 | 114942 (0.224%) | 115226 (0.023%) | 115384 (0.160%) | 115207 (0.006%) |
| 460800 | 454545 (1.357%) | 461538 (0.160%) | 461538 (0.160%) | 460829 (0.006%) |
| **921600** | 909090 (**1.357%**) | 923076 (**0.160%**) | 933333 (**1.273%**) | 925925 (**0.469%**) |
| 1000000 | 精确 | 精确 | 精确 | 精确 |
| 2000000 | 精确 | 精确 | 精确 | 精确 |

**项目最常用的 921600 处处不是精确值**，且各板误差形状互不相同。`[hpm 列实测；
c_board / ch32 列按各自 HAL 宏算术推导，本机无硬件可上板复核]`

> **更正 [2026-09-30]**：本表初版把 hpm 的整数求解器套用到了 c_board / ch32 的时钟上，
> 得出"42/84 MHz 上 921600 误差 3.575%、2 Mbaud 在 42 MHz 被拒"等结论——那两块是
> STM32 式分数分频，不走该求解器，那几格全部作废。核心推论不变。

推论：任何"请求值 vs 回读值"的百分比或相等比较都不成立，只能比分频器整数。

### 2.3 UART 有两个写入者：CDC 会在会话外抢占（已实测）

**现象**：两块 hpm5321 的 `read_uart_setting(0)` 都返回 **baudrate=9596（8N1）**，而端口表
声明 921600。`9596` 只能由 80 MHz / osc=8 / div=1042 产生。`[实测 2026-09-30]`

**链路**：设备暴露 7 个接口 —— 0/1/2 = DMTool vendor、3 = **libhcs**（机器人数据通路）、
4/5 = **CDC**（`/dev/ttyACM*`）、6 = DFU Runtime。**CDC 与 libhcs 是独立接口，机器人数据
不流经 CDC**；但 `SET_LINE_CODING` 是 **EP0 控制传输**，由系统服务 **ModemManager**
（本机 active）在设备插入后自动探测时发出，**不需要 libhcs 会话、不需要应用参与**。

`firmware/hpm_board/app/src/dmtool/dm_adapter.cpp:286-296` 把它真实下发：

```cpp
if (!link::uplink_enabled()) {            // = !session_established()
    if (auto* uart = uart::uart_array[0].try_get(); uart != nullptr) {
        if (bit_rate != 0U)
            (void)uart->set_baudrate(bit_rate);   // 真实改 UART
```

该文件 `:829` 的注释早已记录此风险。**关键性质：抑制不等于恢复**——门控让 CDC 只在会话外
生效。重连时 `apply()` **会**把被改掉的口纠正回来，但只限主机 `configuration_` 里有的口
（构造时给的，或运行期经 `configure_uart*` 写回的）；没配置过的口保持第三方写入的值。
`[实测 2026-09-30，见 3.5 第 4 条]`

**对验证方法的影响（重要）**：`dual_board_test uart` 这类"链路通不通"的检查**不可靠**——
两端被第三方改成同一个值时照样 PASS（与 mc02 那个"同板回环测不出问题"同类陷阱）。
**必须主动比对读回值**。实机验证 UART 前先 `sudo systemctl stop ModemManager`。

### 2.4 `DataId::kUart0` 的真实外设逐板不同

| 板 | `DataId::kUart0` 实物 | 芯片上是否有 UART0 |
|---|---|---|
| hpm5321 | **UART2**（PB08/PB09） | 有，但板上未用 |
| hpm6e8y | **UART1**（PY06/PY07，只有调试孔） | 有，但板上未用 |

且 HPM 的 UART 节点**必须显式设置源与分频**才拿到非默认频率——见
`boards/hpm5321/app/board_app.cpp:45-50`（`init_uart_clock()` 只调
`clock_add_to_group` + `clock_get_frequency`，而 CAN 那边 `:32` 有
`clock_set_source_divider(clock_can0, clk_src_pll1_clk0, 10)`）。

### 2.5 `DataId` 的数值不进任何指纹

`session_layout_fingerprint()`（`core/src/protocol/protocol.hpp:202-228`）只折叠
`SessionType` 的值；EP0 的 `layout_fingerprint()` 只折 `kSessionWireVersion`。
**两者都不含 `DataId`。** 改动 `DataId` 数值**不会被版本门控拦住**，是静默错路由。

**因此：本次任务不动任何 `DataId` 数值。** 需要退休的枚举值只能"退休"不能"重编号"
（与 `0x46 kSetEndpointMode`、`SessionType::kSyncSample` 的处理一致）。

## 3. 已完成的工作

### 3.1 阶段 0：提取共享 EP0 机制（已完成并编译验证）

新增 [firmware/common/app/src/usb/ep0_staging.hpp](firmware/common/app/src/usb/ep0_staging.hpp)
（103 行），`namespace libhcs::firmware::usb::ep0` 导出：

- `kStagingCapacity`（64）
- `g_control_buffer[kStagingCapacity]`
- `g_last_config_error`（`vc::LastConfigErrorPayload`）
- `record_config_error(Request, index, reason, value = 0)`
- `concept Ep0Payload`（`trivially_copyable` + 装得下）
- `reply<Payload>(rhport, request, payload, staging = 默认缓冲)`
- `staged<Payload>(staging = 默认缓冲)`

`firmware/hpm_board/app/src/usb/vendor_control.cpp` 与
`firmware/mc02/app/src/usb/vendor_control.cpp` 各删 65 行本地重复实现，改用
`using` 引入共享名字。**净减 94 行。**

设计约束（改动时不要破坏）：

- **只提取机制，不提取派发**。各请求回答什么、能力位怎么填、下标空间多大留在各板——
  hpm 设 MS OS 2.0 描述符集但不设 `kCapCanModeSettable`，mc02 相反（2026-09-30 起 hpm
  也置 `kCapCanModeSettable`，切法是重初始化控制器，见 hpm_board AGENTS.md EP0 约束 1）。共享派发会需要一层
  板策略模板，把 `firmware/hpm_board/AGENTS.md` 契约表记录的差异藏起来。
- **`staged<T>()` 参数化于源缓冲**（`std::span`，带默认值）。这是为 `ch32_board` 预留的：
  它的载荷在 `USBSS_EP0_Buf` 里由 ISR 填入，不是 `g_control_buffer`。
- 共享头注释里写了"两板断言失败记录不对称"的修法（hpm 曾漏记速率、mc02 曾漏记帧格式），
  合并后不应再分歧。

**验证**：`hpm_board_app` 与 `mc02_app` 均编译链接通过；三个文件 `clang-format` 干净。

### 3.2 阶段 7 部分：examples 清理

- **删除 6 个**（对应已退役的 PTPC/SOF→PTPC 路径或自称一次性）：`sof_probe`、
  `microframe_interference_test`、`microframe_timebase_test`、`time_sync_matrix`、
  `mc02_time_sync_test`、`topo_probe`。CMakeLists 同步删 35 行。
- **修好 3 个**：`dual_board_test`（`a511:5322` → `34b7:6632`）、`uart_cross_test`、
  `rs485_cross_test`。注意后两个的 mc02 判据**必须保持 `a511:0723`**——mc02 固件与板卡类
  仍用旧 vendor（见 4.6）。
- **`hpm5321_loop_probe` 无需改动**（它不枚举 USB，只从 argv 收序列号）。

**未跟进**：`firmware/hpm_board/SOF_TIMEBASE.md`、`firmware/hpm_board/AGENTS.md:293`、
`host/src/time/microframe_timebase.cpp:150` 仍引用已删除的工具（文档更新属阶段 8）。

### 3.3 台架与环境（本机已就绪）

两块 hpm5321 在线，接线：A.CAN1↔B.CAN1、A.CAN2↔B.CAN2、A.UART0↔B.UART0。
序列号 `AF-90A7-144F-BAB7-6363-7E81-5FB4-59F2-DC3C`（板 A）、
`AF-958F-E837-AFA1-0DCA-0EBC-4A77-E893-12EA`（板 B）。两板固件已刷至
`v3.3.1-0.dev.6.gcf404b7`，与当前 SDK 匹配。`dual_board_test link` 六条链路全 PASS。

udev 规则 `/etc/udev/rules.d/99-hcs-boards.rules`（**app 与 bootloader 两个 vendor 都要**）：

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="a511", MODE="0666", GROUP="plugdev"
```

第二条不可省：**应用的 USB 身份是 `34b7:6632`，但 bootloader 仍是 `a511:5322`**，
`dfu-util` detach 后会以旧身份重新枚举。

### 3.4 阶段 1-5、7、8：代码完成（2026-09-30）

四板（`hpm_board_app`、`mc02_app`、`c_board_app`、host SDK 含 examples）均编译链接通过，
hpm 的 ILM 链接期断言通过；改动文件 clang-format 干净（口径见第 5 节）。**未上板**。

落地形状（与第 4 节计划的偏差也写在这里）：

- **wire（阶段 1）**：`UartConfigPayload` 12 字节，`divisor` 是原始寄存器值（hpm
  `DLM:DLL`、STM32 `BRR`），`oversample` 是**倍数**（8..32），不是寄存器编码——hpm 的
  OSCR 字段 0 表示 32，板端上报前已解码。`kConfigErrorVerifyFailed = 7`。
- **板端（阶段 2/3）**：三板同一形状——求解、断言（调用方回显的 `divisor`/`oversample`
  必须等于求解结果）全部前置，然后写入，写后回读分频器不符报 `kConfigErrorVerifyFailed`
  （只有这一种 STALL 发生在写入之后）。hpm 复制了 SDK 的 static 求解器（`solve_divisor`，
  含 `UART_SOC_OVERSAMPLE_MAX` 上界），D2 已修；mc02 `set_framing` 已拆成
  `check_framing` + `commit_framing`；每条 UART STALL 路径都先记锁存。
- **主机（阶段 2/4/5）**：`configure_uart()` / `request_can_mode()` 各只发一次 SET，成功不回读，
  STALL 时读一次锁存后抛出（见 3.6；本条初版曾是"SET 后 GET 回读 + 主机侧 10% 兜底"，已被
  3.6 取代）。`Configuration::uart[8]` 存完整
  `UartSetting`。`hcs::Reconfigurable` 提供共享重配锁；四个 EP0 板类（`Hpm5321`、`Hpm6e8y`、
  `Mc02`、`CBoard`）的运行期 `configure_*` **与重连钩子**都持锁，改动经
  `hcs::reconfigure_uart()` 写回 `configuration_`（`Mc02::configure_can` 同样写回
  `can_fd`）。写回规则：分频器断言跟随它所属的速率——换速率即作废旧断言，否则重连重放
  会拿旧分频器断言新速率而必然 STALL。D4 已修（锁存只读一次）。
- **c_board（阶段 3）**：固件侧 EP0 + 握手门控 + 端口抽象面齐备；**主机 `CBoard` 同步接入
  EP0**（重连钩子读 `kGetInterface`、`configure_dbus/uart1/uart2`、回读）——固件已加握手门，
  主机不接入的话 `CBoard` 永远开不了会话。带内 `uart1_config`/`uart2_config` 暂留，见 4.6。
- **阶段 7**：`libhcs_APP_RS485_ENABLE` 已从 CMake、源码与 L2/L3 现行文档中移除；日志类
  文档（`PACKET_RATE_LOG.md`、`UART_RING_LOG.md`）里的旧开关是当时的实测条件，不改。
  已删工具的引用：`hpm_board/AGENTS.md` 诊断表与 `SOF_TIMEBASE.md` 已标注"已删除、git
  `cf404b7` 可取回"；代码注释里 `[实测 ..., <工具名>]` 形式的来源标注是历史出处，不改。
- **阶段 8**：`hpm_board/AGENTS.md` EP0 契约表（载荷尺寸、`kConfigErrorVerifyFailed`、补上
  漏列的 `0x48 kGetLastConfigError`、"比分频器"取代"用容差比"）、`mc02/AGENTS.md`、
  `mc02/README.md`、`c_board/AGENTS.md`（新增 EP0 一节）、`hpm5321/PINOUT.md` 表头均已更新。

**复查中发现并已修的问题**（前一轮改动留下的，记下来是因为它们都编译得过）：

1. mc02 `kSetUartConfig` 在**写入前**调 `verify_baudrate()`，比的是旧 BRR 与新解——任何
   真正改变速率的请求都会 STALL。已改为写后回读。
2. hpm `solve_divisor()` 返回 OSCR 编码（32→0），`oversample()` 返回倍数（0→32），两者
   在过采样 32 时永不相等。80 MHz 下求解器实际到不了 32（最高 3 Mbaud 用 26），属潜伏
   缺陷；已统一为倍数。
3. 主机写回在速率改变时保留旧 `divisor` 断言，重连必 STALL（见上）。
4. `Hpm5321`/`Hpm6e8y` 的重连钩子不持锁读 `configuration_`，与持锁写回的运行期改口构成
   数据竞争。
5. 主机 `CBoard` 未接入 EP0（见上）。
6. `.scripts/clang-format-check --fix` 被以本机 clang-format **18** 跑过，把 8 个无关文件
   按 18 的风格改写（CI 是 20，会判失败）。已还原。

### 3.5 台架实测（2026-09-30，两块 hpm5321，本次改动的固件同批刷写）

全部 `[实测 2026-09-30]`。探针与测试程序是一次性的，未入库（只 `ep0_config_test` 的新增项入库）。

1. **刷写后即读**：新 SDK 与新固件 `kVersion` 一致；两板 UART0 回读 **9596 baud、divisor
   1042、oversample 8**——ModemManager 在重新枚举后经 CDC 改写，2.3 的现象原样复现，
   且新的整数字段与反推速率自洽（80 MHz / (8 × 1042)）。
2. **`ep0_config_test`**（A、B 各跑一次）：全部通过。新增项：3 Mbaud 回报 divisor 1 /
   oversample 26（非 8 倍过采样路径）；回显错误分频器被拒、锁存 `kConfigErrorVerifyFailed`、
   端口分频器不变。
3. **双板**：两板经 EP0 设 921600 并回读确认 divisor 11 后，`dual_board_test link` 六链路
   全 PASS，`uart 100` 6400/6400 字节零错——速率先确认过，所以这次 PASS 不是 2.3 那种
   "两端被改成同一个错值"的假通过。
4. **D2**：A→B UART0 连续流 96000 字节，其间另一线程每约 3 ms 对 A 发一次 6 Mbaud
   （共 388 次，全部被拒）：收 96000/96000、零错位，A 的分频器仍为 11。修复前每次被拒
   都会先拆在途 TX DMA `[推断，未做旧固件 A/B：旧固件的 kVersion 与新 SDK 不通]`。
5. **重连重放**：构造给 921600 → 运行期先回显分频器断言（divisor 11）再改只给速率的 115200
   → 冻结主机进程（SIGSTOP）6 s 使板端 4 s 租约过期 → 期间另一进程经 EP0 把 UART0 改成
   9600（模拟会话外第二写者）→ 恢复。主机约 450 ms 内 `session-down → up`，重连钩子重放出
   **divisor 87（115200）**：是运行期写回的值，不是构造值，也没有因过期的 divisor 11 断言
   而 STALL。
   注意：USB 复位（`libusb_reset_device`）不能用来做这个测试——传输层把它当 `NO_DEVICE`
   判为永久故障（`kFaulted`），不走重连钩子。

**已知小瑕疵**：`ep0_config_test` 第 8 段结束时把 UART0 留在 115200，与其开头"每条路径都还原
921600"的注释不符（改动前即如此）。台架测完已手动恢复 921600。

### 3.6 一次 SET 即全部：去掉主机回读（2026-09-30，用户要求）

**结论**：正常情况 = 握手 1 次 + 每路 1 个 SET；出错 = 再读 1 次 `0x48` 取原因后抛出。
USB 控制传输一次只能单向带数据，SET（OUT）只能回 ACK/STALL，所以"原因"必须另读——但只在
失败时读。

- **主机**：`configure_uart()` 与 `request_can_mode()` 不再回读，CAN 不再先 GET 时序去回显
  （时序字段传 0 = 不核对），`request_can_mode()` 去掉 `settable` 参数。失败统一走
  `hcs::throw_rejected()`：读一次锁存，请求码/下标对不上即判为陈旧锁存、不当本次原因；只有
  `kConfigErrorVerifyFailed` 提示"可能已写入"。实际值要看显式调 `read_uart_setting()` /
  `read_can_config()`。HCS 的 `balance_infantry.cpp` 同步删掉构造后的回读与 5% 检查。
- **板端**（ACK 必须等于已生效，原先靠主机回读兜的全部移进板端）：
  1. 10% 宽松兜底前移到写入前（`ep0::rate_plausible()`，对求解结果算，`baudrate_for()`），
     三板共用；无 apply 位的纯断言路径同用这一个函数。
  2. `commit_framing()` 后回读帧格式（`ep0::framing_matches()`），不符 `kConfigErrorVerifyFailed`。
  3. CAN 切帧型后回读：hpm 核对帧型 + 仲裁段速率/采样点未被重初始化扰动；mc02 核对帧型。
  4. 补齐锁存：mc02 `kSetCanConfig` 数据段原有 8 条 STALL 全不记，hpm/mc02 SET 的 setup 段
     STALL 也不记——在"失败只读一次锁存"的模型下这会把更早的原因报成本次原因。
- **台架实测**（两块 hpm5321 重刷，`LD_PRELOAD` 截 `libusb_control_transfer` 计数）`[实测 2026-09-30]`：
  构造（UART0 + 两路 CAN）= `0x40` + `0x42`×2 + `0x44`，无读；运行期改速率 = 1 个 `0x44`；
  6 Mbaud / 错分频器回显 = `0x44` STALL + `0x48`，异常原因正确；7E2↔8N1 各 1 个 SET 且
  板端核对通过；CAN1 classic↔FD 各 1 个 `0x42`，另行显式回读确认已切换；重连钩子 =
  `0x40` + `0x44`。`ep0_config_test` 两板全过，`dual_board_test link`/`uart`、D2 流测试、
  重连重放复测全过。

## 4. 各阶段计划（阶段 0-5、7、8 已完成，见 3.4；阶段 6 待做）

### 4.1 阶段 1：wire 变更 + 版本移动（一次性关卡）

`core/include/libhcs/protocol/vendor_control.hpp` 的 `UartConfigPayload` 增加分频器与
过采样回读字段（首选 12 字节）：

```cpp
struct UartConfigPayload {
    uint32_t baudrate;
    uint16_t divisor;      // 实际写入的分频器（mc02 = BRR 低 16 位；hpm = 快照的 DLM:DLL）
    uint8_t oversample;    // hpm 的 OSCR（0 表示 32）；mc02 的 OverSampling
    uint8_t word_length;
    uint8_t parity;
    uint8_t stop_bits;
    uint8_t control;
    uint8_t reserved;
};
```

- GET 填 `divisor`/`oversample`；SET 把它们当**断言回显**（白得 `request_can_mode` 那种
  "整条身份一起断言"的性质）。
- 按维护规则（同文件 `layout_fingerprint()` 附近）补 `fold(offsetof(..., divisor))` 与
  `fold(...oversample)`。**`kVersion` 会自动移动，禁止手改。**
- 新增 `ConfigErrorReason::kConfigErrorVerifyFailed = 7`（区分"求解器拒绝"与"写进去但
  回读不符"），配 `config_error_name()` 分支与 `fold(...)`。
- 四块板的 `kGetUartConfig` 处理器填新字段。

> **`kVersion` 移动同时影响所有板。** 未刷的板在新 SDK 下不可用，直到刷完。台架两块可
> 同批刷；mc02 不在台架上。

### 4.2 阶段 2：分频器精确验证（D1）+ D2

- **板端精确比对**（见 2.1 的四步）。
- **主机侧**：`host/include/libhcs/board/hcs_config.hpp` 删三处百分比测试
  （`hcs_config.hpp:257`、`hpm/vendor_control.cpp:401`、`mc02/vendor_control.cpp:390`
  的 5%），改为对确认过的分频器做整数相等比较。帧格式继续精确比对（已是如此）。
  保留一层宽松兜底抓"离谱值"。两处把 5% 当契约写进注释的地方也要改
  （`vendor_control.hpp:214-216`、`hcs_config.hpp:224-226`）。
- **D2**：`firmware/hpm_board/app/src/uart/uart.hpp:70` 的 `TxBuffer::abort_transmit()`
  在 `uart_set_baudrate()`（`:72`）**之前**无条件执行；求解器拒绝时返回 false 但在途 TX DMA
  已拆，使三处"STALL 严格等于什么都没改"为假。**不能简单后移**（DLAB 窗口要求 DMA 已停，
  注释 `:65-69` 解释充分）——正解是把求解拆出来前置（SDK 的 `uart_calculate_baudrate`
  是 `static` 未导出，但 mc02 已有 `solve_brr` 先例）。同时让 mc02 把 `set_framing` 拆成
  `check_framing` + `commit_framing`，使"STALL 即未改"机械成立。

### 4.3 阶段 3：c_board EP0

`firmware/c_board/app/include/tusb_config.h:45` 已启用 `CFG_TUD_VENDOR 1`，TinyUSB 的弱
`tud_vendor_control_xfer_cb` 已存在，**只是从未覆写**——所以这块最容易。

1. 新增 `firmware/c_board/app/src/usb/vendor_control.cpp`，**以 mc02 为模板**（无 MS OS 2.0
   分支，更简单）。
2. SETUP/DATA/ACK 三段；`uart_count = 3`，下标 0/1/2 = DBUS/UART1/UART2。
3. **加 `set_ep0_handshake_done` 门控** —— 最易漏。hpm 上它在 `session_allowed()` 里门控
   `kStart`；c_board 现在没有。不加则老主机跳过 `kGetInterface` 照样开会话，而板子已忽略其
   带内配置写入——**正是本任务要消灭的静默失败，会在新板上复现**。
4. `caps`：c_board 是 bxCAN，`kCapCanFdLongFrames` 与 `kCapCanModeSettable` 均清。
5. 给 c_board 的 `Uart` 补端口抽象面：**从一开始就"先校验后写"**，新增
   `effective_baudrate()` / 帧格式回读（现在完全没有）。注意两档时钟
   （`uart.hpp:109-113`：USART1/USART6 走 APB2=84MHz，其余 APB1=42MHz）。

### 4.4 阶段 4：`Configuration` 帧格式字段

`hcs_config.hpp:86` 的 `std::optional<uint32_t> uart_baudrate[8]` → `std::optional<UartSetting> uart[8]`
（**替换**而非并行新数组，避免"是否已配置"出现第二个真相源）。

理由：现在 7E2 只能构造后显式配，**第一次重连就静默退回 CubeMX 帧格式**。重放路径可直接
复用已有的 `configure_uart(handler, port, UartSetting)` 重载。需写进文档的语义：
`UartSetting` 全零 = "不改"；optional 未设 = "别碰"；已设但字段为零 = "设速率，帧格式保持
固件现状"。这是 `!` 破坏性变更，单独提交。

### 4.5 阶段 5：D3 / D4 / 不对称

- **D3**：`mc02.hpp:309-338`、`hpm5321.hpp:198-203`、`hpm6e8y.hpp:191` 的 `configure_uart*`
  **不加锁**；**mc02 是唯一持有 `reconfigure_mutex_` 的板且只有 `configure_can` 用它**。
  两件事：(i) 让每块板的 `configure_uart*`/`configure_can` 都持重连锁（四块板结构相同，
  最好把锁提到共享基类）；(ii) 运行期改动**写回 `configuration_`**，使重连重放而非回退。
  **若只能做一件，先做 (ii)**——它才是消除假"不一致"抛错的根因。
- **D4**：`hcs_config.hpp:238-249` 失败路径上 `read_last_config_error()` 被调 **3 次**
  （lambda 内一次、`.request` 一次、`config_error_suffix` 内一次，见 `:171`）。锁存粘滞可
  覆盖，三次读取可能互相矛盾。改为读一次入局部，`config_error_suffix` 接收已读到的锁存。
- `mc02.hpp:430` 给 `settable` 硬编码 `true`，应传板子实报的 `can_mode_settable`。

### 4.6 阶段 6：删除数据流路径（**只能部分删**）

> **状态 [2026-09-30]：未执行。** 前提"c_board 迁移完成"只满足了代码与编译一半，
> 实测做不了（本机无 c_board 硬件）。与 c_board 无关的几项（`kCanNConfig`、
> `kUart0/3/7/10Config`）技术上可以先删，但它们与 c_board 那几项共用同一批
> serializer/deserializer/描述符改动，拆两次做只会把同一片代码翻两遍，故整体等实测后一次做。

**只有 c_board 迁移完成、ch32 明确保留的前提下**才执行。顺序：c_board 上线 EP0 → 补端口
抽象面 → 实测 → 才删其回调。

**可删**：`kCan0Config`..`kCan3Config`（零引用）；`kUart0Config`（hpm/mc02 均已
`return false`）；`kUart3Config`、`kUart7Config`、`kUart10Config`（mc02 已退役）；
`kUartDbusConfig`（c_board 迁移后无用户）。

**必须保留**：`kUart1Config`、`kUart2Config` —— **ch32 的唯一配置路径**。

配套：`core/include/libhcs/spec/uart.hpp` 的 `config_data_id` 成员与构造参数（仅限不再需要
它的板）；`deserializer.cpp:69-75,201-211`、`deserializer.hpp:34,143`、
`serializer.hpp:118-120,453-459`；`handler.hpp:29`、`handler.cpp:739-740,820-823`；
`c_board.hpp:176-193`/`ch32_board.hpp:170-186` 的 `uart1_config`/`uart2_config`。

`DataId` 值只退休不重编号（见 2.5）。

### 4.7 阶段 7 剩余：`AS485_ENABLE` 开关取消 + 已删文档的引用

- 取消 `libhcs_APP_RS485_ENABLE`（见 1 的决策表）。连带：`mc02/vendor_control.cpp` 的
  `kUartIndexCount` 与 `uart_by_index()` 里的 `#ifdef` 可去；`uart_count` 的语义可从
  "下标空间上界"恢复为真实端口数。
- 修 `SOF_TIMEBASE.md` 等文档对已删工具的引用。

### 4.8 阶段 8：文档

- `firmware/hpm_board/AGENTS.md` 的 EP0 契约表是该契约**唯一副本**，需更新载荷尺寸、新增
  reason、并删掉容差说法（现表述为"用容差比，不要用相等比"——阶段 2 后应改为"比分频器"）。
- `firmware/mc02/AGENTS.md` 用散文描述 EP0 下标与 RS-485 开关，阶段 2/7 后即过时。
- `boards/hpm5321/PINOUT.md:89` 的 UART 表头「逻辑名」不准确（该列其实是 DataId 值）。
  CAN 那节表头是规范的 `接口 | 硬件`。建议改为两列并写明 2.4 的对应关系。

## 5. 验证方法

**每阶段**（`AGENTS.md` 要求：每次修改跑 `clang-format-check` 并至少构建一个相关目标）：

```bash
cd <repo>/hcs_core/src/libhcs
cmake --build host/build                        # host SDK
export GNURISCV_TOOLCHAIN_PATH=~/3rd_party/hpm/rv32imac_zicsr_zifencei_multilib_b_ext-linux
cmake --build firmware/hpm_board/build --target hpm_board_app
cmake --build firmware/mc02/build --target mc02_app
cmake --preset release -S firmware/c_board      # 首次; 本机用 /usr/bin/arm-none-eabi-gcc 13.2
cmake --build firmware/c_board/build --target c_board_app
```

**格式检查不要在本机跑 `--fix`。** CI 用 clang-format 20，本机 `/usr/bin/clang-format` 是 18，
`--fix` 会把无关文件改成 18 的风格（3.4 第 6 条）。本机做相对检查用 VS Code cpptools 自带的
23.1（它与已提交代码在 lint 范围内基本一致），且只检查改动文件：

```bash
CF=~/.vscode/extensions/ms-vscode.cpptools-1.35.2-linux-x64/LLVM/bin/clang-format
$CF --dry-run <改动的 .hpp/.cpp>
```

`host/build` 已配置 `BUILD_EXAMPLES=ON`。host preset 是 `linux-debug`/`linux-release`
（**没有** `release`）。

**实机（台架已就绪）**：两块 hpm5321 须用本次改动的固件**同批**刷写（`kVersion` 已移动），
刷写方法见 `firmware/hpm_board/AGENTS.md`「烧录」。

```bash
./host/build/examples/dual_board_test list      # 应报 2 块板
./host/build/examples/dual_board_test link      # 六链路基线
./host/build/examples/dual_board_test uart 100  # UART 双向对拍
./host/build/examples/ep0_config_test <序列号>  # 单板 EP0 契约: 分频器回报、错分频器被拒并锁存 VerifyFailed
```

**注意**（见 2.3）：`uart` 子命令只证明链路通，**不证明波特率正确**——必须另写只读探针比对
`read_uart_setting(0)` 的回读值。实机验证前 `sudo systemctl stop ModemManager`。

`dual_board_test` **独占两块板**，不能与 `hpm5321_loop_probe` 等遥测工具同时跑。

## 6. 已知盲区与纪律

1. **`ch32_board` 本机不可编**（WCH RISC-V 工具链未装，见 `ENV.md`），且烧录要求拔 USB 线。
   本任务跳过它，但其数据流路径必须保留。
2. **台架只有 hpm5321** —— 阶段 3（c_board）无实机验证手段；mc02 需接入才能跑
   `uart_cross_test`。本机**无** c_board / ch32 硬件，2.2 表中那两列是算术推导不是实测。
3. **无自动化测试**：质量门禁只有 clang-format / clang-tidy / 编译。
4. **clang-tidy 很慢**：只在准备提交前跑一次，且只跑改动涉及的 target。
5. **Agent 严禁 `git add`**（根 `AGENTS.md`）——改动留在工作区作为 review 渠道。
6. **既存的 10 个文件未提交改动**（`canN_transmit` → `can_transmit(CanPort,...)` 统一）
   早于本任务，timestamp 11:25-11:30。其中 `host/include/libhcs/board/mc02.hpp` 与
   `c_board.hpp` 现在**同时含两件事的改动**（`can_transmit` 统一 + 本任务的 EP0/锁/写回），
   提交时须按 hunk 拆开，否则违反"每次提交聚焦单个关注点"；`spec/*/can.hpp`、
   `ch32_board.hpp`、`multi_board.hpp` 与三个 `mc02_*` example 只属于前者。
7. **hpm 链接期 ILM 断言**（`boards/hpm5321/linker/app_flash_uf2.ld` 的 `ASSERT`）不得因
   阶段 0 那样的重构而移动热符号。EP0 在主循环、非每帧路径，理论上不冲突，但要确认断言过。
8. **`Lazy::get()` 不是 `try_get()`**：未初始化时 `get()` 在 release 下返回垃圾指针而非
   null（`firmware/*/app/src/utility/lazy.hpp`）。改 EP0 的 UART 下标解析时注意。

## 7. 遗留待办（2026-09-30 记录）

1. **阶段 6**（4.6）：等 c_board 上板实测后再删数据流配置路径。
2. **c_board 不报 CAN 速率**：`kGetCanConfig` 的速率/采样点恒报 0，`kSetCanConfig` 见非零时序字段即
   STALL，所以 `CanSetting{kClassic1M}` 在 `CBoard` 上构造会失败，只能传速率 0。改法：固件从 bxCAN
   `BTR` 反推实际速率与采样点上报，并据此核对非零字段；线格式不变。
3. **c_board CAN 采样点 78.6%**（CubeMX：`Prescaler=3`、`BS1=10TQ`、`BS2=3TQ`，42 MHz → 1 Mbit/s），
   其余板与电机是 87.5%。42 MHz 下凑不出 87.5%，最近 85.7%：`.ioc` 里 `CAN1/CAN2.BS1=CAN_BS1_11TQ`、
   `BS2=CAN_BS2_2TQ`，由人工改后 Generate。经典 CAN 一般能容忍，优先级低。
4. **`CanConfigPayload::reserved1`**：不是对齐需要（前面正好 16 字节），在指纹版本机制下留空位不省任何
   兼容性。可删（payload 变 16 字节、`kVersion` 自动变、全板重刷），宜与下一次线格式改动合并。
5. **mc02 / c_board 的本轮固件改动只有编译验证**，未上板。
