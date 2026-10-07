# mc02 固件指南

> **文档类型**：现行规范（板级）
> **适用范围**：`firmware/mc02/`，DM-MC02 / CtrBoard-H7（STM32H723VGT6）
> **状态**：现行有效（2026-10-06 精简：只留命令与约束，实测与来龙去脉在下列 L3 文档）
> **相关文档**：[仓库根 AGENTS.md](../../AGENTS.md) · [README.md](README.md)（外设、低延迟设计、Generate 后的复原清单） · [UART_RING_LOG.md](UART_RING_LOG.md) · [PACKET_RATE_LOG.md](PACKET_RATE_LOG.md) · [../hpm_board/SOF_TIMEBASE.md](../hpm_board/SOF_TIMEBASE.md) · [../hpm_board/PITFALLS.md](../hpm_board/PITFALLS.md)

## 摘要

Cortex-M7 @ 550 MHz，CAN-FD 常驻 FD+BRS，热路径放 `.itcm`、状态放 `.dtcm`；USB 只能 Full-Speed。
改之前必须知道：外设配置回 CubeMX 改，每次 Generate 之后按 [README.md](README.md) 末节手工复原。

## 1. 构建与烧录

```bash
cmake --preset debug -S firmware/mc02          # 或 release
cmake --build firmware/mc02/build --target mc02_app mc02_bootloader
```

- ARM 工具链 `cmake/gcc-arm-none-eabi.cmake`。日常烧 app 走 DFU（`0xA511:0x0723`，见仓库根 README）；
  bootloader 首烧 / 调试只用 J-Link，不用 ST-Link / OpenOCD。
- **已有 build 目录时 `option()` 默认值不生效**：改开关显式传 `-D...`，用
  `grep libhcs_APP firmware/mc02/build/CMakeCache.txt` 核对。

## 2. 编译开关（`app/CMakeLists.txt`）

| 开关 | 默认 | 作用 |
|---|---|---|
| `libhcs_APP_USB_RX_XFER_SIZE` | 1024 | 一次 bulk OUT 传输的字节数（64 的倍数），1024 是实测拐点 |
| `libhcs_APP_LOOP_BALLAST_CYCLES` | 0 | 测量仪器：每圈主循环忙等周期，出厂永远 0 |
| `libhcs_APP_DEBUG_KNOBS` | OFF | 调试器可写的调优旋钮（`diag/knobs`） |
| `libhcs_APP_USB_RX_HIST` * / `LOOP_PROFILE` * / `CAN_DIAG` * | OFF | 诊断输出，三者都占 `DataId::kUart0`，互斥 |

- 已删除、不要加回：`USB_DWC2_DMA`、`UART_RX_IN_ISR`（`PACKET_RATE_LOG.md`）、`IMU_ENABLE`（IMU 由声明决定）、
  `RS485_ENABLE`（UART2/3 永远编入）、`APP_TIME_SYNC`（时间基准由声明决定）。旧 cache 里的这些变量已无作用。
- 包率对主循环周期强非单调（±35%），USB 性能 A/B 不能只测一个工作点（`PACKET_RATE_LOG.md` 第 1 节）。

## 3. 配置即声明（EP0，核心在 `core/src/link/ep0.hpp`，本板 `app/src/usb/vendor_control.cpp` + `app/src/ports.hpp`）

- **会话胶水在 common（2026-10-06）**：kStart/kKeepalive 应答、租约、断联下线、kTimeAnchor 应答与批量缓冲宿主
  （`deserializer_/serializer_/transmit_buffer_`）在 `firmware/common/app/src/link/host_session.hpp` 的
  `link::HostSession` 模板（三板同一份；本文件里的旧副本已删）。本板 Vendor 只剩传输形态、EP0 门
  （`session_allowed()`）、下行分发钩子（`dispatch_*`，经 Registry）与 TimeSync 策略。
- **上电时所有口都是停的**：FDCAN 留在 INIT，UART 不武装 DMA，IMU 芯片不碰、数据就绪线屏蔽，蜂鸣器与
  GPIO 通道不开。清单（`kApplyManifest`）声明了才启动；会话结束 / 挂起 / 重新枚举时
  `Vendor::stop_channels()` 全停（发送环里已收下的字节照常发完）。
- 口按 DataId 寻址，丝印号即身份（UART1/2/3/7/10、DBUS=`kUartDbus`、`kImu`、`kGpio` 的四根线、`kBuzzer`）。
  身份只有一处来源：`spec/mc02/` 的具名描述符。**"按口列一遍"只有注册表一份**：停口
  `Registry::suspend_all()`、下行分发 `Registry::dispatch<类型>()`、串口轮询 `ports::poll_uarts()`；新增口只改口表、
  驱动对象、绑定三处，不要手写 switch 或下标表。
- CAN：TX 帧型可经 EP0 切（只改 Tx 元素 FDF/BRS，不进 INIT）；仲裁/数据段速率与采样点是**核对不是配置**。
- UART：声明必须给全速率、字长 7/8、校验、停止位、接收极性（缺项 `kConfigErrorIncomplete`），先全量校验后统一
  提交；不提供 9 位字长。速率一致性比 `BRR` 整数。接收极性走 `CR2.RXINV`（DBUS/SBUS 正常、iBUS 反相，
  主机 `Mc02::configure_dbus()`）。
- 重放清单不碰设置没变的口（串口、IMU、GPIO）：接手或回滚之后挂起过的口才整份重写。
- `handle_downlink()` 里的 `started_` 判断是没声明口的最后一道，不要删。
- 改 `CR1` / `CR2` / `BRR` 这类只能在 `UE=0` 时写的寄存器：**先存进来时的 `CR1`，最后原样写回**，不要从新值里
  掩掉 `UE`（`PACKET_RATE_LOG.md` 4.5）。

## 4. 主循环（`app/src/app.cpp` 的 `App::run()`）

- 每趟固定只做：看 USB 事件队列（有才 `tud_task()`）、看毫秒翻没翻（翻了才做会话租约 / DFU / LED / CAN 卡死
  守护）、泵一次上行、读一次位图 `loop::active`（`utility/loop_work.hpp`，位号 = DataId）。**不要加无条件的调用**；
  "用到才跑"的东西往位图里加一位，**置位放在对象自己的 `start()` 里**。
- 每圈要读写的状态放 `.dtcm`，不要放 `.d2_sram`，也不要放 `.data` / `.bss`（AXI SRAM 前 32 KB 非缓存）。

## 5. CAN

- 协议与速率由对端电机决定（1M/5M），优化不要往"升 FD / 提波特率 / 改采样点"走。
- 下行帧直写硬件 FIFO，只有 FIFO 满才进 64 深的软件队列（队列非空时必须让路）。
- DAR 发送请求卡死（ES0491 §2.22.3）由 `Can::recover_stuck_transmits()` 每毫秒守护：挂起超 20 ms 且非 bus-off
  就取消释放槽位，刻意不重发。不重传是用户决定（`../hpm_board/PITFALLS.md` 第 9 节）。
- 中断入口不碰 FLASH：向量表启动时复制进 DTCM（`relocate_vector_table()`，`.dtcm` 复制之后、开中断之前），
  三路 FDCAN 第 0 中断线指向 ITCM 的 `line0_isr<>`。`stm32h7xx_it.c` 的 `FDCANx_IT0_IRQHandler` 仍由 CubeMX
  生成，不要删。热路径（`handle_uplink/handle_downlink`、发送队列排空）在 `.itcm`。

## 6. USB 与 UART

- USB Full-Speed，聚合上限约 800 KB/s：1-2 条 CAN 总线跑满时卡在 CAN 线速，3 条才卡在 USB。
- **下行不背压**：`CFG_TUD_VENDOR_RX_MANUAL_XFER` 不设；移植来的背压代码与 `DOEPCTL.SNAK` 写入已删，不要加回。
- UART：每口一条整环 circular DMA，写指针由主循环读 `NDTR` 推导；DMA 缓冲在 `.d2_sram`（MPU 非缓存），端口对象在
  `.dtcm`。**不要给 UART DMA 开 FIFO/burst**（`NDTR` 会算错）。接收 DMA 的 TC/HT 中断关掉、错误中断保留。
- 错误策略：`CR3.OVRDIS=1`、**`CR3.DDRE=0`**（置 1 一个坏字符就永久杀死端口）、`CR3.EIE=0`（`UART_RING_LOG.md` 第 1 章）。
- 转发按时间：最早未发的字节等满 250 µs 就转发（`rx_buffer.hpp` `kHoldCycles`）；`idle_delimited` 包后静默两个
  字符时间（`tx_buffer.hpp`）。
- **不要用 `HAL_RCCEx_GetPeriphCLKFreq()` 取 UART 内核时钟**（本版 HAL 对两个 UART 组返回 0）：用
  `UART_GETCLOCKSOURCE()` 再按时钟源取频率（HSI 要按 `RCC_FLAG_HSIDIV` 右移）。别照抄 c_board 的写法。
- C++ 层已无可测优化余量，不要重开（数据见 `PACKET_RATE_LOG.md`）；能改变数字的杠杆在应用层批量与主机调度。

## 7. CAN 帧时间戳与共享时间基准（机制与实测见 `../hpm_board/SOF_TIMEBASE.md` 8.7）

- **时间基准是声明，不是编译开关**（v14）：主机 `hcs::Configuration::enable_time_sync()`。第一次开时接通 TIM5
  的 SOF 捕获、标定 TIM3 <-> TIM5（中断关着至多 0.5 ms），每次开都复位时间基准与 SOF 环；关着时 SOFM 不开，
  `__wrap_dcd_int_handler` 与 CAN 接收中断只多读一次 `sync::time_sync_on()`（DTCM）。`stop_channels()` 即关。
- 定时器分工（.ioc 是唯一来源，固件只检查不改写）：TIM5（PSC 0，ARR 0xFFFFFFFF）= SOF 捕获（ITR7）与整条时间轴；
  TIM3（PSC 1，ARR 65535）= FDCAN 外部时间戳，CH4 = PB1 IMU 加热（不开）；TIM23（PSC 274）= 板上 1/4 µs 时间戳；
  TIM2 只做 PWM。`MX_TIM3/5/23_Init()` 上电总调。
- **TIM3 与 TIM5 的 CNT / PSC / ARR / UG 运行时一律不碰**：两者的整数关系（`core/src/time/counter_link.hpp`）每毫秒
  抽查，一破就停打戳。加热 PWM 若要开，只写 CCR4。
- SOF 锁存值按帧号扣掉全速填充位（`core/src/time/usb_sof_bits.hpp`），时间基拟合同样扣。`stamp_of()`、
  `window_covers()` 在 ITCM，没有库调用。
- 对 5321 的 +206 ns 常数不补偿（混着轴偏移与 CAN 接收延迟差，8.7.9）。

## 8. CubeMX 纪律

配置改在 `.ioc`，人工 Generate；禁止直接改 `bsp/cubemx/Core/`。Generate 之后按 README 末节复原（删重新生成的
`int main(void)`、`static void MPU_Config` 改回 `void MPU_Config` 等）。
