# c_board 固件指南

> **文档类型**：现行规范（板级）
> **适用范围**：`firmware/c_board/`，RoboMaster C 板 / DJI C-type（STM32F407IGH6）
> **状态**：现行有效
> **相关文档**：[仓库根 AGENTS.md](../../AGENTS.md)（共享约束） · [仓库根 README.md](../../README.md)（烧录流程）

> 本目录专属指南，叠加在仓库根 `AGENTS.md` 之上（根为共享约束，此处只写 c_board 专属）。

## 摘要

c_board 是四块板里最常规的一块：单核 Cortex-M4F、ARM 工具链、CubeMX 生成 BSP、
app 与 bootloader 两套独立镜像。改这块板的代码注意两件事——**外设配置必须回到
CubeMX 改**（文末 CubeMX 纪律），**DMA 目标缓冲必须放 `.dmaram` 非缓存段**（见
下方「ART 与 DMA 内存布局」）。DFU 烧录流程与其他 STM32 板一致，见
[仓库根 README.md](../../README.md#烧录-appusb-dfu)。调试与 bootloader 首烧**只用
J-Link**，不用 ST-Link / OpenOCD（GDB / Ozone 走 `tools/jlink-debug.sh` /
`tools/ozone-debug.sh`）。

## 芯片与工具链
- MCU：**STM32F407IGH6**（RoboMaster C 板 / DJI C-type），Cortex-M4F。
- ISA/工具链：ARM，`cmake/gcc-arm-none-eabi.cmake`，需 `arm-none-eabi-gcc`（**不是** RISC-V）。

## 构建
```bash
cmake --preset debug -S firmware/c_board
cmake --build firmware/c_board/build --target c_board_app c_board_bootloader
```
- preset：`debug` / `debug-outside` / `release`。`debug-outside` 置 `HOST_DEBUGGER=ON`（外部调试器场景）。
- target：`c_board_app`、`c_board_bootloader`。

## EP0 配置通道 [2026-09-30 接入，未上板]

协议与其他板同一套（`core/include/libhcs/protocol/vendor_control.hpp`），板端在
`app/src/usb/vendor_control.cpp`，契约表见 [hpm_board/AGENTS.md](../hpm_board/AGENTS.md)「EP0 配置通道」。

**会话状态机在 core（2026-10-05），会话胶水在 common（2026-10-06）**：kStart 的 nonce 握手
与 keepalive 租约走 `core/src/link/session.hpp` 的 `core::link::Session`（三块板与主机测试
同一份）。kStart/kKeepalive 的应答、租约检查、断联下线（suspend_all + 忘 EP0 门 + 停时间
基准）、kTimeAnchor 应答与上行批缓冲宿主（`deserializer_/serializer_/transmit_buffer_`）
在 `firmware/common/app/src/link/host_session.hpp` 的 `link::HostSession` 模板里，本板
Vendor 只剩传输形态（try_transmit 分块/ZLP）、EP0 门、下行分发钩子（`dispatch_*`）与
TimeSync 策略（捕获计数来自 SOF 捕获，见下文时间基准一节）。统一时修掉本板的两处漂移：租约 1 s 是 v10 之前的遗留
（现为 4 s）；租约到期此前**不**清 EP0 握手门（只在总线事件里清），不重新插拔换主机程序
会让从不握手的新主机继承上一台的门——现在到期与总线事件同一条 `deactivate_session()` 路径。
租约检查也移出 `try_transmit()`（原先每趟读 48 位定时器），进主循环毫秒杂务。上行批缓冲是
三板共用的模板 `firmware/common/app/src/usb/interrupt_safe_buffer.hpp`
（`usb::UpstreamBatches` 别名），满报警统一为"有会话才点 LED"。

**主循环与 mc02 同一结构**（2026-10-07）：`app/src/utility/loop_work.hpp` 的 `loop::active` 位图，
位号 = DataId；各口在 `resume()`/`start()` 时置位、停口时清位（串口停口后发完才清，见
`ports.hpp` 的 `poll_uarts()`）。主机没声明的口在主循环里不占指令。CAN 的身份来自 `can.hpp`
的 `kCanPorts` 表（`ports.hpp` 用 `static_assert` 钉住与口表同序）。各口的 `describe()` 报真实
启停（EP0 `kGetPortList` 的 `kPortRunning`），不再写死 running。`[只过编译]`

本板差异只有三条：

- **串口对应**：DBUS = `DataId::kUartDbus`（huart3）、UART1 = `kUart1`（huart6）、UART2 = `kUart2`
  （huart1）；EP0 按 DataId 寻址（v10 起），没有下标。
  速率一致性比 `BRR` 整数，USART1/USART6 走 APB2 84 MHz、其余 APB1 42 MHz。
  F407 的 USART 没有 RX 反相位：`rx_polarity` 恒报 1（正常），请求 2（反相）即 STALL
  `kConfigErrorFramingUnsupported`。DBUS 口只能接线上反相的 DBUS/SBUS，iBUS 要接 UART1/UART2。
- **CAN 能力位全清**：bxCAN 无 FD，清单声明只接受 classic。仲裁段速率/采样点由
  `kGetPortConfig` 从 `BTR` 反推上报（1 Mbit/s、785‰），SET 中非零即核对、不符 STALL；
  数据段字段恒报 0，SET 中须为 0。
- **没做 EP0 握手的主机开不了 session**（与 hpm/mc02 同一道门）：主机 `CBoard` 在重连钩子里
  读 `kGetPortList`/`kApplyManifest`（v10 起，与另两块板共用核心 `core/src/link/`）；
  旧版 SDK 连新固件会在 `SESSION_ACK` 超时。

带内 `kUart1Config` / `kUart2Config` / `kUartDbusConfig` 与 `Uart::handle_config()` 已于
2026-10-04 删除（[UART_EP0_MIGRATION.md](../../UART_EP0_MIGRATION.md) 4.6）；运行期改设置用
主机板类的口句柄 `handle(Spec::kUarts.kUartN).configure(...)`（整份声明重放）。

**没声明的口不工作、不占资源**（2026-10-07 起，与另两块板一致）：上电与会话结束时 CAN 停在 INIT 模式
（不上总线、不应答）；串口关接收器（`CR1.RE`）、接收 DMA 与中断都停；IMU 的两条数据就绪线（PC4、PC5）
在 EXTI 里屏蔽、温度不探测，不发起 SPI 读。清单声明了才 resume；下行发往没声明的口由驱动丢弃。CAN 的位时序与
IMU 的挡位是编译期预设，清单里这两项只核对。c_board 只有经典 CAN：与别的板同挂一条线时那条线只能跑
经典帧（它离线时也一样，见 [core/PORT_STATUS.md](../../core/PORT_STATUS.md) 第 4 节）。

**板载蜂鸣器是一路口**（DataId `kBuzzer`，2026-10-07 起，与 mc02 同一驱动
`firmware/common/app/src/buzzer/buzzer.hpp`；本板 binding 在 `app/src/buzzer/buzzer.hpp`）：
PD14 / TIM4 CH3，.ioc 给 1 MHz 计数、4 kHz 周期。声明了才开通道，下行一条记录一个音，
会话结束静音。`[只过编译]`

**PWM 排针是一路 GPIO 口**（DataId `kGpio`，`app/src/gpio/gpio.hpp` 的 `Gpio`；v12 2026-10-05 起
七个引脚是它的七根线 `Spec::kGpios.kPwm1`..`kPwm7`，逐线声明）。清单项是真声明：没声明的线不碰（定时器通道、EXTI 线都不动），
声明成输出或输入才配置，会话结束时输出拉低、输入停采样。PWM6（PI6）与 PWM5（PC6）共用 EXTI6，
`Gpio::kLineCapabilities` 里 PWM6 没有 `kGpioCapReadInterrupt`，声明它的边沿输入会被拒
`[仅编译验证，本机无 c_board]`。

## 共享 SOF 时间基准（2026-10-06 移植自 mc02，未上板）

机制与实测记录见 [hpm_board/SOF_TIMEBASE.md](../hpm_board/SOF_TIMEBASE.md)，`sync/`
两个模块（`sof.{hpp,cpp}`、`timebase.{hpp,cpp}`）的文件头注释载明本板差异。要点：

- **接线与 mc02 同一套**：`--wrap=dcd_int_handler`（app/CMakeLists.txt）把钩子插进
  USB 中断，`OTG_FS_IRQHandler -> tusb_int_handler() -> dcd_int_handler()` 的调用链
  在链接期改道，生成的 `stm32f4xx_it.c` 不被触碰；SOF 状态位在钩子里消费，
  `dcd_int_handler()` 看不到它。时间基准由 EP0 清单声明开/关
  （`vendor_control.cpp` 的 `kBoardCapTimeSync` + `set_time_sync`），随会话结束关闭
  （`vendor.cpp` 的 `deactivate_session()`）。
- **寄存器路径**：OTG_FS 与 mc02 的 OTG_HS 同为 DWC2——`GINTSTS.SOF`、
  `GINTMSK.SOFM`、`DSTS.FNSOF`（全速下数帧，乘 8 折算到 16384 微帧轴）、
  `DSTS.ENUMSPD`。恒全速（无高速 PHY），高速分支仅为防复用错倍数而保留。
- **硬件 SOF 捕获走 TIM2 ITR1 + DMA**：F407 上 OTG_FS 的 SOF 只引到 TIM2 的 ITR1
  （`TIM_TIM2_USBFS_SOF`），TIM2 又是全板 1/4 us 时间戳源，它的捕获值太粗；所以 TIM2 CH1
  （TRC）的捕获只触发 DMA1 Stream5，在边沿那一刻抄下 84 MHz 的 TIM7（.ioc：Prescaler 0、
  Period 65535），中断里用它扣掉入口延迟。TIM7 的配置不对就退回纯中断时间戳。
  `kTimeStatus` 的 capture_fresh/stale_count 数的就是它。设计与未验证项见 `sync/sof.cpp`
  文件头 `[2026-10-07 只过编译]`。
- **本板没有 CAN 帧时间戳**：bxCAN 无帧硬件时间戳，上行 CAN 帧不带 `SofStamp`。
  kTimeAnchor 应答与微帧-本地钟双向换算照常可用（`vendor.hpp`）。
- **DWT->CYCCNT 须先使能**：App 构造函数里 `DEMCR.TRCENA` + `CYCCNTENA`（与 mc02
  同位置）；拟合在 168 MHz 的 CYCCNT 上做（21000 cycles/microframe 恰为 500
  quarter-us），不在 250 ns 量化的 TIM2 上。调试器挂接会停走 CYCCNT，时间基准失效
  到复位为止——测时间基别连 J-Link。
- ROS 侧在板卡配置 yaml 加 `time_sync: true` 即请求（主机按 kBoardCapTimeSync 能力
  位核对）；本板的时间线可用于反馈换算，但 CAN 帧没有跨板时间戳可比。

## 目录结构
- `app/`、`bootloader/`：两套独立镜像（C++ libhcs 层）。
- `bsp/cubemx/`：CubeMX 生成产物（含链接脚本/时钟/外设初始化）。
- `bsp/`：`cmsis-core`、`cmsis-device-f4`、`stm32f4xx-hal-driver`——第三方，视为只读。`SEGGER`（RTT）与 `tinyusb` 已上移到 [firmware/common/bsp/](../common/bsp/)，各板共用一份（见 [firmware/common/AGENTS.md](../common/AGENTS.md)）。
- `app/src/utility/` 里只剩板级事实（`boot_mailbox.hpp`、`assert.cpp`、`loop_work.hpp`）；`lazy.hpp` /
  `ring_buffer.hpp` / `interrupt_lock.hpp` 已统一到
  [firmware/common/app/src/utility/](../common/app/src/utility/)（2026-10-05）。
  公共 `interrupt_lock.hpp` 按架构分支，ARM 版带 mc02 的编译器屏障——本板旧锁没有
  屏障，临界区内的访存可能被优化器挪出，统一时已取带屏障的版本 `[仅编译验证]`。
- `app/src/uart/` 与 mc02 同构（2026-10-05）：`uart.hpp` 的 `UartCommon`（身份、EP0
  速率/帧格式原语与读回）+ 全双工 `Uart`，收发缓冲各一份文件（`tx_buffer.hpp` /
  `rx_buffer.hpp`）。行为不变：F407 没有 TXFIFO/ICR/RQR/RXINV，接收门槛 32 字节是
  中断粒度，`kIdleGapCharacters = 2 + 2` 从 DMA 完成计时——这些都保留，与 mc02 的
  差异只在注释写明的硬件事实上。三个口（含 DBUS）同为 `Uart`：DBUS 的 TX DMA 流
  CubeMX 已接好，只是协议不往 kUartDbus 路由下行。

## ART 与 DMA 内存布局 [2026-10-07 接入，只过编译]

- D-cache 已开启（`app/src/app.cpp` 的 `App::App()`：MPU 区域 0 + FLASH 预取 / I-cache /
  D-cache 三个使能位）。`FLASH_ACR` 三位复位值为 0，CubeMX 只写 LATENCY——缓存与 MPU
  相关配置改在 app.cpp，不进 CubeMX。
- **新增 DMA 目标缓冲必须放 `.dmaram` 段**：独立全局对象 +
  `[[gnu::section(".dmaram")]]`，经构造参数把缓冲引用注入控制器对象（段属性不能放在
  非静态数据成员上，GCC 直接拒绝）。该段落在 0x20018000 起 32K（SRAM1 尾部 + SRAM2），
  由 MPU 区域 0 设为非缓存（TEX1/C0/B0），DMA 与 CPU 双向直通；放别处的 DMA 缓冲在
  D-cache 下会读到旧值。样例：`uart.hpp` 的 `UartDmaMemory`、`spi.hpp` 的
  `Spi::DmaMemory`、`sof.cpp` 的 `g_sof_edge_tim7`。
- CAN 转发热路径在 `.RamFunc` 段（`can.cpp` 的 `libhcs_RAMFUNC`，随 `.data` 由启动
  代码拷进 SRAM 零等待执行；F4 没有 ITCM、CCM 不能取指）。**属性必须写在
  `extern "C"` 之后**——写在前面会被 GCC 静默忽略，函数留在 FLASH。
- `.dmaram` / `DMARAM` 区域与 `*(.RamFunc)` 的段定义在 `bsp/cubemx/STM32F407XX_APP.ld`
  （本次按用户明确要求由 AI 编辑；日常仍守下节纪律）。bootloader 只开预取 + I-cache
  （`bootloader/src/main.cpp`）：不开 D-cache——flash 编程后回读会命中旧缓存行，且
  bootloader 没有 DMA 流量。

## CubeMX 纪律（本板适用）
- 外设/时钟/中断/DMA 配置改在 CubeMX，源为 `bsp/cubemx/c_board_slave.ioc`；AI 只指出改哪个 `.ioc` 字段，由人工 Generate，**禁止**直接改 `bsp/cubemx/Core/` 等生成代码。
- 每次 Generate 之后跑 `.scripts/patch_cubemx`：把 `cmake/stm32cubemx/CMakeLists.txt` 里的本机 STM32Cube 绝对路径换回仓库的 `${libhcs_STM32_*}` 变量，删掉 CubeMX 多生成的 `.mxproject`、`CMakeLists.txt`、`CMakePresets.json`、`STM32F407xx_FLASH.ld` 等。跑完 `git diff` 里这个 CMake 文件应当没有改动。
- `.ioc` 与手维护 `*.ld` 仅在用户明确要求时方可由 AI 编辑。
