# mc02（DM-MC02 / CtrBoard-H7）

> **文档类型**：背景说明（外设、低延迟设计、CubeMX 复原清单）
> **适用范围**：`firmware/mc02/`，达妙 DM-MC02 板（STM32H723VGT6）
> **状态**：现行有效
> **相关文档**：[AGENTS.md](AGENTS.md)（命令与约束以那份为准） · [UART_RING_LOG.md](UART_RING_LOG.md)（UART 环形缓冲实录与实测数据） · [仓库根 README.md](../../README.md)（DFU 烧录流程） · [仓库根 AGENTS.md](../../AGENTS.md)

## 摘要

本文件讲 mc02 这块板**有什么外设、为什么这么设计**，以及一份必须照做的操作清单：
**每次 CubeMX 重新 Generate Code 之后要手工复原哪些改动**。日常构建命令见
[AGENTS.md](AGENTS.md)，烧录见[仓库根 README.md](../../README.md#烧录-appusb-dfu)。

mc02 是达妙 DM-MC02 板（**STM32H723VGT6**，Cortex-M7 @ 550 MHz，LQFP100）上的
CAN/UART <-> USB 转发固件。它与其他板共用 `core/` 和主机 SDK；本目录只放板级固件
（`app/` + `bootloader/`）与 CubeMX BSP（`bsp/cubemx`）。

## 本文导航

| 章节 | 内容 |
|---|---|
| [外设一览](#外设一览) | CAN/UART/USB/IMU/LED 分别挂在哪 |
| [为什么 USB 只能跑 Full-Speed](#为什么-usb-只能跑-full-speed) | 封装限制，非软件问题 |
| [低延迟设计](#低延迟设计) | 时钟、CAN-FD、中断优先级、ITCM 热路径、UART 环与 D2 SRAM |
| [重要：每次 CubeMX Generate 之后](#重要每次-cubemx-generate-code-之后) | **必做**的复原清单 |
| [已配置但未启用的外设](#已配置但未启用的外设) | 为什么它们不影响转发延迟 |
| [引脚与 DMA 余量](#引脚与-dma-余量) | 加外设前先看这里 |
| [构建](#构建) | 编译命令 |

## 外设一览

| 功能 | 外设 | 说明 |
|---|---|---|
| CAN x3 | FDCAN1/2/3 | **CAN-FD + BRS**，仲裁段 1 Mbit/s、数据段 5 Mbit/s |
| UART | USART1（UART1）、USART2（UART2 RS-485）、USART3（UART3 RS-485）、UART7、USART10（UART10）、UART5（DBUS） | DataId 就是丝印号；接收走 `ReceiveToIdle_DMA` |
| USB | OTG_HS 工作在 **Full-Speed** 模式（片内 FS PHY，PA11/PA12） | 原因见下节 |
| IMU | BMI088，挂 **SPI2** | 加速度计片选 PC0，陀螺片选 PC3_C，中断 PE10/PE12 |
| LED | WS2812，挂 SPI6（TX + BDMA） | 可寻址状态灯 |

## 为什么 USB 只能跑 Full-Speed

H72x/H73x 的片内 USB HS PHY **只在 LQFP144 / UFBGA176 封装上引出**。本板用的是 LQFP100，
要跑 USB HS 就必须外挂 ULPI PHY，而这块板没有。所以 USB 被限制在 12 Mbit/s 的
Full-Speed，**转发吞吐的杠杆在 CAN 侧（CAN-FD），不在 USB 侧**。

## 低延迟设计

- **FDCAN 内核时钟 = 80 MHz**，来自 PLL2（`PeriphCommonClock_Config`）。这是唯一能整除
  出精确 5 Mbit/s 数据段的取法（24 MHz 的 HSE 做不到）。
- **CAN-FD 帧类型跟随总线，且可由主机切换**：控制器**常驻** FD+BRS 模式（它是经典
  CAN 的超集，收方向不受影响）；发送帧型默认取 `can.hpp` 的 `kCanPorts` 表（三条总线
  全部 FD），主机可经 EP0（`usb/vendor_control.cpp`）在构造期或运行时切换——应用就是
  改 Tx 元素的 FDF/BRS 标志，**不进 INIT 模式、不重新初始化**（EP0 v2 的
  `kSetCanConfig` + `kCanConfigApply` 位，读回经 `kGetCanConfig`；hpm 板没有这个能力
  位，对它们该请求退化为断言）。payload 里的仲裁/数据段速率字段永远是核对不是配置：
  速率由对端电机硬件决定 `[硬件事实，用户确认 2026-09-12]`。
  每帧 `is_fdcan` 位已废弃 [2026-09-12]，见 `core/src/protocol/protocol.hpp` 的
  `CanHeaderLayout`；下面的逐帧切换描述仅作历史保留：

  > **历史 [2026-09-12 废弃]**：2026-09-12 之前，每一帧的格式通过发送元素的 FDF/BRS 位、
  > 按主机下发的 `is_fdcan` 标志逐帧决定。该位在线协议中已删除，主机不能再逐帧选择帧类型
  > （改为按总线经 EP0 配置）；对端发来的 classic 帧仍照常接收（FD 是超集）。
- **硬件接收时间戳：已禁用**（代码已注释保留）。FDCAN 内部计数器只有 16 位，1 tick =
  1 个标称位时间 = 1 us @ 1 Mbit/s，约 65.5 ms 就回绕，无法满足 `CanDataView::timestamp_us`
  约定的 32 位微秒语义（主机端按 32 位回绕做差分会周期性算出负值）。上行不再携带该字段，
  每帧省 4 字节。若要恢复，需同时放开 `can.hpp` 的
  `HAL_FDCAN_ConfigTimestampCounter/EnableTimestampCounter` 与 `can.cpp` 的赋值，并先把
  值补宽（例如在 ISR 内用自由运行的 TIM5 微秒计数器补齐高位），否则主机侧仍不可用。
- **Bus-off 自动恢复**：`HAL_FDCAN_ErrorStatusCallback` 里清 `CCCR.INIT`。
- **NVIC 优先级**（数值越小优先级越高）：FDCAN **1** > USB **2** > UART/DMA **3**，
  保证电机反馈（CAN RX）永远不会被大块 USB 传输或 UART DMA 拖延。USB 的那一档
  由 `Vendor::Vendor()` 显式设置：TinyUSB 的 `dcd_int_enable` 只调 `NVIC_EnableIRQ`
  不设优先级，而会设优先级的 CubeMX `HAL_PCD_MspInit` 属于 ST 的设备栈、本固件不链接
  它——不显式钉住的话 OTG_HS 会停在复位值 0，反压在 FDCAN 之上。
- **ITCM 热路径**：`can.cpp` 里标了 `libhcs_ITCM` 的 CAN 转发函数，启动时（`App::App()`）
  从 FLASH 拷进零等待 ITCM，把 I-cache/XIP 取指抖动从最坏情况里去掉。链接脚本另按
  mangled name 收进每帧都会调到的叶子函数：`Serializer`、`Bitfield`、
  `InterruptSafeBuffer`、`RingBuffer`、`get_serializer` 和几个 `Lazy` 取值器——否则它们
  留在 FLASH，每次调用都要走一条长跳 veneer（ITCM 在 0x0，FLASH 在 0x08040000，远超 BL
  的跳转范围）。`release`(-O3) 下其中只有 `InterruptSafeBuffer::allocate` 和
  `get_serializer` 是独立函数，其余全被内联，这些规则主要在 `debug`(-Og) 下起作用。
  见 `bsp/linker/STM32H723VGTx_APP.ld`——那里的注释说明了为什么 `.itcm` 必须排在
  `.text` 前面，以及哪些东西**不能**收进去（启动期就会执行的代码，例如
  `Lazy<App>::init`）。`[实测 2026-09-11，arm-none-eabi-nm]`
  - **2026-09-11 更正**：这些名字规则原先写的前缀是 `_ZN7libhcs`，但 `libhcs` 只有 6 个
    字符，实际符号是 `_ZN6libhcs...`，所以**一条都没有匹配上**。本条原先"ISR 路径只剩
    `memcpy` 和 assert 失败路径在 FLASH、占用约 5.6 KB"的说法对当前代码也不成立：修正前
    `release` 的 `.itcm` 只有 1792 B，`allocate` 与 `get_serializer` 都在 FLASH。修正后
    `.itcm` 为 1960 B（`debug` 为 7408 B），并在 `.itcm` 之后加了一条 ASSERT：名字规则再
    失配就直接链接失败。`[实测 2026-09-11]`
  - **仍在 FLASH、每帧都经过的**：CAN 接收中断入口 `FDCAN1/2/3_IT0_IRQHandler` 与
    `HAL_FDCAN_IRQHandler`（合计 960 B；到 `HAL_FDCAN_RxFifo0Callback` 才进入 ITCM），
    以及 `memcpy`（`.itcm` 内 3 处静态调用走 veneer）。`[实测 2026-09-11]`
- **UART 接收 = 一条永不停的环形 DMA**：每个口一条 circular DMA 盖住整个 2048 字节环，
  `CR3.DMAR` 从初始化到掉电一直置位，**没有任何窗口是关着接收的**。写指针不由中断维护，
  消费者在主循环里读 `NDTR` 推导（`2048 - NDTR`）。中断只剩 IDLE（一条
  `idle_count_.fetch_add`）和 DMA 错误两条。相比原来的 32 字节双 bank 方案：ISR 容忍窗口
  从 32 字节时间放大到 2048 字节时间，RX 中断率从 ~9000/s/口 降到按帧率，并且**去掉了主
  循环之外唯一会全局关中断的地方**（bank 记账要读写多个 stream 寄存器，原本跑在
  `__disable_irq()` 里，挡的正是优先级 1 的 FDCAN）。细节与实测见
  [UART_RING_LOG.md](UART_RING_LOG.md)。
- **UART 的 DMA 环放在 D2 SRAM**（`.d2_sram`，0x30000000）。DMA1 是 D2 域主设备，环放在
  D2 SRAM 就不必跨 D2→D1 互联去写 AXI SRAM、也不和 M7 自己的 AXI 访问抢总线。附带效果更
  实在：2026-08-12 那轮三路 TTL 加 DBUS 四个端口对象共 18.7 KB，原本占在 `.data` 里，搬走后 AXI 的 `.data+.bss` 从
  26160 降到 7024 字节，链接脚本尾部那条 32 KB 非缓存 MPU 窗口的 ASSERT 余量从 6.5 KB
  变成 25 KB。**MPU region 1 在 `app.cpp` 里配**（不是 CubeMX 的 `MPU_Config()`），所以
  重新 Generate 不会覆盖、也不进复原清单。
- **不要把 DMA 缓冲移进 `.dtcm`**（像 `can.hpp` 放 CAN 对象那样）：DTCM 只有内核够得到，
  DMA 流指向它会静默地什么也不搬。

## 重要：每次 CubeMX "Generate Code" 之后

`app/src/app.cpp` 提供自己的 `main()`，并直接驱动生成出来的 `*_Config()` / `MX_*_Init()`
函数，因此对生成文件做了少量手工改动。**CubeMX 重新生成会把这些改动冲掉**，需要逐条
重新施加：

1. **`Core/Src/main.c`**：删除重新生成出来的 `int main(void) { ... }` 整块
   （真正的入口是 `app.cpp`，而且它用的是 TinyUSB，不是 `MX_USB_DEVICE_Init`）。
2. **`Core/Src/main.c`**：把 `static void MPU_Config` 改回 `void MPU_Config`
   （函数声明和定义两处都要改）。
3. **`Core/Inc/main.h`**：`SystemClock_Config` / `PeriphCommonClock_Config` /
   `MPU_Config` 三个声明放在 `USER CODE BEGIN EFP` 区块内，正常情况下能在重新生成后
   保留下来——但仍需确认它们还在。

然后在 CubeMX 图形界面里核对以下几项（它们来自 `.ioc`，通常会保留，但要确认）：

- 时钟树里 **FDCAN = 80 MHz**（来自 PLL2）。
- FDCAN1/2/3 的 `FrameFormat = FD_BRS`，数据段 5 Mbit/s，标称段 1 Mbit/s。
- FDCAN **元素数据长度保持 8 字节**——`MessageRAMOffset` 那两个值（`0x200` / `0x400`）
  是按 8 字节元素（16 B，即 4 词）算的；改成 64 字节元素会导致区域重叠。
- SPI2 波特率 <= 10 MHz（BMI088 的上限）；当前分频系数 32（约 5.7 MHz）。

> 相关约束见[仓库根 AGENTS.md 的 CubeMX BSP 修改纪律](../../AGENTS.md#cubemx-bsp-修改纪律)：
> 生成目录禁止直接编辑，配置改动一律回到 `.ioc` / CubeMX。

## 已配置但未启用的外设

`.ioc` 里配了一批当前固件用不到的外设（为将来的 LCD、摄像头、更多串口留位）。
它们**不影响转发延迟**，判断规则只有一条：

> **看 `app/src/app.cpp` 有没有调它的 `MX_xxx_Init()`。**

外设的时钟使能（`__HAL_RCC_USART2_CLK_ENABLE()`）、引脚 AF、NVIC 优先级、DMA 通道配置，
**全都在 `MX_xxx_Init()` 及它调用的 MspInit 里面**。没调用 → 时钟门控关闭 → 那个外设在
硅片上等于不存在：不耗电、不占总线、不产生中断。而且 `-ffunction-sections` +
`-Wl,--gc-sections`（见 `cmake/gcc-arm-none-eabi.cmake`）会把没人引用的 `MX_*_Init`
整段从 FLASH 里剪掉——用 `arm-none-eabi-nm` 查 ELF 可以确认符号表里只有被调用的那些。

当前**已配置但 `app.cpp` 未调用**：DCMI、SPI1（LCD）、UART8、UART9、TIM3、TIM12。
USART2 / USART3 在 `libhcs_APP_RS485_ENABLE`（默认 ON）时会调 `MX_USART2_UART_Init` / `MX_USART3_UART_Init`；开关关掉时它们也落进这一类。

### USART2 / USART3：丝印 UART2 / UART3（RS-485）

这两个口是外壳上的 UART2 / UART3，DataId 就是 `kUart2` / `kUart3`。
默认镜像初始化它们（`libhcs_APP_RS485_ENABLE=ON`，和 IMU 同一类开关）。关掉时不调 `MX_USARTx_UART_Init`、不构造 D2 对象、主循环不 poll，省约 1.8 KB D2 SRAM 和每圈两次 NDTR 读；host 再发 `uart2`/`uart3` 会被固件丢掉。
诊断仍走 `kUart0`，与这两个口无关。

`.ioc` 里配的是 **Hardware Flow Control (RS485)**：`PD4=USART2_DE`、`PB14=USART3_DE`，
生成的 `MX_USARTx_UART_Init` 里已经有 `HAL_RS485Ex_Init(..., UART_DE_POLARITY_HIGH, 0, 0)`，
也就是 `CR3.DEM` 已经打开。**这是 F407 完全没有的能力**——USART 自己在起始位前拉高 DE、
停止位后放开，方向控制不需要任何软件时序。

驱动是 `UartRs485`（`TxBuffer<true>` 加 256 B 环），不是流式 `Uart`。两个口约 1.8 KB D2 SRAM。
实测与半双工换向见 `app/src/uart/uart.hpp` 的注释以及 [AGENTS.md](AGENTS.md)。

原理图（CtrBoard-H7_V1.0-240124 第 5 页）已经结清三件事，不再是「上真实总线前的待办」：

1. **没有半双工回声**。`/RE` 与 DE 绑在同一网，自己发的字节不会回上行。`[实测 2026-08-25 / 2026-09-01]`
2. **DEAT/DEDT 是 16/16**（`.ioc` 里），4.8 Mbaud 下约一个位时间，不是 CubeMX 默认 0。
3. **总线周转**。`TxBuffer<true>` 的 `kTurnaroundDeadline`（1000 us）加上对端 IDLE，一次只放一包。

USART2 占 DMA1_Stream7（RX circular）+ DMA2_Stream2（TX）；USART3 占 DMA2_Stream3 / Stream4。
开关 ON 时 `app.cpp` 会调 `MX_USARTx_UART_Init`，两路 DMA 都在跑；OFF 时时钟门控关闭，`.ioc` 里的 DMA 请求仍在（见下节 `MX_DMA_Init` 哑弹说明）。

三个例外，配了就会真的生效：

1. **`MX_GPIO_Init()` 一定会被调用**（`app.cpp`），所以在 CubeMX 里点成 `GPIO_Output`
   的引脚会真的驱动电平。`gpio.c` 开头那几条 `HAL_GPIO_WritePin` 决定上电默认状态——
   改 GPIO 配置前先确认默认电平（例如 `Power_OUT1_EN` / `Power_OUT2_EN` 目前是拉低）。
2. **`MX_DMA_Init()` 是全局的**：它打开**所有**在 `.ioc` 里配了 DMA 请求的 stream 的
   NVIC，跟对应外设有没有 init 无关。给一个不 init 的外设配 DMA 请求，会得到一个
   「中断使能位已开、但 `HAL_DMA_Init` 从没跑过」的空句柄 handler（`stm32h7xx_it.c`
   里 `HAL_DMA_IRQHandler(&hdma_xxx)` 的 `hdma_xxx` 全零）。时钟关着时触发不了，但是
   个哑弹——**不打算启用的外设不要在 `.ioc` 里给它配 DMA**。
3. **时钟树是全局的**：`RCC` 页上改分频/时钟源会影响所有外设，与 init 无关。

反过来最危险的情况是**`app.cpp` 调了但 `.ioc` 里没配**——那是链接错误
（`undefined reference to MX_xxx_Init`）。CubeMX 会在某些冲突下静默删掉整个外设，
例如 UART5 在 Asynchronous 模式下**必须同时占住 TX 和 RX 两个引脚**（即使外设配的是
`MODE_RX`），一旦 TX 引脚 PC12 被别的信号抢走，CubeMX 加载 `.ioc` 时就把 UART5 整个
移除。改完引脚务必 grep 一遍 `app.cpp` 里的 `MX_*_Init` 是否都还有定义。

## 引脚与 DMA 余量

加外设前先看这张表——**引脚极度紧张，DMA 通道有富余**。

| 资源 | 容量 | 已用 | 说明 |
|---|---|---|---|
| GPIO（LQFP100） | 82 | 81 | **只剩 PB4**，且它是 SPI1_MISO |
| DMA1 stream | 8 | **8** | UART5_RX、USART1/10 RX+TX、UART7 RX+TX、USART2_RX |
| DMA2 stream | 8 | **5** | SPI2 RX+TX、USART2_TX、USART3 RX+TX |
| BDMA channel | 8 | 1 | SPI6_TX（WS2812） |
| MDMA channel | 16 | 0 | 存储器间搬运 |

两条硬约束：

- **DMA1/DMA2 前面是 DMAMUX1，任何 D2 域外设可路由到 16 个 stream 中的任意一个**，
  所以选哪个 stream 无所谓，挑空的就行。
- **SPI6 只能用 BDMA**（它在 D3 域），而 BDMA 够不到 AXI/D2 SRAM，只能访问 D3 SRAM
  （`0x38000000`）——这就是 WS2812 帧缓冲必须放 `.d3_sram` 的原因，见
  `bsp/linker/STM32H723VGTx_APP.ld` 的 `RAM_D3` 区。

FDCAN 不占 DMA（用内部 Message RAM），USB OTG 也不占（自带 DMA 引擎）。
USART2 / USART3 的四条 DMA 已经占掉（见上一节）。DMA1 满了；DMA2 还剩 3 个 stream。
若再把 UART8/9、SPI1_TX、DCMI 的 DMA 也全配上，会超过 DMA1+DMA2 的 16 个上限——DCMI 那一路不能省，优先留给它。

引脚上最紧的是这两组互斥（LQFP100 上各自只有这些候选）：

- `UART5_TX`：PC12、PB6（FDCAN2_TX 占）、PB13（BMI088_SCK 占）
- `SPI6_SCK`：PA5、PB3（SPI1_SCK 占，LCD 用）、PC12

两者都想要 PC12，所以 **DBUS（UART5）和 WS2812（SPI6）二者不能再挪位**；
要腾出引脚只能从 SPI1/LCD、FDCAN2、BMI088 里让，或者把 WS2812 改成 TIM PWM+DMA
驱动（PA7 上可走 TIM3_CH2/AF2 或 TIM14_CH1/AF9）彻底不用 SPI6。

## 构建

```bash
cmake --preset debug -S firmware/mc02
cmake --build firmware/mc02/build --target mc02_app mc02_bootloader
```

preset 与 target 的完整说明见 [AGENTS.md](AGENTS.md#构建)，编译开关见
[AGENTS.md 的「编译开关」](AGENTS.md#编译开关)。
