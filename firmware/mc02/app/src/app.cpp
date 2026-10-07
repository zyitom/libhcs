#include "firmware/mc02/app/src/app.hpp"

#include <cstddef>
#include <cstdint>

#include <bdma.h>
#include <device/usbd.h>
#include <dma.h>
#include <fdcan.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <usart.h>

#include "core/src/utility/assert.hpp"
#include "firmware/common/app/src/utility/interrupt_lock.hpp"
#include "firmware/mc02/app/src/buzzer/buzzer.hpp"
#include "firmware/mc02/app/src/can/can.hpp"
#include "firmware/mc02/app/src/diag/can_diag.hpp"
#include "firmware/mc02/app/src/diag/loop_profile.hpp"
#include "firmware/mc02/app/src/gpio/gpio.hpp"
#include "firmware/mc02/app/src/key/key.hpp"
#include "firmware/mc02/app/src/led/led.hpp"
#include "firmware/mc02/app/src/ports.hpp"
#include "firmware/mc02/app/src/power/power.hpp"
#include "firmware/mc02/app/src/spi/bmi088/accel.hpp"
#include "firmware/mc02/app/src/spi/bmi088/gyro.hpp"
#include "firmware/mc02/app/src/spi/bmi088/service.hpp"
#include "firmware/mc02/app/src/spi/bmi088/temperature.hpp"
#include "firmware/mc02/app/src/sync/sof.hpp"
#include "firmware/mc02/app/src/sync/sof_capture.hpp"
#include "firmware/mc02/app/src/sync/timebase.hpp"
#include "firmware/mc02/app/src/timer/timer.hpp"
#include "firmware/mc02/app/src/uart/uart.hpp"
#include "firmware/mc02/app/src/usb/vendor.hpp"
#include "firmware/mc02/app/src/utility/boot_mailbox.hpp"
#include "firmware/mc02/app/src/utility/loop_work.hpp"
#include "firmware/mc02/app/src/watchdog/watchdog.hpp"

int main() {
    SCB->VTOR = 0x08040000U;
    libhcs::firmware::app.init().run();
}

namespace libhcs::firmware {

// .itcm 段的链接器符号边界: 加载镜像在 FLASH(_siitcm), 运行位置在 ITCM
// (_sitcm.._eitcm)。extern "C" 使其绑定到全局链接器符号, 而非带命名空间的 C++ 符号。
// NOLINTBEGIN(readability-identifier-naming): 名字由 bsp/linker/STM32H723VGTx_APP.ld
// 固定, 仓库命名规范无法适用 -- 在此改名只会链接失败。
extern "C" {
extern uint32_t _siitcm, _sitcm, _eitcm;
extern uint32_t _sidtcm, _sdtcm, _edtcm;
extern uint32_t _sid2sram, _sd2sram, _ed2sram;
}
// NOLINTEND(readability-identifier-naming)

namespace {

// 把存放 UART 的 DMA 环形队列的 D2 SRAM 区设为非缓存, 与 MPU_Config() 对 AXI SRAM 前
// 32 KB 的做法一致。
//
// 0x30000000 落在 Cortex-M7 默认内存映射的 SRAM 区, 属 Normal write-back
// write-allocate -- 可缓存。保持默认会把 DMA 环形队列置于 D-cache 之下: DMA 写入的
// 数据核会读到旧值, 核写了但未 clean 的数据 DMA 会读到旧值。属性与 region 0 相同
// (TEX level 1, C=0, B=0 => Normal 非缓存), 仅另禁了取指 -- 此处没有代码会取指。
//
// 在 MX_MPU_Config() 之后而非其中执行: MPU_Config 位于 bsp/cubemx/Core/Src/main.c,
// 会被 CubeMX 重新生成。该文件只配置了 region 0, 故 region 1 空闲可用。
void configure_d2_sram_mpu_region() {
    MPU_Region_InitTypeDef region = {};
    region.Enable = MPU_REGION_ENABLE;
    region.Number = MPU_REGION_NUMBER1;
    region.BaseAddress = 0x30000000U;
    region.Size = MPU_REGION_SIZE_32KB;
    region.SubRegionDisable = 0x00;
    region.TypeExtField = MPU_TEX_LEVEL1;
    region.AccessPermission = MPU_REGION_FULL_ACCESS;
    region.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    region.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    region.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    region.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_Disable();
    HAL_MPU_ConfigRegion(&region);
    HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);
}

// ---- 中断向量表放在 DTCM ----
//
// 引导程序跳进来时向量表在 FLASH(main() 设的 VTOR = 0x08040000): 每次进中断先从
// FLASH 取入口地址, 指令/数据缓存被 USB 流量挤掉后还要走 AXI 与 FLASH 等待周期, 中断
// 入口的延迟随之抖动。表复制到零等待、CPU 私有的 DTCM 后, 取向量不再碰总线; FDCAN 的
// 三条接收中断同时改指向 ITCM 里的处理器(can::install_interrupt_vectors)。
//
// 表长: 16 个系统异常 + 外设中断(TIM24_IRQn 是 H723 的最后一个)。VTOR 要求表按其长度
// 向上取整到 2 的幂对齐: 179 项 716 字节, 对齐 1024。
constexpr std::size_t kVectorCount = 16U + static_cast<std::size_t>(TIM24_IRQn) + 1U;
static_assert(kVectorCount * sizeof(uint32_t) <= 1024U);
// 子段 .dtcm.vectors: 链接脚本的 *(.dtcm*) 收进 DTCM; 与 .dtcm 里带初值的对象分开命名,
// 免得零初始化的表与它们在同一个输入段里类型冲突。
alignas(1024) [[gnu::section(".dtcm.vectors")]] uint32_t g_vectors[kVectorCount];

void relocate_vector_table() {
    const auto* flash_vectors = reinterpret_cast<const uint32_t*>(SCB->VTOR);
    for (std::size_t i = 0; i < kVectorCount; ++i)
        g_vectors[i] = flash_vectors[i];
    can::install_interrupt_vectors(g_vectors);
    SCB->VTOR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(g_vectors));
    __DSB();
    __ISB();
}

} // namespace

App::App() {
    const utility::InterruptLockGuard guard;

    // 把延迟关键的 CAN 转发代码(.itcm 段)复制进零等待 ITCM, 必须赶在任何代码能
    // 调用它之前。app.cpp 取代了 CubeMX 的 main(), 这里顶替启动代码的 .data 复制
    // 循环。趁缓存尚未开启执行, 写入直达 ITCM; 其后的 DSB/ISB 让取指单元看见新指令。
    for (uint32_t *dst = &_sitcm, *src = &_siitcm; dst < &_eitcm;)
        *dst++ = *src++;
    // 转发热数据(serializer + USB batch + CAN 对象)搬进 DTCM, 上行 ISR 不再触碰
    // AXI 总线。必须在下方 usb/can 初始化之前执行。
    for (uint32_t *dst = &_sdtcm, *src = &_sidtcm; dst < &_edtcm;)
        *dst++ = *src++;
    // 向量表搬进 DTCM。必须在上面的 DTCM 复制之后(那次复制会把表冲成零), 在任何中断
    // 打开之前(本作用域关着中断, HAL_Init 的 SysTick 在后面)。
    relocate_vector_table();

    // UART 的 DMA 环形队列放进 D2 SRAM, 紧邻驱动它们的 DMA1 流(端口对象本身在 DTCM)
    // (见 uart.hpp 与链接脚本的 .d2_sram 段)。先开时钟: 本芯片 RCC_AHB2ENR 复位值
    // 中 D2 SRAM 使能位为 0, 不先开时钟下面的复制会写进被门控的内存。趁缓存未开
    // 执行还意味着复制直达内存 -- configure_d2_sram_mpu_region() 尚未运行, 该区
    // 按默认映射仍可缓存。仅开 SRAM1/SRAM2: H723 把 32 KB D2 SRAM 分为两个
    // 16 KB bank 且没有 SRAM3, 本芯片不存在 __HAL_RCC_D2SRAM3_CLK_ENABLE
    // (stm32h723xx.h 只定义 RCC_AHB2ENR_D2SRAM1EN/2EN, 无 3EN)。
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    for (uint32_t *dst = &_sd2sram, *src = &_sid2sram; dst < &_ed2sram;)
        *dst++ = *src++;

    __DSB();
    __ISB();

    // 对生成代码 MPU_Config() 的包装, 定义在 main.c 的 USER CODE BEGIN 4 内。
    // CubeMX 在任何 USER CODE 块之外把 MPU_Config 的原型重生成为 `static`,
    // 因此无法在 main.h 直接声明; 见 main.h 中 libhcs_mpu_config 的注释。
    libhcs_mpu_config();
    configure_d2_sram_mpu_region();
    SCB_EnableICache();
    SCB_EnableDCache();
    HAL_Init();
    SystemClock_Config();
    // 外设内核时钟全部来自 24 MHz 晶振:
    //   PLL2Q 80 MHz   FDCAN, 1 Mbit/s 仲裁 + 5 Mbit/s 数据段各 16 tq
    //   PLL2P 120 MHz  SPI1/2/3(BMI088 在 SPI2, /16 = 7.5 MHz)与 ADC
    //   PLL3Q 48 MHz   USB 与全部串口。48 MHz 整除 100 k / 1 M / 2 M / 4 M / 4.8 M / 6 M;
    //                  16 倍过采样上限 3 Mbit/s, 默认 4.8 Mbit/s 的两个 RS-485 口
    //                  (USART2/3)因此在 .ioc 里设为 8 倍过采样
    // 由 .ioc 生成; app.cpp 取代了 CubeMX 的 main(), 必须在此调用。
    PeriphCommonClock_Config();

    utility::boot_mailbox.clear();

    // 硬件看门狗（libhcs_WATCHDOG，默认 OFF）：主循环停止喂狗 500ms 即复位。
    // 默认不编进调试镜像——挂死现场比自动恢复更值得看到。
    watchdog::watchdog.init();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // USB 内核时钟是 PLL3Q 48 MHz, 与全部串口同源, 来自 24 MHz 晶振; 选择由 .ioc 生成的
    // PeriphCommonClock_Config() 完成。原先是自由振荡的 HSI48 加 CRS 锁到主机 SOF:
    // HSI48 全温度范围误差 -4.5% .. +3.5%(DS13313 Table 35), USB 全速只容许 +-0.25%,
    // 而 CRS 要先收到 SOF 才能校准 -- 板子热态下重新枚举没有保证。晶振给的时钟从
    // 上电第一个包起就在精度内, CRS 也就不要了。
    HAL_PWREx_EnableUSBVoltageDetector();
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

    MX_GPIO_Init();
    MX_DMA_Init();
    // BDMA 是唯一能到达 SPI6(D3 域)的 DMA 控制器, WS2812 LED 依赖它。CubeMX 把
    // 它的时钟使能与 NVIC 配置单独生成在 bdma.c 而非 MX_DMA_Init() 中, 而 app.cpp
    // 取代了生成的 main(), 故调用必须放在这里。必须先于 MX_SPI6_Init(): 其 MspInit
    // 会对 BDMA_Channel0 跑 HAL_DMA_Init(), 时钟仍被门控时这些寄存器写入被丢弃,
    // HAL_SPI_Transmit_DMA() 随后对永不启动的传输报 HAL_OK, hspi6 永远停在 BUSY_TX,
    // LED 保持熄灭。
    MX_BDMA_Init();
    MX_FDCAN1_Init();
    MX_USART10_UART_Init();
    MX_UART7_Init();
    MX_USART1_UART_Init();
    MX_FDCAN2_Init();
    MX_FDCAN3_Init();
    MX_SPI6_Init();
    MX_SPI2_Init();
    MX_UART5_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    // 板上 1/4 us 时间戳源(timer/timer.hpp)。2026-10-05 前是 TIM5。
    MX_TIM23_Init();
    // TIM3 与 TIM5 归共享时基: TIM5 锁 USB SOF(ITR7)并承载整条时间轴, TIM3 是 FDCAN 外部
    // 时间戳的计数器(sync/sof_capture.hpp)。这里只写好 PSC/ARR, 不开中断、不开通道, 计数器
    // 在主机的清单第一次要时间基准时才起走(sync::time_sync_start())。TIM3 CH4(PB1, IMU
    // 加热)的输出通道不开。
    MX_TIM3_Init();
    MX_TIM5_Init();
    MX_TIM12_Init(); // 蜂鸣器: 引脚进复用功能, 通道不开 -- 主机声明了才启动(buzzer.hpp)

    // 先启动 1/4 微秒时间戳源, 再启动任何会为事件打时间戳的逻辑
    // (UART tx 超时、IMU data-ready EXTI、SPI 上行)。
    timer::timer.init();

    led::led.init();
    // 四个 PWM 引脚: 对象就位, 引脚与定时器通道不碰 -- 主机声明了才启动(gpio/gpio.hpp)。
    gpio::gpio.init();
    buzzer::buzzer.init();
    usb::vendor.init();

    can::can1.init();
    can::can2.init();
    can::can3.init();
    uart::uart1.init();
    uart::uart7.init();
    uart::uart10.init();
    uart::uart_dbus.init();
    uart::uart2.init();
    uart::uart3.init();
    // 板载 IMU: 对象就位, 芯片不碰, 数据就绪线先屏蔽掉 -- 主机声明了才初始化
    // (spi/bmi088/service.hpp)。
    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
    spi::bmi088::temperature.init();
    spi::bmi088::imu_port.suspend();
    // 驱动自带的身份(数据流按它打标)与 EP0 绑定的身份是同一个口: 构造参数在编译期
    // 看不到, 驱动就位后核对一次(ports.hpp)。
    core::utility::assert_always(ports::Registry::identities_match());

    // 两个驱动参与编译但刻意不启动。包含它们是为了让代码参与类型检查、并把引脚宏
    // 持续对齐当前 .ioc; 二者都不改变板子的上电行为。
    //
    //   power.hpp  三路可关断电源轨保持 MX_GPIO_Init() 设置的原样(24 V 关、5 V 开)。
    //              端子上接什么是接线决定, 上电状态归 .ioc 管。
    //   key.hpp    PA15 会被读取, 但长按应触发什么属产品决定而非驱动职责。定了
    //              之后只需 start(key::mc02_config()) 加上 run() 里的一次 poll()。
}

// 主循环。
//
// 每一趟固定要付的只有四样, 其余都挂在条件后面:
//   1. USB 协议栈有没有事件(有才进 tud_task())
//   2. 毫秒有没有翻(翻了才做杂务: 会话租约、DFU 重启、LED、CAN 发送卡死守护)
//   3. 上行泵 usb::vendor->try_transmit()
//   4. loop::active 是不是 0(主机声明过的 CAN / 串口 / IMU, 以及声明了周期采样的引脚)
// 主机没声明的东西因此在这里不占一条指令; 声明了几样, 主循环就长出几样。
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    uint32_t last_tick_ms = HAL_GetTick();

    while (true) {
        diag::note_main_loop();

#if defined(libhcs_APP_LOOP_BALLAST_CYCLES) && libhcs_APP_LOOP_BALLAST_CYCLES
        // 测量仪器, 不随正式固件发布: 让主循环周期增加已知数量的 DWT 周期
        // (550 MHz 下每微秒 550 个), 以便把吞吐量相对主循环周期作图。
        //
        // 这是区分两个候选瓶颈的唯一手段。若包速率跟随配重变化, 主循环就在关键
        // 路径上, 代码布置、内存序与原子操作值得优化; 若纹丝不动, 天花板在主机
        // 调度侧, 该情况改变之前板端任何优化都无法被验证。
        //
        // 刻意忙等而非 sleep: 目的就是让 CPU 像被真实工作那样占住, 中断照常运行。
        {
            const std::uint32_t ballast_start = DWT->CYCCNT;
            while ((DWT->CYCCNT - ballast_start) < (libhcs_APP_LOOP_BALLAST_CYCLES)) {}
        }
#endif

        // USB 协议栈: 事件队列里有东西才进。tud_task() 自己取事件时要先关、后开 USB
        // 中断, 队列空着也照做; 而它除了处理事件之外没有别的职责, 所以先看一眼队列
        // (无锁读两个下标)。看的瞬间刚好有事件入队也无妨, 下一趟就处理了。
        diag::profile::mark(diag::profile::Section::kTudTask);
        if (tud_task_event_ready())
            tud_task();

        // 共享时基(主机的清单要了才开; 关着时只付这一次判断): 重新拟合微帧-周期计数直线,
        // 并重设 SOF 使能, 让挂钩在控制器于背后重新初始化后仍存活。
        //
        // 以定时器而非循环趟数节流。本循环每秒运行数万次且周期随 USB 负载漂移,
        // "每 N 趟一次"的安排会让拟合周期悄悄变成流量的函数; 且如此频繁地重设
        // SOFM 不过是对外设寄存器无谓的读-改-写。
        if (sync::time_sync_on()) {
            static std::uint32_t last_sync_tick_ms = 0;
            const auto sync_tick_ms = static_cast<std::uint32_t>(
                timer::timer->timepoint().time_since_epoch().count() / 4000U);
            if (sync_tick_ms != last_sync_tick_ms) {
                last_sync_tick_ms = sync_tick_ms;
                sync::timebase::poll(sync_tick_ms);
                sync::sof_capture::refit();
                sync::sof_rearm();
            }
        }

        // CAN 遥测(仅 libhcs_APP_CAN_DIAG 构建); 以定时器自行节流, 未到期的每趟
        // 只花一次定时器读取, 开关关闭时编译为空。
        diag::profile::mark(diag::profile::Section::kOther);
        diag::poll();

        // 毫秒杂务: 判据都是毫秒乃至秒级的事, 每毫秒看一次足够, 不必每趟主循环都付。
        // HAL_GetTick() 读的是 RAM 里的计数, 所以没翻毫秒的那些趟只花一次比较。
        if (const uint32_t tick_ms = HAL_GetTick(); tick_ms != last_tick_ms) [[unlikely]] {
            last_tick_ms = tick_ms;

            // 会话租约(4 s)与 DFU 重启(请求后 50 ms)。
            usb::vendor->poll_session();
            usb::poll_dfu_runtime_reboot();

            // CAN 发送卡死守护(判据 20 ms): 只查声明过的总线。
            if ((loop::active & loop::kCanBuses) != 0U)
                can::Can::recover_all_stuck_transmits();

            // 板载 IMU 的温度探针(1 Hz): 只在 IMU 被声明时。
            if ((loop::active & loop::kImu) != 0U)
                spi::bmi088::imu_port.poll_temperature_probe();

            // LED 动画; 仅颜色变化时阻塞(每次一帧 SPI, 按 WS2812 位率约 330 us)。
            // 放在主循环而非中断里, 确保没有任何 ISR 上下文触碰 SPI。上报的是主机会话
            // 状态(nonce 握手加 keepalive 租约), 不只是 USB 枚举 -- 常绿即代表数据确在转发。
            diag::profile::mark(diag::profile::Section::kLed);
            led::led->set_host_connected(usb::vendor->session_established());
            led::led->poll();
        }

        // 上行泵: 中断里序列化好的 batch(CAN 反馈、IMU 样本)从这里往端点搬。每次调用
        // 最多搬一个 64 字节的包, 端点忙就直接返回。
        diag::profile::mark(diag::profile::Section::kUsb);
        usb::vendor->try_transmit();

        // 主机声明过的东西。位图为 0 时到此为止。
        if (const uint32_t active = loop::active; active != 0U) {
            // CAN: 一次看全部三路发送队列, 都没有帧排队时(几乎每趟如此)只需一次加载加
            // 一次分支。这是下行方向(软件队列 -> 硬件 FIFO), 不产生上行数据, 后面不必泵 USB。
            if ((active & loop::kCanBuses) != 0U) {
                diag::profile::mark(diag::profile::Section::kCan);
                can::Can::drain_pending_transmits();
            }

            if ((active & loop::kGpioSampling) != 0U) {
                diag::profile::mark(diag::profile::Section::kGpio);
                gpio::gpio->poll_periodic();
            }

            if ((active & loop::kImu) != 0U) {
                diag::profile::mark(diag::profile::Section::kImu);
                spi::bmi088::imu_port.poll();
            }

            // 串口: 每个口之后泵一次 USB, 它刚出队的字节在还没轮到下一个口之前就开始往
            // 端点搬。所以一圈里 usb->try_transmit() 的次数是 1 + 声明的串口数。
            //
            // profiler(仅 libhcs_APP_LOOP_PROFILE 构建, 平时 mark() 编译为空)把 USB 开销
            // 记到 kUsb、串口记到 kUart, 相对成本由此可见而非凭假设。
            diag::profile::mark(diag::profile::Section::kUart);
            ports::poll_uarts(active, [] {
                diag::profile::mark(diag::profile::Section::kUsb);
                usb::vendor->try_transmit();
                diag::profile::mark(diag::profile::Section::kUart);
            });
        }

        diag::profile::end_pass();

        watchdog::watchdog->feed();
    }
}

} // namespace libhcs::firmware
