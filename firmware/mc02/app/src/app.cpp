#include "firmware/mc02/app/src/app.hpp"

#include <bdma.h>
#include <device/usbd.h>
#include <dma.h>
#include <fdcan.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <usart.h>

#include "firmware/mc02/app/src/can/can.hpp"
#include "firmware/mc02/app/src/diag/can_diag.hpp"
#include "firmware/mc02/app/src/diag/loop_profile.hpp"
#include "firmware/mc02/app/src/gpio/gpio.hpp"
#include "firmware/mc02/app/src/key/key.hpp"
#include "firmware/mc02/app/src/led/led.hpp"
#include "firmware/mc02/app/src/power/power.hpp"
#include "firmware/mc02/app/src/spi/bmi088/accel.hpp"
#include "firmware/mc02/app/src/spi/bmi088/gyro.hpp"
#include "firmware/mc02/app/src/spi/bmi088/service.hpp"
#include "firmware/mc02/app/src/spi/bmi088/temperature.hpp"
#include "firmware/mc02/app/src/sync/sof.hpp"
#include "firmware/mc02/app/src/sync/timebase.hpp"
#include "firmware/mc02/app/src/timer/timer.hpp"
#include "firmware/mc02/app/src/uart/uart.hpp"
#include "firmware/mc02/app/src/usb/vendor.hpp"
#include "firmware/mc02/app/src/utility/boot_mailbox.hpp"
#include "firmware/mc02/app/src/utility/interrupt_lock.hpp"

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

// 把存放 UART 端口对象的 D2 SRAM 区设为非缓存, 与 MPU_Config() 对 AXI SRAM 前
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

    // UART 端口对象与其 DMA 环形队列放进 D2 SRAM, 紧邻驱动它们的 DMA1 流
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
    // 启用 PLL2(80 MHz)作 FDCAN 内核时钟, 支撑 1 Mbit/s 仲裁 + 5 Mbit/s 的
    // CAN-FD 数据段; 启用 PLL3(96 MHz)作 USART2/3/4/5/7/8 组的内核时钟
    // -- 96 MHz 可被 4.8 M、4 M、2 M 与 100 k 整除, 这些端口的所有波特率都落在
    // 零误差的整数 BRR 上。由 .ioc 生成; app.cpp 取代了 CubeMX 的 main(), 必须在此调用。
    PeriphCommonClock_Config();

    utility::boot_mailbox.clear();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    RCC_PeriphCLKInitTypeDef usb_clk = {};
    usb_clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    usb_clk.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&usb_clk) != HAL_OK)
        Error_Handler();
    HAL_PWREx_EnableUSBVoltageDetector();
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

    // 用主机的 SOF 包校准 HSI48。上面的 USB 时钟是自由振荡的 HSI48, DS13313
    // Table 35 给出其精度 ACCHSI48_REL = -4.5% .. +3.5%(TJ = -40..125 C, 常温下
    // 也有 47.5..48.5 MHz)。USB 2.0 full speed 允许 +-0.25%, 原始振荡器超差一个
    // 数量级以上, 板子一热便不可靠 -- 台架上能稳定枚举只因出厂 trim 以 25 C 为中心。
    // CRS 把 HSI48 锁到 1 kHz SOF 并拉到主机的精度, 这是 HSI48 能用作 USB 时钟的
    // 前提。
    //
    // c_board 无 HSI48 也无 CRS, USB 时钟来自晶振 PLL, 天然精确, 无需恢复。
    // 同步源选 USB1: H723 只有一个 OTG_HS 实例, 已在上方使能。
    __HAL_RCC_CRS_CLK_ENABLE();
    RCC_CRSInitTypeDef crs_init = {};
    crs_init.Prescaler = RCC_CRS_SYNC_DIV1;
    crs_init.Source = RCC_CRS_SYNC_SOURCE_USB1;
    crs_init.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    // 一个 1 kHz SOF 周期内数 48 MHz, 再减一。
    crs_init.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000U, 1000U);
    crs_init.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
    crs_init.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
    HAL_RCCEx_CRSConfig(&crs_init);

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
#ifdef libhcs_APP_IMU_ENABLE
    MX_SPI2_Init();
#endif
    MX_UART5_Init();
#ifdef libhcs_APP_RS485_ENABLE
    // 两个 RS-485 端口(机箱侧 UART2/UART3)。两个 MX_*_Init 都会执行
    // HAL_RS485Ex_Init, 即 CR3.DEM 的开启之处。这对端口合计约占 32 KB D2 SRAM 区
    // 的 1.8 KB -- 见 uart.hpp。libhcs_APP_RS485_ENABLE 关闭时整体编译掉,
    // 与 IMU 跳过 MX_SPI2_Init 的方式一致。
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
#endif
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM5_Init();

    // 先启动 1/4 微秒时间戳源, 再启动任何会为事件打时间戳的逻辑
    // (UART tx 超时、IMU data-ready EXTI、SPI 上行)。
    timer::timer.init();

    led::led.init();
    gpio::gpio.init();
    usb::vendor.init();

    // 共享时基。必须位于 usb::vendor.init() 之后: 其 tusb_rhport_init() 整体赋值
    // GINTMSK, 会清掉这里设置的 SOF 使能。不开 libhcs_APP_TIME_SYNC 时编译为空。
    sync::sof_init();

    can::can1.init();
    can::can2.init();
    can::can3.init();
    uart::uart1.init();
    uart::uart7.init();
    uart::uart10.init();
    uart::uart_dbus.init();
#ifdef libhcs_APP_RS485_ENABLE
    uart::uart2.init();
    uart::uart3.init();
#endif
#ifdef libhcs_APP_IMU_ENABLE
    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
    spi::bmi088::temperature.init();
#endif

    // 两个驱动参与编译但刻意不启动。包含它们是为了让代码参与类型检查、并把引脚宏
    // 持续对齐当前 .ioc; 二者都不改变板子的上电行为。
    //
    //   power.hpp  三路可关断电源轨保持 MX_GPIO_Init() 设置的原样(24 V 关、5 V 开)。
    //              端子上接什么是接线决定, 上电状态归 .ioc 管。
    //   key.hpp    PA15 会被读取, 但长按应触发什么属产品决定而非驱动职责。定了
    //              之后只需 start(key::mc02_config()) 加上 run() 里的一次 poll()。
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
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

        diag::profile::mark(diag::profile::Section::kTudTask);
        tud_task();

        // 每趟主循环一次, 而非每次 try_transmit() 一次; 见 Vendor::poll_session。
        usb::vendor->poll_session();
        // USB 下行流控: 重估节流策略并结清端点挂载欠账(见 usb/vendor.hpp)。
        usb::vendor->poll_downlink_arm_if_pending();
        usb::poll_dfu_runtime_reboot();

        // 共享时基: 重新拟合微帧-周期计数直线, 并重设 SOF 使能, 让挂钩在控制器于
        // 背后重新初始化后仍存活。不开 libhcs_APP_TIME_SYNC 时编译为空。
        //
        // 以 TIM5 而非循环趟数节流。本循环每秒运行数万次且周期随 USB 负载漂移,
        // "每 N 趟一次"的安排会让拟合周期悄悄变成流量的函数; 且如此频繁地重设
        // SOFM 不过是对外设寄存器无谓的读-改-写。
        if constexpr (sync::timebase::kEnabled) {
            static std::uint32_t last_sync_tick_ms = 0;
            const auto sync_tick_ms = static_cast<std::uint32_t>(
                timer::timer->timepoint().time_since_epoch().count() / 4000U);
            if (sync_tick_ms != last_sync_tick_ms) {
                last_sync_tick_ms = sync_tick_ms;
                sync::timebase::poll(sync_tick_ms);
                sync::sof_rearm();
            }
        }

        // CAN 遥测(仅 libhcs_APP_CAN_DIAG 构建); 以定时器自行节流, 未到期的每趟
        // 只花一次定时器读取, 开关关闭时编译为空。
        diag::profile::mark(diag::profile::Section::kOther);
        diag::poll();

        diag::profile::mark(diag::profile::Section::kGpio);
        gpio::gpio->poll_periodic_input_samples();

#ifdef libhcs_APP_IMU_ENABLE
        // 把到期的温度探针提升为 pending, 然后每趟至多处理一次 BMI088 SPI 读,
        // 优先顺序为 gyro > accel > temperature。
        diag::profile::mark(diag::profile::Section::kImu);
        spi::bmi088::temperature->poll_pending_probe();
        spi::bmi088::service_pending_reads();
#endif

        // LED 动画; 仅颜色变化时阻塞(每次一帧 SPI, 按 WS2812 位率约 330 us)。
        // 放在此处轮询, 确保没有任何 ISR 上下文触碰 SPI。上报的是主机会话状态
        // (nonce 握手加 keepalive 租约), 不只是 USB 枚举 -- 常绿即代表数据确在转发。
        diag::profile::mark(diag::profile::Section::kLed);
        led::led->set_host_connected(usb::vendor->session_established());
        led::led->poll();

        // usb->try_transmit() 的穿插是刻意的: CAN1 填好的 batch 在还没轮到看 UART1
        // 之前就已开始向端点搬运。profiler 把 USB 开销记到 kUsb、每个数据源记到
        // 各自的分区, 相对成本由此可见而非凭假设。
        diag::profile::mark(diag::profile::Section::kUsb);
        usb::vendor->try_transmit();
        diag::profile::mark(diag::profile::Section::kCan);
        // 一次看全部三路 CAN 发送队列: 都没有帧排队时(几乎每趟如此)只需一次加载
        // 加一次分支。两侧穿插的 usb->try_transmit() 次数与逐路轮询时保持一致。
        can::Can::drain_pending_transmits();
        diag::profile::mark(diag::profile::Section::kUsb);
        usb::vendor->try_transmit();
        diag::profile::mark(diag::profile::Section::kUsb);
        usb::vendor->try_transmit();
        diag::profile::mark(diag::profile::Section::kUsb);
        usb::vendor->try_transmit();
        diag::profile::mark(diag::profile::Section::kUart);
        uart::uart1->try_transmit();
        diag::profile::mark(diag::profile::Section::kUsb);
        usb::vendor->try_transmit();
        diag::profile::mark(diag::profile::Section::kUart);
        uart::uart7->try_transmit();
        diag::profile::mark(diag::profile::Section::kUsb);
        usb::vendor->try_transmit();
        diag::profile::mark(diag::profile::Section::kUart);
        uart::uart10->try_transmit();
        diag::profile::mark(diag::profile::Section::kUsb);
        usb::vendor->try_transmit();
        diag::profile::mark(diag::profile::Section::kUart);
        uart::uart_dbus->try_transmit();
#ifdef libhcs_APP_RS485_ENABLE
        usb::vendor->try_transmit();
        uart::uart2->try_transmit();
        usb::vendor->try_transmit();
        uart::uart3->try_transmit();
#endif

        diag::profile::end_pass();
    }
}

} // namespace libhcs::firmware
