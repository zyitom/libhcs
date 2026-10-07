#include "firmware/c_board/app/src/app.hpp"

#include <cstddef>
#include <cstdint>

#include <can.h>
#include <device/usbd.h>
#include <dma.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <usart.h>
#include <usb_otg.h>

#include "core/src/utility/assert.hpp"
#include "firmware/c_board/app/src/buzzer/buzzer.hpp"
#include "firmware/c_board/app/src/can/can.hpp"
#include "firmware/c_board/app/src/gpio/gpio.hpp"
#include "firmware/c_board/app/src/led/led.hpp"
#include "firmware/c_board/app/src/ports.hpp"
#include "firmware/c_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/c_board/app/src/spi/bmi088/gyro.hpp"
#include "firmware/c_board/app/src/spi/bmi088/service.hpp"
#include "firmware/c_board/app/src/spi/bmi088/temperature.hpp"
#include "firmware/c_board/app/src/spi/spi.hpp"
#include "firmware/c_board/app/src/sync/sof.hpp"
#include "firmware/c_board/app/src/sync/timebase.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/c_board/app/src/uart/uart.hpp"
#include "firmware/c_board/app/src/usb/vendor.hpp"
#include "firmware/c_board/app/src/utility/boot_mailbox.hpp"
#include "firmware/c_board/app/src/utility/loop_work.hpp"
#include "firmware/c_board/app/src/watchdog/watchdog.hpp"
#include "firmware/common/app/src/utility/interrupt_lock.hpp"

int main() {
    SCB->VTOR = 0x08010000U;
    libhcs::firmware::app.init().run();
}

namespace libhcs::firmware {

// 链接脚本给出的 .ccmram / .dmaram 段边界: FLASH 里的加载映像(_si*)与 RAM 里的运行
// 位置(_s*.._e*)。F4 的启动代码只拷 .data(含 .RamFunc)、清 .bss, 这两段的拷贝由
// App 构造函数代办。
extern "C" {
// NOLINTNEXTLINE(readability-identifier-naming)
extern uint32_t _siccmram, _sccmram, _eccmram;
// NOLINTNEXTLINE(readability-identifier-naming)
extern uint32_t _sidmaram, _sdmaram, _edmaram;
}

namespace {

// 把 DMA 目标缓冲区的窗口(.dmaram 段, 0x20018000 起 32K = SRAM1 尾部 16K + SRAM2)
// 设为非缓存。
//
// 0x20018000 落在 Cortex-M4 默认内存映射的 SRAM 区, Normal write-back write-allocate
// -- 可缓存。开启 D-cache(见 App::App())后保持默认会把 DMA 环形队列置于 D-cache 之下:
// DMA 写入的数据核会读到旧值, 核写了但未 clean 的数据 DMA 会读到旧值。TEX level 1,
// C=0, B=0 => Normal 非缓存, 两个方向都直通内存, 免逐传输 clean/invalidate。另禁了
// 取指 -- 这里没有代码。
//
// 必须在开 D-cache 之前执行。c_board 的 CubeMX main.c 没有配置任何 MPU 区域, region 0
// 空闲可用。与 mc02 app.cpp 的 configure_d2_sram_mpu_region() 同一做法: 那边管
// 0x30000000 的 D2 SRAM, 这边管 DMARAM。
void configure_dmaram_mpu_region() {
    MPU_Region_InitTypeDef region = {};
    region.Enable = MPU_REGION_ENABLE;
    region.Number = MPU_REGION_NUMBER0;
    region.BaseAddress = 0x20018000U;
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

    // 在这些全局被用到之前, 把热的纯 CPU 转发状态(序列化器 + USB 批缓冲 + CAN TX
    // 环)拷进零等待的 CCM。M4 没有数据缓存, 写入直接落硬件; 此处中断已屏蔽。
    // 目标必须保持可写; 这个 clang-tidy 检查没有考虑循环体里的自增加存储表达式。
    // NOLINTNEXTLINE(misc-const-correctness)
    for (uint32_t *dst = &_sccmram, *src = &_siccmram; dst < &_eccmram;)
        *dst++ = *src++;

    // 同法拷 .dmaram(DMA 目标缓冲区: 串口收发环与暂存、SPI 收发、SOF 捕获目的字)。
    // 拷贝在 D-cache 开启之前执行(此刻写入直接落内存), 落点稍后由 MPU 设为非缓存,
    // 之后 CPU 与 DMA 对它的访问都绕开缓存。见 configure_dmaram_mpu_region()。
    // NOLINTNEXTLINE(misc-const-correctness)
    for (uint32_t *dst = &_sdmaram, *src = &_sidmaram; dst < &_edmaram;)
        *dst++ = *src++;

    HAL_Init();
    SystemClock_Config();

    // ART 加速器。FLASH_ACR 的这三个位复位后全为 0, CubeMX 的 SystemClock_Config 只
    // 写了 LATENCY(168 MHz 下 5 WS): 不开它们, 每条取指都要付等待周期。
    //  - 预取 + 指令缓存: 取指走 128 位预取队列与 I-cache, 消除 XIP 等待 -- 热路径
    //    代码(can.cpp 的 .RamFunc)进一步搬进了零等待 SRAM, 这是留给其余代码的。
    //  - 数据缓存: 服务 SRAM/CCM 上的普通数据。DMA 一致性由此处 MPU 保障 -- 全部
    //    DMA 目标缓冲都隔离在 .dmaram 非缓存区(见 configure_dmaram_mpu_region()),
    //    USB 是 OTG_FS slave 模式(CPU 搬包 RAM, 无系统内存 DMA), 没有别的暴露。
    configure_dmaram_mpu_region();
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    __HAL_FLASH_DATA_CACHE_ENABLE();
    utility::boot_mailbox.clear();

    // 硬件看门狗（libhcs_WATCHDOG，默认 OFF）：主循环停止喂狗 500ms 即复位。
    // 默认不编进调试镜像——挂死现场比自动恢复更值得看到。
    watchdog::watchdog.init();

    // 数据观察点周期计数器, 跨板共享时间基准的本地时钟(sync/timebase.hpp: 拟合
    // 在 168 MHz 的 CYCCNT 上做, 250 ns 的 TIM2 量化对被测抖动太粗)。与 mc02 同一
    // 位置无条件使能。调试器挂接会停走 CYCCNT, 时间基准随之失效到复位为止。
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // TIM9 必须在 TIM2 之前初始化。
    MX_TIM9_Init();
    MX_TIM2_Init();
    timer::timer.init();
    // SOF 沿的细计数器(84 MHz, 16 位): 只初始化, 不起走 -- 第一次开时间基准时由
    // sync/sof.cpp 的 capture_init() 起走并接上 DMA(见那里的注释)。
    MX_TIM7_Init();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM8_Init();
    MX_TIM4_Init(); // 蜂鸣器: 引脚进复用功能, 通道不开 -- 主机声明了才启动(buzzer.hpp)
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    MX_TIM5_Init();
    MX_USB_OTG_FS_PCD_Init();

    led::led.init();
    usb::vendor.init();
    can::can1.init();
    can::can2.init();
    uart::uart1.init();
    uart::uart2.init();
    uart::uart_dbus.init();
    // 七个 PWM 引脚: 对象就位, 引脚与定时器通道在清单声明前一概不碰(gpio/gpio.hpp)。
    gpio::gpio.init();
    buzzer::buzzer.init();
    // 驱动自带的身份(数据流按它打标)与 EP0 绑定的身份是同一个口: 构造参数在编译期
    // 看不到, 驱动就位后核对一次(ports.hpp)。
    core::utility::assert_always(ports::Registry::identities_match());
    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
    spi::bmi088::temperature.init();
    // 传感器配好了, 但 IMU 口没声明前不工作: 屏蔽数据就绪线(CubeMX 的初始化把它们开着)。
    spi::bmi088::imu_port.suspend();
}

// 主循环, 与 mc02 同一结构。
//
// 每一趟固定要付的只有四样, 其余都挂在条件后面:
//   1. USB 协议栈有没有事件(有才进 tud_task())
//   2. 毫秒有没有翻(翻了才做杂务: 会话租约、DFU 重启、共享时基、LED)
//   3. 上行泵 usb::vendor->try_transmit()
//   4. loop::active 是不是 0(主机声明过的 CAN / 串口 / IMU, 以及声明了周期采样的引脚)
// 主机没声明的东西因此在这里不占一条指令; 声明了几样, 主循环就长出几样。
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    uint32_t last_tick_ms = HAL_GetTick();

    while (true) {
        // USB 协议栈: 事件队列里有东西才进。tud_task() 自己取事件时要先关、后开 USB
        // 中断, 队列空着也照做; 它除了处理事件之外没有别的职责, 所以先看一眼队列。
        if (tud_task_event_ready())
            tud_task();

        // 毫秒杂务: 判据是毫秒级的事, 每毫秒看一次足够, 不必每趟主循环都付。
        // HAL_GetTick() 读 RAM 里的计数, 没翻毫秒的那些趟只花一次比较。
        if (const uint32_t tick_ms = HAL_GetTick(); tick_ms != last_tick_ms) [[unlikely]] {
            last_tick_ms = tick_ms;

            // 会话租约(4 s)与 DFU 重启(请求后 50 ms)。
            usb::vendor->poll_session();
            usb::poll_dfu_runtime_reboot();

            // 共享时基(主机的清单要了才开; 关着时只付一次内存读与一次分支): 重算
            // 微帧-周期计数直线, 并重设 SOF 使能, 让挂钩在控制器于背后被重新初始化
            // (总线复位后的 dcd_init 会重写 GINTMSK)之后仍存活。poll() 自身再以
            // kFitPeriodMs 节流到 20 ms 一次实际工作。
            if (sync::time_sync_on()) {
                sync::timebase::poll(tick_ms);
                sync::sof_rearm();
            }

            // 把会话状态发布给 LED。动画本身由 HAL_IncTick() 以 1 kHz 驱动; 中断里不碰
            // USB 栈。上报会话而不只是枚举完成: 常亮的绿灯说明数据确实在转发。
            led::led->set_host_connected(usb::vendor->session_established());
        }

        // 上行泵: 中断里序列化好的 batch(CAN 反馈、IMU 样本)从这里往端点搬。
        usb::vendor->try_transmit();

        // 主机声明过的东西。位图为 0 时到此为止。
        if (const uint32_t active = loop::active; active != 0U) {
            // CAN: 声明过的总线把软件队列往发送邮箱里搬。这是下行方向, 不产生上行数据。
            if ((active & loop::kCanBuses) != 0U) {
                for (std::size_t index = 0; index < can::kCanCount; ++index) {
                    if ((active & loop::bit(can::kCanPorts[index].data_id)) != 0U)
                        can::can_by_index(index)->drain_pending_transmits();
                }
            }

            if ((active & loop::kGpioSampling) != 0U)
                gpio::gpio->poll_periodic();

            // 板载 IMU: SPI 收尾、温度探测与挂起的读。没声明时数据就绪线是屏蔽的, 这三样
            // 都无事可做, 所以整组挂在 kImu 位后面。
            if ((active & loop::kImu) != 0U) {
                spi::spi1->update();
                spi::bmi088::temperature->poll_pending_probe();
                spi::bmi088::service_pending_reads();
            }

            // 串口: 每个口之后泵一次 USB, 它刚出队的字节在轮到下一个口之前就开始往端点搬。
            ports::poll_uarts(active, [] { usb::vendor->try_transmit(); });
        }

        watchdog::watchdog->feed();
    }
}

} // namespace libhcs::firmware
