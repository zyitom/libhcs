#include "firmware/c_board/app/src/sync/sof.hpp"

#include <atomic>
#include <cstdint>

#include <main.h>

#include "firmware/c_board/app/src/sync/timebase.hpp"
#include "firmware/common/app/src/utility/interrupt_lock.hpp"

namespace libhcs::firmware::sync {
namespace {

// DWC2 寄存器块。OTG_FS 是全局块(USB_OTG_FS_PERIPH_BASE = 0x5000'0000, AHB2),
// 设备块在距其固定偏移处。经 CMSIS 访问而非 TinyUSB 的 dwc2_regs_t, 使这里不依赖
// TinyUSB 的内部布局。
USB_OTG_GlobalTypeDef* global_registers() {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<USB_OTG_GlobalTypeDef*>(USB_OTG_FS_PERIPH_BASE);
}

USB_OTG_DeviceTypeDef* device_registers() {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<USB_OTG_DeviceTypeDef*>(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
}

// ---- 硬件 SOF 捕获: TIM2 ITR1 触发 DMA, 抄下 TIM7 ----
//
// mc02 在 TIM5 ITR7 上锁存 SOF, 用捕获值把中断时间戳回拨到边沿实际到达的时刻。该修正
// 只在被锁存计数器的 tick 远小于中断路径自身的抖动时才有意义(mc02 sof.cpp 的
// capture_init() 有门控判据与实测: 1 MHz 的捕获毫无效果, 275 MHz 的有)。
//
// F407 上 OTG_FS 的 SOF 只引到一个定时器: TIM2 的 ITR1(TIM2_OR.ITR1_RMP = 10,
// HAL 的 TIM_TIM2_USBFS_SOF; RM0090 34.7.2 与 18.4.19)。TIM2 又是全板 1/4 us 时间戳
// 的低字(timer/timer.hpp, 4 MHz, 与 TIM9 级联成 48 位), 它的 CCR 只有 250 ns 一拍 --
// 按上面的判据太粗。但开捕获并不需要动 TIM2 的计数: 通道 1 映射到 TRC、触发选 ITR1、
// SMS 保持 0, 计数器照常走; 捕获事件本身只用来发一个 DMA 请求(CC1DE)。
//
// DMA1 Stream5 Channel3 就是 TIM2_CH1 的请求(RM0090 表 43), 本板 DMA1 只占了 Stream1/3
// (USART3)。它在边沿那一刻把 TIM7->CNT 抄进内存: TIM7 是 APB1 上空着的基本定时器,
// .ioc 给 Prescaler = 0、Period = 65535, 84 MHz 走 16 位(11.9 ns 一拍, 780 us 一圈),
// 一拍 2 个 CPU 周期 -- 落在判据"有效"的一侧。中断里再读一次 TIM7->CNT, 两者之差就是
// 边沿到中断入口过了多久, 从 CYCCNT 时间戳里减去。DMA 的请求-读出延迟是常数, 拟合吸收。
//
// 新鲜性: DMA 每个 SOF 写一次目的字, 中断用完就写回 kNoEdge。TIM7 的 CNT 读成 32 位时高
// 半字恒为 0, DMA 写不出 kNoEdge, 所以"还是 kNoEdge"= 这个 SOF 没有捕获; 不靠龄期去猜
// (16 位一圈只有 780 us, 上一毫秒的旧值模一圈后看起来只有约 220 us)。
//
// 目的字在 .dmaram: DMA 够不着 CCM(.ccmram), 且 D-cache 开启后它必须待在非缓存区
// (app.cpp 的 configure_dmaram_mpu_region())。
//
// 未验证的假设 [2026-10-07, 只过编译]:
//  - ITR1 上确有 SOF 脉冲(手册写明, 本板未上板看); 捕获计数(kTimeStatus 的
//    capture_fresh/stale_count)全是 stale 即反证。
//  - 修正后的单样本 sigma 比纯中断时间戳小。要交替烧录做 A/B, 与 mc02 的做法一样。

constexpr std::uint32_t kNoEdge = 0xFFFF'FFFFU;

// DMA 的目的字。DMA 写、SOF 中断读并写回 kNoEdge; 放 .dmaram 非缓存区(app.cpp 的
// MPU 区域): D-cache 开启后, 普通可缓存内存里的 DMA 写入核会读旧值。
[[gnu::section(".dmaram")]] volatile std::uint32_t g_sof_edge_tim7 = kNoEdge;

std::uint32_t capture_cycles_per_tick = 0;
std::uint32_t capture_max_age_ticks = 0;
bool capture_enabled = false;

// 只由 SOF 中断写, 主循环在关中断下取走(take_capture_counts())。
std::uint32_t capture_fresh = 0;
std::uint32_t capture_stale = 0;

// 一个 APB1 定时器时钟周期折合的 CPU 周期数, 从 RCC 分频读出而非写死(.ioc 改 HPRE 或
// PPRE1 都会改变它)。F4: APB1 分频为 1 时定时器时钟等于 PCLK1, 否则是 PCLK1 的 2 倍。
std::uint32_t cpu_cycles_per_timer_clock() {
    const std::uint32_t hpre_shift =
        AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
    const std::uint32_t apb1_shift =
        APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos];
    const std::uint32_t timer_shift = apb1_shift > 0U ? apb1_shift - 1U : 0U;
    return 1UL << (hpre_shift + timer_shift);
}

// 第一次 time_sync_start() 时调用一次。TIM2 的计数、TIM7 的预分频与周期都来自 .ioc,
// 这里只检查不改写(CubeMX 纪律); 改的是 .ioc 不管的那几项: TIM2 的捕获通道与触发选择、
// ITR1 重映射, 以及 DMA1 Stream5。不满足前提就不接管, 时间基退回纯中断时间戳。
void capture_init() {
    if (TIM7->PSC != 0U || TIM7->ARR != 0xFFFFU)
        return;

    // DMA1 Stream5: 外设 TIM7->CNT -> 内存 g_sof_edge_tim7, 一次一个字, 循环, 最高优先级,
    // 直接模式(无 FIFO), 不开中断。先停下再改。
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while ((DMA1_Stream5->CR & DMA_SxCR_EN) != 0U) {}
    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5
                | DMA_HIFCR_CFEIF5;
    DMA1_Stream5->PAR = reinterpret_cast<std::uint32_t>(&TIM7->CNT);
    DMA1_Stream5->M0AR = reinterpret_cast<std::uint32_t>(&g_sof_edge_tim7);
    DMA1_Stream5->NDTR = 1;
    DMA1_Stream5->FCR = 0;
    DMA1_Stream5->CR = (3UL << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL | DMA_SxCR_MSIZE_1
                     | DMA_SxCR_PSIZE_1 | DMA_SxCR_CIRC;
    DMA1_Stream5->CR |= DMA_SxCR_EN;

    TIM7->CR1 |= TIM_CR1_CEN;

    // TIM2 CH1 映射到 TRC, 触发选 ITR1 = OTG_FS SOF。SMS 保持 0: 计数器从不从属于触发,
    // 只有捕获通道消费它; CR2 的 TRGO(级联 TIM9)不动。
    TIM2->OR = (TIM2->OR & ~TIM_OR_ITR1_RMP) | TIM_TIM2_USBFS_SOF;
    TIM2->CCMR1 = (TIM2->CCMR1 & ~(TIM_CCMR1_CC1S | TIM_CCMR1_IC1PSC | TIM_CCMR1_IC1F))
                | TIM_CCMR1_CC1S; // CC1S = 11: TRC
    TIM2->SMCR = (TIM2->SMCR & ~(TIM_SMCR_TS | TIM_SMCR_SMS)) | TIM_TS_ITR1;
    TIM2->CCER = (TIM2->CCER & ~(TIM_CCER_CC1P | TIM_CCER_CC1NP)) | TIM_CCER_CC1E;
    TIM2->DIER |= TIM_DIER_CC1DE;

    // 缓存一次, ISR 里不再读。约束: TIM7 的 PSC / ARR 不得在运行时改写。
    capture_cycles_per_tick = cpu_cycles_per_timer_clock() * (TIM7->PSC + 1U);
    // 100 us, 单位 tick。中断入口落在边沿之后几微秒以内; 比这更老的不可能是本次 SOF 的。
    capture_max_age_ticks = (timebase::kCyclesPerMicrosecond * 100U) / capture_cycles_per_tick;
    capture_enabled = true;
}

// 从中断时间戳中扣除的周期数; 这个 SOF 没有可信的捕获时为 0(恰好退化为无捕获时的行为)。
// frame_changed: 读完捕获后帧号变了, 处理已被推迟过下一个 SOF, 目的字可能属于那个新的。
std::uint32_t capture_age_cycles(std::uint32_t counter, bool frame_changed) {
    const std::uint32_t edge = g_sof_edge_tim7;
    g_sof_edge_tim7 = kNoEdge;
    const std::uint32_t age = (counter - edge) & 0xFFFFU; // 16 位计数器, 模 2^16
    if (edge == kNoEdge || frame_changed || age > capture_max_age_ticks) {
        ++capture_stale;
        return 0;
    }
    ++capture_fresh;
    return age * capture_cycles_per_tick;
}

bool capture_ready = false; // capture_init() 跑过了

} // namespace

bool time_sync_start() {
    if (time_sync_on())
        return true;

    // 先配置、复位, 再置位、开中断: ISR 看到"开着"时, 它要碰的状态都已就绪。此刻
    // SOFM 是关着的。
    if (!capture_ready) {
        capture_init();
        capture_ready = true;
    }
    timebase::reset();
    g_sof_edge_tim7 = kNoEdge;
    internal::g_time_sync_on.store(true, std::memory_order_release);
    global_registers()->GINTMSK |= USB_OTG_GINTMSK_SOFM;
    return true;
}

void time_sync_stop() {
    if (!time_sync_on())
        return;
    internal::g_time_sync_on.store(false, std::memory_order_release);
    global_registers()->GINTMSK &= ~USB_OTG_GINTMSK_SOFM;
}

void sof_isr_entry() {
    // 本函数在每个 USB 中断里跑: 时间基准关着时只付这一次内存读。
    if (!time_sync_on())
        return;

    // 先读周期计数器, 早于"这是否 SOF"的判断。GINTSTS 是一次 AHB 读, 耗时数十
    // 纳秒且可能被总线流量卡住; 时间戳取在其后会把这一可变停顿折进每个样本。
    // CYCCNT 是核心本地寄存器, 两个周期, 无条件读对最终不是 SOF 的中断毫无代价。
    std::uint32_t now = DWT->CYCCNT;

    USB_OTG_GlobalTypeDef* const global = global_registers();
    const std::uint32_t status = global->GINTSTS;
    if ((status & USB_OTG_GINTSTS_SOF) == 0U)
        return;

    USB_OTG_DeviceTypeDef* const device = device_registers();
    const std::uint32_t frame = (device->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos;

    // 把时间戳回拨到边沿(见上面"硬件 SOF 捕获")。TIM7 放在 SOF 判断之后读, 每毫秒付一次
    // 而非每个 USB 中断付一次; 它与上面 CYCCNT 之间的几十纳秒每轮相同, 是拟合吸收的常数。
    if (capture_enabled) {
        const std::uint32_t counter = TIM7->CNT;
        const std::uint32_t frame_again =
            (device->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos;
        now -= capture_age_cycles(counter, frame_again != frame);
    }

    // 写 1 清零, 且只写这一位: GINTSTS 还有端点中断的只读位, 读改写既多余又可能
    // 丢边沿。
    global->GINTSTS = USB_OTG_GINTSTS_SOF;

    timebase::note_sof(frame, now);
}

CaptureCounts take_capture_counts() {
    const utility::InterruptLockGuard guard;
    const CaptureCounts result{
        .fresh = static_cast<std::uint16_t>(capture_fresh > 0xFFFFU ? 0xFFFFU : capture_fresh),
        .stale = static_cast<std::uint16_t>(capture_stale > 0xFFFFU ? 0xFFFFU : capture_stale),
    };
    capture_fresh = 0;
    capture_stale = 0;
    return result;
}

void sof_rearm() {
    // 只重设中断使能, 不重跑 capture_init(): 捕获与 DMA 的配置一经设定即稳定。
    if (time_sync_on())
        global_registers()->GINTMSK |= USB_OTG_GINTMSK_SOFM;
}

} // namespace libhcs::firmware::sync

// 对 TinyUSB 设备中断处理函数的链接期介入; 钩子为何不能直接作为向量的第一条
// 语句见 sof.hpp。-Wl,--wrap=dcd_int_handler 由 app CMakeLists 加入。本板的调用方
// 是 tusb.c 的 tusb_int_handler()(向量 OTG_FS_IRQHandler 经 USER CODE 段调它),
// 与 mc02 里向量直调 dcd_int_handler() 不同层, 对 --wrap 是同一件事: 任何目标文件
// 对该符号的引用都在链接期改道。
extern "C" {

// 前置双下划线是链接器 --wrap 的 ABI 约定而非风格选择, 故在此抑制保留标识符与
// 命名检查, 而不是遵守。
// NOLINTBEGIN(bugprone-reserved-identifier,readability-identifier-naming)
void __real_dcd_int_handler(std::uint8_t rhport);

void __wrap_dcd_int_handler(std::uint8_t rhport) {
    libhcs::firmware::sync::sof_isr_entry();
    __real_dcd_int_handler(rhport);
}
// NOLINTEND(bugprone-reserved-identifier,readability-identifier-naming)
}
