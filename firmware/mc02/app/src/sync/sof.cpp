#include "firmware/mc02/app/src/sync/sof.hpp"

#include <atomic>
#include <cstdint>

#include <main.h>

#include "firmware/mc02/app/src/sync/sof_capture.hpp"
#include "firmware/mc02/app/src/sync/timebase.hpp"

namespace libhcs::firmware::sync {
namespace {

// DWC2 寄存器块。USB_OTG_HS_PERIPH_BASE 是全局块, 设备块在距其固定偏移处。
// 经 CMSIS 访问而非 TinyUSB 的 dwc2_regs_t, 使这里不依赖 TinyUSB 的内部布局。
USB_OTG_GlobalTypeDef* global_registers() {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<USB_OTG_GlobalTypeDef*>(USB_OTG_HS_PERIPH_BASE);
}

USB_OTG_DeviceTypeDef* device_registers() {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<USB_OTG_DeviceTypeDef*>(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
}

// ---- 硬件 SOF 捕获, TIM5 ITR7 ----
//
// 本芯片上 USB1_OTG_HS 的 SOF 接到两个定时器的内部触发: TIM2 的 ITR5 与 TIM5 的 ITR7
// [RM0468 Rev 3 Table 94 p.554 与 Table 355 p.1712, 两表一致]。把捕获通道映射到 TRC
// 即可在 SOF 边沿锁存计数, 路径上没有软件 -- 是 HPM TRGM->PTPC 路由的直接对应物。
//
// 用 TIM5 不用 TIM2(2026-10-05 起): TIM5 是 32 位、不占引脚, 整个交给共享时基 --
// Prescaler = 0(275 MHz)、Period = 0xFFFFFFFF, 计数器满 32 位回绕。于是
//   - TIM2 完全还给 PA0/PA2 的 PWM, 周期随便改;
//   - FDCAN 时间戳所用的 TIM3 与 TIM5 吃同一个定时器时钟, 2^32 又是 TIM3 一整圈的整数
//     倍, 两者恒为整数关系, CAN 帧的时间戳可以无损换到 TIM5 上(sof_capture.cpp)。
// 板上 1/4 us 时间戳源原本是 TIM5, 已挪到 TIM23(timer/timer.hpp)。
// [更正: 2026-09-07 的扫描记录写"TIM5 无此来源", 与 RM0468 两张表矛盾。H7 的 TS 字段
//  编码不连续(ITR4 起是 01000 而不是 00100), 那次扫描很可能写错了编码。以 kTimeStatus
//  里的新鲜锁存计数为准: TIM5 ITR7 若真没有 SOF, 新鲜数恒为 0。未上板。]
//
// 它不取代中断时间戳, 而是修正: 捕获给出边沿实际比本 handler 早到多久, 从周期
// 计数器上减去它, 即从每个样本中扣除中断入口延迟 -- 常数与抖动一并扣除。
//
// 只在确实更优时启用: 捕获在边沿上精确, 但上限取决于被锁存计数器的 tick。中断路径
// 自身单样本 sigma 为 0.1365 us (75 CPU 周期)[实测 2026-09-07], 于是:
//
//   275 MHz, 2 cycles/tick   tick 远低于抖动 -> 抖动被扣除
//   1 MHz, 550 cycles/tick   tick 远高于抖动 -> 无效果
//
// 后一种是早先的配置, 值得说清, 因为直觉是错的且实测过(当时在 TIM2 上): 1 MHz 下强开
// 修正, 拟合残差毫无变化(0.0314 us, 未修正 0.031-0.039 us, 0 异常, 拟合速率相同)。它不会
// 注入 tick 量化噪声: 抖动远小于 tick 时算出的 age 几乎总是同一个整数, 修正退化为常数,
// 而减常数不改变标准差, 拟合本就把该常数吸收进了偏移。粗捕获无害只是无用, 故那种配置下
// 门控关闭, 省掉每毫秒两次多余的外设读。

std::uint32_t capture_cycles_per_tick = 0;
std::uint32_t capture_max_age_ticks = 0;
bool capture_enabled = false;

// 中断路径的单样本 sigma, 单位 CPU 周期, 本板实测。tick 须远小于该值, 捕获算出
// 的 age 才跟得上抖动而非舍成常数。
constexpr std::uint32_t kIsrSigmaCycles = 75;
constexpr std::uint32_t kCaptureWorthwhileCycles = kIsrSigmaCycles / 3U;

// 一个定时器内核时钟周期(APB1 定时器: TIM2..5、TIM23/24)折合的 CPU 周期数, 从 RCC
// 分频寄存器读出而非写死: .ioc 改 HPRE、D2PPRE1 或 TIMPRE 都会改变它, 写死的值会让
// 修正按错误比例缩放后照减, 把噪声加回去而不是退化为无修正。
//
// CPU 时钟经 HPRE 得 HCLK, 再经 D2PPRE1 得 PCLK1。定时器时钟是 PCLK1 的 2 倍
// (TIMPRE=1 时 4 倍), 但不超过 HCLK -- 见 HAL __HAL_RCC_TIMCLKPRESCALER 的说明。
// 分频全是 2 的幂, 比例是精确整数; 刻意不走 HAL_RCC_GetHCLKFreq(), 那条路要经 PLL
// 的浮点频率计算, 算出的频率未必恰好整除。
std::uint32_t cpu_cycles_per_timer_clock() {
    const std::uint32_t hpre_shift =
        D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_HPRE) >> RCC_D1CFGR_HPRE_Pos] & 0x1FU;
    const std::uint32_t apb1_shift =
        D1CorePrescTable[(RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos] & 0x1FU;
    const std::uint32_t timer_gain_shift = (RCC->CFGR & RCC_CFGR_TIMPRE) != 0U ? 2U : 1U;
    const std::uint32_t timer_shift =
        apb1_shift > timer_gain_shift ? apb1_shift - timer_gain_shift : 0U;
    return 1UL << (hpre_shift + timer_shift);
}

// 第一次 time_sync_start() 时调用一次。
void capture_init() {
    // TIM5 整个归共享时基, app.cpp 上电时调 MX_TIM5_Init()(不开中断、不开通道, 计数器
    // 由这里起走; 不开时间基准的板子上它什么也不做)。计数器须满 32 位回绕: 本文件与
    // sof_capture.cpp 的差值都用普通无符号减法。不满足(.ioc 被改过)就不接管, 时间基退回
    // 纯中断时间戳。
    if (TIM5->ARR != 0xFFFF'FFFFU)
        return;

    // CH2 映射到 TRC, 触发选 ITR7(USB1 OTG_HS SOF)。SMS 保持 0: 计数器从不从属于触发,
    // 只有捕获通道消费它。
    TIM5->CCMR1 = (TIM5->CCMR1 & ~TIM_CCMR1_CC2S) | (0x3UL << TIM_CCMR1_CC2S_Pos);
    TIM5->SMCR = (TIM5->SMCR & ~(TIM_SMCR_TS | TIM_SMCR_SMS)) | TIM_TS_ITR7;
    TIM5->CCER |= TIM_CCER_CC2E;
    TIM5->CR1 |= TIM_CR1_CEN;
    (void)TIM5->CCR2;

    // 缓存一次, ISR 里不再读。约束: TIM5 的 PSC / ARR 不得在运行时改写, 计数器不得复位
    // (不能写 UG, 不能重跑 MX_TIM5_Init) -- 否则跨越改动的样本 age 算错, 且 CAN 帧时间戳
    // 依赖的 TIM3 <-> TIM5 关系随之失效(sof_capture.cpp 每毫秒抽查, 失效即停打戳)。
    capture_cycles_per_tick = cpu_cycles_per_timer_clock() * (TIM5->PSC + 1U);
    // 半毫秒, 单位 tick。比这更老的捕获不可能属于正在处理的 SOF, 拒绝它而非变成
    // 荒谬修正。
    capture_max_age_ticks = (timebase::kCyclesPerMicrosecond * 500U) / capture_cycles_per_tick;
    capture_enabled = capture_cycles_per_tick < kCaptureWorthwhileCycles;

    // 同一路捕获也是 CAN 帧时间戳的 SOF 一侧(sof_capture.hpp)。
    if (capture_enabled)
        sof_capture::configure(capture_cycles_per_tick);
}

// 从中断时间戳中扣除的周期数; 捕获不可信时为 0(恰好退化为无捕获时的行为)。
//
// 与 sof_isr_entry() 入口处读的 CYCCNT 配对, 中间隔着 GINTSTS、DSTS、CCR2 三次外设
// 访问。直觉上这段间隔的抖动会进样本, 该在这里紧挨 CNT 再读一次 CYCCNT; 实测没有可见
// 差别, 故保持现写法: 改成 CNT 后紧跟 CYCCNT 成对读, 单样本 sigma 空载 0.0194-0.0202
// us、灌满下行 0.0201-0.0212 us; 本写法同条件 0.0183-0.0207 / 0.0177-0.0196 us, 范围
// 重叠。[实测 2026-09-16, mc02_time_sync_test 各 60 s, 交替烧录; 当时在 TIM2 上]
std::uint32_t capture_age_cycles(std::uint32_t capture, std::uint32_t counter) {
    if (!capture_enabled)
        return 0;
    // TIM5 满 32 位回绕, 普通无符号减法即是模 2^32 的差。
    const std::uint32_t age = counter - capture;
    if (age > capture_max_age_ticks)
        return 0;
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
    sof_capture::reset();
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

    // 把时间戳回拨到硬件锁存的边沿。放在 SOF 判断之后而非之前读, 使这些外设
    // 访问每毫秒付一次而非每个 USB 中断付一次; 此处与上面读 CYCCNT 之间的几十
    // 纳秒每轮相同, 是拟合吸收的常数(其抖动实测可忽略, 见 capture_age_cycles())。
    std::uint32_t capture = 0;
    std::uint32_t counter = 0;
    if (capture_enabled) {
        capture = TIM5->CCR2;
        counter = TIM5->CNT;
    }
    now -= capture_age_cycles(capture, counter);

    // 读完锁存值再看一次帧号: 变了说明本次处理被推迟到了下一个 SOF 之后, 锁存值
    // 属于那个更新的 SOF。CAN 帧时间戳那一侧据此拒绝配对。
    const std::uint32_t frame_again = (device->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos;

    // 写 1 清零, 且只写这一位: GINTSTS 还有端点中断的只读位, 读改写既多余又可能
    // 丢边沿。
    global->GINTSTS = USB_OTG_GINTSTS_SOF;

    timebase::note_sof(frame, now);
    if (capture_enabled)
        sof_capture::note_sof(frame, capture, counter, frame_again);
}

void sof_rearm() {
    // 只重设中断使能, 不重跑 capture_init(): 捕获配置一经设定即稳定, 以 tick
    // 速率重写 CCMR1/SMCR 纯属无谓的外设流量。
    if (time_sync_on())
        global_registers()->GINTMSK |= USB_OTG_GINTMSK_SOFM;
}

} // namespace libhcs::firmware::sync

// 对 TinyUSB 设备中断处理函数的链接期介入; 钩子为何不能直接作为向量的第一条
// 语句见 sof.hpp。-Wl,--wrap=dcd_int_handler 由 app CMakeLists 加入。
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
