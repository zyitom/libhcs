#include "firmware/mc02/app/src/sync/sof.hpp"

#include <cstdint>

#include <main.h>

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

// ---- 硬件 SOF 捕获, TIM2 ITR5 ----
//
// 本芯片上 USB1_OTG_HS_SOF 接到 TIM2 的内部触发 5, 把 TIM2 捕获通道映射到 TRC
// 即可在 SOF 边沿锁存计数, 路径上没有软件 -- 是 HPM TRGM->PTPC 路由的直接
// 对应物。[实测 2026-09-07: SMCR.TS 扫 ITR0..ITR13, 仅 ITR5 以 SOF 速率出捕获,
//  其余来源全为 0; 捕获间隔下限 999 tick(TIM2 @ 1 MHz), 即 1 ms 全速帧。
//  TIM5 无此来源, 其 ITR5 是 H723 没有的 USB2_OTG_FS。]
//
// 它不取代中断时间戳, 而是修正: 捕获给出边沿实际比本 handler 早到多久, 从周期
// 计数器上减去它, 即从每个样本中扣除中断入口延迟 -- 常数与抖动一并扣除。
//
// 只在确实更优时启用: 捕获在边沿上精确, 但上限取决于被锁存计数器的 tick。中断路径
// 自身单样本 sigma 为 0.1365 us (75 CPU 周期)[实测 2026-09-07], 于是:
//
//   TIM2 @ 275 MHz, 2 cycles/tick   tick 远低于抖动 -> 抖动被扣除
//   TIM2 @ 1 MHz, 550 cycles/tick   tick 远高于抖动 -> 无效果
//
// 当前 .ioc 给 TIM2 Prescaler = 0、Period = 5499999(275 MHz, 仍是 50 Hz 舵机 PWM),
// 属前一种, capture_init() 打开门控; 开与关的实测对比见 timebase.hpp 第 3 点。后一种
// 是 TIM2 早先的配置, 值得说清, 因为直觉是错的且实测过: 1 MHz 下强开修正, 拟合残差
// 毫无变化(0.0314 us, 未修正 0.031-0.039 us, 0 异常, 拟合速率相同)。它不会注入 tick
// 量化噪声: 抖动远小于 tick 时算出的 age 几乎总是同一个整数, 修正退化为常数, 而减常数
// 不改变标准差, 拟合本就把该常数吸收进了偏移。粗捕获无害只是无用, 故那种配置下门控
// 关闭, 省掉每毫秒两次多余的外设读。

std::uint32_t capture_reload = 0;
std::uint32_t capture_cycles_per_tick = 0;
std::uint32_t capture_max_age_ticks = 0;
bool capture_enabled = false;

// 中断路径的单样本 sigma, 单位 CPU 周期, 本板实测。tick 须远小于该值, 捕获算出
// 的 age 才跟得上抖动而非舍成常数。
constexpr std::uint32_t kIsrSigmaCycles = 75;
constexpr std::uint32_t kCaptureWorthwhileCycles = kIsrSigmaCycles / 3U;

// 一个 TIM2 内核时钟周期折合的 CPU 周期数, 从 RCC 分频寄存器读出而非写死: .ioc 改
// HPRE、D2PPRE1 或 TIMPRE 都会改变它, 写死的值会让修正按错误比例缩放后照减, 把噪声
// 加回去而不是退化为无修正。
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

// Only referenced from an if-constexpr branch that TIME_SYNC=OFF discards.
[[maybe_unused]] void capture_init() {
    if constexpr (!timebase::kEnabled)
        return;

    // TIM2 与 PA0/PA2 的舵机 PWM 共用, 本函数只拿 CH2 与触发选择。两者须仍是复位值
    // 才接管: 目前 MX_TIM2_Init 只配 CH1/CH3 的 PWM。哪天 .ioc 给 CH2 配了功能, 或给
    // TIM2 配了从模式、外部时钟, 下面的读改写会与之互相覆盖且毫无报错 -- 那时让出
    // TIM2, capture_enabled 保持 false, 时间基退回纯中断时间戳。
    constexpr std::uint32_t k_channel2_bits =
        TIM_CCMR1_CC2S | TIM_CCMR1_OC2FE | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M | TIM_CCMR1_OC2CE;
    if ((TIM2->CCER & TIM_CCER_CC2E) != 0U || (TIM2->CCMR1 & k_channel2_bits) != 0U
        || (TIM2->SMCR & (TIM_SMCR_SMS | TIM_SMCR_ECE)) != 0U)
        return;

    // CH2 映射到 TRC。CH1 与 CH3 的 PWM 位不动; SMS 保持 0, 计数器从不从属于触发 --
    // 只有捕获通道消费它。
    TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_CC2S) | (0x3UL << TIM_CCMR1_CC2S_Pos);
    TIM2->SMCR = (TIM2->SMCR & ~TIM_SMCR_TS) | TIM_TS_ITR5;
    TIM2->CCER |= TIM_CCER_CC2E;
    (void)TIM2->CCR2;

    // 以下两项在此缓存一次, ISR 里不再读。于是共用 TIM2 的一方有两条约束: ARR 与 PSC
    // 不得在运行时改写(PWM 频率随之冻结), 计数器不得复位(不能写 UG, 不能重跑
    // MX_TIM2_Init) -- 否则跨越改动的样本 age 算错, 落在下面的半毫秒窗口内就是静默的
    // 坏样本。gpio.hpp 已按此只写 CCR。
    capture_reload = TIM2->ARR + 1U;
    capture_cycles_per_tick = cpu_cycles_per_timer_clock() * (TIM2->PSC + 1U);
    // 半毫秒, 单位 tick。比这更老的捕获不可能属于正在处理的 SOF, 拒绝它而非变成
    // 荒谬修正。
    capture_max_age_ticks = (timebase::kCyclesPerMicrosecond * 500U) / capture_cycles_per_tick;
    capture_enabled = capture_cycles_per_tick < kCaptureWorthwhileCycles && capture_reload > 1U;
}

// 从中断时间戳中扣除的周期数; 捕获不可信时为 0(恰好退化为无捕获时的行为)。
//
// 与 sof_isr_entry() 入口处读的 CYCCNT 配对, 中间隔着 GINTSTS、DSTS、CCR2 三次外设
// 访问。直觉上这段间隔的抖动会进样本, 该在这里紧挨 CNT 再读一次 CYCCNT; 实测没有可见
// 差别, 故保持现写法: 改成 CNT 后紧跟 CYCCNT 成对读, 单样本 sigma 空载 0.0194-0.0202
// us、灌满下行 0.0201-0.0212 us; 本写法同条件 0.0183-0.0207 / 0.0177-0.0196 us, 范围
// 重叠。[实测 2026-09-16, mc02_time_sync_test 各 60 s, 交替烧录]
std::uint32_t capture_age_cycles() {
    if (!capture_enabled)
        return 0;
    const std::uint32_t capture = TIM2->CCR2;
    const std::uint32_t counter = TIM2->CNT;
    // TIM2 在 ARR + 1 处回绕而非 2^32, 故差值对 reload 取模, 不能用普通无符号
    // 减法。
    const std::uint32_t age =
        counter >= capture ? counter - capture : (counter + capture_reload) - capture;
    if (age > capture_max_age_ticks)
        return 0;
    return age * capture_cycles_per_tick;
}

} // namespace

void sof_isr_entry() {
    if constexpr (!timebase::kEnabled)
        return;

    // 先读周期计数器, 早于"这是否 SOF"的判断。GINTSTS 是一次 AHB 读, 耗时数十
    // 纳秒且可能被总线流量卡住; 时间戳取在其后会把这一可变停顿折进每个样本。
    // CYCCNT 是核心本地寄存器, 两个周期, 无条件读对最终不是 SOF 的中断毫无代价。
    std::uint32_t now = DWT->CYCCNT;

    USB_OTG_GlobalTypeDef* const global = global_registers();
    const std::uint32_t status = global->GINTSTS;
    if ((status & USB_OTG_GINTSTS_SOF) == 0U)
        return;

    const std::uint32_t frame =
        (device_registers()->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos;

    // 把时间戳回拨到硬件锁存的边沿。放在 SOF 判断之后而非之前读, 使这些外设
    // 访问每毫秒付一次而非每个 USB 中断付一次; 此处与上面读 CYCCNT 之间的几十
    // 纳秒每轮相同, 是拟合吸收的常数(其抖动实测可忽略, 见 capture_age_cycles())。
    now -= capture_age_cycles();

    // 写 1 清零, 且只写这一位: GINTSTS 还有端点中断的只读位, 读改写既多余又可能
    // 丢边沿。
    global->GINTSTS = USB_OTG_GINTSTS_SOF;

    timebase::note_sof(frame, now);
}

void sof_init() {
    if constexpr (timebase::kEnabled) {
        capture_init();
        global_registers()->GINTMSK |= USB_OTG_GINTMSK_SOFM;
    }
}

void sof_rearm() {
    // 只重设中断使能, 不重跑 capture_init(): 捕获配置一经设定即稳定, 以 tick
    // 速率重写 CCMR1/SMCR 纯属无谓的外设流量。
    if constexpr (timebase::kEnabled)
        global_registers()->GINTMSK |= USB_OTG_GINTMSK_SOFM;
}

} // namespace libhcs::firmware::sync

#if defined(libhcs_APP_TIME_SYNC) && libhcs_APP_TIME_SYNC

// 对 TinyUSB 设备中断处理函数的链接期介入; 钩子为何不能直接作为向量的第一条
// 语句见 sof.hpp。-Wl,--wrap=dcd_int_handler 仅在本配置下由 app CMakeLists 加入
// -- 时间基关闭时两个符号都不存在于链接中。
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

#endif
