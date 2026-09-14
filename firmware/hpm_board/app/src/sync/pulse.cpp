#include "firmware/hpm_board/app/src/sync/pulse.hpp"

#if defined(libhcs_APP_PULSE_TEST) && libhcs_APP_PULSE_TEST

# include <hpm_clock_drv.h>
# include <hpm_gptmr_drv.h>
# include <hpm_iomux.h>
# include <hpm_soc.h>

# include "core/src/utility/assert.hpp"
# include "firmware/hpm_board/app/src/sync/timebase.hpp"
# include "firmware/hpm_board/app/src/utility/interrupt_lock.hpp"

namespace libhcs::firmware::sync::pulse {
namespace {

constexpr std::uint8_t kChannel = 1;
// CMP1 把引脚拉高, CMP0 把它拉低 -- 与 "cmp_initial_polarity_high = false"
// 字面读起来的样子相反, 且是实测而非假定: 目标放 CMP0 时, 对端板捕获到的
// 上升沿恰好晚 kPulseWidthTicks, 宽度减半偏移随之减半(10128 ns -> 5133 ns)。
// 故定时边沿放在 CMP1。[实测 2026-08-20]
constexpr std::uint8_t kRisingComparator = 1;
constexpr std::uint8_t kFallingComparator = 0;
// 24 MHz 下的 10 us。足以越过线缆与接收端施密特触发器, 又远短于最小提前量,
// 下一轮布防之前线路早已回到空闲。
constexpr std::uint32_t kPulseWidthTicks = 240;

// 脉冲必须提前多久布防。比较寄存器须在计数器越过它之前写入, 而写入者是主
// 循环, 故下界覆盖一个宽裕的主循环周期加主机调度余量; 上界让外推距离足够
// 短, 拟合斜率误差远小于一个 tick。
constexpr std::int64_t kMinLeadMicroframes = 16;
constexpr std::int64_t kMaxLeadMicroframes = 8000;

constexpr std::uint32_t kFitPeriodMs = 64;

// 拟合在标称值 +-1% 之外即拒收。两颗独立晶振相差至多几十 ppm, 绝不会是
// 百分之几; 出现百分比只能说明环形缓冲损坏, 或时钟树与 init() 的要求不符。
constexpr std::int64_t kMinTicksPerMicroframeQ16 =
    static_cast<std::int64_t>(kNominalTicksPerMicroframe) * 65536 * 99 / 100;
constexpr std::int64_t kMaxTicksPerMicroframeQ16 =
    static_cast<std::int64_t>(kNominalTicksPerMicroframe) * 65536 * 101 / 100;

// SOF 样本, 抽取存储。只存计数器值: 按构造, 样本恰间隔 kSampleDecimation
// 个 microframe(任何断档都会重置环形), 故条目 i 的 microframe 为
// base_microframe + i * kSampleDecimation。每个元素因此只占一个 32 位字 --
// 这很重要, 因为拟合在主循环里运行, 而中断在并发追加。
std::uint32_t sample_ticks[kSampleCount];
std::uint32_t sample_head = 0;
std::uint32_t sample_count = 0;
std::uint64_t sample_base_microframe = 0;
std::uint64_t next_sample_microframe = 0;
std::uint64_t last_microframe = 0;
bool sampling_started = false;
// 每次追加时递增。拟合在读环形前后各取一次, 有变化即重试 -- 一个 seqlock:
// 既不让 8 kHz 中断被拟合阻塞, 也不让拟合读到更新到一半的环形。
volatile std::uint32_t sample_sequence = 0;

// 发布的直线: ticks = reference_ticks + slope * (microframe - reference)。
bool fit_valid = false;
std::uint64_t fit_reference_microframe = 0;
std::int64_t fit_reference_ticks_q16 = 0;
std::uint32_t fit_slope_q16 = 0;
std::uint32_t last_fit_tick_ms = 0;

// 已捕获、待主循环换算的脉冲。ISR 只存原始计数值; 换算需要拟合结果和一次
// 除法, 两者都不该出现在中断里。
constexpr std::uint32_t kCaptureCapacity = 8;
std::uint32_t capture_ticks[kCaptureCapacity];
std::uint32_t capture_head = 0;
std::uint32_t capture_count = 0;

// 首个主机排程请求把外设拉起之前为 false; 为何不在开机时初始化, 见 init()。
bool hardware_ready = false;

std::uint32_t counter_now() {
    return gptmr_channel_get_counter(HPM_GPTMR0, kChannel, gptmr_counter_type_normal);
}

void reset_samples(std::uint64_t microframe, std::uint32_t ticks) {
    sample_head = 0;
    sample_count = 1;
    sample_ticks[0] = ticks;
    sample_base_microframe = microframe;
    next_sample_microframe = microframe + kSampleDecimation;
    // 已发布的直线引用的是刚结束那个纪元的 microframe 编号, 不只是过时,
    // 而是错的: 依据它排程会差出整个纪元偏移。
    fit_valid = false;
}

void push_sample(std::uint32_t ticks) {
    sample_ticks[(sample_head + sample_count) % kSampleCount] = ticks;
    if (sample_count < kSampleCount) {
        sample_count++;
    } else {
        sample_head = (sample_head + 1U) % kSampleCount;
        sample_base_microframe += kSampleDecimation;
    }
}

// 对当前环形做一遍拟合。环形在读取期间被更新(调用方重试)或样本尚不足时
// 返回 false。
bool try_fit() {
    const std::uint32_t sequence_before = sample_sequence;
    const std::uint32_t head = sample_head;
    const std::uint32_t count = sample_count & ~1U; // 只取整数个半窗
    const std::uint64_t base = sample_base_microframe;
    if (count < 32U)
        return false;

    const std::uint32_t half = count / 2U;
    const std::uint32_t origin = sample_ticks[head];
    std::int64_t first_half_sum = 0;
    std::int64_t second_half_sum = 0;
    for (std::uint32_t index = 0; index < count; index++) {
        // 相对最老样本取有符号值, 计数器 179 s 的回绕就只是普通算术。
        const auto value = static_cast<std::int64_t>(
            static_cast<std::int32_t>(sample_ticks[(head + index) % kSampleCount] - origin));
        if (index < half)
            first_half_sum += value;
        else
            second_half_sum += value;
    }
    if (sample_sequence != sequence_before)
        return false;

    // 两个半窗均值之差: 两均值相距 half*half 个样本, 即 half*half *
    // kSampleDecimation 个 microframe。斜率与最小二乘相差仅百分之几, 且绝不
    // 溢出 -- 对计数器绝对值做最小二乘则很可能溢出。
    const std::int64_t denominator =
        static_cast<std::int64_t>(half) * static_cast<std::int64_t>(half) * kSampleDecimation;
    const std::int64_t slope_q16 = ((second_half_sum - first_half_sum) << 16) / denominator;
    if (slope_q16 < kMinTicksPerMicroframeQ16 || slope_q16 > kMaxTicksPerMicroframeQ16)
        return false;

    // 相位取全部样本的均值, 再折算回最老样本处: 均值样本位于其后
    // (count-1)/2 个抽取周期。
    const std::int64_t mean_q16 = ((first_half_sum + second_half_sum) << 16) / count;
    const std::int64_t mean_offset_microframes =
        static_cast<std::int64_t>(kSampleDecimation) * (count - 1U);
    const std::int64_t reference_ticks_q16 = (static_cast<std::int64_t>(origin) << 16) + mean_q16
                                           - (slope_q16 * mean_offset_microframes) / 2;

    const utility::InterruptLockGuard guard;
    fit_reference_microframe = base;
    fit_reference_ticks_q16 = reference_ticks_q16;
    fit_slope_q16 = static_cast<std::uint32_t>(slope_q16);
    fit_valid = true;
    return true;
}

struct Fit {
    std::uint64_t reference_microframe;
    std::int64_t reference_ticks_q16;
    std::uint32_t slope_q16;
};

bool take_fit(Fit& out) {
    const utility::InterruptLockGuard guard;
    if (!fit_valid)
        return false;
    out = {fit_reference_microframe, fit_reference_ticks_q16, fit_slope_q16};
    return true;
}

void bring_up_hardware() {
    // GPTMR0 不在板级资源组里 -- board.c 只加了 gpio、mchtmr、ptpc 等, 各
    // 驱动自行添加自己的(见 boards/hpm5321/app/board_app.cpp 的
    // init_can_clock)。缺了这句, 外设保持时钟门控, 下方首次访问寄存器就会
    // 永久卡死内核: 无 USB、无 DFU, 只能拉低 PA07 或用 JTAG 救回。
    // [实测 2026-08-20: 正是漏掉这句把两块板都变砖。]
    clock_add_to_group(clock_gptmr0, 0);

    // 用晶振, 不用 PLL -- 这正是在此处选 GPTMR 的全部意义, 见头文件。分频
    // 1 保留完整 24 MHz。
    clock_set_source_divider(clock_gptmr0, clk_src_osc24m, 1);
    core::utility::assert_always(clock_get_frequency(clock_gptmr0) == 24'000'000U);

    // 占用 UART0 引脚。刻意为之且已有文档: 本编译选项开启期间 UART0 不可用。
    HPM_IOC->PAD[IOC_PAD_PB08].FUNC_CTL = IOC_PB08_FUNC_CTL_GPTMR0_COMP_1;
    HPM_IOC->PAD[IOC_PAD_PB09].FUNC_CTL = IOC_PB09_FUNC_CTL_GPTMR0_CAPT_1;

    gptmr_channel_config_t config{};
    gptmr_channel_get_default_config(HPM_GPTMR0, &config);
    // 用硬件捕获来边沿, 计数器在整个量程内自由运行: 一旦重装, 拟合描述的
    // 直线就会被折叠。
    config.mode = gptmr_work_mode_capture_at_rising_edge;
    config.reload = 0xFFFFFFFFU;
    config.enable_cmp_output = true;
    config.cmp_initial_polarity_high = false;
    // 两个比较器都停到够不着的位置, 无人请求前绝不发出脉冲。
    config.cmp[0] = 0xFFFFFFFFU;
    config.cmp[1] = 0xFFFFFFFFU;
    core::utility::assert_always(
        gptmr_channel_config(HPM_GPTMR0, kChannel, &config, false) == status_success);

    gptmr_clear_status(HPM_GPTMR0, GPTMR_CH_CAP_IRQ_MASK(kChannel));
    gptmr_enable_irq(HPM_GPTMR0, GPTMR_CH_CAP_IRQ_MASK(kChannel));
    intc_m_enable_irq_with_priority(IRQn_GPTMR0, 2);

    gptmr_start_counter(HPM_GPTMR0, kChannel);
    hardware_ready = true;
}

} // namespace

void init() {
    // 刻意留空。本模块触及的一切 -- 外设时钟门控、引脚复用、定时器通道、
    // 中断 -- 都推迟到首个主机排程请求时拉起, 该请求只可能在 USB 枚举完成、
    // 会话建立之后到来。
    //
    // 出发点是可恢复性而非整洁。这是一条实验性测量路径: 若开机期间在其中
    // 出错, 板子将没有 USB、也就没有 DFU, 且板上无按键(PA07 是 JTAG_TMS),
    // 唯一退路是调试器。推迟初始化把这类故障从"变砖"降级为"诊断不可用",
    // 烧录通路仍在。
}

void note_sof(std::uint64_t microframe) {
    if (!hardware_ready)
        return;

    // 一次普通寄存器读, 挂在已经在读 FRINDEX 和机器定时器的同一中断里, 再
    // 减去 SOF 包本身在线缆上的传输耗时。不减的话, 全速板的整条轴比高速板
    // 落后约 2.8 us -- 实测值, 与包耗时之差吻合到 1% 以内。各板减去各自的
    // 包耗时后, 不论速率, 所有板都对齐到包起始的那一瞬。
    const std::uint32_t ticks = counter_now() - (timebase::sof_packet_delay_ns() * 24U) / 1000U;

    // 步进不是 +1 意味着编号动了, 而不只是时间: 或是丢失 SOF 中断, 或是
    // (更常见)时间线从新起的主机进程拿到 anchor, 纪元向任一方向整体平移
    // 数秒。环形与拟合描述的都是旧纪元, 一并丢弃。此判据必须同时接受后退:
    // 只认前进曾把采样器永久卡死 -- 拟合冻结在死纪元上, 整轮运行拒绝所有
    // 排程, 症状与断线一模一样。[实测 2026-08-20]
    // "相邻"指一个 SOF 间隔: 高速是一个 microframe, 全速是八个 -- FRINDEX
    // 两种速率都数 microframe, 但全速端口每 1 ms 帧只收到一个 SOF。在此
    // 硬编码 +1 会让全速链路上的采样器以纪元跳变完全相同的方式卡死: 每个
    // SOF 重置环形, 拟合永远发不出来。
    const std::uint64_t step = timebase::microframes_per_sof();
    const bool consecutive = sampling_started && microframe == last_microframe + step;
    last_microframe = microframe;
    if (!consecutive) {
        sampling_started = true;
        reset_samples(microframe, ticks);
        sample_sequence = sample_sequence + 1U;
        return;
    }
    if (microframe != next_sample_microframe)
        return;
    push_sample(ticks);
    next_sample_microframe += kSampleDecimation;
    sample_sequence = sample_sequence + 1U;
}

void poll(std::uint32_t tick_ms) {
    if (!hardware_ready)
        return;
    if (tick_ms - last_fit_tick_ms < kFitPeriodMs)
        return;
    last_fit_tick_ms = tick_ms;
    // 三次尝试已属宽裕: 样本每 8 ms 一个, 求和只耗微秒级。三次全败则沿用
    // 上一条拟合, 一个重拟合周期内它的误差仍远小于一个 tick。
    for (std::uint32_t attempt = 0; attempt < 3U; attempt++) {
        if (try_fit())
            return;
    }
}

void isr_handler() {
    if (!gptmr_check_status(HPM_GPTMR0, GPTMR_CH_CAP_IRQ_MASK(kChannel)))
        return;
    gptmr_clear_status(HPM_GPTMR0, GPTMR_CH_CAP_IRQ_MASK(kChannel));

    // 该值由硬件在边沿处锁存, 中断何时运行无关紧要 -- 这正是如此实现的
    // 全部原因。
    const std::uint32_t ticks =
        gptmr_channel_get_counter(HPM_GPTMR0, kChannel, gptmr_counter_type_rising_edge);
    if (capture_count >= kCaptureCapacity)
        return;
    capture_ticks[(capture_head + capture_count) % kCaptureCapacity] = ticks;
    capture_count++;
}

bool schedule(std::uint64_t microframe) {
    if (!hardware_ready)
        bring_up_hardware();

    Fit fit{};
    if (!take_fit(fit))
        return false;

    const auto ahead_of_reference =
        static_cast<std::int64_t>(microframe - fit.reference_microframe);
    // 提前量从计数器当前值量起, 而非从拟合参考点 -- 后者位于约半个窗口
    // 之前。
    const std::int64_t now_offset_microframes =
        (static_cast<std::int64_t>(static_cast<std::int32_t>(
             counter_now() - static_cast<std::uint32_t>(fit.reference_ticks_q16 >> 16)))
         << 16)
        / static_cast<std::int64_t>(fit.slope_q16);
    const std::int64_t lead = ahead_of_reference - now_offset_microframes;
    if (lead < kMinLeadMicroframes || lead > kMaxLeadMicroframes)
        return false;

    const std::int64_t target_q16 =
        fit.reference_ticks_q16 + static_cast<std::int64_t>(fit.slope_q16) * ahead_of_reference;
    const auto target =
        static_cast<std::uint32_t>(static_cast<std::uint64_t>(target_q16 >> 16) & 0xFFFFFFFFU);

    // 输出在 CMP1 匹配时变高、CMP0 匹配时变低(同文件顶部实测结论), 因此
    // 每轮两个比较器都必须更新。只更新 CMP1 恰好能成功一次: 随后引脚一直
    // 保持高(CMP0 停在 0xFFFFFFFF), 第二次 CMP1 匹配不改变电平, 对端便无
    // 边沿可捕获。
    gptmr_update_cmp(HPM_GPTMR0, kChannel, kRisingComparator, target);
    gptmr_update_cmp(HPM_GPTMR0, kChannel, kFallingComparator, target + kPulseWidthTicks);
    return true;
}

bool take_capture(std::uint64_t& microframe_q16) {
    Fit fit{};
    std::uint32_t ticks = 0;
    {
        const utility::InterruptLockGuard guard;
        if (capture_count == 0 || !fit_valid)
            return false;
        ticks = capture_ticks[capture_head];
        capture_head = (capture_head + 1U) % kCaptureCapacity;
        capture_count--;
        fit = {fit_reference_microframe, fit_reference_ticks_q16, fit_slope_q16};
    }

    // 沿脉冲布防所用的同一条直线做逆变换。不做夹逼搜索, 也不在相邻两个
    // SOF 样本间插值: 那会把拟合本要平均掉的单样本中断抖动原样交回。
    const auto delta_ticks = static_cast<std::int64_t>(static_cast<std::int32_t>(
        ticks - static_cast<std::uint32_t>(fit.reference_ticks_q16 >> 16)));
    const std::int64_t delta_ticks_q16 = (delta_ticks << 16) - (fit.reference_ticks_q16 & 0xFFFF);
    const std::int64_t offset_q16 =
        (delta_ticks_q16 << 16) / static_cast<std::int64_t>(fit.slope_q16);

    const std::int64_t absolute_q16 =
        static_cast<std::int64_t>(fit.reference_microframe << 16) + offset_q16;
    if (absolute_q16 < 0)
        return false;
    microframe_q16 = static_cast<std::uint64_t>(absolute_q16);
    return true;
}

std::uint32_t measured_ticks_per_microframe_q16() {
    const utility::InterruptLockGuard guard;
    return fit_valid ? fit_slope_q16 : 0U;
}

} // namespace libhcs::firmware::sync::pulse

extern "C" {
SDK_DECLARE_EXT_ISR_M(IRQn_GPTMR0, hcs_gptmr0_isr)
void hcs_gptmr0_isr(void) { libhcs::firmware::sync::pulse::isr_handler(); }
}

#endif
