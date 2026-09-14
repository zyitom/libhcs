#include "firmware/mc02/app/src/sync/timebase.hpp"

#if defined(libhcs_APP_TIME_SYNC) && libhcs_APP_TIME_SYNC

# include <algorithm>

# include <main.h>

# include "firmware/mc02/app/src/timer/timer.hpp"
# include "firmware/mc02/app/src/utility/interrupt_lock.hpp"

namespace libhcs::firmware::sync::timebase {
namespace {

// 微帧轴在全仓库所有板上是 14 位宽, 与底层硬件计数器的形态无关。EHCI FRINDEX
// 是 14 位微帧; DWC2 全速给 11 位帧, 乘 8 同为 16384 微帧(2.048 s)模数。两者
// 一致才让一个主机锚点对所有板通用。
constexpr std::uint32_t kMicroframeMask = 0x3FFFU;
constexpr std::uint64_t kMicroframeModulus = 0x4000U;

// DSTS.ENUMSPD 的 DWC2 编码: 0 高速, 1 全速(30/60 MHz PHY), 2 低速, 3 全速
// (内置 48 MHz PHY)。mc02 恒枚举为 3; 保留高速分支, 以免此代码被复用到带 ULPI
// PHY 的芯片上时悄悄出错。
constexpr std::uint32_t kEnumSpeedHigh = 0U;

constexpr std::uint32_t kFitPeriodMs = 20U;

// 拟合斜率的接受窗口。偏离标称几个百分点以上不是晶振偏差, 是坏窗口; 拒绝它,
// 坏拟合才不会发布出去。
constexpr std::int64_t kNominalQ16 = static_cast<std::int64_t>(kNominalCyclesPerMicroframe) << 16U;

// x = 0..N-1 等距样本的 sum((x - mean)^2), 是拟合无需计算的常数: N(N^2-1)/12。
constexpr std::int64_t kSxx = static_cast<std::int64_t>(kSampleCount)
                            * ((static_cast<std::int64_t>(kSampleCount) * kSampleCount) - 1) / 12;

// ==== ISR 专有状态: 仅 note_sof() 写, 主循环在 utility::InterruptLockGuard 下读 ====
// 用普通标量而非 atomic: 每个读者都持该锁; 且与计数器不同, 这些量必须作为一致
// 的整组读取, 单变量原子性提供不了这一点。

// 本地、自上电起算的微帧计数。以首个帧读数播种, 使 (counter mod 16384) 从此恒
// 等于硬件计数器的贡献, 锚点才得以是 16384 的纯倍数。
std::uint64_t counter = 0;
bool counter_seeded = false;
std::uint32_t previous_index = 0;

// DWT->CYCCNT 的 64 位扩展。550 MHz 下每 7.81 s 回绕; SOF 每 1 ms 到来, 故低字
// 变小的情形在此是明确的回绕判据, 余量达三个半数量级。
std::uint32_t previous_time = 0;
std::uint32_t time_high = 0;
bool time_seeded = false;

// TIM5, 在同一中断内采样, 纯粹为了让状态报告能把微帧与本固件其余部分打时间戳
// 所用的时钟配成一对。
//
// 拟合跑在 CYCCNT 上, 因为 TIM5 量化到 1 us(timer.hpp: 1 MHz, 按 CNT << 2 读),
// 与被测抖动同量级。但其余所有上行记录 -- IMU、带时间戳的 GPIO -- 带的都是
// TIM5 的 quarter-us 戳, 且 CYCCNT 与 TIM5 每次上电的起点互不相关。若状态发布
// CYCCNT 派生的时间, 主机将持有两个永远无法互相关联的时钟, 任何遥测记录都无法
// 落到微帧轴上。发布 TIM5 才让该轴对时间基准之外的用途可用。
//
// 在周期计数器之后采样, 使这次额外的外设读(D2 总线上约 0.25 us)不会拉长时间戳
// 路径。1 kHz。
std::uint32_t previous_timer_quarter_us = 0;

data::TimeState state = data::TimeState::kInvalid;
std::uint32_t anomaly_count = 0;
std::int64_t anchor_offset = 0;
bool anchored = false;

// 拟合样本环形缓冲。sample[i] 是周期计数器在微帧 (ring_oldest_microframe +
// i * kSampleDecimation) 处的低 32 位, 自 ring_head 起按插入顺序排列。
std::uint32_t sample[kSampleCount];
std::uint32_t ring_head = 0;
std::uint32_t ring_count = 0;
std::uint64_t ring_oldest_microframe = 0;

// ==== 拟合结果: 仅 poll() 写, 所有读者持同一把锁 ====
// 参考点取整个周期(1.8 ns)而非 Q16: 拟合平均 128 个样本正是为了让相位优于单个
// 样本的抖动, 1.8 ns 的舍入比被平均的抖动低三个数量级。斜率保持 Q16: 那里百万
// 分之几会随外推累积。
//
// Q16 cycles per microframe = 68750 * 65536 = 4.5e9, 装不进 hpm_board 4 MHz
// 定时器所用的 uint32。此处以 64 位持有, 仅在换算成 quarter-us(回到 500 * 65536)
// 之后、输出途中收窄。
bool fit_valid = false;
std::uint64_t fit_reference_microframe = 0;
std::uint64_t fit_reference_time = 0;
std::uint64_t fit_cycles_per_microframe_q16 = 0;

// 样本外预测误差, 两次报告之间累积, 单位 Q16 cycles。
//
// 每个抽取样本先按当前已发布的拟合(在本样本存在之前算出)预测, 然后才入窗。
// 因此残差是真正的预报误差 -- 正是定时动作会经历的 -- 而非用被测点自身拟合出
// 的线内残差。
std::int64_t residual_sum_q16 = 0;
std::uint32_t residual_count = 0;
std::uint64_t residual_abs_max_q16 = 0;

std::uint32_t last_fit_tick = 0;

void reset_fit_window() {
    ring_head = 0;
    ring_count = 0;
    fit_valid = false;
    fit_cycles_per_microframe_q16 = 0;
    residual_sum_q16 = 0;
    residual_count = 0;
    residual_abs_max_q16 = 0;
}

void invalidate() {
    state = data::TimeState::kInvalid;
    anchored = false;
    counter_seeded = false;
    reset_fit_window();
}

std::uint64_t now_extended(std::uint32_t low) {
    return (static_cast<std::uint64_t>(time_high) << 32U) | low;
}

USB_OTG_DeviceTypeDef* device_registers() {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<USB_OTG_DeviceTypeDef*>(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
}

std::uint32_t enumerated_speed() {
    const auto status = static_cast<std::uint32_t>(device_registers()->DSTS);
    return (status & static_cast<std::uint32_t>(USB_OTG_DSTS_ENUMSPD))
        >> static_cast<std::uint32_t>(USB_OTG_DSTS_ENUMSPD_Pos);
}

// 当前端口速度下 DSTS.FNSOF 一个计数折合的微帧数。
//
// 全速差异全在此函数。DWC2 手册对 FNSOF 的描述是收到 SOF 的帧或微帧号: 高速
// 枚举时是微帧号, 否则是帧号。mc02 恒为后者, 每个计数折合 8 微帧, 寄存器只承载
// 主机送上总线的 11 位帧号。
//
// 全速下掩到 11 位而非寄存器满 14 位不是保守猜测, 是实测宽度。FNSOF 是 14 位
// 字段, 但全速下只有主机送上总线的那 11 位被填充: 探测原始寄存器 30 s(约 14 个
// 整回绕), 最大值恰为 2047, 故 bit 13..11 恒为 0, 没有更宽的计数器可用。
// [实测 2026-09-07。] hpm_board 有真实 14 位, 只因其高速下该字段是 11 位帧号
// 加 3 位微帧号。
//
// 故主机锚点需消解的回绕在两板上同为 2048 帧 = 16384 微帧 = 2.048 s, 本板无法
// 把它做得更长。
std::uint32_t frame_scale() { return enumerated_speed() == kEnumSpeedHigh ? 1U : 8U; }

// 向负无穷取整的整除。C++ 向零截断, 会让下面的回绕消解关于 0 不对称, 跨在原点
// 两侧的两块板可能选中不同的回绕。
std::int64_t floor_div(std::int64_t numerator, std::int64_t denominator) {
    const std::int64_t quotient = numerator / denominator;
    const std::int64_t remainder = numerator % denominator;
    return (remainder != 0 && ((remainder < 0) != (denominator < 0))) ? quotient - 1 : quotient;
}

// cycles 换算成 quarter-us。整微帧时精确(68750 -> 500), 其余截断; 仅在协议要求
// 该单位处使用。
std::int64_t cycles_to_quarter_us(std::int64_t cycles) {
    return cycles * 4 / static_cast<std::int64_t>(kCyclesPerMicrosecond);
}

std::int64_t quarter_us_to_cycles(std::int64_t quarter_us) {
    return quarter_us * static_cast<std::int64_t>(kCyclesPerMicrosecond) / 4;
}

} // namespace

std::uint32_t microframes_per_sof() { return frame_scale(); }

std::uint32_t sof_packet_delay_ns() {
    // 480 Mbit 下 64 个 bit time, 12 Mbit 下 35 个。
    return enumerated_speed() == kEnumSpeedHigh ? 133U : 2917U;
}

void note_sof(std::uint32_t frame, std::uint32_t now_cycles) {
    // 对齐到包起始。SOF-received 在包收完才置位, 时间戳比包起始晚包传输耗时
    // (全速 2917 ns, 高速 133 ns)。不减的话, "微帧 k 的本地时刻"逐板带一个
    // 速率相关常数, 混速舰队差 ~2.8 us; pulse(hpm_board)与 timebase(hpm_board)
    // 都按同一理由减过, 这里把本板对齐到包起始的约定。整数周期换算精确, 无
    // 量化残差; 中断进入延迟由拟合吸收为常数, 混速残差只剩两板进入延迟之差。
    now_cycles -= sof_packet_delay_ns() * kCyclesPerMicrosecond / 1000U;

    // 周期计数器回绕扩展, 先于一切使用时间戳的代码。
    if (!time_seeded) {
        time_seeded = true;
    } else if (now_cycles < previous_time) {
        time_high++;
    }
    previous_time = now_cycles;
    // 与 previous_time 同一包起始约定, 桥接锚点才内部自洽。TIM5 量化到 1 us,
    // 舍入残差是常数, 进不了残差统计。
    previous_timer_quarter_us = timer::timer->timepoint().time_since_epoch().count()
                              - (sof_packet_delay_ns() + 500U) / 1000U * 4U;

    // 每 SOF 中断微帧轴的期望步进, 以及防止硬件计数器自身回绕被误读为跳变的
    // 掩码。全速下每 1 ms 帧恰一个 SOF, 轴恰步进 8, 低三位恒为 0; 不把偏离此值
    // 当异常, 曾让一块全速 HPM 板永久停在 kInvalid。
    const std::uint32_t scale = frame_scale();
    const std::uint32_t index =
        (frame & ((static_cast<std::uint32_t>(kMicroframeModulus) / scale) - 1U)) * scale;

    if (!counter_seeded) {
        counter_seeded = true;
        previous_index = index;
        // 以帧号播种而非以 0 播种: 计数器低 14 位必须等于硬件读数, 锚点运算才能
        // 化简为 16384 的倍数。
        counter = index;
        state = data::TimeState::kWaitingAnchor;
        reset_fit_window();
        ring_oldest_microframe = counter;
        return;
    }

    const std::uint32_t delta = (index - previous_index) & kMicroframeMask;
    previous_index = index;
    counter += delta;

    if (delta != scale) [[unlikely]] {
        anomaly_count++;
        if (delta > scale && delta < scale * 8U) {
            // 漏了几次中断。计数器仍正确 -- 它按真实 delta 前进 -- 时间线无恙;
            // 仅拟合窗口作废: 缺口两侧的样本会使直线倾斜。
            reset_fit_window();
            ring_oldest_microframe = counter;
        } else {
            // delta 为 0(总线未运行, 枚举窗口即如此)或 8 帧及以上无法解释。
            // 无论哪种, 计数器都不再可信。
            invalidate();
        }
        return;
    }

    if (state == data::TimeState::kInvalid) {
        state = data::TimeState::kWaitingAnchor;
        reset_fit_window();
        ring_oldest_microframe = counter;
    }

    if (counter % kSampleDecimation != 0U)
        return;

    if (fit_valid) {
        // 两侧都先取相对拟合参考的差值, 再做 Q16 移位。对绝对周期计数移位会在
        // 开机约六天后溢出 int64: 550 MHz 计数器只留 48 位余量, 而 hpm_board 的
        // 4 MHz 定时器留 61 位。差值受拟合窗口约束, 此形式没有这种时限。
        const auto distance = static_cast<std::int64_t>(counter - fit_reference_microframe);
        const std::int64_t predicted_q16 =
            distance * static_cast<std::int64_t>(fit_cycles_per_microframe_q16);
        const std::int64_t actual_q16 =
            static_cast<std::int64_t>(now_extended(now_cycles) - fit_reference_time) << 16U;
        const std::int64_t residual_q16 = predicted_q16 - actual_q16;

        residual_sum_q16 += residual_q16;
        residual_count++;
        const auto magnitude =
            static_cast<std::uint64_t>(residual_q16 < 0 ? -residual_q16 : residual_q16);
        residual_abs_max_q16 = std::max(residual_abs_max_q16, magnitude);
    }

    if (ring_count < kSampleCount) {
        if (ring_count == 0U)
            ring_oldest_microframe = counter;
        sample[(ring_head + ring_count) % kSampleCount] = now_cycles;
        ring_count++;
        return;
    }
    sample[ring_head] = now_cycles;
    ring_head = (ring_head + 1U) % kSampleCount;
    ring_oldest_microframe += kSampleDecimation;
}

void poll(std::uint32_t tick_ms) {
    if (tick_ms - last_fit_tick < kFitPeriodMs)
        return;
    last_fit_tick = tick_ms;

    std::uint32_t local_sample[kSampleCount];
    std::uint32_t count = 0;
    std::uint64_t oldest_microframe = 0;
    std::uint64_t now = 0;

    {
        // 有界且短: 128 个字的拷贝, 550 MHz 下远不足 1 us, 每 20 ms 一次。用屏蔽
        // 中断而非无锁快照: 拟合要求 ring 与其原点相互一致, 本固件对此的既定
        // 手法就是中断锁。
        const utility::InterruptLockGuard guard;
        count = ring_count;
        oldest_microframe = ring_oldest_microframe;
        now = now_extended(previous_time);
        for (std::uint32_t index = 0; index < count; index++)
            local_sample[index] = sample[(ring_head + index) % kSampleCount];
    }

    if (count < kSampleCount)
        return;

    // 对等距 x = 0..N-1 做最小二乘; y 取相对首样本的差, 使加宽前算术留在 32 位
    // 内。Sxy 以 sum((2x - (N-1)) * y) 的倍增形式累加, 把半整均值挡在整数运算
    // 之外。
    //
    // 位宽说明 -- 周期计数器使这些量比 hpm_board 大一个量级: y 跨一个窗口,
    // 1.024 s * 550 MHz = 5.6e8, 在 int32 内; sum_xy2 峰值约 9e12, 其 Q16 移位
    // 约 6e17, 均在 int64 内且还余一个数量级。
    const std::uint32_t base = local_sample[0];
    std::int64_t sum_y = 0;
    std::int64_t sum_xy2 = 0;
    for (std::uint32_t index = 0; index < count; index++) {
        const auto y =
            static_cast<std::int64_t>(static_cast<std::int32_t>(local_sample[index] - base));
        sum_y += y;
        sum_xy2 += ((2LL * index) - static_cast<std::int64_t>(count - 1U)) * y;
    }

    // 每抽取样本的周期数, 再折算成每微帧, 均为 Q16。
    const std::int64_t slope_q16 = (sum_xy2 << 16U) / (2 * kSxx);
    const std::int64_t per_microframe_q16 = slope_q16 / kSampleDecimation;

    // 斜率偏离标称超过约 3%(1/32)不是晶振偏差而是坏窗口; 拒绝它, 坏拟合才不会
    // 发布出去。
    if (per_microframe_q16 < kNominalQ16 - (kNominalQ16 / 32)
        || per_microframe_q16 > kNominalQ16 + (kNominalQ16 / 32))
        return;

    // ring 里只有周期计数器的低 32 位。借当前时间重建最老样本的完整值: 它至多比
    // now 早一个窗口(1.024 s), 因而至多差一次回绕 -- 7.81 s 的回绕周期保证这一点。
    std::uint64_t base_extended = (now & ~static_cast<std::uint64_t>(0xFFFFFFFFU)) | base;
    if (base_extended > now)
        base_extended -= static_cast<std::uint64_t>(1) << 32U;

    // 拟合线在最新样本处而非质心处求值: 每次查询都从 now 向前外推, 把参考锚在
    // 前沿才能让外推臂最短。
    const auto newest_index = static_cast<std::int64_t>(count - 1U);
    const std::int64_t mean_y_q16 = (sum_y << 16U) / count;
    const std::int64_t offset_q16 = mean_y_q16 + ((slope_q16 * newest_index) / 2);

    const utility::InterruptLockGuard guard;
    fit_reference_microframe =
        oldest_microframe + (static_cast<std::uint64_t>(newest_index) * kSampleDecimation);
    fit_reference_time = base_extended + static_cast<std::uint64_t>((offset_q16 + 32768) >> 16U);
    fit_cycles_per_microframe_q16 = static_cast<std::uint64_t>(per_microframe_q16);
    fit_valid = true;
    if (state == data::TimeState::kWaitingAnchor && anchored)
        state = data::TimeState::kValid;
}

void apply_anchor(std::uint64_t host_microframe) {
    const utility::InterruptLockGuard guard;

    if (state == data::TimeState::kInvalid && !counter_seeded)
        return;

    // 消解回绕: 选使计数器最接近主机估计的 16384 倍数。四舍五入而非截断, 判定
    // 才对计数器落在估计值哪一侧不敏感。
    const auto difference = static_cast<std::int64_t>(host_microframe - counter);
    const std::int64_t wraps = floor_div(
        difference + static_cast<std::int64_t>(kMicroframeModulus / 2),
        static_cast<std::int64_t>(kMicroframeModulus));
    const std::int64_t offset = wraps * static_cast<std::int64_t>(kMicroframeModulus);

    if (!anchored) {
        anchor_offset = offset;
        anchored = true;
    } else if (offset != anchor_offset) {
        // 存活时间线上回绕改变不能静默接受: 要么计数器丢了超过一秒, 要么主机的
        // 估计丢了超过一秒, 两者都意味着自上次锚点以来的全部定时都定在了错误的
        // 秒上。
        anomaly_count++;
        invalidate();
        return;
    }

    if (fit_valid)
        state = data::TimeState::kValid;
    else if (state == data::TimeState::kInvalid)
        state = data::TimeState::kWaitingAnchor;
}

namespace {

Snapshot snapshot_locked() {
    const std::int64_t mean_cycles_q16 =
        residual_count == 0 ? 0 : residual_sum_q16 / static_cast<std::int64_t>(residual_count);

    return {
        .state = state,
        .microframe = anchored ? static_cast<std::uint64_t>(
                                     static_cast<std::int64_t>(counter) + anchor_offset)
                               : counter,
        // 最后一次 SOF 时的 TIM5, 而非周期计数器: 主机正需要这一对, 把 IMU 或
        // GPIO 记录自带的 quarter-us 戳换到微帧轴、再到它自己的时钟。
        .timestamp_quarter_us = previous_timer_quarter_us,
        // 输出途中换算成协议的 quarter-us tick, 使 68750 cycles/microframe 与
        // 其他板一样发布为 500 * 65536, 主机无需按板缩放。
        .ticks_per_microframe_q16 = static_cast<std::uint32_t>(
            cycles_to_quarter_us(static_cast<std::int64_t>(fit_cycles_per_microframe_q16))),
        // 线上字段 24 位, 钳位而非截断: 静默回绕会把持续的异常流伪装成小计数。
        .anomaly_count = anomaly_count > 0xFFFFFFU ? 0xFFFFFFU : anomaly_count,
        .residual_mean_q16 = static_cast<std::int32_t>(cycles_to_quarter_us(mean_cycles_q16)),
        .residual_abs_max_q16 = static_cast<std::uint32_t>(
            cycles_to_quarter_us(static_cast<std::int64_t>(residual_abs_max_q16))),
        // 钳位而非截断: 协议字段 16 位, 静默回绕会把主机一分钟未锚定的事实变成
        // 貌似合理的小计数。
        .residual_count = residual_count > 0xFFFFU ? 0xFFFFU : residual_count,
    };
}

} // namespace

Snapshot snapshot() {
    const utility::InterruptLockGuard guard;
    return snapshot_locked();
}

Snapshot report() {
    const utility::InterruptLockGuard guard;
    const Snapshot result = snapshot_locked();
    // 在此清零而非任其自由运行: 均值只在有界窗口内才有意义, 跨越重锚定的累加会
    // 把两个不同拟合混在一起。
    residual_sum_q16 = 0;
    residual_count = 0;
    residual_abs_max_q16 = 0;
    return result;
}

// 两个方向都以 TIM5 的 quarter-us 为单位 -- 本板其余上行记录打戳所用的单位 --
// 而拟合本身留在周期计数器上。两者之桥是 SOF 中断里捕获的一对: (previous_time
// cycles, previous_timer_quarter_us)。两个计数器同挂 550 MHz PLL, TIM5 恰为
// SYSCLK/550, 跨换的唯一误差是 TIM5 自身的 1 us 量化 -- 这也正是被换算时间戳的
// 分辨率, 故毫无损失。
namespace {

// 由 TIM5 读数求对应的周期计数器值, 以最后一次 SOF 为锚。减法刻意用 int32, 使
// 其在 TIM5 回绕下依然正确; 遥测记录总在采集后几秒内换算。
std::int64_t cycles_at_quarter_us(std::uint32_t quarter_us) {
    const auto delta_quarter_us = static_cast<std::int32_t>(quarter_us - previous_timer_quarter_us);
    return static_cast<std::int64_t>(now_extended(previous_time))
         + quarter_us_to_cycles(delta_quarter_us);
}

std::uint32_t quarter_us_at_cycles(std::int64_t cycles) {
    const std::int64_t delta = cycles - static_cast<std::int64_t>(now_extended(previous_time));
    // 随计数器一起回绕, 这正是调用方想要的: TIM5 quarter-us 是自由运行的 32 位
    // 值, 所有消费者都取差值。
    return previous_timer_quarter_us + static_cast<std::uint32_t>(cycles_to_quarter_us(delta));
}

} // namespace

bool local_time_of(std::uint64_t microframe, std::uint32_t& out_quarter_us) {
    const utility::InterruptLockGuard guard;
    if (state != data::TimeState::kValid || !fit_valid)
        return false;

    const auto local_microframe =
        static_cast<std::uint64_t>(static_cast<std::int64_t>(microframe) - anchor_offset);
    const auto distance = static_cast<std::int64_t>(local_microframe - fit_reference_microframe);
    const std::int64_t cycles =
        ((distance * static_cast<std::int64_t>(fit_cycles_per_microframe_q16)) + 32768) >> 16U;
    out_quarter_us = quarter_us_at_cycles(static_cast<std::int64_t>(fit_reference_time) + cycles);
    return true;
}

bool microframe_at(std::uint32_t quarter_us, std::uint64_t& out_microframe) {
    const utility::InterruptLockGuard guard;
    if (state != data::TimeState::kValid || !fit_valid)
        return false;

    const std::int64_t distance_cycles =
        cycles_at_quarter_us(quarter_us) - static_cast<std::int64_t>(fit_reference_time);
    const std::int64_t microframes =
        (distance_cycles << 16U) / static_cast<std::int64_t>(fit_cycles_per_microframe_q16);
    out_microframe = static_cast<std::uint64_t>(
        static_cast<std::int64_t>(fit_reference_microframe) + microframes + anchor_offset);
    return true;
}

} // namespace libhcs::firmware::sync::timebase

#endif
