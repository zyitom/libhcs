#include "firmware/hpm_board/app/src/sync/timebase.hpp"

#if defined(libhcs_APP_TIME_SYNC) && libhcs_APP_TIME_SYNC

# include <cstddef>

# include "firmware/hpm_board/app/src/sync/pulse.hpp"
# include "firmware/hpm_board/app/src/sync/timebase.hpp"
# include "firmware/hpm_board/app/src/timer/timer.hpp"
# include "firmware/hpm_board/app/src/utility/interrupt_lock.hpp"

namespace libhcs::firmware::sync::timebase {
namespace {

constexpr std::uint32_t kFrindexMask = 0x3FFFU;
// PORTSC1.PSPD 编码(ChipIdea/EHCI): 0 全速, 1 低速, 2 高速。
constexpr std::uint32_t kPortSpeedHigh = 2U;
constexpr std::uint64_t kFrindexModulus = 0x4000U;
constexpr std::uint32_t kFitPeriodMs = 20U;

// x = 0..N-1 各点对均值的离差平方和。等间距采样下是常数, 拟合无需现算:
// 恒为 N(N^2-1)/12。
constexpr std::int64_t kSxx = static_cast<std::int64_t>(kSampleCount)
                            * (static_cast<std::int64_t>(kSampleCount) * kSampleCount - 1) / 12;

// ISR 独占状态。仅 note_sof() 写入; 主循环在 utility::InterruptLockGuard 下
// 读取。用普通标量而非原子量: 每个读者都会取该锁, 且这些量必须作为一致的一
// 组整体读取 -- 逐变量原子给不出这个保证。

// 本板开机起算的 microframe 计数。用首次 FRINDEX 读数播种, 使此后恒有
// (counter mod 16384) == FRINDEX, anchor 因此只需是 16384 的整数倍。
std::uint64_t counter = 0;
bool counter_seeded = false;
std::uint32_t previous_frindex = 0;

// ISR 所读 32 位机器定时器读数的 64 位扩展。4 MHz 下约 1073 s 回绕一次, 而
// SOF 每 125 us 一次, 故"低字变小"在这里是明确的回绕判据。
std::uint32_t previous_time = 0;
std::uint32_t time_high = 0;
bool time_seeded = false;

data::TimeState state = data::TimeState::kInvalid;
std::uint32_t anomaly_count = 0;
std::int64_t anchor_offset = 0;
bool anchored = false;

// 拟合环形缓冲。sample[i] 是 microframe (ring_oldest_microframe + i *
// kSampleDecimation) 处机器定时器的低 32 位, 自 ring_head 起按写入顺序存放。
std::uint32_t sample[kSampleCount];
std::uint32_t ring_head = 0;
std::uint32_t ring_count = 0;
std::uint64_t ring_oldest_microframe = 0;

// 拟合结果。仅由 poll() 写入, 其余各方读取; 同样靠中断锁保护。
// 参考点取整 tick(0.25 us)而非 Q16: 128 个样本求平均正是为了让相位优于单个
// 样本的抖动, 0.25 us 的舍入远低于本机制瞄准的 ~1 us。斜率保持 Q16 -- 这个
// 量级的百万分之一误差会沿外推距离累积。
bool fit_valid = false;
std::uint64_t fit_reference_microframe = 0;
std::uint64_t fit_reference_time = 0;
std::uint32_t fit_ticks_per_microframe_q16 = 0;

// 样本外预测误差, 两次上报之间累计。
//
// 每个抽取样本先用当前已发布的拟合预测(该拟合算于本样本存在之前), 再加入
// 窗口, 因此残差是真正的预测误差 -- 与定时动作实际承受的一致 -- 而非穿过
// 被测点本身的拟合线的样本内残差。
std::int64_t residual_sum_q16 = 0;
std::uint32_t residual_count = 0;
std::uint32_t residual_abs_max_q16 = 0;

std::uint32_t last_fit_tick = 0;

void reset_fit_window() {
    ring_head = 0;
    ring_count = 0;
    fit_valid = false;
    fit_ticks_per_microframe_q16 = 0;
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

// 向负无穷取整的整除。C++ 默认向零截断, 会使下方的回绕消解在零点两侧不
// 对称, 跨原点的两块板可能选中不同的回绕。
std::int64_t floor_div(std::int64_t numerator, std::int64_t denominator) {
    const std::int64_t quotient = numerator / denominator;
    const std::int64_t remainder = numerator % denominator;
    return (remainder != 0 && ((remainder < 0) != (denominator < 0))) ? quotient - 1 : quotient;
}

} // namespace

std::uint32_t microframes_per_sof() {
    const std::uint32_t speed =
        (HPM_USB0->PORTSC1 & USB_PORTSC1_PSPD_MASK) >> USB_PORTSC1_PSPD_SHIFT;
    return speed == kPortSpeedHigh ? 1U : 8U;
}

std::uint32_t sof_packet_delay_ns() {
    const std::uint32_t speed =
        (HPM_USB0->PORTSC1 & USB_PORTSC1_PSPD_MASK) >> USB_PORTSC1_PSPD_SHIFT;
    // 480 Mbit 下 64 个位时间, 12 Mbit 下 35 个位时间。
    return speed == kPortSpeedHigh ? 133U : 2917U;
}

void note_sof(std::uint32_t frindex, std::uint32_t now_quarter_us) {
    // 对齐到包起始。SOF-received 在包收完才置位, 时间戳比包起始晚包传输耗时
    // (高速 133 ns, 全速 2917 ns)。不减的话, "微帧 k 的本地时刻"逐板带一个
    // 速率相关常数, 混速舰队差 ~2.8 us; pulse 模块对自己的捕获按同一理由减
    // 过, 这里把时间基对齐到同一约定。舍入到 0.25 us tick 的残差与中断进入
    // 延迟一样, 由拟合吸收为各板常数, 混速残差只剩两板进入延迟之差。
    now_quarter_us -= static_cast<std::uint32_t>((sof_packet_delay_ns() + 125U) / 250U);

    // 机器定时器回绕扩展, 先于一切使用时间戳的代码。
    if (!time_seeded) {
        time_seeded = true;
    } else if (now_quarter_us < previous_time) {
        time_high++;
    }
    previous_time = now_quarter_us;

    if (!counter_seeded) {
        counter_seeded = true;
        previous_frindex = frindex;
        // 用帧索引而非零播种: 计数器低 14 位必须恒等于 FRINDEX, anchor 运算
        // 才能归结为 16384 的整数倍。
        counter = frindex;
        state = data::TimeState::kWaitingAnchor;
        reset_fit_window();
        ring_oldest_microframe = counter;
        return;
    }

    const std::uint32_t delta = (frindex - previous_frindex) & kFrindexMask;
    previous_frindex = frindex;
    counter += delta;

    // 每个 SOF 中断预期的 FRINDEX 步进。FRINDEX 数的永远是 MICROFRAME, 但
    // 全速端口每 1 ms 帧只收到一个 SOF, 故步进恒为 8, 低三位恒为零。把它当
    // 异常处理曾让全速板永久停在 kInvalid。
    // [实测 2026-08-20, 同一 xHCI 上的混速对: HS 步进 1 占 100.00000%
    //  (159187 次), FS 步进 8 占 100.00000% (19898 次), ISR 间隔 125.0099 us
    //  对 1000.0784 us -- 恰为 8 倍, 同一时钟。]
    const std::uint32_t step = microframes_per_sof();

    if (delta != step) [[unlikely]] {
        anomaly_count++;
        if (delta > step && delta < step * 8U) {
            // 少量丢失的中断。计数器仍正确 -- 它按真实 delta 前进 -- 时间线
            // 得以保留; 只有拟合窗口作废, 因为缺口两侧的样本会拉歪拟合线。
            reset_fit_window();
            ring_oldest_microframe = counter;
        } else {
            // delta 为 0(总线未运行: 枚举窗口就是这副样子), 或一整帧以上
            // 下落不明。两种情况下计数器都不再可信。
            invalidate();
        }
        return;
    }

    if (state == data::TimeState::kInvalid) {
        state = data::TimeState::kWaitingAnchor;
        reset_fit_window();
        ring_oldest_microframe = counter;
    }

    pulse::note_sof(
        anchored ? static_cast<std::uint64_t>(static_cast<std::int64_t>(counter) + anchor_offset)
                 : counter);

    if (counter % kSampleDecimation != 0U)
        return;

    if (fit_valid) {
        const auto distance = static_cast<std::int64_t>(counter - fit_reference_microframe);
        const std::int64_t predicted_q16 =
            (static_cast<std::int64_t>(fit_reference_time) << 16U)
            + distance * static_cast<std::int64_t>(fit_ticks_per_microframe_q16);
        const std::int64_t actual_q16 = static_cast<std::int64_t>(now_extended(now_quarter_us))
                                     << 16U;
        const std::int64_t residual_q16 = predicted_q16 - actual_q16;

        residual_sum_q16 += residual_q16;
        residual_count++;
        const auto magnitude =
            static_cast<std::uint64_t>(residual_q16 < 0 ? -residual_q16 : residual_q16);
        if (magnitude > residual_abs_max_q16 && magnitude <= 0xFFFFFFFFU)
            residual_abs_max_q16 = static_cast<std::uint32_t>(magnitude);
    }

    if (ring_count < kSampleCount) {
        if (ring_count == 0U)
            ring_oldest_microframe = counter;
        sample[(ring_head + ring_count) % kSampleCount] = now_quarter_us;
        ring_count++;
        return;
    }
    sample[ring_head] = now_quarter_us;
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
        // 有界且短暂: 128 个字拷贝约一微秒, 每 20 ms tick 两次。用屏蔽而非
        // 无锁快照, 因为拟合要求环形缓冲与其起点相互一致, 而本固件对此的
        // 既定手段就是中断锁。
        const utility::InterruptLockGuard guard;
        count = ring_count;
        oldest_microframe = ring_oldest_microframe;
        now = now_extended(previous_time);
        for (std::uint32_t index = 0; index < count; index++)
            local_sample[index] = sample[(ring_head + index) % kSampleCount];
    }

    if (count < kSampleCount)
        return;

    // 对等间距 x = 0..N-1 做最小二乘, y 相对首个样本取值, 使拓宽前的运算
    // 不超出 32 位。Sxy 以变形 sum((2x - (N-1)) * y) 累加, 把半整数均值挡
    // 在整数运算外。
    const std::uint32_t base = local_sample[0];
    std::int64_t sum_y = 0;
    std::int64_t sum_xy2 = 0;
    for (std::uint32_t index = 0; index < count; index++) {
        const auto y =
            static_cast<std::int64_t>(static_cast<std::int32_t>(local_sample[index] - base));
        sum_y += y;
        sum_xy2 += (2LL * index - static_cast<std::int64_t>(count - 1U)) * y;
    }

    // 先求每个抽取样本的 tick 数, 再折算到每 microframe, 均为 Q16。
    const std::int64_t slope_q16 = (sum_xy2 << 16U) / (2 * kSxx);
    const std::int64_t per_microframe_q16 = slope_q16 / kSampleDecimation;

    // 斜率偏离标称值几个百分点就不是晶振偏差, 而是窗口损坏; 拒收它, 坏拟合
    // 就根本不会被发布。
    constexpr std::int64_t kNominalQ16 = static_cast<std::int64_t>(kNominalTicksPerMicroframe)
                                      << 16U;
    if (per_microframe_q16 < kNominalQ16 - kNominalQ16 / 32
        || per_microframe_q16 > kNominalQ16 + kNominalQ16 / 32)
        return;

    // 环形缓冲只存定时器低 32 位。把最老样本的全值挂接到当前时间上重建:
    // 当前时间至多晚一个窗口(1.024 s), 故至多差一次回绕。
    std::uint64_t base_extended = (now & ~static_cast<std::uint64_t>(0xFFFFFFFFU)) | base;
    if (base_extended > now)
        base_extended -= static_cast<std::uint64_t>(1) << 32U;

    // 在最新样本处而非质心处取拟合直线: 所有查询都从当前时刻向前外推,
    // 参考点锚在窗口前缘才能让外推臂最短。
    const std::int64_t mean_y_q16 = (sum_y << 16U) / count;
    const std::int64_t offset_q16 =
        mean_y_q16 + (slope_q16 * static_cast<std::int64_t>(count - 1U)) / 2;

    const utility::InterruptLockGuard guard;
    fit_reference_microframe =
        oldest_microframe + static_cast<std::uint64_t>(count - 1U) * kSampleDecimation;
    fit_reference_time = base_extended + static_cast<std::uint64_t>((offset_q16 + 32768) >> 16U);
    fit_ticks_per_microframe_q16 = static_cast<std::uint32_t>(per_microframe_q16);
    fit_valid = true;
    if (state == data::TimeState::kWaitingAnchor && anchored)
        state = data::TimeState::kValid;
}

void apply_anchor(std::uint64_t host_microframe) {
    const utility::InterruptLockGuard guard;

    if (state == data::TimeState::kInvalid && !counter_seeded)
        return;

    // 消解回绕: 选使计数器最接近主机估计值的 16384 整数倍。四舍五入而非
    // 截断, 决策才不依赖计数器落在估计值的哪一侧。
    const auto difference = static_cast<std::int64_t>(host_microframe - counter);
    const std::int64_t wraps = floor_div(
        difference + static_cast<std::int64_t>(kFrindexModulus / 2),
        static_cast<std::int64_t>(kFrindexModulus));
    const std::int64_t offset = wraps * static_cast<std::int64_t>(kFrindexModulus);

    if (!anchored) {
        anchor_offset = offset;
        anchored = true;
    } else if (offset != anchor_offset) {
        // 方案的硬性规则, 且确实要紧: 已生效时间线上回绕数变化不可静默接受。
        // 要么计数器丢了一秒以上, 要么主机估计值丢了, 两者都意味着上次
        // anchor 之后所有已排定的动作都排在了错误的一秒上。
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
    return {
        .state = state,
        .microframe = anchored ? static_cast<std::uint64_t>(
                                     static_cast<std::int64_t>(counter) + anchor_offset)
                               : counter,
        .timestamp_quarter_us = now_extended(previous_time),
        .ticks_per_microframe_q16 = fit_ticks_per_microframe_q16,
        // 线上字段 24 位, 钳位而非截断: 静默回绕会把持续的异常流伪装成
        // 小计数。
        .anomaly_count = anomaly_count > 0xFFFFFFU ? 0xFFFFFFU : anomaly_count,
        .residual_mean_q16 = residual_count == 0
                               ? 0
                               : static_cast<std::int32_t>(
                                     residual_sum_q16 / static_cast<std::int64_t>(residual_count)),
        .residual_abs_max_q16 = residual_abs_max_q16,
        .residual_count =
            static_cast<std::uint16_t>(residual_count > 0xFFFFU ? 0xFFFFU : residual_count),
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
    // 就地清零而非任其自由累计: 均值只在有界窗口内有意义, 跨重锚的求和会
    // 把两次不同的拟合混在一起。
    residual_sum_q16 = 0;
    residual_count = 0;
    residual_abs_max_q16 = 0;
    return result;
}

bool local_time_of(std::uint64_t microframe, std::uint64_t& out_quarter_us) {
    const utility::InterruptLockGuard guard;
    if (state != data::TimeState::kValid || !fit_valid)
        return false;

    const auto local_microframe =
        static_cast<std::uint64_t>(static_cast<std::int64_t>(microframe) - anchor_offset);
    const auto distance = static_cast<std::int64_t>(local_microframe - fit_reference_microframe);
    const std::int64_t ticks =
        (distance * static_cast<std::int64_t>(fit_ticks_per_microframe_q16) + 32768) >> 16U;
    const std::int64_t result = static_cast<std::int64_t>(fit_reference_time) + ticks;
    if (result < 0)
        return false;
    out_quarter_us = static_cast<std::uint64_t>(result);
    return true;
}

bool microframe_at(std::uint64_t quarter_us, std::uint64_t& out_microframe) {
    const utility::InterruptLockGuard guard;
    if (state != data::TimeState::kValid || !fit_valid)
        return false;

    const auto distance_ticks =
        static_cast<std::int64_t>(quarter_us) - static_cast<std::int64_t>(fit_reference_time);
    const std::int64_t microframes =
        (distance_ticks << 16U) / static_cast<std::int64_t>(fit_ticks_per_microframe_q16);
    out_microframe = static_cast<std::uint64_t>(
        static_cast<std::int64_t>(fit_reference_microframe) + microframes + anchor_offset);
    return true;
}

bool microframe_now(std::uint64_t& out_microframe, std::uint32_t& out_quarter_us) {
    const utility::InterruptLockGuard guard;
    if (state != data::TimeState::kValid || !fit_valid)
        return false;

    // 主循环读数相对最后一次 ISR 读数扩展到 64 位。定时器单调递增, 低字
    // 变小只可能是两次读之间回绕了: 此时 time_high 尚未被下一次 SOF 中断
    // 推进, 本地补上。
    const std::uint32_t low = timer::Timer::timestamp_quarter_us();
    std::uint64_t now = (static_cast<std::uint64_t>(time_high) << 32U) | low;
    if (low < previous_time)
        now += static_cast<std::uint64_t>(1) << 32U;

    const auto distance =
        static_cast<std::int64_t>(now) - static_cast<std::int64_t>(fit_reference_time);
    const std::int64_t microframes =
        (distance << 16U) / static_cast<std::int64_t>(fit_ticks_per_microframe_q16);
    out_microframe = static_cast<std::uint64_t>(
        static_cast<std::int64_t>(fit_reference_microframe) + microframes + anchor_offset);
    out_quarter_us = low;
    return true;
}

} // namespace libhcs::firmware::sync::timebase

#endif
