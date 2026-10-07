#include "firmware/c_board/app/src/sync/timebase.hpp"

#include <cstdint>

#include <main.h>

#include "core/src/time/sof_timebase.hpp"
#include "core/src/time/usb_sof_bits.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/common/app/src/utility/interrupt_lock.hpp"

namespace libhcs::firmware::sync::timebase {
namespace {

// DSTS.ENUMSPD 的 DWC2 编码: 0 高速, 1 全速(30/60 MHz PHY), 2 低速, 3 全速
// (内置 48 MHz PHY)。本板恒为 3 -- OTG_FS 没有高速 PHY, 见 timebase.hpp 第 1 条;
// 保留高速分支, 以免此代码被复用到带高速 PHY 的芯片上时悄悄错一个 8 倍。
constexpr std::uint32_t kEnumSpeedHigh = 0U;

// cycles 换算成 quarter-us。整微帧时精确(21000 -> 500), 其余截断; 仅在协议要求
// 该单位处使用。
constexpr std::int64_t cycles_to_quarter_us(std::int64_t cycles) {
    return cycles * 4 / static_cast<std::int64_t>(kCyclesPerMicrosecond);
}

constexpr std::int64_t quarter_us_to_cycles(std::int64_t quarter_us) {
    return quarter_us * static_cast<std::int64_t>(kCyclesPerMicrosecond) / 4;
}

// 拟合在 DWT->CYCCNT 上(见 timebase.hpp 第 2 条), 输出途中换算成协议的 quarter-us,
// 使 21000 cycles/microframe 与其他板一样发布为 500 * 65536, 主机无需按板缩放。
struct Policy {
    static constexpr std::uint32_t kNominalTicksPerMicroframe = kNominalCyclesPerMicroframe;
    // 残差极值以 64 位持有, 不设上限(与 mc02 同一份)。
    static constexpr std::uint64_t kResidualAbsMaxLimit = ~std::uint64_t{0};
    static constexpr std::int64_t to_quarter_us(std::int64_t cycles) {
        return cycles_to_quarter_us(cycles);
    }
};

// 运算在 core(core/src/time/sof_timebase.hpp)。ISR(note_sof)写, 其余在中断锁下读写。
core::time::SofTimebase<Policy> sof_timebase;

// TIM2, 在同一中断内采样, 纯粹为了让状态报告能把微帧与本固件其余部分打时间戳
// 所用的时钟配成一对。
//
// 拟合跑在 CYCCNT 上, 因为 TIM2 量化到 250 ns(timer.hpp: 4 MHz, 按原值上报),
// 与被测抖动同量级。但其余所有上行记录 -- 带时间戳的 GPIO -- 带的都是 TIM2 的
// quarter-us 戳, 且 CYCCNT 与 TIM2 每次上电的起点互不相关。若状态发布 CYCCNT
// 派生的时间, 主机将持有两个永远无法互相关联的时钟, 任何遥测记录都无法落到
// 微帧轴上。发布 TIM2 才让该轴对时间基准之外的用途可用。
//
// 在周期计数器之后采样, 使这次额外的外设读(约 0.25 us)不会拉长时间戳路径。1 kHz。
// ISR 写, 其余在中断锁下读。
std::uint32_t previous_timer_quarter_us = 0;

USB_OTG_DeviceTypeDef* device_registers() {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<USB_OTG_DeviceTypeDef*>(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
}

std::uint32_t enumerated_speed() {
    const auto status = static_cast<std::uint32_t>(device_registers()->DSTS);
    return (status & static_cast<std::uint32_t>(USB_OTG_DSTS_ENUMSPD))
        >> static_cast<std::uint32_t>(USB_OTG_DSTS_ENUMSPD_Pos);
}

// 当前端口速度下 DSTS.FNSOF 一个计数折合的微帧数。
//
// 全速差异全在此函数。DWC2 手册对 FNSOF 的描述是收到 SOF 的帧或微帧号: 高速
// 枚举时是微帧号, 否则是帧号。本板恒为后者, 每个计数折合 8 微帧, 寄存器只承载
// 主机送上总线的 11 位帧号。
//
// 全速下掩到 11 位而非寄存器满 14 位, 在 mc02 上是实测宽度(FNSOF 是 14 位字段,
// 但全速下只有主机送上总线的那 11 位被填充: 30 s 探测约 14 个整回绕, 最大值恰为
// 2047 [实测 2026-09-07, mc02])。本板与其同为 DWC2、同为全速, 推断同宽度;
// 即便本板的 bits 13..11 真有读数, 掩到 11 位也只是让模数与全仓库一致 --
// 16384 微帧 = 2048 帧 = 2.048 s, 本板无法把它做得更长。[未上板。]
//
// 故主机锚点需消解的回绕在 mc02 与本板上同为 2048 帧 = 16384 微帧 = 2.048 s。
std::uint32_t frame_scale() { return enumerated_speed() == kEnumSpeedHigh ? 1U : 8U; }

} // namespace

std::uint32_t microframes_per_sof() { return frame_scale(); }

std::uint32_t sof_packet_delay_ns() {
    // 480 Mbit 下 64 个 bit time, 12 Mbit 下 35 个。
    return enumerated_speed() == kEnumSpeedHigh ? 133U : 2917U;
}

void reset() {
    const utility::InterruptLockGuard guard;
    sof_timebase.reset();
    previous_timer_quarter_us = 0;
}

void note_sof(std::uint32_t frame, std::uint32_t now_cycles) {
    // 对齐到包起始。SOF-received 在包收完才置位, 时间戳比包起始晚包传输耗时
    // (本板恒全速: 2917 ns)。不减的话, "微帧 k 的本地时刻"逐板带一个速率相关
    // 常数, 混速舰队差 ~2.8 us; pulse(hpm_board)与 timebase(mc02)都按同一理由
    // 减过, 这里把本板对齐到包起始的约定。整数周期换算精确, 无量化残差; 中断
    // 进入延迟由拟合吸收为常数, 混速残差只剩两板进入延迟之差。
    now_cycles -= sof_packet_delay_ns() * kCyclesPerMicrosecond / 1000U;
    // 全速下 SOF 包里每多一个填充位再晚一位(83.3 ns, 约九分之一的帧号有, 见
    // core/src/time/usb_sof_bits.hpp)。mc02 扣掉之后对其帧时间戳之差 sigma
    // 50 -> 24 ns、最差 343 -> 69 ns [实测 2026-10-05]。
    if (frame_scale() == 8U)
        now_cycles -=
            (core::time::full_speed_sof_stuffed_bits(frame) * kCyclesPerMicrosecond + 6U) / 12U;

    sof_timebase.note_time(now_cycles);
    // 与周期计数器同一包起始约定, 桥接锚点才内部自洽。TIM2 量化到 250 ns,
    // 舍入残差是常数, 进不了残差统计。
    previous_timer_quarter_us = timer::timer->timepoint().time_since_epoch().count()
                              - (sof_packet_delay_ns() + 500U) / 1000U * 4U;

    // 帧号放到 14 位微帧轴上, 并给出每 SOF 中断的期望步进。全速下每 1 ms 帧恰一个
    // SOF, 轴恰步进 8, 低三位恒为 0; 不把偏离此值当异常, 曾让一块全速 HPM 板永久
    // 停在 kInvalid。
    const std::uint32_t scale = frame_scale();
    const std::uint32_t index =
        (frame & ((static_cast<std::uint32_t>(core::time::kMicroframeModulus) / scale) - 1U))
        * scale;
    if (!sof_timebase.note_microframe(index, scale))
        return;
    sof_timebase.note_sample(now_cycles);
}

void poll(std::uint32_t tick_ms) {
    if (!sof_timebase.fit_due(tick_ms))
        return;

    decltype(sof_timebase)::Window window;
    {
        // 有界且短: 128 个字的拷贝, 168 MHz 下约 8 us, 每 20 ms 一次。用屏蔽
        // 中断而非无锁快照: 拟合要求 ring 与其原点相互一致, 本固件对此的既定
        // 手法就是中断锁。
        const utility::InterruptLockGuard guard;
        sof_timebase.copy_window(window);
    }
    const auto fit = decltype(sof_timebase)::compute_fit(window);
    if (!fit)
        return;

    const utility::InterruptLockGuard guard;
    sof_timebase.publish_fit(*fit);
}

void apply_anchor(std::uint64_t host_microframe) {
    const utility::InterruptLockGuard guard;
    sof_timebase.apply_anchor(host_microframe);
}

namespace {

Snapshot snapshot_locked() {
    Snapshot result = sof_timebase.snapshot();
    // 最后一次 SOF 时的 TIM2, 而非周期计数器: 主机正需要这一对, 把带时间戳的
    // GPIO 记录自带的 quarter-us 戳换到微帧轴、再到它自己的时钟。
    result.timestamp_quarter_us = previous_timer_quarter_us;
    return result;
}

} // namespace

Snapshot snapshot() {
    const utility::InterruptLockGuard guard;
    return snapshot_locked();
}

Snapshot report() {
    const utility::InterruptLockGuard guard;
    const Snapshot result = snapshot_locked();
    sof_timebase.clear_residuals();
    return result;
}

// 两个方向都以 TIM2 的 quarter-us 为单位 -- 本板其余上行记录打戳所用的单位 --
// 而拟合本身留在周期计数器上。两者之桥是 SOF 中断里捕获的一对: (最后一次 SOF 的
// cycles, previous_timer_quarter_us)。两个计数器同挂 168 MHz SYSCLK -- TIM2 的
// 定时器时钟是 APB1 的 84 MHz(恰为一半), 再经 PSC=21 分到 4 MHz -- 跨换的唯一
// 误差是 TIM2 自身的 250 ns 量化, 而这正是被换算时间戳的分辨率, 故毫无损失。
namespace {

// 由 TIM2 读数求对应的周期计数器值, 以最后一次 SOF 为锚。减法刻意用 int32, 使
// 其在 TIM2 回绕下依然正确; 遥测记录总在采集后几秒内换算。
std::int64_t cycles_at_quarter_us(std::uint32_t quarter_us) {
    const auto delta_quarter_us = static_cast<std::int32_t>(quarter_us - previous_timer_quarter_us);
    return static_cast<std::int64_t>(sof_timebase.latest_time())
         + quarter_us_to_cycles(delta_quarter_us);
}

std::uint32_t quarter_us_at_cycles(std::int64_t cycles) {
    const std::int64_t delta = cycles - static_cast<std::int64_t>(sof_timebase.latest_time());
    // 随计数器一起回绕, 这正是调用方想要的: TIM2 quarter-us 是自由运行的 32 位
    // 值, 所有消费者都取差值。
    return previous_timer_quarter_us + static_cast<std::uint32_t>(cycles_to_quarter_us(delta));
}

} // namespace

bool local_time_of(std::uint64_t microframe, std::uint32_t& out_quarter_us) {
    const utility::InterruptLockGuard guard;
    if (!sof_timebase.answers())
        return false;
    out_quarter_us = quarter_us_at_cycles(sof_timebase.time_of(microframe));
    return true;
}

bool microframe_at(std::uint32_t quarter_us, std::uint64_t& out_microframe) {
    const utility::InterruptLockGuard guard;
    if (!sof_timebase.answers())
        return false;
    out_microframe = sof_timebase.microframe_at(cycles_at_quarter_us(quarter_us));
    return true;
}

bool microframe_at(
    std::uint32_t quarter_us, std::uint64_t& out_microframe, std::uint16_t& out_fraction_q16) {
    const utility::InterruptLockGuard guard;
    if (!sof_timebase.answers())
        return false;
    // 离拟合参考超过 2^30 个周期(168 MHz 下约 6.4 s)说明拟合早已不再更新, 不作答。
    return sof_timebase.microframe_q16_at(
        cycles_at_quarter_us(quarter_us), out_microframe, out_fraction_q16);
}

} // namespace libhcs::firmware::sync::timebase
