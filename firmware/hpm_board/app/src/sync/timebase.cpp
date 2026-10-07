#include "firmware/hpm_board/app/src/sync/timebase.hpp"

#include <cstdint>

#include "core/src/time/sof_timebase.hpp"
#include "firmware/common/app/src/utility/interrupt_lock.hpp"
#include "firmware/hpm_board/app/src/sync/pulse.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"

namespace libhcs::firmware::sync::timebase {
namespace {

// PORTSC1.PSPD 编码(ChipIdea/EHCI): 0 全速, 1 低速, 2 高速。
constexpr std::uint32_t kPortSpeedHigh = 2U;

// 拟合时钟就是协议单位: 4 MHz 机器定时器的 quarter-us, 输出不必换算。
struct Policy {
    static constexpr std::uint32_t kNominalTicksPerMicroframe =
        timebase::kNominalTicksPerMicroframe;
    // 残差极值存 32 位: 超出的(> 4096 s 的 Q16 tick)不计入。
    static constexpr std::uint64_t kResidualAbsMaxLimit = 0xFFFFFFFFU;
    static constexpr std::int64_t to_quarter_us(std::int64_t ticks) { return ticks; }
};

// 运算在 core(core/src/time/sof_timebase.hpp)。ISR(note_sof)写, 其余在中断锁下读写。
core::time::SofTimebase<Policy> sof_timebase;

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

void reset() {
    const utility::InterruptLockGuard guard;
    sof_timebase.reset();
}

void note_sof(std::uint32_t frindex, std::uint32_t now_quarter_us) {
    // 对齐到包起始。SOF-received 在包收完才置位, 时间戳比包起始晚包传输耗时
    // (高速 133 ns, 全速 2917 ns)。不减的话, "微帧 k 的本地时刻"逐板带一个
    // 速率相关常数, 混速舰队差 ~2.8 us; pulse 模块对自己的捕获按同一理由减
    // 过, 这里把时间基对齐到同一约定。舍入到 0.25 us tick 的残差与中断进入
    // 延迟一样, 由拟合吸收为各板常数, 混速残差只剩两板进入延迟之差。
    now_quarter_us -= static_cast<std::uint32_t>((sof_packet_delay_ns() + 125U) / 250U);
    sof_timebase.note_time(now_quarter_us);

    // FRINDEX 本身就是 14 位微帧轴。每个 SOF 中断预期的步进: FRINDEX 数的永远是
    // MICROFRAME, 但全速端口每 1 ms 帧只收到一个 SOF, 故步进恒为 8, 低三位恒为零。
    // 把它当异常处理曾让全速板永久停在 kInvalid。
    // [实测 2026-08-20, 同一 xHCI 上的混速对: HS 步进 1 占 100.00000%
    //  (159187 次), FS 步进 8 占 100.00000% (19898 次), ISR 间隔 125.0099 us
    //  对 1000.0784 us -- 恰为 8 倍, 同一时钟。]
    if (!sof_timebase.note_microframe(frindex, microframes_per_sof()))
        return;

    pulse::note_sof(sof_timebase.current_microframe());
    sof_timebase.note_sample(now_quarter_us);
}

void poll(std::uint32_t tick_ms) {
    if (!sof_timebase.fit_due(tick_ms))
        return;

    decltype(sof_timebase)::Window window;
    {
        // 有界且短暂: 128 个字拷贝约一微秒, 每 20 ms 一次。用屏蔽而非无锁快照,
        // 因为拟合要求环形缓冲与其起点相互一致, 而本固件对此的既定手段就是中断锁。
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
    result.timestamp_quarter_us = sof_timebase.latest_time();
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

bool local_time_of(std::uint64_t microframe, std::uint64_t& out_quarter_us) {
    const utility::InterruptLockGuard guard;
    if (!sof_timebase.answers())
        return false;

    const std::int64_t result = sof_timebase.time_of(microframe);
    if (result < 0)
        return false;
    out_quarter_us = static_cast<std::uint64_t>(result);
    return true;
}

bool microframe_at(std::uint64_t quarter_us, std::uint64_t& out_microframe) {
    const utility::InterruptLockGuard guard;
    if (!sof_timebase.answers())
        return false;

    out_microframe = sof_timebase.microframe_at(static_cast<std::int64_t>(quarter_us));
    return true;
}

bool microframe_now(
    std::uint64_t& out_microframe, std::uint16_t& out_fraction_q16, std::uint32_t& out_quarter_us) {
    const utility::InterruptLockGuard guard;
    if (!sof_timebase.answers())
        return false;

    // 主循环读数相对最后一次 ISR 读数扩展到 64 位(两次读之间回绕了就本地补上: 此时
    // 高位尚未被下一次 SOF 中断推进)。拟合每 20 ms 重算、参考点在窗口前缘, 正常时离
    // 它是几万个 tick; 超过 2^30(268 s)说明拟合早已不再更新, 不作答。
    const std::uint32_t low = timer::Timer::timestamp_quarter_us();
    const std::uint64_t now = sof_timebase.extended_after_latest(low);
    if (!sof_timebase.microframe_q16_at(
            static_cast<std::int64_t>(now), out_microframe, out_fraction_q16))
        return false;
    out_quarter_us = low;
    return true;
}

} // namespace libhcs::firmware::sync::timebase
