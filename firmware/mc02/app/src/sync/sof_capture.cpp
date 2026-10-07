#include "firmware/mc02/app/src/sync/sof_capture.hpp"

#include <array>
#include <bit>

#include <main.h>

#include "core/src/time/counter_link.hpp"
#include "core/src/time/sof_capture_ring.hpp"
#include "core/src/time/usb_sof_bits.hpp"
#include "firmware/common/app/src/utility/interrupt_lock.hpp"
#include "firmware/mc02/app/src/sync/timebase.hpp"

namespace libhcs::firmware::sync::sof_capture {
namespace {

// 本模块的状态全在零等待 DTCM: 每帧的 stamp_of() 要读环发布的直线与下面几个参数,
// 默认的 .bss 在 AXI SRAM 前 32 KB, 是非缓存的(mc02 AGENTS.md)。

// 8 条: 全速端口每 1 ms 一个 SOF, 可读 7 条。帧只用它附近的锁存值定位(帧两侧那一对,
// 或最新一个加上一段的速率, 见 sof_capture_ring.hpp), 用不到更多。
[[gnu::section(".dtcm")]] constinit core::time::SofCaptureRing<8> ring{
    {.modulus = 0, .nominal_per_microframe = 0}
};

// TIM3 <-> TIM5 的整数关系(core/src/time/counter_link.hpp)。
[[gnu::section(".dtcm")]] constinit core::time::CounterLink link{};

constexpr std::uint32_t kTim3Modulus = 0x10000U;

// 抽查时允许的松量, 单位 TIM5 拍: 标定经过的 TRGO -> ITR 同步路径晚一两个时钟。
constexpr std::uint32_t kLinkSlack = 8;

[[gnu::section(".dtcm")]] constinit bool usable = false;
// 锁存值年龄的上限: 半个微帧, 单位 TIM5 拍。
[[gnu::section(".dtcm")]] constinit std::uint32_t fresh_limit = 0;
// SOF 包本身的传输时长, 单位 TIM5 拍(与 timebase 同一约定: 对齐到包起始)。
[[gnu::section(".dtcm")]] constinit std::uint32_t packet_delay = 0;
// TIM3 回绕窗口的 3/4, 单位 ns 与 TIM3 拍。帧起始到中断不能比这更久, 否则 16 位
// 时间戳认不出过了几圈。
[[gnu::section(".dtcm")]] constinit std::uint32_t window_ns = 0;
[[gnu::section(".dtcm")]] constinit std::uint32_t window_ticks = 0;
// SOF 中断里用的帧号步长(全速 8), configure() 时取一次, 不必每个 SOF 读速度寄存器。
[[gnu::section(".dtcm")]] constinit std::uint32_t microframes_per_sof = 8;
// 全速 SOF 包里每多一个填充位, SOF 沿晚 83.3 ns: 0 / 1 / 2 位对应的 TIM5 拍数。高速下
// 一位 2.1 ns, 不补(全为 0)。见 core/src/time/usb_sof_bits.hpp。
[[gnu::section(".dtcm")]] constinit std::array<std::uint32_t, 3> stuffed_delay{};

[[gnu::section(".dtcm")]] constinit std::uint32_t fresh_count = 0;
[[gnu::section(".dtcm")]] constinit std::uint32_t stale_count = 0;

// TIM3 回绕到 0 的那一拍, TIM5 读多少 -- 即 link.offset。做法: 让 TIM3 在更新事件上发
// TRGO, TIM5 的触发暂时选 ITR2(TIM3 TRGO, 同步连接 [RM0468 Table 94]), CH2 从 TRC 捕获;
// 等到一次回绕, 捕获值就是答案(晚 TRGO -> ITR 同步路径那一两个时钟, 常数)。之后触发
// 换回 ITR7(SOF)。TIM3 一圈 477 us, 等 2 ms 没等到就判失败。
//
// 不用"读 TIM5、读 TIM3、读 TIM5"夹出来: 两次 TIM5 读与 TIM3 的采样点之间各隔几个总线
// 周期, 夹出的区间收不到一个点, 只能收到读延迟那么宽(约十拍)。
std::optional<std::uint32_t> measure_offset() {
    const std::uint32_t smcr = TIM5->SMCR;
    const std::uint32_t tim3_cr2 = TIM3->CR2;

    TIM5->SMCR = (smcr & ~TIM_SMCR_TS) | TIM_TS_ITR2;
    TIM3->CR2 = (tim3_cr2 & ~TIM_CR2_MMS) | TIM_TRGO_UPDATE;
    TIM5->SR = ~TIM_SR_CC2IF;

    const std::uint32_t start = DWT->CYCCNT;
    const std::uint32_t timeout = timebase::kCyclesPerMicrosecond * 2000U;
    bool captured = false;
    while (DWT->CYCCNT - start < timeout) {
        if ((TIM5->SR & TIM_SR_CC2IF) != 0U) {
            captured = true;
            break;
        }
    }
    const std::uint32_t offset = TIM5->CCR2;

    TIM3->CR2 = tim3_cr2;
    TIM5->SMCR = smcr;
    TIM5->SR = ~TIM_SR_CC2IF;
    if (!captured)
        return std::nullopt;
    return offset;
}

// 一次"读 TIM5、读 TIM3、读 TIM5", 中断关着, 三次读紧挨着。
bool link_holds() {
    const utility::InterruptLockGuard guard;
    const std::uint32_t before = TIM5->CNT;
    const auto narrow = static_cast<std::uint16_t>(TIM3->CNT);
    const std::uint32_t after = TIM5->CNT;
    return link.agrees(before, narrow, after, kLinkSlack);
}

} // namespace

void configure(std::uint32_t tim5_cycles_per_tick) {
    usable = false;

    // TIM3: 满 16 位(ARR = 65535), 预分频是 TIM5 的 2 的幂倍(拼接要整移位), 一拍不超过
    // 40 ns -- 再粗就不如不打。值来自 .ioc(TIM3.Prescaler = 1, TIM3.Period = 65535),
    // 这里只检查。
    const std::uint32_t tim5_prescale = TIM5->PSC + 1U;
    const std::uint32_t tim3_prescale = TIM3->PSC + 1U;
    if (TIM3->ARR != kTim3Modulus - 1U || tim3_prescale % tim5_prescale != 0U)
        return;
    const std::uint32_t ratio = tim3_prescale / tim5_prescale;
    if (!std::has_single_bit(ratio))
        return;
    const std::uint32_t tim3_cycles_per_tick = tim5_cycles_per_tick * ratio;
    if (tim3_cycles_per_tick * 1000U > timebase::kCyclesPerMicrosecond * 40U)
        return;

    // 每微帧的 TIM5 拍数须为整数, 环的标称值才精确(275 MHz 时 34375)。
    const std::uint32_t cycles_per_microframe = timebase::kNominalCyclesPerMicroframe;
    if (cycles_per_microframe % tim5_cycles_per_tick != 0U)
        return;
    const std::uint32_t nominal = cycles_per_microframe / tim5_cycles_per_tick;

    microframes_per_sof = timebase::microframes_per_sof();
    if (microframes_per_sof == 8U) {
        // 12 Mbit/s: 一位 = 每微秒 TIM5 拍数 / 12。
        const std::uint32_t ticks_per_us = timebase::kCyclesPerMicrosecond / tim5_cycles_per_tick;
        for (std::uint32_t bits = 0; bits < stuffed_delay.size(); ++bits)
            stuffed_delay[bits] = ((bits * ticks_per_us) + 6U) / 12U;
    }
    fresh_limit = nominal / 2U;
    packet_delay = (timebase::sof_packet_delay_ns() * timebase::kCyclesPerMicrosecond)
                 / (1000U * tim5_cycles_per_tick);
    window_ticks = (kTim3Modulus / 4U) * 3U;
    const std::uint64_t tick_ps =
        (std::uint64_t{tim3_cycles_per_tick} * 1'000'000U) / timebase::kCyclesPerMicrosecond;
    window_ns = static_cast<std::uint32_t>((tick_ps * window_ticks) / 1000U);

    // TIM3 走起来(MX_TIM3_Init 不启动计数器); 只置 CEN, CH4 的输出通道不碰。
    TIM3->CR1 |= TIM_CR1_CEN;

    std::optional<std::uint32_t> offset;
    {
        const utility::InterruptLockGuard guard;
        offset = measure_offset();
    }
    if (!offset)
        return;
    link = core::time::CounterLink{
        .shift = static_cast<std::uint32_t>(std::countr_zero(ratio)), .offset = *offset};
    if (!link_holds())
        return;

    const decltype(ring)::Counter counter{.modulus = 0, .nominal_per_microframe = nominal};
    if (!decltype(ring)::supports(counter))
        return;
    {
        const utility::InterruptLockGuard guard;
        ring.configure(counter);
        usable = true;
    }
}

void reset() {
    const utility::InterruptLockGuard guard;
    ring.clear();
    fresh_count = 0;
    stale_count = 0;
}

void note_sof(
    std::uint32_t frame, std::uint32_t capture, std::uint32_t counter, std::uint32_t frame_again) {
    if (!usable)
        return;

    // 锁存值的年龄: 不到半个微帧才算本次 SOF 的; 读完 CCR2/CNT 帧号又变了(处理被推迟
    // 到下一个 SOF 之后), 也不配对。与 hpm_board 的 note_sof() 同一道归属判定。TIM5
    // 满 32 位回绕, 普通无符号减法即是差。
    if (counter - capture >= fresh_limit || frame_again != frame) [[unlikely]] {
        stale_count++;
        return;
    }
    // 对齐到包起始: 减去 35 位的包长, 再减去这个帧号的 SOF 包里插入的填充位。
    const std::uint32_t stuffed = core::time::full_speed_sof_stuffed_bits(frame);
    ring.push(frame * microframes_per_sof, capture - packet_delay - stuffed_delay[stuffed]);
    fresh_count++;
}

void refit() {
    if (!usable)
        return;
    // TIM3 / TIM5 被谁重载过(写 CNT、PSC、ARR、UG), 帧时间戳的换算就不再对: 当即停打戳。
    // 要恢复得重新 configure(), 即复位。
    if (!link_holds()) [[unlikely]]
        usable = false;
}

// CAN 接收中断逐帧调用(时间基准开着时), 与 stamp_of() 一样放 ITCM。
__attribute__((section(".itcm"))) bool window_covers(std::uint32_t frame_age_ns) {
    return usable && frame_age_ns < window_ns;
}

// FDCAN 接收中断每帧调用一次。放 ITCM, 与 Can::handle_uplink() 一致; 拼接与环的
// locate() 都内联在这里。
__attribute__((section(".itcm"))) std::optional<time::SofStamp>
    stamp_of(std::uint16_t fdcan_timestamp) {
    if (!usable)
        return std::nullopt;
    const auto start = link.widen(fdcan_timestamp, TIM5->CNT, window_ticks);
    if (!start)
        return std::nullopt;
    return ring.locate(*start);
}

Counts take_counts() {
    const utility::InterruptLockGuard guard;
    const Counts result{
        .fresh = static_cast<std::uint16_t>(fresh_count > 0xFFFFU ? 0xFFFFU : fresh_count),
        .stale = static_cast<std::uint16_t>(stale_count > 0xFFFFU ? 0xFFFFU : stale_count),
    };
    fresh_count = 0;
    stale_count = 0;
    return result;
}

} // namespace libhcs::firmware::sync::sof_capture
