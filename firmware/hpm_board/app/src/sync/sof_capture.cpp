#include "firmware/hpm_board/app/src/sync/sof_capture.hpp"

#include <hpm_ptpc_drv.h>
#include <hpm_soc.h>
#include <hpm_trgm_drv.h>
#include <hpm_trgmmux_src.h>
#include <hpm_usb_regs.h>

#include "board_app.hpp"
#include "core/src/time/sof_capture_ring.hpp"
#include "firmware/common/app/src/utility/interrupt_lock.hpp"
#include "firmware/hpm_board/app/src/sync/timebase.hpp"

namespace libhcs::firmware::sync::sof_capture {
namespace {

// PTPC 数字模式的纳秒字在 1e9 处回绕。
constexpr std::uint32_t kPtpcModulus = 1'000'000'000U;

// 每微秒 PTPC 走的数(board::kCanTimestampNsPerUs: 5321 为 960, 因数字模式每拍
// 加 floor(1e9/f) 而非精确值)乘 125 us。只作合理性参照, 实际速率逐区间实测。
constexpr std::uint32_t kNominalUnitsPerMicroframe = 125U * board::kCanTimestampNsPerUs;

// 锁存值离"现在"超过半个微帧, 就不是本次 SOF 的。SOF 中断的进入延迟是微秒级,
// 离这条线差一个数量级以上; 上一个 SOF 的锁存值则至少有一整个微帧那么旧。
constexpr std::uint32_t kFreshLimit = kNominalUnitsPerMicroframe / 2U;

constexpr std::uint32_t kFrindexMask = 0x3FFFU;

// 16 条: 双沿捕获下每个微帧一条, 可读 15 条, 1.9 ms。帧只用它附近的锁存值定位(帧两侧
// 那一对, 或最新一个加最近 4 个微帧的速率, 见 sof_capture_ring.hpp), 所以条数只决定
// 能往回查多远: CAN 帧从起始沿到接收中断是帧长加中断延迟, 经典 8 字节帧约 130 us,
// 远在 1.9 ms 之内。不用更长的窗口拟合直线: 本板 SOF 锁存值相对 PTPC 在几毫秒里游走
// 几百纳秒 [实测 2026-10-05, SOF_TIMEBASE.md 8.7.8]。
constinit core::time::SofCaptureRing<16> ring{
    {.modulus = kPtpcModulus, .nominal_per_microframe = kNominalUnitsPerMicroframe}
};

bool routed = false;
std::uint32_t fresh_count = 0;
std::uint32_t stale_count = 0;

} // namespace

void start() {
    // 上一次运行留下的锁存值与计数作废: 中间隔着关掉的那段, 不能与新的配成一对。
    {
        const utility::InterruptLockGuard guard;
        ring.clear();
        fresh_count = 0;
        stale_count = 0;
    }
    if (routed)
        return;

    // USB0 SOF -> PTPC0 capture, 原样直通不整形: 整形只会多一拍 TRGM 时钟的
    // 不确定度。
    trgm_output_t output{};
    output.invert = false;
    output.type = trgm_output_same_as_input;
    output.input = HPM_TRGM0_INPUT_SRC_USB0_SOF;
    trgm_output_config(HPM_TRGM0, HPM_TRGM0_OUTPUT_SRC_MCAN_PTPC0_CAP, &output);

    // 每个沿都覆盖。"capture keep"会把第一个值保持到被读走, 这里要的相反:
    // 漏读一次不该让下一次读到旧值。
    //
    // 双沿: TRGM 的 USB0_SOF 是逐 SOF 翻转一次的电平, 不是每 SOF 一个脉冲 -- 手册里
    // 这路输入就叫 usb0_sof_tog_sync [HPM5300 UM Rev1.1 34.5.9 TRGM_IN], 实测也一致: 只取
    // 上升沿时新鲜率恰为 50.0%(74548 新鲜 / 74547 陈旧, 2026-10-05 两轮 Mc02Bench)。两个沿
    // 都取, 每个 SOF 都有锁存值; 某个 SOF 若没翻转, 之后的沿照样逐个正确配对。
    // [推断: 上升沿与下降沿经同一条同步链, 延迟相同; 奇偶 SOF 若有常数差, 环的
    //  最小二乘把它平均成一半, 未上板单独测]
    ptpc_disable_capture_keep(HPM_PTPC, PTPC_PTPC_0);
    ptpc_config_capture(HPM_PTPC, PTPC_PTPC_0, ptpc_capture_trigger_on_both_edges);
    routed = true;
}

// SOF 中断只在时间基准开着时调到这里(sync::sof_isr_entry()), 那时捕获已接通。
void note_sof(std::uint32_t frindex) {
    // 先读锁存值再读自由计数器: 后者必然晚于本次 SOF 沿, 两者之差就是锁存值的
    // 年龄。只读纳秒字 -- 秒字与它不同拍锁存(SOF_TIMEBASE.md 5.4 第 2 次的教训),
    // 而这里的差值恒小于一秒, 取模即可。
    const std::uint32_t capture = ptpc_get_capture_ns(HPM_PTPC, PTPC_PTPC_0);
    const std::uint32_t now = ptpc_get_timestamp_ns(HPM_PTPC, PTPC_PTPC_0);
    const std::uint32_t age = now >= capture ? now - capture : now + (kPtpcModulus - capture);

    // 陈旧: 本次 SOF 没有产生锁存(只取单沿时就是隔一个 SOF 的那一半), 或捕获尚未
    // 接通好。新鲜但 FRINDEX 已变: 本处理函数被推迟到下一个 SOF 之后才跑到这里,
    // 锁存值属于那个更新的 SOF 而 frindex 不是 -- 同样不能配成一对。
    if (age >= kFreshLimit || (HPM_USB0->FRINDEX & kFrindexMask) != (frindex & kFrindexMask))
        [[unlikely]] {
        stale_count++;
        return;
    }

    // 对齐到 SOF 包起始, 与 timebase 同一约定: 设备在包收完后才给出 SOF 事件,
    // 高速 133 ns、全速 2917 ns, 不减则混速舰队各板差一个常数。
    // [推断: TRGM 的 SOF 沿与 SRI 置位同刻; 未单独上板核对]
    const std::uint32_t delay =
        (timebase::sof_packet_delay_ns() * board::kCanTimestampNsPerUs) / 1000U;
    const std::uint32_t aligned =
        capture >= delay ? capture - delay : capture + (kPtpcModulus - delay);

    ring.push(frindex, aligned);
    fresh_count++;
}

// CAN 接收中断每帧调用一次: 放 ILM, 与 can.cpp 的其余热路径一致。环的 locate()
// 若未被内联, 其 COMDAT 段由链接脚本按名(libhcs::core::time)收进 ILM。
ATTR_PLACE_AT(".fast")
std::optional<time::SofStamp> stamp_of(std::uint32_t ptpc_ns) { return ring.locate(ptpc_ns); }

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
