#pragma once

// 基于 USB microframe 计数器的跨板共享时间基准。
//
// 是什么: 同一 USB 主机控制器下的每块板看到相同的 SOF 包, 因而同一瞬间
// FRINDEX 处处相同 -- 一条由硬件分发的时钟, 没有引入偏差的软件路径。本模块
// 把它变成可用的时间线: 64 位 microframe 计数器; 对本地机器定时器的滑动
// 窗口拟合, 使任意本地时刻可用 microframe 命名、任意 microframe 可用本地
// tick 命名; 以及计数器不可信时拒绝作答的状态机。
//
// 主机时间戳为何不能劣化它: 计数器低 14 位就是 FRINDEX 本身, 硬件精确且
// 各板一致。主机的 anchor 消息只提供回绕数(那些位属于 16384 的哪个倍数),
// 因此 anchor 量化到 2.048 s, 误差须超过 +-1.024 s 才起任何作用。给定同一
// anchor 的两块板, 即使 anchor 大错特错也会算出逐位相同的绝对 microframe
// -- 绝对正确性依赖主机准确, 跨板一致性则与主机时钟毫无关系。这正是拆分
// 的全部意义, 也是 anchor 必须每轮在主机上只算一次、原样发给每块板的原因。
//
// 何时失效: microframe 增量不为 1 的任何情况。delta 为 2..7 是少量丢失
// 中断: 计数器仍可修正(按真实 delta 前进), 但拟合窗口作废, 因为那些样本会
// 拉歪直线。delta 为 0 或 >=8 说明计数器本身存疑 -- 时间线失效, 收到新
// anchor 前不得对它排定任何动作。已验证的硬件上稳态不会落入任一分支
// (SOF_TIMEBASE.md), 但它们是安全网, 不是待删的死代码。
//
// 未定义 libhcs_APP_TIME_SYNC 时整体编译剔除。开启会给板子增加一个 8 kHz
// 中断, 因此默认不开启。

#include <cstdint>

#include "core/include/libhcs/data/datas.hpp"

namespace libhcs::firmware::sync::timebase {

#if defined(libhcs_APP_TIME_SYNC) && libhcs_APP_TIME_SYNC

inline constexpr bool kEnabled = true;

// 每 64 microframe 取一个拟合样本, 128 样本环形缓冲: 8192 microframe
// (1.024 s)基线只占 512 字节 RAM。基线决定斜率精度(端点噪声除以窗口),
// 样本数负责把相位噪声平均下去 -- 128 个样本把 ~2 us 的中断抖动压到
// 0.2 us 以下。
inline constexpr std::uint32_t kSampleDecimation = 64U;
inline constexpr std::uint32_t kSampleCount = 128U;

// 标称每 microframe 的本地 tick 数: 4 MHz 机器定时器, microframe 125 us。
inline constexpr std::uint32_t kNominalTicksPerMicroframe = 500U;

struct Snapshot {
    data::TimeState state;
    // kValid 时为绝对 microframe; 其余状态下是本板自身原点。
    std::uint64_t microframe;
    std::uint64_t timestamp_quarter_us;
    // 拟合出的每 microframe 本地 tick 数, Q16。拟合收敛前为零。
    std::uint32_t ticks_per_microframe_q16;
    std::uint32_t anomaly_count;
    // 自上次 report() 以来本板自身拟合的样本外预测误差, 单位 Q16 timer
    // tick。均值才是会变成跨板偏差的分量; 极值为何不是, 见
    // data::TimeStatusView 的说明。
    std::int32_t residual_mean_q16;
    std::uint32_t residual_abs_max_q16;
    std::uint32_t residual_count;
};

// 当前端口速率下每个 SOF 中断的 microframe 数: 高速 1, 全速 8(FRINDEX 两种
// 速率下都数 microframe, 但全速端口每 1 ms 帧只收到一个 SOF)。发布出来是
// 因为 microframe 流的每个消费者都需要这个步进, 不止本模块。
std::uint32_t microframes_per_sof();

// 当前端口速率下 SOF 包本身的传输时间, 单位 ns。设备在包"已收完"时才置起
// SOF-received 标志, 故该中断里取的每个时间戳都晚这么多 -- 480 Mbit 下
// 0.13 us, 12 Mbit 下 2.92 us。note_sof() 已将其从拟合样本中减去, 使各板的
// "微帧 k 的本地时刻"对齐到包起始; 混速舰队的残余只剩两板中断进入延迟之差。
// [实测 2026-08-20, HS+FS 对: -2854 / -2764 / -2810 ns, 对计算值 2.78 us
//  -- 偏差在 1% 以内。]
std::uint32_t sof_packet_delay_ns();

// ISR 路径, 由 sync::sof_isr_entry() 携其读得的值调用。
void note_sof(std::uint32_t frindex, std::uint32_t now_quarter_us);

// 主循环: 重算拟合。每趟调用都够便宜, 只有每 kFitPeriodMs 才做实事。
void poll(std::uint32_t tick_ms);

// 应用主机 anchor。回绕数不变时幂等; 已生效时间线上回绕数变化按故障处理
// 而非静默接受, 因为它只可能意味着计数器或主机估计值动了超过一秒。
void apply_anchor(std::uint64_t host_microframe);

Snapshot snapshot();

// 同 snapshot(), 但消费残差累加器。仅一个调用方 -- 周期上报状态的那个 --
// 使每个均值覆盖的窗口有明确边界。
Snapshot report();

// 时间线查询。时间线无效时都返回 false, 调用方不可能误对着停摆的时钟
// 排程。
bool local_time_of(std::uint64_t microframe, std::uint64_t& out_quarter_us);
bool microframe_at(std::uint64_t quarter_us, std::uint64_t& out_microframe);

// 主循环查询"现在": 读一次机器定时器, 扩展到 64 位并沿拟合线插值到当前
// 绝对微帧。返回的 (microframe, quarter_us) 是一对同时刻的配对 -- 微帧为
// 插值, quarter_us 为本读数的低 32 位, 与本板其余遥测同一时钟。host_session
// 回答 kTimeAnchor 时用它上报"此刻"而非最后一次 SOF: 后者落后 0..125 us
// 均匀分布, 会原样进入主机对自身时钟的拟合。时间线无效或拟合未收敛时
// 返回 false, 调用方回退到 snapshot() 的锁存对。
bool microframe_now(std::uint64_t& out_microframe, std::uint32_t& out_quarter_us);

#else

inline constexpr bool kEnabled = false;

// 必须与启用分支的 Snapshot 逐字段对应: 头文件之外的调用方无条件读这些
// 成员, 落后于真实结构体的桩只会破坏 TIME_SYNC=OFF 构建 -- 恰是开发时间
// 基准期间最不容易顺手编译的配置。
struct Snapshot {
    data::TimeState state;
    std::uint64_t microframe;
    std::uint64_t timestamp_quarter_us;
    std::uint32_t ticks_per_microframe_q16;
    std::uint32_t anomaly_count;
    std::int32_t residual_mean_q16;
    std::uint32_t residual_abs_max_q16;
    std::uint32_t residual_count;
};

inline void note_sof(std::uint32_t, std::uint32_t) {}
inline void poll(std::uint32_t) {}
inline std::uint32_t microframes_per_sof() { return 1; }
inline std::uint32_t sof_packet_delay_ns() { return 0; }
inline void apply_anchor(std::uint64_t) {}
inline Snapshot snapshot() {
    return {
        .state = data::TimeState::kInvalid,
        .microframe = 0,
        .timestamp_quarter_us = 0,
        .ticks_per_microframe_q16 = 0,
        .anomaly_count = 0,
        .residual_mean_q16 = 0,
        .residual_abs_max_q16 = 0,
        .residual_count = 0,
    };
}
inline Snapshot report() { return snapshot(); }
inline bool local_time_of(std::uint64_t, std::uint64_t&) { return false; }
inline bool microframe_at(std::uint64_t, std::uint64_t&) { return false; }
inline bool microframe_now(std::uint64_t&, std::uint32_t&) { return false; }

#endif

} // namespace libhcs::firmware::sync::timebase
