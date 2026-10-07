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
// 每个镜像都带, 但只在主机的清单要了时间基准时运行(sync::time_sync_start()):
// 它的输入是每秒 8000 次的 SOF 中断, 不要的板子不该付这笔账。
//
// 计数器、拟合、锚点与状态机的运算三块板共用一份: core/src/time/sof_timebase.hpp。
// 本文件与 .cpp 只剩本芯片的事: 读 FRINDEX 与端口速度、对时间戳扣包时长、中断锁、
// 喂 pulse 模块。本板的拟合时钟就是协议的 quarter-us(4 MHz 机器定时器), 不必换算。

#include <cstdint>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/time/sof_timebase.hpp"

namespace libhcs::firmware::sync::timebase {

// 每 64 microframe 取一个拟合样本, 128 样本环形缓冲: 8192 microframe
// (1.024 s)基线只占 512 字节 RAM。基线决定斜率精度(端点噪声除以窗口),
// 样本数负责把相位噪声平均下去 -- 128 个样本把 ~2 us 的中断抖动压到
// 0.2 us 以下。
inline constexpr std::uint32_t kSampleDecimation = core::time::kSampleDecimation;
inline constexpr std::uint32_t kSampleCount = core::time::kSampleCount;

// 标称每 microframe 的本地 tick 数: 4 MHz 机器定时器, microframe 125 us。
inline constexpr std::uint32_t kNominalTicksPerMicroframe = 500U;

// 单位 quarter-us(即本板定时器 tick); 字段说明见 core::time::TimebaseSnapshot。
using Snapshot = core::time::TimebaseSnapshot;

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

// 回到上电状态(计数器未播种、无拟合、未锚定)。只在 SOF 中断关着时调用:
// sync::time_sync_start() 在打开它之前。
void reset();

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
// 绝对微帧。返回的 (microframe + fraction/65536, quarter_us) 是一对同时刻的
// 配对 -- 微帧为插值, quarter_us 为本读数的低 32 位, 与本板其余遥测同一时钟。
// host_session 回答 kTimeAnchor 时用它上报"此刻"而非最后一次 SOF。
//
// 小数部分不是可有可无的: 只报整数微帧等于报"上一个 SOF 时的计数", 相对"此刻"
// 落后 0..125 us 均匀分布。2026-10-03 之前本函数只返回整数, 插值算出的小数在
// 整除里丢掉了, 于是主机拿到的仍是那个落后的值 -- 它既进入主机对自身时钟的拟合
// (均值偏 62.5 us), 也进入"主机计数器与板计数器差几个整数微帧"的判定, 而后者
// 的全部余量只有半个微帧。[读代码得出, 2026-10-03; 未单独上板复测]
// 时间线无效或拟合未收敛时返回 false, 调用方回退到 snapshot() 的锁存对。
bool microframe_now(
    std::uint64_t& out_microframe, std::uint16_t& out_fraction_q16, std::uint32_t& out_quarter_us);

} // namespace libhcs::firmware::sync::timebase
