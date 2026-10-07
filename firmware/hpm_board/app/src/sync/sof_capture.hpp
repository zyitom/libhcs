#pragma once

// 把硬件时间戳(PTPC 域)直接放到共享 USB microframe 轴上。
//
// 是什么: CAN 帧起始沿由 TSU 硬件锁进 PTPC0; 本模块让每个 SOF 沿也由硬件锁进
// 同一个 PTPC0(TRGM: USB0_SOF -> PTPC0 capture)。两个事件落在同一个计数器里,
// 帧在轴上的位置就是
//
//     第 k 微帧 + (帧的锁存值 - SOF k 的锁存值) / (SOF k+1 的 - SOF k 的)
//
// 运算见 core/src/time/sof_capture_ring.hpp。全程不读"现在"、不经机器定时器,
// 所以中断进入抖动进不了结果; 分母就是帧所在那一段里 PTPC 实际走的数, 所以 PTPC 的
// 快慢、以及它在毫秒尺度上的游走也进不了结果。SOF k+1 还没锁到时, 用最近 4 个微帧的
// 速率往前推一两个微帧。曾用穿过 31 个锁存值的直线外推, 被这种游走拖到每块板约 110 ns,
// 见 SOF_TIMEBASE.md 8.7.8。
//
// 与 timebase 的分工: timebase 维护 64 位绝对微帧与"本地机器定时器 <-> 微帧"的
// 拟合, 回答主机的 anchor; 本模块只给带硬件时间戳的记录定位, 结果是
// libhcs::time::SofStamp(FRINDEX 低 10 位 + 14 位小数), 不依赖 anchor, 也不依赖
// timebase 的状态机。
//
// 为什么之前那条 PTPC 路(SOF_TIMEBASE.md 5.4)没走通而这条可以: 那条路把 SOF 锁存值
// 配上软件数的微帧号再去拟合直线, 第五次卡在"锁存值属于哪个 SOF"。这里不拟合,
// 归属也不靠推断 -- note_sof() 逐个 SOF 用 age 判定: 读到的锁存值离"现在"不到半个
// 微帧才算本次 SOF 的, 否则跳过并计数。于是触发信号无论每个 SOF 锁一次还是隔一个
// 锁一次, 环里的每一条都是对的; 跳过的比例经 kTimeStatus 上报。实测 5321 的这个
// 信号逐 SOF 翻转(只取上升沿时恰 50% 新鲜, 2026-10-05), 现取双沿, 应接近 100%。
//
// 主机的清单没要时间基准时(sync::time_sync_on() 为假), CAN 接收中断不调本模块, 帧上
// 不带时间戳, 也不必每帧去读 TSU 寄存器。

#include <cstdint>
#include <optional>

#include "core/include/libhcs/time/sof_stamp.hpp"

namespace libhcs::firmware::sync::sof_capture {

struct Counts {
    std::uint16_t fresh; // 锁存值新鲜、已入环的 SOF 数
    std::uint16_t stale; // 锁存值陈旧(或读取期间又来一个 SOF)、被跳过的 SOF 数
};

// 接通 USB0_SOF -> PTPC0 capture(只第一次), 清空环与计数。刻意不放在 boot 路径:
// 由主机的清单要了时间基准时调用(sync::time_sync_start()), 那时 USB 已枚举, 这里
// 出任何问题都还有 DFU 可救(SOF_TIMEBASE.md 5.5 的教训)。PTPC0 本身由 CAN 初始化启动。
void start();

// SOF 中断路径, 由 sync::sof_isr_entry() 在读完 FRINDEX 之后调用。只入环, 几条指令:
// 这个中断就是 USB 中断, 每 125 us 一次。
void note_sof(std::uint32_t frindex);

// PTPC0 纳秒字的一个读数(CAN TSU 时间戳的低 32 位) -> 共享轴上的位置。任何
// 上下文可调, 无锁; 给不出可信答案时为空。
std::optional<time::SofStamp> stamp_of(std::uint32_t ptpc_ns);

// 取走自上次调用以来的计数。仅周期上报状态的那一处调用。
Counts take_counts();

} // namespace libhcs::firmware::sync::sof_capture
