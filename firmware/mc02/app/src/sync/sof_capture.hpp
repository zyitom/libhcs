#pragma once

// mc02: 把 FDCAN 的硬件帧时间戳放到共享 USB 微帧轴上。hpm_board 同名模块的对应物,
// 运算是同一份(core/src/time/sof_capture_ring.hpp), 区别全在"哪个计数器锁什么":
//
//   SOF 沿     TIM5 CH2 输入捕获, 触发源 ITR7 = USB1 OTG_HS SOF(sync/sof.cpp 的
//              capture_init())。TIM5 32 位、275 MHz(3.6 ns)、满 32 位回绕, 整个归时基。
//   CAN 帧起始 FDCAN 的外部时间戳(TSCC.TSS = 10): 手册写明就是 tim3_cnt[0:15], 在帧
//              起始沿锁进 RX 元素 R1[15:0] [RM0468 Rev 3 61.5.8]。TIM3 只有 16 位。
//
// 两个计数器吃同一个 APB1 定时器时钟, TIM5 满 32 位回绕, 2^32 又是 TIM3 一整圈(65536 拍
// x 预分频之比)的整数倍, 所以任何时刻
//
//     TIM3 = ((TIM5 - k) >> s) & 0xFFFF        s = log2(TIM3 预分频 / TIM5 预分频)
//
// k 是两个计数器起点之差, 常数。configure() 标定一次(让 TIM3 回绕时发 TRGO, 由 TIM5 经
// ITR2 硬件捕获), refit() 每毫秒用"读 TIM5、读 TIM3、读 TIM5"抽查一次。有了它, 接收中断里只读一次
// TIM5, 把帧的 16 位时间戳拼进去就得到帧起始的 TIM5 读数 -- 纯整数, 不读 TIM3, 没有
// "两次读之间"的量化与抖动。之后与 hpm_board 一样: 帧和 SOF 都在 TIM5 上, 由帧附近的
// SOF 锁存值定位。
//
// TIM3 / TIM5 的配置来自 .ioc, 本模块只检查不改写(CubeMX 纪律, 仓库根 AGENTS.md):
//   TIM5.Prescaler = 0, TIM5.Period = 4294967295
//   TIM3.Prescaler = 1, TIM3.Period = 65535    -> 7.3 ns 一拍, 477 us 回绕
// 不满足时不给帧打戳(帧照常转发)。TIM3_CH4(PB1)是 IMU 加热的 PWM: 固件从不启动它,
// CCR4 = 0; 上面的周期下 PWM 频率 2.1 kHz、占空比按 65536 计。谁写了 TIM3 / TIM5 的
// CNT、PSC、ARR 或 UG, 抽查当即发现, 帧不再带时间戳。
//
// 未验证的假设 [2026-10-05, 只过编译]:
//  - TIM5 ITR7 真有 SOF(手册两张表都这么写; 2026-09-07 的扫描记录相反, 见 sof.cpp)。
//  - fdcan1_ts 在 APB 时钟域被采样(手册框图), TIM3 时钟与 APB1 同源同步, 应当不会锁到
//    撕裂的值; 三板台架上与 5321 对同一帧比对, 离群点即反证。三个 FDCAN 共用这一路
//    时间戳输入(框图), 也待上板确认。
//  - TIM5 捕获的 SOF 沿相对线上 SOF 包的延迟与 5321 不同(USB IP、PHY 都不同),
//    两块板的轴之间因此有一个常数, 要在台架上标定。

#include <cstdint>
#include <optional>

#include "core/include/libhcs/time/sof_stamp.hpp"

namespace libhcs::firmware::sync::sof_capture {

struct Counts {
    std::uint16_t fresh; // 锁存值新鲜、已入环的 SOF 数
    std::uint16_t stale; // 锁存值陈旧(或读取期间帧号变了)、被跳过的 SOF 数
};

// sof.cpp 的 capture_init() 确认 TIM5 捕获可用后调用一次(第一次开时间基准时, USB 已
// 枚举): TIM5 一拍折合的 CPU 周期数。在这里检查 TIM3、让它走起来, 并标定 TIM3 <-> TIM5
// 的常数。中断关着跑, 至多一圈 TIM3(477 us)。
void configure(std::uint32_t tim5_cycles_per_tick);

// 清空环与计数: 上一次运行留下的锁存值不能与新的配成一对。只在 SOF 中断关着时调用
// (sync::time_sync_start())。configure() 的结果保留。
void reset();

// SOF 中断路径: frame 为 DSTS.FNSOF, capture / counter 为同一次中断里读到的 TIM5
// CCR2 与 CNT, frame_again 为读完它们之后再读的 FNSOF。只入环。
void note_sof(
    std::uint32_t frame, std::uint32_t capture, std::uint32_t counter, std::uint32_t frame_again);

// 主循环, 约每毫秒一次: 抽查 TIM3 <-> TIM5 关系, 不符即停打戳。
void refit();

// 帧起始到接收中断最长可能隔多久(ns)时, TIM3 的回绕窗口还认得出来。CAN 接收中断按
// 本总线的标称位时间(配置时算好)逐帧问; 认不出(总线太慢、或捕获没配上)就不打戳。
// 一次内存读、一次比较。
bool window_covers(std::uint32_t frame_age_ns);

// FDCAN RX 元素的 16 位时间戳 -> 共享轴上的位置。FDCAN 接收中断里调用。
std::optional<time::SofStamp> stamp_of(std::uint16_t fdcan_timestamp);

// 取走自上次调用以来的计数。仅周期上报状态的那一处调用。
Counts take_counts();

} // namespace libhcs::firmware::sync::sof_capture
