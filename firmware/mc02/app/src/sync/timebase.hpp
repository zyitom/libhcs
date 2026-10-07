#pragma once

// 基于 USB 微帧计数器的 mc02 跨板共享时间基准。
//
// 原理: 同一 USB 主控下的每块板看到的 SOF 相同, 帧计数器在同一时刻读数一致
// ——硬件分发的时钟, 没有软件路径引入偏差。本模块把它变成可用的时间线: 64 位
// 微帧计数器、对本地高分辨率时钟的滑动窗口拟合(本地时刻 <-> 微帧双向换算),
// 以及在计数器不可信时拒绝作答的状态机。
//
// 主机锚点为何无法污染它: 计数器低 14 位直接来自硬件帧计数器, 精确且各板一致;
// 主机锚点只提供回绕(那些位属于 16384 的哪个倍数), 量化到 2.048 s, 误差须超过
// +-1.024 s 才有影响。绝对准确需要主机时钟准确; 跨板一致性则完全不依赖主机时钟。
//
// 计数器、拟合、锚点与状态机的运算三块板共用一份: core/src/time/sof_timebase.hpp。
// 本模块最初移植自 firmware/hpm_board/app/src/sync/timebase.{hpp,cpp}, 去掉 HPM 专属部分
// (PTPC 捕获、硬件 SOF->PTPC 触发路由、CAN TSU 换算、GPTMR 脉冲层)。针对本芯片
// 有三处改动, 也正是错误假设最容易藏身之处, 故在此说明而非留在 diff 里:
//
//  1. 仅全速, 且帧计数器数的是帧。mc02 无 ULPI PHY: USB_OTG_HS 走内置全速 PHY
//     (12 Mbit)。DWC2 的 DSTS.FNSOF 在高速下是微帧号、全速下是帧号, 故每 1 ms
//     一次 SOF 中断、每次 +1; hpm_board 的 EHCI FRINDEX 数微帧、步进 8。乘 8 后
//     两板落在同一微帧轴、同一 16384 模数上 -- 见 .cpp 的 frame_scale()。
//
//  2. 本地时钟是周期计数器, 不是协议的 quarter-us tick。mc02 的 timer::Timer 是
//     TIM23(2026-10-05 前是 TIM5)预分频到 1 MHz、按 CNT << 2 上报, quarter-us 值以 4
//     为步进: 1 us 量化, 与本模块要测的中断抖动同量级。拿与被测量同粗的尺子采样, 结果无法解读, 故
//     拟合在 DWT->CYCCNT(550 MHz, 1.8 ns)上做, 仅在协议要求处换算成 quarter-us。
//     关键方向的换算是精确的: 68750 cycles/microframe 恰为 500 quarter-us, 主机
//     按标称 500 的算法无需改动。
//
//  3. 本芯片存在硬件 SOF 捕获, 用作中断时间戳的修正而非替代(模块最初误以为它
//     不存在; 实测记录在案, 以免重查)。实测 2026-09-07: 把 TIM2 的 SMCR.TS 在
//     ITR0..ITR13 间扫描、CH2 映射到 TRC, 仅 ITR5 以 SOF 速率产生捕获, 其余来源
//     读数恰为 0; 捕获间隔下限 999 个 TIM2 tick(当时 1 MHz), 即 1 ms 全速帧。
//     USB1_OTG_HS_SOF -> TIM2 ITR5 属实, 是 HPM TRGM->PTPC 路由的直接对应物。当时
//     记下的"TIM5 无此来源"与手册矛盾: RM0468 Rev 3 Table 94 / Table 355 都写着
//     USB1 SOF 也接 TIM5 的 ITR7, 那次扫描多半写错了 TS 编码(H7 上 ITR4 起不连续)。
//     (注意这条路径无法靠 grep HAL 得到: STM32F4 有 TIM_TIM2_USBFS_SOF
//     命名, H7 HAL 只暴露 TIM_TS_ITR0..13, 每个定时器的含义在 RM0468 的内部触发表。)
//
//     2026-10-05 起捕获在 TIM5 ITR7 上(32 位、不占引脚, 整个归共享时基, 275 MHz,
//     满 32 位回绕), TIM2 完全还给 PA0/PA2 的 PWM, 理由见 sof.cpp。[TIM5 ITR7 未上板]
//
//     本地时钟仍是 CYCCNT: 捕获只告诉 ISR 边沿比它早到多少定时器 tick, 从周期计数
//     里扣掉(sync/sof.cpp)。硬件捕获在边沿上精确, 但上限取决于被锁存的计数器 --
//     精确边沿锁进粗计数器等于丢掉精确性, 1 MHz 时强开修正实测毫无效果。故捕获用的
//     定时器预分频为 0(275 MHz, tick 3.6 ns)。
//
//     效果 [实测 2026-09-16, 当时在 TIM2 上, host/examples/mc02_time_sync_test 各 60 s, 同一块板
//     交替烧录, 开 4-5 组、关 2 组; "关"为只把捕获门控强制为 false 的对照镜像]:
//
//                   单样本 sigma (空载 / 灌满下行)    最差单样本
//       捕获开      0.0183-0.0207 / 0.0177-0.0196 us   0.10 us
//       捕获关      0.0294-0.0331 / 0.0351-0.0374 us   0.39-1.12 us
//
//     两者都 0 异常、时间线全程有效。最差样本降一个数量级是主要收益: 它来自偶发
//     抢占推迟了 ISR, 捕获关时只能靠 128 样本最小二乘把它埋掉, 捕获开时这段推迟
//     连同常数入口延迟一起被扣除。入口延迟的均值本就不是误差 -- 那是拟合偏移吸收的
//     常数, 且在跑同一代码的两板间完全抵消。
//
//     TIM5 的约束(计数器不得复位、ARR/PSC 不得运行时改写)见 sof.cpp 的 capture_init()。
//
//     J-Link 挂接一次, 本模块就失效到复位为止: 测时间基期间不要连调试器, 读完寄存器
//     先复位再测。[实测 2026-09-16, 两次复现: 一次只读 mem32 会话之后拟合始终无效,
//     样本环里全是近乎相同的周期计数, 推断 CYCCNT 被调试器停走; 重新烧录、不碰
//     J-Link 即恢复。]
//
// 何时失效: 微帧增量不是期望步长。差几步 = 漏了几次中断: 计数器仍可修正(它按
// 真实 delta 前进), 但拟合窗口作废, 缺口两侧样本会使直线倾斜。delta 为 0、或
// 8 步及以上无法解释, 则计数器本身存疑: 时间线失效, 新锚点到来前不得对其调度。
//
// 每个镜像都带, 但只在主机的清单要了时间基准时运行(sync::time_sync_start()): 它的
// 输入是每秒 1000 次的 SOF 中断, 不要的板子不该付这笔账。

#include <cstdint>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/time/sof_timebase.hpp"

namespace libhcs::firmware::sync::timebase {

// 每微秒 CPU 周期数 / 每微帧周期数。SYSCLK = PLL1P = 24 MHz / 3 * (68 + 6144/8192)
// = 550 MHz, 精确值(main.c SystemClock_Config); DWT->CYCCNT 数 CPU 周期, 故这是
// 精确换算而非舍入。
inline constexpr std::uint32_t kCyclesPerMicrosecond = 550U;
inline constexpr std::uint32_t kNominalCyclesPerMicroframe = kCyclesPerMicrosecond * 125U;

// 68750 cycles 恰为 500 quarter-us: 协议标称值与 4 MHz 板一致, 主机无需按板缩放。
// 以 static_assert 断言而非口说。
static_assert(kNominalCyclesPerMicroframe * 4U % kCyclesPerMicrosecond == 0U);
static_assert(kNominalCyclesPerMicroframe * 4U / kCyclesPerMicrosecond == 500U);

// 每 64 微帧取一个拟合样本, 128 样本环形窗口。全速下计数器每 SOF 前进 8 微帧,
// 即每第 8 次中断取一个样本: 间隔 8 ms、基线 1.024 s、512 字节 RAM -- 与
// hpm_board 高速下的间距和窗口一致, 两板的 residual 数值才可直接比较。
//
// 基线决定斜率精度(端点噪声除以窗口长), 样本数把相位噪声平均下去。窗口还须避开
// 550 MHz 下周期计数器 7.81 s 的回绕, 1.024 s 对此有 7 倍余量。
inline constexpr std::uint32_t kSampleDecimation = core::time::kSampleDecimation;
inline constexpr std::uint32_t kSampleCount = core::time::kSampleCount;

// 单位是协议的 quarter-us(拟合在周期计数器上, 输出时换算); 字段说明见
// core::time::TimebaseSnapshot。
using Snapshot = core::time::TimebaseSnapshot;

// 当前端口速度下计数器每 SOF 中断前进的微帧数: 全速 8, 高速 1。微帧流的每个
// 消费者都需要该步长, 故不只是本模块内部使用。
std::uint32_t microframes_per_sof();

// 当前端口速度下 SOF 包本身的传输时长, 单位 ns。设备在 SOF 包整个收到之后才置起
// 标志, 故该中断里取的每个时间戳都晚了这么多 -- 480 Mbit 下 0.13 us, 12 Mbit 下
// 2.92 us。note_sof() 已将其从拟合样本中减去, 使各板的"微帧 k 的本地时刻"对齐到
// 包起始; 混速舰队的残余只剩两板中断进入延迟之差。
// [实测 2026-08-20, 一对 HS+FS 的 HPM 板: -2854 / -2764 / -2810 ns, 与计算的
//  2.78 us 吻合, 误差 1% 以内。]
std::uint32_t sof_packet_delay_ns();

// 回到上电状态(计数器未播种、无拟合、未锚定)。只在 SOF 中断关着时调用:
// sync::time_sync_start() 在打开它之前。
void reset();

// ISR 路径, 由 sync::sof_isr_entry() 以其读到的值调用。frame 为原样读出的
// DSTS.FNSOF; now_cycles 为 DWT->CYCCNT。
void note_sof(std::uint32_t frame, std::uint32_t now_cycles);

// 主循环: 重算拟合。每轮调用都足够便宜; 仅每 kFitPeriodMs 才做实际工作。
void poll(std::uint32_t tick_ms);

// 应用主机锚点。解析出的回绕不变时幂等; 已有效的时间线上回绕一旦改变按故障
// 处理而非静默接受: 那只可能是计数器或主机估计挪动了超过一秒。
void apply_anchor(std::uint64_t host_microframe);

Snapshot snapshot();

// 同 snapshot(), 但消费 residual 累加器。只有一个调用方 -- 发送周期状态的那个
// -- 使每个均值覆盖的窗口有明确定义。
Snapshot report();

// 时间线查询。本地单位是 TIM5 的 quarter-us -- timer::Timer::timepoint() 的
// 返回值, 也是本板 IMU / 带时间戳 GPIO 记录已带的单位 -- 因此 microframe_at()
// 能把现成遥测时间戳直接换到共享轴上。时间线无效时两者返回 false, 调用方不会
// 误调度到死时钟上。
bool local_time_of(std::uint64_t microframe, std::uint32_t& out_quarter_us);
bool microframe_at(std::uint32_t quarter_us, std::uint64_t& out_microframe);
// 同 microframe_at(), 另给出微帧内的小数(1/65536)。回答 kTimeAnchor 用这个:
// 只报整数微帧等于把"此刻"量化到 125 us 的栅格上, 主机拿它与往返时刻配对时,
// 这 0..125 us 的均匀误差原样进入主机一侧的估计。
bool microframe_at(
    std::uint32_t quarter_us, std::uint64_t& out_microframe, std::uint16_t& out_fraction_q16);

} // namespace libhcs::firmware::sync::timebase
