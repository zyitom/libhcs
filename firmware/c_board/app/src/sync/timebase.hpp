#pragma once

// 基于 USB 微帧计数器的 c_board 跨板共享时间基准。
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
// 本模块最初移植自 firmware/mc02/app/src/sync/timebase.{hpp,cpp}(其又移植自 hpm_board),
// 针对本芯片的差异有三处, 也正是错误假设最容易藏身之处,
// 故在此说明而非留在 diff 里:
//
//  1. 恒全速, 且帧计数器数的是帧。OTG_FS 没有高速 PHY, 这一点与 mc02 的
//     "无 ULPI 故只能全速"同为全速, 但本板是控制器本身的属性 -- DSTS.ENUMSPD
//     在 OTG_FS 上恒为 0b11(内置 48 MHz PHY), 不存在高速分支可走。保留高速
//     分支的代码形状(frame_scale() 读速度位), 是免得此代码被复用到带高速
//     PHY 的芯片上时悄悄错一个 8 倍: 每中断 +1 的帧号乘 8 才落上 16384 微帧
//     (2.048 s)的共享轴。微帧轴的形态与全仓库所有板一致, 见 .cpp 的注释。
//
//  2. 本地时钟是周期计数器, 不是协议的 quarter-us tick。本板的 timer::Timer
//     是 TIM2+TIM9(4 MHz, 直接按 quarter-us 上报, 量化 250 ns), 比 mc02 的
//     TIM23(1 MHz, CNT<<2, 量化 1 us)细, 但与被测的中断抖动仍是同量级。
//     拿与被测量同粗的尺子采样, 结果无法解读, 故拟合仍在 DWT->CYCCNT
//     (168 MHz, 5.95 ns)上做, 仅在协议要求处换算成 quarter-us。关键方向的
//     换算是精确的: 21000 cycles/microframe 恰为 500 quarter-us, 主机按标称
//     500 的算法无需改动(下方的 static_assert 断言而非口说)。
//
//     CYCCNT 须先使能(App 构造函数里 DEMCR.TRCENA + DWT_CTRL.CYCCNTENA, 与
//     mc02 同一位置同一写法); J-Link 挂接会停走 CYCCNT, 本模块随之失效到
//     复位为止 -- 测时间基期间不要连调试器, 读完寄存器先复位再测。
//     [实测 2026-09-16 mc02, 两次复现; M4 同一调试单元, 推断同病, 未上板。]
//
//  3. 硬件 SOF 捕获与 mc02 同一层"捕获修正": 时间戳是中断入口处的 CYCCNT 减去
//     边沿到入口的间隔。F407 上 OTG_FS 的 SOF 只引到 TIM2 的 ITR1, 而 TIM2 是全板
//     的 1/4 us 时间戳源, 250 ns 的捕获值太粗; 所以捕获只用来触发 DMA, 在边沿
//     那一刻抄下 84 MHz 的 TIM7(sync/sof.cpp 开头的注释)。捕获不可用(.ioc 改了
//     TIM7, 或 ITR1 上没有脉冲)时退回纯中断时间戳; mc02 的交替烧录 A/B 给了那种
//     形态的预期: 单样本 sigma 约 0.03 us, 最差单样本劣化到 1 us 量级。
//
//     与 CAN 帧时间戳那一侧(sof_capture)的关系: 本板没有那一侧。bxCAN 无
//     帧时间戳, 上行 CAN 帧不带 SofStamp; FDCAN 依托硬件时间戳的
//     SofCaptureRing 路径在本板整个不存在。kTimeStatus 里的 capture_fresh/
//     stale_count 只数 SOF 捕获。
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

// 每微秒 CPU 周期数 / 每微帧周期数。SYSCLK = 12 MHz HSE / 6 * 168 / 2 = 168 MHz,
// 精确值(main.c SystemClock_Config); DWT->CYCCNT 数 CPU 周期, 故这是精确换算而非
// 舍入。
inline constexpr std::uint32_t kCyclesPerMicrosecond = 168U;
inline constexpr std::uint32_t kNominalCyclesPerMicroframe = kCyclesPerMicrosecond * 125U;

// 21000 cycles 恰为 500 quarter-us: 协议标称值与 4 MHz 板一致, 主机无需按板缩放。
// 以 static_assert 断言而非口说。
static_assert(kNominalCyclesPerMicroframe * 4U % kCyclesPerMicrosecond == 0U);
static_assert(kNominalCyclesPerMicroframe * 4U / kCyclesPerMicrosecond == 500U);

// 每 64 微帧取一个拟合样本, 128 样本环形窗口。全速下计数器每 SOF 前进 8 微帧,
// 即每第 8 次中断取一个样本: 间隔 8 ms、基线 1.024 s、512 字节 RAM -- 与
// mc02 全速下及 hpm_board 高速下的间距和窗口一致, 各板的 residual 数值才可直接
// 比较。
//
// 基线决定斜率精度(端点噪声除以窗口长), 样本数把相位噪声平均下去。窗口还须避开
// 168 MHz 下周期计数器 25.6 s 的回绕, 1.024 s 对此有 25 倍余量。
inline constexpr std::uint32_t kSampleDecimation = core::time::kSampleDecimation;
inline constexpr std::uint32_t kSampleCount = core::time::kSampleCount;

// 单位是协议的 quarter-us(拟合在周期计数器上, 输出时换算); 字段说明见
// core::time::TimebaseSnapshot。
using Snapshot = core::time::TimebaseSnapshot;

// 当前端口速度下计数器每 SOF 中断前进的微帧数: 全速 8, 高速 1。本板恒 8; 保留
// 高速分支的理由见文件头第 1 条。微帧流的每个消费者都需要该步长, 故不只是本模块
// 内部使用。
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

// 时间线查询。本地单位是 timer::Timer 的 quarter-us -- 本板其余上行记录打戳
// 所用的单位 -- 因此 microframe_at() 能把现成遥测时间戳直接换到共享轴上。
// 时间线无效时两者返回 false, 调用方不会误调度到死时钟上。
bool local_time_of(std::uint64_t microframe, std::uint32_t& out_quarter_us);
bool microframe_at(std::uint32_t quarter_us, std::uint64_t& out_microframe);
// 同 microframe_at(), 另给出微帧内的小数(1/65536)。回答 kTimeAnchor 用这个:
// 只报整数微帧等于把"此刻"量化到 125 us 的栅格上, 主机拿它与往返时刻配对时,
// 这 0..125 us 的均匀误差原样进入主机一侧的估计。
bool microframe_at(
    std::uint32_t quarter_us, std::uint64_t& out_microframe, std::uint16_t& out_fraction_q16);

} // namespace libhcs::firmware::sync::timebase
