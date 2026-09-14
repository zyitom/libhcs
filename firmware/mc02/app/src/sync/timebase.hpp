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
// 移植自 firmware/hpm_board/app/src/sync/timebase.{hpp,cpp}, 去掉 HPM 专属部分
// (PTPC 捕获、硬件 SOF->PTPC 触发路由、CAN TSU 换算、GPTMR 脉冲层)。针对本芯片
// 有三处改动, 也正是错误假设最容易藏身之处, 故在此说明而非留在 diff 里:
//
//  1. 仅全速, 且帧计数器数的是帧。mc02 无 ULPI PHY: USB_OTG_HS 走内置全速 PHY
//     (12 Mbit)。DWC2 的 DSTS.FNSOF 在高速下是微帧号、全速下是帧号, 故每 1 ms
//     一次 SOF 中断、每次 +1; hpm_board 的 EHCI FRINDEX 数微帧、步进 8。乘 8 后
//     两板落在同一微帧轴、同一 16384 模数上 -- 见 .cpp 的 frame_scale()。
//
//  2. 本地时钟是周期计数器, 不是协议的 quarter-us tick。mc02 的 timer::Timer 是
//     TIM5 预分频到 1 MHz、按 CNT << 2 上报, quarter-us 值以 4 为步进: 1 us 量化,
//     与本模块要测的中断抖动同量级。拿与被测量同粗的尺子采样, 结果无法解读, 故
//     拟合在 DWT->CYCCNT(550 MHz, 1.8 ns)上做, 仅在协议要求处换算成 quarter-us。
//     关键方向的换算是精确的: 68750 cycles/microframe 恰为 500 quarter-us, 主机
//     按标称 500 的算法无需改动。
//
//  3. 本芯片存在硬件 SOF 捕获, 但这里刻意不用(模块最初误以为它不存在; 实测
//     记录在案, 以免重查)。实测 2026-09-07: 把 TIM2 的 SMCR.TS 在 ITR0..ITR13
//     间扫描、CH2 映射到 TRC, 仅 ITR5 以 SOF 速率产生捕获, 其余来源读数恰为 0;
//     捕获间隔下限 999 个 TIM2 tick(1 MHz), 即 1 ms 全速帧。USB1_OTG_HS_SOF ->
//     TIM2 ITR5 属实, 是 HPM TRGM->PTPC 路由的直接对应物。TIM5 无此来源: 其
//     ITR5 是 H723 没有的 USB2_OTG_FS, 且 TIM5 全扫描找不到 SOF 速率的信号。
//     (注意这条路径无法靠 grep HAL 得到: STM32F4 有 TIM_TIM2_USBFS_SOF 命名,
//     H7 HAL 只暴露 TIM_TS_ITR0..13, 每个定时器的含义在 RM0468 的内部触发表。)
//
//     若要接线, 先把 TIM2 预分频设为 0。TIM2 现为 1 MHz, 供 PA0/PA2 的 50 Hz
//     舵机 PWM(gpio.hpp); 把捕获锁进 1 MHz 计数器比现有软件路径更差:
//
//       TIM2 @ 275 MHz 捕获   单样本 sigma ~0.001 us  (3.6 ns / sqrt(12))
//       本软件 ISR 路径       单样本 sigma  0.1365 us [实测 2026-09-07]
//       TIM2 @ 1 MHz 捕获     单样本 sigma  0.289 us  (1 us / sqrt(12))
//
//     硬件捕获在边沿上精确, 但上限取决于被锁存的计数器: 精确边沿锁进粗计数器
//     等于丢掉精确性。HPM 的对应方案可行, 正因为它锁进 1.04 ns 粒度的 PTPC。
//
//     中断路径也远好于其最差样本: 最差 ~2.5 us 是 19 sigma 的偶发抢占, 128 样本
//     最小二乘会把它埋掉。入口延迟的均值根本不是误差——那是拟合偏移吸收的常数,
//     且在跑同一代码的两板间完全抵消。
//
//     预分频 0 与舵机输出兼容: TIM2 是 32 位定时器, 50 Hz 需 ARR 5499999, 装得
//     下。这是 .ioc 改动, 不在此处做。
//
// 何时失效: 微帧增量不是期望步长。差几步 = 漏了几次中断: 计数器仍可修正(它按
// 真实 delta 前进), 但拟合窗口作废, 缺口两侧样本会使直线倾斜。delta 为 0、或
// 8 步及以上无法解释, 则计数器本身存疑: 时间线失效, 新锚点到来前不得对其调度。
//
// libhcs_APP_TIME_SYNC 未定义时整体编译剔除; 启用会给板子增加一个 1 kHz 中断。

#include <cstdint>

#include "core/include/libhcs/data/datas.hpp"

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

#if defined(libhcs_APP_TIME_SYNC) && libhcs_APP_TIME_SYNC

inline constexpr bool kEnabled = true;

// 每 64 微帧取一个拟合样本, 128 样本环形窗口。全速下计数器每 SOF 前进 8 微帧,
// 即每第 8 次中断取一个样本: 间隔 8 ms、基线 1.024 s、512 字节 RAM -- 与
// hpm_board 高速下的间距和窗口一致, 两板的 residual 数值才可直接比较。
//
// 基线决定斜率精度(端点噪声除以窗口长), 样本数把相位噪声平均下去。窗口还须避开
// 550 MHz 下周期计数器 7.81 s 的回绕, 1.024 s 对此有 7 倍余量。
inline constexpr std::uint32_t kSampleDecimation = 64U;
inline constexpr std::uint32_t kSampleCount = 128U;

struct Snapshot {
    data::TimeState state;
    // kValid 时为绝对微帧; 其余情况为本板自身原点。
    std::uint64_t microframe;
    std::uint64_t timestamp_quarter_us;
    // 拟合出的每微帧本地 tick 数, Q16, 单位为协议的 quarter-us(输出时由 cycles
    // 换算)。拟合收敛前为 0。
    std::uint32_t ticks_per_microframe_q16;
    std::uint32_t anomaly_count;
    // 自上次 report() 以来本板拟合的样本外预测误差, Q16 quarter-us。跨板偏差的
    // 主体是均值; 极值为何不算见 data::TimeStatusView。
    std::int32_t residual_mean_q16;
    std::uint32_t residual_abs_max_q16;
    std::uint32_t residual_count;
};

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

#else

inline constexpr bool kEnabled = false;

// 必须逐字段镜像启用版的 Snapshot: 本头文件之外的调用方会无条件读取这些成员,
// stub 落后只在 TIME_SYNC=OFF 构建中出错 -- 恰是时间基开发期间最不易编到的配置。
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
inline std::uint32_t microframes_per_sof() { return 8; }
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
inline bool local_time_of(std::uint64_t, std::uint32_t&) { return false; }
inline bool microframe_at(std::uint32_t, std::uint64_t&) { return false; }

#endif

} // namespace libhcs::firmware::sync::timebase
