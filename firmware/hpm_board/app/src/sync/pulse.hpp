#pragma once

// 用现有 UART0 走线直接测量跨板偏差: GPTMR 硬件比较输出与捕获, 收发两个
// 瞬间都不经过 CPU。
//
// 为何存在: 此前的测量都经 CAN 时间戳单元, 其时基是 PTPC。PTPC 挂在 PLL0
// 下, HPM5300 手册称其为 fractional-N 且支持扩频, 瞬时速率漂移达数百 ppm --
// 从 452 us 到 30 us 的每个错误数字都源于这段漂移, 而非时间线或板子本身。
//
// GPTMR 属于 CLK_SRC_GROUP_COMMON, 可直接用 24 MHz 晶振驱动 -- 正是机器
// 定时器六分频前的同一信号源。无 PLL、无小数分频、无扩频, 速率与晶振同样
// 稳定。
//
// 注意它不是精确的每 microframe 3000 tick。microframe 轴来自 USB 主机时钟
// (SOF), GPTMR 计数器来自本板晶振, 是两颗独立振荡器 -- 实测相差约 +80 ppm,
// 即 ~3000.25 tick/microframe。按 3000 假定会在 50 ms 提前量里引入 4 us
// 误差, 故比值在此拟合, 与 timebase.cpp 对机器定时器的做法相同("两者都
// 出自晶振"的假设不成立, 只有一方如此)。
//
// 布线: 无需新增。UART0 本就跨板互连(A.TXD<->B.RXD 双向), 这些引脚以 ALT1
// 复用为 GPTMR0 通道 1 的比较输出与捕获输入:
//
//   PB08  UART0 TXD  /  GPTMR0_COMP_1   -- 脉冲出
//   PB09  UART0 RXD  /  GPTMR0_CAPT_1   -- 脉冲入
//
// 于是现有线缆成为双向硬件定时链路。开启本选项期间 UART0 不可用, 故单列
// 编译选项。
//
// 测量方法: 各板在约定的 microframe 发脉冲并捕获对方的。A 板脉冲于
// microframe a_tx 发出、在 b_rx 被捕获; B 板则为 b_tx 与 a_rx。于是
//
//     skew = ((b_rx - a_tx) - (a_rx - b_tx)) / 2   -- 双向差分取半
//
// 线缆传播与焊盘/同步器延迟在两个方向符号相同, 在差分中相消 -- 标准的
// 双向交换法。剩下的正是两板各自认定的 microframe k 时刻之差, 也就是共享
// 轴上的定时动作将继承的量。
//
// 分辨率为一个 GPTMR tick, 41.7 ns, 对时间线瞄准的 ~42 ns 而言偏粗, 但无害:
// 量化误差均值为零且被真实抖动扰动, 上千次交换后均值收敛到 1 ns 以下, 分布
// 宽度只增加正交项(42 -> 44 ns), 且该项可被扣除。

#include <cstdint>

namespace libhcs::firmware::sync::pulse {

#if defined(libhcs_APP_PULSE_TEST) && libhcs_APP_PULSE_TEST

inline constexpr bool kEnabled = true;

// 标称每 microframe tick 数: 24 MHz GPTMR, microframe 125 us。只作拟合
// 初值与其周界的合理性界限, 绝非最终使用的值。
inline constexpr std::uint32_t kNominalTicksPerMicroframe = 3000;

// 与 timebase.cpp 的机器定时器拟合同构, 理由相同: 基线(128 x 64
// microframe = 1.024 s)决定斜率精度, 样本数负责把 SOF 中断的进入抖动平均
// 下去。若只凭单个 SOF 样本排程(初版做法), 该样本 +-0.5 us 的抖动会直接
// 进脉冲, 是被测效应的十倍。
inline constexpr std::uint32_t kSampleDecimation = 64;
inline constexpr std::uint32_t kSampleCount = 128;

// 引脚复用、GPTMR 时钟、通道配置。必须在 UART 驱动之后运行: 本模块刻意
// 覆盖的正是后者的引脚复用。
void init();

// SOF 中断: 记录本 microframe 处 GPTMR 计数器的值。
void note_sof(std::uint64_t microframe);

// 主循环 1 kHz tick: 重算 microframe 到 tick 的直线。仅在重拟合周期内做
// 实事。
void poll(std::uint32_t tick_ms);

// 为给定绝对 microframe 布防硬件脉冲。拟合未就绪, 或目标太近(比较寄存器
// 必须在计数器越过之前写入)、太远不可信时返回 false。
bool schedule(std::uint64_t microframe);

// 主循环: 取回一条已捕获脉冲, 换算成 Q16 microframe 的共享轴坐标。无新
// 到数据时返回 false。
bool take_capture(std::uint64_t& microframe_q16);

// 捕获中断入口(在 .cpp 中绑定到 IRQn_GPTMR0)。
void isr_handler();

// 拟合出的每 microframe tick 数, Q16。发布出来供实机核验时钟关系而非
// 盲信。期望 ~196624000(即 3000.25), 而非 3000<<16: 超出部分是本板晶振
// 相对主机 USB 时钟偏快, 且应与机器定时器拟合报出的 +80 ppm 相符。
std::uint32_t measured_ticks_per_microframe_q16();

#else

inline constexpr bool kEnabled = false;

inline void init() {}
inline void note_sof(std::uint64_t) {}
inline void poll(std::uint32_t) {}
inline bool schedule(std::uint64_t) { return false; }
inline bool take_capture(std::uint64_t&) { return false; }
inline std::uint32_t measured_ticks_per_microframe_q16() { return 0; }

#endif

} // namespace libhcs::firmware::sync::pulse
