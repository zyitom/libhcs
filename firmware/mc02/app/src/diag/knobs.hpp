#pragma once

#include <cstdint>

// 供调试器读写的开关与只读镜像, 用于在没有上位机、不改协议、不重编译的前提下
// 对着真实硬件反复调外设。Ozone 的 Watched Data 窗口能在程序运行中刷新并接受
// 写入(Cortex-M7 后台内存访问), 调试会话由此变成一块控制面板: 输入一个频率就能听见。
//
// 为何用镜像而不直接观察对象: 原始采样在 libhcs::firmware::adc::mc02_samples
// 本就可见, 但每次想看电池电压都得手工对 16 个半字求平均再乘分压比; millivolts()
// 则无法被观察 -- Ozone 只对符号表达式求值, 不会调用目标机函数, 想看到计算值
// 就只能由目标机把它算进一个变量, battery_mv 即为此而生。
//
// 为何写 tone_hz 好过在调试器里改 TIM12->ARR(后者本就无需固件支持): 改寄存器
// 只测到发声器本身, 而这里写入与应用走同一个 Buzzer::set_tone(), 重装载运算与
// 占空比映射也一并受测。
//
// 默认整体编译剔除。开启后每轮主循环的开销为一次 HAL_GetTick() 读加一次比较,
// 其余工作按 kPollIntervalMs 节流。该上限是刻意的: 本板下行包速率对主循环周期
// 高度敏感且非单调(见 firmware/mc02/AGENTS.md), 无条件的每轮工作带来的偏移
// 会超过多数基准想要测量的效应本身。

namespace libhcs::firmware::diag::knobs {

#if defined(libhcs_APP_DEBUG_KNOBS) && libhcs_APP_DEBUG_KNOBS

inline constexpr bool kEnabled = true;

// 写入即发声, 0 静音; 低于 Buzzer::kMinFrequencyHz(100 Hz)同样静音,
// 与交给 set_tone() 处理的结果一致。
inline volatile uint16_t tone_hz = 0;
// 0..255。255 对应 50% 占空比, 即无源蜂鸣器的最响档。
inline volatile uint8_t tone_loudness = 128;

// tone_hz 的音名替代, 让音高可以按音高本身输入。tone_semitone 取 buzzer::Pitch
// 值(0 = C, 1 = C#, ... 11 = B), tone_octave 取 scientific pitch 八度,
// 如 {9, 4} 即 A4 = 440 Hz。与 kBootMelody 常量同样走 buzzer::pitch(),
// 重点在于八度换算也一并受测, 而不只是 PWM。
//
// 两个开关不能同时驱动通道, 由 tone_semitone 仲裁: 缺省 kSemitoneOff 时把控制权
// 交还 tone_hz, 从不触碰这些变量的构建行为与从前完全一致。越界值静音而非出错 --
// 调试器写入这些字节时自身不做任何校验, 否则 pitch() 会越界索引 kOctave4Hz[],
// 或移位超出其 uint16_t 返回值能表示的范围。
inline constexpr int8_t kSemitoneOff = -1;
inline constexpr int8_t kSemitoneMax = 11;

// 超过此八度, 1 MHz 计数器已无法有效表达音高: B9 为 15808 Hz, 重装载值仅 63,
// 再高一个八度则溢出 pitch()。
inline constexpr uint8_t kMaxOctave = 9;

inline volatile int8_t tone_semitone = kSemitoneOff;
inline volatile uint8_t tone_octave = 5;

// 电量计的只读镜像, 按 kPollIntervalMs 刷新。
inline volatile uint32_t battery_raw = 0;
inline volatile uint32_t battery_mv = 0;
inline volatile uint8_t battery_started = 0;

void poll();

#else

inline constexpr bool kEnabled = false;

inline void poll() {}

#endif

} // namespace libhcs::firmware::diag::knobs
