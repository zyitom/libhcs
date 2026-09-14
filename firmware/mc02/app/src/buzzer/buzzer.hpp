#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <main.h>
#include <tim.h>

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

// 定时器 PWM 通道上的无源蜂鸣器驱动。
//
// 文件分两段, 且有意如此划分:
//
//   1. mc02 binding 分隔线以上与板无关: 不涉及本板任何句柄、引脚、时钟 -- 定时
//      器、通道、计数速率都作为构造参数传入。拷进其他 STM32 工程无需改动。
//   2. 底部 binding 是唯一 mc02 专属部分: 哪个定时器、哪个通道、计数多快。其他
//      板只改这三个值, 别的不动。
//
// 构造刻意惰性 -- 不碰任何寄存器。在有人调用 start() 之前不发声、不使能定时器
// 通道。只想让驱动在场的工程可以构造而从不启动; 还没跑 MX_TIMx_Init() 的工程
// 不得启动。与 power.hpp 同一纪律、同一理由: 硬件上电行为应由板决定, 而非构造
// 函数运行的副作用。

namespace libhcs::firmware::buzzer {

// 十二平均律音高, 使旋律可用音名而非裸赫兹书写。
//
// 本板外设所参照的厂商 BSP 把这些写成约 130 个 extern const float
// BUZZER_FREQUENCY_* 全局量, 散在一个 .cpp 里 -- 约半 KB 的 float 常量, extern
// 导致无法在使用点折叠、只能运行时加载。改用一个八度加移位即可精确: 八度按
// 定义是 2 的幂, C6 即 C4 << 2 无误差, 其余音高全部在编译期折叠成立即数。
enum class Pitch : uint8_t {
    kC = 0,
    kCSharp = 1,
    kD = 2,
    kDSharp = 3,
    kE = 4,
    kF = 5,
    kFSharp = 6,
    kG = 7,
    kGSharp = 8,
    kA = 9,
    kASharp = 10,
    kB = 11,
};

// 科学音高记谱的第 4 八度(A4 = 440 Hz), 取整到整赫兹。此处舍入误差至多 0.5 Hz,
// 随八度翻倍, 即使 A7 距真值也在 4 Hz 内 -- 远低于压电换能器的音高分辨力, 也
// 低于 1 MHz 计数器本可表达的极限。
inline constexpr uint16_t kOctave4Hz[]{262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};

[[nodiscard]] constexpr uint16_t pitch(Pitch semitone, unsigned octave) {
    const uint32_t base = kOctave4Hz[static_cast<unsigned>(semitone)];
    const uint32_t scaled = octave >= 4 ? base << (octave - 4) : base >> (4 - octave);
    return static_cast<uint16_t>(scaled);
}

// 旋律的一步。频率 0 为休止 -- 通道在时长内静音而非发声。
struct Note {
    uint16_t frequency_hz;
    uint16_t duration_ms;
};

class Buzzer : private core::utility::Immovable {
public:
    // 音量即占空比, 无源蜂鸣器在 50% 最响: 超过后脉冲只是变宽, 并不增加基频处
    // 的驱动能量。故 kLoudnessMax 对应 50% 而非 100% 占空比。
    static constexpr uint8_t kLoudnessMax = 255;
    static constexpr uint8_t kDefaultLoudness = 128;

    // 低于此换能器听不见, 且 1 MHz tick 下重装值本会溢出 16 位计数器
    // (1 MHz / 16 Hz > 65535, 确实装不下)。
    static constexpr uint16_t kMinFrequencyHz = 100;

    // tick_hz 是预分频后的定时器计数速率, 不是内核时钟: 调用方已在 CubeMX 选好
    // 预分频, 这里的每个音符只是 ARR = tick_hz / f - 1。作为参数传入而非从 PSC
    // 与 RCC 树推导, 使这半边文件不掺任何时钟树知识 -- 这正是它可移植的原因。
    constexpr Buzzer(TIM_HandleTypeDef* timer, uint32_t channel, uint32_t tick_hz)
        : timer_(timer)
        , channel_(channel)
        , tick_hz_(tick_hz) {}

    // 使能 PWM 通道, 静音。必须在板的 MX_TIMx_Init() 之后运行 -- 把引脚放进
    // 复用功能的正是它。
    void start() {
        core::utility::assert_always(HAL_TIM_PWM_Start(timer_, channel_) == HAL_OK);
        mute();
        started_ = true;
    }

    [[nodiscard]] bool started() const { return started_; }

    // 发声一个音, 直到下次调用。立即生效: 下面的 update 事件重载影子 ARR, 否则
    // TIM_AUTORELOAD_PRELOAD_ENABLE 会把它扣到当前周期结束 -- 降频时听感为一段
    // 旧音, 最长可达 ARR 微秒。
    void set_tone(uint16_t frequency_hz, uint8_t loudness) {
        if (frequency_hz < kMinFrequencyHz || loudness == 0) {
            mute();
            return;
        }

        const uint32_t reload = tick_hz_ / frequency_hz;
        // reload 的一半即 50% 上限, 故除数是 2 * kLoudnessMax 而非 kLoudnessMax。
        const uint32_t compare = (reload * loudness) / (2U * kLoudnessMax);

        __HAL_TIM_SET_AUTORELOAD(timer_, reload - 1);
        __HAL_TIM_SET_COMPARE(timer_, channel_, compare);
        __HAL_TIM_SET_COUNTER(timer_, 0);
        timer_->Instance->EGR = TIM_EGR_UG;
    }

    void set_tone(Pitch semitone, unsigned octave, uint8_t loudness = kDefaultLoudness) {
        set_tone(pitch(semitone, octave), loudness);
    }

    // 静音但不停定时器。不动 ARR: 下次 set_tone() 反正会重写; 保持通道运行, 旋律
    // 音符间才没有 PWM 重启毛刺。
    void mute() { __HAL_TIM_SET_COMPARE(timer_, channel_, 0); }

    // 开始旋律并返回, 由 poll() 推进。notes 在播放期间按引用持有, 必须有静态
    // 存储期 -- 用下面 kBootMelody 式常量, 而非局部数组。
    void play(std::span<const Note> notes, uint8_t loudness = kDefaultLoudness) {
        if (notes.empty()) {
            stop();
            return;
        }

        sequence_ = notes;
        loudness_ = loudness;
        index_ = 0;
        note_started_tick_ = HAL_GetTick();
        set_tone(sequence_[0].frequency_hz, loudness_);
    }

    void stop() {
        sequence_ = {};
        mute();
    }

    [[nodiscard]] bool playing() const { return !sequence_.empty(); }

    // 主循环轮询。空闲时开销为一次 load 加一个分支, 因此可以无条件放进转发循环;
    // LED 依 HAL_GetTick 节流的理由在此同样适用, 且旋律本以毫秒计量 -- 按循环
    // 迭代计数会在一毫秒内跑完整首旋律。
    void poll() {
        if (sequence_.empty())
            return;

        const uint32_t tick = HAL_GetTick();
        if (tick - note_started_tick_ < sequence_[index_].duration_ms)
            return;

        note_started_tick_ = tick;
        if (++index_ >= sequence_.size()) {
            stop();
            return;
        }
        set_tone(sequence_[index_].frequency_hz, loudness_);
    }

private:
    TIM_HandleTypeDef* timer_;
    uint32_t channel_;
    uint32_t tick_hz_;

    std::span<const Note> sequence_;
    std::size_t index_ = 0;
    uint32_t note_started_tick_ = 0;
    uint8_t loudness_ = kDefaultLoudness;
    bool started_ = false;
};

// 上电短促上行啁啾。作用是在没有主机、也看不到 LED 时宣告固件已到达主循环:
// 这是 WS2812 与 USB 链路都无法独立报告的事 -- bring-up 中途 fault 的板与断电
// 的板看起来一样。
inline constexpr Note kBootMelody[]{
    {.frequency_hz = pitch(Pitch::kC, 6), .duration_ms = 60},
    {.frequency_hz = pitch(Pitch::kE, 6), .duration_ms = 60},
    {.frequency_hz = pitch(Pitch::kG, 6), .duration_ms = 90},
};

// 双音交替, 用于操作者注意力在别处也须察觉的状况。
inline constexpr Note kAlarmMelody[]{
    {.frequency_hz = pitch(Pitch::kA, 5), .duration_ms = 150},
    {.frequency_hz = 0, .duration_ms = 50},
    {.frequency_hz = pitch(Pitch::kA, 5), .duration_ms = 150},
    {.frequency_hz = 0, .duration_ms = 50},
    {.frequency_hz = pitch(Pitch::kA, 5), .duration_ms = 150},
};

// ------- mc02 binding: 本线以上与板无关 -------
//
// PB15, TIM12 CH2。mc02_slave.ioc 给 TIM12 Prescaler = 274、Period = 249, APB1
// 定时器内核跑 275 MHz(APB1 为 137.5 MHz, D2PPRE1 = DIV2 使定时器时钟倍频), 故
// 计数器恰以 275 MHz / 275 = 1 MHz 走 -- 即下面传入的值。生成代码的默认参数恰好
// 发出 1 MHz / 250 = 4 kHz。
//
// 本文件出现之前无人调用 MX_TIM12_Init(): 定时器在 .ioc 里但从未启动, PB15 保持
// 未配置 GPIO, 蜂鸣器哑。该调用现位于 App() 中其他 MX_TIM*_Init 旁边。
//
// 仅限主循环: play() 与 poll() 不碰原子量、绝不在 ISR 中调用; 不像 LED, 其
// buffer-full 计数器会从中断上下文置位。
inline constexpr uint32_t kMc02TimerTickHz = 1000000;

inline constinit utility::Lazy<Buzzer, TIM_HandleTypeDef*, uint32_t, uint32_t> buzzer{
    &htim12, TIM_CHANNEL_2, kMc02TimerTickHz};

} // namespace libhcs::firmware::buzzer
