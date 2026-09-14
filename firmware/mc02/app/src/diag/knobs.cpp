#include "firmware/mc02/app/src/diag/knobs.hpp"

#if defined(libhcs_APP_DEBUG_KNOBS) && libhcs_APP_DEBUG_KNOBS

# include <main.h>

# include "firmware/mc02/app/src/adc/battery.hpp"
# include "firmware/mc02/app/src/buzzer/buzzer.hpp"

namespace libhcs::firmware::diag::knobs {

namespace {

// 快到输入的频率即刻出声, 又慢到每轮开销只有一次 tick 读。16 个 ADC 采样需
// 1.25 ms 才能补满, 因此也不会把同一个平均窗口上报两次。
constexpr uint32_t kPollIntervalMs = 20;

uint32_t last_poll_tick = 0;
uint16_t applied_tone_hz = 0;
uint8_t applied_loudness = 0;

// 把两种点音高的途径归一为 poll() 实际应用的频率。每个 volatile 只读一次:
// 调试器可能在同一开关的两次读取之间落一个写; 若在范围检查之后重读
// tone_semitone, 旧值就会绕过检查。
uint16_t requested_frequency() {
    const int8_t semitone = tone_semitone;
    if (semitone == kSemitoneOff)
        return tone_hz;
    const uint8_t octave = tone_octave;
    if (semitone < 0 || semitone > kSemitoneMax || octave > kMaxOctave)
        return 0;
    return buzzer::pitch(static_cast<buzzer::Pitch>(semitone), octave);
}

} // namespace

void poll() {
    const uint32_t tick = HAL_GetTick();
    if (tick - last_poll_tick < kPollIntervalMs)
        return;
    last_poll_tick = tick;

    battery_raw = adc::battery->raw();
    battery_mv = adc::battery->millivolts();
    battery_started = adc::battery->started() ? 1U : 0U;

    // 边沿触发: 若每 20 ms 重复施加同一个音, 会经 set_tone() 的 EGR 写重启计数器,
    // 音符听起来被切碎。
    const uint16_t requested_hz = requested_frequency();
    const uint8_t requested_loudness = tone_loudness;
    if (requested_hz == applied_tone_hz && requested_loudness == applied_loudness)
        return;
    applied_tone_hz = requested_hz;
    applied_loudness = requested_loudness;

    // 先 stop() 再 set_tone(): 仍在播放中的旋律占有通道, 否则它的下一次 poll()
    // 会在一个音符内覆盖新音。开机提示音长 210 ms, 不这样做则早期输入的频率
    // 会直接丢失。
    buzzer::buzzer->stop();
    buzzer::buzzer->set_tone(requested_hz, requested_loudness);
}

} // namespace libhcs::firmware::diag::knobs

#endif
