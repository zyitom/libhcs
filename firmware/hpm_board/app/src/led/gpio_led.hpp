#pragma once

#include <cstdint>

#include "board_app.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::led {

// 纯 GPIO RGB LED 后端, 用于直接拉高焊盘点亮(高有效)而非 WS2812 串行协议的
// 板。它与 Ws2812 暴露同一接口, 共享的 Led 驱动因此不关心底层硬件。没有 PWM
// 时通道只有开/关两态, 故各通道值按中点阈值化, WS2812 的亮度渐变退化为闪烁,
// 但闪烁型灯光语言得以保留。
class GpioLed : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<GpioLed>;

    static constexpr uint8_t kOnThreshold = 128;

    GpioLed() { board::init_led_pins(); }

    // 与有状态的 LED 后端接口对齐, 尽管 GPIO 实现并不需要实例数据。
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    bool set_value(uint8_t red, uint8_t green, uint8_t blue) {
        red_pin_.set_active(red >= kOnThreshold);
        green_pin_.set_active(green >= kOnThreshold);
        blue_pin_.set_active(blue >= kOnThreshold);
        return true;
    }

private:
    // 构造时一次解析, 而非每次 update 都查板级表: 服务多块 PCB 的板目录要按
    // 运行时身份选焊盘(见 boards/hpm5321/app/board_app.hpp), 且这里跑在
    // 1 kHz, 没有理由每 tick 重新解析三个引脚。GpioPin 是 8 字节 POD, 缓存
    // 三个共 24 字节。
    GpioPin red_pin_ = board::led_red_pin();
    GpioPin green_pin_ = board::led_green_pin();
    GpioPin blue_pin_ = board::led_blue_pin();
};

inline constinit GpioLed::Lazy gpio_led;

} // namespace libhcs::firmware::led
