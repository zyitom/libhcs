#pragma once

#include <cstdint>

#include <gpio.h>
#include <main.h>

#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

// GPIO 输入上的消抖按键。
//
// 与 buzzer.hpp 相同的两段式: mc02 binding 分隔线以上不涉及本板任何引脚, 可原样
// 拷进其他 STM32 工程; 底部 binding 是唯一知道 PA15 的部分。
//
// 构造不做任何事, start() 只锁存当前电平 -- 不配置引脚: 按键输入正是
// MX_GPIO_Init() 已按 .ioc 配好的东西, 在此重配会成为上拉方向的第二个事实来源。
//
// mc02 app 目前没有调用方。刻意保留未启动: 驱动是可复用件, 长按该做什么由板
// 自己决定。

namespace libhcs::firmware::key {

enum class Event : uint8_t {
    kNone = 0,
    // 边沿事件, 在观察到的那次 poll() 上各报告一次。
    kPressed = 1,
    kReleased = 2,
    // 每次按下只触发一次: kPressed 之后 long_press_ms 且仍按住时。触发过它的
    // 长按, 其后仍会报告 kReleased。
    kLongPressed = 3,
};

struct Config {
    GPIO_TypeDef* port;
    uint16_t pin;
    // 按键未按下时引脚所处电平。接地上拉、悬空时读 GPIO_PIN_SET。
    GPIO_PinState released_state;
    // 电平须稳定这么久才算数。20 ms 覆盖所有值得一用的轻触开关的触点抖动; 再加大
    // 只增加延迟。
    uint16_t debounce_ms;
    uint16_t long_press_ms;
};

class Key : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Key>;

    static constexpr uint16_t kDefaultDebounceMs = 20;
    static constexpr uint16_t kDefaultLongPressMs = 2000;

    Key() = default;

    // 锁存当前电平作为起点, 使按住按键上电的板子在首次 poll() 不报出虚假的
    // kPressed。刻意不碰 GPIO 配置; 见文件头。
    void start(const Config& config) {
        config_ = config;
        stable_pressed_ = raw_pressed();
        candidate_pressed_ = stable_pressed_;
        last_change_tick_ = HAL_GetTick();
        long_press_fired_ = stable_pressed_;
        started_ = true;
    }

    [[nodiscard]] bool started() const { return started_; }
    [[nodiscard]] bool pressed() const { return stable_pressed_; }

    // 主循环调用。每次调用至多返回一个事件; 既长按又释放的按下会在某次 poll 报
    // kLongPressed、之后的某次报 kReleased, 绝不同时。
    Event poll() {
        if (!started_)
            return Event::kNone;

        const uint32_t tick = HAL_GetTick();
        const bool raw = raw_pressed();

        if (raw != candidate_pressed_) {
            candidate_pressed_ = raw;
            last_change_tick_ = tick;
            return Event::kNone;
        }

        if (candidate_pressed_ != stable_pressed_) {
            if (tick - last_change_tick_ < config_.debounce_ms)
                return Event::kNone;
            stable_pressed_ = candidate_pressed_;
            // 保持计时从消抖后的边沿重起, 而非原始边沿, 使 long_press_ms 度量的是
            // 调用方被告知的那次按压。
            last_change_tick_ = tick;
            if (stable_pressed_) {
                long_press_fired_ = false;
                return Event::kPressed;
            }
            return Event::kReleased;
        }

        if (stable_pressed_ && !long_press_fired_
            && tick - last_change_tick_ >= config_.long_press_ms) {
            long_press_fired_ = true;
            return Event::kLongPressed;
        }

        return Event::kNone;
    }

private:
    [[nodiscard]] bool raw_pressed() const {
        return HAL_GPIO_ReadPin(config_.port, config_.pin) != config_.released_state;
    }

    Config config_{};
    uint32_t last_change_tick_ = 0;
    bool stable_pressed_ = false;
    bool candidate_pressed_ = false;
    bool long_press_fired_ = false;
    bool started_ = false;
};

// ------- mc02 binding: 本线以上与板无关 -------
//
// PA15, 上拉, 按下接地 -- 悬空读高。bootloader 正依赖同一极性决定是否留在 DFU
// (bootloader/src/main.cpp: 复位时 KEY 为低即停留)。
//
// PA15 也是 JTDI, 但 SWD 调试口只用 SWDIO/SWCLK, 且 .ioc 已把该引脚认领为
// GPIO_Input, 与调试器无冲突。
//
// 以函数返回而非 constinit 对象持有: GPIOA 宏展开为整数转型指针, 不是常量
// 表达式 -- power.hpp 出于同样原因在函数体内读端口宏而非存储它们。
[[nodiscard]] inline Config mc02_config() {
    return Config{
        .port = KEY_GPIO_Port,
        .pin = KEY_Pin,
        .released_state = GPIO_PIN_SET,
        .debounce_ms = Key::kDefaultDebounceMs,
        .long_press_ms = Key::kDefaultLongPressMs,
    };
}

inline constinit Key::Lazy key;

} // namespace libhcs::firmware::key
