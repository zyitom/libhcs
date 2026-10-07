#pragma once

#include <atomic>
#include <cstdint>

#include <main.h>
#include <tim.h>

#include "core/src/utility/assert.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"

namespace libhcs::firmware::led {

class Led {
public:
    Led() {
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) == HAL_OK);
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) == HAL_OK);
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) == HAL_OK);
        reset();
    }

    void reset() {
        uplink_full_reset_counter_.store(0, std::memory_order::relaxed);
        downlink_full_reset_counter_.store(0, std::memory_order::relaxed);
        user_controlling_.store(false, std::memory_order::relaxed);
    }

    // 灯效持续 5 秒
    void uplink_buffer_full() {
        uplink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }
    void downlink_buffer_full() {
        downlink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }

    void update(uint32_t tick) {
        // 用户控制时不改灯效
        if (user_controlling_.load(std::memory_order::relaxed))
            return;

        // 原子递减计数器
        uint16_t uplink_full;
        do {
            uplink_full = uplink_full_reset_counter_.load(std::memory_order::relaxed);
            if (uplink_full == 0)
                break;
        } while (!uplink_full_reset_counter_.compare_exchange_weak(
            uplink_full, uplink_full - 1, std::memory_order::relaxed));

        uint16_t downlink_full;
        do {
            downlink_full = downlink_full_reset_counter_.load(std::memory_order::relaxed);
            if (downlink_full == 0)
                break;
        } while (!downlink_full_reset_counter_.compare_exchange_weak(
            downlink_full, downlink_full - 1, std::memory_order::relaxed));

        if (uplink_full && downlink_full) {
            // 双满: 黄与青交替闪
            if (tick & 128)
                set_value(255, 255, 0);
            else
                set_value(0, 255, 255);
        } else if (uplink_full) {
            // 上行满: 黄灯闪
            if (tick & 128)
                set_value(255, 255, 0);
            else
                set_value(0, 0, 0);
        } else if (downlink_full) {
            // 下行满: 青灯闪
            if (tick & 128)
                set_value(0, 0, 0);
            else
                set_value(0, 255, 255);
        } else if (host_connected_.load(std::memory_order::relaxed)) {
            // 主机会话已建立(nonce 握手完成、保活租约在身): 常亮绿即数据确实
            // 在转发。
            set_value(0, 255, 0);
        } else {
            // 活着但还没有主机会话: 绿灯呼吸。枚举完成而无会话显示"等待", 不是
            // "工作"。
            auto brightness = (tick >> 2) & 511;
            if (brightness > 255)
                brightness = 511 - brightness;
            set_value(0, static_cast<uint8_t>(brightness), 0);
        }
    }

    void set_host_connected(bool connected) {
        host_connected_.store(connected, std::memory_order::relaxed);
    }

    // 非静态, 确保实例化
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    void set_value(uint8_t red, uint8_t green, uint8_t blue) {
        htim5.Instance->CCR1 = blue;
        htim5.Instance->CCR2 = green;
        htim5.Instance->CCR3 = red;
    }

private:
    std::atomic<bool> user_controlling_;
    std::atomic<bool> host_connected_{false};

    std::atomic<uint16_t> uplink_full_reset_counter_, downlink_full_reset_counter_;
};

inline constinit utility::Lazy<Led> led;

} // namespace libhcs::firmware::led
