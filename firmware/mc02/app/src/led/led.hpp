#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

#include <main.h>
#include <spi.h>

#include "firmware/mc02/app/src/utility/lazy.hpp"

namespace libhcs::firmware::led {

class Led {
public:
    Led() { reset(); }

    void reset() {
        uplink_full_reset_counter_.store(0, std::memory_order::relaxed);
        downlink_full_reset_counter_.store(0, std::memory_order::relaxed);
        user_controlling_.store(false, std::memory_order::relaxed);
    }

    void uplink_buffer_full() {
        uplink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }

    void downlink_buffer_full() {
        downlink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }

    // 主循环轮询, 但以 SysTick 节流, 使图案与主循环节奏无关。任何 ISR 上下文中
    // 都不发生 SPI 发送。
    //
    // tick 必须是毫秒计数而非循环迭代计数: run() 视构建选项与负载以 69-85 kHz
    // 旋转(libhcs_APP_LOOP_PROFILE 实测)。按迭代计数曾让每个图案快几十倍 --
    // buffer-full 闪烁与呼吸周期都远超闪烁融合频率, 读起来是恒定的半亮颜色而非
    // 动画。在此节流还使下面 5000 计数的复位窗口名副其实地跨 5 秒而非 13 ms,
    // 并避免 WS2812 帧每秒被重发数千次。
    void poll() {
        if (user_controlling_.load(std::memory_order::relaxed))
            return;

        const auto tick = HAL_GetTick();
        if (tick == last_tick_)
            return;
        last_tick_ = tick;

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
            if (tick & 128)
                set_value(255, 255, 0);
            else
                set_value(0, 255, 255);
        } else if (uplink_full) {
            if (tick & 128)
                set_value(255, 255, 0);
            else
                set_value(0, 0, 0);
        } else if (downlink_full) {
            if (tick & 128)
                set_value(0, 0, 0);
            else
                set_value(0, 255, 255);
        } else if (host_connected_.load(std::memory_order::relaxed)) {
            // 主机会话已建立(nonce 握手完成、keepalive 租约有效): 常亮绿色表示
            // 数据确实在转发。
            set_value(0, 255, 0);
        } else {
            // 存活但尚无主机会话: 绿色呼吸灯。已枚举但无会话仍呈等待态而非
            // 工作态。
            auto brightness = (tick >> 2) & 511;
            if (brightness > 255)
                brightness = 511 - brightness;
            set_value(0, static_cast<uint8_t>(brightness), 0);
        }
    }

    void set_host_connected(bool connected) {
        host_connected_.store(connected, std::memory_order::relaxed);
    }

    // 刻意保持非静态以确保实例化
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    void set_value(uint8_t red, uint8_t green, uint8_t blue) {
        static uint32_t last_color = 0;
        const uint32_t color =
            (static_cast<uint32_t>(red) << 16) | (static_cast<uint32_t>(green) << 8) | blue;
        if (color == last_color)
            return;

        // SPI6 属 D3 域外设, 唯一的 DMA 路径是 BDMA, 后者只够到 D3 SRAM
        // (0x38000000), 永远够不到 AXI/D2 SRAM -- WS2812 帧缓冲因此放 .d3_sram,
        // 32 字节对齐并补齐到 cache line 整数倍, 便于 BDMA 读取前从 D-cache 清洗。
        alignas(32) [[gnu::section(".d3_sram")]] static uint8_t txbuf[128];

        // BDMA 仍在移出上一帧时跳过; 下次颜色变化会重试, 故 last_color 只在成功
        // 启动传输后才提交。阻塞式发送(约 165 us)会在呼吸动画的每次颜色步进上
        // 卡住转发循环, 不可接受。
        if (HAL_SPI_GetState(&hspi6) != HAL_SPI_STATE_READY)
            return;

        std::memset(txbuf, 0, sizeof(txbuf));
        const uint8_t ws2812_high = 0xf0;
        const uint8_t ws2812_low = 0xC0;
        for (int i = 0; i < 8; i++) {
            txbuf[7 - i] = (((green >> i) & 0x01) ? ws2812_high : ws2812_low) >> 1;
            txbuf[15 - i] = (((red >> i) & 0x01) ? ws2812_high : ws2812_low) >> 1;
            txbuf[23 - i] = (((blue >> i) & 0x01) ? ws2812_high : ws2812_low) >> 1;
        }
        SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(txbuf), sizeof(txbuf));
        if (HAL_SPI_Transmit_DMA(&hspi6, txbuf, 124) == HAL_OK)
            last_color = color;
    }

private:
    std::atomic<bool> user_controlling_;
    std::atomic<bool> host_connected_{false};
    std::atomic<uint16_t> uplink_full_reset_counter_, downlink_full_reset_counter_;
    uint32_t last_tick_ = 0;
};

inline constinit utility::Lazy<Led> led;

} // namespace libhcs::firmware::led
