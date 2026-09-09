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

    // Polled from the main loop, but paced off SysTick so the patterns are
    // independent of the loop cadence. No SPI transmit happens in any ISR
    // context.
    //
    // The tick MUST be a millisecond count, not a loop-iteration count: run()
    // spins at 69-85 kHz depending on build options and load (measured with
    // libhcs_APP_LOOP_PROFILE; an earlier version of this comment said 380 kHz,
    // which was never measured and is out by about 4.5x). Counting iterations
    // therefore ran every pattern tens of times too fast -- the buffer-full blink
    // and the breathing cycle both landed far above flicker fusion, so each read
    // as a steady half-brightness colour rather than an animation. Gating here also
    // makes the 5000-count reset windows below span the 5 seconds their
    // magnitude implies instead of 13 ms, and stops the WS2812 frame from being
    // re-sent thousands of times a second.
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
            // Host session established (nonce handshake done, keepalive lease
            // live): steady green means data is actually being forwarded.
            set_value(0, 255, 0);
        } else {
            // Alive but no host session yet: green breathing light. Enumerated
            // without a live session still shows as "waiting", not "working".
            auto brightness = (tick >> 2) & 511;
            if (brightness > 255)
                brightness = 511 - brightness;
            set_value(0, static_cast<uint8_t>(brightness), 0);
        }
    }

    void set_host_connected(bool connected) {
        host_connected_.store(connected, std::memory_order::relaxed);
    }

    // Non-static to ensure instantiation
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    void set_value(uint8_t red, uint8_t green, uint8_t blue) {
        static uint32_t last_color = 0;
        const uint32_t color =
            (static_cast<uint32_t>(red) << 16) | (static_cast<uint32_t>(green) << 8) | blue;
        if (color == last_color)
            return;

        // SPI6 is a D3-domain peripheral, so its only DMA path is BDMA, which
        // reaches D3 SRAM (0x38000000) but never AXI/D2 SRAM -- keep the WS2812
        // frame buffer in .d3_sram, 32-byte aligned and padded to a cache-line
        // multiple so it can be cleaned from D-cache before BDMA reads it.
        alignas(32) [[gnu::section(".d3_sram")]] static uint8_t txbuf[128];

        // Skip if BDMA is still shifting out the previous frame; the next colour
        // change retries, so last_color is committed only on a successful launch.
        // Replaces the old ~165 us blocking HAL_SPI_Transmit that stalled the
        // forwarding loop on every colour step of the breathing animation.
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
