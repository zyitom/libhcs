#pragma once

#include <cstdint>

#include "core/src/utility/immovable.hpp"
#include "firmware/ch32_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::led {

// Minimal status-LED stub for the CH32H417EVT. mc02 drives a WS2812 over SPI for
// its status indicator; this eval board has no such LED wired for libhcs, so
// every method is a no-op. The API surface matches mc02's Led exactly
// (downlink_buffer_full/uplink_buffer_full/poll/set_value) so the CV'd
// forwarding modules and assert path compile unchanged. Replace with a real
// driver (GPIO or on-board RGB) during board bring-up if a panic indicator is
// wanted.
class Led : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Led>;

    Led() = default;

    void init() {}
    void poll() {}

    void set_value(uint8_t /*r*/, uint8_t /*g*/, uint8_t /*b*/) {}

    void downlink_buffer_full() {}
    void uplink_buffer_full() {}
};

inline constinit Led::Lazy led;

} // namespace libhcs::firmware::led
