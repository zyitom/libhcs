#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "firmware/hpm_board/app/src/led/gpio_led.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::led {

inline auto& led_backend = gpio_led;

// CAN bus fault categories.  Deliberately coarse: a CAN controller can only
// reliably tell "nobody acknowledged my frame" apart from "the bits on the wire
// are corrupted".  The specific protocol error (stuff/form/bit/CRC) fluctuates
// frame to frame and does NOT map back to a single physical cause -- reversed
// wiring, a missing 120R termination, a short, a baudrate mismatch and plain
// noise all produce the same mix -- so they are merged into one BUS-ERROR state
// instead of pretending to distinguish them.  Each state renders as a distinct,
// easy-to-read indicator-LED pattern (see the table in Led::update()).
enum class CanFault : uint8_t {
    kNone = 0,    // healthy / no error
    kNoAck,       // ACK error -- alone on the bus / partner unpowered / TX wire cut
    kWiringFault, // Bit0 error: cannot drive the bus dominant -- CAN_H/L shorted
                  // together, reversed, or open (the common physical wiring mistakes)
    kSignalError, // stuff/form/CRC (and the rare Bit1 "stuck dominant"): corrupted
                  // bits whose causes -- missing 120R termination, baudrate mismatch,
                  // noise -- fluctuate frame to frame and cannot be told apart
    kBusOff,      // controller bus-off -- too many errors, recovering/offline
};

class Led {
public:
    using Lazy = utility::Lazy<Led>;

    Led() {
        led_backend.init();
        board::init_can_indicator_pins();
    }

    void reset() {
        uplink_full_reset_counter_.store(0, std::memory_order::relaxed);
        downlink_full_reset_counter_.store(0, std::memory_order::relaxed);
    }

    void uplink_buffer_full() {
        uplink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }

    void downlink_buffer_full() {
        downlink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }

    void update(uint32_t tick) {
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

        // Simple 3-channel GPIO RGB LED (each color is only on/off, no PWM
        // brightness). Color language mirrors the c_board indicator so both
        // boards read the same; yellow is red+green and cyan is green+blue, and
        // all three channels are never lit at once so the status stays readable:
        //   steady green     = host session established (data forwarding)
        //   slow green blink  = alive, waiting for host session
        //   yellow blink      = uplink (board -> host) buffer full
        //   cyan blink        = downlink (host -> board) buffer full
        //   yellow/cyan alt   = both directions congested
        const bool on = (tick & 128U) != 0;
        if (uplink_full && downlink_full) {
            led_backend->set_value(on ? 255 : 0, 255, on ? 0 : 255);
        } else if (uplink_full) {
            led_backend->set_value(on ? 255 : 0, on ? 255 : 0, 0);
        } else if (downlink_full) {
            led_backend->set_value(0, on ? 255 : 0, on ? 255 : 0);
        } else if (host_connected_.load(std::memory_order::relaxed)) {
            // Host session established and healthy: steady green.
            led_backend->set_value(0, 255, 0);
        } else {
            // No host session yet: slow green blink (~1Hz) = alive, waiting for host.
            led_backend->set_value(0, (tick & 512U) ? 255 : 0, 0);
        }

        // CAN bus indicator light language — one independent LED per CAN
        // controller, taken from the board's kCanIndicatorPins table (empty on
        // boards without indicator LEDs).  Four states the controller can
        // actually distinguish, each a clearly different, easy-to-read pattern:
        //   off          = healthy / no errors
        //   slow blink   = NO-ACK: nobody acknowledged -- alone on the bus, the
        //                  partner is unpowered, or the TX wire is cut
        //   fast blink   = WIRING FAULT (Bit0): the bus cannot be driven dominant
        //                  -- CAN_H/L shorted together, reversed, or open
        //   double blink = SIGNAL ERROR: corrupted bits -- missing 120R termination,
        //                  baudrate mismatch or noise (cannot be told apart)
        //   solid on     = BUS-OFF: too many errors; controller recovering/offline
        // The CAN ISR refreshes the per-controller fault on every error
        // interrupt; it decays here ~5 s after the last error, returning to off.
        // Bounded by the runtime count, not the table capacity: the hpm5321 image
        // carries two indicator entries for the dual-CAN PCB, and the single-CAN
        // one has no indicator LEDs at all -- its PB14/PB15 are unpopulated, so
        // driving them would be writing to pads this board does not use.
        for (size_t i = 0; i < board::can_indicator_count(); ++i) {
            uint16_t timeout = can_fault_timeout_[i].load(std::memory_order::relaxed);
            if (timeout) {
                --timeout;
                can_fault_timeout_[i].store(timeout, std::memory_order::relaxed);
            }
            const CanFault fault =
                timeout ? can_fault_[i].load(std::memory_order::relaxed) : CanFault::kNone;
            bool led_on = false;
            switch (fault) {
            case CanFault::kNoAck: led_on = (tick % 1000U) < 500U; break;      // ~1 Hz
            case CanFault::kWiringFault: led_on = (tick % 200U) < 100U; break; // ~5 Hz
            case CanFault::kSignalError: { // two quick flashes, then a pause
                const uint32_t phase = tick % 1200U;
                led_on = phase < 120U || (phase >= 240U && phase < 360U);
                break;
            }
            case CanFault::kBusOff: led_on = true; break; // solid
            case CanFault::kNone: led_on = false; break;
            }
            board::kCanIndicatorPins[i].set_active(led_on);
        }
    }

    void set_host_connected(bool connected) {
        host_connected_.store(connected, std::memory_order::relaxed);
    }

    // CAN bus fault light-code tracking.  Called from the CAN ISR with the
    // controller index (0-based) and the current fault.  The timeout keeps the
    // indicator LED visible for ~5 s after the last error interrupt.  A kNone
    // report only refreshes the timeout and keeps the last concrete fault, so a
    // bus state change (warning/passive) carrying no fresh LEC does not blank a
    // fault that was reported moments earlier.
    void report_can_fault(uint8_t can_index, CanFault fault) {
        // Guards the array bound (capacity). Whether an indicator LED physically
        // exists is update()'s business: recording a fault for a slot that has no
        // LED is harmless, and the check that matters for memory safety is this
        // one.
        if (can_index >= kCanIndicatorCount)
            return;
        can_fault_timeout_[can_index].store(kCanFaultTimeoutTicks, std::memory_order::relaxed);
        if (fault != CanFault::kNone)
            can_fault_[can_index].store(fault, std::memory_order::relaxed);
    }

private:
    // Capacity of the board's indicator table, which sizes the state arrays
    // below. board::can_indicator_count() is how many LEDs are actually
    // populated; the two differ only on the hpm5321 image, whose table is sized
    // for the dual-CAN PCB while the single-CAN one has none.
    static constexpr size_t kCanIndicatorCount = board::kCanIndicatorPins.size();

    // CAN fault indicator state — ISR-safe via atomic stores.  Each CAN
    // controller gets its own fault/timeout pair, refreshed by the CAN ISR on
    // every error interrupt.
    static constexpr uint16_t kCanFaultTimeoutTicks = 5000; // 5 s at 1 kHz
    std::array<std::atomic<uint16_t>, kCanIndicatorCount> can_fault_timeout_{};
    std::array<std::atomic<CanFault>, kCanIndicatorCount> can_fault_{};

    std::atomic<uint16_t> uplink_full_reset_counter_{0};
    std::atomic<uint16_t> downlink_full_reset_counter_{0};
    std::atomic<bool> host_connected_{false};
};

inline constinit Led::Lazy led;

} // namespace libhcs::firmware::led
