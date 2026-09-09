#pragma once

// Decoder for the telemetry record a libhcs_CAN_DIAG firmware emits as a UART0
// uplink frame (firmware/hpm_board/app/src/diag/can_diag.cpp). Shared by the
// tools that need it so the wire format is described once: hpm5321_loop_probe
// reads it under a paced load, usb_packet_rate reads it under a saturating one,
// and the interesting numbers only exist at saturation.
//
// Only two things are decoded here: the fixed header's main-loop counter and the
// USB timing block appended at the very end. Everything between them is the
// variable-length PLIC/CAN section, which these tools do not need and which
// would otherwise couple them to its layout.

#include <cstddef>
#include <cstdint>
#include <span>

namespace libhcs::diag {

// Must match firmware/hpm_board/app/src/diag/can_diag.hpp.
inline constexpr uint8_t kRecordMagic = 0xD1U;
inline constexpr uint8_t kRecordVersion = 6U;

// can_diag emits one record per 100 ms tick, so a record's main_loop_iters is
// iterations per 100000 us -- which is what turns it into a period.
inline constexpr double kRecordPeriodUs = 100000.0;

inline uint32_t get_u32_le(const std::byte* p) {
    return static_cast<uint32_t>(std::to_integer<uint8_t>(p[0]))
         | static_cast<uint32_t>(std::to_integer<uint8_t>(p[1])) << 8
         | static_cast<uint32_t>(std::to_integer<uint8_t>(p[2])) << 16
         | static_cast<uint32_t>(std::to_integer<uint8_t>(p[3])) << 24;
}

inline bool is_record(std::span<const std::byte> payload) {
    if (payload.size() < 20)
        return false;
    if (std::to_integer<uint8_t>(payload[0]) != kRecordMagic)
        return false;
    if (std::to_integer<uint8_t>(payload[1]) != kRecordVersion)
        return false;
    return get_u32_le(payload.data() + 4) == payload.size();
}

inline bool decode_main_loop_iters(std::span<const std::byte> payload, uint32_t& iters) {
    if (!is_record(payload))
        return false;
    iters = get_u32_le(payload.data() + 16);
    return true;
}

// USB bulk OUT endpoint timing, the last 16 bytes of the record. Splits the
// per-packet budget into the device-side turnaround (complete -> re-armed) and
// the starve interval (re-armed -> next complete, i.e. endpoint ready and idle).
// Only the first is something a chained qTD could remove.
//
// The starve figure is only meaningful when the endpoint is SATURATED: at a
// paced load most of it is just "the host had nothing to send yet".
struct UsbOutTiming {
    uint32_t turnaround_cycles;
    uint32_t starve_cycles;
    uint32_t samples;
    uint32_t core_hz;
};

inline bool decode_usb_out_timing(std::span<const std::byte> payload, UsbOutTiming& out) {
    if (!is_record(payload) || payload.size() < 16)
        return false;
    const std::byte* tail = payload.data() + payload.size() - 16;
    out.turnaround_cycles = get_u32_le(tail);
    out.starve_cycles = get_u32_le(tail + 4);
    out.samples = get_u32_le(tail + 8);
    out.core_hz = get_u32_le(tail + 12);
    return out.samples > 0 && out.core_hz > 0;
}

// Running total across records: the sums are per-emit-period, so averaging the
// per-record averages would weight a near-empty period like a busy one.
class UsbOutTimingAccumulator {
public:
    void add(const UsbOutTiming& timing) {
        turnaround_ += timing.turnaround_cycles;
        starve_ += timing.starve_cycles;
        samples_ += timing.samples;
        core_hz_ = timing.core_hz;
    }

    uint64_t samples() const { return samples_; }

    bool summary(double& turnaround_us, double& starve_us) const {
        if (samples_ == 0 || core_hz_ == 0)
            return false;
        const double us_per_cycle = 1e6 / static_cast<double>(core_hz_);
        turnaround_us =
            static_cast<double>(turnaround_) / static_cast<double>(samples_) * us_per_cycle;
        starve_us = static_cast<double>(starve_) / static_cast<double>(samples_) * us_per_cycle;
        return true;
    }

private:
    uint64_t turnaround_ = 0;
    uint64_t starve_ = 0;
    uint64_t samples_ = 0;
    uint32_t core_hz_ = 0;
};

} // namespace libhcs::diag
