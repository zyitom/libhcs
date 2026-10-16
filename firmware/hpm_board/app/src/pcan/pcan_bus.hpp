#pragma once

#include <cstdint>

#include "core/include/libhcs/data/datas.hpp"
#include "firmware/hpm_board/app/src/pcan/pcan_protocol.hpp"

namespace libhcs::firmware::pcan {

// libhcs 的故障位在低三位, uCAN 的低四位却是通道号, 不能直接拼接。
constexpr uint8_t encode_status_bits(uint8_t flags) {
    uint8_t bits = 0;
    if ((flags & data::kCanErrorPassive) != 0)
        bits |= protocol::kStatusPassive;
    if ((flags & data::kCanWarning) != 0)
        bits |= protocol::kStatusWarning;
    if ((flags & data::kCanBusOff) != 0)
        bits |= protocol::kStatusBusOff;
    return bits;
}

enum class ChannelMode : uint8_t { kReset, kNormal, kListenOnly };

// 主机意图与实际在线状态分开: 时序被拒绝时保留意图, 等下一条配置重试,
// 但撤下控制器、关收发闸, 并经 uCAN STATUS 报告通道不可用(BUSOFF)。
template <typename Timing>
struct BusState {
    ChannelMode mode = ChannelMode::kReset;
    bool timing_known = false;
    bool data_known = false;
    bool fd = false;
    bool non_iso = false;
    bool configuration_failed = false;
    Timing nominal{};
    Timing data{};

    [[nodiscard]] bool active() const {
        return mode != ChannelMode::kReset && !configuration_failed;
    }

    [[nodiscard]] uint8_t status_bits(uint8_t flags) const {
        return configuration_failed ? protocol::kStatusBusOff : encode_status_bits(flags);
    }

    template <typename Driver>
    bool apply(Driver& can) {
        if (mode == ChannelMode::kReset) {
            can.set_bus_off();
            configuration_failed = false;
            return false;
        }

        const auto fits = [](const Timing& phase, bool fast) {
            return phase.prescaler >= 1U && phase.prescaler <= (fast ? 32U : 512U)
                && phase.seg1 >= 1U && phase.seg1 <= (fast ? 32U : 256U) && phase.seg2 >= 1U
                && phase.seg2 <= (fast ? 16U : 128U) && phase.sjw >= 1U && phase.sjw <= phase.seg2;
        };
        if (!timing_known || !fits(nominal, false) || (data_known && !fits(data, true))
            || !can.reconfigure_timing(data_known, nominal, data)) {
            // reconfigure_timing 失败时可能已恢复默认配置并上线, 必须再次撤下。
            can.set_bus_off();
            configuration_failed = true;
            return false;
        }

        fd = data_known;
        configuration_failed = false;
        can.set_bus_on(mode == ChannelMode::kListenOnly, non_iso && fd);
        return true;
    }
};

} // namespace libhcs::firmware::pcan
