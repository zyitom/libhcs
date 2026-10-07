#pragma once

#include <atomic>

#include "core/include/libhcs/data/datas.hpp"

namespace libhcs::firmware::utility {

// 一路 CAN 控制器"最近一次真实总线错误"的锁存(data::CanStatusView 的 last_error /
// data_last_error)。错误码寄存器留不住它: M_CAN 的 PSR.LEC/DLEC 读后自清, bxCAN 的
// ESR.LEC 成功收发后复位为 none。所以驱动每次读到错误码都交给 note(), 只记真实错误
// (data::is_bus_error), 上报时用 latest()。三块板的驱动共用这一份, 端口状态的账本因此
// 不必认识任何一种状态的特例。
//
// 单写者: 协议错误中断(hpm、mc02 的 PSR, c_board 的 CAN_SCE 经 HAL_CAN_ErrorCallback)。
// 不能是主循环: LEC 在下一帧成功收发后就被硬件写回 none, 250 ms 一读在忙的总线上几乎
// 看不到错误码(core/PORT_STATUS.md 2.4)。relaxed 的单字节原子, 与裸字节同价。
class LatchedBusError {
public:
    void note(data::CanLastError code) noexcept {
        if (data::is_bus_error(code))
            latest_.store(code, std::memory_order_relaxed);
    }

    // 刚读到的若是真实错误就是它, 否则是锁存的那一个(从没出过错为 kNone)。只读, 不写:
    // 另有写者的驱动(hpm 的中断)也能在主循环里调用。
    [[nodiscard]] data::CanLastError latest(data::CanLastError now) const noexcept {
        return data::is_bus_error(now) ? now : latest_.load(std::memory_order_relaxed);
    }

private:
    std::atomic<data::CanLastError> latest_{data::CanLastError::kNone};
};

static_assert(std::atomic<data::CanLastError>::is_always_lock_free);

} // namespace libhcs::firmware::utility
