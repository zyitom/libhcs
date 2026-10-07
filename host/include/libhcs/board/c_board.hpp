#pragma once

#include <cstdint>

#include <libhcs/board/board.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/data/datas.hpp>
#include <libhcs/spec/c_board/ports.hpp>

namespace libhcs::board {

/**
 * @brief High-level host board interface for C Board.
 *
 * The class owns the transport and protocol stack for a single board connection. The
 * supplied `Callback` is stored by reference, is not owned by the board, and must outlive
 * the board instance.
 *
 * The board may start transport I/O during construction, so receive callbacks may be
 * invoked before the board constructor returns. A common usage pattern is for an
 * enclosing user type to inherit `Callback` and declare the board as its last data
 * member; early callbacks must not depend on invariants established later (delay board
 * construction with `std::optional` when they would).
 *
 * The ports (two bxCAN buses, three UARTs, the on-board IMU, the seven PWM pins) and
 * their capabilities live in the board's inventory (libhcs/spec/c_board/ports.hpp);
 * callback dispatch, transmit gates, port handles and the construction-time declaration
 * are Board<Spec>'s. What remains here is what is genuinely this board's: the USB identity.
 */
class CBoard final : public Board<spec::c_board::Spec> {
public:
    explicit CBoard(
        Board::Callback& callback = default_callback_, std::string_view serial_filter = {},
        const AdvancedOptions& options = {}, const Configuration& configuration = {})
        : Board<spec::c_board::Spec>(
              kVendorId, kProductIds, callback, serial_filter, options, configuration) {}

    CBoard(const CBoard&) = delete;
    CBoard& operator=(const CBoard&) = delete;
    CBoard(CBoard&&) = delete;
    CBoard& operator=(CBoard&&) = delete;
    ~CBoard() = default;

private:
    static constexpr uint16_t kVendorId = 0xA511; // 与固件 usb_descriptors.hpp 的 idVendor 一致
    static constexpr uint16_t kProductId = 0xF407;
    static constexpr std::span<const uint16_t> kProductIds{&kProductId, 1};
};

} // namespace libhcs::board
