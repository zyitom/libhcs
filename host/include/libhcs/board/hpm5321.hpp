#pragma once

#include <cstdint>
#include <span>

#include <libhcs/board/board.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/protocol/usb_identity.hpp>
#include <libhcs/spec/hpm5321/ports.hpp>

namespace libhcs::board {

// Board interface for the HPM5321 boards. ONE class for BOTH PCBs -- the
// single-CAN one (PID 0x6877) and the dual-CAN one (0x6632) -- because they
// run one firmware image that tells itself apart from OTP word 25, and because
// the host learns which ports the PCB actually carries by asking: kGetPortList
// answers before anything is declared, and apply() fails with the port's name
// if the wiring names one the board lacks.
//
// The ids are DM USB2FDCAN ones so that DMTool accepts the board too
// (libhcs/protocol/usb_identity.hpp); the DFU bootloader still reports
// 0xA511:0x5321 / 0x5322. It has no IMU and no GPIO application channels, so
// those callbacks have empty defaults.
//
// Everything else -- callback dispatch, transmit gates, port handles, the
// construction-time declaration -- is Board<Spec>'s, driven by the board's
// port inventory (libhcs/spec/hpm5321/ports.hpp).
class Hpm5321 final : public Board<spec::hpm5321::Spec> {
public:
    explicit Hpm5321(
        Callback& callback = default_callback_, std::string_view serial_filter = {},
        const AdvancedOptions& options = {}, const Configuration& configuration = {})
        : Board<spec::hpm5321::Spec>(
              core::protocol::usb_identity::kDmtoolVendorId, kProductIds, callback, serial_filter,
              options, configuration) {}

    Hpm5321(const Hpm5321&) = delete;
    Hpm5321& operator=(const Hpm5321&) = delete;
    Hpm5321(Hpm5321&&) = delete;
    Hpm5321& operator=(Hpm5321&&) = delete;
    ~Hpm5321() = default;

private:
    // Both PCBs, in the order the scanner should prefer to report them. The
    // firmware picks its own product ID from OTP, so which one answers is a
    // property of the hardware, not of this call.
    static constexpr uint16_t kProductIdsStorage[]{
        core::protocol::usb_identity::kHpm5321SingleCanProductId,
        core::protocol::usb_identity::kHpm5321DualCanProductId};
    static constexpr std::span<const uint16_t> kProductIds{kProductIdsStorage};
};

} // namespace libhcs::board
