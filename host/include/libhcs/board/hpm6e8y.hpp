#pragma once

#include <cstdint>

#include <libhcs/board/board.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/protocol/usb_identity.hpp>
#include <libhcs/spec/hpm6e8y/ports.hpp>

namespace libhcs::board {

// Board interface for the HPM6E8Y board: four MCAN controllers (silkscreen
// CAN0..CAN3) and nothing else -- its only UART is a debug pad, not a
// connector, and the shipping firmware has no driver for it. The port names
// live in libhcs/spec/hpm6e8y/ports.hpp; what each port can do is read from the
// board at construction. Everything else -- callback dispatch, transmit gates,
// port handles, the construction-time declaration -- is Board<Spec>'s.
class Hpm6e8y final : public Board<spec::hpm6e8y::Spec> {
public:
    explicit Hpm6e8y(
        Callback& callback = default_callback_, std::string_view serial_filter = {},
        const AdvancedOptions& options = {}, const Configuration& configuration = {})
        : Board<spec::hpm6e8y::Spec>(
              core::protocol::usb_identity::kDmtoolVendorId, kProductIds, callback, serial_filter,
              options, configuration) {}

    Hpm6e8y(const Hpm6e8y&) = delete;
    Hpm6e8y& operator=(const Hpm6e8y&) = delete;
    Hpm6e8y(Hpm6e8y&&) = delete;
    Hpm6e8y& operator=(Hpm6e8y&&) = delete;
    ~Hpm6e8y() = default;

private:
    static constexpr uint16_t kProductId = 0x6E84;
    static constexpr std::span<const uint16_t> kProductIds{&kProductId, 1};
};

} // namespace libhcs::board
