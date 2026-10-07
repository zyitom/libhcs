#pragma once

#include <cstdint>

#include <libhcs/board/board.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/data/datas.hpp>
#include <libhcs/spec/mc02/ports.hpp>

namespace libhcs::board {

/**
 * @brief High-level host board interface for mc02.
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
 * The ports (three CAN buses, six UARTs, the on-board IMU, the four PWM pins) and their
 * capabilities live in the board's inventory (libhcs/spec/mc02/ports.hpp); callback
 * dispatch, transmit gates, port handles and the construction-time declaration are
 * Board<Spec>'s. What remains here is what is genuinely mc02's: the USB identity, the
 * diagnostic stream, and the DBUS port's receiver presets.
 */
class Mc02 final : public Board<spec::mc02::Spec> {
public:
    class Callback : public Board::Callback {
    public:
        // Telemetry from a diagnostic firmware (CAN_DIAG / LOOP_PROFILE /
        // USB_RX_HIST), on DataId::kUart0 -- not a silkscreen port on this board,
        // so it is not in the inventory and lands here. Accepting it matters:
        // returning false makes the deserializer treat the frame as a protocol
        // error and tear the session down, so a diagnostic build would kill the
        // link it is meant to be diagnosing.
        bool unlisted_uart_receive_callback(
            data::DataId id, const libhcs::data::UartDataView& d) override {
            if (id == data::DataId::kUart0) {
                diagnostic_receive_callback(d);
                return true;
            }
            return false;
        }

        virtual void diagnostic_receive_callback(const libhcs::data::UartDataView& d) { (void)d; }
    };

    explicit Mc02(
        Board::Callback& callback = default_callback_, std::string_view serial_filter = {},
        const AdvancedOptions& options = {}, const Configuration& configuration = {})
        : Board<spec::mc02::Spec>(
              kVendorId, kProductIds, callback, serial_filter, options, configuration) {}

    Mc02(const Mc02&) = delete;
    Mc02& operator=(const Mc02&) = delete;
    Mc02(Mc02&&) = delete;
    Mc02& operator=(Mc02&&) = delete;
    ~Mc02() = default;

    // The receivers the DBUS port takes. The port has an on-board inverter, so
    // the inverted protocols (DBUS, SBUS) run at normal polarity at the MCU pin,
    // and iBUS -- not inverted on the wire -- needs RXINV to cancel the
    // inverter. Each preset sets every field, so switching receivers never
    // inherits a field from the previous one. Apply with
    // handle(Spec::kUarts.kDbus).configure(preset).
    enum class DbusReceiver : uint8_t {
        kDbus, // DJI DR16 (DT7): 100000 8E1, inverted on the wire
        kSbus, // S.BUS / WFLY W.BUS: 100000 8E2 on the wire, received as 8E1
        kIbus, // FlySky iBUS: 115200 8N1, not inverted on the wire
    };
    static constexpr hcs::UartSetting dbus_receiver_setting(DbusReceiver receiver) {
        // SBUS sends a second stop bit; the receiver only checks the first, so
        // one preset serves both 100 kbaud protocols.
        constexpr hcs::UartSetting k100kEven1{
            .baudrate = 100000,
            .word_length = hcs::vc::kUartWordLength8,
            .parity = hcs::vc::kUartParityEven,
            .stop_bits = hcs::vc::kUartStopBits1,
            .rx_polarity = hcs::vc::kUartRxPolarityNormal,
        };
        switch (receiver) {
        case DbusReceiver::kDbus:
        case DbusReceiver::kSbus: return k100kEven1;
        case DbusReceiver::kIbus:
            return {
                .baudrate = 115200,
                .word_length = hcs::vc::kUartWordLength8,
                .parity = hcs::vc::kUartParityNone,
                .stop_bits = hcs::vc::kUartStopBits1,
                .rx_polarity = hcs::vc::kUartRxPolarityInverted,
            };
        }
        return k100kEven1;
    }

    // mc02 uses the shared HCS vendor id (0xA511) and the fixed board-type PID
    // 0x0723; per-device identity lives in the serial number, so pass a
    // serial_filter to target a specific board when several are connected.
    static constexpr uint16_t kVendorId = 0xA511;
    static constexpr uint16_t kProductId = 0x0723;
    static constexpr std::span<const uint16_t> kProductIds{&kProductId, 1};
};

} // namespace libhcs::board
