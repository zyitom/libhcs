// EP0 configuration channel test for the mc02 board -- the one board whose CAN
// bus mode the HOST may change (kCapCanModeSettable).
//
// WHAT IT PROVES. This is the mc02 counterpart of ep0_config_test (hpm5321):
// the same EP0 v2 handshake and read-back contract, exercised on the side of
// the capability line where a mode request is APPLIED rather than asserted:
//
//   1. The board describes itself      -- GET_INTERFACE: 3 CAN buses, 6 UART
//      indexes, all three buses CAN-FD, and the mode-settable capability bit
//      SET (hpm5321 clears it -- the two boards share one protocol, one byte
//      apart).
//   2. The CAN timing identity reads back as one coherent truth: mode=FD,
//      1 Mbit/s arbitration, 5 Mbit/s data, 87.5 per mille sample points --
//      computed by the board from its own bit timing, not echoed requests.
//   3. A host-driven mode switch APPLIES and reads back: CAN1 -> classic, then
//      restore -> FD. In classic mode the data phase timing still reads back
//      5 Mbit/s -- the receiver keeps its FD capability; only what the bus
//      transmits changes. That is exactly why the mode field and the rate
//      fields are orthogonal (see CanConfigPayload).
//   4. UART framing applies and reads back (2 stop bits), untouched fields
//      survive, and an explicit full setting restores the defaults. The
//      sparse-patch rule -- zero means "leave unchanged" -- is what the
//      restore path leans on.
//   5. A rate the divisor solver cannot represent is rejected with the port
//      untouched.
//
// Needs one mc02 board (PID 0x0723). No CAN or UART wiring: every check is a
// control transfer plus the board's own read-back. Pass a serial filter as
// argv[1] to pick one of several attached boards.

#include <cstdint>
#include <cstdio>
#include <exception>
#include <string>
#include <string_view>
#include <utility>

#include <libhcs/board/mc02.hpp>

namespace {

using libhcs::board::AdvancedOptions;
using libhcs::board::Mc02;

// The EP0 channel's host-side helpers and wire enums.
namespace hcs = libhcs::board::hcs;

int g_failures = 0;

void check(bool condition, std::string_view what) {
    std::printf("  [%s] %.*s\n", condition ? "PASS" : "FAIL", static_cast<int>(what.size()),
                what.data());
    if (!condition)
        ++g_failures;
}

// The board's divisor solver accepts a rate within 3% of the request; a
// read-back is correct when it lands inside that band.
bool within_tolerance(uint32_t requested, uint32_t effective) {
    const uint32_t error = effective > requested ? effective - requested : requested - effective;
    return static_cast<uint64_t>(error) * 100U <= static_cast<uint64_t>(requested) * 3U;
}

constexpr uint32_t kDefaultUart1Baudrate = 115200;

} // namespace

int main(int argc, char** argv) {
    const std::string_view filter = argc > 1 ? argv[1] : std::string_view{};
    Mc02::Callback callback;

    try {
        std::printf("1. board self-description\n");
        AdvancedOptions options;
        options.set_dangerously_skip_version_checks(true);
        Mc02 board{callback, filter, options};
        const auto& interface = board.interface();
        std::printf(
            "   can_count=%u uart_count=%u can_fd_mask=0x%02x settable=%d\n", interface.can_count,
            interface.uart_count, interface.can_fd_mask, interface.can_mode_settable ? 1 : 0);
        check(interface.can_count == 3, "reports three CAN buses");
        check(interface.uart_count == 6, "reports the six-slot UART index space");
        check(
            interface.can_fd_mask == 0b111 && board.can1_is_fd() && board.can2_is_fd()
                && board.can3_is_fd(),
            "all three buses report CAN-FD");
        check(interface.can_mode_settable, "CAN mode IS host-settable on this board");

        std::printf("2. CAN timing identity reads back as one coherent truth\n");
        for (const int bus : {0, 1, 2}) {
            const auto config = board.can_config(
                static_cast<hcs::CanPort>(bus + 1)); // silk CAN1..CAN3 -> EP0 index 0..2
            std::printf(
                "   CAN%d: mode=%u arb=%u data=%u sp=%u/%u\n", bus + 1, config.mode,
                config.arbitration_baudrate, config.data_baudrate, config.nominal_sample_point,
                config.data_sample_point);
            check(config.mode == std::to_underlying(hcs::vc::CanMode::kCanFd), "runs CAN-FD");
            check(config.arbitration_baudrate == 1'000'000U, "arbitration phase is 1 Mbit/s");
            check(config.data_baudrate == 5'000'000U, "data phase is 5 Mbit/s");
            check(config.nominal_sample_point == 875U, "nominal sample point is 87.5%");
            check(config.data_sample_point == 875U, "data sample point is 87.5%");
        }

        std::printf("3. host-driven CAN mode switch applies and reads back\n");
        board.configure_can1(false);
        check(!board.can1_is_fd(), "CAN1 switches to classic on request");
        {
            const auto config = board.can_config(hcs::CanPort::kCan1);
            check(config.mode == std::to_underlying(hcs::vc::CanMode::kClassic),
                  "read-back confirms classic TX");
            check(
                config.data_baudrate == 5'000'000U && config.data_sample_point == 875U,
                "and the data phase timing is still there -- the receiver keeps its FD "
                "capability, which is why mode and rates cannot be merged");
        }
        board.configure_can1(true);
        check(board.can1_is_fd() && board.can_config(hcs::CanPort::kCan1).mode
                                    == std::to_underlying(hcs::vc::CanMode::kCanFd),
              "and switches back to CAN-FD on request");

        std::printf("4. UART framing applies, reads back, and restores\n");
        board.configure_uart1(kDefaultUart1Baudrate);
        const uint32_t base_effective = board.uart1_baudrate();
        check(within_tolerance(kDefaultUart1Baudrate, base_effective), "115200 reads back");

        hcs::UartSetting two_stops;
        two_stops.baudrate = kDefaultUart1Baudrate;
        two_stops.stop_bits = hcs::vc::UartStopBits::kUartStopBits2;
        board.configure_uart1(two_stops);
        const auto applied = board.read_uart_setting(1); // EP0 index 1 = UART1
        std::printf(
            "   rate=%u word=%u parity=%u stop=%u\n", applied.baudrate,
            std::to_underlying(applied.word_length), std::to_underlying(applied.parity),
            std::to_underlying(applied.stop_bits));
        check(applied.stop_bits == hcs::vc::UartStopBits::kUartStopBits2,
              "2 stop bits applied and read back from the live registers");
        check(applied.word_length == hcs::vc::UartWordLength::kUartWordLength8,
              "untouched fields stay as they were (8 data bits)");
        check(applied.parity == hcs::vc::UartParity::kUartParityNone,
              "untouched fields stay as they were (no parity)");

        hcs::UartSetting restore;
        restore.baudrate = kDefaultUart1Baudrate;
        restore.stop_bits = hcs::vc::UartStopBits::kUartStopBits1;
        board.configure_uart1(restore);
        check(board.read_uart_setting(1).stop_bits == hcs::vc::UartStopBits::kUartStopBits1,
              "explicit full setting restores the default framing");

        std::printf("5. unrepresentable baudrate is rejected, port unchanged\n");
        bool threw = false;
        try {
            board.configure_uart1(6'000'000U);
        } catch (const std::exception& error) {
            threw = true;
            std::printf("   threw: %s\n", error.what());
        }
        check(threw, "6000000 baud throws instead of reporting success");
        check(
            within_tolerance(kDefaultUart1Baudrate, board.uart1_baudrate()),
            "the port is still running at the rate it had before the rejection");

        std::printf("6. controller status is readable over EP0\n");
        const auto status = board.can_status(hcs::CanPort::kCan1);
        std::printf(
            "   tec=%u rec=%u last_error=%u flags=%u rx_frames=%u\n", status.tec, status.rec,
            status.last_error, status.flags, status.rx_frames);
        // No CAN traffic ran here; what matters is that the request completes and
        // the counters are coherent (TEC/REC sane, rx counter present).
        check(status.rec <= 127U && status.tec <= 255U, "error counters read back in range");

        std::printf("%s (%d failure%s)\n", g_failures == 0 ? "ALL PASSED" : "FAILED", g_failures,
                    g_failures == 1 ? "" : "s");
        return g_failures == 0 ? 0 : 1;
    } catch (const std::exception& error) {
        std::printf("  [FAIL] unexpected exception: %s\n", error.what());
        return 1;
    }
}
