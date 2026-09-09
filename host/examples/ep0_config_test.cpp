// EP0 configuration channel test for the HPM5321 dual-CAN-FD board.
//
// WHAT IT PROVES. Channel configuration used to be write-only: a UART baudrate
// went out as an in-band kUart*Config field whose acceptance the board had no
// way to report, and a CAN frame type was a per-frame header bit that a classic
// bus silently downgraded. Both now ride EP0, where the status stage carries the
// answer. This exercises exactly the outcomes the old path could not express:
//
//   1. The board describes itself      -- GET_INTERFACE agrees with the PCB.
//   2. An accepted rate is applied -- and reads back from the hardware as the
//      rate the DIVISOR produces, which is never exactly the requested one
//      (115200 -> 114942 at 80 MHz). Compare with tolerance, never for equality.
//   3. An UNREPRESENTABLE rate is REJECTED, loudly, and the port keeps running
//      at its previous rate. This is the case that used to look like success.
//   4. A rate inside the solver's 3% tolerance is accepted and reported as the
//      value actually programmed, not the value requested (3000000 -> 3076923).
//   5. A wrong CAN mode expectation throws at construction instead of quietly
//      producing frames of the other type on the wire.
//   6. Construction-time configuration works, and a board object that finished
//      constructing is a board whose configuration is known.
//
// Needs one HPM5321 dual-CAN-FD board (PID 0x5322). No CAN or UART wiring: every
// check is a control transfer plus the board's own read-back. Pass a serial
// filter as argv[1] to pick one of several attached boards.

#include <cstdint>
#include <cstdio>
#include <exception>
#include <string>
#include <string_view>

#include <libhcs/board/hpm5321.hpp>

namespace {

using libhcs::board::AdvancedOptions;
using libhcs::board::Hpm5321;

int g_failures = 0;

// The board's solver accepts a divisor within 3% of the request, so a read-back
// is correct when it is inside that band -- not when it equals the request. An
// equality check here would fail on every rate 80 MHz cannot divide exactly,
// which is most of them.
bool within_tolerance(uint32_t requested, uint32_t effective) {
    const uint32_t error = effective > requested ? effective - requested : requested - effective;
    return static_cast<uint64_t>(error) * 100U <= static_cast<uint64_t>(requested) * 3U;
}

void check(bool condition, std::string_view what) {
    std::printf("  [%s] %.*s\n", condition ? "PASS" : "FAIL", static_cast<int>(what.size()),
                what.data());
    if (!condition)
        ++g_failures;
}

// A rate distinct from the firmware default, used to prove a switch reached the
// hardware. Every path here ends by restoring kDefaultBaudrate, so a run does
// not leave the port at a rate the peer board cannot talk to -- which would look
// exactly like a wiring fault to the next test that runs.
constexpr uint32_t kBaseBaudrate = 115200;

// What the hpm5321 firmware brings UART0 up at (boards/hpm5321/app/board_app.hpp).
constexpr uint32_t kDefaultBaudrate = 921600;

// 80 MHz over a 5-bit oversample and a 16-bit divisor cannot land within 3% of
// this, so the board's solver refuses it -- the case the old in-band path
// reported as success.
constexpr uint32_t kUnrepresentableBaudrate = 6'000'000;

// Inside the tolerance but NOT exact: osc=26 div=1 gives 3076923, 2.56% high.
// The board must report what it programmed, not what was asked for.
constexpr uint32_t kApproximateBaudrate = 3'000'000;

// AdvancedOptions is deliberately non-copyable and non-movable, so it is built
// in place at each construction site rather than returned from a helper.
} // namespace

int main(int argc, char** argv) {
    const std::string_view filter = argc > 1 ? argv[1] : std::string_view{};
    // Nothing here reads data; the callback exists only because the board
    // constructor takes one.
    Hpm5321::Callback callback;

    try {
        std::printf("1. board self-description\n");
        AdvancedOptions options;
        options.set_dangerously_skip_version_checks(true);
        Hpm5321 board{callback, filter, options};
        const auto& interface = board.interface();
        std::printf(
            "   can_count=%u uart_count=%u can_fd_mask=0x%02x\n", interface.can_count,
            interface.uart_count, interface.can_fd_mask);
        check(interface.can_count == 2, "reports two CAN buses");
        check(interface.uart_count >= 1, "reports at least one UART");
        check(board.can1_is_fd() && board.can2_is_fd(), "both buses report CAN-FD");

        std::printf("2. accepted baudrate is applied and reads back\n");
        board.configure_uart0(kBaseBaudrate);
        const uint32_t base_effective = board.uart0_baudrate();
        std::printf("   requested %u, board reports %u\n", kBaseBaudrate, base_effective);
        check(within_tolerance(kBaseBaudrate, base_effective), "115200 reads back within 3%");

        board.configure_uart0(kDefaultBaudrate);
        const uint32_t fast_effective = board.uart0_baudrate();
        std::printf("   requested %u, board reports %u\n", kDefaultBaudrate, fast_effective);
        check(
            within_tolerance(kDefaultBaudrate, fast_effective),
            "921600 reads back within 3%");
        check(
            fast_effective != base_effective,
            "and the read-back moved, so the switch reached the hardware");

        std::printf("3. unrepresentable baudrate is rejected, port unchanged\n");
        bool threw = false;
        try {
            board.configure_uart0(kUnrepresentableBaudrate);
        } catch (const std::exception& error) {
            threw = true;
            std::printf("   threw: %s\n", error.what());
        }
        check(threw, "6000000 baud throws instead of reporting success");
        check(
            board.uart0_baudrate() == fast_effective,
            "the port is still running at the rate it had before the rejection");

        std::printf("4. approximate baudrate reports what was programmed\n");
        board.configure_uart0(kApproximateBaudrate);
        const uint32_t approximate = board.uart0_baudrate();
        std::printf("   requested %u, board reports %u\n", kApproximateBaudrate, approximate);
        check(
            approximate != kApproximateBaudrate,
            "the read-back is the programmed rate, not the requested one");
        check(
            approximate > 2'900'000 && approximate < 3'200'000,
            "and it is within the solver's tolerance");

        board.configure_uart0(kBaseBaudrate);
        check(
            board.uart0_baudrate() == base_effective,
            "115200 again lands on the same divisor as the first time");
        board.configure_uart0(kDefaultBaudrate);
    } catch (const std::exception& error) {
        std::printf("  [FAIL] unexpected exception: %s\n", error.what());
        ++g_failures;
    }

    std::printf("5. wrong CAN mode expectation fails construction\n");
    {
        bool threw = false;
        try {
            AdvancedOptions options;
            options.set_dangerously_skip_version_checks(true);
            Hpm5321::Configuration configuration;
            configuration.can_fd[0] = false; // the board runs CAN-FD on bus 0
            Hpm5321 board{
                callback, filter, options, configuration};
            (void)board;
        } catch (const std::exception& error) {
            threw = true;
            std::printf("   threw: %s\n", error.what());
        }
        check(threw, "opening an FD bus as classic CAN throws");
    }

    std::printf("6. construction-time configuration is applied and verified\n");
    try {
        AdvancedOptions options;
        options.set_dangerously_skip_version_checks(true);
        Hpm5321::Configuration configuration;
        configuration.uart_baudrate[0] = kDefaultBaudrate;
        configuration.can_fd[0] = true;
        configuration.can_fd[1] = true;
        Hpm5321 board{
            callback, filter, options, configuration};
        const uint32_t effective = board.uart0_baudrate();
        std::printf("   board reports %u after construction\n", effective);
        check(
            within_tolerance(kDefaultBaudrate, effective),
            "the constructor left the port at 921600");
    } catch (const std::exception& error) {
        std::printf("  [FAIL] unexpected exception: %s\n", error.what());
        ++g_failures;
    }

    std::printf("%s (%d failure%s)\n", g_failures == 0 ? "ALL PASSED" : "FAILED", g_failures,
                g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
