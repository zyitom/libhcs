// Per-bus CAN health, read from the controller's own error registers over EP0.
//
// WHY IT EXISTS. When a bus delivers nothing, "the wire is bad" and "the
// firmware never transmitted" look identical from the host: both are rx=0. The
// controller knows which it is, in registers no data path exposes -- PSR (last
// error code, error-passive/bus-off state) and ECR (the error counters), plus
// TXBTO/TXBCF, which say whether a frame ever actually left the controller.
//
// This used to require building and flashing a libhcs_CAN_DIAG image, whose
// telemetry then rides DataId::kUart0 and corrupts anything measuring that
// channel. On 2026-09-04 that cost six reflashes and one false UART regression.
// It now runs against the shipping image.
//
// READING THE RESULT, in the order that narrows fastest:
//
//   LEC=ACK, TEC climbing      the frame WAS transmitted and nobody
//                              acknowledged it -- the transmitter and its
//                              transceiver work, the far end is not listening
//   LEC=BIT0                   drove dominant, read back recessive: the bus
//                              cannot be pulled low at all -- CAN_H/CAN_L open,
//                              swapped, or the transceiver is not enabled
//   LEC=STUFF/FORM/CRC/BIT1    bits arrive corrupted -- bit timing mismatch,
//                              missing termination, or noise
//   tx_ok=0 and tx_cancel!=0   nothing ever reached the wire: software, not
//                              wiring (the driver runs with automatic
//                              retransmission disabled, so a failed frame is
//                              cancelled rather than retried)
//
// A healthy bus that has been transmitting reads tx_ok=0xffffffff,
// tx_cancel=0x00000000; a bus whose frames reach nobody reads exactly the
// mirror image. Measured both ways on 2026-09-04.
//
//   can_bus_diag [seconds] [frames_per_tick] [bus_mask] [ab|a|b]
//     bus_mask: bit 0 = CAN1, bit 1 = CAN2

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <libhcs/board/hpm5321.hpp>

// CAN ports are named as the silkscreen on THAT board. 5321 prints CAN1/CAN2.
using libhcs::board::hcs::CanPort;

namespace {

using libhcs::board::AdvancedOptions;
using libhcs::board::Hpm5321;
using libhcs::board::hcs::last_error_name;

constexpr CanPort kPorts[] = {CanPort::kCan1, CanPort::kCan2};

class Board final : public Hpm5321::Callback {
public:
    Board(std::string_view serial, const char* label)
        : label_(label)
        , board_(*this, serial, options_) {}

    // bus_mask: bit 0 drives CAN1, bit 1 drives CAN2. Driving one bus alone is
    // what makes the other's registers a control group.
    void drive(uint32_t per_bus, unsigned bus_mask) {
        std::byte payload[8]{};
        auto builder = board_.start_transmit();
        for (uint32_t i = 0; i < per_bus; ++i) {
            if (bus_mask & 1U)
                builder.can_transmit(CanPort::kCan1, {.can_id = 0x300, .can_data = payload});
            if (bus_mask & 2U)
                builder.can_transmit(CanPort::kCan2, {.can_id = 0x301, .can_data = payload});
        }
    }

    void report() {
        printf("%s\n", label_);
        for (const CanPort port : kPorts) {
            const auto s = board_.can_status(port);
            printf(
                "    CAN%d  TEC=%-3u REC=%-3u LEC=%-9s DLEC=%-9s%s%s%s\n",
                static_cast<int>(port), s.tec, s.rec, last_error_name(s.last_error),
                last_error_name(s.data_last_error),
                (s.flags & libhcs::core::protocol::vendor_control::kCanErrorPassive)
                    ? "  ERR_PASSIVE" : "",
                (s.flags & libhcs::core::protocol::vendor_control::kCanWarning) ? "  WARNING" : "",
                (s.flags & libhcs::core::protocol::vendor_control::kCanBusOff) ? "  BUS_OFF" : "");
            printf(
                "          tx_ok=0x%08x tx_cancelled=0x%08x rx_frames=%-8u rx_fifo=%u  -> %s\n",
                s.tx_occurred, s.tx_cancelled, s.rx_frames, s.rx_fifo_level,
                verdict(s));
        }
    }

private:
    static const char* verdict(const libhcs::core::protocol::vendor_control::CanStatusPayload& s) {
        using LEC = libhcs::core::protocol::vendor_control::LastErrorCode;
        const auto lec = static_cast<LEC>(s.data_last_error == static_cast<uint8_t>(LEC::kNoChange)
                                              ? s.last_error
                                              : s.data_last_error);
        if (s.tx_occurred == 0 && s.tx_cancelled != 0)
            return "nothing ever reached the wire";
        if (lec == LEC::kAck)
            return "transmitted, nobody acknowledged -- far end not listening";
        if (lec == LEC::kBit0)
            return "bus cannot be driven dominant -- CAN_H/L open, swapped, or transceiver off";
        if (lec == LEC::kStuff || lec == LEC::kForm || lec == LEC::kCrc || lec == LEC::kBit1)
            return "corrupted bits -- bit timing, termination, or noise";
        return "healthy";
    }

    void can_receive(CanPort, const libhcs::data::CanDataView&) override {}

    AdvancedOptions options_;
    const char* label_;
    Hpm5321 board_;
};

} // namespace

int main(int argc, char** argv) {
    const int seconds = argc > 1 ? std::atoi(argv[1]) : 3;
    const uint32_t per_tick = argc > 2 ? static_cast<uint32_t>(std::atoi(argv[2])) : 1;
    const unsigned bus_mask = argc > 3 ? static_cast<unsigned>(std::atoi(argv[3])) : 3U;
    const std::string drivers = argc > 4 ? argv[4] : "ab";

    const char* serial_a = std::getenv("HCS_BOARD_A");
    const char* serial_b = std::getenv("HCS_BOARD_B");
    if (!serial_a) {
        printf("set HCS_BOARD_A (and optionally HCS_BOARD_B) to the board serial(s)\n");
        return 2;
    }

    // One board is a valid rig: with CAN1 and CAN2 wired to each other, a single
    // board is its own peer and the two buses' registers are the two ends of the
    // same wire -- exactly what a loopback fault needs.
    Board a{serial_a, "board A"};
    std::unique_ptr<Board> b;
    if (serial_b)
        b = std::make_unique<Board>(serial_b, "board B");
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    printf("driving%s%s from board(s) '%s' for %d s\n\n", (bus_mask & 1U) ? " CAN1(0x300)" : "",
           (bus_mask & 2U) ? " CAN2(0x301)" : "", drivers.c_str(), seconds);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{seconds};
    while (std::chrono::steady_clock::now() < deadline) {
        if (drivers.find('a') != std::string::npos)
            a.drive(per_tick, bus_mask);
        if (b && drivers.find('b') != std::string::npos)
            b->drive(per_tick, bus_mask);
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{300});

    a.report();
    if (b) {
        printf("\n");
        b->report();
    }
    return 0;
}
