// CAN error light-code test for any board with two CAN buses
// (hpm5321_dual_can, c_board, mc02).
//
// Continuously transmits a frame on CAN bus 0 and bus 1 so the board is always
// driving the bus -- CAN errors only arise while a controller is actually
// transmitting/receiving, so an idle bus reports nothing.  Use this to verify
// the indicator-LED light language:
//
//   off          = healthy / no errors
//   1 pulse      = ACK error    (no node ACKing -- nothing connected on the bus)
//   2 pulses     = stuff error  (baudrate mismatch / heavy noise)
//   3 pulses     = form error   (frame format violation)
//   4 pulses     = bit error    (TX vs monitored mismatch -- CAN_H/CAN_L shorted)
//   5 pulses     = CRC error    (signal integrity)
//   fast blink   = bus-off      (controller offline after too many errors)
//
// Quick checks:
//   * Leave a bus disconnected            -> that LED should show 1 pulse (ACK).
//   * Short CAN_H to CAN_L on a bus        -> that LED should show a bit error.
// Stop with Ctrl-C.

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "common/multi_board.hpp"

namespace {

constexpr unsigned kBusCount = 2;        // CAN bus 0 + bus 1
// No classic/FD switch any more: frames go out in the bus's compiled mode.
// Must be unique on the bus: two transmitters sending the SAME id with different
// data collide after arbitration -> bit errors -> SIGNAL (double blink).  Pick an
// id no other node (e.g. the USB2CAN) uses.
constexpr uint32_t kTestCanId = 0x321;
constexpr auto kSendPeriod = std::chrono::milliseconds{5};  // ~200 Hz per bus

std::atomic<bool> g_running{true};
void on_sigint(int) { g_running.store(false); }

}  // namespace

class CanErrorTest : public examples::BoardReceiver {
public:
    void bind(examples::BoardSession* board) { board_ = board; }

    void send_test_frame(unsigned bus) {
        std::array<std::byte, 8> frame{
            std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF},
            std::byte{0x00}, std::byte{0x11}, std::byte{0x22}, std::byte{0x33}};
        board_->transmit([&](examples::BoardTransmitter& tx) {
            tx.can(static_cast<int>(bus),
                {.can_id = kTestCanId, .can_data = frame});
        });
    }

    uint64_t rx_count(unsigned bus) const { return rx_count_[bus].load(); }

private:
    void on_can(int bus, const libhcs::data::CanDataView&) override {
        if (bus >= 0 && bus < static_cast<int>(kBusCount))
            ++rx_count_[bus];
    }

    std::atomic<uint64_t> rx_count_[kBusCount]{};

    examples::BoardSession* board_ = nullptr;
};

int main() {
    std::signal(SIGINT, on_sigint);

    printf("CAN error light-code test (frames follow the bus mode)\n");
    printf("  Transmitting id 0x%03X on CAN bus 0 and bus 1 at ~200 Hz.\n", kTestCanId);
    printf("  Watch the per-bus indicator LEDs:\n");
    printf("    disconnected bus -> 1 pulse (ACK error)\n");
    printf("    CAN_H/CAN_L short -> bit error\n");
    printf("  Ctrl-C to stop.\n");
    printf("Connecting ...\n");

    CanErrorTest agent;
    auto board = examples::connect_any(agent);
    if (!board) {
        fprintf(stderr, "No compatible board found.\n");
        return 1;
    }
    if (board->can_bus_count() < 2) {
        fprintf(stderr, "%.*s has only %d CAN bus; this test needs two.\n",
            static_cast<int>(board->name().size()), board->name().data(),
            board->can_bus_count());
        return 1;
    }
    agent.bind(board.get());
    printf("Connected: %.*s.  Sending ...\n",
        static_cast<int>(board->name().size()), board->name().data());

    std::array<uint64_t, kBusCount> tx_count{};
    auto next_send = std::chrono::steady_clock::now();
    auto last_report = next_send;

    while (g_running.load()) {
        const auto now = std::chrono::steady_clock::now();

        for (unsigned i = 0; i < kBusCount; ++i) {
            agent.send_test_frame(i);
            ++tx_count[i];
        }

        next_send += kSendPeriod;
        if (next_send < now)
            next_send = now;
        std::this_thread::sleep_until(next_send);

        if (now - last_report >= std::chrono::seconds{1}) {
            last_report = now;
            printf("CAN1: tx %lu rx %lu | CAN2: tx %lu rx %lu\n", tx_count[0],
                   agent.rx_count(0), tx_count[1], agent.rx_count(1));
            tx_count = {};
        }
    }

    printf("\nStopped.\n");
    return 0;
}
