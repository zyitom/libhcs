#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <thread>

#include "common/multi_board.hpp"

// UART-only stress test for any project board.
//
// Auto-detects the connected board and sends one UART0 message per loop tick,
// counting both directions, so you can see how many UART messages/bytes per
// second the bridge sustains (and, with libhcs_USB_STATS=1, how that maps onto
// USB bulk traffic). No CAN traffic is generated. Stop with Ctrl-C.

namespace {

// Loop period (1 kHz) and per-message payload. At 921600 baud the UART caps at
// ~92 KB/s, so keep payload * rate under that to avoid backpressure.
constexpr auto k_loop_period = std::chrono::microseconds{1000};
constexpr std::size_t k_payload_size = 32;

std::atomic<bool> g_running{true};
void on_sigint(int) { g_running.store(false); }

} // namespace

class UartStress : public examples::BoardReceiver {
public:
    void bind(examples::BoardSession* board) { board_ = board; }

    // One message: a 16-bit big-endian counter followed by an incrementing
    // pattern, so the receiver side could detect gaps or corruption if echoed.
    void send_round(uint16_t counter) {
        std::array<std::byte, k_payload_size> payload{};
        payload[0] = static_cast<std::byte>(counter >> 8);
        payload[1] = static_cast<std::byte>(counter);
        for (std::size_t i = 2; i < payload.size(); ++i)
            payload[i] = static_cast<std::byte>(i);

        board_->transmit([&](examples::BoardTransmitter& tx) {
            tx.uart(0, {.uart_data = payload, .idle_delimited = true});
        });
        tx_msgs_.fetch_add(1, std::memory_order::relaxed);
        tx_bytes_.fetch_add(payload.size(), std::memory_order::relaxed);
    }

    uint64_t take_tx_msgs() { return tx_msgs_.exchange(0, std::memory_order::relaxed); }
    uint64_t take_tx_bytes() { return tx_bytes_.exchange(0, std::memory_order::relaxed); }
    uint64_t take_rx_msgs() { return rx_msgs_.exchange(0, std::memory_order::relaxed); }
    uint64_t take_rx_bytes() { return rx_bytes_.exchange(0, std::memory_order::relaxed); }

private:
    void on_uart(int port, const libhcs::data::UartDataView& data) override {
        (void)port;
        rx_msgs_.fetch_add(1, std::memory_order::relaxed);
        rx_bytes_.fetch_add(data.uart_data.size(), std::memory_order::relaxed);
    }

    std::atomic<uint64_t> tx_msgs_{0};
    std::atomic<uint64_t> tx_bytes_{0};
    std::atomic<uint64_t> rx_msgs_{0};
    std::atomic<uint64_t> rx_bytes_{0};

    examples::BoardSession* board_ = nullptr;
};

int main() {
    std::signal(SIGINT, on_sigint);

    printf("UART stress - auto-detecting board...\n");
    UartStress agent;
    auto board = examples::connect_any(agent);
    if (!board) {
        fprintf(stderr, "No compatible board found.\n");
        return 1;
    }
    agent.bind(board.get());
    printf(
        "Connected: %.*s. Sending %zu-byte UART0 messages at 1 kHz. Ctrl-C to stop.\n",
        static_cast<int>(board->name().size()), board->name().data(), k_payload_size);

    auto next = std::chrono::steady_clock::now();
    auto last_report = next;
    for (uint16_t counter = 0; g_running.load(); ++counter) {
        agent.send_round(counter);

        next += k_loop_period;
        const auto now = std::chrono::steady_clock::now();
        if (next < now)
            next = now; // fell behind: resync instead of bursting to catch up
        std::this_thread::sleep_until(next);

        if (now - last_report >= std::chrono::seconds{1}) {
            const double dt = std::chrono::duration<double>(now - last_report).count();
            last_report = now;
            const uint64_t tx_msgs = agent.take_tx_msgs();
            const uint64_t rx_msgs = agent.take_rx_msgs();
            const double rx_ratio = tx_msgs ? 100.0 * static_cast<double>(rx_msgs)
                    / static_cast<double>(tx_msgs)
                                            : 0.0;
            printf(
                "UART TX %6.0f msg/s %8.0f B/s | RX %6.0f msg/s %8.0f B/s | rx/tx %5.1f%%\n",
                static_cast<double>(tx_msgs) / dt, static_cast<double>(agent.take_tx_bytes()) / dt,
                static_cast<double>(rx_msgs) / dt, static_cast<double>(agent.take_rx_bytes()) / dt,
                rx_ratio);
        }
    }

    printf("\nStopped.\n");
    return 0;
}
