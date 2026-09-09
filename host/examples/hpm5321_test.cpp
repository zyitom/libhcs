#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <thread>

#include "common/multi_board.hpp"

// Non-interactive sanity tester for any project board. After auto-detecting the
// connected board it keeps sending one CAN frame and one UART message every
// period (on the board's primary CAN1 / UART0) and prints whatever it receives
// back. Works on every board, since all of them expose at least one CAN bus and
// one UART. Stop with Ctrl-C.

namespace {

constexpr auto k_send_period = std::chrono::milliseconds{500};

std::atomic<bool> g_running{true};

void on_sigint(int) { g_running.store(false); }

void print_bytes(std::span<const std::byte> bytes) {
    for (const std::byte b : bytes)
        printf("%02X ", static_cast<unsigned>(std::to_integer<uint8_t>(b)));
}

} // namespace

class Tester : public examples::BoardReceiver {
public:
    void bind(examples::BoardSession* board) { board_ = board; }

    void send_round(uint32_t counter) {
        // CAN1: id 0x123, 4 payload bytes carrying the counter (big-endian).
        const std::byte can_payload[4] = {
            static_cast<std::byte>(counter >> 24), static_cast<std::byte>(counter >> 16),
            static_cast<std::byte>(counter >> 8), static_cast<std::byte>(counter)};
        board_->transmit([&](examples::BoardTransmitter& tx) {
            tx.can(0, {.can_id = 0x123, .can_data = can_payload});
        });

        // UART0: a short ASCII line carrying the counter.
        const std::string text = "hello " + std::to_string(counter) + "\n";
        board_->transmit([&](examples::BoardTransmitter& tx) {
            tx.uart(0, {.uart_data = std::as_bytes(std::span{text.data(), text.size()}),
                        .idle_delimited = true});
        });

        printf("[TX #%u] CAN1 id=0x123 + UART0 \"hello %u\"\n", counter, counter);
    }

private:
    void on_can(int bus, const libhcs::data::CanDataView& data) override {
        printf(
            "[CAN%d RX] id=0x%X%s dlc=%zu data=", bus, data.can_id,
            data.is_extended_can_id ? " ext" : "", data.can_data.size());
        print_bytes(data.can_data);
        printf("\n");
    }

    void on_uart(int port, const libhcs::data::UartDataView& data) override {
        printf("[UART%d RX] %zu bytes: ", port, data.uart_data.size());
        print_bytes(data.uart_data);
        printf("| \"");
        for (const std::byte b : data.uart_data) {
            const auto c = std::to_integer<unsigned char>(b);
            putchar((c >= 0x20 && c < 0x7F) ? static_cast<char>(c) : '.');
        }
        printf("\"\n");
    }

    examples::BoardSession* board_ = nullptr;
};

int main() {
    std::signal(SIGINT, on_sigint);

    printf("CAN/UART tester - auto-detecting board...\n");
    Tester tester;
    auto board = examples::connect_any(tester);
    if (!board) {
        fprintf(stderr, "No compatible board found.\n");
        return 1;
    }
    tester.bind(board.get());
    printf("Connected: %.*s. Sending CAN1 + UART0 every %lldms. Ctrl-C to stop.\n",
        static_cast<int>(board->name().size()), board->name().data(),
        static_cast<long long>(k_send_period.count()));

    for (uint32_t counter = 0; g_running.load(); ++counter) {
        tester.send_round(counter);
        std::this_thread::sleep_for(k_send_period);
    }

    printf("\nStopped.\n");
    return 0;
}
