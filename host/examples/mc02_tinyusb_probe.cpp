#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <print>
#include <span>
#include <string_view>
#include <thread>
#include <vector>

#include <libhcs/board/mc02.hpp>

namespace {

using Board = libhcs::board::Mc02;
using namespace std::chrono_literals;

struct UartEvent {
    std::vector<std::byte> payload;
    bool idle_delimited;
};

struct CanEvent {
    uint32_t id;
    std::vector<std::byte> payload;
    bool is_extended;
};

class Receiver final : public Board::Callback {
public:
    bool wait_uart(int port, std::span<const std::byte> expected) {
        std::unique_lock lock{mutex_};
        return condition_.wait_for(lock, 500ms, [&] {
            auto& queue = uart_[static_cast<size_t>(port)];
            while (!queue.empty()) {
                auto event = std::move(queue.front());
                queue.pop_front();
                if (event.idle_delimited && std::ranges::equal(event.payload, expected)) {
                    return true;
                }
            }
            return false;
        });
    }

    bool wait_can(int bus, uint32_t id, std::span<const std::byte> expected, bool is_extended) {
        std::unique_lock lock{mutex_};
        return condition_.wait_for(lock, 500ms, [&] {
            auto& queue = can_[static_cast<size_t>(bus)];
            while (!queue.empty()) {
                auto event = std::move(queue.front());
                queue.pop_front();
                if (event.id == id && std::ranges::equal(event.payload, expected)
                    && event.is_extended == is_extended) {
                    return true;
                }
            }
            return false;
        });
    }

private:
    void uart7_receive_callback(const libhcs::data::UartDataView& data) override {
        record_uart(0, data);
    }
    void uart10_receive_callback(const libhcs::data::UartDataView& data) override {
        record_uart(1, data);
    }
    void can2_receive_callback(const libhcs::data::CanDataView& data) override {
        record_can(1, data);
    }
    void can3_receive_callback(const libhcs::data::CanDataView& data) override {
        record_can(2, data);
    }

    void record_uart(int port, const libhcs::data::UartDataView& data) {
        {
            const std::scoped_lock lock{mutex_};
            uart_[static_cast<size_t>(port)].push_back({
                {data.uart_data.begin(), data.uart_data.end()},
                data.idle_delimited
            });
        }
        condition_.notify_all();
    }

    void record_can(int bus, const libhcs::data::CanDataView& data) {
        {
            const std::scoped_lock lock{mutex_};
            can_[static_cast<size_t>(bus)].push_back({
                data.can_id,
                {data.can_data.begin(), data.can_data.end()},
                data.is_extended_can_id
            });
        }
        condition_.notify_all();
    }

    std::mutex mutex_;
    std::condition_variable condition_;
    std::array<std::deque<UartEvent>, 2> uart_;
    std::array<std::deque<CanEvent>, 3> can_;
};

std::vector<std::byte> make_payload(size_t size, uint32_t sequence) {
    std::vector<std::byte> payload(size);
    for (size_t i = 0; i < size; ++i)
        payload[i] =
            static_cast<std::byte>((static_cast<size_t>(sequence) * 37U + i * 13U) & 0xFFU);
    return payload;
}

void transmit_uart(Board& board, int port, std::span<const std::byte> payload) {
    auto builder = board.start_transmit();
    const libhcs::data::UartDataView data{.uart_data = payload, .idle_delimited = true};
    if (port == 0)
        builder.uart7_transmit(data);
    else
        builder.uart10_transmit(data);
}

void configure_uart_pair(Board& board, uint32_t baudrate) {
    // EP0 reconfiguration: synchronous and read-back-verified, so unlike the
    // retired in-band config field nothing here needs a settle sleep before the
    // loopback traffic starts.
    board.configure_uart7(baudrate);
    board.configure_uart10(baudrate);
}

class UartRestore final {
public:
    explicit UartRestore(Board& board)
        : board_(board) {}

    UartRestore(const UartRestore&) = delete;
    UartRestore& operator=(const UartRestore&) = delete;

    void restore() noexcept {
        if (!active_)
            return;
        try {
            configure_uart_pair(board_, 115'200U);
        } catch (...) {
            (void)std::fputs(
                "warning: failed to restore UART loopback pair to 115200 baud\n", stderr);
        }
        active_ = false;
    }

    ~UartRestore() { restore(); }

private:
    Board& board_;
    bool active_ = true;
};

bool run_uart_case(
    Board& board, Receiver& receiver, int tx_port, int rx_port, size_t payload_size,
    uint32_t rounds) {
    uint32_t passed = 0;
    for (uint32_t sequence = 0; sequence < rounds; ++sequence) {
        const auto payload = make_payload(payload_size, sequence);
        transmit_uart(board, tx_port, payload);
        if (receiver.wait_uart(rx_port, payload))
            ++passed;
    }
    const size_t usb_transfer_size = payload_size + 2;
    const bool ok = passed == rounds;
    std::println(
        "{} -> {} payload={} USB={}: {}/{} {}", tx_port == 0 ? "UART7" : "UART10",
        rx_port == 0 ? "UART7" : "UART10", payload_size,
        usb_transfer_size, passed, rounds, ok ? "PASS" : "FAIL");
    return ok;
}

void transmit_can(Board& board, int bus, uint32_t id, std::span<const std::byte> payload,
    bool is_extended) {
    auto builder = board.start_transmit();
    const libhcs::data::CanDataView data{
        .can_id = id,
        .can_data = payload,
        .is_extended_can_id = is_extended,
    };
    if (bus == 1)
        builder.can2_transmit(data);
    else
        builder.can3_transmit(data);
}

bool run_can_case(
    Board& board, Receiver& receiver, int tx_bus, int rx_bus, bool is_extended,
    size_t payload_size, uint32_t rounds) {
    // The frame type is the TX bus's own compiled mode -- mc02 runs all three
    // buses CAN-FD -- so the case name reports what the wire actually carries.
    const bool bus_is_fd = tx_bus == 1 ? board.can2_is_fd() : board.can3_is_fd();
    uint32_t passed = 0;
    for (uint32_t sequence = 0; sequence < rounds; ++sequence) {
        const auto payload = make_payload(payload_size, sequence);
        const uint32_t id = is_extended ? 0x1234500U + sequence : 0x500U + (sequence & 0xFFU);
        transmit_can(board, tx_bus, id, payload, is_extended);
        if (receiver.wait_can(rx_bus, id, payload, is_extended))
            ++passed;
    }
    const bool ok = passed == rounds;
    std::println(
        "CAN{} -> CAN{} {} {} payload={}: {}/{} {}", tx_bus, rx_bus,
        bus_is_fd ? "FD+BRS" : "classic", is_extended ? "extended" : "standard", payload_size,
        passed, rounds, ok ? "PASS" : "FAIL");
    return ok;
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 3) {
        std::println(stderr, "usage: {} [USB serial filter] [uart|can]", argv[0]);
        return 2;
    }

    const std::string_view serial_filter = argc >= 2 ? argv[1] : std::string_view{};
    const std::string_view mode = argc == 3 ? argv[2] : std::string_view{};
    if (!mode.empty() && mode != "uart" && mode != "can") {
        std::println(stderr, "usage: {} [USB serial filter] [uart|can]", argv[0]);
        return 2;
    }
    Receiver receiver;
    Board board{receiver, serial_filter};
    bool ok = true;

    std::println("mc02 TinyUSB hardware probe");
    std::println("UART7 <-> UART10 loopback; CAN1=FDCAN2, CAN2=FDCAN3");

    if (mode != "can") {
        for (const size_t payload_size : {61U, 62U, 63U}) {
            ok &= run_uart_case(board, receiver, 0, 1, payload_size, 20);
            ok &= run_uart_case(board, receiver, 1, 0, payload_size, 20);
        }

        configure_uart_pair(board, 2'000'000U);
        UartRestore uart_restore{board};
        ok &= run_uart_case(board, receiver, 0, 1, 62, 200);
        ok &= run_uart_case(board, receiver, 1, 0, 62, 200);
        uart_restore.restore();
    }

    if (mode != "uart") {
        for (const bool is_extended : {false, true}) {
            for (const size_t payload_size : {0U, 8U}) {
                ok &= run_can_case(board, receiver, 1, 2, is_extended, payload_size, 100);
                ok &= run_can_case(board, receiver, 2, 1, is_extended, payload_size, 100);
            }
        }
    }

    std::println("mc02 TinyUSB probe: {}", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
