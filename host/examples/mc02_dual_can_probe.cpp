#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <exception>
#include <mutex>
#include <optional>
#include <print>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <libhcs/board/mc02.hpp>

namespace {

using Board = libhcs::board::Mc02;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

struct CanEvent {
    uint32_t id;
    std::vector<std::byte> payload;
    bool is_extended;
    Clock::time_point arrival;
};

class Receiver final : public Board::Callback {
public:
    std::optional<double> wait_can(
        int bus, uint32_t id, std::span<const std::byte> expected, bool is_extended,
        Clock::time_point sent, std::chrono::milliseconds timeout = 500ms) {
        std::unique_lock lock{mutex_};
        std::optional<double> latency_us;
        condition_.wait_for(lock, timeout, [&] {
            auto& queue = queues_[static_cast<size_t>(bus)];
            while (!queue.empty()) {
                auto event = std::move(queue.front());
                queue.pop_front();
                if (event.id == id && std::ranges::equal(event.payload, expected)
                    && event.is_extended == is_extended) {
                    latency_us =
                        std::chrono::duration<double, std::micro>{event.arrival - sent}.count();
                    return true;
                }
            }
            return false;
        });
        return latency_us;
    }

private:
    void can1_receive_callback(const libhcs::data::CanDataView& data) override { record(1, data); }
    void can2_receive_callback(const libhcs::data::CanDataView& data) override { record(2, data); }
    void can3_receive_callback(const libhcs::data::CanDataView& data) override { record(3, data); }

    void record(int bus, const libhcs::data::CanDataView& data) {
        const auto arrival = Clock::now();
        {
            const std::scoped_lock lock{mutex_};
            queues_[static_cast<size_t>(bus)].push_back({
                data.can_id,
                {data.can_data.begin(), data.can_data.end()},
                data.is_extended_can_id,
                arrival,
            });
        }
        condition_.notify_all();
    }

    std::mutex mutex_;
    std::condition_variable condition_;
    std::array<std::deque<CanEvent>, 4> queues_;
};

std::vector<std::byte> make_payload(size_t size, uint32_t sequence) {
    std::vector<std::byte> payload(size);
    for (size_t i = 0; i < size; ++i)
        payload[i] =
            static_cast<std::byte>((static_cast<size_t>(sequence) * 37U + i * 13U) & 0xFFU);
    return payload;
}

void transmit_can(
    Board& board, int bus, uint32_t id, std::span<const std::byte> payload, bool is_extended) {
    auto builder = board.start_transmit();
    const libhcs::data::CanDataView data{
        .can_id = id,
        .can_data = payload,
        .is_extended_can_id = is_extended,
    };
    switch (bus) {
    case 1: builder.can1_transmit(data); break;
    case 2: builder.can2_transmit(data); break;
    case 3: builder.can3_transmit(data); break;
    default: std::unreachable();
    }
}

bool run_direction(
    std::string_view tx_label, Board& tx_board, std::string_view rx_label, Receiver& rx_receiver,
    int tx_bus, int rx_bus, bool is_extended, size_t payload_size, uint32_t rounds) {
    // The frame type is the TX bus's own compiled mode (mc02 runs all three
    // buses CAN-FD); the case name reports what the wire actually carries.
    const bool bus_is_fd = tx_bus == 1   ? tx_board.can1_is_fd()
                           : tx_bus == 2 ? tx_board.can2_is_fd()
                                         : tx_board.can3_is_fd();
    uint32_t passed = 0;
    for (uint32_t sequence = 0; sequence < rounds; ++sequence) {
        const auto payload = make_payload(payload_size, sequence);
        const uint32_t id = is_extended ? 0x1234500U + sequence : 0x500U + sequence;
        const auto sent = Clock::now();
        transmit_can(tx_board, tx_bus, id, payload, is_extended);
        if (!rx_receiver.wait_can(rx_bus, id, payload, is_extended, sent))
            break;
        ++passed;
    }

    const bool ok = passed == rounds;
    std::println(
        "{}.CAN{} -> {}.CAN{} {} {} payload={}: {}/{} {}", tx_label, tx_bus, rx_label, rx_bus,
        bus_is_fd ? "FD+BRS" : "classic", is_extended ? "extended" : "standard", payload_size,
        passed, rounds, ok ? "PASS" : "FAIL");
    return ok;
}

void configure_current_thread(int cpu, int priority, const char* name) noexcept {
    (void)pthread_setname_np(pthread_self(), name);
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0)
        std::println(stderr, "warning: failed to pin {} to CPU {}", name, cpu);

    sched_param parameter{};
    parameter.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameter) != 0)
        std::println(stderr, "warning: failed to set {} to FIFO {}", name, priority);
}

double percentile(const std::vector<double>& sorted, double fraction) {
    if (sorted.empty())
        return 0.0;
    const auto index = static_cast<size_t>(fraction * static_cast<double>(sorted.size() - 1));
    return sorted[index];
}

bool run_latency_direction(
    std::string_view tx_label, Board& tx_board, std::string_view rx_label, Receiver& rx_receiver,
    int tx_bus, int rx_bus, uint32_t rounds, std::chrono::microseconds inter_round_gap) {
    constexpr uint32_t k_warmup_rounds = 100;
    constexpr uint32_t k_can_id = 0x5A0;
    std::vector<double> samples;
    samples.reserve(rounds);
    uint32_t lost = 0;
    uint32_t consecutive_lost = 0;

    for (uint32_t sequence = 0; sequence < k_warmup_rounds + rounds; ++sequence) {
        const auto payload = make_payload(8, sequence);
        const auto sent = Clock::now();
        transmit_can(tx_board, tx_bus, k_can_id, payload, false);
        const auto latency = rx_receiver.wait_can(rx_bus, k_can_id, payload, false, sent, 10ms);
        if (!latency) {
            if (sequence >= k_warmup_rounds)
                ++lost;
            if (++consecutive_lost == 10)
                break;
        } else {
            consecutive_lost = 0;
            if (sequence >= k_warmup_rounds)
                samples.push_back(*latency);
        }
        if (inter_round_gap.count() > 0)
            std::this_thread::sleep_for(inter_round_gap);
    }

    if (samples.empty()) {
        std::println(
            "{}.CAN{} -> {}.CAN{}: no samples ({} lost)", tx_label, tx_bus, rx_label, rx_bus, lost);
        return false;
    }

    std::ranges::sort(samples);
    double sum = 0.0;
    for (const double sample : samples)
        sum += sample;
    const double average = sum / static_cast<double>(samples.size());
    std::println(
        "{}.CAN{} -> {}.CAN{} FD+BRS 8B: n={} lost={} "
        "min/p50/avg/p90/p99/p99.9/max={:.1f}/{:.1f}/{:.1f}/{:.1f}/{:.1f}/{:.1f}/{:.1f} us",
        tx_label, tx_bus, rx_label, rx_bus, samples.size(), lost, samples.front(),
        percentile(samples, 0.5), average, percentile(samples, 0.9), percentile(samples, 0.99),
        percentile(samples, 0.999), samples.back());
    return lost == 0;
}

} // namespace

int main(int argc, char** argv) {
    const std::string_view mode = argc >= 4 ? std::string_view{argv[3]} : std::string_view{};
    if (argc < 3 || argc > 5 || (!mode.empty() && mode != "latency" && mode != "stress")) {
        std::println(
            stderr, "usage: {} <board-A serial> <board-B serial> [latency|stress [rounds]]",
            argv[0]);
        return 2;
    }

    try {
        const bool measurement_mode = !mode.empty();
        const uint32_t default_rounds = mode == "stress" ? 50'000U : 5000U;
        const uint32_t rounds =
            argc == 5 ? static_cast<uint32_t>(std::strtoul(argv[4], nullptr, 10)) : default_rounds;
        if (measurement_mode && rounds == 0) {
            std::println(stderr, "rounds must be greater than zero");
            return 2;
        }

        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
            std::println(stderr, "warning: mlockall failed");
        configure_current_thread(6, 80, "mc02-probe");

        Receiver receiver_a;
        Receiver receiver_b;
        auto options_a = libhcs::board::bind_advanced_options(
            []() noexcept { configure_current_thread(1, 90, "mc02-usb-a"); });
        auto options_b = libhcs::board::bind_advanced_options(
            []() noexcept { configure_current_thread(3, 90, "mc02-usb-b"); });
        Board board_a{receiver_a, std::string_view{argv[1]}, options_a};
        Board board_b{receiver_b, std::string_view{argv[2]}, options_b};
        bool ok = true;

        std::println("mc02 dual-board CAN probe");
        std::println("A CAN1 <-> B CAN1; A CAN3 <-> B CAN2");
        if (measurement_mode) {
            const auto inter_round_gap = mode == "stress" ? 0us : 1000us;
            std::println(
                "{} mode: {} measured rounds after 100 warmups, gap={} us", mode, rounds,
                inter_round_gap.count());
            ok &=
                run_latency_direction("A", board_a, "B", receiver_b, 1, 1, rounds, inter_round_gap);
            ok &=
                run_latency_direction("B", board_b, "A", receiver_a, 1, 1, rounds, inter_round_gap);
            ok &=
                run_latency_direction("A", board_a, "B", receiver_b, 3, 2, rounds, inter_round_gap);
            ok &=
                run_latency_direction("B", board_b, "A", receiver_a, 2, 3, rounds, inter_round_gap);
            std::println("mc02 dual-board CAN {} probe: {}", mode, ok ? "PASS" : "FAIL");
            return ok ? 0 : 1;
        }

        for (const bool is_extended : {false, true}) {
            for (const size_t payload_size : {0U, 8U}) {
                ok &= run_direction(
                    "A", board_a, "B", receiver_b, 1, 1, is_extended, payload_size, 50);
                ok &= run_direction(
                    "B", board_b, "A", receiver_a, 1, 1, is_extended, payload_size, 50);
                ok &= run_direction(
                    "A", board_a, "B", receiver_b, 3, 2, is_extended, payload_size, 50);
                ok &= run_direction(
                    "B", board_b, "A", receiver_a, 2, 3, is_extended, payload_size, 50);
            }
        }

        std::println("mc02 dual-board CAN probe: {}", ok ? "PASS" : "FAIL");
        return ok ? 0 : 1;
    } catch (const std::exception& exception) {
        std::println(stderr, "mc02 dual-board CAN probe: {}", exception.what());
        return 2;
    }
}
