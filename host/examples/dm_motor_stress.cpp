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

// DM (Damiao) motor stress test, on whichever board is connected (uses the
// board's primary CAN1 + UART0, so it runs on every project board).
//
// The bus carries 9 motors. Per the user's setup:
//   ids 1,2,3,4,5,6,9 are commanded with the trailing byte 0xCD
//   ids 7,8           are commanded with the trailing byte 0x49
// Every control frame is "00 00 00 00 00 00 00 <cmd>" (8 bytes, standard id =
// motor id). Before commanding we broadcast the enable frame
// "FF FF FF FF FF FF FF FC" to every motor.
//
// The control loop runs at 1 kHz and sends all 9 frames in a single USB packet
// per round (9000 frames/s). Each motor answers with a feedback frame on the
// master id (default 0); we count those to see whether the bridge keeps up and
// decode one as a sanity sample. Stop with Ctrl-C (motors are disabled on exit).

namespace {

constexpr std::array<uint32_t, 9> k_motor_ids = {1, 2, 3, 4, 5, 6, 7, 8, 9};

constexpr std::byte cmd_byte(uint32_t id) {
    return (id == 7 || id == 8) ? std::byte{0x49} : std::byte{0xCD};
}

constexpr std::array<std::byte, 8> k_enable = {
    std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF},
    std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFC}};
constexpr std::array<std::byte, 8> k_disable = {
    std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF},
    std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFD}};

// Control loop period: 1 kHz.
constexpr auto k_loop_period = std::chrono::microseconds{1000};

// Optional UART traffic on the same 1 kHz loop: one fixed message per round,
// batched into the same USB packet as the CAN frames. Set k_send_uart to false
// to stress CAN only.
constexpr bool k_send_uart = true;
constexpr std::array<std::byte, 16> k_uart_payload = {
    std::byte{'D'}, std::byte{'M'}, std::byte{'S'}, std::byte{'T'}, std::byte{'R'},
    std::byte{'E'}, std::byte{'S'}, std::byte{'S'}, std::byte{0},   std::byte{1},
    std::byte{2},   std::byte{3},   std::byte{4},   std::byte{5},   std::byte{6},
    std::byte{7}};

// DM-J4310-2EC linear mapping limits (model specific; adjust per motor model).
constexpr float k_pos_max = 12.5F;  // rad
constexpr float k_vel_max = 30.0F;  // rad/s
constexpr float k_tor_max = 10.0F;  // N*m

float uint_to_float(uint32_t value, float min, float max, uint32_t bits) {
    const float span = max - min;
    return static_cast<float>(value) / static_cast<float>((1U << bits) - 1U) * span + min;
}

std::atomic<bool> g_running{true};
void on_sigint(int) { g_running.store(false); }

} // namespace

class DmMotorStress : public examples::BoardReceiver {
public:
    void bind(examples::BoardSession* board) { board_ = board; }

    void broadcast(const std::array<std::byte, 8>& frame) {
        board_->transmit([&](examples::BoardTransmitter& tx) {
            for (const uint32_t id : k_motor_ids)
                tx.can(0, {.can_id = id, .can_data = frame});
        });
    }

    void send_enable() { broadcast(k_enable); }
    void send_disable() { broadcast(k_disable); }

    // One control round: all 9 commands batched into a single USB packet.
    void send_round() {
        board_->transmit([&](examples::BoardTransmitter& tx) {
            for (const uint32_t id : k_motor_ids) {
                const std::array<std::byte, 8> frame = {
                    std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0},
                    std::byte{0}, std::byte{0}, std::byte{0}, cmd_byte(id)};
                tx.can(0, {.can_id = id, .can_data = frame});
            }
            if (k_send_uart)
                tx.uart(0, {.uart_data = k_uart_payload, .idle_delimited = true});
        });
        tx_frames_.fetch_add(k_motor_ids.size(), std::memory_order::relaxed);
        if (k_send_uart) {
            uart_tx_msgs_.fetch_add(1, std::memory_order::relaxed);
            uart_tx_bytes_.fetch_add(k_uart_payload.size(), std::memory_order::relaxed);
        }
    }

    uint64_t take_tx() { return tx_frames_.exchange(0, std::memory_order::relaxed); }
    uint64_t take_rx() { return rx_frames_.exchange(0, std::memory_order::relaxed); }
    uint32_t take_motor_rx(unsigned id) {
        return rx_by_motor_[id & 0x0FU].exchange(0, std::memory_order::relaxed);
    }
    uint64_t take_uart_tx_msgs() { return uart_tx_msgs_.exchange(0, std::memory_order::relaxed); }
    uint64_t take_uart_tx_bytes() { return uart_tx_bytes_.exchange(0, std::memory_order::relaxed); }
    uint64_t take_uart_rx_msgs() { return uart_rx_msgs_.exchange(0, std::memory_order::relaxed); }
    uint64_t take_uart_rx_bytes() { return uart_rx_bytes_.exchange(0, std::memory_order::relaxed); }

    struct Sample {
        unsigned id, err;
        float pos, vel, torque;
        unsigned t_mos, t_rotor;
    };
    Sample last_sample() const {
        const uint64_t raw = last_sample_.load(std::memory_order::relaxed);
        const unsigned d0 = (raw >> 0) & 0xFFU;
        const unsigned pos_raw = (raw >> 8) & 0xFFFFU;
        const unsigned vel_raw = (raw >> 24) & 0xFFFU;
        const unsigned tor_raw = (raw >> 36) & 0xFFFU;
        const unsigned t_mos = (raw >> 48) & 0xFFU;
        const unsigned t_rotor = (raw >> 56) & 0xFFU;
        return {
            d0 & 0x0FU, d0 >> 4U, uint_to_float(pos_raw, -k_pos_max, k_pos_max, 16),
            uint_to_float(vel_raw, -k_vel_max, k_vel_max, 12),
            uint_to_float(tor_raw, -k_tor_max, k_tor_max, 12), t_mos, t_rotor};
    }

private:
    void on_can(int bus, const libhcs::data::CanDataView& data) override {
        (void)bus;
        rx_frames_.fetch_add(1, std::memory_order::relaxed);
        const auto bytes = data.can_data;
        if (bytes.size() < 8)
            return;

        auto at = [&](size_t i) { return std::to_integer<uint32_t>(bytes[i]); };
        const uint32_t d0 = at(0);
        rx_by_motor_[d0 & 0x0FU].fetch_add(1, std::memory_order::relaxed);

        // Pack the whole feedback frame into one 64-bit word for a lock-free
        // "latest sample" snapshot: d0 | pos[16] | vel[12] | torque[12] | temps.
        const uint32_t pos = (at(1) << 8) | at(2);
        const uint32_t vel = (at(3) << 4) | (at(4) >> 4);
        const uint32_t tor = ((at(4) & 0x0FU) << 8) | at(5);
        const uint64_t packed = (uint64_t{d0} << 0) | (uint64_t{pos} << 8)
            | (uint64_t{vel} << 24) | (uint64_t{tor} << 36) | (uint64_t{at(6)} << 48)
            | (uint64_t{at(7)} << 56);
        last_sample_.store(packed, std::memory_order::relaxed);
    }

    void on_uart(int port, const libhcs::data::UartDataView& data) override {
        (void)port;
        uart_rx_msgs_.fetch_add(1, std::memory_order::relaxed);
        uart_rx_bytes_.fetch_add(data.uart_data.size(), std::memory_order::relaxed);
    }

    std::atomic<uint64_t> tx_frames_{0};
    std::atomic<uint64_t> rx_frames_{0};
    std::array<std::atomic<uint32_t>, 16> rx_by_motor_{};
    std::atomic<uint64_t> last_sample_{0};
    std::atomic<uint64_t> uart_tx_msgs_{0};
    std::atomic<uint64_t> uart_tx_bytes_{0};
    std::atomic<uint64_t> uart_rx_msgs_{0};
    std::atomic<uint64_t> uart_rx_bytes_{0};

    examples::BoardSession* board_ = nullptr;
};

int main() {
    std::signal(SIGINT, on_sigint);

    printf("DM motor stress - auto-detecting board...\n");
    DmMotorStress agent;
    auto board = examples::connect_any(agent);
    if (!board) {
        fprintf(stderr, "No compatible board found.\n");
        return 1;
    }
    agent.bind(board.get());

    printf("Connected: %.*s. Enabling %zu motors...\n",
        static_cast<int>(board->name().size()), board->name().data(), k_motor_ids.size());
    for (int i = 0; i < 5 && g_running.load(); ++i) {
        agent.send_enable();
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }

    printf("Commanding at 1 kHz (9 frames/round = 9000 frame/s). Ctrl-C to stop.\n");
    auto next = std::chrono::steady_clock::now();
    auto last_report = next;
    while (g_running.load()) {
        agent.send_round();

        next += k_loop_period;
        const auto now = std::chrono::steady_clock::now();
        if (next < now)
            next = now; // fell behind: resync instead of bursting to catch up
        std::this_thread::sleep_until(next);

        if (now - last_report >= std::chrono::seconds{1}) {
            const double dt = std::chrono::duration<double>(now - last_report).count();
            last_report = now;
            const uint64_t tx = agent.take_tx();
            const uint64_t rx = agent.take_rx();
            const double rx_ratio =
                tx ? 100.0 * static_cast<double>(rx) / static_cast<double>(tx) : 0.0;
            printf(
                "TX %7.0f frame/s | RX %7.0f frame/s | rx/tx %5.1f%% | rx by id:",
                static_cast<double>(tx) / dt, static_cast<double>(rx) / dt, rx_ratio);
            for (const uint32_t id : k_motor_ids)
                printf(" %u:%u", id, agent.take_motor_rx(id));
            const auto s = agent.last_sample();
            printf(
                " | sample id=%u err=%u pos=%.2f vel=%.2f tau=%.2f Tmos=%u Trot=%u\n", s.id,
                s.err, s.pos, s.vel, s.torque, s.t_mos, s.t_rotor);

            if (k_send_uart) {
                printf(
                    "  UART  TX %6.0f msg/s %8.0f B/s | RX %6.0f msg/s %8.0f B/s\n",
                    static_cast<double>(agent.take_uart_tx_msgs()) / dt,
                    static_cast<double>(agent.take_uart_tx_bytes()) / dt,
                    static_cast<double>(agent.take_uart_rx_msgs()) / dt,
                    static_cast<double>(agent.take_uart_rx_bytes()) / dt);
            }
        }
    }

    printf("\nDisabling motors...\n");
    agent.send_disable();
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
    printf("Stopped.\n");
    return 0;
}
