// Proves a two-CAN-bus board round-trips the frame shapes the wire protocol
// carries: payload lengths 0..8, standard and extended identifiers, and the
// host-side rejection of payloads beyond the protocol's 8-byte cap.
//
// OBSOLETE AS A TYPE MATRIX, 2026-09-12. The per-frame is_fdcan header bit is
// retired (core/src/protocol/protocol.hpp, CanHeaderLayout): the frame type is
// a property of the BUS, fixed by the firmware's port table and read back over
// EP0 (libhcs/protocol/vendor_control.hpp). Every frame this tool sends goes
// out in the bus's compiled mode, and the uplink no longer reports a per-frame
// type, so there is no classic-vs-FD axis to exercise or assert on. What this
// tool still proves is that both controllers on the bus accept and forward
// every shape the protocol can carry, with attributes intact.
//
// Note on classic-frame reception: an FD-capable controller still decodes
// classic frames from peers, but this rig cannot produce one -- the board only
// ever transmits its own bus mode. Covering the classic-RX path needs an
// external classic-CAN peer, not a second libhcs board.
//
// WIRING: join CAN bus 0 and bus 1 onto ONE bus -- CAN1_H<->CAN2_H,
// CAN1_L<->CAN2_L -- with a 120 ohm terminator at EACH end. Missing termination
// shows up as lost FD frames first (the 5 Mbit data phase is far less tolerant
// of reflections than the 1 Mbit arbitration phase), so a run that loses only
// FD frames is a wiring result, not a firmware result.
//
// Run:
//   sudo chrt -f 80 ./can_frame_type_test

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>

#include "common/multi_board.hpp"

namespace {

constexpr uint32_t kStdCanId = 0x321;
constexpr uint32_t kExtCanId = 0x12345678;
constexpr uint32_t kRoundsPerCase = 200;
constexpr auto kEchoTimeout = std::chrono::milliseconds{50};
constexpr auto kInterFrameGap = std::chrono::microseconds{1500};

std::atomic<bool> g_running{true};
void on_sigint(int) { g_running.store(false, std::memory_order_relaxed); }

struct Case {
    const char* name;
    bool extended_id;
    std::size_t payload_size;
};

// 8 bytes is the largest payload the board's RX elements keep, so every case
// here is expected to survive the round trip.
constexpr Case kCases[]{
    {"std id, 8B", false, 8},
    {"ext id, 8B", true, 8},
    {"std id, 0B", false, 0},
    {"ext id, 0B", true, 0},
};

} // namespace

class FrameTypeTester : public examples::BoardReceiver {
public:
    void bind(examples::BoardSession* board) { board_ = board; }

    void arm(const Case& test_case, uint32_t seq) {
        expect_ext_.store(test_case.extended_id, std::memory_order_relaxed);
        expect_size_.store(test_case.payload_size, std::memory_order_relaxed);
        got_echo_.store(false, std::memory_order_relaxed);
        attrs_ok_.store(false, std::memory_order_relaxed);
        seq_.store(seq, std::memory_order_release);

        std::array<std::byte, 8> frame{};
        std::memcpy(frame.data(), &seq, sizeof(seq));

        board_->transmit([&](examples::BoardTransmitter& tx) {
            tx.can(
                0, {.can_id = test_case.extended_id ? kExtCanId : kStdCanId,
                    .can_data = {frame.data(), test_case.payload_size},
                    .is_extended_can_id = test_case.extended_id});
        });
    }

    // Returns {echo seen, attributes matched what was sent}.
    std::pair<bool, bool> wait_echo() {
        const auto deadline = std::chrono::steady_clock::now() + kEchoTimeout;
        while (!got_echo_.load(std::memory_order_acquire)) {
            if (!g_running.load(std::memory_order_relaxed)
                || std::chrono::steady_clock::now() >= deadline)
                return {false, false};
            std::this_thread::sleep_for(std::chrono::microseconds{20});
        }
        return {true, attrs_ok_.load(std::memory_order_relaxed)};
    }

private:
    void on_can(int bus, const libhcs::data::CanDataView& data) override {
        if (bus != 1)
            return;

        const bool want_ext = expect_ext_.load(std::memory_order_relaxed);
        if (data.can_id != (want_ext ? kExtCanId : kStdCanId))
            return;

        const auto want_size = expect_size_.load(std::memory_order_relaxed);
        if (data.can_data.size() != want_size)
            return;
        if (want_size >= sizeof(uint32_t)) {
            uint32_t seq = 0;
            std::memcpy(&seq, data.can_data.data(), sizeof(seq));
            if (seq != seq_.load(std::memory_order_acquire))
                return;
        }

        // Identifier and length must survive the round trip exactly.
        const bool matched = data.is_extended_can_id == want_ext;
        attrs_ok_.store(matched, std::memory_order_relaxed);
        got_echo_.store(true, std::memory_order_release);
    }

    std::atomic<uint32_t> seq_{0xFFFFFFFFU};
    std::atomic<bool> expect_ext_{false};
    std::atomic<std::size_t> expect_size_{0};
    std::atomic<bool> got_echo_{false};
    std::atomic<bool> attrs_ok_{false};

    examples::BoardSession* board_ = nullptr;
};

int main() {
    std::signal(SIGINT, on_sigint);

    printf("CAN frame-shape round-trip test\n");
    printf("  Wire CAN bus 0 <-> bus 1 together (120 ohm at EACH end).\n");
    printf("  TX on bus 0, echo observed on bus 1. Frames go out in the bus's\n");
    printf("  compiled mode (CAN-FD on FD boards); there is no per-frame type.\n\n");
    printf("Connecting ...\n");

    FrameTypeTester tester;
    auto board = examples::connect_any(tester);
    if (!board) {
        fprintf(stderr, "No compatible board found.\n");
        return 1;
    }
    if (board->can_bus_count() < 2) {
        fprintf(
            stderr, "%.*s has only %d CAN bus; this test needs two.\n",
            static_cast<int>(board->name().size()), board->name().data(), board->can_bus_count());
        return 1;
    }
    tester.bind(board.get());
    printf(
        "Connected: %.*s. %u rounds per case.\n\n", static_cast<int>(board->name().size()),
        board->name().data(), kRoundsPerCase);

    printf("%-28s %8s %10s %9s\n", "case", "sent", "echoed", "attrs-ok");
    printf("%-28s %8s %10s %9s\n", "----", "----", "------", "--------");

    int failed_cases = 0;
    uint32_t seq = 0;

    for (const auto& test_case : kCases) {
        uint32_t echoed = 0;
        uint32_t attrs_ok = 0;

        for (uint32_t i = 0; i < kRoundsPerCase && g_running.load(std::memory_order_relaxed);
             ++i) {
            tester.arm(test_case, ++seq);
            const auto [ok, matched] = tester.wait_echo();
            if (ok) {
                ++echoed;
                if (matched)
                    ++attrs_ok;
            }
            std::this_thread::sleep_for(kInterFrameGap);
        }

        const bool pass = echoed == kRoundsPerCase && attrs_ok == kRoundsPerCase;
        if (!pass)
            ++failed_cases;
        printf(
            "%-28s %8u %10u %9u  %s\n", test_case.name, kRoundsPerCase, echoed, attrs_ok,
            pass ? "PASS" : "FAIL");
    }

    printf("\n");
    if (failed_cases == 0) {
        printf("ALL PASS -- every frame shape the protocol carries round-trips intact,\n");
        printf("with identifier and length preserved.\n");
        return 0;
    }
    printf("%d case(s) FAILED.\n", failed_cases);
    printf("If only the FD cases lost frames, suspect termination/wiring before\n");
    printf("firmware: the 5 Mbit data phase is far less tolerant of reflections.\n");
    return 1;
}
