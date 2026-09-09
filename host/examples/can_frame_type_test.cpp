// Proves a two-CAN-bus board receives BOTH classic CAN 2.0 and CAN-FD frames
// without any mode switch, and that it reports each frame's real type back to
// the host.
//
// OBSOLETE FOR hpm_board AS OF 2026-09-04. Its transmit half depends on the
// per-frame is_fdcan flag, and hpm_board firmware no longer reads that bit:
// the frame type moved to EP0 as a per-bus setting, so an FD bus now sends
// every frame as FD (firmware can/can.cpp, libhcs/protocol/vendor_control.hpp).
// The three "classic 2.0" cases below can therefore never pass against an
// hpm_board -- the board answers in FD regardless of what was asked. The tool
// still means what it says on mc02 / c_board / ch32_board, whose firmware
// continues to honour the per-frame bit.
//
// Why this is worth testing: the controllers are held in CAN-FD mode
// permanently (CanMode::kCanFd in the board's kCanPorts), but FD mode is a
// strict superset -- an FD-enabled M_CAN decodes the FDF bit per frame and
// accepts classic frames too. Transmission is the opposite: the frame type
// comes from the host's per-frame is_fdcan flag, which defaults to false, so a
// frame is classic unless the caller asks for FD. This tool exercises the whole
// matrix in one run.
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
    bool send_fd;
    bool extended_id;
    std::size_t payload_size;
};

// 8 bytes is the largest payload the board's RX elements keep for either frame
// type, so every case here is expected to survive the round trip.
constexpr Case kCases[]{
    {"classic 2.0, std id, 8B", false, false, 8},
    {"classic 2.0, ext id, 8B", false, true, 8},
    {"classic 2.0, std id, 0B", false, false, 0},
    {"CAN-FD (BRS), std id, 8B", true, false, 8},
    {"CAN-FD (BRS), ext id, 8B", true, true, 8},
    {"CAN-FD (BRS), std id, 0B", true, false, 0},
};

} // namespace

class FrameTypeTester : public examples::BoardReceiver {
public:
    void bind(examples::BoardSession* board) { board_ = board; }

    void arm(const Case& test_case, uint32_t seq) {
        expect_fd_.store(test_case.send_fd, std::memory_order_relaxed);
        expect_ext_.store(test_case.extended_id, std::memory_order_relaxed);
        expect_size_.store(test_case.payload_size, std::memory_order_relaxed);
        got_echo_.store(false, std::memory_order_relaxed);
        type_ok_.store(false, std::memory_order_relaxed);
        seq_.store(seq, std::memory_order_release);

        std::array<std::byte, 8> frame{};
        std::memcpy(frame.data(), &seq, sizeof(seq));

        board_->transmit([&](examples::BoardTransmitter& tx) {
            tx.can(
                0, {.can_id = test_case.extended_id ? kExtCanId : kStdCanId,
                    .can_data = {frame.data(), test_case.payload_size},
                    .is_fdcan = test_case.send_fd,
                    .is_extended_can_id = test_case.extended_id});
        });
    }

    // Returns {echo seen, reported type matched what was sent}.
    std::pair<bool, bool> wait_echo() {
        const auto deadline = std::chrono::steady_clock::now() + kEchoTimeout;
        while (!got_echo_.load(std::memory_order_acquire)) {
            if (!g_running.load(std::memory_order_relaxed)
                || std::chrono::steady_clock::now() >= deadline)
                return {false, false};
            std::this_thread::sleep_for(std::chrono::microseconds{20});
        }
        return {true, type_ok_.load(std::memory_order_relaxed)};
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

        // The whole point: the board must report the type it actually saw on the
        // wire, matching what we asked it to transmit.
        const bool matched = data.is_fdcan == expect_fd_.load(std::memory_order_relaxed)
                          && data.is_extended_can_id == want_ext;
        type_ok_.store(matched, std::memory_order_relaxed);
        got_echo_.store(true, std::memory_order_release);
    }

    std::atomic<uint32_t> seq_{0xFFFFFFFFU};
    std::atomic<bool> expect_fd_{false};
    std::atomic<bool> expect_ext_{false};
    std::atomic<std::size_t> expect_size_{0};
    std::atomic<bool> got_echo_{false};
    std::atomic<bool> type_ok_{false};

    examples::BoardSession* board_ = nullptr;
};

int main() {
    std::signal(SIGINT, on_sigint);

    printf("CAN frame-type acceptance test\n");
    printf("  Wire CAN bus 0 <-> bus 1 together (120 ohm at EACH end).\n");
    printf("  TX on bus 0, echo observed on bus 1.\n\nConnecting ...\n");

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

    printf("%-28s %8s %8s %8s\n", "case", "sent", "echoed", "type-ok");
    printf("%-28s %8s %8s %8s\n", "----", "----", "------", "-------");

    int failed_cases = 0;
    uint32_t seq = 0;

    for (const auto& test_case : kCases) {
        uint32_t echoed = 0;
        uint32_t type_ok = 0;

        for (uint32_t i = 0; i < kRoundsPerCase && g_running.load(std::memory_order_relaxed);
             ++i) {
            tester.arm(test_case, ++seq);
            const auto [ok, matched] = tester.wait_echo();
            if (ok) {
                ++echoed;
                if (matched)
                    ++type_ok;
            }
            std::this_thread::sleep_for(kInterFrameGap);
        }

        const bool pass = echoed == kRoundsPerCase && type_ok == kRoundsPerCase;
        if (!pass)
            ++failed_cases;
        printf(
            "%-28s %8u %8u %8u  %s\n", test_case.name, kRoundsPerCase, echoed, type_ok,
            pass ? "PASS" : "FAIL");
    }

    printf("\n");
    if (failed_cases == 0) {
        printf("ALL PASS -- board accepts classic CAN 2.0 and CAN-FD on the same bus,\n");
        printf("and reports each frame's real type back to the host.\n");
        return 0;
    }
    printf("%d case(s) FAILED.\n", failed_cases);
    printf("If only the CAN-FD cases lost frames, suspect termination/wiring before\n");
    printf("firmware: the 5 Mbit data phase is far less tolerant of reflections.\n");
    return 1;
}
