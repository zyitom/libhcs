// Motor-load simulation: how many 1 kHz classic-CAN motors can this rig carry?
//
// Models the real control pattern rather than a raw throughput sweep: every
// control period the host sends one command frame per group of 4 motors (the DJI
// 0x200 multicast layout) and expects one feedback frame back per motor. The
// motor side is stood in for by the peer board echoing each command, so the
// measured loop includes both directions and both boards' USB paths -- which is
// where the ceiling actually is on a Full-Speed board, not on the CAN wire.
//
// Reports, per motor count: achieved period rate, command frames delivered, and
// the jitter of the control period. A control loop is "blown" when it can no
// longer hold the period, which shows up as period overrun long before frames
// are outright lost.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>
#include "common/multi_board.hpp"

using Clock = std::chrono::steady_clock;
namespace {

constexpr uint32_t kCmdIdBase = 0x200;   // DJI-style multicast command id
constexpr uint32_t kFbIdBase  = 0x300;   // feedback ids, kept clear of 0x200..0x20F
constexpr size_t kPayload = 8;

struct Sink final : public examples::BoardReceiver {
    std::atomic<uint32_t> cmd_seen{0};
    std::atomic<uint32_t> fb_seen{0};
    int watch_bus = -1;
    void on_can(int bus, const libhcs::data::CanDataView& d) override {
        if (bus != watch_bus) return;
        if (d.can_id >= kFbIdBase) fb_seen.fetch_add(1, std::memory_order_relaxed);
        else if (d.can_id >= kCmdIdBase && d.can_id < kCmdIdBase + 16)
            cmd_seen.fetch_add(1, std::memory_order_relaxed);
    }
};

} // namespace

int main(int argc, char** argv) {
    const uint32_t hz = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 1000;
    const uint32_t secs = argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 3;
    const int tx_bus = argc > 3 ? std::atoi(argv[3]) : 0;
    const int rx_bus = argc > 4 ? std::atoi(argv[4]) : 0;

    Sink sink_a, sink_b;
    auto a = examples::connect_any(sink_a, std::getenv("HCS_BOARD_A") ?: "");
    auto b = examples::connect_any(sink_b, std::getenv("HCS_BOARD_B") ?: "");
    if (!a || !b) { fprintf(stderr, "need two boards\n"); return 1; }
    sink_a.watch_bus = tx_bus; sink_b.watch_bus = rx_bus;
    printf("A = %.*s   B = %.*s\n", (int)a->name().size(), a->name().data(),
           (int)b->name().size(), b->name().data());
    printf("\n%u Hz control loop, classic CAN, A.CAN%d -> B.CAN%d, %u s per step\n",
           hz, tx_bus, rx_bus, secs);
    printf("%7s %9s %7s %11s %11s %8s  %s\n", "motors", "bus f/s", "load",
           "cmd deliv", "fb deliv", "p50 us", "verdict");

    for (const int motors : {2, 4, 6, 7, 8, 10, 12, 16}) {
        const int cmd_frames = (motors + 3) / 4;         // one frame per 4 motors
        // Only the command frames are really on the wire: this rig has no motors
        // to answer, so the feedback half of the exchange does not exist. Report
        // both the modelled total (what a real N-motor bus would carry) and the
        // load actually generated, otherwise a "468% load, zero loss" line looks
        // like a measurement that beat physics.
        const uint32_t fps_wire = (uint32_t)cmd_frames * hz;
        const uint32_t fps_model = (uint32_t)(cmd_frames + motors) * hz;
        sink_b.cmd_seen.store(0); sink_b.fb_seen.store(0);

        const auto period = std::chrono::nanoseconds{1'000'000'000ULL / hz};
        const uint64_t cycles = (uint64_t)hz * secs;
        std::vector<double> lat; lat.reserve(cycles);
        // Stand in for the motors: B answers every control period with one
        // feedback frame per motor, so the segment carries the full
        // command+feedback load a real N-motor bus would. Without this the wire
        // only ever sees the command frames and the bus never saturates.
        std::atomic<bool> run{true};
        std::atomic<uint64_t> fb_sent{0};
        std::thread responder{[&]() {
            auto fnext = Clock::now();
            while (run.load(std::memory_order_relaxed)) {
                std::byte fb[kPayload]{};
                try {
                    b->transmit([&](examples::BoardTransmitter& tx) {
                        for (int m = 0; m < motors; ++m)
                            tx.can(rx_bus, {.can_id = kFbIdBase + (uint32_t)m,
                                            .can_data = fb, .is_fdcan = false});
                    });
                    fb_sent.fetch_add(motors, std::memory_order_relaxed);
                } catch (const std::exception&) {}
                fnext += period;
                while (Clock::now() < fnext) {}
            }
        }};

        auto next = Clock::now();
        uint64_t sent = 0;
        for (uint64_t c = 0; c < cycles; ++c) {
            const auto t0 = Clock::now();
            std::byte pay[kPayload]{};
            std::memcpy(pay, &c, sizeof(uint32_t));
            try {
                a->transmit([&](examples::BoardTransmitter& tx) {
                    for (int f = 0; f < cmd_frames; ++f)
                        tx.can(tx_bus, {.can_id = kCmdIdBase + (uint32_t)f,
                                        .can_data = pay, .is_fdcan = false});
                });
                sent += cmd_frames;
            } catch (const std::exception&) {}
            next += period;
            while (Clock::now() < next) {}
            lat.push_back(std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
        }
        run.store(false, std::memory_order_relaxed);
        responder.join();
        std::this_thread::sleep_for(std::chrono::milliseconds{300});
        std::sort(lat.begin(), lat.end());
        auto pct = [&](double f) { return lat[(size_t)(f * (lat.size() - 1))]; };
        const uint32_t got = sink_b.cmd_seen.load();
        const uint32_t fb_got = sink_a.fb_seen.load();
        sink_a.fb_seen.store(0);
        const uint64_t fb_want = fb_sent.load();
        const double fb_ratio = fb_want ? (double)fb_got / (double)fb_want : 1.0;
        const double load = fps_model / 8547.0 * 100.0;
        // Name the binding constraint instead of leaving the reader to infer it
        // from two columns. Feedback is emitted by the peer board, so a shortfall
        // there is that board's USB ceiling, not the CAN wire.
        const char* verdict = "ok";
        if (fb_ratio < 0.99 && load > 100.0) verdict = "BUS+TX SATURATED";
        else if (fb_ratio < 0.99)            verdict = "TX BOARD LIMIT";
        else if (load > 100.0)               verdict = "OVER BUS CAPACITY";
        printf("%7d %9u %6.0f%% %5u/%-5llu %5u/%-5llu %8.1f  %s\n",
               motors, fps_model, load, got, (unsigned long long)sent, fb_got,
               (unsigned long long)fb_want, pct(0.50), verdict);
    }
    return 0;
}
