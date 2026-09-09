// Main-loop period probe for a single HPM5321 DualCan board.
//
// can_stall_probe is the richer tool but it cannot run on this rig: it targets
// the HPM6E8Y (PID 0x6E84), drives CAN0->CAN1 and CAN2->CAN3 as two SINGLE-BOARD
// loopbacks, and therefore assumes four buses. The HPM5321 has two, and the
// two-board rig wires them across to a second board rather than back to itself.
//
// It generates its own load rather than leaning on dual_board_test stress,
// because that tool claims BOTH boards for the whole run and the telemetry
// reader cannot then open board A (libusb ERROR_BUSY).
//
// USE: sweep the rate and take the SLOPE of iterations-vs-rate, not any single
// point. Iterations per 100 ms fall linearly with load, and slope x idle-period
// is the CPU time one unit of load costs the board. Rate 0 gives the idle period
// that scales the slope.
//
// Omit serial_b to load board A's downlink alone (one OUT packet plus the CAN TX
// it triggers). Give it to add the reverse direction, so the extra slope is board
// A's CAN RX and USB uplink. The difference is what splits the cost by direction.
//
// Keep the rate below the ~19.8k f/s CAN-FD ceiling: past it the board spends its
// time on a full TX queue instead of on the packet, and the point leaves the line.
//
// Requires firmware built with -Dlibhcs_CAN_DIAG=ON; without it the board sends
// no telemetry and this prints nothing but the "no records" warning.
//
// Run:
//   ./hpm5321_loop_probe [seconds] [rate] [serial_a] [serial_b]

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <mutex>
#include <span>
#include <string_view>
#include <thread>
#include <vector>

#include <libhcs/board/hpm5321.hpp>

#include "can_diag_record.hpp"


// CAN ports are named as the silkscreen on THAT board. 5321 prints CAN1/CAN2.
// See libhcs/board/hcs_can_port.hpp.
using libhcs::board::hcs::CanPort;

namespace {

using libhcs::diag::kRecordPeriodUs;

using libhcs::diag::decode_main_loop_iters;
using libhcs::diag::decode_usb_out_timing;
using libhcs::diag::UsbOutTiming;

class Receiver final : public libhcs::board::Hpm5321::Callback {
public:
    std::vector<uint32_t> take() {
        const std::scoped_lock guard{mutex_};
        return samples_;
    }

    // Cycle sums are accumulated across records rather than averaged per record,
    // so a quiet period contributes its few samples with the weight it deserves.
    void take_usb(double& turnaround_us, double& starve_us, uint64_t& samples) const {
        const std::scoped_lock guard{mutex_};
        samples = usb_samples_;
        if (usb_samples_ == 0 || usb_core_hz_ == 0) {
            turnaround_us = starve_us = 0.0;
            return;
        }
        const double us_per_cycle = 1e6 / static_cast<double>(usb_core_hz_);
        turnaround_us = static_cast<double>(usb_turnaround_) / static_cast<double>(usb_samples_)
                      * us_per_cycle;
        starve_us =
            static_cast<double>(usb_starve_) / static_cast<double>(usb_samples_) * us_per_cycle;
    }

private:
    void uart0_receive_callback(const libhcs::data::UartDataView& data) override {
        uint32_t iters = 0;
        if (!decode_main_loop_iters(data.uart_data, iters))
            return;
        UsbOutTiming usb{};
        const bool have_usb = decode_usb_out_timing(data.uart_data, usb);
        const std::scoped_lock guard{mutex_};
        samples_.push_back(iters);
        if (have_usb) {
            usb_turnaround_ += usb.turnaround_cycles;
            usb_starve_ += usb.starve_cycles;
            usb_samples_ += usb.samples;
            usb_core_hz_ = usb.core_hz;
        }
    }

    mutable std::mutex mutex_;
    std::vector<uint32_t> samples_;
    uint64_t usb_turnaround_ = 0;
    uint64_t usb_starve_ = 0;
    uint64_t usb_samples_ = 0;
    uint32_t usb_core_hz_ = 0;
};

// Board B only has to transmit, so it needs no callbacks of its own.
class Silent final : public libhcs::board::Hpm5321::Callback {};

constexpr uint32_t kCanIdA = 0x210;
constexpr uint32_t kCanIdB = 0x211;

} // namespace

int main(int argc, char** argv) {
    const int duration_s = argc > 1 ? std::atoi(argv[1]) : 20;
    const uint32_t rate = argc > 2 ? static_cast<uint32_t>(std::atoi(argv[2])) : 0;
    const std::string_view serial_a = argc > 3 ? argv[3] : std::string_view{};
    const std::string_view serial_b = argc > 4 ? argv[4] : std::string_view{};
    if (duration_s <= 0) {
        fprintf(stderr, "seconds must be positive\n");
        return 1;
    }
    Receiver rx;
    Silent silent;
    try {
        libhcs::board::Hpm5321 board_a{rx, serial_a};
        printf("HPM5321 DualCan connected; sampling telemetry for %d s\n", duration_s);

        std::atomic<bool> running{true};
        std::atomic<uint64_t> packets{0};
        std::thread load;

        // Two shapes, because the interesting number is a DIFFERENCE between
        // them. Omitting serial_b loads board A's downlink alone (one OUT packet
        // plus the CAN TX it triggers); giving it adds the reverse direction, so
        // the extra cost is board A's CAN RX and USB uplink. Subtracting the two
        // slopes is what splits per-packet CPU work by direction.
        const bool downlink_only = serial_b.empty();
        if (rate > 0) {
            load = std::thread{[&]() {
                const std::array<std::byte, 8> payload{};
                using clock = std::chrono::steady_clock;
                const auto period = std::chrono::duration_cast<clock::duration>(
                    std::chrono::nanoseconds{1'000'000'000U / rate});
                auto next = clock::now();

                // Never flood from this process: the transport blocks in
                // start_transmit() once the transfer pool is empty, and a
                // blocked sender cannot observe `running`, so the join at the
                // end would hang. Flood belongs in usb_packet_rate, which owns
                // its own deadline. Here the load is always paced.
                std::unique_ptr<libhcs::board::Hpm5321> board_b;
                if (!downlink_only)
                    board_b =
                        std::make_unique<libhcs::board::Hpm5321>(silent, serial_b);

                while (running.load(std::memory_order_relaxed)) {
                    next += period;
                    // The builder is RAII: it ships the packet when it goes out
                    // of scope, so each direction gets its own scope rather than
                    // sharing one packet across two boards.
                    {
                        auto builder = board_a.start_transmit();
                        builder.can_transmit(CanPort::kCan1, 
                            {.can_id = kCanIdA, .can_data = payload, .is_fdcan = true});
                    }
                    packets.fetch_add(1, std::memory_order_relaxed);
                    if (board_b) {
                        auto builder = board_b->start_transmit();
                        builder.can_transmit(CanPort::kCan2, 
                            {.can_id = kCanIdB, .can_data = payload, .is_fdcan = true});
                        packets.fetch_add(1, std::memory_order_relaxed);
                    }
                    std::this_thread::sleep_until(next);
                }
            }};
            printf(
                "load: %u/s, %s\n", rate,
                downlink_only ? "board A downlink only" : "bidirectional (A out, B -> A in)");
        }

        std::this_thread::sleep_for(std::chrono::seconds{duration_s});
        running.store(false, std::memory_order_relaxed);
        if (load.joinable())
            load.join();
        if (rate > 0) {
            printf(
                "packets pushed: %llu  (%.0f packets/s)\n",
                static_cast<unsigned long long>(packets.load()),
                static_cast<double>(packets.load()) / duration_s);
        }
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    auto samples = rx.take();
    // The first record covers the tick during which the session came up, so it
    // reports a partial interval; dropping it keeps the minimum honest.
    if (!samples.empty())
        samples.erase(samples.begin());
    if (samples.empty()) {
        printf("no telemetry records -- is the firmware built with libhcs_CAN_DIAG=ON?\n");
        return 1;
    }

    std::sort(samples.begin(), samples.end());
    const auto at = [&](double q) {
        const size_t index = std::min(samples.size() - 1, static_cast<size_t>(q * samples.size()));
        return samples[index];
    };
    double sum = 0.0;
    for (const uint32_t value : samples)
        sum += value;
    const double mean_iters = sum / static_cast<double>(samples.size());

    // Reported as a period rather than a rate because that is what compares
    // against the 0.72 us / 0.85 us figures already in the board's AGENTS.md.
    printf(
        "records=%zu  main-loop iters/100ms: min %u  p50 %u  max %u  mean %.0f\n", samples.size(),
        samples.front(), at(0.5), samples.back(), mean_iters);
    printf(
        "main-loop period us:   min %.3f  p50 %.3f  max %.3f  mean %.3f\n",
        kRecordPeriodUs / samples.back(), kRecordPeriodUs / at(0.5),
        kRecordPeriodUs / samples.front(), kRecordPeriodUs / mean_iters);

    double turnaround_us = 0.0, starve_us = 0.0;
    uint64_t usb_samples = 0;
    rx.take_usb(turnaround_us, starve_us, usb_samples);
    if (usb_samples == 0) {
        printf("usb bulk OUT: no packets observed\n");
        return 0;
    }
    const double cycle_us = turnaround_us + starve_us;
    printf(
        "usb bulk OUT: n=%llu  cycle %.2f us (%.0f packets/s)\n"
        "  turnaround (device, removable by chained qTD) %.2f us  %.1f%%\n"
        "  starve     (bus + host pace, NOT removable)   %.2f us  %.1f%%\n",
        static_cast<unsigned long long>(usb_samples), cycle_us,
        cycle_us > 0.0 ? 1e6 / cycle_us : 0.0, turnaround_us,
        cycle_us > 0.0 ? 100.0 * turnaround_us / cycle_us : 0.0, starve_us,
        cycle_us > 0.0 ? 100.0 * starve_us / cycle_us : 0.0);
    return 0;
}
