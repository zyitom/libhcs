// Are the host controller's MFINDEX and the board's SOF counter the same axis?
//
// They ought to be, exactly: MFINDEX is the microframe index the xHCI increments
// every 125 us, and the board's counter advances once per SOF packet -- and the
// xHCI emits one SOF per microframe. So the difference between them should be a
// CONSTANT (a fixed offset plus the board-to-host transit), and any drift is a
// rate disagreement between two counters that are supposed to be one clock.
//
// This exists because two independent measurements of "microframes per second
// against CLOCK_MONOTONIC" disagreed by 576 ppm:
//
//   MFINDEX read straight from the mmap'd BAR   124937.5 ns  (-500.0 ppm)
//     three rounds of 80000 microframes, agreeing to 0.3 ppm
//   Timeline's fit through the board round trip 125009.5 ns  (+76.0 ppm)
//     the same number on both an mc02 and two HPM5321
//
// One of them is wrong, and sampling both in the same process against the same
// clock is what tells them apart. A drift here of ~576 ppm means the counters
// really are different and the Timeline is reading the board correctly; a flat
// difference means the counters agree and the Timeline's rate is the error.
//
// Needs root: the xHCI register window comes from the PCI BAR via sysfs.
//
// Run:
//   sudo ./mfindex_vs_board <pci-bdf> [seconds] [serial]
//   sudo ./mfindex_vs_board 0000:00:14.0 40

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <libhcs/board/hpm5321.hpp>
#include <libhcs/time/timeline.hpp>

namespace {

using Clock = libhcs::host::time::Timeline::Clock;
using libhcs::host::time::timeline;

// xHCI 5.3.8: the Runtime Register Space offset is at capability offset 0x18
// with the low five bits reserved, and MFINDEX is the first dword of that space.
class Mfindex {
public:
    explicit Mfindex(const std::string& bdf) {
        const std::string path = "/sys/bus/pci/devices/" + bdf + "/resource0";
        fd_ = open(path.c_str(), O_RDWR | O_SYNC);
        if (fd_ < 0)
            throw std::runtime_error{"open " + path + " (need root)"};
        bar_ =
            static_cast<volatile uint8_t*>(mmap(nullptr, kWindow, PROT_READ, MAP_SHARED, fd_, 0));
        if (bar_ == MAP_FAILED)
            throw std::runtime_error{"mmap " + path};
        const uint32_t rtsoff = *reinterpret_cast<volatile uint32_t*>(bar_ + 0x18) & ~0x1FU;
        mfindex_ = reinterpret_cast<volatile uint32_t*>(bar_ + rtsoff);
    }

    ~Mfindex() {
        if (bar_ != MAP_FAILED)
            munmap(const_cast<uint8_t*>(bar_), kWindow);
        if (fd_ >= 0)
            close(fd_);
    }

    Mfindex(const Mfindex&) = delete;
    Mfindex& operator=(const Mfindex&) = delete;

    // 64-bit extension. The field is 14 bits and wraps every 2.048 s, so any
    // caller sampling faster than half that resolves the wrap unambiguously.
    uint64_t read() {
        const uint32_t raw = *mfindex_ & 0x3FFFU;
        if (seeded_ && raw < previous_)
            high_ += 0x4000U;
        seeded_ = true;
        previous_ = raw;
        return high_ + raw;
    }

private:
    static constexpr size_t kWindow = 65536;
    int fd_ = -1;
    volatile uint8_t* bar_ = static_cast<volatile uint8_t*>(MAP_FAILED);
    volatile uint32_t* mfindex_ = nullptr;
    uint64_t high_ = 0;
    uint32_t previous_ = 0;
    bool seeded_ = false;
};

struct Pair {
    uint64_t board_microframe;
    uint64_t host_microframe;
    Clock::time_point at;
};

class Receiver final : public libhcs::board::Hpm5321::Callback {
public:
    explicit Receiver(Mfindex& mfindex)
        : mfindex_(mfindex) {}

    std::vector<Pair> take() const {
        const std::scoped_lock guard{mutex_};
        return pairs_;
    }

private:
    // Sample MFINDEX inside the callback, so the two counters are read within a
    // microsecond of each other. The board's value is older by the uplink
    // transit, which is a constant and therefore harmless to a drift test.
    void time_status_callback(const libhcs::data::TimeStatusView& data) override {
        if (data.state != libhcs::data::TimeState::kValid)
            return;
        const std::scoped_lock guard{mutex_};
        pairs_.push_back({data.microframe, mfindex_.read(), Clock::now()});
    }

    Mfindex& mfindex_;
    mutable std::mutex mutex_;
    std::vector<Pair> pairs_;
};

} // namespace

int main(int argc, char** argv) {
    const std::string bdf = argc > 1 ? argv[1] : "0000:00:14.0";
    const int duration_s = argc > 2 ? std::atoi(argv[2]) : 40;
    const std::string serial = argc > 3 ? argv[3] : std::string{};

    std::unique_ptr<Mfindex> mfindex;
    try {
        mfindex = std::make_unique<Mfindex>(bdf);
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    Receiver receiver{*mfindex};
    libhcs::board::AdvancedOptions options;
    options.set_enable_time_sync(true);
    std::unique_ptr<libhcs::board::Hpm5321> board;
    try {
        board =
            std::make_unique<libhcs::board::Hpm5321>(receiver, serial, options);
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    printf("sampling %d s on %s...\n", duration_s, bdf.c_str());
    std::this_thread::sleep_for(std::chrono::seconds{duration_s});

    const auto pairs = receiver.take();
    if (pairs.size() < 20) {
        fprintf(stderr, "too few valid time status reports (%zu)\n", pairs.size());
        return 1;
    }

    // If the two counters are one clock, (board - host) is constant. Fit a line
    // through it: the slope IS the rate disagreement, in microframes per
    // microframe, i.e. directly in parts per million once scaled.
    const auto& first = pairs.front();
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    std::vector<double> difference;
    for (const Pair& pair : pairs) {
        const double x = static_cast<double>(pair.host_microframe - first.host_microframe);
        const double y = static_cast<double>(
            static_cast<int64_t>(pair.board_microframe - first.board_microframe)
            - static_cast<int64_t>(pair.host_microframe - first.host_microframe));
        difference.push_back(y);
        n += 1;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    const double slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);

    std::vector<double> sorted = difference;
    std::sort(sorted.begin(), sorted.end());
    const double span = sorted.back() - sorted.front();
    const double elapsed =
        std::chrono::duration<double>{pairs.back().at - pairs.front().at}.count();

    // Independent fits of both counters against CLOCK_MONOTONIC, from the same
    // samples. If these disagree, the Timeline's published rate can be checked
    // against a number derived without it.
    {
        double bn = 0, bx = 0, by = 0, bxx = 0, bxy = 0;
        double hx = 0, hy = 0, hxx = 0, hxy = 0;
        const double t0 =
            std::chrono::duration<double, std::nano>{pairs.front().at.time_since_epoch()}.count();
        for (const Pair& pair : pairs) {
            const double t =
                std::chrono::duration<double, std::nano>{pair.at.time_since_epoch()}.count() - t0;
            const double b = static_cast<double>(pair.board_microframe - first.board_microframe);
            const double h = static_cast<double>(pair.host_microframe - first.host_microframe);
            bn += 1;
            bx += b;
            by += t;
            bxx += b * b;
            bxy += b * t;
            hx += h;
            hy += t;
            hxx += h * h;
            hxy += h * t;
        }
        const double board_period = (bn * bxy - bx * by) / (bn * bxx - bx * bx);
        const double host_period = (bn * hxy - hx * hy) / (bn * hxx - hx * hx);
        printf("\n=== microframe period against CLOCK_MONOTONIC, both from these samples ===\n");
        printf(
            "  board counter : %.1f ns  (%+.1f ppm)\n", board_period,
            (board_period / 125000.0 - 1.0) * 1e6);
        printf(
            "  host MFINDEX  : %.1f ns  (%+.1f ppm)\n", host_period,
            (host_period / 125000.0 - 1.0) * 1e6);
        printf(
            "  Timeline says : %.1f ns  (%+.1f ppm)\n", timeline().measured_period_ns(),
            (timeline().measured_period_ns() / 125000.0 - 1.0) * 1e6);
    }

    printf("\n=== board SOF counter vs host MFINDEX ===\n");
    printf("  %zu paired samples over %.1f s\n", pairs.size(), elapsed);
    printf(
        "  difference (board - host) spans %.0f microframes = %.0f us over the run\n", span,
        span * 125.0);
    printf("  fitted drift: %+.2f ppm\n", slope * 1e6);
    printf(
        "\n  %s\n",
        std::fabs(slope * 1e6) < 20.0
            ? "FLAT -- the two counters are one clock, so the Timeline's rate is the error"
            : "DRIFTING -- the counters really are independent; the Timeline reads the board "
              "right");
    return 0;
}
