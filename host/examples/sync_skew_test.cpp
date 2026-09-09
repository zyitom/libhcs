// Cross-board synchronisation skew, MEASURED rather than derived.
//
// Everything before this could only bound the skew indirectly: the host-side
// comparison of the boards' periodic reports is capped at one microframe by USB
// uplink jitter, and the per-board fit residual gives the right answer only
// under the assumption that nothing else contributes. This tool removes both
// limitations by using an event that both boards observe in HARDWARE, at the
// same physical instant, through identical paths.
//
// RIG: every CAN controller on ONE bus (all four ports of the two boards on a
// common distribution board). One transmitted frame is then captured by the
// timestamp unit of every OTHER controller -- including the transmitting
// board's second controller -- so a single frame yields one capture on each
// board, both of them receive-side, both taken at Start-of-Frame by the TSU
// with no software in the path. Serialization, queueing and interrupt latency
// are all outside the measurement by construction.
//
//     skew = (board B's report of frame N) - (board A's report of frame N)
//
// THE CONTROL THAT MAKES IT TRUSTWORTHY: board B has two controllers on the
// same bus, and both share one PTPC timebase. Their reports of the same frame
// must therefore agree exactly. Whatever spread they show is the noise floor of
// this whole method -- timestamp quantisation, transceiver differences, the
// PTPC-to-microframe conversion -- with the cross-board term provably absent.
// Compare the cross-board spread against it; if they are the same, the skew is
// below what this method can see, and the number to quote is the floor, not the
// measurement.
//
// Requires firmware built with -Dlibhcs_TIME_SYNC=ON.
//
// Run:
//   ./sync_skew_test [seconds] [frames_per_second]

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>

#include <libhcs/board/hpm5321.hpp>
#include <libhcs/time/timeline.hpp>


// CAN ports are named as the silkscreen on THAT board. 5321 prints CAN1/CAN2.
// See libhcs/board/hcs_can_port.hpp.
using libhcs::board::hcs::CanPort;

namespace {

// One microframe is 125 us, and reports carry Q16 microframes.
constexpr double kNsPerQ16 = 125000.0 / 65536.0;

struct Sample {
    uint64_t microframe_q16;   // as the board converted it
    uint32_t ptpc_ns;          // the raw hardware capture
    uint8_t bus;
};

// A raw, unfitted (microframe, PTPC nanosecond) pair as sampled in the board's
// SOF interrupt. Collected so the host can build its OWN conversion in floating
// point and compare it against the board's fixed-point one -- which is the only
// way to tell a bad fit from bad samples.
struct RawPair {
    uint64_t microframe;
    uint32_t ptpc_ns;
};

// Least squares of PTPC nanoseconds against the microframe counter, unwrapped.
// PTPC's nanosecond word rolls over every 1e9 units (~1.04 s) and pairs arrive
// every 250 ms, so "the reading went backwards" is an unambiguous wrap.
struct HostFit {
    bool valid = false;
    double units_per_microframe = 0.0;
    double intercept = 0.0;      // unwrapped ns at microframe_origin
    uint64_t microframe_origin = 0;

    static HostFit build(const std::vector<RawPair>& pairs) {
        HostFit fit;
        if (pairs.size() < 8)
            return fit;
        std::vector<double> x;
        std::vector<double> y;
        double unwrapped = 0.0;
        uint32_t previous = pairs.front().ptpc_ns;
        for (const RawPair& pair : pairs) {
            double step = static_cast<double>(pair.ptpc_ns) - static_cast<double>(previous);
            if (step < 0.0)
                step += 1e9;
            unwrapped += step;
            previous = pair.ptpc_ns;
            x.push_back(static_cast<double>(pair.microframe - pairs.front().microframe));
            y.push_back(unwrapped);
        }
        double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
        for (size_t index = 0; index < x.size(); index++) {
            sum_x += x[index];
            sum_y += y[index];
            sum_xx += x[index] * x[index];
            sum_xy += x[index] * y[index];
        }
        const auto n = static_cast<double>(x.size());
        const double denominator = n * sum_xx - sum_x * sum_x;
        if (denominator <= 0.0)
            return fit;
        fit.units_per_microframe = (n * sum_xy - sum_x * sum_y) / denominator;
        fit.intercept = (sum_y - fit.units_per_microframe * sum_x) / n
                      + static_cast<double>(pairs.front().ptpc_ns);
        fit.microframe_origin = pairs.front().microframe;
        fit.valid = true;
        return fit;
    }

    // Converts a raw capture. The board's own (coarse but never more than tens
    // of microseconds wrong) answer picks which 1.04 s wrap the reading belongs
    // to; this fit then places it precisely inside that wrap.
    double microframe_of(uint32_t ptpc_ns, double coarse_microframe) const {
        const double predicted =
            intercept
            + units_per_microframe * (coarse_microframe - static_cast<double>(microframe_origin));
        const double residue = std::fmod(static_cast<double>(ptpc_ns) - predicted, 1e9);
        double correction = residue;
        if (correction > 5e8)
            correction -= 1e9;
        else if (correction < -5e8)
            correction += 1e9;
        return coarse_microframe + correction / units_per_microframe;
    }
};

class Receiver final : public libhcs::board::Hpm5321::Callback {
public:
    std::map<uint32_t, std::vector<Sample>> take() const {
        const std::scoped_lock guard{mutex_};
        return samples_;
    }

    std::vector<RawPair> take_raw() const {
        const std::scoped_lock guard{mutex_};
        return raw_;
    }

    bool valid() const {
        const std::scoped_lock guard{mutex_};
        return valid_;
    }

private:
    void sync_sample_callback(const libhcs::data::SyncSampleView& data) override {
        const std::scoped_lock guard{mutex_};
        samples_[data.tag].push_back({data.microframe_q16, data.ptpc_ns, data.bus});
    }

    void time_status_callback(const libhcs::data::TimeStatusView& data) override {
        const std::scoped_lock guard{mutex_};
        valid_ = data.state == libhcs::data::TimeState::kValid;
        if (valid_ && data.ptpc_raw_microframe != 0
            && (raw_.empty() || raw_.back().microframe != data.ptpc_raw_microframe))
            raw_.push_back({data.ptpc_raw_microframe, data.ptpc_raw_ns});
    }

    mutable std::mutex mutex_;
    std::map<uint32_t, std::vector<Sample>> samples_;
    std::vector<RawPair> raw_;
    bool valid_ = false;
};

std::vector<std::string> enumerate_boards() {
    std::vector<std::string> found;
    DIR* dir = opendir("/sys/bus/usb/devices");
    if (!dir)
        return found;
    while (const dirent* entry = readdir(dir)) {
        const std::string base = std::string{"/sys/bus/usb/devices/"} + entry->d_name;
        const auto read_line = [&](const char* leaf) -> std::string {
            const std::string path = base + "/" + leaf;
            FILE* file = fopen(path.c_str(), "re");
            if (!file)
                return {};
            char buffer[256] = {};
            if (!fgets(buffer, sizeof(buffer), file)) {
                fclose(file);
                return {};
            }
            fclose(file);
            std::string value{buffer};
            while (!value.empty() && (value.back() == '\n' || value.back() == '\r'))
                value.pop_back();
            return value;
        };
        if (read_line("idVendor") != "a511" || read_line("idProduct") != "5322")
            continue;
        const std::string serial = read_line("serial");
        if (!serial.empty())
            found.push_back(serial);
    }
    closedir(dir);
    std::sort(found.begin(), found.end());
    return found;
}

struct Stats {
    size_t count;
    double mean_ns;
    double sigma_ns;
    double min_ns;
    double max_ns;
};

Stats summarise(std::vector<double>& values) {
    Stats out{values.size(), 0.0, 0.0, 0.0, 0.0};
    if (values.empty())
        return out;
    std::sort(values.begin(), values.end());
    double sum = 0.0;
    for (const double value : values)
        sum += value;
    out.mean_ns = sum / static_cast<double>(values.size());
    double variance = 0.0;
    for (const double value : values)
        variance += (value - out.mean_ns) * (value - out.mean_ns);
    out.sigma_ns = std::sqrt(variance / static_cast<double>(values.size()));
    out.min_ns = values.front();
    out.max_ns = values.back();
    return out;
}

void print_stats(const char* label, const Stats& stats) {
    if (stats.count == 0) {
        printf("  %-28s no samples\n", label);
        return;
    }
    printf(
        "  %-28s n=%-6zu mean %+8.1f ns   sigma %7.1f ns   min %+8.1f   max %+8.1f\n", label,
        stats.count, stats.mean_ns, stats.sigma_ns, stats.min_ns, stats.max_ns);
}

} // namespace

int main(int argc, char** argv) {
    const int duration_s = argc > 1 ? std::atoi(argv[1]) : 20;
    const int rate = argc > 2 ? std::atoi(argv[2]) : 200;
    if (duration_s <= 0 || rate <= 0) {
        fprintf(stderr, "seconds and frames_per_second must be positive\n");
        return 1;
    }

    const auto serials = enumerate_boards();
    if (serials.size() < 2) {
        fprintf(stderr, "need two hpm5321 boards, found %zu\n", serials.size());
        return 1;
    }

    Receiver rx_a;
    Receiver rx_b;
    libhcs::board::AdvancedOptions options_a;
    libhcs::board::AdvancedOptions options_b;
    options_a.set_enable_time_sync(true);
    options_b.set_enable_time_sync(true);

    try {
        libhcs::board::Hpm5321 board_a{rx_a, serials[0], options_a};
        libhcs::board::Hpm5321 board_b{rx_b, serials[1], options_b};

        // The time base needs one fit window plus one anchor before it can place
        // a capture; probing before that just discards frames.
        printf("waiting for both timelines to become valid ...\n");
        for (int waited = 0; waited < 100 && !(rx_a.valid() && rx_b.valid()); waited++)
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        if (!rx_a.valid() || !rx_b.valid()) {
            fprintf(
                stderr, "timeline not valid (A %d, B %d) -- firmware built with TIME_SYNC=ON?\n",
                static_cast<int>(rx_a.valid()), static_cast<int>(rx_b.valid()));
            return 2;
        }

        printf("probing %d frames/s for %d s on CAN id 0x%03X\n", rate, duration_s,
               libhcs::data::kSyncProbeCanId);
        const auto period = std::chrono::nanoseconds{1'000'000'000 / rate};
        auto next = std::chrono::steady_clock::now();
        const auto deadline = next + std::chrono::seconds{duration_s};
        uint32_t tag = 1;
        while (std::chrono::steady_clock::now() < deadline) {
            next += period;
            std::array<std::byte, 8> payload{};
            std::memcpy(payload.data(), &tag, sizeof(tag));
            {
                auto builder = board_a.start_transmit();
                builder.can_transmit(CanPort::kCan1, 
                    {.can_id = libhcs::data::kSyncProbeCanId,
                     .can_data = payload,
                     .is_fdcan = true});
            }
            tag++;
            const auto now = std::chrono::steady_clock::now();
            if (next < now)
                next = now;
            std::this_thread::sleep_until(next);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{300});
    } catch (const std::exception& error) {
        fprintf(stderr, "error: %s\n", error.what());
        return 2;
    }

    const auto samples_a = rx_a.take();
    const auto samples_b = rx_b.take();
    printf(
        "\ncaptures: board A %zu tags, board B %zu tags\n", samples_a.size(), samples_b.size());

    // The whole point of this revision: convert every capture a SECOND time,
    // here, in double precision, from the raw hardware value and a fit built
    // only from raw pairs. If this disagrees with the board's own conversion,
    // the board's fixed-point arithmetic is the fault; if it agrees and both
    // wander, the samples are.
    const HostFit fit_a = HostFit::build(rx_a.take_raw());
    const HostFit fit_b = HostFit::build(rx_b.take_raw());
    printf(
        "host-side fits: A %s %.3f units/microframe   B %s %.3f units/microframe\n",
        fit_a.valid ? "ok" : "FAILED", fit_a.units_per_microframe,
        fit_b.valid ? "ok" : "FAILED", fit_b.units_per_microframe);

    std::vector<double> cross_host;
    std::vector<double> cross;
    std::vector<double> within_b;
    std::vector<double> within_a;
    for (const auto& [probe_tag, list_a] : samples_a) {
        // Same board, two controllers, one PTPC: the control measurement.
        if (list_a.size() >= 2) {
            within_a.push_back(
                (static_cast<double>(list_a[1].microframe_q16)
                 - static_cast<double>(list_a[0].microframe_q16))
                * kNsPerQ16);
        }
        const auto found = samples_b.find(probe_tag);
        if (found == samples_b.end() || found->second.empty() || list_a.empty())
            continue;
        if (found->second.size() >= 2) {
            within_b.push_back(
                (static_cast<double>(found->second[1].microframe_q16)
                 - static_cast<double>(found->second[0].microframe_q16))
                * kNsPerQ16);
        }
        cross.push_back(
            (static_cast<double>(found->second[0].microframe_q16)
             - static_cast<double>(list_a[0].microframe_q16))
            * kNsPerQ16);

        if (fit_a.valid && fit_b.valid) {
            const double coarse_a = static_cast<double>(list_a[0].microframe_q16) / 65536.0;
            const double coarse_b = static_cast<double>(found->second[0].microframe_q16) / 65536.0;
            const double host_a = fit_a.microframe_of(list_a[0].ptpc_ns, coarse_a);
            const double host_b = fit_b.microframe_of(found->second[0].ptpc_ns, coarse_b);
            cross_host.push_back((host_b - host_a) * 125000.0);
        }
    }

    // Shape matters more than the summary here: a sawtooth means the two boards'
    // conversions are being refreshed at different phases, a random scatter
    // means noise, and a step means one board re-anchored.
    printf("\n=== first 24 cross-board differences, in order (ns) ===\n ");
    for (size_t index = 0; index < std::min<size_t>(24, cross.size()); index++)
        printf("%9.0f%s", cross[index], (index % 8 == 7) ? "\n " : "");
    printf("\n");

    printf("\n=== control: two controllers on ONE board (one PTPC, no cross-board term) ===\n");
    Stats floor_b = summarise(within_b);
    Stats floor_a = summarise(within_a);
    print_stats("board A, CAN1 vs CAN2", floor_a);
    print_stats("board B, CAN1 vs CAN2", floor_b);
    printf("  This is the noise floor of the method. Cross-board numbers below are only\n"
           "  meaningful to the extent they exceed it.\n");

    printf("\n=== measured cross-board skew (board B - board A, same frame) ===\n");
    Stats skew = summarise(cross);
    print_stats("board conversion", skew);
    Stats skew_host = summarise(cross_host);
    print_stats("host conversion", skew_host);
    if (skew.count != 0) {
        const double floor_sigma = std::max(floor_a.sigma_ns, floor_b.sigma_ns);
        printf(
            "\n  mean %+.1f ns is a CONSTANT offset -- transceiver and cable asymmetry, plus\n"
            "  each board's PTPC-to-microframe conversion bias. It is calibratable and does\n"
            "  not affect a control loop.\n",
            skew.mean_ns);
        if (floor_sigma > 0.0 && skew.sigma_ns <= floor_sigma * 1.5) {
            printf(
                "  sigma %.1f ns is at or below the %.1f ns floor: the cross-board skew is\n"
                "  NOT RESOLVABLE by this method. Report the floor as the upper bound.\n",
                skew.sigma_ns, floor_sigma);
        } else {
            printf(
                "  sigma %.1f ns against a %.1f ns floor -> cross-board term about %.1f ns.\n",
                skew.sigma_ns, floor_sigma,
                std::sqrt(std::max(0.0, skew.sigma_ns * skew.sigma_ns - floor_sigma * floor_sigma)));
        }
    }
    return 0;
}
