// Host-path USB round-trip probe -- the instrument behind HOST_TUNING.md's
// host-side numbers.
//
// WHAT IT MEASURES: one synchronous control transfer (GET_DESCRIPTOR/DEVICE) to
// the board's EP0, timed in the host clock:
//
//   steady_clock -> usbfs SUBMITURB -> xHCI ring -> wire -> device EP0 (handled
//   by the device USB stack, no application code) -> completion IRQ -> usbfs
//   wakeup -> poll() returns -> steady_clock
//
// That is the SAME host-side submit/wakeup path a bulk request-response uses,
// minus the board's application turnaround and minus the bulk NAK-retry
// scheduling. So:
//
//   USE IT FOR   A/B-ing host settings -- CPU frequency, C-states, cpuidle
//                governor, IRQ placement, IOMMU mode. The knob either moves this
//                number or it does not touch the host path at all.
//   DO NOT USE   as board+USB RTT. The absolute value is not comparable to
//                can_loopback_latency, which measures a bulk round trip through
//                the firmware and is quantized by the host controller's
//                microframe-granular retry of a NAKing bulk endpoint.
//
// Two details decide whether a measurement means anything:
//
//   1. The inter-sample gap must be long enough for the core to actually go
//      idle, or a C-state / frequency A/B measures nothing. Default 1000 us
//      matches a 1 kHz control loop.
//   2. The core's clock is sampled during the run and reported. It is not
//      decoration: on this machine the whole difference between "tuned" and
//      "untuned" turned out to be frequency residency, not C-state exit
//      latency, and without the clock column that is invisible.
//
// Needs root (raw USB device access). Configuration is by environment variable
// so an A/B script can sweep it without rebuilding:
//
//   HCS_RTT_LABEL    text printed in front of the result row  (default "run")
//   HCS_RTT_CPU      CPU to pin to                            (default 7)
//   HCS_RTT_SAMPLES  samples, first 100 discarded as warm-up  (default 3000)
//   HCS_RTT_GAP_US   idle gap between samples                 (default 1000)

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <libusb-1.0/libusb.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

constexpr uint16_t kVendorId = 0xa511; // Helios, every board
constexpr int kWarmupSamples = 100;
constexpr unsigned int kTransferTimeoutMs = 100;
constexpr int kFreqSampleInterval = 100; // samples between clock readings
constexpr int kDeviceDescriptorSize = 18;

int env_int(const char* name, int fallback) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0')
        return fallback;
    char* end = nullptr;
    const auto value = std::strtol(raw, &end, 10);
    if (end == raw || *end != '\0')
        return fallback;
    return static_cast<int>(value);
}

double percentile(const std::vector<double>& sorted, double p) {
    if (sorted.empty())
        return 0.0;
    const auto idx = static_cast<size_t>(static_cast<double>(sorted.size() - 1) * p);
    return sorted[idx];
}

// Current clock of <cpu> in MHz, or 0 if cpufreq does not export it.
//
// Do not trust this on an isolated core. Once the CPU is in nohz_full the
// kernel refuses to IPI it just to sample APERF/MPERF, so scaling_cur_freq
// falls back to cpuinfo_min_freq: it reported a flat 400 MHz while turbostat
// measured the same core at 4.3 GHz. Kept only as the fallback for CPUs where
// the MSR path below is unavailable. See CpuClock.
double read_cpu_mhz(int cpu) {
    std::array<char, 128> path{};
    const int written = std::snprintf(
        path.data(), path.size(), "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", cpu);
    if (written <= 0 || static_cast<size_t>(written) >= path.size())
        return 0.0;

    FILE* file = std::fopen(path.data(), "re");
    if (file == nullptr)
        return 0.0;
    int khz = 0;
    const int scanned = std::fscanf(file, "%d", &khz);
    if (std::fclose(file) != 0)
        return 0.0;
    return scanned == 1 ? static_cast<double>(khz) / 1000.0 : 0.0;
}

// Clock of the CPU this thread is pinned to, read straight from APERF/MPERF.
//
// This is the only reading that survives isolation, and the clock column is the
// whole point of the tool (HOST_TUNING.md section 1 concluded "the mechanism is
// frequency residency, not C-state exit latency" purely from it) -- a column
// that silently reads 400 MHz on an isolated core would invert that conclusion.
//
// APERF and MPERF only advance in C0, so their ratio is the frequency while
// actually working (turbostat's Bzy_MHz), not an idle-weighted average. That is
// what the historical numbers recorded, and for a 1 kHz loop that idles 98% of
// the time the two differ by two orders of magnitude.
class CpuClock {
public:
    explicit CpuClock(int cpu) {
        std::array<char, 64> path{};
        const int written = std::snprintf(path.data(), path.size(), "/dev/cpu/%d/msr", cpu);
        if (written <= 0 || static_cast<size_t>(written) >= path.size())
            return;
        fd_ = ::open(path.data(), O_RDONLY | O_CLOEXEC);
        if (fd_ < 0)
            return;

        // MSR_PLATFORM_INFO bits 15:8 are the max non-turbo ratio, and MPERF
        // ticks at that ratio times the 100 MHz bus clock.
        uint64_t platform_info = 0;
        if (read_msr(kMsrPlatformInfo, &platform_info))
            base_mhz_ = static_cast<double>((platform_info >> 8) & 0xFF) * 100.0;
        if (base_mhz_ <= 0.0)
            reset();
    }

    ~CpuClock() { reset(); }

    CpuClock(const CpuClock&) = delete;
    CpuClock& operator=(const CpuClock&) = delete;

    bool available() const { return fd_ >= 0; }

    // MHz since the previous call, or 0 on the first call and on any failure.
    double sample() {
        uint64_t aperf = 0;
        uint64_t mperf = 0;
        if (!available() || !read_msr(kMsrAperf, &aperf) || !read_msr(kMsrMperf, &mperf))
            return 0.0;

        const uint64_t delta_aperf = aperf - last_aperf_;
        const uint64_t delta_mperf = mperf - last_mperf_;
        const bool primed = last_mperf_ != 0;
        last_aperf_ = aperf;
        last_mperf_ = mperf;
        if (!primed || delta_mperf == 0)
            return 0.0;
        return base_mhz_ * static_cast<double>(delta_aperf) / static_cast<double>(delta_mperf);
    }

private:
    static constexpr uint32_t kMsrPlatformInfo = 0xCE;
    static constexpr uint32_t kMsrMperf = 0xE7;
    static constexpr uint32_t kMsrAperf = 0xE8;

    bool read_msr(uint32_t index, uint64_t* out) const {
        return ::pread(fd_, out, sizeof(*out), static_cast<off_t>(index))
            == static_cast<ssize_t>(sizeof(*out));
    }

    void reset() {
        if (fd_ >= 0)
            ::close(fd_);
        fd_ = -1;
    }

    int fd_ = -1;
    double base_mhz_ = 0.0;
    uint64_t last_aperf_ = 0;
    uint64_t last_mperf_ = 0;
};

// Pin and prioritize like the real transport's event thread, so the measured
// path sees the same scheduling treatment.
void setup_realtime(int cpu) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0)
        printf("Warning: failed to pin to cpu%d\n", cpu);
    sched_param param{};
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
        printf("Warning: no SCHED_FIFO (run as root for stable numbers)\n");
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        printf("Warning: mlockall failed\n");
}

// First device with our vendor id that can actually be opened. EP0 needs no
// interface claim, so this does not disturb a session already using the board.
libusb_device_handle* open_any_board() {
    libusb_device** list = nullptr;
    const ssize_t count = libusb_get_device_list(nullptr, &list);
    if (count < 0)
        return nullptr;

    libusb_device_handle* handle = nullptr;
    for (ssize_t i = 0; i < count; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(list[i], &desc) != 0)
            continue;
        if (desc.idVendor != kVendorId)
            continue;
        if (libusb_open(list[i], &handle) == 0)
            break;
        handle = nullptr;
    }
    libusb_free_device_list(list, 1);
    return handle;
}

} // namespace

int main() {
    const char* label_env = std::getenv("HCS_RTT_LABEL");
    const char* label = (label_env != nullptr && *label_env != '\0') ? label_env : "run";
    const int cpu = env_int("HCS_RTT_CPU", 7);
    const int samples = env_int("HCS_RTT_SAMPLES", 3000);
    const auto gap = std::chrono::microseconds{env_int("HCS_RTT_GAP_US", 1000)};

    setup_realtime(cpu);

    if (libusb_init(nullptr) != 0) {
        fprintf(stderr, "libusb_init failed\n");
        return 1;
    }
    libusb_device_handle* handle = open_any_board();
    if (handle == nullptr) {
        fprintf(stderr, "No %04x:* device could be opened (run as root).\n", kVendorId);
        libusb_exit(nullptr);
        return 1;
    }

    std::array<unsigned char, kDeviceDescriptorSize> descriptor{};
    std::vector<double> rtt_us;
    rtt_us.reserve(static_cast<size_t>(std::max(samples, 0)));
    int failed = 0;
    double mhz_sum = 0.0;
    double mhz_max = 0.0;
    int mhz_count = 0;

    // Primed here so the first in-loop sample already has a delta to work with.
    CpuClock clock{cpu};
    (void)clock.sample();

    for (int i = 0; i < samples; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        const int rc = libusb_control_transfer(
            handle, LIBUSB_ENDPOINT_IN, LIBUSB_REQUEST_GET_DESCRIPTOR, LIBUSB_DT_DEVICE << 8, 0,
            descriptor.data(), static_cast<uint16_t>(descriptor.size()), kTransferTimeoutMs);
        const auto t1 = std::chrono::steady_clock::now();

        if (rc < 0) {
            ++failed;
        } else if (i >= kWarmupSamples) {
            const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
            rtt_us.push_back(static_cast<double>(ns) / 1e3);
        }

        if (i % kFreqSampleInterval == 0) {
            const double mhz = clock.available() ? clock.sample() : read_cpu_mhz(cpu);
            if (mhz > 0.0) {
                mhz_sum += mhz;
                mhz_max = std::max(mhz_max, mhz);
                ++mhz_count;
            }
        }
        std::this_thread::sleep_for(gap);
    }

    libusb_close(handle);
    libusb_exit(nullptr);

    if (rtt_us.empty()) {
        fprintf(stderr, "No samples collected (%d transfers failed).\n", failed);
        return 1;
    }

    std::sort(rtt_us.begin(), rtt_us.end());
    printf(
        "%-26s n=%zu min=%.1f p50=%.1f p90=%.1f p99=%.1f p99.9=%.1f max=%.1f us", label,
        rtt_us.size(), rtt_us.front(), percentile(rtt_us, 0.50), percentile(rtt_us, 0.90),
        percentile(rtt_us, 0.99), percentile(rtt_us, 0.999), rtt_us.back());
    if (mhz_count > 0)
        printf("  | cpu%d %.0f/%.0f MHz mean/max", cpu, mhz_sum / mhz_count, mhz_max);
    if (failed > 0)
        printf("  | %d failed", failed);
    printf("\n");
    return 0;
}
