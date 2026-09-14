#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <libhcs/time/microframe_source.hpp>

namespace libhcs::host::time {
namespace {

// xHCI 1.2, 5.3: the capability registers sit at the start of BAR0.
//   0x00  CAPLENGTH   (u8)   offset to the operational registers
//   0x02  HCIVERSION  (u16)  BCD interface version
//   0x18  RTSOFF      (u32)  runtime register space offset, low 5 bits reserved
// and 5.5.1: MFINDEX is the first dword of the runtime space.
constexpr std::size_t kCapLengthOffset = 0x00;
constexpr std::size_t kHciVersionOffset = 0x02;
constexpr std::size_t kRtsOffOffset = 0x18;
constexpr std::uint32_t kRtsOffMask = ~std::uint32_t{0x1F};

// One page would do for the capability registers, but RTSOFF has been seen at
// 0x2000 and the spec does not bound it tightly; 64 KiB covers every real
// controller while staying a single mapping.
constexpr std::size_t kWindowSize = 65536;

constexpr std::uint16_t kMinHciVersion = 0x0090; // 0.9x predates xHCI 1.0 but exists
constexpr std::uint16_t kMaxHciVersion = 0x0200;
constexpr std::uint8_t kMinCapLength = 0x20;
constexpr std::uint8_t kMaxCapLength = 0x80;

// Behavioural validation. 50 ms at 8 kHz is 400 counts, so quantization alone
// is 0.25% -- tight enough that "not a microframe counter" cannot pass, and
// loose enough that a busy machine's scheduling does not fail a good one. The
// 2 ms window this replaced admitted only 16 counts and could not do better
// than +-6%, which is how a first cut of this check reported 0.95x on a
// perfectly healthy controller. The width detection below runs the same ratio
// over a 300 ms window, so only the ratio bounds are shared.
constexpr double kMinValidationRatio = 0.95;
constexpr double kMaxValidationRatio = 1.05;

// Consistency of one sample against elapsed host time, in microframes. A
// healthy counter disagrees with the host clock only by their relative rate
// error (500 ppm worst case here, so 1.6 microframes over a 400 ms gap); a
// STOPPED counter disagrees by the whole gap. The floor keeps short gaps from
// tripping on their own quantization, the fraction keeps long ones from
// needing a floor large enough to hide a stall.
constexpr double kConsistencyFloor = 64.0;
constexpr double kConsistencyFraction = 0.005;

// Adaptive lead on the predicted edge. Starts generous because it costs one
// busy window to find out what this machine's sleep granularity really is, and
// converges from there. The floor is a few reads' worth: below that the window
// cannot contain a transition even when the prediction is perfect.
constexpr std::chrono::nanoseconds kInitialGuard{80'000};
constexpr std::chrono::nanoseconds kMinGuard{4'000};
constexpr std::chrono::nanoseconds kMaxGuard{2 * 125'000};

// Additive increase when the window opened too late. Sized well above the
// spread of this kernel's wakeup so one overshoot moves the guard clear of it
// rather than creeping back into the same failure.
constexpr std::chrono::nanoseconds kGuardIncrease{16'000};

std::int64_t raw_now_ns() noexcept {
    timespec ts{};
    (void)clock_gettime(CLOCK_MONOTONIC_RAW, &ts); // cannot fail for a valid clock id
    return (ts.tv_sec * 1'000'000'000LL) + ts.tv_nsec;
}

std::int64_t monotonic_now_ns() noexcept {
    timespec ts{};
    (void)clock_gettime(CLOCK_MONOTONIC, &ts); // cannot fail for a valid clock id
    return (ts.tv_sec * 1'000'000'000LL) + ts.tv_nsec;
}

void sleep_until_monotonic(std::int64_t target_ns) noexcept {
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(target_ns / 1'000'000'000LL);
    // tv_nsec is `long` by POSIX, not a width this code gets to choose.
    // NOLINTNEXTLINE(google-runtime-int)
    ts.tv_nsec = static_cast<long>(target_ns % 1'000'000'000LL);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {}
}

std::int64_t to_ns(MicroframeSource::Clock::time_point when) noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(when.time_since_epoch()).count();
}

bool looks_like_pci_address(std::string_view text) noexcept {
    // "0000:00:14.0"
    if (text.size() != 12)
        return false;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        if (i == 4 || i == 7) {
            if (c != ':')
                return false;
        } else if (i == 10) {
            if (c != '.')
                return false;
        } else if (std::isxdigit(static_cast<unsigned char>(c)) == 0) {
            return false;
        }
    }
    return true;
}

} // namespace

// The mapping, the counter extension and the edge predictor. Held behind a
// pointer so the class stays movable (std::expected needs that) without the
// mutex leaking into the public header.
struct MicroframeSource::Impl {
    // Cleanup lives here, not in ~MicroframeSource, so that every early return
    // from for_pci_device -- each of which drops a half-built Impl -- unmaps and
    // closes. A validation failure is the expected path, not the rare one.
    ~Impl() {
        if (bar != nullptr)
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast): munmap needs it
            munmap(const_cast<std::uint8_t*>(bar), kWindowSize);
        if (fd >= 0)
            close(fd);
    }

    Impl() = default;
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;

    std::string pci_device;
    int fd = -1;
    volatile std::uint8_t* bar = nullptr;
    volatile std::uint32_t* mfindex = nullptr;

    mutable std::mutex mutex;
    State state = State::kValid;

    // Wrap width of THIS counter, detected at construction: 1024 (spec 10-bit
    // MFINDEX) or 16384 (14-bit, observed on Intel xHC). Zero only on a
    // moved-from source.
    std::uint32_t modulus = 0;

    // 64-bit extension.
    bool seeded = false;
    std::uint32_t last_raw = 0;
    std::int64_t last_raw_ns = 0;
    std::uint64_t extended = 0;

    // Construction-time measurements.
    std::chrono::nanoseconds read_cost{0};
    double validated_rate_hz = 0.0;

    // steady_clock is CLOCK_MONOTONIC on this platform -- verified rather than
    // assumed, because the edge predictor sleeps on CLOCK_MONOTONIC against a
    // deadline computed from steady_clock. If they ever diverge, prediction is
    // disabled and every hunt is a cold one: slower, never wrong.
    bool steady_is_monotonic = false;

    // Edge predictor. wake_error_high is a decaying maximum of how late
    // clock_nanosleep has been returning, which is the only quantity the guard
    // actually has to cover; it decays so that one bad wakeup does not pin the
    // guard wide forever.
    std::int64_t wake_error_high = 0;
    bool has_anchor = false;
    std::uint64_t anchor_microframe = 0;
    std::int64_t anchor_ns = 0; // steady domain
    std::uint64_t last_edge_microframe = 0;
    std::int64_t last_edge_ns = 0;
    std::chrono::nanoseconds guard = kInitialGuard;

    EdgeStats stats{};

    [[nodiscard]] std::uint32_t read_raw() const noexcept {
        // The 14-bit mask is a superset: a 10-bit register simply reads zero
        // above bit 9. Deltas must be masked with the DETECTED modulus, not
        // this one -- see extend_locked().
        return *mfindex & (kCounterModulus - 1);
    }

    // Continues the 64-bit axis, or fails. Resolving the wrap from the counter
    // alone is impossible past half the wrap period, so elapsed host time picks
    // the wrap count -- and the same comparison, run the other way, is what
    // detects a counter that has stopped.
    bool extend_locked(std::uint32_t raw, std::int64_t now_raw_ns) noexcept {
        if (!seeded) {
            seeded = true;
            last_raw = raw;
            last_raw_ns = now_raw_ns;
            extended = raw;
            return true;
        }

        const std::int64_t elapsed_ns = now_raw_ns - last_raw_ns;
        if (elapsed_ns < 0
            || elapsed_ns > std::chrono::nanoseconds{MicroframeSource::kMaxGap}.count()) {
            state = State::kLostSync;
            return false;
        }

        const double predicted = static_cast<double>(elapsed_ns)
                               / static_cast<double>(MicroframeSource::kMicroframePeriod.count());
        // The DETECTED width, not the 14-bit read mask: masking a 10-bit
        // counter's deltas with 16383 turns its 1024 wrap into an apparent
        // 15/16-of-span jump, i.e. a confidently wrong kCounterStopped.
        const std::uint32_t raw_delta = (raw - last_raw) & (modulus - 1);

        const double wraps = std::max(
            std::round((predicted - static_cast<double>(raw_delta)) / static_cast<double>(modulus)),
            0.0);
        const std::uint64_t delta = raw_delta + (static_cast<std::uint64_t>(wraps) * modulus);

        const double residual = std::fabs(predicted - static_cast<double>(delta));
        const double tolerance = std::max(kConsistencyFloor, predicted * kConsistencyFraction);
        if (residual > tolerance) {
            // The counter did not advance the way elapsed time says it should.
            // The overwhelmingly likely cause is that it stopped -- a suspended
            // or reset controller -- and continuing here is exactly how a
            // plausible wrong timestamp would be produced.
            state = State::kCounterStopped;
            return false;
        }

        extended += delta;
        last_raw = raw;
        last_raw_ns = now_raw_ns;
        return true;
    }
};

MicroframeSource::MicroframeSource(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}

MicroframeSource::MicroframeSource(MicroframeSource&&) noexcept = default;
MicroframeSource& MicroframeSource::operator=(MicroframeSource&&) noexcept = default;

MicroframeSource::~MicroframeSource() = default;

std::string_view MicroframeSource::describe(Error error) noexcept {
    switch (error) {
    case Error::kNoPciDevice: return "no PCI device backs that USB bus (ARM SoC, or bad bus)";
    case Error::kPermissionDenied: return "permission denied (needs root or CAP_SYS_RAWIO)";
    case Error::kMapFailed: return "could not map the controller's BAR";
    case Error::kNotXhci: return "not an xHCI register window (or the controller is in D3)";
    case Error::kCounterStopped: return "MFINDEX does not advance at 8 kHz";
    }
    return "unknown";
}

std::string_view MicroframeSource::describe(State state) noexcept {
    switch (state) {
    case State::kValid: return "valid";
    case State::kLostSync: return "lost sync (sampled too rarely to resolve the wrap)";
    case State::kCounterStopped: return "counter stopped (controller suspended or reset)";
    }
    return "unknown";
}

std::string pci_device_for_usb_bus(int bus_number) {
    std::error_code error;
    const std::filesystem::path link = "/sys/bus/usb/devices/usb" + std::to_string(bus_number);
    const std::filesystem::path resolved = std::filesystem::canonical(link, error);
    if (error)
        return {};
    std::string parent = resolved.parent_path().filename().string();
    if (!looks_like_pci_address(parent))
        return {};
    if (!std::filesystem::exists("/sys/bus/pci/devices/" + parent, error))
        return {};
    return parent;
}

std::vector<int> usb_buses_of_pci_device(std::string_view pci_device) {
    std::vector<int> buses;
    std::error_code error;
    for (const auto& entry : std::filesystem::directory_iterator{"/sys/bus/usb/devices", error}) {
        const std::string name = entry.path().filename().string();
        if (!name.starts_with("usb"))
            continue;
        int bus = 0;
        try {
            bus = std::stoi(name.substr(3));
        } catch (const std::exception&) {
            continue;
        }
        if (pci_device_for_usb_bus(bus) == pci_device)
            buses.push_back(bus);
    }
    std::ranges::sort(buses);
    return buses;
}

std::vector<std::string> usb_controller_pci_devices() {
    std::vector<std::string> devices;
    std::error_code error;
    for (const auto& entry : std::filesystem::directory_iterator{"/sys/bus/pci/devices", error}) {
        std::ifstream class_file{entry.path() / "class"};
        std::string class_code;
        if (!(class_file >> class_code))
            continue;
        // PCI class 0x0C03 is "USB controller"; the low byte is the programming
        // interface (0x30 for xHCI, 0x20 EHCI, 0x10 OHCI, 0x00 UHCI).
        if (!class_code.starts_with("0x0c03"))
            continue;
        devices.push_back(entry.path().filename().string());
    }
    std::ranges::sort(devices);
    return devices;
}

std::expected<MicroframeSource, MicroframeSource::Error>
    MicroframeSource::for_usb_bus(int bus_number) {
    const std::string device = pci_device_for_usb_bus(bus_number);
    if (device.empty())
        return std::unexpected{Error::kNoPciDevice};
    return for_pci_device(device);
}

std::expected<MicroframeSource, MicroframeSource::Error>
    MicroframeSource::for_pci_device(std::string_view pci_device) {
    auto impl = std::make_unique<Impl>();
    impl->pci_device = std::string{pci_device};

    const std::string path = "/sys/bus/pci/devices/" + impl->pci_device + "/resource0";

    // Read-only if the kernel allows it. Some sysfs resource mappings insist on
    // a writable descriptor even for PROT_READ; falling back keeps those
    // working without ever asking for PROT_WRITE.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): POSIX open() is variadic.
    impl->fd = open(path.c_str(), O_RDONLY | O_SYNC);
    if (impl->fd < 0 && (errno == EACCES || errno == EPERM))
        return std::unexpected{Error::kPermissionDenied};
    if (impl->fd < 0)
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): POSIX open() is variadic.
        impl->fd = open(path.c_str(), O_RDWR | O_SYNC);
    if (impl->fd < 0)
        return std::unexpected{
            errno == EACCES || errno == EPERM ? Error::kPermissionDenied : Error::kNoPciDevice};

    void* mapped = mmap(nullptr, kWindowSize, PROT_READ, MAP_SHARED, impl->fd, 0);
    if (mapped == MAP_FAILED) {
        // A read-only descriptor is refused by some kernels for MAP_SHARED.
        close(impl->fd);
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): POSIX open() is variadic.
        impl->fd = open(path.c_str(), O_RDWR | O_SYNC);
        if (impl->fd < 0)
            return std::unexpected{Error::kPermissionDenied};
        mapped = mmap(nullptr, kWindowSize, PROT_READ, MAP_SHARED, impl->fd, 0);
    }
    if (mapped == MAP_FAILED)
        return std::unexpected{Error::kMapFailed};
    impl->bar = static_cast<volatile std::uint8_t*>(mapped);

    // Layout plausibility. A controller parked in D3 answers every MMIO read
    // with all-ones, which fails here rather than later: that is the difference
    // between "this source is unavailable" and "this source is lying".
    const std::uint8_t cap_length = *(impl->bar + kCapLengthOffset);
    const std::uint16_t hci_version =
        *reinterpret_cast<volatile std::uint16_t*>(impl->bar + kHciVersionOffset);
    if (cap_length < kMinCapLength || cap_length > kMaxCapLength)
        return std::unexpected{Error::kNotXhci};
    if (hci_version < kMinHciVersion || hci_version >= kMaxHciVersion)
        return std::unexpected{Error::kNotXhci};

    // Read RTSOFF rather than hardcoding it: it is 0x2000 on Tiger Lake and is
    // not required to be that anywhere else. This is the one field that makes
    // the code portable across controller vendors.
    const std::uint32_t rtsoff =
        *reinterpret_cast<volatile std::uint32_t*>(impl->bar + kRtsOffOffset) & kRtsOffMask;
    if (rtsoff == 0 || rtsoff + sizeof(std::uint32_t) > kWindowSize)
        return std::unexpected{Error::kNotXhci};
    impl->mfindex = reinterpret_cast<volatile std::uint32_t*>(impl->bar + rtsoff);

    // Behavioural validation and counter-width detection in one pass. xHCI
    // 5.5.1 gives MFINDEX 10 bits; the controller this was built on implements
    // 14. Over a 300 ms window a 10-bit counter MUST cross its 128 ms wrap at
    // least twice, and every crossing shows up as a masked delta past half the
    // 14-bit span -- which a 14-bit counter cannot reach in 300 ms at 8 kHz
    // (2400 counts). Every way of being otherwise wrong -- wrong register,
    // suspended, all-ones, a stopped counter -- fails the per-interval
    // consistency and rate checks below.
    static constexpr std::chrono::milliseconds kDetectionWindow{300};
    static constexpr std::chrono::milliseconds kDetectionInterval{30};

    struct DetectionSample {
        std::int64_t gap_ns;
        std::uint32_t delta;
    };
    std::vector<DetectionSample> detection;
    detection.reserve(kDetectionWindow / kDetectionInterval);

    std::uint32_t previous_raw = impl->read_raw();
    const std::int64_t detection_start_ns = raw_now_ns();
    std::int64_t previous_ns = detection_start_ns;
    bool wide_wrap_seen = false;
    for (std::size_t sample = 0; sample < kDetectionWindow / kDetectionInterval; ++sample) {
        sleep_until_monotonic(
            monotonic_now_ns() + std::chrono::nanoseconds{kDetectionInterval}.count());
        const std::uint32_t raw = impl->read_raw();
        const std::int64_t now_ns = raw_now_ns();

        const std::uint32_t delta = (raw - previous_raw) & (kCounterModulus - 1);
        if (delta > kCounterModulus / 2)
            wide_wrap_seen = true;
        detection.push_back({now_ns - previous_ns, delta});
        previous_raw = raw;
        previous_ns = now_ns;
    }

    impl->modulus = wide_wrap_seen ? kCounterModulus / 16U : kCounterModulus;

    std::uint64_t total_advanced = 0;
    for (const DetectionSample& interval : detection) {
        const double predicted = static_cast<double>(interval.gap_ns)
                               / static_cast<double>(MicroframeSource::kMicroframePeriod.count());
        const double wraps = std::max(
            std::round(
                (predicted - static_cast<double>(interval.delta))
                / static_cast<double>(impl->modulus)),
            0.0);
        const double advanced =
            static_cast<double>(interval.delta) + (wraps * static_cast<double>(impl->modulus));
        const double tolerance = std::max(kConsistencyFloor, predicted * kConsistencyFraction);
        if (std::fabs(predicted - advanced) > tolerance)
            return std::unexpected{Error::kCounterStopped};
        total_advanced += static_cast<std::uint64_t>(std::llround(advanced));
    }

    const double seconds = static_cast<double>(previous_ns - detection_start_ns) / 1e9;
    const double expected = seconds * 8000.0;
    if (expected <= 0.0)
        return std::unexpected{Error::kCounterStopped};
    const double ratio = static_cast<double>(total_advanced) / expected;
    if (ratio < kMinValidationRatio || ratio > kMaxValidationRatio)
        return std::unexpected{Error::kCounterStopped};
    impl->validated_rate_hz = static_cast<double>(total_advanced) / seconds;

    // Cost of one read, timed in batches so the two clock_gettime calls a
    // single-read timing would need do not land inside the number. What a
    // caller pays per sample() is this plus those two, which is why sample()
    // costs a little more than read_cost() reports.
    static constexpr int kCostBatches = 32;
    static constexpr int kCostReadsPerBatch = 64;
    std::vector<std::int64_t> costs;
    costs.reserve(kCostBatches);
    for (int batch = 0; batch < kCostBatches; ++batch) {
        const std::int64_t started = raw_now_ns();
        // The accumulator is volatile so the loop survives -O3 without needing
        // an asm barrier, which would be a GNU extension in a build that
        // compiles as strict ISO C++.
        volatile std::uint32_t sink = 0;
        for (int i = 0; i < kCostReadsPerBatch; ++i)
            sink = sink + impl->read_raw();
        const std::int64_t finished = raw_now_ns();
        costs.push_back((finished - started) / kCostReadsPerBatch);
    }
    std::ranges::sort(costs);
    impl->read_cost = std::chrono::nanoseconds{costs[costs.size() / 2]};

    // Verify, do not assume, that steady_clock is CLOCK_MONOTONIC: the edge
    // predictor sleeps on one and computes deadlines from the other.
    const std::int64_t steady_ns = to_ns(Clock::now());
    const std::int64_t mono_ns = monotonic_now_ns();
    impl->steady_is_monotonic = std::llabs(steady_ns - mono_ns) < 1'000'000;

    return MicroframeSource{std::move(impl)};
}

std::optional<MicroframeSource::Sample> MicroframeSource::sample() {
    const std::scoped_lock guard{impl_->mutex};
    if (impl_->state != State::kValid)
        return std::nullopt;

    const std::uint32_t raw = impl_->read_raw();
    const auto at = Clock::now();
    const std::int64_t raw_ns = raw_now_ns();

    if (!impl_->extend_locked(raw, raw_ns))
        return std::nullopt;

    return Sample{.microframe = impl_->extended, .at = at, .raw_ns = raw_ns};
}

std::optional<MicroframeSource::Edge> MicroframeSource::sample_edge() {
    const std::scoped_lock guard{impl_->mutex};
    if (impl_->state != State::kValid)
        return std::nullopt;

    ++impl_->stats.attempts;

    const std::int64_t period_ns = kMicroframePeriod.count();
    std::int64_t poll_start_ns = to_ns(Clock::now());
    bool predicted = false;
    std::int64_t requested_wake_ns = 0;

    // Predict the next edge from the longest baseline available: the anchor is
    // the first edge of this run, so the period estimate improves with every
    // edge and prediction error over hundreds of microframes stays well inside
    // the guard. Without an anchor -- or without a usable sleep clock -- fall
    // through to a cold hunt.
    if (impl_->has_anchor && impl_->steady_is_monotonic
        && impl_->last_edge_microframe > impl_->anchor_microframe) {
        const double fitted_period =
            static_cast<double>(impl_->last_edge_ns - impl_->anchor_ns)
            / static_cast<double>(impl_->last_edge_microframe - impl_->anchor_microframe);
        const std::int64_t guard_ns = impl_->guard.count();

        // First predicted edge far enough ahead that we can still sleep to it.
        const double ahead =
            static_cast<double>(poll_start_ns + guard_ns - impl_->last_edge_ns) / fitted_period;
        const std::uint64_t steps = static_cast<std::uint64_t>(std::floor(ahead)) + 1;
        const std::int64_t target_ns =
            impl_->last_edge_ns
            + static_cast<std::int64_t>(std::llround(fitted_period * static_cast<double>(steps)));

        if (target_ns - guard_ns > poll_start_ns) {
            requested_wake_ns = target_ns - guard_ns;
            sleep_until_monotonic(requested_wake_ns);
            poll_start_ns = to_ns(Clock::now());
            predicted = true;
            impl_->stats.predicted_lead_total +=
                std::chrono::nanoseconds{target_ns - poll_start_ns};
        }
    }

    if (!predicted)
        ++impl_->stats.cold_hunts;

    // The window always spans at least one full period, so a transition is
    // guaranteed to be inside it even when the sleep overshoots and the
    // predicted edge has already gone by. A miss therefore means something
    // pathological, not a slightly late wakeup.
    const std::int64_t deadline_ns = poll_start_ns + period_ns + impl_->guard.count();

    std::uint32_t previous_raw = impl_->read_raw();
    auto previous_at = Clock::now();
    std::int64_t previous_raw_ns = raw_now_ns();
    std::uint64_t reads = 1;

    std::optional<Edge> found;
    while (true) {
        const std::uint32_t current_raw = impl_->read_raw();
        const auto current_at = Clock::now();
        const std::int64_t current_raw_ns = raw_now_ns();
        ++reads;

        if (current_raw != previous_raw) {
            if (!impl_->extend_locked(current_raw, current_raw_ns)) {
                impl_->stats.reads += reads;
                return std::nullopt;
            }
            found = Edge{
                .microframe = impl_->extended,
                .not_before = previous_at,
                .not_after = current_at,
                .raw_not_before_ns = previous_raw_ns,
                .raw_not_after_ns = current_raw_ns,
            };
            break;
        }

        previous_raw = current_raw;
        previous_at = current_at;
        previous_raw_ns = current_raw_ns;

        if (to_ns(current_at) > deadline_ns)
            break;
    }

    const std::int64_t finished_ns = to_ns(Clock::now());
    impl_->stats.reads += reads;
    impl_->stats.busy += std::chrono::nanoseconds{finished_ns - poll_start_ns};

    if (!found) {
        ++impl_->stats.misses;
        impl_->guard = std::min(kMaxGuard, impl_->guard * 2);
        return std::nullopt;
    }

    ++impl_->stats.hits;

    // Adapt the lead. Two earlier versions of this got the SIGNAL wrong, which
    // is worth recording because both looked reasonable:
    //
    //   v1 adapted on the poll duration. Sleeping to (target - guard) makes the
    //      poll last about `guard` by construction, so the loop had no signal
    //      in it at all and the guard never left its initial 80 us.
    //   v2 adapted on the sleep's wake error, which IS a real signal -- this
    //      kernel returns from clock_nanosleep about 56 us late -- and drove
    //      the guard down to just cover it. That made things WORSE (106 us per
    //      edge), because the wake error varies: landing a microsecond LATE
    //      costs a whole 125 us period waiting for the next edge, while landing
    //      early costs only the extra polling. Sizing the guard at the mean of
    //      an asymmetric cost is the wrong target.
    //
    // So the signal is the outcome itself: a poll longer than half a period
    // means the target edge had already gone by. Increase additively on that
    // and decay slowly on success -- the guard then settles just above the
    // point where overshoots become rare, wherever that is on this machine.
    // [Both failures caught by section 5 of microframe_source_test, 2026-09-08.]
    if (predicted) {
        const std::int64_t wake_error_ns = poll_start_ns - requested_wake_ns;
        impl_->wake_error_high =
            std::max(wake_error_ns, impl_->wake_error_high - (impl_->wake_error_high / 64));
        impl_->stats.wake_error_high = std::chrono::nanoseconds{impl_->wake_error_high};
        impl_->stats.wake_error_total += std::chrono::nanoseconds{wake_error_ns};
        ++impl_->stats.wake_samples;

        const std::int64_t polled_ns = to_ns(found->not_after) - poll_start_ns;
        if (polled_ns > period_ns / 2) {
            ++impl_->stats.overshoots;
            impl_->guard = std::min(kMaxGuard, impl_->guard + kGuardIncrease);
        } else {
            const std::chrono::nanoseconds floor_guard{
                std::max(kMinGuard.count(), 4 * impl_->read_cost.count())};
            impl_->guard = std::max(floor_guard, impl_->guard - impl_->guard / 64);
        }
    }
    impl_->stats.guard = impl_->guard;

    if (!impl_->has_anchor) {
        impl_->has_anchor = true;
        impl_->anchor_microframe = found->microframe;
        impl_->anchor_ns = to_ns(found->midpoint());
    }
    impl_->last_edge_microframe = found->microframe;
    impl_->last_edge_ns = to_ns(found->midpoint());

    return found;
}

MicroframeSource::State MicroframeSource::state() const noexcept {
    const std::scoped_lock guard{impl_->mutex};
    return impl_->state;
}

std::uint32_t MicroframeSource::counter_modulus() const noexcept { return impl_->modulus; }

const std::string& MicroframeSource::pci_device() const noexcept { return impl_->pci_device; }

std::chrono::nanoseconds MicroframeSource::read_cost() const noexcept { return impl_->read_cost; }

double MicroframeSource::validated_rate_hz() const noexcept { return impl_->validated_rate_hz; }

MicroframeSource::EdgeStats MicroframeSource::edge_stats() const noexcept {
    const std::scoped_lock guard{impl_->mutex};
    return impl_->stats;
}

} // namespace libhcs::host::time
