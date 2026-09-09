#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <dirent.h>

#include <libhcs/board/mc02.hpp>

// Instrument for the transport's event loop, which the throughput tools cannot
// see. mc02_packet_rate measures the loop while traffic is flowing, where an
// arriving completion wakes the poll immediately and the poll timeout never
// expires. The two costs that only appear when nothing is arriving are:
//
//   IDLE WAKEUPS -- the event thread parks in a bounded poll and re-tests its
//       exit condition when the timeout fires. Every expiry is a scheduler
//       wakeup on a link with no traffic on it. Counted here as the process's
//       voluntary context switches, which is what a blocking poll produces.
//
//   TEARDOWN -- ~Usb() sets its stop flag and then waits for the event thread
//       to notice. The thread only re-tests between poll calls, so a teardown
//       that begins just after the thread parked waits out the whole timeout.
//
// Both are properties of how the loop waits, so both are measured with the link
// deliberately idle: open the board, touch nothing, then close it.

using libhcs::board::Mc02;

namespace {

std::string first_board_serial() {
    std::string result;
    DIR* dir = opendir("/sys/bus/usb/devices");
    if (!dir)
        return result;
    while (dirent* entry = readdir(dir)) {
        std::string base = std::string{"/sys/bus/usb/devices/"} + entry->d_name + "/";
        char buffer[64] = {};
        auto read_attribute = [&](const char* name) -> std::string {
            FILE* file = fopen((base + name).c_str(), "re");
            if (!file)
                return {};
            std::string value = fgets(buffer, sizeof(buffer), file) ? buffer : "";
            fclose(file);
            while (!value.empty() && (value.back() == '\n' || value.back() == '\r'))
                value.pop_back();
            return value;
        };
        if (read_attribute("idVendor") != "a511" || read_attribute("idProduct") != "0723")
            continue;
        result = read_attribute("serial");
        if (!result.empty())
            break;
    }
    closedir(dir);
    return result;
}

// Voluntary context switches: the counter a blocking poll increments once per
// park. Involuntary switches are preemption and say nothing about the loop.
//
// Summed over /proc/self/task/*, NOT read from /proc/self/status: that file
// reports the thread group leader only, and the leader is not the thread doing
// the waiting. Reading it shows 1 switch for the measuring thread's own sleep
// and nothing at all from the event thread.
long voluntary_context_switches() {
    DIR* dir = opendir("/proc/self/task");
    if (!dir)
        return -1;
    long total = 0;
    while (dirent* entry = readdir(dir)) {
        if (entry->d_name[0] == '.')
            continue;
        const std::string path = std::string{"/proc/self/task/"} + entry->d_name + "/status";
        FILE* file = fopen(path.c_str(), "re");
        if (!file)
            continue;
        char line[256];
        while (fgets(line, sizeof(line), file)) {
            if (strncmp(line, "voluntary_ctxt_switches:", 24) == 0) {
                total += strtol(line + 24, nullptr, 10);
                break;
            }
        }
        fclose(file);
    }
    closedir(dir);
    return total;
}

// usbfs hands out DMA-coherent buffers by mmap, so every libusb_dev_mem_alloc()
// is a separate mapping of at least one page regardless of the size asked for.
// Counting the mappings against the bytes actually requested is what shows
// whether the transport is paying for pages it does not use.
struct DevMemFootprint {
    long mappings = 0;
    long kilobytes = 0;
};

DevMemFootprint dev_mem_footprint() {
    DevMemFootprint result;
    FILE* file = fopen("/proc/self/maps", "re");
    if (!file)
        return result;
    char line[512];
    while (fgets(line, sizeof(line), file)) {
        // usbfs mappings name the device node they came from.
        if (!strstr(line, "/dev/bus/usb/"))
            continue;
        unsigned long begin = 0, end = 0;
        if (sscanf(line, "%lx-%lx", &begin, &end) == 2) {
            result.mappings++;
            result.kilobytes += static_cast<long>((end - begin) / 1024);
        }
    }
    fclose(file);
    return result;
}

class Idle final : public Mc02::Callback {
public:
    explicit Idle(std::string_view serial) { board_ = std::make_unique<Mc02>(*this, serial); }
    void close() { board_.reset(); }

    // Put transfers in flight so a teardown has something to race. Closing an
    // idle link only exercises the path where every buffer is already back in
    // its pool, which is the easy half of the slab lifetime.
    void flood(std::chrono::milliseconds duration) {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            for (int i = 0; i < 64; i++) {
                auto builder = board_->start_transmit();
                static const std::byte payload[8] = {};
                builder.uart1_transmit({.uart_data = payload, .idle_delimited = true});
            }
        }
    }

private:
    std::unique_ptr<Mc02> board_;
};

} // namespace

// Repeated open/flood/close. Two things can only fail here and not in a single
// cycle: a mapping that is never returned (the count would climb cycle over
// cycle) and a teardown that unmaps a slab while a transfer still owns a buffer
// out of it (which the transport reports rather than doing).
int churn(const std::string& serial, int cycles) {
    printf("Board: %s\n", serial.c_str());
    printf("%d open/flood/close cycles; mappings must not grow.\n\n", cycles);
    for (int cycle = 1; cycle <= cycles; cycle++) {
        Idle link{serial};
        link.flood(std::chrono::milliseconds{120});
        const DevMemFootprint footprint = dev_mem_footprint();
        const auto start = std::chrono::steady_clock::now();
        link.close();
        const auto end = std::chrono::steady_clock::now();
        printf(
            "cycle %2d: %ld mappings, %ld KiB, teardown %.2f ms\n", cycle, footprint.mappings,
            footprint.kilobytes,
            std::chrono::duration<double, std::milli>{end - start}.count());
    }
    const DevMemFootprint after = dev_mem_footprint();
    printf("\nafter all cycles: %ld mappings, %ld KiB still mapped\n", after.mappings,
           after.kilobytes);
    return after.mappings == 0 ? 0 : 1;
}

int main(int argc, char** argv) {
    const double idle_seconds = argc > 1 ? strtod(argv[1], nullptr) : 10.0;

    const auto serial = first_board_serial();
    if (serial.empty()) {
        printf("No mc02 board found (a511:0723).\n");
        return 1;
    }
    if (argc > 1 && strcmp(argv[1], "churn") == 0)
        return churn(serial, argc > 2 ? atoi(argv[2]) : 10);

    printf("Board: %s\n", serial.c_str());
    printf("Holding the link open and idle for %.1f s, then timing teardown.\n\n", idle_seconds);

    using Clock = std::chrono::steady_clock;

    const auto open_start = Clock::now();
    Idle idle{serial};
    const auto open_end = Clock::now();

    // Settle: enumeration and the first transfer submissions produce switches
    // that are not the steady-state idle behaviour being measured.
    std::this_thread::sleep_for(std::chrono::milliseconds{500});

    const DevMemFootprint dev_mem = dev_mem_footprint();

    const long switches_before = voluntary_context_switches();
    const auto idle_start = Clock::now();
    std::this_thread::sleep_for(std::chrono::duration<double>{idle_seconds});
    const auto idle_end = Clock::now();
    const long switches_after = voluntary_context_switches();

    const auto teardown_start = Clock::now();
    idle.close();
    const auto teardown_end = Clock::now();

    const auto ms = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>{b - a}.count();
    };
    const double idle_elapsed = std::chrono::duration<double>{idle_end - idle_start}.count();

    // One sleep of the measuring thread itself is included in the count; the
    // event thread's contribution is everything above that floor of 1.
    const long switches = switches_after - switches_before;
    printf("open        : %8.2f ms\n", ms(open_start, open_end));
    printf("idle        : %8.2f s\n", idle_elapsed);
    printf("  vol ctxsw : %8ld  (%.1f /s)\n", switches, switches / idle_elapsed);
    printf("teardown    : %8.2f ms   <-- ~Usb()\n", ms(teardown_start, teardown_end));
    printf("dev_mem     : %8ld mappings, %ld KiB mapped\n", dev_mem.mappings, dev_mem.kilobytes);
    return 0;
}
