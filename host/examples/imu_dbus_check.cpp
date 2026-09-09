// IMU + DBUS coverage check.
//
// These two channels had no test at all: every existing example drives CAN and
// UART, and nothing verified that the accelerometer / gyroscope / temperature
// uplinks or the DBUS receiver channel produce data. This tool just listens.
//
// The IMU is checked for plausibility, not accuracy: on a board sitting still on
// a bench the accelerometer must read ~1 g total (gravity) and the gyroscope
// ~0 deg/s. That catches a dead SPI link, a stuck sample and swapped axes, which
// is what an untested channel is likely to be hiding.
//
// DBUS needs a physical DJI receiver to say anything. Absence of DBUS frames is
// reported as "not wired", not as a failure, since the bench rig has no receiver.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include "common/multi_board.hpp"

namespace {

struct Sink final : public examples::BoardReceiver {
    std::atomic<uint32_t> accel_n{0}, gyro_n{0}, temp_n{0}, dbus_n{0};
    std::atomic<uint32_t> dbus_bytes{0}, dbus_sized{0}, dbus_sane{0};
    std::atomic<int> c0{0}, c1{0}, c2{0}, c3{0}, s1{0}, s2{0};
    std::atomic<int> ax{0}, ay{0}, az{0}, gx{0}, gy{0}, gz{0}, temp_raw{0};

    void on_accelerometer(const libhcs::data::ImuAccelerometerDataView& d) override {
        ax.store(d.x); ay.store(d.y); az.store(d.z);
        accel_n.fetch_add(1, std::memory_order_relaxed);
    }
    void on_gyroscope(const libhcs::data::ImuGyroscopeDataView& d) override {
        gx.store(d.x); gy.store(d.y); gz.store(d.z);
        gyro_n.fetch_add(1, std::memory_order_relaxed);
    }
    void on_temperature(const libhcs::data::ImuTemperatureDataView& d) override {
        temp_raw.store(d.raw_register_value);
        temp_n.fetch_add(1, std::memory_order_relaxed);
    }
    // DBUS is a fixed 18-byte frame at ~74 Hz. Counting frames only proves the
    // UART is delivering something; decoding proves it is the real protocol and
    // that the byte order survived the uplink. The four sticks are 11-bit fields
    // centred on 1024 with a 660-count travel, so a decoded value outside
    // [364, 1684] means the bit packing is wrong, not that a stick moved.
    void on_dbus(const libhcs::data::UartDataView& d) override {
        dbus_n.fetch_add(1, std::memory_order_relaxed);
        dbus_bytes.fetch_add(d.uart_data.size(), std::memory_order_relaxed);
        if (d.uart_data.size() != 18)
            return;
        dbus_sized.fetch_add(1, std::memory_order_relaxed);
        const auto* b = reinterpret_cast<const uint8_t*>(d.uart_data.data());
        const int ch0 = ((b[0] | b[1] << 8) & 0x07FF);
        const int ch1 = ((b[1] >> 3 | b[2] << 5) & 0x07FF);
        const int ch2 = ((b[2] >> 6 | b[3] << 2 | b[4] << 10) & 0x07FF);
        const int ch3 = ((b[4] >> 1 | b[5] << 7) & 0x07FF);
        const int sw1 = ((b[5] >> 4) & 0x0003);
        const int sw2 = ((b[5] >> 6) & 0x0003);
        c0.store(ch0); c1.store(ch1); c2.store(ch2); c3.store(ch3);
        s1.store(sw1); s2.store(sw2);
        const bool sane = ch0 >= 364 && ch0 <= 1684 && ch1 >= 364 && ch1 <= 1684
                       && ch2 >= 364 && ch2 <= 1684 && ch3 >= 364 && ch3 <= 1684
                       && sw1 >= 1 && sw1 <= 3 && sw2 >= 1 && sw2 <= 3;
        if (sane)
            dbus_sane.fetch_add(1, std::memory_order_relaxed);
    }
};

} // namespace

int main(int argc, char** argv) {
    const uint32_t secs = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 3;
    Sink sink;
    auto board = examples::connect_any(sink, std::getenv("HCS_BOARD_A") ?: "");
    if (!board) { fprintf(stderr, "no board\n"); return 1; }
    printf("board = %.*s   imu=%s dbus=%s\n", (int)board->name().size(),
           board->name().data(), board->has_imu() ? "yes" : "no",
           board->has_dbus() ? "yes" : "no");
    if (!board->has_imu() && !board->has_dbus()) {
        printf("nothing to check on this board\n");
        return 0;
    }

    // The session only streams once it is established; keep the link alive by
    // transmitting nothing in particular while we listen.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{secs};
    while (std::chrono::steady_clock::now() < deadline) {
        board->transmit([](examples::BoardTransmitter&) {});
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }

    int failures = 0;
    if (board->has_imu()) {
        const uint32_t a = sink.accel_n.load(), g = sink.gyro_n.load(),
                       t = sink.temp_n.load();
        printf("\nIMU samples in %u s: accel=%u gyro=%u temp=%u\n", secs, a, g, t);
        if (!a || !g) {
            printf("  FAIL: no IMU data (SPI link dead, or init() not called)\n");
            ++failures;
        } else {
            // Raw BMI088 counts. The firmware configures Range::k6G (see
            // spi/bmi088/accel.hpp), so 1 g is 32768/6 = 5461 counts. A board at
            // rest must read close to that in magnitude whatever its orientation.
            const double axv = sink.ax.load(), ayv = sink.ay.load(), azv = sink.az.load();
            const double mag = std::sqrt(axv * axv + ayv * ayv + azv * azv);
            const double gxv = sink.gx.load(), gyv = sink.gy.load(), gzv = sink.gz.load();
            const double spin = std::sqrt(gxv * gxv + gyv * gyv + gzv * gzv);
            const double g_counts = 32768.0 / 6.0;  // Range::k6G
            printf("  accel raw [%+6d %+6d %+6d] |a|=%.0f counts = %.2f g\n",
                   sink.ax.load(), sink.ay.load(), sink.az.load(), mag, mag / g_counts);
            printf("  gyro  raw [%+6d %+6d %+6d] |w|=%.0f counts\n",
                   sink.gx.load(), sink.gy.load(), sink.gz.load(), spin);
            printf("  temp  raw 0x%04X\n", sink.temp_raw.load());
            if (mag < 0.5 * g_counts || mag > 1.8 * g_counts) {
                printf("  FAIL: |a| = %.2f g at rest -- expected ~1 g "
                       "(dead axis, wrong range, or stuck sample)\n", mag / g_counts);
                ++failures;
            }
            // 2000 deg/s over int16 => ~16.4 counts per deg/s; allow 5 deg/s.
            if (spin > 5.0 * 16.4) {
                printf("  FAIL: |w| too large for a board at rest\n");
                ++failures;
            }
            if (!t) printf("  note: no temperature samples (probe is low-rate)\n");
        }
    }
    if (board->has_dbus()) {
        const uint32_t d = sink.dbus_n.load(), sized = sink.dbus_sized.load(),
                       sane = sink.dbus_sane.load();
        printf("\nDBUS frames in %u s: %u (%.0f Hz), %u bytes\n", secs, d,
               (double)d / secs, sink.dbus_bytes.load());
        if (!d) {
            printf("  no receiver wired -- not a failure\n");
        } else {
            printf("  18-byte frames: %u/%u   decoded in range: %u/%u\n", sized, d, sane, sized);
            printf("  sticks [%4d %4d %4d %4d]  switches [%d %d]  (centre 1024, range 364-1684)\n",
                   sink.c0.load(), sink.c1.load(), sink.c2.load(), sink.c3.load(),
                   sink.s1.load(), sink.s2.load());
            if (sized != d) {
                printf("  FAIL: %u frames were not 18 bytes -- framing or idle "
                       "delimiting is wrong\n", d - sized);
                ++failures;
            } else if (sane != sized) {
                printf("  FAIL: %u frames decoded out of range -- bit packing or byte "
                       "order is wrong\n", sized - sane);
                ++failures;
            }
        }
    }
    printf("\n%s\n", failures ? "imu_dbus_check: FAIL" : "imu_dbus_check: PASS");
    return failures ? 1 : 0;
}
