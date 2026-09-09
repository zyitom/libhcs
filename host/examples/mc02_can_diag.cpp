// Decoder for mc02's libhcs_APP_CAN_DIAG telemetry (record version 64).
//
// mc02 had no CAN state channel at all, which is what stalled the August 2026
// forwarding investigation: from the host there was no way to separate "the frame
// never reached the wire" from "it was sent and lost". This prints the controller
// state the firmware now ships on the kUart0 uplink.
//
// Build the firmware with -Dlibhcs_APP_CAN_DIAG=ON; the channel is off by
// default because it adds counters to the forwarding hot path and occupies
// DataId::kUart0.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>
#include "common/multi_board.hpp"

namespace {

constexpr uint8_t kRecordMagic = 0xD1U;
constexpr uint8_t kRecordVersion = 64U;
constexpr size_t kCanCount = 3;

uint32_t get_u32(const std::byte* p) {
    uint32_t v = 0;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

// PSR.LEC / PSR.DLEC name the last protocol error the controller saw, which is
// what separates "nobody acknowledged my frame" from "the bits came back wrong".
const char* lec_name(uint32_t code) {
    static const char* const kNames[]{"none", "stuff", "form", "ack",
                                      "bit1", "bit0", "crc", "no-change"};
    return kNames[code & 0x7U];
}

struct Record {
    uint8_t sequence = 0;
    uint32_t tick_ms = 0;
    uint32_t main_loop = 0;
    struct Can {
        uint32_t isr, frames, tx_fail, uplink_drop, psr, ecr, txfqs;
    } can[kCanCount]{};
};

struct Sink final : public examples::BoardReceiver {
    std::atomic<uint32_t> records{0}, bad{0};
    Record last;
    bool have_last = false;

    void on_diagnostic(const libhcs::data::UartDataView& data) override {
        const auto payload = data.uart_data;
        constexpr size_t kSize = 8 + 4 + 4 + kCanCount * 7 * 4;
        if (payload.size() != kSize)
            return;
        if (std::to_integer<uint8_t>(payload[0]) != kRecordMagic)
            return;
        if (std::to_integer<uint8_t>(payload[1]) != kRecordVersion) {
            bad.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        Record r;
        r.sequence = std::to_integer<uint8_t>(payload[2]);
        r.tick_ms = get_u32(payload.data() + 8);
        r.main_loop = get_u32(payload.data() + 12);
        for (size_t i = 0; i < kCanCount; ++i) {
            const std::byte* b = payload.data() + 16 + i * 7 * 4;
            r.can[i] = {get_u32(b), get_u32(b + 4), get_u32(b + 8), get_u32(b + 12),
                        get_u32(b + 16), get_u32(b + 20), get_u32(b + 24)};
        }
        last = r;
        have_last = true;
        records.fetch_add(1, std::memory_order_relaxed);
    }
};

} // namespace

int main(int argc, char** argv) {
    const uint32_t secs = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 3;
    Sink sink;
    auto board = examples::connect_any(sink, std::getenv("HCS_BOARD_A") ?: "");
    if (!board) {
        fprintf(stderr, "no board\n");
        return 1;
    }
    printf("board = %.*s, listening %u s for CAN telemetry\n",
           (int)board->name().size(), board->name().data(), secs);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{secs};
    while (std::chrono::steady_clock::now() < deadline) {
        board->transmit([](examples::BoardTransmitter&) {});
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }

    const uint32_t n = sink.records.load();
    printf("\nrecords: %u (%.0f/s)   version mismatches: %u\n", n, (double)n / secs,
           sink.bad.load());
    if (!n) {
        printf("\nNo telemetry. Either the firmware was not built with\n"
               "-Dlibhcs_APP_CAN_DIAG=ON, or DataId::kUart0 is not reaching the host.\n");
        return 1;
    }

    const Record& r = sink.last;
    printf("seq %u, board tick %u ms, main loop %u iterations per record\n",
           r.sequence, r.tick_ms, r.main_loop);
    printf("\n%4s %9s %9s %9s %12s %10s %6s %6s %7s\n", "can", "isr", "frames",
           "tx_fail", "uplink_drop", "LEC/DLEC", "TEC", "REC", "tx_free");
    for (size_t i = 0; i < kCanCount; ++i) {
        const auto& c = r.can[i];
        printf("%4zu %9u %9u %9u %12u %5s/%-5s %6u %6u %7u%s\n", i + 1, c.isr, c.frames,
               c.tx_fail, c.uplink_drop, lec_name(c.psr), lec_name(c.psr >> 8),
               c.ecr & 0xFFU, (c.ecr >> 8) & 0x7FU, c.txfqs & 0x3FU,
               (c.psr & (1U << 7)) ? "  BUS_OFF" : "");
    }
    printf("\nmc02_can_diag: PASS\n");
    return 0;
}
