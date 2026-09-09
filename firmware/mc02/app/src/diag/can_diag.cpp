#include "firmware/mc02/app/src/diag/can_diag.hpp"

#if defined(libhcs_APP_CAN_DIAG) && libhcs_APP_CAN_DIAG

#include <atomic>
#include <cstring>

#include <fdcan.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "firmware/mc02/app/src/timer/timer.hpp"
#include "firmware/mc02/app/src/usb/helper.hpp"

namespace libhcs::firmware::diag {
namespace {

constexpr std::size_t kCanCount = 3;
constexpr std::uint32_t kEmitPeriodMs = 100U;

struct CanCounters {
    std::atomic<std::uint32_t> isr_entries{0};
    std::atomic<std::uint32_t> frames{0};
    std::atomic<std::uint32_t> tx_fail{0};
    std::atomic<std::uint32_t> uplink_drop{0};
};

CanCounters counters[kCanCount];
std::uint32_t main_loop_count = 0;
std::uint32_t last_main_loop_count = 0;
std::uint32_t last_emit_ms = 0;
std::uint8_t record_sequence = 0;

std::byte* put_u32(std::byte* cursor, std::uint32_t value) {
    std::memcpy(cursor, &value, sizeof(value));
    return cursor + sizeof(value);
}

FDCAN_GlobalTypeDef* can_instance(std::size_t index) {
    switch (index) {
    case 0: return FDCAN1;
    case 1: return FDCAN2;
    default: return FDCAN3;
    }
}

} // namespace

void note_isr_entry(std::size_t can_index) {
    if (can_index < kCanCount)
        counters[can_index].isr_entries.fetch_add(1, std::memory_order::relaxed);
}

void note_frame(std::size_t can_index) {
    if (can_index < kCanCount)
        counters[can_index].frames.fetch_add(1, std::memory_order::relaxed);
}

void note_tx_fail(std::size_t can_index) {
    if (can_index < kCanCount)
        counters[can_index].tx_fail.fetch_add(1, std::memory_order::relaxed);
}

void note_uplink_drop(std::size_t can_index) {
    if (can_index < kCanCount)
        counters[can_index].uplink_drop.fetch_add(1, std::memory_order::relaxed);
}

// Main loop only, so a plain increment is enough.
void note_main_loop() { main_loop_count++; }

void poll() {
    // timepoint() counts quarter-microseconds (1 MHz timer, CNT << 2), so 4000
    // ticks is one millisecond. Wraps every ~18 min; only deltas are used.
    const auto now_ms = static_cast<std::uint32_t>(
        timer::timer->timepoint().time_since_epoch().count() / 4000U);
    if (now_ms - last_emit_ms < kEmitPeriodMs)
        return;
    last_emit_ms = now_ms;

    auto& serializer = usb::get_serializer();

    // 8-byte header + tick + main-loop delta, then 7 words per controller.
    constexpr std::size_t kFixedSize = 8 + 4 + 4;
    constexpr std::size_t kPerCanSize = 7 * 4;
    constexpr std::size_t kRecordSize = kFixedSize + kCanCount * kPerCanSize;

    std::byte record[kRecordSize];
    std::byte* cursor = record;

    *cursor++ = static_cast<std::byte>(kRecordMagic);
    *cursor++ = static_cast<std::byte>(kRecordVersion);
    *cursor++ = static_cast<std::byte>(record_sequence++);
    *cursor++ = static_cast<std::byte>(kCanCount);
    cursor = put_u32(cursor, static_cast<std::uint32_t>(kRecordSize));
    cursor = put_u32(cursor, now_ms);
    cursor = put_u32(cursor, main_loop_count - last_main_loop_count);
    last_main_loop_count = main_loop_count;

    for (std::size_t index = 0; index < kCanCount; index++) {
        auto* const can = can_instance(index);
        cursor = put_u32(cursor, counters[index].isr_entries.load(std::memory_order::relaxed));
        cursor = put_u32(cursor, counters[index].frames.load(std::memory_order::relaxed));
        cursor = put_u32(cursor, counters[index].tx_fail.load(std::memory_order::relaxed));
        cursor = put_u32(cursor, counters[index].uplink_drop.load(std::memory_order::relaxed));
        // PSR is partly clear-on-read for LEC/DLEC: reading it here consumes the
        // code, which is acceptable because nothing else in this firmware reads
        // it -- but it does mean two consumers would race, so keep it that way.
        cursor = put_u32(cursor, can->PSR);
        cursor = put_u32(cursor, can->ECR);
        cursor = put_u32(cursor, can->TXFQS);
    }

    // Best effort by design: if the batch pool is full the record is dropped
    // rather than retried, exactly like a forwarded CAN frame. A gap in the
    // sequence numbers is itself a symptom worth seeing on the host.
    (void)serializer.write_uart(
        static_cast<core::protocol::FieldId>(data::DataId::kUart0),
        {.uart_data = {record, kRecordSize}, .idle_delimited = true}, {});
}

} // namespace libhcs::firmware::diag

#endif
