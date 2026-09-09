#include "firmware/mc02/app/src/diag/usb_rx_hist.hpp"

#if defined(libhcs_APP_USB_RX_HIST) && libhcs_APP_USB_RX_HIST

# include <array>
# include <cstddef>
# include <cstdint>

# include <main.h>

# include "core/include/libhcs/data/datas.hpp"
# include "core/src/protocol/serializer.hpp"
# include "firmware/mc02/app/src/timer/timer.hpp"
# include "firmware/mc02/app/src/usb/helper.hpp"

namespace libhcs::firmware::diag::usb_rx_hist {

namespace {

constexpr std::uint32_t kEmitPeriodMs = 500U;
constexpr std::uint32_t kCyclesPerUs = 550U; // SYSCLK, see app.cpp

// Upper edges in microseconds. The interesting comparisons are the mode around
// 37 us (the flood's steady state) and anything a full 125 us microframe above
// it, so the resolution is concentrated there and coarse elsewhere.
constexpr std::array<std::uint32_t, 14> kEdgesUs = {
    10U, 20U, 30U, 34U, 38U, 42U, 46U, 55U, 70U, 90U, 120U, 160U, 220U, 400U,
};
constexpr std::size_t kBucketCount = kEdgesUs.size() + 1U;

std::array<std::uint32_t, kBucketCount> buckets{};
std::uint64_t accumulated_cycles = 0;
std::uint32_t sample_count = 0;
std::uint32_t worst_cycles = 0;
std::uint32_t best_cycles = 0xFFFFFFFFU;

std::uint32_t last_note_cycles = 0;
bool has_last_note = false;

std::uint32_t last_emit_ms = 0;

std::array<char, 320> line{};

char* put_decimal(char* cursor, const char* end, std::uint32_t value) {
    char digits[10];
    std::size_t count = 0;
    do {
        digits[count++] = static_cast<char>('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (count != 0U && cursor != end)
        *cursor++ = digits[--count];
    return cursor;
}

char* put_text(char* cursor, const char* end, const char* text) {
    while (*text != '\0' && cursor != end)
        *cursor++ = *text++;
    return cursor;
}

void emit(std::uint32_t window_ms) {
    // "rxgap n=<samples> khz=<packets per ms> min=<us> avg=<us.tenths> max=<us>
    // <edge>:<count> ...". The khz= token is also what makes mc02_packet_rate
    // print the line at all: it keys on that name to decide a kUart0 record is
    // a diagnostic worth showing.
    char* cursor = line.data();
    const char* const end = line.data() + line.size();

    const auto avg_cycles = static_cast<std::uint32_t>(accumulated_cycles / sample_count);

    cursor = put_text(cursor, end, "rxgap n=");
    cursor = put_decimal(cursor, end, sample_count);
    cursor = put_text(cursor, end, " khz=");
    cursor = put_decimal(cursor, end, window_ms ? sample_count / window_ms : 0U);
    cursor = put_text(cursor, end, " min=");
    cursor = put_decimal(cursor, end, best_cycles / kCyclesPerUs);
    cursor = put_text(cursor, end, " avg=");
    cursor = put_decimal(cursor, end, avg_cycles / kCyclesPerUs);
    cursor = put_text(cursor, end, ".");
    cursor = put_decimal(cursor, end, (avg_cycles % kCyclesPerUs) * 10U / kCyclesPerUs);
    cursor = put_text(cursor, end, " max=");
    cursor = put_decimal(cursor, end, worst_cycles / kCyclesPerUs);

    for (std::size_t i = 0; i < kBucketCount; ++i) {
        cursor = put_text(cursor, end, " ");
        if (i < kEdgesUs.size())
            cursor = put_decimal(cursor, end, kEdgesUs[i]);
        else
            cursor = put_text(cursor, end, "inf");
        cursor = put_text(cursor, end, ":");
        cursor = put_decimal(cursor, end, buckets[i]);
    }

    auto& serializer = usb::get_serializer();
    (void)serializer.write_uart(
        static_cast<core::protocol::FieldId>(data::DataId::kUart0),
        {
            .uart_data =
                {reinterpret_cast<const std::byte*>(line.data()),
                            static_cast<std::size_t>(cursor - line.data())},
            .idle_delimited = true
    },
        {});

    buckets.fill(0);
    accumulated_cycles = 0;
    sample_count = 0;
    worst_cycles = 0;
    best_cycles = 0xFFFFFFFFU;
}

} // namespace

void note() {
    const auto now = DWT->CYCCNT;
    const auto previous = last_note_cycles;
    last_note_cycles = now;

    if (!has_last_note) {
        has_last_note = true;
        return;
    }

    const auto elapsed = now - previous;
    const auto elapsed_us = elapsed / kCyclesPerUs;

    std::size_t bucket = kEdgesUs.size();
    for (std::size_t i = 0; i < kEdgesUs.size(); ++i) {
        if (elapsed_us < kEdgesUs[i]) {
            bucket = i;
            break;
        }
    }
    ++buckets[bucket];

    accumulated_cycles += elapsed;
    ++sample_count;
    if (elapsed > worst_cycles)
        worst_cycles = elapsed;
    if (elapsed < best_cycles)
        best_cycles = elapsed;

    const auto now_ms =
        static_cast<std::uint32_t>(timer::timer->timepoint().time_since_epoch().count() / 4000U);
    if (now_ms - last_emit_ms < kEmitPeriodMs)
        return;
    const auto window_ms = now_ms - last_emit_ms;
    last_emit_ms = now_ms;

    if (sample_count == 0U)
        return;
    emit(window_ms);
}

} // namespace libhcs::firmware::diag::usb_rx_hist

#endif
