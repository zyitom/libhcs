#include "firmware/mc02/app/src/diag/loop_profile.hpp"

#if defined(libhcs_APP_LOOP_PROFILE) && libhcs_APP_LOOP_PROFILE

# include <array>
# include <cstdint>

# include <main.h>

# include "core/include/libhcs/data/datas.hpp"
# include "core/src/protocol/serializer.hpp"
# include "firmware/mc02/app/src/timer/timer.hpp"
# include "firmware/mc02/app/src/usb/helper.hpp"

namespace libhcs::firmware::diag::profile {

namespace {

constexpr std::uint32_t kEmitPeriodMs = 500U;
constexpr auto kSectionCount = static_cast<std::size_t>(Section::kCount);

constexpr std::array<const char*, kSectionCount> kNames = {
    "tud", "usb", "can", "uart", "imu", "led", "gpio", "other",
};

std::array<std::uint64_t, kSectionCount> accumulated{};
std::array<std::uint32_t, kSectionCount> worst{};
std::uint32_t pass_count = 0;
std::uint32_t worst_pass = 0;
std::uint32_t pass_start = 0;

std::uint32_t last_mark = 0;
auto current = Section::kOther;

std::uint32_t last_emit_ms = 0;

// 跨多次发送复用; 记录在此组装并交给 serializer, 其会在返回前拷入上行批量池。
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

} // namespace

void mark(Section section) {
    const auto now = DWT->CYCCNT;
    const auto index = static_cast<std::size_t>(current);
    const auto elapsed = now - last_mark;

    accumulated[index] += elapsed;
    if (elapsed > worst[index])
        worst[index] = elapsed;

    last_mark = now;
    current = section;
}

void end_pass() {
    mark(Section::kOther);

    const auto now = DWT->CYCCNT;
    const auto pass = now - pass_start;
    pass_start = now;
    if (pass > worst_pass && pass_count != 0U)
        worst_pass = pass;
    ++pass_count;

    const auto now_ms =
        static_cast<std::uint32_t>(timer::timer->timepoint().time_since_epoch().count() / 4000U);
    if (now_ms - last_emit_ms < kEmitPeriodMs)
        return;
    const auto window_ms = now_ms - last_emit_ms;
    last_emit_ms = now_ms;

    if (pass_count == 0U)
        return;

    // 输出格式: "loop n=<轮数> khz=<每 ms 轮数> avg=<平均周期> max=<最大周期>
    // | <段名> <占总量千分比> <平均周期> <最大周期> | ..."
    //
    // 用千分比而非百分比: 小段按百分比会全部显示为 0; 只用整数: 为一个诊断引入
    // 软浮点格式化不值得, 读作千分之几即可。
    char* cursor = line.data();
    const char* const end = line.data() + line.size();

    std::uint64_t total = 0;
    for (const auto value : accumulated)
        total += value;

    cursor = put_text(cursor, end, "loop n=");
    cursor = put_decimal(cursor, end, pass_count);
    cursor = put_text(cursor, end, " khz=");
    cursor = put_decimal(cursor, end, window_ms ? pass_count / window_ms : 0U);
    cursor = put_text(cursor, end, " avg=");
    cursor = put_decimal(cursor, end, static_cast<std::uint32_t>(total / pass_count));
    cursor = put_text(cursor, end, " max=");
    cursor = put_decimal(cursor, end, worst_pass);

    for (std::size_t i = 0; i < kSectionCount; ++i) {
        cursor = put_text(cursor, end, " | ");
        cursor = put_text(cursor, end, kNames[i]);
        cursor = put_text(cursor, end, " ");
        cursor = put_decimal(
            cursor, end, total ? static_cast<std::uint32_t>(accumulated[i] * 1000U / total) : 0U);
        cursor = put_text(cursor, end, " ");
        cursor = put_decimal(cursor, end, static_cast<std::uint32_t>(accumulated[i] / pass_count));
        cursor = put_text(cursor, end, " ");
        cursor = put_decimal(cursor, end, worst[i]);
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

    accumulated.fill(0);
    worst.fill(0);
    pass_count = 0;
    worst_pass = 0;
}

} // namespace libhcs::firmware::diag::profile

#endif
