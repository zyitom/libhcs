#include "firmware/hpm_board/app/src/sync/sof_probe.hpp"

#if defined(libhcs_APP_SOF_DIAG) && libhcs_APP_SOF_DIAG

# include <atomic>
# include <cstddef>
# include <cstring>

# include <hpm_soc.h>
# include <hpm_usb_regs.h>

# include "core/include/libhcs/data/datas.hpp"
# include "core/src/protocol/protocol.hpp"
# include "core/src/protocol/serializer.hpp"
# include "firmware/hpm_board/app/src/link/uplink.hpp"
# include "firmware/hpm_board/app/src/timer/timer.hpp"
# include "firmware/hpm_board/app/src/utility/interrupt_lock.hpp"

namespace libhcs::firmware::sync::sof_probe {
namespace {

constexpr std::uint32_t kEmitPeriodMs = 100U;
constexpr std::uint32_t kFrindexMask = USB_FRINDEX_FRINDEX_MASK;

// ISR 共享状态一律用 std::atomic 而非仅靠中断锁保护的普通标量: SDK 的
// disable_global_irq() 只是一条无 memory clobber 的裸 csrrc, 能挡住中断,
// 挡不住编译器把值留在寄存器里跨过它。下方的排空仍会取锁, 避免读到 ISR 只
// 写到一半的异常条目。
struct AtomicAnomaly {
    std::atomic<std::uint32_t> previous_frame{0};
    std::atomic<std::uint32_t> current_frame{0};
    std::atomic<std::uint32_t> delta{0};
    std::atomic<std::uint32_t> interval_ticks{0};
    // 本中断发生时的状态字, 而非事后重读: 与帧索引停止推进同时出现的端口
    // 状态变化、复位或挂起正是最可能的解释, 主循环再看时它早已消失。
    std::atomic<std::uint32_t> usbsts{0};
    std::atomic<std::uint32_t> portsc1{0};
    // 异常 SOF 当时的本地机器定时器读数, 单位 1/4 us。记录本就带毫秒
    // tick, 此项补充的是同一记录周期内的先后与间距 -- 区分异常扎堆还是
    // 零散。
    std::atomic<std::uint32_t> timestamp{0};
};

// 线上每条异常条目的字数。
constexpr std::size_t kAnomalyWords = 7;

std::atomic<std::uint32_t> sof_count{0};
std::atomic<std::uint32_t> delta_histogram[kDeltaBuckets];
std::atomic<std::uint32_t> advanced_within_isr{0};
std::atomic<std::uint32_t> anomaly_total{0};
std::atomic<std::uint32_t> anomaly_stored{0};
AtomicAnomaly anomalies[kAnomalyCapacity];

std::atomic<std::uint32_t> last_frame{0};
std::atomic<std::uint32_t> last_sof_time{0};
std::atomic<std::uint32_t> microframe_low{0};
std::atomic<std::uint32_t> microframe_high{0};

std::atomic<std::uint32_t> interval_min{0xFFFFFFFFU};
std::atomic<std::uint32_t> interval_max{0};
std::atomic<std::uint32_t> interval_sum_low{0};
std::atomic<std::uint32_t> interval_sum_high{0};
std::atomic<std::uint32_t> interval_count{0};

// 仅 ISR 私有; 其他上下文不触碰。
std::uint32_t isr_previous_frame = 0;
std::uint32_t isr_previous_time = 0;
bool isr_previous_valid = false;

std::uint32_t last_emit_tick = 0;
std::uint8_t record_sequence = 0;

std::byte* put_u32(std::byte* cursor, std::uint32_t value) {
    std::memcpy(cursor, &value, sizeof(value));
    return cursor + sizeof(value);
}

std::uint32_t load(const std::atomic<std::uint32_t>& value) {
    return value.load(std::memory_order::relaxed);
}

void store(std::atomic<std::uint32_t>& value, std::uint32_t next) {
    value.store(next, std::memory_order::relaxed);
}

} // namespace

void note_sof(
    std::uint32_t frame_first, std::uint32_t frame_second, std::uint32_t now, std::uint32_t status,
    std::uint32_t portsc1) {
    store(sof_count, load(sof_count) + 1U);
    store(last_frame, frame_first);
    store(last_sof_time, now);

    if (frame_second != frame_first)
        store(advanced_within_isr, load(advanced_within_isr) + 1U);

    if (!isr_previous_valid) {
        isr_previous_valid = true;
        isr_previous_frame = frame_first;
        isr_previous_time = now;
        return;
    }

    const std::uint32_t delta = (frame_first - isr_previous_frame) & kFrindexMask;
    const std::uint32_t interval = now - isr_previous_time;
    isr_previous_frame = frame_first;
    isr_previous_time = now;

    const std::uint32_t bucket = delta < (kDeltaBuckets - 1U) ? delta : (kDeltaBuckets - 1U);
    store(delta_histogram[bucket], load(delta_histogram[bucket]) + 1U);

    const std::uint32_t low = load(microframe_low) + delta;
    if (low < delta)
        store(microframe_high, load(microframe_high) + 1U);
    store(microframe_low, low);

    if (interval < load(interval_min))
        store(interval_min, interval);
    if (interval > load(interval_max))
        store(interval_max, interval);
    const std::uint32_t sum = load(interval_sum_low) + interval;
    if (sum < interval)
        store(interval_sum_high, load(interval_sum_high) + 1U);
    store(interval_sum_low, sum);
    store(interval_count, load(interval_count) + 1U);

    if (delta == 1U)
        return;

    store(anomaly_total, load(anomaly_total) + 1U);
    const std::uint32_t slot = load(anomaly_stored);
    if (slot >= kAnomalyCapacity)
        return;
    store(anomalies[slot].previous_frame, (frame_first - delta) & kFrindexMask);
    store(anomalies[slot].current_frame, frame_first);
    store(anomalies[slot].delta, delta);
    store(anomalies[slot].interval_ticks, interval);
    store(anomalies[slot].usbsts, status);
    store(anomalies[slot].portsc1, portsc1);
    store(anomalies[slot].timestamp, now);
    store(anomaly_stored, slot + 1U);
}

void poll(std::uint32_t tick) {
    if (tick - last_emit_tick < kEmitPeriodMs)
        return;
    last_emit_tick = tick;

    // 无会话期间计数器照常累计; 会话建立后的首条记录带上此前积攒的全部
    // 数据。
    if (!link::uplink_enabled())
        return;

    // 在主循环采样, 刻意置于任何中断上下文之外: 把主机可跨板比对的帧索引
    // 与主机自身感知的到达时刻配对, 两条独立记录流才能互证两板数的是同一
    // 路 SOF 流。
    const std::uint32_t emit_frame = HPM_USB0->FRINDEX & kFrindexMask;
    const std::uint32_t emit_time = timer::Timer::timestamp_quarter_us();

    // 4 字节头, 之后 18 个固定字加 delta 直方图: record_size、tick、
    // sof_count、[直方图]、races、anomaly_total、last frame + time、
    // 以及 microframe low + high、emit frame + time、interval min/max/
    // sum lo/sum hi/count、timer frequency、异常条目数。
    constexpr std::size_t kFixedSize = 4 + 4 * (18 + kDeltaBuckets);
    constexpr std::size_t kAnomalySize = 4 * kAnomalyWords;
    constexpr std::size_t kMaxRecordSize = kFixedSize + kAnomalyCapacity * kAnomalySize;

    std::uint32_t histogram[kDeltaBuckets] = {};
    std::uint32_t anomaly_snapshot[kAnomalyCapacity][kAnomalyWords] = {};
    std::uint32_t stored = 0;
    std::uint32_t total_sof = 0;
    std::uint32_t races = 0;
    std::uint32_t anomalies_seen = 0;
    std::uint32_t frame_at_sof = 0;
    std::uint32_t time_at_sof = 0;
    std::uint32_t uframe_low = 0;
    std::uint32_t uframe_high = 0;
    std::uint32_t min_interval = 0;
    std::uint32_t max_interval = 0;
    std::uint32_t sum_low = 0;
    std::uint32_t sum_high = 0;
    std::uint32_t samples = 0;

    {
        const utility::InterruptLockGuard guard;

        total_sof = load(sof_count);
        races = load(advanced_within_isr);
        anomalies_seen = load(anomaly_total);
        frame_at_sof = load(last_frame);
        time_at_sof = load(last_sof_time);
        uframe_low = load(microframe_low);
        uframe_high = load(microframe_high);

        for (std::size_t index = 0; index < kDeltaBuckets; index++)
            histogram[index] = load(delta_histogram[index]);

        // 间隔统计只描述一个记录周期, 故在此清零。直方图与各计数器刻意
        // 保持累计: 单条记录被丢弃时, 该周期内的 delta 异常不能随之丢失。
        min_interval = load(interval_min);
        max_interval = load(interval_max);
        sum_low = load(interval_sum_low);
        sum_high = load(interval_sum_high);
        samples = load(interval_count);
        store(interval_min, 0xFFFFFFFFU);
        store(interval_max, 0);
        store(interval_sum_low, 0);
        store(interval_sum_high, 0);
        store(interval_count, 0);

        stored = load(anomaly_stored);
        for (std::uint32_t index = 0; index < stored; index++) {
            anomaly_snapshot[index][0] = load(anomalies[index].previous_frame);
            anomaly_snapshot[index][1] = load(anomalies[index].current_frame);
            anomaly_snapshot[index][2] = load(anomalies[index].delta);
            anomaly_snapshot[index][3] = load(anomalies[index].interval_ticks);
            anomaly_snapshot[index][4] = load(anomalies[index].usbsts);
            anomaly_snapshot[index][5] = load(anomalies[index].portsc1);
            anomaly_snapshot[index][6] = load(anomalies[index].timestamp);
        }
        store(anomaly_stored, 0);
    }

    const std::size_t record_size = kFixedSize + stored * kAnomalySize;

    std::byte record[kMaxRecordSize];
    std::byte* cursor = record;

    *cursor++ = static_cast<std::byte>(kRecordMagic);
    *cursor++ = static_cast<std::byte>(kRecordVersion);
    *cursor++ = static_cast<std::byte>(record_sequence++);
    *cursor++ = static_cast<std::byte>(
        ((HPM_USB0->USBINTR & USB_USBINTR_SRE_MASK) != 0U ? 0x01U : 0x00U)
        | (total_sof != 0U ? 0x02U : 0x00U));
    cursor = put_u32(cursor, static_cast<std::uint32_t>(record_size));
    cursor = put_u32(cursor, tick);
    cursor = put_u32(cursor, total_sof);
    for (const std::uint32_t bucket : histogram)
        cursor = put_u32(cursor, bucket);
    cursor = put_u32(cursor, races);
    cursor = put_u32(cursor, anomalies_seen);
    cursor = put_u32(cursor, frame_at_sof);
    cursor = put_u32(cursor, time_at_sof);
    cursor = put_u32(cursor, uframe_low);
    cursor = put_u32(cursor, uframe_high);
    cursor = put_u32(cursor, emit_frame);
    cursor = put_u32(cursor, emit_time);
    cursor = put_u32(cursor, samples == 0U ? 0U : min_interval);
    cursor = put_u32(cursor, max_interval);
    cursor = put_u32(cursor, sum_low);
    cursor = put_u32(cursor, sum_high);
    cursor = put_u32(cursor, samples);
    cursor = put_u32(cursor, static_cast<std::uint32_t>(timer::Timer::kTimerFrequencyHz));
    cursor = put_u32(cursor, stored);
    for (std::uint32_t index = 0; index < stored; index++) {
        for (const std::uint32_t word : anomaly_snapshot[index])
            cursor = put_u32(cursor, word);
    }

    // 尽力而为, 与此处其他遥测记录一致: 批次池满即丢弃本条而非重试, 缺口
    // 表现为记录序号跳变。
    (void)link::uplink_serializer().write_uart(
        static_cast<core::protocol::FieldId>(data::DataId::kUart0),
        {
            .uart_data = {record, record_size},
              .idle_delimited = true
    });
}

} // namespace libhcs::firmware::sync::sof_probe

#endif
