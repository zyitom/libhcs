#include "firmware/hpm_board/app/src/diag/can_diag.hpp"

#include "firmware/hpm_board/app/src/uart/uart.hpp"

#if defined(libhcs_APP_CAN_DIAG) && libhcs_APP_CAN_DIAG

# include <atomic>
# include <cstring>

# include <hpm_clock_drv.h>
# include <hpm_common.h>
# include <hpm_csr_drv.h>
# include <hpm_mcan_drv.h>
# include <hpm_mcan_regs.h>
# include <hpm_plic_drv.h>
# include <hpm_soc.h>

# include "board_app.hpp"
# include "core/include/libhcs/data/datas.hpp"
# include "core/src/protocol/protocol.hpp"
# include "core/src/protocol/serializer.hpp"
# include "firmware/hpm_board/app/src/link/uplink.hpp"

namespace libhcs::firmware::diag {
namespace {

// Table capacity: sizes the counter array and the worst-case record buffer.
// board::can_port_count() is how many controllers this board actually has, and
// that is what goes on the wire -- the host parser reads the count out of the
// record (host/examples/can_stall_probe.cpp), so a shorter record on the
// single-CAN hpm5321 decodes without any host change.
constexpr std::size_t kCanCapacity = board::kCanPortCapacity;
constexpr std::uint32_t kEmitPeriodMs = 100U;

// Sources 0..191. The board's highest interrupt number is 161 (IRQn_DEBUG1), and
// the three that matter here are MCAN0..3 = 90..93 (word 2) and USB0 = 127
// (word 3); the whole array is shipped anyway so a later question about some
// other source does not need another firmware flash.
constexpr std::size_t kPlicWordCount = 6;

struct CanCounters {
    std::atomic<std::uint32_t> isr_entries{0};
    std::atomic<std::uint32_t> frames{0};
    std::atomic<std::uint32_t> tx_fail{0};
    std::atomic<std::uint32_t> irq_recovered{0};
};

CanCounters counters[kCanCapacity];
std::atomic<std::uint32_t> alloc_fail{0};

// USB bulk OUT endpoint timing. Plain (non-atomic) globals on purpose: both
// notify functions run in tud_task context on the main loop, and poll() reads
// them from the same loop, so there is no cross-context hazard here -- unlike
// the CAN counters above, which the ISR writes.
//
// Sums are reset every emit period. At the measured ~58k packets/s and 100 ms
// per record that is ~5800 samples of a few thousand cycles each, three orders
// of magnitude below a uint32 wrap.
std::uint32_t usb_out_turnaround_cycles = 0;
std::uint32_t usb_out_starve_cycles = 0;
std::uint32_t usb_out_samples = 0;
std::uint32_t usb_out_complete_cycle = 0;
std::uint32_t usb_out_armed_cycle = 0;
bool usb_out_armed_valid = false;
std::uint32_t main_loop_count = 0;
std::uint32_t last_main_loop_count = 0;
std::uint32_t last_emit_tick = 0;
std::uint8_t record_sequence = 0;

std::byte* put_u32(std::byte* cursor, std::uint32_t value) {
    std::memcpy(cursor, &value, sizeof(value));
    return cursor + sizeof(value);
}

// The claim register is deliberately never read: reading it performs a claim and
// would itself steal an interrupt. Pending and enable are plain reads.
std::uint32_t plic_pending_word(std::size_t word) {
    const auto* const base =
        reinterpret_cast<volatile std::uint32_t*>(HPM_PLIC_BASE + HPM_PLIC_PENDING_OFFSET);
    return base[word];
}

// Per-source trigger type: 1 = edge, 0 = level. Which one the MCAN sources use
// decides whether an interrupt asserted while the gateway is in service can be
// lost outright, so it is worth having in the record rather than assumed.
std::uint32_t plic_trigger_word(std::size_t word) {
    const auto* const base =
        reinterpret_cast<volatile std::uint32_t*>(HPM_PLIC_BASE + HPM_PLIC_TRIGGER_TYPE_OFFSET);
    return base[word];
}

std::uint32_t plic_enable_word(std::size_t word) {
    const auto* const base = reinterpret_cast<volatile std::uint32_t*>(
        HPM_PLIC_BASE + HPM_PLIC_ENABLE_OFFSET
        + (static_cast<std::uint32_t>(HPM_PLIC_TARGET_M_MODE) << HPM_PLIC_ENABLE_SHIFT_PER_TARGET));
    return base[word];
}

} // namespace

void note_isr_entry(std::size_t can_index) {
    counters[can_index].isr_entries.fetch_add(1, std::memory_order::relaxed);
}

void note_frame(std::size_t can_index) {
    counters[can_index].frames.fetch_add(1, std::memory_order::relaxed);
}

void note_tx_fail(std::size_t can_index) {
    counters[can_index].tx_fail.fetch_add(1, std::memory_order::relaxed);
}

void note_alloc_fail() { alloc_fail.fetch_add(1, std::memory_order::relaxed); }

// Main loop only, so a plain increment is enough -- no other context touches it.
void note_main_loop() { main_loop_count++; }

void note_irq_recovered(std::size_t can_index) {
    counters[can_index].irq_recovered.fetch_add(1, std::memory_order::relaxed);
}

void note_usb_out_complete() {
    const auto now = static_cast<std::uint32_t>(hpm_csr_get_core_mcycle());
    // The first completion after a re-arm closes a full cycle; before the very
    // first arm there is nothing to attribute the elapsed time to.
    if (usb_out_armed_valid) {
        usb_out_starve_cycles += now - usb_out_armed_cycle;
        usb_out_samples++;
    }
    usb_out_complete_cycle = now;
}

void note_usb_out_armed() {
    const auto now = static_cast<std::uint32_t>(hpm_csr_get_core_mcycle());
    usb_out_turnaround_cycles += now - usb_out_complete_cycle;
    usb_out_armed_cycle = now;
    usb_out_armed_valid = true;
}

void poll(std::uint32_t tick) {
    if (tick - last_emit_tick < kEmitPeriodMs)
        return;
    last_emit_tick = tick;

    // Nothing to send this on yet; the counters keep running and the first
    // record after the session comes up carries the accumulated totals.
    if (!link::uplink_enabled())
        return;

    // 8-byte fixed header + 4 + 2 * kPlicWordCount words + threshold + 8 words
    // per CAN controller.
    constexpr std::size_t kFixedSize = 8 + 4 + 4 * (3 * kPlicWordCount) + 4 + 4 + 4;
    constexpr std::size_t kPerCanSize = 11 * 4;
    // Three extra words: UART0 kernel clock, OSCR, and the DLM:DLL divisor.
    constexpr std::size_t kUartSize = 3 * 4;
    // USB bulk OUT timing, appended LAST so the host can find it from the end of
    // the record without reconstructing the variable-length CAN section: cycle
    // sums for turnaround and starve, the sample count they cover, and the core
    // clock needed to turn cycles into microseconds.
    constexpr std::size_t kUsbSize = 4 * 4;
    // Buffer sized for the capacity; the record actually emitted covers only the
    // controllers this board has, and carries that count in its header.
    constexpr std::size_t kMaxRecordSize =
        kFixedSize + kCanCapacity * kPerCanSize + kUartSize + kUsbSize;

    const std::size_t can_count = board::can_port_count();
    const std::size_t record_size = kFixedSize + can_count * kPerCanSize + kUartSize + kUsbSize;

    std::byte record[kMaxRecordSize];
    std::byte* cursor = record;

    *cursor++ = static_cast<std::byte>(kRecordMagic);
    *cursor++ = static_cast<std::byte>(kRecordVersion);
    *cursor++ = static_cast<std::byte>(record_sequence++);
    *cursor++ = static_cast<std::byte>(can_count);
    cursor = put_u32(cursor, static_cast<std::uint32_t>(record_size));
    cursor = put_u32(cursor, tick);
    cursor = put_u32(cursor, alloc_fail.load(std::memory_order::relaxed));
    cursor = put_u32(cursor, main_loop_count - last_main_loop_count);
    last_main_loop_count = main_loop_count;

    for (std::size_t word = 0; word < kPlicWordCount; word++)
        cursor = put_u32(cursor, plic_pending_word(word));
    for (std::size_t word = 0; word < kPlicWordCount; word++)
        cursor = put_u32(cursor, plic_enable_word(word));
    for (std::size_t word = 0; word < kPlicWordCount; word++)
        cursor = put_u32(cursor, plic_trigger_word(word));
    cursor = put_u32(cursor, __plic_get_threshold(HPM_PLIC_BASE, HPM_PLIC_TARGET_M_MODE));

    for (std::size_t index = 0; index < can_count; index++) {
        auto* const can = reinterpret_cast<MCAN_Type*>(board::kCanPorts[index].base);
        cursor = put_u32(cursor, counters[index].isr_entries.load(std::memory_order::relaxed));
        cursor = put_u32(cursor, counters[index].frames.load(std::memory_order::relaxed));
        cursor = put_u32(cursor, counters[index].tx_fail.load(std::memory_order::relaxed));
        cursor = put_u32(cursor, counters[index].irq_recovered.load(std::memory_order::relaxed));
        cursor = put_u32(cursor, can->IR);
        cursor = put_u32(cursor, can->RXF0S);
        cursor = put_u32(cursor, can->PSR);
        cursor = put_u32(cursor, can->ECR);
        cursor = put_u32(cursor, can->TXFQS);
        // Actual bit-timing registers, so the host can see what the SDK's
        // baudrate solver really programmed rather than what was requested.
        cursor = put_u32(cursor, can->NBTP);
        cursor = put_u32(cursor, can->DBTP);
    }

    // UART0 baudrate evidence: the kernel clock the driver was handed, plus the
    // divisor and oversample rate actually programmed. A baudrate that looks
    // right in source but wrong on the wire shows up here.
    //
    // The divisor comes from Uart's snapshot, NOT from reading DLM/DLL here.
    // Reading them requires setting LCR.DLAB, and DLAB makes offset 0x20 mean
    // DLL instead of THR -- which is where the TX DMA writes. This sampler runs
    // on the main loop while that DMA is live, so the earlier read-back version
    // of this block let any byte transferred inside the window overwrite the
    // divisor latch. That destroyed the baudrate it was trying to measure, and
    // reported the pre-clobber value while doing it: the record showed a correct
    // divisor for a port that had gone silent. OSCR is a plain register, unaffected
    // by DLAB, so it is still read directly.
    {
        cursor = put_u32(cursor, uart::uart_array[0]->clock_hz());
        cursor = put_u32(cursor, uart::uart_array[0]->oscr());
        cursor = put_u32(cursor, uart::uart_array[0]->divisor());
    }

    // USB bulk OUT endpoint timing, and the reset of its accumulators. Emitting
    // the raw sums plus the sample count rather than an average keeps the
    // division on the host, where a period with zero packets is visibly zero
    // samples instead of a divide by zero.
    cursor = put_u32(cursor, usb_out_turnaround_cycles);
    cursor = put_u32(cursor, usb_out_starve_cycles);
    cursor = put_u32(cursor, usb_out_samples);
    cursor = put_u32(cursor, clock_get_frequency(clock_cpu0));
    usb_out_turnaround_cycles = 0;
    usb_out_starve_cycles = 0;
    usb_out_samples = 0;

    // Best effort by design: if the batch pool is full the record is dropped
    // rather than retried, exactly like a forwarded CAN frame. A gap in the
    // record sequence numbers is itself a symptom worth seeing on the host.
    (void)link::uplink_serializer().write_uart(
        static_cast<core::protocol::FieldId>(data::DataId::kUart0),
        {
            .uart_data = {record, record_size},
              .idle_delimited = true
    });
}

} // namespace libhcs::firmware::diag

#endif
