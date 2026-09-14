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

// 表容量: 决定计数数组与最坏情况记录缓冲的尺寸。board::can_port_count() 是本板
// 实际拥有的控制器数, 上线的也是它 -- 主机解析器从记录里读出计数
// (host/examples/can_stall_probe.cpp), 单 CAN hpm5321 的较短记录无需改动主机即可
// 解码。
constexpr std::size_t kCanCapacity = board::kCanPortCapacity;
constexpr std::uint32_t kEmitPeriodMs = 100U;

// 覆盖中断源 0..191。本板最高中断号是 161(IRQn_DEBUG1), 这里真正关心的三个是
// MCAN0..3 = 90..93(第 2 字)与 USB0 = 127(第 3 字); 整组数组照发, 以后要查其他
// 中断源就不必再刷固件。
constexpr std::size_t kPlicWordCount = 6;

struct CanCounters {
    std::atomic<std::uint32_t> isr_entries{0};
    std::atomic<std::uint32_t> frames{0};
    std::atomic<std::uint32_t> tx_fail{0};
    std::atomic<std::uint32_t> irq_recovered{0};
};

CanCounters counters[kCanCapacity];
std::atomic<std::uint32_t> alloc_fail{0};

// USB bulk OUT 端点计时。刻意用普通(非原子)全局量: 两个通知函数都在主循环的
// tud_task 上下文运行, poll() 也在同一循环读它们, 这里没有跨上下文风险 -- 与上面
// 由 ISR 写入的 CAN 计数器不同。
//
// 和值每个上报周期清零。按实测约 58k 包/s、每条记录 100 ms 计, 约 5800 个样本、
// 每个几千周期, 距 uint32 回绕还差三个数量级。
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

// 刻意从不读 claim 寄存器: 读它就是一次 claim, 本身会偷走一个中断。pending 与
// enable 是普通读。
std::uint32_t plic_pending_word(std::size_t word) {
    const auto* const base =
        reinterpret_cast<volatile std::uint32_t*>(HPM_PLIC_BASE + HPM_PLIC_PENDING_OFFSET);
    return base[word];
}

// 每个源的触发类型: 1 = 边沿, 0 = 电平。MCAN 源用哪一种, 决定了网关处理期间到来
// 的中断会不会被整体丢失, 值得写进记录而不是靠假设。
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

// 仅主循环调用, 普通自增即可 -- 没有其他上下文碰它。
void note_main_loop() { main_loop_count++; }

void note_irq_recovered(std::size_t can_index) {
    counters[can_index].irq_recovered.fetch_add(1, std::memory_order::relaxed);
}

void note_usb_out_complete() {
    const auto now = static_cast<std::uint32_t>(hpm_csr_get_core_mcycle());
    // 重挂后的第一次 complete 闭合一个完整周期; 首次挂载之前没有可归属的计时
    // 起点。
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

    // 还没有可发送它的通道; 计数器继续累计, 会话建立后的第一条记录带上累计总量。
    if (!link::uplink_enabled())
        return;

    // 记录布局: 8 字节定长头 + tick/alloc_fail/主循环增量, PLIC pending/enable/
    // trigger 各 kPlicWordCount 个字, threshold, 然后每个 CAN 控制器 11 个字
    // (kPerCanSize)。
    constexpr std::size_t kFixedSize = 8 + 4 + 4 * (3 * kPlicWordCount) + 4 + 4 + 4;
    constexpr std::size_t kPerCanSize = 11 * 4;
    // 额外三个字: UART0 内核时钟、OSCR 与 DLM:DLL 除数。
    constexpr std::size_t kUartSize = 3 * 4;
    // USB bulk OUT 计时, 追加在记录最末, 主机从尾部即可定位, 无需重建变长的 CAN
    // 段: turnaround 与 starve 的周期和、覆盖它们的样本数, 以及把周期换算成微秒
    // 所需的内核时钟频率。
    constexpr std::size_t kUsbSize = 4 * 4;
    // 缓冲按容量取尺寸; 实际发出的记录只覆盖本板拥有的控制器, 数量记在头里。
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
        // 真实的位时序寄存器: 主机由此看到 SDK 波特率求解器实际编程的值, 而非
        // 请求的值。
        cursor = put_u32(cursor, can->NBTP);
        cursor = put_u32(cursor, can->DBTP);
    }

    // UART0 波特率证据: 驱动拿到的内核时钟, 加上实际编程的除数与过采样率。
    // 源码里看着对、线上不对的波特率会在这里现形。
    //
    // 除数取自 Uart 的快照, 而不是在这里读 DLM/DLL: 读它们要先置 LCR.DLAB, 而
    // DLAB 会让偏移 0x20 从 THR 变成 DLL -- 恰是发送 DMA 写入的位置。本采样器在
    // 主循环上运行, 与该 DMA 并发; 早先直接读寄存器的版本曾让窗口内传输的任意
    // 字节覆写除数锁存, 摧毁了它本要测量的波特率, 还回报覆盖前的旧值: 记录显示
    // 除数正确, 端口却已哑掉。OSCR 是普通寄存器, 不受 DLAB 影响, 仍直接读。
    {
        cursor = put_u32(cursor, uart::uart_array[0]->clock_hz());
        cursor = put_u32(cursor, uart::uart_array[0]->oscr());
        cursor = put_u32(cursor, uart::uart_array[0]->divisor());
    }

    // USB bulk OUT 端点计时及其累加器清零。上报原始和值加样本数而非平均值, 除法
    // 留给主机: 零包周期呈现为零样本, 而不是除零。
    cursor = put_u32(cursor, usb_out_turnaround_cycles);
    cursor = put_u32(cursor, usb_out_starve_cycles);
    cursor = put_u32(cursor, usb_out_samples);
    cursor = put_u32(cursor, clock_get_frequency(clock_cpu0));
    usb_out_turnaround_cycles = 0;
    usb_out_starve_cycles = 0;
    usb_out_samples = 0;

    // 刻意尽力而为: 批缓冲池满时记录直接丢弃、不重试, 与转发的 CAN 帧同样处理。
    // 记录序号出现空洞本身就是值得主机看到的一个症状。
    (void)link::uplink_serializer().write_uart(
        static_cast<core::protocol::FieldId>(data::DataId::kUart0),
        {
            .uart_data = {record, record_size},
              .idle_delimited = true
    });
}

} // namespace libhcs::firmware::diag

#endif
