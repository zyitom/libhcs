#include "firmware/hcs_board/app/src/xcore/secondary_core.hpp"

// EtherCAT 归档后的残留。单核构建从头文件取得内联 no-op, 本翻译单元对它们
// 为空(SDK 两种方式都会把它编入构建)。
#if defined(libhcs_APP_RELEASE_CORE1) && libhcs_APP_RELEASE_CORE1

# include <cstddef>
# include <cstdint>

# include <board.h>
# include <hpm_clock_drv.h>
# include <hpm_debug_console.h>
# include <hpm_mbx_drv.h>
# include <hpm_soc.h>

extern "C" {
/* SDK 示例助手自带的头没有 extern "C" 保护, 且依赖 uint8_t/uint32_t 已声明,
 * 故必须跟在上面的头之后、置于本块内。 */
# include <multicore_common.h>
}

# include "core/include/libhcs/data/datas.hpp"
# include "core/src/protocol/protocol.hpp"
# include "core/src/protocol/serializer.hpp"
# include "firmware/hcs_board/app/src/link/uplink.hpp"
# include "firmware/hcs_board/app/src/timer/timer.hpp"
# include "firmware/hcs_board/ecat/common/xcore_channel.hpp"
# include "firmware/hcs_board/ecat/common/xcore_diag.hpp"

namespace libhcs::firmware::xcore {
namespace {

// 仅 core0 本地: publish_channel() 在 core1 存在前一次性赋值, 之后只读,
// 无需自身同步 -- 跨核顺序由通道内容承载。
ecat::XcoreChannel* g_channel = nullptr;

// 每轮主循环搬运字节数的上界; 真正的防护是下方的非阻塞 FIFO 检查, 此值只是
// 给循环封顶。
constexpr std::size_t kDiagBytesPerPass = 32;

} // namespace

void publish_channel() {
    // board_init_pmp()(经 board_init())已把 SHARE_RAM 映射为非 cache + AMO,
    // 在此 placement 构造通道是该区域的首次访问。
    g_channel = &ecat::xcore_channel_init();

    // 在这里(而非 core1 上)打开共享的 MBX0 时钟门: core1 释放后可能立即写
    // MBX0B, 写被门控的 mailbox 会静默丢失。只复位 mailbox, 不使能中断 --
    // 本迁移步骤里 core1 没有诊断环之外要告诉 core0 的事。
    clock_add_to_group(clock_mbx0, 0);
    mbx_init(HPM_MBX0A);
}

ecat::XcoreChannel* channel() { return g_channel; }

void ring_uplink_doorbell() {
    // 先发布环内字节, 再敲门铃。XcoreRing::try_push 以 release store 结束,
    // 它把载荷排在索引之前, 但在 RISC-V 上并不能把该非 cache 写排到随后的
    // 设备寄存器写之前; 保证 core1 看到邮箱字时已看到推入字节的是全量 fence
    // (内存 + I/O, 双向)。HPM 多核示例漏了它, 省掉会偶发"门铃到了载荷没到"
    // 的难查故障。
    __asm__ volatile("fence" ::: "memory");
    // 发送失败说明单字邮箱里已有未决的敲门; 忽略即可 -- 一次中断足够,
    // 处理器会重读环形队列。
    (void)mbx_send_message(HPM_MBX0A, 0);
}

void release_core1() {
    // 经 SDP DMA 把内嵌镜像拷入 core1 的 ILM, 从 core0 的 D-cache 冲刷源区间,
    // 然后启动该核。幂等: SDK 助手先查 sysctl_is_cpu_released()。
    multicore_release_cpu(HPM_CORE1, SEC_CORE_IMG_START);
}

bool wait_for_core1_eeprom(std::uint32_t timeout_ms) {
    if (g_channel == nullptr)
        return true;

    // 忙等而非休眠: 只在启动期跑一次, 此时 USB 与 CAN/UART 驱动尚不存在,
    // 唯一必须推进的是 flash RPC, 它由 MBX1A 中断服务因而能抢占本循环。顺路
    // 排空诊断, core1 的启动日志才不会堵在等待之后。
    const std::uint64_t deadline =
        timer::Timer::timestamp64_quarter_us()
        + static_cast<std::uint64_t>(timeout_ms) * (timer::Timer::kTimerFrequencyHz / 1000U);

    while (g_channel->flash.eeprom_ready.load(std::memory_order::acquire) == 0) {
        if (timer::Timer::timestamp64_quarter_us() >= deadline)
            return false;
        poll_diagnostics();
    }
    return true;
}

# if defined(libhcs_APP_DIAG_OVER_USB) && libhcs_APP_DIAG_OVER_USB

// 把 core1 的诊断文本以 UART0 上行帧中继给主机。core1 唯一的日志通路是
// SHARE_RAM 环, 唯一的出口是板载控制台 -- FT2232 调试头上的 UART, 而只接
// EtherCAT 与 USB 的板上那里什么都不接, core1 的启动日志因此完全不可达。
// 按需启用: DataId::kUart0 是本板的真实数据口(UART1), 使用它的主机不能在
// 字节流里混进日志。
void relay_diagnostics_over_usb() {
    // 在排空之前检查。core1 日志最值得读的部分写于启动期, 比主机能建立会话早
    // 数秒; 先排空再因无载波丢弃, 扔掉的恰好是值得读的行。留在环里, 主机到来
    // 时仍在; 若主机永远不来, 环会填满, core1 的写入在生产者侧被丢 -- 这是
    // 它文档化的约定。
    if (!link::uplink_enabled())
        return;

    // 每轮至多一帧, 有界, 防止话痨的 core1 垄断主循环或批缓冲。文本原样
    // 转发, 由主机打印。
    char text[120];
    const std::size_t size = ecat::xcore_diag_drain(g_channel->diag, text, sizeof(text));
    if (size == 0)
        return;

    (void)link::uplink_serializer().write_uart(
        static_cast<core::protocol::FieldId>(data::DataId::kUart0),
        {
            .uart_data = {reinterpret_cast<const std::byte*>(text), size},
              .idle_delimited = true
    });
}

# endif

void poll_diagnostics() {
# if defined(libhcs_APP_DIAG_OVER_USB) && libhcs_APP_DIAG_OVER_USB
    relay_diagnostics_over_usb();
    return;
# else
    // 不为日志字节阻塞主循环。console_send_byte() 在 THR 空时忙等(115200 波特
    // 下每字节约 87 us), core1 启动消息的一阵突发会把 tud_task() 与 CAN/UART
    // 发送泵卡住毫秒级。因此先窥探 TX FIFO, 满即停; 字节留在环里, 下一轮
    // 继续。诊断绝不能换来数据通路延迟。
    for (std::size_t i = 0; i < kDiagBytesPerPass; ++i) {
        char byte = 0;
        if (ecat::xcore_diag_drain(g_channel->diag, &byte, 1) == 0)
            return;
        if (!board_console_try_send_byte(static_cast<std::uint8_t>(byte))) {
            // FIFO 满: 该字节已出环, 用阻塞调用发掉而非丢弃, 然后结束本轮。
            console_send_byte(static_cast<std::uint8_t>(byte));
            return;
        }
    }
# endif
}

} // namespace libhcs::firmware::xcore

#endif
