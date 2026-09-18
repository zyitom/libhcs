#include "firmware/hpm_board/boards/hpm6e8y/app/app.hpp"

#include <cstddef>
#include <cstdint>

#include <board.h>
#include <device/usbd.h>
#include <hpm_dma_mgr.h>
#include <hpm_l1c_drv.h>

#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/diag/can_diag.hpp"
#include "firmware/hpm_board/app/src/led/led.hpp"
#include "firmware/hpm_board/app/src/sync/pulse.hpp"
#include "firmware/hpm_board/app/src/sync/sof.hpp"
#include "firmware/hpm_board/app/src/sync/sof_probe.hpp"
#include "firmware/hpm_board/app/src/sync/timebase.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"
#include "firmware/hpm_board/app/src/uart/uart.hpp"
#include "firmware/hpm_board/app/src/usb/vendor.hpp"
#include "firmware/hpm_board/app/src/utility/boot_mailbox.hpp"
#include "firmware/hpm_board/app/src/utility/interrupt_lock.hpp"

int main() { libhcs::firmware::app.init().run(); }

namespace libhcs::firmware {

App::App() {
    {
        const utility::InterruptLockGuard guard;

        board_init();
        board_init_usb();
        dma_mgr_init();

        // 使能 D-cache write-around: 流式写入不占 cache 行, 把 16 KiB
        // D-cache 留给热点控制结构。
        l1c_dc_enable_writearound();

        boot::BootMailbox::clear();

        led::led.init();
        timer::timer.init();
    }

    {
        const utility::InterruptLockGuard guard;

        // 必须在下方 CAN/UART 的 init() 之前: 那些 init 挂接的驱动 ISR 直接
        // 串行进入协议栈, 协议栈实例得先存在。
        //
        usb::vendor.init();

        // 必须在 usb::vendor.init() 之后, 不能更早: tud_init() -> dcd_init()
        // 会整体重赋 USBINTR, 更早挂上的 SOF 使能会被覆盖。未编译时间基或
        // SOF 探针时为 no-op。
        sync::sof_init();

        // 上界是 can_count() 而非数组容量: hpm5321 镜像的表按双 CAN PCB 分配
        // 尺寸, 单 CAN PCB 则留下末位槽不构造。在那里 init 会给 MCAN3 上时钟,
        // 并把 PA30/PA31 从 LED 手里抢走。未初始化的 Lazy 保持惰性。
        for (size_t i = 0; i < can::can_count(); ++i)
            can::can_array[i].init();

        for (auto& board_uart : uart::uart_array)
            board_uart.init();

        // 必须在 CAN 驱动之后(正是它们把 PTPC0 打开): 这里把 USB
        // Start-of-Frame 接入 PTPC 的硬件捕获。未编译时间基时为 no-op。

        // 刻意放在 UART 驱动之后: 这里会改写 UART0 的引脚复用, 把 GPTMR0 的
        // 比较输出与捕获输入放上那些焊盘。启用 pulse 测试的构建里 UART0 不可用。
        sync::pulse::init();
    }
}

namespace {

// LED 依据: 常绿必须意味着"帧正在被转发", 故跟随 CAN/UART 驱动串行接入的
// 那个会话。
bool host_session_established() { return usb::vendor->session_established(); }

} // namespace

// 刻意保持非静态以确保实例化
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    uint32_t last_tick = 0;
    while (true) {
        diag::note_main_loop();
        tud_task();

        // 紧随 tud_task() 排空 CAN 软件发送队列: 本轮的下行帧正是在这里由
        // TinyUSB 交付。若推迟到 1 kHz LED/遥测块之后, 那块的工作会插在帧
        // 到达与帧上线之间。
        for (size_t i = 0; i < can::can_count(); ++i)
            can::can_array[i]->try_transmit();

        usb::poll_dfu_runtime_reboot();

        // LED 记账放在这里按 1 kHz tick 节奏跑, 而不在 mchtmr ISR 里: MTIP
        // 绕过 PLIC 优先级阈值, ISR 侧做这件事会连优先级 3 的 CAN ISR 都抢先,
        // 加重转发热路径。LED 状态反映会话握手(nonce + keepalive 租约)而非
        // 单纯的 USB 枚举: 常绿表示数据确实在转发; 已枚举但无活跃会话的主机
        // 停留在"等待"闪烁。
        const uint32_t tick = timer::timer->tick_count();
        if (tick != last_tick) {
            last_tick = tick;
            led::led->set_host_connected(host_session_established());
            led::led->update(tick);

            // 共享时基: 重拟合微帧到本地定时器的直线, 并重新挂上 SOF 使能,
            // 使钩子在控制器于背后被重建后仍存活。未编译时均为 no-op。
            sync::timebase::poll(tick);
            sync::pulse::poll(tick);
            sync::sof_rearm();

            usb::vendor->poll_pulse_captures();

            // USB SOF / FRINDEX 校验遥测(仅 libhcs_APP_SOF_DIAG 构建), 与下方
            // CAN 遥测同用 1 kHz tick。
            sync::sof_probe::poll(tick);

            // CAN 转发遥测(仅 libhcs_APP_CAN_DIAG 构建)。同一 1 kHz tick 节奏,
            // 且在下方传输泵之前发出, 使本 tick 产生的记录当轮就离开。
            diag::poll(tick);
        }

        // CAN 中断送达看门狗。必须每轮都跑, 不能挂在 1 kHz tick 上: RX FIFO0
        // 装得下 32 个元素, 按本板的转发速率算, 距帧丢失的余量不足两毫秒。
        for (size_t i = 0; i < can::can_count(); ++i)
            can::can_array[i]->poll();

        // 主机传输泵。
        usb::vendor->try_transmit();

        for (auto& board_uart : uart::uart_array)
            board_uart->try_transmit();
    }
}

} // namespace libhcs::firmware
