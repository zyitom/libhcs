#include "firmware/hpm_board/boards/hpm5321/app/app.hpp"

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

        // 开启 D-cache write-around: 流式写绕过 cache 分配, 把 16 KiB D-cache
        // 留给热点控制结构。
        l1c_dc_enable_writearound();

        boot::BootMailbox::clear();

        led::led.init();
        timer::timer.init();
    }

    {
        const utility::InterruptLockGuard guard;

        // 必须在下面 CAN 与 UART 的 init() 之前: 它们使能的驱动 ISR 会直接
        // 串行进入协议栈, 协议栈实例必须先存在。
        //
        usb::vendor.init();

        // 必须在 usb::vendor.init() 之后, 绝不能更早: tud_init() -> dcd_init()
        // 会整体重赋 USBINTR, 更早使能的 SOF 会被覆盖。时间基或 SOF 探针未
        // 编译进来时为空操作。
        sync::sof_init();

        // 以 can_count() 而非数组大小为界: hpm5321 镜像的表按双 CAN 板定尺寸,
        // 单 CAN 板不构造最后一个槽位。若在那里初始化, 会给 MCAN3 上时钟并抢走
        // LED 的 PA30/PA31。未构造的 Lazy 保持惰性。
        for (size_t i = 0; i < can::can_count(); ++i)
            can::can_array[i].init();

        for (auto& board_uart : uart::uart_array)
            board_uart.init();

        // 刻意放在 UART 驱动之后: 这里覆写 UART0 的引脚复用, 把 GPTMR0 的比较
        // 输出与捕获输入放上这些焊盘。启用 pulse 测试的构建中 UART0 不可用。
        sync::pulse::init();
    }
}

namespace {

// LED 依据: 常绿必须表示"帧正在被转发", 故跟随 CAN/UART 驱动所串行进入的
// 会话。
bool host_session_established() {
    return usb::vendor->session_established() || dmtool::session_established();
}

} // namespace

// 保持非静态以确保实例化
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    uint32_t last_tick = 0;
    while (true) {
        diag::note_main_loop();
        tud_task();

        // 紧跟 tud_task() 排空 CAN 软件发送队列, TinyUSB 在那里递交本轮的
        // 下行帧。若放到 1 kHz LED/遥测块之后, 帧到达与上线之间会插入该块的
        // 工作。无控制器有排队帧时仅一次加载加一次分支, 几乎每轮如此; 见
        // Can::drain_pending_transmits()。
        can::Can::drain_pending_transmits();

        usb::poll_dfu_runtime_reboot();

        // LED 簿记放在这里按 1 kHz tick 节奏跑, 而非 mchtmr ISR 内: MTIP 绕过
        // PLIC 优先级阈值, ISR 内的工作会连优先级 3 的 CAN ISR 也抢占, 拖累
        // 转发热路径。LED 状态反映会话握手(nonce + keepalive 租约), 而非仅仅
        // USB 枚举: 常绿表示数据确在转发; 已枚举但无活跃会话的主机停留在
        // "等待"闪烁。
        const uint32_t tick = timer::timer->tick_count();
        if (tick != last_tick) {
            last_tick = tick;
            led::led->set_host_connected(host_session_established());
            led::led->update(tick);

            // 共享时间基: 重拟合微帧到本地定时器的映射, 并重新使能 SOF, 使
            // 钩子在控制器于身后被重新初始化后仍能存活。未编译进来时均为
            // 空操作。
            sync::timebase::poll(tick);
            sync::pulse::poll(tick);
            sync::sof_rearm();

            usb::vendor->poll_pulse_captures();

            // USB SOF / FRINDEX 校验遥测(仅 libhcs_APP_SOF_DIAG 构建), 与
            // 下面 CAN 遥测共用 1 kHz tick。
            sync::sof_probe::poll(tick);

            // CAN 转发遥测(仅 libhcs_APP_CAN_DIAG 构建)。按同一 1 kHz tick
            // 节流, 且在下面的传输泵之前发出, 使本 tick 产生的记录在本轮离板。
            diag::poll(tick);
        }

        // CAN 中断投递看门狗。必须每轮运行, 不能按 1 kHz tick: RX FIFO0 容
        // 32 个元素, 按本板转发速率算, 距丢帧的余量不到两毫秒。
        for (size_t i = 0; i < can::can_count(); ++i)
            can::can_array[i]->poll();

        // 主机传输泵。
        usb::vendor->try_transmit();

        for (auto& board_uart : uart::uart_array)
            board_uart->try_transmit();
    }
}

} // namespace libhcs::firmware
