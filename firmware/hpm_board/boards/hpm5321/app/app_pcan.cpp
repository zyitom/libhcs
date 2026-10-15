#include "firmware/hpm_board/boards/hpm5321/app/app.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>

#include <board.h>
#include <device/usbd.h>
#include <hpm_dma_mgr.h>
#include <hpm_l1c_drv.h>

#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/led/led.hpp"
#include "firmware/hpm_board/app/src/pcan/pcan_adapter.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"
#include "firmware/hpm_board/app/src/utility/boot_mailbox.hpp"
#include "firmware/common/app/src/utility/interrupt_lock.hpp"
#include "firmware/hpm_board/app/src/watchdog/watchdog.hpp"

int main() { libhcs::firmware::app.init().run(); }

namespace libhcs::firmware {

// PCAN 人格的 App: 与 app.cpp 同一副骨架, 差别只在 -- 没有 libhcs 会话 / DMTool /
// UART / 跨板脉冲 / SOF 时间基(本镜像没有那些模块), 周期工作换成 pcan 的排期。
//
// 上电即 PCAN 卡, 但所有通道停在 reset 模式(真卡语义): 枚举后主机不发
// NORMAL_MODE, 控制器不上总线 -- 上电默认配置(1M/5M)不会拿去打一条速率未知的
// 真总线。

App::App() {
    {
        const utility::InterruptLockGuard guard;

        board_init();
        board_init_usb();
        dma_mgr_init();

        // 开启 D-cache write-around: 流式写绕过 cache 分配(与默认镜像同一取舍)。
        l1c_dc_enable_writearound();

        boot::BootMailbox::clear();

        // 硬件看门狗(libhcs_WATCHDOG, 默认 OFF)。
        watchdog::watchdog.init();

        led::led.init();
        timer::timer.init();
    }

    {
        const utility::InterruptLockGuard guard;

        // 先于 CAN 初始化: CAN ISR 会经 pcan 的闸与队列串行进协议栈, 实例必须
        // 先存在(与默认镜像对 usb::vendor 的次序约束同理)。
        pcan::usb_init();

        // 以 can_count() 为界: 表按双 CAN 板定尺寸, 单 CAN 板不构造最后一个槽位
        // (构造它等于给 MCAN3 上时钟并抢走 LED 的焊盘)。
        for (std::size_t i = 0; i < can::can_count(); ++i) {
            can::can_array[i].init();
            // PCAN 语义: 通道停在 reset 模式等主机命令。
            can::can_array[i]->set_bus_off();
        }
    }
}

// 保持非静态以确保实例化
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    uint32_t last_tick = 0;
    while (true) {
        // USB 协议栈: 事件队列里有东西才进(与 app.cpp 同一取舍)。
        if (tud_task_event_ready())
            tud_task();

        // 紧跟 tud_task() 排空 CAN 软件发送队列, TinyUSB 在那里递交本轮的下行帧。
        can::Can::drain_pending_transmits();

        const uint32_t tick = timer::timer->tick_count();
        if (tick != last_tick) {
            last_tick = tick;
            // 常绿 = PCAN 的"驱动已加载且至少一路总线上线"; 枚举后无 DRVLD 时停在
            // 等待闪烁。
            led::led->set_host_connected(pcan::session_established());
            led::led->update(tick);

            // PCAN 的周期排期(CALIBRATION / ERROR / STATUS)与 DFU 重启的延迟。
            pcan::poll_tick(tick);
            pcan::poll_dfu_reboot();
        }

        // CAN 中断投递看门狗: 只查在总线上的控制器(set_bus_off 清掉了它的位,
        // reset 模式的通道不占主循环一次调用)。
        for (uint32_t running = can::Can::running_mask(); running != 0; running &= running - 1U)
            can::can_array[static_cast<size_t>(std::countr_zero(running))]->poll();

        // PCAN 记录流泵(内联闸: 无事时一次加载加一次分支)。
        pcan::poll();

        watchdog::watchdog->feed();
    }
}

} // namespace libhcs::firmware
