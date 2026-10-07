#pragma once

// mc02 的 DWC2 控制器上的 USB Start-of-Frame 钩子, 以及共享时间基准的开关。
//
// 角色与 hpm_board 的 sync/sof.hpp 相同 -- 板上得知自己所处微帧的唯一入口 --
// 但底层 USB IP 不同, 整条寄存器路径都不同: 状态位 GINTSTS.SOF、使能位
// GINTMSK.SOFM、帧计数器 DSTS.FNSOF(全速下数帧)、速度位 DSTS.ENUMSPD。其中
// 帧计数器的差异影响最大: DWC2 的 FNSOF 只有高速下才是微帧号; 全速 -- mc02
// 无 ULPI PHY, 也只能全速 -- 下是帧号, 每中断步进 1, 而 EHCI 的 FRINDEX 步进
// 8。缩放在 sync::timebase 里做, 见其 frame_scale() 的注释。
//
// 如何进入中断向量、以及为何形式特殊: hpm_board 持有自己的 USB 向量, 把钩子作为
// 第一条语句调用。mc02 的向量在 bsp/cubemx/Core/Src/stm32h7xx_it.c, 是 CubeMX
// 产物, 本仓库禁止编辑(见 AGENTS.md 的 CubeMX BSP 纪律一节), 它只是转调 TinyUSB
// 的 dcd_int_handler()。因此钩子在链接期介入: -Wl,--wrap=dcd_int_handler 把向量
// 的调用引到 __wrap_dcd_int_handler(), 先打时间戳再链到真身。生成的代码不被触碰。
//
// 时间基准每个镜像都带, 主机在清单里要了才开(time_sync_start(), 由 EP0
// kApplyManifest 调用)。关着时 GINTMSK.SOFM 不开, 钩子在每个 USB 中断里只多一次
// 内存读和一次分支 -- 加上这一层调用本身, 几个周期, 恒定。
//
// SOF 状态位在这里被消费, dcd_int_handler() 永远看不到它, 也就不会排队
// DCD_EVENT_SOF: 时间基开启的板对类驱动呈现的 USB 行为与关闭时完全相同。

#include <atomic>
#include <cstdint>

namespace libhcs::firmware::sync {

namespace internal {
// 只由主循环(EP0 清单、会话结束)写; ISR 读。放 DTCM: USB 中断、CAN 接收中断与主循环每趟
// 都读它, 而 .data/.bss 所在的 AXI SRAM 前 32 KB 是非缓存的。
[[gnu::section(".dtcm")]] inline constinit std::atomic<bool> g_time_sync_on{false};
} // namespace internal

// 时间基准在运行: SOF 入环、CAN 帧打时间戳、应答 kTimeAnchor。任何上下文可调。
//
// 同核的中断与主循环之间只需挡住编译器重排(signal fence), 不需要硬件屏障: 读到
// true 时, time_sync_start() 在置位之前做的复位与配置对本中断都已可见。
[[nodiscard]] inline bool time_sync_on() noexcept {
    const bool on = internal::g_time_sync_on.load(std::memory_order_relaxed);
    std::atomic_signal_fence(std::memory_order_acquire);
    return on;
}

// 开时间基准: 第一次时接通 TIM5 的 SOF 捕获并标定 TIM3 <-> TIM5(中断关着约 0.5 ms,
// 见 sof_capture::configure()), 每次都把状态清零, 再开 SOF 中断。幂等。主循环调用(EP0
// kApplyManifest, 端口应用之前), 那时 USB 已枚举, 端口速度确定。
[[nodiscard]] bool time_sync_start();

// 关时间基准: 关 SOF 中断, 之后的 CAN 帧不带时间戳, kTimeAnchor 不再应答。幂等。
void time_sync_stop();

// 读取时间戳与帧计数器, 然后应答 SOF。由 __wrap_dcd_int_handler() 在真处理器
// 之前调用。
void sof_isr_entry();

// 时间基准开着时重新使能 SOF。足够便宜, 可放主循环周期任务; 这也是钩子能在控制器
// 于背后被重新初始化后存活的原因 -- dcd_int_handler() 每遇到自己未预期的 SOF 都会
// 自行清掉 SOFM。
void sof_rearm();

} // namespace libhcs::firmware::sync
