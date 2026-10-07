#pragma once

// USB Start-of-Frame 钩子: 一个入口, 两个消费者; 以及共享时间基准的开关。
//
// 共享时间基准所依赖的一切都从这里进入。设备控制器每 125 us microframe 置
// 一次 SRI, 并根据主机的 SOF 包被动更新 FRINDEX, 故本处理函数 -- 且仅它 --
// 是板子得知当前 microframe 的地方。两个消费者要求寄存器读取发生在同一
// 瞬间, 且该读取必须是中断向量里的第一件事, 因此共用一个钩子而非各自
// 安装:
//
//   * sync::timebase / sync::sof_capture -- 共享时间基准(计数器、拟合、锚定;
//                        CAN 帧的硬件时间戳)。每个镜像都带, 主机在清单里要了才开
//                        (time_sync_start(), 由 EP0 kApplyManifest 调用)。
//   * sync::sof_probe -- 验证仪器(delta 直方图、端口状态), 仅 libhcs_APP_SOF_DIAG
//                        构建, 见 SOF_TIMEBASE.md 的说明。
//
// 时间基准关着时 SOF 中断不开(USBINTR.SRE = 0), 本钩子在每个 USB 中断里只多一次
// 内存读和一次分支; CAN 接收中断同样只多一次判断, 帧不带时间戳。
//
// SRI 状态位在此消费, TinyUSB 的设备 ISR 永远看不到 SOF, 也不会排队
// DCD_EVENT_SOF。因此开启时间基准的板子对 class 驱动呈现的 USB 行为与
// 未开启的完全一致。

#include <atomic>

#include "core/src/link/ownership.hpp"

namespace libhcs::firmware::sync {

namespace internal {
// 只由主循环(EP0 清单、归属交还)写; ISR 读。
inline constinit std::atomic<bool> g_time_sync_on{false};
} // namespace internal

// 时间基准在运行: SOF 入环、CAN 帧打时间戳、应答 kTimeAnchor。任何上下文可调。
//
// 同核的中断与主循环之间只需挡住编译器重排(signal fence), 不需要硬件 fence: 读到
// true 时, time_sync_start() 在置位之前做的复位与配置对本中断都已可见。
[[nodiscard]] inline bool time_sync_on() noexcept {
    const bool on = internal::g_time_sync_on.load(std::memory_order_relaxed);
    std::atomic_signal_fence(std::memory_order_acquire);
    return on;
}

// 开时间基准: 状态清零、接通 SOF -> PTPC 捕获、开 SOF 中断。幂等。主循环调用(EP0
// kApplyManifest, 端口应用之前); 不在上电路径里接 PTPC 捕获, 理由见 sof_capture::start()。
[[nodiscard]] bool time_sync_start();

// 关时间基准: 关 SOF 中断, 之后的 CAN 帧不带时间戳, kTimeAnchor 不再应答。幂等。
void time_sync_stop();

// 归属交接的一步(core/src/link/ownership.hpp): 板子交还附加功能时关时间基准 --
// 会话结束、握手后没开出会话、主机断开、清单回滚, 都走这条路。接手时什么也不做:
// 开不开由清单决定。
struct TimeSyncHandoff {
    static void to_libhcs() {}
    static void to_tools() { time_sync_stop(); }
};
static_assert(core::link::HandoffStep<TimeSyncHandoff>);

// USB0 向量的第一条语句, 位于 dcd_int_handler() 之前。
void sof_isr_entry();

// SOF 探针构建在上电时就开 SOF 中断; 别的构建什么也不做(时间基准由清单打开)。必须在
// tud_init() 之后运行: 其 dcd_init() 会整体覆写 USBINTR。
void sof_init();

// 需要 SOF 中断时重新使能它。开销小, 挂在主循环的周期性工作里; 控制器被重新初始化
// 之后, 钩子也靠它存活。
void sof_rearm();

} // namespace libhcs::firmware::sync
