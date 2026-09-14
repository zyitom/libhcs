#pragma once

// USB Start-of-Frame 钩子: 一个入口, 两个消费者。
//
// 共享时间基准所依赖的一切都从这里进入。设备控制器每 125 us microframe 置
// 一次 SRI, 并根据主机的 SOF 包被动更新 FRINDEX, 故本处理函数 -- 且仅它 --
// 是板子得知当前 microframe 的地方。两个消费者要求寄存器读取发生在同一
// 瞬间, 且该读取必须是中断向量里的第一件事, 因此共用一个钩子而非各自
// 安装:
//
//   * sync::timebase  -- 生产用时间基准(计数器、拟合、锚定)。
//   * sync::sof_probe -- 验证仪器(delta 直方图、端口状态), 见
//                        SOF_TIMEBASE.md 的说明。
//
// 两者可任选、同开或全关; 全关时它成为空操作, 编译器将其从 USB 向量中
// 整个移除。
//
// SRI 状态位在此消费, TinyUSB 的设备 ISR 永远看不到 SOF, 也不会排队
// DCD_EVENT_SOF。因此开启时间基准的板子对 class 驱动呈现的 USB 行为与
// 未开启的完全一致。

namespace libhcs::firmware::sync {

// USB0 向量的第一条语句, 位于 dcd_int_handler() 之前。
void sof_isr_entry();

// 使能 SOF 中断。必须在 tud_init() 之后运行: 其 dcd_init() 会整体覆写
// 整个 USBINTR。
void sof_init();

// 重新使能 SOF。开销小, 可挂在主循环的周期性工作里调用; 控制器被重新
// 初始化之后, 钩子也靠它存活。
void sof_rearm();

} // namespace libhcs::firmware::sync
