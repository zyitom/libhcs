#pragma once

#include <cstdint>

namespace libhcs::firmware::diag::usb_rx_hist {

// 相邻两次 bulk OUT 传输完成间隔的分布, 用板载 DWT 周期计数器测量。
//
// 要回答的问题: 改用 DWC2 控制器内部 DMA 后, 达成速率平均每包多花约 1 us,
// 而 CPU 的工作量严格变小(该切换已成过去, 见 firmware/mc02/AGENTS.md)。
// 平均值与两种机制都相容, 而两者需要的修复相反:
//
//   均匀后移 -- 每一包真的都慢约 1 us: 成本在控制器自身的每次传输开销
//               (DMA 仲裁、endpoint 重新武装)。
//   双峰分布 -- 几乎每包与从前同价, 只有一小部分错过主机的下一个事务窗口,
//               整整等一个 125 us microframe。此时修法是缩短从包到达到
//               endpoint 重新武装的路径, 而不是优化拷贝本身。
//
// 采样点在 tud_vendor_rx_cb, 即主循环里的 usbd 任务上下文而非中断。每个样本
// 因此最多混入一轮循环(本板约 8 us)的抖动 -- 对区分 1 us 后移与 125 us 跳变
// 无关紧要, 且让测量代码不必进 bsp/。
//
// 与其他诊断通道一样占用 DataId::kUart0, 彼此互斥。默认整体编译剔除。

#if defined(libhcs_APP_USB_RX_HIST) && libhcs_APP_USB_RX_HIST

inline constexpr bool kEnabled = true;

// 记录一次完成的 bulk OUT 传输, 每 500 ms 发出一条记录。
void note();

#else

inline constexpr bool kEnabled = false;

inline void note() {}

#endif

} // namespace libhcs::firmware::diag::usb_rx_hist
