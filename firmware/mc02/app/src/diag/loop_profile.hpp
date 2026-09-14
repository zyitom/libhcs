#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace libhcs::firmware::diag::profile {

// 主循环的时间到底花在哪 -- 用测量回答, 不靠争论。
//
// 要回答的问题: 聚合 UART 转发超过约 780 KB/s 后板子开始丢数据, 且吞吐随负载
// 上升反而下降 -- 是拥塞崩溃, 不是链路饱和。此刻打开 IMU 还要再损失约 18.5%,
// 证明瓶颈确实在 CPU(等饱和 USB 链路的 CPU 本可以把这份额外工作白拿走)。
// 但这回答不了循环里哪一段贵: "主循环工作"是一个类目, 不是函数。
//
// 测量手段: 每个 section 边界读一次 DWT->CYCCNT, 把差值记到刚离开的段。
// DWT 是 550 MHz 自由运行的 32 位周期计数器, 已在 app.cpp 使能, 此前无人读取。
// 每轮约 12 个边界、每次约 5 个周期, 对 2.6 us 的一轮约占 4% -- 足以轻微移动
// 绝对数值, 不足以改变各项间的排序, 而这正是本工具的目的。默认整体编译剔除。

#if defined(libhcs_APP_LOOP_PROFILE) && libhcs_APP_LOOP_PROFILE

inline constexpr bool kEnabled = true;

enum class Section : std::uint8_t {
    kTudTask, // TinyUSB 设备任务: USB 栈自身的处理
    kUsb,     // usb::vendor->try_transmit(), 全部七处交错调用
    kCan,     // Can::drain_pending_transmits(): 三路 CAN 队列
    kUart,    // uart1/2/3/dbus 的 try_transmit(): RX 出队加 TX 出队
    kImu,     // BMI088 探询与待读服务
    kLed,     // 会话状态加 WS2812 刷新
    kGpio,    // 周期性输入采样
    kOther,   // 未归入上述各段的其余部分, 含主循环本身
    kCount,
};

// 把上次 mark 以来的周期数记到正离开的段, 然后开始计时指定段。比每段一个
// RAII 作用域便宜: 每边界读一次计数器而非两次。
void mark(Section section);

// 结束本轮并计数, 每 kEmitPeriodMs 发出一条记录。
void end_pass();

#else

inline constexpr bool kEnabled = false;

enum class Section : std::uint8_t {
    kTudTask,
    kUsb,
    kCan,
    kUart,
    kImu,
    kLed,
    kGpio,
    kOther,
    kCount,
};

inline void mark(Section) {}
inline void end_pass() {}

#endif

} // namespace libhcs::firmware::diag::profile
