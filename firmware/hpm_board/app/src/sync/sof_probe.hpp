#pragma once

// USB-SOF 跨板时间基准的第一步: 在任何时间线算法写就之前, 先证明 FRINDEX
// 确实随每个 SOF 中断精确加一。
//
// 为何单列一个固件步骤: 整套方案依赖设备控制器的帧索引是主机 microframe
// 计数器的被动硬件副本 -- 挂在同一主机控制器下的每块板都由相同 SOF 边沿
// 驱动, 读数必然一致。若控制器在递增 FRINDEX 之前先置 SOF-received 标志,
// 中断入口处读到的就是旧值, 观测到的 delta 序列退化为 0, 2, 0, 2 交替。
// 两种情形都给出看似正常的计数器, 却恰差一个 microframe 的系统性偏移 --
// 125 us, 比设计目标 ~1 us 高两个数量级。等回绕计数状态机和主机协议都
// 上线之后再排查, 就得同时分辨算法缺陷与硅片竞态; 故在此单独回答, 板上
// 无其他干扰。
//
// 记录因此携带如下独立判据:
//   * 相邻 FRINDEX delta 的完整直方图(主要证据);
//   * 每个异常 delta 对应的 USBSTS、PORTSC1(端口速率、挂起、PHY 低功耗)
//     与毫秒 tick -- 足以区分枚举期伪象与稳态现象;
//   * 同一中断内紧跟首读之后的第二次 FRINDEX 读 -- "advanced within ISR"
//     计数非零, 即恰好采在递增边沿上的直接特征;
//   * ISR 到 ISR 的本地定时器间隔, 应以 125 us 为中心。delta 为 1 而间隔
//     250 us 意味着中断丢失、计数器静默出错, delta 直方图自身看不出这一点。
//
// 寄存器读取与 SRI 应答都在 sync/sof.cpp -- 它与本模块及 sync::timebase
// 共享唯一的 SOF 钩子; 探针只接收读数。
//
// 未定义 libhcs_APP_SOF_DIAG 时整体编译剔除, 量产镜像既不含计数器, 也不含
// 8 kHz 中断。

#include <cstdint>

namespace libhcs::firmware::sync::sof_probe {

#if defined(libhcs_APP_SOF_DIAG) && libhcs_APP_SOF_DIAG

inline constexpr bool kEnabled = true;

// UART0 上行载荷的线上格式。小端; 除尾部异常条目(条目数随记录携带)外
// 布局固定。
inline constexpr std::uint8_t kRecordMagic = 0xD2U;
inline constexpr std::uint8_t kRecordVersion = 2U;

// delta 桶 0..8, 另有一桶兜住 9 及以上。
inline constexpr std::uint32_t kDeltaBuckets = 10U;

// 每条记录保留的异常数。每次上报即清空, 运行中随时可见, 而非只看到开机后
// 最初几条 -- 初版记录(每 100 ms 仅 8 槽)在枚举期即被填满, 此后的异常全部
// 无人看见。容量远高于实测每条约 3 条; 主机将存量计数与自增总数交叉核对,
// 溢出不可能静默通过。
inline constexpr std::uint32_t kAnomalyCapacity = 12U;

// ISR 路径, 由 sync::sof_isr_entry() 携其读得的值调用。
void note_sof(
    std::uint32_t frindex, std::uint32_t frindex_again, std::uint32_t now_quarter_us,
    std::uint32_t usbsts, std::uint32_t portsc1);

// 主循环采样器; 1 kHz tick 下每 kEmitPeriodMs 上报一条记录。
void poll(std::uint32_t tick);

#else

inline constexpr bool kEnabled = false;

inline void note_sof(std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t) {}
inline void poll(std::uint32_t) {}

#endif

} // namespace libhcs::firmware::sync::sof_probe
