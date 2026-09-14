#pragma once

// 针对 CAN 转发停滞的遥测(缘起见 ecat/CORE_SWAP_MIGRATION.md 第 6 节): 单核镜像
// (USB 与 CAN 同挂一个 PLIC)高负载时, 主机停止收到转发帧, 且持续到复位为止。
// 本板没有接 JTAG 或串口控制台, 正常要用停机调试器读的状态改为在主循环采样, 作为
// UART0 上行帧发出。
//
// 停滞期间主机会话仍在运行 -- protocol::Handler 在 keepalive ack 无应答时会终止
// 进程, 而当时的运行全程都在打印 -- 所以数据面停了, 这些记录途经的路径也没停。
// 这正是主循环遥测可用的前提。
//
// 一条快照刻意混装三层, 以便区分故障位置:
//   * ISR 进入计数          -- CAN 中断还在送达吗?
//   * MCAN IR/RXF0S/PSR/ECR -- 控制器还看得到总线流量吗?
//   * PLIC pending/enable   -- 是否有源在请求却从未送达?
// 进入计数冻结 + RXF0S 非空 + PLIC pending 置位 = 中断丢在网关与核之间;
// 进入计数冻结 + RXF0S 空闲 + PSR bus-off = 停的是控制器本身。
//
// 未定义 libhcs_APP_CAN_DIAG 时整体编译消失, 生产构建的转发热路径零开销。

#include <cstddef>
#include <cstdint>

namespace libhcs::firmware::diag {

#if defined(libhcs_APP_CAN_DIAG) && libhcs_APP_CAN_DIAG

inline constexpr bool kEnabled = true;

// UART0 上行负载的线上格式。小端、定长布局, 主机解码器无需长度协商。
inline constexpr std::uint8_t kRecordMagic = 0xD1U;
inline constexpr std::uint8_t kRecordVersion = 6U;

// 热路径通知。全部是单次 relaxed 原子加; CAN ISR 分别按每次中断、每个转发帧调用
// 前两个。
void note_isr_entry(std::size_t can_index);
void note_frame(std::size_t can_index);
void note_tx_fail(std::size_t can_index);
void note_alloc_fail();

// 主循环每迭代自增一次。按相对上一条记录的迭代数上报, 除以记录周期即得循环周期
// -- 下行字节或上行批在轮到它之前要付的等待延迟。
void note_main_loop();

// 由 Can::poll() 每次释放卡住的 PLIC claim 时计数。
void note_irq_recovered(std::size_t can_index);

// USB bulk OUT 端点计时, 由接收完成回调调用。把每包开销拆成判断链式 qTD 是否
// 值得做的仅有的两段:
//
//   turnaround = complete -> 重挂。设备侧。链式 qTD 消除的就是它: 当前传输完成时
//                下一个缓冲已备好。
//   starve     = 重挂 -> 下一次 complete。端点就绪且空闲, 这里流逝的是总线时间加
//                主机控制器发出下一个 token 的耗时, 链式 qTD 缩短不了。
//
// 两者都用 CSR_MCYCLE 采样(一条指令, 核时钟分辨率), 测量不扰动被测对象 --
// 本记录自身的历史里就有一条"观察者摧毁被测对象"的教训。
void note_usb_out_complete();
void note_usb_out_armed();

// 主循环采样器。每 kEmitPeriodMs 个 1 kHz tick 发出一条记录; 上行未在传数据时
// 为空操作。
void poll(std::uint32_t tick);

#else

inline constexpr bool kEnabled = false;

inline void note_isr_entry(std::size_t) {}
inline void note_frame(std::size_t) {}
inline void note_tx_fail(std::size_t) {}
inline void note_alloc_fail() {}
inline void note_main_loop() {}
inline void note_irq_recovered(std::size_t) {}
inline void note_usb_out_complete() {}
inline void note_usb_out_armed() {}
inline void poll(std::uint32_t) {}

#endif

} // namespace libhcs::firmware::diag
