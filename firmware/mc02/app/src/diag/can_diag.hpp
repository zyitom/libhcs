#pragma once

// mc02 的 CAN 遥测, 仿照 hpm_board 的 app/src/diag/can_diag.hpp。
//
// 本板此前完全无法上报 CAN 控制器状态: 读不到 PSR/ECR/TXFQS, 也没有任何计数器。
// 于是排查疑似转发故障时唯一的观测手段就是上位机收到与否, "帧根本没上总线"与
// "发了但随后丢了"在上位机侧无法区分。hpm_board 因自身 CAN 闩锁问题的排查
// 早已有此通道, mc02 现在补上等价物。
//
// 借用 DataId::kUart0 上行: 该 id 在本板并不对应丝印 UART(丝印为
// kUart1/2/3/7/10 加 DBUS), 诊断构建因此不再与外壳 RS-485 口冲突。
// hpm_board 的诊断用同一 id, 上位机解码器可共用头部, 由 record version 区分两者。
//
// 未定义 libhcs_APP_CAN_DIAG 时整体编译剔除, 生产构建的转发热路径零负担。

#include <cstddef>
#include <cstdint>

namespace libhcs::firmware::diag {

#if defined(libhcs_APP_CAN_DIAG) && libhcs_APP_CAN_DIAG

inline constexpr bool kEnabled = true;

// kUart0 上行负载的线上格式。小端, 固定布局, 上位机解码器无需长度协商。
// 版本 >= 64 表示 mc02 变体: 每控制器块是 FDCAN(PSR/ECR/TXFQS)而非 MCAN,
// 解码器不得将其当作 hpm_board 的记录。
inline constexpr std::uint8_t kRecordMagic = 0xD1U;
inline constexpr std::uint8_t kRecordVersion = 64U;

// 热路径通知: 各自只是一次 relaxed 原子加。前两个由 RX ISR 调用,
// 分别按中断次数与按转发的帧数计数。
void note_isr_entry(std::size_t can_index);
void note_frame(std::size_t can_index);
// 已被要求发送但未能入队(软件环形队列满)的帧数。
void note_tx_fail(std::size_t can_index);
// 因上行批量池满而丢弃的已收帧数, 即曾经的静默丢帧路径。
void note_uplink_drop(std::size_t can_index);

void note_main_loop();

// 主循环采样器, 每 kEmitPeriodMs 发出一条记录; 会话未建立时为空操作。
void poll();

#else

inline constexpr bool kEnabled = false;

inline void note_isr_entry(std::size_t) {}
inline void note_frame(std::size_t) {}
inline void note_tx_fail(std::size_t) {}
inline void note_uplink_drop(std::size_t) {}
inline void note_main_loop() {}
inline void poll() {}

#endif

} // namespace libhcs::firmware::diag
