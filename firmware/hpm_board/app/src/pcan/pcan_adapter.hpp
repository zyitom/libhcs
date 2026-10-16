#pragma once

// PCAN-USB FD 仿真的接缝: CAN 驱动与主循环接入本模块只经这个头。实现(USB 应用类
// 驱动、命令分发、记录流)在 pcan_adapter.cpp, EP0 与 USB 初始化胶水在
// pcan_control.cpp, 协议编解码在 pcan_protocol.hpp。本头刻意保持轻量 -- 不包含
// can/usb 任何头, 因为 can.cpp 反过来要包含它(与 dm_adapter.hpp 同一纪律)。
//
// ---- 上下文 ----
//
// 闸标志只由主循环(tud_task 里的 USB 回调、poll())写, ISR 只读。每路 CAN 的接收
// 队列由该路 CAN ISR 独占生产、主循环消费(utility::RingBuffer 的 SPSC 用法)。
// 接收过滤位图(64 行 x 32 位)由主循环整字写、CAN ISR 整字读: RV32 对齐 32 位读写
// 原子, 竞态窗口只是"一帧按半更新的位图判定", 与真实适配器的并发语义相同。
//
// 本模块只在 pcan 人格(libhcs_APP_PCAN_PERSONALITY)构建里编译: 那个镜像没有
// libhcs 会话与 DMTool, 板子整体就是一块 PCAN 卡(见 PCAN_PROTOCOL.md)。

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "firmware/hpm_board/app/src/pcan/pcan_protocol.hpp"

namespace libhcs::firmware::pcan {

// 表容量(双 CAN PCB); 单 CAN 板第二通道不构造, 见 can.hpp 的 can_count()。
inline constexpr std::size_t kChannelCount = 2;

// 接收过滤位图: 64 行 x 32 位标准 ID。行 = id >> 5, 位 = id & 31; 位 = 1 收。
inline constexpr std::size_t kFilterRows = 64;

// 需要主循环服务的功能, 位或进 internal::g_service。
inline constexpr uint8_t kServiceRx = 1U << 0;       // 有通道在收: 泵 data IN 记录流
inline constexpr uint8_t kServicePeriodic = 1U << 1; // 周期工作到点(CALIBRATION / ERROR / STATUS)
inline constexpr uint8_t kServiceTx = 1U << 2;       // 等待 CAN/回显槽的 USB OUT 包

namespace internal {

// 单写者(主循环), ISR 只读, 故只用 load/store, 不用 RMW(与 dmtool 同一纪律)。
inline constinit std::atomic<uint8_t> g_service{0};
// 位 i = 本板 CAN i 的接收帧进 PCAN 记录流。
inline constinit std::atomic<uint8_t> g_rx_mask{0};

// 标准 ID 接收位图。全 1 = 全收(Linux 只发全收; Windows 工具可能过滤)。
// constinit 零值是"全滤", Adapter 构造(早于任何 CAN 中断可达本模块)填全 1。
inline constinit std::array<std::array<uint32_t, kFilterRows>, kChannelCount> g_filter{};

void poll_slow();

} // namespace internal

// ---- 闸(内联) ----

// CAN RX ISR 调用(Can::handle_pcan_uplink 经 push_can_rx 前的过滤判定)。
[[nodiscard]] inline bool filter_accepts(std::size_t can_index, uint32_t std_id) {
    const auto& rows = internal::g_filter[can_index < kChannelCount ? can_index : 0];
    return (rows[std_id >> 5] & (1U << (std_id & 31U))) != 0U;
}

// CAN RX ISR 在排空路径调用(Can::drain_without_session 的 pcan 分支)。
[[nodiscard]] inline bool rx_enabled(std::size_t can_index) {
    return ((internal::g_rx_mask.load(std::memory_order::relaxed) >> can_index) & 1U) != 0;
}

// 主循环每轮调用: 没有任何服务待办时只有一次加载加一次分支。
inline void poll() {
    if (internal::g_service.load(std::memory_order::relaxed) != 0) [[unlikely]]
        internal::poll_slow();
}

// ---- ISR 侧入口 ----

// 一帧 CAN 事件。时间戳存 PTPC 原始 {秒, 纳秒}: 换算成真实微秒要 64 位除法,
// 放到主循环做, 不进优先级 3 的 CAN ISR(与 dmtool::CanFrameEvent 同一取舍)。
struct RxEvent {
    uint32_t id = 0;
    uint32_t timestamp_sec = 0;
    uint32_t timestamp_ns = 0;
    uint8_t dlc = 0; // 线上 DLC(0-15; 经典帧 DLC>8 已由驱动归一到 8)
    bool extended = false;
    bool remote = false;
    bool fd = false;
    bool bitrate_switch = false;
    bool error_state = false;
    std::array<uint8_t, 64> data{};
};

// CAN i 的 ISR 调用(Can::handle_pcan_uplink)。标准帧先过 g_filter 位图, 扩展帧
// 直接收(位图只覆盖标准 ID 空间, 与真实硬件一致)。队列满时丢帧并计数(OVERRUN
// 记录在主循环按计数差补发)。
void push_can_rx(std::size_t can_index, const RxEvent& event);

struct TxComplete {
    uint32_t timestamp_sec = 0;
    uint32_t timestamp_ns = 0;
    uint16_t flags = 0;
    uint8_t marker = 0;
};

// 仅在真实 TX Event FIFO 报告发送成功后调用, 不把 USB 收到请求当作 CAN 已发出。
void push_tx_complete(std::size_t can_index, const TxComplete& event);

// ---- 主循环侧 ----

// 1 kHz tick 块调用: 周期工作(CALIBRATION 到点、ERROR/STATUS 变化检测)的排期。
void poll_tick(uint32_t tick);

// EP0 的 REQ_FCT/DRVLD(pcan_control.cpp 调用): 主机驱动装载状态, LED 判据用。
void set_driver_loaded(bool loaded);

// LED 判据(app_pcan.cpp 的 1 kHz tick 块): PCAN 的"驱动已加载且至少一路总线上线"。
[[nodiscard]] bool session_established();

// 设备序列号与通道 device id: EP0 的 fw_info/blinfo 应答与 USB 字符串描述符共用,
// 在 Adapter 构造时从 OTP 派生一次(DEVID_SET 只改 RAM)。
[[nodiscard]] uint32_t serial_number();
[[nodiscard]] uint32_t device_id(std::size_t channel);

// USB 初始化与 EP0 / 引导胶水(实现在 pcan_control.cpp; 主循环的 DFU 延时重启也
// 在那里)。声明放本头是因为 app_pcan.cpp 经它取用。
void usb_init();
void poll_dfu_reboot();

} // namespace libhcs::firmware::pcan
