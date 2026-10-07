#pragma once

// DMTool 仿真的接缝: CAN/UART 驱动、主循环与 libhcs 会话接入本模块只经这个头。
// 实现(USB 应用类驱动、命令分发、记录流、CDC 桥)在 dm_adapter.cpp, 协议编解码
// 在 dm_protocol.hpp。本头刻意保持轻量 -- 不包含 can/uart/usb 任何头, 因为
// uart.hpp、can.cpp、vendor.hpp 反过来要包含它。
//
// ---- 与 libhcs 的关系: 互斥, libhcs 优先, libhcs 的路径不多一条指令 ----
//
// 板子归谁由 link/ownership.hpp 的状态机决定, 切换点是 libhcs 的 EP0 握手。本模块
// 只挂在"板子不归 libhcs 会话"的分支上:
//   - CAN / UART RX ISR 先判 libhcs 会话, 会话不在时才调用本模块的冷函数;
//   - 主循环的 DMTool 工作挂在 HostSession::next_batch() 本来就有的"会话未建立"
//     分支里(poll());
//   - DMTool 的三个 USB 接口由本模块的 TinyUSB 应用类驱动承接, 不占 vendor 类
//     实例: libhcs 仍是 vendor 实例 0, bulk 回调与发送路径与没有 DMTool 时相同;
//   - libhcs 一握手, 本模块整体让位: 采集、CDC 桥全部关闭, 队列与 CDC 的收发 FIFO
//     清空, 端点对主机不再应答(Handoff::to_libhcs())。
// 以上都对照没有 DMTool 支持的镜像逐函数比过反汇编(DMTOOL_PROTOCOL.md 第 5 节)。
// 本模块的代码全部留在 FLASH, 不占 libhcs 热路径用的 ILM。
//
// ---- 上下文 ----
//
// 闸标志只由主循环(tud_task 里的 USB 回调、poll())写, ISR 只读。每路 CAN 的接收
// 队列由该路 ISR 独占生产、主循环消费; UART->CDC 队列由 UART ISR 生产。全是
// utility::RingBuffer 的 SPSC 用法。

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "firmware/hpm_board/app/src/dmtool/dm_protocol.hpp"

namespace libhcs::firmware::dmtool {

// DMTool 的通道号是 bool(界面的"通道1/通道2" = 0/1), 映射到本板 CAN 序号
// 0/1(丝印 CAN1/CAN2)。
inline constexpr std::size_t kChannelCount = 2;

// 需要主循环服务的功能, 位或进 internal::g_service。
inline constexpr uint8_t kServiceCapture = 1U << 0;   // 有 CAN 在采集: 泵 0x81 / 0x83 记录流
inline constexpr uint8_t kServiceCdcBridge = 1U << 1; // CDC 串口已按 UART 实际波特率打开
inline constexpr uint8_t kServiceAck = 1U << 2;       // 有一条应答等 0x82 端点空出

namespace internal {

// 见文件头注释: 单写者(主循环), ISR 只读, 故只用 load/store, 不用 RMW。
inline constinit std::atomic<uint8_t> g_service{0};
// 位 i = 本板 CAN i 的接收帧进 DMTool 记录流。
inline constinit std::atomic<uint8_t> g_capture_mask{0};
// CDC 桥上最近一次真有字节走过(任一方向)时的 1 kHz tick。只在主循环读写。
inline constinit uint32_t g_cdc_traffic_tick = 0;
inline constinit bool g_cdc_traffic_seen = false;

void poll_slow();

} // namespace internal

// ---- 闸(内联) ----

// CAN RX ISR 在 libhcs 会话不在时调用(Can::drain_without_session)。
[[nodiscard]] inline bool capture_enabled(std::size_t can_index) {
    return ((internal::g_capture_mask.load(std::memory_order::relaxed) >> can_index) & 1U) != 0;
}

// CDC 桥接通之后多久没有字节走过就不再算"在用"(1 kHz tick)。
inline constexpr uint32_t kCdcIdleTicks = 1000;

// 主机视角的"DMTool 会话已建立": 任一通道在采集, 或 CDC 串口桥接通且最近 1 s 内真有
// 字节走过。供 app.cpp 的 LED 判据使用(主循环 1 kHz tick 调用) -- 常绿与 libhcs 会话
// 同义, 否则 DMTool 连接期间 LED 一直停在等待闪烁。
//
// 桥接通本身不算: 它不看 DTR(Windows 上 DMTool 打开串口不拉 DTR), 而 Linux 的 cdc_acm
// 在枚举时就发一次 SET_LINE_CODING(9600 8N1), 板子把它写进 UART, 速率自然对上, 桥随即
// 接通 -- 只看桥的话, 每次刷写或插上之后 LED 都常亮, 不再慢闪。[实测 2026-10-05: 闲置的
// 5321 常亮绿]
[[nodiscard]] inline bool session_established(uint32_t tick) {
    if (internal::g_capture_mask.load(std::memory_order::relaxed) != 0)
        return true;
    if ((internal::g_service.load(std::memory_order::relaxed) & kServiceCdcBridge) == 0)
        return false;
    return internal::g_cdc_traffic_seen && tick - internal::g_cdc_traffic_tick < kCdcIdleTicks;
}

// 主循环在没有 libhcs 会话的轮次调用(HostSession::next_batch 的会话未建立分支)。
// DMTool 与 CDC 都没在用时只有一次加载加一次分支。
inline void poll() {
    if (internal::g_service.load(std::memory_order::relaxed) != 0) [[unlikely]]
        internal::poll_slow();
}

// ---- ISR 侧入口 ----

// 一帧 CAN 事件。时间戳存 PTPC 原始 {秒, 纳秒}: 换算成真实纳秒要 64 位除法,
// 放到主循环做, 不进优先级 3 的 CAN ISR。
struct CanFrameEvent {
    uint32_t id = 0;
    uint32_t timestamp_sec = 0;
    uint32_t timestamp_ns = 0;
    uint8_t dlc = 0;
    protocol::CanFrameFlags flags;
    std::array<uint8_t, 64> data{}; // 元素已扩到 64 字节: DLC 9-15 的 FD 长帧照实交付
};

// CAN i 的 ISR 调用(Can::handle_dm_uplink)。队列满时丢帧。
void push_can_rx(std::size_t can_index, const CanFrameEvent& event);

// UART ISR 在 libhcs 会话不在时调用: CDC 串口桥接通时字节进其上行队列, 否则
// 丢弃。队列满时丢字节 -- 与真实 USB 转串口在主机不读时的表现一致。参数是标量
// 而非 span: 见 DMTOOL_PROTOCOL.md 第 5 节(关乎调用方 libhcs 分支的代码生成)。
[[gnu::cold]] void uart_rx_without_session(
    const std::byte* data, std::size_t size, const std::byte* data2, std::size_t size2);

// ---- 板子的归属 ----

// link/ownership.hpp 的一个交接步骤(HandoffStep)。
//
// to_libhcs(): libhcs 握手, 本模块让位。让位不止是"不处理": 除了 EP0、libhcs 自己那
// 一对 bulk 端点与 CDC 的 bulk OUT, 这块板的其余端点(DMTool 的三对、CDC 串口的 bulk IN
// 与通知端点)在控制器里全部关掉, 对主机的令牌不应答。原因在总线上而不在 CPU 上 --
// 主机若还开着 DMTool 或开着串口, 它挂在 IN 端点上的读请求会让主机控制器不停地轮询
// 那个端点, 白占 libhcs 的总线时间(串口开着时洪泛包率实测掉 25%)。为什么是不应答而
// 不是 STALL、各自的实测代价, 以及 CDC 的 bulk OUT 为什么例外, 见 dm_adapter.cpp 的
// Adapter::isolate() 与 DMTOOL_PROTOCOL.md 5.1。代价是 libhcs 接管期间 DMTool 必然报错
// 断开、串口读不到数据、写进来的字节被丢掉 -- 它们本就是没有 libhcs 主机时的附加功能,
// HCS 退出后照常可用。只在板子真正易手时调一次; 实现仍是幂等的。
//
// to_tools(): 板子交还附加功能(会话结束、握手后没开出会话、挂起、拔线)。所有端点
// 恢复应答, DMTool 的端点解除 halt 并重新挂上 OUT, DMTool 可以重新打开本板; CDC 的
// bulk IN 留着 halt 给主机下次读串口时自己清(理由见 Adapter::release())。主机上一直
// 开着的串口不用重新打开, 桥按它记着的线路编码重新接通。没让位过时是空操作。
struct Handoff {
    static void to_libhcs();
    static void to_tools();
};

} // namespace libhcs::firmware::dmtool
