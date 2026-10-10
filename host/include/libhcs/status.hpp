#pragma once

#include <array>
#include <cstdint>

#include <libhcs/export.hpp>

// v4 链路与板卡的公开状态类型（HCS-USB.md §8.1「状态」一行）：LinkState 沿用 v3
// 的三态，Statistics 与 hcs-ecat 的形状对齐（单调计数 + 对数分桶直方图），任何
// 线程随时可读，读到的是各字段独立的近似快照。

namespace libhcs {

/// 链路状态：只有一档是"一切正常"。
enum class LinkState : std::uint8_t {
    kUp,          // 会话在跑，数据双向流动
    kSessionDown, // 正在恢复：传输活着但会话断了（板子重启、端点挂起），保活线程在重开
    kFaulted,     // 不可恢复：设备消失。这个 Board 对象废了，只能重建
};

/// v4 的主机侧生命周期（HCS-USB.md §2.2）。kOpened / kConfigured 不是独立档位：
/// 打开与配置（EP0 清单一次事务）都在 start() 里完成，清单被接受之前会话根本
/// 不会开，所以对外可见的状态从 Armed 起。
enum class BoardState : std::uint8_t {
    kIdle,    // 构造后 / stop() 后：可声明端口，未碰设备
    kArmed,   // start() 成功：收数据正常，发送只放行安全帧
    kRunning, // run() 之后：正常收发
};

/// 以 2 为底的对数分桶直方图，单位 ns：第 i 桶计 [2^i, 2^(i+1)) ns，max_ns 记
/// 观测到的最大值（与 hcs-ecat 的 Histogram 同形，两个 SDK 的诊断可以同一套
/// 工具读）。这是快照形态（纯数组）；运行中的每桶一个原子计数在 SDK 内部，
/// 读时拷出来。
struct Histogram {
    std::array<std::uint64_t, 32> buckets{};
    std::uint64_t max_ns = 0;
};

/// 一块板从 start() 起的累计统计。全部单调计数，任何线程随时读；诊断用足够，
/// 不保证跨字段一致。
struct Statistics {
    std::uint64_t receives = 0;       // Board::receive() 的调用次数
    std::uint64_t sends = 0;          // Board::send() 的提交次数
    std::uint64_t rx_frames = 0;      // 进了 CAN 接收环的帧数（各通道合计）
    std::uint64_t rx_bytes = 0;       // 进了 UART 接收环的字节数
    std::uint64_t tx_frames = 0;      // 交给 IO 线程发出的 CAN 帧数
    std::uint64_t tx_bytes = 0;       // 交给 IO 线程发出的 UART 字节数
    std::uint64_t rx_dropped = 0;     // 接收环满而被丢最旧的记录数（帧 + 块）
    std::uint64_t tx_dropped = 0;     // stage 满 / 发送门关着 / 链路断而丢的记录数
    std::uint64_t session_losses = 0; // 会话断开（掉到 kSessionDown）的次数
    std::uint64_t uplink_stream_errors = 0; // 上行流解析或路由错误的次数
    Histogram keepalive_rtt;                // 保活往返（IO 线程测，直方图）
};

/// 一个通道的计数器（单调，任何线程读）。`board_*` 是板子随保活应答推送的
/// 运行时状态快照（CAN 控制器错误计数、UART 收错计数），core 的 datas.hpp 有
/// 各字段的解释。
struct ChannelCounters {
    std::uint64_t rx = 0;         // 进接收环的记录数（UART 是块数）
    std::uint64_t rx_dropped = 0; // 接收环满丢最旧
    std::uint64_t tx = 0;         // 交给 IO 线程的记录数（含安全帧）
    std::uint64_t tx_dropped = 0; // stage 满 / 门关着 / 链路断
    std::uint64_t safe_sends = 0; // 其中安全帧的次数
};

} // namespace libhcs
