#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>

#include <libhcs/data/datas.hpp>
#include <libhcs/export.hpp>
#include <libhcs/status.hpp>
#include <libhcs/time/sample_time.hpp>

// v4 的拉取式通道（HCS-USB.md §2.3）：IO 线程把板子发来的记录写进每个通道的
// 接收环，使用者在自己的线程里 receive() 之后用 frames() / bytes() / samples()
// 读这一拍的数据；发送用 stage() 暂存、Board::send() 统一提交。周期阶段的方法
// 全部 noexcept：满了返回 false 并计数，不抛异常——发送侧的配置校验（口存不存
// 在、帧型）在 add_* 声明阶段就做完了。
//
// 通道对象由 Board 创建并持有，引用终身稳定；start() 之前从 add_* 拿到，之后
// 只用这个句柄，不再按名字查。

namespace libhcs {

/// 一帧 CAN（经典或 FD 短/长帧的负载都在这里，8 字节以上是长帧编码）。
/// `time` 是 SDK 在 IO 线程上算好的样本时刻：SOF 轴锁定时来自硬件时间戳
/// （Source::kBoardStamp），否则是到达时刻（Source::kHostArrival）。
struct CanFrame {
    std::uint32_t id = 0;
    bool extended = false;
    bool remote = false;
    std::uint8_t size = 0; // 负载字节数，0..64
    std::array<std::byte, 64> data{};
    time::SampleTime time{};

    friend constexpr bool operator==(const CanFrame&, const CanFrame&) = default;
};

/// 一段上行串口字节。板子按自己的批量切法送，一段就是一个 UartChunk；串口
/// 数据没有硬件时间戳，`time` 总是到达时刻。
inline constexpr std::size_t kUartChunkBytes = 512;

struct UartChunk {
    std::uint16_t size = 0;      // ≤ kUartChunkBytes
    bool idle_delimited = false; // 板子在空闲定界处切的块（协议帧边界提示）
    std::array<std::byte, kUartChunkBytes> data{};
    time::SampleTime time{};

    friend constexpr bool operator==(const UartChunk&, const UartChunk&) = default;
};

/// 板载 IMU 的一路样本。原始值（传感器寄存器整数），换算成物理量用声明时
/// 的 ImuSetting 里的量程——SDK 只到字节，缩放是使用者的算术。
struct ImuAccelerometer {
    std::int16_t x = 0, y = 0, z = 0;
    time::SampleTime time{};
};
struct ImuGyroscope {
    std::int16_t x = 0, y = 0, z = 0;
    time::SampleTime time{};
};
struct ImuTemperature {
    std::uint16_t raw_register_value = 0;
    time::SampleTime time{};
};

/// ImuChannel::samples() 一次交出的三路样本。
struct ImuData {
    std::span<const ImuAccelerometer> accelerometer;
    std::span<const ImuGyroscope> gyroscope;
    std::span<const ImuTemperature> temperature;
};

/// 各通道的状态快照：SDK 侧计数器 + 板子随保活应答推送的运行时状态。
struct CanChannelStatus {
    ChannelCounters counters;
    data::CanStatusView board; // 板上控制器的最新快照（至多晚一个保活轮次）
};
struct UartChannelStatus {
    ChannelCounters counters;
    data::UartStatusView board;
};
struct ImuChannelStatus {
    ChannelCounters counters; // rx 按"路"合并计（accel/gyro/temp 各记一条）
};

/// 一条 CAN 总线通道。由 Board::add_can 创建。
class libhcs_API CanChannel final {
public:
    CanChannel(const CanChannel&) = delete;
    CanChannel& operator=(const CanChannel&) = delete;
    ~CanChannel();

    /// 丝印名（"CAN1"），即声明时传给 add_can 的名字。
    [[nodiscard]] std::string_view name() const noexcept;

    // ---- 周期阶段（receive() 之后读；stage 任何时候可调）----

    /// 这一拍收到的帧，按到达顺序。只在 Board::receive() 之后到下一次
    /// receive() 之间有效。
    [[nodiscard]] std::span<const CanFrame> frames() const noexcept libhcs_NONBLOCKING;

    /// 暂存一帧等 Board::send() 提交。满了或帧不合法（长帧但口不是 FD、还没
    /// start()）返回 false 并计数，不抛。
    bool stage(const CanFrame& frame) noexcept libhcs_NONBLOCKING;

    // ---- 配置阶段 ----

    /// 声明安全帧：request_safe / 会话断开 / Armed 状态下，这个口发的是这里
    /// 登记的帧（按登记顺序），而不是暂存的内容。最多 8 帧；帧必须与本口声明的
    /// 帧型相容（经典口上 8 字节以内）。只能在 start() 之前设置。
    void set_safe_frames(std::span<const CanFrame> frames);

    /// 本口的计数与板子报的运行时状态。
    [[nodiscard]] CanChannelStatus status() const noexcept;

private:
    friend class Board;
    struct Impl;
    explicit CanChannel(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

/// 一路串口通道。由 Board::add_uart 创建。
class libhcs_API UartChannel final {
public:
    UartChannel(const UartChannel&) = delete;
    UartChannel& operator=(const UartChannel&) = delete;
    ~UartChannel();

    [[nodiscard]] std::string_view name() const noexcept;

    // ---- 周期阶段 ----

    /// 这一拍收到的字节块，按到达顺序。
    [[nodiscard]] std::span<const UartChunk> chunks() const noexcept libhcs_NONBLOCKING;

    /// 暂存一段字节（≤ kUartChunkBytes）等 Board::send() 提交。满了、超长或
    /// 还没 start() 返回 false 并计数，不抛。
    bool stage(std::span<const std::byte> bytes, bool idle_delimited = false) noexcept
        libhcs_NONBLOCKING;

    // ---- 配置阶段 ----

    /// 声明安全字节（≤ kUartChunkBytes）：语义同 CanChannel::set_safe_frames，
    /// 每拍发一次。只能在 start() 之前设置。
    void set_safe_bytes(std::span<const std::byte> bytes, bool idle_delimited = false);

    [[nodiscard]] UartChannelStatus status() const noexcept;

private:
    friend class Board;
    struct Impl;
    explicit UartChannel(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

/// 板载 IMU 通道（一块板至多一个）。由 Board::add_imu 创建。
class libhcs_API ImuChannel final {
public:
    ImuChannel(const ImuChannel&) = delete;
    ImuChannel& operator=(const ImuChannel&) = delete;
    ~ImuChannel();

    [[nodiscard]] std::string_view name() const noexcept; // "IMU"

    /// 这一拍收到的三路样本，各自按时间序。
    [[nodiscard]] ImuData samples() const noexcept libhcs_NONBLOCKING;

    [[nodiscard]] ImuChannelStatus status() const noexcept;

private:
    friend class Board;
    struct Impl;
    explicit ImuChannel(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

} // namespace libhcs
