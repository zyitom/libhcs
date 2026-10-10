#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <libhcs/board.hpp>
#include <libhcs/channel.hpp>
#include <libhcs/status.hpp>
#include <libhcs/transport.hpp>

#include "host/src/board/spec_table.hpp"
#include "host/src/channel/spsc_ring.hpp"
#include "host/src/session/data_sink.hpp"
#include "host/src/session/engine.hpp"

// Board 的内部结构：通道的存储（接收环 / 单拍快照 / 暂存区 / 安全帧）与板子
// 本体的实现核心（BoardCore，公开 Board 的 Impl 只是它的壳）。公开的
// CanChannel / UartChannel / ImuChannel 包着这里的指针，成员函数在 board.cpp。
//
// 线程分工（一处写清）：
//   IO 线程      往各 ring_* 写（DataCallback 路径）；drain 各 tx_ring_ 并发送；
//               链路掉线时补发安全帧。
//   使用者线程   receive() 把 ring_* 拷进 tick_*；stage() 写 staging_；send() 把
//               staging_ 搬进 tx_ring_（满了计丢弃）并敲门铃。
// 计数器全部 relaxed 原子：诊断用，不需要跨字段的序。

namespace libhcs::host::board {

namespace cfg = ::libhcs::board::hcs; // 声明类型（CanSetting/UartSetting/ImuSetting）

struct BoardCore; // 通道要回头问板子（状态门、EP0 学到的口能力）

struct CanChannelImpl {
    CanChannelImpl(
        BoardCore& owner, std::string_view port_name, data::DataId id,
        const cfg::CanSetting& can_setting, std::uint32_t rx_capacity, std::uint32_t tx_capacity)
        : owner_(&owner)
        , name(port_name)
        , data_id(id)
        , setting(can_setting)
        , ring_(rx_capacity)
        , tick_(rx_capacity)
        , staging_(tx_capacity)
        , tx_ring_(2 * tx_capacity) {}

    BoardCore* owner_;
    std::string_view name;
    data::DataId data_id = data::DataId::kExtend;
    cfg::CanSetting setting{};
    channel::DropOldestRing<CanFrame> ring_;    // IO 线程 → 使用者线程
    std::vector<CanFrame> tick_;                // receive() 的快照区（配置期定容）
    std::uint32_t tick_count_ = 0;
    std::vector<CanFrame> staging_;             // 单拍暂存（前 staged_ 条有效）
    std::uint32_t staged_ = 0;
    channel::DropOldestRing<CanFrame> tx_ring_; // 使用者线程 → IO 线程
    static constexpr std::size_t kMaxSafeFrames = 8;
    std::array<CanFrame, kMaxSafeFrames> safe_frames_{};
    std::uint32_t safe_count_ = 0;
    // start() 时按声明与板子报的能力填：这个口此刻能不能发长帧（>8 字节）。
    // 随 state_ 的 release/acquire 配对发布。
    bool long_frames_ok_ = false;

    std::atomic<std::uint64_t> rx_{0};
    std::atomic<std::uint64_t> tx_{0};
    std::atomic<std::uint64_t> tx_dropped_{0};
    std::atomic<std::uint64_t> safe_sends_{0};
};

struct UartChannelImpl {
    UartChannelImpl(
        BoardCore& owner, std::string_view port_name, data::DataId id,
        const cfg::UartSetting& uart_setting, std::uint32_t rx_capacity, std::uint32_t tx_capacity)
        : owner_(&owner)
        , name(port_name)
        , data_id(id)
        , setting(uart_setting)
        , ring_(rx_capacity)
        , tick_(rx_capacity)
        , staging_(tx_capacity)
        , tx_ring_(2 * tx_capacity) {}

    BoardCore* owner_;
    std::string_view name;
    data::DataId data_id = data::DataId::kExtend;
    cfg::UartSetting setting{};
    channel::DropOldestRing<UartChunk> ring_;
    std::vector<UartChunk> tick_;
    std::uint32_t tick_count_ = 0;
    std::vector<UartChunk> staging_;
    std::uint32_t staged_ = 0;
    channel::DropOldestRing<UartChunk> tx_ring_;
    UartChunk safe_chunk_{}; // 安全字节一整块；size 0 = 没声明
    std::atomic<std::uint64_t> rx_{0};
    std::atomic<std::uint64_t> rx_bytes_{0};
    std::atomic<std::uint64_t> tx_{0};
    std::atomic<std::uint64_t> tx_bytes_{0};
    std::atomic<std::uint64_t> tx_dropped_{0};
    std::atomic<std::uint64_t> safe_sends_{0};
};

struct ImuChannelImpl {
    explicit ImuChannelImpl(
        BoardCore& owner, const cfg::ImuSetting& imu_setting, std::uint32_t capacity)
        : owner_(&owner)
        , setting(imu_setting)
        , accelerometer_ring_(capacity)
        , gyroscope_ring_(capacity)
        , temperature_ring_(capacity)
        , accelerometer_(capacity)
        , gyroscope_(capacity)
        , temperature_(capacity) {}

    static constexpr std::string_view kName = "IMU";
    BoardCore* owner_;
    cfg::ImuSetting setting{};
    channel::DropOldestRing<ImuAccelerometer> accelerometer_ring_;
    channel::DropOldestRing<ImuGyroscope> gyroscope_ring_;
    channel::DropOldestRing<ImuTemperature> temperature_ring_;
    std::vector<ImuAccelerometer> accelerometer_;
    std::vector<ImuGyroscope> gyroscope_;
    std::vector<ImuTemperature> temperature_;
    std::uint32_t accelerometer_count_ = 0;
    std::uint32_t gyroscope_count_ = 0;
    std::uint32_t temperature_count_ = 0;
    std::atomic<std::uint64_t> rx_{0}; // 三路合计
};

// 板子的实现核心。Board 的公开方法逐个转到这里。
struct BoardCore : data::DataCallback {
    explicit BoardCore(BoardOptions options)
        : options_(std::move(options)) {}

    // ---- 生命周期（board.cpp）----
    void start();
    void stop() noexcept;
    void run();
    void arm();
    void request_safe(bool safe) noexcept;

    // ---- 周期（board.cpp）----
    void receive() noexcept;
    void send() noexcept;

    // ---- 状态（board.cpp）----
    [[nodiscard]] Statistics statistics() const noexcept;
    [[nodiscard]] std::string port_list_text(spec::PortKind kind) const;

    // ---- IO 线程回调（board.cpp）----
    void on_io_wake() noexcept;
    void on_link_state(LinkState state) noexcept;

    // DataCallback：上行记录 → 接收环。
    bool can_receive_callback(
        data::DataId id, const data::CanDataView& data,
        const libhcs::time::SampleTime& time) noexcept override;
    bool uart_receive_callback(
        data::DataId id, const data::UartDataView& data,
        const libhcs::time::SampleTime& time) noexcept override;
    bool gpio_digital_read_result_callback(
        std::uint8_t line, const data::GpioDigitalDataView& data,
        const libhcs::time::SampleTime& time) noexcept override;
    bool gpio_analog_read_result_callback(
        std::uint8_t line, const data::GpioAnalogDataView& data,
        const libhcs::time::SampleTime& time) noexcept override;
    void accelerometer_receive_callback(
        const data::ImuAccelerometerDataView& data,
        const libhcs::time::SampleTime& time) noexcept override;
    void gyroscope_receive_callback(
        const data::ImuGyroscopeDataView& data,
        const libhcs::time::SampleTime& time) noexcept override;
    void temperature_receive_callback(
        const data::ImuTemperatureDataView& data,
        const libhcs::time::SampleTime& time) noexcept override;

    // 私有工具（board.cpp）
private:
    void send_safe_frames_now() noexcept;

public:
    BoardOptions options_;
    const BoardTypeInfo* type_ = nullptr;
    std::unique_ptr<Transport> transport_;
    std::unique_ptr<session::Engine> engine_;

    std::atomic<BoardState> state_{BoardState::kIdle};
    std::atomic<bool> safe_requested_{false};
    bool safe_latched_ = false; // IO 线程（安全请求的边沿检测）

    // 声明攒在一份 v3 的 Configuration 里，start() 与重连钩子都用它跑 apply()。
    cfg::Configuration configuration_{};
    std::atomic<cfg::Interface> interface_{};
    std::mutex configure_mutex_; // add_* / set_safe_* 与重连钩子的 apply() 互斥

    std::vector<std::unique_ptr<CanChannel>> cans_;
    std::vector<CanChannel*> can_view_;
    std::vector<CanChannelImpl*> can_impls_; // BoardCore 内部走存储，不碰通道壳
    std::vector<std::unique_ptr<UartChannel>> uarts_;
    std::vector<UartChannel*> uart_view_;
    std::vector<UartChannelImpl*> uart_impls_;
    std::unique_ptr<ImuChannel> imu_;
    ImuChannelImpl* imu_impl_ = nullptr;
    // DataId → 通道存储（IO 线程的路由表，start 前填好，之后只读）。
    static constexpr std::size_t kDataIdSlots = 16;
    std::array<CanChannelImpl*, kDataIdSlots> can_by_id_{};
    std::array<UartChannelImpl*, kDataIdSlots> uart_by_id_{};

    std::atomic<std::uint64_t> receives_{0};
    std::atomic<std::uint64_t> sends_{0};
};

} // namespace libhcs::host::board
