#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <utility>

#include <libhcs/data/datas.hpp>
#include <libhcs/protocol/vendor_control.hpp>
#include <libhcs/status.hpp>
#include <libhcs/time/sample_time.hpp>
#include <libhcs/transport.hpp>

#include "core/src/protocol/deserializer.hpp"
#include "host/src/logging/logging.hpp"
#include "host/src/session/data_sink.hpp"
#include "host/src/time/board_timebase.hpp"
#include "host/src/time/usb_frame_axis.hpp"
#include "host/src/utility/seqlock.hpp"

namespace libhcs::host::session {

// v4 的链接引擎（HCS-USB.md §2.1 的 session/）：会话握手与租约、保活、共享
// 时间基准、端口状态账本、EP0 透传，外加一个 IO 线程——会话保活与门铃驱动的
// 发送提交都跑在它上面（v3 的 Handler 拆掉 PacketBuilder 之后的形态；传输层从
// 内部 buffer 接口换成了公开的 libhcs::Transport）。
//
// 数据去向是 data::DataCallback（Board::Impl 实现，写各通道的接收环）；发送由
// Board 的 wake 钩子在 IO 线程上经 StreamWriter 完成。除 start() 外没有阻塞的
// 公开调用。
class Engine final : public core::protocol::DeserializeCallback {
public:
    /// IO 线程与 USB 事件线程的放置（BoardOptions 同名字段填进来）。
    struct ThreadConfig {
        int cpu = -1;
        int rt_priority = 0;
        void (*setup)() noexcept = nullptr;
    };

    /// @param transport 已打开的传输。引擎不拥有它，但引用终身稳定。
    Engine(Transport& transport, data::DataCallback& sink, ThreadConfig threads);

    /// EP0 载荷上限（v3 Handler 同名常量搬来）：最大板型的清单。EP0 的收发都
    /// 在一块固定 scratch 里过，不按请求分配。
    static constexpr std::size_t kVendorControlPayloadMax =
        sizeof(core::protocol::vendor_control::ManifestPayload);

    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;
    Engine(Engine&&) = delete;
    Engine& operator=(Engine&&) = delete;

    ~Engine() override { stop(); }

    // ---- 钩子（start() 之前装好）----
    /// 会话（重）开之前在 IO 线程跑一次：板子会忘掉握手，重连必须重放 EP0 配置。
    void set_before_session(std::function<void()> hook) { before_session_ = std::move(hook); }
    /// IO 线程被门铃叫醒时跑（Board 的发送提交 / 安全帧补发）。不抛。
    void set_wake_hook(std::move_only_function<void() noexcept> hook) {
        wake_hook_ = std::move(hook);
    }
    /// 链路状态变化时在 IO 线程跑一次（Board 借此在掉线时补发安全帧）。
    void set_link_observer(std::function<void(LinkState)> observer) {
        link_observer_ = std::move(observer);
    }

    // ---- 生命周期 ----
    /// 开会话并起 IO 线程。超时抛 std::runtime_error。可重复（stop 之后）。
    void start();
    /// 停 IO 线程。幂等，不抛。
    void stop() noexcept;

    // ---- 周期阶段 ----
    /// 门铃：叫醒 IO 线程跑 wake 钩子。RT 线程上调用：一次原子置位加一次
    /// notify，绝不等待——锁只 try：拿不到说明 IO 线程正拿着它在跑（它进等待
    /// 前会再看一眼标志），跳过 notify 也不会丢这次叫醒。
    void notify_io() noexcept {
        wake_pending_.store(true, std::memory_order_release);
        if (session_mutex_.try_lock()) {
            session_mutex_.unlock();
            session_cv_.notify_all();
        }
    }

    // ---- 状态（任何线程）----
    [[nodiscard]] LinkState link_state() const noexcept;
    /// 一个口的运行时状态快照（板子随保活应答推送的最新值）。
    [[nodiscard]] data::PortStatusVariant port_status(data::DataId port) const noexcept;

    // ---- 时间 ----
    void set_time_sync(bool on);
    /// 主机时刻 → 板钟（两个映射都锁定后才有值）。IO 线程语义同 v3。
    [[nodiscard]] std::optional<libhcs::time::BoardClock::time_point>
        board_time_at(libhcs::time::HostTime when) const;

    // ---- EP0 配置通道（配置期，阻塞；满足 hcs_config.hpp 的 Ep0Channel）----
    bool vendor_control_out(
        std::uint8_t request, std::uint16_t index, const void* payload, std::size_t size);
    bool vendor_control_in(
        std::uint8_t request, std::uint16_t index, void* payload, std::size_t size);

    // ---- 统计（任何线程读）----
    void keepalive_rtt_into(Histogram& out) const noexcept;
    [[nodiscard]] std::uint64_t session_losses() const noexcept {
        return session_losses_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] std::uint64_t uplink_stream_errors() const noexcept {
        return uplink_stream_errors_.load(std::memory_order_relaxed);
    }

    /// 跨板对时测量专用（固件 PULSE_TEST 构建）；不属于控制路径。
    void send_pulse_schedule(std::uint64_t microframe);

private:
    void io_loop();
    void apply_thread_config() const noexcept;
    template <typename View>
    [[nodiscard]] libhcs::time::SampleTime imu_sample_time(const View& data) noexcept;

    void establish_session();
    void refresh_session();
    void send_session_control(data::SessionType type, std::string_view operation_name);
    void send_time_anchor();
    void handle_session_failure(const std::exception& exception) noexcept;
    void reset_link_state();
    void receive_stream(std::span<const std::byte> buffer);
    void attach_time_source();
    [[nodiscard]] time::Affine<time::FrameClock, time::HostClock> axis() const noexcept;
    [[nodiscard]] bool session_established() const noexcept {
        return session_established_.load(std::memory_order_acquire);
    }
    void note_link_state(LinkState now) noexcept;
    bool vendor_control(
        std::uint8_t request_type, std::uint8_t request, std::uint16_t index,
        std::span<std::byte> payload);

    // DeserializeCallback：上行记录进 Board 的接收环（经 sink）。异常在
    // guard_callback 里就地吞掉——一条坏记录不该带崩 IO 线程。
    template <typename Callable>
    static std::invoke_result_t<Callable> guard_callback(Callable&& callable) noexcept;
    bool can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override;
    bool uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override;
    bool gpio_digital_data_deserialized_callback(
        std::uint8_t line, const data::GpioDigitalDataView& data) override;
    bool gpio_analog_data_deserialized_callback(
        std::uint8_t line, const data::GpioAnalogDataView& data) override;
    bool gpio_read_deserialized_callback(std::uint8_t line) override;
    bool buzzer_tone_deserialized_callback(const data::BuzzerToneDataView& data) override;
    void accelerometer_deserialized_callback(const data::ImuAccelerometerDataView& data) override;
    void gyroscope_deserialized_callback(const data::ImuGyroscopeDataView& data) override;
    void temperature_deserialized_callback(const data::ImuTemperatureDataView& data) override;
    void session_control_deserialized_callback(const data::SessionControlView& data) override;
    void time_status_deserialized_callback(const data::TimeStatusView& data) override;
    void pulse_report_deserialized_callback(const data::PulseReportView& data) override;
    void port_status_deserialized_callback(
        std::uint32_t nonce, data::DataId port, const data::PortStatusVariant& status) override;
    void error_callback(core::protocol::FieldId field, data::DownlinkError reason) override;

    static constexpr auto kSessionAckTimeout = std::chrono::milliseconds{200};
    static constexpr size_t kSessionAckRetryCount = 5;
    // 保活轮次，也是时间锚的周期：必须显著短于最短的板端租约（c_board 1 s），
    // 历史与依据见 v3 handler.cpp 同名常量的注释（2026-09-14 起 250 ms）。
    static constexpr auto kSessionRefreshInterval = std::chrono::milliseconds{250};
    static constexpr size_t kSessionStartAckSize = sizeof(core::protocol::SessionHeader);

    Transport& transport_;
    data::DataCallback& sink_;
    const ThreadConfig threads_;

    core::protocol::Deserializer deserializer_;
    logging::Logger logger_;

    mutable std::mutex session_mutex_;
    std::condition_variable session_cv_;
    std::atomic<bool> session_established_{false};
    std::atomic<bool> link_restart_pending_{false};
    std::atomic<bool> wake_pending_{false};
    std::atomic<bool> stop_requested_{false};
    std::uint64_t session_start_ack_count_ = 0;
    std::uint64_t session_keepalive_ack_count_ = 0;
    std::uint32_t expected_session_nonce_ = 0;
    std::array<std::byte, kSessionStartAckSize> expected_session_start_ack_{};

    LinkState observed_link_state_ = LinkState::kUp; // 只在 IO 线程读改（notify 用）
    std::uint64_t consecutive_session_failures_ = 0; // IO 线程
    std::function<void()> before_session_;
    std::move_only_function<void() noexcept> wake_hook_;
    std::function<void(LinkState)> link_observer_;
    std::thread io_thread_;

    // ---- 端口状态账本：接收线程写，任意线程读 ----
    static constexpr std::size_t kPortStatusSlots = 16;
    [[nodiscard]] static bool port_status_slot(data::DataId port) noexcept {
        return std::to_underlying(port) < kPortStatusSlots;
    }
    void log_status_change(
        data::DataId port, const data::PortStatusVariant& previous,
        const data::PortStatusVariant& now);
    static void note_change(
        data::DataId, const data::LinkStatusView* before, const data::LinkStatusView& now,
        const logging::Logger& logger);
    static void note_change(
        data::DataId port, const data::CanStatusView* before, const data::CanStatusView& now,
        const logging::Logger& logger);
    static const char* can_state_name(std::uint8_t flags) noexcept;
    std::array<utility::Seqlock<data::PortStatusVariant>, kPortStatusSlots> port_status_{};

    // ---- 共享时间基准 ----
    std::atomic<bool> time_sync_enabled_{false};
    bool time_source_attached_ = false;
    std::shared_ptr<time::UsbFrameAxis> timeline_;
    time::BoardTimebase board_timebase_;
    time::UsbFrameAxis::Clock::time_point arrival_{};
    std::atomic<time::UsbFrameAxis::Clock::time_point> time_anchor_sent_at_{};
    std::array<std::byte, kSessionStartAckSize> session_start_ack_window_{};
    size_t session_start_ack_window_size_ = 0;
    bool awaiting_session_start_ack_ = true;

    // ---- 统计（IO 线程写，任意线程读）----
    static constexpr std::size_t kHistogramBuckets = 32;
    static void bucket_into(
        const std::array<std::atomic<std::uint64_t>, kHistogramBuckets>& buckets,
        const std::atomic<std::uint64_t>& max_ns, Histogram& out) noexcept;
    static void record_histogram_ns(
        std::array<std::atomic<std::uint64_t>, kHistogramBuckets>& buckets,
        std::atomic<std::uint64_t>& max_ns, std::uint64_t value_ns) noexcept;
    std::array<std::atomic<std::uint64_t>, kHistogramBuckets> rtt_buckets_{};
    std::atomic<std::uint64_t> rtt_max_ns_{0};
    std::atomic<std::uint64_t> session_losses_{0};
    std::atomic<std::uint64_t> uplink_stream_errors_{0};
    std::atomic<std::uint64_t> uplink_error_occurrences_{0};
};

} // namespace libhcs::host::session
