#include "host/src/session/engine.hpp"

#include <algorithm>
#include <bit>
#include <cstring>
#include <exception>
#include <format>
#include <random>
#include <stdexcept>
#include <utility>
#include <variant>

#include <pthread.h>
#include <sched.h>

#include <libhcs/protocol/vendor_control.hpp>

#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "host/src/logging/logging.hpp"
#include "host/src/session/writer.hpp"
#include "host/src/time/sample_timer.hpp"
#include "libhcs/board/hcs_config.hpp"
#include "libhcs/spec/port.hpp"

// 会话/保活/时间基准/端口状态这一层逐行继承 v3 的 Handler（host/src/protocol/
// handler.cpp，2026-10-11 v4 重写时移入）：那些行为（nonce 握手、ack 滑窗、
// 保活节律、锚点往返、状态限频日志）是量过的，不动。这里改的只有四件事：
//   1. 传输层换成公开的 libhcs::Transport（发送经 StreamWriter，不再从传输层
//      借缓冲）；
//   2. IO 线程除保活外还吃门铃（notify_io），醒来跑 wake 钩子——Board 的发送
//      提交与安全帧补发都挂在它上面，RT 线程只剩一次原子置位加 notify；
//   3. 链路状态变化经 link observer 报给 Board（掉线即补发安全帧的宿主）；
//   4. 统计（保活往返直方图、会话断开与上行错误计数）就地累积，随
//      Board::statistics() 交出去。
// PacketBuilder 与构造期 before_session 回调从公开接口里消失：前者被
// stage/send 取代，后者变成了 Board::start() 的内部步骤。

namespace {
using libhcs::core::protocol::Serializer;

// v3 同名函数照搬：kStartAck 的精确字节来自板子同源的 Serializer，手工拼头
// 曾经让每次会话开始都超时（2026-09-11）。
std::array<std::byte, sizeof(libhcs::core::protocol::SessionHeader)>
    make_session_start_ack(std::uint32_t nonce) {
    using libhcs::core::protocol::SerializeBuffer;
    using libhcs::core::protocol::SessionHeader;

    class FixedBuffer final : public SerializeBuffer {
    public:
        explicit FixedBuffer(std::span<std::byte> storage) noexcept
            : storage_(storage) {}

        std::span<std::byte> allocate(std::size_t size) noexcept override {
            if (size > storage_.size() - used_)
                return {};
            const std::span<std::byte> region = storage_.subspan(used_, size);
            used_ += size;
            return region;
        }

        [[nodiscard]] std::size_t used() const noexcept { return used_; }

    private:
        std::span<std::byte> storage_;
        std::size_t used_ = 0;
    };

    std::array<std::byte, sizeof(SessionHeader)> ack{};
    FixedBuffer buffer{ack};
    const auto result = Serializer{buffer}.write_session_control(
        {.type = libhcs::data::SessionType::kStartAck, .nonce = nonce});
    libhcs::core::utility::assert_always(
        result == Serializer::SerializeResult::kSuccess && buffer.used() == ack.size());
    return ack;
}

} // namespace

namespace {

// IO 线程上只计数 + 1st/2nd/4th... 限频的一行（HCS-USB.md §1.2）。
void note_application_callback_exception(std::string_view what) noexcept {
    static std::atomic<std::uint64_t> occurrences{0};
    const std::uint64_t count = occurrences.fetch_add(1, std::memory_order_relaxed) + 1;
    if (libhcs::host::logging::should_log_occurrence(count))
        libhcs::host::logging::get_logger().error(
            "Application callback threw (x{}): {}; the record was dropped", count, what);
}

} // namespace

namespace libhcs::host::session {

// 引擎自己就是 Ep0Channel 的实现——hcs_config.hpp 的整个声明流程（读口表、
// 发清单、读结果）跑在它上面。
static_assert(board::hcs::Ep0Channel<Engine>);

Engine::Engine(Transport& transport, data::DataCallback& sink, ThreadConfig threads)
    : transport_(transport)
    , sink_(sink)
    , threads_(threads)
    , deserializer_(*this)
    , logger_(transport.serial()) {
    // 随机 nonce：一条重启过的传输里可能还排着上一会话的字节，只有本会话的
    // nonce 能界定安全的字段边界。
    std::random_device random_device;
    std::uniform_int_distribution<std::uint32_t> distribution;
    expected_session_nonce_ = distribution(random_device);
    expected_session_start_ack_ = make_session_start_ack(expected_session_nonce_);

    // USB bulk 的完成是任意切片的一条可靠字节流，不是协议字段边界。
    transport_.receive([this](std::span<const std::byte> buffer) { receive_stream(buffer); });
    transport_.on_link_restart([this] {
        // 重启不能延续上一次的定界。回调在传输的恢复线程上，不许碰反序列化器
        // （只有接收线程可以）：只标记，下一次 receive_stream() 先复位再吃新字节。
        link_restart_pending_.store(true, std::memory_order_release);
    });
}

void Engine::start() {
    establish_session();
    observed_link_state_ = LinkState::kUp;
    consecutive_session_failures_ = 0;

    // 线程先落自己的放置再停靠：creator 随后让传输把它的核与调度复制过来
    // （v3 的停靠礼节——停靠必须是无限期等待，继承核上的定时等待会饿死，
    // 见 transport/usb 的注释）。
    auto configured = std::make_shared<std::atomic<bool>>(false);
    io_thread_ = std::thread{[this, configured] {
        apply_thread_config();
        configured->wait(false, std::memory_order_acquire);
        io_loop();
    }};
    transport_.configure_session_thread(io_thread_);
    configured->store(true, std::memory_order_release);
    configured->notify_one();
}

void Engine::stop() noexcept {
    stop_requested_.store(true, std::memory_order_relaxed);
    { const std::scoped_lock guard{session_mutex_}; }
    session_cv_.notify_all();
    if (io_thread_.joinable())
        io_thread_.join();
    stop_requested_.store(false, std::memory_order_relaxed);
    wake_pending_.store(false, std::memory_order_relaxed);
}

void Engine::apply_thread_config() const noexcept {
    // 与 v3 transport/usb 的 apply_io_thread_scheduling 同一套（那里还有失败
    // 日志；这里静默降级——放置失败不该挡住会话）。
    if (threads_.setup)
        threads_.setup();
    if (threads_.cpu < 0)
        return;
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(threads_.cpu, &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0)
        return;
    if (threads_.rt_priority <= 0)
        return;
    sched_param param{};
    param.sched_priority = threads_.rt_priority;
    (void)pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
}

LinkState Engine::link_state() const noexcept {
    if (transport_.link_faulted())
        return LinkState::kFaulted;
    return session_established() ? LinkState::kUp : LinkState::kSessionDown;
}

void Engine::note_link_state(LinkState now) noexcept {
    if (now == observed_link_state_)
        return;
    observed_link_state_ = now;
    if (link_observer_)
        link_observer_(now);
}

void Engine::io_loop() {
    using Clock = std::chrono::steady_clock;
    auto next_keepalive = Clock::now();

    while (!stop_requested_.load(std::memory_order_relaxed)) {
        {
            std::unique_lock lock{session_mutex_};
            session_cv_.wait_until(lock, next_keepalive, [this] {
                return stop_requested_.load(std::memory_order_relaxed)
                    || wake_pending_.load(std::memory_order_acquire) || !session_established();
            });
        }
        if (stop_requested_.load(std::memory_order_relaxed))
            break;

        // 门铃优先：发送提交的尾延迟不跟保活的节奏走。
        if (wake_pending_.exchange(false, std::memory_order_acq_rel)) {
            if (wake_hook_)
                wake_hook_(); // noexcept 钩子，抛不进来
        }

        const auto now = Clock::now();
        if (now >= next_keepalive || !session_established()) {
            try {
                if (session_established()) {
                    refresh_session();
                } else {
                    if (before_session_)
                        before_session_();
                    establish_session();
                }

                // 锚点跟在保活后面：只发给刚确认活着的会话。
                if (time_sync_enabled_.load(std::memory_order_relaxed) && session_established())
                    send_time_anchor();

                consecutive_session_failures_ = 0;
            } catch (const std::exception& exception) {
                handle_session_failure(exception);
            }
            next_keepalive = Clock::now() + kSessionRefreshInterval;
        }

        note_link_state(link_state());
    }
}

template <typename Callable>
std::invoke_result_t<Callable> Engine::guard_callback(Callable&& callable) noexcept {
    // 反序列化协程的 promise 把逃逸异常当致命 assert（core/src/coroutine/
    // lifo.hpp），所以异常必须在这里——库/应用边界上——就地吞掉。抛错的记录
    // 丢的是它自己，不是链路。
    using Result = std::invoke_result_t<Callable>;
    try {
        return std::forward<Callable>(callable)();
    } catch (const std::exception& exception) {
        note_application_callback_exception(exception.what());
    } catch (...) {
        note_application_callback_exception("a non-std exception");
    }
    if constexpr (std::is_same_v<Result, bool>)
        return true;
}

bool Engine::can_deserialized_callback(core::protocol::FieldId id, const data::CanDataView& data) {
    if (!session_established())
        return true;
    return guard_callback([&] {
        return sink_.can_receive_callback(
            id, data, time::time_of_stamped(data.sof_stamp, arrival_, axis()));
    });
}

bool Engine::uart_deserialized_callback(
    core::protocol::FieldId id, const data::UartDataView& data) {
    if (!session_established())
        return true;
    // 串口线没有自己的时间戳，到达时刻是唯一诚实的答案。
    return guard_callback([&] {
        return sink_.uart_receive_callback(id, data, libhcs::time::SampleTime{.host = arrival_});
    });
}

bool Engine::gpio_digital_data_deserialized_callback(
    std::uint8_t line, const data::GpioDigitalDataView& data) {
    if (!session_established())
        return true;
    return guard_callback([&] {
        std::optional<libhcs::time::BoardClock::time_point> board;
        if (data.timestamp_quarter_us)
            board = board_timebase_.extend(*data.timestamp_quarter_us);
        return sink_.gpio_digital_read_result_callback(
            line, data, time::time_of_board_ticks(board, arrival_, axis(), board_timebase_.map()));
    });
}

bool Engine::gpio_analog_data_deserialized_callback(
    std::uint8_t line, const data::GpioAnalogDataView& data) {
    if (!session_established())
        return true;
    return guard_callback(
        [&] { return sink_.gpio_analog_read_result_callback(line, data, {.host = arrival_}); });
}

// 两个下行独有记录上了上行线 = 对端糊涂了，按路由错误报，不静默（会话没开时
// 收到什么都是无主数据，不报错）。
bool Engine::gpio_read_deserialized_callback(std::uint8_t) { return !session_established(); }

bool Engine::buzzer_tone_deserialized_callback(const data::BuzzerToneDataView&) {
    return !session_established();
}

void Engine::accelerometer_deserialized_callback(const data::ImuAccelerometerDataView& data) {
    if (!session_established())
        return;
    guard_callback([&] { sink_.accelerometer_receive_callback(data, imu_sample_time(data)); });
}

void Engine::gyroscope_deserialized_callback(const data::ImuGyroscopeDataView& data) {
    if (!session_established())
        return;
    guard_callback([&] { sink_.gyroscope_receive_callback(data, imu_sample_time(data)); });
}

void Engine::temperature_deserialized_callback(const data::ImuTemperatureDataView& data) {
    if (!session_established())
        return;
    guard_callback([&] { sink_.temperature_receive_callback(data, imu_sample_time(data)); });
}

void Engine::session_control_deserialized_callback(const data::SessionControlView& data) {
    if (data.nonce != expected_session_nonce_)
        return;

    bool notify = false;
    {
        const std::scoped_lock guard{session_mutex_};
        switch (data.type) {
        case data::SessionType::kStartAck:
            session_established_.store(true, std::memory_order_release);
            ++session_start_ack_count_;
            notify = true;
            break;
        case data::SessionType::kKeepaliveAck:
            ++session_keepalive_ack_count_;
            notify = true;
            break;
        default: break;
        }
    }
    if (notify)
        session_cv_.notify_all();
}

void Engine::time_status_deserialized_callback(const data::TimeStatusView& data) {
    if (data.nonce != expected_session_nonce_)
        return;

    const auto now = time::UsbFrameAxis::Clock::now();
    const auto sent = time_anchor_sent_at_.load(std::memory_order_acquire);

    // 只有已锚定的板在共享轴上；只有应答真正锚点的报告才有往返可界定。
    if (timeline_ && data.state == data::TimeState::kValid
        && sent.time_since_epoch().count() != 0) {
        const double position = static_cast<double>(data.microframe)
                              + (static_cast<double>(data.microframe_fraction_q16) / 65536.0);
        timeline_->observe(position, sent, now);
    }

    // 无论轴怎样，这个报告都是"板钟 ↔ 轴"的又一次配对。
    board_timebase_.observe(data);

    guard_callback([&] { sink_.time_status_callback(data); });
}

void Engine::pulse_report_deserialized_callback(const data::PulseReportView& data) {
    if (data.nonce != expected_session_nonce_)
        return;
    guard_callback([&] { sink_.pulse_report_callback(data); });
}

void Engine::port_status_deserialized_callback(
    std::uint32_t nonce, data::DataId port, const data::PortStatusVariant& status) {
    if (nonce != expected_session_nonce_ || !port_status_slot(port))
        return;
    auto& slot = port_status_[std::to_underlying(port)];
    const data::PortStatusVariant previous = slot.load();
    slot.store(status);
    log_status_change(port, previous, status);
}

data::PortStatusVariant Engine::port_status(data::DataId port) const noexcept {
    return port_status_slot(port) ? port_status_[std::to_underlying(port)].load()
                                  : data::PortStatusVariant{};
}

std::optional<libhcs::time::BoardClock::time_point>
    Engine::board_time_at(libhcs::time::HostTime when) const {
    if (!timeline_)
        return std::nullopt;
    const time::Affine<libhcs::time::BoardClock, time::HostClock> board_to_host =
        time::compose(timeline_->map(), board_timebase_.map());
    if (!board_to_host.valid())
        return std::nullopt;
    return board_to_host.inverse()(when);
}

void Engine::log_status_change(
    data::DataId port, const data::PortStatusVariant& previous,
    const data::PortStatusVariant& now) {
    std::visit(
        [&]<typename View>(const View& current) {
            if constexpr (requires {
                              note_change(port, std::get_if<View>(&previous), current, logger_);
                          })
                note_change(port, std::get_if<View>(&previous), current, logger_);
        },
        now);
}

void Engine::note_change(
    data::DataId, const data::LinkStatusView* before, const data::LinkStatusView& now,
    const logging::Logger& logger) {
    const bool moved = before != nullptr ? now.downlink_errors != before->downlink_errors
                                         : now.downlink_errors != 0;
    if (!moved)
        return;
    logger.error(
        "Board reports {} downlink record error(s) since boot; last: {} in downlink transfer "
        "{} of this session, {}",
        now.downlink_errors, spec::channel_name(now.last_field), now.last_transfer,
        data::downlink_error_name(now.last_reason));
}

void Engine::note_change(
    data::DataId port, const data::CanStatusView* before, const data::CanStatusView& now,
    const logging::Logger& logger) {
    const std::uint8_t previous_flags = before != nullptr ? before->flags : 0;
    if (now.flags == previous_flags)
        return;
    logger.warn(
        "{}: bus state {} -> {} (tec={} rec={} last={})", spec::channel_name(port),
        can_state_name(previous_flags), can_state_name(now.flags), static_cast<unsigned>(now.tec),
        static_cast<unsigned>(now.rec), data::last_error_name(now.last_error));
}

const char* Engine::can_state_name(std::uint8_t flags) noexcept {
    if ((flags & data::kCanBusOff) != 0U)
        return "bus-off";
    if ((flags & data::kCanErrorPassive) != 0U)
        return "error-passive";
    if ((flags & data::kCanWarning) != 0U)
        return "warning";
    return "error-active";
}

void Engine::error_callback(const core::protocol::FieldId field, const data::DownlinkError reason) {
    // 计数永远走（统计里看得到），日志按 1st/2nd/4th... 限频。
    uplink_stream_errors_.fetch_add(1, std::memory_order_relaxed);
    const std::uint64_t count =
        uplink_error_occurrences_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (!logging::should_log_occurrence(count))
        return;
    logger_.error(
        "Uplink stream error at field {} ({}): {}", static_cast<int>(field),
        spec::channel_name(field), data::downlink_error_name(reason));
}

// ---- 会话 ----

void Engine::reset_link_state() {
    deserializer_.finish_transfer();
    awaiting_session_start_ack_ = true;
    session_start_ack_window_size_ = 0;
    {
        // 状态变化与条件变量互斥量配对，保活线程不会错过重启通知。
        const std::scoped_lock guard{session_mutex_};
        session_established_.store(false, std::memory_order_release);
    }
    session_cv_.notify_all();
}

void Engine::receive_stream(std::span<const std::byte> buffer) {
    // 一次传输一个时钟读，不是一条记录一个：这段缓冲里解码出的每个 SampleTime
    // 共享它（到达时间的回退值，CAN 戳回绕展开的参考）。
    arrival_ = time::UsbFrameAxis::Clock::now();

    // 延迟消费的链路重启在这里落地：本函数只在接收线程上跑，这就是它不需要
    // 数据路径上的锁的原因。
    if (link_restart_pending_.load(std::memory_order_acquire)) [[unlikely]] {
        while (link_restart_pending_.exchange(false, std::memory_order_acq_rel))
            reset_link_state();
    }

    if (!awaiting_session_start_ack_) {
        deserializer_.feed(buffer);
        return;
    }

    for (size_t i = 0; i < buffer.size(); ++i) {
        if (session_start_ack_window_size_ < session_start_ack_window_.size()) {
            session_start_ack_window_[session_start_ack_window_size_++] = buffer[i];
        } else {
            std::ranges::move(
                session_start_ack_window_.begin() + 1, session_start_ack_window_.end(),
                session_start_ack_window_.begin());
            session_start_ack_window_.back() = buffer[i];
        }

        if (session_start_ack_window_size_ != session_start_ack_window_.size()
            || session_start_ack_window_ != expected_session_start_ack_) {
            continue;
        }

        awaiting_session_start_ack_ = false;
        deserializer_.feed(session_start_ack_window_);
        deserializer_.feed(buffer.subspan(i + 1));
        return;
    }
}

void Engine::establish_session() {
    for (size_t attempt = 0; attempt < kSessionAckRetryCount; ++attempt) {
        std::uint64_t previous_session_start_ack_count = 0;
        {
            const std::scoped_lock guard{session_mutex_};
            previous_session_start_ack_count = session_start_ack_count_;
        }

        {
            StreamWriter writer{transport_};
            const auto result = core::protocol::Serializer{writer}.write_session_control(
                {.type = data::SessionType::kStart, .nonce = expected_session_nonce_});
            core::utility::assert_debug(
                result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            if (result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]]
                throw std::runtime_error{
                    "Failed to transmit Session Start: transmit buffer unavailable"};
        }

        std::unique_lock lock{session_mutex_};
        if (session_cv_.wait_for(
                lock, kSessionAckTimeout, [this, previous_session_start_ack_count] {
                    return stop_requested_.load(std::memory_order_relaxed)
                        || session_start_ack_count_ > previous_session_start_ack_count;
                })) {
            // 新的 kStart 重启板子自己的时间拟合：扩展基与板钟映射随之重来。
            board_timebase_.reset();
            return;
        }
    }

    throw std::runtime_error{"Timed out waiting for SESSION_ACK"};
}

void Engine::refresh_session() {
    for (size_t attempt = 0; attempt < kSessionAckRetryCount; ++attempt) {
        if (!session_established())
            return;

        std::uint64_t previous_session_keepalive_ack_count = 0;
        {
            const std::scoped_lock guard{session_mutex_};
            previous_session_keepalive_ack_count = session_keepalive_ack_count_;
        }

        const auto sent_at = std::chrono::steady_clock::now();
        {
            StreamWriter writer{transport_};
            const auto result = core::protocol::Serializer{writer}.write_session_control(
                {.type = data::SessionType::kKeepalive, .nonce = expected_session_nonce_});
            core::utility::assert_debug(
                result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            if (result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]]
                throw std::runtime_error{
                    "Failed to transmit Session Keepalive: transmit buffer unavailable"};
        }

        std::unique_lock lock{session_mutex_};
        if (session_cv_.wait_for(
                lock, kSessionAckTimeout, [this, previous_session_keepalive_ack_count] {
                    return stop_requested_.load(std::memory_order_relaxed) || !session_established()
                        || session_keepalive_ack_count_ > previous_session_keepalive_ack_count;
                })) {
            if (session_keepalive_ack_count_ > previous_session_keepalive_ack_count)
                record_histogram_ns(
                    rtt_buckets_, rtt_max_ns_,
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - sent_at)
                        .count());
            return;
        }
    }

    throw std::runtime_error{"Timed out waiting for SESSION_KEEPALIVE_ACK"};
}

void Engine::send_pulse_schedule(std::uint64_t microframe) {
    StreamWriter writer{transport_};
    (void)core::protocol::Serializer{writer}.write_pulse_schedule(
        {.nonce = expected_session_nonce_, .microframe = microframe});
}

bool Engine::vendor_control(
    std::uint8_t request_type, std::uint8_t request, std::uint16_t index,
    std::span<std::byte> payload) {
    switch (transport_.vendor_control(request_type, request, index, payload)) {
    case Transport::ControlResult::kOk: return true;
    case Transport::ControlResult::kStalled: return false;
    case Transport::ControlResult::kUnsupported:
        throw std::runtime_error{
            "This transport has no control endpoint: channel configuration over EP0 is "
            "available on USB only."};
    case Transport::ControlResult::kFailed:
    default:
        throw std::runtime_error(
            std::string{"EP0 vendor request 0x"} + std::format("{:02x}", request)
            + " failed; see the transport log for details.");
    }
}

bool Engine::vendor_control_out(
    std::uint8_t request, std::uint16_t index, const void* payload, std::size_t size) {
    // 写请求也过一道 scratch 缓冲：公开 Transport 的签名是可变 span，不把调用
    // 方缓冲区的 const 交出去。
    std::array<std::byte, kVendorControlPayloadMax> scratch{};
    if (size > scratch.size())
        throw std::invalid_argument{"EP0 payload too large"};
    if (size != 0)
        std::memcpy(scratch.data(), payload, size);
    return vendor_control(
        core::protocol::vendor_control::kRequestTypeOut, request, index,
        std::span<std::byte>{scratch.data(), size});
}

bool Engine::vendor_control_in(
    std::uint8_t request, std::uint16_t index, void* payload, std::size_t size) {
    std::array<std::byte, kVendorControlPayloadMax> scratch{};
    if (size > scratch.size())
        throw std::invalid_argument{"EP0 payload too large"};
    const bool ok = vendor_control(
        core::protocol::vendor_control::kRequestTypeIn, request, index,
        std::span<std::byte>{scratch.data(), size});
    if (ok && size != 0)
        std::memcpy(payload, scratch.data(), size);
    return ok;
}

// ---- 时间基准 ----

void Engine::attach_time_source() {
    const int bus = transport_.usb_bus_number();
    timeline_ = time::UsbFrameAxis::for_usb_bus(bus);
    if (timeline_->controller_status()) {
        logger_.info("Time sync: reading the microframe counter of USB bus {}", bus);
    } else {
        logger_.warn(
            "Time sync: the host controller's microframe counter on USB bus {} cannot be "
            "read; timestamps fall back to the USB round-trip fit",
            bus);
    }
}

void Engine::set_time_sync(bool on) {
    if (on && !time_source_attached_) {
        attach_time_source();
        time_source_attached_ = true;
    }
    time_sync_enabled_.store(on, std::memory_order_relaxed);
}

time::Affine<time::FrameClock, time::HostClock> Engine::axis() const noexcept {
    return timeline_ ? timeline_->map() : time::Affine<time::FrameClock, time::HostClock>{};
}

template <typename View>
libhcs::time::SampleTime Engine::imu_sample_time(const View& data) noexcept {
    return time::time_of_board_ticks(
        board_timebase_.extend(data.timestamp_quarter_us), arrival_, axis(), board_timebase_.map());
}

void Engine::send_time_anchor() {
    const auto now = time::UsbFrameAxis::Clock::now();
    if (!timeline_)
        return;
    const std::uint64_t microframe = timeline_->anchor_for(now);

    core::protocol::Serializer::SerializeResult result;
    {
        StreamWriter writer{transport_};
        core::protocol::Serializer serializer{writer};
        result = serializer.write_time_anchor(
            {.nonce = expected_session_nonce_, .microframe = microframe});
    }
    core::utility::assert_debug(
        result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    // 与保活不同，锚点丢了不致命：损失一段收敛时间，不是会话。
    if (result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]] {
        logger_.error("Failed to transmit Time Anchor: transmit buffer full");
        return;
    }
    // 只在锚点真的发出之后记录时刻：应答第 N 轮的 kTimeStatus 可能在第 N+1 轮
    // 失败之后才到，拿失败轮的时间戳配对会错位一整个周期。
    time_anchor_sent_at_.store(now, std::memory_order_release);
}

// ---- 故障路径 ----

void Engine::handle_session_failure(const std::exception& exception) noexcept try {
    // 正在停机：现在的失败是拆收，不是要修的故障。try_recover_link() 会重开
    // 设备并等它重新出现，最长 2 s，~Engine 的 join() 不该陪着等。
    if (stop_requested_.load(std::memory_order_relaxed))
        return;

    // 一次新的掉线才计数（保活重试期间的重复失败不算）。
    const bool was_established = session_established();
    if (was_established)
        session_losses_.fetch_add(1, std::memory_order_relaxed);

    // refresh_session() 放弃时不清标志，这里清：下一趟必须重开会话，而不是继续
    // 保活一个板子已经忘掉的会话。
    {
        const std::scoped_lock guard{session_mutex_};
        session_established_.store(false, std::memory_order_release);
    }

    ++consecutive_session_failures_;

    const bool recovered = transport_.try_recover_link();

    // 1st/2nd/4th/8th... 限频：一块彻底拔掉的板子每次都失败，不能淹没日志。
    if (logging::should_log_occurrence(consecutive_session_failures_))
        logger_.error(
            "Failed to refresh session ({} in a row): {}. Link recovery {}; retrying.",
            consecutive_session_failures_, exception.what(),
            recovered ? "attempted" : "not available");

    // 重试配速：会话断着的时候循环的等待条件立即为真，没有这段一个死板会把
    // 这个线程转满速。
    std::unique_lock lock{session_mutex_};
    (void)session_cv_.wait_for(lock, kSessionRefreshInterval, [this] {
        return stop_requested_.load(std::memory_order_relaxed);
    });
} catch (const std::exception& nested) {
    // 函数 try 块：这条路径本身在线程的 catch 里，逃出去就是 terminate。
    logger_.error("While handling a session failure: {}", nested.what());
}

// ---- 统计 ----

void Engine::record_histogram_ns(
    std::array<std::atomic<std::uint64_t>, kHistogramBuckets>& buckets,
    std::atomic<std::uint64_t>& max_ns, std::uint64_t value_ns) noexcept {
    const std::uint64_t bucket =
        value_ns == 0 ? 0 : std::min<std::uint64_t>(63 - std::countl_zero(value_ns), 31);
    buckets[bucket].fetch_add(1, std::memory_order_relaxed);
    std::uint64_t known = max_ns.load(std::memory_order_relaxed);
    while (value_ns > known
           && !max_ns.compare_exchange_weak(
               known, value_ns, std::memory_order_relaxed, std::memory_order_relaxed)) {}
}

void Engine::bucket_into(
    const std::array<std::atomic<std::uint64_t>, kHistogramBuckets>& buckets,
    const std::atomic<std::uint64_t>& max_ns, Histogram& out) noexcept {
    for (std::size_t i = 0; i < kHistogramBuckets; ++i)
        out.buckets[i] = buckets[i].load(std::memory_order_relaxed);
    out.max_ns = max_ns.load(std::memory_order_relaxed);
}

void Engine::keepalive_rtt_into(Histogram& out) const noexcept {
    bucket_into(rtt_buckets_, rtt_max_ns_, out);
}

} // namespace libhcs::host::session
