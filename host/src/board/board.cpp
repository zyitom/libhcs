#include <algorithm>
#include <cstring>
#include <format>
#include <memory>
#include <stdexcept>
#include <utility>

#include "core/src/protocol/serializer.hpp"
#include "host/src/board/board_impl.hpp"
#include "host/src/session/writer.hpp"
#include "host/src/transport/usb/usb_transport.hpp"
#include "libhcs/spec/port.hpp"

// v4 的 Board（HCS-USB.md §2）：外观模式——使用者只看见一块板和它的通道，看不
// 见传输、会话引擎和 IO 线程。这里实现的是生命周期、拉取式通道的搬运、发送门
// （Armed / request_safe 只放行安全帧）与统计聚合；EP0 声明、会话、时间基准都
// 在已有的件里（hcs_config.hpp 的 apply、session/engine.cpp）。
//
// 通道壳（CanChannel…）只被 Board 与通道自己的方法碰；BoardCore 的内部路径
// （IO 线程、统计）走 can_impls_/uart_impls_ 里的存储指针。

namespace libhcs {

using host::board::BoardCore;
using host::board::CanChannelImpl;
using host::board::ImuChannelImpl;
using host::board::UartChannelImpl;

// ---- 公开通道类：内部存储的薄壳 ----

struct CanChannel::Impl final : CanChannelImpl {
    using CanChannelImpl::CanChannelImpl;
};
struct UartChannel::Impl final : UartChannelImpl {
    using UartChannelImpl::UartChannelImpl;
};
struct ImuChannel::Impl final : ImuChannelImpl {
    using ImuChannelImpl::ImuChannelImpl;
};

CanChannel::CanChannel(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}

CanChannel::~CanChannel() = default;

std::string_view CanChannel::name() const noexcept { return impl_->name; }

std::span<const CanFrame> CanChannel::frames() const noexcept {
    return {impl_->tick_.data(), impl_->tick_count_};
}

bool CanChannel::stage(const CanFrame& frame) noexcept {
    CanChannelImpl& d = *impl_;
    // 帧型合法性在声明阶段定（fd 与板子报的长帧能力合成 long_frames_ok_，
    // start() 时随状态发布）；这里只剩位测试与容量检查，满了计数不抛。
    if (frame.size > sizeof(frame.data) || (frame.size > 8 && !d.long_frames_ok_)
        || d.owner_->state_.load(std::memory_order_acquire) == BoardState::kIdle
        || d.staged_ >= d.staging_.size()) [[unlikely]] {
        d.tx_dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    d.staging_[d.staged_++] = frame;
    return true;
}

void CanChannel::set_safe_frames(std::span<const CanFrame> frames) {
    CanChannelImpl& d = *impl_;
    if (d.owner_->state_.load(std::memory_order_acquire) != BoardState::kIdle)
        throw std::logic_error{
            std::format("{}: safe frames can only be declared before start()", d.name)};
    if (frames.size() > CanChannelImpl::kMaxSafeFrames)
        throw std::invalid_argument{std::format(
            "{}: at most {} safe frames, {} given", d.name, CanChannelImpl::kMaxSafeFrames,
            frames.size())};
    for (const auto& frame : frames) {
        if (frame.size > 8 && !d.setting.fd)
            throw std::invalid_argument{std::format(
                "{}: a safe frame carries {} bytes but the bus is declared classic CAN", d.name,
                frame.size)};
        if (frame.size > sizeof(frame.data))
            throw std::invalid_argument{
                std::format("{}: a safe frame carries more than 64 bytes", d.name)};
    }
    for (std::size_t i = 0; i < frames.size(); ++i)
        d.safe_frames_[i] = frames[i];
    d.safe_count_ = static_cast<std::uint32_t>(frames.size());
}

CanChannelStatus CanChannel::status() const noexcept {
    const CanChannelImpl& d = *impl_;
    CanChannelStatus out;
    out.counters = {
        .rx = d.rx_.load(std::memory_order_relaxed),
        .rx_dropped = d.ring_.dropped(),
        .tx = d.tx_.load(std::memory_order_relaxed),
        .tx_dropped = d.tx_dropped_.load(std::memory_order_relaxed),
        .safe_sends = d.safe_sends_.load(std::memory_order_relaxed),
    };
    if (d.owner_->engine_ != nullptr) {
        const auto any = d.owner_->engine_->port_status(d.data_id);
        if (const auto* view = std::get_if<data::CanStatusView>(&any))
            out.board = *view;
    }
    return out;
}

UartChannel::UartChannel(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}

UartChannel::~UartChannel() = default;

std::string_view UartChannel::name() const noexcept { return impl_->name; }

std::span<const UartChunk> UartChannel::chunks() const noexcept {
    return {impl_->tick_.data(), impl_->tick_count_};
}

bool UartChannel::stage(std::span<const std::byte> bytes, bool idle_delimited) noexcept {
    UartChannelImpl& d = *impl_;
    if (bytes.size() > kUartChunkBytes
        || d.owner_->state_.load(std::memory_order_acquire) == BoardState::kIdle
        || d.staged_ >= d.staging_.size()) [[unlikely]] {
        d.tx_dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    UartChunk& slot = d.staging_[d.staged_++];
    slot = {};
    slot.size = static_cast<std::uint16_t>(bytes.size());
    slot.idle_delimited = idle_delimited;
    std::memcpy(slot.data.data(), bytes.data(), bytes.size());
    return true;
}

void UartChannel::set_safe_bytes(std::span<const std::byte> bytes, bool idle_delimited) {
    UartChannelImpl& d = *impl_;
    if (d.owner_->state_.load(std::memory_order_acquire) != BoardState::kIdle)
        throw std::logic_error{
            std::format("{}: safe bytes can only be declared before start()", d.name)};
    if (bytes.size() > kUartChunkBytes)
        throw std::invalid_argument{std::format(
            "{}: safe bytes are at most {} bytes, {} given", d.name, kUartChunkBytes,
            bytes.size())};
    d.safe_chunk_ = {};
    d.safe_chunk_.size = static_cast<std::uint16_t>(bytes.size());
    d.safe_chunk_.idle_delimited = idle_delimited;
    std::memcpy(d.safe_chunk_.data.data(), bytes.data(), bytes.size());
}

UartChannelStatus UartChannel::status() const noexcept {
    const UartChannelImpl& d = *impl_;
    UartChannelStatus out;
    out.counters = {
        .rx = d.rx_.load(std::memory_order_relaxed),
        .rx_dropped = d.ring_.dropped(),
        .tx = d.tx_.load(std::memory_order_relaxed),
        .tx_dropped = d.tx_dropped_.load(std::memory_order_relaxed),
        .safe_sends = d.safe_sends_.load(std::memory_order_relaxed),
    };
    if (d.owner_->engine_ != nullptr) {
        const auto any = d.owner_->engine_->port_status(d.data_id);
        if (const auto* view = std::get_if<data::UartStatusView>(&any))
            out.board = *view;
    }
    return out;
}

ImuChannel::ImuChannel(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}

ImuChannel::~ImuChannel() = default;

// NOLINTNEXTLINE(readability-convert-member-functions-to-static): 三个通道类的
// name() 是同一个接口形状，IMU 的名字是常量不改变这一点。
std::string_view ImuChannel::name() const noexcept { return ImuChannelImpl::kName; }

ImuData ImuChannel::samples() const noexcept {
    ImuChannelImpl& d = *impl_;
    return {
        .accelerometer = {d.accelerometer_.data(), d.accelerometer_count_},
        .gyroscope = {    d.gyroscope_.data(),     d.gyroscope_count_},
        .temperature = {  d.temperature_.data(),   d.temperature_count_},
    };
}

ImuChannelStatus ImuChannel::status() const noexcept {
    ImuChannelStatus out;
    out.counters.rx = impl_->rx_.load(std::memory_order_relaxed);
    return out;
}

// ---- BoardCore：生命周期与数据搬运 ----

namespace {

using core::protocol::Serializer;

// CanFrame → 线格式视图（下行方向，不带戳）。
data::CanDataView view_of(const CanFrame& frame) noexcept {
    return {
        .can_id = frame.id,
        .can_data = {frame.data.data(), frame.size},
        .is_extended_can_id = frame.extended,
        .is_remote_transmission = frame.remote,
    };
}

} // namespace

std::string BoardCore::port_list_text(spec::PortKind kind) const {
    std::string text;
    for (const auto& port : type_->ports) {
        if (port.kind != kind)
            continue;
        if (!text.empty())
            text += ", ";
        text += spec::channel_name(port.data_id);
    }
    return text.empty() ? "(none)" : text;
}

void BoardCore::start() {
    if (state_.load(std::memory_order_acquire) != BoardState::kIdle)
        throw std::logic_error{"Board::start(): the board is already started (stop() first)"};

    if (transport_ == nullptr) {
        // 默认 libusb 传输：扫描、打开、产品串版本核对都在它的构造里完成，
        // 找不到板或版本不符在这里抛——这就是"打开"这一步。
        transport_ = std::make_unique<host::transport::usb::UsbTransport>(
            type_->vendor_id, type_->product_ids, options_.serial, options_.io_thread_cpu,
            options_.io_thread_rt_priority, options_.thread_setup,
            options_.dangerously_skip_version_checks);
    }
    if (engine_ == nullptr) {
        engine_ = std::make_unique<session::Engine>(
            *transport_, *this,
            session::Engine::ThreadConfig{
                .cpu = options_.io_thread_cpu,
                .rt_priority = options_.io_thread_rt_priority,
                .setup = options_.thread_setup,
            });
        engine_->set_wake_hook([this]() noexcept { on_io_wake(); });
        engine_->set_link_observer([this](LinkState state) { on_link_state(state); });
    }

    // EP0 声明：读口表（版本核对）、验证接线、两阶段提交、读回核对——全在
    // hcs_config.hpp 的 apply() 里，失败抛出并带差异。这就是"配置"这一步。
    {
        const std::scoped_lock guard{configure_mutex_};
        configuration_.time_sync = options_.time_sync;
        interface_.store(cfg::apply(*engine_, configuration_), std::memory_order_release);
        engine_->set_time_sync(configuration_.time_sync);
    }
    // 板子在会话结束时会忘掉声明：重连钩子重放同一份清单（v3 的机制照搬）。
    engine_->set_before_session([this] {
        const std::scoped_lock guard{configure_mutex_};
        interface_.store(cfg::apply(*engine_, configuration_), std::memory_order_release);
        engine_->set_time_sync(configuration_.time_sync);
    });

    engine_->start();

    // 长帧门：声明是 FD 且板子报这个口承载长帧（构造握手学到的快照）。
    const cfg::Interface interface = interface_.load(std::memory_order_relaxed);
    for (CanChannelImpl* d : can_impls_)
        d->long_frames_ok_ = interface.can_fd(d->data_id) && interface.long_frames(d->data_id);

    state_.store(BoardState::kArmed, std::memory_order_release);
}

void BoardCore::stop() noexcept {
    if (engine_ != nullptr)
        engine_->stop();
    state_.store(BoardState::kIdle, std::memory_order_release);
    safe_latched_ = false;
    // 拍内与待发的数据全部作废：stop 之后的世界从零开始。
    for (CanChannelImpl* d : can_impls_) {
        d->staged_ = 0;
        d->tick_count_ = 0;
        d->ring_.consume_all();
        d->tx_ring_.consume_all();
    }
    for (UartChannelImpl* d : uart_impls_) {
        d->staged_ = 0;
        d->tick_count_ = 0;
        d->ring_.consume_all();
        d->tx_ring_.consume_all();
    }
    if (imu_impl_ != nullptr) {
        imu_impl_->accelerometer_count_ = imu_impl_->gyroscope_count_ =
            imu_impl_->temperature_count_ = 0;
        imu_impl_->accelerometer_ring_.consume_all();
        imu_impl_->gyroscope_ring_.consume_all();
        imu_impl_->temperature_ring_.consume_all();
    }
}

void BoardCore::run() {
    BoardState expected = BoardState::kArmed;
    if (!state_.compare_exchange_strong(expected, BoardState::kRunning))
        throw std::logic_error{"Board::run(): only legal from Armed (start() first)"};
}

void BoardCore::arm() {
    BoardState expected = BoardState::kRunning;
    if (!state_.compare_exchange_strong(expected, BoardState::kArmed))
        throw std::logic_error{"Board::arm(): only legal from Running"};
}

void BoardCore::request_safe(bool safe) noexcept {
    safe_requested_.store(safe, std::memory_order_release);
    // 叫醒 IO 线程处理边沿：请求安全态时立刻补发一轮安全帧，不等下一拍。
    if (engine_ != nullptr)
        engine_->notify_io();
}

void BoardCore::receive() noexcept {
    receives_.fetch_add(1, std::memory_order_relaxed);
    for (CanChannelImpl* d : can_impls_) {
        d->tick_count_ = 0;
        d->ring_.drain([d](const CanFrame& frame) noexcept { d->tick_[d->tick_count_++] = frame; });
    }
    for (UartChannelImpl* d : uart_impls_) {
        d->tick_count_ = 0;
        d->ring_.drain(
            [d](const UartChunk& chunk) noexcept { d->tick_[d->tick_count_++] = chunk; });
    }
    if (imu_impl_ != nullptr) {
        ImuChannelImpl* d = imu_impl_;
        d->accelerometer_count_ = 0;
        d->accelerometer_ring_.drain([d](const ImuAccelerometer& sample) noexcept {
            d->accelerometer_[d->accelerometer_count_++] = sample;
        });
        d->gyroscope_count_ = 0;
        d->gyroscope_ring_.drain([d](const ImuGyroscope& sample) noexcept {
            d->gyroscope_[d->gyroscope_count_++] = sample;
        });
        d->temperature_count_ = 0;
        d->temperature_ring_.drain([d](const ImuTemperature& sample) noexcept {
            d->temperature_[d->temperature_count_++] = sample;
        });
    }
}

void BoardCore::send() noexcept {
    sends_.fetch_add(1, std::memory_order_relaxed);

    const BoardState state = state_.load(std::memory_order_acquire);
    const bool link_up = engine_ != nullptr && engine_->link_state() == LinkState::kUp;
    // 发送门：Running 且未请求安全态才放暂存内容；否则只放安全帧。
    const bool gated =
        state != BoardState::kRunning || safe_requested_.load(std::memory_order_acquire);

    if (state == BoardState::kIdle || !link_up) {
        // 没在跑，或链路断着（断开瞬间 IO 线程已补发过安全帧）：暂存作废。
        for (CanChannelImpl* d : can_impls_) {
            d->tx_dropped_.fetch_add(d->staged_, std::memory_order_relaxed);
            d->staged_ = 0;
        }
        for (UartChannelImpl* d : uart_impls_) {
            d->tx_dropped_.fetch_add(d->staged_, std::memory_order_relaxed);
            d->staged_ = 0;
        }
        return;
    }

    for (CanChannelImpl* d : can_impls_) {
        if (gated) {
            d->tx_dropped_.fetch_add(d->staged_, std::memory_order_relaxed);
            d->staged_ = 0;
            for (std::uint32_t i = 0; i < d->safe_count_; ++i) {
                d->tx_ring_.push(d->safe_frames_[i]);
                d->safe_sends_.fetch_add(1, std::memory_order_relaxed);
            }
        } else {
            for (std::uint32_t i = 0; i < d->staged_; ++i)
                d->tx_ring_.push(d->staging_[i]);
            d->tx_.fetch_add(d->staged_, std::memory_order_relaxed);
            d->staged_ = 0;
        }
    }
    for (UartChannelImpl* d : uart_impls_) {
        if (gated) {
            d->tx_dropped_.fetch_add(d->staged_, std::memory_order_relaxed);
            d->staged_ = 0;
            if (d->safe_chunk_.size != 0) {
                d->tx_ring_.push(d->safe_chunk_);
                d->safe_sends_.fetch_add(1, std::memory_order_relaxed);
            }
        } else {
            std::uint64_t bytes = 0;
            for (std::uint32_t i = 0; i < d->staged_; ++i) {
                d->tx_ring_.push(d->staging_[i]);
                bytes += d->staging_[i].size;
            }
            d->tx_.fetch_add(d->staged_, std::memory_order_relaxed);
            d->tx_bytes_.fetch_add(bytes, std::memory_order_relaxed);
            d->staged_ = 0;
        }
    }

    engine_->notify_io();
}

Statistics BoardCore::statistics() const noexcept {
    Statistics out;
    out.receives = receives_.load(std::memory_order_relaxed);
    out.sends = sends_.load(std::memory_order_relaxed);
    for (const CanChannelImpl* d : can_impls_) {
        out.rx_frames += d->rx_.load(std::memory_order_relaxed);
        out.rx_dropped += d->ring_.dropped();
        out.tx_frames += d->tx_.load(std::memory_order_relaxed);
        out.tx_dropped += d->tx_dropped_.load(std::memory_order_relaxed);
    }
    for (const UartChannelImpl* d : uart_impls_) {
        out.rx_bytes += d->rx_bytes_.load(std::memory_order_relaxed);
        out.rx_dropped += d->ring_.dropped();
        out.tx_bytes += d->tx_bytes_.load(std::memory_order_relaxed);
        out.tx_dropped += d->tx_dropped_.load(std::memory_order_relaxed);
    }
    if (imu_impl_ != nullptr)
        out.rx_frames += imu_impl_->rx_.load(std::memory_order_relaxed);
    if (engine_ != nullptr) {
        out.session_losses = engine_->session_losses();
        out.uplink_stream_errors = engine_->uplink_stream_errors();
        engine_->keepalive_rtt_into(out.keepalive_rtt);
    }
    return out;
}

// ---- IO 线程：发送提交、安全帧、上行入环 ----

void BoardCore::on_io_wake() noexcept {
    // 安全请求的边沿：置位时立即补发一轮（不占发送环——那归使用者线程生产）。
    const bool requested = safe_requested_.load(std::memory_order_acquire);
    if (requested != safe_latched_) {
        safe_latched_ = requested;
        if (requested)
            send_safe_frames_now();
    }

    // 一拍的所有端口打进尽量少的包：一个 StreamWriter，写满自动切包。
    session::StreamWriter writer{*transport_};
    Serializer serializer{writer};
    for (CanChannelImpl* d : can_impls_) {
        d->tx_ring_.drain([&](const CanFrame& frame) noexcept {
            if (serializer.write_can(d->data_id, view_of(frame))
                != Serializer::SerializeResult::kSuccess)
                d->tx_dropped_.fetch_add(1, std::memory_order_relaxed);
        });
    }
    for (UartChannelImpl* d : uart_impls_) {
        d->tx_ring_.drain([&](const UartChunk& chunk) noexcept {
            const data::UartDataView view{
                .uart_data = {chunk.data.data(), chunk.size},
                .idle_delimited = chunk.idle_delimited,
            };
            if (serializer.write_uart(d->data_id, view) != Serializer::SerializeResult::kSuccess)
                d->tx_dropped_.fetch_add(1, std::memory_order_relaxed);
        });
    }
}

void BoardCore::send_safe_frames_now() noexcept {
    session::StreamWriter writer{*transport_};
    Serializer serializer{writer};
    for (CanChannelImpl* d : can_impls_) {
        for (std::uint32_t i = 0; i < d->safe_count_; ++i) {
            (void)serializer.write_can(d->data_id, view_of(d->safe_frames_[i]));
            d->safe_sends_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    for (UartChannelImpl* d : uart_impls_) {
        if (d->safe_chunk_.size == 0)
            continue;
        const data::UartDataView view{
            .uart_data = {d->safe_chunk_.data.data(), d->safe_chunk_.size},
            .idle_delimited = d->safe_chunk_.idle_delimited,
        };
        (void)serializer.write_uart(d->data_id, view);
        d->safe_sends_.fetch_add(1, std::memory_order_relaxed);
    }
}

void BoardCore::on_link_state(LinkState state) noexcept {
    if (state == LinkState::kUp)
        return;
    // 掉线：先补发一轮安全帧——传输还活着的话，还在转发的板子（c_board 型）
    // 会把它发上线；板上看门狗（U3）才是这条路径的正解，这是它的前置兜底。
    send_safe_frames_now();
    // 发送环里的内容在恢复后就是陈旧指令了：清掉并计数。
    for (CanChannelImpl* d : can_impls_)
        d->tx_ring_.drain([d](const CanFrame&) noexcept {
            d->tx_dropped_.fetch_add(1, std::memory_order_relaxed);
        });
    for (UartChannelImpl* d : uart_impls_)
        d->tx_ring_.drain([d](const UartChunk&) noexcept {
            d->tx_dropped_.fetch_add(1, std::memory_order_relaxed);
        });
}

bool BoardCore::can_receive_callback(
    data::DataId id, const data::CanDataView& data, const libhcs::time::SampleTime& time) noexcept {
    if (state_.load(std::memory_order_acquire) == BoardState::kIdle)
        return true; // 停机后板子还可能发来残余数据：静默丢，不算谁的错
    const std::size_t slot = std::to_underlying(id);
    if (slot >= kDataIdSlots || can_by_id_[slot] == nullptr)
        return false; // 板子在一个没声明的口上发数据：按路由错误报，不吞
    CanChannelImpl& d = *can_by_id_[slot];
    CanFrame frame;
    frame.id = data.can_id;
    frame.extended = data.is_extended_can_id;
    frame.remote = data.is_remote_transmission;
    frame.size = static_cast<std::uint8_t>(std::min(data.can_data.size(), sizeof(frame.data)));
    std::memcpy(frame.data.data(), data.can_data.data(), frame.size);
    frame.time = time;
    d.ring_.push(frame);
    d.rx_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

bool BoardCore::uart_receive_callback(
    data::DataId id, const data::UartDataView& data,
    const libhcs::time::SampleTime& time) noexcept {
    if (state_.load(std::memory_order_acquire) == BoardState::kIdle)
        return true;
    const std::size_t slot = std::to_underlying(id);
    if (slot >= kDataIdSlots || uart_by_id_[slot] == nullptr)
        return false;
    UartChannelImpl& d = *uart_by_id_[slot];
    // 上行记录最长 1023 字节，块容量 512：切开放进环，空闲定界标在最后一块。
    std::size_t offset = 0;
    do {
        UartChunk chunk;
        const std::size_t take =
            std::min<std::size_t>(kUartChunkBytes, data.uart_data.size() - offset);
        chunk.size = static_cast<std::uint16_t>(take);
        chunk.idle_delimited = data.idle_delimited && offset + take == data.uart_data.size();
        std::memcpy(chunk.data.data(), data.uart_data.data() + offset, take);
        chunk.time = time;
        d.ring_.push(chunk);
        d.rx_bytes_.fetch_add(take, std::memory_order_relaxed);
        offset += take;
    } while (offset < data.uart_data.size());
    d.rx_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

// GPIO 与蜂鸣器不在 v4 的通道模型里（老 HCS 冻结在 v3）；板子上它们没有被声明的
// 路径，正常情况下这两个回调不会触发，触发即为路由错误。
bool BoardCore::gpio_digital_read_result_callback(
    std::uint8_t, const data::GpioDigitalDataView&, const libhcs::time::SampleTime&) noexcept {
    return false;
}

bool BoardCore::gpio_analog_read_result_callback(
    std::uint8_t, const data::GpioAnalogDataView&, const libhcs::time::SampleTime&) noexcept {
    return false;
}

void BoardCore::accelerometer_receive_callback(
    const data::ImuAccelerometerDataView& data, const libhcs::time::SampleTime& time) noexcept {
    if (imu_impl_ == nullptr || state_.load(std::memory_order_acquire) == BoardState::kIdle)
        return;
    imu_impl_->accelerometer_ring_.push({.x = data.x, .y = data.y, .z = data.z, .time = time});
    imu_impl_->rx_.fetch_add(1, std::memory_order_relaxed);
}

void BoardCore::gyroscope_receive_callback(
    const data::ImuGyroscopeDataView& data, const libhcs::time::SampleTime& time) noexcept {
    if (imu_impl_ == nullptr || state_.load(std::memory_order_acquire) == BoardState::kIdle)
        return;
    imu_impl_->gyroscope_ring_.push({.x = data.x, .y = data.y, .z = data.z, .time = time});
    imu_impl_->rx_.fetch_add(1, std::memory_order_relaxed);
}

void BoardCore::temperature_receive_callback(
    const data::ImuTemperatureDataView& data, const libhcs::time::SampleTime& time) noexcept {
    if (imu_impl_ == nullptr || state_.load(std::memory_order_acquire) == BoardState::kIdle)
        return;
    imu_impl_->temperature_ring_.push(
        {.raw_register_value = data.raw_register_value, .time = time});
    imu_impl_->rx_.fetch_add(1, std::memory_order_relaxed);
}

// ---- Board：声明与转发 ----

struct Board::Impl final : BoardCore {
    using BoardCore::BoardCore;
};

Board::Board(BoardOptions options, std::unique_ptr<Transport> transport)
    : impl_(std::make_unique<Impl>(std::move(options))) {
    const host::board::BoardTypeInfo* type =
        host::board::find_board_type(impl_->options_.board_type);
    if (type == nullptr) {
        std::string names;
        for (const auto& known : host::board::kBoardTypes) {
            if (!names.empty())
                names += ", ";
            names += known.name;
        }
        throw BoardTypeError{std::format(
            "unknown board_type '{}'; known board types are: {}", impl_->options_.board_type,
            names)};
    }
    impl_->type_ = type;
    impl_->transport_ = std::move(transport);
}

Board::~Board() { impl_->stop(); }

CanChannel& Board::add_can(std::string_view port_name, board::hcs::CanSetting setting) {
    if (impl_->state_.load(std::memory_order_acquire) != BoardState::kIdle)
        throw std::logic_error{"Board::add_can(): ports are declared before start()"};

    const auto port = host::board::find_port(*impl_->type_, port_name, spec::PortKind::kCan);
    if (!port)
        throw PortNotFoundError{std::format(
            "board type {} has no CAN port named '{}'; its CAN ports are: {}", impl_->type_->name,
            port_name, impl_->port_list_text(spec::PortKind::kCan))};
    const std::size_t slot = std::to_underlying(port->data_id);
    if (impl_->can_by_id_[slot] != nullptr)
        throw PortAlreadyDeclaredError{
            std::format("{} is already declared on this board", port_name)};

    auto storage = std::make_unique<CanChannel::Impl>(
        *impl_, port_name, port->data_id, setting, impl_->options_.can_rx_frames,
        impl_->options_.can_tx_frames);
    impl_->can_by_id_[slot] = storage.get();
    impl_->can_impls_.push_back(storage.get());
    impl_->configuration_.declare_can(port->data_id, setting);
    auto channel = std::unique_ptr<CanChannel>{new CanChannel{std::move(storage)}};
    impl_->can_view_.push_back(channel.get());
    impl_->cans_.push_back(std::move(channel));
    return *impl_->cans_.back();
}

UartChannel& Board::add_uart(std::string_view port_name, board::hcs::UartSetting setting) {
    if (impl_->state_.load(std::memory_order_acquire) != BoardState::kIdle)
        throw std::logic_error{"Board::add_uart(): ports are declared before start()"};

    const auto port = host::board::find_port(*impl_->type_, port_name, spec::PortKind::kUart);
    if (!port)
        throw PortNotFoundError{std::format(
            "board type {} has no UART port named '{}'; its UART ports are: {}", impl_->type_->name,
            port_name, impl_->port_list_text(spec::PortKind::kUart))};
    const std::size_t slot = std::to_underlying(port->data_id);
    if (impl_->uart_by_id_[slot] != nullptr)
        throw PortAlreadyDeclaredError{
            std::format("{} is already declared on this board", port_name)};

    auto storage = std::make_unique<UartChannel::Impl>(
        *impl_, port_name, port->data_id, setting, impl_->options_.uart_rx_chunks,
        impl_->options_.uart_tx_chunks);
    impl_->uart_by_id_[slot] = storage.get();
    impl_->uart_impls_.push_back(storage.get());
    impl_->configuration_.declare_uart(port->data_id, setting);
    auto channel = std::unique_ptr<UartChannel>{new UartChannel{std::move(storage)}};
    impl_->uart_view_.push_back(channel.get());
    impl_->uarts_.push_back(std::move(channel));
    return *impl_->uarts_.back();
}

ImuChannel& Board::add_imu(board::hcs::ImuSetting setting) {
    if (impl_->state_.load(std::memory_order_acquire) != BoardState::kIdle)
        throw std::logic_error{"Board::add_imu(): ports are declared before start()"};
    if (impl_->imu_ != nullptr)
        throw PortAlreadyDeclaredError{"the on-board IMU is already declared"};
    const bool has_imu =
        std::ranges::any_of(impl_->type_->ports, [](const spec::PortDescriptor& port) {
            return port.kind == spec::PortKind::kImu;
        });
    if (!has_imu)
        throw PortNotFoundError{
            std::format("board type {} has no on-board IMU", impl_->type_->name)};

    auto storage =
        std::make_unique<ImuChannel::Impl>(*impl_, setting, impl_->options_.imu_rx_samples);
    impl_->imu_impl_ = storage.get();
    impl_->configuration_.declare_imu(setting);
    impl_->imu_.reset(new ImuChannel{std::move(storage)});
    return *impl_->imu_;
}

void Board::start() { impl_->start(); }

void Board::run() { impl_->run(); }

void Board::arm() { impl_->arm(); }

void Board::stop() noexcept { impl_->stop(); }

void Board::receive() noexcept { impl_->receive(); }

void Board::send() noexcept { impl_->send(); }

void Board::request_safe(bool safe) noexcept { impl_->request_safe(safe); }

BoardState Board::state() const noexcept { return impl_->state_.load(std::memory_order_acquire); }

LinkState Board::link_state() const noexcept {
    return impl_->engine_ != nullptr ? impl_->engine_->link_state() : LinkState::kSessionDown;
}

Statistics Board::statistics() const noexcept { return impl_->statistics(); }

std::span<CanChannel* const> Board::can_channels() const noexcept { return impl_->can_view_; }

std::span<UartChannel* const> Board::uart_channels() const noexcept { return impl_->uart_view_; }

ImuChannel* Board::imu_channel() const noexcept { return impl_->imu_.get(); }

std::string_view Board::serial_number() const noexcept {
    return impl_->transport_ != nullptr ? impl_->transport_->serial() : std::string_view{};
}

} // namespace libhcs
