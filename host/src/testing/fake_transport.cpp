#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include <libhcs/testing/fake_transport.hpp>

#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"

// 假板的实现：固件一侧的解析器（core 的 Deserializer——固件跑的就是这一份）加
// 一个小状态机。下行解析产生应答字节（会话 ack、kTimeStatus），在 transmit()
// 的末尾同步交给 Board 的接收回调——不排队、不起线程，测试因此是确定性的。
// 测试线程的 inject_* 与 IO 线程的 transmit() 用一把互斥锁串行。

namespace libhcs {

namespace {

namespace vc = core::protocol::vendor_control;
using core::protocol::DeserializeCallback;
using core::protocol::Deserializer;
using core::protocol::FieldId;
using core::protocol::SerializeBuffer;
using core::protocol::Serializer;

// 往一个 vector 里序列化的小缓冲。
class VectorBuffer final : public SerializeBuffer {
public:
    std::span<std::byte> allocate(std::size_t size) noexcept override {
        const std::size_t used = bytes_.size();
        bytes_.resize(used + size);
        return {bytes_.data() + used, size};
    }

    [[nodiscard]] std::vector<std::byte> take() && noexcept { return std::move(bytes_); }

private:
    std::vector<std::byte> bytes_;
};

} // namespace

struct FakeTransport::Impl final : DeserializeCallback {
    Impl(std::span<const FakeTransport::Port> port_table, std::uint8_t caps)
        : deserializer(*this) {
        ports.assign(port_table.begin(), port_table.end());
        board_caps = caps;
    }

    // ---- 板子状态 ----
    std::vector<FakeTransport::Port> ports;
    std::uint8_t board_caps = 0;
    std::uint16_t version = vc::kVersion;
    bool disconnected = false;
    std::atomic<bool> faulted{false};
    std::vector<FakeTransport::ManifestEntrySeen> manifest;

    // ---- 合成时间轴：构造时刻起的 steady_clock，125 us 一微帧 ----
    std::chrono::steady_clock::time_point epoch{std::chrono::steady_clock::now()};

    [[nodiscard]] std::uint64_t nanos_since_epoch() const noexcept {
        return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::steady_clock::now() - epoch)
                                              .count());
    }
    [[nodiscard]] std::uint64_t microframe_now() const noexcept {
        return nanos_since_epoch() / 125'000U;
    }
    [[nodiscard]] std::uint32_t quarter_us_now() const noexcept {
        return static_cast<std::uint32_t>(nanos_since_epoch() / 250U);
    }
    [[nodiscard]] time::SofStamp stamp_now() const noexcept {
        // 全部折成 1/16384 微帧的 tick 再截 24 位：与真板一样，只有低 10 位整
        // 帧号 + 分数上线路。
        const std::uint64_t ticks = nanos_since_epoch() * 16'384U / 125'000U;
        return time::SofStamp{static_cast<std::uint32_t>(ticks & time::SofStamp::kMask)};
    }

    // ---- 传输面 ----
    std::mutex mutex;
    std::function<void(std::span<const std::byte>)> receive_callback;
    Deserializer deserializer;
    std::vector<std::byte> pending_uplink; // 本次 transmit() 解析产生的应答
    std::vector<FakeTransport::SentCan> sent_can;
    std::vector<FakeTransport::SentUart> sent_uart;

    void deliver(std::span<const std::byte> bytes) const {
        if (receive_callback && !bytes.empty())
            receive_callback(bytes);
    }

    template <typename Write>
    void queue_reply(Write&& write) {
        VectorBuffer buffer;
        Serializer serializer{buffer};
        (void)write(serializer);
        std::vector<std::byte> bytes = std::move(buffer).take();
        pending_uplink.insert(pending_uplink.end(), bytes.begin(), bytes.end());
    }

    // ---- DeserializeCallback：固件视角 ----

    bool can_deserialized_callback(FieldId id, const data::CanDataView& view) override {
        sent_can.push_back(FakeTransport::SentCan{
            .port = id,
            .id = view.can_id,
            .extended = view.is_extended_can_id,
            .remote = view.is_remote_transmission,
            .payload = {view.can_data.begin(), view.can_data.end()},
        });
        return true;
    }

    bool uart_deserialized_callback(FieldId id, const data::UartDataView& view) override {
        sent_uart.push_back(FakeTransport::SentUart{
            .port = id,
            .bytes = {view.uart_data.begin(), view.uart_data.end()},
            .idle_delimited = view.idle_delimited,
        });
        return true;
    }

    bool gpio_digital_data_deserialized_callback(
        std::uint8_t, const data::GpioDigitalDataView&) override {
        return false; // v4 不声明 GPIO，板子收到即拒
    }
    bool gpio_analog_data_deserialized_callback(
        std::uint8_t, const data::GpioAnalogDataView&) override {
        return false;
    }
    bool gpio_read_deserialized_callback(std::uint8_t) override { return false; }
    bool buzzer_tone_deserialized_callback(const data::BuzzerToneDataView&) override {
        return false;
    }
    void accelerometer_deserialized_callback(const data::ImuAccelerometerDataView&) override {}
    void gyroscope_deserialized_callback(const data::ImuGyroscopeDataView&) override {}
    void temperature_deserialized_callback(const data::ImuTemperatureDataView&) override {}

    void session_control_deserialized_callback(const data::SessionControlView& view) override {
        switch (view.type) {
        case data::SessionType::kStart:
            queue_reply([&](Serializer& s) {
                (void)s.write_session_control(
                    {.type = data::SessionType::kStartAck, .nonce = view.nonce});
            });
            break;
        case data::SessionType::kKeepalive:
            queue_reply([&](Serializer& s) {
                (void)s.write_session_control(
                    {.type = data::SessionType::kKeepaliveAck, .nonce = view.nonce});
            });
            break;
        default: break;
        }
    }

    void time_anchor_deserialized_callback(const data::TimeAnchorView& view) override {
        // 一块时间基准有效、晶振完美的板：位置就是合成轴上的当前值。fraction
        // 取零即可——往返拟合与展开都不依赖它非零。
        queue_reply([&](Serializer& s) {
            (void)s.write_time_status(data::TimeStatusView{
                .nonce = view.nonce,
                .microframe = microframe_now(),
                .microframe_fraction_q16 = 0,
                .timestamp_quarter_us = quarter_us_now(),
                .ticks_per_microframe_q16 = 500U << 16,
                .state = data::TimeState::kValid,
                .anomaly_count = 0,
                .residual_mean_q16 = 0,
                .residual_abs_max_q16 = 0,
                .residual_count = 0,
            });
        });
    }

    void error_callback(FieldId, data::DownlinkError) override {
        // 主机发来的字节解析失败：真板会记一笔下行错误；这里没有要验的路径，忽略。
    }
};

// ---- 构造 / 析构 ----

FakeTransport::FakeTransport(std::span<const Port> ports, std::uint8_t board_caps)
    : impl_(std::make_unique<Impl>(ports, board_caps)) {}

FakeTransport::~FakeTransport() = default;

// ---- 上行注入 ----

void FakeTransport::inject_can_frame(
    data::DataId port, std::uint32_t can_id, std::span<const std::byte> payload,
    std::optional<time::SofStamp> stamp, bool extended) {
    VectorBuffer buffer;
    Serializer serializer{buffer};
    (void)serializer.write_can(
        port, {
                  .can_id = can_id,
                  .can_data = payload,
                  .is_extended_can_id = extended,
                  .sof_stamp = stamp,
              });
    std::vector<std::byte> bytes = std::move(buffer).take();
    const std::scoped_lock lock{impl_->mutex};
    impl_->deliver(bytes);
}

void FakeTransport::inject_uart_bytes(
    data::DataId port, std::span<const std::byte> bytes, bool idle_delimited) {
    VectorBuffer buffer;
    Serializer serializer{buffer};
    (void)serializer.write_uart(port, {.uart_data = bytes, .idle_delimited = idle_delimited});
    std::vector<std::byte> out = std::move(buffer).take();
    const std::scoped_lock lock{impl_->mutex};
    impl_->deliver(out);
}

void FakeTransport::inject_imu_accelerometer(std::int16_t x, std::int16_t y, std::int16_t z) {
    VectorBuffer buffer;
    Serializer serializer{buffer};
    (void)serializer.write_imu_accelerometer(
        {.x = x, .y = y, .z = z, .timestamp_quarter_us = impl_->quarter_us_now()});
    std::vector<std::byte> bytes = std::move(buffer).take();
    const std::scoped_lock lock{impl_->mutex};
    impl_->deliver(bytes);
}

void FakeTransport::inject_imu_gyroscope(std::int16_t x, std::int16_t y, std::int16_t z) {
    VectorBuffer buffer;
    Serializer serializer{buffer};
    (void)serializer.write_imu_gyroscope(
        {.x = x, .y = y, .z = z, .timestamp_quarter_us = impl_->quarter_us_now()});
    std::vector<std::byte> bytes = std::move(buffer).take();
    const std::scoped_lock lock{impl_->mutex};
    impl_->deliver(bytes);
}

void FakeTransport::inject_imu_temperature(std::uint16_t raw_register_value) {
    VectorBuffer buffer;
    Serializer serializer{buffer};
    (void)serializer.write_imu_temperature(
        {.raw_register_value = raw_register_value,
         .timestamp_quarter_us = impl_->quarter_us_now()});
    std::vector<std::byte> bytes = std::move(buffer).take();
    const std::scoped_lock lock{impl_->mutex};
    impl_->deliver(bytes);
}

// ---- 下行捕获 ----

std::vector<FakeTransport::SentCan> FakeTransport::take_sent_can() {
    const std::scoped_lock lock{impl_->mutex};
    std::vector<SentCan> out = std::move(impl_->sent_can);
    impl_->sent_can.clear();
    return out;
}

std::vector<FakeTransport::SentUart> FakeTransport::take_sent_uart() {
    const std::scoped_lock lock{impl_->mutex};
    std::vector<SentUart> out = std::move(impl_->sent_uart);
    impl_->sent_uart.clear();
    return out;
}

// ---- EP0 状态 ----

std::vector<FakeTransport::ManifestEntrySeen> FakeTransport::last_manifest() const {
    const std::scoped_lock lock{impl_->mutex};
    return impl_->manifest;
}

void FakeTransport::set_reported_version(std::uint16_t version) {
    const std::scoped_lock lock{impl_->mutex};
    impl_->version = version;
}

// ---- 故障注入 ----

void FakeTransport::simulate_disconnect(bool disconnected) {
    const std::scoped_lock lock{impl_->mutex};
    impl_->disconnected = disconnected;
}

void FakeTransport::simulate_removal() { impl_->faulted.store(true, std::memory_order_relaxed); }

// ---- 合成时间轴 ----

std::chrono::steady_clock::time_point FakeTransport::epoch() const noexcept { return impl_->epoch; }

std::int64_t
    FakeTransport::microframe_at(std::chrono::steady_clock::time_point when) const noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(when - impl_->epoch).count()
         / 125'000;
}

time::SofStamp FakeTransport::stamp_now() const noexcept { return impl_->stamp_now(); }

// ---- Transport 接口 ----

void FakeTransport::transmit(std::span<const std::byte> bytes) {
    const std::scoped_lock lock{impl_->mutex};
    if (impl_->faulted.load(std::memory_order_relaxed))
        return; // 拔走了：字节没人收

    // "断线"按一块收得到但不应答的板建模：下行照常解析（take_sent_* 看得见，
    // 断线期间主机补发的安全帧因此可断言），应答一个都不回——会话超时由此来。
    impl_->pending_uplink.clear();
    impl_->deserializer.feed(bytes);
    impl_->deserializer.finish_transfer();

    if (impl_->disconnected)
        return;
    // 应答在解析完成后一次性交给 Board：不嵌套进解析过程。
    impl_->deliver(impl_->pending_uplink);
    impl_->pending_uplink.clear();
}

void FakeTransport::receive(std::function<void(std::span<const std::byte>)> callback) {
    const std::scoped_lock lock{impl_->mutex};
    impl_->receive_callback = std::move(callback);
}

FakeTransport::ControlResult FakeTransport::vendor_control(
    std::uint8_t request_type, std::uint8_t request, std::uint16_t index,
    std::span<std::byte> payload) {
    (void)index;
    if (request_type != vc::kRequestTypeIn && request_type != vc::kRequestTypeOut)
        return ControlResult::kUnsupported;

    const std::scoped_lock lock{impl_->mutex};
    if (impl_->disconnected || impl_->faulted.load(std::memory_order_relaxed))
        return ControlResult::kFailed;

    using vc::Request;
    switch (static_cast<Request>(request)) {
    case Request::kGetPortList: {
        if (payload.size() < sizeof(vc::PortListPayload))
            return ControlResult::kFailed;
        vc::PortListPayload out{};
        out.version = impl_->version;
        out.port_count = static_cast<std::uint8_t>(impl_->ports.size());
        out.board_caps = impl_->board_caps;
        for (std::size_t i = 0; i < impl_->ports.size() && i < std::size(out.ports); ++i)
            out.ports[i] = {
                .data_id = impl_->ports[i].data_id,
                .kind = impl_->ports[i].kind,
                .capabilities = impl_->ports[i].capabilities,
                .status = impl_->ports[i].status,
            };
        std::memcpy(payload.data(), &out, sizeof(out));
        return ControlResult::kOk;
    }
    case Request::kApplyManifest: {
        constexpr std::size_t kEntriesOffset = offsetof(vc::ManifestPayload, entries);
        const std::size_t entry_size = sizeof(vc::ManifestEntry);
        if (payload.size() < kEntriesOffset || (payload.size() - kEntriesOffset) % entry_size != 0)
            return ControlResult::kFailed;
        vc::ManifestPayload header{};
        std::memcpy(&header, payload.data(), payload.size());
        const std::size_t count = std::min<std::size_t>(
            header.entry_count, (payload.size() - kEntriesOffset) / entry_size);
        impl_->manifest.clear();
        for (std::size_t i = 0; i < count; ++i)
            impl_->manifest.push_back(ManifestEntrySeen{
                .data_id = header.entries[i].data_id,
                .kind = header.entries[i].kind,
                .line = header.entries[i].line,
                .flags = header.flags,
            });
        return ControlResult::kOk; // 照单全收：结果查询见下
    }
    case Request::kGetManifestResult: {
        if (payload.size() < sizeof(vc::ManifestResultPayload))
            return ControlResult::kFailed;
        vc::ManifestResultPayload out{};
        out.outcome = std::to_underlying(vc::ManifestOutcome::kManifestApplied);
        out.entry_count = static_cast<std::uint8_t>(impl_->manifest.size());
        for (std::size_t i = 0; i < impl_->manifest.size() && i < std::size(out.entries); ++i) {
            out.entries[i] = {};
            out.entries[i].data_id = impl_->manifest[i].data_id;
            out.entries[i].line = impl_->manifest[i].line;
        }
        std::memcpy(payload.data(), &out, sizeof(out));
        return ControlResult::kOk;
    }
    case Request::kGetPortConfig:
    case Request::kGetLastConfigError:
        // 读回一律全零：v4 的 start() 不依赖它们（IMU 的声明核对在板端，假板
        // 照单全收）。留给需要它们的扩展。
        std::fill(payload.begin(), payload.end(), std::byte{0});
        return ControlResult::kOk;
    default: return ControlResult::kStalled;
    }
}

bool FakeTransport::link_faulted() const noexcept {
    return impl_->faulted.load(std::memory_order_relaxed);
}

} // namespace libhcs
