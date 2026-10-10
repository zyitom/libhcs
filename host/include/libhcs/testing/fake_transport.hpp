#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include <libhcs/data/datas.hpp>
#include <libhcs/export.hpp>
#include <libhcs/protocol/vendor_control.hpp>
#include <libhcs/time/sof_stamp.hpp>
#include <libhcs/transport.hpp>

// 测试用的假传输（HCS-USB.md §2.1 的 FakeTransport）：在内存里演一块板，不碰
// USB。Board 的整条路径——EP0 声明、会话握手、保活、拉取式通道、时间基准——
// 都能对着它跑。
//
// 它内部就是固件那一侧的解析器（core 的 Deserializer）加一个小状态机：
//   - kStart / kKeepalive 按协议应答（同步，在 transmit() 里就地回）；
//   - kTimeAnchor 按一块"时间基准有效"的板应答，位置来自一条合成微帧轴
//     （构造时刻起的 steady_clock，125 us 一帧）；
//   - EP0 按构造时给的口表回答 kGetPortList，kApplyManifest 照单全收并记下。
//
// 线程模型：transmit() 由 Board 的 IO 线程（或 start() 的调用线程）调，inject_*
// 从测试线程调，内部一把互斥锁串行化；应答与注入都在调用线程上同步回调，测试
// 不需要等、不需要轮询。

namespace libhcs {

using core::protocol::vendor_control::kBoardCapTimeSync;

class libhcs_API FakeTransport final : public Transport {
public:
    /// 一项口表条目：{DataId 数值, PortKind 数值, capabilities, status}。
    struct Port {
        std::uint8_t data_id;
        std::uint8_t kind;
        std::uint8_t capabilities;
        std::uint8_t status;
    };

    /// @param ports 板子自报的口表（kGetPortList 的内容）。默认：空表——
    ///              任何声明都会在 start() 的接线核对处被拒绝。
    /// @param board_caps BoardCapability 位；默认带时间基准。
    explicit FakeTransport(
        std::span<const Port> ports = {}, std::uint8_t board_caps = kBoardCapTimeSync);
    ~FakeTransport() final;

    FakeTransport(const FakeTransport&) = delete;
    FakeTransport& operator=(const FakeTransport&) = delete;
    FakeTransport(FakeTransport&&) = delete;
    FakeTransport& operator=(FakeTransport&&) = delete;

    // ---- 上行注入（任意线程；Board 未 start 或会话未开时被丢弃）----

    /// 注入一帧上行 CAN。stamp 为空 = 板子没盖时间戳（SampleTime 退到达达时刻）。
    void inject_can_frame(
        data::DataId port, std::uint32_t can_id, std::span<const std::byte> payload,
        std::optional<time::SofStamp> stamp = std::nullopt, bool extended = false);
    void inject_uart_bytes(
        data::DataId port, std::span<const std::byte> bytes, bool idle_delimited = false);
    void inject_imu_accelerometer(std::int16_t x, std::int16_t y, std::int16_t z);
    void inject_imu_gyroscope(std::int16_t x, std::int16_t y, std::int16_t z);
    void inject_imu_temperature(std::uint16_t raw_register_value);

    // ---- 下行捕获（任意线程；取出后清空）----

    struct SentCan {
        data::DataId port;
        std::uint32_t id;
        bool extended;
        bool remote;
        std::vector<std::byte> payload;
    };
    struct SentUart {
        data::DataId port;
        std::vector<std::byte> bytes;
        bool idle_delimited;
    };
    [[nodiscard]] std::vector<SentCan> take_sent_can();
    [[nodiscard]] std::vector<SentUart> take_sent_uart();

    // ---- EP0 状态（任意线程）----

    struct ManifestEntrySeen {
        std::uint8_t data_id;
        std::uint8_t kind;
        std::uint8_t line;
        std::uint8_t flags; // 清单头的 flags 字节（时间基准声明在位 0）
    };
    [[nodiscard]] std::vector<ManifestEntrySeen> last_manifest() const;
    /// 让口表报一个不一样的 EP0 版本号——版本核对路径的测试。
    void set_reported_version(std::uint16_t version);

    // ---- 故障注入 ----

    /// true = 板子不应答了：transmit 吞掉、EP0 失败，但链路本身"还在"
    /// （LinkState 会掉到 kSessionDown，保活不断重试）。
    void simulate_disconnect(bool disconnected);
    /// 板子被拔走：link_faulted() 起效（LinkState::kFaulted，不可恢复）。
    void simulate_removal();

    // ---- 合成时间轴（测试给共享轴喂样本用）----

    /// 假板的纪元（构造时刻）。合成微帧轴：position(t) = (t - epoch) / 125 us。
    [[nodiscard]] std::chrono::steady_clock::time_point epoch() const noexcept;
    /// t 时刻在合成轴上的位置（整微帧，带符号——早于纪元的时刻是负值，喂拟合
    /// 用负样本完全合法）。
    [[nodiscard]] std::int64_t
        microframe_at(std::chrono::steady_clock::time_point when) const noexcept;
    /// 此刻的 CAN 时间戳（合成轴位置打包成 24 位 SofStamp）。
    [[nodiscard]] time::SofStamp stamp_now() const noexcept;

    // ---- Transport 接口 ----

    void transmit(std::span<const std::byte> bytes) final;
    void receive(std::function<void(std::span<const std::byte>)> callback) final;
    ControlResult vendor_control(
        std::uint8_t request_type, std::uint8_t request, std::uint16_t index,
        std::span<std::byte> payload) final;
    [[nodiscard]] bool link_faulted() const noexcept final;
    [[nodiscard]] std::string_view serial() const noexcept final { return "FAKE-0001"; }
    [[nodiscard]] int usb_bus_number() const noexcept final { return -1; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace libhcs
