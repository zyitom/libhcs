#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/link/interrupt_safe_buffer.hpp"
#include "firmware/hpm_board/app/src/sync/pulse.hpp"
#include "firmware/hpm_board/app/src/sync/timebase.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"
#include "firmware/hpm_board/app/src/uart/uart.hpp"

namespace libhcs::firmware::link {

// 本应用所有主机传输共用的主机会话端点: libhcs 会话生命周期 (kStart nonce +
// keepalive 租约) 与到 CAN/UART 驱动的下行分发, 连同上行 serializer 及其中断
// 安全批量缓冲。传输层 (USB vendor class、EtherCAT PD 流) 子类化它, 只保留
// 自己的发送形态: 当前批量如何上线。
class HostSession
    : private core::protocol::DeserializeCallback
    , private core::utility::Immovable {
public:
    HostSession() = default;

    // 会话租约 4000 ms = 主机轮次(kSessionRefreshInterval, 1 s)的四倍余,
    // 单位 0.25 us/tick (4 MHz mchtmr)。租期必须大于轮次, 否则 1 Hz 轮次落在
    // 到期边界上, 板子会静默丢弃 keepalive [实测 2026-09-13, 1 Hz 下 ~70% 轮次
    // 超时, 根因即租期与轮次同长]。
    static constexpr uint64_t kSessionLeaseQuarterUs = 16'000'000U;

    core::protocol::Serializer& serializer() { return serializer_; }

    // 主机会话握手 (kStart nonce + keepalive 租约) 建立且数据确实在转发后为
    // true -- 区别于单纯的传输连通 (USB 枚举 / EtherCAT OP)。
    bool session_established() const { return session_established_; }

    // kStart ack 入队后为 true: 从此刻起上行流可携带驱动遥测。
    bool uplink_enabled() const { return uplink_enabled_; }

    void handle_downlink(std::span<const std::byte> buffer) { deserializer_.feed(buffer); }

    // 把 time base 积压的硬件脉冲捕获排上上行。主循环调用; time base 未编译
    // 进来或没有真的捕获到探针帧时是空操作。换算需要历史环和一次除法, 放在
    // 主循环而非捕获中断里做。
    void poll_pulse_captures() {
        if constexpr (!sync::pulse::kEnabled)
            return;
        if (!session_established_)
            return;
        uint64_t captured_q16 = 0;
        while (sync::pulse::take_capture(captured_q16)) {
            (void)serializer_.write_pulse_report({
                .nonce = current_session_nonce_,
                .scheduled_microframe = pending_pulse_microframe_,
                .captured_microframe_q16 = captured_q16,
                .ticks_per_microframe_q16 = sync::pulse::measured_ticks_per_microframe_q16(),
                .flags = static_cast<uint8_t>(
                    (pending_pulse_armed_ ? data::kPulseArmed : data::PulseReportFlags{})
                    | data::kPulseCaptured),
            });
        }
    }

    void finish_downlink_transfer() { deserializer_.finish_transfer(); }

    void deactivate_session() {
        uplink_enabled_ = false;
        session_established_ = false;
        session_deactivated_callback();
    }

protected:
    // 传输层发送路径第 1 步: 先刷新会话租约, 再交出当前在途的批量 (没有就弹出
    // 下一个待发的)。会话未建立或无待发时返回 nullptr。批量在 finish_batch()
    // 之前一直是"当前"的, 部分传输 (USB 分包) 与整批重试 (环背压) 都能从断点
    // 继续。
    const InterruptSafeBuffer::Batch* next_batch() {
        refresh_session_state();

        if (!session_established_)
            return nullptr;

        if (!transmitting_batch_)
            transmitting_batch_ = transmit_buffer_.pop_batch();
        return transmitting_batch_;
    }

    // 传输层发送路径第 2 步: 当前批量已完整上线, 归还池子。
    void finish_batch() {
        InterruptSafeBuffer::release_batch(transmitting_batch_);
        transmitting_batch_ = nullptr;
    }

    // 新 nonce 的 kStart 重置了流: 在途批量与所有待发批量刚被丢弃。传输层应
    // 重置传输进度。
    virtual void session_activated_callback() {}

    // 该传输层现在是否接受 kStart。承载带外配置握手的传输层在握手完成前拒绝,
    // 使没做过握手的主机无法开会话后按板子不认可的假设行动 -- 具体地, 帧类型
    // 移到 EP0 之前的旧主机会假定线上已不再协商的逐帧 classic/FD 选择, 在期望
    // 经典帧的地方收到 FD 帧, 且没有任何地方报告该错配。
    //
    // 默认 true, 且必须保持 true: EtherCAT 过程数据传输层没有可挂这类握手的
    // 控制端点。
    virtual bool session_allowed() const { return true; }

    // 会话结束 -- 租约到期、总线复位或拔线。带外握手门控的传输层在此忘记
    // 握手, 下一个主机必须自己做: tud_mount_cb 只在重新枚举时触发, 不拔线换
    // 主机程序不会重新枚举。没有这一步, 不握手的主机会继承上一主机的 --
    // 实测过, 直接穿门而过。
    virtual void session_deactivated_callback() {}

    // 让传输层把第二个 deserializer 接进同一会话, 供协议走多条管道的传输层
    // 使用 (USB CAN 端点对)。回调即会话门控保持共享; 只有帧状态按管道隔离 --
    // 两个管道的字节交织进一个 deserializer 会把两边都搞坏, 这正是想要的。
    core::protocol::DeserializeCallback& deserialize_callback() { return *this; }

private:
    void activate_session(uint32_t nonce) {
        if (transmitting_batch_) {
            InterruptSafeBuffer::release_batch(transmitting_batch_);
            transmitting_batch_ = nullptr;
        }
        transmit_buffer_.clear();
        session_activated_callback();

        current_session_nonce_ = nonce;
        last_session_refresh_ = timer::Timer::timestamp64_quarter_us();
        session_established_ = true;
        uplink_enabled_ = false;
    }

    void refresh_session_state() {
        if (!session_established_)
            return;

        if (timer::Timer::timestamp64_quarter_us() - last_session_refresh_ < kSessionLeaseQuarterUs)
            return;

        deactivate_session();
    }

    bool can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        if (!session_established_)
            return true;
        // 以 can_count() 而非 kCanCount 为界: 这是主机提供的输入, 单路 hpm5321
        // 的镜像仍带着一个从未构造的第二表槽。发往该未用 DataId (kCan2) 的帧
        // 必须被拒绝 (false = "本板不认这个 field id"), 而不是派发给未初始化
        // 的 Can。
        for (size_t i = 0; i < can::can_count(); i++) {
            if (can::kCanDataIds[i] != static_cast<data::DataId>(id))
                continue;
            can::can_array[i]->handle_downlink(data);
            return true;
        }
        return false;
    }

    bool uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        if (!session_established_)
            return true;
        for (auto& board_uart : uart::uart_array) {
            if (static_cast<core::protocol::FieldId>(board_uart->data_id()) == id) {
                board_uart->handle_downlink(data);
                return true;
            }
        }
        return false;
    }

    // 带内 UART 配置, 已在 USB 传输层废弃: 该返回值表达不了"波特率被拒" --
    // 它只表示"认识这个 field id", 不表示"操作成功"; 把配置字段回显到上行也
    // 不行, UartConfig 按契约是纯下行通道, 主机处理器把上行的一份当路由错误,
    // 令 deserializer 失败并杀掉链路 (实测: UART 从 PASS 掉到 0/320)。配置现
    // 走 EP0, 状态阶段天然携带应答; 见 usb/vendor_control.cpp 与
    // libhcs/protocol/vendor_control.hpp。
    bool uart_config_deserialized_callback(
        core::protocol::FieldId id, const data::UartConfigView& data) override {
        if (!session_established_)
            return true;
        // 拒绝而非忽略。false 让 deserializer 在本次传输余下部分进入丢弃模式,
        // 这正是对本传输层不再实现的字段的正确回答: 悄悄丢掉会让主机以为已切
        // 到从未变过的波特率 -- 移去 EP0 正是为终结这个失败。
        (void)id;
        (void)data;
        return false;
    }

    // 本板无 GPIO 应用; 主机的 GPIO 命令被忽略。
    bool gpio_digital_data_deserialized_callback(
        uint8_t channel_index, const data::GpioDigitalDataView& data) override {
        if (!session_established_)
            return true;
        (void)channel_index;
        (void)data;
        return false;
    }

    bool gpio_analog_data_deserialized_callback(
        uint8_t channel_index, const data::GpioAnalogDataView& data) override {
        if (!session_established_)
            return true;
        (void)channel_index;
        (void)data;
        return false;
    }

    bool gpio_digital_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (!session_established_)
            return true;
        (void)channel_index;
        (void)data;
        return false;
    }

    bool gpio_analog_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (!session_established_)
            return true;
        (void)channel_index;
        (void)data;
        return false;
    }

    void accelerometer_deserialized_callback(const data::ImuAccelerometerDataView& data) override {
        (void)data;
    }

    void gyroscope_deserialized_callback(const data::ImuGyroscopeDataView& data) override {
        (void)data;
    }

    void temperature_deserialized_callback(const data::ImuTemperatureDataView& data) override {
        (void)data;
    }

    void session_control_deserialized_callback(const data::SessionControlView& data) override {
        switch (data.type) {
        case data::SessionType::kStart: {
            // 静默拒绝。会话协议没有否定应答, 打不开会话的主机约一秒后会自己
            // 超时报错; 什么都不说已是这一层唯一能说的话。
            if (!session_allowed())
                return;

            const bool same_session = session_established_ && data.nonce == current_session_nonce_;

            if (!same_session)
                activate_session(data.nonce);
            else
                last_session_refresh_ = timer::Timer::timestamp64_quarter_us();

            const auto result = serializer_.write_session_control(
                {.type = data::SessionType::kStartAck, .nonce = data.nonce});
            core::utility::assert_always(
                result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            uplink_enabled_ = true;
            break;
        }
        case data::SessionType::kKeepalive:
            if (!session_established_ || data.nonce != current_session_nonce_)
                return;

            last_session_refresh_ = timer::Timer::timestamp64_quarter_us();
            {
                const auto result = serializer_.write_session_control(
                    {.type = data::SessionType::kKeepaliveAck, .nonce = data.nonce});
                core::utility::assert_always(
                    result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            }
            break;
        default: return;
        }
    }

    // 共享时间基。锚点放在会话字段里正是要它与会话同生共死: 失去主机的板子
    // 不应继续维护主机日后假定仍对齐的时间线。与 keepalive 一样检查 nonce, 并
    // 在同一次交换中应答, 主机不必再跑一轮就能拿到板子状态。
    void time_anchor_deserialized_callback(const data::TimeAnchorView& data) override {
        if (!session_established_ || data.nonce != current_session_nonce_)
            return;

        sync::timebase::apply_anchor(data.microframe);

        const auto snapshot = sync::timebase::report();

        // 上报"此刻"的配对, 而非最后一次 SOF 时的。snapshot.microframe 是上
        // 一个 Start-of-Frame 锁存的计数, 高速下落后 0..125 us -- 均匀分布,
        // 均值 62.5 us。主机把收到的值与本往返的中点配对, 直接上报锁存值会
        // 把这段均匀的量化噪声注入主机对自身时钟的拟合。mc02 已按同一理由
        // 插值 [实测 2026-09-07]; 时间线无效时回退到锁存对。
        auto reported_microframe = snapshot.microframe;
        auto reported_quarter_us = static_cast<uint32_t>(snapshot.timestamp_quarter_us);
        {
            std::uint64_t microframe_now = 0;
            std::uint32_t now_quarter_us = 0;
            if (sync::timebase::microframe_now(microframe_now, now_quarter_us)) {
                reported_microframe = microframe_now;
                reported_quarter_us = now_quarter_us;
            }
        }

        (void)serializer_.write_time_status({
            .nonce = data.nonce,
            .microframe = reported_microframe,
            .timestamp_quarter_us = reported_quarter_us,
            .ticks_per_microframe_q16 = snapshot.ticks_per_microframe_q16,
            .state = snapshot.state,
            .anomaly_count = snapshot.anomaly_count,
            .residual_mean_q16 = snapshot.residual_mean_q16,
            .residual_abs_max_q16 = snapshot.residual_abs_max_q16,
            .residual_count = static_cast<uint16_t>(snapshot.residual_count),
        });
    }

    // 硬件脉冲交换。主机向每块板发相同的目标 microframe; 各板在该处触发并
    // 捕获其他板的脉冲。
    void pulse_schedule_deserialized_callback(const data::PulseScheduleView& data) override {
        if (!session_established_ || data.nonce != current_session_nonce_)
            return;
        pending_pulse_microframe_ = data.microframe;
        pending_pulse_armed_ = sync::pulse::schedule(data.microframe);

        // 无论是否 armed 都应答每一条 schedule。放不上目标又不作声的板子, 在
        // 主机看来与"触发了但没听到回声"无法区分 -- 这两种失败需要相反的修法。
        (void)serializer_.write_pulse_report({
            .nonce = current_session_nonce_,
            .scheduled_microframe = data.microframe,
            .captured_microframe_q16 = 0,
            .ticks_per_microframe_q16 = sync::pulse::measured_ticks_per_microframe_q16(),
            .flags = static_cast<uint8_t>(
                pending_pulse_armed_ ? data::kPulseArmed : data::PulseReportFlags{}),
        });
    }

    void error_callback() override {
        // 本层之下的每个传输层都无损交付字节 (USB bulk、EtherCAT 停等 ARQ),
        // 反序列化出错即主机/固件的帧定界 bug。恢复发生在下一个传输边界 /
        // 链路重启, 经 finish_downlink_transfer()。
    }

    core::protocol::Deserializer deserializer_{*this};

    InterruptSafeBuffer transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};

    const InterruptSafeBuffer::Batch* transmitting_batch_ = nullptr;
    bool uplink_enabled_ = false;
    bool session_established_ = false;
    uint32_t current_session_nonce_ = 0;
    uint64_t last_session_refresh_ = 0;
    uint64_t pending_pulse_microframe_ = 0;
    bool pending_pulse_armed_ = false;
};

} // namespace libhcs::firmware::link
