#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>

#include <class/vendor/vendor_device.h>
#include <device/usbd.h>
#include <main.h>
#include <tusb.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/can/can.hpp"
#include "firmware/mc02/app/src/gpio/gpio.hpp"
#include "firmware/mc02/app/src/sync/sof.hpp"
#include "firmware/mc02/app/src/sync/timebase.hpp"
#include "firmware/mc02/app/src/timer/timer.hpp"
#include "firmware/mc02/app/src/uart/uart.hpp"
#include "firmware/mc02/app/src/usb/interrupt_safe_buffer.hpp"
#include "firmware/mc02/app/src/usb/usb_descriptors.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

namespace libhcs::firmware::usb {

void poll_dfu_runtime_reboot();

class Vendor
    : private core::protocol::DeserializeCallback
    , private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Vendor>;

    static constexpr size_t kMaxPacketSize = 64;
    // 主机轮次为 1 s(kSessionRefreshInterval), 租期 4 s = 四个轮次未到即失效。
    static constexpr auto kSessionLease = std::chrono::milliseconds{4000};

    Vendor() {
        usb::usb_descriptors.init();

        // 在此固定 USB 中断优先级: 没有别处会做, 且必须赶在控制器能触发中断之前
        // -- tusb_rhport_init -> dcd_init -> dcd_int_enable 只调 NVIC_EnableIRQ,
        // 之后再设优先级会留下处于复位值的窗口。本应设置优先级的 HAL_PCD_MspInit
        // 属于 ST 设备栈, 本固件没有链接它。NVIC 优先级寄存器复位为 0, 不设此行则
        // OTG_HS 将运行在抢占优先级 0 -- 高于 FDCAN(1) -- 每次 CAN RX ISR 都可能被
        // 一整个 dwc2 中断(FIFO 搬运加端点状态机)推迟。设为 2 才让既定的
        // FDCAN(1) > USB(2) > UART/DMA(3) 层级在镜像中成立, 且所有权留在本处,
        // 不会悄然回退。
        HAL_NVIC_SetPriority(OTG_HS_IRQn, 2, 0);

        core::utility::assert_always(tusb_rhport_init(0, nullptr));
    }

    core::protocol::Serializer& serializer() { return serializer_; }

    // 会话保活检查, 独立于 try_transmit()。
    //
    // kSessionLease 为 4 s, 每趟主循环检查一次已比被测对象精细约千倍。放在
    // try_transmit() 顶部则每趟执行九次, 每次都经 Timer::timepoint() 读 TIM5 的
    // CNT -- 在全板最热的循环上做一次 D2 外设访问。唯一调用点在 app.cpp 主循环。
    void poll_session() { refresh_session_state(); }

    // 主机完成 nonce 握手并持有 keepalive 租约后为 true, 即数据确在转发。
    // 区别于仅 USB 枚举 -- 枚举成功不代表有主机在通信。
    bool session_established() const { return session_established_; }

    void deactivate_session() {
        session_established_ = false;
        // 连同会话一并遗忘 EP0 握手。tud_mount_cb 只在重新枚举时触发, 不重新插拔
        // 线缆地更换主机程序不会重新枚举 -- 否则从不做握手的新主机会继承上一台
        // 主机的通行门(实测于 hpm_board: 它径直穿了过去)。
        ep0_handshake_done_ = false;
        // 会话带走它下发过的 PWM/GPIO 输出, 理由见 Gpio::stop_outputs()。
        gpio::gpio->stop_outputs();
    }

    // 由 EP0 kGetInterface 处理器(usb/vendor_control.cpp)置位: 读接口本身就是
    // 握手, 走到这一步的主机已被告知通道数与 CAN 模式, 不可能是 EP0 配置通道出现
    // 之前的旧主机。
    void set_ep0_handshake_done(bool value) { ep0_handshake_done_ = value; }

    void handle_downlink(std::span<const std::byte> buffer, bool finished) {
        deserializer_.feed(buffer);
        if (finished)
            deserializer_.finish_transfer();
    }

    void finish_downlink_transfer() { deserializer_.finish_transfer(); }

    // 检查按代价从低到高排序, 且刻意不刷新会话 -- 见下方 poll_session()。
    //
    // batch 池是普通 RAM; tud_vendor_n_write_available() 读 TinyUSB 的端点状态,
    // 同样是 RAM。没有待发内容时两者都不值得执行, 而 app.cpp 对每个数据源各调一次
    // -- 每趟多次 -- "有没有活"这一测试之前的任何开销都要乘上九倍。
    bool try_transmit() {
        if (!session_established_) {
            return false;
        }

        if (!transmitting_batch_) {
            transmitting_batch_ = transmit_buffer_.pop_batch();
        }
        if (!transmitting_batch_)
            return false;

        if (!tud_vendor_n_write_available(0))
            return false;

        const auto data = transmitting_batch_->data();

        const auto target_size = std::min(data.size() - transmitted_size_, kMaxPacketSize);

        const auto* src = reinterpret_cast<const uint8_t*>(data.data() + transmitted_size_);

        if (target_size) {
            core::utility::assert_debug(tud_vendor_n_write(0, src, target_size) == target_size);
        } else {
            // 为长度恰为端点尺寸整数倍的 batch 收尾。非缓冲 vendor 模式
            // (RX/TX_BUFSIZE == 0)下, TinyUSB 把零长写直接提交到端点成为 ZLP。
            // 返回值在成功与端点占用失败时同为 0, 无从 assert; 上方的
            // write_available() 已确认端点空闲。
            tud_vendor_n_write(0, src, 0);
        }

        transmitted_size_ += target_size;
        if (transmitted_size_ == data.size() && target_size < kMaxPacketSize) {
            transmit_buffer_.release_batch(transmitting_batch_);
            transmitting_batch_ = nullptr;
            transmitted_size_ = 0;
        }

        return true;
    }

private:
    void activate_session(uint32_t nonce) {
        // 新会话取代旧会话时, 旧主机下发的输出不能留给新主机继承。
        gpio::gpio->stop_outputs();
        if (transmitting_batch_) {
            transmit_buffer_.release_batch(transmitting_batch_);
            transmitting_batch_ = nullptr;
            transmitted_size_ = 0;
        }
        transmit_buffer_.clear();

        current_session_nonce_ = nonce;
        last_session_refresh_ = timer::timer->timepoint();
        session_established_ = true;
    }

    bool can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        if (!session_established_)
            return true;
        switch (id) {
        case data::DataId::kCan1: can::can1->handle_downlink(data); return true;
        case data::DataId::kCan2: can::can2->handle_downlink(data); return true;
        case data::DataId::kCan3: can::can3->handle_downlink(data); return true;
        default: return false;
        }
    }

    bool uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        if (!session_established_)
            return true;
        switch (id) {
        case data::DataId::kUart1: uart::uart1->handle_downlink(data); return true;
#ifdef libhcs_APP_RS485_ENABLE
        case data::DataId::kUart2: uart::uart2->handle_downlink(data); return true;
        case data::DataId::kUart3: uart::uart3->handle_downlink(data); return true;
#endif
        case data::DataId::kUart7: uart::uart7->handle_downlink(data); return true;
        case data::DataId::kUart10: uart::uart10->handle_downlink(data); return true;
        default: return false;
        }
    }

    bool uart_config_deserialized_callback(
        core::protocol::FieldId id, const data::UartConfigView& data) override {
        if (!session_established_)
            return true;
        // 已弃用(2026-09-12): 配置移到 EP0, 由 status stage 原生携带板端应答。
        // 本回调返回的 bool 含义是"该字段 id 已识别"而非"操作成功", 因此被除数
        // 求解拒绝的波特率对主机不可见 -- 本板上这个故障模式真实发生过(见
        // AGENTS.md: HAL_RCCEx_GetPeriphCLKFreq() 返回 0, 所有运行时波特率请求被
        // 静默忽略, 同板回环也检测不到)。拒绝而非忽略: 返回 false 会让反序列化器
        // 在本次传输的剩余部分进入丢弃模式, 旧主机以为波特率已生效的假设会在此
        // 显式失败。
        (void)id;
        (void)data;
        return false;
    }

    bool gpio_digital_data_deserialized_callback(
        uint8_t channel_index, const data::GpioDigitalDataView& data) override {
        if (!session_established_)
            return true;
        if (data.timestamp_quarter_us.has_value())
            return false;
        if (channel_index >= spec::mc02::kGpioDescriptors.size())
            return false;
        if (!spec::mc02::kGpioDescriptors[channel_index].supports(
                spec::GpioCapability::kDigitalWrite))
            return false;
        gpio::gpio->handle_digital_write(channel_index, data);
        return true;
    }

    bool gpio_analog_data_deserialized_callback(
        uint8_t channel_index, const data::GpioAnalogDataView& data) override {
        if (!session_established_)
            return true;
        if (channel_index >= spec::mc02::kGpioDescriptors.size())
            return false;
        if (!spec::mc02::kGpioDescriptors[channel_index].supports(
                spec::GpioCapability::kAnalogWrite))
            return false;
        gpio::gpio->handle_analog_write(channel_index, data);
        return true;
    }

    bool gpio_digital_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (!session_established_)
            return true;
        if (channel_index >= spec::mc02::kGpioDescriptors.size())
            return false;
        const auto& gpio = spec::mc02::kGpioDescriptors[channel_index];
        if (!data.supported(gpio))
            return false;
        gpio::gpio->handle_digital_read(channel_index, data);
        return true;
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
            // 主机完成 EP0 接口握手之前静默拒绝。会话协议没有否定应答, 开不了
            // 会话的主机约一秒后自行触发 ack 超时并给出自己的报错; 沉默是本层
            // 唯一能说的话。没有这道门, EP0 通道之前的旧主机会带着线路上已不再
            // 逐帧协商的 CAN 帧类型假设直接开会话。
            if (!ep0_handshake_done_)
                return;

            const bool same_session = session_established_ && data.nonce == current_session_nonce_;

            if (!same_session)
                activate_session(data.nonce);
            else
                last_session_refresh_ = timer::timer->timepoint();

            const auto result = serializer_.write_session_control(
                {.type = data::SessionType::kStartAck, .nonce = data.nonce});
            core::utility::assert_always(
                result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            break;
        }
        case data::SessionType::kKeepalive:
            if (!session_established_ || data.nonce != current_session_nonce_)
                return;

            last_session_refresh_ = timer::timer->timepoint();
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

    // 共享时基。时间锚搭乘会话字段, 正因它必须随会话存亡: 失去主机的板没有理由
    // 继续维护主机日后可能以为仍对齐的时间线。与 keepalive 一样校验 nonce, 并在
    // 同一次交互中应答, 主机无需第二个往返即可取得板端状态。
    void time_anchor_deserialized_callback(const data::TimeAnchorView& data) override {
        if (!session_established_ || data.nonce != current_session_nonce_)
            return;

        sync::timebase::apply_anchor(data.microframe);

        const auto snapshot = sync::timebase::report();

        // 上报"此刻"的配对, 而非最后一次 SOF 时的。
        //
        // snapshot.microframe 是上一次 Start-of-Frame 锁存的计数, full speed 下
        // 落后 0..1 ms -- 均匀分布, 均值 500 us。主机把收到的值与本往返的中点配对,
        // 若上报最后一次 SOF, 它会把该微帧关联到比实际晚约 450 us 的时刻, 并额外
        // 叠加均匀分布的 1 ms 延迟噪声。
        // [实测 2026-09-07, host/examples/mc02_time_sync_test.cpp --causality 的
        //  因果探针: 2185 次探测中 100% 的换算落在其自身 send/reply 括区之外
        //  +440..+491 us, 位置残差恰为 1 ms 均匀分布, sigma 288 us = 1000/sqrt(12)。]
        //
        // 插值到当前时刻只花一次 TIM5 读加一次乘法, microframe_at() 正是为此存在。
        // 时间线无效时回退到锁存值 -- 唯一插值无意义的情形。
        auto reported_microframe = snapshot.microframe;
        auto reported_quarter_us = static_cast<uint32_t>(snapshot.timestamp_quarter_us);
        {
            const auto now_quarter_us =
                static_cast<uint32_t>(timer::timer->timepoint().time_since_epoch().count());
            uint64_t microframe_now = 0;
            if (sync::timebase::microframe_at(now_quarter_us, microframe_now)) {
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

    void error_callback() override {
        // TODO: 经专用错误路径上报 USB 下行反序列化错误。
    }

    void refresh_session_state() {
        if (!session_established_)
            return;

        if (!timer::timer->check_expired(last_session_refresh_, kSessionLease))
            return;

        deactivate_session();
    }

    core::protocol::Deserializer deserializer_{*this};

    InterruptSafeBuffer transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};

    const InterruptSafeBuffer::Batch* transmitting_batch_ = nullptr;
    size_t transmitted_size_ = 0;
    bool session_established_ = false;
    bool ep0_handshake_done_ = false;

    uint32_t current_session_nonce_ = 0;
    timer::Timer::TimePoint last_session_refresh_ = timer::Timer::TimePoint::min();
};

// 置于零等待 DTCM(.dtcm, 开机复制): 转发 ISR 写 serializer/USB batch 缓冲时
// 全程不触碰 AXI 总线。
[[gnu::section(".dtcm")]] inline constinit Vendor::Lazy vendor;

} // namespace libhcs::firmware::usb
