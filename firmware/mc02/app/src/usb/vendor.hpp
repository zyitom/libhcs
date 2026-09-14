#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>

#include <class/vendor/vendor_device.h>
#include <device/usbd.h>
#include <device/usbd_pvt.h> // usbd_edpt_busy: 审计钩子用, 此 fork 未在 usbd.h 声明
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

    // ---- USB 下行流控(移植自 hpm_board 的同名机制) ----
    //
    // 重新挂载 bulk OUT 端点, 除非 CAN 软件发送队列水位太高、再收一个包也装不下。
    //
    // 为什么要背压: 不加干预时类驱动在 tud_vendor_rx_cb() 返回后立刻重挂端点,
    // 软件队列已满的板子会继续收下只能丢弃的帧 -- 丢帧无声, 主机也永远学不会
    // 放慢。这是本协议唯一的背压手段 -- 线上没有任何流控字段。
    //
    // 与 hpm_board 的关键差异: 那边(ChipIdea)不重挂端点控制器就自动回 NAK; 这边
    // 的 DWC2 实测不会 -- 未挂载的 transfer 之后, 后续包仍被 ACK 进接收 FIFO 然后
    // 在 dcd 层无声丢弃(实测 2026-09-12: 不重挂时主机全速灌 24k 帧/s 零阻塞)。
    // 所以这里必须显式置 DOEPCTL.SNAK 才能真正扣住端点, 恢复时 CNAK 后再挂。
    // 只碰 vendor OUT 这一个端点, EP0 与其余端点不受影响。
    //
    // 为什么不是无损保证: 一个包最多装若干条 CAN 记录, 而进入节流时最多已有
    // RX_XFER_SIZE/64 个包在途。它只约束持续速率, 杜绝不了溢出; 按最坏包深定
    // 水位会把限流压到队列的 1/4 以下, 每次正常突发都损失吞吐。
    //
    // 稳态只是两次 bool 读取加一次返回: rx 完成回调早已重挂端点, 通常既无欠账
    // 也无节流。策略评估缓存给接收回调使用(在回调里逐包评估实测损 2.3% 包率,
    // 见 hpm_board 同名机制), 原则上任何重活都不进接收路径。
    void poll_downlink_arm() {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        // 挂载前不碰端点: SET_CONFIGURATION 之前 ep_out 尚未建立, 而此处的
        // DOEPCTL 写与 USB ISR 的端点配置并发(枚举期 ISR 频繁触发)。
        if (!vendor_mounted_)
            return;
        // 只读主循环钩子缓存的结论, 不在这里遍历 CAN 队列: 本函数运行在接收完成
        // 回调里, 该路径上新增的耗时按 1.2-1.4 倍折损包率。
        if (throttle_active_) {
            arm_pending_ = true;
            vendor_out_nak(true); // 扣住: DWC2 必须显式 SNAK, 不重挂是不够的
            return;
        }
        // 端点暂时挂不上(尚未打开, 或已有一个 transfer 在飞)时保留欠账, 由
        // poll_downlink_arm_if_pending() 重试。首次挂载也由这里完成: manual 模式
        // 把它留给应用, 别处无人负责。
        vendor_out_nak(false); // CNAK 先于重挂, 否则挂上的 transfer 收不到包
        arm_pending_ = !tud_vendor_n_read_xfer(0);
#endif
    }

    // 主循环钩子。重估节流策略(避开接收热路径)、把结论缓存给接收回调, 然后结清
    // 尚未完成的挂载。
    void poll_downlink_arm_if_pending() {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        if (arm_pending_ || throttle_active_ || (++throttle_tick_ & 0xFU) == 0U)
            throttle_active_ = downlink_throttled();
        audit_downlink_arm();
        if (arm_pending_)
            poll_downlink_arm();
#endif
    }

    // 端点重新可用后的初次挂载入口。欠账只由回调设置 -- mount、suspend、会话
    // 拆除。任何不经过这些回调就取消已挂 transfer 的路径都会让板子永久失聪:
    // arm_pending_ 为 false, 钩子不再动作, 而硬件上没有挂任何 transfer。总线
    // 复位会摧毁硬件持有的 transfer, 而上面的逻辑都不会再设置欠账 -- 所以不再
    // 信任欠账, 审计钩子直接查端点(hpm_board 2026-09-03 实测的永久失聪案例)。
    void reset_downlink_arm() {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        arm_pending_ = true;
#endif
    }

    // SET_CONFIGURATION(挂载)/拆除(suspend、umount)时由 TinyUSB 回调设置。
    // 流控的端点寄存器写只发生在挂载窗口内。
    void set_vendor_mounted([[maybe_unused]] bool mounted) {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        vendor_mounted_ = mounted;
#endif
    }

    // 主机完成 nonce 握手并持有 keepalive 租约后为 true, 即数据确在转发。
    // 区别于仅 USB 枚举 -- 枚举成功不代表有主机在通信。
    bool session_established() const { return session_established_; }

    void deactivate_session() {
        session_established_ = false;
        // 连同会话一并遗忘 EP0 握手。tud_mount_cb 只在重新枚举时触发, 不重新插拔
        // 线缆地更换主机程序不会重新枚举 -- 否则从不做握手的新主机会继承上一台
        // 主机的通行门(实测于 hpm_board: 它径直穿了过去)。
        ep0_handshake_done_ = false;
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
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
    // vendor bulk OUT 端点的每端点 NAK(DWC2 DOEPCTL.SNAK/CNAK)。hpm 的
    // ChipIdea 不重挂即 NAK, DWC2 必须显式置位 -- 实测见 poll_downlink_arm。
    // 只作用于 vendor OUT 这一个端点; EP0(配置通道)与其余端点不受影响。
    static void vendor_out_nak(bool nak) {
        // 这套 CMSIS 没有 USB_OTG_HS_Device 便捷宏, 也没有 OUTEP_CFG 数组成员;
        // 按 RM0468 布局由外设基址 + 0x800(设备区) + 0xB00(OUT 端点区) + ep*0x20
        // 构造, 与 HAL 的 USBx_OUTEP(i) 宏同算术。
        auto* out_ep = reinterpret_cast<USB_OTG_OUTEndpointTypeDef*>(
            USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE + 0xB00U
            + (UsbDescriptors::kEpnumVendorDataOut & 0x0FU) * 0x20U);
        if (nak)
            out_ep->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
        else
            out_ep->DOEPCTL |= USB_OTG_DOEPCTL_CNAK; // CNAK 清除 SNAK 状态
    }

    // 迟滞。在队列接近顶部时限流, 真正排空后才解除: 单阈值会让每次出队后的下一
    // 轮就重挂端点, 节流恰好在延迟最差的深度上抖动。队列深度与 hpm_board 同为
    // 64, 水位 1:1 平移。
    static constexpr size_t kThrottleEngageDepth = can::Can::kTransmitQueueSize * 3 / 4;
    static constexpr size_t kThrottleReleaseDepth = can::Can::kTransmitQueueSize / 4;

    // 逃生阀。停止排空的总线(bus-off, 或根本没有其他节点应答)会让 OUT 端点永远
    // 关闭; 而该端点同时承载 UART 下行与会话 keepalive, 一路 CAN 故障就会拖垮
    // 整条链路, 比丢弃发往该总线的帧糟糕得多。队列在释放水位之上停留这么久后,
    // 停止扣住端点。健康总线按实测约 19.8k 帧/s 排空全部 64 槽约需 3.2 ms, 20 ms
    // 只在总线真正卡死时才会耗尽; 会话租期 4000 ms, 余量充分。
    static constexpr std::chrono::milliseconds kThrottleDeadline{20};

    bool downlink_throttled() {
        const size_t depth = can::max_transmit_queue_depth();

        // 本轮已放弃背压, 管道保持开放, 直到总线证明自己又在排空 -- 若在启动
        // 水位重新限流, 只会把同一个 20 ms 停滞循环重演。
        if (throttle_abandoned_) {
            if (depth <= kThrottleReleaseDepth)
                throttle_abandoned_ = false;
            return false;
        }

        if (!throttle_active_) {
            if (depth < kThrottleEngageDepth)
                return false;
            throttle_active_ = true;
            throttle_started_ = timer::timer->timepoint();
            return true;
        }

        if (depth <= kThrottleReleaseDepth) {
            throttle_active_ = false;
            return false;
        }

        if (timer::timer->check_expired(throttle_started_, kThrottleDeadline)) {
            throttle_active_ = false;
            throttle_abandoned_ = true;
            return false;
        }

        return true;
    }

    // 审计: 不再信任欠账, 每 256 轮约 340 us 采样一次端点真实状态。它捕捉的故障
    // 是永久性的, 采样率只需快到人眼无感即可。稳态开销是一次自增加一次掩码 --
    // arm_pending_ 为 false 时短路, 走不到端点查询。
    void audit_downlink_arm() {
        if (arm_pending_ || throttle_active_)
            return;
        if ((++arm_audit_tick_ & 0xFFU) != 0U)
            return;
        if (!usbd_edpt_busy(0, UsbDescriptors::kEpnumVendorDataOut))
            arm_pending_ = true;
    }
#endif

    void activate_session(uint32_t nonce) {
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

#if CFG_TUD_VENDOR_RX_MANUAL_XFER
    // 下行流控状态, 见 public 区同名方法。
    bool vendor_mounted_ = false; // SET_CONFIGURATION 之后才允许碰端点寄存器
    bool arm_pending_ = true;     // manual 模式的初次挂载欠账, 上电即欠
    bool throttle_active_ = false;
    bool throttle_abandoned_ = false;
    uint32_t throttle_tick_ = 0;
    uint32_t arm_audit_tick_ = 0;
    timer::Timer::TimePoint throttle_started_ = timer::Timer::TimePoint::min();
#endif
    uint32_t current_session_nonce_ = 0;
    timer::Timer::TimePoint last_session_refresh_ = timer::Timer::TimePoint::min();
};

// 置于零等待 DTCM(.dtcm, 开机复制): 转发 ISR 写 serializer/USB batch 缓冲时
// 全程不触碰 AXI 总线。
[[gnu::section(".dtcm")]] inline constinit Vendor::Lazy vendor;

} // namespace libhcs::firmware::usb
