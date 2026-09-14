#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#if defined(libhcs_APP_USB_FULL_SPEED) && libhcs_APP_USB_FULL_SPEED
# include <hpm_soc.h>
#endif

#include <class/vendor/vendor_device.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <device/usbd_pvt.h>
#include <tusb.h>

#include "board_app.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/link/host_session.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"
#include "firmware/hpm_board/app/src/usb/usb_descriptors.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::usb {

void poll_dfu_runtime_reboot();

// USB vendor class 主机传输: 负责 TinyUSB 启动与传输形态(max-packet 分块 + ZLP
// 收尾)。会话生命周期与下行分发在共用的 link::HostSession 中。
class Vendor : public link::HostSession {
public:
    using Lazy = utility::Lazy<Vendor>;

    Vendor() {
        usb::usb_descriptors.init();

        const tusb_rhport_init_t init_config{
            .role = TUSB_ROLE_DEVICE,
            .speed = board::usb_use_high_speed() ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL,
        };
        core::utility::assert_always(tusb_rhport_init(0, &init_config));

#if defined(libhcs_APP_USB_FULL_SPEED) && libhcs_APP_USB_FULL_SPEED
        // 上面的 speed 字段只是建议值: ChipIdea 设备驱动只报告端口实际协商出的
        // 速率, 从不强制(见 dcd_ci_hs.c, 它只读 PORTSC1_PORT_SPEED)。强制全速靠
        // 控制器位: PFSC 关掉高速 chirp, 端口便只能以 12 Mbit 起来。必须放在
        // tusb_rhport_init 之后设置, dcd_init 会复位控制器并清掉它。[RM: PORTSC1.PFSC]
        HPM_USB0->PORTSC1 |= USB_PORTSC1_PFSC_MASK;
#endif

        // tusb_rhport_init -> dcd_init 已使能 USB 中断; 这里显式钉住优先级, 让
        // CAN(3) > USB(2) > UART(1) 的抢占层级归本文件所有, SDK 默认值变化时不会
        // 悄悄退化。抢占式中断开启时, CAN RX(3) 可打断运行中的 USB ISR(2), 电机
        // 反馈不必排在 USB ISR 的尾部之后。
        intc_m_enable_irq_with_priority(IRQn_USB0, 2);
    }

    // USB 传输有边界(不同于 EtherCAT PD 流): 短包即结束一次传输, 帧定界恢复因此
    // 可以重新同步。
    void handle_downlink(std::span<const std::byte> buffer, bool finished) {
        link::HostSession::handle_downlink(buffer);
        if (finished)
            finish_downlink_transfer();
    }

    // 重新挂载 bulk OUT 端点, 除非 CAN 发送队列水位太高、再收一个包也装不下。
    // 主循环每轮调用一次。
    //
    // 为什么要限流: 不加干预时类驱动在 tud_vendor_rx_cb() 返回后立刻重挂端点,
    // 软件队列已满的板子会继续收下只能丢弃的帧 -- 丢帧无声, 主机也永远学不会放慢。
    // 不挂端点则控制器以 NAK 应答, 拖住主机在飞的 URB, 其 64 个 transfer 池耗尽后
    // 发送方便阻塞在 acquire_transmit_buffer() 里。这是本协议唯一的背压手段 --
    // 线上没有任何流控字段。
    //
    // 为什么不是无损保证: 一个 512 字节 OUT 包最多装约 46 条 8 字节 CAN 记录
    // (每条 11 字节)或约 170 条 DLC-0 记录, 而队列只有 64 深 -- 且队列开始积压时
    // 最多已有 64 个包在途。它只约束持续速率, 杜绝不了溢出; 按最坏包深定水位会把
    // 限流压到队列的 1/4 以下, 每次正常突发都损失吞吐。
    // CFG_TUD_VENDOR_RX_MANUAL_XFER=0 时本段整体编译消失, 类驱动自行重挂端点,
    // 无背压。保留两条路径让它成为一个宏即可切换的 A/B, 而不是回滚提交 -- 否则
    // 根本无从测量它对包率的代价。
    void poll_downlink_arm() {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        // 只读主循环钩子缓存的结论, 不在这里遍历 CAN 队列: 本函数运行在接收完成
        // 回调里, 该路径上新增的耗时按 1.2-1.4 倍折损包率 -- 逐包评估策略实测白损
        // 2.3%。
        if (throttle_active_) {
            arm_pending_ = true;
            return;
        }
        // 端点暂时挂不上(尚未打开, 或已有一个 transfer 在飞)时保留欠账, 由下面的
        // 主循环钩子重试。首次挂载也由这里完成: manual 模式把它留给应用, 别处无人
        // 负责。
        arm_pending_ = !tud_vendor_n_read_xfer(0);
#endif
    }

    // 主循环钩子。重估节流策略(避开热路径)、把结论缓存给上面的接收回调用, 然后
    // 结清仍未完成的挂载。
    //
    // 稳态只是两次 bool 读取加一次返回: rx 完成回调早已重挂端点, 通常既无欠账也
    // 无节流。策略只在节流已生效(以便解除)、有欠账或粗粒度 tick 到点(每 16 轮约
    // 21 us; 队列排空需毫秒级、水位有 16 槽余量, 不会漏过任何状态变化)时重估。
    // 每轮都评估意味着每秒约 77 万次队列遍历, 实测白损 2.3% 包率。仍必须落到主
    // 循环的只有两件事: 首包到来前的初次挂载, 以及被节流队列排空后的解除 --
    // 两者都没有完成回调可以搭乘。
    void poll_downlink_arm_if_pending() {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        if (arm_pending_ || throttle_active_ || (++throttle_tick_ & 0xFU) == 0U)
            throttle_active_ = downlink_throttled();
        audit_downlink_arm();
        if (arm_pending_)
            poll_downlink_arm();
#endif
    }

    // 上述欠账是端点唯一的重挂来源, 且只由回调设置 -- mount、suspend、会话拆除。
    // 任何不经过这些回调就取消已挂 transfer 的路径都会让板子永久失聪: arm_pending_
    // 为 false, 钩子不再动作, 而硬件上没有挂任何 transfer。这并非假想, TinyUSB
    // 自己的 vendord_set_itf() 就是如此 -- 它 STALL 并清除两个 bulk 端点, 其后的
    // 自动重挂在 `#if CFG_TUD_VENDOR_RX_MANUAL_XFER == 0` 内, 本构建把它编译掉了。
    //
    // 所以不再信任欠账, 直接查端点。每 256 轮约 340 us 采样一次; 它捕捉的故障是
    // 永久性的, 采样率只需快到人眼无感即可。稳态开销是一次自增加一次掩码 --
    // arm_pending_ 为 false 时短路, 走不到端点查询。
    void audit_downlink_arm() {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        if (arm_pending_ || throttle_active_)
            return;
        if ((++arm_audit_tick_ & 0xFFU) != 0U)
            return;
        if (!usbd_edpt_busy(0, UsbDescriptors::kEpnumCdc0DataOut))
            arm_pending_ = true;
#endif
    }

    // 端点被拆除后重新欠下初次挂载。manual 传输模式把首次挂载留给应用, 而稳态下
    // 无欠账 -- rx 完成回调已重挂端点, arm_pending_ 为 false。总线复位会摧毁硬件
    // 持有的 transfer, 而上面的逻辑都不会再设置欠账: 钩子被一个已不存在的欠账
    // 门控, 板子于是发送正常却再也收不到一个字节, 直到 MCU 重启。
    // 实测 2026-09-03: 主机在板子仍由 VBUS 供电时重启, 两个 bulk OUT 端点对每个
    // 字节都 NAK, EP0 控制传输却完好, 所有会话都死在 SESSION_ACK 上。
    //
    // 任意上下文设置欠账都安全: tud_vendor_n_read_xfer() 拒绝时 poll_downlink_arm()
    // 会保留欠账, 主循环重试到端点重新打开为止。
    // 由 EP0 配置处理器在本主机读过板子的接口描述符后置位(usb/vendor_control.cpp)。
    // 每次拆除时清零, 新主机必须自己做握手, 不能继承上一台主机的状态。
    void set_ep0_handshake_done(bool done) { ep0_handshake_done_ = done; }

    bool session_allowed() const override { return ep0_handshake_done_; }

    void session_deactivated_callback() override { ep0_handshake_done_ = false; }

    void reset_downlink_arm() {
#if CFG_TUD_VENDOR_RX_MANUAL_XFER
        arm_pending_ = true;
#endif
    }

    bool try_transmit() {
        const auto* batch = next_batch();
        if (!batch)
            return false;

        if (!tud_vendor_n_write_available(0))
            return false;

        const auto data = batch->data();

        const std::size_t max_packet_size = max_packet_size_;
        const auto target_size = std::min(data.size() - transmitted_size_, max_packet_size);

        if (target_size) {
            const auto* src = reinterpret_cast<const uint8_t*>(data.data() + transmitted_size_);
            core::utility::assert_debug(tud_vendor_n_write(0, src, target_size) == target_size);
        } else {
            // TinyUSB 0.21 direct mode 经普通写 API 提交 ZLP。返回值是传输长度,
            // 成功 ZLP 与出错同为 0; 上面的 write_available() 已证明端点已挂载且
            // 未被占用。
            static constexpr uint8_t kZlpByte = 0;
            (void)tud_vendor_n_write(0, &kZlpByte, 0);
        }

        transmitted_size_ += target_size;
        if (transmitted_size_ == data.size() && target_size < max_packet_size) {
            finish_batch();
            transmitted_size_ = 0;
        }

        return true;
    }

protected:
    void session_activated_callback() override {
        // 为发送路径缓存端点尺寸。tud_speed_get() 是真实调用 -- 位于 usbd.c, 跨
        // TU 边界不会内联, 而 app.cpp 每轮对每个流量源调用一次本函数, 逐次读取
        // 就是在单字节加载外包一圈 jal/ret。
        //
        // 在此缓存是安全的: 速率由枚举固定, 且枚举完成前不可能存在会话 -- 主机
        // 必须先经这个端点连上板子才能开会话。重新协商意味着总线复位, 会话被丢弃
        // 后会再次执行到这里。
        max_packet_size_ = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512U : 64U;

        transmitted_size_ = 0;
        // CAN 管道的批缓冲池是独立的, HostSession 自身的 reset 够不到这里: 残留
        // 批次会被发进新会话, 对着错误的流解码。
    }

private:
    // 迟滞。在队列接近顶部时限流, 真正排空后才解除: 单阈值会让每次出队后的下一轮
    // 就重挂端点, 节流恰好在延迟最差的深度上抖动。
    static constexpr size_t kThrottleEngageDepth = can::Can::kTransmitQueueSize * 3 / 4;
    static constexpr size_t kThrottleReleaseDepth = can::Can::kTransmitQueueSize / 4;

    // 逃生阀。停止排空的总线(bus-off, 或根本没有其他节点应答)会让 OUT 端点永远
    // 关闭; 而该端点同时承载 UART 下行与会话 keepalive, 一路 CAN 故障就会拖垮整条
    // 链路, 比丢弃发往该总线的帧糟糕得多。队列在释放水位之上停留这么久后, 停止
    // 扣住端点。健康总线按实测约 19.8k 帧/s 排空全部 64 槽约需 3.2 ms, 20 ms 只在
    // 总线真正卡死时才会耗尽; 会话租期 1000 ms, 余量充分。
    static constexpr uint64_t kThrottleDeadlineQuarterUs = 20'000ULL * 4U;

    bool downlink_throttled() {
        const size_t depth = can::max_transmit_queue_depth();

        // 本轮已放弃背压, 管道保持开放, 直到总线证明自己又在排空 -- 若在启动水位
        // 重新限流, 只会把同一个 20 ms 停滞循环重演。
        if (throttle_abandoned_) {
            if (depth <= kThrottleReleaseDepth)
                throttle_abandoned_ = false;
            return false;
        }

        if (!throttling_) {
            if (depth < kThrottleEngageDepth)
                return false;
            throttling_ = true;
            throttle_started_ = timer::Timer::timestamp64_quarter_us();
            return true;
        }

        if (depth <= kThrottleReleaseDepth) {
            throttling_ = false;
            return false;
        }

        if (timer::Timer::timestamp64_quarter_us() - throttle_started_
            >= kThrottleDeadlineQuarterUs) {
            throttling_ = false;
            throttle_abandoned_ = true;
            return false;
        }

        return true;
    }

    size_t transmitted_size_ = 0;
    // 端点尺寸(字节), 每次会话激活时刷新。全速值是安全默认: 按 64 字节分块在高速
    // 端点上同样正确, 只是更慢; 反过来则会越过端点容量。
    std::size_t max_packet_size_ = 64;

    // 初始为 true: 任何包到来之前端点必须先挂载一次, 而只有主循环钩子能做这件事。
    bool arm_pending_ = true;

    // 见 set_ep0_handshake_done()。
    bool ep0_handshake_done_ = false;

    // downlink_throttled() 的缓存结论, 每轮主循环刷新一次。
    bool throttle_active_ = false;
    uint32_t throttle_tick_ = 0;

    // 刻意与 throttle_tick_ 分开: 后者只在未触发过重估的轮次上推进, 不是时钟。
    // 见 audit_downlink_arm()。
    uint32_t arm_audit_tick_ = 0;
    bool throttling_ = false;
    bool throttle_abandoned_ = false;
    uint64_t throttle_started_ = 0;
};

inline constinit Vendor::Lazy vendor;

} // namespace libhcs::firmware::usb
