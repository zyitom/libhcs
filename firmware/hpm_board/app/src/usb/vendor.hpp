#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include <class/vendor/vendor_device.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <tusb.h>

#include "board_app.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/hpm_board/app/src/dmtool/dm_adapter.hpp"
#include "firmware/hpm_board/app/src/link/host_session.hpp"
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

    // 由 EP0 配置处理器在本主机读过板子的接口描述符后置位(usb/vendor_control.cpp)。
    // 每次拆除时清零, 新主机必须自己做握手, 不能继承上一台主机的状态。
    void set_ep0_handshake_done(bool done) { ep0_handshake_done_ = done; }

    bool session_allowed() const override { return ep0_handshake_done_; }

    void session_deactivated_callback() override {
        ep0_handshake_done_ = false;
        // libhcs 让出板子: DMTool 端点解除隔离(见 dmtool/dm_adapter.hpp)。
        dmtool::on_libhcs_session_end();
    }

    bool try_transmit() {
        // 没有 libhcs 会话的主循环轮次交给 DMTool 仿真: 两种主机互斥使用, 这个
        // 回调只在 next_batch() 本来就有的"会话未建立"分支里执行 -- 会话在时一条
        // 指令也不多(见 dmtool/dm_adapter.hpp)。
        const auto* batch = next_batch([] { dmtool::poll(); });
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

        // libhcs 优先: DMTool 仿真与 CDC 串口桥就此让位(见 dmtool/dm_adapter.hpp)。
        dmtool::on_libhcs_session();
    }

private:
    size_t transmitted_size_ = 0;
    // 端点尺寸(字节), 每次会话激活时刷新。全速值是安全默认: 按 64 字节分块在高速
    // 端点上同样正确, 只是更慢; 反过来则会越过端点容量。
    std::size_t max_packet_size_ = 64;

    // 见 set_ep0_handshake_done()。
    bool ep0_handshake_done_ = false;
};

inline constinit Vendor::Lazy vendor;

} // namespace libhcs::firmware::usb
