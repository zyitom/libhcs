#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>

#include <class/vendor/vendor_device.h>
#include <device/usbd.h>
#include <main.h>
#include <tusb.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/spec/mc02/ports.hpp"
#include "core/src/link/session.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/common/app/src/link/host_session.hpp"
#include "firmware/common/app/src/usb/interrupt_safe_buffer.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"
#include "firmware/mc02/app/src/buzzer/buzzer.hpp"
#include "firmware/mc02/app/src/can/can.hpp"
#include "firmware/mc02/app/src/gpio/gpio.hpp"
#include "firmware/mc02/app/src/sync/sof.hpp"
#include "firmware/mc02/app/src/sync/sof_capture.hpp"
#include "firmware/mc02/app/src/sync/timebase.hpp"
#include "firmware/mc02/app/src/timer/timer.hpp"
#include "firmware/mc02/app/src/uart/uart.hpp"
#include "firmware/mc02/app/src/usb/helper.hpp"
#include "firmware/mc02/app/src/usb/usb_descriptors.hpp"

namespace libhcs::firmware::usb {

void poll_dfu_runtime_reboot();

// 上行满的板级报告: 无会话时的积压是预期稳态(见 usb/helper.hpp 的
// uplink_session_active), 只有主机正在排空时的满才点亮 LED。
struct UplinkAllocFailed {
    static void report() {
        if (uplink_session_active())
            led::led->uplink_buffer_full();
    }
};

// batch 对齐到 Cortex-M7 D-cache 行(32 B)。所有成员经模板实例化, 修饰名带
// InterruptSafeBuffer 前缀, 链接脚本的 ITCM 规则照常匹配(宿主类 HostSession 的
// 成员名里同样带着它)。
using UpstreamBatches = InterruptSafeBuffer<32, UplinkAllocFailed>;

// 共享时间基准与本板时刻的板级事实(firmware/common/app/src/link/host_session.hpp
// 的策略契约)。本地时刻来自 TIM23(1 MHz, 按 CNT << 2 上报), "此刻"插值读同一
// 定时器后走 timebase::microframe_at()。
struct TimeSync {
    static bool on() { return sync::time_sync_on(); }

    static void apply_anchor(std::uint64_t host_microframe) {
        sync::timebase::apply_anchor(host_microframe);
    }

    static auto report() { return sync::timebase::report(); }

    static bool interpolate_now(
        std::uint32_t& now_quarter_us, std::uint64_t& out_microframe,
        std::uint16_t& out_fraction_q16) {
        now_quarter_us = timer::timer->timepoint().time_since_epoch().count();
        return sync::timebase::microframe_at(now_quarter_us, out_microframe, out_fraction_q16);
    }

    static link::CaptureCounts take_capture_counts() {
        const auto counts = sync::sof_capture::take_counts();
        return {counts.fresh, counts.stale};
    }

    static std::uint32_t now_quarter_us() {
        return timer::timer->timepoint().time_since_epoch().count();
    }
};

// USB vendor class 主机传输: 负责 TinyUSB 启动与传输形态(max-packet 分块 + ZLP
// 收尾)。会话生命周期、下行分发与 kTimeAnchor 应答在共用的
// firmware/common/app/src/link/host_session.hpp(2026-10-06 起与 hpm/c_board 同一份;
// 原先本文件里的 kStart/kKeepalive、租约、锚点应答与批量缓冲成员皆已删去)。
class Vendor : public link::HostSession<UpstreamBatches, TimeSync> {
public:
    using Lazy = utility::Lazy<Vendor>;

    static constexpr size_t kMaxPacketSize = 64;

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

    // 所有 CAN / UART / GPIO 引脚与板载 IMU 回到未声明状态。握手被作废(重新枚举)时也要调。
    // 定义在 vendor.cpp: IMU 的头文件反过来包含本文件。
    static void stop_channels();

    // 由 EP0 kApplyManifest 处理器(usb/vendor_control.cpp)置位, 门的状态见下方
    // ep0_handshake_done_ 成员的注释。
    void set_ep0_handshake_done(bool value) { ep0_handshake_done_ = value; }

    // 检查按代价从低到高排序, 且刻意不刷新会话 -- 租约检查在 poll_session()。
    //
    // batch 池是普通 RAM; tud_vendor_n_write_available() 读 TinyUSB 的端点状态,
    // 同样是 RAM。没有待发内容时两者都不值得执行, 而 app.cpp 对每个数据源各调一次
    // -- 每趟多次 -- "有没有活"这一测试之前的任何开销都要乘上九倍。
    bool try_transmit() {
        const auto* batch = next_batch();
        if (!batch)
            return false;

        if (!tud_vendor_n_write_available(0))
            return false;

        const auto data = batch->data();

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
            finish_batch();
            transmitted_size_ = 0;
        }

        return true;
    }

    bool session_allowed() const noexcept override { return ep0_handshake_done_; }

protected:
    // 会话结束 -- 租约到期、总线复位、挂起或拔线(基类 deactivate_session() 的
    // 下半段): 连同会话一并遗忘 EP0 握手。tud_mount_cb 只在重新枚举时触发, 不重新
    // 插拔线缆地更换主机程序不会重新枚举 -- 否则从不做握手的新主机会继承上一台
    // 主机的通行门(实测于 hpm_board: 它径直穿了过去)。会话带走它声明过的口:
    // 没有主机在用的 CAN / UART 不留在线上, PWM 输出回到低电平(理由见
    // gpio::Pin::suspend)。下一个主机的清单会重新声明自己要的那些
    // (usb/vendor_control.cpp)。时间基准一并关(随会话走, stop_channels())。
    void session_deactivated_callback() override {
        ep0_handshake_done_ = false;
        stop_channels();
    }

    // 新会话: 旧主机下发的输出值不能留给新主机继承。
    void session_activated_callback() override {
        gpio::gpio->zero_outputs();
        buzzer::buzzer_port.silence();
        transmitted_size_ = 0;
    }

    // ---- 下行分发(定义在 vendor.cpp, 经 ports::Registry 分发)。会话门在基类
    // 已把守; 不是本板这一类口的 DataId 才算不认识(返回 false)。没声明(或方向
    // 不符)的口收到的数据由驱动自己丢弃。 ----
    bool dispatch_can(core::protocol::FieldId id, const data::CanDataView& data) override;
    bool dispatch_uart(core::protocol::FieldId id, const data::UartDataView& data) override;
    void report_port_status(core::link::PortStatusRound& round) override;
    bool dispatch_gpio_digital(uint8_t line, const data::GpioDigitalDataView& data) override;
    bool dispatch_gpio_analog(uint8_t line, const data::GpioAnalogDataView& data) override;
    bool dispatch_gpio_read(uint8_t line) override;
    bool dispatch_buzzer(const data::BuzzerToneDataView& data) override;

private:
    size_t transmitted_size_ = 0;
    // EP0 握手门(会话状态本身在 core::link::Session), 由 EP0 kApplyManifest 处理器
    // (usb/vendor_control.cpp)置位: 清单被完整应用即握手, 走到这一步的主机已被告知
    // 通道数与 CAN 模式, 不可能是 EP0 配置通道出现之前的旧主机。会话结束(到期、
    // 总线事件)时一并清掉, 见 session_deactivated_callback()。
    bool ep0_handshake_done_ = false;
};

// 置于零等待 DTCM(.dtcm, 开机复制): 转发 ISR 写 serializer/USB batch 缓冲时
// 全程不触碰 AXI 总线。
[[gnu::section(".dtcm")]] inline constinit Vendor::Lazy vendor;

} // namespace libhcs::firmware::usb
