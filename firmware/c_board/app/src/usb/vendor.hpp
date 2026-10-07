#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>

#include <class/vendor/vendor_device.h>
#include <tusb.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/spec/c_board/ports.hpp"
#include "core/src/link/session.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/buzzer/buzzer.hpp"
#include "firmware/c_board/app/src/can/can.hpp"
#include "firmware/c_board/app/src/gpio/gpio.hpp"
#include "firmware/c_board/app/src/sync/sof.hpp"
#include "firmware/c_board/app/src/sync/timebase.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/c_board/app/src/uart/uart.hpp"
#include "firmware/c_board/app/src/usb/helper.hpp"
#include "firmware/c_board/app/src/usb/usb_descriptors.hpp"
#include "firmware/common/app/src/link/host_session.hpp"
#include "firmware/common/app/src/usb/interrupt_safe_buffer.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"

namespace libhcs::firmware::usb {

void poll_dfu_runtime_reboot();

// 上行满的板级报告: 无会话时的积压是预期稳态(见 usb/helper.hpp 的
// uplink_session_active), 只有主机正在排空时的满才点亮 LED。旧版本无条件报,
// 与 mc02 的会话门控漂移, 统一时取门控版。
struct UplinkAllocFailed {
    static void report() {
        if (uplink_session_active())
            led::led->uplink_buffer_full();
    }
};

// batch 对齐 alignof(size_t): M4 没有缓存, 不需要缓存行对齐。
using UpstreamBatches = InterruptSafeBuffer<alignof(size_t), UplinkAllocFailed>;

// 共享时间基准与本板时刻的板级事实(firmware/common/app/src/link/host_session.hpp
// 的策略契约)。本地时刻来自 TIM2(4 MHz, 按 quarter-us 上报), "此刻"插值读同一
// 定时器后走 timebase::microframe_at()。捕获计数是 SOF 捕获(TIM2 ITR1 触发 DMA 抄
// TIM7, sync/sof.cpp)的新鲜/过时次数; bxCAN 无帧时间戳, 没有 CAN 那一侧。
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
        const auto counts = sync::take_capture_counts();
        return {counts.fresh, counts.stale};
    }

    static std::uint32_t now_quarter_us() {
        return timer::timer->timepoint().time_since_epoch().count();
    }
};

// USB vendor class 主机传输: 负责 TinyUSB 启动与传输形态(max-packet 分块 + ZLP
// 收尾)。会话生命周期、下行分发与 kTimeAnchor 应答在共用的
// firmware/common/app/src/link/host_session.hpp(2026-10-06 起与 hpm/mc02 同一份;
// 原先本文件里的 kStart/kKeepalive、租约、锚点应答与批量缓冲成员皆已删去)。
class Vendor : public link::HostSession<UpstreamBatches, TimeSync> {
public:
    using Lazy = utility::Lazy<Vendor>;

    static constexpr size_t kMaxPacketSize = 64;

    Vendor() {
        usb::usb_descriptors.init();
        core::utility::assert_always(tusb_rhport_init(0, nullptr));
    }

    // EP0 清单声明: 主机的 kApplyManifest 被完整应用即置位, 之后才允许开会话。
    // EP0 处理器(usb/vendor_control.cpp)在应答之后调用它。会话结束(到期、总线
    // 事件)时由 session_deactivated_callback() 一并清掉。
    void set_ep0_handshake_done(bool done) { ep0_handshake_done_ = done; }

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
            // 批长恰好是端点尺寸整数倍时补一个终结包。非缓冲 vendor 模式
            // (RX/TX_BUFSIZE == 0)下 TinyUSB 把零长写直接作为 ZLP 提交到端点。
            // 成功与认领端点失败的返回值都是 0, 没有可断言的东西; 上面的
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
    // 主机的通行门(实测于 hpm_board: 它径直穿了过去)。与 mc02 同一约定。每个口
    // 由自己的 suspend() 停 -- 引脚拉低、输入停采样(见 gpio::Pin::suspend); 本板
    // CAN、UART 与 IMU 的 suspend() 是空操作, 继续跑。口集合就是注册表, 不在这里
    // 再列一遍。定义在 vendor.cpp(ports.hpp 反向包含本文件, 不能进头文件)。
    void session_deactivated_callback() override;

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
    // EP0 握手门(会话状态本身在 core::link::Session), 由 usb/vendor_control.cpp
    // 置位, 会话结束与总线事件时清掉。见 session_control_deserialized_callback()
    // 的说明(基类)。
    bool ep0_handshake_done_ = false;
};

// 放在零等待的 CCM(.ccmram, 上电时由 App::App() 拷入), 上行序列化器与 USB 批缓冲
// 写入时不碰 AHB 总线、不与 DMA 争用。FS USB 是 CPU PIO(片内无 DMA), 所以这里是
// 纯 CPU 的。
[[gnu::section(".ccmram")]] inline constinit Vendor::Lazy vendor;

} // namespace libhcs::firmware::usb
