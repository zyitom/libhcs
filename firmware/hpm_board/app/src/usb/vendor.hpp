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
#include "core/include/libhcs/data/datas.hpp"
#include "core/src/link/ownership.hpp"
#include "core/src/link/registry.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/common/app/src/link/host_session.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"
#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/diag/can_diag.hpp"
#include "firmware/hpm_board/app/src/dmtool/dm_adapter.hpp"
#include "firmware/hpm_board/app/src/led/led.hpp"
#include "firmware/hpm_board/app/src/link/uplink.hpp"
#include "firmware/hpm_board/app/src/sync/pulse.hpp"
#include "firmware/hpm_board/app/src/sync/sof.hpp"
#include "firmware/hpm_board/app/src/sync/sof_capture.hpp"
#include "firmware/hpm_board/app/src/sync/timebase.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"
#include "firmware/hpm_board/app/src/uart/uart.hpp"
#include "firmware/hpm_board/app/src/usb/usb_descriptors.hpp"
#include "ports.hpp"

namespace libhcs::firmware::usb {

void poll_dfu_runtime_reboot();

} // namespace libhcs::firmware::usb

namespace libhcs::firmware::link {

// 上行满的板级报告: LED 只在有主机排空时点 -- 无会话的积压是预期稳态(见公共头的
// 判据; 旧版本无条件报, 与 mc02 漂移, 统一时取门控版); diag 的分配失败计数无条件
// 记, 它是工程遥测, 恰恰要覆盖无会话时段。
//
// 必须在 link 命名空间: hpm5321 链接脚本的 ILM tripwire 按修饰名点名
// InterruptSafeBuffer<32, link::UplinkAllocFailed>::allocate, 改名/改命名空间
// 都会让 ASSERT 失败(app_flash_uf2.ld 第 114 行)。
struct UplinkAllocFailed {
    static void report() {
        if (uplink_enabled())
            led::led->uplink_buffer_full();
        diag::note_alloc_fail();
    }
};

} // namespace libhcs::firmware::link

namespace libhcs::firmware::usb {

// batch 对齐到 L1 缓存行。所有成员经模板实例化, 修饰名带
// libhcs::firmware::usb::InterruptSafeBuffer 前缀, hpm5321 链接脚本的 ILM 规则
// 按此前缀匹配(2026-10-05 起与本模板同名同命名空间; 2026-10-06 起宿主类
// HostSession 的成员名里同样带着它)。
using UpstreamBatches = usb::InterruptSafeBuffer<HPM_L1C_CACHELINE_SIZE, link::UplinkAllocFailed>;

// 共享时间基准与本板时刻的板级事实(firmware/common/app/src/link/host_session.hpp
// 的策略契约)。hpm 的本地时刻来自 64 位 MTIME, "此刻"插值由 timebase 自带的
// microframe_now() 完成 -- 它读的是同一个时钟。
struct TimeSync {
    static bool on() { return sync::time_sync_on(); }

    static void apply_anchor(std::uint64_t host_microframe) {
        sync::timebase::apply_anchor(host_microframe);
    }

    static auto report() { return sync::timebase::report(); }

    static bool interpolate_now(
        std::uint32_t& now_quarter_us, std::uint64_t& out_microframe,
        std::uint16_t& out_fraction_q16) {
        return sync::timebase::microframe_now(out_microframe, out_fraction_q16, now_quarter_us);
    }

    static link::CaptureCounts take_capture_counts() {
        const auto counts = sync::sof_capture::take_counts();
        return {counts.fresh, counts.stale};
    }

    static std::uint32_t now_quarter_us() {
        return static_cast<std::uint32_t>(timer::Timer::timestamp64_quarter_us());
    }
};

// 板子归属的交接(core/src/link/ownership.hpp)。接手时附加功能先停, 此后不会再有
// DMTool 的帧或串口桥的字节进 CAN / UART; 再把端口全部挂起(core 的 PortHandoff), 清单
// 才从干净状态开始动端口。交还时反过来: 端口先全部恢复(附加功能不走 EP0, 通道必须是
// 通的, 与上电状态相同), 附加功能再接手 -- CDC 桥这时按它记着的线路编码写回 UART。
// 时间基准排在最后: 交还时它最先关(倒序), 接手时不动(开不开由清单决定)。
using BoardOwnership = core::link::Ownership<
    dmtool::Handoff, core::link::PortHandoff<ports::Registry>, sync::TimeSyncHandoff>;

// USB vendor class 主机传输: 负责 TinyUSB 启动与传输形态(max-packet 分块 + ZLP
// 收尾)。会话生命周期、下行分发与 kTimeAnchor 应答在共用的
// firmware/common/app/src/link/host_session.hpp。
class Vendor : public link::HostSession<UpstreamBatches, TimeSync> {
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
    // 可以重新同步。基类同名方法的两参形态与本板 rx 回调一致, 这里只转一笔。
    void handle_downlink(std::span<const std::byte> buffer, bool finished) {
        HostSession::handle_downlink(buffer, finished);
    }

    // ---- 板子的归属(core/src/link/ownership.hpp) ----
    //
    // 交接是冷路径, 定义在 vendor.cpp: 内联进主循环的话, 两份交还路径(恢复全部 CAN /
    // UART)会摊进 App::run 的寄存器分配里。

    // 清单事务的归属半边(link::ep0 的 kApplyManifest): 校验通过后接手, 全部生效后
    // 落定, 中途失败则还给接手前的主人。
    void begin_claim();
    void commit_claim(uint64_t now);
    void abort_claim();

    // 主机走了: 总线挂起、重新枚举或拔线。会话作废, 板子交还附加功能; 下一个主机必须
    // 自己握手, 不能继承上一个主机的状态。
    void drop_host();

    // 握手之后一个租约内没开出会话, 板子交还附加功能。主循环按 1 kHz 调用。
    void poll_ownership();

    // 会话结束, 板子交还附加功能。
    void release_ownership();

    // 把 time base 积压的硬件脉冲捕获排上上行。主循环调用; 没有真的捕获到探针帧
    // 时是空操作。换算需要历史环和一次除法, 放在主循环而非捕获中断里做。
    void poll_pulse_captures() {
        if constexpr (!sync::pulse::kEnabled)
            return;
        if (!session().established())
            return;
        uint64_t captured_q16 = 0;
        while (sync::pulse::take_capture(captured_q16)) {
            (void)serializer().write_pulse_report({
                .nonce = session().nonce(),
                .scheduled_microframe = pending_pulse_microframe_,
                .captured_microframe_q16 = captured_q16,
                .ticks_per_microframe_q16 = sync::pulse::measured_ticks_per_microframe_q16(),
                .flags = static_cast<uint8_t>(
                    (pending_pulse_armed_ ? data::kPulseArmed : data::PulseReportFlags{})
                    | data::kPulseCaptured),
            });
        }
    }

    bool session_allowed() const noexcept override { return ownership_.session_allowed(); }

    // 虚函数留在类内: 在 vendor.cpp 里定义会让它成为 key function, 整张虚表连同
    // HostSession 那几个放 .fast 的内联回调都落进 vendor.cpp, 与那里的 .fast 段冲突。
    void session_deactivated_callback() override { release_ownership(); }

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

        // 附加功能在握手时就已让位, 这里只把板子从"等会话"交给会话租约看守。
        ownership_.on_session_started();
    }

    // ---- 下行分发(hpm 的口集合: CAN 与 UART 走数组, 没有 GPIO/蜂鸣器) ----

    ATTR_PLACE_AT(".fast")
    bool dispatch_can(core::protocol::FieldId id, const data::CanDataView& data) override {
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

    ATTR_PLACE_AT(".fast")
    bool dispatch_uart(core::protocol::FieldId id, const data::UartDataView& data) override {
        for (auto& board_uart : uart::uart_array) {
            if (static_cast<core::protocol::FieldId>(board_uart->data_id()) == id) {
                board_uart->handle_downlink(data);
                return true;
            }
        }
        return false;
    }

    // 硬件脉冲交换。主机向每块板发相同的目标 microframe; 各板在该处触发并
    // 捕获其他板的脉冲。无论是否 armed 都应答每一条 schedule: 放不上目标又不
    // 作声的板子, 在主机看来与"触发了但没听到回声"无法区分 -- 这两种失败需要
    // 相反的修法。
    void report_port_status(core::link::PortStatusRound& round) override {
        core::link::offer_port_status<ports::Registry>(round);
    }

    void dispatch_pulse_schedule(const data::PulseScheduleView& data) override {
        pending_pulse_microframe_ = data.microframe;
        pending_pulse_armed_ = sync::pulse::schedule(data.microframe);

        (void)serializer().write_pulse_report({
            .nonce = session().nonce(),
            .scheduled_microframe = data.microframe,
            .captured_microframe_q16 = 0,
            .ticks_per_microframe_q16 = sync::pulse::measured_ticks_per_microframe_q16(),
            .flags = static_cast<uint8_t>(
                pending_pulse_armed_ ? data::kPulseArmed : data::PulseReportFlags{}),
        });
    }

private:
    size_t transmitted_size_ = 0;
    // 端点尺寸(字节), 每次会话激活时刷新。全速值是安全默认: 按 64 字节分块在高速
    // 端点上同样正确, 只是更慢; 反过来则会越过端点容量。
    std::size_t max_packet_size_ = 64;

    uint64_t pending_pulse_microframe_ = 0;
    bool pending_pulse_armed_ = false;

    // 握手的时限与会话租约同长。正常路径上主机握手后几毫秒就开会话; 开不出来时它每次
    // 重试前都会重新握手(host 的 keepalive_loop 先跑 before-session hook), 时限到了
    // 交还附加功能不会让 libhcs 少一次机会。
    BoardOwnership ownership_{kSessionLeaseQuarterUs};
};

inline constinit Vendor::Lazy vendor;

} // namespace libhcs::firmware::usb
