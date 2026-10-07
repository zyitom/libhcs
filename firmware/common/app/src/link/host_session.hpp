#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/link/downlink_errors.hpp"
#include "core/src/link/port_status.hpp"
#include "core/src/link/session.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/common/app/src/usb/interrupt_safe_buffer.hpp"

namespace libhcs::firmware::link {

// 硬件 SOF 捕获计数(kTimeStatus 的 fresh/stale 两字段)。命名空间作用域: 各板的
// TimeSync 策略要先于 HostSession 实例化返回它。
struct CaptureCounts {
    std::uint16_t fresh;
    std::uint16_t stale;
};

// 本应用所有主机传输共用的主机会话端点: 会话协议胶水(kStart 握手、keepalive 租约、
// 到期与断联的下线动作、kTimeAnchor 应答、下行分发的会话门)加上上行 serializer 及
// 其中断安全批量缓冲。原先这套胶水在 hpm/mc02/c_board 各写一份(细节已漂移过一轮,
// 2026-10-05 先把纯状态机收进 core::link::Session, 2026-10-06 把胶水收进本模板),
// 各板的 Vendor 只剩"板子才有"的部分:
//
//   - 传输形态(try_transmit 的分块与 ZLP、端点尺寸、与 libhcs 互斥的附加功能);
//   - 时间与同步从哪个外设来: TimeSync 策略(见下);
//   - 下行数据到哪个驱动: dispatch_* 虚函数;
//   - 运行时状态从哪些口来: report_port_status()(一行 core::link::offer_port_status);
//   - 门从哪来: session_allowed()(mc02/c_board 的 EP0 握手标志、hpm 的归属状态);
//   - 会话开/关时板子各自清什么、停什么: session_activated/deactivated_callback。
//
// ---- 两个模板参数 ----
//
// UpstreamBatchesT: InterruptSafeBuffer 的板级实例(对齐与上行满的报告策略是板级
// 事实)。作为模板参数留在修饰名里, 缓存芯片链接脚本的 ILM/DTCM 规则按
// InterruptSafeBuffer 前缀匹配照常命中。
//
// TimeSync: 共享时间基准与本板时刻的静态策略, 全是"板子才有"的外设事实, 因此
// 基类不包含任何板级头文件、只经它调用:
//
//   static bool on();                       // sync::time_sync_on()
//   static void apply_anchor(uint64_t);     // sync::timebase::apply_anchor()
//   static auto report();                   // sync::timebase::report()
//   static bool interpolate_now(            // "此刻"在微帧轴上的位置(含小数)。
//       uint32_t& now_quarter_us,           // hpm 经 timebase::microframe_now();
//       uint64_t& out_microframe,           // mc02/c_board 读板级定时器后走
//       uint16_t& out_fraction_q16);        // timebase::microframe_at()。
//   static CaptureCounts take_capture_counts(); // 硬件 SOF 捕获计数, 无捕获路径
//                                               // 的板(如 c_board)恒 {0, 0}。
//   static uint32_t now_quarter_us();       // 1/4 us 的 uint32_t, 按 2^32 回绕。
template <typename UpstreamBatchesT, typename TimeSync>
class HostSession
    : private core::protocol::DeserializeCallback
    , private core::utility::Immovable {
public:
    // 会话租约: core::link::Session::kLeaseTicks(4 s, 1/4 us tick), 同时是清单
    // 接手后等主机开会话的时限。租期必须大于主机轮次(250 ms), 否则轮次落在到期
    // 边界上, 板子会静默丢弃 keepalive [实测 2026-09-13, 1 Hz 轮次下 ~70% 轮次
    // 超时, 根因即租期与轮次同长]。
    static constexpr uint64_t kSessionLeaseQuarterUs = core::link::Session::kLeaseTicks;

    HostSession() = default;

    core::protocol::Serializer& serializer() { return serializer_; }

    // 主机会话握手建立且数据确实在转发后为 true -- 区别于单纯的传输连通
    // (USB 枚举)。
    bool session_established() const { return session_.established(); }

    void handle_downlink(std::span<const std::byte> buffer, bool finished) {
        deserializer_.feed(buffer);
        if (finished)
            finish_downlink_transfer();
    }

    // 一次下行传输的边界。先递增账本的传输计数再恢复解析: 其间报出的错误按
    // "第 (N + 1) 次传输"记在链路状态(data::LinkStatusView)上。
    void finish_downlink_transfer() {
        downlink_errors_.end_transfer();
        deserializer_.finish_transfer();
    }

    // 会话租约检查, 主循环每毫秒调用一次(时间源读一次外设, 所以不放在每趟主循环
    // 都走的发送路径上)。到期时走 deactivate_session()。
    void poll_session() {
        if (session_.poll(TimeSync::now_quarter_us()))
            deactivate_session();
    }

    // 会话到此为止(租约到期、总线复位、挂起、拔线)。带外握手门控的传输层在
    // session_deactivated_callback() 里忘记握手, 下一个主机必须自己做:
    // tud_mount_cb 只在重新枚举时触发, 不拔线换主机程序不会重新枚举。没有这一步,
    // 不握手的主机会继承上一主机的门 -- 实测过, 直接穿门而过。
    void deactivate_session() {
        session_.end();
        session_deactivated_callback();
    }

protected:
    using Batch = typename UpstreamBatchesT::Batch;

    // 该传输层现在是否接受 kStart。USB 在清单握手完成前拒绝, 使没做过握手的主机
    // 无法开会话后按板子不认可的假设行动 -- 具体地, 帧类型移到 EP0 之前的旧主机
    // 会假定线上已不再协商的逐帧 classic/FD 选择, 在期望经典帧的地方收到 FD 帧,
    // 且没有任何地方报告该错配。门的状态各板来源不同(EP0 握手标志 / 归属状态机),
    // 故为纯虚: 会话状态机自己不持有门。
    [[nodiscard]] virtual bool session_allowed() const noexcept = 0;

    // 新 nonce 的 kStart 重置了流: 在途批量与所有待发批量刚被丢弃。传输层应
    // 重置传输进度。
    virtual void session_activated_callback() {}

    // 会话结束 -- 租约到期、总线复位或拔线。忘门、停口、交还归属都在这里。
    virtual void session_deactivated_callback() {}

    // 会话/租约时刻, 以及 kTimeAnchor 应答。1/4 us 的 uint32_t, 按 2^32 回绕。
    [[nodiscard]] std::uint32_t now_quarter_us() const { return TimeSync::now_quarter_us(); }

    core::link::Session& session() { return session_; }

    // ---- 传输层发送路径 ----

    // 第 1 步: 交出当前在途的批量 (没有就弹出下一个待发的)。会话未建立或无待发
    // 时返回 nullptr。租约由 poll_session() 另行检查。批量在 finish_batch() 之前
    // 一直是"当前"的, 部分传输 (USB 分包) 与整批重试 (环背压) 都能从断点继续。
    //
    // without_session 只在会话未建立的那条分支里调用: 给与 libhcs 互斥的主机
    // (DMTool 仿真)一个主循环槽位, 会话在时它不占一条指令。没有互斥主机的板
    // 用无参重载。
    const Batch* next_batch() {
        return next_batch([] {});
    }

    template <std::invocable WithoutSession>
    const Batch* next_batch(WithoutSession&& without_session) {
        if (!session_.established()) {
            std::forward<WithoutSession>(without_session)();
            return nullptr;
        }

        if (!transmitting_batch_)
            transmitting_batch_ = transmit_buffer_.pop_batch();
        return transmitting_batch_;
    }

    // 第 2 步: 当前批量已完整上线, 归还池子。
    void finish_batch() {
        UpstreamBatchesT::release_batch(transmitting_batch_);
        transmitting_batch_ = nullptr;
    }

    // ---- 下行分发 ----
    // 会话门在基类统一把守(未建立时一律吞掉); 到这里的一定是已建立会话上的数据。
    // 返回 false = "本板不认这个口/线", 与发给本板没有的口同一回答。

    virtual bool dispatch_can(core::protocol::FieldId id, const data::CanDataView& data) = 0;
    virtual bool dispatch_uart(core::protocol::FieldId id, const data::UartDataView& data) = 0;
    // 本板没有该类口时用默认拒绝。
    virtual bool dispatch_gpio_digital(uint8_t, const data::GpioDigitalDataView&) { return false; }
    virtual bool dispatch_gpio_analog(uint8_t, const data::GpioAnalogDataView&) { return false; }
    virtual bool dispatch_gpio_read(uint8_t) { return false; }
    virtual bool dispatch_buzzer(const data::BuzzerToneDataView&) { return false; }

    // 硬件脉冲交换(hpm 专属)。主机向每块板发相同的目标 microframe; 各板在该处
    // 触发并捕获其他板的脉冲。没有脉冲路径的板: 静默不应答 -- 会话协议对
    // schedule 本就没有否定应答之外的话可说。
    virtual void dispatch_pulse_schedule(const data::PulseScheduleView&) {}

    // 每个 keepalive 轮次一次, 紧跟在应答之后: 把本板在跑的 CAN/UART 口的状态交给
    // round, 变了的随即写进上行流。各板的实现就是
    // core::link::offer_port_status<ports::Registry>(round) -- 注册表在板子那边,
    // 基类不认识它。
    virtual void report_port_status(core::link::PortStatusRound& round) = 0;

private:
    // "现在几点"由 TimeSync 提供; 会话状态机的裁决与应答是协议事实, 只写这一份。

    void session_control_deserialized_callback(const data::SessionControlView& data) override {
        switch (data.type) {
        case data::SessionType::kStart: {
            // 门没开时静默拒绝。会话协议没有否定应答, 打不开会话的主机约一个轮次后
            // 自己超时报错; 什么都不说已是这一层唯一能说的话。
            const auto start = session_.on_start(data.nonce, session_allowed(), now_quarter_us());
            if (start == core::link::Session::Start::kRefused)
                return;
            if (start == core::link::Session::Start::kOpened)
                on_session_opened();
            // 每个 kStart 都开一份新的状态基线, 同 nonce 重开(kRenewed)也一样: 下一轮
            // 每个在跑的口重发一遍全量, 主机用它覆盖快照(主机从不清快照, 计数的差就不会
            // 因清零跳变)。kStart 只在连接/重连时出现, 不在轮次里。
            port_status_.reset();

            const auto result = serializer_.write_session_control(
                {.type = data::SessionType::kStartAck, .nonce = data.nonce});
            core::utility::assert_always(
                result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            break;
        }
        case data::SessionType::kKeepalive:
            if (!session_.on_keepalive(data.nonce, now_quarter_us()))
                return;
            {
                const auto result = serializer_.write_session_control(
                    {.type = data::SessionType::kKeepaliveAck, .nonce = data.nonce});
                core::utility::assert_always(
                    result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            }
            // kKeepaliveAck 的格式不动; 运行时的错误与状态都追加在它后面, 一口一条
            // kPortStatus, 变了才发(kStart 之后第一轮每个口都发, 作基线): 先是链路本身
            // (下行流错误), 再是本板在跑的 CAN/UART 口。每轮每口至多一条, 天然限流。
            {
                core::link::PortStatusRound round{port_status_, serializer_, data.nonce};
                round.offer(data::DataId::kSession, downlink_errors_.status());
                report_port_status(round);
            }
            break;
        default: return;
        }
    }

    // 共享时间基。锚点放在会话字段里正是要它与会话同生共死: 失去主机的板子
    // 不应继续维护主机日后假定仍对齐的时间线。与 keepalive 一样检查 nonce, 并
    // 在同一次交换中应答, 主机不必再跑一轮就能拿到板子状态。时间基准没被这次的清单
    // 要过(TimeSync::on() 为假)就不应答: 板子此刻没有时间线可报。
    void time_anchor_deserialized_callback(const data::TimeAnchorView& data) override {
        if (!session_.established() || data.nonce != session_.nonce() || !TimeSync::on())
            return;

        TimeSync::apply_anchor(data.microframe);

        const auto snapshot = TimeSync::report();
        const auto captures = TimeSync::take_capture_counts();

        // 上报"此刻"在微帧轴上的位置(含小数), 而非最后一次 SOF 时的计数。
        //
        // snapshot.microframe 是上一个 Start-of-Frame 锁存的计数, 落后最多一个
        // 微帧(全速 0..1 ms, 高速 0..125 us)-- 均匀分布, 均值为半个微帧。主机把
        // 收到的值与本往返的中点配对, 报整数就等于把这段量化噪声原样交给主机。
        // [实测 2026-09-07, host/examples/mc02_time_sync_test.cpp --causality 的
        //  因果探针: 2185 次探测中 100% 的换算落在其自身 send/reply 括区之外
        //  +440..+491 us, 位置残差恰为 1 ms 均匀分布, sigma 288 us = 1000/sqrt(12)。]
        //
        // 插值到当前时刻只花一次板级定时器读加一次乘法(interpolate_now() 正是为此
        // 存在)。时间线无效或拟合未收敛时回退到锁存对, 小数为零 -- 唯一插值无意义
        // 的情形。连同微帧内的小数一起报 [2026-10-03]: 此前只报整数微帧, 插值的
        // 结果被量化回微帧栅格, 残留 0..1 微帧的均匀误差。
        auto reported_microframe = snapshot.microframe;
        std::uint16_t reported_fraction_q16 = 0;
        auto reported_quarter_us = static_cast<std::uint32_t>(snapshot.timestamp_quarter_us);
        {
            std::uint32_t now_quarter_us = 0;
            std::uint64_t microframe_now = 0;
            std::uint16_t fraction_now_q16 = 0;
            if (TimeSync::interpolate_now(now_quarter_us, microframe_now, fraction_now_q16)) {
                reported_microframe = microframe_now;
                reported_fraction_q16 = fraction_now_q16;
                reported_quarter_us = now_quarter_us;
            }
        }

        (void)serializer_.write_time_status({
            .nonce = data.nonce,
            .microframe = reported_microframe,
            .microframe_fraction_q16 = reported_fraction_q16,
            .timestamp_quarter_us = reported_quarter_us,
            .ticks_per_microframe_q16 = snapshot.ticks_per_microframe_q16,
            .state = snapshot.state,
            .anomaly_count = snapshot.anomaly_count,
            .residual_mean_q16 = snapshot.residual_mean_q16,
            .residual_abs_max_q16 = snapshot.residual_abs_max_q16,
            .residual_count = static_cast<std::uint16_t>(snapshot.residual_count),
            .capture_fresh_count = captures.fresh,
            .capture_stale_count = captures.stale,
        });
    }

    void pulse_schedule_deserialized_callback(const data::PulseScheduleView& data) override {
        if (!session_.established() || data.nonce != session_.nonce())
            return;
        dispatch_pulse_schedule(data);
    }

    // 新 nonce 的会话: 在途与待发的批量不留; 下行传输从头编号(错误计数是自由计数,
    // 不清; 状态账本的基线在 kStart 处另清, 见上)。传输层重置自己的进度, 板级回调清旧主机的输出(GPIO/PWM 等)。
    void on_session_opened() {
        if (transmitting_batch_) {
            UpstreamBatchesT::release_batch(transmitting_batch_);
            transmitting_batch_ = nullptr;
        }
        transmit_buffer_.clear();
        downlink_errors_.begin_session();
        session_activated_callback();
    }

    // 下行按口类分发; 每一类口先过同一道会话门。没有的口类由 dispatch_* 默认拒绝。
    bool can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        if (!session_.established())
            return true;
        return dispatch_can(id, data);
    }

    bool uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        if (!session_.established())
            return true;
        return dispatch_uart(id, data);
    }

    bool gpio_digital_data_deserialized_callback(
        uint8_t line, const data::GpioDigitalDataView& data) override {
        if (!session_.established())
            return true;
        return dispatch_gpio_digital(line, data);
    }

    bool gpio_analog_data_deserialized_callback(
        uint8_t line, const data::GpioAnalogDataView& data) override {
        if (!session_.established())
            return true;
        return dispatch_gpio_analog(line, data);
    }

    bool gpio_read_deserialized_callback(uint8_t line) override {
        if (!session_.established())
            return true;
        return dispatch_gpio_read(line);
    }

    bool buzzer_tone_deserialized_callback(const data::BuzzerToneDataView& data) override {
        if (!session_.established())
            return true;
        return dispatch_buzzer(data);
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

    void error_callback(core::protocol::FieldId field, data::DownlinkError reason) override {
        // 拒收(kRefused)与解析失败(格式坏/未知字段)都记进账本, 随下一次 keepalive
        // 应答上报。本层之下的每个传输层都无损交付字节 (USB bulk、EtherCAT 停等
        // ARQ), 反序列化出错即主机/固件的帧定界 bug 或主机发错了口; 恢复发生在
        // 下一个传输边界 / 链路重启, 经 finish_downlink_transfer()。
        downlink_errors_.note(reason, field);
    }

    core::protocol::Deserializer deserializer_{*this};

    UpstreamBatchesT transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};

    const Batch* transmitting_batch_ = nullptr;
    core::link::Session session_;
    core::link::DownlinkErrors downlink_errors_;
    core::link::PortStatusLedger port_status_;
};

} // namespace libhcs::firmware::link
