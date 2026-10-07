#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <optional>
#include <utility>

#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_mcan_drv.h>
#include <hpm_mcan_regs.h>
#include <hpm_mcan_soc.h>
#include <hpm_ptpc_drv.h>
#include <hpm_soc.h>
#include <hpm_soc_feature.h>

#include "board_app.hpp"
#include "core/include/libhcs/data/datas.hpp"
#include "core/src/link/port.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/common/app/src/utility/event_counter.hpp"
#include "firmware/common/app/src/utility/latched_bus_error.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"
#include "firmware/common/app/src/utility/ring_buffer.hpp"
#include "firmware/hpm_board/app/src/can/bit_timing.hpp"
#include "firmware/hpm_board/app/src/can/can_port.hpp"
#include "firmware/hpm_board/app/src/led/led.hpp"
#include "firmware/hpm_board/app/src/link/uplink.hpp"

namespace libhcs::firmware::can {

namespace vc = libhcs::core::protocol::vendor_control;

using board::CanMode;
using board::CanPort;

// 下行帧的装配策略(见 Can::submit_downlink)。端口模式始终是上限 -- 经典模式的端口
// 发不出 FD -- 策略只回答这一帧想要什么: 要不要 FD / BRS, 以及线上 DLC 是由调用方
// 给定(explicit_dlc 有值)还是按负载长度推导(std::nullopt)。
template <typename Policy>
concept DownlinkFramePolicy = requires(const Policy& policy) {
    { policy.wants_fd() } -> std::same_as<bool>;
    { policy.wants_bitrate_switch() } -> std::same_as<bool>;
    { policy.explicit_dlc() } -> std::same_as<std::optional<uint8_t>>;
};

class Can : private core::utility::Immovable {
public:
    // 这个驱动作为 EP0 口能做什么(spec/port.hpp 的能力位, kGetPortList 原样上报): 主机
    // 可切帧型(经典时控制器关 FD)、可设两段速率(按 87.5% 采样点求解, can/bit_timing.hpp)、
    // 承载 64 字节长帧(RX 元素与 TX 缓冲按 64 字节配)。hpm5321 与 hpm6e8y 同一个驱动,
    // 同一组能力。
    static constexpr uint8_t kPortCapabilities =
        spec::kCanCapModeSettable | spec::kCanCapFdLongFrames | spec::kCanCapRateSettable;

    // 仅凭逻辑 CAN 序号构造, CanPort 经 board::can_port() 推导。Lazy 的参数用
    // 序号而非 port: 每个构造点都不依赖端口表的内容。
    using Lazy = utility::Lazy<Can, data::DataId, size_t>;

    // 上电默认速率(连同端口表的帧型)。libhcs 由 host 经 EP0 下发实际速率(apply_setting);
    // DMTool 失败救援也回落到这两个值。
    static constexpr uint32_t kArbitrationBaudrate = 1'000'000;
    static constexpr uint32_t kCanFdDataBaudrate = 5'000'000;

    // 两个位相的采样点, 全场钉在 87.5‰ -- 与总线上其他板一致, 而非 SDK 求解器
    // 的 75% 下限(那一档实测 FD 投递 0/40000, 见构造函数长注释)。EP0 的
    // kGetPortConfig 读回与清单声明核对用的就是这两个编译期值。
    static constexpr uint16_t kNominalSamplePointPerMille = 875U;
    static constexpr uint16_t kDataSamplePointPerMille = 875U;

    // 本驱动使能的 IE 掩码, 也是 ISR 返回前对 IR 重测的同一掩码。两处共用一个
    // 常量: ISR 循环只有在恰好覆盖所有能拉高中断线的源时才正确, 两者不能漂移
    // (见 irq_handler)。
    static constexpr uint32_t kEnabledInterrupts =
        MCAN_INT_RXFIFO0_NEW_MSG | MCAN_INT_BUS_OFF_STATUS | MCAN_INT_WARNING_STATUS
        | MCAN_INT_ERROR_PASSIVE | MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE
        | MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE;

    explicit Can(data::DataId data_id, size_t board_can_index)
        : Can(data_id, board::can_port(board_can_index), board_can_index) {}

    explicit Can(data::DataId data_id, CanPort port, size_t board_can_index)
        : data_id_(data_id)
        , can_base_(reinterpret_cast<MCAN_Type*>(port.base))
        , irq_num_(port.irq_num)
        , canfd_(port.mode == CanMode::kCanFd)
        , can_index_(board_can_index)
        , libhcs_setting_(default_setting(port.mode)) {

        // 只允许初始化 PCB 实际存在的端口。单路 hpm5321 的第二个槽位引脚是 LED
        // 阴极, 构造它等于把 PA30/PA31 交给一个不存在的收发器。
        core::utility::assert_always(board_can_index < board::can_port_count());

        // Message RAM 布局是板级事务: 各 SoC 的 MCAN 缓冲区可放的位置不同
        // (专用 AHB RAM 或 section 定位数组), 由板层发放区域 (board_app.cpp)。
        const mcan_msg_buf_attr_t attr = board::can_message_ram(board_can_index);
        auto status = mcan_set_msg_buf_attr(can_base_, &attr);
        core::utility::assert_always(status == status_success);

        const uint32_t can_source_clock_freq = board::init_can(can_base_);
        if (can_source_clock_freq % 1'000'000U == 0)
            can_clock_mhz_ = static_cast<uint16_t>(can_source_clock_freq / 1'000'000U);

        // 先一次性启动共享的 PTPC0 时基, 再把本控制器的 TSU 输入指向它。
        // PTPC 由 AHB 时钟组 (clock_ptpc) 供时钟。
        const uint32_t ptpc_freq = clock_get_frequency(clock_ptpc);
        // PTPC 数字模式下, 纳秒计数器按 floor(1e9 / ptpc_freq) ns 的整数步前进
        // (160 MHz 时为 6 ns), 而非精确的 1e9 / ptpc_freq (6.25 ns), 上报时间
        // 偏慢。上报纳秒换算微秒时除以 (ptpc_freq_MHz * step) 而非 1000, 误差
        // 恰好抵消 (160 MHz 时 160 * 6 = 960)。handle_uplink() 把该除数固化为
        // 编译期 kTsNsPerUs, ISR 无需运行时除法 -- 时钟树若变, 须同步核对此式。
        const uint32_t ptpc_step_ns = 1'000'000'000U / ptpc_freq;
        core::utility::assert_always((ptpc_freq / 1'000'000U) * ptpc_step_ns == kTsNsPerUs);

        static bool ptpc_timebase_started = false;
        if (!ptpc_timebase_started) {
            ptpc_config_t ptpc_config;
            ptpc_get_default_config(HPM_PTPC, &ptpc_config);
            ptpc_config.src_frequency = ptpc_freq;
            ptpc_config.ns_rollover_mode = ptpc_ns_counter_rollover_digital;
            core::utility::assert_always(
                ptpc_init(HPM_PTPC, PTPC_PTPC_0, &ptpc_config) == status_success);
            ptpc_init_timer(HPM_PTPC, PTPC_PTPC_0);
            ptpc_timebase_started = true;
        }
        ptpc_set_timer_output(HPM_PTPC, mcan_get_instance_from_base(can_base_), false);

        // 构造期初始化失败不停机: 其余端口与 USB 照常, 该路由 LED/can_status 暴露。
        (void)init_controller(libhcs_setting_, can_source_clock_freq);

        enable_interrupts();
        // CAN RX 是转发关键路径 (电机反馈 -> 主机)。优先级 3, 高于 USB (2) 与
        // UART (1) -- 保证 CAN 帧不被批量 USB 传输或 DMA 回调拖延。
        intc_m_enable_irq_with_priority(port.irq_num, 3);
        // 上电即在总线上(没有 libhcs 主机时板子是 DMTool 的适配器)。
        set_suspended(false);
    }

    // 每条 (重新) 初始化控制器的路径都以它结束: 使能本驱动的中断源, 并忘掉发送槽的
    // 历史 (初始化后 TXBCF 全部置位, 不是真的作废; 见 send_to_fifo())。
    void enable_interrupts() {
        used_tx_slots_ = 0;
        mcan_enable_interrupts(can_base_, kEnabledInterrupts);
    }

    // 往发送 FIFO 写一帧, 顺带数单发作废的帧: libhcs 关掉了自动重传, 输掉仲裁或出错的
    // 发送就此作废, 不动 TEC/REC。一个槽只在上一次发送结束 (成功或作废) 后才被重新分配,
    // 所以写入前读到的 TXBCF 那一位就是这个槽上一次的结果, 每次结果正好看一次。
    // DAR 模式的自动作废不置 IR.TCF (实测: 打开 TCF 中断与 TXBCIE 后计数恒为 0),
    // 所以不能靠中断数。代价: 每帧多读一次 TXBCF。
    [[gnu::always_inline]] hpm_stat_t send_to_fifo(mcan_tx_frame_t* frame) {
        const uint32_t cancelled_before = can_base_->TXBCF;
        uint32_t slot = 0;
        const hpm_stat_t status = mcan_transmit_via_txfifo_nonblocking(can_base_, frame, &slot);
        if (status == status_success) {
            const uint32_t bit = 1U << slot;
            if ((cancelled_before & used_tx_slots_ & bit) != 0U) [[unlikely]]
                ++cancelled_frames_;
            used_tx_slots_ |= bit;
        }
        return status;
    }

    [[nodiscard]] data::DataId data_id() const { return data_id_; }

    // 运行时状态(core/src/link/port_status.hpp), 每个 keepalive 轮次在主循环读一次。
    //
    // PSR.LEC/DLEC 读后自清为"无变化", 而协议错误中断(kEnabledInterrupts)每次出错都会
    // 进 ISR 读 PSR -- 错误码多半已被 ISR 读走。所以 ISR 把读到的错误码交给锁存
    // (LatchedBusError, ISR 是它唯一的写者), 这里只读: 自己这次读到真实错误就用它,
    // 否则用锁存的。
    [[nodiscard]] data::CanStatusView read_status() const {
        const uint32_t psr = can_base_->PSR;
        const uint32_t ecr = can_base_->ECR;
        uint8_t flags = 0;
        if ((psr >> 5) & 1U)
            flags |= data::kCanErrorPassive;
        if ((psr >> 6) & 1U)
            flags |= data::kCanWarning;
        if ((psr >> 7) & 1U)
            flags |= data::kCanBusOff;
        return {
            .tec = static_cast<uint8_t>(ecr & 0xFFU),
            .rec = static_cast<uint8_t>((ecr >> 8) & 0x7FU),
            .last_error = last_bus_error_.latest(static_cast<data::CanLastError>(psr & 0x7U)),
            .data_last_error =
                last_data_bus_error_.latest(static_cast<data::CanLastError>((psr >> 8) & 0x7U)),
            .flags = flags,
            .tx_cancelled = static_cast<uint16_t>(cancelled_frames_),
            .tx_dropped = tx_dropped_.count(),
            .rx_dropped = rx_dropped_.count(),
            .rx_lost = rx_lost_.count(),
        };
    }

    // 转发热路径 -- out-of-line 定义在 can.cpp 的 ILM (.fast) 段, 消除最坏
    // 转发延迟中的 FLASH-XIP 取指抖动。理由及为何不内联在类内见 can.cpp。
    // handle_uplink 至多从 RX FIFO0 读一帧并返回是否消费了帧, ISR 据此循环
    // 排空 FIFO。
    // libhcs 下行: 帧型跟端口, 线上 DLC 按负载长度推导(短帧按字节数, 长帧经
    // DLC 编解码, 仅限 FD 总线)。返回 false = TX 队列满被丢(调用方自行取舍)。
    bool handle_downlink(const data::CanDataView& data);

    // DMTool 仿真路径的一帧下行请求: 帧型与线上 DLC 都按主机逐帧给出(真适配器
    // 界面上的 FD / BRS 勾选就是逐帧的; 长帧 DLC 9-15 直传)。本身即一个
    // DownlinkFramePolicy。
    struct RequestedFrame {
        uint8_t wire_dlc;
        bool fd;
        bool bitrate_switch;

        [[nodiscard]] constexpr bool wants_fd() const { return fd; }
        [[nodiscard]] constexpr bool wants_bitrate_switch() const { return bitrate_switch; }
        [[nodiscard]] constexpr std::optional<uint8_t> explicit_dlc() const { return wire_dlc; }
    };

    // DMTool 仿真路径的下行(FLASH, 非热路径)。端口能力仍是上限: 经典模式下
    // 请求 FD 会降级成经典帧, BRS 只在实际发 FD 帧时才置位。
    // 返回 false = TX 队列满(DMTool 适配器据此对 USB OUT 施加背压, 帧不丢)。
    bool handle_downlink_as(const data::CanDataView& data, RequestedFrame frame);

    bool handle_uplink(core::protocol::FieldId field_id, core::protocol::Serializer& serializer);
    void irq_handler();

    // 实际写进控制器的位时序, 从 NBTP/DBTP 读回(寄存器字段 +1 即 tq 数与分频
    // 系数)。DMTool 的读波特率命令用它回报硬件事实, 而不是回显请求值。
    struct PhaseTiming {
        uint32_t prescaler, seg1, seg2, sjw; // seg1 = 传播段 + 相位段 1
    };
    struct BitTiming {
        uint32_t clock_hz; // 0 表示未知
        PhaseTiming nominal;
        PhaseTiming data;  // 仅 FD 端口有意义
    };
    [[nodiscard]] BitTiming bit_timing() const {
        const uint32_t nbtp = can_base_->NBTP;
        const uint32_t dbtp = can_base_->DBTP;
        const PhaseTiming nominal{
            .prescaler = MCAN_NBTP_NBRP_GET(nbtp) + 1U,
            .seg1 = MCAN_NBTP_NTSEG1_GET(nbtp) + 1U,
            .seg2 = MCAN_NBTP_NTSEG2_GET(nbtp) + 1U,
            .sjw = MCAN_NBTP_NSJW_GET(nbtp) + 1U,
        };
        const PhaseTiming data{
            .prescaler = MCAN_DBTP_DBRP_GET(dbtp) + 1U,
            .seg1 = MCAN_DBTP_DTSEG1_GET(dbtp) + 1U,
            .seg2 = MCAN_DBTP_DTSEG2_GET(dbtp) + 1U,
            .sjw = MCAN_DBTP_DSJW_GET(dbtp) + 1U,
        };
        return {.clock_hz = can_clock_mhz_ * 1'000'000U, .nominal = nominal, .data = data};
    }

    [[nodiscard]] bool is_fd() const { return canfd_; }

    // 控制器当前帧型(硬件事实)。经典 = 控制器关 FD(CCCR.FDOE=0), 硬件上发不出任何 FD 位。
    [[nodiscard]] CanMode mode() const { return canfd_ ? CanMode::kCanFd : CanMode::kClassic; }

    // libhcs 的总线设置: 帧型与两段速率由 host 经 EP0 下发(接线事实, host 代码知道),
    // 采样点与 TDC 是本板的实测策略, 不在其中。类型是核心的统一设置(link::CanSetting),
    // 主机测试的假驱动与固件驱动收同一个东西。
    using BusSetting = core::link::CanSetting;

    [[nodiscard]] static constexpr BusSetting default_setting(CanMode mode) {
        return {
            .fd = mode == CanMode::kCanFd,
            .arbitration_baudrate = kArbitrationBaudrate,
            .data_baudrate = kCanFdDataBaudrate};
    }

    [[nodiscard]] const BusSetting& setting() const { return libhcs_setting_; }

    // libhcs 经 EP0 应用总线设置(核心 link::ep0 的清单阶段二): 与硬件现状一致只记下选择,
    // 否则丢弃旧设置下排队的帧并重跑 libhcs 配置。解不出(87.5% 采样点下凑不出该速率)时
    // 救回原设置, 返回 false。
    [[nodiscard]] bool apply_setting(const BusSetting& setting);

    // 一份设置应用成功后应有的时序: 速率即所求, 采样点钉死在 87.5%。经典模式没有数据段,
    // 两项报 0。
    using TimingIdentity = core::link::CanTimingValue;

    [[nodiscard]] static constexpr TimingIdentity expected_of(const BusSetting& setting) {
        const bool fd = setting.fd;
        return {
            .arbitration_baudrate = setting.arbitration_baudrate,
            .data_baudrate = fd ? setting.data_baudrate : 0U,
            .nominal_sample_point = kNominalSamplePointPerMille,
            .data_sample_point = fd ? kDataSamplePointPerMille : uint16_t{0},
        };
    }

    // 纯求解: 这份设置能不能原样落到本控制器上(bit_timing.hpp, 不碰寄存器)。0 = 能;
    // 否则返回落不下的那一段速率。EP0 清单的校验阶段用它, 应用阶段的 init_controller()
    // 走 SDK 写进去的是同一个解。
    [[nodiscard]] uint32_t unrepresentable_rate(const BusSetting& setting) const {
        return bit_timing::unrepresentable_rate(
            can_clock_mhz_ * 1'000'000U, setting.fd, setting.arbitration_baudrate,
            setting.data_baudrate, kNominalSamplePointPerMille, kDataSamplePointPerMille);
    }

    // 从 NBTP/DBTP 重构的硬件事实; 源时钟非整 MHz(未知)时速率报 0。
    [[nodiscard]] TimingIdentity timing_identity() const {
        const BitTiming timing = bit_timing();
        const auto rate = [&timing](const PhaseTiming& phase) -> uint32_t {
            const uint32_t divisor = phase.prescaler * (1U + phase.seg1 + phase.seg2);
            return divisor != 0U ? timing.clock_hz / divisor : 0U;
        };
        const auto sample_point = [](const PhaseTiming& phase) {
            return static_cast<uint16_t>(
                (1U + phase.seg1) * 1000U / (1U + phase.seg1 + phase.seg2));
        };
        return {
            .arbitration_baudrate = rate(timing.nominal),
            .data_baudrate = canfd_ ? rate(timing.data) : 0U,
            .nominal_sample_point = sample_point(timing.nominal),
            .data_sample_point = canfd_ ? sample_point(timing.data) : uint16_t{0},
        };
    }

    // DMTool SETUP_BUARD 的运行时重配(实现见 can.cpp): 按命令给定的 TQ 参数
    // (分频/seg1/seg2/sjw, 与 mcan_bit_timing_param_t 同语义)直接写低级位时序,
    // 并切换 FD/经典模式。请求非法或硬件拒绝时端口保持原配置并返回 false。
    bool reconfigure_timing(bool fd, PhaseTiming nominal, PhaseTiming data);

    // ---- 清单声明: 主机没声明的总线不上线 ----
    //
    // libhcs 主机的清单声明被完整应用时, 声明的总线以声明的设置上线(apply_setting),
    // 未声明的总线留在挂起状态: 控制器进 INIT 模式撤下总线 -- 不应答别人的帧、不发
    // 错误帧 -- 中断全关。归属交还(会话结束、挂起、重新枚举)时全部恢复(resume):
    // 没有 libhcs 主机时板子是 DMTool 的适配器, 那条路不走 EP0, 总线必须是通的。
    //
    // 只在主循环调用(EP0 处理器与会话状态机都经 tud_task() 到达)。
    [[nodiscard]] bool suspended() const { return suspended_; }
    void suspend();
    void resume();

    // ---- 端口接口(core/src/link/ 的通用 CAN 操作按这一组原语工作) ----
    //
    // 全部是冷路径(EP0 与主循环的看门狗/挂起恢复), 不在转发热路径上。
    [[nodiscard]] bool running() const { return !suspended_; }
    // 此刻实际发送的帧型(硬件事实): 经典 = 控制器关 FD, 硬件上发不出任何 FD 位。
    [[nodiscard]] bool fd_now() const { return canfd_; }
    // 寄存器重构的位时序事实(timing_identity 的统一名)。
    [[nodiscard]] TimingIdentity timing() const { return timing_identity(); }
    // kGetPortConfig 的应答: 硬件事实而非"上次请求"。
    void read_config(vc::CanConfigPayload& out) const;
    // kGetPortList 的状态位。
    [[nodiscard]] core::link::PortStatus describe() const {
        return {.running = running(), .fd = canfd_};
    }

    // 在总线上的控制器, 每个一位(位号 = board_can_index): 主循环按它调 poll()。
    [[nodiscard]] static uint32_t running_mask() { return running_mask_; }

    // 主循环看门狗, 处理 PLIC 已接受却从未送达的中断请求。健康路径仅两次寄存器
    // 读; 修复的故障及实测依据见 can.cpp。
    void poll();

    // 把本控制器的软件发送队列排入 MCAN TX FIFO。队列只在硬件 FIFO 满时才有
    // 内容 (见 can.cpp 的 handle_downlink), 故几乎总为空; 判空内联在此, 空队列
    // 零调用开销。留给仍逐个遍历控制器的主循环 (hpm6e8y); hpm5321 走
    // drain_pending_transmits()。
    void try_transmit() {
        if (transmit_buffer_.readable() == 0) [[likely]]
            return;
        drain_transmit_queue();
    }

    // 所有控制器共用的主循环入口。transmit_pending_mask_ 每控制器一位, 表示其
    // 队列可能仍有帧; 任何一路都没排队 -- 几乎每轮都如此 -- 时只需一次加载
    // 加一次分支, 与板上有几路总线无关。
    //
    // mask 是普通数据而非原子量: 读写双方都只在主循环运行 -- handle_downlink
    // 仅经 tud_task() 到达 (TinyUSB vendor class 未注册 xfer_isr,
    // tud_vendor_rx_cb 不会在 USB 中断里运行), 排空也只发生在这里。不变量:
    // 队列非空时对应位必为 1。handle_downlink 在每次入队尝试后置位,
    // drain_transmit_queue 确认队列已空才清位。debug 构建在空闲路径上校验
    // 该不变量。
    static void drain_pending_transmits() {
        if (transmit_pending_mask_ == 0) [[likely]] {
            core::utility::assert_debug_lazy([]() noexcept { return transmit_queues_empty(); });
            return;
        }
        drain_pending_transmits_slow();
    }

    // 软件发送队列中等待的帧数。
    [[nodiscard]] size_t transmit_queue_depth() const { return transmit_buffer_.readable(); }

    static constexpr size_t kTransmitQueueSize = 64;

    // 中断处理的一轮: 确认 `flags` 并处理之。从 irq_handler 拆出, 后者可重测
    // IR 并重复调用; 只处理一轮就返回会丢中断, 原因见该处注释。
    void handle_interrupt_flags(uint32_t flags);

private:
    // ---- 消息 RAM 布局 ----
    //
    // SDK 的默认布局是经典 CAN 预设(8 字节元素、各 32 项), 本板全部重排: 元素
    // 扩到 64 字节(DMTool 仿真要收发 DLC 9-15 的长帧), 深度按下面的常量。
    // **超出控制器的 640 词 mcan_init 会静默失败, 两路控制器一起死** -- 所以
    // 预算在本文件编译期核对, 而不是靠上板才发现。
    //
    // 深度的取法: RX 16 让 ISR 迟到一会儿也不丢帧(1 Mbit 经典帧下约 1.8 ms 的
    // 余量), TX 12 是硬件侧的突发吸收, 再往上由软件发送队列(kTransmitQueueSize
    // = 64)承接。过滤器 16+16 是 sync 滤波器用的余量。
    static constexpr uint32_t kStdFilterCount = 16;
    static constexpr uint32_t kExtFilterCount = 16;
    static constexpr uint32_t kRxFifoElemCount = 16;
    static constexpr uint32_t kTxFifoElemCount = 12;

    // 元素字节数与 SDK 的 mcan_config_ram() 同公式: 数据域 + 8 字节头。
    static constexpr uint32_t kMsgRamElemBytes = 64U + MCAN_MESSAGE_HEADER_SIZE_IN_BYTES;
    static constexpr uint32_t kMsgRamBytes =
        (kStdFilterCount * MCAN_FILTER_ELEM_STD_ID_SIZE)
        + (kExtFilterCount * MCAN_FILTER_ELEM_EXT_ID_SIZE)
        + ((kRxFifoElemCount + kTxFifoElemCount) * kMsgRamElemBytes)
        + (kTxFifoElemCount * MCAN_TXEVT_ELEM_SIZE);
    static_assert(kMsgRamBytes <= MCAN_MSG_BUF_SIZE_IN_WORDS * sizeof(uint32_t));

    // libhcs_config() 与 DMTool 重配 reconfigure_timing 共用同一份布局: 两者必须一致,
    // 否则一次 DMTool 会话之后 libhcs 跑的就是另一套深度。自动重传不在其中 -- 两处
    // 各有立场, 见各自现场。
    static void apply_message_ram_layout(mcan_config_t& config) noexcept {
        config.ram_config.enable_rxbuf = false;
        config.ram_config.rxbuf_elem_count = 0U;
        // rxfifos[1] 与 rxbuf 都是经典预设残留, 不关则总量超 640 词。
        config.ram_config.rxfifos[1].enable = false;
        config.ram_config.rxfifos[1].elem_count = 0U;
        config.ram_config.std_filter_elem_count = kStdFilterCount;
        config.ram_config.ext_filter_elem_count = kExtFilterCount;
        config.ram_config.rxfifos[0].elem_count = kRxFifoElemCount;
        config.ram_config.rxfifos[0].data_field_size = MCAN_DATA_FIELD_SIZE_64BYTES;
        config.ram_config.txbuf_data_field_size = MCAN_DATA_FIELD_SIZE_64BYTES;
        config.ram_config.txbuf_dedicated_txbuf_elem_count = 0;
        config.ram_config.txbuf_fifo_or_queue_elem_count = kTxFifoElemCount;
        config.ram_config.txfifo_or_txqueue_mode = MCAN_TXBUF_OPERATION_MODE_FIFO;
        // TX event FIFO 本驱动不读(发送完成不产生工作, TX 中断也没开), 但 SDK
        // 的默认值是经典预设的 32 项, 白占 256 字节。跟着 TX FIFO 走即可。
        config.ram_config.tx_evt_fifo_elem_count = kTxFifoElemCount;
        config.ram_config.tx_evt_fifo_watermark = 1U;
    }

    // libhcs_config() 的时间戳单元与 sync 过滤器(实现见 can.cpp)。
    static void apply_timestamping(mcan_config_t& config) noexcept;

    // libhcs 的整套控制器配置(建造者): 构造、resume()、EP0 应用设置共用这一份, 各处逐字段
    // 一致由此保证。速率来自 setting, 采样点/TDC/时间戳是本板策略。
    [[nodiscard]] mcan_config_t libhcs_config(const BusSetting& setting) const;

    // 按 libhcs 配置初始化控制器, 成功才更新 canfd_。
    bool init_controller(const BusSetting& setting, uint32_t clock_hz);

    // 运行时重初始化到 mode(deinit -> 重挂消息 RAM -> init -> 开中断), 不检查发送队列。
    bool reinit(const BusSetting& setting);
    void take_off_bus();

    // libhcs 的下行策略: 三个答案全是编译期常量。实例化后这些判断不留下任何
    // 指令 -- 尤其 explicit_dlc() 恒为空, "调用方给定 DLC"那条分支与它要占的
    // 寄存器在 libhcs 路径上根本不存在(曾经是一个运行时参数, 每帧多一次判断,
    // 还要为它多存取一个 callee-saved 寄存器)。
    struct PortFrame {
        static constexpr bool wants_fd() { return true; }
        static constexpr bool wants_bitrate_switch() { return true; }
        static constexpr std::optional<uint8_t> explicit_dlc() { return std::nullopt; }
    };
    static_assert(DownlinkFramePolicy<PortFrame>);
    static_assert(DownlinkFramePolicy<RequestedFrame>);

    // 两个下行入口共用的帧装配与入队逻辑 (定义在 can.cpp, 两处调用点各内联
    // 一份): libhcs 的 handle_downlink 那份留在 ILM, DMTool 的
    // handle_downlink_as 那份随入口留在 FLASH。强制内联, 因此不产生独立符号,
    // 也就不会与 .fast 里的普通函数发生段类型冲突。
    //
    // 帧型走模板策略而不是运行时参数: 端口策略的答案是编译期常量, 分支被消掉,
    // canfd_ 的读取也留在原位(布尔参数会迫使它提前读进 callee-saved 寄存器)。
    template <DownlinkFramePolicy Policy>
    [[gnu::always_inline]] inline bool
        submit_downlink(const data::CanDataView& data, Policy policy);

    // try_transmit() 的 out-of-line 函数体, 位于 .fast (can.cpp)。发现队列已空
    // 后清掉本控制器在 transmit_pending_mask_ 中的位。
    void drain_transmit_queue();

    // drain_pending_transmits() 的 out-of-line 一半, 位于 .fast (can.cpp)。
    static void drain_pending_transmits_slow();

    // drain_pending_transmits() 背后的 debug 构建不变量检查 (can.cpp)。
    static bool transmit_queues_empty();

    // 该控制器的队列可能仍有帧时按 board_can_index 置位。不变量见
    // drain_pending_transmits()。
    static inline constinit uint32_t transmit_pending_mask_ = 0;

    // 在总线上(未挂起)的控制器按 board_can_index 置位。主循环的中断看门狗只走置位
    // 的那些(running_mask()), 被 libhcs 握手挂起的总线在主循环里不占一次调用。
    // 只由 set_suspended() 改写, 只在主循环读写。
    static inline constinit uint32_t running_mask_ = 0;

    // suspended_ 与 running_mask_ 的唯一写入点: 两者永远一致。
    void set_suspended(bool suspended) {
        suspended_ = suspended;
        if (suspended)
            running_mask_ &= ~(1U << can_index());
        else
            running_mask_ |= 1U << can_index();
    }

    // 本控制器在 board::kCanPorts 中的位置。存下来而非从 data_id 推导: 6E8Y 用
    // kCan0..kCan3, 5321 用 kCan1..kCan2, 减去 kCan1 得不到板内序号。
    std::size_t can_index() const { return can_index_; }

    // 读出并归一化一帧 RX FIFO。返回 `true` 表示消费了一个元素; `valid` 恒为
    // 归一化成功(FD 长帧 12-64 字节完整承载, 经典 DLC 9-15 钳到 8)。
    bool read_uplink(data::CanDataView& out, uint8_t storage[64], bool& valid);
    void serialize_uplink(
        core::protocol::FieldId field_id, const data::CanDataView& data,
        core::protocol::Serializer& serializer);

    // libhcs 会话不在时 ISR 的排空路径: DMTool 在采集本路就经 handle_dm_uplink
    // 交给它, 否则丢弃(can.cpp)。与 read_uplink 刻意分开写: 那条是 libhcs 热路径,
    // 不为 DMTool 多带一个分支或参数。
    [[gnu::cold, gnu::noinline]] void drain_without_session();
    bool handle_dm_uplink();

    // 把 MCAN Last Error Code (仲裁相位或数据相位) 归类为指示 LED 状态 --
    // 即 CAN 控制器电气上真正能支撑的粒度:
    //   ack_error  -> kNoAck       : 帧发送正常, 无人应答。
    //   bit0_error -> kWiringFault : 发 dominant 回读 recessive -- 总线驱动不了
    //                                dominant (CAN_H/L 短路、接反或断开), 硬故障
    //                                时稳定。
    //   stuff/form/crc/bit1 -> kSignalError : 位被破坏; 具体码会漂移, 且成因
    //                                (缺终端电阻、波特率不匹配、噪声) 无法区分。
    //   no_error/no_change -> kNone。
    static led::CanFault classify_can_fault(uint8_t last_error_code) {
        switch (last_error_code) {
        case mcan_last_error_code_no_error:
        case mcan_last_error_code_no_change: return led::CanFault::kNone;
        case mcan_last_error_code_ack_error: return led::CanFault::kNoAck;
        case mcan_last_error_code_bit0_error: return led::CanFault::kWiringFault;
        default: return led::CanFault::kSignalError;
        }
    }

    // 把 PTPC 上报的纳秒数换算成真实微秒的除数; 固定为编译期, 使 ISR 侧换算
    // 只含常量除法 (GCC 降为乘加移位), 并在构造函数里对照真实时钟树断言。值
    // 取决于本板的 PTPC (AHB) 时钟 -- 如 HPM5321 为 160 MHz * 6 ns 步 = 960,
    // HPM6E80 为 200 MHz * 5 ns 步 = 1000 -- 由各板 board_app.hpp 提供。
    static constexpr uint32_t kTsNsPerUs = board::kCanTimestampNsPerUs;

    const data::DataId data_id_;
    MCAN_Type* can_base_;
    const uint32_t irq_num_;
    // 控制器当前是否开 FD, 即发送帧型的上限。EP0 apply_setting 与 DMTool 重配都会改它;
    // 仅主循环写(与 handle_downlink 同线程), ISR 不读。
    bool canfd_;
    // board::init_can() 配出的位时序源时钟, 以 MHz 计(非整 MHz 记 0), 只供
    // bit_timing() 回报。刻意放进 canfd_ 之后的对齐空洞: Can 的尺寸与各成员偏移
    // 不变, ISR 与主循环里对 can_array 的寻址逐条指令保持原样。
    uint16_t can_clock_mhz_ = 0;
    const std::size_t can_index_;

    // poll() 用的中断记账。irq_count_ 仅 ISR 写、仅主循环读, 普通 32 位计数
    // 即可 -- RV32 上对齐的读写是原子的, 且只关心变化而非具体值。
    uint32_t irq_count_ = 0;
    // 板子自己丢的帧(read_status() 报给主机): 下行撞上发送队列满(主循环写),
    // 上行撞上上行批量池满(接收中断写)。
    utility::EventCounter tx_dropped_;
    utility::EventCounter rx_dropped_;
    // RX FIFO0 溢出(IR.RF0L): 控制器里就丢了的帧, 每次 ISR 见到这个标志记一次(ISR 写)。
    utility::EventCounter rx_lost_;
    // ISR 从 PSR 读到的最近一次真实错误码(仲裁段 / 数据段), 只 ISR 写, read_status() 读。
    utility::LatchedBusError last_bus_error_;
    utility::LatchedBusError last_data_bus_error_;
    // 开机以来作废的发送 (单发: 输掉仲裁或出错), 由 send_to_fifo() 在重用发送槽时清点;
    // 最近还没被重用的槽 (至多 FIFO 深度个) 的结果要等下一次重用才计入。只在主循环
    // (tud_task 与发送队列排空) 读写。
    uint32_t cancelled_frames_ = 0;
    // 本次初始化以来写过帧的发送槽。
    uint32_t used_tx_slots_ = 0;

    // 接收中断进入时的 CSR_MCYCLE, 帧序列化完成后闭合。写与读都在同一中断内。
    uint32_t uplink_opened_at_ = 0;
    uint32_t watchdog_irq_count_ = 0;
    bool watchdog_armed_ = false;
    // 见 suspend()。放在 watchdog_armed_ 之后的对齐空洞里: 不挪动其他成员的偏移。
    // 仅主循环读写。
    bool suspended_ = false;

    // 32 元素 MCAN TX FIFO 之前的软件发送队列, USB 突发超过总线消化速率时
    // 缓冲而非丢弃。生产者 handle_downlink、消费者 try_transmit, 都在主循环
    // (TinyUSB 把接收回调推迟到 tud_task), 是普通的单生产者单消费者环形队列,
    // 无跨上下文风险。
    // 排队的帧, 压缩存储。mcan_tx_frame_t 有 72 字节, 因其 data union 按
    // 64 字节 CAN-FD 负载取尺寸, 而本协议 CAN 数据上限 8 字节 (3 位 DLC, 见
    // core/src/protocol/serializer.hpp), 故 8 字节头加 8 字节数据就是全部所需。
    // 原样存 SDK 类型则每槽 72 B 只有 16 B 是活的, 同样的 RAM 换来 4 倍队列
    // 深度 -- 正是突发测量曾欠缺的。
    struct QueuedFrame {
        uint32_t header[2]; // mcan_tx_frame_t 的 T0/T1 字
        uint8_t data[64];   // DMTool 长帧: DLC 9-15 -> 12-64 字节
    };
    static_assert(sizeof(QueuedFrame) == 72);

    utility::RingBuffer<QueuedFrame, kTransmitQueueSize> transmit_buffer_;

    // host 经 EP0 下发的总线设置, resume() 的还原目标。冷数据放类尾: 不挪动热路径
    // 成员的偏移。初值为端口表帧型 + 上电默认速率。
    BusSetting libhcs_setting_;
};

// 以下全部由板级 CAN 端口表 (board::kCanPorts) 构建, 没有逐端口宏: 数量、
// FD 模式与分发都跟随该表。Message RAM 来自 board::can_message_ram()。
//
// kCanCount 是表的容量 -- 镜像携带多少个 Can 槽位。多数板上恰好也是实际
// 控制器数。当一个板目录服务两块 PCB 时 (boards/hpm5321), 表按容量更大的
// 变体取尺寸, board::can_port_count() 报告运行时实际存在的数量, 所有循环
// 都必须以它为界: 给 PCB 不存在的端口构造 Can 会去点亮引脚属于别的东西的
// 控制器。can_port_count() 之上的槽位保持未初始化, Lazy 使其安全惰性 --
// init() 之前 try_get() 返回 nullptr。
constexpr size_t kCanCount = board::kCanPortCapacity;
static_assert(kCanCount == std::size(board::kCanPorts));
static_assert(kCanCount >= 1 && kCanCount <= 4);

// 本板实际存在的控制器数。除双变体 hpm5321 镜像外等于 kCanCount。
inline size_t can_count() { return board::can_port_count(); }

template <std::size_t... indices>
constexpr std::array<data::DataId, sizeof...(indices)>
    make_can_data_ids(std::index_sequence<indices...>) {
    return {board::kCanPorts[indices].data_id...};
}

constexpr auto kCanDataIds = make_can_data_ids(std::make_index_sequence<kCanCount>{});

namespace internal {

template <std::size_t index>
consteval Can::Lazy make_can() {
    return Can::Lazy{kCanDataIds[index], index};
}

template <std::size_t... indices>
consteval std::array<Can::Lazy, sizeof...(indices)>
    make_can_array(std::index_sequence<indices...>) {
    return {make_can<indices>()...};
}

} // namespace internal

inline constinit auto can_array = internal::make_can_array(std::make_index_sequence<kCanCount>{});

// libhcs 握手开始 / 作废时对本 PCB 实有的全部控制器生效。以 can_count() 为界并用
// try_get() 保护, 理由同下。
inline void suspend_all() {
    for (size_t i = 0; i < can_count(); ++i) {
        if (Can* can = can_array[i].try_get())
            can->suspend();
    }
}

inline void resume_all() {
    for (size_t i = 0; i < can_count(); ++i) {
        if (Can* can = can_array[i].try_get())
            can->resume();
    }
}

// 本 PCB 实有控制器中最深的软件发送队列(Can::transmit_queues_empty() 用它)。以
// can_count() 为界并用 try_get() 保护, 理由同其他对 can_array 的循环: 单路
// hpm5321 的尾部槽位从未构造。
inline size_t max_transmit_queue_depth() {
    size_t depth = 0;
    for (size_t i = 0; i < can_count(); ++i) {
        if (const Can* can = can_array[i].try_get())
            depth = std::max(depth, can->transmit_queue_depth());
    }
    return depth;
}

} // namespace libhcs::firmware::can
