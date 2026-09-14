#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
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
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/hpm_board/app/src/can/can_port.hpp"
#include "firmware/hpm_board/app/src/led/led.hpp"
#include "firmware/hpm_board/app/src/link/uplink.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"
#include "firmware/hpm_board/app/src/utility/ring_buffer.hpp"

namespace libhcs::firmware::can {

using board::CanMode;
using board::CanPort;

class Can : private core::utility::Immovable {
public:
    // 仅凭逻辑 CAN 序号构造, CanPort 经 board::can_port() 推导。Lazy 的参数用
    // 序号而非 port: 每个构造点都不依赖端口表的内容。
    using Lazy = utility::Lazy<Can, data::DataId, size_t>;

    // 波特率为编译期常量; 模式由 board_app.hpp 的 CanPort 表按板固定。
    static constexpr uint32_t kArbitrationBaudrate = 1'000'000;
    static constexpr uint32_t kCanFdDataBaudrate = 5'000'000;

    // 两个位相的采样点, 全场钉在 87.5‰ -- 与总线上其他板一致, 而非 SDK 求解器
    // 的 75% 下限(那一档实测 FD 投递 0/40000, 见构造函数长注释)。EP0 的
    // kGetCanConfig 读回与 kSetCanConfig 核对用的就是这两个编译期值。
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
        , can_index_(board_can_index) {

        // 只允许初始化 PCB 实际存在的端口。单路 hpm5321 的第二个槽位引脚是 LED
        // 阴极, 构造它等于把 PA30/PA31 交给一个不存在的收发器。
        core::utility::assert_always(board_can_index < board::can_port_count());

        // Message RAM 布局是板级事务: 各 SoC 的 MCAN 缓冲区可放的位置不同
        // (专用 AHB RAM 或 section 定位数组), 由板层发放区域 (board_app.cpp)。
        const mcan_msg_buf_attr_t attr = board::can_message_ram(board_can_index);
        auto status = mcan_set_msg_buf_attr(can_base_, &attr);
        core::utility::assert_always(status == status_success);

        const uint32_t can_source_clock_freq = board::init_can(can_base_);

        mcan_config_t config;
        mcan_get_default_config(can_base_, &config);
        config.baudrate = kArbitrationBaudrate;
        config.mode = mcan_mode_normal;
        config.enable_canfd = canfd_;
        if (canfd_) {
            config.baudrate_fd = kCanFdDataBaudrate;

            // 发送延迟补偿 (TDC), 该数据段速率下必需; 此前从未开启 --
            // mcan_get_default_config() 将结构体清零, 该字段保持 false,
            // mcan_init() 会直接清掉 DBTP.TDC。
            //
            // 数据段一位 200 ns, 主采样点在 87.5% = 175 ns。高速 CAN 收发器的
            // TXD->RXD 环路延迟通常 120-255 ns, 不补偿时发送节点自检回读会读到
            // 上一位, 报 bit error。TDC 把回读移到 (实测环路延迟 + TDCO) 处的
            // 二级采样点, 跟随收发器而不是假定它足够快。
            //
            // 这是下方采样点修复的另一半: 那处让总线两端对齐"在哪采样", 这里让
            // 发送自检扛得住自家收发器。症状同族 (PSR.DLEC = bit error, TEC
            // 升入 error-passive, 单方向失败而经典 CAN 正常), 只修一半时漏掉
            // 很容易。
            //
            // mc02 自 FDCAN bring-up 起就开了 (mc02/app/src/can/can.hpp:
            // HAL_FDCAN_EnableTxDelayCompensation, offset =
            // DataPrescaler * DataTimeSeg1)。两板同为 80 MHz 内核时钟下的
            // 5 Mbit 数据段, 本板不补则两者不对称。
            //
            // ssp_offset 保持 0, 让 SDK 从刚解出的位时序推导 TDCO: DBTP.DTSEG1
            // + 2, 单位 mtq。仅当数据段分频为 1 时才等于主采样点 -- TDCO 计的
            // 是 CAN 时钟周期而非 time quanta -- mcan_init() 之后的 assert
            // 钉住了这一前提。
            config.enable_tdc = true;
        }

        // 两个相位都把采样点钉在 87.5%, 因为总线上其他板子实际就是这么跑的。
        //
        // 硬规则是同段所有节点必须在同一点采样 (两个相位之间不必彼此一致 --
        // 这里恰好一致)。参照 CubeMX 板 (mc02、c_board): 两相位均
        // tseg1/tseg2 = 13/2 = 87.5%, 标称相位分频 5 (16 TQ), 数据相位分频 1。
        // 注意这不是厂商表格对 1 Mbit 仲裁相位给出的 ">800 kbit/s 用 75%" 指导
        // -- 实测按指导而非按总线来配会失败: 标称相位留在 SDK 的 75%
        // (59/20, 80 TQ)、只钉数据相位时, 本板的 FD 下行掉到 0/40000,
        // PSR.DLEC = bit1 error。
        //
        // SDK 自己不会到 87.5%: 其窗口是 [750, 875], 求解器爬过 MINIMUM 就停,
        // 永远落在 75.0%, 875 用不到。数据段一位 200 ns, 12.5 个点的分歧让两端
        // 差 25 ns, 接收方锁错位 -- 最初表现为 mc02 不 ACK 本板的 FD 帧
        // (PSR.DLEC = ACK error, TEC 升入 error-passive), 而经典 CAN 与反方向
        // 正常。
        config.can20_samplepoint_min = kNominalSamplePointPerMille;
        config.can20_samplepoint_max = kNominalSamplePointPerMille;
        config.canfd_samplepoint_min = kDataSamplePointPerMille;
        config.canfd_samplepoint_max = kDataSamplePointPerMille;
        // 即使 CAN-FD 也保持默认 8 字节元素: 本总线帧的数据从不超过 8 字节,
        // RAM 占用与经典 CAN 相同。
        config.ram_config.txbuf_dedicated_txbuf_elem_count = 0;
        config.ram_config.txbuf_fifo_or_queue_elem_count = MCAN_TXBUF_SIZE_CAN_DEFAULT;
        config.ram_config.txfifo_or_txqueue_mode = MCAN_TXBUF_OPERATION_MODE_FIFO;
        config.disable_auto_retransmission = true;

        // 经时间戳单元 (TSU) 取 64 位硬件时间戳。本 SoC 的 MCAN 没有可用的内部
        // TSU 时基 (TBCS 被综合固定为 "external"), 每个控制器的 TSU 都由经
        // TBSEL 槽 0 接入的唯一共享 PTPC0 时基供时。PTPC 维护 IEEE-1588
        // {seconds:nanoseconds} 计数器, handle_uplink() 把它折算成微秒。所有
        // 控制器共用 PTPC0, 时间戳在同一时钟上, 跨总线可直接比较。
        config.use_timestamping_unit = true;
        config.tsu_config.enable_tsu = true;
        config.tsu_config.enable_64bit_timestamp = true;
        config.tsu_config.use_ext_timebase = true;
        config.tsu_config.ext_timebase_src = MCAN_TSU_EXT_TIMEBASE_SRC_TBSEL_0;
        config.tsu_config.tbsel_option = MCAN_TSU_TBSEL_PTPC0;
        config.tsu_config.capture_on_sof = true;
        config.tsu_config.prescaler = 1; // 外部时基下不使用
        config.timestamp_cfg.counter_prescaler = 1;
        config.timestamp_cfg.timestamp_selection = MCAN_TIMESTAMP_SEL_EXT_TS_VAL_USED;

        // 外部 TSU 只给被标记为 sync message 的过滤器所接收的帧打时间戳 (仅在
        // CCCR.UTSU 置位时求值)。默认 accept-all 过滤器的 sync_message = 0,
        // 故换成 mask 0 的 accept-all sync 过滤器 (标准帧与扩展帧各一) --
        // 否则任何帧都不会有时间戳。
        mcan_filter_elem_t std_sync_filter{};
        std_sync_filter.filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;
        std_sync_filter.filter_config = MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO0_IF_MATCH;
        std_sync_filter.can_id_type = MCAN_CAN_ID_TYPE_STANDARD;
        std_sync_filter.sync_message = 1U;
        std_sync_filter.filter_id = 0U;
        std_sync_filter.filter_mask = 0U;
        mcan_filter_elem_t ext_sync_filter = std_sync_filter;
        ext_sync_filter.can_id_type = MCAN_CAN_ID_TYPE_EXTENDED;
        config.all_filters_config.std_id_filter_list.filter_elem_list = &std_sync_filter;
        config.all_filters_config.std_id_filter_list.mcan_filter_elem_count = 1;
        config.all_filters_config.ext_id_filter_list.filter_elem_list = &ext_sync_filter;
        config.all_filters_config.ext_id_filter_list.mcan_filter_elem_count = 1;

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

        mcan_init(can_base_, &config, can_source_clock_freq);

        // 上面的自动 TDCO 以 DTSEG1 为单位推导, 但按 mtq 编程, 只有数据段分频
        // 为 1 时才落在采样点上。80 MHz / 5 Mbit 下 SDK 求解器选的正是
        // brp=1 tseg1=13 tseg2=2 (-Dlibhcs_CAN_DIAG=ON 可打印), 但任一数字
        // 变动后求解器可以另选 -- 二级采样点悄悄错位与完全没补偿看起来一样,
        // 在此用 assert 拦住。
        if (canfd_)
            core::utility::assert_always(MCAN_DBTP_DBRP_GET(can_base_->DBTP) == 0U);

        mcan_enable_interrupts(can_base_, kEnabledInterrupts);
        // CAN RX 是转发关键路径 (电机反馈 -> 主机)。优先级 3, 高于 USB (2) 与
        // UART (1) -- 保证 CAN 帧不被批量 USB 传输或 DMA 回调拖延。
        intc_m_enable_irq_with_priority(port.irq_num, 3);
    }

    [[nodiscard]] data::DataId data_id() const { return data_id_; }

    // 控制器错误状态, 供 EP0 状态查询 (usb/vendor_control.cpp)。每次直接读
    // 寄存器而非缓存副本: PSR.LEC 读后自清为"无变化", 缓存副本会永远报旧
    // 错误, 而重读则会对其他读者隐瞒错误。读者只有一个。
    struct Status {
        uint8_t tec, rec, last_error, data_last_error, flags;
        uint32_t tx_occurred, tx_cancelled, rx_frames, rx_fifo_level;
    };

    [[nodiscard]] Status status() const {
        const uint32_t psr = can_base_->PSR;
        const uint32_t ecr = can_base_->ECR;
        uint8_t flags = 0;
        if ((psr >> 5) & 1U)
            flags |= 1U << 0; // error passive 态
        if ((psr >> 6) & 1U)
            flags |= 1U << 1; // warning 态
        if ((psr >> 7) & 1U)
            flags |= 1U << 2; // bus off 态
        return {
            .tec = static_cast<uint8_t>(ecr & 0xFFU),
            .rec = static_cast<uint8_t>((ecr >> 8) & 0x7FU),
            .last_error = static_cast<uint8_t>(psr & 0x7U),
            .data_last_error = static_cast<uint8_t>((psr >> 8) & 0x7U),
            .flags = flags,
            .tx_occurred = can_base_->TXBTO,
            .tx_cancelled = can_base_->TXBCF,
            .rx_frames = forwarded_frames_,
            .rx_fifo_level = can_base_->RXF0S & 0x7FU,
        };
    }

    // 转发热路径 -- out-of-line 定义在 can.cpp 的 ILM (.fast) 段, 消除最坏
    // 转发延迟中的 FLASH-XIP 取指抖动。理由及为何不内联在类内见 can.cpp。
    // handle_uplink 至多从 RX FIFO0 读一帧并返回是否消费了帧, ISR 据此循环
    // 排空 FIFO。
    void handle_downlink(const data::CanDataView& data);
    bool handle_uplink(core::protocol::FieldId field_id, core::protocol::Serializer& serializer);
    void irq_handler();

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

    // 软件发送队列中等待的帧数。USB 下行 arm 策略据此判断再收一个 OUT 包是否
    // 会撑爆队列; 见 usb/vendor.hpp。
    [[nodiscard]] size_t transmit_queue_depth() const { return transmit_buffer_.readable(); }

    static constexpr size_t kTransmitQueueSize = 64;

    // 中断处理的一轮: 确认 `flags` 并处理之。从 irq_handler 拆出, 后者可重测
    // IR 并重复调用; 只处理一轮就返回会丢中断, 原因见该处注释。
    void handle_interrupt_flags(uint32_t flags);

private:
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

    // 本控制器在 board::kCanPorts 中的位置。存下来而非从 data_id 推导: 6E8Y 用
    // kCan0..kCan3, 5321 用 kCan1..kCan2, 减去 kCan1 得不到板内序号。
    std::size_t can_index() const { return can_index_; }

    // 读出并归一化一帧 RX FIFO。返回 `true` 表示消费了一个元素; `valid` 区分
    // 可表示的帧与被 8 字节 RX 元素截断存储的 FD 负载。
    bool read_uplink(data::CanDataView& out, uint8_t storage[8], bool& valid);
    static void serialize_uplink(
        core::protocol::FieldId field_id, const data::CanDataView& data,
        core::protocol::Serializer& serializer);

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
    const bool canfd_;
    const std::size_t can_index_;

    // poll() 用的中断记账。irq_count_ 仅 ISR 写、仅主循环读, 普通 32 位计数
    // 即可 -- RV32 上对齐的读写是原子的, 且只关心变化而非具体值。
    uint32_t irq_count_ = 0;
    // 开机以来交给 serializer 的帧数。仅接收路径 (ISR 上下文) 写、仅主循环的
    // EP0 状态查询读 -- RV32 上对齐的 32 位读写是原子的, 只关心变化, 无需同步。
    uint32_t forwarded_frames_ = 0;

    // 接收中断进入时的 CSR_MCYCLE, 帧序列化完成后闭合。写与读都在同一中断内。
    uint32_t uplink_opened_at_ = 0;
    uint32_t watchdog_irq_count_ = 0;
    bool watchdog_armed_ = false;

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
        uint8_t data[8];
    };
    static_assert(sizeof(QueuedFrame) == 16);

    utility::RingBuffer<QueuedFrame, kTransmitQueueSize> transmit_buffer_;
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

// 本 PCB 实有控制器中最深的软件发送队列。USB 下行 arm 策略以此限流: 任意一路
// 总线积压就足以开始丢帧, 与是哪一路无关。以 can_count() 为界并用 try_get()
// 保护, 理由同其他对 can_array 的循环: 单路 hpm5321 的尾部槽位从未构造。
inline size_t max_transmit_queue_depth() {
    size_t depth = 0;
    for (size_t i = 0; i < can_count(); ++i) {
        if (const Can* can = can_array[i].try_get())
            depth = std::max(depth, can->transmit_queue_depth());
    }
    return depth;
}

} // namespace libhcs::firmware::can
