#include "firmware/hpm_board/app/src/can/can.hpp"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <iterator>

#include "core/include/libhcs/protocol/can_dlc.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/hpm_board/app/src/diag/can_diag.hpp"
#include "firmware/hpm_board/app/src/diag/latency.hpp"
#include "firmware/hpm_board/app/src/dmtool/dm_adapter.hpp"

// CAN 转发热路径在此以 out-of-line 方式定义于 ILM (.fast) 段, 而非内联在
// can.hpp。ILM 零等待、永不 I-cache miss, 从而把 FLASH-XIP 取指抖动从最坏
// 转发延迟中去除。out-of-line (而非类内 inline) 是刻意的: 放入 .fast 的
// inline/COMDAT 函数体会与这里的普通 .fast 函数冲突 (GCC "section type
// conflict")。
//
// HPM5321 的板级链接脚本还按名把本胶水调用的部分函数拉进 ILM: mcan_read_rxfifo、
// write_can (已内联到此处) 所经的上行批量分配器、下行反序列化器。仍在 FLASH、
// 每帧到达的: 接收侧的 mcan_get_timestamp_from_received_message 与 memcpy,
// 发送侧的 mcan_transmit_via_txfifo_nonblocking。把它们连同 memset/memmove、
// HostSession 回调与上行访问器也搬进去, 2026-09-11 实测无变化: 8+8 轮下
// CAN 往返 p50 99.7 us, p99 ~122 us, p99.9 ~126 us。

namespace libhcs::firmware::can {

template <DownlinkFramePolicy Policy>
bool Can::submit_downlink(const data::CanDataView& data, Policy policy) {
    mcan_tx_frame_t frame{};
    if (data.is_extended_can_id) {
        frame.use_ext_id = true;
        frame.ext_id = data.can_id;
    } else {
        frame.use_ext_id = false;
        frame.std_id = data.can_id;
    }
    // 端口模式是上限: 经典模式下无论请求什么都发经典帧, BRS 只在 FD 帧上有意义。
    const bool send_fd = canfd_ && policy.wants_fd();
    frame.canfd_frame = send_fd;
    frame.bitrate_switch = send_fd && policy.wants_bitrate_switch();
    frame.rtr = data.is_remote_transmission;

    // DLC: <=8 字节按字节数直写; 长帧(负载 12-64 字节)经 DLC 编解码取线上码
    // (9-15) -- 4 位字段: 字节数 16 直写会截断成 0, 线上 DLC 0, 接收方全丢。
    // DMTool 路径由策略给定线上 DLC; libhcs 下行长帧在此推导, 但先验证本帧
    // 确实以 FD 发出: 会话建立后 DMTool 可把总线切回经典, 主机侧的门禁与总线
    // 的实际模式之间存在竞态窗口, 经典帧没有长帧的表达, 丢弃而非直写。
    core::utility::assert_debug(data.can_data.size() <= 64);
    if (const auto explicit_dlc = policy.explicit_dlc()) {
        frame.dlc = *explicit_dlc;
    } else if (data.can_data.size() > 8) {
        // 9-11 等不在 DLC 表内的长度没有线上表达, 同样丢弃。
        const uint8_t wire_dlc = core::protocol::dlc_from_payload_len(data.can_data.size());
        if (!send_fd || wire_dlc == core::protocol::kDlcInvalid)
            return true; // 有意丢弃: 该长度在本端口没有线上表达, 不 hold
        frame.dlc = wire_dlc;
    } else {
        frame.dlc = static_cast<uint8_t>(data.can_data.size());
    }
    if (!data.can_data.empty())
        std::memcpy(frame.data_8, data.can_data.data(), data.can_data.size());

    // 控制器有空间就直写, 只在 TX FIFO 满时才排队。FIFO 有 32 元素, 但主机
    // 送入的速率不受总线消化速率约束: USB 一次交出上个 (micro)frame 以来积压
    // 的全部, 平均速率远低于总线容量时突发仍可能超过 32。以前直写失败就放弃,
    // 突发被整批丢弃, 唯一迹象是青色 LED。
    //
    // 队列一旦非空就不得绕过, 否则后来的帧会越过先来的。尽管 RingBuffer 对
    // peek_front() 有"仅消费者"警告, 在这里检查是安全的: 生产者
    // (handle_downlink, 经 tud_task 到达) 与消费者 (try_transmit) 都在主循环
    // 同一线程上运行。
    //
    // 只在溢出时排队, 常见路径不添加任何额外工作。
    if (transmit_buffer_.peek_front() == nullptr
        && mcan_transmit_via_txfifo_nonblocking(can_base_, &frame, nullptr) == status_success) {
        diag::latency::close_downlink();
        return true;
    }

    // 压缩进队列元素: T0/T1 加至多 64 个数据字节 (DMTool 长帧引入)。mcan_tx_frame_t
    // 开头的字正是 T0/T1, 可直接拷贝; 下方 static_assert 钉住这一布局假设。
    static_assert(offsetof(mcan_tx_frame_t, data_8) == 8);
    QueuedFrame queued;
    std::memcpy(queued.header, &frame, sizeof(queued.header));
    std::memcpy(queued.data, frame.data_8, sizeof(queued.data));

    // 队列满时返回 false, 由调用方决定丢弃还是施加背压(帧留在 USB 端点等重发)。
    // LED 与 note_tx_fail 只在真正丢弃的路径上由入口函数点亮。
    if (!transmit_buffer_.emplace_back(queued))
        return false;
    // 无论入队是否成功: 被拒绝说明队列已满, 两种情况队列都非空。见
    // Can::drain_pending_transmits()。
    transmit_pending_mask_ |= 1U << can_index();
    return true;
}

ATTR_PLACE_AT(".fast")
bool Can::handle_downlink(const data::CanDataView& data) {
    // 帧类型跟总线走, 不跟帧走。曾按主机头部位逐帧选择; 该位已整体废弃 (见
    // core/src/protocol/protocol.hpp 的 CanHeaderLayout), 模式即本控制器
    // 当前的 FD/经典模式, 主机在构造握手时经 EP0 读回
    // (libhcs/protocol/vendor_control.hpp)。再读头部位会让主机与板子对线上
    // 已经定好的帧产生分歧。
    // libhcs 下行在队列满时丢弃(2026-09-14 决策): 过期控制帧重发不如丢。
    if (submit_downlink(data, PortFrame{}))
        return true;
    led::led->downlink_buffer_full();
    diag::note_tx_fail(can_index());
    return false;
}

// DMTool 仿真路径 (FLASH, 非热路径): 帧型按主机逐帧的请求走, 端口能力是上限。
// 经典模式的端口上请求 FD 会被降级成经典帧 -- 真适配器也发不出 FD; BRS 只在
// 实际发 FD 帧时才有意义。达妙电机的 bootloader 是经典 CAN, 升级流程逐帧带的
// 就是"非 FD", 一律按端口模式发 FD 会让这些帧进不了电机。
bool Can::handle_downlink_as(const data::CanDataView& data, RequestedFrame frame) {
    // 队列满时返回 false: DMTool 适配器侧对 0x03 施加背压(不重挂 OUT 端点),
    // 帧留在 USB 端点等重发, 突发零丢帧(2026-09-22, 电机 IAP 实测)。
    return submit_downlink(data, frame);
}

ATTR_PLACE_AT(".fast")
void Can::drain_transmit_queue() {
    while (const QueuedFrame* queued = transmit_buffer_.peek_front()) {
        // 从压缩记录重建 SDK 帧。零初始化, 使得无论头部 DLC 取多少, 高于本协议
        // 8 字节上限的数据字都有定义值。
        mcan_tx_frame_t frame{};
        std::memcpy(&frame, queued->header, sizeof(queued->header));
        std::memcpy(frame.data_8, queued->data, sizeof(queued->data));

        // FIFO 满: 其余留到下一轮, 本控制器的 pending 位同样保留。
        if (mcan_transmit_via_txfifo_nonblocking(can_base_, &frame, nullptr) != status_success)
            return;
        // 队列 API 把所有权移交给回调; 丢弃已完成的帧本就不需要移动它。
        // NOLINTNEXTLINE(cppcoreguidelines-rvalue-reference-param-not-moved)
        transmit_buffer_.pop_front([](QueuedFrame&&) noexcept {});
    }
    // 只有 peek_front() 已发现队列为空才会执行到这里。
    transmit_pending_mask_ &= ~(1U << can_index());
}

// Can::drain_pending_transmits() 的 out-of-line 一半: 只在某个控制器的硬件
// FIFO 溢出到软件队列时到达。置位者只有本 PCB 实有的控制器, 下标到不了
// can_array 未构造的槽位。
ATTR_PLACE_AT(".fast")
void Can::drain_pending_transmits_slow() {
    uint32_t pending = transmit_pending_mask_;
    for (size_t index = 0; pending != 0; ++index, pending >>= 1U) {
        if ((pending & 1U) != 0U)
            can_array[index]->drain_transmit_queue();
    }
}

// Can::drain_pending_transmits() 背后的 debug 构建不变量检查。
bool Can::transmit_queues_empty() { return max_transmit_queue_depth() == 0; }

ATTR_PLACE_AT(".fast")
bool Can::read_uplink(data::CanDataView& data, uint8_t storage[64], bool& valid) {
    valid = false;
    data = {};
    mcan_rx_message_t rx;
    if (mcan_read_rxfifo(can_base_, 0, &rx) != status_success)
        return false;

    // rx.dlc 是线上原始的 4 位 DLC, 不是字节数, 0-15 每个值都可能从其他节点
    // 合法到来 -- 必须在此归一化, 不能信任下游 (serializer 会拒绝超出契约的
    // view, 而该拒绝不能变成针对外部输入的 assert):
    //   - 远程帧没有数据字段; 其 DLC 编码的是请求长度, 线协议无法表达, 按空
    //     负载转发。(ISO CAN-FD 没有远程帧, FD 长帧 + 远程的组合在硬件上就
    //     不存在, 线协议同样定为保留。)
    //   - 经典帧可带 DLC 9-15, CAN 规范要求按 8 字节处理。
    //   - FD 帧 DLC 0-15 全部经 DLC 表直映 (短帧值与经典一致, 长帧 12-64
    //     字节): RX 元素数据字段已配成 64 字节 (rxfifos[0].data_field_size),
    //     负载完整, 记录流用 IsLongFrame 编码承载。
    size_t data_length = 0;
    if (!rx.rtr)
        data_length = rx.canfd_frame ? std::min<size_t>(core::protocol::payload_length(rx.dlc), 64)
                                     : std::min<size_t>(rx.dlc, 8);

    data.is_extended_can_id = rx.use_ext_id;
    data.is_remote_transmission = rx.rtr;
    data.can_id = data.is_extended_can_id ? rx.ext_id : rx.std_id;
    if (data_length != 0)
        std::memcpy(storage, rx.data_8, data_length);
    data.can_data = {reinterpret_cast<const std::byte*>(storage), data_length};

    // SOF 处捕获的 64 位 TSU 时间戳, 来自共享 PTPC0 时基。PTPC 给出 IEEE-1588
    // {seconds:nanoseconds} 对 (高 32 位为秒, 低 32 位为纳秒, 取值
    // [0, 1e9)); 上报纳秒除以 kTsNsPerUs (160 MHz 时为 960 而非 1000) 即换成
    // 微秒, 一步同时抵消 PTPC 的数字步进误差。
    //
    // 换算刻意只用 32 位: RV32 没有 64 位除法指令, 这里的 64/32 除法会落成
    // __udivdi3 库循环 (约数百周期), 且在最高优先级 ISR 里。把秒与纳秒两个字
    // 分开,
    //   us = sec * (1e9 / 960) + sec * (640 / 960 == 2/3) + ns / 960
    // 只含常量除法, GCC 降为乘加移位。相对精确商每次至多截断 1 us, 误差不
    // 累积。结果约每 71.6 min 回绕一次; 主机只用差值, 回绕安全。
    // status_mcan_timestamp_not_exist (帧未被 sync 过滤器匹配) 保持
    // std::nullopt。
    mcan_timestamp_value_t ts_value;
    if (mcan_get_timestamp_from_received_message(can_base_, &rx, &ts_value) == status_success
        && ts_value.is_64bit) {
        const auto sec = static_cast<uint32_t>(ts_value.ts_64bit >> 32);
        const auto ns = static_cast<uint32_t>(ts_value.ts_64bit);
        data.timestamp_us =
            (sec * (1'000'000'000U / kTsNsPerUs)) + ((sec * 2U) / 3U) + (ns / kTsNsPerUs);
    }

    valid = true;
    return true;
}

ATTR_PLACE_AT(".fast")
void Can::serialize_uplink(
    core::protocol::FieldId field_id, const data::CanDataView& data,
    core::protocol::Serializer& serializer) {

    const auto result = serializer.write_can(field_id, data);
    if (result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]]
        led::led->uplink_buffer_full();
    // 经上述归一化后, 线上输入到不了这里; 只防内部契约回归。
    core::utility::assert_always(
        result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
}

ATTR_PLACE_AT(".fast")
bool Can::handle_uplink(core::protocol::FieldId field_id, core::protocol::Serializer& serializer) {
    data::CanDataView data;
    uint8_t storage[64];
    bool valid = false;
    if (!read_uplink(data, storage, valid))
        return false;
    if (valid) {
        serialize_uplink(field_id, data, serializer);
        // 转发路径上无条件自增一次 -- 旁边的 diag 计数器不同, 这个必须存在于
        // 发行镜像: 主机靠它区分"这条总线根本没收到帧"与"收到了但下游丢弃了",
        // 这个区分是每次死总线排查的第一个分叉。
        ++forwarded_frames_;
        diag::latency::close_uplink(uplink_opened_at_);
        diag::note_frame(can_index());
    }

    return true;
}

// 不进 .fast: 没有 libhcs 会话时才走到, ILM 留给 libhcs 热路径。
void Can::drain_without_session() {
    if (dmtool::capture_enabled(can_index())) {
        while (handle_dm_uplink()) {}
        return;
    }
    mcan_rx_message_t rx;
    while (mcan_read_rxfifo(can_base_, 0, &rx) == status_success) {}
}

bool Can::handle_dm_uplink() {
    mcan_rx_message_t rx;
    if (mcan_read_rxfifo(can_base_, 0, &rx) != status_success)
        return false;

    // rx.dlc 就是线上 DLC 码(0-15, 元素 R1 原样)。FD 长帧(DLC 9-15)元素已扩
    // 到 64 字节, 负载完整; 经典帧的 DLC 9-15 按规范只有 8 字节有效, 记录按 8
    // 交付(DMTool 按记录 DLC 读负载, 归一避免多读垃圾)。记录负载 =
    // payload_length(DLC) 字节, 数据不足部分由编码端补 0。
    uint8_t wire_dlc = static_cast<uint8_t>(rx.dlc);
    if (!rx.canfd_frame && wire_dlc > 8U)
        wire_dlc = 8U;

    const dmtool::protocol::CanFrameFlags flags{
        .extended = rx.use_ext_id != 0,
        .remote = rx.rtr != 0,
        .fd = rx.canfd_frame != 0,
        .bitrate_switch = rx.bitrate_switch != 0,
    };
    dmtool::CanFrameEvent event{
        .id = rx.use_ext_id ? rx.ext_id : rx.std_id,
        .dlc = wire_dlc,
        .flags = flags,
    };
    if (rx.rtr == 0)
        std::memcpy(
            event.data.data(), rx.data_8,
            std::min<size_t>(dmtool::protocol::payload_length(wire_dlc), event.data.size()));

    mcan_timestamp_value_t ts_value;
    if (mcan_get_timestamp_from_received_message(can_base_, &rx, &ts_value) == status_success
        && ts_value.is_64bit) {
        event.timestamp_sec = static_cast<uint32_t>(ts_value.ts_64bit >> 32);
        event.timestamp_ns = static_cast<uint32_t>(ts_value.ts_64bit);
    }

    dmtool::push_can_rx(can_index(), event);
    ++forwarded_frames_;
    return true;
}

// DMTool SETUP_BUARD 的运行时重配(声明见 can.hpp)。
//
// 用低级位时序路径(use_lowlevel_timing_setting)直接写命令给定的 TQ 参数, 而非
// 折算成波特率再交给 SDK 求解器 -- DMTool 的 seg1/seg2/分频就是 TQ 语义, 照抄
// 才能保证 DMTool 界面上显示的采样点与板子实际一致; 求解器永远收敛到自己的
// 采样点窗口(见构造函数 87.5% 的注释), 会悄悄改写用户的选择。
//
// FD/经典切换: DMTool 界面的 CAN2.0/FDCAN 开关随命令的 fd 字节到来, 对 FD
// 控制器关掉 enable_canfd 即经典模式; canfd_ 运行时标志随之更新, 发送帧类型
// (handle_downlink)与 GET_BAUDRATE 的应答(is_fd)自动跟随。
//
// 数据段 TDC: SDK 的自动 TDCO 公式(DTSEG1+1 个 mtq)只在数据段分频为 1 时落在
// 采样点上(TDCO 以 CAN 时钟周期计, 见构造函数注释), 分频为 2 的预设(5M/4M/
// 2.5M/2M)必须显式给 ssp_offset = 分频 × (seg1+1)。分频 > 2 时 SDK 拒绝 TDC
// (收发器环路延迟在半个位时间内尚可自检), 关掉并接受。
//
// 不进 .fast: 只在 DMTool 配置按钮时运行, ILM 留给转发热路径。
bool Can::reconfigure_timing(bool fd, PhaseTiming nominal, PhaseTiming data) {
    // 改速窗口内不得有在途发送(与 handle_downlink 同线程, 此检查充分)。
    if (transmit_buffer_.readable() != 0)
        return false;

    mcan_config_t config;
    mcan_get_default_config(can_base_, &config);
    config.mode = mcan_mode_normal;
    config.enable_canfd = fd;

    const mcan_bit_timing_param_t nominal_param{
        .prescaler = static_cast<uint16_t>(nominal.prescaler),
        .num_seg1 = static_cast<uint16_t>(nominal.seg1),
        .num_seg2 = static_cast<uint16_t>(nominal.seg2),
        .num_sjw = static_cast<uint8_t>(nominal.sjw),
        .enable_tdc = false,
    };
    mcan_bit_timing_param_t data_param{
        .prescaler = static_cast<uint16_t>(data.prescaler),
        .num_seg1 = static_cast<uint16_t>(data.seg1),
        .num_seg2 = static_cast<uint16_t>(data.seg2),
        .num_sjw = static_cast<uint8_t>(data.sjw),
        .enable_tdc = false,
    };

    if (fd) {
        // 数据段分频 ≤ 2 才允许 TDC(SDK 约束)。TDCO 必须落在收发器环路延迟之后
        // (本板实测: SDK 自动值 seg1+1 = 14 mtq = 175 ns 可用, 更早的采样会读到
        // 还没回来的位 -> bit error -> 帧被丢), 所以:
        //   分频 1: ssp_offset = 0, 走 SDK 自动公式(seg1+1, 即实测可用的那套);
        //   分频 2: 显式给 TDCO = 分频 x (seg1 + 1 + seg2/2) -- 数据位中点之后,
        //           5M 预设(2,5,2)下 = 14 mtq, 与分频 1 的实测可用值相同。
        if (data.prescaler == 1U) {
            data_param.enable_tdc = true;
            config.enable_tdc = true;
        } else if (data.prescaler == 2U) {
            const uint32_t ssp = data.prescaler * (data.seg1 + 1U + data.seg2 / 2U);
            data_param.enable_tdc = true;
            config.enable_tdc = true;
            config.tdc_config.ssp_offset = static_cast<uint8_t>(ssp);
            config.tdc_config.filter_window_length = static_cast<uint8_t>(ssp);
        }
        config.canfd_timing = data_param;
        config.can_timing = nominal_param;
    } else {
        // 经典模式: 只有标称相位有意义, 数据相位参数硬件不使用。
        config.can_timing = nominal_param;
    }
    config.use_lowlevel_timing_setting = true;

    // 时间戳单元与同步过滤器: 与构造函数逐字相同 -- 时间戳是 DMTool 数据帧
    // 的一部分, 重配后必须继续打戳。
    config.use_timestamping_unit = true;
    config.tsu_config.enable_tsu = true;
    config.tsu_config.enable_64bit_timestamp = true;
    config.tsu_config.use_ext_timebase = true;
    config.tsu_config.ext_timebase_src = MCAN_TSU_EXT_TIMEBASE_SRC_TBSEL_0;
    config.tsu_config.tbsel_option = MCAN_TSU_TBSEL_PTPC0;
    config.tsu_config.capture_on_sof = true;
    config.tsu_config.prescaler = 1;
    config.timestamp_cfg.counter_prescaler = 1;
    config.timestamp_cfg.timestamp_selection = MCAN_TIMESTAMP_SEL_EXT_TS_VAL_USED;
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
    apply_message_ram_layout(config);
    // 本函数只被 DMTool 仿真调用, 这里是全仓库唯一开自动重传的地方。libhcs
    // 不重传是刻意的(过期的控制指令重发不如丢掉), 但 DMTool 是通用适配器:
    // 它的电机固件升级一块镜像要连发 1025 帧、全部成功才等到一个 "OK", 丢一帧
    // 整块作废, 而丢帧对上位机不可见。libhcs 会话建立时 restore_default_timing()
    // 会连同时序一起把不重传恢复回去。
    config.disable_auto_retransmission = false;

    mcan_deinit(can_base_);
    const mcan_msg_buf_attr_t attr = board::can_message_ram(can_index_);
    (void)mcan_set_msg_buf_attr(can_base_, &attr);

    const bool applied =
        mcan_init(can_base_, &config, can_clock_mhz_ * 1'000'000U) == status_success;
    if (!applied) [[unlikely]] {
        // 参数被 SDK 判为非法: 用编译期时序救回端口, 让总线上其他节点保持可用。
        mcan_deinit(can_base_);
        (void)mcan_set_msg_buf_attr(can_base_, &attr);

        mcan_config_t restore;
        mcan_get_default_config(can_base_, &restore);
        restore.baudrate = kArbitrationBaudrate;
        restore.mode = mcan_mode_normal;
        restore.enable_canfd = canfd_;
        if (canfd_) {
            restore.baudrate_fd = kCanFdDataBaudrate;
            restore.enable_tdc = true;
        }
        restore.can20_samplepoint_min = kNominalSamplePointPerMille;
        restore.can20_samplepoint_max = kNominalSamplePointPerMille;
        restore.canfd_samplepoint_min = kDataSamplePointPerMille;
        restore.canfd_samplepoint_max = kDataSamplePointPerMille;
        // 救援路径同样走共用布局: 这里原本是另一套深度(过滤器 1+1/RX 20/TX 14),
        // 连同 SDK 默认的 32 项 TX event FIFO 合计 2716 字节, 超出控制器的 2560,
        // mcan_init 会回 status_mcan_ram_out_of_range -- 而返回值在这里是被丢弃的,
        // 于是"救援"本身把端口救死。
        apply_message_ram_layout(restore);
        restore.disable_auto_retransmission = true;
        restore.use_timestamping_unit = true;
        restore.tsu_config.enable_tsu = true;
        restore.tsu_config.enable_64bit_timestamp = true;
        restore.tsu_config.use_ext_timebase = true;
        restore.tsu_config.ext_timebase_src = MCAN_TSU_EXT_TIMEBASE_SRC_TBSEL_0;
        restore.tsu_config.tbsel_option = MCAN_TSU_TBSEL_PTPC0;
        restore.tsu_config.capture_on_sof = true;
        restore.tsu_config.prescaler = 1;
        restore.timestamp_cfg.counter_prescaler = 1;
        restore.timestamp_cfg.timestamp_selection = MCAN_TIMESTAMP_SEL_EXT_TS_VAL_USED;
        const mcan_filter_elem_t std_restore = std_sync_filter;
        restore.all_filters_config.std_id_filter_list.filter_elem_list = &std_restore;
        restore.all_filters_config.std_id_filter_list.mcan_filter_elem_count = 1;
        const mcan_filter_elem_t ext_restore = ext_sync_filter;
        restore.all_filters_config.ext_id_filter_list.filter_elem_list = &ext_restore;
        restore.all_filters_config.ext_id_filter_list.mcan_filter_elem_count = 1;
        (void)mcan_init(can_base_, &restore, can_clock_mhz_ * 1'000'000U);
        return false;
    }

    canfd_ = fd;
    mcan_enable_interrupts(can_base_, kEnabledInterrupts);
    return true;
}

// 恢复编译期位时序与端口 FD 模式(声明见 can.hpp)。配置序列与构造函数一致,
// 仅是运行时再次执行; 消息 RAM 属性与 PTPC 时基不受 mcan_deinit 影响, 重下发
// 是幂等的。
bool Can::restore_default_timing() {
    if (transmit_buffer_.readable() != 0)
        return false;

    mcan_config_t config;
    mcan_get_default_config(can_base_, &config);
    config.baudrate = kArbitrationBaudrate;
    config.mode = mcan_mode_normal;
    config.enable_canfd = port_fd_;
    if (port_fd_) {
        config.baudrate_fd = kCanFdDataBaudrate;
        config.enable_tdc = true;
    }
    config.can20_samplepoint_min = kNominalSamplePointPerMille;
    config.can20_samplepoint_max = kNominalSamplePointPerMille;
    config.canfd_samplepoint_min = kDataSamplePointPerMille;
    config.canfd_samplepoint_max = kDataSamplePointPerMille;
    apply_message_ram_layout(config);
    config.disable_auto_retransmission = true;
    config.use_timestamping_unit = true;
    config.tsu_config.enable_tsu = true;
    config.tsu_config.enable_64bit_timestamp = true;
    config.tsu_config.use_ext_timebase = true;
    config.tsu_config.ext_timebase_src = MCAN_TSU_EXT_TIMEBASE_SRC_TBSEL_0;
    config.tsu_config.tbsel_option = MCAN_TSU_TBSEL_PTPC0;
    config.tsu_config.capture_on_sof = true;
    config.tsu_config.prescaler = 1;
    config.timestamp_cfg.counter_prescaler = 1;
    config.timestamp_cfg.timestamp_selection = MCAN_TIMESTAMP_SEL_EXT_TS_VAL_USED;
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

    mcan_deinit(can_base_);
    const mcan_msg_buf_attr_t attr = board::can_message_ram(can_index_);
    (void)mcan_set_msg_buf_attr(can_base_, &attr);
    const bool applied =
        mcan_init(can_base_, &config, can_clock_mhz_ * 1'000'000U) == status_success;
    if (applied) [[likely]]
        canfd_ = port_fd_;
    mcan_enable_interrupts(can_base_, kEnabledInterrupts);
    return applied;
}

ATTR_PLACE_AT(".fast")
void Can::irq_handler() {
    // 在读标志之前先计数, 使"已送达但为空"的中断也可见: 入口计数冻结正是区分
    // "中断不再到来"与"控制器不再接收"的信号。
    diag::note_isr_entry(can_index());
    // 在此打戳, 由下面的 handle_uplink 消费: 两者都运行在同一中断里, 普通成员
    // 无需同步。
    uplink_opened_at_ = diag::latency::now();
    irq_count_++;

    uint32_t flags = mcan_get_interrupt_flags(can_base_);

    if (!flags) [[unlikely]]
        return;

    // 先处理再重读, 直到 IR 显示已使能源清空才返回。正确性靠重读保证, 不是
    // 优化 -- 没有它控制器会永久失聪:
    //
    // M_CAN 的中断线电平由 (IR & IE) 驱动, 但 PLIC 网关按 EDGE 锁存: 只有该
    // 表达式从零变非零才登记新请求。进入时清一次 IR 再去排空, 排空中到来的帧
    // 会再次置起 RF0N, 之后无人清除 -- 线保持高, 不再产生边沿, 中断永远不再
    // 送达。FIFO 随即充满并保持满, 而总线完全健康。16 kHz/流 下实测到的正是
    // 该状态:
    //   IR = 0x0001000f (RF0N|RF0W|RF0F|RF0L 置位), RXF0S fill = 32, F0F = 1,
    //   RF0L = 1, PSR 干净 (无 bus-off、无 error-passive, act = rx),
    //   PLIC 该源 pending 位为 0, ISR 入口计数冻结。
    // 只有复位能救回来, 这也是它看着像 PLIC claim 丢失而非边沿丢失的原因。
    //
    // 有了循环, 处理期间置起的任何标志都在本次调用内被处理并清除, ISR 只在
    // 观察到 IR 无已使能位后才返回 -- 即线确实为低, 下一帧才能再次产生边沿。
    // 循环必然终止: 一轮排空整个 FIFO 所需时间远小于一个 CAN-FD 帧在线上的
    // 时间。
    do {
        handle_interrupt_flags(flags);
        flags = mcan_get_interrupt_flags(can_base_);
    } while ((flags & kEnabledInterrupts) != 0);
}

// 主循环修复: PLIC 已接受却再未送达的中断请求。
//
// 16 kHz/流 下从运行中的板子读出的实测故障态
// (host/examples/can_stall_probe.cpp):
//   IR = 0x0001000f -> RF0N 置位且 IE 已使能, M_CAN 中断线为高
//   RXF0S fill = 32, F0F = 1, RF0L = 1  -> FIFO 已充满并开始丢帧
//   PSR 干净 (无 bus-off、无 error-passive, act = rx) -> 总线正常
//   PLIC 该源触发类型 = 0 -> LEVEL 触发, 不是边沿
//   PLIC 该源 pending 位 = 0
//   ISR 入口计数冻结, 直至复位
//
// 电平触发网关在线为高时必须保持请求有效; 唯一不满足的状态是"已被 claim、
// 永不完成", 即该源的 completion 丢了。这与 SDK 包在每个 ISR 外的嵌套中断
// 包装一致: 处理期间重开 mstatus.MIE, 返回后才写 PLIC completion。
//
// 不去追查 completion 具体丢在哪, 而是让条件自愈: 对目标未在服务的源写
// completion 的定义是被忽略, 因此除非网关真的卡住, 这里多写一次是无操作;
// 真卡住则释放它, 线仍为高, 立刻重新拉起中断。
//
// 健康路径的代价是一次 MMIO 读加一次比较; 触发条件是中断线连续两轮主循环为
// 高且其间无 ISR 入口。正常时不会发生: 中断使能且源为电平触发时, ISR 会在
// 主循环看第二次之前进入。
void Can::poll() {
    const uint32_t flags = mcan_get_interrupt_flags(can_base_);
    if ((flags & kEnabledInterrupts) == 0) [[likely]] {
        watchdog_armed_ = false;
        return;
    }

    const uint32_t count = irq_count_;
    if (!watchdog_armed_ || count != watchdog_irq_count_) {
        // 首次见到, 或上次之后 ISR 已运行过。线在这里为高只说明有一个中断正在
        // 路上。
        watchdog_armed_ = true;
        watchdog_irq_count_ = count;
        return;
    }

    watchdog_armed_ = false;
    diag::note_irq_recovered(can_index());
    intc_m_complete_irq(irq_num_);
}

ATTR_PLACE_AT(".fast")
void Can::handle_interrupt_flags(uint32_t flags) {
    mcan_clear_interrupt_flags(can_base_, flags);

    if (flags & MCAN_INT_RXFIFO0_NEW_MSG) [[likely]] {
        // 彻底排空 FIFO: RF0N 是状态位不是计数器, 一次中断可能对应多个缓冲帧。
        if (link::uplink_enabled()) {
            auto& serializer = link::uplink_serializer();
            while (handle_uplink(data_id_, serializer)) {}
        } else {
            // 没有 libhcs 会话: 交给 DMTool 或丢弃。整段放在 .fast 之外的单独函数
            // 里, 本函数的 libhcs 分支因此与没有 DMTool 支持时逐条指令相同。
            drain_without_session();
        }
    }

    if (flags
        & (MCAN_INT_BUS_OFF_STATUS | MCAN_INT_WARNING_STATUS | MCAN_INT_ERROR_PASSIVE
           | MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE | MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE)) {
        // 任何错误中断都刷新指示 LED, 错误持续期间指示保持可见。bus-off 是
        // 致命态, 直接获胜; 否则由 Last Error Code (LEC) 指明线上的协议错误。
        // CAN-FD 仲裁相位用 LEC、更快的数据相位用 DLEC -- 仲裁相位无具体错误
        // 码时优先取数据相位码。不带新 LEC 的总线状态变化 (warning/passive)
        // 报 kNone, 只刷新超时 (见 Led::report_can_fault)。
        led::CanFault fault = led::CanFault::kNone;
        if (mcan_is_in_busoff_state(can_base_)) {
            fault = led::CanFault::kBusOff;
            // bus-off 置起 CCCR.INIT 并停住控制器。本板是转发桥, 线上瞬时故障
            // (下游节点拔掉、无 ACK) 不能让 CAN 端口离线到重启为止。清 INIT
            // 启动标准 bus-off 恢复序列 -- 控制器等总线空闲 (129 * 11 个
            // recessive 位), 复位错误计数器后自动恢复。总线持续故障时只是回到
            // bus-off 再重试, 端口保持存活。
            mcan_enter_normal_mode(can_base_);
        } else {
            fault = classify_can_fault(mcan_get_last_error_code(can_base_));
            if (fault == led::CanFault::kNone)
                fault = classify_can_fault(mcan_get_data_phase_last_error_code(can_base_));
        }
        // 指示 LED 按本控制器在 board::kCanPorts 中的位置索引,
        // report_can_fault 的逐控制器状态即以它为键。旧的"第一路否则第二路"
        // 两路测试把第二路之上的所有控制器都压到指示 1 上, 板上有超过两个
        // CAN 口加配套指示表时, CAN2/CAN3 的故障会点亮 CAN1 的 LED 并覆盖
        // CAN2 自己的状态。今天只是潜伏问题 -- 唯一超过 2 路 CAN 的板
        // (hpm6e8y) 的 kCanIndicatorPins 为空, report_can_fault() 对其做边界
        // 检查, 坏下标被丢弃; 正确推导防止这类板一旦配上 LED 就变成真实误报。
        led::led->report_can_fault(static_cast<uint8_t>(can_index()), fault);
    }
}

} // namespace libhcs::firmware::can

namespace libhcs::firmware::board {

ATTR_PLACE_AT(".fast") void can_irq_handler(size_t board_can_index) {
    core::utility::assert_debug(board_can_index < can::kCanCount);

    can::can_array[board_can_index]->irq_handler();
}

} // namespace libhcs::firmware::board
