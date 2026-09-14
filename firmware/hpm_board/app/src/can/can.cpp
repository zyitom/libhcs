#include "firmware/hpm_board/app/src/can/can.hpp"

#include <algorithm>
#include <cstddef>
#include <cstring>

#include "core/src/utility/assert.hpp"
#include "firmware/hpm_board/app/src/diag/can_diag.hpp"
#include "firmware/hpm_board/app/src/diag/latency.hpp"

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

ATTR_PLACE_AT(".fast")
void Can::handle_downlink(const data::CanDataView& data) {
    mcan_tx_frame_t frame{};
    if (data.is_extended_can_id) {
        frame.use_ext_id = true;
        frame.ext_id = data.can_id;
    } else {
        frame.use_ext_id = false;
        frame.std_id = data.can_id;
    }
    // 帧类型跟总线走, 不跟帧走。曾按主机头部位逐帧选择; 该位已整体废弃 (见
    // core/src/protocol/protocol.hpp 的 CanHeaderLayout), 模式即本控制器
    // 编译期的 CanPort::mode, 主机在构造握手时经 EP0 读回
    // (libhcs/protocol/vendor_control.hpp)。再读头部位会让主机与板子对线上
    // 已经定好的帧产生分歧。
    const bool send_fd = canfd_;
    frame.canfd_frame = send_fd;
    frame.bitrate_switch = send_fd;
    frame.rtr = data.is_remote_transmission;

    core::utility::assert_debug(data.can_data.size() <= 8);
    frame.dlc = data.can_data.size();
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
        return;
    }

    // 压缩进队列元素: T0/T1 加至多 8 个数据字节。mcan_tx_frame_t 开头的字正是
    // T0/T1, 可直接拷贝; 下方 static_assert 钉住这一布局假设。
    static_assert(offsetof(mcan_tx_frame_t, data_8) == 8);
    QueuedFrame queued;
    std::memcpy(queued.header, &frame, sizeof(queued.header));
    std::memcpy(queued.data, frame.data_8, sizeof(queued.data));

    if (!transmit_buffer_.emplace_back(queued)) {
        led::led->downlink_buffer_full();
        diag::note_tx_fail(can_index());
    }
    // 无论入队是否成功: 被拒绝说明队列已满, 两种情况队列都非空。见
    // Can::drain_pending_transmits()。
    transmit_pending_mask_ |= 1U << can_index();
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
bool Can::read_uplink(data::CanDataView& data, uint8_t storage[8], bool& valid) {
    valid = false;
    data = {};
    mcan_rx_message_t rx;
    if (mcan_read_rxfifo(can_base_, 0, &rx) != status_success)
        return false;

    // rx.dlc 是线上原始的 4 位 DLC, 不是字节数, 0-15 每个值都可能从其他节点
    // 合法到来 -- 必须在此归一化, 不能信任下游 (serializer 会拒绝超出契约的
    // view, 而该拒绝不能变成针对外部输入的 assert):
    //   - 远程帧没有数据字段; 其 DLC 编码的是请求长度, 线协议无法表达, 按空
    //     负载转发。
    //   - 经典帧可带 DLC 9-15, CAN 规范要求按 8 字节处理。
    //   - DLC > 8 的 FD 帧有 12-64 字节数据, 而 RX 元素数据字段只有 8 字节,
    //     硬件已截断存储, 线协议上限也是 8 字节 -- 转发等于交付被悄悄破坏的
    //     数据。丢帧 (但继续排空)。
    if (rx.canfd_frame && rx.dlc > 8) [[unlikely]]
        return true;
    const size_t data_length = rx.rtr ? 0 : std::min<size_t>(rx.dlc, 8);

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
    uint8_t storage[8];
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
            mcan_rx_message_t rx;
            while (mcan_read_rxfifo(can_base_, 0, &rx) == status_success) {}
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
