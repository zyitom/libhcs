#include "firmware/mc02/app/src/can/can.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include <fdcan.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/spec/mc02/ports.hpp"
#include "core/src/link/port.hpp"
#include "firmware/mc02/app/src/diag/can_diag.hpp"
#include "firmware/mc02/app/src/led/led.hpp"
#include "firmware/mc02/app/src/usb/helper.hpp"

// 把 CAN 转发热路径放进零等待 ITCM(启动时由 app.cpp 从 FLASH 拷入, 见 .itcm 链接段),
// 以消除最坏转发延迟中的 I-cache miss 和 FLASH-XIP 取指抖动。被调用的 HAL 与共享
// serializer 仍在 FLASH -- 只有这层胶水代码跑在 ITCM。函数体保持 out-of-line
// (不写成类内 inline), 因为 inline/COMDAT 函数放自定义 section 会触发 GCC section 类型冲突。
#define libhcs_ITCM __attribute__((section(".itcm")))

namespace libhcs::firmware::can {

namespace vc = libhcs::core::protocol::vendor_control;
namespace link = libhcs::core::link;

libhcs_ITCM uint32_t Can::hardware_free_slots() const noexcept {
    return hal_can_handle_->Instance->TXFQS & FDCAN_TXFQS_TFFL;
}

// 在控制器当前的 put index 处写入一个 Tx 元素并请求发送。调用方必须先查
// hardware_free_slots(): FIFO 满时 TFQPI 仍会读出一个待发送的槽位, 直接写会覆盖未发出的帧。
libhcs_ITCM void Can::push_to_hardware(const TransmitMailboxData& mailbox_data) noexcept {
    auto* hcan = hal_can_handle_;
    const auto put_index = (hcan->Instance->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;

    // NOLINTBEGIN(readability-identifier-naming): mirrors the message RAM word names.
    struct TxMailbox {
        uint32_t TIR;
        uint32_t TDTR;
        uint32_t TDLR;
        uint32_t TDHR;
    };
    // NOLINTEND(readability-identifier-naming)
    auto* target_mailbox = reinterpret_cast<TxMailbox*>(
        hcan->msgRam.TxBufferSA + (put_index * hcan->Init.TxElmtSize * 4U));

    target_mailbox->TIR = mailbox_data.identifier;
    target_mailbox->TDTR = mailbox_data.control;
    target_mailbox->TDLR = mailbox_data.data[0];
    target_mailbox->TDHR = mailbox_data.data[1];

    // 单发作废的清点, 与 hpm_board 的 send_to_fifo() 同法: 一个槽只在上一次发送结束(成功
    // 或作废)后才重新分配, 所以写入前 TXBCF 的这一位就是这个槽上一次的结果, 每次结果正好
    // 看一次; 最近还没被重用的槽(至多 FIFO 深度个)要等下一次重用才计入。代价: 每帧多读
    // 一次 TXBCF(D2 域外设读)。
    const uint32_t bit = 1UL << put_index;
    if ((hcan->Instance->TXBCF & used_tx_slots_ & bit) != 0U) [[unlikely]]
        cancelled_frames_.note();
    used_tx_slots_ |= bit;

    hcan->Instance->TXBAR = bit;
    hcan->LatestTxFifoQRequest = bit;
}

libhcs_ITCM void Can::handle_downlink(const data::CanDataView& data) {
    // 没声明的总线不发: 控制器不在总线上, 主机往它写的帧直接丢弃。
    if (!started_) [[unlikely]]
        return;
    // 本板不广告 kCapCanFdLongFrames(RX FIFO 元素仍为 8 字节, 主机 SDK 侧也已
    // 拒绝长负载), 该守卫防止对端 bug 在无 debug 构建上溢出 8 字节 TX 元素。
    if (data.can_data.size() > 8)
        return;
    TransmitMailboxData mailbox{};

    if (data.is_extended_can_id) {
        mailbox.identifier = (data.can_id << 0) | FDCAN_EXTENDED_ID;
    } else {
        mailbox.identifier = (data.can_id << 18) | FDCAN_STANDARD_ID;
    }
    mailbox.identifier |= data.is_remote_transmission ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;

    const auto dlc = static_cast<uint32_t>(data.can_data.size());

    // 帧类型属于总线而非单帧。曾按主机头部位逐帧选择, 该位已废弃, 模式现由
    // kCanPorts 表按总线固定(见 can.hpp), 主机在构造握手时经 EP0 读回。若仍读
    // 头部位, 主机与板子可能对线上已定死的帧类型各执一词。控制器保持 FD 能力
    // (CubeMX 配置 FDCAN_FRAME_FD_BRS), 对端发来的经典帧仍能接收。
    mailbox.control = dlc << 16;
    if (canfd_)
        mailbox.control |= FDCAN_FD_CAN | FDCAN_BRS_ON;

    if (!data.can_data.empty())
        std::memcpy(mailbox.data, data.can_data.data(), data.can_data.size());

    // FIFO 有空位就直接写控制器, 只有 FIFO 满才入队。这里由 tud_task() 内的
    // tud_vendor_rx_cb 调用, 所以之前的无条件入队会让每一帧都等到主循环末尾的
    // try_transmit(), 排在 DFU 轮询、GPIO 采样、一次 BMI088 SPI 读和 LED 轮询之后
    // (WS2812 换色刷新约 330 us)。这段时间全部加到单向延迟上, 更糟的是每轮长短不一,
    // 长尾正来自这里。
    //
    // 它还限制了突发能力: 队列只由 try_transmit() 排空, 于是一个 USB 包里的帧全积在
    // 队列, 包解析完之前一帧都到不了 32 元素的 FIFO。有效上限就是队列深度本身 --
    // 实测每个下行包只发出 min(N, 16) 帧。
    //
    // 队列非空时不能绕过它, 否则这里写入的帧会插到已排队的帧前面。虽然 RingBuffer
    // 注明 peek_front() 属于消费者, 但从生产者调用是安全的: 生产者(handle_downlink,
    // 经 tud_task 调用)与消费者(try_transmit)都跑在主循环的同一线程。
    if (transmit_buffer_.peek_front() == nullptr && hardware_free_slots() != 0) {
        push_to_hardware(mailbox);
        return;
    }

    const auto copy = [&mailbox](std::byte* storage) noexcept {
        new (storage) TransmitMailboxData{mailbox};
    };
    if (!transmit_buffer_.emplace_back_n(copy, 1)) {
        tx_dropped_.note();
        led::led->downlink_buffer_full();
        diag::note_tx_fail(diag_index());
    }
    // 无论入队成败都置位: 入队被拒说明队列已满, 两种情况下队列都非空。
    // 见 can.hpp 的 Can::drain_pending_transmits()。
    transmit_pending_mask_ |= 1U << diag_index();
}

libhcs_ITCM void Can::handle_uplink(data::DataId field_id, core::protocol::Serializer& serializer) {
    core::utility::assert_always(hal_can_handle_->State == HAL_FDCAN_STATE_BUSY);
    auto* hal_can_instance = hal_can_handle_->Instance;

    // NOLINTBEGIN(readability-identifier-naming): mirrors the message RAM word names.
    struct RxMailbox {
        uint32_t RIR;
        uint32_t RDTR;
        uint32_t RDLR;
        uint32_t RDHR;
    };
    // NOLINTEND(readability-identifier-naming)

    // 在这一次中断内排空整个 RX FIFO0: 把已排队的报文全部处理掉, 而不是每条报文再进
    // 一次中断。这不会增加延迟 -- 中断仍在第一条新报文时触发, 第一条处理得同样快;
    // 循环只是顺带收掉处理期间堆积的报文, 否则它们每条都要多一次 ISR 进出。
    // 净效果是突发延迟更低, 绝不会更高。
    while ((hal_can_instance->RXF0S & FDCAN_RXF0S_F0FL) != 0U) {
        const auto get_index = (hal_can_instance->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;

        auto* rx_mailbox = reinterpret_cast<RxMailbox*>(
            hal_can_handle_->msgRam.RxFIFO0SA
            + (get_index * hal_can_handle_->Init.RxFifo0ElmtSize * 4U));

        const uint32_t rdtr = rx_mailbox->RDTR;
        data::CanDataView can_data{};
        // Rx 元素 R1 的 FDF 位表示本帧以 FD 格式到达。线上协议已无逐帧类型标志
        // (类型属于总线, 见 can.hpp), 读它只为区分 DLC 9..15 的两种含义,
        // 见下方的丢弃与钳位处理。
        const bool rx_fd = (rdtr & FDCAN_FD_CAN) != 0U;
        can_data.is_extended_can_id = static_cast<bool>(rx_mailbox->RIR & 0x40000000U);
        can_data.is_remote_transmission = static_cast<bool>(rx_mailbox->RIR & 0x20000000U);

        if (can_data.is_extended_can_id) {
            can_data.can_id = rx_mailbox->RIR & 0x1FFFFFFFU;
        } else {
            can_data.can_id = (rx_mailbox->RIR & 0x1FFC0000U) >> 18;
        }

        // 帧起始时刻的 16 位硬件时间戳在 R1 bits[15:0](外部计数器 = TIM3, 见 can.hpp 的
        // config_can())。换成共享微帧轴上的位置; 换不出就不带, serializer 直接省掉该
        // 字段。主机的清单没要时间基准时只付第一个判断。
        if (sync::time_sync_on() && longest_frame_age_ns_ != 0U
            && sync::sof_capture::window_covers(longest_frame_age_ns_))
            can_data.sof_stamp = sync::sof_capture::stamp_of(static_cast<uint16_t>(rdtr));

        size_t can_data_length = (rdtr & 0x000F0000U) >> 16;
        if (can_data.is_remote_transmission)
            can_data_length = 0;
        // DLC 是从线上原样读来的。FD 帧 DLC 9..15 表示 12..64 字节数据, 但 RX 元素只存
        // 8 字节(FDCAN_DATA_BYTES_8), 且线协议上限也是 8, 转发出去就是静默截断 --
        // 直接丢弃该帧, 但继续排空 FIFO。经典帧 DLC 9..15 是合法的,
        // 按 CAN 规范表示 8 字节数据。
        if (rx_fd && can_data_length > 8) [[unlikely]] {
            hal_can_instance->RXF0A = get_index;
            continue;
        }
        can_data_length = std::min<size_t>(can_data_length, 8);

        alignas(uint32_t) std::array<std::byte, 8> payload{};
        const uint32_t rdlr = rx_mailbox->RDLR;
        const uint32_t rdhr = rx_mailbox->RDHR;
        std::memcpy(payload.data(), &rdlr, sizeof(uint32_t));
        std::memcpy(payload.data() + 4, &rdhr, sizeof(uint32_t));
        can_data.can_data = {payload.data(), can_data_length};

        // kBadAlloc 表示上行批量池已满, 这一帧没有被序列化。以前这里不检查返回值,
        // 下面的 RXF0A 却照常确认报文, 于是帧被丢掉且毫无记录 -- 没有计数、没有 LED,
        // 上位机也无法区分"丢了"和"根本没来"。在这块全速板(1 ms 帧间隔)上普通负载
        // 就能触发: mc02 的 CAN2<->CAN3 对接使得一个携带两个 CAN 字段的下行包让两个
        // 控制器都收到两帧, 发 2 帧变成要上行 4 帧, 池子随即耗尽。实测包里排第二的那个
        // CAN 字段约 30% 的帧静默消失, 看起来像发送或仲裁故障, 其实都不是。
        //
        // 与 hpm_board 的 Can::serialize_uplink 一致, 那边一直有这个标记。无论如何
        // 下面都会确认报文: 从 RX FIFO 重试会卡住排空循环、连累更新的帧, 所以池满时
        // 仍然丢帧 -- 这里的意义只是让它不再静默: 计进 rx_dropped_, 随端口状态报给
        // 主机 -- "总线有帧但我们丢了"不能看起来像"总线什么都没来", 这是每次排查死
        // 总线的第一个分叉。
        const auto uplink_result = serializer.write_can(field_id, can_data);
        if (uplink_result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]] {
            rx_dropped_.note();
            led::led->uplink_buffer_full();
            diag::note_uplink_drop(diag_index());
        } else {
            diag::note_frame(diag_index());
        }
        core::utility::assert_always(
            uplink_result != core::protocol::Serializer::SerializeResult::kInvalidArgument);

        hal_can_instance->RXF0A = get_index;
    }
}

// 协议错误中断(IR.PEA / PED)里调用, 与 hpm_board 的 ISR 同法: PSR 只读这一次, LEC/DLEC
// 读后自清, 读到的真实错误码交给锁存(唯一的写者)。冷路径, 不放进 ITCM。
void Can::note_bus_errors() {
    const uint32_t psr = hal_can_handle_->Instance->PSR;
    last_bus_error_.note(static_cast<data::CanLastError>(psr & FDCAN_PSR_LEC));
    last_data_bus_error_.note(
        static_cast<data::CanLastError>((psr & FDCAN_PSR_DLEC) >> FDCAN_PSR_DLEC_Pos));
}

// 运行时状态(core/src/link/port_status.hpp), 每个 keepalive 轮次在主循环读一次; 冷
// 路径, 刻意不放进 ITCM 热路径段。HAL 读到的就是 M_CAN 寄存器的原始编码 --
// FDCAN_PROTOCOL_ERROR_* 的 0..7 与 data::CanLastError 同序 -- 无需转换。错误码多半
// 已被协议错误中断读走并锁存, 这里只读: 自己这次读到真实错误就用它, 否则用锁存的。
// 另一个读 PSR 的是 bus-off 恢复路径(为 BusOff 电平), 它会顺带清掉那一刻的错误码。
// 单发作废在 push_to_hardware() 里按发送槽清点。RX FIFO0 溢出(IR.RF0L)不在中断使能
// 掩码里, 中断路径(line0_isr / HAL)都不碰它, 这里每轮看一次、见到就记一次并清掉 --
// 一个计数是"这一轮里溢出过", 接收热路径上不多读一次 IR。
data::CanStatusView Can::read_status() {
    if ((hal_can_handle_->Instance->IR & FDCAN_IR_RF0L) != 0U) {
        hal_can_handle_->Instance->IR = FDCAN_IR_RF0L; // 写 1 清, 只清这一位
        rx_lost_.note();
    }

    FDCAN_ProtocolStatusTypeDef protocol_status{};
    core::utility::assert_always(
        HAL_FDCAN_GetProtocolStatus(hal_can_handle_, &protocol_status) == HAL_OK);
    FDCAN_ErrorCountersTypeDef error_counters{};
    core::utility::assert_always(
        HAL_FDCAN_GetErrorCounters(hal_can_handle_, &error_counters) == HAL_OK);

    const auto last_error = static_cast<data::CanLastError>(protocol_status.LastErrorCode);
    const auto data_last_error = static_cast<data::CanLastError>(protocol_status.DataLastErrorCode);

    uint8_t flags = 0;
    if (protocol_status.ErrorPassive != 0U)
        flags |= data::kCanErrorPassive;
    if (protocol_status.Warning != 0U)
        flags |= data::kCanWarning;
    if (protocol_status.BusOff != 0U)
        flags |= data::kCanBusOff;
    return {
        .tec = static_cast<uint8_t>(error_counters.TxErrorCnt),
        .rec = static_cast<uint8_t>(error_counters.RxErrorCnt),
        .last_error = last_bus_error_.latest(last_error),
        .data_last_error = last_data_bus_error_.latest(data_last_error),
        .flags = flags,
        .tx_cancelled = cancelled_frames_.count(),
        .tx_dropped = tx_dropped_.count(),
        .rx_dropped = rx_dropped_.count(),
        .rx_lost = rx_lost_.count(),
    };
}

// 寄存器预设重构的位时序事实: 分频器与时间段来自 CubeMX 生成的 Init, 内核时钟
// 经 RCC 解析(PLL2Q/PLL3Q/HSI), 与 UART 的实际波特率读回同一算术 -- 永远不是
// "上次请求了什么"。控制器具备 FD 能力, 数据段速率无论当前 TX 模式照报。
link::CanTimingValue Can::timing() const {
    const auto& init = hal_can_handle_->Init;
    const uint32_t clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    const auto rate = [clock_hz](uint32_t prescaler, uint32_t seg1, uint32_t seg2) {
        const uint32_t divisor = prescaler * (1U + seg1 + seg2);
        return divisor != 0U ? clock_hz / divisor : 0U;
    };
    const auto sample_point = [](uint32_t seg1, uint32_t seg2) -> uint16_t {
        const uint32_t total = 1U + seg1 + seg2;
        return total != 0U ? static_cast<uint16_t>((1U + seg1) * 1000U / total) : 0U;
    };
    return {
        .arbitration_baudrate =
            rate(init.NominalPrescaler, init.NominalTimeSeg1, init.NominalTimeSeg2),
        .data_baudrate = rate(init.DataPrescaler, init.DataTimeSeg1, init.DataTimeSeg2),
        .nominal_sample_point = sample_point(init.NominalTimeSeg1, init.NominalTimeSeg2),
        .data_sample_point = sample_point(init.DataTimeSeg1, init.DataTimeSeg2),
    };
}

// kGetPortConfig 的应答: 当前帧型 + 寄存器重构的速率与采样点。控制器保持 FD 能力,
// 经典模式下数据段速率照报(它仍照常解码对端发来的 FD 帧)。
void Can::read_config(vc::CanConfigPayload& out) const {
    const link::CanTimingValue t = timing();
    out = {
        .mode = std::to_underlying(canfd_ ? vc::CanMode::kCanFd : vc::CanMode::kClassic),
        .control = 0,
        .reserved0 = 0,
        .arbitration_baudrate = t.arbitration_baudrate,
        .data_baudrate = t.data_baudrate,
        .nominal_sample_point = t.nominal_sample_point,
        .data_sample_point = t.data_sample_point,
        .reserved1 = 0,
    };
}

libhcs_ITCM bool Can::drain_transmit_queue() {
    core::utility::assert_always(hal_can_handle_->State == HAL_FDCAN_STATE_BUSY);

    // 现在只有撞上 FIFO 满的帧才会进队列, 常见情况是队列为空。所以要在
    // hardware_free_slots() 之前先判空: pop_front_n 按值取 count, 把该调用直接当参数
    // 会导致每次都读 TXFQS, 无论有没有东西要发 -- 那是一次 D2 域(USB 所在域)的外设读,
    // 按主循环频率乘总线数发生。hpm_board 的 Can::try_transmit 用 peek_front() 循环,
    // 没有这个问题。
    size_t sent = 0;
    if (transmit_buffer_.readable() != 0) {
        sent = transmit_buffer_.pop_front_n(
            [this](const TransmitMailboxData& mailbox_data) noexcept {
                push_to_hardware(mailbox_data);
            },
            hardware_free_slots());
    }

    // 只有队列确认已空才清本控制器的 pending 位。若期间硬件 FIFO 又被填满,
    // 帧和位都留给下一轮。见 can.hpp 的 Can::drain_pending_transmits()。
    if (transmit_buffer_.readable() == 0)
        transmit_pending_mask_ &= ~(1U << diag_index());

    return sent != 0;
}

// Can::drain_pending_transmits() 的 out-of-line 一半: 仅当某个控制器的硬件 FIFO
// 溢出进软件队列时才会到达。
libhcs_ITCM void Can::drain_pending_transmits_slow() {
    const uint32_t pending = transmit_pending_mask_;
    if ((pending & (1U << 0U)) != 0U)
        can1->drain_transmit_queue();
    if ((pending & (1U << 1U)) != 0U)
        can2->drain_transmit_queue();
    if ((pending & (1U << 2U)) != 0U)
        can3->drain_transmit_queue();
}

// Can::drain_pending_transmits() 背后的 debug 构建不变量检查。
bool Can::transmit_queues_empty() {
    return can1->transmit_buffer_.readable() == 0 && can2->transmit_buffer_.readable() == 0
        && can3->transmit_buffer_.readable() == 0;
}

// 把控制器撤下总线。标志先落: 之后才到的 RX / bus-off 中断见到它就直接返回。
// HAL_FDCAN_Stop 置 CCCR.INIT 与 CCE: 控制器不再参与总线, 硬件 Tx 请求与 Rx FIFO 随
// CCE 清空(RM0468: TXBRP / RXF0S 等在 CCE 置位时复位); bus-off 期间内核已自行置
// INIT 而 HAL 状态仍是 BUSY, 走的是同一条收尾。位时序、TDC 与过滤器是受保护的
// 配置寄存器, 不受影响, 下一次 start() 直接可用。
void Can::stop() {
    if (!started_)
        return;
    started_ = false;

    constexpr auto ok = HAL_OK;
    core::utility::assert_always(
        HAL_FDCAN_DeactivateNotification(
            hal_can_handle_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF
                                 | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR)
        == ok);
    core::utility::assert_always(HAL_FDCAN_Stop(hal_can_handle_) == ok);
    // INIT 之后 TXBCF 的旧位不再是"上一次发送的结果": 发送槽的历史一并忘掉。
    used_tx_slots_ = 0;

    // 软件队列里排着的帧属于刚结束的会话, 不留给下一个。
    (void)transmit_buffer_.clear();
    transmit_pending_mask_ &= ~(1U << diag_index());
    bus_off_ = false;
    stuck_request_since_ms_ = 0;
    loop::clear(loop::bit(kCanPorts[diag_index()].data_id));
}

void Can::recover_all_stuck_transmits() {
    can1->recover_stuck_transmits();
    can2->recover_stuck_transmits();
    can3->recover_stuck_transmits();
}

// ES0491 (STM32H72x/73x) 2.22.3 的软件守护。本控制器以 DAR 模式运行
// (AutoRetransmission = DISABLE, .ioc 固定): 勘误指出, 仲裁在前两个标识符位上失败时
// 一次发送可能既不上总线也不被取消 -- TXBRP 的请求位永久挂起, 该槽位从此不参与
// 发送, 反复命中后 32 槽硬件 FIFO 逐次耗尽, 端口静默瘫痪。正常 DAR 语义下请求至多
// 一个帧时间(~150 us)内终结, 仲裁丢失立即取消; 挂起超过 kStuckRequestThresholdMs
// 且不在 bus-off(恢复期间请求位合法地长期非零, 见 bus_off_)只剩这一种硬件状态。
//
// 处置按勘误 workaround 取消请求(TXBCR), 释放槽位; 帧与正常 DAR 仲裁失败同语义地
// 丢弃(TXBCF 置位, 槽位重用时计进 tx_cancelled)。刻意不执行 workaround 的"重发"半步:
// 取消一完成槽位就回到 TFQPI 分配, 同线程的下一次 handle_downlink 可能已把新帧写进
// 同一槽位, 此时对旧请求位再置 TXBAR 会把新帧发两遍。丢弃一个已停滞 20 ms 的帧,
// 好过让一个槽位永久蒸发。
constexpr uint32_t kStuckRequestThresholdMs = 20;

void Can::recover_stuck_transmits() {
    // 没启动的控制器没有发送请求可查, 也不去碰它的寄存器。
    if (!started_)
        return;
    const uint32_t pending = hal_can_handle_->Instance->TXBRP;
    if (pending == 0U || bus_off_) {
        stuck_request_since_ms_ = 0;
        return;
    }

    const uint32_t now_ms = HAL_GetTick();
    if (stuck_request_since_ms_ == 0U) {
        // 0 保留为"无计时"哨兵; HAL_GetTick 开机后从 0 走起。
        stuck_request_since_ms_ = now_ms != 0U ? now_ms : 1U;
        return;
    }
    if (now_ms - stuck_request_since_ms_ < kStuckRequestThresholdMs)
        return;

    hal_can_handle_->Instance->TXBCR = pending;
    stuck_request_since_ms_ = 0;
    led::led->downlink_buffer_full();
    diag::note_tx_fail(diag_index());
}

extern "C" libhcs_ITCM void
    HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its) {
    (void)rx_fifo0_its;

    Can* can;
    data::DataId field_id;
    std::size_t diag_index;

    if (hfdcan == &hfdcan1) {
        can = can1.get();
        field_id = spec::mc02::Spec::Cans::kCan1.data_id;
        diag_index = 0;
    } else if (hfdcan == &hfdcan2) {
        can = can2.get();
        field_id = spec::mc02::Spec::Cans::kCan2.data_id;
        diag_index = 1;
    } else if (hfdcan == &hfdcan3) {
        can = can3.get();
        field_id = spec::mc02::Spec::Cans::kCan3.data_id;
        diag_index = 2;
    } else {
        return;
    }

    // 按中断计数而非按帧计数: RX FIFO 非空而计数不再增长, 才能区分"中断不再送达"
    // 与"控制器收不到报文"。
    diag::note_isr_entry(diag_index);

    // 正在停口: stop() 已落标志、还没来得及撤通知的那一小段里到的中断, 直接放过。
    // 控制器随即进 INIT, FIFO 由硬件清空。
    if (!can->started()) [[unlikely]]
        return;

    can->handle_uplink(field_id, usb::get_serializer());
}

extern "C" void
    HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t error_status_its) {
    if (!(error_status_its & FDCAN_IT_BUS_OFF))
        return;

    Can* can;
    if (hfdcan == &hfdcan1)
        can = can1.get();
    else if (hfdcan == &hfdcan2)
        can = can2.get();
    else if (hfdcan == &hfdcan3)
        can = can3.get();
    else
        return;

    // 同上: 正在停口的控制器不做 bus-off 恢复, 否则这里清掉的 INIT 会把它重新拉上总线。
    if (!can->started())
        return;

    FDCAN_ProtocolStatusTypeDef status;
    HAL_FDCAN_GetProtocolStatus(hfdcan, &status);
    // IR.BO 在 Bus_Off 置位与清零两个沿都会触发: 置位沿启动恢复流程, 清零沿只解除
    // recover_stuck_transmits() 的屏蔽。
    can->note_bus_off(status.BusOff != 0U);
    if (status.BusOff == 0U)
        return;

    // 总线关闭时 M_CAN 内核会自行置位 CCCR.INIT 并停机。HAL 状态仍是 BUSY, 所以
    // HAL_FDCAN_Start() 会拒绝 -- 直接清 INIT 以启动标准恢复流程: 控制器等待
    // 129 * 11 个连续隐性位, 复位错误计数器后恢复运行。总线持续故障时它只会再次
    // 进入 bus-off 并重试, 端口不用重启也能保活。
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
}

namespace {

// FDCAN 第 0 中断线。本固件开了四种通知(RX FIFO0 新消息、bus-off、两段协议错误, 见
// Can::start), 中断线选择(ILS)从未配置, 全部落在第 0 线。接收是每帧一次的热路径: 读一次 IR 直接
// 分发, 不经 CubeMX 存根(stm32h7xx_it.c)与 HAL_FDCAN_IRQHandler -- 后者在 FLASH 里
// 300 余条指令, 每次进来都把十几类中断源逐一读一遍寄存器。协议错误也在这里直接锁存
// 错误码; bus-off 和任何意料之外的源照旧交给 HAL, 行为与原路径一致。
template <FDCAN_HandleTypeDef* kHandle>
Can& can_of() {
    if constexpr (kHandle == &hfdcan1)
        return *can1;
    else if constexpr (kHandle == &hfdcan2)
        return *can2;
    else
        return *can3;
}

template <FDCAN_HandleTypeDef* kHandle>
libhcs_ITCM void line0_isr() {
    constexpr std::uint32_t kProtocolErrors = FDCAN_IR_PEA | FDCAN_IR_PED;
    FDCAN_GlobalTypeDef* const instance = kHandle->Instance;
    const std::uint32_t pending = instance->IR & instance->IE;
    if ((pending & FDCAN_IR_RF0N) != 0U) {
        instance->IR = FDCAN_IR_RF0N;
        HAL_FDCAN_RxFifo0Callback(kHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    }
    // 协议错误也在这里直接处理, 不经 HAL(它只会置 ErrorCode 再回调, 读不到错误码)。
    if ((pending & kProtocolErrors) != 0U) [[unlikely]] {
        instance->IR = pending & kProtocolErrors;
        can_of<kHandle>().note_bus_errors();
    }
    if ((pending & ~(FDCAN_IR_RF0N | kProtocolErrors)) != 0U) [[unlikely]]
        HAL_FDCAN_IRQHandler(kHandle);
}

std::uint32_t isr_address(void (*isr)()) {
    return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(isr));
}

} // namespace

void install_interrupt_vectors(std::uint32_t* vectors) {
    // 外设中断 n 在表里的下标是 16 + n(前 16 项是系统异常)。
    vectors[16 + FDCAN1_IT0_IRQn] = isr_address(&line0_isr<&hfdcan1>);
    vectors[16 + FDCAN2_IT0_IRQn] = isr_address(&line0_isr<&hfdcan2>);
    vectors[16 + FDCAN3_IT0_IRQn] = isr_address(&line0_isr<&hfdcan3>);
}

} // namespace libhcs::firmware::can
