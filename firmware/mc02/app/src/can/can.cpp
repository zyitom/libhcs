#include "firmware/mc02/app/src/can/can.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include <fdcan.h>

#include "core/include/libhcs/data/datas.hpp"
#include "firmware/mc02/app/src/diag/can_diag.hpp"
#include "firmware/mc02/app/src/led/led.hpp"
#include "firmware/mc02/app/src/usb/helper.hpp"

// 把 CAN 转发热路径放进零等待 ITCM(启动时由 app.cpp 从 FLASH 拷入, 见 .itcm 链接段),
// 以消除最坏转发延迟中的 I-cache miss 和 FLASH-XIP 取指抖动。被调用的 HAL 与共享
// serializer 仍在 FLASH -- 只有这层胶水代码跑在 ITCM。函数体保持 out-of-line
// (不写成类内 inline), 因为 inline/COMDAT 函数放自定义 section 会触发 GCC section 类型冲突。
#define libhcs_ITCM __attribute__((section(".itcm")))

namespace libhcs::firmware::can {

libhcs_ITCM
uint32_t Can::hardware_free_slots() const noexcept {
    return hal_can_handle_->Instance->TXFQS & FDCAN_TXFQS_TFFL;
}

// 在控制器当前的 put index 处写入一个 Tx 元素并请求发送。调用方必须先查
// hardware_free_slots(): FIFO 满时 TFQPI 仍会读出一个待发送的槽位, 直接写会覆盖未发出的帧。
libhcs_ITCM
void Can::push_to_hardware(const TransmitMailboxData& mailbox_data) noexcept {
    auto* hcan = hal_can_handle_;
    const auto put_index =
        (hcan->Instance->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;

    struct TxMailbox {
        uint32_t TIR;
        uint32_t TDTR;
        uint32_t TDLR;
        uint32_t TDHR;
    };
    auto* target_mailbox = reinterpret_cast<TxMailbox*>(
        hcan->msgRam.TxBufferSA + (put_index * hcan->Init.TxElmtSize * 4U));

    target_mailbox->TIR = mailbox_data.identifier;
    target_mailbox->TDTR = mailbox_data.control;
    target_mailbox->TDLR = mailbox_data.data[0];
    target_mailbox->TDHR = mailbox_data.data[1];

    hcan->Instance->TXBAR = (1UL << put_index);
    hcan->LatestTxFifoQRequest = (1UL << put_index);
}

libhcs_ITCM
void Can::handle_downlink(const data::CanDataView& data) {
    TransmitMailboxData mailbox{};

    if (data.is_extended_can_id) {
        mailbox.identifier = (data.can_id << 0) | FDCAN_EXTENDED_ID;
    } else {
        mailbox.identifier = (data.can_id << 18) | FDCAN_STANDARD_ID;
    }
    mailbox.identifier |= data.is_remote_transmission ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;

    core::utility::assert_debug(data.can_data.size() <= 8);
    const auto dlc = static_cast<uint32_t>(data.can_data.size());

    // 控制器常驻 FD+BRS 模式, 这是经典 CAN 的严格超集: 帧格式由 Tx 元素(T1)里的
    // FDF/BRS 位逐帧决定 -- 经典帧(is_fdcan == false)两位清零发出, FD 帧切到
    // 5 Mbit/s 数据段; 不需要进 INIT 模式重配, 热路径上只多两个位。
    mailbox.control = dlc << 16;
    if (data.is_fdcan)
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
        *new (storage) TransmitMailboxData{mailbox};
    };
    if (!transmit_buffer_.emplace_back_n(copy, 1)) {
        led::led->downlink_buffer_full();
        diag::note_tx_fail(diag_index());
    }
    // Whether or not the push succeeded: a refused push means the queue is full,
    // so it is non-empty either way. See Can::drain_pending_transmits() in can.hpp.
    transmit_pending_mask_ |= 1U << diag_index();
}

libhcs_ITCM
void Can::handle_uplink(data::DataId field_id, core::protocol::Serializer& serializer) {
    core::utility::assert_always(hal_can_handle_->State == HAL_FDCAN_STATE_BUSY);
    auto* hal_can_instance = hal_can_handle_->Instance;

    struct RxMailbox {
        uint32_t RIR;
        uint32_t RDTR;
        uint32_t RDLR;
        uint32_t RDHR;
    };

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
        // Rx 元素 R1 的 bit 21 (FDF) 标记 FD 帧; 转发时原样带上,
        // 让上位机知道每帧的真实格式。
        can_data.is_fdcan = static_cast<bool>(rdtr & FDCAN_FD_CAN);
        can_data.is_extended_can_id = static_cast<bool>(rx_mailbox->RIR & 0x40000000U);
        can_data.is_remote_transmission = static_cast<bool>(rx_mailbox->RIR & 0x20000000U);

        if (can_data.is_extended_can_id) {
            can_data.can_id = rx_mailbox->RIR & 0x1FFFFFFFU;
        } else {
            can_data.can_id = (rx_mailbox->RIR & 0x1FFC0000U) >> 18;
        }

        // 不用硬件 RX 时间戳: FDCAN 内部计数器只有 16 位, 上报值约 65.5 ms 回绕,
        // 不满足 CanDataView::timestamp_us 的 32 位微秒约定
        // (见 core/include/libhcs/data/datas.hpp)。留空则 serializer 直接省掉该字段,
        // 每个上行帧省 4 字节。若要恢复, 除取消下面这行注释和 config_can() 里的
        // FDCAN_TIMESTAMP_* 配置外, 还要先把值加宽(例如叠加自由运行的 TIM5 微秒计数器),
        // 上位机才能继续用普通的 32 位回绕差值。
        
        // 帧起始时刻捕获的 16 位硬件时间戳(R1 bits[15:0])。内部计数器每个标称位时间
        // 走一格, 在 1 Mbit/s 仲裁速率(预分频 1)下即 1 us, 所以数值本身就是微秒。
        // can_data.timestamp_us = static_cast<uint32_t>(rdtr & 0x0000FFFFU);

        size_t can_data_length = (rdtr & 0x000F0000U) >> 16;
        if (can_data.is_remote_transmission)
            can_data_length = 0;
        // DLC 是从线上原样读来的。FD 帧 DLC 9..15 表示 12..64 字节数据, 但 RX 元素只存
        // 8 字节(FDCAN_DATA_BYTES_8), 且线协议上限也是 8, 转发出去就是静默截断 --
        // 直接丢弃该帧, 但继续排空 FIFO。经典帧 DLC 9..15 是合法的,
        // 按 CAN 规范表示 8 字节数据。
        if (can_data.is_fdcan && can_data_length > 8) [[unlikely]] {
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
        // 仍然丢帧 -- 这里的意义只是让它不再静默。
        const auto uplink_result = serializer.write_can(field_id, can_data);
        if (uplink_result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]] {
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

libhcs_ITCM
bool Can::drain_transmit_queue() {
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

    // Clear this controller's pending bit only once its queue is really empty. A
    // FIFO that filled first leaves both the frames and the bit for the next
    // pass. See Can::drain_pending_transmits() in can.hpp.
    if (transmit_buffer_.readable() == 0)
        transmit_pending_mask_ &= ~(1U << diag_index());

    return sent != 0;
}

// Out-of-line half of Can::drain_pending_transmits(): only reached when some
// controller's hardware FIFO overflowed into its software queue.
libhcs_ITCM
void Can::drain_pending_transmits_slow() {
    const uint32_t pending = transmit_pending_mask_;
    if ((pending & (1U << 0U)) != 0U)
        can1->drain_transmit_queue();
    if ((pending & (1U << 1U)) != 0U)
        can2->drain_transmit_queue();
    if ((pending & (1U << 2U)) != 0U)
        can3->drain_transmit_queue();
}

// Debug-build invariant check behind Can::drain_pending_transmits().
bool Can::transmit_queues_empty() {
    return can1->transmit_buffer_.readable() == 0 && can2->transmit_buffer_.readable() == 0
        && can3->transmit_buffer_.readable() == 0;
}

extern "C" libhcs_ITCM void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its) {
    (void)rx_fifo0_its;

    Can* can;
    data::DataId field_id;
    std::size_t diag_index;

    if (hfdcan == &hfdcan1) {
        can = can1.get();
        field_id = data::DataId::kCan1;
        diag_index = 0;
    } else if (hfdcan == &hfdcan2) {
        can = can2.get();
        field_id = data::DataId::kCan2;
        diag_index = 1;
    } else if (hfdcan == &hfdcan3) {
        can = can3.get();
        field_id = data::DataId::kCan3;
        diag_index = 2;
    } else {
        return;
    }

    // 按中断计数而非按帧计数: RX FIFO 非空而计数不再增长, 才能区分"中断不再送达"
    // 与"控制器收不到报文"。
    diag::note_isr_entry(diag_index);

    can->handle_uplink(field_id, usb::get_serializer());
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(
    FDCAN_HandleTypeDef* hfdcan, uint32_t error_status_its) {
    if (!(error_status_its & FDCAN_IT_BUS_OFF))
        return;

    FDCAN_ProtocolStatusTypeDef status;
    HAL_FDCAN_GetProtocolStatus(hfdcan, &status);
    if (status.BusOff == 0U)
        return;

    // 总线关闭时 M_CAN 内核会自行置位 CCCR.INIT 并停机。HAL 状态仍是 BUSY, 所以
    // HAL_FDCAN_Start() 会拒绝 -- 直接清 INIT 以启动标准恢复流程: 控制器等待
    // 129 * 11 个连续隐性位, 复位错误计数器后恢复运行。总线持续故障时它只会再次
    // 进入 bus-off 并重试, 端口不用重启也能保活。
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
}

} // namespace libhcs::firmware::can
