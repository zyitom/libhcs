#include "firmware/c_board/app/src/can/can.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <utility>

#include <can.h>
#include <stm32f407xx.h>
#include <stm32f4xx_hal_can.h>
#include <stm32f4xx_hal_rcc.h>

#include "core/include/libhcs/data/datas.hpp"
#include "firmware/c_board/app/src/led/led.hpp"
#include "firmware/c_board/app/src/usb/helper.hpp"

// 把 CAN 转发热路径放进零等待的 SRAM(.RamFunc 段, 在 .data 里随启动代码从 FLASH 拷入,
// 见 bsp/cubemx/STM32F407XX_APP.ld), 消除最坏转发延迟中的 I-cache miss 与 FLASH 取指
// 等待(168 MHz 下 5 WS)。F4 没有 ITCM, CCM 只挂 D-bus 不能取指, 从 SRAM 执行是它的
// 替代; 与 mc02 把同职责函数放进 .itcm 同一意图。函数体保持 out-of-line(不写成类内
// inline): 类内 inline 函数没法整体进自定义 section。
#define libhcs_RAMFUNC __attribute__((section(".RamFunc")))

namespace libhcs::firmware::can {

namespace vc = libhcs::core::protocol::vendor_control;
namespace link = libhcs::core::link;

// 本控制器(STM32F407 bxCAN)没有 CAN-FD: 帧型是编译期事实(见 can.hpp), 这里只核对,
// 不按帧携带的类型标志分流。
libhcs_RAMFUNC void Can::handle_downlink(const data::CanDataView& data) {
    // 没声明的总线不发: 控制器不在总线上, 主机往它写的帧直接丢弃(没声明的口的最后一道)。
    if (!started_) [[unlikely]]
        return;
    // 主机 SDK 在 can_transmit() 拒绝长 payload(本板从不宣告
    // kCapCanFdLongFrames); 但对端有 bug 时不能让 8 字节邮箱在无调试构建里
    // 溢出, 所以丢弃而不是断言。
    if (data.can_data.size() > 8)
        return;
    auto construct = [&data](std::byte* storage) noexcept {
        auto& mailbox = *new (storage) TransmitMailboxData{};

        mailbox.identifier =
            ((data.is_extended_can_id ? data.can_id << CAN_TI0R_EXID_Pos
                                      : data.can_id << CAN_TI0R_STID_Pos)
             | (data.is_extended_can_id ? CAN_ID_EXT : CAN_ID_STD)
             | (data.is_remote_transmission ? CAN_RTR_REMOTE : CAN_RTR_DATA) | CAN_TI0R_TXRQ);

        mailbox.data_length_and_timestamp = data.can_data.size();
        if (!data.can_data.empty())
            std::memcpy(mailbox.data, data.can_data.data(), data.can_data.size());
    };

    if (!transmit_buffer_.emplace_back_n(construct, 1)) {
        tx_dropped_.note();
        led::led->downlink_buffer_full();
    }
}

libhcs_RAMFUNC void
    Can::handle_uplink(data::DataId field_id, core::protocol::Serializer& serializer) {
    auto hal_can_state = hal_can_handle_->State;
    auto* hal_can_instance = hal_can_handle_->Instance;

    core::utility::assert_always(
        (hal_can_state == HAL_CAN_STATE_READY) || (hal_can_state == HAL_CAN_STATE_LISTENING));
    // 在这一次挂起的通知里把 RX FIFO0 整个排空, 而不是每条消息都进出一次
    // 中断。bxCAN 的 FIFO0 只有 3 深, 突发下这样省掉每帧一次的中断进出开销。
    for (uint32_t rf0r = hal_can_instance->RF0R; (rf0r & CAN_RF0R_FMP0) != 0U;
         rf0r = hal_can_instance->RF0R) {
        // FOVR0: FIFO 满时又来了一帧, 在控制器里就丢了。下面释放邮箱的 RF0R |= RFOM0
        // 是读改写, 会顺手清掉这个 rc_w1 位, 所以只能在这里数。
        if ((rf0r & CAN_RF0R_FOVR0) != 0U) [[unlikely]]
            rx_lost_.note();
        const auto rir = hal_can_instance->sFIFOMailBox[CAN_RX_FIFO0].RIR;
        const auto rdtr = hal_can_instance->sFIFOMailBox[CAN_RX_FIFO0].RDTR;

        data::CanDataView data{};
        data.is_extended_can_id = static_cast<bool>(CAN_RI0R_IDE & rir);
        data.is_remote_transmission = static_cast<bool>(CAN_RI0R_RTR & rir);

        if (data.is_extended_can_id) {
            data.can_id = ((CAN_RI0R_EXID | CAN_RI0R_STID) & rir) >> CAN_RI0R_EXID_Pos;
        } else {
            data.can_id = (CAN_RI0R_STID & rir) >> CAN_TI0R_STID_Pos;
        }

        size_t can_data_length = (CAN_RDT0R_DLC & rdtr) >> CAN_RDT0R_DLC_Pos;
        if (data.is_remote_transmission)
            can_data_length = 0;
        // DLC 是从线上原样来的, 其他节点发 DLC 9-15 的 classic 帧也合法, CAN
        // 规范说按 8 字节处理。钳制住, 免得外来帧把契约外的视图推进序列化器
        // -- 被拒会触发下面的断言, 让整块板停在外部可控的输入上。
        else if (can_data_length > 8)
            can_data_length = 8;

        alignas(uint32_t) std::array<std::byte, 8> can_data{};
        const uint32_t rdlr = hal_can_instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR;
        const uint32_t rdhr = hal_can_instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR;
        std::memcpy(can_data.data(), &rdlr, sizeof(uint32_t));
        std::memcpy(can_data.data() + 4, &rdhr, sizeof(uint32_t));
        data.can_data = {can_data.data(), can_data_length};

        const auto result = serializer.write_can(field_id, data);
        if (result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]]
            rx_dropped_.note();
        core::utility::assert_always(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);

        hal_can_instance->RF0R |= CAN_RF0R_RFOM0;
    }
}

libhcs_RAMFUNC bool Can::drain_pending_transmits() {
    auto* hcan = hal_can_handle_;

    auto state = hcan->State;
    core::utility::assert_always(
        (state == HAL_CAN_STATE_READY) || (state == HAL_CAN_STATE_LISTENING));

    const uint32_t tsr = hcan->Instance->TSR;
    const unsigned int free_mailbox_count =
        !!(tsr & CAN_TSR_TME0) + !!(tsr & CAN_TSR_TME1) + !!(tsr & CAN_TSR_TME2);

    return transmit_buffer_.pop_front_n(
        [this, hcan](const TransmitMailboxData& mailbox_data) noexcept {
            const uint32_t status = hcan->Instance->TSR;
            auto target_mailbox_index = (status & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
            core::utility::assert_always(target_mailbox_index <= 2);
            note_previous_outcome(status, target_mailbox_index);

            auto& target_mailbox = hal_can_handle_->Instance->sTxMailBox[target_mailbox_index];
            target_mailbox.TDTR = mailbox_data.data_length_and_timestamp;
            target_mailbox.TDLR = mailbox_data.data[0];
            target_mailbox.TDHR = mailbox_data.data[1];
            target_mailbox.TIR = mailbox_data.identifier;
        },
        free_mailbox_count);
}

libhcs_RAMFUNC void Can::note_previous_outcome(uint32_t status, uint32_t mailbox) {
    const uint32_t completed = CAN_TSR_RQCP0 << (8U * mailbox);
    const uint32_t succeeded = CAN_TSR_TXOK0 << (8U * mailbox);
    if ((status & completed) == 0U)
        return; // 上电以来没用过, 或已清点过
    if ((status & succeeded) == 0U) [[unlikely]]
        cancelled_frames_.note();
    hal_can_handle_->Instance->TSR = completed;
}

link::CanTimingValue Can::timing() const {
    return {arbitration_baudrate(), 0U, static_cast<uint16_t>(arbitration_sample_point()), 0U};
}

uint32_t Can::arbitration_baudrate() const {
    const BitTiming timing = bit_timing();
    return HAL_RCC_GetPCLK1Freq() / (timing.prescaler * timing.total());
}

uint32_t Can::arbitration_sample_point() const {
    const BitTiming timing = bit_timing();
    return (1U + timing.seg1) * 1000U / timing.total();
}

Can::BitTiming Can::bit_timing() const {
    const uint32_t btr = hal_can_handle_->Instance->BTR;
    return {
        .prescaler = ((btr & CAN_BTR_BRP) >> CAN_BTR_BRP_Pos) + 1U,
        .seg1 = ((btr & CAN_BTR_TS1) >> CAN_BTR_TS1_Pos) + 1U,
        .seg2 = ((btr & CAN_BTR_TS2) >> CAN_BTR_TS2_Pos) + 1U,
    };
}

void Can::read_config(vc::CanConfigPayload& out) const {
    out = {
        .mode = std::to_underlying(vc::CanMode::kClassic),
        .control = 0,
        .reserved0 = 0,
        .arbitration_baudrate = arbitration_baudrate(),
        .data_baudrate = 0,
        .nominal_sample_point = static_cast<uint16_t>(arbitration_sample_point()),
        .data_sample_point = 0,
        .reserved1 = 0,
    };
}

void Can::note_bus_errors() {
    const uint32_t code = hal_can_handle_->ErrorCode;
    constexpr std::pair<uint32_t, data::CanLastError> kCodes[] = {
        {HAL_CAN_ERROR_STF, data::CanLastError::kStuff},
        {HAL_CAN_ERROR_FOR,  data::CanLastError::kForm},
        {HAL_CAN_ERROR_ACK,   data::CanLastError::kAck},
        { HAL_CAN_ERROR_BR,  data::CanLastError::kBit1}, // 发隐性读回显性
        { HAL_CAN_ERROR_BD,  data::CanLastError::kBit0}, // 发显性读回隐性
        {HAL_CAN_ERROR_CRC,   data::CanLastError::kCrc},
    };
    for (const auto& [bit, last_error] : kCodes) {
        if ((code & bit) != 0U)
            last_bus_error_.note(last_error);
    }
    (void)HAL_CAN_ResetError(hal_can_handle_);
}

data::CanStatusView Can::read_status() {
    // ESR 位域(RM0090): REC[31:24]、TEC[23:16]、LEC[6:4]、BOFF[2]、EPVF[1]、EWGF[0]。
    // [实测 2026-10-07: 原先按 TEC[7:0] / LEC[18:16] 读, TEC 恒报 0]
    const uint32_t esr = hal_can_handle_->Instance->ESR;
    const auto last_error = static_cast<data::CanLastError>((esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
    uint8_t flags = 0;
    if ((esr & CAN_ESR_EPVF) != 0U)
        flags |= data::kCanErrorPassive;
    if ((esr & CAN_ESR_EWGF) != 0U)
        flags |= data::kCanWarning;
    if ((esr & CAN_ESR_BOFF) != 0U)
        flags |= data::kCanBusOff;
    return {
        .tec = static_cast<uint8_t>((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos),
        .rec = static_cast<uint8_t>((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos),
        .last_error = last_bus_error_.latest(last_error),
        .data_last_error = data::CanLastError::kNone,
        .flags = flags,
        .tx_cancelled = cancelled_frames_.count(),
        .tx_dropped = tx_dropped_.count(),
        .rx_dropped = rx_dropped_.count(),
        .rx_lost = rx_lost_.count(),
    };
}

void Can::stop() {
    if (!started_)
        return;
    started_ = false;
    loop::clear(loop::bit(data_id()));
    // 邮箱里没发出去的、软件队列里排着的, 都属于刚结束的会话, 不留给下一个。中止的
    // 请求也会置 RQCPx(TXOKx = 0): 连同已完成的一并清掉, 免得下个会话的作废清点
    // (note_previous_outcome)把它们算成作废。
    (void)HAL_CAN_AbortTxRequest(
        hal_can_handle_, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    core::utility::assert_always(HAL_CAN_Stop(hal_can_handle_) == HAL_OK);
    hal_can_handle_->Instance->TSR = CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2;
    (void)transmit_buffer_.clear();
}

// 属性写在 extern "C" 之后: 放在它前面会被 GCC 静默忽略, 函数留在 FLASH。
extern "C" libhcs_RAMFUNC void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (auto* can = can_by_index(Can::index_of(hcan)))
        can->handle_uplink(can->data_id(), usb::get_serializer());
}

// LEC / ERR 中断(CAN_SCE 向量, 经 HAL_CAN_IRQHandler)。HAL 只在有错误码时回调。
extern "C" libhcs_RAMFUNC void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan) {
    if (auto* can = can_by_index(Can::index_of(hcan)))
        can->note_bus_errors();
}

} // namespace libhcs::firmware::can
