#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>

extern "C" {
#include "ch32h417.h"
}

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/ch32_board/app/src/board_app.hpp"
#include "firmware/ch32_board/app/src/led/led.hpp"
#include "firmware/ch32_board/app/src/link/uplink.hpp"
#include "firmware/ch32_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::can {

// Classic bxCAN forwarding driver, structured like upstream hpm_board's Can:
// board-agnostic (takes a board::CanPort, defers pins/AF/clock to
// board::init_can), downlink transmits DIRECTLY to the controller's hardware TX
// mailboxes (no software TX ring, no try_transmit -- bxCAN's three TX mailboxes
// serve that role), and a single irq_handler() drains RX FIFO0.
//
// CH32H417 CAN is CAN 2.0B only (no CAN-FD), so is_fdcan is always false and
// there is no data-phase timing to configure.
class Can : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Can, board::CanPort>;

    explicit Can(board::CanPort port)
        : data_id_(port.data_id)
        , can_base_(reinterpret_cast<CAN_TypeDef*>(port.base)) {
        const uint32_t can_clock = board::init_can(port);

        CAN_InitTypeDef config = {};
        config.CAN_TTCM = DISABLE;
        config.CAN_ABOM = ENABLE;  // auto bus-off recovery
        config.CAN_AWUM = DISABLE;
        config.CAN_NART = DISABLE; // retransmit on arbitration loss / error
        config.CAN_RFLM = DISABLE; // FIFO overwrites: newest frame wins
        config.CAN_TXFP = DISABLE; // priority by identifier, not by request order
        config.CAN_Mode = CAN_Mode_Normal;

        // Bit timing: SYNC_SEG(1) + BS1 + BS2 quanta per bit, sample point after
        // BS1. The EVT reference uses 12 tq (BS1=6/BS2=5) sampling at 58%, which
        // is both unusually early for a motor bus and not divisible into this
        // part's 100 MHz peripheral clock (100e6 / 1e6 / 12 = 8.33). 20 tq
        // divides exactly (prescaler 5) and puts the sample point at 16/20 = 80%,
        // the CiA-recommended value for 1 Mbit/s.
        config.CAN_SJW = CAN_SJW_1tq;
        config.CAN_BS1 = CAN_BS1_15tq;
        config.CAN_BS2 = CAN_BS2_4tq;
        const uint32_t prescaler = can_clock / (port.bitrate * kQuantaPerBit);
        // An exact division is required: a truncated prescaler silently shifts
        // the bit rate and the bus never acks. If this fires after a clock-tree
        // change, re-pick BS1/BS2 so that (1 + BS1 + BS2) divides
        // peripheral_clock() / bitrate -- keep the sample point near 80%.
        core::utility::assert_always(prescaler >= 1 && prescaler <= 1024);
        core::utility::assert_always(can_clock == prescaler * port.bitrate * kQuantaPerBit);
        config.CAN_Prescaler = static_cast<uint16_t>(prescaler);
        core::utility::assert_always(CAN_Init(can_base_, &config) == CAN_InitStatus_Success);

        init_accept_all_filter();

        CAN_ITConfig(can_base_, CAN_IT_FMP0, ENABLE);
        NVIC_EnableIRQ(port.irq_num);
    }

    [[nodiscard]] data::DataId data_id() const { return data_id_; }

    void handle_downlink(const data::CanDataView& data) {
        CanTxMsg msg = {};
        if (data.is_extended_can_id) {
            msg.IDE = CAN_Id_Extended;
            msg.ExtId = data.can_id & 0x1FFFFFFFu;
        } else {
            msg.IDE = CAN_Id_Standard;
            msg.StdId = data.can_id & 0x7FFu;
        }
        msg.RTR = data.is_remote_transmission ? CAN_RTR_Remote : CAN_RTR_Data;

        core::utility::assert_debug(data.can_data.size() <= 8);
        msg.DLC = static_cast<uint8_t>(data.can_data.size());
        if (!data.can_data.empty())
            std::memcpy(msg.Data, data.can_data.data(), data.can_data.size());

        // Direct to a hardware TX mailbox; NO_MB means all three are busy -> drop.
        if (CAN_Transmit(can_base_, &msg) == CAN_TxStatus_NoMailBox)
            led::led->downlink_buffer_full();
    }

    void handle_uplink(core::protocol::Serializer& serializer) {
        while (CAN_MessagePending(can_base_, CAN_FIFO0) > 0) {
            CanRxMsg rx = {};
            CAN_Receive(can_base_, CAN_FIFO0, &rx);

            data::CanDataView data;
            data.is_fdcan = false;
            data.is_extended_can_id = (rx.IDE == CAN_Id_Extended);
            data.is_remote_transmission = (rx.RTR == CAN_RTR_Remote);
            data.can_id = data.is_extended_can_id ? rx.ExtId : rx.StdId;

            // A remote frame carries a DLC but no payload; classic CAN caps the
            // payload at 8 regardless of what the DLC field claims.
            size_t length = data.is_remote_transmission ? 0 : rx.DLC;
            if (length > 8)
                length = 8;
            data.can_data = {reinterpret_cast<const std::byte*>(rx.Data), length};

            core::utility::assert_always(
                serializer.write_can(data_id_, data)
                != core::protocol::Serializer::SerializeResult::kInvalidArgument);
        }
    }

    void irq_handler() {
        // Drop received frames until the host has acked kStart: serializing
        // before the session is up would only fill the batch ring with frames
        // nobody drains.
        if (!link::uplink_enabled()) {
            drain_fifo();
            return;
        }
        handle_uplink(link::uplink_serializer());
    }

private:
    // SYNC_SEG + BS1 + BS2, matching the CAN_BS1_15tq / CAN_BS2_4tq above.
    static constexpr uint32_t kQuantaPerBit = 1 + 15 + 4;

    void init_accept_all_filter() {
        // The filter block is a single shared resource owned by CAN1: banks
        // [0, CAN2 start) belong to CAN1 and [CAN2 start, ...) to CAN2. The EVT
        // reference splits at bank 14 (and 28 for CAN3), which is also the
        // reset default of CAN_FilterInit's slave start bank.
        uint8_t filter_number = 0;
        if (can_base_ == CAN2)
            filter_number = 14;
        else if (can_base_ == CAN3)
            filter_number = 28;

        CAN_FilterInitTypeDef filter = {};
        filter.CAN_FilterNumber = filter_number;
        filter.CAN_FilterMode = CAN_FilterMode_IdMask;
        filter.CAN_FilterScale = CAN_FilterScale_32bit;
        // Zero id + zero mask = accept everything; the host does the filtering.
        filter.CAN_FilterIdHigh = 0x0000;
        filter.CAN_FilterIdLow = 0x0000;
        filter.CAN_FilterMaskIdHigh = 0x0000;
        filter.CAN_FilterMaskIdLow = 0x0000;
        filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
        filter.CAN_FilterActivation = ENABLE;
        CAN_FilterInit(&filter);
    }

    // Sessionless path: the FIFO still has to be emptied or FMP0 stays asserted
    // and the ISR re-enters forever.
    void drain_fifo() {
        while (CAN_MessagePending(can_base_, CAN_FIFO0) > 0) {
            CanRxMsg rx = {};
            CAN_Receive(can_base_, CAN_FIFO0, &rx);
        }
    }

    const data::DataId data_id_;
    CAN_TypeDef* can_base_;
};

namespace internal {

template <size_t... I>
constexpr auto make_can_array(std::index_sequence<I...> /*unused*/) {
    return std::array<Can::Lazy, sizeof...(I)>{Can::Lazy{board::kCanPorts[I]}...};
}

} // namespace internal

// Built straight from the board port table, as on hpm_board: adding a bus is a
// board_app.hpp table entry, nothing here changes.
inline constinit auto can_array =
    internal::make_can_array(std::make_index_sequence<board::kCanPortCount>{});
constexpr size_t kCanCount = board::kCanPortCount;

} // namespace libhcs::firmware::can
