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
    // Constructed from the logical CAN index alone, with the CanPort derived from
    // it via board::can_port(). The index, not the port, is the Lazy argument: it
    // keeps every construction site independent of the port table's contents.
    using Lazy = utility::Lazy<Can, data::DataId, size_t>;

    // Baudrates are compile-time constants; the mode is fixed per board via
    // the CanPort table in board_app.hpp.
    static constexpr uint32_t kArbitrationBaudrate = 1'000'000;
    static constexpr uint32_t kCanFdDataBaudrate = 5'000'000;

    // The IE mask this driver arms, and the same mask the ISR re-tests against
    // IR before returning. One constant for both: the ISR loop is only correct
    // while it covers exactly the sources that can hold the interrupt line
    // high, so these two must not be able to drift apart (see irq_handler).
    static constexpr uint32_t kEnabledInterrupts =
        MCAN_INT_RXFIFO0_NEW_MSG | MCAN_INT_BUS_OFF_STATUS | MCAN_INT_WARNING_STATUS | MCAN_INT_ERROR_PASSIVE
        | MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE | MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE;

    explicit Can(data::DataId data_id, size_t board_can_index)
        : Can(data_id, board::can_port(board_can_index), board_can_index) {}

    explicit Can(data::DataId data_id, CanPort port, size_t board_can_index)
        : data_id_(data_id)
        , can_base_(reinterpret_cast<MCAN_Type*>(port.base))
        , irq_num_(port.irq_num)
        , canfd_(port.mode == CanMode::kCanFd)
        , can_index_(board_can_index) {

        // Only ports the PCB actually has may be brought up. On the single-CAN
        // hpm5321 the second slot's pads are LED cathodes, so constructing it
        // would hand PA30/PA31 to a transceiver that is not there.
        core::utility::assert_always(board_can_index < board::can_port_count());

        // Message RAM placement is board business: SoCs differ in where MCAN
        // buffers may live (dedicated AHB RAM vs a section-placed array), so
        // the board hands out the region (board_app.cpp).
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

            // Transmitter Delay Compensation, mandatory at this data-phase rate
            // and previously never enabled here -- mcan_get_default_config()
            // memsets the struct, so the field stayed false and mcan_init()
            // cleared DBTP.TDC outright.
            //
            // A 5 Mbit data bit is 200 ns and the primary sample point sits at
            // 87.5% = 175 ns. A high-speed CAN transceiver's TXD->RXD loop delay
            // is typically 120-255 ns, so without compensation a transmitting
            // node can read back the PREVIOUS bit while checking its own output
            // and raise a bit error. TDC moves that read-back to a secondary
            // sample point placed at (measured loop delay + TDCO), which follows
            // the transceiver instead of assuming it is fast enough.
            //
            // This is the other half of the sample-point fix below. That one
            // made both ends of the bus agree on WHERE to sample; this one makes
            // the transmitter's self-check survive its own transceiver. The
            // symptom family is the same (PSR.DLEC = bit error, TEC climbing
            // into error-passive, one direction failing while classic CAN
            // works), which is why the surviving half was easy to miss.
            //
            // mc02 has done this since its FDCAN bring-up (mc02/app/src/can/
            // can.hpp: HAL_FDCAN_EnableTxDelayCompensation with offset =
            // DataPrescaler * DataTimeSeg1). Both boards run the same 5 Mbit
            // data phase off the same 80 MHz kernel clock, so leaving this board
            // uncompensated made the pair asymmetric.
            //
            // ssp_offset stays 0, which asks the SDK to derive TDCO from the
            // bit timing it just solved: DBTP.DTSEG1 + 2 in mtq. That equals the
            // primary sample point only while the data-phase prescaler is 1 --
            // TDCO counts CAN clock periods, not time quanta -- which the assert
            // after mcan_init() pins down.
            config.enable_tdc = true;
        }

        // Sample point pinned to 87.5% in BOTH phases, because that is what the
        // other boards on this bus actually run.
        //
        // The overriding rule is that every node on a segment must sample at the
        // same point (the two phases need not agree with each other -- but here
        // they happen to). The CubeMX boards (mc02, c_board) are the reference:
        // both phases use tseg1/tseg2 = 13/2 = 87.5%, the nominal phase at
        // prescaler 5 (16 TQ) and the data phase at prescaler 1. Note this is
        // NOT the 75%-above-800-kbit/s guidance a vendor table would give for a
        // 1 Mbit arbitration phase -- and matching the guidance instead of the
        // bus was measured to fail: with the nominal phase left at the SDK's
        // 75% (59/20, 80 TQ) and only the data phase pinned, FD delivery from
        // this board dropped to 0/40000 with PSR.DLEC = bit1 error.
        //
        // The SDK will not arrive here on its own: its window is [750, 875] and
        // the solver stops as soon as it climbs past the MINIMUM, so it always
        // lands on 75.0% and the 875 is never reached. A 5 Mbit data bit is
        // 200 ns, so a 12.5-point disagreement puts the ends 25 ns apart and the
        // receiver clocks the wrong bit -- originally seen as mc02 never ACKing
        // an FD frame from this board (PSR.DLEC = ACK error, TEC climbing into
        // error-passive) while classic CAN and the reverse direction worked.
        config.can20_samplepoint_min = 875U;
        config.can20_samplepoint_max = 875U;
        config.canfd_samplepoint_min = 875U;
        config.canfd_samplepoint_max = 875U;
        // Keep the default 8-byte element size even for CAN-FD: the frames on this
        // bus never exceed 8 data bytes, so RAM usage stays identical to classic CAN.
        config.ram_config.txbuf_dedicated_txbuf_elem_count = 0;
        config.ram_config.txbuf_fifo_or_queue_elem_count = MCAN_TXBUF_SIZE_CAN_DEFAULT;
        config.ram_config.txfifo_or_txqueue_mode = MCAN_TXBUF_OPERATION_MODE_FIFO;
        config.disable_auto_retransmission = true;

        // 64-bit hardware timestamp via the Timestamp Unit (TSU). This SoC's
        // MCAN has no usable internal TSU timebase (TBCS is fixed to "external"
        // by synthesis), so every controller's TSU is fed from a single shared
        // PTPC0 timebase routed through TBSEL slot 0. PTPC keeps an IEEE-1588
        // {seconds:nanoseconds} counter; handle_uplink() folds it to
        // microseconds. Because all controllers share PTPC0, their timestamps
        // sit on one common clock and are directly comparable across buses.
        config.use_timestamping_unit = true;
        config.tsu_config.enable_tsu = true;
        config.tsu_config.enable_64bit_timestamp = true;
        config.tsu_config.use_ext_timebase = true;
        config.tsu_config.ext_timebase_src = MCAN_TSU_EXT_TIMEBASE_SRC_TBSEL_0;
        config.tsu_config.tbsel_option = MCAN_TSU_TBSEL_PTPC0;
        config.tsu_config.capture_on_sof = true;
        config.tsu_config.prescaler = 1; // unused for an external timebase
        config.timestamp_cfg.counter_prescaler = 1;
        config.timestamp_cfg.timestamp_selection = MCAN_TIMESTAMP_SEL_EXT_TS_VAL_USED;

        // The external TSU only timestamps a received frame when the filter that
        // accepts it is marked as a sync message (evaluated only while CCCR.UTSU
        // is set). The default accept-all filters have sync_message = 0, so swap
        // in accept-all (mask 0) sync filters for both standard and extended IDs
        // -- otherwise no frame is ever timestamped.
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

        // Start the shared PTPC0 timebase once, then point this controller's TSU
        // input at it. PTPC is clocked from the AHB clock group (clock_ptpc).
        const uint32_t ptpc_freq = clock_get_frequency(clock_ptpc);
        // PTPC digital mode advances its nanosecond counter by an integer step
        // of floor(1e9 / ptpc_freq) ns (6 ns at 160 MHz) rather than the exact
        // 1e9 / ptpc_freq (6.25 ns), so reported time runs slow. Converting
        // reported nanoseconds to microseconds by dividing by
        // (ptpc_freq_MHz * step) instead of 1000 cancels the error exactly
        // (160 * 6 = 960 at 160 MHz). handle_uplink() bakes this divisor in as
        // the compile-time kTsNsPerUs so the ISR needs no runtime division --
        // verify the clock tree still matches it.
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

        // The auto TDCO above is expressed in DTSEG1 units but programmed in
        // mtq, so it only lands on the sample point while the data-phase
        // prescaler is 1. That is what the SDK solver picks at 80 MHz / 5 Mbit
        // (brp=1 tseg1=13 tseg2=2, which -Dlibhcs_CAN_DIAG=ON prints), but the
        // solver is free to choose otherwise if either number ever moves --
        // and a silently mis-placed secondary sample point looks exactly like
        // no compensation at all. Trap it here instead.
        if (canfd_)
            core::utility::assert_always(MCAN_DBTP_DBRP_GET(can_base_->DBTP) == 0U);

        mcan_enable_interrupts(can_base_, kEnabledInterrupts);
        // CAN RX is the forwarding-critical path (motor feedback -> host).
        // Priority 3: above USB (2) and UART (1) -- ensures CAN frames are
        // never delayed by bulk USB transfers or DMA callbacks.
        intc_m_enable_irq_with_priority(port.irq_num, 3);
    }

    [[nodiscard]] data::DataId data_id() const { return data_id_; }

    // Controller error state, for the EP0 status query (usb/vendor_control.cpp).
    // Read straight from the registers on each call rather than cached: PSR.LEC
    // self-clears to "no change" when read, so a cached copy would report a
    // stale error forever and a fresh one would hide errors from any other
    // reader. There is only one reader.
    struct Status {
        uint8_t tec, rec, last_error, data_last_error, flags;
        uint32_t tx_occurred, tx_cancelled, rx_frames, rx_fifo_level;
    };

    [[nodiscard]] Status status() const {
        const uint32_t psr = can_base_->PSR;
        const uint32_t ecr = can_base_->ECR;
        uint8_t flags = 0;
        if ((psr >> 5) & 1U)
            flags |= 1U << 0; // error passive
        if ((psr >> 6) & 1U)
            flags |= 1U << 1; // warning
        if ((psr >> 7) & 1U)
            flags |= 1U << 2; // bus off
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

    // Forwarding hot path -- defined out-of-line in can.cpp, in the ILM (.fast)
    // section, to remove FLASH-XIP fetch jitter from the worst case. See can.cpp
    // for the rationale and why they are not inline-in-class.
    // handle_uplink reads (at most) one frame from RX FIFO0 and returns whether
    // a frame was consumed, so the ISR can drain the FIFO in a loop.
    void handle_downlink(const data::CanDataView& data);
    bool handle_uplink(core::protocol::FieldId field_id, core::protocol::Serializer& serializer);
    void irq_handler();

    // Main-loop watchdog for an interrupt request that was accepted by the PLIC
    // but never delivered. Two register reads on the healthy path; see can.cpp
    // for the failure it repairs and the evidence behind it.
    void poll();

    // Drain the software transmit queue into the MCAN TX FIFO. Called every
    // main-loop pass; two loads and a return when the queue is empty. See
    // handle_downlink in can.cpp for why the queue exists.
    void try_transmit();

    // How many frames are waiting in the software transmit queue. The USB
    // downlink arming policy reads this to decide whether accepting another OUT
    // packet would overrun the queue; see usb/vendor.hpp.
    [[nodiscard]] size_t transmit_queue_depth() const { return transmit_buffer_.readable(); }

    static constexpr size_t kTransmitQueueSize = 64;

    // One pass of the interrupt handler: acknowledge `flags` and act on them.
    // Split out of irq_handler so the latter can re-test IR and repeat; see the
    // comment there for why returning after a single pass loses the interrupt.
    void handle_interrupt_flags(uint32_t flags);

private:
    // Position of this controller in board::kCanPorts. Stored rather than
    // derived from data_id: 6E8Y uses kCan0..kCan3 while 5321 uses kCan1..
    // kCan2, so subtracting kCan1 is not a board index.
    std::size_t can_index() const { return can_index_; }

    // Read and normalize one RX FIFO frame. `true` means an element was
    // consumed; `valid` distinguishes a representable frame from an FD payload
    // that the configured 8-byte RX element had to truncate.
    bool read_uplink(data::CanDataView& out, uint8_t storage[8], bool& valid);
    static void serialize_uplink(
        core::protocol::FieldId field_id, const data::CanDataView& data,
        core::protocol::Serializer& serializer);

    // Classifies an MCAN Last Error Code (arbitration or data phase) into an
    // indicator-LED state -- the granularity a CAN controller can actually back
    // up electrically:
    //   ack_error  -> kNoAck       : frame sent fine, nobody acknowledged.
    //   bit0_error -> kWiringFault : sent dominant, read back recessive -- the bus
    //                                cannot be driven dominant (CAN_H/L shorted,
    //                                reversed, or open).  Stable on a hard fault.
    //   stuff/form/crc/bit1 -> kSignalError : corrupted bits; the exact code
    //                                fluctuates and its causes (no termination,
    //                                baudrate mismatch, noise) are indistinguishable.
    //   no_error/no_change -> kNone.
    static led::CanFault classify_can_fault(uint8_t last_error_code) {
        switch (last_error_code) {
        case mcan_last_error_code_no_error:
        case mcan_last_error_code_no_change: return led::CanFault::kNone;
        case mcan_last_error_code_ack_error: return led::CanFault::kNoAck;
        case mcan_last_error_code_bit0_error: return led::CanFault::kWiringFault;
        default: return led::CanFault::kSignalError; // stuff / format / bit1 / crc
        }
    }

    // Divisor that turns a PTPC reported-nanosecond count into true
    // microseconds; fixed at compile time so the ISR-side conversion uses only
    // constant divisions (multiply-and-shift after GCC), and asserted against
    // the real clock tree in the constructor. The value depends on the board's
    // PTPC (AHB) clock -- e.g. 960 = 160 MHz * 6 ns step on HPM5321, 1000 =
    // 200 MHz * 5 ns step on HPM6E80 -- so each board_app.hpp provides it.
    static constexpr uint32_t kTsNsPerUs = board::kCanTimestampNsPerUs;

    const data::DataId data_id_;
    MCAN_Type* can_base_;
    const uint32_t irq_num_;
    const bool canfd_;
    const std::size_t can_index_;

    // Interrupt bookkeeping for poll(). irq_count_ is written only by the ISR
    // and read only by the main loop, so a plain 32-bit counter is enough --
    // aligned loads and stores are atomic on RV32 and only the change matters,
    // not the exact value.
    uint32_t irq_count_ = 0;
    // Frames handed to the serializer since boot. Written only by the receive
    // path (ISR context), read only by the EP0 status query on the main loop --
    // an aligned 32-bit load/store is atomic on RV32 and only the change
    // matters, so no synchronization is needed.
    uint32_t forwarded_frames_ = 0;

    // CSR_MCYCLE at receive-interrupt entry, closed once the frame has been
    // serialized. Written and read inside the same interrupt.
    uint32_t uplink_opened_at_ = 0;
    uint32_t watchdog_irq_count_ = 0;
    bool watchdog_armed_ = false;

    // Software transmit queue in front of the 32-element MCAN TX FIFO, so a USB
    // burst that outruns the bus is buffered instead of discarded. Producer is
    // handle_downlink, consumer is try_transmit; both run in the main loop
    // (TinyUSB defers its receive callback to tud_task), so this is a plain
    // single-producer single-consumer ring with no cross-context hazard.
    // Queued frame, compressed. mcan_tx_frame_t is 72 bytes because its data
    // union is sized for a 64-byte CAN-FD payload, but this protocol caps CAN
    // data at 8 bytes (3-bit DLC, see core/src/protocol/serializer.hpp), so 8
    // header bytes plus 8 data bytes is all that can ever be needed. Storing the
    // SDK type verbatim would spend 72 B per slot for 16 B of live data, so at
    // equal RAM this buys 4x the queue depth -- which is exactly what the burst
    // measurements were short of.
    struct QueuedFrame {
        uint32_t header[2]; // mcan_tx_frame_t words T0/T1
        uint8_t data[8];
    };
    static_assert(sizeof(QueuedFrame) == 16);

    utility::RingBuffer<QueuedFrame, kTransmitQueueSize> transmit_buffer_;
};

// Everything below is built from the board's CAN port table (board::kCanPorts),
// so there are no per-port macros: the count, the FD mode and the dispatch all
// follow the table. Message RAM comes from board::can_message_ram().
//
// kCanCount is the table CAPACITY -- how many Can slots the image carries. On
// most boards that is also how many controllers exist. On a board directory that
// serves two PCBs (boards/hpm5321) the table is sized for the larger variant and
// board::can_port_count() reports how many are actually present at run time,
// which is what every loop must bound on: constructing a Can for a port the PCB
// does not have would clock a controller whose pads belong to something else.
// Slots at or above can_port_count() are left uninitialized, and Lazy leaves
// them safely inert -- try_get() returns nullptr until init() runs.
constexpr size_t kCanCount = board::kCanPortCapacity;
static_assert(kCanCount == std::size(board::kCanPorts));
static_assert(kCanCount >= 1 && kCanCount <= 4);

// Controllers actually present on this board. Equal to kCanCount except on the
// dual-variant hpm5321 image.
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

// Deepest software transmit queue across the controllers this PCB actually has.
// The USB downlink arming policy throttles on this: one backed-up bus is already
// enough to start dropping frames, whichever bus it is. Bounded by can_count()
// and guarded by try_get() for the same reason every other loop over can_array
// is: on the single-CAN hpm5321 the trailing slot was never constructed.
inline size_t max_transmit_queue_depth() {
    size_t depth = 0;
    for (size_t i = 0; i < can_count(); ++i) {
        if (const Can* can = can_array[i].try_get())
            depth = std::max(depth, can->transmit_queue_depth());
    }
    return depth;
}

} // namespace libhcs::firmware::can
