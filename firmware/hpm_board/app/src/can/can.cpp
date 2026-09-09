#include "firmware/hpm_board/app/src/can/can.hpp"

#include "firmware/hpm_board/app/src/sync/timebase.hpp"

#include <algorithm>
#include <cstddef>
#include <cstring>

#include "core/src/utility/assert.hpp"
#include "firmware/hpm_board/app/src/diag/can_diag.hpp"
#include "firmware/hpm_board/app/src/diag/latency.hpp"

// The CAN forwarding hot path is defined out-of-line here, in the ILM (.fast)
// section, rather than inline in can.hpp. ILM is zero-wait-state and never
// I-cache-misses, so this removes FLASH-XIP fetch jitter from the worst-case
// forwarding latency. Out-of-line (not inline-in-class) is deliberate: an
// inline/COMDAT function placed in .fast collides with the plain .fast
// functions here (a GCC "section type conflict"). Leaf calls into the MCAN
// driver (mcan_read_rxfifo) and the shared serializer (write_can) stay in
// FLASH -- they are SDK/core code -- so the win is on our glue plus the inlined
// MCAN register helpers, not the whole path.

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
    // Frame type follows the BUS, not the frame. It used to be chosen per frame
    // from the host's is_fdcan flag; that flag now lives on EP0 as a per-bus
    // setting the host reads back during its construction handshake
    // (libhcs/protocol/vendor_control.hpp), so the only value that can be in
    // force here is this controller's own compiled mode. Reading the header bit
    // as well would let a host and a board disagree about a frame that the wire
    // had already settled.
    const bool send_fd = canfd_;
    frame.canfd_frame = send_fd;
    frame.bitrate_switch = send_fd;
    frame.rtr = data.is_remote_transmission;

    core::utility::assert_debug(data.can_data.size() <= 8);
    frame.dlc = data.can_data.size();
    if (!data.can_data.empty())
        std::memcpy(frame.data_8, data.can_data.data(), data.can_data.size());

    // Straight to the controller while it has room, and only queue behind a
    // full TX FIFO. The FIFO is 32 elements, but the host does not deliver at
    // the rate the bus drains: USB hands over whatever accumulated since the
    // last (micro)frame, so a burst can exceed 32 even when the average rate is
    // well under bus capacity. Writing directly and giving up -- what this did
    // before -- dropped those bursts outright, and the only sign was a cyan LED.
    //
    // The queue must not be bypassed once it is non-empty, or a later frame
    // would overtake an earlier one. Checking it here is safe despite
    // RingBuffer's consumer-only warning on peek_front(): producer
    // (handle_downlink, reached from tud_task) and consumer (try_transmit) both
    // run in the main loop, on the same thread.
    //
    // Queueing only on overflow keeps the common path free of any added work.
    if (transmit_buffer_.peek_front() == nullptr
        && mcan_transmit_via_txfifo_nonblocking(can_base_, &frame, nullptr) == status_success) {
        diag::latency::close_downlink();
        return;
    }

    // Compress into the queue element: T0/T1 plus the (at most 8) data bytes.
    // mcan_tx_frame_t's leading words are exactly T0/T1, so they copy straight
    // across; the static_assert below pins that layout assumption.
    static_assert(offsetof(mcan_tx_frame_t, data_8) == 8);
    QueuedFrame queued;
    std::memcpy(queued.header, &frame, sizeof(queued.header));
    std::memcpy(queued.data, frame.data_8, sizeof(queued.data));

    if (!transmit_buffer_.emplace_back(queued)) {
        led::led->downlink_buffer_full();
        diag::note_tx_fail(can_index());
    }
}

ATTR_PLACE_AT(".fast")
void Can::try_transmit() {
    while (const QueuedFrame* queued = transmit_buffer_.peek_front()) {
        // Rebuild the SDK frame from the compressed record. Zero-initialized so
        // the data words above the 8 bytes this protocol can carry are defined,
        // whatever DLC the header asks for.
        mcan_tx_frame_t frame{};
        std::memcpy(&frame, queued->header, sizeof(queued->header));
        std::memcpy(frame.data_8, queued->data, sizeof(queued->data));

        if (mcan_transmit_via_txfifo_nonblocking(can_base_, &frame, nullptr) != status_success)
            return; // FIFO full; the rest stays queued for the next pass
        // The queue API transfers ownership to the callback; discarding the
        // completed frame intentionally does not require moving from it.
        // NOLINTNEXTLINE(cppcoreguidelines-rvalue-reference-param-not-moved)
        transmit_buffer_.pop_front([](QueuedFrame&&) noexcept {});
    }
}

ATTR_PLACE_AT(".fast")
bool Can::read_uplink(data::CanDataView& data, uint8_t storage[8], bool& valid) {
    valid = false;
    data = {};
    mcan_rx_message_t rx;
    if (mcan_read_rxfifo(can_base_, 0, &rx) != status_success)
        return false;

    // rx.dlc is the raw 4-bit DLC from the wire, not a byte count, and every
    // value 0-15 can legally arrive from other bus nodes -- so it must be
    // normalized here instead of trusted downstream (the serializer rejects
    // out-of-contract views, and that rejection must not translate into an
    // assert on externally-controlled input):
    //   - Remote frames carry no data field; their DLC encodes the requested
    //     length, which the wire protocol cannot express. Forward them empty.
    //   - Classic frames may carry DLC 9-15, which the CAN spec says to treat
    //     as 8 data bytes.
    //   - FD frames with DLC > 8 hold 12-64 data bytes. The RX element data
    //     field is 8 bytes, so the hardware stored the payload truncated, and
    //     the wire protocol caps at 8 bytes anyway -- forwarding would deliver
    //     silently corrupted data. Drop the frame (but keep draining).
    if (rx.canfd_frame && rx.dlc > 8) [[unlikely]]
        return true;
    const size_t data_length = rx.rtr ? 0 : std::min<size_t>(rx.dlc, 8);

    data.is_fdcan = rx.canfd_frame;
    data.is_extended_can_id = rx.use_ext_id;
    data.is_remote_transmission = rx.rtr;
    data.can_id = data.is_extended_can_id ? rx.ext_id : rx.std_id;
    if (data_length != 0)
        std::memcpy(storage, rx.data_8, data_length);
    data.can_data = {reinterpret_cast<const std::byte*>(storage), data_length};

    // 64-bit TSU timestamp captured at SOF from the shared PTPC0 timebase.
    // PTPC delivers an IEEE-1588 {seconds:nanoseconds} pair (high 32 bits =
    // seconds, low 32 bits = nanoseconds in [0, 1e9)); dividing reported
    // nanoseconds by kTsNsPerUs (= 960 at 160 MHz, not 1000) converts to
    // microseconds and undoes the PTPC digital-step error in one step.
    //
    // The conversion is deliberately 32-bit only: RV32 has no 64-bit divide
    // instruction, so a 64/32 division here would become a __udivdi3 library
    // loop (~hundreds of cycles) inside the highest-priority ISR. With the
    // seconds and nanoseconds words kept apart,
    //   us = sec * (1e9 / 960) + sec * (640 / 960 == 2/3) + ns / 960
    // needs only constant divisions, which GCC lowers to multiply-and-shift.
    // Versus the exact quotient this truncates at most 1 us per conversion
    // and the error does not accumulate. The result wraps every ~71.6 min;
    // the host only uses deltas, which are wrap-safe.
    // status_mcan_timestamp_not_exist (frame not matched by a sync filter)
    // leaves the field as std::nullopt.
    mcan_timestamp_value_t ts_value;
    if (mcan_get_timestamp_from_received_message(can_base_, &rx, &ts_value) == status_success
        && ts_value.is_64bit) {
        const auto sec = static_cast<uint32_t>(ts_value.ts_64bit >> 32);
        const auto ns = static_cast<uint32_t>(ts_value.ts_64bit);
        data.timestamp_us =
            (sec * (1'000'000'000U / kTsNsPerUs)) + ((sec * 2U) / 3U) + (ns / kTsNsPerUs);

        // Cross-board synchronisation probe. Only this one CAN id takes the
        // branch, so the cost on the forwarding hot path is a compare; the raw
        // capture is queued untouched because converting it to the shared axis
        // costs two 64-bit divisions, which have no business running inside the
        // highest-priority interrupt on this board.
        if constexpr (sync::timebase::kEnabled) {
            if (data.can_id == data::kSyncProbeCanId && data_length >= sizeof(uint32_t))
                [[unlikely]] {
                uint32_t tag = 0;
                std::memcpy(&tag, storage, sizeof(tag));
                // The nanosecond word only, matching how the time base samples
                // PTPC: the seconds register is not latched with it and a
                // composed pair is not coherent. Nothing needs the epoch.
                sync::timebase::note_can_capture(tag, ns, static_cast<uint8_t>(data_id_));
            }
        }
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
    // Unreachable for wire input after the normalization above; guards only
    // against internal contract regressions.
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
        // One increment on the forwarding path, unconditional -- unlike the
        // diag counter next to it, this one has to exist in the shipping image:
        // it is what tells a host "this bus has received nothing at all" from
        // "this bus is receiving but something downstream drops it", and that
        // distinction is the first fork of every dead-bus investigation.
        ++forwarded_frames_;
        diag::latency::close_uplink(uplink_opened_at_);
        diag::note_frame(can_index());
    }

    return true;
}

ATTR_PLACE_AT(".fast")
void Can::irq_handler() {
    // Counted before the flag read so a delivered-but-empty interrupt still
    // shows up: a frozen entry count is the signal that separates "the
    // interrupt stopped arriving" from "the controller stopped receiving".
    diag::note_isr_entry(can_index());
    // Stamped here, consumed in handle_uplink below: both run in this same
    // interrupt, so a plain member needs no synchronization.
    uplink_opened_at_ = diag::latency::now();
    irq_count_++;

    uint32_t flags = mcan_get_interrupt_flags(can_base_);

    if (!flags) [[unlikely]]
        return;

    // Handle, then re-read, and only return once IR shows no enabled source
    // left. The re-read is what makes this correct, and it is not an
    // optimization -- without it the controller goes permanently deaf:
    //
    // The M_CAN drives its interrupt line from the level of (IR & IE), but the
    // PLIC gateway latches it as an EDGE: a new request is registered only when
    // that expression goes from zero to non-zero. Clearing IR once on entry and
    // then draining leaves a window in which a frame arriving mid-drain sets
    // RF0N again. Nothing clears it afterwards, so the line stays high, no
    // further edge is ever produced, and the interrupt is never delivered
    // again. The FIFO then fills and stays full while the bus is perfectly
    // healthy. Measured in exactly that state at 16 kHz per stream:
    //   IR = 0x0001000f (RF0N|RF0W|RF0F|RF0L set), RXF0S fill = 32, F0F = 1,
    //   RF0L = 1, PSR clean (no bus-off, no error-passive, act = rx),
    //   PLIC pending bit for the source clear, ISR entry count frozen.
    // Only a reset recovered it, which is what made this look like a lost PLIC
    // claim rather than a missed edge.
    //
    // With the loop, anything set during handling is handled and cleared in
    // this same invocation, so the ISR returns only after observing IR with no
    // enabled bit -- i.e. with the line genuinely low, so the next frame does
    // produce an edge. The loop terminates because one pass drains the whole
    // FIFO in far less time than a CAN-FD frame takes on the wire.
    do {
        handle_interrupt_flags(flags);
        flags = mcan_get_interrupt_flags(can_base_);
    } while ((flags & kEnabledInterrupts) != 0);
}

// Main-loop repair for an interrupt request the PLIC accepted but never
// delivered again.
//
// Measured failure state, read out of the running board at 16 kHz per stream
// (host/examples/can_stall_probe.cpp):
//   IR = 0x0001000f -> RF0N set and IE has it enabled, so the M_CAN interrupt
//                      line is HIGH
//   RXF0S fill = 32, F0F = 1, RF0L = 1  -> the FIFO filled and started dropping
//   PSR clean (no bus-off, no error-passive, act = rx) -> the bus is fine
//   PLIC trigger type for the source = 0 -> LEVEL triggered, not edge
//   PLIC pending bit for the source = 0
//   ISR entry count frozen, forever, until reset
//
// A level-triggered gateway with the line high must keep the request asserted;
// the only state in which it does not is "already claimed, never completed", so
// the completion for this source was lost. That is consistent with the nested
// interrupt wrapper the SDK generates around every ISR, which re-enables
// mstatus.MIE for the duration of the handler and writes the PLIC completion
// only after it returns.
//
// Rather than depend on where exactly the completion went missing, this makes
// the condition self-correcting: a completion write is defined to be ignored
// for a source the target is not servicing, so issuing one here is a no-op
// unless the gateway really is stuck -- in which case it releases it and the
// line, still high, immediately re-raises the interrupt.
//
// Cost on the healthy path is one MMIO read plus a compare, and it triggers
// only when the line has been high across two consecutive main-loop passes
// with no ISR entry in between. That cannot happen normally: with interrupts
// enabled and a level-sensitive source, the ISR is entered before the main loop
// gets to look twice.
void Can::poll() {
    const uint32_t flags = mcan_get_interrupt_flags(can_base_);
    if ((flags & kEnabledInterrupts) == 0) [[likely]] {
        watchdog_armed_ = false;
        return;
    }

    const uint32_t count = irq_count_;
    if (!watchdog_armed_ || count != watchdog_irq_count_) {
        // First sighting, or the ISR has run since the previous one. The line
        // being high here only means an interrupt is on its way.
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
        // Drain the FIFO completely: RF0N is a status bit, not a counter, so
        // one interrupt may stand for several buffered frames.
        if (link::uplink_enabled()) {
            // can_uplink_serializer(), not uplink_serializer(): with the CAN
            // endpoint split enabled these frames go into their own batch pool
            // and out on their own bulk endpoint, so a UART batch cannot occupy
            // the pipe ahead of them. Identical to uplink_serializer() otherwise.
            auto& serializer = link::can_uplink_serializer();
            while (handle_uplink(data_id_, serializer)) {}
        } else {
            mcan_rx_message_t rx;
            while (mcan_read_rxfifo(can_base_, 0, &rx) == status_success) {}
        }
    }

    if (flags
        & (MCAN_INT_BUS_OFF_STATUS | MCAN_INT_WARNING_STATUS | MCAN_INT_ERROR_PASSIVE
           | MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE | MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE)) {
        // Any error interrupt refreshes the indicator LED so it stays
        // visible while errors keep occurring.  Bus-off is the fatal state
        // and wins outright; otherwise the Last Error Code (LEC) names the
        // protocol error on the wire.  For CAN-FD the arbitration phase uses
        // LEC and the faster data phase uses DLEC -- prefer a concrete data-
        // phase code when the arbitration phase reports none.  A bus state
        // change (warning/passive) carrying no fresh LEC reports kNone, which
        // only refreshes the timeout (see Led::report_can_fault).
        led::CanFault fault = led::CanFault::kNone;
        if (mcan_is_in_busoff_state(can_base_)) {
            fault = led::CanFault::kBusOff;
            // Bus-off latches CCCR.INIT and halts the controller. This is a
            // forwarding bridge: a transient fault on the wire (downstream
            // node unplugged, no ACK) must not take the CAN port offline
            // until reboot. Clearing INIT starts the standard bus-off
            // recovery sequence -- the controller waits for the bus to go
            // idle (129 * 11 recessive bits), resets its error counters and
            // resumes automatically. While the bus stays faulty it simply
            // cycles back to bus-off and retries, keeping the port alive.
            mcan_enter_normal_mode(can_base_);
        } else {
            fault = classify_can_fault(mcan_get_last_error_code(can_base_));
            if (fault == led::CanFault::kNone)
                fault = classify_can_fault(mcan_get_data_phase_last_error_code(can_base_));
        }
        // Index the indicator by this controller's position in board::kCanPorts,
        // which is what report_can_fault's per-controller state is keyed on.
        // The previous two-way "first port or else the second" test collapses
        // every controller above the second onto indicator 1, so on a board with
        // more than two CAN ports plus a matching indicator table, a CAN2/CAN3
        // fault would light CAN1's LED and overwrite CAN2's own state. Latent
        // rather than
        // observable today -- the only >2-CAN board (hpm6e8y) ships an empty
        // kCanIndicatorPins, and report_can_fault() bounds-checks against it, so
        // the bad index was discarded there. Deriving it correctly keeps that
        // from becoming a real misreport the moment such a board gains LEDs.
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
