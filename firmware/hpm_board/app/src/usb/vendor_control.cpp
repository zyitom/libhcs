#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include <common/tusb_types.h>
#include <device/usbd.h>
#include <hpm_clock_drv.h>
#include <hpm_soc.h>

#include "core/include/libhcs/protocol/vendor_control.hpp"
#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/diag/latency.hpp"
#include "firmware/hpm_board/app/src/uart/uart.hpp"
#include "firmware/hpm_board/app/src/usb/vendor.hpp"

// EP0 configuration channel -- the board half of libhcs/protocol/vendor_control.hpp.
//
// WHERE THIS RUNS. TinyUSB queues the setup packet from the USB ISR and decodes
// it in tud_task(), so everything below executes on the main loop, on the same
// thread and at the same point in the pass as the bulk downlink callback. That
// matters for the UART path: applying a baudrate aborts the transmit DMA and
// sets LCR.DLAB, during which a DMA write aimed at THR would land in the
// divisor latch instead (uart/uart.hpp). Doing it from interrupt context would
// reintroduce exactly that race.
//
// NOT GATED ON THE SESSION. The host applies configuration while constructing
// the board object, before its keepalive thread has opened a session; and a
// board whose session has lapsed must still answer a read-back. Configuration
// is transport state, not data-plane state.

namespace {

namespace vc = libhcs::core::protocol::vendor_control;

namespace can = libhcs::firmware::can;
namespace uart = libhcs::firmware::uart;
namespace latency = libhcs::firmware::diag::latency;

// Staging buffer for the control data stage. One request is in flight at a
// time by construction -- EP0 is a single serialized pipe and this callback
// runs to completion on the main loop -- so a single static buffer is enough.
// Sized for the largest payload; a request whose wLength disagrees is stalled
// before anything is copied.
alignas(4) uint8_t g_control_buffer[64];

// The board's own view of itself, assembled per request rather than cached: the
// hpm5321 image serves two PCBs and can_port_count() is a runtime read of the
// board identity, so a value baked in at start-up would be a second source of
// truth for something that already has one.
vc::InterfacePayload interface_payload() {
    vc::InterfacePayload payload{};
    payload.version = vc::kVersion;
    payload.can_count = static_cast<uint8_t>(can::can_count());
    payload.uart_count = static_cast<uint8_t>(uart::kUartCount);
    for (std::size_t i = 0; i < can::can_count(); ++i) {
        if (libhcs::firmware::board::can_port(i).mode
            == libhcs::firmware::board::CanMode::kCanFd)
            payload.can_fd_mask |= static_cast<uint8_t>(1U << i);
    }
    return payload;
}

bool can_index_valid(uint16_t index) { return index < can::can_count(); }

bool uart_index_valid(uint16_t index) { return index < uart::kUartCount; }

// Caller must have checked can_index_valid(): board::can_port() indexes the
// port table directly, and on the single-CAN hpm5321 the trailing entry
// describes a controller whose pads are LED cathodes.
vc::CanMode can_mode(uint16_t index) {
    return libhcs::firmware::board::can_port(index).mode
                == libhcs::firmware::board::CanMode::kCanFd
             ? vc::CanMode::kCanFd
             : vc::CanMode::kClassic;
}

// Stage one payload and start the IN data stage for it. Taking the payload by
// type rather than a pointer and a length is what keeps the staged bytes and
// the advertised size from ever disagreeing.
//
// wLength is checked exactly rather than clamped: a host asking for a different
// size is speaking a different version of this interface, and truncating the
// answer would let it decode garbage as a valid reply.
template <typename Payload>
bool reply(uint8_t rhport, const tusb_control_request_t* request, const Payload& payload) {
    static_assert(std::is_trivially_copyable_v<Payload>);
    static_assert(sizeof(Payload) <= sizeof(g_control_buffer));
    if (request->wLength != sizeof(Payload))
        return false;
    std::memcpy(g_control_buffer, &payload, sizeof(Payload));
    return tud_control_xfer(rhport, request, g_control_buffer, sizeof(Payload));
}

// The mirror of reply(): decode the OUT data stage into its payload type.
template <typename Payload>
Payload staged() {
    static_assert(std::is_trivially_copyable_v<Payload>);
    static_assert(sizeof(Payload) <= sizeof(g_control_buffer));
    Payload payload{};
    std::memcpy(&payload, g_control_buffer, sizeof(Payload));
    return payload;
}

bool handle_setup(uint8_t rhport, const tusb_control_request_t* request) {
    const auto index = request->wIndex;

    switch (static_cast<vc::Request>(request->bRequest)) {
    case vc::Request::kGetInterface: {
        if (request->bmRequestType != vc::kRequestTypeIn || index != 0)
            return false;
        if (!reply(rhport, request, interface_payload()))
            return false;
        // Reading the interface IS the handshake: a host that got this far has
        // been told the channel count and the CAN modes, so it cannot be one
        // that predates the move of configuration to EP0. Only from here on is
        // a session allowed -- see HostSession::session_allowed().
        libhcs::firmware::usb::vendor->set_ep0_handshake_done(true);
        return true;
    }

    case vc::Request::kGetCanConfig: {
        if (request->bmRequestType != vc::kRequestTypeIn || !can_index_valid(index))
            return false;
        return reply(
            rhport, request,
            vc::CanConfigPayload{.mode = static_cast<uint8_t>(can_mode(index)), .reserved = {}});
    }

    case vc::Request::kGetLatencyBreakdown: {
        if (request->bmRequestType != vc::kRequestTypeIn)
            return false;
        const auto down = latency::downlink;
        const auto up = latency::uplink;
        // A non-zero index clears the accumulators after the read, so a caller
        // can bracket a measurement instead of always seeing everything since
        // boot. wIndex rather than wValue because the host-side helper only
        // exposes wIndex -- wValue is always sent as zero.
        if (index != 0)
            latency::reset();
        return reply(rhport, request, vc::LatencyBreakdownPayload{
            .downlink_count = down.count,
            .downlink_min_cycles = down.count ? down.min_cycles : 0,
            .downlink_max_cycles = down.max_cycles,
            .downlink_sum_cycles = down.sum_cycles,
            .uplink_count = up.count,
            .uplink_min_cycles = up.count ? up.min_cycles : 0,
            .uplink_max_cycles = up.max_cycles,
            .uplink_sum_cycles = up.sum_cycles,
            .cpu_hz = clock_get_frequency(clock_cpu0),
            .reserved = 0,
        });
    }

    case vc::Request::kGetCanStatus: {
        if (request->bmRequestType != vc::kRequestTypeIn || !can_index_valid(index))
            return false;
        const can::Can* bus = can::can_array[index].try_get();
        if (bus == nullptr)
            return false;
        const auto s = bus->status();
        return reply(rhport, request, vc::CanStatusPayload{
            .tec = s.tec,
            .rec = s.rec,
            .last_error = s.last_error,
            .data_last_error = s.data_last_error,
            .flags = s.flags,
            .reserved = {},
            .tx_occurred = s.tx_occurred,
            .tx_cancelled = s.tx_cancelled,
            .rx_frames = s.rx_frames,
            .rx_fifo_level = s.rx_fifo_level,
        });
    }

    case vc::Request::kGetUartConfig: {
        if (request->bmRequestType != vc::kRequestTypeIn || !uart_index_valid(index))
            return false;
        const uart::Uart* port = uart::uart_array[index].try_get();
        if (port == nullptr)
            return false;
        return reply(
            rhport, request,
            vc::UartConfigPayload{.baudrate = port->effective_baudrate(), .reserved = 0});
    }

    case vc::Request::kSetCanConfig:
        if (request->bmRequestType != vc::kRequestTypeOut || !can_index_valid(index)
            || request->wLength != sizeof(vc::CanConfigPayload))
            return false;
        // Accept the data stage; the value is validated once it has arrived.
        return tud_control_xfer(rhport, request, g_control_buffer, request->wLength);

    case vc::Request::kSetUartConfig:
        if (request->bmRequestType != vc::kRequestTypeOut || !uart_index_valid(index)
            || request->wLength != sizeof(vc::UartConfigPayload))
            return false;
        if (uart::uart_array[index].try_get() == nullptr)
            return false;
        return tud_control_xfer(rhport, request, g_control_buffer, request->wLength);

    default: return false;
    }
}

// Returning false here stalls the status stage, which is the whole point of
// moving configuration onto EP0: it is the acknowledgement the bulk stream
// could not carry. A stall means the setting was NOT applied and the hardware
// is untouched -- the host's read-back will show the old value.
bool handle_data(const tusb_control_request_t* request) {
    const auto index = request->wIndex;

    // The DATA stage fires for IN transfers too, once the reply has been sent,
    // and a false here would stall a read that already succeeded. Only the two
    // OUT requests have anything left to do.
    if (request->bmRequestType != vc::kRequestTypeOut)
        return true;

    switch (static_cast<vc::Request>(request->bRequest)) {
    case vc::Request::kSetCanConfig: {
        const auto payload = staged<vc::CanConfigPayload>();
        if (payload.mode > static_cast<uint8_t>(vc::CanMode::kCanFd))
            return false;
        // Validated, not applied -- see CanConfigPayload for why the controller
        // is never re-initialized. Agreement is all the host needs, because the
        // board's own frames already follow its compiled mode.
        return static_cast<vc::CanMode>(payload.mode) == can_mode(index);
    }

    case vc::Request::kSetUartConfig: {
        const auto payload = staged<vc::UartConfigPayload>();
        uart::Uart* port = uart::uart_array[index].try_get();
        if (port == nullptr)
            return false;
        return port->set_baudrate(payload.baudrate);
    }

    default: return false;
    }
}

} // namespace

// Overrides TinyUSB's weak definition, which stalls every vendor request. Any
// vendor-type request reaches this regardless of recipient (usbd.c dispatches
// on bmRequestType_bit.type before it ever looks at the recipient), so the
// codes in vendor_control.hpp are the only namespace this shares -- the DFU
// runtime interface uses CLASS requests and cannot collide.
extern "C" bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    switch (stage) {
    case CONTROL_STAGE_SETUP: return handle_setup(rhport, request);
    case CONTROL_STAGE_DATA: return handle_data(request);
    default: return true; // CONTROL_STAGE_ACK
    }
}
