#include "firmware/ch32_board/app/src/usb/vendor.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

extern "C" {
#include "ch32h417_usbss_device.h"
}

#include "core/src/protocol/serializer.hpp"
#include "firmware/ch32_board/app/src/link/uplink.hpp"
#include "firmware/ch32_board/app/src/usb/bench.hpp"

namespace libhcs::firmware {

// This application's host transport is the USB SuperSpeed vendor class; bind the
// transport-neutral hooks the CAN/UART drivers call to it.
namespace link {

core::protocol::Serializer& uplink_serializer() { return usb::vendor->serializer(); }

bool uplink_enabled() { return usb::vendor->uplink_enabled(); }

} // namespace link

namespace usb {
namespace ss {

namespace {
// Uplink TX staging buffer for bulk IN endpoint 1, aligned for the USBSS DMA.
__attribute__((aligned(4))) uint8_t tx_packet_buffer[Vendor::kMaxPacketSize];

// Set when an IN packet is armed on EP1, cleared by the EP1 IN-complete ISR hook
// below. volatile: shared between the forwarding loop and the USBSS interrupt.
volatile bool tx_in_flight = false;
} // namespace

bool enumerated() { return USBSS_DevEnumStatus != 0; }

bool tx_ready() { return enumerated() && !tx_in_flight; }

// Release the transmit gate when the SuperSpeed link drops.
//
// tx_in_flight is otherwise cleared only by the EP1 IN completion interrupt,
// and that interrupt never arrives if the link goes down while a packet is
// armed: cable removal, hot reset and warm reset all run
// USBSS_Reset_Init(ENABLE) -> USBSS_Device_Endp_Init() -> USBSS_USB_CLR_ALL,
// which discards the armed chain in hardware. The flag would then stay set for
// the rest of the power cycle, tx_ready() would never come back, and the ack
// for the host's next kStart could not be sent -- the board enumerates and
// accepts downlink but answers nothing, i.e. it looks dead after an
// unplug/replug until it is reset.
//
// Edge-triggered (link up -> down) rather than level-triggered: a bare
// SET_CONFIGURATION(0) also clears USBSS_DevEnumStatus but leaves the endpoint
// armed, and clearing the gate there would let the next batch overwrite a
// buffer the controller still owns.
void poll_link_reset() {
    static bool link_was_up = false;

    const bool link_is_up = enumerated();
    if (link_was_up && !link_is_up)
        tx_in_flight = false;
    link_was_up = link_is_up;
}

// Arm one bulk-IN packet on EP1. The (DMA, CHAIN_LEN, CHAIN_EXP_NUMP) sequence
// mirrors the demo's real EP3 IN arming in USBSS_Device_Endp_Init(); EXP_NUMP is
// the SuperSpeed burst count and writing it is what starts the transfer
// (RM 27.2.3, RB_TX_CHAIN_EN), so it stays last.
//
// EXP_NUMP is always 1 here even though the endpoint companion descriptor
// advertises bMaxBurst 15. That is not an oversight to "optimize" away: a burst
// packet other than the last must be exactly 1024 bytes (RM 27.2.5.1), while a
// batch is capped at core::protocol::kProtocolBufferSize == 1023. One batch can
// therefore never be more than one (always short) packet, and coalescing several
// batches into full 1024-byte packets would change the uplink framing the host
// SDK parses. Raising the ceiling is a protocol change in core/ + host/, not a
// firmware-local one.
bool tx_write(const uint8_t* data, size_t size) {
    if (size > sizeof(tx_packet_buffer))
        return false;
    std::memcpy(tx_packet_buffer, data, size);

    tx_in_flight = true;
    USBSSD->EP1_TX.UEP_TX_DMA = reinterpret_cast<uint32_t>(tx_packet_buffer);
    USBSSD->EP1_TX.UEP_TX_CHAIN_LEN = static_cast<uint16_t>(size);
    USBSSD->EP1_TX.UEP_TX_CHAIN_EXP_NUMP =
        static_cast<uint8_t>((size + Vendor::kMaxPacketSize - 1) / Vendor::kMaxPacketSize);

    // TODO(usb-bringup): temporary, pairs with the EP1 OUT counters.
    {
        auto* diag = reinterpret_cast<volatile uint32_t*>(0x20170000u);
        diag[23]++; // uplink packets armed
        diag[24] = size;
    }
    return true;
}

bool tx_write_zlp() {
    tx_in_flight = true;
    USBSSD->EP1_TX.UEP_TX_DMA = reinterpret_cast<uint32_t>(tx_packet_buffer);
    USBSSD->EP1_TX.UEP_TX_CHAIN_LEN = 0;
    USBSSD->EP1_TX.UEP_TX_CHAIN_EXP_NUMP = 1;
    return true;
}

} // namespace ss

// ---- USBSS interrupt hooks (called from patched ch32h417_usbss_it.c) ---------
// These replace the CH372 demo's EP1 hardware DMA loopback with genuine libhcs
// endpoint servicing. See bsp/PROVENANCE.md for the vendored-ISR patch record.

extern "C" {

// EP1 bulk IN (device -> host uplink) transfer complete: ack the chain and free
// the endpoint so the forwarding loop can arm the next batch.
void usb_ss_ep1_in_complete(void) {
#if libhcs_CH32_USB_BENCH
    bench::handle_ep1_in();
#else
    USBSSD->EP1_TX.UEP_TX_CHAIN_ST |= USBSS_EP_TX_CHAIN_IF;
    ss::tx_in_flight = false;

    // TODO(usb-bringup): temporary, pairs with the EP1 OUT counters.
    {
        auto* diag = reinterpret_cast<volatile uint32_t*>(0x20170000u);
        diag[25]++; // uplink packets the host actually collected
    }
#endif
}

// EP1 bulk OUT (host -> device downlink) transfer complete: hand the received
// bytes to the deserializer, then re-arm the RX chain.
//
// The three chained-DMA registers do not mean what their names suggest
// (reference manual 27.2.4, table 27-10):
//   UEP_RX_CHAIN_LEN   length of the LAST packet of the burst only; every
//                      earlier packet is exactly UEP_RX_DMA_OFS (1024) bytes,
//                      which the protocol mandates (RM 27.2.5.1, RB_UH_TX_LEN).
//   UEP_RX_CHAIN_NUMP  packets actually received into this chain.
//   UEP_RX_DMA         advanced past the data by the time this fires, so the
//                      start of the burst has to be walked back.
// This is the same computation the vendor ISR does for its own EP2 OUT case.
void usb_ss_ep1_out_complete(void) {
#if libhcs_CH32_USB_BENCH
    bench::handle_ep1_out();
#else
    const uint32_t nump = USBSSD->EP1_RX.UEP_RX_CHAIN_NUMP;
    const uint32_t offset = USBSSD->EP1_RX.UEP_RX_DMA_OFS;
    const uint32_t last_packet_size = USBSSD->EP1_RX.UEP_RX_CHAIN_LEN;

    const auto* data =
        reinterpret_cast<const std::byte*>(USBSSD->EP1_RX.UEP_RX_DMA - nump * offset);
    const uint32_t size = nump ? (nump - 1) * offset + last_packet_size : 0;

    // TODO(usb-bringup): temporary. The address computation above assumes
    // UEP_RX_DMA auto-advances past the received data -- inferred from the
    // vendor ISR's own EP2 OUT case, not stated in the reference manual. Record
    // the raw registers so the assumption can be checked against hardware.
    {
        auto* diag = reinterpret_cast<volatile uint32_t*>(0x20170000u);
        diag[15]++; // EP1 OUT completions
        diag[16] = nump;
        diag[17] = offset;
        diag[18] = last_packet_size;
        diag[19] = USBSSD->EP1_RX.UEP_RX_DMA;                    // as read after the transfer
        diag[20] = reinterpret_cast<uint32_t>(USBSS_EP1_Rx_Buf); // armed base, for comparison
        diag[21] = size;
        // First bytes actually landed in the buffer base, so a wrong start
        // address is visible even when the computed one points elsewhere.
        diag[22] = *reinterpret_cast<const volatile uint32_t*>(USBSS_EP1_Rx_Buf);
    }

    // A burst ends on a short packet, which is what closes the USB transfer --
    // the total size does not decide it, the last packet does.
    vendor->handle_downlink({data, size}, last_packet_size < Vendor::kMaxPacketSize);

    // Hand the hardware the OTHER buffer. UEP_RX_CFG resets to CHAIN_AUTO=1 and
    // the endpoint owns four chains (RM 27.2.4.5), so a burst can land while
    // this handler is still reading; re-arming the buffer we just read would let
    // it be overwritten underneath us. USBSS_Device_Endp_Init() pre-loads both
    // halves and leaves EP1_Chain_Sel on the one it armed last.
    EP1_Chain_Sel ^= 0x01;
    USBSSD->EP1_RX.UEP_RX_DMA = reinterpret_cast<uint32_t>(
        &USBSS_EP1_Rx_Buf[DEF_USB_EP1_SS_SIZE * DEF_ENDP1_OUT_BURST_LEVEL * EP1_Chain_Sel]);
    // Writing MAX_NUMP is what re-enables the chain, so it comes after the DMA
    // address (RM 27.2.4.6, RB_EP_RX_CHAIN_EN).
    USBSSD->EP1_RX.UEP_RX_CHAIN_MAX_NUMP = DEF_ENDP1_OUT_BURST_LEVEL;
    USBSSD->EP1_RX.UEP_RX_CHAIN_ST |= USBSS_EP_RX_CHAIN_IF;
#endif
}

} // extern "C"

} // namespace usb
} // namespace libhcs::firmware
