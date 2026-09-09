#include "firmware/ch32_board/app/src/usb/bench.hpp"

#if libhcs_CH32_USB_BENCH

# include <cstddef>
# include <cstdint>

extern "C" {
# include "ch32h417_usbss_device.h"
}

namespace libhcs::firmware::usb::bench {
namespace {

// One SuperSpeed bulk packet, and the burst the endpoint companion descriptor
// advertises (bMaxBurst 15 means 16 packets). 16 x 1024 = 16 KiB per chain,
// which is also the size of one half of the vendor RX buffer.
constexpr uint32_t kPacketSize = DEF_USB_EP1_SS_SIZE;
constexpr uint32_t kBurstPackets = DEF_ENDP1_IN_BURST_LEVEL;

// Source pattern. Never read back, so its contents do not matter -- but it must
// not share storage with the RX chains, which the OUT path re-arms underneath us.
__attribute__((aligned(4))) uint8_t tx_buffer[kPacketSize * kBurstPackets];

volatile Mode mode = Mode::kIdle;
volatile bool tx_busy = false;

// Counters live in the same debugger-readable RAM window the rest of the
// bring-up instrumentation uses (PITFALLS.md 4.6), past the words app.cpp owns,
// so a run can be inspected over SDI when the host side is not trusted.
volatile uint32_t* counters() { return reinterpret_cast<volatile uint32_t*>(0x201700A0u); }

// Arm one full burst on EP1 IN.
//
// The register quartet is what the vendor demo's EP3 self-echo does, and the
// order matters: EXP_NUMP is written last because writing it is what starts the
// transfer (RM 27.2.3, RB_TX_CHAIN_EN).
//   UEP_TX_DMA      base of the burst
//   UEP_TX_DMA_OFS  stride between packets -- production tx_write() never sets
//                   this because it only ever arms EXP_NUMP = 1
//   UEP_TX_CHAIN_LEN length of the LAST packet only (mirrors the RX side); a
//                   full-length last packet keeps the transfer un-terminated,
//                   which is what a throughput stream wants
//   UEP_TX_CHAIN_EXP_NUMP packets in the burst
void arm_source_burst() {
    tx_busy = true;
    USBSSD->EP1_TX.UEP_TX_DMA = reinterpret_cast<uint32_t>(tx_buffer);
    USBSSD->EP1_TX.UEP_TX_DMA_OFS = kPacketSize;
    USBSSD->EP1_TX.UEP_TX_CHAIN_LEN = kPacketSize;
    USBSSD->EP1_TX.UEP_TX_CHAIN_EXP_NUMP = kBurstPackets;
}

// Bounce the burst that just landed straight back out. DMA points into the RX
// chain the caller has already finished with; the OUT path re-arms the OTHER
// half, so the hardware is never reading and writing the same memory.
void arm_echo(uint32_t start, uint32_t offset, uint32_t last_len, uint32_t nump) {
    tx_busy = true;
    USBSSD->EP1_TX.UEP_TX_DMA = start;
    USBSSD->EP1_TX.UEP_TX_DMA_OFS = offset;
    USBSSD->EP1_TX.UEP_TX_CHAIN_LEN = last_len;
    USBSSD->EP1_TX.UEP_TX_CHAIN_EXP_NUMP = nump;
}

void rearm_receive() {
    EP1_Chain_Sel ^= 0x01;
    USBSSD->EP1_RX.UEP_RX_DMA = reinterpret_cast<uint32_t>(
        &USBSS_EP1_Rx_Buf[DEF_USB_EP1_SS_SIZE * DEF_ENDP1_OUT_BURST_LEVEL * EP1_Chain_Sel]);
    USBSSD->EP1_RX.UEP_RX_CHAIN_MAX_NUMP = DEF_ENDP1_OUT_BURST_LEVEL;
    USBSSD->EP1_RX.UEP_RX_CHAIN_ST |= USBSS_EP_RX_CHAIN_IF;
}

// 64-bit byte totals kept as two 32-bit words: the debugger reads words, and a
// 5 Gbit/s run overflows 32 bits in under 9 seconds.
void add_bytes(size_t low_index, uint32_t amount) {
    auto* c = counters();
    const uint32_t before = c[low_index];
    c[low_index] = before + amount;
    if (c[low_index] < before)
        c[low_index + 1]++;
}

} // namespace

void handle_ep1_out() {
    const uint32_t nump = USBSSD->EP1_RX.UEP_RX_CHAIN_NUMP;
    const uint32_t offset = USBSSD->EP1_RX.UEP_RX_DMA_OFS;
    const uint32_t last_len = USBSSD->EP1_RX.UEP_RX_CHAIN_LEN;
    const uint32_t start = USBSSD->EP1_RX.UEP_RX_DMA - nump * offset;
    const uint32_t size = nump ? (nump - 1) * offset + last_len : 0;

    // Mode commands are recognised in every mode, including kSink, so the host
    // can always steer the board without a power cycle.
    bool command = false;
    if (size >= 8 && *reinterpret_cast<const volatile uint32_t*>(start) == kCommandMagic) {
        const auto requested = *reinterpret_cast<const volatile uint8_t*>(start + 4);
        if (requested <= static_cast<uint8_t>(Mode::kEcho)) {
            mode = static_cast<Mode>(requested);
            counters()[0] = requested;
            for (size_t i = 1; i <= 7; i++)
                counters()[i] = 0;
            tx_busy = false;
            command = true;
        }
    }

    counters()[1]++;
    counters()[7] = size;
    add_bytes(2, size);

    const Mode current = mode;
    // Echo before re-arming: arm_echo() reads the chain the burst landed in, and
    // rearm_receive() flips the hardware to the other half.
    if (current == Mode::kEcho && !command && !tx_busy && nump)
        arm_echo(start, offset, last_len, nump);

    rearm_receive();
}

void handle_ep1_in() {
    USBSSD->EP1_TX.UEP_TX_CHAIN_ST |= USBSS_EP_TX_CHAIN_IF;
    tx_busy = false;

    counters()[4]++;
    add_bytes(5, kPacketSize * kBurstPackets);

    // Re-arm from the interrupt rather than the main loop: the gap between
    // bursts is then one ISR latency instead of one loop iteration, which is the
    // difference between measuring the pipe and measuring our polling rate.
    if (mode == Mode::kSource)
        arm_source_burst();
}

void poll() {
    // Only kicks off the first burst of a run (and recovers if a link reset ate
    // the armed chain); steady state is driven by handle_ep1_in().
    if (mode == Mode::kSource && !tx_busy && USBSS_DevEnumStatus != 0)
        arm_source_burst();
}

} // namespace libhcs::firmware::usb::bench

#endif // libhcs_CH32_USB_BENCH
