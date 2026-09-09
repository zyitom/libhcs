#pragma once

// USB SuperSpeed bulk benchmark mode -- measures what the EP1 pipe can do with
// every application-layer cost removed, so a disappointing forwarding number can
// be attributed to the right layer.
//
// Enabled only by -Dlibhcs_CH32_USB_BENCH=1 (see CMakeLists.txt). With the flag
// off this header expands to nothing and the production firmware is byte-for-byte
// unaffected, which is the point: the bench deliberately breaks the protocol
// (no deserializer, no session, no framing) and must never ship enabled.
//
// WHY A FIRMWARE MODE IS NEEDED AT ALL
//   uplink   there is no data source. With no peripherals attached the
//            serializer stays empty, so the host can never measure EP1 IN.
//   downlink every packet is fed to the deserializer, which scans it byte by
//            byte. At 5 Gbit/s that hits the CPU long before it hits the pipe,
//            so the number would describe the parser, not the link.
//
// The bench also raises the burst count: production tx_write() always arms
// EXP_NUMP = 1 because a protocol batch is capped at 1023 bytes, while the
// endpoint companion descriptor advertises bMaxBurst 15. The bench arms full
// 16 x 1024 byte bursts, which is what the hardware is actually rated for.

#if libhcs_CH32_USB_BENCH

# include <cstdint>

namespace libhcs::firmware::usb::bench {

// Host-selected mode. The host switches modes by sending one OUT packet whose
// first word is kCommandMagic; the byte after it is the Mode value. A command
// packet is recognised in every mode, so one build serves all three tests.
enum class Mode : uint8_t {
    kIdle = 0,   // nothing armed; the resting state after enumeration
    kSink = 1,   // count and discard OUT traffic  -> measures host->device
    kSource = 2, // flood IN with full bursts      -> measures device->host
    kEcho = 3,   // bounce each OUT burst back     -> measures round-trip latency
};

inline constexpr uint32_t kCommandMagic = 0xB0B0B0B0u;

// Called from the patched EP1 interrupt hooks in vendor.cpp and from the main
// loop. All three are no-ops until the host selects a mode.
void handle_ep1_out();
void handle_ep1_in();
void poll();

} // namespace libhcs::firmware::usb::bench

#endif // libhcs_CH32_USB_BENCH
