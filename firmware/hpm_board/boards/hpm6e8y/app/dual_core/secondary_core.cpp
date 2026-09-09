#include "firmware/hcs_board/app/src/xcore/secondary_core.hpp"

// Single-core builds get inline no-ops from the header, so this translation unit
// is empty for them (the SDK globs it in either way).
#if defined(libhcs_APP_RELEASE_CORE1) && libhcs_APP_RELEASE_CORE1

# include <cstddef>
# include <cstdint>

# include <board.h>
# include <hpm_clock_drv.h>
# include <hpm_debug_console.h>
# include <hpm_mbx_drv.h>
# include <hpm_soc.h>

extern "C" {
// The SDK sample helper carries no extern "C" guard of its own (it is only ever
// consumed from C sources), and it relies on uint8_t/uint32_t already being
// declared, so it has to follow the headers above inside this block.
# include <multicore_common.h>
}

# include "core/include/libhcs/data/datas.hpp"
# include "core/src/protocol/protocol.hpp"
# include "core/src/protocol/serializer.hpp"
# include "firmware/hcs_board/app/src/link/uplink.hpp"
# include "firmware/hcs_board/app/src/timer/timer.hpp"
# include "firmware/hcs_board/ecat/common/xcore_channel.hpp"
# include "firmware/hcs_board/ecat/common/xcore_diag.hpp"

namespace libhcs::firmware::xcore {
namespace {

// Core0-local. Set once by publish_channel() before core1 exists, read-only
// afterwards, so no synchronization of its own is needed -- the channel contents
// carry the cross-core ordering. Named g_channel because channel() below is the
// accessor the data plane uses.
ecat::XcoreChannel* g_channel = nullptr;

// Upper bound on bytes moved per main-loop pass. The real protection is the
// non-blocking FIFO check below, not this cap; it just bounds the loop.
constexpr std::size_t kDiagBytesPerPass = 32;

} // namespace

void publish_channel() {
    // board_init_pmp() (from board_init()) has already mapped SHARE_RAM
    // non-cacheable with AMO enabled; placement-constructing the channel is the
    // first access to that region.
    g_channel = &ecat::xcore_channel_init();

    // Open the shared MBX0 clock gate here rather than on core1: core1 may poke
    // MBX0B immediately after release, and a write to a gated mailbox is
    // silently lost. Only the mailbox is reset -- no interrupt is enabled,
    // because in this migration step core1 has nothing to tell core0 that the
    // diagnostic ring does not already carry.
    clock_add_to_group(clock_mbx0, 0);
    mbx_init(HPM_MBX0A);
}

ecat::XcoreChannel* channel() { return g_channel; }

void ring_uplink_doorbell() {
    // Publish the ring bytes BEFORE the doorbell. XcoreRing::try_push ends in a
    // release store, which orders the payload ahead of the index but does NOT
    // order that non-cacheable store ahead of the following device-register
    // write on RISC-V. A full fence (memory + I/O, both directions) is what
    // guarantees core1 observes the pushed bytes once it sees the mailbox word.
    // The HPM multicore samples omit this; dropping it yields a rare, ugly
    // failure where the doorbell arrives but the payload does not.
    __asm__ volatile("fence" ::: "memory");
    // Send failure means a poke is already pending in the single-word mailbox;
    // ignore it -- one interrupt is enough, and the handler re-reads the ring.
    (void)mbx_send_message(HPM_MBX0A, 0);
}

void release_core1() {
    // Copies the embedded image into core1's ILM over SDP DMA, flushes the
    // source range from core0's D-cache, then starts the core. Idempotent: the
    // SDK helper checks sysctl_is_cpu_released() first.
    multicore_release_cpu(HPM_CORE1, SEC_CORE_IMG_START);
}

bool wait_for_core1_eeprom(std::uint32_t timeout_ms) {
    if (g_channel == nullptr)
        return true;

    // Busy-wait rather than sleep: this runs once, during start-up, with
    // nothing else to do -- USB and the CAN/UART drivers do not exist yet, and
    // the only work that must proceed is the flash RPC, which is serviced from
    // the MBX1A interrupt and therefore preempts this loop. Diagnostics are
    // drained as we go so core1's boot log is not stuck behind the wait.
    const std::uint64_t deadline =
        timer::Timer::timestamp64_quarter_us()
        + static_cast<std::uint64_t>(timeout_ms) * (timer::Timer::kTimerFrequencyHz / 1000U);

    while (g_channel->flash.eeprom_ready.load(std::memory_order::acquire) == 0) {
        if (timer::Timer::timestamp64_quarter_us() >= deadline)
            return false;
        poll_diagnostics();
    }
    return true;
}

# if defined(libhcs_APP_DIAG_OVER_USB) && libhcs_APP_DIAG_OVER_USB

// Relay core1's diagnostic text to the host as UART0 uplink frames.
//
// core1's only log path is the SHARE_RAM ring, and the only sink for it is the
// board console -- a UART on the FT2232 debug header. Boards that are wired for
// EtherCAT and USB alone (which is every board this has been brought up on) have
// nothing attached there, so core1's boot log, including everything the emulated
// EEPROM has to say about a first-boot SII rewrite, is simply unreachable.
//
// Opt-in, because DataId::kUart0 is a real data port on this board (UART1) and a
// host using it must not find log text mixed into its byte stream.
void relay_diagnostics_over_usb() {
    // Checked BEFORE draining. The interesting part of core1's log is written
    // during boot, seconds before a host can possibly have a session up --
    // draining first and discarding for want of a carrier would throw away
    // exactly the lines worth reading. Left in the ring, they are still there
    // when the host arrives; if none ever does, the ring fills and core1's
    // writes are dropped at the producer, which is its documented contract.
    if (!link::uplink_enabled())
        return;

    // One frame per pass, bounded, so a chatty core1 cannot monopolize the loop
    // or the batch pool. Text is forwarded verbatim; the host prints it.
    char text[120];
    const std::size_t size = ecat::xcore_diag_drain(g_channel->diag, text, sizeof(text));
    if (size == 0)
        return;

    (void)link::uplink_serializer().write_uart(
        static_cast<core::protocol::FieldId>(data::DataId::kUart0),
        {
            .uart_data = {reinterpret_cast<const std::byte*>(text), size},
              .idle_delimited = true
    });
}

# endif

void poll_diagnostics() {
# if defined(libhcs_APP_DIAG_OVER_USB) && libhcs_APP_DIAG_OVER_USB
    relay_diagnostics_over_usb();
    return;
# else
    // Never block the main loop for a log byte. console_send_byte() busy-waits
    // on THR-empty (~87 us per byte at 115200 baud), so draining a burst of
    // core1 boot messages through it would stall tud_task() and the CAN/UART
    // transmit pumps for milliseconds at a time -- USB enumeration would jitter
    // exactly while core1 has something interesting to say. Instead, peek at the
    // TX FIFO and stop as soon as it is full; the bytes stay in the ring and the
    // next pass continues. Diagnostics must never cost data-path latency.
    for (std::size_t i = 0; i < kDiagBytesPerPass; ++i) {
        char byte = 0;
        if (ecat::xcore_diag_drain(g_channel->diag, &byte, 1) == 0)
            return;
        if (!board_console_try_send_byte(static_cast<std::uint8_t>(byte))) {
            // FIFO full: this byte is already out of the ring, so emit it with
            // the blocking call rather than dropping it, then yield the pass.
            console_send_byte(static_cast<std::uint8_t>(byte));
            return;
        }
    }
# endif
}

} // namespace libhcs::firmware::xcore

#endif
