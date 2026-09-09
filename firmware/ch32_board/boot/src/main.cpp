// V3F boot core for ch32_board.
//
// CH32H417 is a dual-core part where the V3F core is the BOOT core: it comes out
// of reset at flash 0x0, brings up the clock tree, then wakes the V5F core at
// 0x00010000 (Core_V5F_StartAddr) which runs the libhcs forwarding fast path.
//
// V3F is the OFFLOAD core. The forwarding hot path (USB SS <-> CAN/UART) lives
// entirely on V5F with zero cross-core hops, so its average latency stays at the
// single-core optimum. V3F takes over the non-forwarding, would-be-preempting
// work (diagnostics now; IMU sampling / attitude solve / CAN-FD preprocessing
// later) and publishes it to a shared-SRAM mailbox that V5F drains off the hot
// path. The net effect is lower tail latency / jitter on the forwarding path.
//
// The wake / handshake sequence mirrors EVT CPU/IPC's V3F main.

extern "C" {
#include "ch32h417.h"
#include "ch32h417_usbss_device.h"
#include "debug.h"
#include "usb_desc.h"
// Provided by boot/User/system_ch32h417.c (the V3F variant). Enables HSE and
// configures the system + USB SS PLLs -- the V5F app's system variant does NOT
// do this, so the boot core must, or the USB SS PLL never locks (HSE stays off).
void SystemInit(void);
}

#include "firmware/ch32_board/boot/src/flash/validation.hpp"
#include "firmware/ch32_board/boot/src/mailbox.hpp"
#include "firmware/ch32_board/boot/src/usb/dfu.hpp"
#include "firmware/ch32_board/boot/src/utility/boot_mailbox.hpp"

namespace libhcs::firmware::boot {
namespace {

inline constexpr uintptr_t kBootDiagAddr = 0x20170000u;

// Why the bootloader IS the V3F image rather than a separate partition: on this
// part V3F comes out of reset first, owns the clock tree, and decides whether to
// wake V5F at all. That is exactly a bootloader's job, so folding the two avoids
// a third image and a second clock bring-up. "Launching the app" therefore means
// NVIC_WakeUp_V5F() rather than a jump -- V3F stays resident afterwards and runs
// the offload loop.
enum class BootDecision : uint8_t {
    kLaunchApp,
    kEnterDfu,
};

BootDecision decide_boot_mode() {
    const uint32_t request = utility::boot_mailbox().consume_request();

    // The application asked to be reflashed (host sent DFU_DETACH); honour it
    // even if the image currently in flash is perfectly valid.
    if (request == utility::BootMailbox::kMailboxRequestEnterDfu)
        return BootDecision::kEnterDfu;

    // A freshly manifested image gets one unconditional launch: its record was
    // just written and hash-checked, so re-verifying here would only burn time.
    if (request == utility::BootMailbox::kMailboxRequestBootAppOnce)
        return BootDecision::kLaunchApp;

    // Cold boot: refuse to wake V5F into a torn or absent image. This is the
    // check that makes an interrupted download safe -- the metadata record is
    // committed only after the whole image is programmed and read back.
    return flash::app_image_is_valid() ? BootDecision::kLaunchApp : BootDecision::kEnterDfu;
}

// DFU mode: enumerate as a DFU-mode device and serve downloads from the EP0
// class-request handler in usb/dfu_transport.cpp until the host is done. V5F is
// never woken here -- an image that failed validation must not run, and one
// being replaced must not be executing while its slot is erased.
[[noreturn]] void run_dfu_mode() {
    usb::dfu.reset();

    // Same USB bring-up order the application core uses (app/src/app.cpp), and
    // for the same reason: USBSS_Device_Init() calls NVIC_EnableIRQ, so every
    // consumer the interrupt path touches has to exist first. Here that is just
    // usb::dfu, reset above. Chip is the silicon revision the vendor stack gates
    // a LINK_INT_CTRL bit on; leaving it 0 mis-configures the link on rev >= 3.
    Chip = ((DBGMCU_GetCHIPID() >> 4) & 0x0F);
    libhcs_usb_init_descriptors();
    USB_Timer_Init();
    USBSS_Device_Init(ENABLE);

    while (true) {
        // usb::dfu is driven entirely from the USBSS interrupt and holds no
        // volatile members, so without a barrier the compiler may hoist these
        // loads out of the loop -- at -O2 that turns the poll into a permanent
        // spin. The barrier costs nothing here; the flash work happens in the ISR.
        __asm volatile("" ::: "memory");

        // Manifestation committed the metadata record and left a boot-app-once
        // request in the boot mailbox; DETACH is the host asking for the same
        // reset without a download. Either way the device is expected to drop
        // off the bus and come back, so tear the link down explicitly first --
        // resetting with the PHY live leaves the host waiting out its own
        // timeout instead of seeing a disconnect.
        if (!usb::dfu.manifestation_complete() && !usb::dfu.detach_requested())
            continue;

        // Let the in-flight control transfer finish its status stage before the
        // link goes away, otherwise the host reports the request as failed.
        Delay_Ms(10);
        USBSS_Device_Init(DISABLE);
        NVIC_SystemReset();
    }
}

void allocate_v5f_irqs() {
    NVIC_SetAllocateIRQ(USBHS_IRQn, Core_ID_V5F);
    NVIC_SetAllocateIRQ(USBSS_IRQn, Core_ID_V5F);
    NVIC_SetAllocateIRQ(USBSS_LINK_IRQn, Core_ID_V5F);
    NVIC_SetAllocateIRQ(TIM12_IRQn, Core_ID_V5F);
    NVIC_SetAllocateIRQ(CAN1_RX0_IRQn, Core_ID_V5F);
    NVIC_SetAllocateIRQ(CAN2_RX0_IRQn, Core_ID_V5F);
    NVIC_SetAllocateIRQ(USART1_IRQn, Core_ID_V5F);
    // uart2 is USART3, not USART2 -- see the routing table in
    // app/src/board_app.hpp. This list is keyed by PERIPHERAL, so it has to
    // follow whatever kUartPorts/kCanPorts actually select; allocating the wrong
    // vector leaves the driver's interrupt on this core and the port deaf.
    NVIC_SetAllocateIRQ(USART3_IRQn, Core_ID_V5F);
}

[[noreturn]] void boot_main() {
    // V3F is the boot core: it owns the clock tree. Enable HSE and bring up the
    // system / USB SS PLLs here so that when V5F is woken it finds HSE ready and
    // its USBSS_PLL_Init can lock (root cause of the earlier USBSS_PLL stall).
    SystemInit();

    // HSE/clock diagnostic snapshot, taken at runtime right after SystemInit
    // (which enables HSE + configures the PLLs). Read back via debugger at
    // 0x20170000: diag[0]=RCC->CTLR (HSEON=bit16, HSERDY=bit17, USBSS_PLLRDY=bit23),
    // diag[1]=RCC->CFGR0 (SWS = current SYSCLK source), diag[2]=RCC->PLLCFGR.
    // Runtime reads are reliable, unlike halted debugger writes to clock regs.
    {
        volatile uint32_t* diag = reinterpret_cast<volatile uint32_t*>(kBootDiagAddr);
        diag[0] = RCC->CTLR;
        diag[1] = RCC->CFGR0;
        diag[2] = RCC->PLLCFGR;
    }

    SystemAndCoreClockUpdate();
    {
        volatile uint32_t* diag = reinterpret_cast<volatile uint32_t*>(kBootDiagAddr);
        diag[3] = SystemClock;
        diag[4] = SystemCoreClock;
        diag[5] = HCLKClock;
    }
    Delay_Init();
    USART_Printf_Init(921600);
    // HSE clock diagnostic over UART (does not need a working debug module).
    // Read on a serial terminal @921600: RCC->CTLR bit16=HSEON, bit17=HSERDY.
    // If HSERDY(bit17)=1 the crystal started; if HSEON=1 but HSERDY=0 it did not.
    // SysClk far above HSI (e.g. ~100 MHz on V3F) also confirms HSE+PLL are live.
    printf(
        "V3F boot: SysClk=%u HCLK=%u RCC->CTLR=%08x RCC->CFGR0=%08x [HSEON=b16 HSERDY=b17]\r\n",
        static_cast<unsigned>(SystemCoreClock), static_cast<unsigned>(HCLKClock),
        static_cast<unsigned>(RCC->CTLR), static_cast<unsigned>(RCC->CFGR0));

    const BootDecision decision = decide_boot_mode();
    if (decision == BootDecision::kEnterDfu) {
        printf("V3F: entering DFU (app image invalid or detach requested)\r\n");
        run_dfu_mode();
    }

    // Construct the cross-core mailbox BEFORE releasing V5F, so the V5F consumer
    // can never observe an uninitialised RingBuffer.
    init_shared();
    allocate_v5f_irqs();

    // Wake the V5F forwarding core (PC := Core_V5F_StartAddr == 0x00010000).
    {
        volatile uint32_t* diag = reinterpret_cast<volatile uint32_t*>(kBootDiagAddr);
        diag[6] = Core_V5F_StartAddr;
        diag[7] = NVIC->WAKEIP[1];
        diag[8] = NVIC->SCTLR;
        NVIC->SCTLR &= ~(1u << 5);
        __asm volatile("fence rw, rw" ::: "memory");
    }
    NVIC_WakeUp_V5F(Core_V5F_StartAddr);
    __asm volatile("fence rw, rw" ::: "memory");
    {
        volatile uint32_t* diag = reinterpret_cast<volatile uint32_t*>(kBootDiagAddr);
        diag[9] = NVIC->WAKEIP[1];
        diag[10] = NVIC->SCTLR;
    }
    printf("V3F released V5F @ %08x\r\n", static_cast<unsigned>(Core_V5F_StartAddr));

    // Startup barrier: wait until V5F has finished bringing up its forwarding
    // path (it sets v5f_ready). Keeps mailbox producers/consumers ordered.
    while (shared().v5f_ready == 0)
        ;
    printf("V5F ready; V3F entering offload loop\r\n");

    // Offload loop. Publishes a heartbeat/diagnostic record roughly every 1 ms.
    // This is a placeholder for real offload work and, more importantly, it
    // exercises the cross-core data path end to end so it can be validated on
    // hardware. emplace_back drops on a full ring (non-hot-path, best effort).
    for (uint32_t sequence = 0;; ++sequence) {
        // The application core cannot reset the chip itself (see the comment on
        // SharedBlock::reset_request), so a host-requested DFU detach arrives
        // here: V5F has already parked itself and written the boot mailbox, and
        // the reset below lands in decide_boot_mode() one boot later.
        if (shared().reset_request == kResetRequestMagic) {
            printf("V3F: reset requested by V5F (DFU detach)\r\n");
            NVIC_SystemReset();
        }

        shared().telemetry.emplace_back(
            TelemetryRecord{
                .timestamp = 0,
                .sequence = sequence,
                .value = 0,
            });
        Delay_Ms(1);
    }
}

} // namespace
} // namespace libhcs::firmware::boot

int main() { libhcs::firmware::boot::boot_main(); }
