/* Linker script for STM32H723VGTx — Application
 * Flash layout:
 *   Sector 0 (0x08000000, 128 KB): Bootloader
 *   Sector 1 (0x08020000, 128 KB): Metadata
 *   Sector 2-7 (0x08040000, 768 KB): App  ← this binary
 */

ENTRY(Reset_Handler)

MEMORY
{
    DTCMRAM  (xrw): ORIGIN = 0x20000000, LENGTH = 128K - 64
    MAILBOX   (rw): ORIGIN = 0x2001FFC0, LENGTH = 64
    RAM      (xrw): ORIGIN = 0x24000000, LENGTH = 128K
    RAM_D2   (xrw): ORIGIN = 0x30000000, LENGTH = 32K
    RAM_D3   (xrw): ORIGIN = 0x38000000, LENGTH = 16K
    ITCMRAM  (xrw): ORIGIN = 0x00000000, LENGTH = 64K
    FLASH     (rx): ORIGIN = 0x08040000, LENGTH = 768K
}

/* Main/ISR stack lives in zero-wait DTCM (CPU-private, never a DMA target), so
 * every call frame, local and ISR context push is single-cycle and free of AXI
 * bus-contention jitter -- the lowest-latency place for the stack on this part.
 * Grows down from just below the boot mailbox at the top of DTCM. */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
_Min_Heap_Size  = 0x2000;
_Min_Stack_Size = 0x2000;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* Latency-critical CAN forwarding code, executed from zero-wait ITCM so it
     * never depends on the I-cache or incurs FLASH-XIP fetch jitter. Loaded from
     * FLASH and copied to ITCM at boot by app.cpp (App constructor).
     *
     * Placed ahead of .text deliberately: the per-function input sections listed
     * below are named .text.<mangled>, so the *(.text*) wildcard would swallow
     * them -- the linker assigns each input section to the first output section
     * that matches it in script order.
     *
     * WARNING: nothing selected here may execute before that copy loop. The boot
     * path (Reset_Handler, SystemInit, main, utility::Lazy<App>::init) runs from
     * FLASH while ITCM still holds garbage, so a pattern matching e.g.
     * utility::Lazy would fault at startup. Keep the patterns narrow. */
    _siitcm = LOADADDR(.itcm);
    .itcm :
    {
        . = ALIGN(4);
        _sitcm = .;
        *(.itcm)
        *(.itcm*)

        /* Leaf calls of the CAN RX ISR. Left in FLASH they cost, per frame, a
         * long-branch veneer (ITCM at 0x0 is far outside the BL reach of FLASH
         * at 0x08040000) plus an I-cache-dependent fetch. Matched on the mangled
         * class prefix, so member signatures may change freely; a namespace or
         * class rename silently stops matching, and the only consequence is that
         * the veneers come back -- slower, still correct. Every name component is
         * length-prefixed: `libhcs` is 6 characters, so the prefix is _ZN6libhcs.
         * These rules said _ZN7libhcs until 2026-09-11 and matched nothing at all;
         * the ASSERT right after this section now makes that a link error. */
        *(.text._ZN6libhcs4core8protocol10Serializer*)         /* write_can + size helpers  */
        *(.text._ZN6libhcs4core7utility8Bitfield*)             /* protocol header bit packing */
        *(.text._ZN6libhcs8firmware3usb19InterruptSafeBuffer*) /* uplink batch allocation   */
        *(.text._ZN6libhcs8firmware7utility10RingBuffer*)      /* CAN transmit ring         */
        *(.text._ZZN6libhcs8firmware3can3Can*)                 /* out-of-line Can lambdas   */
        *(.text._ZN6libhcs8firmware3usb14get_serializerEv)

        /* Lazy accessors reached once per ISR entry. Full mangled names, since a
         * utility::Lazy wildcard would also catch Lazy<App>::init(), which main()
         * calls before the ITCM image exists. Template arguments are part of the
         * name, so these are best-effort: if a constructor signature changes the
         * match is lost and the call reverts to a veneer. */
        *(.text._ZN6libhcs8firmware7utility4LazyINS0_3can3CanEJP19FDCAN_HandleTypeDefmEE3getEv)
        *(.text._ZN6libhcs8firmware7utility4LazyINS0_3led3LedEJEEptEv)
        *(.text._ZN6libhcs8firmware7utility4LazyINS0_3usb6VendorEJEEptEv)

        . = ALIGN(4);
        _eitcm = .;
    } >ITCMRAM AT> FLASH

    /* Tripwire for the mangled-name rules above. InterruptSafeBuffer::allocate is
     * virtual, so every build type emits it out of line: if it is not in ITCM, the
     * rules no longer match what the compiler emits. */
    ASSERT(
        _ZN6libhcs8firmware3usb19InterruptSafeBuffer8allocateEj < ORIGIN(ITCMRAM) + LENGTH(ITCMRAM),
        ".itcm name rules matched nothing: InterruptSafeBuffer::allocate is outside ITCM. Recheck the mangled prefixes there against arm-none-eabi-nm.")

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab (READONLY) : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM (READONLY) :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .preinit_array (READONLY) :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH
    .init_array (READONLY) :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH
    .fini_array (READONLY) :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH

    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        *(.RamFunc)
        *(.RamFunc*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    /* Hot forwarding data (serializer + USB batch buffers + CAN objects) in
     * zero-wait DTCM, so the uplink ISR's data accesses never reach the AXI bus
     * and never contend with DMA -- completing the "CPU off the bus" hot path.
     * Loaded from FLASH and copied to DTCM at boot by app.cpp. These are CPU-only
     * structures; UART DMA buffers deliberately stay in non-cacheable AXI. */
    _sidtcm = LOADADDR(.dtcm);
    .dtcm :
    {
        . = ALIGN(32);
        _sdtcm = .;
        *(.dtcm)
        *(.dtcm*)
        . = ALIGN(32);
        _edtcm = .;
    } >DTCMRAM AT> FLASH

    /* The four UART port objects, each carrying its DMA rings.
     *
     * DMA1 and DMA2 are D2-domain masters. Reaching the AXI SRAM at 0x24000000
     * means crossing the D2-to-D1 interconnect for every byte and contending
     * with the M7's own AXI traffic; SRAM1/SRAM2 at 0x30000000 are on the D2 AHB
     * matrix the streams already sit on, so the transfers stay inside the
     * domain. (DTCM is not an option: DMA1/DMA2 cannot reach it at all -- that
     * is why .dtcm above is restricted to CPU-only structures.)
     *
     * The second effect is the one that shows up in the link: these objects
     * total ~18.7 KB and were in .data, which is what pushed AXI .data+.bss to
     * 26 KB against the 32 KB non-cacheable MPU window asserted at the end of
     * this file. Moving them out drops that to about 7 KB. FLASH cost is
     * unchanged -- the load image was already being paid for in .data.
     *
     * Loaded from FLASH and copied at boot by app.cpp, which also enables the
     * D2 SRAM clocks first and installs the MPU region that makes this range
     * non-cacheable. Nothing here may be touched before that copy runs. */
    _sid2sram = LOADADDR(.d2_sram);
    .d2_sram :
    {
        . = ALIGN(32);
        _sd2sram = .;
        *(.d2_sram)
        *(.d2_sram*)
        . = ALIGN(32);
        _ed2sram = .;
    } >RAM_D2 AT> FLASH

    .tbss (NOLOAD) : ALIGN(4)
    {
        _sbss = .;
        __bss_start__ = _sbss;
        *(.tbss .tbss.*)
        . = ALIGN(4);
    } >RAM

    .bss (NOLOAD) : ALIGN(4)
    {
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM

    .boot_mailbox (NOLOAD) :
    {
        . = ALIGN(4);
        KEEP(*(.boot_mailbox))
        KEEP(*(.boot_mailbox.*))
        . = ALIGN(4);
    } >MAILBOX

    ._user_heap_stack (NOLOAD) :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >RAM

    /* WS2812 LED frame buffer for SPI6 TX. SPI6 is a D3-domain peripheral whose
     * only DMA path is BDMA, which can reach D3 SRAM (0x38000000) but never the
     * AXI/D2 SRAMs -- so this buffer must live here. NOLOAD: the CPU fills and
     * cache-cleans it before each transfer (see led.hpp). */
    .d3_sram (NOLOAD) :
    {
        . = ALIGN(32);
        *(.d3_sram)
        *(.d3_sram*)
        . = ALIGN(32);
    } >RAM_D3

    /DISCARD/ :
    {
        libc.a:*(*)
        libm.a:*(*)
        libgcc.a:*(*)
    }
}

/* DMA buffers that remain in AXI SRAM (the USB endpoint buffers and the SPI
 * transfer buffers) live in .data or .bss, so they sit below _ebss. MPU region 0
 * -- Core/Src/main.c MPU_Config -- maps the first 32 KB of AXI SRAM as
 * non-cacheable, which is what keeps DMA writes visible to the CPU without
 * explicit cache maintenance.
 *
 * Nothing in the toolchain pins those buffers below that line, so a buffer that
 * drifted past 0x24008000 would have the D-cache serving the core stale copies
 * of DMA-written bytes -- silent corruption, and invisible to a two-board UART
 * test because both ends would have to corrupt identically to compare equal.
 * Turn it into a link error instead.
 *
 * The UART rings are no longer counted here: they moved to .d2_sram above. That
 * left roughly 25 KB of headroom under the 32 KB line rather than the 6 KB this
 * assert used to leave.
 *
 * ._user_heap_stack sits above _ebss and may cross the line freely: _estack is
 * in DTCM (see the top of this file) and no DMA touches the heap. */
ASSERT(
    _ebss <= ORIGIN(RAM) + 32K,
    "AXI SRAM .data+.bss exceeds the 32 KB non-cacheable MPU region 0; DMA buffers past 0x24008000 would lose cache coherency. Either shrink .data/.bss, enlarge MPU region 0 in CubeMX, or move the DMA buffers into .d2_sram.")

/* D2 SRAM is 32 KB and holds whole port objects, so overflowing it is easy to do
 * by adding a port or growing a ring. The linker would report a region overflow
 * anyway; this names the section so the message says which one. */
ASSERT(
    _ed2sram - _sd2sram <= LENGTH(RAM_D2),
    ".d2_sram exceeds the 32 KB D2 SRAM. Shrink the UART rings or move a port back to AXI SRAM.")
