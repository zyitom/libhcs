/* Linker script for STM32H723VGTx — Bootloader
 * Flash layout:
 *   Sector 0 (0x08000000, 128 KB): Bootloader
 *   Sector 1 (0x08020000, 128 KB): Metadata
 *   Sector 2-7 (0x08040000, 768 KB): App
 */

ENTRY(Reset_Handler)

MEMORY
{
    DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K - 64
    MAILBOX  (rw) : ORIGIN = 0x2001FFC0, LENGTH = 64
    AXI_SRAM (xrw): ORIGIN = 0x24000000, LENGTH = 128K
    FLASH    (rx) : ORIGIN = 0x08000000, LENGTH = 128K
}

_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
_Min_Heap_Size  = 0x400;
_Min_Stack_Size = 0x800;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

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
        . = ALIGN(4);
        _edata = .;
    } >DTCMRAM AT> FLASH

    .tbss (NOLOAD) : ALIGN(4)
    {
        _sbss = .;
        __bss_start__ = _sbss;
        *(.tbss .tbss.*)
        . = ALIGN(4);
    } >DTCMRAM

    .bss (NOLOAD) : ALIGN(4)
    {
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >DTCMRAM

    .flash_sector_cache (NOLOAD) :
    {
        . = ALIGN(32);
        KEEP(*(.flash_sector_cache))
        KEEP(*(.flash_sector_cache.*))
        . = ALIGN(32);
    } >AXI_SRAM

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
    } >DTCMRAM

    /DISCARD/ :
    {
        libc.a:*(*)
        libm.a:*(*)
        libgcc.a:*(*)
    }
}
