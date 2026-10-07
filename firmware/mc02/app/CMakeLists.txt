add_executable(mc02_app)

set(MC02_APP_TINYUSB_ROOT "${libhcs_PROJECT_ROOT}/firmware/common/bsp/tinyusb")
cmake_path(ABSOLUTE_PATH MC02_APP_TINYUSB_ROOT NORMALIZE)
set(MC02_APP_SEGGER_ROOT "${libhcs_PROJECT_ROOT}/firmware/common/bsp/SEGGER")
set(MC02_APP_TINYUSB_COMMON_SOURCES
    "${MC02_APP_TINYUSB_ROOT}/src/tusb.c"
    "${MC02_APP_TINYUSB_ROOT}/src/common/tusb_fifo.c"
    "${MC02_APP_TINYUSB_ROOT}/src/device/usbd.c"
    "${MC02_APP_TINYUSB_ROOT}/src/portable/synopsys/dwc2/dwc2_common.c"
    "${MC02_APP_TINYUSB_ROOT}/src/portable/synopsys/dwc2/dcd_dwc2.c"
)

add_library(tinyusb_app OBJECT)
target_sources(tinyusb_app PRIVATE
    ${MC02_APP_TINYUSB_COMMON_SOURCES}
    "${MC02_APP_TINYUSB_ROOT}/src/class/vendor/vendor_device.c"
    "${MC02_APP_TINYUSB_ROOT}/src/class/dfu/dfu_rt_device.c"
)
target_include_directories(tinyusb_app SYSTEM PUBLIC
    "${MC02_APP_TINYUSB_ROOT}/src"
)
target_include_directories(tinyusb_app PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_definitions(tinyusb_app PUBLIC
    CFG_TUSB_MCU=OPT_MCU_STM32H7
)
target_link_libraries(tinyusb_app PUBLIC
    stm32cubemx
)

# stm32cubemx OBJECT sources (main.c, stm32h7xx_it.c) need TinyUSB and SEGGER headers
target_include_directories(stm32cubemx SYSTEM PRIVATE
    "${MC02_APP_TINYUSB_ROOT}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_definitions(stm32cubemx PRIVATE
    CFG_TUSB_MCU=OPT_MCU_STM32H7
)
target_include_directories(stm32cubemx SYSTEM PRIVATE
    "${MC02_APP_SEGGER_ROOT}/RTT"
    "${MC02_APP_SEGGER_ROOT}/Config"
    "${libhcs_PROJECT_ROOT}/firmware/c_board"
)

file(GLOB_RECURSE MC02_APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
    "${libhcs_PROJECT_ROOT}/core/src/*.cpp"
    "${libhcs_PROJECT_ROOT}/core/src/*.c"
    "${MC02_APP_SEGGER_ROOT}/RTT/*.c"
    "${MC02_APP_SEGGER_ROOT}/RTT/*.S"
)
target_sources(mc02_app PRIVATE
    ${MC02_APP_SOURCES}
)

target_include_directories(mc02_app SYSTEM PRIVATE
    "${MC02_APP_SEGGER_ROOT}/RTT"
    "${MC02_APP_SEGGER_ROOT}/Config"
)

target_link_libraries(mc02_app PRIVATE
    tinyusb_app
)

# Enclosure UART2 / UART3 (USART2 / USART3 RS-485) are ALWAYS compiled in.
#
# There used to be a libhcs_APP_RS485_ENABLE switch here. It was removed
# [2026-09-30, UART_EP0_MIGRATION.md phase 7]: the ports exist on every board of
# this type and are the enclosure's UART2/UART3, so a build that leaves them out
# only produces an image whose EP0 index space has two holes -- a host cannot
# tell "this board has no UART2" from "this image compiled UART2 out", and the
# port index has to keep meaning the same physical port across images.
#
# The cost is ~1.8 KB of D2 SRAM resident regardless of whether the ports are
# used (the Lazy<T> objects and their DMA rings are placed at link time, not at
# init()), plus two NDTR reads per main-loop pass. Accepted deliberately.
#
# Bytes one bulk OUT transfer requests. The endpoint stays a 64-byte Full-Speed
# bulk endpoint either way; this only asks DWC2 to accept N packets back to back
# before the device has to re-arm. 64 is the old behaviour exactly. See the
# comment in app/include/tusb_config.h.
#
# 1024 measured on two boards [2026-08-12]: bulk OUT payload throughput at
# 256-byte records, 8 per transfer, against this value --
#
#     64 -> 652 KB/s   128 -> 708   256 -> 856   512 -> 971
#   1024 -> 1034 KB/s  2048 -> 1034 (saturated)
#
# so 1024 is the knee, +59% over the old 64, and 85% of the 1216 KB/s the
# Full-Speed bulk schedule allows. Costs 960 bytes of AXI SRAM for the endpoint
# buffer. Small single-record packets are unaffected either way (~22600/s at
# every size): those are short packets, so the transfer completes on arrival and
# never waits to fill. Round-trip latency measured unchanged (rs485_cross_test
# ping-pong p50 3298 us at 1024 vs 3324 us at 64).
set(libhcs_APP_USB_RX_XFER_SIZE
    "1024"
    CACHE STRING "Bytes requested per bulk OUT transfer (multiple of 64)")
target_compile_definitions(
    tinyusb_app PUBLIC libhcs_APP_USB_RX_XFER_SIZE=${libhcs_APP_USB_RX_XFER_SIZE})
target_compile_definitions(
    mc02_app PRIVATE libhcs_APP_USB_RX_XFER_SIZE=${libhcs_APP_USB_RX_XFER_SIZE})

# There is deliberately no switch for the DWC2 controller's internal DMA. It was
# built and measured on hardware (firmware/mc02/AGENTS.md, 2026-08-28): it costs
# 3.0% of the small-packet rate at two operating points 35% apart, and buys
# nothing anywhere, because what it removes is interrupt CPU time the board has
# in surplus while what it adds -- the core moving the packet to SRAM before it
# raises XFRC -- lands on the path that actually gates throughput, the race to
# re-arm the OUT endpoint before the host's next token. Slave mode is the only
# supported configuration; do not re-add the option without re-measuring.

# Same switch and same default as hpm_board's: OFF, because it adds counters to
# the CAN forwarding hot path and occupies DataId::kUart0 on the uplink.
# Main-loop section timing on the kUart0 uplink, using the DWT cycle counter.
# OFF by default: it costs about one counter read per section boundary, and it
# shares DataId::kUart0 with libhcs_APP_CAN_DIAG, so the two are mutually
# exclusive.
# RS-485 ports are always on: they are the enclosure's UART2 / UART3, not an
# overlay on a spare DataId. Diagnostic builds emit on kUart0, which is not a
# silkscreen port on this board, so they no longer collide with these two.
# Needs the USART2/USART3 DMA entries in the .ioc to have been regenerated.

# There is deliberately no switch for draining the RS-485 receive rings from
# their IDLE ISR instead of from the main loop. It was built and measured on two
# cross-wired boards (firmware/mc02/AGENTS.md, 2026-08-31): it does shorten the
# main loop by 277 cycles per pass, but that time is surplus, the USB packet rate
# does not improve once the loop-period phase artefact is controlled for, and a
# 200-byte RS-485 round trip gets 26% slower because publishing only on IDLE
# gives up the chunked streaming the main-loop drain provides. Do not re-add the
# option without re-measuring.

# Distribution of the gap between consecutive bulk OUT completions, on the
# kUart0 uplink. OFF by default: it costs a DWT read plus a bucket search per
# received packet, and it shares DataId::kUart0 with the other diagnostics.
option(libhcs_APP_USB_RX_HIST "Emit bulk OUT inter-arrival histogram on the kUart0 uplink" OFF)
if(libhcs_APP_USB_RX_HIST)
    target_compile_definitions(mc02_app PRIVATE libhcs_APP_USB_RX_HIST=1)
endif()

option(libhcs_APP_LOOP_PROFILE "Emit main-loop section timing on the kUart0 uplink" OFF)
if(libhcs_APP_LOOP_PROFILE)
    target_compile_definitions(mc02_app PRIVATE libhcs_APP_LOOP_PROFILE=1)
endif()

# Measurement instrument for "is the main loop on the critical path". Busy-waits
# this many DWT cycles per loop pass (550 per microsecond at 550 MHz), so
# throughput can be plotted against loop period. 0 compiles the ballast out.
# Never set in a shipped build.
set(libhcs_APP_LOOP_BALLAST_CYCLES
    "0"
    CACHE STRING "Busy-wait cycles added to each main-loop pass (0 = disabled)")
if(NOT libhcs_APP_LOOP_BALLAST_CYCLES STREQUAL "0")
    target_compile_definitions(
        mc02_app PRIVATE libhcs_APP_LOOP_BALLAST_CYCLES=${libhcs_APP_LOOP_BALLAST_CYCLES})
endif()

# Bring-up aid, never shipped: exposes writable tone/loudness knobs and
# read-only battery mirrors for a debugger's watch window. Costs one tick read
# per main-loop pass when on, nothing when off. See src/diag/knobs.hpp.
option(libhcs_APP_DEBUG_KNOBS "Expose debugger-writable tuning knobs" OFF)
if(libhcs_APP_DEBUG_KNOBS)
    target_compile_definitions(mc02_app PRIVATE libhcs_APP_DEBUG_KNOBS=1)
endif()

# Shared cross-board USB-SOF time base (src/sync/). In every image and not a
# build option: the host asks for it in its EP0 manifest (kManifestFlagTimeSync)
# and only then does the board unmask the SOF interrupt (1 kHz).
#
# The --wrap is how the SOF hook reaches the interrupt vector. OTG_HS_IRQHandler
# lives in bsp/cubemx/Core/Src/stm32h7xx_it.c, which is generated output this
# repo does not edit, and all it does is call TinyUSB's dcd_int_handler(). The
# linker reroutes that call to __wrap_dcd_int_handler() in src/sync/sof.cpp,
# which timestamps and then chains to __real_dcd_int_handler(). Nothing
# generated is touched; with the time base off the hook costs one load and a
# branch per USB interrupt.
target_link_options(mc02_app PRIVATE "LINKER:--wrap=dcd_int_handler")

option(libhcs_APP_CAN_DIAG "Emit CAN telemetry on the kUart0 uplink" OFF)
if(libhcs_APP_CAN_DIAG)
    target_compile_definitions(mc02_app PRIVATE libhcs_APP_CAN_DIAG=1)
endif()

mc02_apply_target_options(
    mc02_app
    "${CMAKE_CURRENT_SOURCE_DIR}/../bsp/linker/STM32H723VGTx_APP.ld"
)

libhcs_add_dfu_image(
    TARGET mc02_app
    INPUT_ELF $<TARGET_FILE:mc02_app>
    OUTPUT_BINARY $<TARGET_FILE_DIR:mc02_app>/mc02_app.bin
    OUTPUT_DFU $<TARGET_FILE_DIR:mc02_app>/mc02_app.dfu
    VENDOR_ID 0xA511
    PRODUCT_ID 0x0723
    DEVICE_ID 0x0300
    COMMENT "Generating mc02_app.dfu"
)

# IWDG hardware watchdog (src/watchdog/watchdog.hpp): resets the board when the
# main loop stops feeding it for 500 ms -- the hard backstop against any hang.
# OFF by default because a watchdog turns every hang into a silent reboot:
# debug builds must be able to see the hang instead. IWDG cannot be stopped
# once started, hence a compile-time gate rather than a runtime one.
option(libhcs_APP_WATCHDOG "Reset the board when the main loop stops feeding the IWDG" OFF)
if(libhcs_APP_WATCHDOG)
    target_compile_definitions(mc02_app PRIVATE libhcs_APP_WATCHDOG=1)
endif()
