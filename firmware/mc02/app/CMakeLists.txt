add_executable(mc02_app)

set(MC02_APP_TINYUSB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../c_board/bsp/tinyusb")
cmake_path(ABSOLUTE_PATH MC02_APP_TINYUSB_ROOT NORMALIZE)
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
    "${CMAKE_CURRENT_SOURCE_DIR}/../../c_board/bsp/SEGGER/RTT"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../c_board/bsp/SEGGER/Config"
    "${libhcs_PROJECT_ROOT}/firmware/c_board"
)

file(GLOB_RECURSE MC02_APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
    "${libhcs_PROJECT_ROOT}/core/src/*.cpp"
    "${libhcs_PROJECT_ROOT}/core/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../c_board/bsp/SEGGER/RTT/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../c_board/bsp/SEGGER/RTT/*.S"
)
target_sources(mc02_app PRIVATE
    ${MC02_APP_SOURCES}
)

target_include_directories(mc02_app SYSTEM PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../c_board/bsp/SEGGER/RTT"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../c_board/bsp/SEGGER/Config"
)

target_link_libraries(mc02_app PRIVATE
    tinyusb_app
)

option(libhcs_APP_IMU_ENABLE "Enable BMI088 initialization and runtime sampling" ON)
if(libhcs_APP_IMU_ENABLE)
    target_compile_definitions(mc02_app PRIVATE libhcs_APP_IMU_ENABLE=1)
endif()

# Enclosure UART2 / UART3 (USART2 / USART3 RS-485). Same shape as the IMU
# switch: the ports exist on the board, so the default image brings them up.
# Set OFF when a build will never talk on those buses -- skips MX_USARTx_UART_Init,
# the two D2 SRAM objects (~1.8 KB), and the per-pass NDTR polls. DataIds stay
# kUart2 / kUart3 either way; an OFF image just rejects those fields.
option(libhcs_APP_RS485_ENABLE "Enable USART2/USART3 RS-485 (silkscreen UART2/UART3)" ON)
if(libhcs_APP_RS485_ENABLE)
    target_compile_definitions(mc02_app PRIVATE libhcs_APP_RS485_ENABLE=1)
endif()

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

# Shared cross-board USB-SOF time base (src/sync/). OFF by default: it adds a
# 1 kHz interrupt and, more to the point, the kTimeAnchor/kTimeStatus session
# types carry a payload after the session header -- a host that enables time sync
# against firmware without this loses downlink framing, so the two switches must
# be turned on together (host side: AdvancedOptions::set_enable_time_sync).
#
# The --wrap is how the SOF hook reaches the interrupt vector. OTG_HS_IRQHandler
# lives in bsp/cubemx/Core/Src/stm32h7xx_it.c, which is generated output this
# repo does not edit, and all it does is call TinyUSB's dcd_int_handler(). The
# linker reroutes that call to __wrap_dcd_int_handler() in src/sync/sof.cpp,
# which timestamps and then chains to __real_dcd_int_handler(). Nothing
# generated is touched and the default build does not carry the flag at all.
option(libhcs_APP_TIME_SYNC "Run the shared USB-SOF time base" OFF)
if(libhcs_APP_TIME_SYNC)
    target_compile_definitions(mc02_app PRIVATE libhcs_APP_TIME_SYNC=1)
    target_link_options(mc02_app PRIVATE "LINKER:--wrap=dcd_int_handler")
endif()

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
