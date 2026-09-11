cmake_minimum_required(VERSION 3.21...3.26)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(libhcs-firmware LANGUAGES C CXX ASM)

include("${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/libhcs_firmware.cmake")
libhcs_setup_project_context(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

if(libhcs_SOURCE_PATH_MAP_COMPILE_OPTIONS)
    add_compile_options(${libhcs_SOURCE_PATH_MAP_COMPILE_OPTIONS})
endif()

set(CH32_BSP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/bsp")

# WCH standard peripheral library, a submodule pinned to the ch32h417-evt subset
# of openwch/ch32h417 (currently V1.5). Unmodified vendor code; our USBSS patches
# live in bsp/usb/ instead. See bsp/PROVENANCE.md.
set(CH32_WCH_ROOT "${CH32_BSP_ROOT}/ch32h417-evt/wch")
if(NOT EXISTS "${CH32_WCH_ROOT}/Core/core_riscv.c")
    message(FATAL_ERROR
        "WCH peripheral library missing at ${CH32_WCH_ROOT}. "
        "Run: git submodule update --init "
        "firmware/ch32_board/bsp/ch32h417-evt")
endif()

# CH32H417 is a dual-core part: the V3F core boots at flash 0x0 and wakes the
# V5F core at 0x10000. Both cores are the same Qingke family, so they share one
# arch flag set; they differ only in startup, linker script and the Core_*
# define. See boot/src/main.cpp for the wake/handshake sequence.
#
# Soft-float on purpose (was rv32imafc / ilp32f): the firmware and core use no
# floating point anywhere, yet GCC's __attribute__((interrupt)) with a hard-
# float ABI saves caller-saved FP registers on EVERY interrupt. Qingke HPE
# (intsyscr=0x0F in the vendor startup) already pushes GPRs; the FP stores are
# extra. Without F, __riscv_flen is undefined and that prologue vanishes. The
# vendor startup still writes mstatus FS=Dirty (0x6088); that is submodule code
# and unused FPU enable is harmless. Revert if on-board float math is needed.
# Keep in sync with WCH_ARCH_FLAGS in cmake/toolchain-wch-riscv.cmake.
set(CH32_ARCH_FLAGS
    -march=rv32imac_zicsr_zifencei
    -mabi=ilp32
    -mcmodel=medany
)

set(CH32_V5F_LINKER_SCRIPT "${CH32_WCH_ROOT}/Ld/V5F/Link_v5f.ld")
set(CH32_V3F_LINKER_SCRIPT "${CH32_WCH_ROOT}/Ld/V3F/Link_v3f.ld")

# ---- WCH standard peripheral library (vendored, treated as third-party) ------
file(GLOB CH32_WCH_PERIPH_SOURCES CONFIGURE_DEPENDS
    "${CH32_WCH_ROOT}/Peripheral/src/*.c"
)

# Board-support sources shared by both cores. NOTE: system_ch32h417.c is per-core,
# NOT shared -- the V3F boot core uses boot/User (with SystemInit/SetSysClock that
# enables HSE and configures the PLLs), while the V5F app core uses app/User (the
# woken-core variant that only reads the clock). Startup is per-core too.
set(CH32_WCH_COMMON_SOURCES
    ${CH32_WCH_PERIPH_SOURCES}
    "${CH32_WCH_ROOT}/Core/core_riscv.c"
    "${CH32_WCH_ROOT}/Debug/debug.c"
    "${CH32_BSP_ROOT}/syscalls.c"
)

set(CH32_WCH_INCLUDE_DIRS
    "${CH32_WCH_ROOT}/Core"
    "${CH32_WCH_ROOT}/Peripheral/inc"
    "${CH32_WCH_ROOT}/Debug"
    "${CMAKE_CURRENT_SOURCE_DIR}/app/User"
)

# ---- CH372 USBSS device stack (vendored, V5F only) ---------------------------
file(GLOB CH32_USB_SOURCES CONFIGURE_DEPENDS "${CH32_BSP_ROOT}/usb/*.c")

# USB SuperSpeed bulk benchmark build. OFF by default and it must stay that way
# for anything but a measurement run: it hands EP1 to app/src/usb/bench.cpp,
# which bypasses the deserializer, the session and the framing entirely, so the
# board no longer speaks the libhcs protocol. Use Release (Debug used to sit at
# ~96% of the 128 KB slot when it was unoptimized -g; it is -Og now, but the
# bench image still does not belong in a Debug slot):
#   cmake --preset release -S firmware/ch32_board -Dlibhcs_CH32_USB_BENCH=ON
option(libhcs_CH32_USB_BENCH "Build the V5F app in USB SS bulk benchmark mode" OFF)

# ---- V5F application ---------------------------------------------------------
# The libhcs C++ app (app/src) provides main() via app.cpp, replacing the WCH
# demo's app/User/main.c (which is intentionally excluded). The shared core
# protocol sources are compiled in alongside, as on the other boards.
file(GLOB_RECURSE CH32_APP_CXX_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/app/src/*.cpp"
    "${libhcs_PROJECT_ROOT}/core/src/*.cpp"
)

set(CH32_APP_SOURCES
    ${CH32_APP_CXX_SOURCES}
    "${CMAKE_CURRENT_SOURCE_DIR}/app/User/ch32h417_it.c"
)

# ---- Shared compile/link options for both cores ------------------------------
set(CH32_COMMON_COMPILE_OPTIONS
    ${CH32_ARCH_FLAGS}
    -ffunction-sections
    -fdata-sections
    # WCH vendor headers use the GNU 'asm' keyword; the repo builds strict C11
    # (no GNU extensions), so alias it to the standard-mode spelling instead of
    # enabling gnu11 for the whole target.
    $<$<COMPILE_LANGUAGE:C>:-Dasm=__asm__>
    $<$<COMPILE_LANGUAGE:C>:-Dtypeof=__typeof__>
    $<$<COMPILE_LANGUAGE:CXX>:-Dasm=__asm__>
    $<$<COMPILE_LANGUAGE:CXX>:-Dtypeof=__typeof__>
    # WCH declares its ISRs __attribute__((interrupt("WCH-Interrupt-fast"))).
    # Mainline GCC rejects that argument, drops the whole attribute and emits a
    # plain 'ret' epilogue: mstatus.MIE is never restored and the Qingke hardware
    # stack (intsyscr=0x0F) is never popped, so the FIRST vendored ISR to fire
    # (TIM12 / USBSS / USBSS_LINK / USBHS) permanently kills interrupts and the
    # USB link state machine stalls. Rewriting the attribute to the argument-less
    # form makes GCC emit 'mret'; the hardware push/pop just becomes redundant.
    # Only rewrites 'interrupt' when it is followed by '(' -- the argument-less
    # __attribute__((interrupt)) used by our own ISRs is left untouched.
    -Dinterrupt\(x\)=interrupt
    -fvisibility=hidden
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-unwind-tables>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-asynchronous-unwind-tables>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

# link options minus the -T script, which is per-core.
set(CH32_COMMON_LINK_OPTIONS
    ${CH32_ARCH_FLAGS}
    -nostartfiles
    --specs=nano.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

# ==============================================================================
# V5F forwarding core (fast path): USB SS <-> CAN/UART, kept single-core / zero
# cross-core on the hot path so its average latency stays optimal.
# ==============================================================================
add_executable(ch32_board_app
    ${CH32_WCH_COMMON_SOURCES}
    "${CMAKE_CURRENT_SOURCE_DIR}/app/User/system_ch32h417.c"
    "${CH32_WCH_ROOT}/Startup/startup_ch32h417_v5f.S"
    ${CH32_USB_SOURCES}
    ${CH32_APP_SOURCES}
)

target_include_directories(ch32_board_app PRIVATE
    ${CH32_WCH_INCLUDE_DIRS}
    "${CH32_BSP_ROOT}/usb"
    "${libhcs_PROJECT_ROOT}/core/include"
    "${libhcs_PROJECT_ROOT}"
)

target_compile_definitions(ch32_board_app PRIVATE
    Core_V5F
    Run_Core=Run_Core_V3FandV5F
    # 0 = never fall back from USB 3.0 SuperSpeed to USB 2.0 High Speed.
    # This board exists for the 5 Gbps link, and USB 2.0 costs the debug
    # interface: its D+/D- are PB8/PB9, which are also SWCLK/SWDIO, so a
    # fallback (which the stock stack triggers ~1.5 s after SS training fails,
    # e.g. whenever no cable is attached) silently kills SDI until a power
    # cycle. Set to 1 to restore stock behaviour and USB 2.0 host support.
    libhcs_USBSS_HS_FALLBACK=0
    # 0 = the application configuration descriptor: the libhcs bulk pair on
    # interface 0 plus a DFU run-time interface (answered by usb/dfu_runtime.cpp).
    # The bootloader target sets this to 1 and gets the DFU-mode descriptor
    # instead; both live in bsp/usb/usb_desc.c.
    libhcs_DFU_DEVICE=0
    libhcs_CH32_USB_BENCH=$<BOOL:${libhcs_CH32_USB_BENCH}>
    libhcs_PROJECT_VERSION_STRING="${libhcs_PROJECT_VERSION}"
    $<$<CONFIG:Release>:NDEBUG>
)

target_compile_options(ch32_board_app PRIVATE ${CH32_COMMON_COMPILE_OPTIONS})

target_link_options(ch32_board_app PRIVATE
    ${CH32_COMMON_LINK_OPTIONS}
    -T${CH32_V5F_LINKER_SCRIPT}
    "-Wl,-Map=$<TARGET_FILE_DIR:ch32_board_app>/$<TARGET_FILE_BASE_NAME:ch32_board_app>.map"
)

set_target_properties(ch32_board_app PROPERTIES OUTPUT_NAME "ch32_board_app" SUFFIX ".elf")

add_custom_command(TARGET ch32_board_app POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
        "$<TARGET_FILE:ch32_board_app>"
        "$<TARGET_FILE_DIR:ch32_board_app>/ch32_board_app.bin"
    COMMAND ${CMAKE_SIZE} "$<TARGET_FILE:ch32_board_app>"
    VERBATIM
)

# DFU payload for dfu-util. Unlike mc02/c_board this is NOT built with
# libhcs_add_dfu_image: that helper appends the ImageHash suffix those
# bootloaders parse, whereas this bootloader hashes the image it actually
# programmed and stores the digest in its own metadata record (see
# boot/src/flash/metadata.hpp). The DFU payload must therefore be byte-identical
# to the .bin the debugger path flashes, or the two would disagree about the
# image the metadata describes. The only thing added is the DFU suffix, which
# keeps dfu-util from pushing another board's image onto this one.
find_program(CH32_DFU_SUFFIX_EXECUTABLE dfu-suffix)
if(CH32_DFU_SUFFIX_EXECUTABLE)
    add_custom_command(TARGET ch32_board_app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
            "$<TARGET_FILE_DIR:ch32_board_app>/ch32_board_app.bin"
            "$<TARGET_FILE_DIR:ch32_board_app>/ch32_board_app.dfu"
        COMMAND "${CH32_DFU_SUFFIX_EXECUTABLE}"
            -v 0xA511 -p 0xD403
            -a "$<TARGET_FILE_DIR:ch32_board_app>/ch32_board_app.dfu"
        COMMENT "Generating ch32_board_app.dfu"
        VERBATIM
    )
else()
    message(STATUS "dfu-suffix not found: skipping ch32_board_app.dfu (install dfu-util)")
endif()

# ==============================================================================
# V3F boot / offload core: boots at 0x0, wakes V5F, then runs non-forwarding
# offload work published to a shared-SRAM mailbox (see boot/src).
# ==============================================================================
# The V3F image is both the boot core and the bootloader: it validates the V5F
# application image before waking it, and hosts the DFU download path. Sources
# under boot/src are header-only apart from main.cpp.
add_executable(ch32_board_boot
    ${CH32_WCH_COMMON_SOURCES}
    "${CMAKE_CURRENT_SOURCE_DIR}/boot/User/system_ch32h417.c"
    "${CH32_WCH_ROOT}/Startup/startup_ch32h417_v3f.S"
    "${CMAKE_CURRENT_SOURCE_DIR}/boot/src/main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/boot/src/usb/dfu_transport.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/boot/src/usb/descriptors.cpp"
    ${CH32_USB_SOURCES}
)

target_include_directories(ch32_board_boot PRIVATE
    ${CH32_WCH_INCLUDE_DIRS}
    "${CH32_BSP_ROOT}/usb"
    "${libhcs_PROJECT_ROOT}/core/include"
    "${libhcs_PROJECT_ROOT}"
)

target_compile_definitions(ch32_board_boot PRIVATE
    Core_V3F
    Run_Core=Run_Core_V3FandV5F
    # Console selection for the vendor Debug/debug.c printf. Its per-core default
    # would put the V3F console on USART1/PA9 -- which is the libhcs uart1 TX
    # pin on this board (board_app.hpp), so every boot would shift a 921600-baud
    # banner onto a forwarding port. USART8 is PB4/PB3 = J4.5/J4.6, the 1x06
    # header the schematic labels TX8/RX8, and is what the V5F application core
    # already defaults to -- so both cores share one physical console.
    DEBUG=DEBUG_UART8
    libhcs_USBSS_HS_FALLBACK=0
    # 1 = enumerate as a DFU-mode device (single interface, no endpoints, DFU
    # functional descriptor), which is what boot/src/usb/dfu_transport.cpp serves.
    libhcs_DFU_DEVICE=1
    libhcs_PROJECT_VERSION_STRING="${libhcs_PROJECT_VERSION}"
    $<$<CONFIG:Release>:NDEBUG>
)

target_compile_options(ch32_board_boot PRIVATE ${CH32_COMMON_COMPILE_OPTIONS})

target_link_options(ch32_board_boot PRIVATE
    ${CH32_COMMON_LINK_OPTIONS}
    -T${CH32_V3F_LINKER_SCRIPT}
    "-Wl,-Map=$<TARGET_FILE_DIR:ch32_board_boot>/$<TARGET_FILE_BASE_NAME:ch32_board_boot>.map"
)

set_target_properties(ch32_board_boot PROPERTIES OUTPUT_NAME "ch32_board_boot" SUFFIX ".elf")

add_custom_command(TARGET ch32_board_boot POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
        "$<TARGET_FILE:ch32_board_boot>"
        "$<TARGET_FILE_DIR:ch32_board_boot>/ch32_board_boot.bin"
    COMMAND ${CMAKE_SIZE} "$<TARGET_FILE:ch32_board_boot>"
    VERBATIM
)

# ==============================================================================
# Merged single-flash image: V3F @ 0x0 + V5F @ 0x10000 (see cmake/merge_hex.py).
# This is the "flash both cores at once" artifact, emitted in both forms:
#   .hex - absolute addresses, what openocd's `program` takes
#   .bin - flat image based at 0x00000000, gaps filled with 0xFF; what MounRiver
#          Studio's download step and WCH-LinkUtility take (same shape as the
#          Merge.bin those tools build themselves). Useful whenever openocd is
#          being uncooperative, since the vendor tooling flashes this reliably.
# ==============================================================================
set(CH32_MERGED_HEX "${CMAKE_CURRENT_BINARY_DIR}/ch32_board_merged.hex")
set(CH32_MERGED_BIN "${CMAKE_CURRENT_BINARY_DIR}/ch32_board_merged.bin")
set(CH32_METADATA_HEX "${CMAKE_CURRENT_BINARY_DIR}/ch32_board_metadata.hex")

# Must match boot/src/flash/layout.hpp -- kMetadataStartAddress and
# kAppMaxImageSize. The merged image carries the app-image metadata record the
# bootloader validates before waking V5F; the DFU path writes that record itself,
# but debugger flashing bypasses DFU, so the build has to supply it or the app
# never launches. See cmake/gen_metadata_hex.py.
set(CH32_METADATA_ADDRESS 0x00070000)
set(CH32_APP_MAX_IMAGE_SIZE 0x00060000)

add_custom_command(
    OUTPUT "${CH32_METADATA_HEX}"
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_metadata_hex.py"
        "$<TARGET_FILE_DIR:ch32_board_app>/ch32_board_app.bin"
        "--address=${CH32_METADATA_ADDRESS}"
        "--max-image-size=${CH32_APP_MAX_IMAGE_SIZE}"
        -o "${CH32_METADATA_HEX}"
    DEPENDS ch32_board_app "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_metadata_hex.py"
    COMMENT "Generating app-image metadata record @${CH32_METADATA_ADDRESS}"
    VERBATIM
)

# Both outputs come from one command: merge_hex.py parses the fragments into
# absolute addresses and emits the .hex and the flat .bin from the same map.
# Do NOT go back to concatenating the fragments as text and flattening the
# result with objcopy -- objcopy writes Extended Segment Address records (0x02)
# for these images while the metadata fragment uses Extended Linear Address
# records (0x04), and mixing the two addressing modes in one file put the V5F
# image on top of the V3F bootloader at 0x0 and the metadata record at 0x90000.
add_custom_command(
    OUTPUT "${CH32_MERGED_HEX}" "${CH32_MERGED_BIN}"
    COMMAND ${CMAKE_OBJCOPY} -O ihex
        "$<TARGET_FILE:ch32_board_boot>" "${CMAKE_CURRENT_BINARY_DIR}/ch32_board_boot.hex"
    COMMAND ${CMAKE_OBJCOPY} -O ihex
        "$<TARGET_FILE:ch32_board_app>" "${CMAKE_CURRENT_BINARY_DIR}/ch32_board_app_img.hex"
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/cmake/merge_hex.py"
        "${CMAKE_CURRENT_BINARY_DIR}/ch32_board_boot.hex"
        "${CMAKE_CURRENT_BINARY_DIR}/ch32_board_app_img.hex"
        "${CH32_METADATA_HEX}"
        --output-hex "${CH32_MERGED_HEX}"
        --output-bin "${CH32_MERGED_BIN}"
    DEPENDS ch32_board_boot ch32_board_app "${CH32_METADATA_HEX}"
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/merge_hex.py"
    COMMENT "Merging V3F@0x0 + V5F@0x10000 + metadata into ch32_board_merged.hex/.bin"
    VERBATIM
)

add_custom_target(ch32_board_merged ALL DEPENDS "${CH32_MERGED_HEX}" "${CH32_MERGED_BIN}")
