cmake_minimum_required(VERSION 3.21...3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

# Fold the selected board into the output name so the produced ELF/BIN/DFU make
# the target board obvious and the wrong image cannot be flashed by mistake.
set(APP_NAME "hpm_board_app_${BOARD}")

# PCAN 人格(-Dlibhcs_APP_PCAN_PERSONALITY=ON, 见 app/src/pcan/ 与
# firmware/hpm_board/PCAN_PROTOCOL.md): 独立镜像, 整板枚举为 PEAK PCAN-USB FD
# (单 CAN)/PCAN-USB Pro FD(双 CAN)。libhcs / DMTool / CDC / UART 的源被过滤,
# 产物名带 _pcan 后缀。默认构建(未定义)与开启前完全一致 -- 该保证由"改动前后
# 默认镜像 .dfu 的 sha256 相同"在评审中核对。
option(libhcs_APP_PCAN_PERSONALITY
       "Enumerate the whole board as a PEAK PCAN-USB FD adapter (separate image)" OFF)
if(libhcs_APP_PCAN_PERSONALITY)
    set(APP_NAME "hpm_board_app_${BOARD}_pcan")
endif()

if(NOT libhcs_HPM_BOARD_SUPERBUILDING)
    message(FATAL_ERROR
        "Do not configure firmware/hpm_board/app directly.\n"
        "Use the hpm_board superbuild instead:\n")
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/../../common/cmake/libhcs_firmware.cmake")

set(HPM_BUILD_TYPE "flash_uf2" CACHE STRING "HPM build type")

# HPM5321 owns a board-specific linker script because its normal application
# layout places selected libhcs forwarding sections in ILM. Set the override
# before find_package(hpm-sdk); the SDK resolves its linker script while the
# package is loaded, before the board CMakeLists can select one.
if(BOARD STREQUAL "hpm5321")
    set(CUSTOM_GCC_LINKER_FILE
        "${BOARD_SEARCH_PATH}/${BOARD}/linker/app_flash_uf2.ld")
endif()

# On HPM6E8Y, optionally move the CAN forwarding leaf calls from FLASH XIP to
# ILM. Our own glue is already in .fast via ATTR_PLACE_AT, but the SDK's MCAN
# driver -- called on every frame -- is not, so the hot path still takes XIP
# fetches.
#
# USB quantizes the round trip to the 125 us microframe, so a microsecond
# saved here rarely drops a whole quantum. Evidence: HOST_TUNING.md 4.2.
#
# OFF keeps the stock SDK script, which is the configuration every measurement
# before 2026-08-01 was taken on.
option(libhcs_APP_ILM_HOT_PATH "Run the HPM6E8Y CAN driver from ILM" OFF)
if(libhcs_APP_ILM_HOT_PATH AND NOT BOARD STREQUAL "hpm5321")
    set(CUSTOM_GCC_LINKER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/hpm6e80_flash_uf2_ilm_hot.ld")
endif()

# CAN forwarding telemetry (src/diag/can_diag.*): ISR entry counts, MCAN status
# registers and the PLIC pending/enable arrays, sampled from the main loop and
# shipped out as UART0 uplink frames. Used to diagnose the high-load forwarding
# stall of ecat/CORE_SWAP_MIGRATION.md section 6 on a board with no JTAG or
# serial console attached. OFF keeps the CAN hot path free of the counters.
option(libhcs_APP_CAN_DIAG "Emit CAN forwarding telemetry on the UART0 uplink" OFF)

# The shared cross-board time base (src/sync/timebase.*, src/sync/sof.*) is in
# every image and is not a build option: the host asks for it in its EP0
# manifest (kManifestFlagTimeSync), and only then does the board arm the
# device controller's SOF interrupt -- 8000 interrupts per second that a board
# whose host did not ask never pays. See SOF_TIMEBASE.md.

# Cross-board skew measurement by hardware pulse exchange (src/sync/pulse.*).
# Uses GPTMR0 channel 1's compare output and capture input, which on this board
# share pads with UART0 -- so UART0 DOES NOT WORK in a build with this on. That
# is the whole reason it is a separate option and defaults OFF.
option(libhcs_APP_PULSE_TEST "Measure cross-board skew with GPTMR pulses" OFF)

# USB SOF probe (src/sync/sof_probe.*): enables the device controller's
# SOF-received interrupt and records the delta between consecutive FRINDEX
# readings, plus the ISR-to-ISR interval measured on the local machine timer.
# Step 1 of the cross-board time base -- it answers whether FRINDEX advances by
# exactly one per SOF, or whether the status flag leads the increment, before any
# of the timeline algorithm exists. OFF costs a production image nothing: the ISR
# hook compiles to nothing and the 8 kHz interrupt is never enabled.
option(libhcs_APP_SOF_DIAG "Emit USB SOF / FRINDEX validation telemetry on the UART0 uplink" OFF)

# EWDG hardware watchdog (src/watchdog/watchdog.hpp): resets the board when the
# main loop stops feeding it for 500 ms -- the hard backstop against any hang,
# USB IRQ spin included. OFF by default because a watchdog turns every hang into
# a silent reboot: debug builds must be able to see the hang instead. Turn it on
# for match images.
option(libhcs_APP_WATCHDOG "Reset the board when the main loop stops feeding the EWDG" OFF)

set(CONFIG_DMA_MGR 1)
set(CONFIG_USB_DEVICE 1)
set(libhcs_BUILDING_FIRMWARE_APP 1)

# Always use the SDK pinned by this project's bsp/hpm_sdk submodule, ignoring any
# external HPM_SDK_BASE in the environment, so the build is reproducible.
set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/../bsp/hpm_sdk")
find_package(hpm-sdk REQUIRED HINTS "$ENV{HPM_SDK_BASE}")

get_property(libhcs_USB_PID GLOBAL PROPERTY libhcs_USB_PID)
if("${libhcs_USB_PID}" STREQUAL "")
    message(FATAL_ERROR "libhcs_USB_PID must be set by the selected board")
endif()

# PID stamped into the .dfu container's suffix, which dfu-util checks against the
# connected device before writing. The hpm5321 image serves two PCBs and reports
# its PID at run time, so boards/hpm5321 sets the wildcard 0xFFFF instead. Vendor
# id and CRC are still verified.
get_property(libhcs_DFU_PRODUCT_ID GLOBAL PROPERTY libhcs_DFU_PRODUCT_ID)
if("${libhcs_DFU_PRODUCT_ID}" STREQUAL "")
    set(libhcs_DFU_PRODUCT_ID ${libhcs_USB_PID})
endif()

libhcs_setup_project_context(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

project(hpm_board_app VERSION 3 LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

sdk_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-unwind-tables>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-asynchronous-unwind-tables>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>)
if(libhcs_SOURCE_PATH_MAP_COMPILE_OPTIONS)
    sdk_compile_options(${libhcs_SOURCE_PATH_MAP_COMPILE_OPTIONS})
endif()

if(CMAKE_BUILD_TYPE STREQUAL "debug")
    sdk_compile_options(-Og)
endif()

if(libhcs_APP_CAN_DIAG)
    sdk_compile_definitions(-Dlibhcs_APP_CAN_DIAG=1)
endif()
if(libhcs_APP_PULSE_TEST)
    sdk_compile_definitions(-Dlibhcs_APP_PULSE_TEST=1)
endif()
if(libhcs_APP_SOF_DIAG)
    sdk_compile_definitions(-Dlibhcs_APP_SOF_DIAG=1)
endif()
if(libhcs_APP_WATCHDOG)
    sdk_compile_definitions(-Dlibhcs_APP_WATCHDOG=1)
endif()

# PCAN 人格的编译定义与不兼容项: 三个 UART0 遥测选项都借 UART0(PCAN 人格不编
# UART 模块)。
if(libhcs_APP_PCAN_PERSONALITY)
    foreach(diag_option IN ITEMS CAN_DIAG PULSE_TEST SOF_DIAG)
        if(libhcs_APP_${diag_option})
            message(FATAL_ERROR
                "libhcs_APP_${diag_option} uses the UART0 diag path and cannot be combined "
                "with libhcs_APP_PCAN_PERSONALITY")
        endif()
    endforeach()
    sdk_compile_definitions(-Dlibhcs_APP_PCAN_PERSONALITY=1)
endif()

sdk_compile_definitions(-Dlibhcs_PROJECT_VERSION_STRING="${libhcs_PROJECT_VERSION}")
sdk_compile_definitions(-DCFG_TUSB_MCU=OPT_MCU_HPM)
sdk_compile_definitions(-DUSB_HOST_MCU_CORE=HPM_CORE0)

sdk_inc("${CMAKE_CURRENT_SOURCE_DIR}/include")
sdk_app_inc("${libhcs_PROJECT_ROOT}/core/include")
sdk_app_inc("${libhcs_PROJECT_ROOT}")

include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/current_tinyusb.cmake")
# PCAN 人格: 只有 DFU runtime(端点全归 pcan 的应用类驱动); 默认构建三类齐上。
if(libhcs_APP_PCAN_PERSONALITY)
    set(libhcs_TINYUSB_CLASSES src/class/dfu/dfu_rt_device.c)
else()
    set(libhcs_TINYUSB_CLASSES
        src/class/vendor/vendor_device.c
        src/class/cdc/cdc_device.c
        src/class/dfu/dfu_rt_device.c)
endif()
libhcs_add_current_tinyusb(${libhcs_TINYUSB_CLASSES})

file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    "${libhcs_PROJECT_ROOT}/core/src/*.cpp"
    "${libhcs_PROJECT_ROOT}/core/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)
# 人格的源互斥: pcan 模块(接缝符号 usbd_app_driver_get_cb / 描述符回调 /
# tud_vendor_control_xfer_cb / hcs_usb0_isr / link::uplink_*)只在人格构建里
# 编译, 默认人格的接缝文件反之 -- 一个构建只能有一份定义。
# diag/can_diag.cpp 走 UART0 上行, 人格构建一并排除(latency.hpp 是纯头, 保留)。
if(libhcs_APP_PCAN_PERSONALITY)
    list(FILTER PROJECT_SOURCE EXCLUDE REGEX
        "/src/(dmtool|uart)/|/src/usb/(vendor|vendor_control|usb_descriptors)\\.cpp$|/src/link/uplink_usb\\.cpp$|/src/diag/can_diag\\.cpp$")
else()
    list(FILTER PROJECT_SOURCE EXCLUDE REGEX "/src/pcan/")
endif()
sdk_app_src(${PROJECT_SOURCE})


sdk_linker_global_symbols("_uf2_bl_length=0x20000")

# PCAN 人格不实例化 libhcs 的上行批量缓冲模板, hpm5321 链接脚本里那条按修饰名
# 点名的 ILM tripwire 在该构建里无符号可查(这版 binutils 对表达式里的未定义
# 符号直接报错)。把该修饰名 defsym 成 0: 断言平凡通过, 脚本保持两构建共用;
# 下方 memcpy/mcan 那条 tripwire 对两个人格都照常生效。
if(libhcs_APP_PCAN_PERSONALITY)
    sdk_linker_global_symbols(
        "_ZN6libhcs8firmware3usb19InterruptSafeBufferILj32ENS0_4link17UplinkAllocFailedEE8allocateEj=0"
    )
endif()

set_property(TARGET app PROPERTY CXX_STANDARD 23)
set_property(TARGET app PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET app PROPERTY CXX_EXTENSIONS OFF)

get_target_property(_hpm_incs ${HPM_SDK_LIB_ITF} INTERFACE_INCLUDE_DIRECTORIES)
if(_hpm_incs)
    set_target_properties(${HPM_SDK_LIB_ITF} PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_hpm_incs}")
endif()

libhcs_add_dfu_image(
    TARGET ${APP_ELF_NAME}
    INPUT_BINARY "${EXECUTABLE_OUTPUT_PATH}/${APP_BIN_NAME}"
    OUTPUT_DFU "${EXECUTABLE_OUTPUT_PATH}/${APP_NAME}.dfu"
    VENDOR_ID 0xA511
    PRODUCT_ID ${libhcs_DFU_PRODUCT_ID}
    DEVICE_ID 0x0300
    COMMENT "Generating ${APP_NAME}.dfu"
)
