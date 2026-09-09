cmake_minimum_required(VERSION 3.21...3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

project(hpm_board_superbuild LANGUAGES NONE)

include("${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/libhcs_firmware.cmake")

# hpm5321 builds one image serving both HPM5321 PCBs -- single-CAN and dual-CAN --
# selecting how many CAN ports to bring up, and its LED tables, at run time from
# OTP word 25
# (common/board_identity.hpp).
set(BOARD "hpm5321" CACHE STRING "Target board (hpm5321)")
set_property(CACHE BOARD PROPERTY STRINGS hpm5321)
set(BOARD_SEARCH_PATH "${CMAKE_CURRENT_SOURCE_DIR}/boards" CACHE PATH "Board search path")

if(BOARD STREQUAL "hpm5321_dual_can")
    message(FATAL_ERROR
        "hpm5321_dual_can was merged into hpm5321; build with -DBOARD=hpm5321")
endif()
# Bitmanip is enabled by the HPM SoC CMake via the granular _zba_zbb_zbc_zbs
# extensions, so the umbrella "b" here is redundant. Keeping it makes the arch
# string "rv32imafdcb_..._zba_zbb_zbc_zbs", which recent binutils reject ("cannot
# find default versions of the ISA extension `b'"). Drop the redundant "b".
#
# Soft-float on purpose (was rv32imafdc / ilp32d): the firmware and core use no
# floating point anywhere, yet with a hard-float ABI the SDK's preemptive ISR
# wrapper saves/restores 20 caller-saved FP registers as doubles on EVERY
# interrupt entry/exit (SAVE_FPU_CONTEXT in hpm_interrupt.h) -- pure forwarding
# latency spent on nothing. Without F/D, __riscv_flen is undefined and the FPU
# context code vanishes. Revert if on-board float math ever becomes necessary.
set(RV_ARCH "rv32imac" CACHE STRING "RISC-V arch")
set(RV_ABI "ilp32" CACHE STRING "RISC-V ABI")
set(BOARD_BOOTLOADER_MODE "auto" CACHE STRING "Bootloader mode (auto / manual)")
set_property(CACHE BOARD_BOOTLOADER_MODE PROPERTY STRINGS auto manual)

# Forwarded to the app sub-build as libhcs_APP_CAN_DIAG; see app/CMakeLists.txt.
option(libhcs_CAN_DIAG "Emit CAN forwarding telemetry on the UART0 uplink" OFF)

# Forwarded to the app sub-build as libhcs_APP_TIME_SYNC; see app/CMakeLists.txt.
option(libhcs_TIME_SYNC "Run the shared USB-SOF time base" OFF)

# Forwarded to the app sub-build as libhcs_APP_PULSE_TEST; see app/CMakeLists.txt.
option(libhcs_PULSE_TEST "Measure cross-board skew with GPTMR pulses (takes over UART0)" OFF)

# Forwarded to the app sub-build as libhcs_APP_SOF_DIAG; see app/CMakeLists.txt.
option(libhcs_SOF_DIAG "Emit USB SOF / FRINDEX validation telemetry on the UART0 uplink" OFF)

# Forwarded to the app sub-build as libhcs_APP_USB_FULL_SPEED. Enumerates the
# device at FULL speed instead of HIGH -- a test vehicle, not a product setting:
# it costs bulk throughput (64-byte packets) and coarsens the SOF time base from
# 125 us microframes to 1 ms frames. See SOF_TIMEBASE.md.
option(libhcs_USB_FULL_SPEED "Enumerate USB at full speed (test only)" OFF)

# Forwarded to the app sub-build as libhcs_APP_ILM_HOT_PATH. HPM5321 always
# uses its board-owned linker layout; this switch applies to HPM6E8Y.
option(libhcs_ILM_HOT_PATH "Run the HPM6E8Y CAN driver from ILM" OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "debug")
endif()

libhcs_setup_project_context(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

function(add_hpm_board_subbuild target_name source_subdir)
    set(source_dir "${CMAKE_CURRENT_SOURCE_DIR}/${source_subdir}")
    set(binary_dir "${CMAKE_CURRENT_BINARY_DIR}/${source_subdir}")
    set(compile_commands_file "${binary_dir}/compile_commands.json")

    set(configure_args
        -G "${CMAKE_GENERATOR}"
        -S "${source_dir}"
        -B "${binary_dir}"
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -Dlibhcs_HPM_BOARD_SUPERBUILDING=ON
        -UBOARD_SEARCH_DIR
        -USDK_BOARD_DIR
        -DBOARD=${BOARD}
        -DBOARD_SEARCH_PATH=${BOARD_SEARCH_PATH}
        -DRV_ARCH=${RV_ARCH}
        -DRV_ABI=${RV_ABI}
        -DHOST_DEBUGGER=${HOST_DEBUGGER}
        -DSOURCE_PATH_MAP=${libhcs_SOURCE_PATH_MAP}
        -Dlibhcs_PROJECT_VERSION=${libhcs_PROJECT_VERSION}
    )
    if(CMAKE_MAKE_PROGRAM)
        list(APPEND configure_args -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM})
    endif()
    list(APPEND configure_args ${ARGN})

    add_custom_target(
        ${target_name}
        BYPRODUCTS "${compile_commands_file}"
        COMMAND ${CMAKE_COMMAND} ${configure_args}
        COMMAND ${CMAKE_COMMAND} --build "${binary_dir}"
        USES_TERMINAL
        VERBATIM
    )
endfunction()

add_hpm_board_subbuild(
    hpm_board_app
    app
    -Dlibhcs_APP_CAN_DIAG=${libhcs_CAN_DIAG}
    -Dlibhcs_APP_TIME_SYNC=${libhcs_TIME_SYNC}
    -Dlibhcs_APP_PULSE_TEST=${libhcs_PULSE_TEST}
    -Dlibhcs_APP_SOF_DIAG=${libhcs_SOF_DIAG}
    -Dlibhcs_APP_USB_FULL_SPEED=${libhcs_USB_FULL_SPEED}
    -Dlibhcs_APP_ILM_HOT_PATH=${libhcs_ILM_HOT_PATH}
)
add_hpm_board_subbuild(
    hpm_board_bootloader
    bootloader
    -DBOARD_BOOTLOADER_MODE=${BOARD_BOOTLOADER_MODE}
)

set(HPM_BOARD_APP_COMPILE_COMMANDS "${CMAKE_CURRENT_BINARY_DIR}/app/compile_commands.json")
set(HPM_BOARD_BOOTLOADER_COMPILE_COMMANDS "${CMAKE_CURRENT_BINARY_DIR}/bootloader/compile_commands.json")
set(HPM_BOARD_MERGED_COMPILE_COMMANDS "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json")

set(HPM_BOARD_COMPILE_COMMANDS_INPUTS
    "${HPM_BOARD_APP_COMPILE_COMMANDS}"
    "${HPM_BOARD_BOOTLOADER_COMPILE_COMMANDS}"
)
set(HPM_BOARD_COMPILE_COMMANDS_TARGETS hpm_board_app hpm_board_bootloader)

add_custom_command(
    OUTPUT "${HPM_BOARD_MERGED_COMPILE_COMMANDS}"
    COMMAND
    ${CMAKE_COMMAND}
    "-DOUTPUT_FILE=${HPM_BOARD_MERGED_COMPILE_COMMANDS}"
    "-DINPUT_FILES=${HPM_BOARD_COMPILE_COMMANDS_INPUTS}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/merge_compile_commands.cmake"
    DEPENDS
    ${HPM_BOARD_COMPILE_COMMANDS_TARGETS}
    ${HPM_BOARD_COMPILE_COMMANDS_INPUTS}
    COMMENT "Merging hpm_board compile commands"
    VERBATIM
)

add_custom_target(
    hpm_board_compile_commands
    DEPENDS "${HPM_BOARD_MERGED_COMPILE_COMMANDS}"
)

add_custom_target(hpm_board ALL DEPENDS hpm_board_compile_commands)
