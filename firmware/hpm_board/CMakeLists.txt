cmake_minimum_required(VERSION 3.21...3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

project(hpm_board_superbuild LANGUAGES NONE)

include("${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/libhcs_firmware.cmake")

# hpm5321 构建单一镜像同时服务两块 HPM5321 PCB(单 CAN 与双 CAN), 运行时按
# OTP word 25 选择初始化多少路 CAN 及其 LED 表(common/board_identity.hpp)。
set(BOARD "hpm5321" CACHE STRING "Target board (hpm5321)")
set_property(CACHE BOARD PROPERTY STRINGS hpm5321)
set(BOARD_SEARCH_PATH "${CMAKE_CURRENT_SOURCE_DIR}/boards" CACHE PATH "Board search path")

if(BOARD STREQUAL "hpm5321_dual_can")
    message(FATAL_ERROR
        "hpm5321_dual_can was merged into hpm5321; build with -DBOARD=hpm5321")
endif()
# Bitmanip 已由 HPM SoC CMake 经细粒度的 _zba_zbb_zbc_zbs 扩展开启, 这里的伞扩展
# "b"是冗余的; 保留它会使 arch 串变成 "rv32imafdcb_..._zba_zbb_zbc_zbs", 新版
# binutils 直接拒绝("cannot find default versions of the ISA extension `b'")。
# 故去掉冗余的 "b"。
#
# 刻意软浮点(原为 rv32imafdc / ilp32d): 固件与 core 全程不用浮点, 而硬浮点 ABI 下
# SDK 的抢占式 ISR 包装器在每次中断进入/退出都按 double 保存/恢复 20 个
# caller-saved FP 寄存器(hpm_interrupt.h 的 SAVE_FPU_CONTEXT) -- 纯白付的转发
# 延迟。没有 F/D 时 __riscv_flen 未定义, FPU 上下文代码随之消失。板上将来若真需要
# 浮点运算再改回。
set(RV_ARCH "rv32imac" CACHE STRING "RISC-V arch")
set(RV_ABI "ilp32" CACHE STRING "RISC-V ABI")
set(BOARD_BOOTLOADER_MODE "auto" CACHE STRING "Bootloader mode (auto / manual)")
set_property(CACHE BOARD_BOOTLOADER_MODE PROPERTY STRINGS auto manual)

# 透传给 app 子构建为 libhcs_APP_CAN_DIAG; 见 app/CMakeLists.txt。
option(libhcs_CAN_DIAG "Emit CAN forwarding telemetry on the UART0 uplink" OFF)

# 透传给 app 子构建为 libhcs_APP_TIME_SYNC; 见 app/CMakeLists.txt。
option(libhcs_TIME_SYNC "Run the shared USB-SOF time base" OFF)

# 透传给 app 子构建为 libhcs_APP_PULSE_TEST; 见 app/CMakeLists.txt。
option(libhcs_PULSE_TEST "Measure cross-board skew with GPTMR pulses (takes over UART0)" OFF)

# 透传给 app 子构建为 libhcs_APP_SOF_DIAG; 见 app/CMakeLists.txt。
option(libhcs_SOF_DIAG "Emit USB SOF / FRINDEX validation telemetry on the UART0 uplink" OFF)

# 透传给 app 子构建为 libhcs_APP_USB_FULL_SPEED。以 FULL 速度而非 HIGH 枚举设备
# -- 是测试手段, 不是产品设置: 损失 bulk 吞吐(64 字节包), 并把 SOF 时基从 125 us
# 微帧粗化到 1 ms 帧。见 SOF_TIMEBASE.md。
option(libhcs_USB_FULL_SPEED "Enumerate USB at full speed (test only)" OFF)

# 透传给 app 子构建为 libhcs_APP_ILM_HOT_PATH。HPM5321 始终用板自有的链接布局;
# 该开关只作用于 HPM6E8Y。
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
