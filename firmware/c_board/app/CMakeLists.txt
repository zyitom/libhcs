add_executable(c_board_app)

set(C_BOARD_APP_TINYUSB_ROOT "${libhcs_PROJECT_ROOT}/firmware/common/bsp/tinyusb")
cmake_path(ABSOLUTE_PATH C_BOARD_APP_TINYUSB_ROOT NORMALIZE)
set(C_BOARD_APP_SEGGER_ROOT "${libhcs_PROJECT_ROOT}/firmware/common/bsp/SEGGER")
set(C_BOARD_APP_TINYUSB_COMMON_SOURCES
    "${C_BOARD_APP_TINYUSB_ROOT}/src/tusb.c"
    "${C_BOARD_APP_TINYUSB_ROOT}/src/common/tusb_fifo.c"
    "${C_BOARD_APP_TINYUSB_ROOT}/src/device/usbd.c"
    "${C_BOARD_APP_TINYUSB_ROOT}/src/portable/synopsys/dwc2/dwc2_common.c"
    "${C_BOARD_APP_TINYUSB_ROOT}/src/portable/synopsys/dwc2/dcd_dwc2.c"
)

add_library(tinyusb_app OBJECT)
target_sources(tinyusb_app PRIVATE
    ${C_BOARD_APP_TINYUSB_COMMON_SOURCES}
    "${C_BOARD_APP_TINYUSB_ROOT}/src/class/vendor/vendor_device.c"
    "${C_BOARD_APP_TINYUSB_ROOT}/src/class/dfu/dfu_rt_device.c"
)
target_include_directories(tinyusb_app SYSTEM PUBLIC
    "${C_BOARD_APP_TINYUSB_ROOT}/src"
)
target_include_directories(tinyusb_app PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_definitions(tinyusb_app PUBLIC
    CFG_TUSB_MCU=OPT_MCU_STM32F4
)
target_link_libraries(tinyusb_app PUBLIC
    stm32cubemx
)

file(GLOB_RECURSE C_BOARD_APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
    "${libhcs_PROJECT_ROOT}/core/src/*.cpp"
    "${libhcs_PROJECT_ROOT}/core/src/*.c"
    "${C_BOARD_APP_SEGGER_ROOT}/RTT/*.c"
    "${C_BOARD_APP_SEGGER_ROOT}/RTT/*.S"
)
target_sources(c_board_app PRIVATE
    ${C_BOARD_APP_SOURCES}
)

target_include_directories(c_board_app SYSTEM PRIVATE
    "${C_BOARD_APP_SEGGER_ROOT}/RTT"
    "${C_BOARD_APP_SEGGER_ROOT}/Config"
)

target_link_libraries(c_board_app PRIVATE
    tinyusb_app
)

c_board_apply_target_options(c_board_app "STM32F407XX_APP.ld")

# Shared cross-board USB-SOF time base (src/sync/). In every image and not a
# build option: the host asks for it in its EP0 manifest (kManifestFlagTimeSync)
# and only then does the board unmask the SOF interrupt (1 kHz).
#
# The --wrap is how the SOF hook reaches the interrupt vector. OTG_FS_IRQHandler
# lives in bsp/cubemx/Core/Src/stm32f4xx_it.c, which is generated output this
# repo does not edit, and its USER CODE block calls TinyUSB's tusb_int_handler(),
# which chains to dcd_int_handler(). The linker reroutes that call to
# __wrap_dcd_int_handler() in src/sync/sof.cpp, which timestamps and then chains
# to __real_dcd_int_handler(). Nothing generated is touched; with the time base
# off the hook costs one load and a branch per USB interrupt.
target_link_options(c_board_app PRIVATE "LINKER:--wrap=dcd_int_handler")

libhcs_add_dfu_image(
    TARGET c_board_app
    INPUT_ELF $<TARGET_FILE:c_board_app>
    OUTPUT_BINARY $<TARGET_FILE_DIR:c_board_app>/c_board_app.bin
    OUTPUT_DFU $<TARGET_FILE_DIR:c_board_app>/c_board_app.dfu
    VENDOR_ID 0xA511
    PRODUCT_ID 0xF407
    DEVICE_ID 0x0300
    COMMENT "Generating c_board_app.dfu"
)

# IWDG hardware watchdog (src/watchdog/watchdog.hpp): resets the board when the
# main loop stops feeding it for 500 ms -- the hard backstop against any hang.
# OFF by default because a watchdog turns every hang into a silent reboot:
# debug builds must be able to see the hang instead. IWDG cannot be stopped
# once started, hence a compile-time gate rather than a runtime one.
option(libhcs_APP_WATCHDOG "Reset the board when the main loop stops feeding the IWDG" OFF)
if(libhcs_APP_WATCHDOG)
    target_compile_definitions(c_board_app PRIVATE libhcs_APP_WATCHDOG=1)
endif()
