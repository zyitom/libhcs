add_executable(c_board_app)

set(C_BOARD_APP_TINYUSB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../bsp/tinyusb")
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
