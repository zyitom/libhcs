add_executable(mc02_bootloader)

set(MC02_BOOTLOADER_TINYUSB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../c_board/bsp/tinyusb")
cmake_path(ABSOLUTE_PATH MC02_BOOTLOADER_TINYUSB_ROOT NORMALIZE)
set(MC02_BOOTLOADER_TINYUSB_COMMON_SOURCES
    "${MC02_BOOTLOADER_TINYUSB_ROOT}/src/tusb.c"
    "${MC02_BOOTLOADER_TINYUSB_ROOT}/src/common/tusb_fifo.c"
    "${MC02_BOOTLOADER_TINYUSB_ROOT}/src/device/usbd.c"
    "${MC02_BOOTLOADER_TINYUSB_ROOT}/src/portable/synopsys/dwc2/dwc2_common.c"
    "${MC02_BOOTLOADER_TINYUSB_ROOT}/src/portable/synopsys/dwc2/dcd_dwc2.c"
)

add_library(tinyusb_bootloader OBJECT)
target_sources(tinyusb_bootloader PRIVATE
    ${MC02_BOOTLOADER_TINYUSB_COMMON_SOURCES}
    "${MC02_BOOTLOADER_TINYUSB_ROOT}/src/class/dfu/dfu_device.c"
)
target_include_directories(tinyusb_bootloader SYSTEM PUBLIC
    "${MC02_BOOTLOADER_TINYUSB_ROOT}/src"
)
target_include_directories(tinyusb_bootloader PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_definitions(tinyusb_bootloader PUBLIC
    CFG_TUSB_MCU=OPT_MCU_STM32H7
)
target_link_libraries(tinyusb_bootloader PUBLIC
    stm32cubemx
)

file(GLOB_RECURSE MC02_BOOTLOADER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)
target_sources(mc02_bootloader PRIVATE
    ${MC02_BOOTLOADER_SOURCES}
)

target_include_directories(mc02_bootloader PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

mc02_apply_target_options(
    mc02_bootloader
    "${CMAKE_CURRENT_SOURCE_DIR}/../bsp/linker/STM32H723VGTx_BOOTLOADER.ld"
)
target_link_libraries(mc02_bootloader PRIVATE
    tinyusb_bootloader
)
