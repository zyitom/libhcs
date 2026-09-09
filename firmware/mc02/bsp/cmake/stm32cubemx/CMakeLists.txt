cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

set(MC02_BSP_ROOT   "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(MX_HAL_DRV      "${MC02_BSP_ROOT}/stm32h7xx-hal-driver")
set(MX_CUBEMX       "${MC02_BSP_ROOT}/cubemx")
set(MX_CMSIS_DEVICE "${MC02_BSP_ROOT}/cmsis-device-h7")
set(MX_CMSIS_CORE   "${MC02_BSP_ROOT}/../../c_board/bsp/cmsis-core")

set(MX_Defines_Syms
    USE_HAL_DRIVER
    STM32H723xx
    $<$<CONFIG:Debug>:DEBUG>
)

set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${MX_CUBEMX}/Core/Inc
    ${MX_HAL_DRV}/Inc
    ${MX_HAL_DRV}/Inc/Legacy
    ${MX_CMSIS_DEVICE}/Include
    ${MX_CMSIS_CORE}/Core/Include
)

set(MX_Application_Src
    ${MX_CUBEMX}/Core/Src/main.c
    ${MX_CUBEMX}/Core/Src/gpio.c
    # Battery-voltage ADC on PC4 (ADC1_INP4). Listed here rather than picked up
    # automatically because this file is hand-maintained: CubeMX writes its own
    # source list to bsp/cubemx/cmake/stm32cubemx/CMakeLists.txt, which nothing
    # in the build references. Enabling a new peripheral in the .ioc therefore
    # always needs a matching line here -- and leaving it out is not a silent
    # omission, it is a link error, because the generated stm32h7xx_it.c gains an
    # IRQ handler referencing the new file's DMA handle (hdma_adc1).
    ${MX_CUBEMX}/Core/Src/adc.c
    ${MX_CUBEMX}/Core/Src/fdcan.c
    ${MX_CUBEMX}/Core/Src/dma.c
    ${MX_CUBEMX}/Core/Src/bdma.c
    ${MX_CUBEMX}/Core/Src/spi.c
    ${MX_CUBEMX}/Core/Src/tim.c
    ${MX_CUBEMX}/Core/Src/usart.c
    ${MX_CUBEMX}/Core/Src/memorymap.c
    ${MX_CUBEMX}/Core/Src/stm32h7xx_it.c
    ${MX_CUBEMX}/Core/Src/stm32h7xx_hal_msp.c
    ${MX_CUBEMX}/Core/Src/sysmem.c
    ${MX_CUBEMX}/Core/Src/syscalls.c
    ${MX_CUBEMX}/Core/Startup/startup_stm32h723vgtx.s
)

set(STM32_Drivers_Src
    ${MX_CUBEMX}/Core/Src/system_stm32h7xx.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_adc.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_adc_ex.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_fdcan.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_rcc.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_rcc_ex.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_flash.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_flash_ex.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_gpio.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_dma.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_dma_ex.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_mdma.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_pwr.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_pwr_ex.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_cortex.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_exti.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_spi.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_spi_ex.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_tim.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_tim_ex.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_uart.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_uart_ex.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_pcd.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_pcd_ex.c
    ${MX_HAL_DRV}/Src/stm32h7xx_ll_usb.c
    ${MX_HAL_DRV}/Src/stm32h7xx_hal_hsem.c
)

add_library(stm32cubemx OBJECT)
target_sources(stm32cubemx PRIVATE ${STM32_Drivers_Src})
target_sources(stm32cubemx PRIVATE ${MX_Application_Src})

# The CubeMX-generated main.c defines int main(), which collides at link time
# with the real entry point in app/src/app.cpp and bootloader/src/main.cpp --
# both replace the CubeMX main() and call SystemClock_Config()/MPU_Config()/
# MX_*_Init() directly. Rename the generated main to a dead symbol at compile
# time so the startup's "bl main" binds to the real (app/bootloader) main. This
# is the regeneration-proof equivalent of c_board hand-deleting its main():
# CubeMX Generate only rewrites bsp/cubemx/, never this hand-maintained file, so
# the fix survives (a hand-delete gets clobbered on every Generate, as it was
# here by commit 0582395). SystemClock_Config/PeriphCommonClock_Config/
# MPU_Config/Error_Handler in the same file are separate functions, untouched.
set_source_files_properties(${MX_CUBEMX}/Core/Src/main.c
    PROPERTIES COMPILE_DEFINITIONS "main=cubemx_unused_main")

# Vendor/generated headers (HAL, CMSIS, CubeMX Core/Inc) are included as SYSTEM
# so clang (and thus the --warnings-as-errors=* clang-tidy gate) suppresses
# diagnostics originating inside them. This lets the vendored H7 CMSIS stay
# byte-for-byte upstream instead of carrying a local patch: ST's H7
# system_stm32h7xx.h declares ExitRun0Mode with __attribute__((optimize("Os"))),
# a GCC-only attribute clang flags as -Wignored-attributes. Handling it here,
# rather than editing the vendor source, keeps HAL/CMSIS clean enough to track
# as upstream git submodules (like c_board's F4 HAL, which has no such code).
target_include_directories(stm32cubemx SYSTEM PUBLIC ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx PUBLIC ${MX_Defines_Syms})
