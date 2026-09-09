cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_HAL_DRIVER 
	STM32F407xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Inc
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Inc/Legacy
    ${libhcs_STM32_DEVICE_ROOT}/Include
    ${libhcs_CMSIS_CORE_ROOT}/Core/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/usb_otg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32f407xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32f4xx.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_can.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_rcc.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_rcc_ex.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_flash.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_flash_ex.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_flash_ramfunc.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_gpio.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_dma_ex.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_dma.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_pwr.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_pwr_ex.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_cortex.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_exti.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_spi.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_tim.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_tim_ex.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_uart.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_pcd.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_hal_pcd_ex.c
    ${libhcs_STM32_HAL_DRIVER_ROOT}/Src/stm32f4xx_ll_usb.c
)

# Create stm32cubemx object library
add_library(stm32cubemx OBJECT)
target_sources(stm32cubemx PRIVATE ${STM32_Drivers_Src})
target_sources(stm32cubemx PRIVATE ${MX_Application_Src})
target_include_directories(stm32cubemx PUBLIC ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx PUBLIC ${MX_Defines_Syms})
