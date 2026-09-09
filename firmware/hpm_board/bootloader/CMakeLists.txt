cmake_minimum_required(VERSION 3.21...3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

# Fold the selected board into the output name so the bootloader ELF/BIN makes
# the target board obvious and the wrong image cannot be flashed by mistake.
set(APP_NAME "hpm_board_bootloader_${BOARD}")

if(NOT libhcs_HPM_BOARD_SUPERBUILDING)
    message(FATAL_ERROR
        "Do not configure firmware/hpm_board/bootloader directly.\n"
        "Use the hpm_board superbuild instead:\n")
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/../../common/cmake/libhcs_firmware.cmake")

set(HPM_BUILD_TYPE "flash_xip" CACHE STRING "HPM build type")
set(BOARD_BOOTLOADER_MODE "auto" CACHE STRING "Bootloader mode (auto / manual)")
set_property(CACHE BOARD_BOOTLOADER_MODE PROPERTY STRINGS auto manual)

set(CONFIG_USB_DEVICE 1)

# Always use the SDK pinned by this project's bsp/hpm_sdk submodule, ignoring any
# external HPM_SDK_BASE in the environment, so the build is reproducible.
set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/../bsp/hpm_sdk")
find_package(hpm-sdk REQUIRED HINTS "$ENV{HPM_SDK_BASE}")

libhcs_setup_project_context(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

project(hpm_board_bootloader VERSION 3 LANGUAGES C CXX ASM)

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

if(NOT "${BOARD_BOOTLOADER_MODE}" STREQUAL "auto"
    AND NOT "${BOARD_BOOTLOADER_MODE}" STREQUAL "manual")
    message(FATAL_ERROR "BOARD_BOOTLOADER_MODE must be auto or manual")
endif()

if("${BOARD_BOOTLOADER_MODE}" STREQUAL "auto")
    set(bootloader_mode_auto 1)
else()
    set(bootloader_mode_auto 0)
endif()

sdk_compile_definitions(-Dlibhcs_PROJECT_VERSION_STRING="${libhcs_PROJECT_VERSION}")
sdk_compile_definitions(-DCFG_TUSB_MCU=OPT_MCU_HPM)
sdk_compile_definitions(-DUSB_HOST_MCU_CORE=HPM_CORE0)
sdk_compile_definitions(-Dlibhcs_BOOTLOADER_MODE_AUTO=${bootloader_mode_auto})

sdk_inc("${CMAKE_CURRENT_SOURCE_DIR}/include")
sdk_app_inc("${libhcs_PROJECT_ROOT}/core/include")
sdk_app_inc("${libhcs_PROJECT_ROOT}")

include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/current_tinyusb.cmake")
libhcs_add_current_tinyusb(
    src/class/dfu/dfu_device.c
)

file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)
sdk_app_src(${PROJECT_SOURCE})

sdk_linker_global_symbols("_flash_size=0x1F000")

set_property(TARGET app PROPERTY CXX_STANDARD 23)
set_property(TARGET app PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET app PROPERTY CXX_EXTENSIONS OFF)

get_target_property(_hpm_incs ${HPM_SDK_LIB_ITF} INTERFACE_INCLUDE_DIRECTORIES)
if(_hpm_incs)
    set_target_properties(${HPM_SDK_LIB_ITF} PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_hpm_incs}")
endif()
