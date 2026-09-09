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


set(MC02_BSP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/bsp")

add_subdirectory("${MC02_BSP_ROOT}/cmake/stm32cubemx" SYSTEM)


function(mc02_apply_target_options target linker_file)
    target_include_directories(${target} PRIVATE
        ${libhcs_PROJECT_ROOT}/core/include
        ${libhcs_PROJECT_ROOT}
    )

    target_compile_options(${target} PRIVATE
        -mcpu=cortex-m7
        -mfpu=fpv5-d16
        -mfloat-abi=hard
        -fvisibility=hidden
        $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wpedantic>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-unwind-tables>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-asynchronous-unwind-tables>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    )

    target_compile_definitions(${target} PRIVATE
        libhcs_PROJECT_VERSION_STRING="${libhcs_PROJECT_VERSION}"
        $<$<CONFIG:Release>:NDEBUG>
    )

    target_link_libraries(${target} PRIVATE
        stdc++
        m
        stm32cubemx
    )
    cmake_path(IS_ABSOLUTE linker_file linker_is_abs)
    if(linker_is_abs)
        set(linker_path "${linker_file}")
    else()
        set(linker_path "${MC02_BSP_HAL_ROOT}/${linker_file}")
    endif()

    target_link_options(${target} PRIVATE
        -mcpu=cortex-m7
        -mfpu=fpv5-d16
        -mfloat-abi=hard
        "-Wl,-Map=$<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.map"
        "-T${linker_path}"
    )
endfunction()


add_subdirectory(app)
add_subdirectory(bootloader)
