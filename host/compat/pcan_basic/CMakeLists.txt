cmake_minimum_required(VERSION 3.21)
project(libhcs_pcan_basic LANGUAGES C)

if(NOT WIN32)
    message(FATAL_ERROR "This compatibility DLL requires a Windows toolchain")
endif()

add_library(PCANBasic SHARED pcan_basic.c pcan_basic.def)
set_target_properties(PCANBasic PROPERTIES
    PREFIX ""
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF)
target_link_libraries(PCANBasic PRIVATE winusb setupapi)
if(MINGW)
    target_compile_options(PCANBasic PRIVATE -Wall -Wextra -Werror)
    target_link_options(PCANBasic PRIVATE -Wl,--kill-at -Wl,--enable-stdcall-fixup)
endif()
