# Bare-metal RISC-V toolchain for the WCH CH32H417 (Qingke V5F core).
#
# The Qingke V5F implements RV32IMAFC plus WCH's proprietary "xw" compressed
# extension. Formal builds use the independent MounRiver GCC15 toolchain but do
# not enable xw, keeping the generated firmware on the standard RISC-V ISA.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv32)

# CMake re-runs this file inside every try_compile sub-project, where -D cache
# entries from the outer configure are NOT visible. Without this the compiler
# probe would fail (or silently fall back to PATH) whenever the toolchain is
# selected with -DWCH_TOOLCHAIN_PATH= rather than the environment variable.
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES WCH_TOOLCHAIN_PATH WCH_TOOLCHAIN_PREFIX)

# Locate the cross toolchain. Priority:
#   1. -DWCH_TOOLCHAIN_PATH=<dir with bin/>
#   2. environment WCH_TOOLCHAIN_PATH
#   3. riscv32-wch-elf-* on PATH
if(NOT WCH_TOOLCHAIN_PATH AND DEFINED ENV{WCH_TOOLCHAIN_PATH})
    set(WCH_TOOLCHAIN_PATH "$ENV{WCH_TOOLCHAIN_PATH}")
endif()

# MounRiver's "RISC-V Embedded GCC15" uses riscv32-wch-elf-. Keep an explicit
# override for repackaged installations, but never fall back to the HPM
# riscv32-unknown-elf toolchain used by hpm_board.
if(NOT WCH_TOOLCHAIN_PREFIX AND DEFINED ENV{WCH_TOOLCHAIN_PREFIX})
    set(WCH_TOOLCHAIN_PREFIX "$ENV{WCH_TOOLCHAIN_PREFIX}")
endif()

if(WCH_TOOLCHAIN_PREFIX)
    set(_tc_prefix_candidates "${WCH_TOOLCHAIN_PREFIX}")
else()
    set(_tc_prefix_candidates "riscv32-wch-elf-")
endif()

set(_tc_prefix "")
if(WCH_TOOLCHAIN_PATH)
    set(_tc_bin "${WCH_TOOLCHAIN_PATH}/bin")
    foreach(_candidate IN LISTS _tc_prefix_candidates)
        if(EXISTS "${_tc_bin}/${_candidate}gcc")
            set(_tc_prefix "${_candidate}")
            break()
        endif()
    endforeach()
    if(NOT _tc_prefix)
        message(
            FATAL_ERROR
            "No RISC-V GCC found under ${_tc_bin}. Tried prefixes: "
            "${_tc_prefix_candidates}. Point WCH_TOOLCHAIN_PATH at the directory "
            "that CONTAINS bin/ (for MounRiver that is one of the per-version "
            "directories under MRS_Toolchain_*/Toolchain/, not Toolchain/ itself), "
            "or set WCH_TOOLCHAIN_PREFIX explicitly."
        )
    endif()
    set(CMAKE_C_COMPILER   "${_tc_bin}/${_tc_prefix}gcc")
    set(CMAKE_CXX_COMPILER "${_tc_bin}/${_tc_prefix}g++")
    set(CMAKE_ASM_COMPILER "${_tc_bin}/${_tc_prefix}gcc")
    set(CMAKE_OBJCOPY      "${_tc_bin}/${_tc_prefix}objcopy" CACHE FILEPATH "")
    set(CMAKE_SIZE         "${_tc_bin}/${_tc_prefix}size"    CACHE FILEPATH "")
else()
    foreach(_candidate IN LISTS _tc_prefix_candidates)
        find_program(_wch_tc_gcc "${_candidate}gcc")
        if(_wch_tc_gcc)
            set(_tc_prefix "${_candidate}")
            break()
        endif()
    endforeach()
    if(NOT _tc_prefix)
        message(
            FATAL_ERROR
            "No RISC-V GCC on PATH. Tried: ${_tc_prefix_candidates}. Set "
            "WCH_TOOLCHAIN_PATH to a toolchain root containing bin/."
        )
    endif()
    set(CMAKE_C_COMPILER   "${_tc_prefix}gcc")
    set(CMAKE_CXX_COMPILER "${_tc_prefix}g++")
    set(CMAKE_ASM_COMPILER "${_tc_prefix}gcc")
    set(CMAKE_OBJCOPY      "${_tc_prefix}objcopy" CACHE FILEPATH "")
    set(CMAKE_SIZE         "${_tc_prefix}size"    CACHE FILEPATH "")
endif()

message(STATUS "ch32_board RISC-V toolchain: ${CMAKE_C_COMPILER}")

# Soft-float on purpose (was rv32imafc / ilp32f). The firmware uses no floating
# point, but GCC's __attribute__((interrupt)) with a hard-float ABI saves the
# caller-saved FP registers in software on every ISR. Qingke HPE (intsyscr=0x0F)
# already pushes GPRs; the extra FP stores are pure forwarding latency. Without
# F, __riscv_flen is undefined and that prologue vanishes. Revert if on-board
# float math ever becomes necessary. Keep this string in sync with
# CH32_ARCH_FLAGS in CMakeLists.txt.
set(WCH_ARCH_FLAGS "-march=rv32imac_zicsr_zifencei -mabi=ilp32 -mcmodel=medany")

set(CMAKE_C_FLAGS_INIT   "${WCH_ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${WCH_ARCH_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${WCH_ARCH_FLAGS} -x assembler-with-cpp")

# CMake's Debug default is -g with no -O. The V5F app slot is 128 KB; an
# unoptimized Debug image already sat at ~96% FLASH. -Og keeps it debuggable
# and is what the other boards use. Release keeps -g: DWARF is not PT_LOAD, so
# the flashed bytes stay identical to -g0 while crash dumps remain symbolizable.
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-O3 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

# Do not try to link a full hosted test program during compiler detection.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
