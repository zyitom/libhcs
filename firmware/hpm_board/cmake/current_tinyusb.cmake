include_guard()

# Share the same TinyUSB checkout as c_board and mc02. HPM SDK still owns the
# SoC, PHY and register headers, but its bundled TinyUSB sources are not built.
set(libhcs_CURRENT_TINYUSB_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../c_board/bsp/tinyusb")
cmake_path(ABSOLUTE_PATH libhcs_CURRENT_TINYUSB_ROOT NORMALIZE)

if(NOT EXISTS "${libhcs_CURRENT_TINYUSB_ROOT}/src/tusb.c")
    message(FATAL_ERROR
        "TinyUSB submodule is missing at ${libhcs_CURRENT_TINYUSB_ROOT}.\n"
        "Initialize firmware/c_board/bsp/tinyusb before building hpm_board.")
endif()

function(libhcs_add_current_tinyusb)
    set(tinyusb_sources
        "${libhcs_CURRENT_TINYUSB_ROOT}/src/tusb.c"
        "${libhcs_CURRENT_TINYUSB_ROOT}/src/common/tusb_fifo.c"
        "${libhcs_CURRENT_TINYUSB_ROOT}/src/device/usbd.c"
        "${libhcs_CURRENT_TINYUSB_ROOT}/src/portable/chipidea/ci_hs/dcd_ci_hs.c"
    )
    foreach(relative_source IN LISTS ARGN)
        list(APPEND tinyusb_sources "${libhcs_CURRENT_TINYUSB_ROOT}/${relative_source}")
    endforeach()

    sdk_src(${tinyusb_sources})
    sdk_sys_inc(
        "${libhcs_CURRENT_TINYUSB_ROOT}/src"
        "${libhcs_CURRENT_TINYUSB_ROOT}/src/portable/chipidea/ci_hs"
    )
endfunction()
