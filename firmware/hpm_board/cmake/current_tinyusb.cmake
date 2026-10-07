include_guard()

# 与 c_board、mc02 共用同一份 TinyUSB checkout。SoC、PHY 与寄存器头仍归 HPM SDK
# 所有, 但其自带的 TinyUSB 源码不参与编译。
set(libhcs_CURRENT_TINYUSB_ROOT "${libhcs_PROJECT_ROOT}/firmware/common/bsp/tinyusb")

if(NOT EXISTS "${libhcs_CURRENT_TINYUSB_ROOT}/src/tusb.c")
    message(FATAL_ERROR
        "TinyUSB submodule is missing at ${libhcs_CURRENT_TINYUSB_ROOT}.\n"
        "Initialize firmware/common/bsp/tinyusb before building hpm_board.")
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
