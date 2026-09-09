sdk_inc(.)
sdk_src(board.c)

# Secondary-core (core1) images see the same board files; mirror the SDK board
# convention so BOARD_RUNNING_CORE selects the per-core defaults (console off,
# mchtmr1, ...).
sdk_compile_definitions_ifdef(BUILD_FOR_SECONDARY_CORE BOARD_RUNNING_CORE=HPM_CORE1)

# Unique per board: the DFU bootloader enumerates with this PID and dfu-util
# matches images against it (0x5321/0x5322 the two HPM5321 PCBs).
#
# Single-variant board, so this is a genuine compile-time constant. Contrast
# boards/hpm5321, whose one image serves two PCBs and picks its PID at run time.
set_property(GLOBAL PROPERTY libhcs_USB_PID 0x6E84)
sdk_compile_definitions(libhcs_USB_PID=0x6E84)

# The USB application reuses the shared app-layer drivers; this board directory
# provides their port tables and pin hooks.
if(libhcs_BUILDING_FIRMWARE_APP)
    sdk_app_inc(app)
    # The application (App + main) is per board: see app/app.cpp.
    sdk_app_src(app/board_app.cpp app/app.cpp)
endif()

# Two orthogonal knobs feed board_init_clock() (see the comment there for the
# clock-group semantics):
#
#   BOARD_FIELDBUS_ON_CORE0    CAN0..CAN3 + UART1 sit in core0's clock group
#   BOARD_SECONDARY_CORE_USED  core1 gets released, so group 1 must exist
#                              (cpu1 + mchtmr1, and the fieldbus peripherals
#                              only when they are not already in group 0)
#
# The USB application is single-core: it runs USB and CAN on core0 and never
# releases core1, so a fieldbus peripheral left in group 1 would stay
# clock-gated forever. Either variable may be pre-set by a parent project.
if(NOT DEFINED BOARD_FIELDBUS_ON_CORE0)
    if(libhcs_BUILDING_FIRMWARE_APP)
        set(BOARD_FIELDBUS_ON_CORE0 1)
    else()
        set(BOARD_FIELDBUS_ON_CORE0 0)
    endif()
endif()
if(NOT DEFINED BOARD_SECONDARY_CORE_USED)
    if(NOT libhcs_BUILDING_FIRMWARE_APP)
        set(BOARD_SECONDARY_CORE_USED 1)
    else()
        set(BOARD_SECONDARY_CORE_USED 0)
    endif()
endif()
sdk_compile_definitions(BOARD_FIELDBUS_ON_CORE0=${BOARD_FIELDBUS_ON_CORE0})
sdk_compile_definitions(BOARD_SECONDARY_CORE_USED=${BOARD_SECONDARY_CORE_USED})
