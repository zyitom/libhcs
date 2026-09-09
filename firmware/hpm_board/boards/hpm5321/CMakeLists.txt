sdk_inc(.)
sdk_src(board.c)

# This board directory serves BOTH HPM5321 PCBs -- single-CAN and dual-CAN --
# from one image, so its USB PID is not a build-time constant. The
# firmware reports 0x5321 or 0x5322 at run time from the OTP board identity (see
# common/board_identity.hpp and app/src/usb/usb_descriptors.hpp). The definition
# below is only the fallback baked into the descriptor constant and the value the
# bootloader reports on a board it recognizes; it is NOT what a merged app
# enumerates as.
set_property(GLOBAL PROPERTY libhcs_USB_PID 0x5321)
sdk_compile_definitions(libhcs_USB_PID=0x5321)

# Turn on the runtime OTP variant check. Only this board defines it: the two
# accepted values of OTP word 25 are HPM5321 observations, and the other boards
# read a different value there (boards/hpm6e8y/README.md records 0x6), so an
# ungated check would refuse to boot them.
sdk_compile_definitions(libhcs_BOARD_OTP_IDENTITY=1)

# The DFU container's PID must accept either board, since one .dfu file now
# serves both. dfu-util's wildcard (0xFFFF) matches any product id while still
# checking the vendor id and the CRC.
set_property(GLOBAL PROPERTY libhcs_DFU_PRODUCT_ID 0xFFFF)

if(libhcs_BUILDING_FIRMWARE_APP)
    sdk_app_inc(app)
    # The application (App + main) is per board: see app/app.cpp.
    sdk_app_src(app/board_app.cpp app/app.cpp)
endif()
