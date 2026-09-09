#include "firmware/ch32_board/app/src/usb/descriptors.hpp"

#include <cstdint>

extern "C" {
#include "usb_desc.h"
}

#include "firmware/ch32_board/common/usb_identity.hpp"

extern "C" {

// Defined here rather than in the vendored bsp/usb/usb_desc.c: these three are
// the libhcs device identity, and the product string has to carry the build's
// version. See the libhcs LOCAL PATCH notes in usb_desc.c / usb_desc.h.
uint8_t MyManuInfo[libhcs::firmware::usb::kStringDescriptorSize];
uint8_t MyProdInfo[libhcs::firmware::usb::kStringDescriptorSize];
uint8_t MySerNumInfo[libhcs::firmware::usb::kStringDescriptorSize];

void libhcs_usb_init_descriptors(void) {
    namespace usb = libhcs::firmware::usb;

    // The host SDK's device scanner matches this exact string after the VID/PID
    // match, so it must stay in sync with c_board/mc02.
    usb::write_string_descriptor(MyManuInfo, usb::kManufacturerString);
    usb::write_string_descriptor(MyProdInfo, "HCS Agent v" libhcs_PROJECT_VERSION_STRING);
    usb::write_serial_descriptor(MySerNumInfo);
}

} // extern "C"
