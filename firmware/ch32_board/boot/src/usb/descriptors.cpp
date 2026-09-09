// Device identity for the bootloader image.
//
// Same VID/PID and the same UID-derived serial as the application, so a host can
// follow one board across a DFU cycle; only the product string differs, which is
// what tells "running application" apart from "sitting in DFU mode".

#include <cstdint>

extern "C" {
#include "usb_desc.h"
}

#include "firmware/ch32_board/common/usb_identity.hpp"

extern "C" {

uint8_t MyManuInfo[libhcs::firmware::usb::kStringDescriptorSize];
uint8_t MyProdInfo[libhcs::firmware::usb::kStringDescriptorSize];
uint8_t MySerNumInfo[libhcs::firmware::usb::kStringDescriptorSize];

void libhcs_usb_init_descriptors(void) {
    namespace usb = libhcs::firmware::usb;

    usb::write_string_descriptor(MyManuInfo, usb::kManufacturerString);
    usb::write_string_descriptor(MyProdInfo, "HCS Bootloader v" libhcs_PROJECT_VERSION_STRING);
    usb::write_serial_descriptor(MySerNumInfo);
}

} // extern "C"
