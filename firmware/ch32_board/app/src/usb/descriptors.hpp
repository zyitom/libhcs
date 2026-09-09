#pragma once

// The libhcs device identity for the WCH USBSS stack. The manufacturer,
// product and serial-number string descriptors that the vendored
// bsp/usb/usb_desc.c used to define are built here instead (see the
// libhcs LOCAL PATCH notes there): the product string has to carry
// libhcs_PROJECT_VERSION_STRING, which the host SDK's device scanner checks
// after matching VID 0xA511 / PID 0xD403.
//
// libhcs_usb_init_descriptors() is declared in usb_desc.h (C linkage, so the
// vendored stack can see it) and must run before USBSS_Device_Init().
