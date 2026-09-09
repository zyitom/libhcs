#pragma once

namespace libhcs::firmware::usb {

// Deferred half of the DFU_DETACH handler: the control transfer's status stage
// must complete before the device disappears, so the ISR only sets a flag and
// the forwarding loop calls this to record the request and reset. Safe to call
// every iteration -- it is a single volatile load when nothing is pending.
void poll_dfu_runtime_reboot();

} // namespace libhcs::firmware::usb
