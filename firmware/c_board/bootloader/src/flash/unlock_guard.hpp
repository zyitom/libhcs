#pragma once

#include <main.h>

namespace libhcs::firmware::flash {

// RAII around the flash control-register lock.
//
// An unlock failure is reported through ok() rather than trapping: the
// bootloader is the last line of recovery, so a flash controller that refuses
// to unlock has to surface as a DFU error status the host can act on, not as a
// HardFault that silently takes the device off the bus.
class UnlockGuard {
public:
    UnlockGuard()
        : unlocked_(HAL_FLASH_Unlock() == HAL_OK) {}

    UnlockGuard(const UnlockGuard&) = delete;
    UnlockGuard& operator=(const UnlockGuard&) = delete;
    UnlockGuard(UnlockGuard&&) = delete;
    UnlockGuard& operator=(UnlockGuard&&) = delete;

    // Re-locking cannot be recovered from and cannot fail in a way that
    // invalidates work already committed, so its status is intentionally
    // dropped; the next unlock would report the controller as unusable.
    ~UnlockGuard() { (void)HAL_FLASH_Lock(); }

    bool ok() const { return unlocked_; }

private:
    bool unlocked_;
};

} // namespace libhcs::firmware::flash
