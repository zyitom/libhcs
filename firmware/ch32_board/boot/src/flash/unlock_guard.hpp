#pragma once

extern "C" {
#include "ch32h417.h"
}

namespace libhcs::firmware::flash {

// RAII around the flash controller's write lock. FLASH_Unlock()/FLASH_Lock()
// return void on this SDK (unlike STM32's HAL_StatusTypeDef), so failure shows
// up as a rejected erase/program rather than here; the writer checks those.
class UnlockGuard {
public:
    UnlockGuard() { FLASH_Unlock(); }

    UnlockGuard(const UnlockGuard&) = delete;
    UnlockGuard& operator=(const UnlockGuard&) = delete;
    UnlockGuard(UnlockGuard&&) = delete;
    UnlockGuard& operator=(UnlockGuard&&) = delete;

    ~UnlockGuard() { FLASH_Lock(); }
};

} // namespace libhcs::firmware::flash
