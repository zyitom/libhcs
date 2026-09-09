#pragma once

// WCH's core_riscv.h (pulled in via ch32h417.h) provides __disable_irq /
// __enable_irq, which toggle the global interrupt-enable bit in mstatus.MIE --
// the RISC-V equivalent of the CMSIS intrinsics mc02 uses on Cortex-M.
extern "C" {
#include "ch32h417.h"
}

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"

namespace libhcs::firmware::utility {

class InterruptMutex {
public:
    InterruptMutex() = delete;

    static void lock() {
        __disable_irq();
        ++lock_count_;
    }

    static void unlock() {
        core::utility::assert_debug(lock_count_ > 0);
        if (--lock_count_ == 0) {
            __enable_irq();
        }
    }

private:
    static inline int lock_count_ = 0;
};

class InterruptLockGuard : private core::utility::Immovable {
public:
    InterruptLockGuard() { InterruptMutex::lock(); }

    InterruptLockGuard(const InterruptLockGuard&) = delete;
    InterruptLockGuard& operator=(const InterruptLockGuard&) = delete;
    InterruptLockGuard(InterruptLockGuard&&) = delete;
    InterruptLockGuard& operator=(InterruptLockGuard&&) = delete;

    ~InterruptLockGuard() { InterruptMutex::unlock(); }
};

} // namespace libhcs::firmware::utility
