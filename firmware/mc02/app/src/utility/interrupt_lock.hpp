#pragma once

#include <atomic>

#include <main.h> // IWYU pragma: keep (disable/enable irq)

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"

namespace libhcs::firmware::utility {

class InterruptMutex {
public:
    InterruptMutex() = delete;

    static void lock() {
        __disable_irq();
        // CMSIS 的 cpsid i 自带 memory clobber, 此栅栏是冗余的; 显式写出让本
        // 锁的编译器语义不依赖第三方头文件的实现细节, 与 hpm_board 的锁同一
        // 约定。
        std::atomic_signal_fence(std::memory_order_seq_cst);
        ++lock_count_;
    }

    static void unlock() {
        core::utility::assert_debug(lock_count_ > 0);
        if (--lock_count_ == 0) {
            std::atomic_signal_fence(std::memory_order_seq_cst);
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
