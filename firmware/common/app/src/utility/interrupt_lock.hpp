#pragma once

#include <atomic>

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"

// 关全局中断的锁, 两个架构各一份实现, 约定相同: 锁内的内存访问必须被编译器
// 挡在临界区里, 而不只被 CPU 挡住, 所以在关、开中断两侧各放一道
// atomic_signal_fence。中断对本核就是"信号", 该栅栏恰好是它的编译器语义;
// 单核上零指令, 硬件侧顺序由关中断原语本身保证。

#if defined(__arm__)

# include <main.h> // IWYU pragma: keep (disable/enable irq)

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

#elif defined(__riscv)

# include <hpm_csr_regs.h>
# include <hpm_soc.h>

namespace libhcs::firmware::utility {

// SDK 的 disable_global_irq() 是一条无 memory clobber 的裸 csrrc
// (arch/riscv/riscv_core.h), 编译器可以把普通内存访问自由移过它 -- sof_probe
// 因此改用 atomic; 对其余 ISR 共享状态, 这里在开关中断两侧各放一道
// atomic_signal_fence: 单核上零指令, 硬件侧由 csr 指令本身顺序保证。
class InterruptLockGuard : core::utility::Immovable {
public:
    InterruptLockGuard() noexcept
        : flags_(disable_global_irq(CSR_MSTATUS_MIE_MASK)) {
        std::atomic_signal_fence(std::memory_order_seq_cst);
    }

    InterruptLockGuard(const InterruptLockGuard&) = delete;
    InterruptLockGuard& operator=(const InterruptLockGuard&) = delete;
    InterruptLockGuard(InterruptLockGuard&&) = delete;
    InterruptLockGuard& operator=(InterruptLockGuard&&) = delete;

    ~InterruptLockGuard() noexcept {
        std::atomic_signal_fence(std::memory_order_seq_cst);
        restore_global_irq(flags_ & CSR_MSTATUS_MIE_MASK);
    }

private:
    uint32_t flags_;
};

} // namespace libhcs::firmware::utility

#else
# error "unsupported architecture: no interrupt lock primitives"
#endif
