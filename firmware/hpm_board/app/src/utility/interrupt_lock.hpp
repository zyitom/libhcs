#pragma once

#include <atomic>
#include <cstdint>

#include <hpm_csr_regs.h>
#include <hpm_soc.h>

#include "core/src/utility/immovable.hpp"

namespace libhcs::firmware::utility {

// 锁内的内存访问必须被编译器挡在临界区里, 而不只被 CPU 挡住。SDK 的
// disable_global_irq() 是一条无 memory clobber 的裸 csrrc
// (arch/riscv/riscv_core.h), 编译器可以把普通内存访问自由移过它 -- sof_probe
// 因此改用 atomic; 对其余 ISR 共享状态, 这里在开关中断两侧各放一道
// atomic_signal_fence: 中断对本核就是"信号", 该栅栏恰好是它的编译器语义,
// 单核上零指令, 硬件侧由 csr 指令本身顺序保证。
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
