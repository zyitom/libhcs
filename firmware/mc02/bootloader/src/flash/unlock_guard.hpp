#pragma once

#include <main.h>

namespace libhcs::firmware::flash {

// flash 控制寄存器锁的 RAII 封装。
//
// 解锁失败通过 ok() 上报而不 trap: bootloader 是恢复的最后防线, 拒绝解锁的
// flash 控制器必须以主机可处理的 DFU 错误状态呈现, 而不是 HardFault 后悄悄
// 掉线。
class UnlockGuard {
public:
    UnlockGuard()
        : unlocked_(HAL_FLASH_Unlock() == HAL_OK) {}

    UnlockGuard(const UnlockGuard&) = delete;
    UnlockGuard& operator=(const UnlockGuard&) = delete;
    UnlockGuard(UnlockGuard&&) = delete;
    UnlockGuard& operator=(UnlockGuard&&) = delete;

    // 重新上锁不可恢复, 也不会使已提交的写入失效, 因此有意丢弃其返回值;
    // 若控制器真已不可用, 下一次解锁自会报告。
    ~UnlockGuard() { (void)HAL_FLASH_Lock(); }

    bool ok() const { return unlocked_; }

private:
    bool unlocked_;
};

} // namespace libhcs::firmware::flash
