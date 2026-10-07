#pragma once

#include <atomic>
#include <cstdint>

namespace libhcs::firmware::utility {

// 单写者事件计数: 一个上下文(中断或主循环)累加, 别的上下文只读。端口运行时状态
// (core/src/link/port_status.hpp)的各项计数都是它; 线上只带低 16 位(按 2^16 回绕,
// 主机取相邻两次之差), 所以 count() 直接给 16 位。
//
// relaxed 的读 + 写, 不是 fetch_add: 单写者用不着原子读改写, 在 Cortex-M 与 RV32
// 上编出来就是一条 ldr/str(lw/sw), 与原先的裸 uint32_t 同价, 却不再是标准意义上的
// 数据竞争。只在出错分支上累加, 热路径不经过这里。
class EventCounter {
public:
    void note(std::uint32_t events = 1U) noexcept {
        count_.store(count_.load(std::memory_order_relaxed) + events, std::memory_order_relaxed);
    }

    [[nodiscard]] std::uint16_t count() const noexcept {
        return static_cast<std::uint16_t>(count_.load(std::memory_order_relaxed));
    }

private:
    std::atomic<std::uint32_t> count_{0};
};

static_assert(std::atomic<std::uint32_t>::is_always_lock_free);

} // namespace libhcs::firmware::utility
