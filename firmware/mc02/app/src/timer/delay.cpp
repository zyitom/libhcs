#include <chrono>
#include <concepts>
#include <cstdint>
#include <ratio>
#include <type_traits>

#include <stm32h7xx_hal.h>

namespace libhcs::firmware::timer {

namespace {

// 忙等由 STM32 DWT(Data Watchpoint and Trace)周期计数器驱动。与 timer::timer
// (TIM5)不同, 它在该时间戳源启动之前、以及中断关闭时都能工作 -- HAL_Delay 被
// App() 里运行于中断锁下的 HAL 时钟与外设初始化调用, 那时 timer.init() 还没跑。
// timer.init() 之后的传感器初始化延时改用 timer::timer->spin_wait(); 本文件保留
// DWT 方案只为 HAL_Delay。
constexpr uint32_t kSystemFrequency = 550'000'000;
using SysFreqDuration = std::chrono::duration<uint32_t, std::ratio<1, kSystemFrequency>>;

void delay_basic(SysFreqDuration d) {
    if (!d.count()) [[unlikely]]
        return;

    const uint32_t start = DWT->CYCCNT;
    const uint32_t end = start + d.count();

    if (end < start) {
        while (DWT->CYCCNT >= start)
            ;
    }
    while (DWT->CYCCNT < end)
        ;
}

template <std::integral Rep, typename Period>
void delay(std::chrono::duration<Rep, Period> d) {
    if constexpr (std::is_signed_v<Rep>) {
        if (d.count() < 0) [[unlikely]]
            return;
    }

    using DurationT = std::chrono::duration<uint32_t, Period>;
    auto casted = DurationT{static_cast<uint32_t>(d.count())};
    static constexpr auto kMax = std::chrono::floor<DurationT>(SysFreqDuration::max());
    static_assert(kMax.count() > 0, "Unit too large; choose a smaller unit");

    while (casted > kMax) {
        casted -= kMax;
        delay_basic(SysFreqDuration::max());
    }
    delay_basic(std::chrono::round<SysFreqDuration>(casted));
}

} // namespace

extern "C" void HAL_Delay(uint32_t ms) { delay(std::chrono::milliseconds(ms)); }

extern "C" void HAL_IncTick() { uwTick += 1; }

} // namespace libhcs::firmware::timer
