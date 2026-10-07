#include <chrono>
#include <cstdint>

#include <stm32f4xx_hal.h>

#include "firmware/c_board/app/src/led/led.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"

namespace libhcs::firmware::timer {

// 重写 HAL_Delay, 保证它在关中断时也能工作, 同时大幅提高精度。
extern "C" void HAL_Delay(uint32_t delay) {
    timer::timer->spin_wait(timer::Timer::to_duration48_checked(std::chrono::milliseconds(delay)));
}

// 把这个没有用处的函数改造成周期性执行低优先级任务的地方, 省下一个专用定时器外设。
extern "C" void HAL_IncTick() {
    const uint32_t tick = uwTick + 1;
    uwTick = tick;
    led::led->update(tick);
}

} // namespace libhcs::firmware::timer
