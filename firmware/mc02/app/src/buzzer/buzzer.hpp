#pragma once

#include <cstdint>

#include <main.h>
#include <tim.h>

#include "firmware/common/app/src/buzzer/buzzer.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"

namespace libhcs::firmware::buzzer {

// mc02 binding: 驱动与口的通用部分在 firmware/common/app/src/buzzer/buzzer.hpp。
//
// PB15, TIM12 CH2。mc02_slave.ioc 给 TIM12 Prescaler = 274、Period = 249, APB1
// 定时器内核跑 275 MHz(APB1 为 137.5 MHz, D2PPRE1 = DIV2 使定时器时钟倍频), 故
// 计数器恰以 275 MHz / 275 = 1 MHz 走 -- 即下面传入的值。生成代码的默认参数恰好
// 发出 1 MHz / 250 = 4 kHz。
//
// App() 在上电时调 MX_TIM12_Init() 并 init() 本驱动(不碰寄存器), 通道不启动: 蜂鸣器是
// EP0 的一路口(DataId::kBuzzer, 下面的 BuzzerPort), 主机声明了才 start()。
//
// 仅限主循环: play() 与 poll() 不碰原子量、绝不在 ISR 中调用; 不像 LED, 其
// buffer-full 计数器会从中断上下文置位。
inline constexpr uint32_t kMc02TimerTickHz = 1000000;

inline constinit utility::Lazy<Buzzer, TIM_HandleTypeDef*, uint32_t, uint32_t> buzzer{
    &htim12, TIM_CHANNEL_2, kMc02TimerTickHz};

// 蜂鸣器口(firmware/common/app/src/buzzer/buzzer.hpp 的 BuzzerPort)接在上面这个驱动上。
inline constinit BuzzerPort<buzzer> buzzer_port;

} // namespace libhcs::firmware::buzzer
