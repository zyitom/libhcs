#pragma once

#include <cstdint>

#include <main.h>
#include <tim.h>

#include "firmware/common/app/src/buzzer/buzzer.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"

namespace libhcs::firmware::buzzer {

// c_board binding: 驱动与口的通用部分在 firmware/common/app/src/buzzer/buzzer.hpp。
//
// PD14, TIM4 CH3(《RoboMaster 开发板 C 型用户手册》蜂鸣器一节: 贴片无源蜂鸣器, 额定
// 4000 Hz)。c_board_slave.ioc 给 TIM4 Prescaler = 84-1、Period = 250-1, APB1 定时器
// 时钟 84 MHz(PCLK1 42 MHz, APB1 分频非 1 时定时器时钟倍频), 故计数器以
// 84 MHz / 84 = 1 MHz 走 -- 即下面传入的值; 生成代码的周期对应 4 kHz, 比较值为 0。
//
// App() 在上电时调 MX_TIM4_Init() 并 init() 本驱动(不碰寄存器), 通道不启动: 蜂鸣器是
// EP0 的一路口(DataId::kBuzzer), 主机声明了才 start()。与 mc02 同一纪律: 只在主循环。
inline constexpr uint32_t kCBoardTimerTickHz = 1000000;

inline constinit utility::Lazy<Buzzer, TIM_HandleTypeDef*, uint32_t, uint32_t> buzzer{
    &htim4, TIM_CHANNEL_3, kCBoardTimerTickHz};

inline constinit BuzzerPort<buzzer> buzzer_port;

} // namespace libhcs::firmware::buzzer
