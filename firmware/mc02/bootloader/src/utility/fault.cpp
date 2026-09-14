#include <main.h>

// 生成的 stm32h7xx_it.c 中四个 fault handler 的恢复钩子, 该文件同时编入应用与
// bootloader: 应用借此复位进 DFU, 故障镜像不会变砖(见 app/src/utility/assert.cpp);
// bootloader 则有意不复位 -- 它是恢复的最后防线, 其 fault 几乎必然是确定性的,
// 复位只会原地循环, 设备永远无法保持枚举, 主机也就烧不进任何东西。直接返回,
// 由 fault handler 自带的 while(1) 停机, 故障现场原样留给调试器。
extern "C" void libhcs_fault_recover(void) {}

// 共享的 MX_GPIO_Init() 所使能 EXTI 组的杂散中断处理。
//
// gpio.c 由描述应用引脚配置的同一份 .ioc 生成, 且编入两个镜像: 其末尾把 BMI088
// 的 data-ready 线(PE10 INT1_ACC, PE12 INT1_GYRO)配成上升沿 EXTI 输入, 并调用
// HAL_NVIC_EnableIRQ(EXTI15_10_IRQn) -- 对只需要 KEY 与 USB 的 bootloader 而言
// 纯属附带伤害。
//
// 只有应用定义了 EXTI15_10_IRQHandler(app/src/gpio/gpio.cpp)。缺少本定义时,
// bootloader 会落入 startup_stm32h723vgtx.s 的弱别名 Default_Handler -- 一条
// 无条件 `b .`: 这不是 fault, 调试器看不到任何异常; 中断派发成功后在抢占优先
// 级 4 上永远空转, 饿死 SysTick 与线程模式, USB 尚未枚举 bootloader 即已停摆,
// 表现为整板变砖。
//
// 只有从运行中的应用热复位才会进入该状态: BMI088 是外部芯片, CPU 复位不影响
// 它, 它会持续 free-running 并把第一个 data-ready 脉冲送到这里; 冷上电时传感
// 器同样断电、以 suspend 态起来, 无法复现。
//
// 清除该组全部 6 根线而非仅 .ioc 当前使用的两根, 以防重新生成的 gpio.c 使能
// 10..15 中的其他引脚时挂死复现。
extern "C" void EXTI15_10_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(
        GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
}
