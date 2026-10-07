#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <gpio.h>
#include <main.h>
#include <tim.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/protocol/vendor_control.hpp"
#include "core/include/libhcs/spec/mc02/ports.hpp"
#include "core/src/link/port.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"
#include "firmware/mc02/app/src/timer/timer.hpp"
#include "firmware/mc02/app/src/usb/helper.hpp"
#include "firmware/mc02/app/src/utility/loop_work.hpp"

namespace libhcs::firmware::gpio {

namespace vc = libhcs::core::protocol::vendor_control;
namespace link = libhcs::core::link;

// 有没有引脚要主循环去周期采样: 引脚状态每次变化后重算, 结果记在 loop::active 的
// kGpioSampling 位(定义在本文件末尾, 它要看全部引脚)。
void refresh_sampling();

// ---- GPIO 口 = PWM 排针, 一个引脚 = 口上的一根线 ----
//
// 排针是一路口(DataId::kGpio), 四个引脚是它的线(spec/mc02/ports.hpp 的 Spec::kGpios),
// 逐线声明, 与 CAN/UART 同一条规矩: 清单声明了才动。没声明的线 -- 定时器通道不启动、
// EXTI 线不武装、主循环里不采样 -- 不占一条指令。上电时什么都不碰: 引脚保持
// MX_GPIO_Init / MX_TIMx_Init 给的样子(定时器复用, 通道输出没使能, 等于悬空)。
//
// 线的原语(core/src/link/port_ops.hpp 的 GpioLineDriver):
//   configure()  写引脚寄存器: 输出 = 定时器复用推挽, 比较值 0, 第一次用到时启动该定时器
//                通道; 输入 = 输入模式 + 上下拉 + 边沿选择(中断先屏蔽)。设置没变就什么
//                都不写: 运行期改别的口会重放整份清单, 正在跑的 PWM 不能因此掉到 0。
//   resume()     开工: 输入解除边沿屏蔽、开始周期采样; 输出开始接收写入。
//   suspend()    停: 输出比较值归零, 引脚拉低而不是放成高阻(悬空的信号线会被电调读成
//                杂波脉冲); 输入屏蔽边沿、停采样; 下行的写与读请求一律丢弃。
//
// 定时器通道启动后不再停: HAL_TIM_PWM_Stop 在最后一个通道关掉时连计数器一起停。当初是因为
// TIM2 的计数器同时是 SOF 捕获的时基; 2026-10-05 起捕获挪到 TIM5(sync/sof.cpp), TIM2 只做
// PWM, 不停的做法保留(无害)。比较值只写 CCR、不发 UG。
//
// 全部在主循环调用(EP0 处理器、下行回调、会话状态机都经 tud_task() 到达), 只有
// handle_edge() 在 EXTI 中断里。
class Pin : private core::utility::Immovable {
public:
    struct Wiring {
        GPIO_TypeDef* gpio_port;
        uint16_t gpio_pin;
        uint8_t alternate_function;
        TIM_HandleTypeDef* timer;
        uint32_t timer_channel;
        IRQn_Type exti_irq;
    };

    Pin(uint8_t line, const Wiring& wiring)
        : line_(line)
        , wiring_(wiring)
        // CCR1..CCR4 连续排列, 通道号(0/4/8/12)右移两位即偏移 -- 与 __HAL_TIM_SET_COMPARE
        // 同一算法, 只是只算一次。
        , compare_register_(&wiring.timer->Instance->CCR1 + (wiring.timer_channel >> 2U)) {}

    // 在 GPIO 口上的线号: 记录流里的引脚身份。
    [[nodiscard]] uint8_t line() const { return line_; }
    [[nodiscard]] uint16_t gpio_pin() const { return wiring_.gpio_pin; }

    // ---- EP0 口 ----

    [[nodiscard]] bool running() const { return running_; }

    void suspend() {
        running_ = false;
        if (setting_.mode == vc::kGpioModeOutput)
            *compare_register_ = 0;
        disarm_edges();
        refresh_sampling();
    }

    void resume() {
        if (setting_.mode == vc::kGpioModeOff)
            return; // 从没声明过: 没有可恢复的
        if (setting_.mode == vc::kGpioModeInput) {
            next_sample_ = timer::timer->timepoint();
            arm_edges();
        }
        running_ = true;
        refresh_sampling();
    }

    void configure(const vc::GpioConfigPayload& setting) {
        if (setting.mode == setting_.mode && setting.pull == setting_.pull
            && setting.input_flags == setting_.input_flags
            && setting.period_ms == setting_.period_ms)
            return;

        // 改方向的窗口里不接收写入, 也不响应边沿。
        running_ = false;
        disarm_edges();
        if (setting.mode == vc::kGpioModeOutput)
            configure_output();
        else
            configure_input(setting);
        setting_ = setting;
        sample_period_ =
            setting.period_ms == 0U
                ? kNoPeriod
                : timer::Timer::to_duration_checked(std::chrono::milliseconds{setting.period_ms});
        refresh_sampling();
    }

    // 写后回读: 方向(MODER)与上下拉(PUPDR)。上下拉的寄存器编码与 vc::GpioPull 相同
    // (00 无、01 上拉、10 下拉)。
    [[nodiscard]] bool matches(const vc::GpioConfigPayload& setting) const {
        const uint32_t shift = 2U * pin_number();
        const uint32_t mode = (wiring_.gpio_port->MODER >> shift) & 0x3U;
        const uint32_t pull = (wiring_.gpio_port->PUPDR >> shift) & 0x3U;
        if (setting.mode == vc::kGpioModeOutput)
            return mode == kModerAlternate && pull == 0U;
        return mode == kModerInput && pull == setting.pull;
    }

    void read_config(vc::GpioConfigPayload& out) const {
        out = running_ ? setting_ : vc::GpioConfigPayload{};
    }

    // ---- 数据通路 ----

    void handle_digital_write(const data::GpioDigitalDataView& data) {
        if (!running_ || setting_.mode != vc::kGpioModeOutput)
            return;
        *compare_register_ = data.high ? counter_period() : 0U;
    }

    void handle_analog_write(const data::GpioAnalogDataView& data) {
        if (!running_ || setting_.mode != vc::kGpioModeOutput)
            return;
        *compare_register_ = duty16_to_compare(data.value);
    }

    // kRead: 现在采一次。上下拉在声明时就已生效, 到主机发得出这条请求时早已稳定(EP0 声明
    // 与它之间至少隔一次控制传输), 不必再等。
    void handle_read_request() {
        if (!running_ || setting_.mode != vc::kGpioModeInput)
            return;
        publish(read_level(), current_timestamp_quarter_us());
    }

    // 主循环, kGpioSampling 位置着时: 到期就采一次。
    void poll_periodic() {
        if (!running_ || sample_period_ == kNoPeriod)
            return;
        if (!timer::timer->check_reached(next_sample_))
            return;
        publish(read_level(), current_timestamp_quarter_us());
        next_sample_ = timer::timer->timepoint() + sample_period_;
    }

    // EXTI 中断。边沿只在声明了边沿且正在运行时才被解除屏蔽, 这里的检查挡的是屏蔽前
    // 已挂起的那一个。
    void handle_edge() {
        if (!running_ || !edges_armed_)
            return;
        const uint32_t timestamp_quarter_us = current_timestamp_quarter_us();
        publish(read_level(), timestamp_quarter_us);
    }

    // 新会话取代旧会话: 旧主机写的输出值不留给新主机继承, 方向与运行状态不变。
    void zero_output() {
        if (setting_.mode == vc::kGpioModeOutput)
            *compare_register_ = 0;
    }

    [[nodiscard]] bool samples_periodically() const {
        return running_ && sample_period_ != kNoPeriod;
    }

private:
    static constexpr auto kNoPeriod = timer::Timer::Duration::zero();
    static constexpr uint32_t kModerInput = 0U;
    static constexpr uint32_t kModerAlternate = 2U;
    static constexpr uint32_t kEdges = vc::kGpioInputRisingEdge | vc::kGpioInputFallingEdge;

    [[nodiscard]] uint32_t pin_number() const {
        return static_cast<uint32_t>(__builtin_ctz(wiring_.gpio_pin));
    }

    void configure_output() {
        // HAL_GPIO_Init() 只在新模式带中断时才改 EXTI 寄存器: 以边沿输入用过的引脚切到
        // 输出后, 边沿配置还在。HAL_GPIO_DeInit() 解除该线的映射与触发, 再清掉可能已
        // 挂起的标志(PE13 与 BMI088 数据就绪共用 EXTI15_10)。
        HAL_GPIO_DeInit(wiring_.gpio_port, wiring_.gpio_pin);
        __HAL_GPIO_EXTI_CLEAR_IT(wiring_.gpio_pin);

        *compare_register_ = 0; // 从低电平开始
        GPIO_InitTypeDef gpio_init = {};
        gpio_init.Pin = wiring_.gpio_pin;
        gpio_init.Mode = GPIO_MODE_AF_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        gpio_init.Alternate = wiring_.alternate_function;
        HAL_GPIO_Init(wiring_.gpio_port, &gpio_init);

        if (!channel_started_) {
            core::utility::assert_always(
                HAL_TIM_PWM_Start(wiring_.timer, wiring_.timer_channel) == HAL_OK);
            channel_started_ = true;
        }
    }

    void configure_input(const vc::GpioConfigPayload& setting) {
        const bool rising = (setting.input_flags & vc::kGpioInputRisingEdge) != 0U;
        const bool falling = (setting.input_flags & vc::kGpioInputFallingEdge) != 0U;
        GPIO_InitTypeDef gpio_init = {};
        gpio_init.Pin = wiring_.gpio_pin;
        if (rising && falling)
            gpio_init.Mode = GPIO_MODE_IT_RISING_FALLING;
        else if (rising)
            gpio_init.Mode = GPIO_MODE_IT_RISING;
        else if (falling)
            gpio_init.Mode = GPIO_MODE_IT_FALLING;
        else
            gpio_init.Mode = GPIO_MODE_INPUT;
        switch (setting.pull) {
        case vc::kGpioPullUp: gpio_init.Pull = GPIO_PULLUP; break;
        case vc::kGpioPullDown: gpio_init.Pull = GPIO_PULLDOWN; break;
        default: gpio_init.Pull = GPIO_NOPULL; break;
        }
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(wiring_.gpio_port, &gpio_init);
        // HAL 带中断模式时顺手解除了屏蔽; 开不开由 resume() 决定。
        if (rising || falling) {
            CLEAR_BIT(EXTI->IMR1, wiring_.gpio_pin);
            __HAL_GPIO_EXTI_CLEAR_IT(wiring_.gpio_pin);
        }
    }

    void arm_edges() {
        if ((setting_.input_flags & kEdges) == 0U)
            return;
        // 本线的中断向量第一次被用到时才使能。EXTI15_10 由 MX_GPIO_Init 为 BMI088 使能过,
        // 已使能的不再改它的优先级。
        if (NVIC_GetEnableIRQ(wiring_.exti_irq) == 0U) {
            HAL_NVIC_SetPriority(wiring_.exti_irq, 4, 0);
            HAL_NVIC_EnableIRQ(wiring_.exti_irq);
        }
        __HAL_GPIO_EXTI_CLEAR_IT(wiring_.gpio_pin);
        SET_BIT(EXTI->IMR1, wiring_.gpio_pin);
        edges_armed_ = true;
    }

    // 只撤自己武装过的线: EXTI 线按引脚号共用, 别的口(或 IMU)可能正用着同号的线。
    void disarm_edges() {
        if (!edges_armed_)
            return;
        CLEAR_BIT(EXTI->IMR1, wiring_.gpio_pin);
        __HAL_GPIO_EXTI_CLEAR_IT(wiring_.gpio_pin);
        edges_armed_ = false;
    }

    [[nodiscard]] bool read_level() const {
        return HAL_GPIO_ReadPin(wiring_.gpio_port, wiring_.gpio_pin) == GPIO_PIN_SET;
    }

    void publish(bool high, uint32_t timestamp_quarter_us) const {
        const std::optional<uint32_t> timestamp =
            (setting_.input_flags & vc::kGpioInputTimestamp) != 0U
                ? std::optional<uint32_t>{timestamp_quarter_us}
                : std::nullopt;
        core::utility::assert_debug(
            usb::get_serializer().write_gpio_digital_value(
                line_, {.high = high, .timestamp_quarter_us = timestamp})
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    [[nodiscard]] static uint32_t current_timestamp_quarter_us() {
        return timer::timer->timepoint().time_since_epoch().count();
    }

    // 所属定时器的 ARR + 1。运行时读取而非编译期常量: 两个定时器不共用周期。TIM2 跑
    // 275 MHz、ARR 5499999, TIM1 跑 1 MHz、ARR 19999, 两者都产生 50 Hz。TIM2 曾同时是
    // USB-SOF 捕获定时器, 所以周期被锁死; 2026-10-05 起捕获在 TIM5(sync/sof.cpp), TIM2 的
    // 周期可以按需在 .ioc 里改。
    [[nodiscard]] uint32_t counter_period() const { return wiring_.timer->Instance->ARR + 1U; }

    // 64 位中间量: TIM2 周期为 5500000 计数时, 乘积在 duty 约 780 处溢出 32 位, 会静默
    // 回绕。
    [[nodiscard]] uint32_t duty16_to_compare(uint16_t duty) const {
        const uint64_t period = counter_period();
        return static_cast<uint32_t>(((static_cast<uint64_t>(duty) * period) + 32767U) / 65535U);
    }

    const uint8_t line_;
    const Wiring wiring_;
    volatile uint32_t* const compare_register_;

    // 写进引脚寄存器的设置; kGpioModeOff = 从没声明过。挂起不清它: 重新声明成同样的设置
    // 时 configure() 不必重写引脚, resume() 按它重新开工。
    vc::GpioConfigPayload setting_{};
    bool running_ = false;
    bool channel_started_ = false;
    bool edges_armed_ = false;
    timer::Timer::Duration sample_period_ = kNoPeriod;
    timer::Timer::TimePoint next_sample_{};
};

// GPIO 口的驱动: 四根线。对象在启动时构造(不碰硬件), 线由清单逐根声明启动。
class Gpio : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Gpio>;
    using Spec = spec::mc02::Spec;

    // 口的原语(core/src/link/port_ops.hpp 的 GpioPortDriver)。四根线都在定时器通道上, 也都
    // 有自己的 EXTI 线 -- 全部能力。口清单里 GPIO 口的能力字节是线数。
    static constexpr uint8_t kLineCount = 4;
    static constexpr std::array<uint8_t, kLineCount> kLineCapabilities{
        spec::kGpioCapPwmPin, spec::kGpioCapPwmPin, spec::kGpioCapPwmPin, spec::kGpioCapPwmPin};
    static constexpr uint8_t kPortCapabilities = kLineCount;
    static_assert(kLineCount == Spec::kGpioLineCount);

    Pin pwm1{
        Spec::Gpios::kPwm1.line,
        {GPIOA, GPIO_PIN_0, GPIO_AF1_TIM2, &htim2, TIM_CHANNEL_1, EXTI0_IRQn}
    };
    Pin pwm2{
        Spec::Gpios::kPwm2.line,
        {GPIOA, GPIO_PIN_2, GPIO_AF1_TIM2, &htim2, TIM_CHANNEL_3, EXTI2_IRQn}
    };
    Pin pwm3{
        Spec::Gpios::kPwm3.line,
        {GPIOE, GPIO_PIN_9, GPIO_AF1_TIM1, &htim1, TIM_CHANNEL_1, EXTI9_5_IRQn}
    };
    Pin pwm4{
        Spec::Gpios::kPwm4.line,
        {GPIOE, GPIO_PIN_13, GPIO_AF1_TIM1, &htim1, TIM_CHANNEL_3, EXTI15_10_IRQn}
    };

    // 第 index 根线; 本口没有这根线时为空(下行记录的线号来自主机, 不可信)。
    [[nodiscard]] Pin* line(uint8_t index) { return index < kLineCount ? pins()[index] : nullptr; }

    // 整口: 会话结束、清单回滚、归属交接。resume() 只让声明过的线重新开工。
    void suspend() {
        for (Pin* p : pins())
            p->suspend();
    }
    void resume() {
        for (Pin* p : pins())
            p->resume();
    }
    [[nodiscard]] link::PortStatus describe() {
        bool running = false;
        for (Pin* p : pins())
            running = running || p->running();
        return {.running = running, .fd = false};
    }

    // 新会话取代旧会话: 只清输出值, 引脚仍按刚被接受的声明工作。
    void zero_outputs() {
        for (Pin* p : pins())
            p->zero_output();
    }

    // 主循环入口, 只在 loop::active 的 kGpioSampling 位置着时调用: 没有引脚配周期采样时
    // -- 绝大多数接线如此 -- 主循环根本不进来, 也就不去读定时器(一次 D2 域外设访问)。
    void poll_periodic() {
        for (Pin* p : pins())
            p->poll_periodic();
    }

    // EXTI 回调里, IMU 的两条线之外的边沿。
    void handle_edge(uint16_t gpio_pin) {
        for (Pin* p : pins()) {
            if (p->gpio_pin() == gpio_pin)
                p->handle_edge();
        }
    }

    [[nodiscard]] bool any_sampling() {
        for (Pin* p : pins()) {
            if (p->samples_periodically())
                return true;
        }
        return false;
    }

private:
    [[nodiscard]] std::array<Pin*, kLineCount> pins() { return {&pwm1, &pwm2, &pwm3, &pwm4}; }
};

inline constinit Gpio::Lazy gpio;

inline void refresh_sampling() {
    if (gpio->any_sampling())
        loop::set(loop::kGpioSampling);
    else
        loop::clear(loop::kGpioSampling);
}

} // namespace libhcs::firmware::gpio
