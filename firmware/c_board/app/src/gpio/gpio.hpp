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
#include "core/include/libhcs/spec/c_board/ports.hpp"
#include "core/src/link/port.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/c_board/app/src/usb/helper.hpp"
#include "firmware/c_board/app/src/utility/loop_work.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"

namespace libhcs::firmware::gpio {

namespace vc = libhcs::core::protocol::vendor_control;
namespace link = libhcs::core::link;

// 有没有引脚在周期采样; 每次引脚状态变化后重算(定义在本文件末尾, 会看一遍每个引脚)。
void refresh_sampling();

// ---- GPIO 口 = PWM 排针, 一个引脚 = 口上的一根线 ----
//
// 排针是一路口(DataId::kGpio), 七个引脚是它的线(spec/c_board/ports.hpp 的 Spec::kGpios),
// 逐线声明, 规矩与 mc02 相同: 清单声明了才做事。没声明的线不启动定时器通道、不武装
// EXTI 线、主循环里不采样。上电时什么都不碰: 引脚保持 MX_GPIO_Init / MX_TIMx_Init 留下
// 的样子(定时器复用功能、通道输出未使能, 即悬空)。与本板的 CAN 和 UART 不同(不转换、
// 常开), 线没有要为谁保持的运行状态, 所以它的清单项是真声明。
//
// 线的原语(core/src/link/port_ops.hpp 的 GpioLineDriver):
//   configure()  写引脚寄存器: 输出 = 定时器复用功能、推挽、比较值 0, 首次启动其
//                定时器通道; 输入 = 输入模式 + 上下拉 + 边沿选择(中断屏蔽)。设置没变
//                就什么都不写: 运行期重声明另一个口会重放整份清单, 正在跑的 PWM
//                不能因此掉到 0。
//   resume()     开工: 输入解除边沿屏蔽并开始周期采样; 输出开始接受写入。
//   suspend()    停: 输出的比较值归 0, 引脚保持拉低而不是放高阻(悬空的信号线在
//                电调上是噪声脉冲); 输入屏蔽边沿、停采样; 写入与读请求一律丢弃。
//
// 全部在主循环跑(EP0 处理器、下行回调与会话状态机都经 tud_task() 到达), 例外是
// handle_edge(): 它是 EXTI 中断服务。
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
        // CCR1..CCR4 连续, 通道码(0/4/8/12)右移两位就是偏移 -- __HAL_TIM_SET_COMPARE
        // 的算法, 这里只算一次。
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
            return; // 从未声明: 无从恢复
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

        // 方向切换期间不写入、不响应边沿。
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

    // 写后回读: 方向(MODER)与上下拉(PUPDR)。PUPDR 的编码即 vc::GpioPull 的
    // (00 无, 01 上拉, 10 下拉)。
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
        *compare_register_ = data.high ? kPwmCounterPeriod : 0U;
    }

    void handle_analog_write(const data::GpioAnalogDataView& data) {
        if (!running_ || setting_.mode != vc::kGpioModeOutput)
            return;
        *compare_register_ =
            ((static_cast<uint32_t>(data.value) * kPwmCounterPeriod) + 32767U) / 65535U;
    }

    // kRead: 立即采样一次。上下拉在引脚被声明时已生效, 到主机发来任何请求之间
    // 至少隔着一次控制传输, 没有还在稳定的电平。
    void handle_read_request() {
        if (!running_ || setting_.mode != vc::kGpioModeInput)
            return;
        publish(read_level(), current_timestamp_quarter_us());
    }

    // 主循环: 周期到了采一个样。
    void poll_periodic() {
        if (!running_ || sample_period_ == kNoPeriod)
            return;
        if (!timer::timer->check_reached(next_sample_))
            return;
        publish(read_level(), current_timestamp_quarter_us());
        next_sample_ = timer::timer->timepoint() + sample_period_;
    }

    // EXTI 中断服务。线只在它的引脚声明了边沿且在跑时才解除屏蔽; 这个检查接住
    // 屏蔽那一刻还挂着的那个边沿。
    void handle_edge() {
        if (!running_ || !edges_armed_)
            return;
        const uint32_t timestamp_quarter_us = current_timestamp_quarter_us();
        publish(read_level(), timestamp_quarter_us);
    }

    // 新会话接替旧会话: 旧主机的输出值不继承; 方向与在跑状态按声明保持。
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
    // TIM1 与 TIM8 共用这个周期(ARR + 1), 都是 50 Hz。
    static constexpr uint32_t kPwmCounterPeriod = 60000;

    [[nodiscard]] uint32_t pin_number() const {
        return static_cast<uint32_t>(__builtin_ctz(wiring_.gpio_pin));
    }

    void configure_output() {
        // HAL_GPIO_Init() 只在中断模式下才重写 EXTI 寄存器, 引脚上一次当边沿输入用
        // 的边沿配置会残留。HAL_GPIO_DeInit() 把线解除映射(仅当它映射到本引脚的
        // 端口)并清掉触发; 再清一次已经挂起的边沿。
        HAL_GPIO_DeInit(wiring_.gpio_port, wiring_.gpio_pin);
        __HAL_GPIO_EXTI_CLEAR_IT(wiring_.gpio_pin);

        *compare_register_ = 0; // 从低开始
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
        // 中断模式下 HAL 会解除线的屏蔽; 是否保持由 resume() 决定。
        if (rising || falling) {
            CLEAR_BIT(EXTI->IMR, wiring_.gpio_pin);
            __HAL_GPIO_EXTI_CLEAR_IT(wiring_.gpio_pin);
        }
    }

    void arm_edges() {
        if ((setting_.input_flags & kEdges) == 0U)
            return;
        // 向量在第一个引脚需要它时才使能。EXTI9_5 已被 MX_GPIO_Init 为 BMI088 陀螺仪
        // 使能; 已使能的向量保持其优先级。
        if (NVIC_GetEnableIRQ(wiring_.exti_irq) == 0U) {
            HAL_NVIC_SetPriority(wiring_.exti_irq, 4, 0);
            HAL_NVIC_EnableIRQ(wiring_.exti_irq);
        }
        __HAL_GPIO_EXTI_CLEAR_IT(wiring_.gpio_pin);
        SET_BIT(EXTI->IMR, wiring_.gpio_pin);
        edges_armed_ = true;
    }

    // 只屏蔽本引脚武装过的那条线: 线按引脚号共用(PWM5 PC6 与 PWM6 PI6 都是 6 号
    // 线), 无条件屏蔽会把别的口的边沿一起切掉。
    void disarm_edges() {
        if (!edges_armed_)
            return;
        CLEAR_BIT(EXTI->IMR, wiring_.gpio_pin);
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

    const uint8_t line_;
    const Wiring wiring_;
    volatile uint32_t* const compare_register_;

    // 引脚寄存器里的设置; kGpioModeOff = 从未声明。挂起时保留它: 相同设置的重声明
    // 于是什么都不重写, resume() 也从它出发。
    vc::GpioConfigPayload setting_{};
    bool running_ = false;
    bool channel_started_ = false;
    bool edges_armed_ = false;
    timer::Timer::Duration sample_period_ = kNoPeriod;
    timer::Timer::TimePoint next_sample_{};
};

// GPIO 口的驱动: 七根线。对象在上电时构造(不碰硬件); 线被逐根声明才开工。
class Gpio : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Gpio>;
    using Spec = spec::c_board::Spec;

    // 口的原语(core/src/link/port_ops.hpp 的 GpioPortDriver)。逐线能力是接线事实: PWM6(PI6)
    // 与 PWM5(PC6)共用 EXTI 6 号线, PWM5 有边沿中断, PWM6 就没有。口清单里 GPIO 口的能力
    // 字节是线数。
    static constexpr uint8_t kLineCount = 7;
    static constexpr uint8_t kNoEdge = spec::kGpioCapPwmPin & ~spec::kGpioCapReadInterrupt;
    static constexpr std::array<uint8_t, kLineCount> kLineCapabilities{
        spec::kGpioCapPwmPin, spec::kGpioCapPwmPin, spec::kGpioCapPwmPin,
        spec::kGpioCapPwmPin, spec::kGpioCapPwmPin, kNoEdge,
        spec::kGpioCapPwmPin};
    static constexpr uint8_t kPortCapabilities = kLineCount;
    static_assert(kLineCount == Spec::kGpioLineCount);

    Pin pwm1{
        Spec::Gpios::kPwm1.line,
        {CHANNEL1_GPIO_Port, CHANNEL1_Pin, GPIO_AF1_TIM1, &htim1, TIM_CHANNEL_1, EXTI9_5_IRQn}
    };
    Pin pwm2{
        Spec::Gpios::kPwm2.line,
        {CHANNEL2_GPIO_Port, CHANNEL2_Pin, GPIO_AF1_TIM1, &htim1, TIM_CHANNEL_2, EXTI15_10_IRQn}
    };
    Pin pwm3{
        Spec::Gpios::kPwm3.line,
        {CHANNEL3_GPIO_Port, CHANNEL3_Pin, GPIO_AF1_TIM1, &htim1, TIM_CHANNEL_3, EXTI15_10_IRQn}
    };
    Pin pwm4{
        Spec::Gpios::kPwm4.line,
        {CHANNEL4_GPIO_Port, CHANNEL4_Pin, GPIO_AF1_TIM1, &htim1, TIM_CHANNEL_4, EXTI15_10_IRQn}
    };
    Pin pwm5{
        Spec::Gpios::kPwm5.line,
        {CHANNEL5_GPIO_Port, CHANNEL5_Pin, GPIO_AF3_TIM8, &htim8, TIM_CHANNEL_1, EXTI9_5_IRQn}
    };
    Pin pwm6{
        Spec::Gpios::kPwm6.line,
        {CHANNEL6_GPIO_Port, CHANNEL6_Pin, GPIO_AF3_TIM8, &htim8, TIM_CHANNEL_2, EXTI9_5_IRQn}
    };
    Pin pwm7{
        Spec::Gpios::kPwm7.line,
        {CHANNEL7_GPIO_Port, CHANNEL7_Pin, GPIO_AF3_TIM8, &htim8, TIM_CHANNEL_3, EXTI9_5_IRQn}
    };

    // 第 index 根线; 本口没有这根线时为空(下行记录的线号来自主机, 不可信)。
    [[nodiscard]] Pin* line(uint8_t index) { return index < kLineCount ? pins()[index] : nullptr; }

    // 整口: 会话结束、清单回滚。resume() 只让声明过的线重新开工。
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

    // 新会话接替旧会话: 只清输出值; 引脚照刚接受的清单继续工作。
    void zero_outputs() {
        for (Pin* p : pins())
            p->zero_output();
    }

    // 主循环入口, 只在 loop::active 的 kGpioSampling 位置着时调用: 没有引脚配周期采样时
    // -- 绝大多数接线如此 -- 主循环根本不进来。与 mc02 同法。
    void poll_periodic() {
        for (Pin* p : pins())
            p->poll_periodic();
    }

    // EXTI 回调, 服务不是 BMI088 的那些边沿。线上的每个引脚都被问一遍; 武装过它的
    // 那个才应答(见 Pin::disarm_edges)。
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
    [[nodiscard]] std::array<Pin*, kLineCount> pins() {
        return {&pwm1, &pwm2, &pwm3, &pwm4, &pwm5, &pwm6, &pwm7};
    }
};

inline constinit Gpio::Lazy gpio;

inline void refresh_sampling() {
    if (gpio->any_sampling())
        loop::set(loop::kGpioSampling);
    else
        loop::clear(loop::kGpioSampling);
}

} // namespace libhcs::firmware::gpio
