#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <gpio.h>
#include <main.h>
#include <tim.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/spec/mc02/gpio.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/timer/timer.hpp"
#include "firmware/mc02/app/src/usb/helper.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

namespace libhcs::firmware::gpio {

class Gpio : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Gpio>;

    Gpio() {
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) == HAL_OK);
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) == HAL_OK);
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_OK);
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) == HAL_OK);

        // 通道 EXTI 线: PA0->EXTI0、PA2->EXTI2、PE9->EXTI9_5。EXTI15_10(PE13)
        // 已由 MX_GPIO_Init 使能 -- 与 BMI088 数据就绪引脚共用, 故仅其 handler
        // 放在 gpio.cpp。
        HAL_NVIC_SetPriority(EXTI0_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        HAL_NVIC_SetPriority(EXTI2_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(EXTI2_IRQn);
        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

        constexpr data::GpioReadConfigView k_default_input_config{};
        for (const auto& gpio : spec::mc02::kGpioDescriptors) {
            set_pwm_compare(gpio.channel_index, 0);
            configure_digital_input_mode(gpio.channel_index, k_default_input_config);
        }
    }

    void handle_digital_write(uint8_t channel_index, const data::GpioDigitalDataView& data) {
        configure_output_mode(channel_index);
        set_pwm_compare(channel_index, data.high ? pwm_counter_period(channel_index) : 0);
    }

    void handle_analog_write(uint8_t channel_index, const data::GpioAnalogDataView& data) {
        configure_output_mode(channel_index);
        set_pwm_compare(channel_index, duty16_to_pwm_compare(channel_index, data.value));
    }

    // 会话结束(租约过期、USB 挂起或拔出、被新会话取代)时由 usb::Vendor 调用。写这些输出
    // 的主机已经不在, 最后一个占空比不能无限期保持 -- 接电调或电机时那就是一直转。比较值
    // 归零使输出通道恒低、不再有脉冲(数字输出同样拉低), 设备看到的是信号丢失而非一条仍然
    // 有效的旧指令。不切回高阻输入: 悬空的信号线可能被读成杂波脉冲。输入通道的引脚不接
    // 定时器, 写比较值对它们无影响。
    //
    // 只写比较寄存器, 新值在下一个 PWM 周期(50 Hz 下至多 20 ms)生效。刻意不用 UG 立即
    // 更新: 那会复位整个定时器的计数器, 而 TIM2 的计数器同时是 SOF 捕获的时基
    // (sync/sof.cpp)。
    void stop_outputs() {
        for (const auto& gpio : spec::mc02::kGpioDescriptors)
            set_pwm_compare(gpio.channel_index, 0);
    }

    void handle_digital_read(uint8_t channel_index, const data::GpioReadConfigView& data) {
        const bool reconfigured = configure_digital_input_mode(channel_index, data);

        if (data.asap) {
            if (reconfigured)
                wait_pull_settle();
            const auto& hardware = channel_hardware(channel_index);
            const bool high =
                HAL_GPIO_ReadPin(hardware.gpio_port, hardware.gpio_pin) == GPIO_PIN_SET;
            const uint32_t timestamp_quarter_us = current_timestamp_quarter_us();
            publish_digital_input_sample(channel_index, high, timestamp_quarter_us);
        }
    }

    void poll_periodic_input_samples() {
        const auto now = timer::timer->timepoint();

        for (std::size_t channel_index = 0; channel_index < kChannelCount; ++channel_index) {
            auto& state = channel_states_[channel_index];
            if (state.mode != GpioMode::kDigitalInput || state.sample_period == kNoPeriod)
                continue;
            if (!timer::timer->check_reached(state.next_sample_deadline))
                continue;

            const auto& hardware = channel_hardware(static_cast<uint8_t>(channel_index));
            const bool high =
                HAL_GPIO_ReadPin(hardware.gpio_port, hardware.gpio_pin) == GPIO_PIN_SET;
            const uint32_t timestamp_quarter_us = current_timestamp_quarter_us();
            publish_digital_input_sample(
                static_cast<uint8_t>(channel_index), high, timestamp_quarter_us);
            state.next_sample_deadline = now + state.sample_period;
        }
    }

    void handle_input_edge_interrupt(uint16_t gpio_pin) {
        const uint8_t channel_index = channel_index_from_exti_line(exti_line_from_pin(gpio_pin));
        if (channel_index == kInvalidChannelIndex)
            return;

        auto& state = channel_state(channel_index);
        if (state.mode != GpioMode::kDigitalInput || (!state.rising_edge && !state.falling_edge))
            return;

        const uint32_t timestamp_quarter_us = current_timestamp_quarter_us();
        const auto& hardware = channel_hardware(channel_index);
        const bool high = HAL_GPIO_ReadPin(hardware.gpio_port, hardware.gpio_pin) == GPIO_PIN_SET;

        publish_digital_input_sample(channel_index, high, timestamp_quarter_us);
    }

private:
    enum class GpioMode : uint8_t { kOutput = 0, kDigitalInput = 1 };

    struct ChannelState {
        GpioMode mode = GpioMode::kOutput;
        bool rising_edge = false;
        bool falling_edge = false;
        bool capture_timestamp = false;
        data::GpioPull pull = data::GpioPull::kNone;
        timer::Timer::Duration sample_period = timer::Timer::Duration::zero();
        timer::Timer::TimePoint next_sample_deadline;
    };

    struct ChannelHardware {
        GPIO_TypeDef* gpio_port;
        uint16_t gpio_pin;
        uint8_t alternate_function;
        volatile uint32_t* compare_register;
        // 所属定时器的 ARR。运行时读取而非编译期常量: 这四个通道背后的两个定时器
        // 不必再共用周期。TIM2 同时是 USB-SOF 捕获定时器(sync/sof.cpp), 要让
        // 捕获精细到值得使用须跑 275 MHz、ARR 5499999; TIM1 保持 1 MHz、
        // ARR 19999。两者仍都产生 50 Hz。只读不写: TIM2 的 ARR/PSC 只能在 .ioc 里改,
        // 捕获在启动时缓存了二者, 运行时改写 PWM 频率会让 SOF 修正静默算错。
        volatile uint32_t* autoreload_register;
    };
    static constexpr auto kNoPeriod = timer::Timer::Duration::zero();
    static constexpr std::size_t kChannelCount = std::size(spec::mc02::kGpioDescriptors);
    static constexpr uint8_t kInvalidChannelIndex = 0xFFU;

    void configure_output_mode(uint8_t channel_index) {
        auto& state = channel_state(channel_index);
        if (state.mode == GpioMode::kOutput)
            return;

        state.mode = GpioMode::kOutput;
        configure_hal_gpio_output(channel_index);
    }

    // 返回 true 表示硬件被重新配置过(电平/边沿/上拉/采样周期任一变化)。调用方
    // 若要立刻采样, 需先 wait_pull_settle()。
    bool configure_digital_input_mode(uint8_t channel_index, const data::GpioReadConfigView& data) {
        auto& state = channel_state(channel_index);

        auto rising_edge = data.rising_edge;
        auto falling_edge = data.falling_edge;
        const auto pull = data.pull;
        const auto sample_period =
            (data.period_ms == 0)
                ? kNoPeriod
                : timer::Timer::to_duration_checked(std::chrono::milliseconds{data.period_ms});

        if (state.mode != GpioMode::kDigitalInput //
            || state.rising_edge != rising_edge || state.falling_edge != falling_edge
            || state.capture_timestamp != data.capture_timestamp || state.pull != pull
            || state.sample_period != sample_period) {

            state.mode = GpioMode::kDigitalInput;
            state.rising_edge = rising_edge;
            state.falling_edge = falling_edge;
            state.capture_timestamp = data.capture_timestamp;
            state.pull = pull;
            state.sample_period = sample_period;
            state.next_sample_deadline = timer::timer->timepoint();

            configure_hal_gpio_input(channel_index, rising_edge, falling_edge, pull);
            return true;
        }
        return false;
    }

    // 上拉/下拉翻转后, 内部 ~40 kOhm 拉阻给走线电容充放电需要数微秒; 重配后立即
    // 采样读到的是翻转前的电平(实测 2026-09-18: 上拉重配后的 asap 采样稳定读回
    // 旧的低电平)。只在重配后的 asap 路径上等待, 周期采样与边沿捕获不在重配
    // 瞬间读引脚, 不受影响。主循环上下文, 100 us 一次性的代价可忽略。
    static void wait_pull_settle() {
        const auto deadline = timer::timer->timepoint()
                            + timer::Timer::to_duration_checked(std::chrono::microseconds{100});
        while (!timer::timer->check_reached(deadline)) {}
    }

    void set_pwm_compare(uint8_t channel_index, uint32_t compare) {
        *channel_hardware(channel_index).compare_register = compare;
    }

    void configure_hal_gpio_output(uint8_t channel_index) {
        const auto& hardware = channel_hardware(channel_index);

        // HAL_GPIO_Init() 只在新模式带中断时才改 EXTI 寄存器: 以边沿触发读过的通道切到
        // 输出后, 边沿中断仍然使能, PWM 每个边沿都白进一次 EXTI 中断(PE13 与 BMI088
        // 数据就绪共用 EXTI15_10)。HAL_GPIO_DeInit() 解除该线的映射与使能, 再清掉可能
        // 已挂起的标志。state.mode 已先置为输出, 其间进来的中断会被丢弃。
        HAL_GPIO_DeInit(hardware.gpio_port, hardware.gpio_pin);
        __HAL_GPIO_EXTI_CLEAR_IT(hardware.gpio_pin);

        GPIO_InitTypeDef gpio_init = {};
        gpio_init.Pin = hardware.gpio_pin;
        gpio_init.Mode = GPIO_MODE_AF_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        gpio_init.Alternate = hardware.alternate_function;
        HAL_GPIO_Init(hardware.gpio_port, &gpio_init);
    }

    void configure_hal_gpio_input(
        uint8_t channel_index, bool rising_edge, bool falling_edge, data::GpioPull pull) {
        const auto& hardware = channel_hardware(channel_index);

        GPIO_InitTypeDef gpio_init = {};
        gpio_init.Pin = hardware.gpio_pin;
        if (rising_edge && falling_edge) {
            gpio_init.Mode = GPIO_MODE_IT_RISING_FALLING;
        } else if (rising_edge) {
            gpio_init.Mode = GPIO_MODE_IT_RISING;
        } else if (falling_edge) {
            gpio_init.Mode = GPIO_MODE_IT_FALLING;
        } else {
            gpio_init.Mode = GPIO_MODE_INPUT;
        }
        gpio_init.Pull = hal_gpio_pull(pull);
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(hardware.gpio_port, &gpio_init);
    }

    void publish_digital_input_sample(
        uint8_t channel_index, bool high, uint32_t timestamp_quarter_us) {
        const auto& state = channel_state(channel_index);
        const std::optional<uint32_t> timestamp_to_publish =
            state.capture_timestamp ? std::optional<uint32_t>{timestamp_quarter_us} : std::nullopt;

        auto& serializer = usb::get_serializer();
        core::utility::assert_debug(
            serializer.write_gpio_digital_value(
                channel_index, {.high = high, .timestamp_quarter_us = timestamp_to_publish})
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    [[nodiscard]] static uint32_t current_timestamp_quarter_us() {
        return timer::timer->timepoint().time_since_epoch().count();
    }

    static uint8_t exti_line_from_pin(uint16_t gpio_pin) {
        core::utility::assert_debug(gpio_pin != 0);

        uint8_t line = 0;
        while ((gpio_pin & 0x1U) == 0U) {
            gpio_pin >>= 1;
            ++line;
        }

        return line;
    }

    static uint8_t channel_index_from_exti_line(uint8_t exti_line) {
        switch (exti_line) {
        case 0: return spec::mc02::kGpioDescriptors.kPwm1.channel_index;  // PA0
        case 2: return spec::mc02::kGpioDescriptors.kPwm2.channel_index;  // PA2
        case 9: return spec::mc02::kGpioDescriptors.kPwm3.channel_index;  // PE9
        case 13: return spec::mc02::kGpioDescriptors.kPwm4.channel_index; // PE13
        default: return kInvalidChannelIndex;
        }
    }

    ChannelState& channel_state(uint8_t channel_index) {
        const auto index = static_cast<std::size_t>(channel_index);
        core::utility::assert_debug(index < kChannelCount);
        return channel_states_[index];
    }

    const ChannelHardware& channel_hardware(uint8_t channel_index) const {
        const auto index = static_cast<std::size_t>(channel_index);
        core::utility::assert_debug(index < kChannelCount);
        return channel_hardware_[index];
    }

    uint32_t pwm_counter_period(uint8_t channel_index) const {
        return *channel_hardware(channel_index).autoreload_register + 1U;
    }

    // 64 位中间量: TIM2 周期为 5500000 计数时, 乘积在 duty 约 780 处溢出 32 位,
    // 会静默回绕。
    uint32_t duty16_to_pwm_compare(uint8_t channel_index, uint16_t duty) const {
        const uint64_t period = pwm_counter_period(channel_index);
        return static_cast<uint32_t>(((static_cast<uint64_t>(duty) * period) + 32767U) / 65535U);
    }

    static uint32_t hal_gpio_pull(data::GpioPull pull) {
        switch (pull) {
        case data::GpioPull::kNone: return GPIO_NOPULL;
        case data::GpioPull::kUp: return GPIO_PULLUP;
        case data::GpioPull::kDown: return GPIO_PULLDOWN;
        default: core::utility::assert_failed_debug(); return GPIO_NOPULL;
        }
    }

    const ChannelHardware channel_hardware_[kChannelCount]{
        {
         .gpio_port = GPIOA,
         .gpio_pin = GPIO_PIN_0,
         .alternate_function = GPIO_AF1_TIM2,
         .compare_register = &htim2.Instance->CCR1,
         .autoreload_register = &htim2.Instance->ARR,
         }, // ch0 PA0 TIM2_CH1 (EXTI0)
        {
         .gpio_port = GPIOA,
         .gpio_pin = GPIO_PIN_2,
         .alternate_function = GPIO_AF1_TIM2,
         .compare_register = &htim2.Instance->CCR3,
         .autoreload_register = &htim2.Instance->ARR,
         }, // ch1 PA2 TIM2_CH3 (EXTI2)
        {
         .gpio_port = GPIOE,
         .gpio_pin = GPIO_PIN_9,
         .alternate_function = GPIO_AF1_TIM1,
         .compare_register = &htim1.Instance->CCR1,
         .autoreload_register = &htim1.Instance->ARR,
         }, // ch2 PE9 TIM1_CH1 (EXTI9)
        {
         .gpio_port = GPIOE,
         .gpio_pin = GPIO_PIN_13,
         .alternate_function = GPIO_AF1_TIM1,
         .compare_register = &htim1.Instance->CCR3,
         .autoreload_register = &htim1.Instance->ARR,
         }, // ch3 PE13 TIM1_CH3 (EXTI13)
    };

    ChannelState channel_states_[kChannelCount];
};

inline constinit Gpio::Lazy gpio;

} // namespace libhcs::firmware::gpio
