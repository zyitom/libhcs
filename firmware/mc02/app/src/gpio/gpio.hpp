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

        // Channel EXTI lines: PA0->EXTI0, PA2->EXTI2, PE9->EXTI9_5. EXTI15_10
        // (PE13) is already enabled by MX_GPIO_Init -- it is shared with the
        // BMI088 data-ready pins, so only its handler lives in gpio.cpp.
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

    void handle_digital_read(uint8_t channel_index, const data::GpioReadConfigView& data) {
        configure_digital_input_mode(channel_index, data);

        if (data.asap) {
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
        // The owning timer's ARR. Read at runtime rather than baked into a
        // constant because the two timers behind these four channels no longer
        // have to share a period: TIM2 is also the USB-SOF capture timer
        // (sync/sof.cpp), and making that capture fine enough to be worth using
        // means running TIM2 at 275 MHz with ARR 5499999, while TIM1 stays at
        // 1 MHz with ARR 19999. Both still produce 50 Hz.
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

    void configure_digital_input_mode(uint8_t channel_index, const data::GpioReadConfigView& data) {
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
        }
    }

    void set_pwm_compare(uint8_t channel_index, uint32_t compare) {
        *channel_hardware(channel_index).compare_register = compare;
    }

    void configure_hal_gpio_output(uint8_t channel_index) {
        const auto& hardware = channel_hardware(channel_index);

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

    // 64-bit intermediate: at TIM2's 5500000-count period the product overflows
    // 32 bits at a duty of about 780, which would have silently wrapped.
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
