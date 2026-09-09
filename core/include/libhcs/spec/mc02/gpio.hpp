#pragma once

#include <cstddef>
#include <cstdint>
#include <iterator>

#include <libhcs/spec/gpio.hpp>

namespace libhcs::spec::mc02 {

namespace internal {
class GpioDescriptors;
}

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class GpioDescriptor : public spec::GpioDescriptor {
    friend internal::GpioDescriptors;
    constexpr GpioDescriptor(uint8_t channel_index, GpioCapability capability_mask)
        : spec::GpioDescriptor(channel_index, capability_mask) {}

public:
    GpioDescriptor(const GpioDescriptor&) = delete;
    GpioDescriptor& operator=(const GpioDescriptor&) = delete;
    GpioDescriptor(GpioDescriptor&&) = delete;
    GpioDescriptor& operator=(GpioDescriptor&&) = delete;

    [[nodiscard]] constexpr bool operator==(const GpioDescriptor& other) const noexcept {
        return channel_index == other.channel_index;
    }
};

namespace internal {
class GpioDescriptors {
    // mc02 exposes four pins that, like c_board, can each act in three roles:
    // digital output, PWM/analog output, or digital input (level/periodic/edge,
    // with pull and timestamp). All four have a free EXTI line, so all advertise
    // the full PWM + read capability set.
    static constexpr GpioDescriptor kArray[]{
        {0, kPwmCapabilities}, // TIM2 CH1 (PA0), EXTI0
        {1, kPwmCapabilities}, // TIM2 CH3 (PA2), EXTI2
        {2, kPwmCapabilities}, // TIM1 CH1 (PE9), EXTI9
        {3, kPwmCapabilities}  // TIM1 CH3 (PE13), EXTI13
    };
    static_assert(channel_indices_match_indices(kArray));

public:
    constexpr GpioDescriptors() = default;

    static constexpr std::size_t size() noexcept { return std::size(kArray); }

    static constexpr const GpioDescriptor& operator[](std::size_t channel_index) noexcept {
        return kArray[channel_index];
    }

    static constexpr const GpioDescriptor* begin() noexcept { return std::begin(kArray); }

    static constexpr const GpioDescriptor* end() noexcept { return std::end(kArray); }

    static constexpr const GpioDescriptor& kPwm1 = kArray[0];
    static constexpr const GpioDescriptor& kPwm2 = kArray[1];
    static constexpr const GpioDescriptor& kPwm3 = kArray[2];
    static constexpr const GpioDescriptor& kPwm4 = kArray[3];
};
} // namespace internal

inline constexpr internal::GpioDescriptors kGpioDescriptors{};

} // namespace libhcs::spec::mc02
