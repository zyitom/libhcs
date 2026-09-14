#pragma once

#include <cstddef>
#include <iterator>

#include <libhcs/data/datas.hpp>
#include <libhcs/spec/uart.hpp>

namespace libhcs::spec::mc02 {

namespace internal {
class UartDescriptors;
}

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class UartDescriptor : public spec::UartDescriptor {
    friend internal::UartDescriptors;
    constexpr UartDescriptor(data::DataId data_id, data::DataId config_data_id)
        : spec::UartDescriptor(data_id, config_data_id) {}

public:
    UartDescriptor(const UartDescriptor&) = delete;
    UartDescriptor& operator=(const UartDescriptor&) = delete;
    UartDescriptor(UartDescriptor&&) = delete;
    UartDescriptor& operator=(UartDescriptor&&) = delete;

    [[nodiscard]] constexpr bool operator==(const UartDescriptor& other) const noexcept {
        return data_id == other.data_id;
    }
};

namespace internal {
class UartDescriptors {
    // Enclosure silkscreen: UART1 / UART2 / UART3 / UART7 / UART10 plus DBUS.
    // UART2 and UART3 are the RS-485 transceivers (USART2 / USART3).
    static constexpr UartDescriptor kArray[]{
        UartDescriptor{data::DataId::kUartDbus, data::DataId::kUartDbusConfig},
        UartDescriptor{   data::DataId::kUart1,    data::DataId::kUart1Config},
        UartDescriptor{   data::DataId::kUart2,    data::DataId::kUart2Config},
        UartDescriptor{   data::DataId::kUart3,    data::DataId::kUart3Config},
        UartDescriptor{   data::DataId::kUart7,    data::DataId::kUart7Config},
        UartDescriptor{  data::DataId::kUart10,   data::DataId::kUart10Config},
    };

public:
    constexpr UartDescriptors() = default;

    static constexpr std::size_t size() noexcept { return std::size(kArray); }

    static constexpr const UartDescriptor& operator[](std::size_t index) noexcept {
        return kArray[index];
    }

    static constexpr const UartDescriptor* begin() noexcept { return std::begin(kArray); }

    static constexpr const UartDescriptor* end() noexcept { return std::end(kArray); }

    static constexpr const UartDescriptor* find(data::DataId data_id) noexcept {
        for (const auto& descriptor : kArray) {
            if (descriptor.data_id == data_id)
                return &descriptor;
        }
        return nullptr;
    }

    static constexpr const UartDescriptor& kDbus = kArray[0];
    static constexpr const UartDescriptor& kUart1 = kArray[1];
    static constexpr const UartDescriptor& kUart2 = kArray[2];
    static constexpr const UartDescriptor& kUart3 = kArray[3];
    static constexpr const UartDescriptor& kUart7 = kArray[4];
    static constexpr const UartDescriptor& kUart10 = kArray[5];
};
} // namespace internal

inline constexpr internal::UartDescriptors kUartDescriptors{};

} // namespace libhcs::spec::mc02
