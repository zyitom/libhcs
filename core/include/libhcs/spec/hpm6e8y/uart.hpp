#pragma once

#include <cstddef>
#include <iterator>

#include <libhcs/data/datas.hpp>
#include <libhcs/spec/uart.hpp>

namespace libhcs::spec::hpm6e8y {

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
    static constexpr UartDescriptor kArray[]{
        UartDescriptor{data::DataId::kUart0, data::DataId::kUart0Config},
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

    static constexpr const UartDescriptor& kUart0 = kArray[0];
};
} // namespace internal

inline constexpr internal::UartDescriptors kUartDescriptors{};

} // namespace libhcs::spec::hpm6e8y
