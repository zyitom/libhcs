#pragma once

#include <cstddef>
#include <iterator>

#include <libhcs/data/datas.hpp>
#include <libhcs/spec/uart.hpp>

namespace libhcs::spec::ch32_board {

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
// The two USARTs brought up in firmware/ch32_board/app/src/board_app.hpp, in the
// same order (index N here is uartN there).
//
// The config ids address runtime reconfiguration: the firmware routes them to
// Uart::handle_config, which reprograms the port's baudrate away from the
// power-on value its kUartPorts entry fixes. Reached from the host through
// Ch32Board::PacketBuilder::uartN_config.
class UartDescriptors {
    static constexpr UartDescriptor kArray[]{
        UartDescriptor{data::DataId::kUart1, data::DataId::kUart1Config},
        UartDescriptor{data::DataId::kUart2, data::DataId::kUart2Config},
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

    static constexpr const UartDescriptor& kUart1 = kArray[0];
    static constexpr const UartDescriptor& kUart2 = kArray[1];
};
} // namespace internal

inline constexpr internal::UartDescriptors kUartDescriptors{};

} // namespace libhcs::spec::ch32_board
