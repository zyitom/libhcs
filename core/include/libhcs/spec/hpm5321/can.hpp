#pragma once

#include <cstddef>
#include <iterator>

#include <libhcs/data/datas.hpp>
#include <libhcs/spec/can.hpp>

// One firmware image serves both hpm5321 PCBs: the single-CAN board (PID
// 0x5321) and the dual-CAN one (0x5322), which the board tells apart from OTP
// word 25. This table is therefore the IMAGE's capacity, not any one board's --
// how many buses are really present is read over EP0 at connect time and lives
// in board::hcs::Interface::can_count.
namespace libhcs::spec::hpm5321 {

namespace internal {
class CanDescriptors;
}

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class CanDescriptor : public spec::CanDescriptor {
    friend internal::CanDescriptors;
    constexpr explicit CanDescriptor(data::DataId data_id)
        : spec::CanDescriptor(data_id) {}

public:
    CanDescriptor(const CanDescriptor&) = delete;
    CanDescriptor& operator=(const CanDescriptor&) = delete;
    CanDescriptor(CanDescriptor&&) = delete;
    CanDescriptor& operator=(CanDescriptor&&) = delete;

    [[nodiscard]] constexpr bool operator==(const CanDescriptor& other) const noexcept {
        return data_id == other.data_id;
    }
};

namespace internal {
class CanDescriptors {
    static constexpr CanDescriptor kArray[]{
        CanDescriptor{data::DataId::kCan1},
        CanDescriptor{data::DataId::kCan2},
    };

public:
    constexpr CanDescriptors() = default;

    static constexpr std::size_t size() noexcept { return std::size(kArray); }

    static constexpr const CanDescriptor& operator[](std::size_t index) noexcept {
        return kArray[index];
    }

    static constexpr const CanDescriptor* begin() noexcept { return std::begin(kArray); }

    static constexpr const CanDescriptor* end() noexcept { return std::end(kArray); }

    static constexpr const CanDescriptor* find(data::DataId data_id) noexcept {
        for (const auto& descriptor : kArray) {
            if (descriptor.data_id == data_id)
                return &descriptor;
        }
        return nullptr;
    }

    static constexpr const CanDescriptor& kCan1 = kArray[0];
    static constexpr const CanDescriptor& kCan2 = kArray[1];
};
} // namespace internal

inline constexpr internal::CanDescriptors kCanDescriptors{};

} // namespace libhcs::spec::hpm5321
