#pragma once

#include <array>
#include <cstddef>
#include <iterator>

#include <libhcs/data/datas.hpp>
#include <libhcs/spec/can.hpp>

namespace libhcs::spec::mc02 {

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
        CanDescriptor{data::DataId::kCan3},
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
    static constexpr const CanDescriptor& kCan3 = kArray[2];
};
} // namespace internal

inline constexpr internal::CanDescriptors kCanDescriptors{};

// Port -> wire id, indexed by CanPort. Same numbering as the silkscreen on
// this board: CanPort::kCanN is DataId::kCanN. Spelled as a table because the
// transmit path takes a port as a VALUE, so the mapping must be expressive at
// run time -- the port's name is what the error messages carry, and the wire
// id is what write_can() takes.
inline constexpr std::array<data::DataId, 3> kCanIds{
    data::DataId::kCan1,
    data::DataId::kCan2,
    data::DataId::kCan3,
};

// Port names for diagnostics, indexed the same way.
inline constexpr std::array<const char*, 3> kCanNames{"CAN1", "CAN2", "CAN3"};

} // namespace libhcs::spec::mc02
