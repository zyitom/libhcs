#pragma once

#include <array>
#include <cstddef>
#include <iterator>

#include <libhcs/data/datas.hpp>
#include <libhcs/spec/can.hpp>

namespace libhcs::spec::ch32_board {

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
// CH32H417 exposes classic bxCAN 2.0B only (no CAN-FD), on the two controllers
// the firmware brings up in firmware/ch32_board/app/src/board_app.hpp. Keep this
// list and that kCanPorts table in the same order: index N here is canN there.
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

    /// Position in this table. ch32_board has no EP0 configuration channel (its UARTs are
    /// configured in the data stream), so unlike the other boards this is not an EP0 index.
    static constexpr std::size_t index_of(const CanDescriptor& descriptor) noexcept {
        return static_cast<std::size_t>(&descriptor - std::begin(kArray));
    }

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

// Port -> wire id and port name, both indexed by CanPort. Same numbering as
// the silkscreen on this board: CanPort::kCanN is DataId::kCanN. Spelled as
// tables because the transmit path takes a port as a VALUE, so the mapping has
// to be expressive at run time.
inline constexpr std::array<data::DataId, 2> kCanIds{
    data::DataId::kCan1,
    data::DataId::kCan2,
};
inline constexpr std::array<const char*, 2> kCanNames{"CAN1", "CAN2"};

} // namespace libhcs::spec::ch32_board
