#pragma once

#include <libhcs/data/datas.hpp>

namespace libhcs::spec {

struct CanDescriptor {
    constexpr explicit CanDescriptor(data::DataId data_id) noexcept
        : data_id(data_id) {}

    data::DataId data_id;
};

} // namespace libhcs::spec
