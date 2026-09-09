#pragma once

#include <libhcs/data/datas.hpp>

namespace libhcs::spec {

struct UartDescriptor {
    constexpr UartDescriptor(data::DataId data_id, data::DataId config_data_id) noexcept
        : data_id(data_id)
        , config_data_id(config_data_id) {}

    data::DataId data_id;
    // Downlink id carrying runtime configuration for this port. Pairing it with
    // data_id here is what lets a caller reconfigure a port it only holds a
    // descriptor for, without a second lookup table.
    data::DataId config_data_id;
};

} // namespace libhcs::spec
