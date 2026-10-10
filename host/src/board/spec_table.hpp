#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include <libhcs/protocol/usb_identity.hpp>
#include <libhcs/spec/c_board/ports.hpp>
#include <libhcs/spec/hpm5321/ports.hpp>
#include <libhcs/spec/hpm6e8y/ports.hpp>
#include <libhcs/spec/mc02/ports.hpp>
#include <libhcs/spec/port.hpp>

// 板型名 → 运行时板型信息（v4 的 BoardOptions 是运行时对象，不再是模板）。
// 口表与 USB 标识都来自 core 的 Spec 常量：名字（"CAN1"、"DBUS"…）就是
// channel_name() 生成的丝印名，同一份事实不在两处各写一遍。

namespace libhcs::host::board {

struct BoardTypeInfo {
    std::string_view name;
    std::span<const spec::PortDescriptor> ports;
    std::uint16_t vendor_id;
    std::span<const std::uint16_t> product_ids;
};

// 各板型的 PID 表要有静态存储期（span 指着它）。
inline constexpr std::array kHpm5321ProductIds{
    core::protocol::usb_identity::kHpm5321SingleCanProductId,
    core::protocol::usb_identity::kHpm5321DualCanProductId};
inline constexpr std::array kMc02ProductIds{std::uint16_t{0x0723}};   // mc02.hpp 的 kProductId
inline constexpr std::array kCBoardProductIds{std::uint16_t{0xF407}}; // c_board.hpp 的 kProductId
inline constexpr std::array kHpm6e8yProductIds{std::uint16_t{0x6E84}};

// BoardOptions.board_type 逐项比对用的全表（顺序即文档里的顺序）。
inline constexpr std::array kBoardTypes{
    BoardTypeInfo{
                  .name = "hpm5321",
                  .ports = spec::hpm5321::Spec::kPorts,
                  .vendor_id = core::protocol::usb_identity::kDmtoolVendorId,
                  .product_ids = kHpm5321ProductIds},
    BoardTypeInfo{
                  .name = "mc02",
                  .ports = spec::mc02::Spec::kPorts,
                  .vendor_id = 0xA511,
                  .product_ids = kMc02ProductIds   },
    BoardTypeInfo{
                  .name = "c_board",
                  .ports = spec::c_board::Spec::kPorts,
                  .vendor_id = 0xA511,
                  .product_ids = kCBoardProductIds },
    BoardTypeInfo{
                  .name = "hpm6e8y",
                  .ports = spec::hpm6e8y::Spec::kPorts,
                  .vendor_id = core::protocol::usb_identity::kDmtoolVendorId,
                  .product_ids = kHpm6e8yProductIds},
};

/// 按板型名查表；查不到返回 nullptr。
[[nodiscard]] constexpr const BoardTypeInfo* find_board_type(std::string_view name) noexcept {
    for (const auto& type : kBoardTypes)
        if (type.name == name)
            return &type;
    return nullptr;
}

/// 在一块板型的口表里按丝印名 + 类型找口；找不到为空。名字就是 add_can /
/// add_uart 的 port_name（spec::channel_name 生成的那些）。
[[nodiscard]] constexpr std::optional<spec::PortDescriptor>
    find_port(const BoardTypeInfo& board, std::string_view name, spec::PortKind kind) noexcept {
    for (const auto& port : board.ports) {
        if (port.kind != kind)
            continue;
        if (spec::channel_name(port.data_id) == name)
            return port;
    }
    return std::nullopt;
}

} // namespace libhcs::host::board
