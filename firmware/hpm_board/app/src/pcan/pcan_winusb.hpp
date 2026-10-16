#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace libhcs::firmware::pcan::winusb {

// Independent bridge identity: never bind the proprietary PEAK kernel driver.
inline constexpr uint16_t kVendorId = 0xA511;
inline constexpr uint16_t kSingleProductId = 0xF531;
inline constexpr uint16_t kDualProductId = 0xF532;
inline constexpr uint8_t kVendorCode = 0x21;
inline constexpr std::string_view kInterfaceGuid = "{74CE6BD6-FFAE-49C6-AC5A-BC43F7AE2A19}";

// Microsoft OS 2.0: interface 0 is the bridge; interface 1 is DFU runtime.
// Property names are REG_SZ names (one NUL); their REG_MULTI_SZ values have two.
inline constexpr auto kDescriptorSet = []() consteval {
    constexpr std::string_view name = "DeviceInterfaceGUIDs";
    constexpr std::string_view dfu_guid = "{74CE6BD6-FFAE-49C6-AC5A-BC43F7AE2A1A}";
    constexpr std::size_t name_bytes = (name.size() + 1) * 2;
    constexpr std::size_t value_bytes = (kInterfaceGuid.size() + 2) * 2;
    constexpr std::size_t property_length = 10 + name_bytes + value_bytes;
    constexpr std::size_t function_length = 8 + 20 + property_length;
    constexpr std::size_t total_length = 10 + 8 + 2 * function_length;
    std::array<uint8_t, total_length> out{};
    std::size_t p = 0;
    const auto le16 = [&](uint16_t value) {
        out[p++] = static_cast<uint8_t>(value);
        out[p++] = static_cast<uint8_t>(value >> 8);
    };
    const auto text = [&](std::string_view value, std::size_t nul_count) {
        for (char c : value) {
            out[p++] = static_cast<uint8_t>(c);
            out[p++] = 0;
        }
        for (std::size_t i = 0; i < nul_count; ++i)
            le16(0);
    };
    le16(10);
    le16(0);
    le16(0);
    le16(0x0603);
    le16(static_cast<uint16_t>(total_length));
    le16(8);
    le16(1);
    le16(0);
    le16(static_cast<uint16_t>(total_length - 10));
    for (uint8_t interface = 0; interface < 2; ++interface) {
        le16(8);
        le16(2);
        out[p++] = interface;
        out[p++] = 0;
        le16(static_cast<uint16_t>(function_length));
        le16(20);
        le16(3);
        for (char c : std::string_view{"WINUSB"})
            out[p++] = static_cast<uint8_t>(c);
        for (std::size_t i = 0; i < 10; ++i)
            out[p++] = 0;
        le16(static_cast<uint16_t>(property_length));
        le16(4);
        le16(7);
        le16(static_cast<uint16_t>(name_bytes));
        text(name, 1);
        le16(static_cast<uint16_t>(value_bytes));
        text(interface == 0 ? kInterfaceGuid : dfu_guid, 2);
    }
    return out;
}();

} // namespace libhcs::firmware::pcan::winusb
