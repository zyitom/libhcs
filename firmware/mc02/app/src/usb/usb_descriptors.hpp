#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string_view>
#include <tuple>

#include <class/dfu/dfu.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <main.h>
#include <tusb_config.h>

#include "core/src/utility/assert.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

namespace libhcs::firmware::usb {

class UsbDescriptors {
public:
    UsbDescriptors() { compute_serial(); }

    const uint8_t* get_device_descriptor() const {
        return reinterpret_cast<const uint8_t*>(&device_descriptor_);
    }

    static const uint8_t* get_configuration_descriptor(uint8_t /*index*/) {
        return kConfigurationDescriptor;
    }

    uint16_t const* get_string_descriptor(uint8_t index, uint16_t langid) {
        (void)langid;
        uint8_t str_size;

        if (index == 0) {
            std::memcpy(&string_buffer_[1], kLanguageId.data(), kLanguageId.size());
            str_size = 1;
        } else {
            std::string_view str;
            switch (index) {
            case 1: str = kManufacturerString; break;
            case 2: str = kProductString; break;
            case 3: str = std::string_view{serial_string_.data(), serial_string_.size() - 1}; break;
            case 4: str = kDfuRuntimeString; break;
            default: return nullptr;
            }

            constexpr auto kMaxSize = std::min<size_t>(
                std::tuple_size_v<decltype(string_buffer_)> - 1,
                (std::numeric_limits<uint8_t>::max() - 2) / 2);

            str_size = static_cast<uint8_t>(std::min<size_t>(str.size(), kMaxSize));
            for (uint8_t i = 0; i < str_size; ++i)
                string_buffer_[i + 1] = static_cast<uint16_t>(str[i]);
        }

        string_buffer_[0] =
            (TUSB_DESC_STRING << 8) | static_cast<uint16_t>((2u * str_size) + 2u);
        return string_buffer_.data();
    }

private:
    void compute_serial() {
        // idProduct 是 mc02 板型的固定 PID(0x0723), 与 bootloader 及 DFU 镜像
        // 后缀一致。每台设备的区分信息在下方序列号中。
        std::array<uint32_t, 3> uid{HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
        mix_uid_entropy(uid);

        auto* cursor = serial_string_.data() + 3;
        for (const auto& word : uid) {
            cursor = write_hex_u16(static_cast<uint16_t>(word >> 16u), cursor) + 1;
            cursor = write_hex_u16(static_cast<uint16_t>(word), cursor) + 1;
        }
        core::utility::assert_debug(cursor == serial_string_.data() + serial_string_.size());
    }

    static constexpr void mix_uid_entropy(std::array<uint32_t, 3>& uid) {
        auto& [a, b, c] = uid;
        const auto mix = [](uint32_t v) { v *= 0x9E3779B9u; return v ^ (v >> 16u); };
        a ^= mix(b ^ c); b ^= mix(a ^ c); c ^= mix(a ^ b);
        a ^= mix(b + c); b ^= mix(a + c); c ^= mix(a + b);
        a ^= mix(b ^ (c >> 5u)); b ^= mix(a ^ (c << 5u)); c ^= mix(a ^ b);
        a += mix(b); b += mix(c); c += mix(a);
    }

    static char* write_hex_u16(uint16_t value, char* buf) {
        static constexpr char kHex[] = "0123456789ABCDEF";
        *buf++ = kHex[(value >> 12u) & 0xFu];
        *buf++ = kHex[(value >> 8u) & 0xFu];
        *buf++ = kHex[(value >> 4u) & 0xFu];
        *buf++ = kHex[value & 0xFu];
        return buf;
    }

private: // 设备描述符
    tusb_desc_device_t device_descriptor_{
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,

        .bDeviceClass = TUSB_CLASS_UNSPECIFIED,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

        .idVendor = 0xA511,
        .idProduct = 0x0723,
        .bcdDevice = 0x0100,

        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,

        .bNumConfigurations = 0x01,
    };

private: // 配置描述符
         // NOLINTNEXTLINE(cppcoreguidelines-use-enum-class)
    enum InterfaceNumber : uint8_t {
        kItfNumVendor = 0,
        kItfNumDfuRuntime,
        kItfNumTotal,
    };

    static constexpr size_t kConfigTotalLen = TUD_CONFIG_DESC_LEN
                                            + CFG_TUD_VENDOR * TUD_VENDOR_DESC_LEN
                                            + CFG_TUD_DFU_RUNTIME * TUD_DFU_RT_DESC_LEN;

    // vendor 批量端点号。usb/vendor.hpp 的下行流控审计钩子要读 OUT 端点的
    // 在途状态, 因此公开(与 hpm_board 的 UsbDescriptors 一致)。
  public:
    static constexpr uint8_t kEpnumVendorDataOut = 0x01;
    static constexpr uint8_t kEpnumVendorDataIn = 0x81;

    static constexpr uint8_t const kConfigurationDescriptor[] = {
        TUD_CONFIG_DESCRIPTOR(
            1, kItfNumTotal, 0, kConfigTotalLen, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
        TUD_VENDOR_DESCRIPTOR(kItfNumVendor, 0, kEpnumVendorDataOut, kEpnumVendorDataIn, 64),
        TUD_DFU_RT_DESCRIPTOR(
            kItfNumDfuRuntime, 4, DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_WILL_DETACH, 1000, 1024),
    };
    static_assert(sizeof(kConfigurationDescriptor) == kConfigTotalLen);

private: // 字符串描述符
    static constexpr std::array<uint8_t, 2> kLanguageId = {0x09, 0x04};
    static constexpr std::string_view kManufacturerString = "Helios";
    static constexpr std::string_view kProductString =
        "HCS Slave v" libhcs_PROJECT_VERSION_STRING;
    static constexpr std::string_view kDfuRuntimeString = "DFU Runtime";
    std::array<char, 33> serial_string_{"D4-0000-0000-0000-0000-0000-0000"};

    std::array<uint16_t, 128> string_buffer_{};
};

inline constinit utility::Lazy<UsbDescriptors> usb_descriptors;

} // namespace libhcs::firmware::usb
