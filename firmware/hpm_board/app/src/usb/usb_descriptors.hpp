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
#include <hpm_otp_drv.h>
#include <hpm_soc_feature.h>
#include <tusb_config.h>
#include <tusb_option.h>

#include "core/src/utility/assert.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"
#include "firmware/hpm_board/common/board_identity.hpp"

namespace libhcs::firmware::usb {

class UsbDescriptors {
public:
    // Align endpoint numbering to STM32 HAL style:
    // EP1 OUT: data OUT, EP1 IN: data IN.
    //
    // Public because the endpoint address is not only descriptor content: the
    // vendor transport audits the OUT endpoint's armed state against it, and a
    // second copy of the number is exactly the kind of thing that drifts.
    static constexpr uint8_t kEpnumCdc0DataOut = 0x01;
    static constexpr uint8_t kEpnumCdc0DataIn = 0x81;

    UsbDescriptors() {
        update_serial_string();
        update_product_id();
    }

    // Non-static because idProduct is decided at run time on boards whose image
    // serves two PCBs: the hpm5321 app reports 0x5321 or 0x5322 from the OTP
    // identity, so one binary keeps both boards' existing USB identity and the
    // host needs no change (host/src/transport/usb/device_scanner.hpp matches the
    // PID exactly). Single-variant boards report their compile-time PID.
    uint8_t const* get_device_descriptor() const {
        return reinterpret_cast<uint8_t const*>(&device_descriptor_);
    }

    static uint8_t const* get_configuration_descriptor(uint8_t index) {
        (void)index; // For multiple configurations

        if constexpr (TUD_OPT_HIGH_SPEED)
            return (tud_speed_get() == TUSB_SPEED_HIGH) ? kConfigurationDescriptorHs
                                                        : kConfigurationDescriptorFs;
        else
            return kConfigurationDescriptorFs;
    }

    uint16_t const* get_string_descriptor(uint8_t index, uint16_t langid) {
        (void)langid;
        uint8_t str_size;

        if (index == 0) {
            std::memcpy(&descriptor_string_buffer_[1], kLanguageId.data(), kLanguageId.size());
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
            constexpr auto max_size = std::min<size_t>(
                std::tuple_size_v<decltype(descriptor_string_buffer_)> - 1,
                (std::numeric_limits<uint8_t>::max() - 2) / 2);

            str_size = static_cast<uint8_t>(std::min<size_t>(str.size(), max_size));

            // Convert ASCII string into UTF-16
            for (uint8_t i = 0; i < str_size; i++)
                descriptor_string_buffer_[i + 1] = static_cast<uint16_t>(str[i]);
        }

        // first byte is length (including header), second byte is string type
        descriptor_string_buffer_[0] =
            (TUSB_DESC_STRING << 8) | static_cast<uint16_t>((2 * str_size) + 2);

        return descriptor_string_buffer_.data();
    }

private:
    static constexpr size_t kUuidWordCount = OTP_SOC_UUID_LEN / sizeof(uint32_t);
    static_assert((OTP_SOC_UUID_LEN % sizeof(uint32_t)) == 0);

    void update_serial_string() {
        std::array<uint32_t, kUuidWordCount> uuid{};

        for (size_t i = 0; i < uuid.size(); ++i)
            uuid[i] = otp_read_from_shadow(OTP_SOC_UUID_IDX + static_cast<uint32_t>(i));

        mix_uid_entropy(uuid);

        auto* cursor = serial_string_.data() + 3;
        for (const auto& word : uuid) {
            cursor = write_hex_u16(static_cast<uint16_t>(word >> 16), cursor) + 1;
            cursor = write_hex_u16(static_cast<uint16_t>(word), cursor) + 1;
        }
        core::utility::assert_debug(cursor == serial_string_.data() + serial_string_.size());
    }

    // Report the PID matching the PCB this chip is on. A no-op on single-variant
    // boards, where board::kOtpIdentityEnabled is false and the compile-time PID
    // already in kDeviceDescriptor is correct.
    //
    // The product string is deliberately NOT varied: the host matches it exactly
    // against "HCS Agent v<version>", and the PID is what distinguishes the two
    // boards there. Nothing else in the descriptor set changes, so a board sees
    // byte-identical descriptors to the ones its own build used to emit.
    void update_product_id() {
        if constexpr (!board::kOtpIdentityEnabled)
            return;

        const auto& identity = board::board_identity();

        // The bootloader refuses to jump here unless the identity resolved, so
        // reaching this with kUnknown means the app was started some other way
        // (a debugger, or a bootloader predating the check). Trap in debug; in
        // release keep the compile-time PID rather than inventing one.
        core::utility::assert_debug(identity.recognized());

        if (identity.variant == board::BoardVariant::kSingleCan)
            device_descriptor_.idProduct = kSingleCanProductId;
        else if (identity.variant == board::BoardVariant::kDualCanFd)
            device_descriptor_.idProduct = kDualCanFdProductId;
    }

    static constexpr void mix_uid_entropy(std::array<uint32_t, kUuidWordCount>& uid) {
        static_assert(kUuidWordCount == 4U);

        auto& [a, b, c, d] = uid;

        const auto mix_step = [](uint32_t v) {
            v *= 0x9E3779B9U;
            return v ^ (v >> 16U);
        };

        a ^= mix_step(b ^ c ^ d);
        b ^= mix_step(a ^ c ^ d);
        c ^= mix_step(a ^ b ^ d);
        d ^= mix_step(a ^ b ^ c);

        a ^= mix_step(b + c + d);
        b ^= mix_step(a + c + d);
        c ^= mix_step(a + b + d);
        d ^= mix_step(a + b + c);

        a ^= mix_step((b << 5) ^ (c >> 3) ^ d);
        b ^= mix_step((c << 7) ^ (d >> 5) ^ a);
        c ^= mix_step((d << 11) ^ (a >> 7) ^ b);
        d ^= mix_step((a << 13) ^ (b >> 11) ^ c);

        a += mix_step(b ^ d);
        b += mix_step(c ^ a);
        c += mix_step(d ^ b);
        d += mix_step(a ^ c);
    }

    static char* write_hex_u16(uint16_t value, char* buffer) {
        static constexpr char hex_lut[] = "0123456789ABCDEF";

        *buffer++ = hex_lut[(value >> 12) & 0xF];
        *buffer++ = hex_lut[(value >> 8) & 0xF];
        *buffer++ = hex_lut[(value >> 4) & 0xF];
        *buffer++ = hex_lut[value & 0xF];

        return buffer;
    }

private: // Device Descriptor
    // The two hpm5321 PCBs' allocated PIDs. Same values the separate builds used,
    // so hosts, udev rules and dfu-util command lines are unaffected by the
    // merge; only which one a given binary reports is now decided at run time.
    static constexpr uint16_t kSingleCanProductId = 0x5321;
    // Bus-powered, and NO remote wakeup. The bit used to be set here while the
    // firmware never called tud_remote_wakeup() even once -- a descriptor that
    // claimed a capability the device does not have. A host may legitimately
    // suspend a bus and wait to be woken; advertising wakeup without
    // implementing it is the one way to make that wait never end.
    static constexpr uint8_t kConfigAttributes = 0;

    static constexpr uint16_t kDualCanFdProductId = 0x5322;

    static constexpr tusb_desc_device_t kDeviceDescriptor = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,

        .bDeviceClass = TUSB_CLASS_UNSPECIFIED,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

        .idVendor = 0xA511,
        .idProduct = libhcs_USB_PID,
        .bcdDevice = 0x0300,

        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,

        .bNumConfigurations = 0x01,
    };

    // Mutable copy: the constructor patches idProduct from the board identity.
    // Everything else stays exactly as the constant above declares it.
    tusb_desc_device_t device_descriptor_ = kDeviceDescriptor;

private: // Configuration Descriptor
         // NOLINTNEXTLINE(cppcoreguidelines-use-enum-class)
    enum InterfaceNumber : uint8_t {
        kItfNumVendor = 0,
        kItfNumDfuRuntime,
        kItfNumTotal,
    };

    static constexpr size_t kConfigTotalLen = TUD_CONFIG_DESC_LEN
                                            + CFG_TUD_VENDOR * TUD_VENDOR_DESC_LEN
                                            + CFG_TUD_DFU_RUNTIME * TUD_DFU_RT_DESC_LEN;

    // There is deliberately no second bulk pair. One was added 2026-08-07 to
    // give CAN a pipe of its own and removed 2026-09-05: an idle IN endpoint is
    // polled continuously by the host controller whether or not it carries
    // data, which cost a quarter of the packet rate and one host service
    // interval on the median of every CAN frame -- more than the head-of-line
    // blocking it removed. See firmware/hpm_board/AGENTS.md.

    static constexpr uint8_t const kConfigurationDescriptorFs[] = {
        // Config number, interface count, string index, total length, attribute, power in mA
        TUD_CONFIG_DESCRIPTOR(
            1, kItfNumTotal, 0, kConfigTotalLen, kConfigAttributes, 100),

        // Interface number, string index, EP data address (out, in) and size.
        TUD_VENDOR_DESCRIPTOR(kItfNumVendor, 0, kEpnumCdc0DataOut, kEpnumCdc0DataIn, 64),
        TUD_DFU_RT_DESCRIPTOR(
            kItfNumDfuRuntime, 4, DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_WILL_DETACH, 1000, 1024),
    };
    static_assert(sizeof(kConfigurationDescriptorFs) == kConfigTotalLen);

    static constexpr uint8_t const kConfigurationDescriptorHs[] = {
        // Config number, interface count, string index, total length, attribute, power in mA
        TUD_CONFIG_DESCRIPTOR(
            1, kItfNumTotal, 0, kConfigTotalLen, kConfigAttributes, 100),

        // Interface number, string index, EP data address (out, in) and size.
        TUD_VENDOR_DESCRIPTOR(kItfNumVendor, 0, kEpnumCdc0DataOut, kEpnumCdc0DataIn, 512),
        TUD_DFU_RT_DESCRIPTOR(
            kItfNumDfuRuntime, 4, DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_WILL_DETACH, 1000, 1024),
    };
    static_assert(sizeof(kConfigurationDescriptorHs) == kConfigTotalLen);

private: // String Descriptor
    static constexpr std::array<uint8_t, 2> kLanguageId = {0x09, 0x04};
    static constexpr std::string_view kManufacturerString = "Helios";
    static constexpr std::string_view kProductString =
        "HCS Agent v" libhcs_PROJECT_VERSION_STRING;
    static constexpr std::string_view kDfuRuntimeString = "DFU Runtime";
    std::array<char, 43> serial_string_{"AF-0000-0000-0000-0000-0000-0000-0000-0000"};

    std::array<uint16_t, 128> descriptor_string_buffer_{};
};
inline constinit utility::Lazy<UsbDescriptors> usb_descriptors;

} // namespace libhcs::firmware::usb
