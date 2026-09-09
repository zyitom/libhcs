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

#include "firmware/hpm_board/bootloader/src/utility/assert.hpp"
#include "firmware/hpm_board/common/board_identity.hpp"

namespace libhcs::firmware::usb {

class UsbDescriptors {
public:
    UsbDescriptors() {
        update_serial_string();
        update_board_identity_strings();
    }

    // Non-static because idProduct is decided at run time: an unrecognized board
    // enumerates under a sentinel PID so the fault is visible in plain `lsusb`
    // and no host tool mistakes the device for a flashable board.
    uint8_t const* get_device_descriptor() const {
        return reinterpret_cast<uint8_t const*>(&device_descriptor_);
    }

    static uint8_t const* get_configuration_descriptor(uint8_t index) {
        (void)index;
        return kConfigurationDescriptorFs;
    }

    uint16_t const* get_string_descriptor(uint8_t index, uint16_t langid) {
        (void)langid;
        uint8_t str_size = 0U;

        if (index == 0U) {
            std::memcpy(&descriptor_string_buffer_[1], kLanguageId.data(), kLanguageId.size());
            str_size = 1U;
        } else {
            std::string_view str;
            switch (index) {
            case 1: str = kManufacturerString; break;
            case 2: str = product_string_; break;
            case 3:
                str = std::string_view{serial_string_.data(), serial_string_.size() - 1U};
                break;
            case 4: str = kAlt0String; break;
            default: return nullptr;
            }

            constexpr auto max_size = std::min<size_t>(
                std::tuple_size_v<decltype(descriptor_string_buffer_)> - 1U,
                (std::numeric_limits<uint8_t>::max() - 2U) / 2U);
            str_size = static_cast<uint8_t>(std::min<size_t>(str.size(), max_size));

            for (uint8_t i = 0U; i < str_size; ++i)
                descriptor_string_buffer_[i + 1U] = static_cast<uint16_t>(str[i]);
        }

        descriptor_string_buffer_[0] =
            (TUSB_DESC_STRING << 8) | static_cast<uint16_t>((2U * str_size) + 2U);
        return descriptor_string_buffer_.data();
    }

private:
    static constexpr size_t kUuidWordCount = OTP_SOC_UUID_LEN / sizeof(uint32_t);
    static_assert((OTP_SOC_UUID_LEN % sizeof(uint32_t)) == 0U);

    void update_serial_string() {
        std::array<uint32_t, kUuidWordCount> uuid{};

        for (size_t i = 0U; i < uuid.size(); ++i)
            uuid[i] = otp_read_from_shadow(OTP_SOC_UUID_IDX + static_cast<uint32_t>(i));

        mix_uid_entropy(uuid);

        auto* cursor = serial_string_.data() + 3;
        for (const auto& word : uuid) {
            cursor = write_hex_u16(static_cast<uint16_t>(word >> 16U), cursor) + 1;
            cursor = write_hex_u16(static_cast<uint16_t>(word), cursor) + 1;
        }
        utility::assert_debug(cursor == serial_string_.data() + serial_string_.size());
    }

    // Patch the runtime-decided half of the descriptors from the OTP board
    // identity. A recognized board is indistinguishable from the old separate
    // builds: it enumerates under the PID its own PCB has always used, and shows
    // the plain bootloader product string.
    //
    // Reporting the variant PID rather than the compile-time one matters here,
    // not just in the app. One bootloader binary now serves both HPM5321 PCBs,
    // and its compile-time PID can only be one of them (0x5321). Leaving it at
    // that would make a dual board enumerate its DFU interface as 0x5321, which
    // breaks every `dfu-util -d 0xa511:0x5322` command line in the repo
    // (flash-dual.sh, flash-dual-bootloader.sh, BUILD_ENVIRONMENT.md) and any
    // udev rule keyed on it. The merge is supposed to be invisible from the host
    // side; the PID is the one field that makes that true.
    //
    // An unrecognized board deliberately becomes conspicuous instead. `lsusb`
    // shows the sentinel PID, and the product string -- which `lsusb -v` prints
    // and which needs no host-side support at all -- carries the offending word
    // 25 value, so the first diagnostic step needs nothing but a USB cable. The
    // sentinel is outside the allocated PID range, so no host tool in this repo
    // matches it (host/src/transport/usb/device_scanner.hpp requires an exact
    // PID) and DFU refuses the download besides.
    void update_board_identity_strings() {
        const auto& identity = board::board_identity();
        if (identity.recognized()) {
            if (identity.variant == board::BoardVariant::kSingleCan)
                device_descriptor_.idProduct = kSingleCanProductId;
            else if (identity.variant == board::BoardVariant::kDualCanFd)
                device_descriptor_.idProduct = kDualCanFdProductId;
            return;
        }

        device_descriptor_.idProduct = kUnknownBoardProductId;

        auto* cursor = unknown_board_string_.data() + kUnknownBoardPrefix.size();
        cursor = write_hex_u16(static_cast<uint16_t>(identity.otp_word >> 16U), cursor);
        cursor = write_hex_u16(static_cast<uint16_t>(identity.otp_word), cursor);
        utility::assert_debug(
            cursor == unknown_board_string_.data() + unknown_board_string_.size() - 1U);

        product_string_ =
            std::string_view{unknown_board_string_.data(), unknown_board_string_.size() - 1U};
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

        *buffer++ = hex_lut[(value >> 12U) & 0xFU];
        *buffer++ = hex_lut[(value >> 8U) & 0xFU];
        *buffer++ = hex_lut[(value >> 4U) & 0xFU];
        *buffer++ = hex_lut[value & 0xFU];
        return buffer;
    }

private: // Device Descriptor
    // The two HPM5321 PCBs' allocated PIDs, reported from the OTP identity so one
    // binary enumerates each PCB under its own PID. Only reachable on a board
    // that enables the identity check; a single-variant board resolves to
    // BoardVariant::kFixed and keeps libhcs_USB_PID below untouched.
    static constexpr uint16_t kSingleCanProductId = 0x5321;
    static constexpr uint16_t kDualCanFdProductId = 0x5322;

    // PID reported when OTP word 25 held neither known value. Stays in the 53xx
    // block with the 5321 PCBs but is neither 0x5321 nor 0x5322, so no host tool
    // in this repo will bind to it.
    static constexpr uint16_t kUnknownBoardProductId = 0x53FF;

    static constexpr tusb_desc_device_t kDeviceDescriptor = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = 0x00,
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

    // Mutable copy: the constructor overrides idProduct on an unrecognized
    // board. Everything else stays exactly as the constant above declares it.
    tusb_desc_device_t device_descriptor_ = kDeviceDescriptor;

private: // Configuration Descriptor
    static constexpr uint8_t kItfNumDfu = 0U;
    static constexpr uint8_t kItfNumTotal = 1U;
    static constexpr size_t kConfigTotalLen = TUD_CONFIG_DESC_LEN + TUD_DFU_DESC_LEN(1);

    static constexpr uint8_t kConfigurationDescriptorFs[] = {
        TUD_CONFIG_DESCRIPTOR(1, kItfNumTotal, 0, kConfigTotalLen, 0, 100),
        TUD_DFU_DESCRIPTOR(kItfNumDfu, 1, 4, DFU_ATTR_CAN_DOWNLOAD, 1000, CFG_TUD_DFU_XFER_BUFSIZE),
    };
    static_assert(sizeof(kConfigurationDescriptorFs) == kConfigTotalLen);

private: // String Descriptor
    static constexpr std::array<uint8_t, 2> kLanguageId = {0x09, 0x04};
    static constexpr std::string_view kManufacturerString = "Helios";
    static constexpr std::string_view kProductString = "HCS DFU Bootloader";
    static constexpr std::string_view kAlt0String = "Internal Flash";
    std::array<char, 43> serial_string_{"AF-0000-0000-0000-0000-0000-0000-0000-0000"};
    std::array<uint16_t, 128> descriptor_string_buffer_{};

    // Error product string for an unrecognized board. The eight X's are
    // overwritten with word 25 in hex by update_board_identity_strings(); the
    // literal's length is what sizes the array, so the two cannot drift apart.
    static constexpr std::string_view kUnknownBoardPrefix = "HCS DFU UNKNOWN BOARD OTP25=0x";
    std::array<char, 39> unknown_board_string_{"HCS DFU UNKNOWN BOARD OTP25=0xXXXXXXXX"};
    static_assert(kUnknownBoardPrefix.size() + 8U == 38U);

    // Product string actually reported. Points at the constant on a recognized
    // board and at unknown_board_string_ otherwise.
    std::string_view product_string_{kProductString};
};

UsbDescriptors& get_usb_descriptors();

} // namespace libhcs::firmware::usb
