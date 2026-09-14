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

    // 非静态: idProduct 在运行时决定。未识别的板以哨兵 PID 枚举, 使故障在
    // 朴素的 `lsusb` 里可见, 且主机工具不会把设备误认成可刷写的板。
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

    // 用 OTP 板型身份修补描述符中运行时决定的部分。被识别的板与单板型构建
    // 无法区分: 以其 PCB 一直使用的 PID 枚举, 显示普通 bootloader 的 product string。
    //
    // 必须上报板型 PID 而非编译期 PID, 这一点在 bootloader 比在 app 更要紧:
    // 一个 bootloader 二进制同时服务两块 HPM5321 PCB, 而编译期 PID 只能是其中
    // 之一(0x5321)。若维持不变, 双 CAN 板的 DFU 接口会以 0x5321 枚举, 仓库里
    // 所有 `dfu-util -d 0xa511:0x5322` 命令(tools/flash.sh hpm5321、
    // flash-dual-bootloader.sh、BUILD_ENVIRONMENT.md)及以其为键的 udev 规则
    // 全部失效。合并本应对主机侧不可见, PID 正是保证这一点的字段。
    //
    // 未识别的板则刻意醒目: `lsusb` 显示哨兵 PID, product string(`lsusb -v`
    // 可见, 无需任何主机侧支持)携带肇事 word 25 值, 首步诊断只需一根 USB 线。
    // 哨兵 PID 在分配区间之外, 本仓库没有主机工具会匹配它
    // (host/src/transport/usb/device_scanner.hpp 要求 PID 精确匹配), DFU 也会
    // 拒绝下载。
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

private: // 设备描述符
    // 两块 HPM5321 PCB 分配到的 PID, 由 OTP 身份决定上报哪个, 使一个二进制让
    // 每块 PCB 以自己的 PID 枚举。仅在启用身份检查的板上可达; 单板型板解析为
    // BoardVariant::kFixed, 下面 libhcs_USB_PID 保持不动。
    static constexpr uint16_t kSingleCanProductId = 0x5321;
    static constexpr uint16_t kDualCanFdProductId = 0x5322;

    // OTP word 25 非任一已知值时上报的 PID。留在与 5321 PCB 同一个 53xx 段,
    // 但既非 0x5321 也非 0x5322, 仓库内没有主机工具会绑定它。
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

    // 可变副本: 构造函数在未识别板上覆写 idProduct, 其余字段与上面的常量
    // 完全一致。
    tusb_desc_device_t device_descriptor_ = kDeviceDescriptor;

private: // 配置描述符
    static constexpr uint8_t kItfNumDfu = 0U;
    static constexpr uint8_t kItfNumTotal = 1U;
    static constexpr size_t kConfigTotalLen = TUD_CONFIG_DESC_LEN + TUD_DFU_DESC_LEN(1);

    static constexpr uint8_t kConfigurationDescriptorFs[] = {
        TUD_CONFIG_DESCRIPTOR(1, kItfNumTotal, 0, kConfigTotalLen, 0, 100),
        TUD_DFU_DESCRIPTOR(kItfNumDfu, 1, 4, DFU_ATTR_CAN_DOWNLOAD, 1000, CFG_TUD_DFU_XFER_BUFSIZE),
    };
    static_assert(sizeof(kConfigurationDescriptorFs) == kConfigTotalLen);

private: // 字符串描述符
    static constexpr std::array<uint8_t, 2> kLanguageId = {0x09, 0x04};
    static constexpr std::string_view kManufacturerString = "Helios";
    static constexpr std::string_view kProductString = "HCS DFU Bootloader";
    static constexpr std::string_view kAlt0String = "Internal Flash";
    std::array<char, 43> serial_string_{"AF-0000-0000-0000-0000-0000-0000-0000-0000"};
    std::array<uint16_t, 128> descriptor_string_buffer_{};

    // 未识别板的错误 product string。八个 X 由 update_board_identity_strings()
    // 覆写为 word 25 的十六进制; 数组大小取自该字面量的长度, 两者不会失配。
    static constexpr std::string_view kUnknownBoardPrefix = "HCS DFU UNKNOWN BOARD OTP25=0x";
    std::array<char, 39> unknown_board_string_{"HCS DFU UNKNOWN BOARD OTP25=0xXXXXXXXX"};
    static_assert(kUnknownBoardPrefix.size() + 8U == 38U);

    // 实际上报的 product string。被识别时指向常量, 否则指向 unknown_board_string_。
    std::string_view product_string_{kProductString};
};

UsbDescriptors& get_usb_descriptors();

} // namespace libhcs::firmware::usb
