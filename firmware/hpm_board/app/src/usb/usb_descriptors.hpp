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
    // 端点编号对齐 STM32 HAL 风格: EP1 OUT 为数据 OUT, EP1 IN 为数据 IN。
    //
    // 公开的原因: 端点地址不只是描述符内容, vendor 传输要拿它审计 OUT 端点的挂载
    // 状态; 第二份编号正是会漂移的那种东西。
    static constexpr uint8_t kEpnumCdc0DataOut = 0x01;
    static constexpr uint8_t kEpnumCdc0DataIn = 0x81;

    UsbDescriptors() {
        update_serial_string();
        update_product_id();
    }

    // 非 static: 镜像服务两块 PCB 的板子上 idProduct 由运行时决定 -- hpm5321 应用
    // 按 OTP 标识上报 0x5321 或 0x5322, 一个二进制保住两块板既有的 USB 身份, 主机
    // 无需改动(host/src/transport/usb/device_scanner.hpp 精确匹配 PID)。单形态
    // 板子上报编译期 PID。
    uint8_t const* get_device_descriptor() const {
        return reinterpret_cast<uint8_t const*>(&device_descriptor_);
    }

    static uint8_t const* get_configuration_descriptor(uint8_t index) {
        (void)index; // 多配置预留

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

            // ASCII 字符串转 UTF-16
            for (uint8_t i = 0; i < str_size; i++)
                descriptor_string_buffer_[i + 1] = static_cast<uint16_t>(str[i]);
        }

        // 首字节为长度(含头部), 次字节为描述符类型
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

    // 上报与芯片所在 PCB 匹配的 PID。单形态板子上是空操作: board::kOtpIdentityEnabled
    // 为 false, kDeviceDescriptor 里的编译期 PID 本就正确。
    //
    // 产品字符串刻意不随板变化: 主机对 "HCS Agent v<version>" 精确匹配, 区分两块
    // 板靠的是 PID。描述符集其余部分不变, 板子看到的字节与原独立构建的产出完全
    // 一致。
    void update_product_id() {
        if constexpr (!board::kOtpIdentityEnabled)
            return;

        const auto& identity = board::board_identity();

        // bootloader 在标识未解析时拒绝跳转到这里, 以 kUnknown 走到这里说明应用
        // 是别的途径启动的(调试器, 或早于该检查的 bootloader)。debug 下断言;
        // release 保留编译期 PID 而不是编造一个。
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

private: // 设备描述符
    // 两块 hpm5321 PCB 分到的 PID, 与原独立构建所用相同: 主机、udev 规则与
    // dfu-util 命令行不受合并影响; 变的只是某个二进制上报哪一个, 现由运行时决定。
    static constexpr uint16_t kSingleCanProductId = 0x5321;
    // 总线供电, 且不支持远程唤醒。该位曾置 1 而固件从未调用过 tud_remote_wakeup()
    // -- 描述符声明了设备不具备的能力。主机挂起总线后等待唤醒是合法行为, 宣告
    // 唤醒却不实现正是让这个等待永不结束的方式。
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

    // 可变副本: 构造函数按板标识修补 idProduct, 其余字段与上面的常量声明完全
    // 一致。
    tusb_desc_device_t device_descriptor_ = kDeviceDescriptor;

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

    // 刻意不加第二对 bulk 端点(2026-08-07 曾为 CAN 单开一对, 2026-09-05 移除):
    // 空闲 IN 端点无论有无数据都会被主机控制器持续轮询, 代价是四分之一的包率、
    // 每帧中位数多等一个主机服务间隔 -- 超过它消除的队头阻塞。见
    // firmware/hpm_board/AGENTS.md。

    static constexpr uint8_t const kConfigurationDescriptorFs[] = {
        // 配置序号, 接口数, 字符串索引, 总长度, 属性, 电流(mA)
        TUD_CONFIG_DESCRIPTOR(1, kItfNumTotal, 0, kConfigTotalLen, kConfigAttributes, 100),

        // 接口号, 字符串索引, EP 数据地址(out, in)与尺寸。
        TUD_VENDOR_DESCRIPTOR(kItfNumVendor, 0, kEpnumCdc0DataOut, kEpnumCdc0DataIn, 64),
        TUD_DFU_RT_DESCRIPTOR(
            kItfNumDfuRuntime, 4, DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_WILL_DETACH, 1000, 1024),
    };
    static_assert(sizeof(kConfigurationDescriptorFs) == kConfigTotalLen);

    static constexpr uint8_t const kConfigurationDescriptorHs[] = {
        // 配置序号, 接口数, 字符串索引, 总长度, 属性, 电流(mA)
        TUD_CONFIG_DESCRIPTOR(1, kItfNumTotal, 0, kConfigTotalLen, kConfigAttributes, 100),

        // 接口号, 字符串索引, EP 数据地址(out, in)与尺寸。
        TUD_VENDOR_DESCRIPTOR(kItfNumVendor, 0, kEpnumCdc0DataOut, kEpnumCdc0DataIn, 512),
        TUD_DFU_RT_DESCRIPTOR(
            kItfNumDfuRuntime, 4, DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_WILL_DETACH, 1000, 1024),
    };
    static_assert(sizeof(kConfigurationDescriptorHs) == kConfigTotalLen);

private: // 字符串描述符
    static constexpr std::array<uint8_t, 2> kLanguageId = {0x09, 0x04};
    static constexpr std::string_view kManufacturerString = "Helios";
    static constexpr std::string_view kProductString = "HCS Agent v" libhcs_PROJECT_VERSION_STRING;
    static constexpr std::string_view kDfuRuntimeString = "DFU Runtime";
    std::array<char, 43> serial_string_{"AF-0000-0000-0000-0000-0000-0000-0000-0000"};

    std::array<uint16_t, 128> descriptor_string_buffer_{};
};
inline constinit utility::Lazy<UsbDescriptors> usb_descriptors;

} // namespace libhcs::firmware::usb
