#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string_view>
#include <tuple>
#include <utility>

#include <class/cdc/cdc.h>
#include <class/dfu/dfu.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <hpm_otp_drv.h>
#include <hpm_soc_feature.h>
#include <tusb_config.h>
#include <tusb_option.h>

#include "core/include/libhcs/protocol/usb_identity.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"
#include "firmware/hpm_board/common/board_identity.hpp"

namespace libhcs::firmware::usb {

// DMTool 仿真的三个接口, 接口号即 DMTool claim 的 0/1/2, 端点号是 DMTool 二进制
// 里的立即数(firmware/hpm_board/DMTOOL_PROTOCOL.md)。它们由 dmtool 的应用类驱动
// 承接, 不走 TinyUSB 的 vendor 类: vendor 类因此只有 libhcs 一个实例(实例 0),
// bulk 回调与发送路径与没有 DMTool 时逐条相同。
enum class DmInterface : uint8_t {
    kData = 0,    // EP 0x01 OUT 心跳 / 0x81 IN 接收记录流
    kCommand = 1, // EP 0x02 OUT 命令 / 0x82 IN 应答
    kCan = 2,     // EP 0x03 OUT CAN 发送 / 0x83 IN 发送回显记录流
};
inline constexpr uint8_t kDmInterfaceCount = 3;

class UsbDescriptors {
public:
    UsbDescriptors() {
        update_serial_string();
        update_product_id();
    }

    // 非 static: 两块 PCB 共用一个镜像, idProduct 由运行时按 OTP 标识决定。
    uint8_t const* get_device_descriptor() const {
        return reinterpret_cast<uint8_t const*>(&device_descriptor_);
    }

    static uint8_t const* get_configuration_descriptor(uint8_t index) {
        (void)index; // 多配置预留

        if constexpr (TUD_OPT_HIGH_SPEED)
            return (tud_speed_get() == TUSB_SPEED_HIGH) ? kConfigurationDescriptorHs.data()
                                                        : kConfigurationDescriptorFs.data();
        else
            return kConfigurationDescriptorFs.data();
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
            case 3: str = serial_string(); break;
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

    // 序列号串(不含结尾 NUL)。DMTool 的读 SN 命令回的也是它, 两处看到同一个号。
    [[nodiscard]] std::string_view serial_string() const {
        return {serial_string_.data(), serial_string_.size() - 1};
    }

    // 产品串, 即 libhcs 主机认板用的 "HCS Agent v<version>"。DMTool 的读版本命令
    // 回的也是它。
    static constexpr std::string_view product_string() { return kProductString; }

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

    // 按 PCB 上报 PID: 单 CAN 板 0x6877、双 CAN 板 0x6632, 都在 DMTool 认的表里
    // (libhcs/protocol/usb_identity.hpp)。PID 让板型在打开设备之前就可见(lsusb、
    // udev、dfu-util); 打开后 EP0 kGetInterface 的 can_count 报的是同一个事实。
    // 单形态板子上是空操作: board::kOtpIdentityEnabled 为 false, 保留编译期值。
    //
    // 产品字符串刻意不随板变化: 主机对 "HCS Agent v<version>" 精确匹配。
    void update_product_id() {
        if constexpr (!board::kOtpIdentityEnabled)
            return;

        const auto& identity = board::board_identity();

        // bootloader 在标识未解析时拒绝跳转到这里, 以 kUnknown 走到这里说明应用
        // 是别的途径启动的(调试器, 或早于该检查的 bootloader)。debug 下断言;
        // release 保留编译期 PID 而不是编造一个。
        core::utility::assert_debug(identity.recognized());

        namespace id = core::protocol::usb_identity;
        if (identity.variant == board::BoardVariant::kSingleCan)
            device_descriptor_.idProduct = id::kHpm5321SingleCanProductId;
        else if (identity.variant == board::BoardVariant::kDualCanFd)
            device_descriptor_.idProduct = id::kHpm5321DualCanProductId;
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
    // 总线供电, 且不支持远程唤醒。该位曾置 1 而固件从未调用过 tud_remote_wakeup()
    // -- 描述符声明了设备不具备的能力。主机挂起总线后等待唤醒是合法行为, 宣告
    // 唤醒却不实现正是让这个等待永不结束的方式。
    static constexpr uint8_t kConfigAttributes = 0;

    // 借用达妙 USB2FDCAN 的 VID:PID: DMTool 只按它编译进去的 VID:PID 表认设备,
    // 不看任何字符串。libhcs 主机仍靠产品串 "HCS Agent v<version>" 认板(真
    // 适配器没有这个串)。编译期 PID 取单 CAN 板的, 构造函数按板修补。bootloader
    // 不受影响, 仍按 OTP 报 0xA511:0x5321/0x5322。见 libhcs/protocol/usb_identity.hpp。

    static constexpr tusb_desc_device_t kDeviceDescriptor = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0210, // >= 0x0210: Windows 8.1+ reads BOS to discover the MS OS 2.0
                          // WCID platform capability (see kBosDescriptor below). Other
                          // hosts do not request BOS; behavior is otherwise unchanged.

        .bDeviceClass = TUSB_CLASS_UNSPECIFIED,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

        .idVendor = core::protocol::usb_identity::kDmtoolVendorId,
        .idProduct = core::protocol::usb_identity::kHpm5321SingleCanProductId,
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
    static constexpr auto kLibhcsPipe = core::protocol::usb_identity::kDmtoolCoexistDataPipe;

    // NOLINTNEXTLINE(cppcoreguidelines-use-enum-class)
    enum InterfaceNumber : uint8_t {
        kItfNumDmData = std::to_underlying(DmInterface::kData),
        kItfNumDmCommand = std::to_underlying(DmInterface::kCommand),
        kItfNumDmCan = std::to_underlying(DmInterface::kCan),
        kItfNumLibhcs = std::to_underlying(DmInterface::kCan) + 1,
        kItfNumCdc = std::to_underlying(DmInterface::kCan)
                   + 2, // CDC 控制接口, 数据接口紧随其后(TUD_CDC_DESCRIPTOR 的约定)
        kItfNumCdcData = std::to_underlying(DmInterface::kCan) + 3,
        kItfNumDfuRuntime = std::to_underlying(DmInterface::kCan) + 4,
        kItfNumTotal = std::to_underlying(DmInterface::kCan) + 5,
    };
    static_assert(kItfNumLibhcs == kLibhcsPipe.interface_number);
    static_assert(kItfNumLibhcs == kDmInterfaceCount);

    // DMTool 的三对端点是它二进制里的立即数; libhcs 管道见 usb_identity.hpp。
    static constexpr uint8_t kEpDmDataOut = 0x01;
    static constexpr uint8_t kEpDmDataIn = 0x81;
    static constexpr uint8_t kEpDmCommandOut = 0x02;
    static constexpr uint8_t kEpDmCommandIn = 0x82;
    static constexpr uint8_t kEpDmCanOut = 0x03;
    static constexpr uint8_t kEpDmCanIn = 0x83;
    static constexpr uint8_t kEpCdcDataOut = 0x05;
    static constexpr uint8_t kEpCdcDataIn = 0x85;
    static constexpr uint8_t kEpCdcNotify = 0x86;

    // DMTool 的三个接口与 vendor 接口同形(TUD_VENDOR_DESCRIPTOR), 只是换了驱动。
    static constexpr size_t kVendorShapedInterfaceCount = kDmInterfaceCount + CFG_TUD_VENDOR;

    static constexpr size_t kConfigTotalLen =
        TUD_CONFIG_DESC_LEN + (kVendorShapedInterfaceCount * TUD_VENDOR_DESC_LEN)
        + (CFG_TUD_CDC * TUD_CDC_DESC_LEN) + (CFG_TUD_DFU_RUNTIME * TUD_DFU_RT_DESC_LEN);

    // 多出来的端点不给 libhcs 数据面添成本: bulk 端点只在主机挂着传输时才被主机
    // 控制器调度, DMTool 不跑它那三对就一次事务都没有(2026-08-07 那次分端点吃掉
    // 四分之一包率, 是因为自家主机常驻挂着第二对管道, 见 USB_OPTIMIZATION_LOG.md
    // 4.5 节)。CDC 通知端点同理, Linux cdc_acm 只在串口打开期间挂它的 URB。
    //
    // 通知端点的 bInterval 从 TUD_CDC_DESCRIPTOR 写死的 1 改成 16: 本固件从不发
    // CDC 通知, 串口打开时也没必要每个微帧空轮询一次(高速下 16 = 2^15 微帧,
    // 约 4 s)。
    static constexpr uint8_t kCdcNotifyInterval = 16;
    static constexpr size_t kCdcNotifyIntervalOffset =
        TUD_CONFIG_DESC_LEN + (kVendorShapedInterfaceCount * TUD_VENDOR_DESC_LEN)
        + (8 + 9 + 5 + 5 + 4 + 5) + 6;

    // 类内的 lambda 而非成员函数: 成员函数体要到类完整后才可用, 而这两个常量
    // 正是在类体里初始化的。
    static constexpr auto kMakeConfigurationDescriptor = [](uint16_t bulk_size) consteval {
        // 先落 C 数组: 长度由初始化列表推出, 可以与 kConfigTotalLen 对账(std::array
        // 的花括号初始化少给元素只会静默补零)。
        const uint8_t bytes[] = {
            // 配置序号, 接口数, 字符串索引, 总长度, 属性, 电流(mA)
            TUD_CONFIG_DESCRIPTOR(1, kItfNumTotal, 0, kConfigTotalLen, kConfigAttributes, 100),

            // 接口号, 字符串索引, EP 数据地址(out, in)与尺寸。前三个归 dmtool 的应用
            // 类驱动, 第四个是 vendor 类唯一的实例(libhcs)。
            TUD_VENDOR_DESCRIPTOR(kItfNumDmData, 0, kEpDmDataOut, kEpDmDataIn, bulk_size),
            TUD_VENDOR_DESCRIPTOR(kItfNumDmCommand, 0, kEpDmCommandOut, kEpDmCommandIn, bulk_size),
            TUD_VENDOR_DESCRIPTOR(kItfNumDmCan, 0, kEpDmCanOut, kEpDmCanIn, bulk_size),
            TUD_VENDOR_DESCRIPTOR(
                kItfNumLibhcs, 0, kLibhcsPipe.out_endpoint, kLibhcsPipe.in_endpoint, bulk_size),

            // 接口号, 字符串索引, 通知 EP 与尺寸, 数据 EP(out, in)与尺寸。
            TUD_CDC_DESCRIPTOR(
                kItfNumCdc, 0, kEpCdcNotify, 8, kEpCdcDataOut, kEpCdcDataIn, bulk_size),

            TUD_DFU_RT_DESCRIPTOR(
                kItfNumDfuRuntime, 4, DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_WILL_DETACH, 1000, 1024),
        };
        static_assert(sizeof(bytes) == kConfigTotalLen);

        std::array<uint8_t, kConfigTotalLen> descriptor{};
        std::ranges::copy(bytes, descriptor.begin());
        descriptor[kCdcNotifyIntervalOffset] = kCdcNotifyInterval;
        return descriptor;
    };

    static constexpr auto kConfigurationDescriptorFs = kMakeConfigurationDescriptor(64);
    static constexpr auto kConfigurationDescriptorHs = kMakeConfigurationDescriptor(512);

    // 改的确实是通知端点的 bInterval: TinyUSB 宏的字节布局一变, 编译期即失败。
    static_assert(
        kConfigurationDescriptorHs[kCdcNotifyIntervalOffset - 6] == 7
        && kConfigurationDescriptorHs[kCdcNotifyIntervalOffset - 5] == TUSB_DESC_ENDPOINT
        && kConfigurationDescriptorHs[kCdcNotifyIntervalOffset - 4] == kEpCdcNotify
        && kConfigurationDescriptorHs[kCdcNotifyIntervalOffset - 3] == TUSB_XFER_INTERRUPT);

public: // Windows WCID (MS OS 2.0): 免驱 WinUSB 绑定
    // 这块板的 5 个非 CDC 接口(3 个 DMTool + libhcs + DFU Runtime)是 vendor/DFU
    // 类, Windows 没有收件箱驱动, 插上即代码 28 "驱动未安装"。Linux 的 libusb/
    // usbfs 无需内核驱动即可 claim vendor 接口, 所以 Linux 侧从未暴露此问题。
    // MS OS 2.0 平台能力(经 BOS 广告)让 Windows 8.1+ 在首次枚举时用
    // kMsOsVendorCode 拉 WCID 描述符集, 按 CompatibleID "WINUSB" 自动绑定收件箱
    // 的 WinUSB 驱动 -- 与原装达妙适配器同一机制, 无需 Zadig/INF。CDC 两个接口
    // 刻意不列入: 它们由 usbser 收件箱驱动服务, 声明 WINUSB 会把 COM 口顶掉。
    //
    // 性质: 纯 EP0 枚举期一次性数据(Windows 首次安装时读取并缓存), 不触及 bulk
    // 数据面; Linux 不请求 BOS, 数据面零开销。
    //
    // 请求码 0x21 是 Microsoft 文档示例惯例; libhcs EP0 配置通道占用 0x40..0x47
    // (vendor_control.hpp), 二者不冲突。拦截点在 vendor_control.cpp 的
    // handle_setup 顶部。
    static constexpr uint8_t kMsOsVendorCode = 0x21;

    // MS OS 2.0 描述符集, 字节布局按 "Microsoft OS 2.0 descriptors" 规范手搓
    // (TinyUSB 只给了 BOS 侧的宏, 集合本身自备)。每层 wLength 均含自身。
    static constexpr auto kMakeMsOs20Set = []() consteval {
        constexpr std::string_view guid = "{4E1F6C1A-8B3D-4E7A-9C5B-2F0D1A3B4C5E}";
        constexpr std::string_view property_name = "DeviceInterfaceGUIDs";
        // REG_MULTI_SZ: 名与值各为 UTF-16LE 文本加两个 NUL 字符收尾。
        constexpr std::size_t name_bytes = (property_name.size() + 2) * 2;
        constexpr std::size_t data_bytes = (guid.size() + 2) * 2;
        constexpr std::size_t registry_property_length = 8 + name_bytes + 2 + data_bytes;
        // 头 4 + CompatibleID 8 + SubCompatibleID 8 (MS OS 2.0 是 20 字节; 24 是
        // 1.0 的布局)。
        constexpr std::size_t compatible_id_length = 20;
        constexpr std::size_t function_subset_length =
            8 + compatible_id_length + registry_property_length;
        constexpr std::size_t config_subset_length = 8 + (5 * function_subset_length);
        constexpr std::size_t total_length = 10 + config_subset_length;

        std::array<uint8_t, total_length> out{};
        std::size_t p = 0;
        const auto le16 = [&out, &p](uint16_t value) {
            out[p++] = static_cast<uint8_t>(value);
            out[p++] = static_cast<uint8_t>(value >> 8);
        };
        const auto utf16 = [&out, &p](std::string_view text) {
            for (const char c : text) {
                out[p++] = static_cast<uint8_t>(c);
                out[p++] = 0;
            }
            // REG_MULTI_SZ 双 NUL 收尾: 两个 UTF-16 NUL, 共 4 字节 -- 长度公式
            // (size + 2) * 2 与此对应。
            out[p++] = 0;
            out[p++] = 0;
            out[p++] = 0;
            out[p++] = 0;
        };

        // 描述符集头: dwWindowsVersion 0x06030000 = Windows 8.1, 能读它的最低版本。
        le16(10);
        le16(0x0000); // MS_OS_20_SET_HEADER_DESCRIPTOR
        le16(0x0000);
        le16(0x0603); // dwWindowsVersion 低/高半字
        le16(static_cast<uint16_t>(total_length));

        // 配置子集头(配置 0)。
        le16(8);
        le16(0x0001); // MS_OS_20_SUBSET_HEADER_CONFIGURATION
        out[p++] = 0;
        out[p++] = 0;
        le16(static_cast<uint16_t>(config_subset_length));

        for (const uint8_t interface :
             {kItfNumDmData, kItfNumDmCommand, kItfNumDmCan, kItfNumLibhcs, kItfNumDfuRuntime}) {
            // 功能子集头: 一个 vendor/DFU 接口。
            le16(8);
            le16(0x0002); // MS_OS_20_SUBSET_HEADER_FUNCTION
            out[p++] = interface;
            out[p++] = 0;
            le16(static_cast<uint16_t>(function_subset_length));

            // CompatibleID "WINUSB": 收件箱 WinUSB 据此绑定该接口。
            le16(static_cast<uint16_t>(compatible_id_length));
            le16(0x0003); // MS_OS_20_FEATURE_COMPATBLE_ID
            for (const char c : std::string_view{"WINUSB"})
                out[p++] = static_cast<uint8_t>(c);
            for (std::size_t i = 0; i < 2 + 8; ++i)
                out[p++] = 0; // CompatibleID NUL 补齐 + SubCompatibleID(未用)

            // DeviceInterfaceGUIDs 注册属性(REG_MULTI_SZ)。WinUSB 绑定后设备接口
            // 以此 GUID 暴露; 按 VID:PID 打开的软件(libusb/DMTool/dfu-util)不读
            // 它, SetupDi 枚举需要。五接口共用一个类 GUID。
            le16(static_cast<uint16_t>(registry_property_length));
            le16(0x0004); // MS_OS_20_FEATURE_REG_PROPERTY
            le16(0x0007); // REG_MULTI_SZ
            le16(static_cast<uint16_t>(name_bytes));
            utf16(property_name);
            le16(static_cast<uint16_t>(data_bytes));
            utf16(guid);
        }

        return out;
    };
    static constexpr auto kMsOs20DescriptorSet = kMakeMsOs20Set();

    // 布局校验: 按规范的层级结构走一遍 wLength 链 -- 集头 10 字节, 配置子集头
    // 8 字节, 其后 5 个功能子集按各自 wLength 前进(配置子集的 wTotalLength 覆盖
    // 头 + 全部功能子集, 不在外层重复走)。Windows 解析器正是这样读, 段错位或
    // wLength 写错都是编译错误。
    static constexpr uint16_t kMsOs20SetLength = kMsOs20DescriptorSet.size();
    static constexpr auto kMsOs20Walk = []() consteval {
        const auto rd16 = [](std::size_t offset) {
            return static_cast<uint16_t>(
                kMsOs20DescriptorSet[offset] | (kMsOs20DescriptorSet[offset + 1] << 8));
        };
        struct Result {
            uint16_t set_end;                         // 走完全部层后的游标, 应等于集合总长
            uint16_t config_span;                     // 配置子集实际跨度, 应等于总长 - 集头 10
        };
        std::size_t q = rd16(0);                      // set header wLength = 10
        const uint16_t config_declared = rd16(q + 6); // config subset wTotalLength
        q += 8;                                       // configuration subset header
        const std::size_t config_start = q;
        for (std::size_t i = 0; i < 5; ++i)
            q += rd16(q + 6); // function subset wSubsetLength (header offset 6)
        return Result{
            .set_end = static_cast<uint16_t>(q + ((q - 10 == config_declared) ? 0 : 1)),
            .config_span = static_cast<uint16_t>(q - config_start + 8)};
    }();
    static_assert(kMsOs20Walk.set_end == kMsOs20SetLength);
    static_assert(kMsOs20Walk.config_span == kMsOs20SetLength - 10);

    // BOS: 向 Windows 广告 MS OS 2.0 平台能力(平台能力 UUID + 集合长度 +
    // vendor code)。由 usb_descriptors.cpp 的 tud_descriptor_bos_cb 应答。
    static constexpr uint8_t kBosDescriptor[] = {
        TUD_BOS_DESCRIPTOR(5 + 28, 1),
        TUD_BOS_MS_OS_20_DESCRIPTOR(kMsOs20SetLength, kMsOsVendorCode),
    };
    static_assert(sizeof(kBosDescriptor) == 33);

    static uint8_t const* get_bos_descriptor() { return kBosDescriptor; }
    static uint8_t const* get_ms_os_20_set() { return kMsOs20DescriptorSet.data(); }

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
