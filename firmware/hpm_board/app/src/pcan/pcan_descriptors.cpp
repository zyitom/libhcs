#include "firmware/hpm_board/app/src/pcan/pcan_descriptors.hpp"

#include <algorithm>
#include <cstring>

#include <tusb_config.h>

#include "firmware/hpm_board/app/src/pcan/pcan_adapter.hpp"

// PCAN 人格的描述符数据与 TinyUSB 回调。字段依据见 PCAN_PROTOCOL.md: 真实
// PCAN-USB FD 的身份事实(VID:PID / 端点 / 接口形状)取自内核 peak_usb 驱动的
// 设备表与 fw_info 协议, 字符串按 PEAK 惯例(内核不读, Windows 可能显示)。
// 接口/端点形状显式摆平(不写循环生成): 4 份表一眼可对。

namespace libhcs::firmware::pcan {

namespace {

// 总线供电、不支持远程唤醒(与默认镜像同一取舍: 宣告唤醒却不实现正是让主机
// 挂起后的等待永不结束的方式)。TUD_CONFIG_DESCRIPTOR 的属性字节自带 0x80。
constexpr uint8_t kConfigAttributes = 0;

constexpr uint8_t kItfNumCan = 0;
constexpr uint8_t kItfNumDfuRuntime = 1;
constexpr uint8_t kItfNumTotal = 2;

// 配置 = 头 9 + CAN 接口(9 + 7N) + DFU-RT 接口(18)。
constexpr std::size_t config_total_len(std::size_t endpoint_count) {
    return TUD_CONFIG_DESC_LEN + (9 + 7 * endpoint_count) + TUD_DFU_RT_DESC_LEN;
}

// bulk 端点描述符(7 字节: 头 / 地址 / 类型 / 尺寸 LE16 / bInterval 0)。
constexpr std::array<uint8_t, 7> bulk_endpoint(uint8_t address, uint16_t size) {
    return {
        7,
        TUSB_DESC_ENDPOINT,
        address,
        TUSB_XFER_BULK,
        static_cast<uint8_t>(size),
        static_cast<uint8_t>(size >> 8),
        0,
    };
}

// 单通道(4 端点)与双通道(5 端点)两份显式表, 端点号即 PCAN 协议的身份事实
// (fw_info 的 EXT 端点表与此一致)。
constexpr auto make_single(uint16_t size) {
    constexpr std::size_t total = config_total_len(4);
    std::array<uint8_t, total> out{};
    std::size_t p = 0;
    const auto emit = [&](auto range) {
        for (const uint8_t byte : range)
            out[p++] = byte;
    };
    emit(std::array<uint8_t, 9>{
        9, TUSB_DESC_CONFIGURATION, static_cast<uint8_t>(total), static_cast<uint8_t>(total >> 8),
        kItfNumTotal, 1, 0, static_cast<uint8_t>(0x80U | kConfigAttributes), 50});
    emit(std::array<uint8_t, 9>{
        9, TUSB_DESC_INTERFACE, kItfNumCan, 0, 4, TUSB_CLASS_VENDOR_SPECIFIC, 0, 0, 0});
    emit(bulk_endpoint(protocol::kEpCmdOut, size));
    emit(bulk_endpoint(protocol::kEpCmdIn, size));
    emit(bulk_endpoint(protocol::kEpDataOut0, size));
    emit(bulk_endpoint(protocol::kEpDataIn, size));
    emit(std::array<uint8_t, TUD_DFU_RT_DESC_LEN>{TUD_DFU_RT_DESCRIPTOR(
        kItfNumDfuRuntime, 4, DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_WILL_DETACH, 1000, 1024)});
    return out;
}

constexpr auto make_dual(uint16_t size) {
    constexpr std::size_t total = config_total_len(5);
    std::array<uint8_t, total> out{};
    std::size_t p = 0;
    const auto emit = [&](auto range) {
        for (const uint8_t byte : range)
            out[p++] = byte;
    };
    emit(std::array<uint8_t, 9>{
        9, TUSB_DESC_CONFIGURATION, static_cast<uint8_t>(total), static_cast<uint8_t>(total >> 8),
        kItfNumTotal, 1, 0, static_cast<uint8_t>(0x80U | kConfigAttributes), 50});
    emit(std::array<uint8_t, 9>{
        9, TUSB_DESC_INTERFACE, kItfNumCan, 0, 5, TUSB_CLASS_VENDOR_SPECIFIC, 0, 0, 0});
    emit(bulk_endpoint(protocol::kEpCmdOut, size));
    emit(bulk_endpoint(protocol::kEpCmdIn, size));
    emit(bulk_endpoint(protocol::kEpDataOut0, size));
    emit(bulk_endpoint(protocol::kEpDataIn, size));
    emit(bulk_endpoint(protocol::kEpDataOut1, size));
    emit(std::array<uint8_t, TUD_DFU_RT_DESC_LEN>{TUD_DFU_RT_DESCRIPTOR(
        kItfNumDfuRuntime, 4, DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_WILL_DETACH, 1000, 1024)});
    return out;
}

constexpr auto kConfigurationFsSingle = make_single(64);
constexpr auto kConfigurationFsDual = make_dual(64);
constexpr auto kConfigurationHsSingle = make_single(512);
constexpr auto kConfigurationHsDual = make_dual(512);

constexpr std::array<uint8_t, 2> kLanguageId = {0x09, 0x04};
constexpr std::string_view kManufacturerString = "PEAK-System";
constexpr std::string_view kProductStringSingle = "PCAN-USB FD";
constexpr std::string_view kProductStringDual = "PCAN-USB Pro FD";
constexpr std::string_view kDfuRuntimeString = "DFU Runtime";

} // namespace

Descriptors::Descriptors() {
    device_descriptor_ = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200, // 不读 BOS 的取法; 本人格刻意不广告 MS OS 2.0(见文件头)
        .bDeviceClass = 0x00, // 类由接口定义(vendor + DFU)
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor = core::protocol::usb_identity::kPcanVendorId,
        // 编译期取单通道值, 下面按板型补丁(与默认镜像的 update_product_id 同一
        // 模式; 双 CAN 板上报 Pro FD 的身份)。未识别的板(调试器启动)保留编译期值。
        .idProduct = core::protocol::usb_identity::kPcanUsbFdProductId,
        .bcdDevice = 0x0354, // 与 fw_version 3.5.4 呼应(真实 Pro FD 当前固件)
        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,
        .bNumConfigurations = 0x01,
    };
    if (board::board_identity().dual_can())
        device_descriptor_.idProduct = core::protocol::usb_identity::kPcanUsbProFdProductId;

    // 序列号十进制化(u32 最长 10 位): iSerialNumber 显示的就是 fw_info.ser_no。
    uint32_t serial = serial_number();
    std::size_t length = 0;
    do {
        serial_decimal_[length++] = static_cast<char>('0' + serial % 10U);
        serial /= 10U;
    } while (serial != 0U && length < serial_decimal_.size() - 1U);
    std::ranges::reverse(serial_decimal_.begin(), serial_decimal_.begin() + length);
    if (length < serial_decimal_.size())
        serial_decimal_[length] = '\0';
}

uint8_t const* Descriptors::get_configuration_descriptor(uint8_t index) {
    (void)index; // 多配置预留

    const bool dual = board::board_identity().dual_can();
    if constexpr (TUD_OPT_HIGH_SPEED) {
        if (tud_speed_get() == TUSB_SPEED_HIGH)
            return dual ? kConfigurationHsDual.data() : kConfigurationHsSingle.data();
        return dual ? kConfigurationFsDual.data() : kConfigurationFsSingle.data();
    } else {
        return dual ? kConfigurationFsDual.data() : kConfigurationFsSingle.data();
    }
}

uint16_t const* Descriptors::get_string_descriptor(uint16_t index) {
    static std::array<uint16_t, 64> buffer{};

    uint8_t str_size;
    if (index == 0) {
        std::memcpy(&buffer[1], kLanguageId.data(), kLanguageId.size());
        str_size = 1;
    } else {
        std::string_view str;
        switch (index) {
        case 1: str = kManufacturerString; break;
        case 2:
            str = board::board_identity().dual_can() ? kProductStringDual : kProductStringSingle;
            break;
        case 3:
            str = std::string_view{serial_decimal_.data(), std::strlen(serial_decimal_.data())};
            break;
        case 4: str = kDfuRuntimeString; break;
        default: return nullptr;
        }

        str_size = static_cast<uint8_t>(std::min<std::size_t>(str.size(), buffer.size() - 1));
        for (uint8_t i = 0; i < str_size; i++)
            buffer[i + 1] = static_cast<uint16_t>(str[i]);
    }

    buffer[0] = (TUSB_DESC_STRING << 8) | static_cast<uint16_t>((2 * str_size) + 2);
    return buffer.data();
}

} // namespace libhcs::firmware::pcan

// TinyUSB 描述符回调(pcan 构建里的唯一定义; 默认镜像的那份在 usb_descriptors.cpp)。
extern "C" {

uint8_t const* tud_descriptor_device_cb(void) {
    return libhcs::firmware::pcan::descriptors->get_device_descriptor();
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    return libhcs::firmware::pcan::Descriptors::get_configuration_descriptor(index);
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    return libhcs::firmware::pcan::descriptors->get_string_descriptor(index);
}

} // extern "C"
