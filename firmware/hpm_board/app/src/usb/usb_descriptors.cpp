#include "firmware/hpm_board/app/src/usb/usb_descriptors.hpp"

#include <cstdint>

namespace libhcs::firmware::usb {

// TinyUSB 描述符回调
extern "C" {

// 设备描述符
// 收到 GET DEVICE DESCRIPTOR 时调用, 应用返回指向描述符的指针
uint8_t const* tud_descriptor_device_cb(void) { return usb_descriptors->get_device_descriptor(); }

// 配置描述符
// 收到 GET CONFIGURATION DESCRIPTOR 时调用, 应用返回指向描述符的指针;
// 描述符内容必须存活到传输完成
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    return usb_descriptors->get_configuration_descriptor(index);
}

// 字符串描述符
// 收到 GET STRING DESCRIPTOR 请求时调用, 应用返回指向描述符的指针;
// 描述符内容必须存活到传输完成
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    return usb_descriptors->get_string_descriptor(index, langid);
}

// BOS 描述符
// Windows 8.1+ 读它发现 MS OS 2.0 WCID 平台能力, 再用 vendor code 拉描述符集
// (拦截点在 vendor_control.cpp 的 handle_setup)。集合内容与动机见
// usb_descriptors.hpp 的 WCID 一节; Linux 主机不请求, 数据面零开销。
uint8_t const* tud_descriptor_bos_cb(void) { return UsbDescriptors::get_bos_descriptor(); }

} // extern "C"

} // namespace libhcs::firmware::usb
