#pragma once

// PCAN 人格的 USB 描述符: 整板枚举为 PEAK PCAN-USB FD(单 CAN PCB, PID 0x0012)/
// PCAN-USB Pro FD(双 CAN PCB, PID 0x0011), 接口 0 = uCAN vendor 接口(端点表见
// pcan_protocol.hpp 的 kEp* 常量), 接口 1 = DFU runtime(刷回默认镜像的通道)。
//
// 与默认镜像(firmware/hpm_board/app/src/usb/usb_descriptors.hpp)的差别:
//   - VID 借 PEAK 的 0x0C72, PID 运行时按 OTP 板型补丁(同一镜像服务两块 PCB);
//   - 无 BOS / MS OS 2.0(WCID): Windows 上 WinUSB 会抢绑 vendor 接口, 反而挡住
//     PEAK 自己的驱动; bcdUSB 取 0x0200, 主机因此不会来问 BOS;
//   - 无 CDC / libhcs / DMTool 接口。
// 配置描述符按"单/双通道 x FS/HS"四份 consteval 表生成, 运行时按板型选择;
// TinyUSB 的描述符回调在 pcan_descriptors.cpp 给出(pcan 构建里的唯一定义)。

#include <array>
#include <cstdint>

#include <class/dfu/dfu.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <tusb_option.h>

#include "core/include/libhcs/protocol/usb_identity.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"
#include "firmware/hpm_board/common/board_identity.hpp"

namespace libhcs::firmware::pcan {

class Descriptors {
public:
    Descriptors();

    [[nodiscard]] uint8_t const* get_device_descriptor() const {
        return reinterpret_cast<uint8_t const*>(&device_descriptor_);
    }

    [[nodiscard]] static uint8_t const* get_configuration_descriptor(uint8_t index);

    [[nodiscard]] uint16_t const* get_string_descriptor(uint16_t index);

private:
    // 可变副本: 构造函数按板型补丁 idProduct, 其余字段与常量声明完全一致。
    tusb_desc_device_t device_descriptor_;
    // 序列号的十进制串(u32 最长 10 位 + NUL): iSerialNumber 与 fw_info.ser_no
    // 同源(pcan::serial_number()), 主机两处看到同一个号。
    std::array<char, 11> serial_decimal_{};
};

inline constinit utility::Lazy<Descriptors> descriptors;

} // namespace libhcs::firmware::pcan
