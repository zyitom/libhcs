#include <array>
#include <cstdint>
#include <cstring>
#include <span>

#include <common/tusb_types.h>
#include <device/usbd.h>
#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_interrupt.h>
#include <hpm_mchtmr_drv.h>
#include <hpm_soc.h>
#include <tusb.h>

#include "board_app.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/hpm_board/app/src/link/uplink.hpp"
#include "firmware/hpm_board/app/src/pcan/pcan_adapter.hpp"
#include "firmware/hpm_board/app/src/pcan/pcan_descriptors.hpp"
#include "firmware/hpm_board/app/src/utility/boot_mailbox.hpp"

// PCAN 人格的 EP0 / USB 初始化 / 引导胶水 -- 默认镜像里由 usb/vendor.cpp 与
// usb/vendor_control.cpp 承担的那些符号, 在 pcan 构建里归本文件(每个构建恰好
// 一份, 见 PCAN_PROTOCOL.md 的符号所有权表)。
//
// 运行位置: TinyUSB 在 USB ISR 里排队 setup 包, 在 tud_task() 中解码, 因此下面
// 的 EP0 代码全部运行在主循环上 -- 与 bulk 回调同线程, 可以直接动 Adapter 状态。

namespace {

namespace pp = libhcs::firmware::pcan::protocol;

constexpr uint32_t kDfuRuntimeResetDelayMs = 50U;

// REQ_FCT/DRVLD 的 OUT 数据段(16 字节, 主机只写前 2 字节: [0]=0, [1]=已加载)。
uint8_t g_drvld_buffer[pp::kDrvldLength] = {};

// EP0 IN 应答缓冲: fw_info(36) / blinfo(28) 就地编码, 比两者都大。
std::array<uint8_t, pp::FwInfo::kSize> g_reply_buffer{};

volatile bool g_dfu_runtime_reboot_requested = false;
volatile uint32_t g_dfu_runtime_reboot_requested_ms = 0U;

uint32_t runtime_ms() {
    const uint64_t ticks_per_ms = static_cast<uint64_t>(clock_get_frequency(clock_mchtmr0)) / 1000U;
    return static_cast<uint32_t>(mchtmr_get_count(HPM_MCHTMR) / ticks_per_ms);
}

} // namespace

// 数据平面的归属: 本构建没有 libhcs 会话, 上行永远关闭。这两个名字是
// link/uplink.hpp 声明的接缝 -- "谁拥有数据面谁定义" -- 默认构建链接
// link/uplink_usb.cpp, 这里给出 pcan 构建的那一份(uplink_usb.cpp 文件头的
// 注释就是为这种替换写的)。can.cpp 对 uplink_enabled() 的调用每帧一次, 与
// 默认构建同在 .fast; uplink_serializer() 实际不可达, 空缓冲使任何走错路的
// 调用得到 kBadAlloc 而非未定义行为。
namespace {

struct NullSerializeBuffer final : libhcs::core::protocol::SerializeBuffer {
    std::span<std::byte> allocate(std::size_t size) noexcept override {
        (void)size;
        return {};
    }
};
constinit NullSerializeBuffer g_null_buffer{};

} // namespace

namespace libhcs::firmware::link {

ATTR_PLACE_AT(".fast")
core::protocol::Serializer& uplink_serializer() {
    static core::protocol::Serializer serializer{g_null_buffer};
    return serializer;
}

ATTR_PLACE_AT(".fast")
bool uplink_enabled() { return false; }

} // namespace libhcs::firmware::link

namespace libhcs::firmware::pcan {

void usb_init() {
    descriptors.init();

    const tusb_rhport_init_t init_config{
        .role = TUSB_ROLE_DEVICE,
        .speed = board::usb_use_high_speed() ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL,
    };
    core::utility::assert_always(tusb_rhport_init(0, &init_config));

    // 与默认镜像(vendor.hpp)同一处钉法: tusb_rhport_init -> dcd_init 已使能 USB
    // 中断, 这里显式钉住优先级, CAN(3) > USB(2) 的抢占层级归本文件所有。
    intc_m_enable_irq_with_priority(IRQn_USB0, 2);
}

void poll_dfu_reboot() {
    if (!g_dfu_runtime_reboot_requested)
        return;

    if ((runtime_ms() - g_dfu_runtime_reboot_requested_ms) < kDfuRuntimeResetDelayMs)
        return;

    boot::BootMailbox::reboot_to_bootloader();
}

} // namespace libhcs::firmware::pcan

// TinyUSB 设备回调与 EP0 vendor 请求(pcan 构建里的唯一定义)。
extern "C" {

// USB0 中断向量: 与默认镜像同一理由在应用代码绑定(SDK 把 ISR 留在示例里)。
// 没有 SOF 钩子: 本人格不用共享时间基, SOF 中断永不使能。
SDK_DECLARE_EXT_ISR_M(IRQn_USB0, hcs_usb0_isr)
void hcs_usb0_isr(void) { dcd_int_handler(0); }

// PCAN 的 EP0 vendor 请求(内核 pcan_usb_pro_send_req): bmRequestType =
// vendor|recipient-OTHER, wIndex 恒 0。别的 vendor 请求一律 STALL。wLength 精确
// 匹配而非截断 -- 要求不同尺寸的主机说的是本协议的另一个版本。
bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR)
        return false;

    if (request->bRequest == pp::kReqFct) {
        // DRVLD: OUT 数据段带上"驱动已加载"标志, 到达后再改状态。
        if (request->wValue != pp::kFctDrvld || request->wLength != pp::kDrvldLength)
            return false;
        if (stage == CONTROL_STAGE_SETUP)
            return tud_control_xfer(rhport, request, g_drvld_buffer, pp::kDrvldLength);
        if (stage == CONTROL_STAGE_DATA)
            libhcs::firmware::pcan::set_driver_loaded(g_drvld_buffer[1] != 0U);
        return true;
    }

    if (request->bRequest != pp::kReqInfo || stage != CONTROL_STAGE_SETUP)
        return true; // DATA/ACK 阶段无事可做

    switch (request->wValue) {
    case pp::kInfoFw: {
        if (request->wLength != pp::FwInfo::kSize)
            return false;
        const pp::FwInfo info{
            .device_ids =
                {libhcs::firmware::pcan::device_id(0), libhcs::firmware::pcan::device_id(1)},
            .serial_number = libhcs::firmware::pcan::serial_number(),
        };
        const auto encoded = info.encode();
        static_assert(encoded.size() <= g_reply_buffer.size());
        std::memcpy(g_reply_buffer.data(), encoded.data(), encoded.size());
        return tud_control_xfer(
            rhport, request, g_reply_buffer.data(), static_cast<uint16_t>(encoded.size()));
    }
    case pp::kInfoBl: {
        if (request->wLength != pp::BlInfo::kSize)
            return false;
        const pp::BlInfo info{
            .serial_number = libhcs::firmware::pcan::serial_number(),
        };
        const auto encoded = info.encode();
        static_assert(encoded.size() <= g_reply_buffer.size());
        std::memcpy(g_reply_buffer.data(), encoded.data(), encoded.size());
        return tud_control_xfer(
            rhport, request, g_reply_buffer.data(), static_cast<uint16_t>(encoded.size()));
    }
    default: return false;
    }
}

void tud_dfu_runtime_reboot_to_dfu_cb() {
    libhcs::firmware::boot::BootMailbox::request_enter_dfu();
    g_dfu_runtime_reboot_requested_ms = runtime_ms();
    g_dfu_runtime_reboot_requested = true;
}

} // extern "C"
