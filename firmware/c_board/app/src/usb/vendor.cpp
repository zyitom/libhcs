#include "firmware/c_board/app/src/usb/vendor.hpp"

#include <cstddef>
#include <cstdint>

#include <main.h>

#include "core/src/protocol/serializer.hpp"
#include "firmware/c_board/app/src/ports.hpp"
#include "firmware/c_board/app/src/usb/helper.hpp"
#include "firmware/c_board/app/src/utility/boot_mailbox.hpp"

namespace {

constexpr uint32_t kDfuRuntimeResetDelayMs = 50U;

volatile bool g_dfu_runtime_reboot_requested = false;
volatile uint32_t g_dfu_runtime_reboot_requested_tick = 0U;

[[noreturn]] void reset_system() {
    __DSB();
    __ISB();
    NVIC_SystemReset();
    while (true) {}
}

} // namespace

namespace libhcs::firmware::usb {

// 会话结束(租约到期、总线复位、挂起或拔线)的板级半边: 每个口由自己的 suspend()
// 停, 时间基准一并关 -- 下一个主机的清单要了才再开。基类的 deactivate_session()
// 先结束会话状态, 再调到这里。
void Vendor::session_deactivated_callback() {
    ep0_handshake_done_ = false;
    ports::Registry::suspend_all();
    sync::time_sync_stop();
}

// 下行数据按 DataId 交给口的驱动, 经 ports::Registry 分发。会话门已在基类把守;
// 不是本板这一类口的 DataId 才算不认识(返回 false)。没声明(或方向不符)的口收到
// 的数据由驱动自己丢弃。

bool Vendor::dispatch_can(core::protocol::FieldId id, const data::CanDataView& data) {
    return ports::Registry::dispatch<spec::PortKind::kCan>(id, [&data](auto, auto& bus) {
        bus.handle_downlink(data);
        return true;
    });
}

void Vendor::report_port_status(core::link::PortStatusRound& round) {
    core::link::offer_port_status<ports::Registry>(round);
}

bool Vendor::dispatch_uart(core::protocol::FieldId id, const data::UartDataView& data) {
    return ports::Registry::dispatch<spec::PortKind::kUart>(id, [&data](auto binding, auto& port) {
        // DBUS 只承载接收机的上行流: 发给它的数据得到与"没有这个口"相同的应答
        // (它的驱动能发送, 但协议没有往那儿路由)。
        if constexpr (decltype(binding)::data_id == spec::c_board::Spec::Uarts::kDbus.data_id) {
            return false;
        } else {
            port.handle_downlink(data);
            return true;
        }
    });
}

namespace {
// GPIO 记录按线号到 GPIO 口的那根线; 本口没有的线号与 CAN/UART 的未知口一样不认。
template <typename F>
bool dispatch_gpio_line(uint8_t line, F&& f) {
    return ports::Registry::dispatch<spec::PortKind::kGpio>(
        data::DataId::kGpio, [line, &f](auto, auto& port) {
            gpio::Pin* pin = port.line(line);
            if (pin == nullptr)
                return false;
            f(*pin);
            return true;
        });
}
} // namespace

bool Vendor::dispatch_gpio_digital(uint8_t line, const data::GpioDigitalDataView& data) {
    if (data.timestamp_quarter_us.has_value())
        return false;
    return dispatch_gpio_line(line, [&data](gpio::Pin& pin) { pin.handle_digital_write(data); });
}

bool Vendor::dispatch_gpio_analog(uint8_t line, const data::GpioAnalogDataView& data) {
    return dispatch_gpio_line(line, [&data](gpio::Pin& pin) { pin.handle_analog_write(data); });
}

bool Vendor::dispatch_gpio_read(uint8_t line) {
    return dispatch_gpio_line(line, [](gpio::Pin& pin) { pin.handle_read_request(); });
}

bool Vendor::dispatch_buzzer(const data::BuzzerToneDataView& data) {
    return ports::Registry::dispatch<spec::PortKind::kBuzzer>(
        data::DataId::kBuzzer, [&data](auto, auto& port) {
            port.handle_tone(data);
            return true;
        });
}

core::protocol::Serializer& get_serializer() { return vendor->serializer(); }

bool uplink_session_active() { return vendor->session_established(); }

void poll_dfu_runtime_reboot() {
    if (!g_dfu_runtime_reboot_requested)
        return;

    if ((HAL_GetTick() - g_dfu_runtime_reboot_requested_tick) < kDfuRuntimeResetDelayMs)
        return;

    reset_system();
}

// TinyUSB 设备回调
extern "C" {

void tud_vendor_rx_cb(uint8_t itf, const uint8_t* buffer, uint32_t size) {
    if (itf != 0) [[unlikely]]
        return;

    usb::vendor->handle_downlink(
        {reinterpret_cast<const std::byte*>(buffer), size}, size < Vendor::kMaxPacketSize);
}

void tud_dfu_runtime_reboot_to_dfu_cb() {
    utility::boot_mailbox.request_enter_dfu();
    g_dfu_runtime_reboot_requested_tick = HAL_GetTick();
    g_dfu_runtime_reboot_requested = true;
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    usb::vendor->deactivate_session();
    usb::vendor->finish_downlink_transfer();
}

void tud_resume_cb() {}

void tud_mount_cb() {
    // 新主机必须自己完成 EP0 握手。
    usb::vendor->set_ep0_handshake_done(false);
}

void tud_umount_cb() {
    usb::vendor->deactivate_session();
    usb::vendor->finish_downlink_transfer();
}

} // extern "C"

} // namespace libhcs::firmware::usb
