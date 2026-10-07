#include "firmware/mc02/app/src/usb/vendor.hpp"

#include <cstddef>
#include <cstdint>

#include <main.h>

#include "core/src/protocol/serializer.hpp"
#include "firmware/mc02/app/src/diag/usb_rx_hist.hpp"
#include "firmware/mc02/app/src/ports.hpp"
#include "firmware/mc02/app/src/spi/bmi088/service.hpp"
#include "firmware/mc02/app/src/utility/boot_mailbox.hpp"

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

core::protocol::Serializer& get_serializer() { return vendor->serializer(); }

// 每个口按自己的 suspend() 停: CAN 下总线、串口停收(待发字节照常发完)、IMU 停采样并
// 忘掉"在跑"(下一个主机声明同样的设置时要重新启动它)、输出引脚拉低。口的全集就是
// 注册表, 这里不另列一遍。
void Vendor::stop_channels() {
    ports::Registry::suspend_all();
    // 时间基准同样随会话走: 下一个主机的清单要了才再开。
    sync::time_sync_stop();
}

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
    return ports::Registry::dispatch<spec::PortKind::kUart>(id, [&data](auto, auto& port) {
        // 只收不发的 DBUS 口没有下行: 发给它的数据与发给不存在的口同一回答。
        if constexpr (requires { port.handle_downlink(data); }) {
            port.handle_downlink(data);
            return true;
        } else {
            return false;
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

bool uplink_session_active() { return vendor->session_established(); }

// tud_dfu_runtime_reboot_to_dfu_cb() 的延迟半边: 等请求 DFU 的那次控制传输有
// 时间完成后再复位。由主循环轮询, 延迟期间 CAN/UART 转发照常运行。
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

    diag::usb_rx_hist::note();

    // "主机的传输在此结束"判定的是短传输, 而非短包: DWC2 结束一次 OUT 传输的条件
    // 是请求的 rx_xfer_len 已收满, 或收到短于 wMaxPacketSize 的包。因此长度小于
    // 请求值即意味着后者。若改与端点的 64 字节比较, 则 rx_xfer_len 一旦超过 64,
    // 每个包之后传输都会被判结束, 协议帧会被拆散。
    usb::vendor->handle_downlink(
        {reinterpret_cast<const std::byte*>(buffer), size}, size < CFG_TUD_VENDOR_RX_EPSIZE);
}

void tud_dfu_runtime_reboot_to_dfu_cb() {
    utility::boot_mailbox.request_enter_dfu();
    __DSB();
    __ISB();
    // TinyUSB 在 CONTROL_STAGE_SETUP 调用本回调, 紧接在 tud_control_status() 把
    // ZLP 排队之后、其真正发出之前。在 STM32H723 上此处直接 NVIC_SystemReset() 会
    // 在复位传播前的窗口内损坏 mailbox 写入(STM32F4 直接复位可行, 本芯片不同)。
    // 把复位推迟到 poll_dfu_runtime_reboot(), 让 ZLP 先完成。
    g_dfu_runtime_reboot_requested_tick = HAL_GetTick();
    g_dfu_runtime_reboot_requested = true;
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    // 忘 EP0 握手在 session_deactivated_callback() 里, 与租约到期同一路径。
    usb::vendor->deactivate_session();
    usb::vendor->finish_downlink_transfer();
}

void tud_resume_cb() {}

void tud_mount_cb() {
    // 新主机必须自己完成 EP0 握手, 上一次握手声明过的通道随之作废。
    usb::vendor->set_ep0_handshake_done(false);
    Vendor::stop_channels();
}

void tud_umount_cb() {
    usb::vendor->deactivate_session();
    usb::vendor->finish_downlink_transfer();
}

} // extern "C"

} // namespace libhcs::firmware::usb
