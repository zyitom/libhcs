#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string_view>
#include <thread>

#include <libhcs/export.hpp>
#include <libhcs/transport.hpp>

#include "host/src/transport/transport.hpp"

namespace libhcs::host::transport::usb {

// 公开 Transport（libhcs::Transport）挡在 libusb 后面的适配层：Board 默认创建
// 它（按板型 USB 标识扫描打开），U2 的 usbfs 数据通路将来实现同一个公开接口
// 替掉这里，Board 与引擎不感知。
//
// transmit() 把调用方交来的字节按内部缓冲（一个 USB 包的量）切块拷入再提交；
// 多一次 memcpy，发生在 IO 线程而不是 RT 线程上——零拷贝路径是 U2 的事。
class UsbTransport final : public libhcs::Transport {
public:
    /// @param thread_setup 在 USB 事件线程上跑一次的钩子（BoardOptions 的同名
    ///                     字段）；可为空。
    UsbTransport(
        std::uint16_t vendor_id, std::span<const std::uint16_t> product_ids,
        std::string_view serial_filter, int io_thread_cpu, int io_thread_rt_priority,
        void (*thread_setup)() noexcept, bool dangerously_skip_version_checks = false);

    ~UsbTransport() final;

    UsbTransport(const UsbTransport&) = delete;
    UsbTransport& operator=(const UsbTransport&) = delete;
    UsbTransport(UsbTransport&&) = delete;
    UsbTransport& operator=(UsbTransport&&) = delete;

    void transmit(std::span<const std::byte> bytes) final;
    void receive(std::function<void(std::span<const std::byte>)> callback) final;
    void on_link_restart(std::function<void()> callback) final;
    ControlResult vendor_control(
        std::uint8_t request_type, std::uint8_t request, std::uint16_t index,
        std::span<std::byte> payload) final;
    [[nodiscard]] bool link_faulted() const noexcept final;
    bool try_recover_link() final;
    [[nodiscard]] std::string_view serial() const noexcept final;
    [[nodiscard]] int usb_bus_number() const noexcept final;
    void configure_session_thread(std::thread& thread) noexcept final;

private:
    // 把 Board 的无参钩子装进 AdvancedOptions 的有签名钩子（借 bind 的手法，
    // 但类要能被指名才能放进 unique_ptr——AdvancedOptions 不可拷移）。
    class SetupOptions;

    std::unique_ptr<::libhcs::board::AdvancedOptions> options_;
    std::unique_ptr<::libhcs::host::transport::Transport> internal_; // 内部 libusb 传输
};

} // namespace libhcs::host::transport::usb
