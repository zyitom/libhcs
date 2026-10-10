#include "host/src/transport/usb/usb_transport.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

#include "core/src/protocol/constant.hpp"
#include "host/src/transport/transport.hpp"
#include "libhcs/board/common.hpp"

namespace libhcs::host::transport::usb {

// AdvancedOptions 的钩子签名带 const AdvancedOptions&（v3 起如此）；Board 的
// 公开钩子是无参的。派生类持有无参钩子，经一个静态跳板喂给旧签名。
class UsbTransport::SetupOptions final : public ::libhcs::board::AdvancedOptions {
public:
    explicit SetupOptions(void (*setup)() noexcept)
        : setup_(setup) {
        thread_setup = [](const ::libhcs::board::AdvancedOptions& self) noexcept {
            static_cast<const SetupOptions&>(self).setup_();
        };
    }

private:
    void (*setup_)() noexcept;
};

UsbTransport::UsbTransport(
    std::uint16_t vendor_id, std::span<const std::uint16_t> product_ids,
    std::string_view serial_filter, int io_thread_cpu, int io_thread_rt_priority,
    void (*thread_setup)() noexcept, bool dangerously_skip_version_checks) {
    if (thread_setup != nullptr)
        options_ = std::make_unique<SetupOptions>(thread_setup);
    else
        options_ = std::make_unique<::libhcs::board::AdvancedOptions>();
    options_->set_dangerously_skip_version_checks(dangerously_skip_version_checks);
    options_->set_io_thread_affinity(io_thread_cpu, io_thread_rt_priority);
    // 扫描、打开、版本核对（产品串）都在这一步完成，找不到板或版本不符抛出。
    internal_ = create_transport(vendor_id, product_ids, serial_filter, *options_);
}

UsbTransport::~UsbTransport() = default;

void UsbTransport::transmit(std::span<const std::byte> bytes) {
    // 内部缓冲一块恰好一个 USB 包；这里只管切与拷。拿不到缓冲 = 传输挂了，
    // 剩下的字节丢弃——引擎侧的计数不区分这一层，反正同一块板已经在掉数据。
    while (!bytes.empty()) {
        const std::size_t chunk = std::min(bytes.size(), core::protocol::kProtocolBufferSize);
        auto buffer = internal_->acquire_transmit_buffer();
        if (!buffer) [[unlikely]]
            return;
        std::memcpy(buffer->data().data(), bytes.data(), chunk);
        internal_->transmit(std::move(buffer), chunk);
        bytes = bytes.subspan(chunk);
    }
}

void UsbTransport::receive(std::function<void(std::span<const std::byte>)> callback) {
    internal_->receive(std::move(callback));
}

void UsbTransport::on_link_restart(std::function<void()> callback) {
    internal_->on_link_restart(std::move(callback));
}

libhcs::Transport::ControlResult UsbTransport::vendor_control(
    std::uint8_t request_type, std::uint8_t request, std::uint16_t index,
    std::span<std::byte> payload) {
    using Internal = ::libhcs::host::transport::Transport;
    switch (internal_->vendor_control(request_type, request, index, payload)) {
    case Internal::ControlResult::kOk: return ControlResult::kOk;
    case Internal::ControlResult::kStalled: return ControlResult::kStalled;
    case Internal::ControlResult::kUnsupported: return ControlResult::kUnsupported;
    case Internal::ControlResult::kFailed: break;
    }
    return ControlResult::kFailed;
}

bool UsbTransport::link_faulted() const noexcept { return internal_->link_faulted(); }

bool UsbTransport::try_recover_link() { return internal_->try_recover_link(); }

std::string_view UsbTransport::serial() const noexcept { return internal_->serial(); }

int UsbTransport::usb_bus_number() const noexcept { return internal_->usb_bus_number(); }

void UsbTransport::configure_session_thread(std::thread& thread) noexcept {
    internal_->configure_session_thread(thread);
}

} // namespace libhcs::host::transport::usb
