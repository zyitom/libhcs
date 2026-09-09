#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include "core/src/utility/assert.hpp"
#include "firmware/ch32_board/app/src/link/host_session.hpp"
#include "firmware/ch32_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::usb {

// Thin WCH USBSS bulk-endpoint transport, defined in vendor.cpp. These are what
// TinyUSB's tud_vendor_* calls are on the other boards; TinyUSB has no USB 3.0
// support at all (tusb_speed_t stops at HIGH), so the WCH stack drives EP1
// directly. Bulk IN (device->host uplink) and bulk OUT (host->device downlink)
// are the libhcs uplink/downlink pipes.
namespace ss {
bool tx_ready();                                 // bulk IN endpoint free?
bool tx_write(const uint8_t* data, size_t size); // arm one IN packet
bool tx_write_zlp();                             // arm a zero-length IN packet
bool enumerated();                               // device configured on the host?
void poll_link_reset();                          // recover the TX gate on link loss
} // namespace ss

// USB SuperSpeed vendor-class host transport: keeps only the transmission shape
// (max-packet chunking + ZLP termination). Session lifecycle and downlink
// dispatch live in the shared link::HostSession, exactly as on hpm_board.
class Vendor : public link::HostSession {
public:
    using Lazy = utility::Lazy<Vendor>;

    // USB 3.0 SuperSpeed bulk max packet size. Unlike the TinyUSB boards this is
    // fixed: the SS descriptors advertise 1024 and there is no FS/HS fallback to
    // switch it (see libhcs_USBSS_HS_FALLBACK).
    static constexpr size_t kMaxPacketSize = 1024;

    Vendor() = default;

    // USB transfers have boundaries: a short packet finishes the transfer, so
    // framing recovery can resynchronize there.
    void handle_downlink(std::span<const std::byte> buffer, bool finished) {
        link::HostSession::handle_downlink(buffer);
        if (finished)
            finish_downlink_transfer();
    }

    bool try_transmit() {
        const auto* batch = next_batch();
        if (!batch)
            return false;

        if (!ss::tx_ready())
            return false;

        const auto data = batch->data();
        const auto target_size = std::min(data.size() - transmitted_size_, kMaxPacketSize);

        if (target_size) {
            const auto* src = reinterpret_cast<const uint8_t*>(data.data() + transmitted_size_);
            core::utility::assert_debug(ss::tx_write(src, target_size));
        } else {
            core::utility::assert_debug(ss::tx_write_zlp());
        }

        transmitted_size_ += target_size;
        // A transfer ends on a short packet; an exact multiple of the max packet
        // size needs an explicit ZLP, which the next call emits (target_size 0).
        if (transmitted_size_ == data.size() && target_size < kMaxPacketSize) {
            finish_batch();
            transmitted_size_ = 0;
        }

        return true;
    }

private:
    void session_activated_callback() override { transmitted_size_ = 0; }

    size_t transmitted_size_ = 0;
};

inline constinit Vendor::Lazy vendor;

} // namespace libhcs::firmware::usb
