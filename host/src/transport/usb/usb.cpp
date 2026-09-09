#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <format>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <libusb.h>
#include <pthread.h>
#include <sched.h>

#include "core/src/protocol/constant.hpp"
#include "core/src/utility/assert.hpp"
#include "host/src/logging/logging.hpp"
#include "host/src/transport/transport.hpp"
#include "host/src/transport/usb/device_scanner.hpp"
#include "host/src/transport/usb/helper.hpp"
#include "host/src/utility/final_action.hpp"
#include "host/src/utility/pi_mutex.hpp"
#include "host/src/utility/ring_buffer.hpp"

namespace libhcs::host::transport::usb {

namespace {

// libusb keeps completion statuses and error codes in two different enums, so
// libusb_error_name() prints a wrong-but-plausible string for a status. Name
// them here instead.
constexpr std::string_view transfer_status_name(libusb_transfer_status status) noexcept {
    switch (status) {
    case LIBUSB_TRANSFER_COMPLETED: return "COMPLETED";
    case LIBUSB_TRANSFER_ERROR: return "ERROR";
    case LIBUSB_TRANSFER_TIMED_OUT: return "TIMED_OUT";
    case LIBUSB_TRANSFER_CANCELLED: return "CANCELLED";
    case LIBUSB_TRANSFER_STALL: return "STALL";
    case LIBUSB_TRANSFER_NO_DEVICE: return "NO_DEVICE";
    case LIBUSB_TRANSFER_OVERFLOW: return "OVERFLOW";
    }
    return "UNKNOWN";
}

} // namespace

class Usb : public Transport {
public:
    explicit Usb(
        uint16_t usb_vid, std::span<const uint16_t> usb_pids, std::string_view serial_filter,
        const ConnectionOptions& options)
        : logger_(logging::get_logger())
        , free_transmit_transfers_(kTransmitTransferCount) {

        usb_init(usb_vid, usb_pids, serial_filter, options);
        utility::FinalAction rollback_on_failure{[this]() noexcept {
            destroy_free_transmit_transfers();
            free_dev_mem_slabs();
            libusb_release_interface(libusb_device_handle_, kTargetInterface);
            libusb_close(libusb_device_handle_);
            libusb_exit(libusb_context_);
        }};

        // Opt-in uplink size instrumentation. How full each bulk transfer runs is
        // not derivable from the decoded stream -- the fields of one transfer are
        // handed to the callback individually, and their sizes say nothing about
        // how they were packed -- and usbmon needs root. So the transport counts
        // its own completion lengths when asked, and stays untouched otherwise.
        if (const char* enable = std::getenv("libhcs_USB_RX_HISTOGRAM");
            enable && enable[0] == '1') {
            rx_length_histogram_ =
                std::make_unique<std::atomic<uint32_t>[]>(core::protocol::kProtocolBufferSize + 1);
        }

        if (const char* enable = std::getenv("libhcs_USB_CB_TIMING"); enable && enable[0] == '1') {
            callback_timing_ = true;
        }

        // Probe dev_mem availability: if the kernel usbfs driver supports DMA-coherent
        // buffers the allocation will succeed; fall back to heap allocation silently.
        auto* probe = libusb_dev_mem_alloc(libusb_device_handle_, 1);
        if (probe) {
            libusb_dev_mem_free(libusb_device_handle_, probe, 1);
            dev_mem_available_ = true;
            logger_.info("libusb dev_mem (zero-copy) available");
        }

        init_transmit_transfers();

        // Affinity and priority are applied before the caller's own hook, so a
        // hook that wants to override either of them still can.
        const int io_cpu = options.io_thread_cpu;
        const int io_prio = options.io_thread_rt_priority;
        if (options.thread_setup || io_cpu >= 0) {
            std::atomic<bool> thread_setup_done{false};
            event_thread_ = std::thread{[this, &options, io_cpu, io_prio, &thread_setup_done]() {
                apply_io_thread_scheduling(io_cpu, io_prio);
                if (options.thread_setup)
                    options.thread_setup(options);
                thread_setup_done.store(true, std::memory_order_release);
                thread_setup_done.notify_one();
                handle_events();
            }};
            // Wait until thread_setup returns, so any bound options state remains alive.
            thread_setup_done.wait(false, std::memory_order_acquire);
        } else {
            event_thread_ = std::thread{[this]() { handle_events(); }};
        }

        rollback_on_failure.disable();
    }

    Usb(const Usb&) = delete;
    Usb& operator=(const Usb&) = delete;
    Usb(Usb&&) = delete;
    Usb& operator=(Usb&&) = delete;

    ~Usb() override {
        dump_rx_length_histogram();
        dump_callback_timing();
        {
            const std::scoped_lock guard{transmit_transfer_mutex_};
            stop_handling_events_.store(true, std::memory_order::relaxed);
        }
        transmit_transfer_cv_.notify_all();
        destroy_free_transmit_transfers();

        // Cancel the pending receives explicitly instead of leaving it to
        // libusb_close(). Close does not reliably complete IN transfers sitting on
        // a second interface: measured, with the CAN endpoint pair claimed, 16 of
        // 16 transfers on 0x82 were still outstanding after close (6 of 16 when
        // the interface was released first), and handle_events() then spun on an
        // active_transfers_ count that could never reach zero. Cancelling before
        // close makes every one of them deliver LIBUSB_TRANSFER_CANCELLED through
        // the normal callback path, which is what decrements the count.
        {
            const std::scoped_lock guard{rx_transfer_mutex_};
            for (auto* rx : rx_transfers_)
                libusb_cancel_transfer(rx->transfer);
        }

        // ...and wait for those cancellations to actually be delivered before
        // closing the handle. Cancelling and closing back to back loses whatever
        // the event thread had not yet reaped: measured, 6 of 32 receive
        // transfers stayed outstanding that way, and handle_events() then span on
        // a count that could never reach zero. Bounded so a wedged controller
        // cannot turn teardown into a hang -- the loop below is what still
        // guarantees termination if that ever happens.
        for (int i = 0; i < kTeardownDrainRounds; i++) {
            {
                const std::scoped_lock guard{rx_transfer_mutex_};
                if (rx_transfers_.empty())
                    break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
        }

        // Before the handle goes away, and after the drain above has accounted
        // for every buffer.
        free_dev_mem_slabs();

        libusb_release_interface(libusb_device_handle_, kTargetInterface);

        // libusb_close() reliably cancels all pending transfers and invokes their callbacks,
        // avoiding race conditions present in other cancellation methods
        libusb_close(libusb_device_handle_);

        // Guarantees join() returns even if some transfer never reports back:
        // whatever is still counted at this point can no longer be delivered,
        // because the handle it belonged to is closed.
        abandon_remaining_transfers_.store(true, std::memory_order::relaxed);

        if (event_thread_.joinable())
            event_thread_.join();

        libusb_exit(libusb_context_);
    }

    std::unique_ptr<TransportBuffer> acquire_transmit_buffer() noexcept override {
        TransferWrapper* transfer = nullptr;
        {
            std::unique_lock guard{transmit_transfer_mutex_};
            transmit_transfer_cv_.wait(guard, [this]() {
                return stop_handling_events_.load(std::memory_order::relaxed)
                    || link_faulted_.load(std::memory_order::relaxed)
                    || free_transmit_transfers_.readable() != 0;
            });
            // A faulted link must not park a caller here forever waiting for a
            // completion that can no longer arrive. StreamBuffer treats nullptr
            // as "no buffer this round" and drops the batch, which is the
            // truthful answer.
            if (stop_handling_events_.load(std::memory_order::relaxed)
                || link_faulted_.load(std::memory_order::relaxed))
                return nullptr;
            free_transmit_transfers_.pop_front(
                [&transfer](TransferWrapper* value) noexcept { transfer = value; });
        }
        core::utility::assert_debug(transfer != nullptr);

        return std::unique_ptr<TransportBuffer>{transfer};
    }

    void transmit(std::unique_ptr<TransportBuffer> buffer, size_t size) override {
        core::utility::assert_debug(static_cast<bool>(buffer));

        if (size > core::protocol::kProtocolBufferSize)
            throw std::invalid_argument("Transmit size exceeds maximum transfer length");

        // Neither of the two failures below may throw, and that is not a style
        // choice: the only caller is StreamBuffer::finalize_buffer(), which is
        // noexcept, so an exception here is std::terminate() with extra steps.
        // Measured 2026-09-05 by resetting the device under a running flood --
        // the throw that used to live on the submit-failure path did exactly
        // that. Both failures recycle the buffer instead; dropping a packet on a
        // link that is already gone is the honest outcome, and the counters and
        // the fault log say so.
        if (link_faulted_.load(std::memory_order::relaxed)) [[unlikely]] {
            const uint64_t count =
                transfer_errors_.transmit_dropped.fetch_add(1, std::memory_order::relaxed) + 1;
            if (logging::should_log_occurrence(count))
                logger_.error("Dropping a transmit on a faulted link (x{})", count);
            release_transmit_buffer(std::move(buffer));
            return;
        }

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
        auto& transfer = static_cast<TransferWrapper*>(buffer.get())->transfer_;
        transfer->length = static_cast<int>(size);

        const int ret = libusb_submit_transfer(transfer);
        if (ret != 0) [[unlikely]] {
            const uint64_t count =
                transfer_errors_.transmit_error.fetch_add(1, std::memory_order::relaxed) + 1;
            if (logging::should_log_occurrence(count))
                logger_.error(
                    "Failed to submit transmit transfer: {} ({}) (x{}); the packet was lost", ret,
                    helper::libusb_errname(ret), count);
            if (ret == LIBUSB_ERROR_NO_DEVICE)
                fault("device disconnected while submitting a transmit transfer");
            // The wrapper never reached libusb, so it is still ours. Letting the
            // unique_ptr destroy it instead would shrink the pool by one and log
            // an "externally destroyed" complaint on the way out.
            release_transmit_buffer(std::move(buffer));
            return;
        }

        // If success: Ownership is transferred to libusb
        std::ignore = buffer.release();
    }

    void release_transmit_buffer(std::unique_ptr<TransportBuffer> buffer) override {
        core::utility::assert_debug(static_cast<bool>(buffer));

        {
            const std::scoped_lock guard{transmit_transfer_mutex_};
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
            auto* wrapper = static_cast<TransferWrapper*>(buffer.release());
            free_transmit_transfers_.emplace_back(wrapper);
        }
        transmit_transfer_cv_.notify_one();
    }

    // Fired after a reconnect. The protocol layer uses it to drop a protocol
    // field that was half-received when the device vanished -- the deserializer
    // carries that state across transfers, so without this the first bytes of
    // the new connection get appended to the truncated old field and the
    // SESSION_ACK that follows is mis-parsed.
    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    void on_link_restart(std::function<void()> callback) override {
        link_restart_callback_ = std::move(callback);
    }

    void receive(std::function<void(std::span<const std::byte>)> callback) override {
        if (!callback)
            throw std::invalid_argument{"Callback function cannot be null"};
        if (receive_callback_)
            throw std::logic_error{"Receive function can only be called once"};

        receive_callback_ = std::move(callback);
        receive_endpoint_ = kInEndpoint;
        init_receive_transfers(kInEndpoint);
    }

    // EP0 vendor request, synchronous. libusb runs this through its own
    // event-handling path, which coexists with the dedicated event thread
    // above -- a sync transfer that cannot take the event lock waits on the
    // waiters lock instead of spinning, so the two do not fight.
    //
    // Synchronous is right here even though every data path in this class is
    // asynchronous: configuration happens once, while the board object is being
    // constructed, and the caller has nothing to do until the board has
    // answered. It costs one round trip (~76 us on this host, usb_ep0_rtt) and
    // never touches the steady-state path.
    //
    // LIBUSB_ERROR_PIPE is a STALL, which the board uses to say "rejected", so
    // it is a normal answer here rather than an error.
    ControlResult vendor_control(
        uint8_t request_type, uint8_t request, uint16_t index,
        std::span<std::byte> payload) override {
        // There is no handle between a fault and a successful re-open, and the
        // keepalive walks straight into this: its next pass re-runs the board's
        // before-session hook, whose first act is an EP0 read of the interface
        // descriptor. Passing a null handle to libusb segfaults inside the
        // library -- measured 2026-09-06 by deauthorizing the port, which keeps
        // the board away long enough for a re-open to fail. A device reset never
        // showed it: the board was always back before the first retry.
        if (libusb_device_handle_ == nullptr) [[unlikely]]
            return ControlResult::kFailed;

        const int ret = libusb_control_transfer(
            libusb_device_handle_, request_type, request, 0, index,
            reinterpret_cast<unsigned char*>(payload.data()), static_cast<uint16_t>(payload.size()),
            kControlTimeoutMs);
        if (ret == LIBUSB_ERROR_PIPE)
            return ControlResult::kStalled;
        if (ret < 0) {
            logger_.warn(
                "EP0 vendor request 0x{:02x} (index {}) failed: {} ({})", request, index, ret,
                helper::libusb_errname(ret));
            return ControlResult::kFailed;
        }
        // A short transfer means the two sides disagree about the payload, which
        // is the one outcome that must not be read as success: the caller would
        // decode uninitialized bytes as the board's answer.
        if (std::cmp_not_equal(ret, payload.size()))
            return ControlResult::kFailed;
        return ControlResult::kOk;
    }

    // Cheap, local repair of a link that libusb still believes is up: clear both
    // bulk halts and rebuild whatever the receive pool lost. Called from the
    // keepalive thread once the board has stopped answering, never from the
    // event thread -- libusb_clear_halt() is a synchronous control transfer and
    // would deadlock against the event handling its own completion needs.
    //
    // clear_halt is safe to issue on a healthy endpoint too: on this board the
    // device-side handler only writes ENDPTCTRL (toggle reset + clear stall) and
    // the primed transfer survives it, measured 2026-09-05. It is still only
    // done on a link that has already failed, because resetting the data toggle
    // mid-stream can cost the packet in flight.
    bool try_recover_link() override {
        // A faulted link has nothing left to clear -- the handle is dead. The
        // only repair at this level is to open the device again.
        if (link_faulted_.load(std::memory_order::relaxed))
            return try_reconnect();

        if (libusb_device_handle_ == nullptr) [[unlikely]]
            return try_reconnect(); // nothing to clear a halt on

        const uint64_t attempt =
            link_recovery_attempts_.fetch_add(1, std::memory_order::relaxed) + 1;
        bool acted = false;

        for (const unsigned char endpoint : {kInEndpoint, kOutEndpoint}) {
            const int ret = libusb_clear_halt(libusb_device_handle_, endpoint);
            if (ret == 0) {
                acted = true;
                continue;
            }
            if (ret == LIBUSB_ERROR_NO_DEVICE) {
                fault("device disconnected during link recovery");
                return false;
            }
            logger_.warn(
                "Link recovery: clear_halt(0x{:02x}) failed: {} ({})", endpoint, ret,
                helper::libusb_errname(ret));
        }

        const size_t restored = replenish_receive_transfers();
        logger_.warn(
            "Link recovery attempt {}: halts cleared={}, receive transfers restored={} (pool "
            "{}/{})",
            attempt, acted, restored, receive_transfer_count(), kReceiveTransferCount);

        return acted || restored != 0;
    }

    bool link_faulted() const noexcept override {
        return link_faulted_.load(std::memory_order::relaxed);
    }

private:
    // Bring every outstanding transfer home and let the device go, WITHOUT
    // touching the libusb context or the event thread polling it. That is the
    // difference from the destructor's teardown: this one is survivable.
    void quiesce_device() noexcept {
        if (libusb_device_handle_ == nullptr)
            return;

        {
            const std::scoped_lock guard{rx_transfer_mutex_};
            for (auto* rx : rx_transfers_)
                libusb_cancel_transfer(rx->transfer);
        }
        for (int i = 0; i < kTeardownDrainRounds && receive_transfer_count() != 0; i++)
            std::this_thread::sleep_for(std::chrono::milliseconds{2});

        // Every transmit wrapper must be back from libusb before the pool is
        // destroyed -- freeing one the controller still owns is a use-after-free
        // that only shows up under load. A dead device completes them promptly
        // with NO_DEVICE, which is what puts them back.
        for (int i = 0; i < kTeardownDrainRounds; i++) {
            {
                const std::scoped_lock guard{transmit_transfer_mutex_};
                if (free_transmit_transfers_.readable() == kTransmitTransferCount)
                    break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
        }

        destroy_free_transmit_transfers();
        free_dev_mem_slabs(); // needs the handle, so before the close below
        libusb_release_interface(libusb_device_handle_, kTargetInterface);
        libusb_close(libusb_device_handle_);
        libusb_device_handle_ = nullptr;
    }

    // Re-select the SAME board (by the serial recorded at construction), claim
    // it and rebuild both transfer pools. The protocol layer above needs no
    // notification: its keepalive already re-runs the EP0 handshake and opens a
    // fresh session whenever the session is down.
    bool reopen_device() noexcept {
        try {
            const std::string_view filter = serial_.empty() ? std::string_view{} : serial_;
            libusb_device_handle_ = DeviceScanner::select_device(
                libusb_context_, vendor_id_, product_ids_, filter, reconnect_options_);
        } catch (const std::exception& exception) {
            const uint64_t count = reconnect_failures_.fetch_add(1, std::memory_order::relaxed) + 1;
            if (logging::should_log_occurrence(count))
                logger_.warn(
                    "Reconnect: the board is not back yet (x{}): {}", count, exception.what());
            libusb_device_handle_ = nullptr;
            return false;
        }

        if (const int ret = libusb_claim_interface(libusb_device_handle_, kTargetInterface);
            ret != 0) [[unlikely]] {
            // Throttled like the scan failure above it: a board that stays away
            // fails this on every retry, four times a second, for as long as the
            // outage lasts. (Deauthorizing the port is what makes libusb_open
            // succeed and the claim fail -- the device is listed but has no
            // usable interfaces.)
            const uint64_t count = reconnect_failures_.fetch_add(1, std::memory_order::relaxed) + 1;
            if (logging::should_log_occurrence(count))
                logger_.warn(
                    "Reconnect: failed to claim interface {}: {} ({}) (x{})", kTargetInterface, ret,
                    helper::libusb_errname(ret), count);
            libusb_close(libusb_device_handle_);
            libusb_device_handle_ = nullptr;
            return false;
        }

        // Re-probed rather than assumed: the same board can come back on a
        // controller whose usbfs does not offer DMA-coherent buffers.
        dev_mem_available_ = false;
        if (auto* probe = libusb_dev_mem_alloc(libusb_device_handle_, 1)) {
            libusb_dev_mem_free(libusb_device_handle_, probe, 1);
            dev_mem_available_ = true;
        }

        // Before the receive pool is armed, which is the only window where this
        // is serialized against deserializer_.feed() the way the interface
        // requires: quiesce_device() left no receive transfer outstanding, so no
        // callback can be running, and none can start until the loop below.
        if (link_restart_callback_)
            link_restart_callback_();

        try {
            init_transmit_transfers();
            if (receive_callback_)
                init_receive_transfers(receive_endpoint_);
        } catch (const std::exception& exception) {
            logger_.error("Reconnect: failed to rebuild the transfer pools: {}", exception.what());
            quiesce_device();
            return false;
        }

        // Senders parked in acquire_transmit_buffer() are waiting on a pool that
        // was empty a moment ago.
        transmit_transfer_cv_.notify_all();
        return true;
    }

    // Escalation from try_recover_link() once the link is faulted: clearing a
    // halt cannot help when the handle itself is dead, but re-opening the device
    // can, and after a reset or a DFU reflash the same board is back within a
    // second or two.
    bool try_reconnect() {
        const std::scoped_lock guard{reconnect_mutex_};
        if (!link_faulted_.load(std::memory_order::relaxed))
            return true; // another attempt already succeeded

        quiesce_device();
        if (!reopen_device())
            return false;

        link_faulted_.store(false, std::memory_order::release);
        reconnect_failures_.store(0, std::memory_order::relaxed);
        logger_.warn(
            "USB link re-opened (serial {}); the session will be re-established by the keepalive",
            serial_.empty() ? "unknown" : serial_);
        return true;
    }

public:
private:
    static constexpr unsigned int kControlTimeoutMs = 1000;

    class TransferWrapper : public TransportBuffer {
        friend class Usb;

    public:
        explicit TransferWrapper(Usb& self)
            : self_(self)
            , transfer_(self_.create_libusb_transfer())
            , buffer_(self_.alloc_transfer_buffer())
            , buffer_is_dev_mem_(self_.dev_mem_available_) {}

        TransferWrapper(const TransferWrapper&) = delete;
        TransferWrapper& operator=(const TransferWrapper&) = delete;
        TransferWrapper(TransferWrapper&&) = delete;
        TransferWrapper& operator=(TransferWrapper&&) = delete;

        ~TransferWrapper() override {
            if (transfer_) {
                logging::get_logger().error(
                    "USB TransferBuffer {} was destroyed externally - this is undefined behavior. "
                    "Buffers must be returned via transmit() or "
                    "release_transmit_buffer(). ",
                    static_cast<void*>(this));
                destroy();
            }
        }

        BufferSpanType data() const noexcept override {
            return BufferSpanType{
                reinterpret_cast<std::byte*>(buffer_), core::protocol::kProtocolBufferSize};
        }

        void destroy() noexcept {
            self_.free_transfer_buffer(buffer_, buffer_is_dev_mem_);
            buffer_ = nullptr;
            self_.destroy_libusb_transfer(transfer_);
            transfer_ = nullptr;
        }

    private:
        Usb& self_;

        libusb_transfer* transfer_;
        unsigned char* buffer_;
        bool buffer_is_dev_mem_;
        // Which pool this wrapper goes back to on completion. Set once at
        // construction; a buffer never migrates between channels.
    };

    void usb_init(
        uint16_t vendor_id, std::span<const uint16_t> product_ids, std::string_view serial_filter,
        const ConnectionOptions& options) {
        // Kept for reconnection. The serial filter the caller passed may be
        // empty (match any board of this product), which is fine for the first
        // open and wrong for a re-open: after a reset there may be two boards on
        // the bus and only one of them is the board this object was talking to.
        // The exact serial is read back below, once there is a handle.
        vendor_id_ = vendor_id;
        product_ids_.assign(product_ids.begin(), product_ids.end());
        reconnect_options_.dangerously_skip_version_checks =
            options.dangerously_skip_version_checks;

        if (const int ret = libusb_init(&libusb_context_); ret != 0) [[unlikely]] {
            throw std::runtime_error(
                std::format(
                    "Failed to initialize libusb: {} ({})", ret, helper::libusb_errname(ret)));
        }
        utility::FinalAction exit_libusb{[this]() noexcept { libusb_exit(libusb_context_); }};

        libusb_device_handle_ = DeviceScanner::select_device(
            libusb_context_, vendor_id, product_ids, serial_filter, options);
        utility::FinalAction close_device_handle{
            [this]() noexcept { libusb_close(libusb_device_handle_); }};

        if (const int ret = libusb_claim_interface(libusb_device_handle_, kTargetInterface);
            ret != 0) [[unlikely]] {
            throw std::runtime_error(
                std::format(
                    "Failed to claim interface {}: {} ({})", kTargetInterface, ret,
                    helper::libusb_errname(ret)));
        }

        serial_ = read_serial(libusb_device_handle_);

        // Libusb successfully initialized
        close_device_handle.disable();
        exit_libusb.disable();
    }

    // The device's own serial string, which is what a re-open matches on. Empty
    // when it cannot be read; reconnection then falls back to the caller's
    // original filter and refuses if that is ambiguous.
    static std::string read_serial(libusb_device_handle* handle) noexcept {
        libusb_device_descriptor descriptor{};
        if (libusb_get_device_descriptor(libusb_get_device(handle), &descriptor) != 0
            || descriptor.iSerialNumber == 0)
            return {};
        std::array<unsigned char, 128> buffer{};
        const int length = libusb_get_string_descriptor_ascii(
            handle, descriptor.iSerialNumber, buffer.data(), static_cast<int>(buffer.size()));
        if (length <= 0)
            return {};
        return std::string{
            reinterpret_cast<const char*>(buffer.data()), static_cast<std::size_t>(length)};
    }

    void init_transmit_transfers() {
        TransferWrapper* transmit_transfers[kTransmitTransferCount] = {};
        try {
            for (auto& wrapper : transmit_transfers) {
                wrapper = new TransferWrapper{*this};
                auto* transfer = wrapper->transfer_;

                libusb_fill_bulk_transfer(
                    transfer, libusb_device_handle_, kOutEndpoint, wrapper->buffer_, 0,
                    [](libusb_transfer* transfer) {
                        auto* wrapper = static_cast<TransferWrapper*>(transfer->user_data);
                        wrapper->self_.usb_transmit_complete_callback(wrapper);
                    },
                    wrapper, 0);
                transfer->flags = libusb_transfer_flags::LIBUSB_TRANSFER_ADD_ZERO_PACKET;
            }
        } catch (...) {
            for (auto& wrapper : transmit_transfers) {
                if (wrapper) {
                    wrapper->destroy();
                    delete wrapper;
                }
            }
            throw;
        }

        auto* iter = transmit_transfers;
        free_transmit_transfers_.push_back_n(
            [&iter]() noexcept { return *iter++; }, kTransmitTransferCount);
    }

    // Pins this thread and optionally raises it to SCHED_FIFO. Failures are
    // logged, never fatal: an unprivileged process should still get a working
    // link, only with the free-running tail.
    void apply_io_thread_scheduling(int cpu, int rt_priority) noexcept {
        if (cpu < 0)
            return;
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(static_cast<unsigned>(cpu), &set);
        if (const int ret = pthread_setaffinity_np(pthread_self(), sizeof(set), &set); ret != 0) {
            logger_.warn("could not pin the USB event thread to CPU {}: {}", cpu, ret);
        } else {
            logger_.info("USB event thread pinned to CPU {}", cpu);
        }
        if (rt_priority <= 0)
            return;
        const sched_param param{.sched_priority = rt_priority};
        if (const int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param); ret != 0) {
            logger_.warn(
                "could not raise the USB event thread to SCHED_FIFO {}: {} (needs CAP_SYS_NICE)",
                rt_priority, ret);
        } else {
            logger_.info("USB event thread at SCHED_FIFO {}", rt_priority);
        }
    }

    void handle_events() {
        // Bounded wait rather than libusb_handle_events(), whose internal timeout
        // is 60 s. The loop only re-tests active_transfers_ between calls, so a
        // teardown whose last completions arrive while this thread is parked in a
        // long poll leaves ~Usb() blocked in join() for that whole timeout. The
        // single-channel case happened not to hit it; adding the CAN endpoint
        // pair -- 16 more IN transfers that idle indefinitely because the board
        // only writes to them when it has CAN to send -- made it reproducible.
        //
        // This costs ten wakeups a second on an otherwise idle event thread and
        // nothing at all when traffic is flowing: an arriving completion still
        // wakes the poll immediately, exactly as before.
        static constexpr timeval kPollTimeout{.tv_sec = 0, .tv_usec = 100'000};

        // REJECTED, do not reintroduce: busy polling this loop with a zero
        // timeout deadlocks any caller that pins and raises itself before
        // opening the transport. The event thread is spawned from this class's
        // constructor and inherits the creating thread's affinity and policy, so
        // with a tool like can_loopback_latency -- SCHED_FIFO 80 pinned to CPU 0
        // -- both threads land on one core at equal priority, the spinner never
        // yields, equal-priority FIFO threads never preempt each other, and
        // host-tuning.sh has already set sched_rt_runtime_us=-1 so no RT throttle
        // breaks it. Measured: zero round trips completed in 60 s. Removing the
        // parked poll saves a scheduler wake-up worth a few microseconds and
        // costs the whole link.
        // Runs while there is anything left to reap OR no teardown has been asked
        // for. The second half is what lets a reconnect happen underneath: while
        // the device is being re-opened the transfer count legitimately passes
        // through zero, and the old "while (active_transfers_)" would have taken
        // this thread with it, leaving a re-opened device with nobody polling
        // its completions. Teardown is unchanged -- stop_handling_events_ is set
        // first there, so the loop still exits as soon as the last transfer is
        // reaped.
        while ((active_transfers_.load(std::memory_order::relaxed) != 0
                || !stop_handling_events_.load(std::memory_order::relaxed))
               && !abandon_remaining_transfers_.load(std::memory_order::relaxed)) {
            timeval timeout = kPollTimeout;
            const int ret =
                libusb_handle_events_timeout_completed(libusb_context_, &timeout, nullptr);
            // Ignored until 2026-09-05, which meant a context that had stopped
            // handling events looked exactly like an idle one: this loop would
            // spin at full speed with no completion ever arriving and nothing
            // said. INTERRUPTED is the ordinary "a signal arrived" answer and is
            // not an error.
            if (ret != 0 && ret != LIBUSB_ERROR_INTERRUPTED) [[unlikely]] {
                const uint64_t count =
                    event_loop_errors_.fetch_add(1, std::memory_order::relaxed) + 1;
                if (logging::should_log_occurrence(count))
                    logger_.error(
                        "libusb event handling failed: {} ({}) (x{})", ret,
                        helper::libusb_errname(ret), count);
                if (ret == LIBUSB_ERROR_NO_DEVICE)
                    fault("device disconnected while handling events");
            }
        }
    }

    // Thin wrapper for RX transfers to track buffer allocation type alongside the transfer.
    struct RxTransfer {
        Usb* self;
        libusb_transfer* transfer;
        unsigned char* buffer;
        bool is_dev_mem;
    };

    void init_receive_transfers(unsigned char endpoint) {
        for (size_t i = 0; i < kReceiveTransferCount; i++) {
            if (const int ret = submit_receive_transfer(endpoint); ret != 0) [[unlikely]]
                throw std::runtime_error(
                    std::format(
                        "Failed to submit receive transfer: {} ({})", ret,
                        helper::libusb_errname(ret)));
        }
    }

    // One place where a receive transfer is built, registered and submitted, so
    // the initial arming and the recovery path cannot drift apart. Returns a
    // libusb error code; on failure the transfer is already retired.
    [[nodiscard]] int submit_receive_transfer(unsigned char endpoint) {
        auto* rx = new RxTransfer{
            .self = this,
            .transfer = create_libusb_transfer(),
            .buffer = alloc_transfer_buffer(),
            .is_dev_mem = dev_mem_available_,
        };

        libusb_fill_bulk_transfer(
            rx->transfer, libusb_device_handle_, endpoint, rx->buffer,
            static_cast<int>(core::protocol::kProtocolBufferSize),
            [](libusb_transfer* transfer) {
                static_cast<RxTransfer*>(transfer->user_data)
                    ->self->usb_receive_complete_callback(transfer);
            },
            rx, 0);
        rx->transfer->flags = 0;

        {
            // Registered before submit: the completion callback can fire on
            // the event thread the instant submit returns, and it must find
            // itself in the list to unregister.
            const std::scoped_lock guard{rx_transfer_mutex_};
            rx_transfers_.push_back(rx);
        }

        const int ret = libusb_submit_transfer(rx->transfer);
        if (ret != 0) [[unlikely]]
            retire_receive_transfer(rx);
        return ret;
    }

    // Refill the pool to its full depth. A halted endpoint retires transfers
    // instead of re-submitting them, so without this the link would stay dark
    // even after the halt is cleared. Returns how many were restored.
    size_t replenish_receive_transfers() noexcept {
        if (receive_endpoint_ == 0 || link_faulted_.load(std::memory_order::relaxed))
            return 0;

        const size_t present = receive_transfer_count();
        if (present >= kReceiveTransferCount)
            return 0;

        size_t restored = 0;
        for (size_t missing = kReceiveTransferCount - present; missing > 0; --missing) {
            try {
                if (submit_receive_transfer(receive_endpoint_) != 0)
                    break;
            } catch (const std::exception& exception) {
                logger_.error("Failed to allocate a receive transfer: {}", exception.what());
                break;
            }
            ++restored;
        }
        return restored;
    }

    void usb_transmit_complete_callback(TransferWrapper* wrapper) {
        // Checked before the wrapper goes back to the pool, and never allowed to
        // stop it going back: a leaked wrapper would block every later sender in
        // acquire_transmit_buffer(). Reporting is all this can do -- but before
        // 2026-09-05 it did not even do that, so a stalled OUT endpoint dropped
        // every packet while the protocol layer counted them as transmitted.
        if (const libusb_transfer* transfer = wrapper->transfer_) [[likely]]
            note_transmit_completion(transfer);

        // Share mutex with teardown so destructor can block callbacks before draining the queue
        utility::PriorityInheritingMutex& mutex = transmit_transfer_mutex_;
        utility::PriorityInheritingConditionVariable& cv = transmit_transfer_cv_;
        auto& pool = free_transmit_transfers_;

        {
            const std::scoped_lock guard{mutex};

            if (stop_handling_events_.load(std::memory_order::relaxed)) [[unlikely]] {
                wrapper->destroy();
                delete wrapper;
                return;
            }

            pool.emplace_back(wrapper);
        }
        cv.notify_one();
    }

    void note_transmit_completion(const libusb_transfer* transfer) noexcept {
        if (transfer->status == LIBUSB_TRANSFER_COMPLETED) [[likely]] {
            // ADD_ZERO_PACKET means a completed transfer carried every byte; a
            // short one is a real partial write and the protocol field in it is
            // now truncated on the wire.
            if (transfer->actual_length == transfer->length) [[likely]]
                return;
            const uint64_t count =
                transfer_errors_.transmit_short.fetch_add(1, std::memory_order::relaxed) + 1;
            if (logging::should_log_occurrence(count))
                logger_.error(
                    "Transmit transfer sent {} of {} bytes (x{}); the packet was truncated",
                    transfer->actual_length, transfer->length, count);
            return;
        }

        if (transfer->status == LIBUSB_TRANSFER_CANCELLED) [[unlikely]]
            return; // teardown

        if (transfer->status == LIBUSB_TRANSFER_STALL) {
            const uint64_t count =
                transfer_errors_.transmit_stall.fetch_add(1, std::memory_order::relaxed) + 1;
            if (logging::should_log_occurrence(count))
                logger_.error(
                    "Transmit endpoint 0x{:02x} halted (STALL x{}); packets are being dropped "
                    "until the link is recovered",
                    kOutEndpoint, count);
            return;
        }

        const uint64_t count =
            transfer_errors_.transmit_error.fetch_add(1, std::memory_order::relaxed) + 1;
        if (logging::should_log_occurrence(count))
            logger_.error(
                "Transmit transfer failed with status {} (x{}); the packet was lost",
                transfer_status_name(transfer->status), count);
        if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE)
            fault("device disconnected while transmitting");
    }

    void usb_receive_complete_callback(libusb_transfer* transfer) {
        auto* rx = static_cast<RxTransfer*>(transfer->user_data);

        if (stop_handling_events_.load(std::memory_order::relaxed)) [[unlikely]] {
            retire_receive_transfer(rx);
            return;
        }

        // Until 2026-09-05 this field was never read. A stalled endpoint then
        // completed every transfer instantly with actual_length == 0, and the
        // re-submit below put it straight back -- a full-speed spin delivering
        // nothing, with no log line to say so.
        if (transfer->status != LIBUSB_TRANSFER_COMPLETED) [[unlikely]] {
            if (!handle_receive_failure(rx, transfer->status))
                return; // transfer retired; it must not be touched again
            // Deliberately NOT delivered even when actual_length > 0: a failed
            // transfer's payload is partial or garbage, and the deserializer
            // would fold it into the protocol stream as if it were real. Re-arm
            // and wait for a clean one; framing resynchronizes on a short packet.
        } else if (transfer->actual_length > 0) {
            const auto* first = reinterpret_cast<std::byte*>(transfer->buffer);
            const auto size = static_cast<std::size_t>(transfer->actual_length);
            if (rx_length_histogram_) [[unlikely]]
                rx_length_histogram_[std::min(size, core::protocol::kProtocolBufferSize)].fetch_add(
                    1, std::memory_order::relaxed);
            auto& callback = receive_callback_;
            if (callback_timing_) [[unlikely]] {
                const auto started = std::chrono::steady_clock::now();
                invoke_receive_callback(callback, {first, size});
                const auto elapsed =
                    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::steady_clock::now() - started)
                                              .count());
                auto& stats = bulk_callback_stats_;
                stats.count.fetch_add(1, std::memory_order::relaxed);
                stats.total_ns.fetch_add(elapsed, std::memory_order::relaxed);
                uint64_t previous_max = stats.max_ns.load(std::memory_order::relaxed);
                while (elapsed > previous_max
                       && !stats.max_ns.compare_exchange_weak(
                           previous_max, elapsed, std::memory_order::relaxed)) {}
            } else {
                invoke_receive_callback(callback, {first, size});
            }
        }

        const int ret = libusb_submit_transfer(transfer);
        if (ret != 0) [[unlikely]] {
            logger_.error(
                "Failed to re-submit receive transfer: {} ({})", ret, helper::libusb_errname(ret));
            retire_receive_transfer(rx);
            // One transfer lost is survivable -- try_recover_link() replenishes
            // the pool. Losing all of them is not: nothing would ever be
            // received again, and no later event could say so.
            if (ret == LIBUSB_ERROR_NO_DEVICE)
                fault("device disconnected while re-submitting a receive transfer");
            else if (receive_transfer_count() == 0)
                fault("every receive transfer was lost to submit failures");
        }
    }

    // The callback belongs to the application, and it is invoked from a C
    // callback libusb made into this thread: an exception escaping here unwinds
    // through libusb's own frames and ends in std::terminate. Catching costs
    // nothing when nothing throws, and turns "the process died decoding a
    // packet" into one log line and a dropped packet.
    void invoke_receive_callback(
        const std::function<void(std::span<const std::byte>)>& callback,
        std::span<const std::byte> data) noexcept {
        try {
            callback(data);
        } catch (const std::exception& exception) {
            const uint64_t count =
                receive_callback_exceptions_.fetch_add(1, std::memory_order::relaxed) + 1;
            if (logging::should_log_occurrence(count))
                logger_.error(
                    "Receive callback threw (x{}): {}; the packet was dropped", count,
                    exception.what());
        } catch (...) {
            const uint64_t count =
                receive_callback_exceptions_.fetch_add(1, std::memory_order::relaxed) + 1;
            if (logging::should_log_occurrence(count))
                logger_.error(
                    "Receive callback threw a non-std exception (x{}); the packet was dropped",
                    count);
        }
    }

    // Returns whether `rx` is still alive and may be re-submitted by the caller.
    //
    // STALL retires the transfer on purpose rather than re-submitting it: a
    // halted endpoint fails every submission instantly, so retrying here would
    // spin the event thread at full speed. The pool is rebuilt by
    // try_recover_link(), which the keepalive thread reaches within one refresh
    // interval and which can clear the halt first -- something this callback
    // cannot do, because a control transfer needs the event handling that this
    // very thread is running.
    bool handle_receive_failure(RxTransfer* rx, libusb_transfer_status status) noexcept {
        switch (status) {
        case LIBUSB_TRANSFER_CANCELLED:
            // Teardown or a deliberate recovery cancel; ownership ends here.
            retire_receive_transfer(rx);
            return false;

        case LIBUSB_TRANSFER_NO_DEVICE:
            retire_receive_transfer(rx);
            fault("device disconnected");
            return false;

        case LIBUSB_TRANSFER_STALL: {
            const uint64_t count =
                transfer_errors_.receive_stall.fetch_add(1, std::memory_order::relaxed) + 1;
            if (logging::should_log_occurrence(count))
                logger_.error(
                    "Receive endpoint 0x{:02x} halted (STALL x{}); retiring the transfer until "
                    "the link is recovered",
                    receive_endpoint_, count);
            retire_receive_transfer(rx);
            return false;
        }

        case LIBUSB_TRANSFER_ERROR:
        case LIBUSB_TRANSFER_TIMED_OUT:
        case LIBUSB_TRANSFER_OVERFLOW:
        case LIBUSB_TRANSFER_COMPLETED:
        default: {
            const uint64_t count =
                transfer_errors_.receive_error.fetch_add(1, std::memory_order::relaxed) + 1;
            if (logging::should_log_occurrence(count))
                logger_.warn(
                    "Receive transfer completed with status {} (x{}); re-submitting",
                    transfer_status_name(status), count);
            return true;
        }
        }
    }

    void retire_receive_transfer(RxTransfer* rx) noexcept {
        {
            // Unregister under the same lock the destructor cancels under, so
            // it can never hand libusb a transfer this callback already freed.
            const std::scoped_lock guard{rx_transfer_mutex_};
            std::erase(rx_transfers_, rx);
        }
        free_transfer_buffer(rx->buffer, rx->is_dev_mem);
        destroy_libusb_transfer(rx->transfer);
        delete rx;
    }

    size_t receive_transfer_count() noexcept {
        const std::scoped_lock guard{rx_transfer_mutex_};
        return rx_transfers_.size();
    }

    // Reported once, from whichever thread noticed first. Waking the transmit
    // waiters is the point: acquire_transmit_buffer() blocks on a completion
    // that is never coming.
    void fault(std::string_view reason) noexcept {
        if (link_faulted_.exchange(true, std::memory_order::acq_rel))
            return;
        logger_.error(
            "USB link faulted: {}. The transport now refuses traffic; the board must be "
            "re-opened.",
            reason);
        { const std::scoped_lock guard{transmit_transfer_mutex_}; }
        transmit_transfer_cv_.notify_all();
    }

    void destroy_free_transmit_transfers() noexcept {
        free_transmit_transfers_.pop_front_n([](TransferWrapper* wrapper) noexcept {
            wrapper->destroy();
            delete wrapper;
        });
    }

    // Percentiles over the raw counts, plus how the sizes land against the
    // endpoint's max packet size -- which is what decides whether a stream is
    // costing whole USB packets or riding along in one that was already going.
    void dump_rx_length_histogram() const noexcept {
        if (!rx_length_histogram_)
            return;
        uint64_t total = 0, bytes = 0;
        for (size_t size = 0; size <= core::protocol::kProtocolBufferSize; ++size) {
            const uint64_t count = rx_length_histogram_[size].load(std::memory_order::relaxed);
            total += count;
            bytes += count * size;
        }
        if (total == 0)
            return;
        const int mps =
            libusb_get_max_packet_size(libusb_get_device(libusb_device_handle_), kInEndpoint);
        fprintf(
            stderr, "[rx-histogram] %llu transfers, %llu bytes, mean %.1f B/transfer\n",
            static_cast<unsigned long long>(total), static_cast<unsigned long long>(bytes),
            static_cast<double>(bytes) / static_cast<double>(total));
        const double marks[] = {0.01, 0.50, 0.90, 0.99, 1.00};
        uint64_t seen = 0;
        size_t mark = 0;
        for (size_t size = 0; size <= core::protocol::kProtocolBufferSize && mark < 5; ++size) {
            seen += rx_length_histogram_[size].load(std::memory_order::relaxed);
            while (mark < 5
                   && static_cast<double>(seen) >= marks[mark] * static_cast<double>(total)) {
                fprintf(stderr, "[rx-histogram]   p%-3.0f %4zu B", marks[mark] * 100.0, size);
                if (mps > 0)
                    fprintf(
                        stderr, "  = %.2f x mps(%d), last packet %zu/%d B full",
                        static_cast<double>(size) / mps, mps, size % static_cast<size_t>(mps), mps);
                fprintf(stderr, "\n");
                mark++;
            }
        }
    }

    // How long the decode callbacks occupy the single event thread. The mean
    // bounds what a second libusb context could recover on the OTHER channel:
    // a CAN completion can only be delayed by a UART decode that is already
    // running, so if the bulk mean is small the split cannot pay for itself.
    void dump_callback_timing() const noexcept {
        if (!callback_timing_)
            return;
        const auto wall_ns =
            static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now() - opened_at_)
                                    .count());
        const auto report = [wall_ns](const char* name, const CallbackStats& stats) noexcept {
            const uint64_t count = stats.count.load(std::memory_order::relaxed);
            if (count == 0)
                return;
            const auto total = static_cast<double>(stats.total_ns.load(std::memory_order::relaxed));
            fprintf(
                stderr, "[cb-timing] %-8s n=%llu  mean %.2f us  max %.2f us  event thread %.2f%%\n",
                name, static_cast<unsigned long long>(count),
                total / static_cast<double>(count) / 1000.0,
                static_cast<double>(stats.max_ns.load(std::memory_order::relaxed)) / 1000.0,
                wall_ns > 0.0 ? total / wall_ns * 100.0 : 0.0);
        };
        report("bulk", bulk_callback_stats_);
    }

    // usbfs hands out DMA-coherent memory by mmap, so libusb_dev_mem_alloc()
    // costs a whole page however little is asked for. One call per 1023-byte
    // buffer therefore wasted three quarters of every mapping: measured 80
    // mappings holding 320 KiB for 80 KiB of payload on a single-interface
    // board, and double that once the CAN endpoint pair is claimed. Carving the
    // buffers out of a few large mappings keeps the zero-copy property and
    // spends a page on payload instead of on rounding.
    //
    // Measured on mc02, single interface: 80 mappings holding 320 KiB before,
    // 5 holding 80 KiB after -- the payload is 80 KiB either way.
    //
    // This is a memory-footprint change and NOT a throughput one. The mappings
    // are set up once at open and never touched again on the transfer path, so
    // there is nothing here for the packet rate to notice, and it did not:
    // 26899-27107 transfers/s before, 26937-27050 after, one run-to-run spread.
    // Do not expect this to buy latency or rate; it buys pinned kernel memory,
    // which is what usbfs_memory_mb caps across every process on the machine.
    unsigned char* alloc_transfer_buffer() {
        if (dev_mem_available_) {
            if (!dev_mem_slabs_.empty()) {
                auto& slab = dev_mem_slabs_.back();
                if (slab.size - slab.used >= kDevMemBufferStride) {
                    auto* buffer = slab.base + slab.used;
                    slab.used += kDevMemBufferStride;
                    dev_mem_buffers_outstanding_.fetch_add(1, std::memory_order::relaxed);
                    return buffer;
                }
            }
            if (auto* base = libusb_dev_mem_alloc(libusb_device_handle_, kDevMemSlabSize)) {
                dev_mem_slabs_.push_back(
                    {.base = base, .size = kDevMemSlabSize, .used = kDevMemBufferStride});
                dev_mem_buffers_outstanding_.fetch_add(1, std::memory_order::relaxed);
                return base;
            }
            // Whatever slabs already exist stay in use; only further buffers
            // come from the heap.
            dev_mem_available_ = false;
        }
        return new unsigned char[core::protocol::kProtocolBufferSize];
    }

    void free_transfer_buffer(unsigned char* buf, bool is_dev_mem) noexcept {
        // Slab-backed buffers are not individually unmappable: they are returned
        // wholesale by free_dev_mem_slabs() once nothing can reference them.
        // Counting them back in is what tells that function it is safe to run.
        if (is_dev_mem)
            dev_mem_buffers_outstanding_.fetch_sub(1, std::memory_order::relaxed);
        else
            delete[] buf;
    }

    // Return the slab mappings. Safe only once every buffer carved out of them
    // has been handed back: an outstanding URB holds a kernel DMA mapping into
    // its buffer, and unmapping that early would leave the controller writing
    // into memory this process no longer owns. If some transfer never reported
    // back -- the case the bounded drain in ~Usb() exists for -- the mappings
    // are deliberately leaked instead, which costs a process that is tearing
    // down nothing and is strictly safer than guessing.
    void free_dev_mem_slabs() noexcept {
        if (dev_mem_slabs_.empty())
            return;
        if (const auto outstanding = dev_mem_buffers_outstanding_.load(std::memory_order::relaxed);
            outstanding != 0) {
            logger_.warn(
                "{} DMA buffer(s) never returned; leaking {} dev_mem mapping(s) rather than "
                "unmapping memory the controller may still own",
                outstanding, dev_mem_slabs_.size());
            dev_mem_slabs_.clear();
            return;
        }
        for (const auto& slab : dev_mem_slabs_)
            libusb_dev_mem_free(libusb_device_handle_, slab.base, slab.size);
        dev_mem_slabs_.clear();
    }

    libusb_transfer* create_libusb_transfer() {
        auto* transfer = libusb_alloc_transfer(0);
        if (!transfer)
            throw std::bad_alloc{};
        active_transfers_.fetch_add(1, std::memory_order::relaxed);
        return transfer;
    }

    void destroy_libusb_transfer(libusb_transfer* transfer) noexcept {
        libusb_free_transfer(transfer);
        active_transfers_.fetch_sub(1, std::memory_order::relaxed);
    }

    static constexpr int kTargetInterface = 0x00;

    static constexpr unsigned char kOutEndpoint = 0x01;
    static constexpr unsigned char kInEndpoint = 0x81;

    // Second vendor interface, carrying CAN only. Presence is detected from the
    // configuration descriptor rather than assumed from a build flag, so one host
    // binary drives both a split-endpoint board and a single-pipe one.

    static constexpr size_t kTransmitTransferCount = 64;
    // Depth of the pre-submitted async RX transfer pool. At USB 3.0 SuperSpeed
    // (5 Gbit/s) a 4-deep pool cannot hide the submit/callback round-trip and
    // caps throughput well below the link rate; 16 keeps the bulk-IN pipe full.
    // Harmless for FS/HS boards (just a little more pinned buffer memory).
    static constexpr size_t kReceiveTransferCount = 16;

    logging::Logger& logger_;

    // Null unless libhcs_USB_RX_HISTOGRAM=1; indexed by completion length.
    std::unique_ptr<std::atomic<uint32_t>[]> rx_length_histogram_;

    // Set by libhcs_USB_CB_TIMING=1. Off, the receive path is unchanged.
    struct CallbackStats {
        std::atomic<uint64_t> count = 0;
        std::atomic<uint64_t> total_ns = 0;
        std::atomic<uint64_t> max_ns = 0;
    };
    bool callback_timing_ = false;
    std::chrono::steady_clock::time_point opened_at_ = std::chrono::steady_clock::now();
    CallbackStats bulk_callback_stats_;

    libusb_context* libusb_context_ = nullptr;
    libusb_device_handle* libusb_device_handle_ = nullptr;

    std::thread event_thread_;

    std::atomic<int> active_transfers_ = 0;
    std::atomic<bool> stop_handling_events_ = false;

    // Set once when the link is beyond local repair (the device is gone, or the
    // receive pool could not be kept alive). Everything after that point refuses
    // work instead of pretending to do it: acquire_transmit_buffer() hands back
    // nullptr, transmit() throws, and try_recover_link() stops trying. Replaces
    // an unconditional std::terminate() -- a bridge losing its board is the
    // application's decision to make, not the transport's.
    std::atomic<bool> link_faulted_ = false;

    // Completions that were not COMPLETED. Before these existed the status field
    // was never read at all: a stalled IN endpoint re-submitted forever at full
    // speed, and a failed OUT transfer went back to the pool as if it had been
    // sent, which lost the packet silently.
    struct TransferErrorCounters {
        std::atomic<uint64_t> receive_stall = 0;
        std::atomic<uint64_t> receive_error = 0;
        std::atomic<uint64_t> transmit_stall = 0;
        std::atomic<uint64_t> transmit_error = 0;
        std::atomic<uint64_t> transmit_short = 0;
        std::atomic<uint64_t> transmit_dropped = 0;
    };
    TransferErrorCounters transfer_errors_;

    // Which endpoint receive() armed, so recovery can rebuild the pool without
    // the caller passing it again.
    unsigned char receive_endpoint_ = 0;
    std::atomic<uint64_t> link_recovery_attempts_ = 0;
    std::atomic<uint64_t> event_loop_errors_ = 0;
    std::atomic<uint64_t> receive_callback_exceptions_ = 0;

    // Everything a re-open needs. ConnectionOptions is neither copyable nor
    // movable and the caller's instance does not outlive construction, so only
    // the one field the device scanner reads is kept.
    uint16_t vendor_id_ = 0;
    std::vector<uint16_t> product_ids_;
    std::string serial_;
    ConnectionOptions reconnect_options_;
    std::mutex reconnect_mutex_;
    std::atomic<uint64_t> reconnect_failures_ = 0;
    // Set once the device handle is closed: any transfer still counted then is
    // never going to complete, so the event loop must stop waiting for it.
    std::atomic<bool> abandon_remaining_transfers_ = false;
    static constexpr int kTeardownDrainRounds = 100; // 2 ms each

    // One mapping per this many bytes of buffer, rather than one per buffer.
    // Sized to hold a whole number of buffers so a slab wastes nothing: at the
    // 1024-byte stride below that is 16 per slab, and the pools ask for 80
    // buffers on a single-interface board (64 transmit + 16 receive) or 160 with
    // the CAN pair, both exact multiples. A larger slab would round back up --
    // 64 KiB mapped 128 KiB for the same 80 KiB of payload.
    static constexpr size_t kDevMemSlabSize = 16 * 1024;
    // Buffers are handed straight to the controller, so give each one its own
    // cache line instead of letting two share the line at a slab boundary.
    static constexpr size_t kDevMemBufferStride =
        (core::protocol::kProtocolBufferSize + 63U) & ~size_t{63U};

    struct DevMemSlab {
        unsigned char* base;
        size_t size;
        size_t used;
    };

    utility::RingBuffer<TransferWrapper*> free_transmit_transfers_;
    bool dev_mem_available_ = false;
    // Only ever touched from the thread that builds and tears down the pools;
    // the completion callbacks reach the counter below, not this vector.
    std::vector<DevMemSlab> dev_mem_slabs_;
    std::atomic<size_t> dev_mem_buffers_outstanding_ = 0;
    utility::PriorityInheritingMutex transmit_transfer_mutex_;
    utility::PriorityInheritingConditionVariable transmit_transfer_cv_;

    std::function<void(std::span<const std::byte>)> receive_callback_;
    std::function<void()> link_restart_callback_;

    // Priority channel. Deliberately a parallel set of members rather than a
    // generalized pool: the existing single-channel paths stay byte-for-byte as
    // they were, so a board without the second interface runs exactly the code
    // it ran before.

    // Every submitted receive transfer, so teardown can cancel them. Guarded
    // because the completion callback removes its own entry from the event
    // thread while the destructor walks the list from the main thread.
    std::mutex rx_transfer_mutex_;
    std::vector<RxTransfer*> rx_transfers_;
};

std::unique_ptr<Transport> create_transport(
    uint16_t usb_vid, std::span<const uint16_t> usb_pids, std::string_view serial_filter,
    const ConnectionOptions& options) {
    return std::make_unique<usb::Usb>(usb_vid, usb_pids, serial_filter, options);
}

} // namespace libhcs::host::transport::usb
