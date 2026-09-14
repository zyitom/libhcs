#include "libhcs/protocol/handler.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <format>
#include <memory>
#include <mutex>
#include <new>
#include <random>
#include <span>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <type_traits>
#include <utility>

#include <libhcs/protocol/vendor_control.hpp>

#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "host/src/logging/logging.hpp"
#include "host/src/protocol/stream_buffer.hpp"
#include "host/src/transport/transport.hpp"
#include "libhcs/board/common.hpp"
#include "libhcs/data/datas.hpp"
#include "libhcs/time/timeline.hpp"

namespace libhcs::host::protocol {

class Handler::Impl : public core::protocol::DeserializeCallback {
public:
    static constexpr auto kSessionAckTimeout = std::chrono::milliseconds{200};
    static constexpr size_t kSessionAckRetryCount = 5;
    // Also the anchor period for the shared time base. The board's session
    // lease is 4 s, so 1 s is ample for the keepalive itself.
    //
    // WHY IT WAS 50 ms. Each anchor exchange contributed one sample to the
    // Timeline's fit of (microframe -> host time), whose error falls as sqrt(N)
    // over a bounded window, so a shorter period bought accuracy directly --
    // measured on mc02, 250 ms -> 50 ms took the fitted phase from 18.5 us to
    // 8.4 us.
    //
    // WHY IT IS 1 s. Two things replaced that fit. MicroframeTimebase reads the
    // same axis from the controller's own counter at ~20 ns, and the boards'
    // absolute axes are self-sustaining once anchored (counter + local fit, the
    // anchor only resolves a wrap) -- so nothing on the precision path consumes
    // the arrival timing of these packets any more. What the round still buys:
    // the anchor wrap watchdog (a board counter that loses >1 s is caught within
    // one round), Timeline/MicroframeTimebase observations, and board health.
    // 1 Hz keeps all of that while the steady-state sync traffic drops to
    // ~60 B/s. The cost is one-time, on the fallback path only: with the source
    // unavailable, the round-trip fit converges on a 17 min window, and the
    // integer-offset lock needs 32 observations = 32 s.
    // 2026-09-13: 1 Hz 轮次在 hpm 板上诱发 keepalive 丢 ack(与轮次周期强耦合,
    // mc02 正常;租约 4 s 后仍复现,疑似下行 arm/节流或链路恢复后的端点状态,
    // 待定位)。回落到实测稳定的 250 ms。
    static constexpr auto kSessionRefreshInterval = std::chrono::seconds{1};

    Impl(
        std::unique_ptr<transport::Transport> transport, data::DataCallback& callback,
        bool enable_time_sync)
        : callback_(callback)
        , deserializer_(*this)
        , expected_session_nonce_(generate_session_nonce())
        , time_sync_enabled_(enable_time_sync)
        , expected_session_start_ack_(make_session_start_ack(expected_session_nonce_))
        , transport_(std::move(transport)) {
        // USB bulk completions are arbitrary slices of one reliable byte
        // stream, not protocol-field boundaries.
        transport_->receive([this](std::span<const std::byte> buffer) { receive_stream(buffer); });
        transport_->on_link_restart([this] {
            // A new ARQ generation cannot continue a partially received
            // protocol field. This callback runs on the transport's recovery
            // thread -- NOT the receive thread, despite what an earlier
            // comment claimed here -- and the deserializer may only be
            // touched from the receive thread. So all it does is mark the
            // stream poisoned: the next receive_stream() performs the reset
            // before feeding any byte of the new connection, serialized with
            // feed() by construction rather than by a drain timeout.
            link_restart_pending_.store(true, std::memory_order_release);
        });

        // NOT started here: a transport may have an out-of-band handshake that
        // has to complete before the first kStart, and the caller can only run
        // it once this object exists. See Handler's constructor.
    }

    [[nodiscard]] Handler::LinkState link_state() const noexcept {
        if (transport_->link_faulted())
            return Handler::LinkState::kFaulted;
        return session_established() ? Handler::LinkState::kUp : Handler::LinkState::kSessionDown;
    }

    // Opens the session and starts the keepalive. Separate from construction so
    // a caller can do transport-level configuration in between; throws if the
    // board never acknowledges.
    void start() {
        establish_session();
        // Created from the caller's thread, so it inherits that thread's
        // placement -- possibly a core the caller is about to busy-wait on. It
        // parks on an untimed wait until the transport has moved it: a timed
        // wait armed on the inherited core would expire through that core's
        // timer thread, which the busy-wait starves.
        // See Transport::configure_session_thread().
        auto configured = std::make_shared<std::atomic<bool>>(false);
        keepalive_thread_ = std::thread{[this, configured] {
            configured->wait(false, std::memory_order_acquire);
            keepalive_loop();
        }};
        transport_->configure_session_thread(keepalive_thread_);
        configured->store(true, std::memory_order_release);
        configured->notify_one();
    }

    // Re-run the caller's out-of-band handshake before re-opening a session.
    // The board forgets the handshake when a session ends, so a reconnect that
    // skipped this would be refused exactly like a host that never handshook.
    void set_before_session(std::function<void()> hook) { before_session_ = std::move(hook); }

    void run_before_session() {
        if (before_session_)
            before_session_();
    }

    ~Impl() override {
        stop_keepalive_.store(true, std::memory_order_relaxed);
        session_cv_.notify_all();
        if (keepalive_thread_.joinable())
            keepalive_thread_.join();

        transport_.reset();
    }

    // The constructor hands three `this`-capturing lambdas to transport_, and the
    // destructor joins a thread and releases that transport. Copying or moving an
    // Impl would leave those registered callbacks pointing at the old object and
    // duplicate ownership of both the thread and the transport. Nothing does so
    // today -- Handler holds Impl by raw pointer and moves the pointer, not the
    // object -- so deleting these turns a latent footgun into a compile error.
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    PacketBuilder start_transmit() { return PacketBuilder{transport_.get()}; }

    // Every application callback below runs from inside the deserializer's
    // coroutine, and that coroutine's promise treats an escaping exception as a
    // fatal assert (core/src/coroutine/lifo.hpp: unhandled_exception() ->
    // assert_failed_debug()). So an application whose CAN handler threw used to
    // die with a message about coroutines rather than about its own callback --
    // measured 2026-09-06, and the transport-level guard could not see it
    // because the promise intercepts the exception first.
    //
    // Contain it here, which is the actual library/application boundary. A
    // throwing callback costs the record it was handed, not the bridge. The bool
    // callbacks report "handled" on a throw, so the caller does not also log an
    // "unexpected field id" that never happened.
    template <typename Callable>
    static std::invoke_result_t<Callable> guard_callback(Callable&& callable) noexcept {
        using Result = std::invoke_result_t<Callable>;
        try {
            return std::forward<Callable>(callable)();
        } catch (const std::exception& exception) {
            note_application_callback_exception(exception.what());
        } catch (...) {
            note_application_callback_exception("a non-std exception");
        }
        if constexpr (std::is_same_v<Result, bool>)
            return true;
    }

    static void note_application_callback_exception(std::string_view what) noexcept {
        static std::atomic<uint64_t> occurrences{0};
        const uint64_t count = occurrences.fetch_add(1, std::memory_order::relaxed) + 1;
        if (logging::should_log_occurrence(count))
            logging::get_logger().error(
                "Application callback threw (x{}): {}; the record was dropped", count, what);
    }

    bool can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        if (!session_established())
            return true;
        if (!guard_callback([&] { return callback_.can_receive_callback(id, data); })) {
            logging::get_logger().error("Unexpected can field id: ", static_cast<int>(id));
            return false;
        }
        return true;
    }

    bool uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        if (!session_established())
            return true;
        if (!guard_callback([&] { return callback_.uart_receive_callback(id, data); })) {
            logging::get_logger().error("Unexpected uart field id: ", static_cast<int>(id));
            return false;
        }
        return true;
    }

    // UART config is a downlink-only channel: the host emits it, boards never
    // send it back. Receiving one on the uplink means the peer is confused, so
    // report it as a routing error rather than silently ignoring it.
    bool uart_config_deserialized_callback(
        core::protocol::FieldId id, const data::UartConfigView& data) override {
        (void)data;
        if (!session_established())
            return true;
        logging::get_logger().error(
            "Unexpected uart config field on uplink: ", static_cast<int>(id));
        return false;
    }

    bool gpio_digital_data_deserialized_callback(
        uint8_t channel_index, const data::GpioDigitalDataView& data) override {
        if (!session_established())
            return true;
        if (!guard_callback(
                [&] { return callback_.gpio_digital_read_result_callback(channel_index, data); })) {
            logging::get_logger().error(
                "Unexpected gpio channel index: ", static_cast<int>(channel_index));
            return false;
        }
        return true;
    }

    bool gpio_analog_data_deserialized_callback(
        uint8_t channel_index, const data::GpioAnalogDataView& data) override {
        if (!session_established())
            return true;
        if (!guard_callback(
                [&] { return callback_.gpio_analog_read_result_callback(channel_index, data); })) {
            logging::get_logger().error(
                "Unexpected gpio channel index: ", static_cast<int>(channel_index));
            return false;
        }
        return true;
    }

    bool gpio_digital_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (!session_established())
            return true;
        (void)channel_index;
        (void)data;
        logging::get_logger().error("Unexpected gpio digital read config field in uplink");
        return false;
    }

    bool gpio_analog_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (!session_established())
            return true;
        (void)channel_index;
        (void)data;
        logging::get_logger().error("Unexpected gpio analog read config field in uplink");
        return false;
    }

    void accelerometer_deserialized_callback(const data::ImuAccelerometerDataView& data) override {
        if (!session_established())
            return;
        guard_callback([&] { callback_.accelerometer_receive_callback(data); });
    }

    void gyroscope_deserialized_callback(const data::ImuGyroscopeDataView& data) override {
        if (!session_established())
            return;
        guard_callback([&] { callback_.gyroscope_receive_callback(data); });
    }

    void temperature_deserialized_callback(const data::ImuTemperatureDataView& data) override {
        if (!session_established())
            return;
        guard_callback([&] { callback_.temperature_receive_callback(data); });
    }

    void session_control_deserialized_callback(const data::SessionControlView& data) override {
        if (data.nonce != expected_session_nonce_)
            return;

        bool notify = false;
        {
            const std::scoped_lock guard{session_mutex_};
            switch (data.type) {
            case data::SessionType::kStartAck:
                session_established_.store(true, std::memory_order_release);
                ++session_start_ack_count_;
                notify = true;
                break;
            case data::SessionType::kKeepaliveAck:
                ++session_keepalive_ack_count_;
                notify = true;
                break;
            default: break;
            }
        }
        if (notify)
            session_cv_.notify_all();
    }

    void time_status_deserialized_callback(const data::TimeStatusView& data) override {
        if (data.nonce != expected_session_nonce_)
            return;

        // Midpoint of the round trip, not the arrival instant: the board sampled
        // its counter somewhere between our send and this arrival, and splitting
        // the difference cancels the bulk of the USB transit bias. What is left
        // is the down/up asymmetry, which is the floor on how well this host can
        // place a microframe on its own clock -- and it is irrelevant to
        // cross-board agreement, which never goes through this clock at all.
        const auto now = time::Timeline::Clock::now();
        const auto sent = time_anchor_sent_at_.load(std::memory_order_acquire);
        const auto sampled_at =
            sent.time_since_epoch().count() == 0 ? now : sent + (now - sent) / 2;

        // Only an anchored board is on the shared axis; before that it reports
        // its own boot-relative origin, which would poison the fit.
        if (data.state == data::TimeState::kValid)
            time::timeline().observe(data.microframe, sampled_at);

        guard_callback([&] { callback_.time_status_callback(data); });
    }

    void pulse_report_deserialized_callback(const data::PulseReportView& data) override {
        if (data.nonce != expected_session_nonce_)
            return;
        guard_callback([&] { callback_.pulse_report_callback(data); });
    }

    void error_callback() override {
        logging::get_logger().error("Deserializer encountered an error while parsing input");
    }

private:
    // kSession (14) fits the short field header, whose id nibble shares the
    // first byte with SessionHeader, so the whole field is one SessionHeader.
    // make_session_start_ack() checks this against the serializer itself.
    static constexpr size_t kSessionStartAckSize = sizeof(core::protocol::SessionHeader);

    // The exact bytes the board sends for this Handler's kStart. They come from
    // the same Serializer the board runs rather than being assembled here: until
    // 2026-09-11 this hand-built an extended field header, 6 bytes that the
    // board's 5-byte ack never matched, and every session start timed out.
    static std::array<std::byte, kSessionStartAckSize> make_session_start_ack(uint32_t nonce) {
        class FixedBuffer final : public core::protocol::SerializeBuffer {
        public:
            explicit FixedBuffer(std::span<std::byte> storage) noexcept
                : storage_(storage) {}

            std::span<std::byte> allocate(std::size_t size) noexcept override {
                if (size > storage_.size() - used_)
                    return {};
                const std::span<std::byte> region = storage_.subspan(used_, size);
                used_ += size;
                return region;
            }

            [[nodiscard]] std::size_t used() const noexcept { return used_; }

        private:
            std::span<std::byte> storage_;
            std::size_t used_ = 0;
        };

        std::array<std::byte, kSessionStartAckSize> ack{};
        FixedBuffer buffer{ack};
        const auto result = core::protocol::Serializer{buffer}.write_session_control(
            {.type = data::SessionType::kStartAck, .nonce = nonce});
        core::utility::assert_always(
            result == core::protocol::Serializer::SerializeResult::kSuccess
            && buffer.used() == ack.size());
        return ack;
    }

    // Idempotent: a restart that lands while an earlier one is being consumed
    // simply runs this a second time.
    void reset_link_state() {
        deserializer_.finish_transfer();
        awaiting_session_start_ack_ = true;
        session_start_ack_window_size_ = 0;
        {
            // Pair the state change with the condition-variable mutex so the
            // keepalive thread cannot miss the restart notification.
            const std::scoped_lock guard{session_mutex_};
            session_established_.store(false, std::memory_order_release);
        }
        session_cv_.notify_all();
    }

    // A reopened transport may still complete queued bytes from the previous
    // session. Only this Handler's nonce identifies a safe field boundary.
    void receive_stream(std::span<const std::byte> buffer) {
        // Every call of this function runs on the transport's receive thread,
        // the one thread the deserializer may be touched from -- which is what
        // makes consuming the deferred link restart here safe without any
        // lock on the data path.
        if (link_restart_pending_.load(std::memory_order_acquire)) [[unlikely]] {
            // exchange, not load-then-clear: a re-open that lands between the
            // load above and the clear must not be lost to it.
            while (link_restart_pending_.exchange(false, std::memory_order_acq_rel))
                reset_link_state();
        }

        if (!awaiting_session_start_ack_) {
            deserializer_.feed(buffer);
            return;
        }

        for (size_t i = 0; i < buffer.size(); ++i) {
            if (session_start_ack_window_size_ < session_start_ack_window_.size()) {
                session_start_ack_window_[session_start_ack_window_size_++] = buffer[i];
            } else {
                std::ranges::move(
                    session_start_ack_window_.begin() + 1, session_start_ack_window_.end(),
                    session_start_ack_window_.begin());
                session_start_ack_window_.back() = buffer[i];
            }

            if (session_start_ack_window_size_ != session_start_ack_window_.size()
                || session_start_ack_window_ != expected_session_start_ack_) {
                continue;
            }

            awaiting_session_start_ack_ = false;
            deserializer_.feed(session_start_ack_window_);
            deserializer_.feed(buffer.subspan(i + 1));
            return;
        }
    }

    [[nodiscard]] bool session_established() const {
        return session_established_.load(std::memory_order_acquire);
    }

    void establish_session() {
        for (size_t attempt = 0; attempt < kSessionAckRetryCount; ++attempt) {
            uint64_t previous_session_start_ack_count = 0;
            {
                const std::scoped_lock guard{session_mutex_};
                previous_session_start_ack_count = session_start_ack_count_;
            }

            send_session_start();

            std::unique_lock lock{session_mutex_};
            if (session_cv_.wait_for(
                    lock, kSessionAckTimeout, [this, previous_session_start_ack_count] {
                        return stop_keepalive_.load(std::memory_order_relaxed)
                            || session_start_ack_count_ > previous_session_start_ack_count;
                    })) {
                if (stop_keepalive_.load(std::memory_order_relaxed))
                    return;
                return;
            }
        }

        throw std::runtime_error{"Timed out waiting for SESSION_ACK"};
    }

    void send_session_start() { send_session_control(data::SessionType::kStart, "Session Start"); }

    void refresh_session() {
        for (size_t attempt = 0; attempt < kSessionAckRetryCount; ++attempt) {
            if (!session_established())
                return;

            uint64_t previous_session_keepalive_ack_count = 0;
            {
                const std::scoped_lock guard{session_mutex_};
                previous_session_keepalive_ack_count = session_keepalive_ack_count_;
            }

            send_session_keepalive();

            std::unique_lock lock{session_mutex_};
            if (session_cv_.wait_for(
                    lock, kSessionAckTimeout, [this, previous_session_keepalive_ack_count] {
                        return stop_keepalive_.load(std::memory_order_relaxed)
                            || !session_established()
                            || session_keepalive_ack_count_ > previous_session_keepalive_ack_count;
                    })) {
                return;
            }
        }

        throw std::runtime_error{"Timed out waiting for SESSION_KEEPALIVE_ACK"};
    }

    void send_session_keepalive() {
        send_session_control(data::SessionType::kKeepalive, "Session Keepalive");
    }

    void send_session_control(data::SessionType type, std::string_view operation_name) {
        core::protocol::Serializer::SerializeResult result;
        {
            StreamBuffer buffer{*transport_};
            core::protocol::Serializer serializer{buffer};
            result =
                serializer.write_session_control({.type = type, .nonce = expected_session_nonce_});
        }

        core::utility::assert_debug(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
        if (result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]]
            throw std::runtime_error(
                std::string{"Failed to transmit "} + std::string{operation_name}
                + ": Transmit buffer unavailable (acquire failed)");
    }

public:
    // Asks this board to fire a hardware pulse at an absolute microframe. The
    // caller must send the SAME value to every board -- that is what makes the
    // two-way difference cancel the path delay.
    void send_pulse_schedule(uint64_t microframe) {
        StreamBuffer buffer{*transport_};
        core::protocol::Serializer serializer{buffer};
        (void)serializer.write_pulse_schedule(
            {.nonce = expected_session_nonce_, .microframe = microframe});
    }

    // EP0 configuration channel. `payload` is written for an OUT request and
    // filled for an IN one; see libhcs/protocol/vendor_control.hpp.
    //
    // Bypasses the session entirely -- no nonce, no keepalive, no serializer.
    // Configuration is applied while the board object is being constructed,
    // before the keepalive thread has opened a session, and it must keep
    // answering after one has lapsed.
    bool vendor_control(
        uint8_t request_type, uint8_t request, uint16_t index, std::span<std::byte> payload) {
        switch (transport_->vendor_control(request_type, request, index, payload)) {
        case transport::Transport::ControlResult::kOk: return true;
        case transport::Transport::ControlResult::kStalled: return false;
        case transport::Transport::ControlResult::kUnsupported:
            throw std::runtime_error(
                "This transport has no control endpoint: channel configuration over EP0 is "
                "available on USB only. An EtherCAT-connected board is configured through the "
                "in-band config fields instead.");
        case transport::Transport::ControlResult::kFailed:
        default:
            throw std::runtime_error(
                std::string{"EP0 vendor request 0x"} + std::format("{:02x}", request)
                + " failed; see the transport log for the libusb error.");
        }
    }

private:
    void send_time_anchor() {
        // One process-wide axis, queried per round: see libhcs/time/timeline.hpp
        // for why a per-board origin would leave the boards mutually offset by
        // whole seconds while each looked perfectly healthy on its own.
        const auto now = time::Timeline::Clock::now();
        const uint64_t microframe = time::timeline().anchor_for(now);

        core::protocol::Serializer::SerializeResult result;
        {
            StreamBuffer buffer{*transport_};
            core::protocol::Serializer serializer{buffer};
            result = serializer.write_time_anchor(
                {.nonce = expected_session_nonce_, .microframe = microframe});
        }
        core::utility::assert_debug(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
        // Deliberately not fatal, unlike the keepalive: a dropped anchor costs
        // one period of timeline convergence, never the session.
        if (result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]] {
            logging::get_logger().error("Failed to transmit Time Anchor: transmit buffer full");
            return;
        }
        // Stored only after the anchor actually went out. A kTimeStatus that
        // answers round N can still arrive after round N+1 failed; pairing it
        // with a failed round's timestamp would misplace it by a whole period.
        time_anchor_sent_at_.store(now, std::memory_order_release);
    }

    void keepalive_loop() {
        while (!stop_keepalive_.load(std::memory_order_relaxed)) {
            {
                std::unique_lock lock{session_mutex_};
                (void)session_cv_.wait_for(lock, kSessionRefreshInterval, [this] {
                    return stop_keepalive_.load(std::memory_order_relaxed)
                        || !session_established();
                });
            }
            if (stop_keepalive_.load(std::memory_order_relaxed))
                break;

            try {
                if (session_established()) {
                    refresh_session();
                } else {
                    run_before_session();
                    establish_session();
                }

                // After the keepalive, so an anchor is only ever sent on a
                // session the board has just confirmed is alive.
                if (time_sync_enabled_ && session_established())
                    send_time_anchor();

                consecutive_session_failures_ = 0;
            } catch (const std::exception& exception) {
                handle_session_failure(exception);
            }
        }
    }

    // Until 2026-09-05 this was std::terminate(). Losing a board is the
    // application's decision, not this thread's -- and the failure it fires on
    // is often repairable: the board answers EP0 while its bulk endpoints are
    // halted or un-armed, which is the one thing a transport-level recovery can
    // fix and no amount of protocol retrying can.
    void handle_session_failure(const std::exception& exception) noexcept try {
        // refresh_session() gives up without clearing the flag, so clear it
        // here: the next pass must re-open the session rather than keep
        // keepaliving one the board has already forgotten.
        {
            const std::scoped_lock guard{session_mutex_};
            session_established_.store(false, std::memory_order::release);
        }

        ++consecutive_session_failures_;

        const bool recovered = transport_->try_recover_link();

        // The 1st, 2nd, 4th, 8th ... failure. A board that is simply unplugged
        // fails every attempt forever, and that must not bury the log.
        if (logging::should_log_occurrence(consecutive_session_failures_))
            logging::get_logger().error(
                "Failed to refresh session ({} in a row): {}. Link recovery {}; retrying.",
                consecutive_session_failures_, exception.what(),
                recovered ? "attempted" : "not available");

        // Pace the retries. The loop's own wait returns immediately while the
        // session is down, and a faulted transport fails instantly, so without
        // this a dead board would spin this thread at full speed.
        std::unique_lock lock{session_mutex_};
        (void)session_cv_.wait_for(lock, kSessionRefreshInterval, [this] {
            return stop_keepalive_.load(std::memory_order_relaxed);
        });
    } catch (const std::exception& nested) {
        // A function-try-block, because this one runs on the catch path of a
        // thread whose escaping exception used to be the std::terminate() this
        // whole change removes. Nothing here may throw its way out.
        logging::get_logger().error("While handling a session failure: {}", nested.what());
    }

    static uint32_t generate_session_nonce() {
        std::random_device random_device;
        std::uniform_int_distribution<uint32_t> distribution;
        return distribution(random_device);
    }

    data::DataCallback& callback_;
    core::protocol::Deserializer deserializer_;

    mutable std::mutex session_mutex_;
    std::condition_variable session_cv_;
    std::atomic<bool> session_established_{false};
    // Set by the transport's recovery thread, consumed by receive_stream();
    // the deserializer itself is only ever touched on the receive thread.
    std::atomic<bool> link_restart_pending_{false};
    uint64_t session_start_ack_count_ = 0;
    uint64_t session_keepalive_ack_count_ = 0;
    uint32_t expected_session_nonce_ = 0;
    bool time_sync_enabled_ = false;
    std::atomic<time::Timeline::Clock::time_point> time_anchor_sent_at_;
    std::array<std::byte, kSessionStartAckSize> expected_session_start_ack_{};
    std::array<std::byte, kSessionStartAckSize> session_start_ack_window_{};
    size_t session_start_ack_window_size_ = 0;
    bool awaiting_session_start_ack_ = true;

    std::function<void()> before_session_;
    std::unique_ptr<transport::Transport> transport_;

    // Keepalive thread only; reset by the first pass that gets through cleanly.
    uint64_t consecutive_session_failures_ = 0;

    std::atomic<bool> stop_keepalive_{false};
    std::thread keepalive_thread_;
};

namespace {

struct PacketBuilderImpl {
    explicit PacketBuilderImpl(transport::Transport& transport) noexcept
        : buffer_(transport)
        , serializer_(buffer_) {}

    PacketBuilderImpl(PacketBuilderImpl&& other) noexcept
        : buffer_(std::move(other.buffer_))
        , serializer_(buffer_) {}

    PacketBuilderImpl& operator=(PacketBuilderImpl&&) = delete;
    PacketBuilderImpl(const PacketBuilderImpl&) = delete;
    PacketBuilderImpl& operator=(const PacketBuilderImpl&) = delete;
    ~PacketBuilderImpl() = default;

    // `write_*` returns `true` if args are valid; it never reports transport/resource issues.
    // - `kInvalidArgument` => `false` (user error)
    // - `kBadAlloc` => logged and ignored (`true`) (internal/transient)
    [[nodiscard]] bool write_can(data::DataId field_id, const data::CanDataView& view) noexcept {
        return process_result(serializer_.write_can(field_id, view));
    }

    [[nodiscard]] bool write_uart(data::DataId field_id, const data::UartDataView& view) noexcept {
        return process_result(serializer_.write_uart(field_id, view));
    }

    [[nodiscard]] bool
        write_uart_config(data::DataId field_id, const data::UartConfigView& view) noexcept {
        return process_result(serializer_.write_uart_config(field_id, view));
    }

    [[nodiscard]] bool write_gpio_digital_data(
        uint8_t channel_index, const data::GpioDigitalDataView& view) noexcept {
        if (view.timestamp_quarter_us.has_value()) [[unlikely]]
            return false;
        return process_result(serializer_.write_gpio_digital_value(channel_index, view));
    }

    [[nodiscard]] bool write_gpio_digital_read_config(
        uint8_t channel_index, const data::GpioReadConfigView& view) noexcept {
        return process_result(serializer_.write_gpio_digital_read_config(channel_index, view));
    }

    [[nodiscard]] bool write_gpio_analog_data(
        uint8_t channel_index, const data::GpioAnalogDataView& view) noexcept {
        return process_result(serializer_.write_gpio_analog_value(channel_index, view));
    }

    [[nodiscard]] bool
        write_imu_accelerometer(const data::ImuAccelerometerDataView& view) noexcept {
        return process_result(serializer_.write_imu_accelerometer(view));
    }

    [[nodiscard]] bool write_imu_gyroscope(const data::ImuGyroscopeDataView& view) noexcept {
        return process_result(serializer_.write_imu_gyroscope(view));
    }

private:
    static bool process_result(core::protocol::Serializer::SerializeResult result) {
        using core::protocol::Serializer;
        if (result == Serializer::SerializeResult::kSuccess) [[likely]]
            return true;
        if (result == Serializer::SerializeResult::kBadAlloc) {
            // Reachable in steady state since 2026-09-05: a faulted transport
            // hands out no buffers at all, so a caller in a tight loop hits this
            // on every attempt. Unthrottled it produced 473 MB of identical
            // lines in twenty seconds.
            static std::atomic<uint64_t> occurrences{0};
            if (const uint64_t count = occurrences.fetch_add(1, std::memory_order::relaxed) + 1;
                logging::should_log_occurrence(count))
                logging::get_logger().error(
                    "Transmit buffer unavailable (acquire failed) x{}", count);
            return true;
        }
        if (result == Serializer::SerializeResult::kInvalidArgument) {
            return false;
        }
        core::utility::assert_failed_debug();
    }

    StreamBuffer buffer_;
    core::protocol::Serializer serializer_;
};

} // namespace

Handler::PacketBuilder::PacketBuilder(void* transport_ptr) noexcept {
    static_assert(sizeof(PacketBuilderImpl) <= sizeof(storage_));
    static_assert(alignof(PacketBuilderImpl) <= alignof(std::uintptr_t));

    auto& transport_ref = *static_cast<transport::Transport*>(transport_ptr);
    std::construct_at(reinterpret_cast<PacketBuilderImpl*>(storage_), transport_ref);
}

Handler::PacketBuilder::~PacketBuilder() noexcept {
    std::destroy_at(std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_)));
}

bool Handler::PacketBuilder::write_can(
    data::DataId field_id, const data::CanDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))->write_can(field_id, view);
}

bool Handler::PacketBuilder::write_uart(
    data::DataId field_id, const data::UartDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))->write_uart(field_id, view);
}

bool Handler::PacketBuilder::write_uart_config(
    data::DataId field_id, const data::UartConfigView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_uart_config(field_id, view);
}

bool Handler::PacketBuilder::write_gpio_digital_data(
    uint8_t channel_index, const data::GpioDigitalDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_gpio_digital_data(channel_index, view);
}

bool Handler::PacketBuilder::write_gpio_digital_read_config(
    uint8_t channel_index, const data::GpioReadConfigView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_gpio_digital_read_config(channel_index, view);
}

bool Handler::PacketBuilder::write_gpio_analog_data(
    uint8_t channel_index, const data::GpioAnalogDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_gpio_analog_data(channel_index, view);
}

Handler::Handler(
    uint16_t usb_vid, std::span<const uint16_t> usb_pids, std::string_view serial_filter,
    const board::AdvancedOptions& options, data::DataCallback& callback,
    const BeforeSession& before_session)
    : impl_(new Impl(
          transport::usb::create_transport(usb_vid, usb_pids, serial_filter, options), callback,
          options.enable_time_sync)) {
    // The hook runs with the transport up but no session yet -- the only window
    // in which an out-of-band handshake can precede the first kStart.
    //
    // Both of these can throw, and a throw from a constructor BODY does not run
    // this object's destructor -- impl_ would leak, and with it the claimed
    // libusb interface, so the next open of the same board fails with
    // ERROR_BUSY. That does not happen when the work sits inside Impl's own
    // constructor, which is where it used to be.
    try {
        if (before_session) {
            // Registered as well as run, so the keepalive thread can repeat it
            // on a reconnect -- the board forgets the handshake when a session
            // ends.
            // By value: `before_session` is a constructor parameter and would
            // dangle the moment construction returns, while this hook has to
            // survive for every later reconnect.
            impl_->set_before_session([this, hook = before_session] { hook(*this); });
            before_session(*this);
        }
        impl_->start();
    } catch (...) {
        delete impl_;
        impl_ = nullptr;
        throw;
    }
}

Handler::Handler(Handler&& other) noexcept
    : impl_(std::exchange(other.impl_, nullptr)) {}

Handler& Handler::operator=(Handler&& other) noexcept {
    if (this == &other)
        return *this;
    delete impl_;
    impl_ = std::exchange(other.impl_, nullptr);
    return *this;
}

Handler::~Handler() noexcept { delete impl_; }

Handler::PacketBuilder Handler::start_transmit() noexcept {
    core::utility::assert_debug(impl_);
    return impl_->start_transmit();
}

Handler::LinkState Handler::link_state() const noexcept {
    core::utility::assert_debug(impl_);
    return impl_->link_state();
}

void Handler::send_pulse_schedule(uint64_t microframe) noexcept {
    core::utility::assert_debug(impl_);
    impl_->send_pulse_schedule(microframe);
}

bool Handler::vendor_control_out(
    uint8_t request, uint16_t index, const void* payload, size_t size) {
    core::utility::assert_debug(impl_);
    // libusb writes from the caller's buffer but the transport signature is one
    // mutable span for both directions, so copy through a scratch buffer rather
    // than casting away const on something the caller owns.
    std::array<std::byte, kVendorControlPayloadMax> scratch{};
    if (size > scratch.size())
        throw std::invalid_argument{"EP0 payload too large"};
    // memcpy(p, nullptr, 0) is pedantically UB; a size-0 EP0 request is legal.
    if (size != 0)
        std::memcpy(scratch.data(), payload, size);
    return impl_->vendor_control(
        core::protocol::vendor_control::kRequestTypeOut, request, index,
        std::span<std::byte>{scratch.data(), size});
}

bool Handler::vendor_control_in(uint8_t request, uint16_t index, void* payload, size_t size) {
    core::utility::assert_debug(impl_);
    std::array<std::byte, kVendorControlPayloadMax> scratch{};
    if (size > scratch.size())
        throw std::invalid_argument{"EP0 payload too large"};
    const bool ok = impl_->vendor_control(
        core::protocol::vendor_control::kRequestTypeIn, request, index,
        std::span<std::byte>{scratch.data(), size});
    if (ok && size != 0)
        std::memcpy(payload, scratch.data(), size);
    return ok;
}

} // namespace libhcs::host::protocol
