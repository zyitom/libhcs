#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>
#include <variant>

#include <libhcs/board/common.hpp>
#include <libhcs/data/callback.hpp>
#include <libhcs/export.hpp>
#include <libhcs/protocol/vendor_control.hpp>
#include <libhcs/time/sample_time.hpp>

namespace libhcs::host::protocol {

class libhcs_API Handler {
public:
    class libhcs_API PacketBuilder {
    public:
        PacketBuilder(const PacketBuilder&) = delete;
        PacketBuilder& operator=(const PacketBuilder&) = delete;
        PacketBuilder(PacketBuilder&&) = delete;
        PacketBuilder& operator=(PacketBuilder&&) = delete;

        ~PacketBuilder() noexcept;

        bool write_can(data::DataId field_id, const data::CanDataView& view) noexcept;

        bool write_uart(data::DataId field_id, const data::UartDataView& view) noexcept;

        // GPIO, addressed by the line of the board's GPIO port (data::DataId::kGpio).
        bool write_gpio_digital_data(
            std::uint8_t line, const data::GpioDigitalDataView& view) noexcept;

        bool write_gpio_analog_data(
            std::uint8_t line, const data::GpioAnalogDataView& view) noexcept;

        // Asks an input line for one sample now.
        bool write_gpio_read(std::uint8_t line) noexcept;

        // The tone the buzzer plays from now on.
        bool write_buzzer_tone(const data::BuzzerToneDataView& view) noexcept;

    private:
        friend class Handler;

        explicit PacketBuilder(void* transport) noexcept;

        // Two StreamBuffer + Serializer pairs since CAN can be routed to its own
        // transport channel. A static_assert in handler.cpp pins the real size.
        alignas(std::uintptr_t) std::uint8_t storage_[16 * sizeof(std::uintptr_t)];
    };

    /**
     * @brief Runs once with the transport up but no session open yet.
     *
     * This is the only window in which a transport-level handshake can precede
     * the first kStart. The hpm_board classes use it to apply and verify their
     * EP0 channel configuration, so that a session never opens against a board
     * whose configuration the host has not confirmed.
     *
     * Throwing from here aborts construction, exactly as a failed session would.
     */
    using BeforeSession = std::function<void(Handler&)>;

    Handler(
        uint16_t usb_vid, std::span<const uint16_t> usb_pids, std::string_view serial_filter,
        const board::AdvancedOptions& options, data::DataCallback& callback,
        const BeforeSession& before_session = nullptr);

    /**
     * @brief Single-product convenience overload.
     *
     * The span above exists because one firmware image can ship under several
     * product IDs; a board with exactly one says so here instead of declaring a
     * one-element array. The value is used during construction only, so the
     * temporary it spans cannot outlive its use.
     */
    Handler(
        uint16_t usb_vid, uint16_t usb_pid, std::string_view serial_filter,
        const board::AdvancedOptions& options, data::DataCallback& callback,
        const BeforeSession& before_session = nullptr)
        : Handler(
              usb_vid, std::span<const uint16_t>{&usb_pid, 1}, serial_filter, options, callback,
              before_session) {}

    Handler(const Handler&) = delete;
    Handler& operator=(const Handler&) = delete;
    Handler(Handler&& other) noexcept;
    Handler& operator=(Handler&& other) noexcept;

    ~Handler() noexcept;

    /**
     * @brief What the link is doing, for an application that has to react.
     *
     * The keepalive thread repairs what it can on its own, so kSessionDown is
     * normal and transient -- it is what a board looks like for the few hundred
     * milliseconds after a hiccup. kFaulted is terminal for THIS object: the
     * transport has stopped accepting traffic and only destroying the board and
     * constructing a new one can recover it.
     */
    enum class LinkState : std::uint8_t {
        kUp,          // session established; data is flowing
        kSessionDown, // transport alive, session being (re)established
        kFaulted,     // the device is gone; this board object cannot recover
    };

    [[nodiscard]] LinkState link_state() const noexcept;

    /// @brief A port's runtime status, from the most recent kPortStatus the
    /// board pushed after a keepalive ack (one record per port whose status
    /// changed, so this is at most one keepalive round old; every running port
    /// once right after each session start). Lock-free and free of USB
    /// traffic: callable from any thread, the control loop included. The
    /// counts wrap at 2^16: take differences of two reads modulo 2^16. A port
    /// this board does not have, or never declared, stays all-zero; one that
    /// is no longer declared keeps its last snapshot.
    ///
    /// The variant holds the kind the port's DataId implies (std::monostate
    /// before the first report, or for a port with no status kind). The link
    /// itself is a port too: status<data::LinkStatusView>(DataId::kSession) is
    /// the board's account of the downlink records it could not deliver.
    [[nodiscard]] data::PortStatusVariant port_status(data::DataId port) const noexcept;

    /// The same, as one kind: status<data::CanStatusView>(DataId::kCan1).
    /// All-zero when the port has reported nothing of that kind.
    template <typename View>
    [[nodiscard]] View status(data::DataId port) const noexcept {
        const data::PortStatusVariant any = port_status(port);
        const View* view = std::get_if<View>(&any);
        return view != nullptr ? *view : View{};
    }

    PacketBuilder start_transmit() noexcept;

    /**
     * @brief Asks the board to fire a hardware synchronisation pulse at an
     * absolute microframe of the shared USB-SOF timeline.
     *
     * Send the IDENTICAL microframe to every board in one round: each board's
     * capture offset from it carries the path delay plus the skew with opposite
     * sign, so differencing the boards' reports cancels the path delay. Requires
     * firmware built with -Dlibhcs_PULSE_TEST=ON.
     */
    void send_pulse_schedule(uint64_t microframe) noexcept;

    /**
     * @brief Runs the shared USB-SOF time base on this link from the next round on.
     *
     * Sends a kTimeAnchor alongside every keepalive and consumes the board's
     * kTimeStatus reply, which is what every SampleTime the data callbacks
     * receive is computed from. Call it only once the board has accepted a
     * manifest that asked for the time base (hcs::Configuration::time_sync) --
     * the board classes do so from the before-session hook, right after
     * hcs::apply(), so that the two can never disagree. A board that was not
     * asked does not answer the anchors.
     */
    void set_time_sync(bool on);

    /**
     * @brief The board's own clock, read backwards from a host instant.
     *
     * The inverse of what the data callbacks hand out: which instant on this
     * board's quarter-microsecond timer is simultaneous with `when` on this
     * machine's CLOCK_MONOTONIC. Costs one affine conversion, nothing else.
     *
     * Empty until both the controller's axis and this board's clock mapping
     * are locked (the first seconds of a session), and on a board whose time
     * base was never enabled. Call from the link's IO thread (a board class's
     * callback, or an application that keeps its own ordering) -- the mapping
     * it reads is owned by that thread.
     */
    [[nodiscard]] std::optional<libhcs::time::BoardClock::time_point>
        board_time_at(libhcs::time::HostTime when) const;

    /**
     * @brief Largest EP0 configuration payload this API will carry.
     *
     * v10's port declaration (kApplyManifest) is sized for the largest board's
     * port inventory and spans multiple EP0 packets -- TinyUSB's control
     * machinery chunks it on the device side, and libusb handles multi-packet
     * control transfers natively. The bound exists so the implementation can
     * stage the transfer in a fixed buffer instead of allocating per request;
     * it is derived from the protocol so the two cannot drift apart.
     */
    static constexpr size_t kVendorControlPayloadMax =
        sizeof(core::protocol::vendor_control::ManifestPayload);

    /**
     * @brief Sends one EP0 vendor configuration request to the board.
     *
     * The channel configuration path: UART baudrates and CAN bus modes travel
     * here rather than in the data stream, because a control transfer's status
     * stage reports whether the board accepted the setting and the bulk stream
     * cannot. Board classes call this while constructing, and read the setting
     * back before returning, so a constructed board object is a board whose
     * configuration is known rather than assumed.
     *
     * Independent of the session handshake: no nonce, no keepalive, and it
     * works before the first session has opened.
     *
     * @param request   A core::protocol::vendor_control::Request code.
     * @param index     Channel index (CAN bus or UART port); wIndex on the wire.
     * @param payload   Bytes to send (out) or the buffer to fill (in).
     * @param size      Payload size; must match what the board expects exactly.
     *
     * @return true when the board completed the request. false means the board
     *         STALLed it -- an explicit rejection, with nothing changed on the
     *         board; for a baudrate that is the divisor solver refusing the rate.
     *
     * @throws std::runtime_error when the transport has no control endpoint
     *         (any EtherCAT backend) or the transfer failed outright.
     * @throws std::invalid_argument when size exceeds kVendorControlPayloadMax.
     */
    bool vendor_control_out(uint8_t request, uint16_t index, const void* payload, size_t size);
    bool vendor_control_in(uint8_t request, uint16_t index, void* payload, size_t size);

private:
    class Impl;
    Impl* impl_ = nullptr;
};

} // namespace libhcs::host::protocol
