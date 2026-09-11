#pragma once

#include <functional>
#include <type_traits>
#include <utility>

namespace libhcs::board {

/**
 * @brief Advanced transport options passed during board construction.
 *
 * `bind_advanced_options()` returns an object derived from `AdvancedOptions` whose `thread_setup`
 * callback depends on state stored in that derived object. Copying or moving the base type would
 * slice away that state while retaining a now-dangling function pointer, so `AdvancedOptions` is
 * intentionally non-copyable and non-movable.
 *
 * Construct `AdvancedOptions` directly during board construction, or explicitly copy the required
 * plain data fields into a fresh `AdvancedOptions` instance instead of copying an existing object.
 *
 * @warning Never copy, assign, or reuse any function pointer from an object returned by
 * `bind_advanced_options()` in any other `AdvancedOptions` instance; doing so is undefined
 * behavior.
 */
class AdvancedOptions {
public:
    AdvancedOptions() = default;

    AdvancedOptions(const AdvancedOptions&) = delete;
    AdvancedOptions& operator=(const AdvancedOptions&) = delete;
    AdvancedOptions(AdvancedOptions&&) = delete;
    AdvancedOptions& operator=(AdvancedOptions&&) = delete;

    ~AdvancedOptions() = default;

    bool dangerously_skip_version_checks = false;

    /**
     * @brief CPU to pin the transport's event thread to, or -1 to leave it free.
     *
     * This is the single largest latency lever measured on this link, and it
     * acts on the tail rather than the average. With an isolated core and
     * SCHED_FIFO (2026-09-04, CAN round trip under 82 kB/s of UART):
     *
     *     free-running   p99.9 = 529 us   max = 990 us
     *     pinned + RT    p99.9 = 161 us   max = 196 us
     *
     * Roughly 3.3x on p99.9 and 5x on max -- an order of magnitude more than
     * any USB-side tuning available on this hardware. Prefer a core that the
     * kernel has been told to leave alone (isolcpus / nohz_full); pinning to a
     * core the scheduler still uses buys much less.
     *
     * The session keepalive thread follows this thread: same CPUs, and one RT
     * priority step below it when it runs SCHED_FIFO/RR. That also covers
     * pinning done inside thread_setup. Leave both unset and the keepalive is
     * an ordinary thread that an RT busy-wait on its core can starve, which
     * ends the session after one second.
     */
    int io_thread_cpu = -1;

    /**
     * @brief SCHED_FIFO priority for that thread, or 0 to leave the policy alone.
     *
     * Only applied when io_thread_cpu is set. Needs CAP_SYS_NICE (or root);
     * failure is reported through the transport log and does not abort the
     * connection -- an unprivileged run still works, just with the free-running
     * tail above.
     *
     * @warning Do NOT pin an SCHED_FIFO thread to the same core as another
     * equally-prioritised busy thread. Equal-priority FIFO threads never
     * preempt each other, and the repository has a measured case of exactly
     * that deadlocking a link (see the event-loop comment in transport/usb).
     */
    int io_thread_rt_priority = 0;

    AdvancedOptions& set_io_thread_affinity(int cpu, int rt_priority = 0) {
        io_thread_cpu = cpu;
        io_thread_rt_priority = rt_priority;
        return *this;
    }

    /**
     * @brief Callback invoked on the transport event thread before transport I/O handling begins.
     *
     * This hook is intended only for per-thread environment setup, such as thread priority,
     * CPU affinity, thread naming, or other OS-level thread configuration.
     *
     * @warning This callback runs during transport construction, before the enclosing board object
     * finishes construction. If that board is itself a member of another object, that enclosing
     * object may also still be under construction.
     * @warning The callback must not access the board object being constructed, any state whose
     * lifetime depends on construction having finished, or any transport/protocol APIs. In
     * particular, do not capture and use `this` from an object that is still being constructed.
     */
    void (*thread_setup)(const AdvancedOptions&) noexcept = nullptr;

    /**
     * @brief Runs the shared USB-SOF time base on this link.
     *
     * Sends a kTimeAnchor alongside every keepalive and consumes the board's
     * kTimeStatus reply, feeding libhcs::host::time::timeline().
     *
     * @warning Opt-in, and it must stay that way: the time-sync session types
     * carry a payload after the session header, so a firmware that does not know
     * them cannot skip it and loses framing on the downlink. Enable it only for
     * boards built with -Dlibhcs_TIME_SYNC=ON.
     */
    bool enable_time_sync = false;

    AdvancedOptions& set_enable_time_sync(bool value) {
        enable_time_sync = value;
        return *this;
    }

    AdvancedOptions& set_dangerously_skip_version_checks(bool value) {
        dangerously_skip_version_checks = value;
        return *this;
    }

    AdvancedOptions& set_thread_setup(void (*value)(const AdvancedOptions&) noexcept) {
        thread_setup = value;
        return *this;
    }
};

/**
 * @brief Binds callable to `AdvancedOptions`.
 *
 * @warning The returned object must outlive any use of its function pointers.
 * @warning Do not store, copy, move, or slice the returned object as `AdvancedOptions`.
 */
template <typename FunctorT>
requires std::is_nothrow_invocable_v<const std::decay_t<FunctorT>&>
auto bind_advanced_options(FunctorT&& thread_setup_impl) {
    using Functor = std::decay_t<FunctorT>;

    class OptionsImpl : public AdvancedOptions {
    public:
        explicit OptionsImpl(Functor thread_setup_impl)
            : thread_setup_impl_(std::move(thread_setup_impl)) {
            thread_setup = [](const AdvancedOptions& self) noexcept {
                std::invoke(static_cast<const OptionsImpl&>(self).thread_setup_impl_);
            };
        }

    private:
        Functor thread_setup_impl_;
    };

    return OptionsImpl{std::forward<FunctorT>(thread_setup_impl)};
}

} // namespace libhcs::board
