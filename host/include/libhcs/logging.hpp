#pragma once

// Where the host SDK's own diagnostics go.
//
// The SDK logs from the threads a control loop depends on: the USB event
// thread, the session keepalive thread, and whoever is calling transmit(). Two
// things follow from that, and this header is both of them.
//
// 1. On its own, the SDK writes each line to stderr -- but never waits for it.
//    stderr is normally a pipe (a launcher, journald); when its reader falls
//    behind, an ordinary write(2) blocks, and the link would stall behind a log
//    line. So a line that cannot be written right now is dropped and counted,
//    and the count is reported once stderr takes writes again. Nothing has to be
//    configured for this.
//
// 2. An application that has a logging system of its own can take the lines
//    over, and then the SDK does not touch stderr at all:
//
//        class QueueSink final : public libhcs::host::logging::Sink {
//        public:
//            void write(const libhcs::host::logging::Record& record) noexcept override {
//                queue_.try_push(record.level, record.source, record.message); // copies
//            }
//        };
//
//        QueueSink sink;
//        const libhcs::host::logging::ScopedSink route{sink}; // before the boards
//
// The SDK knows nothing about what is behind a Sink, and depends on nothing but
// the standard library to offer one.

#include <cstdint>
#include <string_view>

#include <libhcs/export.hpp>

namespace libhcs::host::logging {

enum class Level : std::uint8_t {
    kTrace = 0,
    kDebug = 1,
    kInfo = 2,
    kWarn = 3,
    kErr = 4,
    kCritical = 5,
    kOff = 6,
};

/**
 * @brief One finished log line, as handed to a Sink.
 *
 * Both views point into the logging thread's stack and are valid only for the
 * duration of Sink::write(). Copy whatever has to outlive the call.
 */
struct Record {
    Level level;

    /**
     * @brief USB serial number of the board the line is about.
     *
     * Empty for lines that are not tied to one board: the device scan, and
     * anything logged before a board has been opened. With several boards in one
     * process this is the only thing that tells their lines apart.
     */
    std::string_view source;

    /// The text alone: no level, no source, no trailing newline. At most about 1000 bytes.
    std::string_view message;
};

/**
 * @brief Destination for the SDK's log lines, implemented by the application.
 *
 * write() is called on whichever SDK thread logged, with no lock held, so it may
 * run on several threads at once -- and one of them is the USB event thread. It
 * must not block and should not allocate: keeping that thread away from anything
 * that can wait is the reason to install a sink in the first place.
 *
 * An interface rather than a callback plus a context pointer: the object is the
 * context, and one pointer to it is all the SDK has to swap, which keeps
 * installing a sink a single atomic store.
 */
class Sink {
public:
    Sink(const Sink&) = delete;
    Sink& operator=(const Sink&) = delete;

    virtual void write(const Record& record) noexcept = 0;

protected:
    Sink() = default;
    // The SDK never owns a sink and never deletes one through this type.
    ~Sink() = default;
};

/**
 * @brief Routes every subsequent SDK log line to `sink`; nullptr restores stderr.
 *
 * Process-wide and callable from any thread at any time. Prefer ScopedSink,
 * which cannot forget the second half.
 *
 * A thread that was already inside the previous sink keeps running it to
 * completion, so a sink must outlive not just its installation but every board
 * that could still be logging through it. In practice: install before the first
 * board is constructed, remove after the last one is destroyed.
 *
 * @return The sink that was installed before, or nullptr if lines went to stderr.
 */
libhcs_API Sink* set_sink(Sink* sink) noexcept;

/**
 * @brief Installs a sink for as long as this object lives, then puts the previous one back.
 *
 * Scopes must nest: the one constructed last is destroyed first.
 */
class [[nodiscard]] ScopedSink {
public:
    explicit ScopedSink(Sink& sink) noexcept
        : previous_(set_sink(&sink)) {}

    ~ScopedSink() { (void)set_sink(previous_); }

    ScopedSink(const ScopedSink&) = delete;
    ScopedSink& operator=(const ScopedSink&) = delete;

private:
    Sink* previous_;
};

/**
 * @brief Lines the SDK discarded because stderr could not take them without waiting.
 *
 * Counts only the built-in stderr output; with a sink installed, what happens to
 * a line is the sink's business. The same number is also reported on stderr
 * itself as soon as it accepts writes again.
 */
[[nodiscard]] libhcs_API std::uint64_t stderr_lines_dropped() noexcept;

} // namespace libhcs::host::logging
