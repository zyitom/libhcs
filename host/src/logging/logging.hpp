#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <format>
#include <string_view>
#include <utility>

#include <libhcs/logging.hpp>

#include "core/src/utility/assert.hpp"

namespace libhcs::host::logging {

namespace detail {

// The application's sink, or nullptr for the built-in stderr output. Written by
// set_sink(), read on every log line. One pointer, so installing and reading it
// are each a single atomic access with nothing to tear against.
inline constinit std::atomic<Sink*> active_sink{nullptr};

// Hands one finished line (prefix and newline included) to stderr without ever
// waiting for it; see logging.cpp.
void write_stderr_line(std::string_view line) noexcept;

} // namespace detail

#ifndef libhcs_LOGGING_LEVEL
# define libhcs_LOGGING_LEVEL kInfo
#endif

/**
 * @brief Whether the `count`-th occurrence of a repeating condition should be logged.
 *
 * True for the 1st, 2nd, 4th, 8th ... occurrence, so a fault that repeats per
 * packet still says so immediately, keeps saying so while it is rare, and then
 * falls silent instead of burying the log. Measured need: a disconnected board
 * made a flood loop emit 473 MB of identical error lines in twenty seconds.
 *
 * @param count 1-based occurrence number, i.e. the value AFTER incrementing.
 */
constexpr bool should_log_occurrence(std::uint64_t count) noexcept {
    return count != 0 && (count & (count - 1U)) == 0U;
}

/**
 * @brief Formats log lines and hands them to whichever output is current.
 *
 * A small value, not a singleton: each transport owns one that carries its
 * board's serial number, so that with several boards in a process a line says
 * which of them it is about. Code with no board to speak for uses get_logger().
 *
 * Logging is const and keeps no state between lines, so one Logger may be used
 * from any number of threads at once. set_source() is the exception: call it
 * before the object is shared.
 */
class Logger {
public:
    static constexpr Level kLoggingLevel = Level::libhcs_LOGGING_LEVEL;

    // Long enough for the serial numbers these boards report; a longer one is
    // cut, which still tells boards apart.
    static constexpr std::size_t kSourceCapacity = 32;

    constexpr Logger() noexcept = default;

    explicit constexpr Logger(std::string_view source) noexcept { set_source(source); }

    constexpr void set_source(std::string_view source) noexcept {
        source_size_ = static_cast<std::uint8_t>(std::min(source.size(), source_.size()));
        std::copy_n(source.data(), source_size_, source_.data());
    }

    [[nodiscard]] constexpr std::string_view source() const noexcept {
        return {source_.data(), source_size_};
    }

    static constexpr bool should_log(Level level) noexcept { return level >= kLoggingLevel; }

public: // Logging.Formatted
    template <typename... Args>
    void trace(std::format_string<Args...> fmt, Args&&... args) const {
        log_internal(Level::kTrace, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void debug(std::format_string<Args...> fmt, Args&&... args) const {
        log_internal(Level::kDebug, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void info(std::format_string<Args...> fmt, Args&&... args) const {
        log_internal(Level::kInfo, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void warn(std::format_string<Args...> fmt, Args&&... args) const {
        log_internal(Level::kWarn, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void error(std::format_string<Args...> fmt, Args&&... args) const {
        log_internal(Level::kErr, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void critical(std::format_string<Args...> fmt, Args&&... args) const {
        log_internal(Level::kCritical, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log(Level level, std::format_string<Args...> fmt, Args&&... args) const {
        log_internal(level, fmt, std::forward<Args>(args)...);
    }

public: // Logging.Raw
    template <typename T>
    void trace(const T& msg) const {
        log_internal(Level::kTrace, msg);
    }

    template <typename T>
    void debug(const T& msg) const {
        log_internal(Level::kDebug, msg);
    }

    template <typename T>
    void info(const T& msg) const {
        log_internal(Level::kInfo, msg);
    }

    template <typename T>
    void warn(const T& msg) const {
        log_internal(Level::kWarn, msg);
    }

    template <typename T>
    void error(const T& msg) const {
        log_internal(Level::kErr, msg);
    }

    template <typename T>
    void critical(const T& msg) const {
        log_internal(Level::kCritical, msg);
    }

    template <typename T>
    void log(Level level, const T& msg) const {
        log_internal(level, msg);
    }

private:
    static constexpr std::string_view level_name(Level level) {
        if (level == Level::kTrace)
            return "trace";
        if (level == Level::kDebug)
            return "debug";
        if (level == Level::kInfo)
            return "info";
        if (level == Level::kWarn)
            return "warn";
        if (level == Level::kErr)
            return "error";
        if (level == Level::kCritical)
            return "critical";
        core::utility::assert_failed_debug();
    }

    // The line is formatted here, on the caller's thread, into a stack buffer:
    // no lock, no allocation, and whoever receives it gets it in one piece.
    // Longer messages truncate: a log line is not a storage format.
    //
    // Formatting is all this template does. Where the line goes is decided by
    // the two branches below, and neither of them may make this thread wait --
    // a sink by contract (libhcs/logging.hpp), stderr by construction
    // (write_stderr_line).
    template <typename... Args>
    void log_internal(Level level, std::format_string<Args...> fmt, Args&&... args) const {
        if (!should_log(level))
            return;

        // Uninitialized on purpose: exactly the bytes that were written below
        // are emitted, so a zero-fill would be a 1 KiB memset per log line
        // bought for nothing.
        std::array<char, 1024> line;
        char* const end = line.data() + line.size() - 1; // room for the newline

        // Loaded once: a set_sink() racing with this line must not make it
        // format for one destination and deliver to the other.
        if (Sink* const sink = detail::active_sink.load(std::memory_order::acquire)) {
            const auto message =
                std::format_to_n(line.data(), end - line.data(), fmt, std::forward<Args>(args)...);
            sink->write(
                Record{
                    .level = level,
                    .source = source(),
                    .message = std::string_view{line.data(), message.out}
            });
            return;
        }

        char* out =
            std::format_to_n(line.data(), end - line.data(), "[libhcs] [{}] ", level_name(level))
                .out;
        if (source_size_ != 0)
            out = std::format_to_n(out, end - out, "[{}] ", source()).out;
        out = std::format_to_n(out, end - out, fmt, std::forward<Args>(args)...).out;
        *out++ = '\n';
        detail::write_stderr_line(std::string_view{line.data(), out});
    }

    template <typename T>
    void log_internal(Level level, const T& msg) const {
        log_internal(level, "{}", msg);
    }

    std::array<char, kSourceCapacity> source_{};
    std::uint8_t source_size_ = 0;
};

/// The logger for lines that are not about any one board.
[[nodiscard]] inline const Logger& get_logger() noexcept {
    static constinit const Logger logger{};
    return logger;
}

} // namespace libhcs::host::logging
