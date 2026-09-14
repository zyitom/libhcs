#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <format>
#include <string_view>
#include <utility>

#include "core/src/utility/assert.hpp"

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

#ifndef libhcs_LOGGING_LEVEL
# define libhcs_LOGGING_LEVEL kInfo
#endif

class Logger {
public:
    static constexpr Level kLoggingLevel = Level::libhcs_LOGGING_LEVEL;

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;
    ~Logger() = default;

public: // Singleton
    static Logger& get_instance() noexcept {
        static Logger logger{};
        return logger;
    }

public: // Logging
    static constexpr bool should_log(Level level) { return level >= kLoggingLevel; }

public: // Logging.Formatted
    template <typename... Args>
    void trace(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kTrace, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void debug(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kDebug, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void info(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kInfo, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void warn(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kWarn, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void error(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kErr, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void critical(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kCritical, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log(Level level, std::format_string<Args...> fmt, Args&&... args) {
        log_internal(level, fmt, std::forward<Args>(args)...);
    }

public: // Logging.Raw
    template <typename T>
    void trace(const T& msg) {
        log_internal(Level::kTrace, msg);
    }

    template <typename T>
    void debug(const T& msg) {
        log_internal(Level::kDebug, msg);
    }

    template <typename T>
    void info(const T& msg) {
        log_internal(Level::kInfo, msg);
    }

    template <typename T>
    void warn(const T& msg) {
        log_internal(Level::kWarn, msg);
    }

    template <typename T>
    void error(const T& msg) {
        log_internal(Level::kErr, msg);
    }

    template <typename T>
    void critical(const T& msg) {
        log_internal(Level::kCritical, msg);
    }

    template <typename T>
    void log(Level level, const T& msg) {
        log_internal(level, msg);
    }

private:
    constexpr Logger() noexcept = default;

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

    // One line, one write. stderr is unbuffered, so the previous prefix-write
    // plus println-write pair was two write(2) syscalls per line, and two
    // threads logging concurrently tore each other's lines apart mid-line.
    // Formatting into a single stack buffer and emitting it with one fwrite
    // costs no lock and no allocation on the caller, and the kernel delivers
    // each write() whole. Longer messages truncate: a log line is not a
    // storage format.
    template <typename... Args>
    void log_internal(Level level, std::format_string<Args...> fmt, Args&&... args) {
        if (!should_log(level))
            return;

        // Uninitialized on purpose: exactly the bytes that were written below
        // are emitted, so a zero-fill would be a 1 KiB memset per log line
        // bought for nothing.
        std::array<char, 1024> line;
        const auto prefix =
            std::format_to_n(line.data(), line.size() - 1, "[libhcs] [{}] ", level_name(level));
        const auto remaining = static_cast<std::size_t>(line.data() + line.size() - 1 - prefix.out);
        const auto message =
            std::format_to_n(prefix.out, remaining, fmt, std::forward<Args>(args)...);
        *message.out = '\n';
        // A failed write to stderr has nowhere left to be reported.
        (void)std::fwrite(
            line.data(), 1, static_cast<std::size_t>(message.out - line.data()) + 1, stderr);
    }

    template <typename T>
    void log_internal(Level level, const T& msg) {
        log_internal(level, "{}", msg);
    }
};

inline Logger& get_logger() { return Logger::get_instance(); }

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

} // namespace libhcs::host::logging
