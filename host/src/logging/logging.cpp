#include "host/src/logging/logging.hpp"

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <format>
#include <string_view>

#include <libhcs/logging.hpp>

#if defined(__linux__)
# include <poll.h>
# include <sys/uio.h>
# include <unistd.h>
#endif

namespace libhcs::host::logging {

namespace {

// Lines given up on since the last time that was reported, and in total.
constinit std::atomic<std::uint64_t> unreported_drops{0};
constinit std::atomic<std::uint64_t> total_drops{0};

// Writes `text` to stderr only if that can be done without waiting. False means
// the write would have blocked and nothing was written.
//
// stderr pointed at a pipe whose reader has stalled blocks write(2) for as long
// as the reader stays stalled, and the thread sitting in that write is usually
// the libusb event thread -- the whole link dies silently behind one log line.
// The file is not ours to make non-blocking (O_NONBLOCK lives on the open file
// description, which the launcher and every other writer share), so the wait is
// refused per call instead.
#if defined(__linux__)

bool try_write(std::string_view text) noexcept {
    const int fd = ::fileno(stderr);
    if (fd < 0)
        return true; // stderr is closed: nothing to wait for, nowhere to write

    // RWF_NOWAIT is the exact request -- "write this, or tell me it would
    // block" -- and pipes, sockets and regular files honour it. A line is far
    // below PIPE_BUF, so on a pipe it goes out whole or not at all.
    const iovec chunk{.iov_base = const_cast<char*>(text.data()), .iov_len = text.size()};
    ssize_t written = 0;
    do {
        written = ::pwritev2(fd, &chunk, 1, -1, RWF_NOWAIT);
    } while (written < 0 && errno == EINTR);
    if (written >= 0)
        return true;
    if (errno == EAGAIN)
        return false;
    if (errno != EOPNOTSUPP && errno != ENOSYS && errno != EINVAL)
        return true; // a real error; a failed write to stderr has nowhere to be reported

    // Terminals (and kernels older than the flag) do not take RWF_NOWAIT. Ask
    // first, then write. That is weaker: POLLOUT says there is room, not that it
    // will still be there when the write arrives, and a terminal does not say
    // how much. Hence the fallback and not the method -- and note_slow_write()
    // below for the write that waited anyway.
    pollfd ready{.fd = fd, .events = POLLOUT, .revents = 0};
    if (::poll(&ready, 1, 0) <= 0 || (ready.revents & POLLOUT) == 0)
        return false;
    const ssize_t ignored = ::write(fd, text.data(), text.size());
    (void)ignored;
    return true;
}

#else

// No portable way to refuse the wait here; this is the behaviour every platform
// had before.
bool try_write(std::string_view text) noexcept {
    (void)std::fwrite(text.data(), 1, text.size(), stderr);
    return true;
}

#endif

// Dropping is the policy, but it must not be a silent one: the first line that
// gets through again is preceded by how many did not.
bool report_drops() noexcept {
    const std::uint64_t pending = unreported_drops.exchange(0, std::memory_order::relaxed);
    if (pending == 0)
        return true;

    std::array<char, 96> notice;
    const auto result = std::format_to_n(
        notice.data(), notice.size(),
        "[libhcs] [warn] {} log line(s) dropped: stderr was not writable\n", pending);
    if (try_write(std::string_view{notice.data(), result.out}))
        return true;

    unreported_drops.fetch_add(pending, std::memory_order::relaxed); // still owed
    return false;
}

// Evidence for the cases the refusal above cannot cover -- a platform without
// it, a terminal that took the room between the poll and the write, a full
// disk: a write that did go through, but slowly, names its own delay.
constexpr auto kSlowWriteThreshold = std::chrono::milliseconds{100};

void note_slow_write(std::chrono::steady_clock::duration elapsed) noexcept {
    if (elapsed < kSlowWriteThreshold)
        return;
    static constinit std::atomic<std::uint64_t> occurrences{0};
    const auto count = occurrences.fetch_add(1, std::memory_order::relaxed) + 1;
    if (!should_log_occurrence(count))
        return;
    std::array<char, 160> line;
    const auto result = std::format_to_n(
        line.data(), line.size(),
        "[libhcs] [warn] stderr write took {:.1f} ms (x{}); the logging thread was stalled by "
        "a backed-up stderr\n",
        std::chrono::duration<double, std::milli>(elapsed).count(), count);
    // result.out, not result.size: size is the untruncated length.
    (void)try_write(std::string_view{line.data(), result.out});
}

} // namespace

void detail::write_stderr_line(std::string_view line) noexcept {
    const auto started = std::chrono::steady_clock::now();
    if (report_drops() && try_write(line)) {
        note_slow_write(std::chrono::steady_clock::now() - started);
        return;
    }
    unreported_drops.fetch_add(1, std::memory_order::relaxed);
    total_drops.fetch_add(1, std::memory_order::relaxed);
}

Sink* set_sink(Sink* sink) noexcept {
    return detail::active_sink.exchange(sink, std::memory_order::acq_rel);
}

std::uint64_t stderr_lines_dropped() noexcept {
    return total_drops.load(std::memory_order::relaxed);
}

} // namespace libhcs::host::logging
