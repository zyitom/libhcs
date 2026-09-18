#pragma once

// Priority-inheriting mutex and its matching condition variable.
//
// WHY THIS EXISTS. The USB transport's free-transfer pools are touched by two
// threads with different scheduling priorities: the libusb event thread, which
// callers are expected to raise (transport.hpp's options.thread_setup exists for
// exactly that), and whatever application thread calls acquire/release. glibc
// gives std::mutex PTHREAD_PRIO_NONE, so a low-priority holder that gets
// preempted inside the critical section blocks the high-priority event thread
// for the whole of the preemption -- classic priority inversion, and the
// critical section's own length has nothing to do with how long it lasts.
//
// PTHREAD_PRIO_INHERIT makes the kernel boost the holder to the waiter's
// priority for the duration, which bounds the block by the critical section
// instead of by the scheduler. On a PREEMPT_RT kernel that is a real rtmutex.
//
// WHAT THIS IS NOT. It is insurance, not a measured fix. The equivalent lock in
// the EtherCAT backend (igh.cpp's LockedByteRing) was once suspected of causing
// that path's ~50 ms tail; LATENCY_ROADMAP_V2.md found the actual cause was
// kernel.sched_rt_runtime_us throttling, and the lock -- still a plain mutex --
// now measures max 48.3 us over 2M cycles. So do not expect this to move a
// number that is already clean; it removes an unbounded case, nothing more.
//
// std::condition_variable cannot wait on a non-std::mutex, and
// std::condition_variable_any would put its own internal mutex on the notify
// path -- the very path being protected here. Hence the thin pthread_cond_t
// wrapper rather than a standard-library type.

#include <chrono>
#include <cstdlib>
#include <mutex>

#include <pthread.h>

namespace libhcs::host::utility {

class PriorityInheritingMutex {
public:
    PriorityInheritingMutex() noexcept {
        pthread_mutexattr_t attributes;
        pthread_mutexattr_init(&attributes);
        // Best effort: a platform or kernel without PI support leaves the mutex
        // at the default protocol, which is exactly what std::mutex would have
        // given. Failing construction over a missing optimization would be worse
        // than running without it.
        // TEMPORARY A/B knob. Set libhcs_USB_PI_MUTEX=0 to construct plain
        // mutexes instead, so the same binary can measure both configurations
        // interleaved -- comparing two builds would let a slow drift masquerade
        // as the effect. Remove once the question is settled.
        if (priority_inheritance_enabled())
            pthread_mutexattr_setprotocol(&attributes, PTHREAD_PRIO_INHERIT);
        pthread_mutex_init(&mutex_, &attributes);
        pthread_mutexattr_destroy(&attributes);
    }

    ~PriorityInheritingMutex() { pthread_mutex_destroy(&mutex_); }

    PriorityInheritingMutex(const PriorityInheritingMutex&) = delete;
    PriorityInheritingMutex& operator=(const PriorityInheritingMutex&) = delete;
    PriorityInheritingMutex(PriorityInheritingMutex&&) = delete;
    PriorityInheritingMutex& operator=(PriorityInheritingMutex&&) = delete;

    void lock() noexcept { pthread_mutex_lock(&mutex_); }
    void unlock() noexcept { pthread_mutex_unlock(&mutex_); }
    bool try_lock() noexcept { return pthread_mutex_trylock(&mutex_) == 0; }

    pthread_mutex_t* native_handle() noexcept { return &mutex_; }

private:
    static bool priority_inheritance_enabled() noexcept {
        static const bool kEnabled = [] {
            const char* const disable = std::getenv("libhcs_USB_PI_MUTEX");
            return disable == nullptr || disable[0] != '0';
        }();
        return kEnabled;
    }

    pthread_mutex_t mutex_;
};

class PriorityInheritingConditionVariable {
public:
    PriorityInheritingConditionVariable() noexcept {
        pthread_condattr_t attributes;
        pthread_condattr_init(&attributes);
        // Timeouts must run on the monotonic clock: pthread_cond_timedwait
        // measures against the clock the condattr pins, and the default is
        // CLOCK_REALTIME -- a wall-clock step (settimeofday, an NTP step at
        // boot) would stretch or cut short every bounded wait on this condvar.
        pthread_condattr_setclock(&attributes, CLOCK_MONOTONIC);
        pthread_cond_init(&condition_, &attributes);
        pthread_condattr_destroy(&attributes);
    }

    ~PriorityInheritingConditionVariable() { pthread_cond_destroy(&condition_); }

    PriorityInheritingConditionVariable(const PriorityInheritingConditionVariable&) = delete;
    PriorityInheritingConditionVariable&
        operator=(const PriorityInheritingConditionVariable&) = delete;
    PriorityInheritingConditionVariable(PriorityInheritingConditionVariable&&) = delete;
    PriorityInheritingConditionVariable& operator=(PriorityInheritingConditionVariable&&) = delete;

    void notify_one() noexcept { pthread_cond_signal(&condition_); }
    void notify_all() noexcept { pthread_cond_broadcast(&condition_); }

    // pthread_cond_wait releases and reacquires the mutex behind the lock
    // object's back. That is safe here because the lock is held on entry and on
    // return, so unique_lock's ownership flag stays true and correct throughout.
    template <typename Predicate>
    void wait(std::unique_lock<PriorityInheritingMutex>& lock, Predicate predicate) {
        while (!predicate())
            pthread_cond_wait(&condition_, lock.mutex()->native_handle());
    }

    // Bounded wait against an absolute deadline, so a caller that loops on
    // spurious wakeups keeps one fixed deadline instead of restarting the clock
    // on every pass. Returns on notify, spurious wakeup or deadline alike: the
    // caller re-checks its own condition. libstdc++'s steady_clock is
    // CLOCK_MONOTONIC, the clock the condattr above pins, so its epoch count
    // converts to the timespec directly.
    void wait_until(
        std::unique_lock<PriorityInheritingMutex>& lock,
        std::chrono::steady_clock::time_point deadline) noexcept {
        const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(deadline);
        const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - seconds);
        const timespec until{
            .tv_sec = seconds.time_since_epoch().count(), .tv_nsec = nanos.count()};
        (void)pthread_cond_timedwait(&condition_, lock.mutex()->native_handle(), &until);
    }

private:
    pthread_cond_t condition_;
};

} // namespace libhcs::host::utility
