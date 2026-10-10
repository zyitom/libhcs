#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string_view>
#include <thread>

#include <libhcs/export.hpp>

// v4 的传输层策略接口（HCS-USB.md §2.1）：Board 只经它碰 USB，测试注入
// FakeTransport 即可脱离硬件。
//
// 一个实现包住一条已经打开的、可靠有序的双向字节流（bulk IN/OUT）加一条 EP0
// 控制通道。怎么拿到这条流是实现自己的事：libusb 版在构造时扫描并打开设备
// （libhcs/transport/usb，经 Board 的默认工厂创建），U2 的 usbfs 版将来实现同
// 一个接口，FakeTransport（libhcs/testing/fake_transport.hpp）在内存里演一块
// 板。
//
// 线程约定：
//   - transmit() 从 Board 的 IO 线程（或 start() 的调用线程，会话还没开时）
//     调用；实现可以拷贝，但不能阻塞长——对 USB 实现就是一次异步提交。
//   - receive() 只能调一次；此后回调从传输自己的线程调用，同一时刻至多一个
//     线程在回调里。回调返回后，传入的 span 即失效。
//   - vendor_control() 阻塞到应答或失败，只在配置期（start() / 重连钩子）用。

namespace libhcs {

class libhcs_API Transport {
public:
    Transport() = default;
    virtual ~Transport() noexcept = default;

    Transport(const Transport&) = delete;
    Transport& operator=(const Transport&) = delete;
    Transport(Transport&&) = delete;
    Transport& operator=(Transport&&) = delete;

    // ---- 数据面 ----

    /// 把这些字节发给板子（下行流）。按到达顺序可靠投递。
    virtual void transmit(std::span<const std::byte> bytes) = 0;

    /// 注册上行流的接收回调。span 只在回调内有效。
    virtual void receive(std::function<void(std::span<const std::byte>)> callback) = 0;

    /// 传输层重启通知（可选能力）：一条重启了的数据流不能延续上一次的字节流
    /// 定界，Board 在此之后要复位协议状态再吃新字节。回调在传输自己的线程上，
    /// 不做实事，只做标记。
    virtual void on_link_restart(std::function<void()> callback) { (void)callback; }

    // ---- EP0 控制通道（配置期，阻塞）----
    //
    // 一个 EP0 厂商请求（libhcs/protocol/vendor_control.hpp 的编码）。payload
    // 对 OUT 是待发字节，对 IN 是应答缓冲；返回值区分四件事：成了、板子明确
    // 拒了（STALL）、这条传输根本没有控制端点、I/O 失败。后两者抛不抛由调用
    // 方（hcs_config.hpp 的配置流程）决定。
    enum class ControlResult : std::uint8_t {
        kOk,
        kStalled,
        kUnsupported,
        kFailed,
    };

    virtual ControlResult vendor_control(
        std::uint8_t request_type, std::uint8_t request, std::uint16_t index,
        std::span<std::byte> payload) = 0;

    // ---- 链路状态与放置 ----

    /// 链路是否已不可恢复地断了。断了之后 transmit 静默丢弃，vendor_control
    /// 失败；这个 Board 对象只能重建。
    [[nodiscard]] virtual bool link_faulted() const noexcept = 0;

    /// 链路级别的本地修复（清端点挂起、重挂接收池），会话重开之前由 IO 线程
    /// 调用。true = 修了点东西。没有可修的传输返回默认 false。
    virtual bool try_recover_link() { return false; }

    /// 对端序列号（日志用），没有则空。传输生命周期内不变。
    [[nodiscard]] virtual std::string_view serial() const noexcept = 0;

    /// 板子所在的 USB 总线号（时间轴按主机控制器分轴用），非 USB 传输 -1。
    [[nodiscard]] virtual int usb_bus_number() const noexcept = 0;

    /// Board 的 IO 线程起来之后调用一次：拥有自有线程的传输把 IO 线程放到和
    /// 自己同样的核与调度上（会话租约和 I/O 同生共死）。默认无事可做。
    virtual void configure_session_thread(std::thread& thread) noexcept { (void)thread; }
};

} // namespace libhcs
