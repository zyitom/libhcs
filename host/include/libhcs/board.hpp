#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

#include <libhcs/board/hcs_config.hpp>
#include <libhcs/channel.hpp>
#include <libhcs/export.hpp>
#include <libhcs/status.hpp>
#include <libhcs/transport.hpp>

// v4 的顶层对象（HCS-USB.md §2 / §8.1）：一块板的外观。运行时对象、pimpl，
// 不是模板——板型是 BoardOptions 里的一个名字，端口在运行时按丝印名
// （"CAN1"、"UART7"、"DBUS"）查。使用方式和 hcs-ecat 的 Master 一致：
//
//     libhcs::Board board{{
//         .board_type = "hpm5321",
//         .serial = "A1B2",
//         .time_sync = true,
//     }};
//     auto& can1 = board.add_can("CAN1", libhcs::board::hcs::kClassic1M);
//     auto& uart0 = board.add_uart("UART0", libhcs::board::hcs::uart_8n1(921600));
//     can1.set_safe_frames(zero_current_frames);  // 按设备协议给，例如全零电流帧
//     board.start();
//     board.run();
//     // 每拍：
//     board.receive();               // 把各通道这一拍的数据准备好
//     for (const auto& f : can1.frames()) ...
//     can1.stage(frame);
//     board.send();                  // 提交并叫醒 IO 线程
//
// 生命周期（HCS-USB.md §2.2）：打开与配置（EP0 清单一次事务、读回核对）都在
// start() 里完成，失败抛异常并带差异；start() 成功即 Armed——收数据正常，发送
// 只放行安全帧；run() 之后才是正常收发；stop() 任何时候可调。会话断开由 IO
// 线程自动重开（LinkState::kSessionDown），设备拔掉则 Faulted、只能重建对象。
//
// 配置期（构造、add_*、set_safe_*、start）抛异常；周期方法（receive/send/stage/
// frames/...）全部 noexcept。周期方法必须从同一个线程调用（接收环与暂存区是
// SPSC 的：IO 线程是另一端）；request_safe / link_state / statistics 任何线程可调。
// IO 线程是 SDK 自己的线程，核号与优先级经 BoardOptions 配置。

namespace libhcs {

/// BoardOptions 不认识的板型名。
class BoardTypeError final : public std::logic_error {
public:
    using std::logic_error::logic_error;
};

/// 端口名在这块板型上不存在，或类型不符（拿 "CAN1" 去 add_uart）。消息里列出
/// 这块板有哪些口。
class PortNotFoundError final : public std::logic_error {
public:
    using std::logic_error::logic_error;
};

/// 同一个口声明了两次，或给没有 IMU 的板型 add_imu。
class PortAlreadyDeclaredError final : public std::logic_error {
public:
    using std::logic_error::logic_error;
};

/// 一块板的构造参数。聚合类型，用指定初始化器填。
struct BoardOptions {
    /// 板型名："hpm5321" / "mc02" / "c_board" / "hpm6e8y"。决定 USB 标识与
    /// 端口表（名字 → 丝印）。
    std::string board_type = "hpm5321";
    /// 序列号过滤：非空时只匹配这个序列号的板；空则同型第一块。
    std::string serial;
    /// 只在明确要求时关掉固件版本核对（USB 产品串 + EP0 线格式指纹）。
    bool dangerously_skip_version_checks = false;
    /// 声明共享 USB-SOF 时间基准：CAN 帧带硬件时间戳，SampleTime 换算成
    /// CLOCK_MONOTONIC。板子报不出这项能力时 start() 抛错。
    bool time_sync = false;

    // ---- SDK 自己的线程 ----
    /// IO 线程（会话保活、发送提交）绑的核，-1 不绑。这是本链路最大的尾延迟
    /// 手段，实测见 AdvancedOptions 的注释（p99.9 529→161 us）。
    int io_thread_cpu = -1;
    /// IO 线程的 SCHED_FIFO 优先级，0 不改策略。需要 CAP_SYS_NICE。
    int io_thread_rt_priority = 0;
    /// SDK 的每个自有线程（IO 线程、USB 事件线程）启动时各调用一次，做线程
    /// 命名、mlockall 一类的事。不得访问 Board。
    void (*thread_setup)() noexcept = nullptr;

    // ---- 通道容量（每个通道，配置期一次分配）----
    std::uint32_t can_rx_frames = 256;  // CAN 接收环帧数
    std::uint32_t can_tx_frames = 128;  // CAN 单拍暂存帧数
    std::uint32_t uart_rx_chunks = 64;  // UART 接收环块数
    std::uint32_t uart_tx_chunks = 64;  // UART 单拍暂存块数
    std::uint32_t imu_rx_samples = 256; // IMU 每路接收环样本数
};

class libhcs_API Board final {
public:
    /// @param transport 为空时用默认的 libusb 传输（按板型 USB 标识扫描打开）；
    /// 测试注入 FakeTransport。注入的传输已被打开并会尽快回调 receive。
    explicit Board(BoardOptions options = {}, std::unique_ptr<Transport> transport = nullptr);
    ~Board(); // 等价于 stop()

    Board(const Board&) = delete;
    Board& operator=(const Board&) = delete;
    Board(Board&&) = delete;
    Board& operator=(Board&&) = delete;

    // ---- 声明（start() 之前）----

    /// 声明一路 CAN：丝印名 + 帧型与速率（接线事实）。名字在板型的端口表里
    /// 查，查不到抛 PortNotFoundError 并列出这块板有哪些口；重复声明抛
    /// PortAlreadyDeclaredError。start() 之后调用是逻辑错误。
    [[nodiscard]] CanChannel& add_can(std::string_view port_name, board::hcs::CanSetting setting);

    /// 声明一路串口：丝印名 + 完整设置（baudrate/字长/校验/停止位/极性都要给
    /// 全，见 hcs_config.hpp 的 UartSetting）。
    [[nodiscard]] UartChannel&
        add_uart(std::string_view port_name, board::hcs::UartSetting setting);

    /// 声明板载 IMU（物理量量程全给，见 ImuSetting）。板型没有 IMU 抛
    /// PortNotFoundError；重复声明抛 PortAlreadyDeclaredError。
    [[nodiscard]] ImuChannel& add_imu(board::hcs::ImuSetting setting);

    // ---- 生命周期 ----

    /// 打开设备、版本与能力核对、EP0 清单两阶段提交、读回核对、开会话。
    /// 失败抛异常并附差异（哪个口、什么原因、板子此刻什么状态）。成功后处于
    /// Armed：数据在收，发送只放行安全帧。可重复调用（stop 之后再 start）。
    void start();

    /// Armed → Running：正常收发。其他状态下调用抛 std::logic_error。
    void run();

    /// Running → Armed：发送重新只放行安全帧。其他状态下调用抛 std::logic_error。
    void arm();

    /// 停会话、收线程，回 kIdle。可重复调用，任何状态可调。
    void stop() noexcept;

    // ---- 周期阶段（使用者线程，每拍）----

    /// 把各通道这一拍的数据从接收环取进快照区。之后 frames()/chunks()/
    /// samples() 读到的就是这一拍的内容，到下一次 receive() 为止。
    void receive() noexcept libhcs_NONBLOCKING;

    /// 提交各通道暂存的发送内容并叫醒 IO 线程（它把这一拍所有端口的记录打
    /// 成尽量少的 USB 包发出）。Armed / request_safe 状态下只发各口声明的
    /// 安全帧，暂存内容丢弃并计数。
    void send() noexcept libhcs_NONBLOCKING;

    // ---- 任何线程 ----

    /// 请求/撤销全体安全态：true 时每个输出通道只发安全帧（下一次 send() 起，
    /// IO 线程同时立刻补发一轮）。原子置位，不阻塞。
    void request_safe(bool safe) noexcept;

    [[nodiscard]] BoardState state() const noexcept;
    [[nodiscard]] LinkState link_state() const noexcept;
    [[nodiscard]] Statistics statistics() const noexcept;

    /// 声明的通道，按 add_* 的顺序。start() 之后接线表就固定了。
    [[nodiscard]] std::span<CanChannel* const> can_channels() const noexcept;
    [[nodiscard]] std::span<UartChannel* const> uart_channels() const noexcept;
    /// 没声明 IMU 时为空。
    [[nodiscard]] ImuChannel* imu_channel() const noexcept;

    /// 对板板的序列号（start 之后有效；start 前为空）。
    [[nodiscard]] std::string_view serial_number() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace libhcs
