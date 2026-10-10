// v4 主机侧接口的端到端测试：Board + FakeTransport，不碰 USB。覆盖任务清单：
// 声明与端口名查找的报错、start（清单、版本核对、缺口、能力）、拉取式收发
// （多帧、跨拍、溢出计数）、stage 满、request_safe / Armed 的安全帧、断线与拔出
// 的 LinkState、时间戳换算、IMU 通道、stop/再 start。
//
// FakeTransport 的应答是同步的（transmit 里就地回），所以多数断言不需要等；
// 只有经 IO 线程的发送提交（send() 之后）是异步的，用 eventually() 轮询。假板
// 的下行捕获是"取出即清空"，轮询时攒进 Capture，避免两次 take 之间到货的帧
// 被丢掉。

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <thread>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include <libhcs/board.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/testing/fake_transport.hpp>

#include "host/src/time/usb_frame_axis.hpp"

namespace {

namespace hcs = libhcs::board::hcs;
namespace spec = libhcs::spec;
using libhcs::Board;
using libhcs::BoardOptions;
using libhcs::BoardState;
using libhcs::FakeTransport;
using libhcs::LinkState;
using libhcs::data::DataId;
using DataIdValue = std::underlying_type_t<DataId>;

using Clock = std::chrono::steady_clock;

// 轮询到谓词为真（或超时假）。IO 线程的发送提交是异步的，这里给足余量。
template <typename Pred>
bool eventually(Pred&& pred, std::chrono::milliseconds timeout = std::chrono::milliseconds{2000}) {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        if (pred())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    return pred();
}

// 花括号字面量到 span 的桥（std::vector 隐式转 span）。
std::vector<std::byte> bytes(auto&&... values)
    requires (std::constructible_from<std::byte, decltype(values)> && ...)
{
    return std::vector<std::byte>{static_cast<std::byte>(values)...};
}

testing::AssertionResult mentions(const std::string& message, const std::string& expected) {
    if (message.find(expected) != std::string::npos)
        return testing::AssertionSuccess();
    return testing::AssertionFailure()
           << "expected the error to mention \"" << expected << "\", got: \"" << message << "\"";
}

constexpr std::uint8_t kCanCaps =
    spec::kCanCapModeSettable | spec::kCanCapRateSettable | spec::kCanCapFdLongFrames;

constexpr FakeTransport::Port port_of(DataId id, spec::PortKind kind, std::uint8_t caps = 0,
                                      std::uint8_t status = spec::kPortRunning) {
    return {
        .data_id = static_cast<std::uint8_t>(id),
        .kind = std::to_underlying(kind),
        .capabilities = caps,
        .status = status,
    };
}

// hpm5321 双 CAN 板的口表（kGetPortList 的内容）。
constexpr FakeTransport::Port kHpm5321DualPorts[]{
    port_of(DataId::kCan1, spec::PortKind::kCan, kCanCaps, spec::kPortRunning | spec::kPortFd),
    port_of(DataId::kCan2, spec::PortKind::kCan, kCanCaps, spec::kPortRunning | spec::kPortFd),
    port_of(DataId::kUart0, spec::PortKind::kUart),
};

// mc02 里本测试用到的一部分（CAN1、DBUS、IMU）。
constexpr FakeTransport::Port kMc02Ports[]{
    port_of(DataId::kCan1, spec::PortKind::kCan, kCanCaps, spec::kPortRunning | spec::kPortFd),
    port_of(DataId::kUartDbus, spec::PortKind::kUart, spec::kUartCapRxPolaritySettable),
    port_of(DataId::kImu, spec::PortKind::kImu),
};

libhcs::CanFrame make_frame(std::uint32_t id, std::byte fill, std::uint8_t size = 8) {
    libhcs::CanFrame frame;
    frame.id = id;
    frame.size = size;
    frame.data.fill(fill);
    return frame;
}

bool same_frame(const libhcs::CanFrame& sent, const FakeTransport::SentCan& captured) {
    return sent.id == captured.id && sent.extended == captured.extended
        && sent.remote == captured.remote && sent.size == captured.payload.size()
        && std::memcmp(sent.data.data(), captured.payload.data(), sent.size) == 0;
}

class BoardV4Test : public ::testing::Test {
protected:
    // 板型/时间基准这些常用开关的简写（语句式赋值，不触 -Wmissing-field-initializers）。
    static BoardOptions options_for(std::string_view board_type, bool time_sync = false) {
        BoardOptions options;
        options.board_type = board_type;
        options.time_sync = time_sync;
        return options;
    }

    // 造一块接在 FakeTransport 上的板。返回引用与 fake_ 终身稳定。
    Board& make_board(
        BoardOptions options = {}, std::span<const FakeTransport::Port> ports = kHpm5321DualPorts,
        std::uint8_t board_caps = libhcs::kBoardCapTimeSync) {
        auto transport = std::make_unique<FakeTransport>(ports, board_caps);
        fake_ = transport.get();
        board_ = std::make_unique<Board>(std::move(options), std::move(transport));
        return *board_;
    }

    Board& make_typed_board(
        std::string_view board_type, std::span<const FakeTransport::Port> ports,
        std::uint8_t board_caps = libhcs::kBoardCapTimeSync) {
        return make_board(options_for(board_type), ports, board_caps);
    }

    // 攒式取下行捕获：eventually 的每趟都先 pull，谓词看到的是累计。
    void pull() {
        auto can = fake_->take_sent_can();
        sent_can_.insert(sent_can_.end(), can.begin(), can.end());
        auto uart = fake_->take_sent_uart();
        sent_uart_.insert(sent_uart_.end(), uart.begin(), uart.end());
    }

    FakeTransport* fake_ = nullptr;
    std::unique_ptr<Board> board_;
    std::vector<FakeTransport::SentCan> sent_can_;
    std::vector<FakeTransport::SentUart> sent_uart_;
};

// ---- 声明与端口名查找 ----

TEST_F(BoardV4Test, UnknownBoardTypeListsTheKnownOnes) {
    try {
        const Board board{options_for("rmcs")};
        (void)board;
        FAIL() << "expected BoardTypeError";
    } catch (const libhcs::BoardTypeError& error) {
        EXPECT_TRUE(mentions(error.what(), "rmcs"));
        EXPECT_TRUE(mentions(error.what(), "hpm5321"));
        EXPECT_TRUE(mentions(error.what(), "mc02"));
    }
}

TEST_F(BoardV4Test, UnknownPortNameListsTheBoardsPorts) {
    auto& board = make_board();
    try {
        (void)board.add_can("CAN9", hcs::kClassic1M);
        FAIL() << "expected PortNotFoundError";
    } catch (const libhcs::PortNotFoundError& error) {
        EXPECT_TRUE(mentions(error.what(), "CAN9"));
        EXPECT_TRUE(mentions(error.what(), "CAN1"));
        EXPECT_TRUE(mentions(error.what(), "CAN2"));
    }

    // 类型不符：拿 CAN 的名字去 add_uart，消息列出串口。
    try {
        (void)board.add_uart("CAN1", hcs::uart_8n1(115200));
        FAIL() << "expected PortNotFoundError";
    } catch (const libhcs::PortNotFoundError& error) {
        EXPECT_TRUE(mentions(error.what(), "UART0"));
    }
}

TEST_F(BoardV4Test, DeclaringTheSamePortTwiceIsRejected) {
    auto& board = make_board();
    (void)board.add_can("CAN1", hcs::kClassic1M);
    EXPECT_THROW((void)board.add_can("CAN1", hcs::kFd1M5M), libhcs::PortAlreadyDeclaredError);
}

TEST_F(BoardV4Test, BoardTypeWithoutThePortStillAcceptsTheName) {
    // hpm6e8y：CAN0 存在（丝印从 0 起），但一块串口都没有、也没有 IMU。
    constexpr FakeTransport::Port hpm6e8y[]{
        port_of(DataId::kCan0, spec::PortKind::kCan, kCanCaps, spec::kPortRunning | spec::kPortFd),
    };
    auto& board = make_typed_board("hpm6e8y", hpm6e8y);
    (void)board.add_can("CAN0", hcs::kClassic1M);
    EXPECT_THROW((void)board.add_uart("UART0", hcs::uart_8n1(115200)), libhcs::PortNotFoundError);
    EXPECT_THROW(
        (void)board.add_imu({.accelerometer_range_g = 6,
                       .accelerometer_rate_hz = 1600,
                       .gyroscope_range_dps = 2000,
                       .gyroscope_rate_hz = 2000,
                       .gyroscope_bandwidth_hz = 230}),
        libhcs::PortNotFoundError);
}

TEST_F(BoardV4Test, StagingBeforeStartIsRefusedAndCounted) {
    auto& board = make_board();
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    EXPECT_FALSE(can1.stage(make_frame(0x200, std::byte{1})));
    EXPECT_EQ(can1.status().counters.tx_dropped, 1U);
}

// ---- start：清单、版本、能力、缺口 ----

TEST_F(BoardV4Test, StartAppliesTheManifestAndEndsArmed) {
    auto& board = make_board();
    (void)board.add_can("CAN1", hcs::kClassic1M);
    (void)board.add_uart("UART0", hcs::uart_8n1(921600));

    board.start();
    EXPECT_EQ(board.state(), BoardState::kArmed);
    EXPECT_EQ(board.link_state(), LinkState::kUp);

    const auto manifest = fake_->last_manifest();
    ASSERT_EQ(manifest.size(), 2U);
    EXPECT_EQ(manifest[0].data_id, std::to_underlying(DataId::kCan1));
    EXPECT_EQ(manifest[1].data_id, std::to_underlying(DataId::kUart0));
}

TEST_F(BoardV4Test, StartFailsWhenTheBoardLacksThePort) {
    // 单 CAN 板的口表：CAN2 的名字过得了板型表（spec 里有），start 的清单核对
    // 处被板子自报的口表拒绝。
    constexpr FakeTransport::Port single_can[]{
        port_of(DataId::kCan1, spec::PortKind::kCan, kCanCaps),
    };
    auto& board = make_board({}, single_can);
    (void)board.add_can("CAN2", hcs::kClassic1M);
    try {
        board.start();
        FAIL() << "expected the manifest to be rejected";
    } catch (const std::exception& error) {
        EXPECT_TRUE(mentions(error.what(), "CAN2"));
    }
    EXPECT_EQ(board.state(), BoardState::kIdle);
}

TEST_F(BoardV4Test, StartFailsOnVersionMismatch) {
    auto& board = make_board();
    (void)board.add_can("CAN1", hcs::kClassic1M);
    fake_->set_reported_version(0xBEEF);
    try {
        board.start();
        FAIL() << "expected a version mismatch";
    } catch (const std::exception& error) {
        EXPECT_TRUE(mentions(error.what(), "version"));
    }
}

TEST_F(BoardV4Test, TimeSyncNeedsTheBoardCapability) {
    auto& board = make_board(options_for("hpm5321", true), kHpm5321DualPorts, 0);
    (void)board.add_can("CAN1", hcs::kClassic1M);
    EXPECT_THROW(board.start(), std::exception);
}

// ---- 拉取式收发 ----

TEST_F(BoardV4Test, ReceivePullsAllFramesOfTheTick) {
    auto& board = make_board();
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    board.start();

    fake_->inject_can_frame(DataId::kCan1, 0x201, bytes(std::byte{1}, std::byte{2}));
    fake_->inject_can_frame(DataId::kCan1, 0x202, bytes(std::byte{3}));
    fake_->inject_can_frame(DataId::kCan1, 0x203, bytes(std::byte{4}, std::byte{5}, std::byte{6}));

    board.receive();
    ASSERT_EQ(can1.frames().size(), 3U);
    EXPECT_EQ(can1.frames()[0].id, 0x201U);
    EXPECT_EQ(can1.frames()[0].size, 2U);
    EXPECT_EQ(can1.frames()[0].data[0], std::byte{1});
    EXPECT_EQ(can1.frames()[1].id, 0x202U);
    EXPECT_EQ(can1.frames()[2].id, 0x203U);
    EXPECT_EQ(can1.frames()[2].size, 3U);

    // 拍内数据被 receive 取走：下一拍没有新数据就是空。
    board.receive();
    EXPECT_TRUE(can1.frames().empty());
}

TEST_F(BoardV4Test, TicksDoNotMix) {
    auto& board = make_board();
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    board.start();

    fake_->inject_can_frame(DataId::kCan1, 0x101, bytes(std::byte{1}));
    board.receive();
    ASSERT_EQ(can1.frames().size(), 1U);

    fake_->inject_can_frame(DataId::kCan1, 0x102, bytes(std::byte{2}));
    fake_->inject_can_frame(DataId::kCan1, 0x103, bytes(std::byte{3}));
    board.receive();
    ASSERT_EQ(can1.frames().size(), 2U);
    EXPECT_EQ(can1.frames()[0].id, 0x102U);
    EXPECT_EQ(can1.frames()[1].id, 0x103U);
}

TEST_F(BoardV4Test, ReceiveRingDropsTheOldestAndCounts) {
    BoardOptions options;
        options.can_rx_frames = 4;
        auto& board = make_board(options);
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    board.start();

    for (std::uint32_t id = 1; id <= 6; ++id)
        fake_->inject_can_frame(DataId::kCan1, id, bytes(std::byte{static_cast<std::uint8_t>(id)}));

    board.receive();
    ASSERT_EQ(can1.frames().size(), 4U);
    EXPECT_EQ(can1.frames().front().id, 3U) << "the newest four survive";
    EXPECT_EQ(can1.frames().back().id, 6U);
    EXPECT_EQ(can1.status().counters.rx_dropped, 2U);
    EXPECT_EQ(board.statistics().rx_dropped, 2U);
}

TEST_F(BoardV4Test, UartChunksCarryBytesAndIdleFlag) {
    auto& board = make_board();
    auto& uart0 = board.add_uart("UART0", hcs::uart_8n1(100000));
    board.start();

    const std::array<std::byte, 5> bytes{
        std::byte{9}, std::byte{8}, std::byte{7}, std::byte{6}, std::byte{5}};
    fake_->inject_uart_bytes(DataId::kUart0, bytes, true);
    board.receive();

    ASSERT_EQ(uart0.chunks().size(), 1U);
    EXPECT_EQ(uart0.chunks()[0].size, 5U);
    EXPECT_TRUE(uart0.chunks()[0].idle_delimited);
    EXPECT_EQ(std::memcmp(uart0.chunks()[0].data.data(), bytes.data(), bytes.size()), 0);
    EXPECT_EQ(uart0.chunks()[0].time.source, libhcs::time::SampleTime::Source::kHostArrival);
}

TEST_F(BoardV4Test, StageAndSendGoOutAsOneBurst) {
    auto& board = make_board();
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    board.start();
    board.run();

    const auto first = make_frame(0x301, std::byte{0x11});
    const auto second = make_frame(0x302, std::byte{0x22});
    EXPECT_TRUE(can1.stage(first));
    EXPECT_TRUE(can1.stage(second));
    board.send();

    ASSERT_TRUE(eventually([&] {
        pull();
        return sent_can_.size() >= 2U;
    }));
    ASSERT_EQ(sent_can_.size(), 2U);
    EXPECT_TRUE(same_frame(first, sent_can_[0]));
    EXPECT_TRUE(same_frame(second, sent_can_[1]));
    EXPECT_EQ(can1.status().counters.tx, 2U);
}

TEST_F(BoardV4Test, StageFullReturnsFalseAndCounts) {
    BoardOptions options;
        options.can_tx_frames = 2;
        auto& board = make_board(options);
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    board.start();

    EXPECT_TRUE(can1.stage(make_frame(1, std::byte{1})));
    EXPECT_TRUE(can1.stage(make_frame(2, std::byte{2})));
    EXPECT_FALSE(can1.stage(make_frame(3, std::byte{3})));
    EXPECT_EQ(can1.status().counters.tx_dropped, 1U);
}

TEST_F(BoardV4Test, LongFramesAreGatedByTheDeclaredBusType) {
    auto& board = make_board();
    auto& classic = board.add_can("CAN1", hcs::kClassic1M);
    auto& fd = board.add_can("CAN2", hcs::kFd1M5M);
    board.start();
    board.run();

    EXPECT_FALSE(classic.stage(make_frame(1, std::byte{0}, 12))) << "12 bytes on classic CAN";
    EXPECT_TRUE(fd.stage(make_frame(2, std::byte{0}, 12)));
}

TEST_F(BoardV4Test, ImuSamplesComeOutPerKind) {
    auto& board = make_typed_board("mc02", kMc02Ports);
    auto& imu = board.add_imu({.accelerometer_range_g = 6,
                               .accelerometer_rate_hz = 1600,
                               .gyroscope_range_dps = 2000,
                               .gyroscope_rate_hz = 2000,
                               .gyroscope_bandwidth_hz = 230});
    board.start();

    fake_->inject_imu_accelerometer(100, -200, 300);
    fake_->inject_imu_gyroscope(-1, 2, -3);
    fake_->inject_imu_temperature(0x1234);
    board.receive();

    const libhcs::ImuData samples = imu.samples();
    ASSERT_EQ(samples.accelerometer.size(), 1U);
    EXPECT_EQ(samples.accelerometer[0].x, 100);
    EXPECT_EQ(samples.accelerometer[0].y, -200);
    EXPECT_EQ(samples.accelerometer[0].z, 300);
    ASSERT_EQ(samples.gyroscope.size(), 1U);
    EXPECT_EQ(samples.gyroscope[0].z, -3);
    ASSERT_EQ(samples.temperature.size(), 1U);
    EXPECT_EQ(samples.temperature[0].raw_register_value, 0x1234);
}

// ---- 安全帧与生命周期门 ----

TEST_F(BoardV4Test, ArmedSendsOnlySafeFrames) {
    auto& board = make_board();
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    const auto safe = make_frame(0x205, std::byte{0});
    can1.set_safe_frames(std::array<libhcs::CanFrame, 1>{safe});
    board.start(); // Armed

    EXPECT_TRUE(can1.stage(make_frame(0x301, std::byte{0x55})));
    board.send();

    ASSERT_TRUE(eventually([&] {
        pull();
        return !sent_can_.empty();
    }));
    ASSERT_EQ(sent_can_.size(), 1U);
    EXPECT_TRUE(same_frame(safe, sent_can_[0])) << "the staged frame is held back";
    EXPECT_GE(can1.status().counters.tx_dropped, 1U);
    EXPECT_GE(can1.status().counters.safe_sends, 1U);

    // run() 之后正常帧放行；arm() 回到 Armed 又只发安全帧。
    board.run();
    const auto normal = make_frame(0x302, std::byte{0x66});
    EXPECT_TRUE(can1.stage(normal));
    board.send();
    ASSERT_TRUE(eventually([&] {
        pull();
        return !sent_can_.empty() && sent_can_.back().id == normal.id;
    }));

    board.arm();
    const auto stale = make_frame(0x303, std::byte{0x77});
    EXPECT_TRUE(can1.stage(stale));
    board.send();
    ASSERT_TRUE(eventually([&] {
        pull();
        return !sent_can_.empty() && sent_can_.back().id == safe.id;
    }));
}

TEST_F(BoardV4Test, RequestSafeReplacesStagedFrames) {
    auto& board = make_board();
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    const auto safe = make_frame(0x205, std::byte{0});
    can1.set_safe_frames(std::array<libhcs::CanFrame, 1>{safe});
    board.start();
    board.run();

    const auto normal = make_frame(0x301, std::byte{0x55});
    EXPECT_TRUE(can1.stage(normal));
    board.send();
    ASSERT_TRUE(eventually([&] {
        pull();
        return !sent_can_.empty() && sent_can_.back().id == normal.id;
    }));

    board.request_safe(true);
    EXPECT_TRUE(can1.stage(make_frame(0x302, std::byte{0x66})));
    board.send();
    ASSERT_TRUE(eventually([&] {
        pull();
        return !sent_can_.empty() && sent_can_.back().id == safe.id;
    }));

    // 撤销之后恢复正常发送。
    board.request_safe(false);
    EXPECT_TRUE(can1.stage(make_frame(0x303, std::byte{0x77})));
    board.send();
    ASSERT_TRUE(eventually([&] {
        pull();
        return !sent_can_.empty() && sent_can_.back().id == 0x303;
    }));
}

TEST_F(BoardV4Test, UartSafeBytesGoOutWhenGated) {
    auto& board = make_board();
    auto& uart0 = board.add_uart("UART0", hcs::uart_8n1(115200));
    const std::array<std::byte, 3> safe_bytes{std::byte{0}, std::byte{0}, std::byte{0}};
    uart0.set_safe_bytes(safe_bytes);
    board.start();

    EXPECT_TRUE(uart0.stage(bytes(std::byte{1}, std::byte{2})));
    board.send();
    ASSERT_TRUE(eventually([&] {
        pull();
        return !sent_uart_.empty();
    }));
    ASSERT_EQ(sent_uart_.size(), 1U);
    ASSERT_EQ(sent_uart_[0].bytes.size(), 3U);
    EXPECT_EQ(sent_uart_[0].bytes[0], std::byte{0});
}

// ---- 链路状态 ----

TEST_F(BoardV4Test, RemovalFaultsTheLink) {
    auto& board = make_board();
    (void)board.add_can("CAN1", hcs::kClassic1M);
    board.start();
    fake_->simulate_removal();
    EXPECT_EQ(board.link_state(), LinkState::kFaulted);
}

TEST_F(BoardV4Test, DisconnectDropsToSessionDownThenRecovers) {
    auto& board = make_board();
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    const auto safe = make_frame(0x205, std::byte{0});
    can1.set_safe_frames(std::array<libhcs::CanFrame, 1>{safe});
    board.start();
    board.run();

    fake_->simulate_disconnect(true);
    EXPECT_TRUE(eventually(
        [&] { return board.link_state() == LinkState::kSessionDown; },
        std::chrono::milliseconds{8000}))
        << "keepalive retries (5 x 200 ms) then the session drops";
    EXPECT_GE(board.statistics().session_losses, 1U);

    // 掉线期间：IO 线程补发了安全帧（假板断线时仍解析下行，看得见）。
    EXPECT_TRUE(eventually([&] {
        pull();
        return std::ranges::any_of(
            sent_can_, [&](const FakeTransport::SentCan& frame) { return frame.id == safe.id; });
    }));

    // 恢复：重连钩子重放清单，会话回来，正常发送继续。
    fake_->simulate_disconnect(false);
    EXPECT_TRUE(eventually(
        [&] { return board.link_state() == LinkState::kUp; }, std::chrono::milliseconds{8000}));

    const auto after = make_frame(0x401, std::byte{0x11});
    EXPECT_TRUE(can1.stage(after));
    board.send();
    ASSERT_TRUE(eventually([&] {
        pull();
        return !sent_can_.empty() && sent_can_.back().id == after.id;
    }));
}

TEST_F(BoardV4Test, StopIsIdempotentAndRestartWorks) {
    auto& board = make_board();
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    board.start();
    board.run();

    board.stop();
    EXPECT_EQ(board.state(), BoardState::kIdle);
    board.stop(); // 幂等

    // 停机后的收发是安静的：stage 被拒，receive 为空。
    EXPECT_FALSE(can1.stage(make_frame(1, std::byte{1})));
    fake_->inject_can_frame(DataId::kCan1, 7, bytes(std::byte{7}));
    board.receive();
    EXPECT_TRUE(can1.frames().empty());

    // 再 start：同一份声明重新下发，会话回来。
    board.start();
    EXPECT_EQ(board.state(), BoardState::kArmed);
    EXPECT_EQ(board.link_state(), LinkState::kUp);
    EXPECT_EQ(fake_->last_manifest().size(), 1U);
}

// ---- 时间戳换算 ----

TEST_F(BoardV4Test, CanStampsConvertToMonotonicThroughTheSharedAxis) {
    auto& board = make_board(options_for("hpm5321", true));
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);

    // 共享轴按 USB 总线分轴；假板没有总线，落在进程共用的未绑定轴（bus -1）。
    // 时间基准锁定要 16 个样本，锚点一轮一个（250 ms）太慢——这里直接给轴喂
    // 同一映射（假板纪元起的 steady_clock / 125 us，斜率精确 125000 ns/帧）的
    // 20 个样本，喂完即锁定。
    const auto axis = libhcs::host::time::UsbFrameAxis::for_usb_bus(-1);
    for (int step = 20; step >= 1; --step) {
        const auto when = Clock::now() - std::chrono::milliseconds{100 * step};
        axis->observe(
            static_cast<double>(fake_->microframe_at(when)), when - std::chrono::microseconds{50},
            when + std::chrono::microseconds{50});
    }

    board.start();

    const auto before = Clock::now();
    fake_->inject_can_frame(DataId::kCan1, 0x111, bytes(std::byte{1}), fake_->stamp_now());
    const auto after = Clock::now();
    board.receive();

    ASSERT_EQ(can1.frames().size(), 1U);
    const auto& frame = can1.frames()[0];
    EXPECT_EQ(frame.time.source, libhcs::time::SampleTime::Source::kBoardStamp);
    EXPECT_GE(frame.time.host, before - std::chrono::milliseconds{5});
    EXPECT_LE(frame.time.host, after + std::chrono::milliseconds{5});
}

TEST_F(BoardV4Test, UnstampedFramesFallBackToArrivalTime) {
    auto& board = make_board();
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    board.start();

    const auto before = Clock::now();
    fake_->inject_can_frame(DataId::kCan1, 0x111, bytes(std::byte{1})); // 无戳
    const auto after = Clock::now();
    board.receive();

    ASSERT_EQ(can1.frames().size(), 1U);
    EXPECT_EQ(can1.frames()[0].time.source, libhcs::time::SampleTime::Source::kHostArrival);
    EXPECT_GE(can1.frames()[0].time.host, before);
    EXPECT_LE(can1.frames()[0].time.host, after);
}

// ---- 统计 ----

TEST_F(BoardV4Test, StatisticsAccumulate) {
    auto& board = make_board();
    auto& can1 = board.add_can("CAN1", hcs::kClassic1M);
    auto& uart0 = board.add_uart("UART0", hcs::uart_8n1(115200));
    board.start();
    board.run();

    fake_->inject_can_frame(DataId::kCan1, 1, bytes(std::byte{1}));
    fake_->inject_can_frame(DataId::kCan1, 2, bytes(std::byte{2}));
    const std::array<std::byte, 4> bytes{};
    fake_->inject_uart_bytes(DataId::kUart0, bytes);
    board.receive();

    can1.stage(make_frame(3, std::byte{3}));
    uart0.stage(bytes);
    board.send();
    ASSERT_TRUE(eventually([&] {
        pull();
        return !sent_can_.empty() && !sent_uart_.empty();
    }));

    const auto statistics = board.statistics();
    EXPECT_EQ(statistics.receives, 1U);
    EXPECT_EQ(statistics.sends, 1U);
    EXPECT_EQ(statistics.rx_frames, 2U);
    EXPECT_EQ(statistics.rx_bytes, 4U);
    EXPECT_EQ(statistics.tx_frames, 1U);
    EXPECT_EQ(statistics.tx_bytes, 4U);
    EXPECT_EQ(statistics.rx_dropped, 0U);
    EXPECT_EQ(statistics.session_losses, 0U);
}

} // namespace
