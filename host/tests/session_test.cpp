// core::link::Session 的单元测试: kStart 的 nonce 握手与 keepalive 租约这一段
// 协议逻辑, 固件里跑的就是这一份(三块板的 Vendor / HostSession 经它裁决)。状态机
// 不碰外设, 时间由测试直接给 -- 包括 32 位 1/4 us 计数在 2^32 附近的回绕。
//
// 同文件再测 core::link::DownlinkErrors: 下行流错误账本, 固件在 error_callback()
// 里记账, 它的 status() 是链路这个"口"(DataId::kSession)的运行时状态。三块板共用这一份。
//
// 以及 core::link::PortStatusLedger / PortStatusRound: 端口运行时状态"变了才发"的账本,
// keepalive 应答后把状态变了的口(含链路本身)各写一条 kPortStatus。

#include <array>
#include <cstddef>
#include <span>

#include <gtest/gtest.h>

#include <core/include/libhcs/data/datas.hpp>
#include <core/src/link/downlink_errors.hpp>
#include <core/src/link/port_status.hpp>
#include <core/src/protocol/serializer.hpp>
#include <core/src/link/session.hpp>

namespace {

using libhcs::core::link::DownlinkErrors;
using libhcs::core::link::Session;
using libhcs::data::DataId;
using libhcs::data::DownlinkError;
namespace data = libhcs::data;

constexpr uint32_t kLease = Session::kLeaseTicks;

// 握手门没开: kStart 被静默拒绝, 什么都不发生。
TEST(Session, StartIsRefusedWhileGateIsClosed) {
    Session session;
    EXPECT_FALSE(session.established());
    EXPECT_EQ(session.on_start(7, false, 100), Session::Start::kRefused);
    EXPECT_FALSE(session.established());

    // 门开着开了会话之后又关上(门是每次 kStart 传入的板级状态): 新的 kStart 被
    // 拒, 已建立的会话不受影响, 照常由租约守着。
    EXPECT_EQ(session.on_start(7, true, 200), Session::Start::kOpened);
    EXPECT_TRUE(session.on_keepalive(7, 300));
    EXPECT_EQ(session.on_start(8, false, 400), Session::Start::kRefused);
    EXPECT_TRUE(session.established());
    EXPECT_FALSE(session.poll(300 + kLease - 1));
    EXPECT_TRUE(session.poll(300 + kLease));
    EXPECT_FALSE(session.established());
}

// 首次 kStart 开新会话; 同 nonce 的 kStart 只刷新不重开。
TEST(Session, SameNonceStartRenewsInsteadOfReopening) {
    Session session;
    EXPECT_EQ(session.on_start(42, true, 0), Session::Start::kOpened);
    EXPECT_TRUE(session.established());
    EXPECT_EQ(session.nonce(), 42U);

    // 快到租约尽头时主机重发 kStart: 不算新会话(板子不清批量), 租约从新算。
    EXPECT_EQ(session.on_start(42, true, kLease - 1), Session::Start::kRenewed);
    EXPECT_TRUE(session.established());
    EXPECT_FALSE(session.poll(2 * kLease - 2));
    EXPECT_TRUE(session.established());
    EXPECT_TRUE(session.poll(2 * kLease - 1));
    EXPECT_FALSE(session.established());
}

// 会话在时来了不同 nonce: 是新会话(板子要清批量与旧主机的输出)。
TEST(Session, DifferentNonceReopensTheSession) {
    Session session;
    EXPECT_EQ(session.on_start(1, true, 0), Session::Start::kOpened);
    EXPECT_EQ(session.on_start(2, true, 500), Session::Start::kOpened);
    EXPECT_EQ(session.nonce(), 2U);
    // 旧 nonce 的 keepalive 不再被认。
    EXPECT_FALSE(session.on_keepalive(1, 600));
    EXPECT_TRUE(session.on_keepalive(2, 600));
}

// 错 nonce 或无会话的 keepalive 不应答、不刷新租约。
TEST(Session, KeepaliveIsRefusedWithoutEstablishedSessionOrMatchingNonce) {
    Session session;
    EXPECT_FALSE(session.on_keepalive(9, 0));

    EXPECT_EQ(session.on_start(9, true, 0), Session::Start::kOpened);
    EXPECT_FALSE(session.on_keepalive(8, 100));
    // 被拒的 keepalive 没刷新租约: 从开算起到期满即失效。
    EXPECT_FALSE(session.poll(kLease - 1));
    EXPECT_TRUE(session.poll(kLease));
}

// 租约到期即失效; poll 只在到期的那个时刻报一次。
TEST(Session, LeaseExpiryFiresOnceThenStaysDown) {
    Session session;
    EXPECT_EQ(session.on_start(5, true, 0), Session::Start::kOpened);
    EXPECT_FALSE(session.poll(1));
    EXPECT_FALSE(session.poll(kLease - 1));
    EXPECT_TRUE(session.poll(kLease));
    EXPECT_FALSE(session.established());
    // 失效后不再重复报; keepalive 不认。
    EXPECT_FALSE(session.poll(kLease + 1));
    EXPECT_FALSE(session.poll(kLease * 10));
    EXPECT_FALSE(session.on_keepalive(5, kLease * 10));
    // end() 之后同 nonce 的 kStart 也是新会话。
    EXPECT_EQ(session.on_start(5, true, kLease * 10 + 1), Session::Start::kOpened);
}

// 端()显式结束(总线复位/挂起/拔线路径)后, keepalive 不应答。
TEST(Session, EndKillsTheSessionImmediately) {
    Session session;
    EXPECT_EQ(session.on_start(3, true, 0), Session::Start::kOpened);
    session.end();
    EXPECT_FALSE(session.established());
    EXPECT_FALSE(session.on_keepalive(3, 10));
    EXPECT_FALSE(session.poll(kLease));
}

// 时间回绕: 1/4 us 的 uint32_t 约 1073 s 一圈, 会话横跨回绕点时无符号差照常比较。
TEST(Session, LeaseJudgementIsCorrectAcrossTheWraparound) {
    Session session;
    const uint32_t opened_at = 0xFFFF'FFF0U; // 距回绕 16 tick
    EXPECT_EQ(session.on_start(11, true, opened_at), Session::Start::kOpened);

    // 回绕之后再走 (kLease - 16) 个 tick 才到期; 差一个 tick 都不算到期。
    EXPECT_FALSE(session.poll(opened_at + kLease - 1));
    EXPECT_TRUE(session.established());
    EXPECT_TRUE(session.poll(opened_at + kLease));
    EXPECT_FALSE(session.established());
}

// 回绕点上重开与刷新同样成立。
TEST(Session, RenewalWorksAcrossTheWraparound) {
    Session session;
    EXPECT_EQ(session.on_start(11, true, 0xFFFF'FFFCU), Session::Start::kOpened);
    // 4 tick 后(已回绕到 0x0000'0000)keepalive 刷新。
    EXPECT_TRUE(session.on_keepalive(11, 0x0000'0000U));
    // 从刷新点再撑满一个租约才到期。
    EXPECT_FALSE(session.poll(kLease - 1));
    EXPECT_TRUE(session.poll(kLease));
}

// ---- DownlinkErrors: 链路这个"口"的状态来源 ----

// 一条没能交付的下行记录: 计数加一, 记下字段号与原因。
TEST(DownlinkErrors, AnErrorMovesTheLinkStatus) {
    DownlinkErrors errors;
    EXPECT_EQ(errors.status(), data::LinkStatusView{});

    errors.note(DownlinkError::kRefused, DataId::kCan3);
    EXPECT_EQ(errors.status().downlink_errors, 1U);
    EXPECT_EQ(errors.status().last_field, DataId::kCan3);
    EXPECT_EQ(errors.status().last_reason, DownlinkError::kRefused);

    errors.note(DownlinkError::kMalformed, DataId::kCan1);
    EXPECT_EQ(errors.status().downlink_errors, 2U);
    EXPECT_EQ(errors.status().last_reason, DownlinkError::kMalformed);
}

// 错误记在它发生的传输上: 传输结束(end_transfer)之前进来的错误算这一次的。
TEST(DownlinkErrors, CountsTheTransferTheErrorHappenedIn) {
    DownlinkErrors errors;
    errors.note(DownlinkError::kRefused, DataId::kCan3);        // 第 1 次传输内
    errors.end_transfer();
    errors.end_transfer();                                      // 第 2 次传输, 无错
    errors.note(DownlinkError::kUnknownField, DataId::kExtend); // 第 3 次传输内
    errors.end_transfer();

    EXPECT_EQ(errors.status().downlink_errors, 2U);
    EXPECT_EQ(errors.status().last_transfer, 3U);
    EXPECT_EQ(errors.status().last_reason, DownlinkError::kUnknownField);
}

// 新会话: 传输从头编号; 错误计数是自由计数, 不清(清了主机取的差会跳)。
TEST(DownlinkErrors, ANewSessionRenumbersTransfersButKeepsCounting) {
    DownlinkErrors errors;
    errors.note(DownlinkError::kRefused, DataId::kCan1);
    errors.end_transfer();
    errors.end_transfer();

    errors.begin_session();
    errors.note(DownlinkError::kRefused, DataId::kCan2);
    EXPECT_EQ(errors.status().downlink_errors, 2U);
    EXPECT_EQ(errors.status().last_transfer, 1U);
    EXPECT_EQ(errors.status().last_field, DataId::kCan2);
}

// ---- 端口运行时状态 ----

using libhcs::core::link::PortStatusLedger;
using libhcs::core::link::PortStatusRound;
using libhcs::core::protocol::SerializeBuffer;
using libhcs::core::protocol::Serializer;

// 每个口第一次都发(基线, 干净的也发); 之后状态变了发一次, 不变不再发。账本只比较,
// 不认识任何一种状态的特例(CAN 错误码的锁存在驱动里)。
TEST(PortStatusLedger, ReportsOnlyWhatChanged) {
    PortStatusLedger ledger;
    EXPECT_TRUE(ledger.changed(DataId::kCan1, data::CanStatusView{}))
        << "the first offer is the baseline";
    ledger.commit(DataId::kCan1, data::CanStatusView{});
    EXPECT_FALSE(ledger.changed(DataId::kCan1, data::CanStatusView{}));

    const data::CanStatusView passive{.tec = 130, .flags = data::kCanErrorPassive};
    EXPECT_TRUE(ledger.changed(DataId::kCan1, passive));
    ledger.commit(DataId::kCan1, passive);
    EXPECT_FALSE(ledger.changed(DataId::kCan1, passive));

    // 另一个口、另一种状态各记各的; 链路本身也是一个口。
    EXPECT_TRUE(ledger.changed(DataId::kCan2, passive));
    EXPECT_TRUE(ledger.changed(DataId::kUart0, data::UartStatusView{.overrun = 1}));
    EXPECT_TRUE(ledger.changed(DataId::kSession, data::LinkStatusView{}));
}

// 新会话(kStart): 每个口下一次都发一条作基线, 干净的也发 -- 主机从不清快照, 靠它覆盖;
// 发过之后回到"变了才发"。
TEST(PortStatusLedger, ResetMakesEveryPortTheNextBaseline) {
    PortStatusLedger ledger;
    const data::UartStatusView dirty{.framing = 3};
    ledger.commit(DataId::kUart0, dirty);
    EXPECT_FALSE(ledger.changed(DataId::kUart0, dirty));
    ledger.reset();
    EXPECT_TRUE(ledger.changed(DataId::kUart0, dirty));
    EXPECT_TRUE(ledger.changed(DataId::kCan1, data::CanStatusView{}))
        << "a clean port is part of the baseline too";
    ledger.commit(DataId::kCan1, data::CanStatusView{});
    EXPECT_FALSE(ledger.changed(DataId::kCan1, data::CanStatusView{}));
}

// 上行缓冲一次只给 capacity 字节的序列化缓冲。
class FixedBuffer final : public SerializeBuffer {
public:
    explicit FixedBuffer(std::size_t capacity) noexcept
        : capacity_(capacity) {}
    std::span<std::byte> allocate(std::size_t size) noexcept override {
        if (used_ + size > capacity_)
            return {};
        const std::span<std::byte> out{storage_.data() + used_, size};
        used_ += size;
        return out;
    }
    [[nodiscard]] std::size_t used() const noexcept { return used_; }
    void drain() noexcept { used_ = 0; }

private:
    std::array<std::byte, 256> storage_{};
    std::size_t capacity_;
    std::size_t used_ = 0;
};

// 上行满写不进去就不记账: 下一轮同样的变化再发一次, 不会因为一次丢弃而永远不报。
TEST(PortStatusRound, AStatusThatDidNotFitIsOfferedAgain) {
    PortStatusLedger ledger;
    FixedBuffer buffer{10}; // 装不下一条 17 字节的 CAN 状态
    Serializer serializer{buffer};
    const data::CanStatusView bus_off{.tec = 255, .flags = data::kCanBusOff};

    PortStatusRound{ledger, serializer, 1}.offer(DataId::kCan1, bus_off);
    EXPECT_EQ(buffer.used(), 0U);
    EXPECT_TRUE(ledger.changed(DataId::kCan1, bus_off)) << "not recorded as sent";

    FixedBuffer roomy{256};
    Serializer next{roomy};
    PortStatusRound{ledger, next, 1}.offer(DataId::kCan1, bus_off);
    EXPECT_EQ(roomy.used(), 5U + 1U + 11U);
    EXPECT_FALSE(ledger.changed(DataId::kCan1, bus_off));
    PortStatusRound{ledger, next, 1}.offer(DataId::kCan1, bus_off);
    EXPECT_EQ(roomy.used(), 5U + 1U + 11U) << "unchanged: nothing more written";
}

// 链路与其他口走同一条路: 先发一条基线, 之后只有新的下行错误才再发。
TEST(PortStatusRound, TheLinkReportsOnlyNewDownlinkErrors) {
    PortStatusLedger ledger;
    DownlinkErrors errors;
    FixedBuffer wire{256};
    Serializer serializer{wire};
    const auto round = [&] {
        PortStatusRound{ledger, serializer, 1}.offer(DataId::kSession, errors.status());
    };
    constexpr std::size_t kRecord = 5U + 1U + 6U; // 会话头 + 口号/长度 + 链路正文

    round();
    EXPECT_EQ(wire.used(), kRecord) << "the baseline";
    round();
    EXPECT_EQ(wire.used(), kRecord) << "no new error, nothing more";
    errors.note(DownlinkError::kRefused, DataId::kCan3);
    round();
    EXPECT_EQ(wire.used(), 2 * kRecord);
}

} // namespace
