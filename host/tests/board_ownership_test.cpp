// Who a board serves: libhcs, or its built-in extras (the DMTool adapter and the
// CDC serial bridge on the HPM5321).
//
// The two never share the board: one side has it, the other is shut out
// completely. A libhcs manifest is the only way in, and it is a transaction --
// once validated, the board is taken from the extras (begin_claim) before any
// port is touched; a fully applied manifest keeps it (commit_claim); one that
// fails half-way gives it back to whoever had it (abort_claim). The state
// machine that enforces this (core/src/link/ownership.hpp) knows no peripheral:
// the firmware hands it a list of steps, and what matters is which steps run,
// in which order, and when. Here the steps only write to a journal, so the
// order is checked exactly; on the board the same steps stop the DMTool
// adapter, close its endpoints and suspend every port.

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "core/src/link/ownership.hpp"

namespace {

using libhcs::core::link::Owner;

std::vector<std::string>& journal() {
    static std::vector<std::string> entries;
    return entries;
}

template <char name>
struct Step {
    static void to_libhcs() { journal().push_back(std::string{name} + "->libhcs"); }
    static void to_tools() { journal().push_back(std::string{name} + "->tools"); }
};

constexpr std::uint64_t kTimeout = 4000;

using Ownership = libhcs::core::link::Ownership<Step<'a'>, Step<'b'>, Step<'c'>>;

class OwnershipTest : public testing::Test {
protected:
    void SetUp() override { journal().clear(); }

    // A clock that counts how often it is read.
    auto clock(std::uint64_t now) {
        return [this, now] {
            ++clock_reads_;
            return now;
        };
    }

    // One accepted manifest: the transaction's happy path.
    void accept(std::uint64_t now) {
        ownership_.begin_claim();
        ownership_.commit_claim(now);
    }

    Ownership ownership_{kTimeout};
    int clock_reads_ = 0;
};

const std::vector<std::string> kHandOver{"a->libhcs", "b->libhcs", "c->libhcs"};
const std::vector<std::string> kHandBack{"c->tools", "b->tools", "a->tools"};

TEST_F(OwnershipTest, APoweredUpBoardServesItsExtrasAndRefusesSessions) {
    EXPECT_EQ(ownership_.owner(), Owner::kTools);
    EXPECT_FALSE(ownership_.session_allowed());
    EXPECT_TRUE(journal().empty());
}

TEST_F(OwnershipTest, AnAcceptedManifestHandsTheBoardToLibhcsInListedOrder) {
    accept(100);

    EXPECT_EQ(ownership_.owner(), Owner::kLibhcs);
    EXPECT_TRUE(ownership_.session_allowed());
    EXPECT_EQ(journal(), kHandOver);
}

// Last taken over, first given back -- the same rule as construction and
// destruction. On the board: channels resume before the extras restart, so the
// extras never see a suspended bus.
TEST_F(OwnershipTest, ReleaseHandsTheBoardBackInReverseOrder) {
    accept(100);
    journal().clear();

    ownership_.release();

    EXPECT_EQ(ownership_.owner(), Owner::kTools);
    EXPECT_FALSE(ownership_.session_allowed());
    EXPECT_EQ(journal(), kHandBack);
}

TEST_F(OwnershipTest, ReleasingABoardTheExtrasAlreadyOwnDoesNothing) {
    ownership_.release();
    EXPECT_TRUE(journal().empty());

    accept(100);
    ownership_.release();
    journal().clear();
    ownership_.release();
    EXPECT_TRUE(journal().empty());
}

// A redeclaration while libhcs already owns the board is a new declaration
// round for the ports, not a new hand-over: the extras are already shut out, and
// suspending every port again would bounce buses the host keeps using.
TEST_F(OwnershipTest, AManifestWhileLibhcsOwnsTheBoardDoesNotHandItOverAgain) {
    accept(100);
    accept(200);

    EXPECT_EQ(journal(), kHandOver);
    EXPECT_EQ(ownership_.owner(), Owner::kLibhcs);
}

// The extras are shut out before the manifest touches a port, so they never
// see a half-applied declaration.
TEST_F(OwnershipTest, TheBoardIsTakenFromTheExtrasBeforeTheManifestApplies) {
    ownership_.begin_claim();

    EXPECT_EQ(journal(), kHandOver);
    EXPECT_EQ(ownership_.owner(), Owner::kLibhcs);
    EXPECT_FALSE(ownership_.session_allowed()) << "nothing is declared yet";
}

// A manifest that fails half-way gives the board back to the extras it was
// taken from: their ports resume, the bridge comes back, nothing of libhcs's
// half-state is left behind.
TEST_F(OwnershipTest, AManifestFailingHalfWayGivesTheBoardBackToTheExtras) {
    ownership_.begin_claim();
    journal().clear();

    ownership_.abort_claim();

    EXPECT_EQ(ownership_.owner(), Owner::kTools);
    EXPECT_FALSE(ownership_.session_allowed());
    EXPECT_EQ(journal(), kHandBack);
}

// ...but a board libhcs already had stays with libhcs: its ports are off until
// the host declares again, and the claim timeout or the session lease hands it
// back as usual.
TEST_F(OwnershipTest, AManifestFailingWhileLibhcsOwnsTheBoardKeepsItWithLibhcs) {
    accept(1000);
    ownership_.on_session_started();
    journal().clear();

    ownership_.begin_claim();
    ownership_.abort_claim();

    EXPECT_EQ(ownership_.owner(), Owner::kLibhcs);
    EXPECT_TRUE(ownership_.session_allowed());
    EXPECT_TRUE(journal().empty());
}

TEST_F(OwnershipTest, AFailedManifestLeavesTheClaimTimeoutRunning) {
    accept(1000);
    ownership_.begin_claim();
    ownership_.abort_claim();
    journal().clear();

    ownership_.poll(clock(1000 + kTimeout));
    EXPECT_EQ(ownership_.owner(), Owner::kTools);
    EXPECT_EQ(journal(), kHandBack);
}

// A host whose manifest was accepted and then leaves -- or dies before kStart --
// must not keep the board from its extras forever.
TEST_F(OwnershipTest, AManifestWithoutASessionLapsesAfterTheTimeout) {
    accept(1000);
    journal().clear();

    ownership_.poll(clock(1000 + kTimeout - 1));
    EXPECT_EQ(ownership_.owner(), Owner::kLibhcs);
    EXPECT_TRUE(journal().empty());

    ownership_.poll(clock(1000 + kTimeout));
    EXPECT_EQ(ownership_.owner(), Owner::kTools);
    EXPECT_EQ(journal(), kHandBack);
}

TEST_F(OwnershipTest, ARepeatedManifestRestartsTheTimeout) {
    accept(1000);
    accept(3000);

    ownership_.poll(clock(1000 + kTimeout));
    EXPECT_EQ(ownership_.owner(), Owner::kLibhcs);

    ownership_.poll(clock(3000 + kTimeout));
    EXPECT_EQ(ownership_.owner(), Owner::kTools);
}

// Once a session runs, its lease guards the board; the handshake timeout no
// longer applies, however long the session lasts.
TEST_F(OwnershipTest, ASessionStopsTheHandshakeTimeout) {
    accept(1000);
    ownership_.on_session_started();

    ownership_.poll(clock(1000 + (100 * kTimeout)));
    EXPECT_EQ(ownership_.owner(), Owner::kLibhcs);
    EXPECT_EQ(clock_reads_, 0);
}

// A host whose keepalive failed re-runs its manifest while the board may still
// hold its old session. The board stays guarded by that session's lease, not
// by a fresh claim timeout.
TEST_F(OwnershipTest, AManifestDuringASessionKeepsTheSessionInCharge) {
    accept(1000);
    ownership_.on_session_started();
    accept(2000);

    ownership_.poll(clock(2000 + (10 * kTimeout)));
    EXPECT_EQ(ownership_.owner(), Owner::kLibhcs);
}

TEST_F(OwnershipTest, TheSessionEndingGivesTheBoardBack) {
    accept(1000);
    ownership_.on_session_started();
    journal().clear();

    ownership_.release();

    EXPECT_EQ(ownership_.owner(), Owner::kTools);
    EXPECT_EQ(journal(), kHandBack);
}

// The firmware gates kStart on session_allowed(), so this cannot happen there;
// if it ever did, it must not give libhcs a board no manifest declared.
TEST_F(OwnershipTest, ASessionWithoutAManifestDoesNotTakeTheBoard) {
    ownership_.on_session_started();

    EXPECT_EQ(ownership_.owner(), Owner::kTools);
    EXPECT_TRUE(journal().empty());
}

// Reading the time is a few peripheral register reads on the board, and poll()
// runs every millisecond: only a handshake that is waiting for its session pays
// for it.
TEST_F(OwnershipTest, PollReadsTheClockOnlyWhileAManifestIsWaiting) {
    ownership_.poll(clock(0));
    EXPECT_EQ(clock_reads_, 0);

    accept(0);
    ownership_.poll(clock(1));
    EXPECT_EQ(clock_reads_, 1);

    ownership_.on_session_started();
    ownership_.poll(clock(2));
    EXPECT_EQ(clock_reads_, 1);
}

// The board's clock is a 64-bit counter of quarter microseconds, so it does not
// wrap in practice -- but the comparison is a difference, so it would survive.
TEST_F(OwnershipTest, TheTimeoutIsMeasuredAcrossAClockWrap) {
    constexpr std::uint64_t near_wrap = ~std::uint64_t{0} - 10;
    accept(near_wrap);

    ownership_.poll(clock(near_wrap + kTimeout - 1));
    EXPECT_EQ(ownership_.owner(), Owner::kLibhcs);

    ownership_.poll(clock(near_wrap + kTimeout));
    EXPECT_EQ(ownership_.owner(), Owner::kTools);
}

} // namespace
