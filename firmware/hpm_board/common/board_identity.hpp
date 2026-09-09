#pragma once

#include <cstdint>

#include <hpm_otp_drv.h>

namespace libhcs::firmware::board {

// Which physical PCB this chip is soldered to, read at run time out of the
// factory-programmed OTP shadow rather than baked in at compile time.
//
// OPT-IN, PER BOARD
//
// Only the hpm5321 board enables this, by defining libhcs_BOARD_OTP_IDENTITY=1
// in its CMakeLists.txt, because it is the only board whose directory serves two
// electrically different PCBs. Every other board keeps a single-variant identity
// that is recognized unconditionally and reports its compile-time PID, so the
// shared bootloader and app code below can be written without #if at the call
// sites.
//
// This gate is not cosmetic. The refusal path keys on two specific values of word
// 25, and those values are hpm5321 observations: boards/hpm6e8y/README.md records
// word 25 = 0x00000006 on that part. An ungated check would therefore classify
// every hpm6e8y and hpm6e80ivm1 as unrecognized and refuse to boot hardware that
// has nothing to do with the ambiguity this solves.
//
// WHY THIS EXISTS
//
// The two HPM5321 PCBs in this project are electrically incompatible on two
// pads: PA30/PA31 are the green/red LED cathodes on the single-CAN board and
// MCAN3 RXD/TXD on the dual-CAN-FD board. A single firmware image therefore
// cannot configure those pads until it knows which board it is on, and getting
// it wrong in the dual->single direction drives a transceiver output against an
// LED network. So the image has to ask the hardware, and it has to refuse to
// guess.
//
// WHAT THE EVIDENCE ACTUALLY SUPPORTS  [实测 2026-08-05, 4 chips]
//
// OTP shadow word 25 reads 0 on both single-CAN boards sampled and 2 on both
// dual-CAN-FD boards sampled. Everything else that differs between the four
// dumps (word 5 lot number, word 21 TSNS trim, words 88-91 UUID) is per-die.
//
// This is NOT proof that word 25 encodes the board type. The four samples are
// batch-ordered -- lots 375/377 are the dual boards, 378/380 the single ones --
// so "board variant" and "production batch" are perfectly confounded in the
// sample, and word 25 is undefined in the SDK's hpm_otp_table.h (the HPM6E8Y
// board notes call the same word a program-count/config flag). Distinguishing
// the two would need a counterexample from an interleaved batch: a single board
// from lot 375-377, or a dual board from lot >=378.
//
// The strictness below is what makes that acceptable: only the two observed
// values are ever accepted, and anything else stops the boot instead of picking
// a default. A batch artifact would then show up as a refusal to run, which is
// loud and inspectable, rather than as a misconfigured pad.
//
// COST AND SAFETY OF THE READ
//
// One MMIO load per boot. otp_read_from_shadow() bounds-checks the index and
// reads HPM_OTP->SHADOW[index]; otp_init()/otp_deinit() are empty in the SDK,
// there is no clock gate and no command sequence on the read path. Programming
// OTP is an entirely separate mechanism (2.5 V LDO enable, the UNLOCK magic, and
// writes to FUSE[]), none of which this touches -- so reads cannot wear or
// damage the array. Both halves of this firmware already read four OTP shadow
// words on every boot for the UUID-derived USB serial number; this adds a fifth.
enum class BoardVariant : uint8_t {
    kSingleCan, // one classic CAN, RGB LED on PA29/PA30/PA31, no CAN indicators
    kDualCanFd, // two CAN-FD (MCAN0 + MCAN3 on PA30/PA31), LEDs on PA26/PA27/PA28
    kFixed,     // board with only one variant: nothing to discriminate
    kUnknown,   // word 25 held neither known value -- refuse to run
};

// Whether this board resolves its variant from OTP at all. False on every
// single-variant board, where the identity is kFixed and always recognized.
inline constexpr bool kOtpIdentityEnabled =
#if defined(libhcs_BOARD_OTP_IDENTITY) && libhcs_BOARD_OTP_IDENTITY
    true;
#else
    false;
#endif

// OTP shadow word carrying the discriminator, and the only two values this
// firmware is willing to act on.
inline constexpr uint32_t kVariantOtpIndex = 25U;
inline constexpr uint32_t kVariantOtpValueSingleCan = 0U;
inline constexpr uint32_t kVariantOtpValueDualCanFd = 2U;

struct BoardIdentity {
    BoardVariant variant = BoardVariant::kFixed;

    // The word as it was actually read, kept so a refusal can report the value
    // that caused it instead of just "unknown". This is the number to quote when
    // investigating a board that will not boot.
    uint32_t otp_word = 0U;

    [[nodiscard]] constexpr bool recognized() const { return variant != BoardVariant::kUnknown; }

    [[nodiscard]] constexpr bool dual_can() const { return variant == BoardVariant::kDualCanFd; }
};

inline BoardIdentity read_board_identity() {
    BoardIdentity identity;

    // Single-variant boards do not read OTP at all: there is nothing to
    // discriminate, and the two accepted values below are hpm5321 numbers that
    // would reject those parts outright.
    if constexpr (!kOtpIdentityEnabled) {
        identity.variant = BoardVariant::kFixed;
        return identity;
    }

    identity.otp_word = otp_read_from_shadow(kVariantOtpIndex);

    if (identity.otp_word == kVariantOtpValueSingleCan) {
        identity.variant = BoardVariant::kSingleCan;
    } else if (identity.otp_word == kVariantOtpValueDualCanFd) {
        identity.variant = BoardVariant::kDualCanFd;
    } else {
        identity.variant = BoardVariant::kUnknown;
    }

    return identity;
}

// Cached across the whole image: OTP cannot change while the part is powered, so
// one read per boot is enough and every caller sees the same answer. Callers on
// the pad-configuration path (board_app.cpp) rely on that -- two reads that
// disagreed would configure MCAN3 and the LEDs against each other.
inline const BoardIdentity& board_identity() {
    static const BoardIdentity kIdentity = read_board_identity();
    return kIdentity;
}

} // namespace libhcs::firmware::board
