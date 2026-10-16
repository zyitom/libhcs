#include <algorithm>
#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "firmware/hpm_board/app/src/pcan/pcan_bus.hpp"
#include "firmware/hpm_board/app/src/pcan/pcan_device_check.hpp"
#include "firmware/hpm_board/app/src/pcan/pcan_tx.hpp"
#include "firmware/hpm_board/app/src/pcan/pcan_winusb.hpp"

namespace {

namespace pcan = libhcs::firmware::pcan;
namespace protocol = pcan::protocol;
namespace data = libhcs::data;

using Timing = protocol::PhaseTiming;
using State = pcan::BusState<Timing>;

struct FakeCan {
    bool accepts = true;
    bool online = false;
    bool listen_only = false;
    bool non_iso = false;
    bool fd = false;
    unsigned configurations = 0;
    unsigned starts = 0;

    bool reconfigure_timing(bool requested_fd, Timing, Timing) {
        ++configurations;
        online = true; // 驱动失败救援也可能重新上线。
        if (accepts)
            fd = requested_fd;
        return accepts;
    }

    void set_bus_off() { online = false; }

    void set_bus_on(bool listen, bool non_iso_mode) {
        ++starts;
        online = true;
        listen_only = listen;
        non_iso = non_iso_mode;
    }
};

State fd_state() {
    return {
        .mode = pcan::ChannelMode::kNormal,
        .timing_known = true,
        .data_known = true,
        .nominal = {.prescaler = 1, .seg1 = 59, .seg2 = 20, .sjw = 10},
        .data = {.prescaler = 1, .seg1 = 11,  .seg2 = 4,  .sjw = 2}
    };
}

TEST(PcanStatus, InternalFlagsNeverBecomeChannelNumbers) {
    for (uint8_t flags = 0; flags < 8; ++flags) {
        for (uint8_t channel = 0; channel < 2; ++channel) {
            std::array<uint8_t, 16> bytes{};
            const uint8_t bits = pcan::encode_status_bits(flags);
            ASSERT_EQ(
                protocol::encode_status({.channel_bits = uint8_t(channel | bits)}, bytes), 16U);
            EXPECT_EQ(bytes[12] & 0x0F, channel);
            EXPECT_EQ(bytes[12] & 0xE0, flags << 5);
        }
    }
    EXPECT_EQ(pcan::encode_status_bits(data::kCanErrorPassive | data::kCanWarning), 0x60);
    EXPECT_EQ(pcan::encode_status_bits(0xF8), 0);
}

TEST(PcanBus, ValidFdTimingStartsTheController) {
    State state = fd_state();
    FakeCan can;
    EXPECT_TRUE(state.apply(can));
    EXPECT_TRUE(state.active());
    EXPECT_TRUE(state.fd);
    EXPECT_TRUE(can.online);
    EXPECT_TRUE(can.fd);
    EXPECT_FALSE(can.non_iso);
    EXPECT_EQ(state.status_bits(0), 0);
}

TEST(PcanBus, FailedConfigurationStopsEvenTheDriversFallback) {
    State state = fd_state();
    FakeCan can{.accepts = false};
    EXPECT_FALSE(state.apply(can));
    EXPECT_FALSE(state.active());
    EXPECT_FALSE(can.online);
    EXPECT_FALSE(state.fd);
    EXPECT_EQ(can.starts, 0U);
    EXPECT_EQ(state.status_bits(0), 0x80);

    can.accepts = true;
    EXPECT_TRUE(state.apply(can));
    EXPECT_TRUE(state.active());
    EXPECT_TRUE(can.online);
    EXPECT_TRUE(state.fd);
    EXPECT_EQ(state.status_bits(0), 0);
}

TEST(PcanBus, UnrepresentableTimingsStayOffline) {
    for (unsigned invalid = 0; invalid < 7; ++invalid) {
        State state = fd_state();
        switch (invalid) {
        case 0: state.nominal.prescaler = 513; break;
        case 1: state.nominal.seg1 = 257; break;
        case 2: state.nominal.sjw = 21; break;
        case 3: state.data.prescaler = 33; break;
        case 4: state.data.seg1 = 33; break;
        case 5: state.data.seg2 = 17; break;
        case 6: state.data.sjw = 0; break;
        }
        FakeCan can{.online = true};
        EXPECT_FALSE(state.apply(can)) << invalid;
        EXPECT_FALSE(state.active()) << invalid;
        EXPECT_FALSE(can.online) << invalid;
        EXPECT_EQ(can.configurations, 0U) << invalid;
        EXPECT_EQ(can.starts, 0U) << invalid;
        EXPECT_EQ(state.status_bits(0), 0x80) << invalid;
    }
}

TEST(PcanBus, MissingTimingCannotUseAnUnrelatedDefault) {
    State state{.mode = pcan::ChannelMode::kNormal};
    FakeCan can;
    EXPECT_FALSE(state.apply(can));
    EXPECT_FALSE(can.online);
    EXPECT_FALSE(state.active());
    EXPECT_EQ(state.status_bits(0), 0x80);
}

TEST(PcanBus, ResetClosesTheChannelAndClearsConfigurationFailure) {
    State state = fd_state();
    FakeCan can{.accepts = false};
    EXPECT_FALSE(state.apply(can));
    state.mode = pcan::ChannelMode::kReset;
    EXPECT_FALSE(state.apply(can));
    EXPECT_FALSE(state.active());
    EXPECT_FALSE(can.online);
    EXPECT_FALSE(state.configuration_failed);
    EXPECT_EQ(can.configurations, 1U);
}

TEST(PcanBus, ListenOnlyAndNonIsoFollowTheAppliedFrameMode) {
    State state = fd_state();
    state.mode = pcan::ChannelMode::kListenOnly;
    state.non_iso = true;
    FakeCan can;
    ASSERT_TRUE(state.apply(can));
    EXPECT_TRUE(can.listen_only);
    EXPECT_TRUE(can.non_iso);
    state.data_known = false;
    ASSERT_TRUE(state.apply(can));
    EXPECT_FALSE(state.fd);
    EXPECT_FALSE(can.fd);
    EXPECT_FALSE(can.non_iso);
}

TEST(PcanTiming, NominalSegmentOf256DoesNotWrapToZero) {
    const std::array<uint8_t, 6> args{96, 0, 0, 255, 0, 0};
    EXPECT_EQ(protocol::decode_timing_slow(args).timing.seg1, 256);
}

TEST(PcanCalibration, CarriesTheUsbFrameAcrossItsWrapWithTheFullTimestamp) {
    std::array<uint8_t, 16> bytes{};
    uint64_t timestamp = 0x12345678FFFF0000ULL;
    for (uint16_t frame : {2046, 2047, 0, 1}) {
        ASSERT_EQ(
            protocol::encode_calibration(
                {.timestamp_us = timestamp, .usb_frame_index = frame}, bytes),
            bytes.size());
        EXPECT_EQ(protocol::load16(bytes.data()), 16U);
        EXPECT_EQ(protocol::load16(bytes.data() + 2), protocol::kMsgCalibration);
        EXPECT_EQ(protocol::load32(bytes.data() + 4), static_cast<uint32_t>(timestamp));
        EXPECT_EQ(protocol::load32(bytes.data() + 8), static_cast<uint32_t>(timestamp >> 32));
        EXPECT_EQ(protocol::load16(bytes.data() + 12), frame);
        EXPECT_EQ(protocol::load16(bytes.data() + 14), 0U);
        timestamp += 100000;
    }
    std::array<uint8_t, 15> short_buffer{};
    EXPECT_EQ(protocol::encode_calibration({}, short_buffer), 0U);
}

TEST(PcanEcho, PreservesTheWindowsTagClientAndLongFramePayload) {
    pcan::TxEchoes<4> echoes;
    std::array<uint8_t, 64> data{};
    for (std::size_t i = 0; i < data.size(); ++i)
        data[i] = static_cast<uint8_t>(i);
    const protocol::TxFrame request{
        .tag_low = 0x12345678,
        .tag_high = 0x9ABCDEF0,
        .can_id = 0x01010101,
        .data = data,
        .channel_dlc = 0xF0,
        .client = 0x37,
        .flags = protocol::kTxFlagSelfReceive | protocol::kTxFlagExtId,
    };
    ASSERT_TRUE(echoes.requested(request));
    const auto marker = echoes.reserve(1, request);
    ASSERT_TRUE(marker);
    const auto record = echoes.completed(
        *marker, 100, protocol::kRxFlagExtDataLen | protocol::kRxFlagBitrateSwitch);
    ASSERT_TRUE(record);
    std::array<uint8_t, 92> bytes{};
    ASSERT_EQ(protocol::encode_can_rx(*record, bytes), bytes.size());
    EXPECT_EQ(protocol::load32(bytes.data() + 12), 0x12345678U);
    EXPECT_EQ(protocol::load32(bytes.data() + 16), 0x9ABCDEF0U);
    EXPECT_EQ(bytes[20], 0xF1);
    EXPECT_EQ(bytes[21], 0x37);
    EXPECT_EQ(protocol::load16(bytes.data() + 22), 0xB6);
    EXPECT_EQ(protocol::load32(bytes.data() + 24), 0x01010101U);
    EXPECT_TRUE(std::equal(data.begin(), data.end(), bytes.begin() + 28));
}

TEST(PcanEcho, FullWindowWaitsForCompletionWithoutOverwritingTags) {
    pcan::TxEchoes<2> echoes;
    protocol::TxFrame request{.tag_low = 1, .flags = protocol::kTxFlagSelfReceive};
    const auto first = echoes.reserve(0, request);
    request.tag_low = 2;
    const auto second = echoes.reserve(0, request);
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_FALSE(echoes.reserve(0, request));
    EXPECT_EQ(echoes.completed(*first, 0, 0)->tag_low, 1U);
    EXPECT_EQ(echoes.completed(*second, 0, 0)->tag_low, 2U);
    echoes.release(*second);
    request.tag_low = 3;
    const auto reused = echoes.reserve(0, request);
    ASSERT_TRUE(reused);
    EXPECT_EQ(*reused, *second);
    EXPECT_EQ(echoes.completed(*first, 0, 0)->tag_low, 1U);
}

TEST(PcanEcho, CompletionCreditsKeepAdvancingForTenThousandRequests) {
    pcan::TxEchoes<4> echoes;
    for (uint32_t batch = 0; batch < 2500; ++batch) {
        std::array<uint8_t, 4> markers{};
        for (uint32_t i = 0; i < 4; ++i) {
            const auto marker = echoes.reserve(
                0, {.tag_low = batch * 4 + i, .flags = protocol::kTxFlagSelfReceive});
            ASSERT_TRUE(marker) << batch;
            markers[i] = *marker;
        }
        EXPECT_FALSE(echoes.reserve(0, {}));
        for (uint32_t i = 0; i < 4; ++i) {
            const auto record = echoes.completed(markers[i], batch, 0);
            ASSERT_TRUE(record);
            EXPECT_EQ(record->tag_low, batch * 4 + i);
            EXPECT_NE(record->flags & protocol::kRxFlagSelfReceive, 0);
            echoes.release(markers[i]);
        }
    }
}

TEST(PcanEcho, CancellationAndResetReleasePendingRequests) {
    pcan::TxEchoes<1> echoes;
    EXPECT_FALSE(echoes.requested({}));
    EXPECT_TRUE(echoes.requested({.flags = protocol::kTxFlagLoopedBack}));
    const auto marker = echoes.reserve(0, {});
    ASSERT_TRUE(marker);
    echoes.release(*marker); // 下行入队失败, 原 USB 请求保留待重试。
    EXPECT_FALSE(echoes.completed(*marker, 0, 0));
    ASSERT_TRUE(echoes.reserve(0, {}));
    echoes.clear();
    EXPECT_FALSE(echoes.completed(*marker, 0, 0));
    EXPECT_TRUE(echoes.reserve(0, {}));
    EXPECT_FALSE(echoes.completed(255, 0, 0));
}

TEST(PcanEcho, RemoteAndClassicRecordsUseTheActualWireFormat) {
    pcan::TxEchoes<2> echoes;
    const auto remote = echoes.reserve(
        0, {.channel_dlc = 0x80, .flags = protocol::kTxFlagRtr | protocol::kTxFlagSelfReceive});
    ASSERT_TRUE(remote);
    EXPECT_TRUE(echoes.completed(*remote, 0, 0)->data.empty());
    const auto classic =
        echoes.reserve(0, {.channel_dlc = 0xF0, .flags = protocol::kTxFlagLoopedBack});
    ASSERT_TRUE(classic);
    const auto record = echoes.completed(*classic, 0, 0);
    ASSERT_TRUE(record);
    EXPECT_EQ(record->data.size(), 8U);
    EXPECT_EQ(record->flags, protocol::kRxFlagLoopedBack);
}

TEST(PcanWinUsb, MicrosoftDescriptorSubsetsAndRegistryStringsAreBounded) {
    const auto& bytes = pcan::winusb::kDescriptorSet;
    const auto word = [&](std::size_t offset) {
        return static_cast<unsigned>(bytes.at(offset))
             | static_cast<unsigned>(bytes.at(offset + 1)) << 8;
    };
    EXPECT_EQ(word(0), 10U);
    EXPECT_EQ(word(2), 0U);
    EXPECT_EQ(word(8), bytes.size());
    EXPECT_EQ(word(12), 1U);
    EXPECT_EQ(word(16), bytes.size() - 10);
    std::size_t offset = 18;
    for (unsigned interface = 0; interface < 2; ++interface) {
        ASSERT_LT(offset + 8, bytes.size());
        EXPECT_EQ(word(offset), 8U);
        EXPECT_EQ(word(offset + 2), 2U);
        EXPECT_EQ(bytes[offset + 4], interface);
        const auto end = offset + word(offset + 6);
        offset += 8;
        EXPECT_EQ(word(offset), 20U);
        EXPECT_EQ(word(offset + 2), 3U);
        EXPECT_TRUE(std::equal(bytes.begin() + offset + 4, bytes.begin() + offset + 10, "WINUSB"));
        offset += 20;
        EXPECT_EQ(word(offset + 2), 4U);
        EXPECT_EQ(word(offset + 4), 7U); // REG_MULTI_SZ
        const auto name_length = word(offset + 6);
        EXPECT_EQ(name_length, 42U);
        EXPECT_EQ(word(offset + 8 + name_length - 2), 0U);
        const auto value_length = word(offset + 8 + name_length);
        EXPECT_EQ(value_length, 80U);
        EXPECT_EQ(offset + 10 + name_length + value_length, end);
        EXPECT_EQ(word(end - 4), 0U);
        EXPECT_EQ(word(end - 2), 0U);
        EXPECT_EQ(offset + word(offset), end);
        offset = end;
    }
    EXPECT_EQ(offset, bytes.size());
}

TEST(PcanDeviceCheck, Aes128MatchesTheFips197KnownAnswer) {
    const pcan::device_check::Block key = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                           0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
    const pcan::device_check::Block input = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                                             0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    const pcan::device_check::Block expected = {0x69, 0xC4, 0xE0, 0xD8, 0x6A, 0x7B, 0x04, 0x30,
                                                0xD8, 0xCD, 0xB7, 0x80, 0x70, 0xB4, 0xC5, 0x5A};
    EXPECT_EQ(pcan::device_check::encrypt(input, key), expected);
}

TEST(PcanDeviceCheck, CommandReplyMatchesAnIndependentlyCheckedDriverVector) {
    pcan::device_check::Exchange exchange;
    std::array<uint8_t, 24> challenge{};
    protocol::store16(challenge.data(), 0xA0);
    protocol::store32(challenge.data() + 2, 16);
    protocol::store16(challenge.data() + 6, 1);
    for (std::size_t i = 0; i < 16; ++i)
        challenge[i + 8] = static_cast<uint8_t>(i);
    std::array<uint8_t, 8> request{};
    protocol::store16(request.data(), 0xD0);
    protocol::store16(request.data() + 6, 1);
    ASSERT_TRUE(exchange.receive(challenge));
    ASSERT_TRUE(exchange.receive(request));
    const std::array<uint8_t, 72> expected = {0xD1, 0x00, 0x11, 0x00, 0x00, 0x00, 0x01, 0x00,
                                              0x00, 0x22, 0x0C, 0x17, 0xEC, 0xE6, 0xF3, 0x57,
                                              0x49, 0x8A, 0x67, 0xC3, 0x20, 0x89, 0x20, 0x8A,
                                              0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    EXPECT_EQ(exchange.reply(), expected);
}

TEST(PcanDeviceCheck, InitializationRecordsNeverBecomeCanCommandsOrStaleReplies) {
    pcan::device_check::Exchange exchange;
    std::array<uint8_t, 512> challenge{};
    challenge.fill(0xFF);
    protocol::store16(challenge.data(), 0xA0);
    protocol::store32(challenge.data() + 2, 16);
    protocol::store16(challenge.data() + 6, 1);
    std::array<uint8_t, 8> request{};
    protocol::store16(request.data(), 0xD0);
    protocol::store16(request.data() + 6, 1);
    EXPECT_TRUE(exchange.receive(request));
    EXPECT_FALSE(exchange.pending());
    EXPECT_TRUE(exchange.receive(std::span{challenge}.first(23)));
    EXPECT_TRUE(exchange.receive(request));
    EXPECT_FALSE(exchange.pending());
    ASSERT_TRUE(exchange.receive(challenge));
    EXPECT_FALSE(exchange.pending());
    ASSERT_TRUE(exchange.receive(request));
    ASSERT_TRUE(exchange.pending());
    const auto first = exchange.reply();
    EXPECT_EQ(protocol::load16(first.data()), 0xD1);
    EXPECT_EQ(first.size(), 72U);
    EXPECT_EQ(protocol::load32(first.data() + 2), 17U);
    EXPECT_EQ(protocol::load16(first.data() + 6), 1);
    EXPECT_EQ(protocol::load16(first.data() + 2), 0x11);
    EXPECT_EQ(first[8], 0);
    exchange.completed();
    EXPECT_FALSE(exchange.pending());
    challenge[8] ^= 1;
    ASSERT_TRUE(exchange.receive(challenge));
    ASSERT_TRUE(exchange.receive(request));
    EXPECT_NE(exchange.reply(), first);
    exchange.reset();
    EXPECT_TRUE(exchange.receive(request));
    EXPECT_FALSE(exchange.pending());
    request[0] = 1; // Normal CAN reset remains on the ordinary command path.
    EXPECT_FALSE(exchange.receive(request));
}

} // namespace
