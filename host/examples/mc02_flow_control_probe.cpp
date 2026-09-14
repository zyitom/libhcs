// mc02 下行流控探针: 以最快速度向 CAN1 灌固定数量的 classic 帧, 测量 host 侧
// 写循环的墙钟时间。
//
// 原理: 总线无对端时排空率≈0, 软件 TX 队列必然涨满。
//   CFG_TUD_VENDOR_RX_MANUAL_XFER=1(流控开): 队列到 3/4 水位后板子对 OUT 包回
//     NAK, host 的 64 池 URB 耗尽、写调用阻塞, 20ms 逃生阀周期性放行 -- 墙钟
//     时间被显著拉长, 且 100 帧分片耗时应呈"长-短-长"的节流纹波。
//   =0(流控关): 写永不像背压那样阻塞, 帧在 CAN 层无声丢弃 -- 墙钟时间短且平。
//
// 用法: mc02_flow_control_probe [串口过滤] [帧数, 默认 3000]

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <string_view>

#include <libhcs/board/mc02.hpp>

namespace {

using libhcs::board::AdvancedOptions;
using libhcs::board::Mc02;

} // namespace

int main(int argc, char** argv) {
    const std::string_view filter = argc > 1 ? argv[1] : std::string_view{};
    const uint32_t frames = argc > 2 ? static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10))
                                     : 3000U;

    Mc02::Callback callback;
    try {
        AdvancedOptions options;
        options.set_dangerously_skip_version_checks(true);
        Mc02 board{callback, filter, options};

        std::printf(
            "流控探针: 向 CAN1 灌 %u 帧 classic(8B), CAN1 当前 %s, 无对端 = 排空率 0\n", frames,
            board.can1_is_fd() ? "FD" : "classic");

        const auto start = std::chrono::steady_clock::now();
        uint32_t slice_frames = 0;
        auto slice_start = std::chrono::steady_clock::now();

        for (uint32_t sequence = 0; sequence < frames; ++sequence) {
            std::array<std::byte, 8> payload{};
            for (size_t i = 0; i < payload.size(); ++i)
                payload[i] = static_cast<std::byte>((sequence + i) & 0xFFU);

            auto builder = board.start_transmit();
            builder.can1_transmit({.can_id = 0x321,
                                   .can_data = {payload.data(), payload.size()},
                                   .is_extended_can_id = false});
            ++slice_frames;

            if (slice_frames == 100) {
                const auto now = std::chrono::steady_clock::now();
                const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    now - slice_start)
                                    .count();
                std::printf("  frames %5u-%5u: %6lld ms\n", sequence - 99, sequence, ms);
                slice_start = now;
                slice_frames = 0;
            }
        }

        const auto total = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - start)
                               .count();
        std::printf("合计: %u 帧耗时 %lld ms (host 侧有效速率 %llu 帧/s)\n", frames, total,
                    total > 0 ? static_cast<unsigned long long>(frames) * 1000ULL
                                    / static_cast<unsigned long long>(total)
                              : 0ULL);
        return 0;
    } catch (const std::exception& error) {
        std::printf("FAIL: %s\n", error.what());
        return 1;
    }
}
