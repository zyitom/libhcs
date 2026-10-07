#pragma once

#include <cstdint>
#include <optional>

// MCAN 位时序的纯求解: 不碰寄存器, 不含芯片头, 编译期可求值。
//
// 为什么有这一份: EP0 清单事务要求校验阶段判完一切可预见的拒绝(core/src/link/
// port_ops.hpp 的 RateSettableCanDriver), 而 HPM SDK 的求解器
// (hpm_mcan_drv.c 的 mcan_calc_bit_timing_from_baudrate)是 static 的, 只能经 mcan_init()
// 边解边写。这里逐步照搬它(HPM SDK v1.12.0)的算法与限值表, 让"这个速率凑不凑得出"
// 在写硬件之前就有答案。照搬而不是另写一个"更好的"求解: 判定必须与真正写进去的那次
// 求解一致, 否则校验放行的设置会在应用阶段被拒(整份清单回滚), 或者反过来误拒。若两者
// 将来漂移, 应用阶段的回读核对(Can::init_controller)仍会拦住, 代价是一次回滚而不是
// 错误的时序上总线。升级 SDK 时对照该函数复核本文件。
namespace libhcs::firmware::can::bit_timing {

// hpm_mcan_drv.c 的 k_mcan_bit_timing_tbl, 逐项同值。
struct Limits {
    uint32_t tq_min;
    uint32_t tq_max;
    uint32_t seg1_max; // 含同步段
    uint32_t seg2_min;
    uint32_t seg2_max;
    uint32_t min_diff_seg1_minus_seg2;
    uint32_t prescaler_max;
};
inline constexpr Limits kCan20{
    .tq_min = 8,
    .tq_max = 256 + 128,
    .seg1_max = 256,
    .seg2_min = 2,
    .seg2_max = 128,
    .min_diff_seg1_minus_seg2 = 2,
    .prescaler_max = 512};
inline constexpr Limits kCanFdNominal{
    .tq_min = 8,
    .tq_max = 256 + 128,
    .seg1_max = 256,
    .seg2_min = 1,
    .seg2_max = 128,
    .min_diff_seg1_minus_seg2 = 2,
    .prescaler_max = 512};
inline constexpr Limits kCanFdData{
    .tq_min = 8,
    .tq_max = 32 + 16,
    .seg1_max = 32,
    .seg2_min = 2,
    .seg2_max = 16,
    .min_diff_seg1_minus_seg2 = 1,
    .prescaler_max = 32};

// SDK 对 src_clk / baudrate 的下限(MIN_TQ_MUL_PRESCALE)。
inline constexpr uint32_t kMinTqTimesPrescaler = 8;

// 一个相位的解: 分频与 time quanta 划分(num_seg1 含同步段, 与 SDK 内部同口径)。
struct Phase {
    uint32_t prescaler;
    uint32_t num_tq;
    uint32_t num_seg1;
    uint32_t num_seg2;
};

// mcan_find_optimal_prescaler 的照搬。0 = 找不到。
constexpr uint32_t find_prescaler(
    uint32_t num_tq_mul_prescaler, uint32_t start_prescaler, uint32_t max_tq, uint32_t min_tq) {
    for (uint32_t prescaler = start_prescaler; prescaler <= num_tq_mul_prescaler; ++prescaler) {
        if (num_tq_mul_prescaler / prescaler < min_tq)
            return 0;
        if (num_tq_mul_prescaler / prescaler <= max_tq && num_tq_mul_prescaler % prescaler == 0)
            return prescaler;
    }
    return 0;
}

// mcan_calc_bit_timing_from_baudrate 的照搬(采样点窗口 [sp_min, sp_max], 单位 ‰)。
constexpr std::optional<Phase> solve(
    uint32_t clock_hz, uint32_t baudrate, uint32_t sp_min, uint32_t sp_max, const Limits& limits) {
    if (baudrate == 0U || clock_hz == 0U || clock_hz / baudrate < kMinTqTimesPrescaler
        || clock_hz / baudrate < limits.tq_min)
        return std::nullopt;

    const uint32_t num_tq_mul_prescaler = clock_hz / baudrate;
    uint32_t start_prescaler = 1U;
    while (start_prescaler <= limits.prescaler_max) {
        const uint32_t prescaler =
            find_prescaler(num_tq_mul_prescaler, start_prescaler, limits.tq_max, limits.tq_min);
        if (prescaler < start_prescaler || prescaler > limits.prescaler_max)
            return std::nullopt;
        const uint32_t num_tq = num_tq_mul_prescaler / prescaler;

        uint32_t num_seg2 = (num_tq - limits.min_diff_seg1_minus_seg2) / 2U;
        uint32_t num_seg1 = num_tq - num_seg2;
        while (num_seg2 > limits.seg2_max) {
            --num_seg2;
            ++num_seg1;
        }

        // 把划分推向采样点窗口; 撞到段长边界就换下一个分频。
        bool seg_limit_hit = false;
        while ((num_seg1 * 1000U) / num_tq < sp_min) {
            ++num_seg1;
            --num_seg2;
            if (num_seg2 < limits.seg2_min || num_seg1 > limits.seg1_max) {
                seg_limit_hit = true;
                break;
            }
        }
        if (seg_limit_hit) {
            start_prescaler = prescaler + 1U;
            continue;
        }
        // 越过窗口上沿: SDK 在这里整体放弃, 不再试更大的分频。
        if ((num_seg1 * 1000U) / num_tq > sp_max)
            return std::nullopt;
        if (num_seg2 >= limits.seg2_min && num_seg1 <= limits.seg1_max)
            return Phase{
                .prescaler = prescaler,
                .num_tq = num_tq,
                .num_seg1 = num_seg1,
                .num_seg2 = num_seg2};
        start_prescaler = prescaler + 1U;
    }
    return std::nullopt;
}

// 解出的相位真按所求落地: 速率整除得回(与 Can::timing_identity 从寄存器重构的同一算式)
// 且采样点恰为所求。SDK 用整除求分频, 除不尽时会落在近似速率上 -- 那不算解出。
constexpr bool lands_exactly(
    const std::optional<Phase>& phase, uint32_t clock_hz, uint32_t baudrate,
    uint32_t sample_point) {
    return phase.has_value() && clock_hz / (phase->prescaler * phase->num_tq) == baudrate
        && (phase->num_seg1 * 1000U) / phase->num_tq == sample_point;
}

// 本板的策略(can.hpp 的 libhcs_config 与 init_controller): 两个相位都钉在一个采样点,
// FD 数据段分频必须为 1(自动 TDCO 只在那时落在采样点上)。返回 0 = 该设置能原样落地;
// 否则返回落不下的那一段速率。
constexpr uint32_t unrepresentable_rate(
    uint32_t clock_hz, bool fd, uint32_t arbitration_baudrate, uint32_t data_baudrate,
    uint32_t nominal_sample_point, uint32_t data_sample_point) {
    const auto nominal = solve(
        clock_hz, arbitration_baudrate, nominal_sample_point, nominal_sample_point,
        fd ? kCanFdNominal : kCan20);
    if (!lands_exactly(nominal, clock_hz, arbitration_baudrate, nominal_sample_point))
        return arbitration_baudrate;
    if (!fd)
        return 0;
    const auto data =
        solve(clock_hz, data_baudrate, data_sample_point, data_sample_point, kCanFdData);
    if (!lands_exactly(data, clock_hz, data_baudrate, data_sample_point) || data->prescaler != 1U)
        return data_baudrate;
    return 0;
}

// 现场的两种总线在 80 MHz CAN 时钟下的解, 与 -Dlibhcs_CAN_DIAG=ON 打印的 SDK 结果一致:
// 仲裁段 1 Mbit = 分频 1、80 TQ、seg1 70 / seg2 10; 数据段 5 Mbit = 分频 1、16 TQ、
// seg1 14 / seg2 2(DBTP.DTSEG1 = 13)。7 Mbit 凑不出(80 / 7 不整除)。
static_assert(unrepresentable_rate(80'000'000, false, 1'000'000, 0, 875, 875) == 0);
static_assert(unrepresentable_rate(80'000'000, true, 1'000'000, 5'000'000, 875, 875) == 0);
static_assert(solve(80'000'000, 1'000'000, 875, 875, kCanFdNominal)->num_seg1 == 70);
static_assert(solve(80'000'000, 5'000'000, 875, 875, kCanFdData)->num_seg2 == 2);
static_assert(unrepresentable_rate(80'000'000, true, 1'000'000, 7'000'000, 875, 875) == 7'000'000);
static_assert(unrepresentable_rate(80'000'000, false, 3'000'000, 0, 875, 875) == 3'000'000);

} // namespace libhcs::firmware::can::bit_timing
