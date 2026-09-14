#pragma once

#include <cstddef>
#include <cstdint>

#include <adc.h>
#include <main.h>

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

// 电阻分压接入 ADC 通道的电池/电源轨电压, 循环 DMA 采样, 无需任何代码等待转换。
//
// 与 buzzer.hpp、key.hpp 相同的两段式: 下面的类不涉及本板任何句柄或引脚; 底部
// binding 涉及。
//
// 本驱动依赖以下 .ioc 设置。若某次 Generate 把它们改回去, 读到的将是 0 或垃圾
// 而非报错, 先查这里:
//
//   1. ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR。CubeMX 默认
//      (..._DR)根本不发起 DMA 请求: 流武装后永远干等, 静默无数据。
//   2. SamplingTime = ADC_SAMPLETIME_64CYCLES_5, 见 kMinRecommendedSamplingTime。
//   3. ClockPrescaler 选 ADC_CLOCK_ASYNC_DIV128: PLL2P 120 MHz -> 937.5 kHz,
//      刻意选慢。DS13313 表 80 按 CR.BOOST 把 fADC 上限分档为 6.25/12.5/25/50
//      MHz, 但 ADC_ConfigureBoostMode() 在分档前先把 fADC 减半(Rev.V 除 2), 故
//      高于 6.25 MHz 时总被归入低一档, 任何预分频都救不了。欠 boost 的代价是
//      SAR 比较器偏置电流: 16 位下表现为貌似合理的错误数值, 从不报错。两个参照
//      实现同处此频段 -- 官方 CtrBoard-H7_ADC 示例跑 1.5 MHz, 厂商 BSP 187.5 kHz。
//      一次转换 64.5 + 8.5 = 73 cycles 约 78 us, 16 项缓冲每 1.25 ms 刷新一轮。
//   4. DMA2_Stream5 的 NVIC 抢占优先级 5。此处其余一切都更紧迫: FDCAN 1、
//      USB 2、DMA 与 UART 3、SPI 与 EXTI 4。CubeMX 默认 0, 高于 CAN 接收。
//   5. PeriphCommonClock_Config 里的 RCC_PERIPHCLK_ADC, 来自 PLL2P。丢了它 ADC
//      就没有内核时钟: 校准永不完成, 下面的 start() 只好先查时钟再等待。

namespace libhcs::firmware::adc {

struct Config {
    ADC_HandleTypeDef* adc;
    // 循环 DMA 的目的地址。必须位于 ADC 的 DMA 控制器可达且不在 D-cache 之后的
    // 内存 -- 本芯片上 DTCM 完全出局(DMA1/DMA2 无法寻址), 普通 AXI SRAM 除非已
    // 有 MPU 区域将其设为非缓存, 否则也不行。
    volatile uint16_t* samples;
    std::size_t sample_count;
    // 分压比 (上阻 + 下阻) / 下阻, 即引脚电压乘以它还原电源轨电压的系数。
    uint16_t divider_ratio;
    // ADC 满量程基准电压, 毫伏。
    uint16_t reference_mv;
    // 满量程计数: 16 位为 65536, 12 位为 4096。
    uint32_t full_scale;
};

class Battery : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Battery>;

    static constexpr uint32_t kFullScale16Bit = 65536;
    static constexpr uint16_t kVrefMillivolts = 3300;

    // 要紧的是采样窗口的绝对时长而非周期数。分压网络 100k||10k = 9.1 kOhm 加
    // ADC 约 200 Ohm 对 6 pF 采样电容, 稳定到 16 位需 R * C * ln(2^17) 约 0.7 us;
    // 太短会读低并掺入上一次转换。937.5 kHz 下 64.5 cycles 即 69 us。改动此项时
    // 连同预分频(上面第 3 条)一起改。
    static constexpr uint32_t kMinRecommendedSamplingTime = ADC_SAMPLETIME_64CYCLES_5;

    Battery() = default;

    // 先校准, 再把循环缓冲交给 DMA。校准不可省且 CubeMX 从不生成: 未校准的 H7
    // ADC 带有在 16 位下足以起作用的偏移误差。
    //
    // ADC 起不来时返回 false 并让电量计保持停止, 而非断言。这一区分对本固件
    // 要紧: assert_always 是 __builtin_trap(), 会落入 fault handler, 而
    // libhcs_fault_recover() 对 fault 的响应是请求 DFU 并复位(assert.cpp)。
    // 一个拒绝启动的电量计会把整块板停进 bootloader -- CAN、UART、USB 转发全灭
    // -- 而它只产出一个仅作状态报告、转发路径无人消费的读数。软失败让板子继续
    // 干活; millivolts() 读 0, started() 说明原因。
    [[nodiscard]] bool start(const Config& config) {
        config_ = config;
        // 编程错误而非运行时状况: 这些都来自 mc02_config() 的编译期常量, 此处
        // 失败只可能是 binding 改错了, trap 是对的。
        core::utility::assert_always(config_.adc != nullptr);
        core::utility::assert_always(config_.samples != nullptr);
        core::utility::assert_always(config_.sample_count > 0);
        core::utility::assert_always(config_.full_scale > 0);

        // 校准之前检查, 因为校准无法快速失败。没有内核时钟时 ADCAL 位永不清零,
        // HAL_ADCEx_Calibration_Start 会空转满 ADC_CALIBRATION_TIMEOUT 的
        // 633600000 次易失迭代才返回 HAL_ERROR -- 主循环停摆、USB 失服数十秒。
        // PLL2 未就绪或 ADC 时钟 mux 从未指向它时此处返回 0, 恰是 Generate 把
        // RCC_PERIPHCLK_ADC 从 PeriphCommonClock_Config 里弄丢后的样子(上面
        // 第 5 条)。
        if (HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC) == 0U)
            return false;

        // 用 OFFSET_LINEARITY 而非普通 OFFSET: 16 位下积分非线性值得换更长的
        // 校准(16384 对 1280 个 ADC 时钟), 且只在 bring-up 跑一次。按上面选的
        // 937.5 kHz 内核时钟约阻塞 17 ms, 故调用方在进入转发循环之前而非其中
        // 运行它。
        if (HAL_ADCEx_Calibration_Start(config_.adc, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED)
            != HAL_OK)
            return false;

        // 无论传输宽度如何, HAL_ADC_Start_DMA 的 pData 都是 uint32_t*; DMA 搬
        // 半字是因为流是那样配置的。
        //
        // 去 volatile 不可避免且无害: HAL 只把地址交给 DMA 控制器, 从不解引用。
        // 缓冲保持 volatile 是为 raw() -- 那里才要紧: 本程序无人写这些样本, 若无
        // volatile, 编译器有权把整个平均折叠成零初始化值。
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        auto* buffer = reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(config_.samples));
        if (HAL_ADC_Start_DMA(config_.adc, buffer, static_cast<uint32_t>(config_.sample_count))
            != HAL_OK)
            return false;

        started_ = true;
        return true;
    }

    [[nodiscard]] bool started() const { return started_; }

    // 循环缓冲的均值。读 DMA 正在并发写的缓冲在此是刻意且安全的: 每元素一次
    // 16 位存储, 样本要么旧要么新, 绝无撕裂混合; 对内容每次只移动一项的窗口取
    // 平均, 与对稳定的窗口取平均同样有意义。无锁、无 cache 维护, 转发循环无需
    // 等待任何东西。
    [[nodiscard]] uint32_t raw() const {
        // start() asserts sample_count > 0 before setting started_; the analyzer
        // cannot see that invariant, so the division keeps its own guard.
        if (!started_ || config_.sample_count == 0)
            return 0;
        uint32_t sum = 0;
        for (std::size_t i = 0; i < config_.sample_count; ++i)
            sum += config_.samples[i];
        return sum / config_.sample_count;
    }

    // 电源轨电压, 毫伏。
    //
    // 全程整数: 最宽中间量为 raw * reference_mv * divider_ratio, 16 位转换、
    // 3300 mV、11:1 分压下峰值 65535 * 3300 * 11 = 2.38e9 -- 在 uint32_t 内;
    // 乘法排在除法之前, 精度不被提前丢掉。
    [[nodiscard]] uint32_t millivolts() const {
        return raw() * config_.reference_mv * config_.divider_ratio / config_.full_scale;
    }

private:
    Config config_{};
    bool started_ = false;
};

// ------- mc02 binding: 本线以上与板无关 -------
//
// PC4 -> ADC1_INP4, 经分压接电池。ADC1 的 DMA 请求在 DMA2_Stream5, 属 D2 域:
// 可达 0x30000000 的 D2 SRAM 与 0x24000000 的 AXI SRAM, 不可达 DTCM。.d2_sram
// 是正确归宿 -- app.cpp 的 MPU 区域 1 已把该范围设为非缓存, raw() 读 DMA 写入的
// 样本因此无需任何 invalidate。
//
// 11:1 与 3300 mV 基准有三个一致来源: 板卡手册(DM-MC-Board02 V1.1 第 18 节)
// 画了 R86 = 100k 对 R87 = 10k, 官方 CtrBoard-H7_ADC 示例按 adc * 3.3 / 65535
// * 11 计算, 厂商 BSP 同式但除 65536。三者皆非实测, 信任绝对读数前仍应用万用表
// 对已知输入核对。
inline constexpr uint16_t kMc02DividerRatio = 11;

// 16 样本 * 78 us 即 1.25 ms 平均窗 -- 足以压住转换噪声, 又远不足以掩盖真实的
// 电压跌落。
inline constexpr std::size_t kMc02SampleCount = 16;

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
[[gnu::section(".d2_sram")]] inline volatile uint16_t mc02_samples[kMc02SampleCount];

[[nodiscard]] inline Config mc02_config() {
    return Config{
        .adc = &hadc1,
        .samples = mc02_samples,
        .sample_count = kMc02SampleCount,
        .divider_ratio = kMc02DividerRatio,
        .reference_mv = Battery::kVrefMillivolts,
        .full_scale = Battery::kFullScale16Bit,
    };
}

inline constinit Battery::Lazy battery;

} // namespace libhcs::firmware::adc
