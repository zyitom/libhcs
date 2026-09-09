#pragma once

#include <cstddef>
#include <cstdint>

#include <adc.h>
#include <main.h>

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

// Battery / rail voltage from a resistor divider on an ADC channel, sampled by
// circular DMA so no code has to wait on a conversion.
//
// Two halves, same split as buzzer.hpp and key.hpp: the class below names no
// handle or pin of this board; the binding at the bottom does.
//
// Four .ioc settings this driver depends on. If a Generate moves any back, it
// reads zeros or garbage rather than failing, so check here first:
//
//   1. ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR. The CubeMX
//      default (..._DR) raises no DMA request at all: the stream arms and waits
//      forever, silently.
//   2. SamplingTime = ADC_SAMPLETIME_64CYCLES_5. See
//      kMinRecommendedSamplingTime.
//   3. ClockPrescaler = ADC_CLOCK_ASYNC_DIV128: PLL2P 120 MHz -> 937.5 kHz.
//      Deliberately slow. DS13313 Table 80 caps fADC per CR.BOOST at
//      6.25/12.5/25/50 MHz, but ADC_ConfigureBoostMode() halves fADC before
//      bucketing against those same numbers ("divider by 2 for Rev.V"), so
//      above 6.25 MHz it always programs one bucket low and no prescaler fixes
//      that. Under-boosting costs SAR comparator bias current: at 16 bits, a
//      plausible wrong number, never an error. Both reference implementations
//      stay in the same band -- the official CtrBoard-H7_ADC example runs
//      1.5 MHz, the vendor BSP 187.5 kHz.
//      A conversion is 64.5 + 8.5 = 73 cycles ~= 78 us, so a 16-entry buffer
//      refills every 1.25 ms.
//   4. DMA2_Stream5 at NVIC preemption priority 5. Everything else here is more
//      urgent: FDCAN 1, USB 2, DMA and UART 3, SPI and EXTI 4. CubeMX defaults
//      this to 0, above CAN receive.
//   5. RCC_PERIPHCLK_ADC in PeriphCommonClock_Config, sourced from PLL2P. Lose
//      it and the ADC has no kernel clock at all: calibration never completes
//      and start() below has to check the clock before it will wait on one.

namespace libhcs::firmware::adc {

struct Config {
    ADC_HandleTypeDef* adc;
    // Circular DMA destination. Must live in memory the ADC's DMA controller can
    // reach and that is not behind the D-cache -- on this part that rules out
    // DTCM entirely (DMA1/DMA2 cannot address it) and rules out plain AXI SRAM
    // unless an MPU region has made it non-cacheable.
    volatile uint16_t* samples;
    std::size_t sample_count;
    // Divider ratio as (top + bottom) / bottom, i.e. the factor the pin voltage
    // must be multiplied by to recover the rail voltage.
    uint16_t divider_ratio;
    // ADC full-scale reference in millivolts.
    uint16_t reference_mv;
    // Full-scale count: 65536 for 16-bit, 4096 for 12-bit.
    uint32_t full_scale;
};

class Battery : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Battery>;

    static constexpr uint32_t kFullScale16Bit = 65536;
    static constexpr uint16_t kVrefMillivolts = 3300;

    // What matters is the sampling window in absolute time, not in cycles. The
    // divider's 100k||10k = 9.1 kOhm plus the ADC's ~200 Ohm into a 6 pF sample
    // capacitor needs R * C * ln(2^17) ~= 0.7 us to settle to 16 bits; too short
    // reads low and drags in the previous conversion. 64.5 cycles at 937.5 kHz
    // is 69 us. Move this and the prescaler (item 3 above) together.
    static constexpr uint32_t kMinRecommendedSamplingTime = ADC_SAMPLETIME_64CYCLES_5;

    Battery() = default;

    // Calibrate, then hand the DMA its circular buffer. Calibration is not
    // optional and CubeMX never generates it: an uncalibrated H7 ADC carries an
    // offset error large enough to matter at 16 bits.
    //
    // Returns false and leaves the gauge stopped if the ADC will not come up,
    // rather than asserting. The distinction matters on this firmware: an
    // assert_always is __builtin_trap(), which lands in the fault handlers, and
    // libhcs_fault_recover() answers a fault by requesting DFU and resetting
    // (assert.cpp). A refusing battery gauge would therefore park the whole
    // board in the bootloader -- CAN, UART and USB forwarding all dead -- for a
    // reading that only reports status and that nothing on the forwarding path
    // consumes. Failing soft keeps the board doing its job; millivolts() reads 0
    // and started() says why.
    [[nodiscard]] bool start(const Config& config) {
        config_ = config;
        // Programming errors, not runtime conditions: every one of these comes
        // from a compile-time constant in mc02_config(), so a failure here can
        // only mean the binding was edited wrong, and trapping is right.
        core::utility::assert_always(config_.adc != nullptr);
        core::utility::assert_always(config_.samples != nullptr);
        core::utility::assert_always(config_.sample_count > 0);
        core::utility::assert_always(config_.full_scale > 0);

        // Checked before calibrating because calibration cannot fail fast. With
        // no kernel clock the ADC's ADCAL bit never clears and
        // HAL_ADCEx_Calibration_Start spins its full ADC_CALIBRATION_TIMEOUT of
        // 633600000 volatile iterations before returning HAL_ERROR -- tens of
        // seconds with the main loop stopped and USB unserviced. This returns 0
        // when PLL2 is not ready or the ADC clock mux was never pointed at it,
        // which is exactly what a Generate that drops RCC_PERIPHCLK_ADC from
        // PeriphCommonClock_Config leaves behind (item 5 of the list above).
        if (HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC) == 0U)
            return false;

        // OFFSET_LINEARITY rather than plain OFFSET: at 16 bits the integral
        // non-linearity is worth the longer calibration (16384 ADC clocks
        // against 1280), and this runs once at bring-up. At the 937.5 kHz kernel
        // clock chosen above that is ~17 ms of blocking, which is why the caller
        // runs it before entering the forwarding loop rather than inside it.
        if (HAL_ADCEx_Calibration_Start(config_.adc, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED)
            != HAL_OK)
            return false;

        // HAL_ADC_Start_DMA's pData is uint32_t* whatever the transfer width is;
        // the DMA moves half-words because the stream was configured that way.
        //
        // Stripping volatile is unavoidable and harmless: HAL only hands the
        // address to the DMA controller, it never dereferences it. The buffer
        // stays volatile for raw(), where it matters -- nothing in this program
        // writes those samples, so without volatile the compiler is entitled to
        // fold the whole average down to the zero-initialized value.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        auto* buffer = reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(config_.samples));
        if (HAL_ADC_Start_DMA(config_.adc, buffer, static_cast<uint32_t>(config_.sample_count))
            != HAL_OK)
            return false;

        started_ = true;
        return true;
    }

    [[nodiscard]] bool started() const { return started_; }

    // Mean of the circular buffer. Reading a buffer the DMA is concurrently
    // writing is deliberate and safe here: each element is a single 16-bit
    // store, so a sample is either the old one or the new one, never a torn
    // mix, and averaging over a window whose contents shift by one entry is
    // exactly as meaningful as averaging a settled one. No lock, no cache
    // maintenance, nothing the forwarding loop has to wait for.
    [[nodiscard]] uint32_t raw() const {
        if (!started_)
            return 0;
        uint32_t sum = 0;
        for (std::size_t i = 0; i < config_.sample_count; ++i)
            sum += config_.samples[i];
        return sum / config_.sample_count;
    }

    // Rail voltage in millivolts.
    //
    // Integer throughout: the widest intermediate is raw * reference_mv *
    // divider_ratio, which for a 16-bit conversion at 3300 mV and an 11:1
    // divider peaks at 65535 * 3300 * 11 = 2.38e9 -- inside uint32_t, with the
    // multiplications ordered before the divide so no precision is thrown away
    // first.
    [[nodiscard]] uint32_t millivolts() const {
        return raw() * config_.reference_mv * config_.divider_ratio / config_.full_scale;
    }

private:
    Config config_{};
    bool started_ = false;
};

// ---------------------------------------------------------------------------
// mc02 binding. Everything above this line is board-agnostic.
// ---------------------------------------------------------------------------
//
// PC4 -> ADC1_INP4, fed from the battery through a divider. ADC1's DMA request
// is on DMA2_Stream5, which sits in the D2 domain: it can reach D2 SRAM at
// 0x30000000 and AXI SRAM at 0x24000000, but NOT DTCM. .d2_sram is the right
// home -- app.cpp's MPU region 1 already makes that range non-cacheable, which
// is what lets raw() read DMA-written samples without any invalidate.
//
// 11:1 and the 3300 mV reference have three sources that agree: the board
// manual (DM-MC-Board02 V1.1 section 18) draws R86 = 100k over R87 = 10k, the
// official CtrBoard-H7_ADC example computes adc * 3.3 / 65535 * 11, and the
// vendor BSP does the same over 65536. None of them is a measurement, so still
// check a known input against a multimeter before trusting an absolute reading.
inline constexpr uint16_t kMc02DividerRatio = 11;

// 16 samples at 78 us each is a 1.25 ms averaging window -- enough to suppress
// conversion noise, far too short to hide a real sag.
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
