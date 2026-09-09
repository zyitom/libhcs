#include <chrono>
#include <concepts>
#include <cstdint>
#include <ratio>
#include <type_traits>

#include <stm32h7xx_hal.h>

namespace libhcs::firmware::timer {

namespace {

// The STM32 DWT (Data Watchpoint and Trace) cycle counter drives an accurate
// busy-wait. Unlike timer::timer (TIM5), it works before that timestamp source is
// started and with interrupts disabled -- HAL_Delay is invoked from the HAL clock
// and peripheral init that runs in App() (under an interrupt lock) before
// timer.init(). Sensor-init delays that run after timer.init() use
// timer::timer->spin_wait() instead; this stays DWT-based only for HAL_Delay.
constexpr uint32_t kSystemFrequency = 550'000'000;
using SysFreqDuration = std::chrono::duration<uint32_t, std::ratio<1, kSystemFrequency>>;

void delay_basic(SysFreqDuration d) {
	if (!d.count()) [[unlikely]]
		return;

	const uint32_t start = DWT->CYCCNT;
	const uint32_t end = start + d.count();

	if (end < start) {
		while (DWT->CYCCNT >= start)
			;
	}
	while (DWT->CYCCNT < end)
		;
}

template <std::integral Rep, typename Period>
void delay(std::chrono::duration<Rep, Period> d) {
	if constexpr (std::is_signed_v<Rep>) {
		if (d.count() < 0) [[unlikely]]
			return;
	}

	using DurationT = std::chrono::duration<uint32_t, Period>;
	auto casted = DurationT{static_cast<uint32_t>(d.count())};
	constexpr auto kMax = std::chrono::floor<DurationT>(SysFreqDuration::max());
	static_assert(kMax.count() > 0, "Unit too large; choose a smaller unit");

	while (casted > kMax) {
		casted -= kMax;
		delay_basic(SysFreqDuration::max());
	}
	delay_basic(std::chrono::round<SysFreqDuration>(casted));
}

} // namespace

// Rewrite the HAL_Delay function to ensure that it works when interrupts are
// disabled, while significantly improving accuracy.
extern "C" void HAL_Delay(uint32_t ms) {
	delay(std::chrono::milliseconds(ms));
}

// Minimal HAL_IncTick: only advances the millisecond counter. All low-priority
// periodic work (LED animation) has been moved to the main loop so the SysTick
// ISR stays fast and bounded.
extern "C" void HAL_IncTick() {
	uwTick += 1;
}

} // namespace libhcs::firmware::timer
