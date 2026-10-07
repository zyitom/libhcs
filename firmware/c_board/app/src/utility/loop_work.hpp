#pragma once

#include <cstdint>
#include <utility>

#include "core/include/libhcs/data/datas.hpp"

namespace libhcs::firmware::loop {

// 主循环的位图: "有人要用"的东西各占一位, 主循环每趟只读这一个字。与 mc02 同一套
// (mc02/app/src/utility/loop_work.hpp), 区别只在本板有哪些口与放在哪块内存。
//
// 规矩是: 主机没声明的东西, 在主循环里不占一条指令。CAN、串口、板载 IMU 与引脚的周期
// 采样都由对象自己在启动时置位(各驱动的 start() / resume()); 会话结束或新一轮声明时
// 各自清位。位图为 0 -- 还没有主机, 或者主机什么都没声明 -- 时, 主循环里这些东西加起来
// 只是一次加载加一次分支。
//
// 口的位号就是它的 DataId: 不另设"第几路"的编号, 也就没有一张要与口表保持同序的表。
//
// 普通数据而非原子量, 放零等待的 CCM(.ccmram, 上电时由 App 构造函数拷入): 置位与清位
// 都只发生在主循环(EP0 处理器、下行回调、会话状态机都经 tud_task() 或主循环本身到达),
// 中断里没有人读写它。
[[gnu::section(".ccmram")]] inline constinit uint32_t active = 0;

[[nodiscard]] constexpr uint32_t bit(data::DataId id) noexcept {
    return 1U << std::to_underlying(id);
}

// CAN: 声明过(控制器在总线上)的总线。
inline constexpr uint32_t kCanBuses = bit(data::DataId::kCan1) | bit(data::DataId::kCan2);

// 串口: 声明过的口, 加上停口之后发送环还没排空的口。
inline constexpr uint32_t kUarts =
    bit(data::DataId::kUart1) | bit(data::DataId::kUart2) | bit(data::DataId::kUartDbus);

// 板载 IMU 在采样。
inline constexpr uint32_t kImu = bit(data::DataId::kImu);

// 至少有一个引脚声明了周期采样(gpio/gpio.hpp 的 refresh_sampling)。不是某一个口的事, 占
// 位 0: DataId 0 是扩展头 kExtend, 永远不是口。
inline constexpr uint32_t kGpioSampling = bit(data::DataId::kExtend);

static_assert(
    std::to_underlying(data::DataId::kUartDbus) < 32
    && std::to_underlying(data::DataId::kImu) < 32);
static_assert((kCanBuses & kUarts) == 0 && ((kCanBuses | kUarts) & (kImu | kGpioSampling)) == 0);

inline void set(uint32_t bits) { active |= bits; }
inline void clear(uint32_t bits) { active &= ~bits; }

} // namespace libhcs::firmware::loop
