#pragma once

// DMTool 保存配置(0x10)的持久化: 写 flash 最后一个 4KB 扇区, 掉电保持。
//
// 扇区占用(1MB NOR): bootloader 0x0-0x1FFFF, 元数据 0x1F000-0x1FFFF, 应用
// 0x20000 起(当前 ~129KB, bootloader 允许最大 896KB)。参数区取最后一个扇区
// 0xFF000 -- 与应用的可用空间相距最远; 若未来应用镜像真的增长到触及这里,
// 链接器的空间断言会先失败。
//
// 擦写期间全局关中断(ROM API 要求, bootloader 同款), CAN RX 中断会停 ~100ms
// 量级 -- 保存是用户显式的低频动作, 高速率下瞬时丢帧可接受。

#include <array>
#include <cstddef>
#include <cstdint>

namespace libhcs::firmware::dmtool::persist {

inline constexpr std::size_t kChannelCount = 2;

// 每通道存储的位时序(TQ 语义, 与 SETUP_BUARD 负载字段同源)。
struct ChannelTiming {
    bool fd;
    uint8_t nominal_prescaler, nominal_seg1, nominal_seg2, nominal_sjw;
    uint8_t data_prescaler, data_seg1, data_seg2, data_sjw;
};

using Config = std::array<ChannelTiming, kChannelCount>;

// 读取持久化的配置。无有效存储(magic/版本/CRC 不符)返回 false。
bool load(Config& out);

// 擦除并写入配置。flash 不可用或编程失败返回 false。
bool store(const Config& config);

} // namespace libhcs::firmware::dmtool::persist
