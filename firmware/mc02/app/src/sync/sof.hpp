#pragma once

// mc02 的 DWC2 控制器上的 USB Start-of-Frame 钩子。
//
// 角色与 hpm_board 的 sync/sof.hpp 相同 -- 板上得知自己所处微帧的唯一入口 --
// 但底层 USB IP 不同, 整条寄存器路径都不同: 状态位 GINTSTS.SOF、使能位
// GINTMSK.SOFM、帧计数器 DSTS.FNSOF(全速下数帧)、速度位 DSTS.ENUMSPD。其中
// 帧计数器的差异影响最大: DWC2 的 FNSOF 只有高速下才是微帧号; 全速 -- mc02
// 无 ULPI PHY, 也只能全速 -- 下是帧号, 每中断步进 1, 而 EHCI 的 FRINDEX 步进
// 8。缩放在 sync::timebase 里做, 见其 frame_scale() 的注释。
//
// 如何进入中断向量、以及为何形式特殊: hpm_board 持有自己的 USB 向量, 把钩子作为
// 第一条语句调用。mc02 的向量在 bsp/cubemx/Core/Src/stm32h7xx_it.c, 是 CubeMX
// 产物, 本仓库禁止编辑(见 AGENTS.md 的 CubeMX BSP 纪律一节), 它只是转调 TinyUSB
// 的 dcd_int_handler()。因此钩子在链接期介入: -Wl,--wrap=dcd_int_handler 把向量
// 的调用引到下面的 __wrap_dcd_int_handler(), 先打时间戳再链到真身。生成的代码
// 不被触碰; libhcs_APP_TIME_SYNC 关闭时 wrap 标志根本不传, 默认构建的链接与
// 从前完全一致。
//
// 晚一拍进入的代价就是那一次调用本身: 几个周期, 550 MHz 下几十纳秒, 且恒定。
//
// SOF 状态位在这里被消费, dcd_int_handler() 永远看不到它, 也就不会排队
// DCD_EVENT_SOF: 时间基开启的板对类驱动呈现的 USB 行为与关闭时完全相同。

#include <cstdint>

namespace libhcs::firmware::sync {

// 读取时间戳与帧计数器, 然后应答 SOF。由 __wrap_dcd_int_handler() 在真处理器
// 之前调用; 时间基被编译剔除时是空函数, 编译器会整体删除。
void sof_isr_entry();

// 使能 GINTMSK.SOFM。必须在 tusb_rhport_init() 之后运行: 其 dcd_init() 会整体
// 覆写 GINTMSK。
void sof_init();

// 重新使能 SOF。足够便宜, 可放主循环周期任务; 这也是钩子能在控制器于背后被
// 重新初始化后存活的原因 -- dcd_int_handler() 每遇到自己未预期的 SOF 都会自行
// 清掉 SOFM。
void sof_rearm();

} // namespace libhcs::firmware::sync
