#pragma once

// c_board 的 USB OTG_FS Start-of-Frame 钩子, 以及共享时间基准的开关。
//
// 角色与 hpm_board / mc02 的 sync/sof.hpp 相同 -- 板上得知自己所处微帧的唯一入口 --
// 但控制器是 F407 的 OTG_FS: 芯片上唯一的 DWC2 设备核, 没有高速 PHY, 恒枚举为全速
// (内置 48 MHz PHY)。寄存器路径与 mc02 的 OTG_HS 同族: 状态位 GINTSTS.SOF、使能位
// GINTMSK.SOFM、帧计数器 DSTS.FNSOF(全速下数帧, 每中断步进 1; 全速差异全在
// sync::timebase 的 frame_scale(), 那里乘 8 折算成微帧)、速度位 DSTS.ENUMSPD。
//
// 硬件 SOF 捕获(hpm PTPC / mc02 TIM5 ITR7 的对应物): F407 上 OTG_FS 的 SOF 只引到
// TIM2 的 ITR1(TIM2_OR.ITR1_RMP, HAL 的 TIM_TIM2_USBFS_SOF)。TIM2 是全板 1/4 us 时间戳
// 源, 它的捕获值太粗, 所以捕获事件只用来触发 DMA, 在边沿那一刻抄下 84 MHz 的 TIM7 --
// 时间戳仍是中断入口的 DWT->CYCCNT, 减去 TIM7 量出的边沿到入口的间隔。细节与未验证的
// 假设见 sof.cpp 开头的注释。
//
// 如何进入中断向量、以及为何形式特殊: 与 mc02 同一套理由。本板的向量
// OTG_FS_IRQHandler 在 bsp/cubemx/Core/Src/stm32f4xx_it.c, 是 CubeMX 产物, 禁止
// 编辑(见 AGENTS.md 的 CubeMX BSP 纪律); 它在 USER CODE 段调用 TinyUSB 的
// tusb_int_handler(), 后者转调 dcd_int_handler()。因此钩子同样在链接期介入:
// -Wl,--wrap=dcd_int_handler 把 tusb_int_handler() 里那一次调用引到
// __wrap_dcd_int_handler(), 先打时间戳再链到真身。生成的代码不被触碰。
//
// 时间基准每个镜像都带, 主机在清单里要了才开(time_sync_start(), 由 EP0
// kApplyManifest 调用)。关着时 GINTMSK.SOFM 不开, 钩子在每个 USB 中断里只多一次
// 内存读和一次分支 -- 加上这一层调用本身, 几个周期, 恒定。
//
// SOF 状态位在这里被消费, dcd_int_handler() 永远看不到它, 也就不会走到它的
// "SOF 未被显式启用就把 SOFM 关掉"的分支(dcd_dwc2.c 为 iso 类驱动保留的行为),
// 更不会排队 SOF 事件: 时间基开启的板对类驱动呈现的 USB 行为与关闭时完全相同。

#include <atomic>
#include <cstdint>

namespace libhcs::firmware::sync {

namespace internal {
// 只由主循环(EP0 清单、会话结束)写; ISR 读。放 CCM(.ccmram, 上电时由 App 构造函数
// 拷入): USB 中断与主循环每趟都读它, CCM 是零等待的专有总线, 不与 USB/CAN 的 DMA
// 争 AHB。M4 没有数据缓存, 不存在缓存一致性问题。
[[gnu::section(".ccmram")]] inline constinit std::atomic<bool> g_time_sync_on{false};
} // namespace internal

// 时间基准在运行: SOF 样本入拟合窗口、应答 kTimeAnchor。任何上下文可调。
//
// 同核的中断与主循环之间只需挡住编译器重排(signal fence), 不需要硬件屏障: 读到
// true 时, time_sync_start() 在置位之前做的复位与配置对本中断都已可见。
[[nodiscard]] inline bool time_sync_on() noexcept {
    const bool on = internal::g_time_sync_on.load(std::memory_order_relaxed);
    std::atomic_signal_fence(std::memory_order_acquire);
    return on;
}

// 开时间基准: 第一次时先接好捕获(TIM2 通道 + DMA, sof.cpp 的 capture_init()), 再把状态
// 清零、开 SOF 中断。幂等。主循环调用(EP0 kApplyManifest, 端口应用之前), 那时 USB 已
// 枚举, 端口速度确定。
[[nodiscard]] bool time_sync_start();

// 关时间基准: 关 SOF 中断, kTimeAnchor 不再应答。幂等。
void time_sync_stop();

// 读取时间戳与帧计数器, 然后应答 SOF。由 __wrap_dcd_int_handler() 在真处理器
// 之前调用。
void sof_isr_entry();

// 时间基准开着时重新使能 SOF。足够便宜, 可放主循环毫秒杂务; 这也是钩子能在控制器
// 于背后被重新初始化后存活的原因 -- dcd_int_handler() 每遇到自己未预期的 SOF 都会
// 自行清掉 SOFM。
void sof_rearm();

// 自上次取走以来, SOF 中断找到了新鲜捕获的次数与没找到(或已过时)的次数, 饱和到 16 位。
// 主循环调用(kTimeStatus)。
struct CaptureCounts {
    std::uint16_t fresh;
    std::uint16_t stale;
};
[[nodiscard]] CaptureCounts take_capture_counts();

} // namespace libhcs::firmware::sync
