#pragma once

#include <cstddef>
#include <cstdint>

#include "core/include/libhcs/protocol/vendor_control.hpp"

// EP0 配置通道的**机制**残余 —— 各板的 vendor_control.cpp 胶水共用。
//
// 请求分发、清单的两阶段提交、按口类型的设置校验与应用, 全部在核心
// (core/src/link/ep0.hpp 与 port_ops.hpp), 对三块板与主机测试的假驱动是同一份代码。
// 这里只剩两样纯机制: 暂存缓冲与拒绝原因锁存。策略(各请求回答什么、能力位、归属
// 切换)是各板上下文的事, 在各自的胶水文件里。
//
// 运行位置(三板一致, 且是硬约束): TinyUSB 在 USB 中断里排队 setup 包, 在 tud_task()
// 中解码, 因此下面这些代码全部运行在主循环上, 与 bulk 下行回调同线程、同趟主循环的
// 同一位置。UART 路径依赖这一点 —— 应用波特率会中止发送 DMA 并置 LCR.DLAB(hpm)/
// 在 TX DMA 运行中改写 BRR(mc02), 若改在中断上下文执行, 恰好重新引入那个竞争。

namespace libhcs::firmware::usb::ep0 {

namespace vc = libhcs::core::protocol::vendor_control;

// 控制数据段的暂存上限。USB 全速 EP0 的 wMaxPacketSize 是 64, 但 v10 的清单声明
// (kApplyManifest)按最大清单(mc02 的 10 个口)是 244 字节: TinyUSB 的控制传输按
// CFG_TUD_ENDPOINT0_BUFSIZE 分包搬运, 累积进这一个缓冲(usbd_control.c 的
// data_stage_xact)。IN 应答最大的是清单结果(84 字节), 同一缓冲两用。
inline constexpr std::size_t kStagingCapacity = sizeof(vc::ManifestPayload);

// 板端独占的一份暂存存储。同一时刻至多一个请求在途 —— EP0 是单条串行管道, 且
// 本回调在主循环上一次跑完 —— 单个静态缓冲足够。wLength 不符的请求在任何拷贝
// 发生前即被 STALL。
alignas(4) inline uint8_t g_control_buffer[kStagingCapacity];

// 拒绝原因锁存: 每次 STALL 都记录"谁、哪个口、为什么、什么值", 供主机经
// kGetLastConfigError 读回 —— STALL 本身不带数据（USB 规范决定状态段长度为零），
// 原因只能走读回。粘滞到下一次拒绝覆盖；仅复位清零。单写者（主循环上的 EP0
// 处理器）单读者（同）。
//
// 每个 return false 之前都应记一次: 锁存的全部价值在于"主机能看到确切原因"。
// index 字段按请求码 carrying wIndex: 按口寻址的请求里它是 DataId。
inline vc::LastConfigErrorPayload g_last_config_error{
    .request = 0,
    .reason = static_cast<uint8_t>(vc::ConfigErrorReason::kConfigErrorNone),
    .index = 0,
    .value = 0,
    .reserved = 0,
};

inline void record_config_error(
    vc::Request request, uint16_t index, vc::ConfigErrorReason reason, uint32_t value = 0) {
    g_last_config_error = {
        .request = static_cast<uint8_t>(request),
        .reason = static_cast<uint8_t>(reason),
        .index = index,
        .value = value,
        .reserved = 0,
    };
}

} // namespace libhcs::firmware::usb::ep0
