#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

#include <common/tusb_types.h>
#include <device/usbd.h>

#include "core/include/libhcs/protocol/vendor_control.hpp"

// EP0 配置通道的**机制**一半 —— 所有板的 vendor_control.cpp 共用。
//
// 这里只放"机制": 暂存缓冲、拒绝原因锁存、staged()/reply()。**不放策略** ——
// 每个请求回答什么、能力位怎么填、下标空间多大，都由各板自己的 vendor_control.cpp
// 决定。各板在这些地方确实不同（hpm_board 设 MS OS 2.0 描述符集、切 CAN 帧型靠重初始化
// 控制器；mc02 只翻 Tx 元素标志；c_board 模式写死），把派发也提取出来会需要一层板策略模板，反而把
// 各自的 AGENTS.md 契约表所记录的差异藏起来。
//
// 提取的动因是**重复**而不是抽象: 同一份 staged()/reply()/record_config_error()
// 原先在 hpm_board 与 mc02 各写一遍、逐字相同。给 c_board 加 EP0 会变成第三份。
//
// 运行位置（两板一致，且是硬约束）: TinyUSB 在 USB 中断里排队 setup 包，在
// tud_task() 中解码，因此下面这些代码全部运行在主循环上，与 bulk 下行回调同线程、
// 同趟主循环的同一位置。UART 路径依赖这一点 —— 应用波特率会中止发送 DMA 并置
// LCR.DLAB（hpm）/ 在 TX DMA 运行中改写 BRR（mc02），若改在中断上下文执行，恰好
// 重新引入那个竞争。

namespace libhcs::firmware::usb::ep0 {

namespace vc = libhcs::core::protocol::vendor_control;

// 控制数据段的暂存上限。USB 全速 EP0 的 wMaxPacketSize 是 64，且本项目所有 EP0
// 载荷都远小于它；按此定容与 SDK 的 kVendorControlPayloadMax 一致。
inline constexpr std::size_t kStagingCapacity = 64;

// 板端独占的一份暂存存储。同一时刻至多一个请求在途 —— EP0 是单条串行管道，且
// 本回调在主循环上一次跑完 —— 单个静态缓冲足够。wLength 不符的请求在任何拷贝
// 发生前即被 STALL。
//
// 不是每个传输层都用得上: 传入自己的缓冲即可（ch32_board 的载荷在 USBSS_EP0_Buf
// 里，由 ISR 在应答前填入）。寄存这份是为了给 TinyUSB 板一个默认实现。
alignas(4) inline uint8_t g_control_buffer[kStagingCapacity];

// 拒绝原因锁存: 每次 STALL 都记录"谁、哪条通道、为什么、什么值"，供主机经
// kGetLastConfigError 读回 —— STALL 本身不带数据（USB 规范决定状态段长度为零），
// 原因只能走读回。粘滞到下一次拒绝覆盖；仅复位清零。单写者（主循环上的 EP0
// 处理器）单读者（同）。
//
// 每个 return false 之前都应记一次: 锁存的全部价值在于"主机能看到确切原因"，
// 漏记等于对该失败模式放弃这个能力。hpm_board 与 mc02 曾各漏一半（一方漏速率、
// 一方漏帧格式），合并到此处后不会再分歧。
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

// EP0 载荷的形状约束: 可直接按字节搬运，且装得进暂存缓冲。
template <typename Payload>
concept Ep0Payload = std::is_trivially_copyable_v<Payload> && (sizeof(Payload) <= kStagingCapacity);

// 暂存一个载荷并为其发起 IN 数据段。按类型而非"指针+长度"传参，才能保证暂存字节
// 与所声明的长度永不脱节；static_assert 让缓冲溢出在编译期死掉。
//
// wLength 精确匹配而非截断: 要求不同尺寸的主机说的是本接口的另一个版本，截短应答
// 会让它把垃圾解码成合法回复。
template <Ep0Payload Payload>
bool reply(
    uint8_t rhport, const tusb_control_request_t* request, const Payload& payload,
    std::span<uint8_t> staging = {g_control_buffer, kStagingCapacity}) {
    if (request->wLength != sizeof(Payload))
        return false;
    std::memcpy(staging.data(), &payload, sizeof(Payload));
    return tud_control_xfer(rhport, request, staging.data(), sizeof(Payload));
}

// reply() 的镜像: 把 OUT 数据段解码回载荷类型。缓冲可指定，理由见 g_control_buffer
// 的注释。
template <Ep0Payload Payload>
Payload staged(std::span<const uint8_t> staging = {g_control_buffer, kStagingCapacity}) {
    Payload payload{};
    std::memcpy(&payload, staging.data(), sizeof(Payload));
    return payload;
}

// 主机只发一次 SET、不再回读, 所以 ACK 必须等于"已生效": 下面两条是各板 UART 处理器共用的判据。

// 宽松兜底: 实际速率偏离请求超 10% 视为离谱; 严格判据仍是分频器整数。
inline bool rate_plausible(uint32_t requested, uint32_t achieved) {
    const uint64_t error = achieved > requested ? achieved - requested : requested - achieved;
    return error * 10U <= static_cast<uint64_t>(requested);
}

// 帧格式判据: 每个非零字段都等于端口活寄存器的解码值(0 = 不关心)。
template <typename Port>
bool framing_matches(const Port& port, uint32_t word_length, uint32_t parity, uint32_t stop_bits) {
    return (word_length == 0U || word_length == port.word_length())
        && (parity == 0U || parity == port.parity())
        && (stop_bits == 0U || stop_bits == port.stop_bits());
}

} // namespace libhcs::firmware::usb::ep0
