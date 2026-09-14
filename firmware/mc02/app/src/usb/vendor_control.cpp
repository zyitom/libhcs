#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include <common/tusb_types.h>
#include <device/usbd.h>

#include "core/include/libhcs/protocol/vendor_control.hpp"
#include "firmware/mc02/app/src/can/can.hpp"
#include "firmware/mc02/app/src/uart/uart.hpp"
#include "firmware/mc02/app/src/usb/vendor.hpp"

// EP0 配置通道 -- libhcs/protocol/vendor_control.hpp 的 mc02 半边。
//
// 执行上下文: TinyUSB 在 USB 中断里排队 setup 包, 在 tud_task() 中解码, 因此下面
// 全部代码运行在主循环上, 与 bulk 下行回调同线程、同趟主循环的同一位置。这对 UART
// 路径很关键: 应用波特率会在 TX DMA 运行中改写 BRR, 切换窗口内的 RX 字节可能被打乱
// (uart/uart.hpp)。若从中断上下文执行, 该窗口会扩大到 ISR 抢占的一切范围。
//
// 不受会话门控: 主机在构造板对象时应用配置, 早于其 keepalive 线程开会话; 且会话
// 已过期的板也必须应答读回。配置是传输层状态, 不是数据面状态。

namespace {

namespace vc = libhcs::core::protocol::vendor_control;

namespace can = libhcs::firmware::can;
namespace uart = libhcs::firmware::uart;
namespace usb = libhcs::firmware::usb;

// 控制传输 data stage 的暂存缓冲。按构造同一时刻至多一个请求在途 -- EP0 是单条
// 串行管道, 且本回调在主循环上一次跑完 -- 单个静态缓冲即可。按最大负载定容;
// wLength 不符的请求在任何复制发生前即被 STALL。
alignas(4) uint8_t g_control_buffer[64];

// 拒绝原因锁存: 每次 STALL 都记录"谁、哪条通道、为什么、什么值", 供主机经
// kGetLastConfigError 读回 -- STALL 本身不带数据(USB 定义), 原因只能走读回。
// 粘滞到下一次拒绝覆盖; 仅复位清零。单写者(主循环上的 EP0 处理器)单读者(同)。
struct LastConfigError {
    uint8_t request;
    uint8_t reason;
    uint16_t index;
    uint32_t value;
};

LastConfigError g_last_config_error{0, static_cast<uint8_t>(vc::ConfigErrorReason::kConfigErrorNone), 0, 0};

void record_config_error(vc::Request request, uint16_t index, vc::ConfigErrorReason reason,
                         uint32_t value = 0) {
    g_last_config_error = {static_cast<uint8_t>(request), static_cast<uint8_t>(reason), index,
                           value};
}

// EP0 的 UART 下标顺序即板端描述符表(core/include/libhcs/spec/mc02/uart.hpp):
// 0=DBUS, 1=UART1, 2=UART2, 3=UART3, 4=UART7, 5=UART10。下标空间固定, 主机下标
// 永远指同一端口; libhcs_APP_RS485_ENABLE=OFF 的构建只是下标 2/3 背后没有对象,
// 下方处理器对它们 STALL 而非重新编号。uart_count 因此上报整个下标空间而非存活
// 端口数: 收缩它会使更高下标的含义全部移位。
constexpr uint8_t kUartIndexCount = 6;

vc::CanMode can_mode(uint16_t index);

// 板对自己的描述, 按请求现组而非缓存。FD 掩码上报的是"当前"模式: 主机可能已通过
// kSetCanConfig 重配过某条总线; CAN 对象尚未构造时, 编译期端口表就是唯一出处。
// caps 位告知主机本板的 CAN 模式可由其配置(见 kSetCanConfig)。
vc::InterfacePayload interface_payload() {
    vc::InterfacePayload payload{};
    payload.version = vc::kVersion;
    payload.can_count = static_cast<uint8_t>(can::kCanCount);
    payload.uart_count = kUartIndexCount;
    payload.caps = vc::kCapCanModeSettable;
    for (std::size_t i = 0; i < can::kCanCount; ++i) {
        if (can_mode(i) == vc::CanMode::kCanFd)
            payload.can_fd_mask |= static_cast<uint8_t>(1U << i);
    }
    return payload;
}

bool can_index_valid(uint16_t index) { return index < can::kCanCount; }

// 调用方必须已按 kUartIndexCount 校验下标。返回 null 表示该端口在本构建中不存活
// (RS-485 被编译掉)或 init() 尚未运行。
uart::UartCommon* uart_by_index(uint16_t index) {
    switch (index) {
    case 0: return uart::uart_dbus.get();
    case 1: return uart::uart1.get();
#ifdef libhcs_APP_RS485_ENABLE
    case 2: return uart::uart2.get();
    case 3: return uart::uart3.get();
#endif
    case 4: return uart::uart7.get();
    case 5: return uart::uart10.get();
    default: return nullptr;
    }
}

// 该总线当前实际发送的帧型。init() 运行前没有对象可问, 由编译期端口表作答; 此后
// 以运行值为准 -- 主机可能已通过 kSetCanConfig 重配过该总线。
vc::CanMode can_mode(uint16_t index) {
    const can::Can* bus = can::can_by_index(index);
    const bool fd =
        bus != nullptr ? bus->fd_mode() : can::kCanPorts[index].mode == can::CanMode::kCanFd;
    return fd ? vc::CanMode::kCanFd : vc::CanMode::kClassic;
}

// 控制器实际被整定的速率, 从 CubeMX 生成的分频器与 .ioc 路由给 FDCAN 的内核时钟
// (PLL2, 80 MHz)重构 -- 与 UART 的实际波特率读回同一算术, 永远不是"上次请求了什么"。
// 数据段速率只要控制器具备 FD 能力就照报, 与当前 TX 模式无关: 具备 FD 能力的
// FDCAN 在发送 classic 帧的同时仍照常解码对端发来的 FD 帧。
uint32_t can_arbitration_baudrate(uint16_t index) {
    const auto& init = can::kCanPorts[index].handle->Init;
    const uint32_t divisor =
        init.NominalPrescaler * (1U + init.NominalTimeSeg1 + init.NominalTimeSeg2);
    return divisor ? HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) / divisor : 0;
}

uint32_t can_data_baudrate(uint16_t index) {
    const auto& init = can::kCanPorts[index].handle->Init;
    const uint32_t divisor = init.DataPrescaler * (1U + init.DataTimeSeg1 + init.DataTimeSeg2);
    return divisor ? HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) / divisor : 0;
}

// 两个位相的采样点, 从 CubeMX 生成的位时序段算出(千分比): 同步段 + Seg1 占整个
// 位时间的比例。80 MHz / (5 * (1+13+2)) 的仲裁段即 (1+13)/16 = 875‰, 数据段同值
// -- 与总线上其他板一致正是固件初始化的目标。
uint32_t can_arbitration_sample_point(uint16_t index) {
    const auto& init = can::kCanPorts[index].handle->Init;
    const uint32_t total = 1U + init.NominalTimeSeg1 + init.NominalTimeSeg2;
    return total ? (1U + init.NominalTimeSeg1) * 1000U / total : 0;
}

uint32_t can_data_sample_point(uint16_t index) {
    const auto& init = can::kCanPorts[index].handle->Init;
    const uint32_t total = 1U + init.DataTimeSeg1 + init.DataTimeSeg2;
    return total ? (1U + init.DataTimeSeg1) * 1000U / total : 0;
}

// 暂存一个负载并为其发起 IN data stage。按类型而非"指针+长度"传参, 才能保证
// 暂存字节与所声明的长度永不脱节。
//
// wLength 精确匹配而非截断: 要求不同大小的主机说的是本接口的另一个版本, 截短
// 应答会让它把垃圾解码成有效回复。
template <typename Payload>
bool reply(uint8_t rhport, const tusb_control_request_t* request, const Payload& payload) {
    static_assert(std::is_trivially_copyable_v<Payload>);
    static_assert(sizeof(Payload) <= sizeof(g_control_buffer));
    if (request->wLength != sizeof(Payload))
        return false;
    std::memcpy(g_control_buffer, &payload, sizeof(Payload));
    return tud_control_xfer(rhport, request, g_control_buffer, sizeof(Payload));
}

// reply() 的镜像: 把 OUT data stage 解码为对应的负载类型。
template <typename Payload>
Payload staged() {
    static_assert(std::is_trivially_copyable_v<Payload>);
    static_assert(sizeof(Payload) <= sizeof(g_control_buffer));
    Payload payload{};
    std::memcpy(&payload, g_control_buffer, sizeof(Payload));
    return payload;
}

bool handle_setup(uint8_t rhport, const tusb_control_request_t* request) {
    const auto index = request->wIndex;

    switch (static_cast<vc::Request>(request->bRequest)) {
    case vc::Request::kGetInterface: {
        if (request->bmRequestType != vc::kRequestTypeIn || index != 0)
            return false;
        if (!reply(rhport, request, interface_payload()))
            return false;
        // 读接口本身就是握手: 走到这里的主机已被告知通道数与 CAN 模式, 不可能是
        // 配置移入 EP0 之前的旧主机。只有此后才允许开会话
        // -- 见 Vendor::session_control_deserialized_callback()。
        usb::vendor->set_ep0_handshake_done(true);
        return true;
    }

    case vc::Request::kGetCanConfig: {
        if (request->bmRequestType != vc::kRequestTypeIn || !can_index_valid(index)) {
            record_config_error(vc::Request::kGetCanConfig, index,
                                vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        return reply(rhport, request, vc::CanConfigPayload{
            .mode = static_cast<uint8_t>(can_mode(index)),
            .control = 0,
            .reserved0 = 0,
            .arbitration_baudrate = can_arbitration_baudrate(index),
            .data_baudrate = can_data_baudrate(index),
            .nominal_sample_point = static_cast<uint16_t>(can_arbitration_sample_point(index)),
            .data_sample_point = static_cast<uint16_t>(can_data_sample_point(index)),
            .reserved1 = 0,
        });
    }

    case vc::Request::kGetCanStatus: {
        if (request->bmRequestType != vc::kRequestTypeIn || !can_index_valid(index)) {
            record_config_error(vc::Request::kGetCanStatus, index,
                                vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        const can::Can* bus = can::can_by_index(index);
        if (bus == nullptr)
            return false;
        const auto s = bus->status();
        return reply(rhport, request, vc::CanStatusPayload{
            .tec = s.tec,
            .rec = s.rec,
            .last_error = s.last_error,
            .data_last_error = s.data_last_error,
            .flags = s.flags,
            .reserved = {},
            .tx_occurred = s.tx_occurred,
            .tx_cancelled = s.tx_cancelled,
            .rx_frames = s.rx_frames,
            .rx_fifo_level = s.rx_fifo_level,
        });
    }

    case vc::Request::kGetUartConfig: {
        if (request->bmRequestType != vc::kRequestTypeIn || index >= kUartIndexCount) {
            record_config_error(vc::Request::kGetUartConfig, index,
                                vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        const uart::UartCommon* port = uart_by_index(index);
        if (port == nullptr) {
            record_config_error(vc::Request::kGetUartConfig, index,
                                vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        // 硬件事实而非"上次请求": 波特率从实际编程的分频器重构, 帧格式从活
        // 寄存器解码; control 恒为 0。
        return reply(rhport, request, vc::UartConfigPayload{
            .baudrate = port->effective_baudrate(),
            .word_length = static_cast<uint8_t>(port->word_length()),
            .parity = static_cast<uint8_t>(port->parity()),
            .stop_bits = static_cast<uint8_t>(port->stop_bits()),
            .control = 0,
        });
    }

    case vc::Request::kGetLastConfigError: {
        if (request->bmRequestType != vc::kRequestTypeIn || index != 0)
            return false;
        return reply(rhport, request, vc::LastConfigErrorPayload{
            .request = g_last_config_error.request,
            .reason = g_last_config_error.reason,
            .index = g_last_config_error.index,
            .value = g_last_config_error.value,
            .reserved = 0,
        });
    }

    case vc::Request::kSetCanConfig:
        if (request->bmRequestType != vc::kRequestTypeOut || !can_index_valid(index)
            || request->wLength != sizeof(vc::CanConfigPayload))
            return false;
        // 接受 data stage; 数据到达后再校验其值。
        return tud_control_xfer(rhport, request, g_control_buffer, request->wLength);

    case vc::Request::kSetUartConfig:
        if (request->bmRequestType != vc::kRequestTypeOut || index >= kUartIndexCount
            || request->wLength != sizeof(vc::UartConfigPayload)) {
            record_config_error(vc::Request::kSetUartConfig, index,
                                vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        if (uart_by_index(index) == nullptr) {
            record_config_error(vc::Request::kSetUartConfig, index,
                                vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        return tud_control_xfer(rhport, request, g_control_buffer, request->wLength);

    default: return false;
    }
}

// 此处返回 false 会 STALL status stage, 这正是配置搬上 EP0 的全部意义: 它是 bulk
// 流承载不了的那份应答。STALL 表示设置未被应用、硬件未动 -- 主机读回会看到旧值。
bool handle_data(const tusb_control_request_t* request) {
    const auto index = request->wIndex;

    // IN 传输在应答发出后也会触发 DATA stage, 此处返回 false 会把已经成功的读也
    // STALL 掉。只有两个 OUT 请求还有后续工作要做。
    if (request->bmRequestType != vc::kRequestTypeOut)
        return true;

    switch (static_cast<vc::Request>(request->bRequest)) {
    case vc::Request::kSetCanConfig: {
        const auto payload = staged<vc::CanConfigPayload>();
        if (payload.mode > static_cast<uint8_t>(vc::CanMode::kCanFd))
            return false;
        if ((payload.control & ~vc::kCanConfigApply) != 0U)
            return false;
        // 速率与采样点是核对不是配置(位时序是本板实测整定的事实, 速率由对端硬件
        // 决定 -- 见 CanConfigPayload): 非零就必须与板子的实际值一致, 否则 STALL。
        if (payload.arbitration_baudrate != 0
            && payload.arbitration_baudrate != can_arbitration_baudrate(index))
            return false;
        if (payload.data_baudrate != 0 && payload.data_baudrate != can_data_baudrate(index))
            return false;
        if (payload.nominal_sample_point != 0
            && payload.nominal_sample_point != can_arbitration_sample_point(index))
            return false;
        if (payload.data_sample_point != 0
            && payload.data_sample_point != can_data_sample_point(index))
            return false;

        // 模式是本板唯一可由主机配置的 CAN 属性(InterfacePayload 的 caps 位已
        // 告知): 应用 = 改 Tx 元素的 FDF/BRS 标志, 控制器保持 FD 能力, 不进 INIT
        // 模式、不重新初始化, 收方向不受影响。无 kCanConfigApply 位时退化为纯
        // 核对: 与当前模式不一致即 STALL。换模式与排队中的数据不保序, 静默链路
        // 后再切的义务在主机 -- 与波特率切换同一约定。
        const bool current = can_mode(index) == vc::CanMode::kCanFd;
        const bool want_fd = payload.mode == static_cast<uint8_t>(vc::CanMode::kCanFd);
        if (want_fd != current) {
            if ((payload.control & vc::kCanConfigApply) == 0U)
                return false;
            can::Can* bus = can::can_by_index(index);
            if (bus == nullptr)
                return false;
            bus->set_fd_mode(want_fd);
        }
        return true;
    }

    case vc::Request::kSetUartConfig: {
        const auto payload = staged<vc::UartConfigPayload>();
        if ((payload.control & ~vc::kUartConfigApply) != 0U)
            return false;
        uart::UartCommon* port = uart_by_index(index);
        if (port == nullptr)
            return false;

        // 先全量校验后统一提交: 波特率先解不写(solve_brr 失败即 STALL, 寄存器
        // 原样), 帧格式在校验通过后一次 UE 下拉内写完(set_framing 内部校验先于
        // 任何写), 最后提交分频器(无失败路径)。因此 STALL 严格等于"什么都没改"。
        // 无 kUartConfigApply 位时退化为纯断言: 每个非零字段都必须与端口当前
        // 状态一致 -- 波特率用 5% 容差(与主机读回同一约定), 帧格式精确比对。
        uint32_t brr = 0;
        if (payload.baudrate != 0U && !port->solve_brr(payload.baudrate, brr)) {
            record_config_error(vc::Request::kSetUartConfig, index,
                                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable,
                                payload.baudrate);
            return false;
        }

        if ((payload.control & vc::kUartConfigApply) != 0U) {
            if (!port->set_framing(payload.word_length, payload.parity, payload.stop_bits)) {
                record_config_error(vc::Request::kSetUartConfig, index,
                                    vc::ConfigErrorReason::kConfigErrorFramingUnsupported);
                return false;
            }
            if (payload.baudrate != 0U)
                port->commit_brr(payload.baudrate, brr);
            return true;
        }

        if (payload.baudrate != 0U) {
            const uint32_t effective = port->effective_baudrate();
            const uint64_t error = effective > payload.baudrate
                                     ? effective - payload.baudrate
                                     : payload.baudrate - effective;
            if (error * 100U > static_cast<uint64_t>(payload.baudrate) * 5U)
                return false;
        }
        if (payload.word_length != 0U && payload.word_length != port->word_length()) {
            record_config_error(vc::Request::kSetUartConfig, index,
                                vc::ConfigErrorReason::kConfigErrorFramingUnsupported);
            return false;
        }
        if (payload.parity != 0U && payload.parity != port->parity()) {
            record_config_error(vc::Request::kSetUartConfig, index,
                                vc::ConfigErrorReason::kConfigErrorFramingUnsupported);
            return false;
        }
        if (payload.stop_bits != 0U && payload.stop_bits != port->stop_bits()) {
            record_config_error(vc::Request::kSetUartConfig, index,
                                vc::ConfigErrorReason::kConfigErrorFramingUnsupported);
            return false;
        }
        return true;
    }

    default: return false;
    }
}

} // namespace

// 覆盖 TinyUSB 的弱定义(后者 STALL 所有 vendor 请求)。任何 vendor 类型的请求
// 无论 recipient 为何都会到达此处(usbd.c 先按 bmRequestType_bit.type 分发, 之后
// 才看 recipient), 因此请求码共享的命名空间只有 vendor_control.hpp 这一处
// -- DFU runtime 接口使用 CLASS 请求, 不会相撞。
extern "C" bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    switch (stage) {
    case CONTROL_STAGE_SETUP: return handle_setup(rhport, request);
    case CONTROL_STAGE_DATA: return handle_data(request);
    default: return true; // CONTROL_STAGE_ACK
    }
}
