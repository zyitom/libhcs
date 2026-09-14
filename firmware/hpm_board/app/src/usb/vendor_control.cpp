#include "core/include/libhcs/protocol/vendor_control.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include <common/tusb_types.h>
#include <device/usbd.h>
#include <hpm_clock_drv.h>
#include <hpm_soc.h>

#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/diag/latency.hpp"
#include "firmware/hpm_board/app/src/uart/uart.hpp"
#include "firmware/hpm_board/app/src/usb/vendor.hpp"

// EP0 配置通道 -- libhcs/protocol/vendor_control.hpp 的板端一半。
//
// 运行位置: TinyUSB 在 USB ISR 里排队 setup 包, 在 tud_task() 中解码, 因此下面的
// 代码全部运行在主循环上, 与 bulk 下行回调同线程、同轮次的同一点。这对 UART 路径
// 至关重要: 应用波特率会中止发送 DMA 并置 LCR.DLAB, 期间发往 THR 的 DMA 写会落进
// 除数锁存(uart/uart.hpp); 若改在中断上下文执行, 恰好重新引入该竞争。
//
// 不受会话门控: 主机在构造 board 对象时应用配置, 早于其 keepalive 线程开会话;
// 会话已失效的板子也必须能应答读回。配置是传输层状态, 不是数据面状态。

namespace {

namespace vc = libhcs::core::protocol::vendor_control;

namespace can = libhcs::firmware::can;
namespace uart = libhcs::firmware::uart;
namespace latency = libhcs::firmware::diag::latency;

// 控制数据段的暂存缓冲。同一时刻至多一个请求在途 -- EP0 是单条串行管道, 本回调
// 在主循环上一次跑完 -- 一个静态缓冲足够。按最大 payload 定尺寸; wLength 不符的
// 请求在任何拷贝发生前即被 STALL。
alignas(4) uint8_t g_control_buffer[64];

// 拒绝原因锁存: 每次 STALL 都记录"谁、哪条通道、为什么、什么值", 供主机经
// kGetLastConfigError 读回 -- STALL 本身不带数据(USB 定义), 原因只能走读回。
// 粘滞到下一次拒绝覆盖; 仅复位清零。
struct LastConfigError {
    uint8_t request;
    uint8_t reason;
    uint16_t index;
    uint32_t value;
};

LastConfigError g_last_config_error{
    .request = 0,
    .reason = static_cast<uint8_t>(vc::ConfigErrorReason::kConfigErrorNone),
    .index = 0,
    .value = 0,
};

void record_config_error(
    vc::Request request, uint16_t index, vc::ConfigErrorReason reason, uint32_t value = 0) {
    g_last_config_error = {
        .request = static_cast<uint8_t>(request),
        .reason = static_cast<uint8_t>(reason),
        .index = index,
        .value = value,
    };
}

// 板子对自身的描述, 按请求现场组装而非缓存: hpm5321 镜像服务两块 PCB,
// can_port_count() 是对板标识的运行时读取, 启动时烘焙一份等于给已有唯一来源的
// 东西再造第二个事实来源。
vc::InterfacePayload interface_payload() {
    vc::InterfacePayload payload{};
    payload.version = vc::kVersion;
    payload.can_count = static_cast<uint8_t>(can::can_count());
    payload.uart_count = static_cast<uint8_t>(uart::kUartCount);
    for (std::size_t i = 0; i < can::can_count(); ++i) {
        if (libhcs::firmware::board::can_port(i).mode == libhcs::firmware::board::CanMode::kCanFd)
            payload.can_fd_mask |= static_cast<uint8_t>(1U << i);
    }
    return payload;
}

bool can_index_valid(uint16_t index) { return index < can::can_count(); }

bool uart_index_valid(uint16_t index) { return index < uart::kUartCount; }

// 调用方必须先过 can_index_valid(): board::can_port() 直接对端口表取下标, 单 CAN
// 的 hpm5321 上表尾条目对应的控制器焊盘实为 LED 阴极。
vc::CanMode can_mode(uint16_t index) {
    return libhcs::firmware::board::can_port(index).mode == libhcs::firmware::board::CanMode::kCanFd
             ? vc::CanMode::kCanFd
             : vc::CanMode::kClassic;
}

// 暂存一个 payload 并为其启动 IN 数据段。按类型传参而非指针加长度, 才能保证
// 暂存的字节与声明的尺寸永不脱节。
//
// wLength 精确匹配而非截断: 要求不同尺寸的主机说的是另一个版本的接口, 截断应答
// 会让它把垃圾解码成合法回复。
template <typename Payload>
bool reply(uint8_t rhport, const tusb_control_request_t* request, const Payload& payload) {
    static_assert(std::is_trivially_copyable_v<Payload>);
    static_assert(sizeof(Payload) <= sizeof(g_control_buffer));
    if (request->wLength != sizeof(Payload))
        return false;
    std::memcpy(g_control_buffer, &payload, sizeof(Payload));
    return tud_control_xfer(rhport, request, g_control_buffer, sizeof(Payload));
}

// reply() 的镜像: 把 OUT 数据段解码回 payload 类型。
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
        // 读取接口本身就是握手: 走到这里的主机已被告知通道数与 CAN 模式, 不可能
        // 是配置迁到 EP0 之前的旧主机。仅从此刻起才允许会话 -- 见
        // HostSession::session_allowed()。
        libhcs::firmware::usb::vendor->set_ep0_handshake_done(true);
        return true;
    }

    case vc::Request::kGetCanConfig: {
        if (request->bmRequestType != vc::kRequestTypeIn || !can_index_valid(index)) {
            record_config_error(
                vc::Request::kGetCanConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        // 硬件事实而非"上次请求": 模式即编译期端口表的值; 数据段速率仅在该模式
        // 确实是 FD 时有意义(本板 classic 编译从不出现, 保守返回 0)。
        const bool fd = can_mode(index) == vc::CanMode::kCanFd;
        return reply(
            rhport, request,
            vc::CanConfigPayload{
                .mode = static_cast<uint8_t>(can_mode(index)),
                .control = 0,
                .reserved0 = 0,
                .arbitration_baudrate = can::Can::kArbitrationBaudrate,
                .data_baudrate = fd ? can::Can::kCanFdDataBaudrate : 0,
                .nominal_sample_point = can::Can::kNominalSamplePointPerMille,
                .data_sample_point =
                    static_cast<uint16_t>(fd ? can::Can::kDataSamplePointPerMille : 0),
                .reserved1 = 0,
            });
    }

    case vc::Request::kGetLastConfigError: {
        if (request->bmRequestType != vc::kRequestTypeIn || index != 0)
            return false;
        return reply(
            rhport, request,
            vc::LastConfigErrorPayload{
                .request = g_last_config_error.request,
                .reason = g_last_config_error.reason,
                .index = g_last_config_error.index,
                .value = g_last_config_error.value,
                .reserved = 0,
            });
    }

    case vc::Request::kGetLatencyBreakdown: {
        if (request->bmRequestType != vc::kRequestTypeIn)
            return false;
        const auto down = latency::downlink;
        const auto up = latency::uplink;
        // index 非零时读后清零累加器, 调用方可以夹住一段测量, 而不必永远看到开机
        // 以来的全部。用 wIndex 而非 wValue: 主机侧辅助函数只暴露 wIndex, wValue
        // 恒以 0 发送。
        if (index != 0)
            latency::reset();
        return reply(
            rhport, request,
            vc::LatencyBreakdownPayload{
                .downlink_count = down.count,
                .downlink_min_cycles = down.count ? down.min_cycles : 0,
                .downlink_max_cycles = down.max_cycles,
                .downlink_sum_cycles = down.sum_cycles,
                .uplink_count = up.count,
                .uplink_min_cycles = up.count ? up.min_cycles : 0,
                .uplink_max_cycles = up.max_cycles,
                .uplink_sum_cycles = up.sum_cycles,
                .cpu_hz = clock_get_frequency(clock_cpu0),
                .reserved = 0,
            });
    }

    case vc::Request::kGetCanStatus: {
        if (request->bmRequestType != vc::kRequestTypeIn || !can_index_valid(index))
            return false;
        const can::Can* bus = can::can_array[index].try_get();
        if (bus == nullptr)
            return false;
        const auto s = bus->status();
        return reply(
            rhport, request,
            vc::CanStatusPayload{
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
        if (request->bmRequestType != vc::kRequestTypeIn || !uart_index_valid(index)) {
            record_config_error(
                vc::Request::kGetUartConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        const uart::Uart* port = uart::uart_array[index].try_get();
        if (port == nullptr) {
            record_config_error(
                vc::Request::kGetUartConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        // 硬件事实而非"上次请求": 波特率从实际写入的分频器与过采样率重建,
        // 帧格式从 LCR 解码; control 恒为 0。
        return reply(
            rhport, request,
            vc::UartConfigPayload{
                .baudrate = port->effective_baudrate(),
                .word_length = static_cast<uint8_t>(port->word_length()),
                .parity = static_cast<uint8_t>(port->parity()),
                .stop_bits = static_cast<uint8_t>(port->stop_bits()),
                .control = 0,
            });
    }

    case vc::Request::kSetCanConfig:
        if (request->bmRequestType != vc::kRequestTypeOut || !can_index_valid(index)
            || request->wLength != sizeof(vc::CanConfigPayload))
            return false;
        // 先接受数据段, 值等到达后再校验。
        return tud_control_xfer(rhport, request, g_control_buffer, request->wLength);

    case vc::Request::kSetUartConfig:
        if (request->bmRequestType != vc::kRequestTypeOut || !uart_index_valid(index)
            || request->wLength != sizeof(vc::UartConfigPayload))
            return false;
        if (uart::uart_array[index].try_get() == nullptr)
            return false;
        return tud_control_xfer(rhport, request, g_control_buffer, request->wLength);

    default: return false;
    }
}

// 在这里返回 false 会 STALL 状态段 -- 这正是配置搬到 EP0 的全部意义: 这是 bulk
// 流承载不了的应答。STALL 意味着设置未生效、硬件未被触碰, 主机读回会看到旧值。
bool handle_data(const tusb_control_request_t* request) {
    const auto index = request->wIndex;

    // IN 传输在应答发出后也会触发 DATA 段, 此时返回 false 会把已经成功的读取
    // STALL 掉。只有两个 OUT 请求还有事要做。
    if (request->bmRequestType != vc::kRequestTypeOut)
        return true;

    switch (static_cast<vc::Request>(request->bRequest)) {
    case vc::Request::kSetCanConfig: {
        const auto payload = staged<vc::CanConfigPayload>();
        if (payload.mode > static_cast<uint8_t>(vc::CanMode::kCanFd)) {
            record_config_error(
                vc::Request::kSetCanConfig, index, vc::ConfigErrorReason::kConfigErrorBadRequest,
                payload.mode);
            return false;
        }
        if ((payload.control & ~vc::kCanConfigApply) != 0U) {
            record_config_error(
                vc::Request::kSetCanConfig, index, vc::ConfigErrorReason::kConfigErrorBadRequest,
                payload.control);
            return false;
        }
        // 速率字段是核对不是配置(位时序是本板实测整定的事实): 非零就必须与编译
        // 期常量一致, 否则 STALL。
        if (payload.arbitration_baudrate != 0
            && payload.arbitration_baudrate != can::Can::kArbitrationBaudrate)
            return false;
        if (payload.data_baudrate != 0 && payload.data_baudrate != can::Can::kCanFdDataBaudrate) {
            record_config_error(
                vc::Request::kSetCanConfig, index,
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, payload.data_baudrate);
            return false;
        }
        if (payload.nominal_sample_point != 0
            && payload.nominal_sample_point != can::Can::kNominalSamplePointPerMille) {
            record_config_error(
                vc::Request::kSetCanConfig, index,
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable,
                payload.nominal_sample_point);
            return false;
        }
        if (payload.data_sample_point != 0
            && payload.data_sample_point != can::Can::kDataSamplePointPerMille) {
            record_config_error(
                vc::Request::kSetCanConfig, index,
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, payload.data_sample_point);
            return false;
        }
        // 只校验, 不应用: 本板不设 kCapCanModeSettable 位 -- 控制器以 enable_canfd
        // 初始化, classic 化意味着收不到任何 FD 帧(实测 0/50), 且 re-init 会把钉死
        // 的采样点/TDC/PTPC 时基全部置于风险之下, 见 CanConfigPayload。kCanConfigApply
        // 位在本板没有语义: 请求与编译期模式一致才 ACK, 否则一律 STALL。
        if (static_cast<vc::CanMode>(payload.mode) != can_mode(index)) {
            record_config_error(
                vc::Request::kSetCanConfig, index, vc::ConfigErrorReason::kConfigErrorModeFixed,
                payload.mode);
            return false;
        }
        return true;
    }

    case vc::Request::kSetUartConfig: {
        const auto payload = staged<vc::UartConfigPayload>();
        if ((payload.control & ~vc::kUartConfigApply) != 0U) {
            record_config_error(
                vc::Request::kSetUartConfig, index, vc::ConfigErrorReason::kConfigErrorBadRequest,
                payload.control);
            return false;
        }
        uart::Uart* port = uart::uart_array[index].try_get();
        if (port == nullptr) {
            record_config_error(
                vc::Request::kSetUartConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }

        // 先全量校验后统一提交: 帧格式先纯校验(check_framing 不碰寄存器), 再
        // 切波特率(set_baudrate 自身失败即 STALL 且未动任何寄存器), 最后落帧
        // 格式。因此 STALL 严格等于"什么都没改"。无 kUartConfigApply 位时退化
        // 为纯断言: 每个非零字段都必须与端口当前状态一致 -- 波特率用 5% 容差
        // (与主机读回同一约定), 帧格式精确比对。
        if (!port->check_framing(payload.word_length, payload.parity, payload.stop_bits)) {
            record_config_error(
                vc::Request::kSetUartConfig, index,
                vc::ConfigErrorReason::kConfigErrorFramingUnsupported);
            return false;
        }

        if ((payload.control & vc::kUartConfigApply) != 0U) {
            if (payload.baudrate != 0U && !port->set_baudrate(payload.baudrate)) {
                record_config_error(
                    vc::Request::kSetUartConfig, index,
                    vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, payload.baudrate);
                return false;
            }
            port->commit_framing(payload.word_length, payload.parity, payload.stop_bits);
            return true;
        }

        if (payload.baudrate != 0U) {
            const uint32_t effective = port->effective_baudrate();
            const uint64_t error = effective > payload.baudrate ? effective - payload.baudrate
                                                                : payload.baudrate - effective;
            if (error * 100U > static_cast<uint64_t>(payload.baudrate) * 5U) {
                record_config_error(
                    vc::Request::kSetUartConfig, index,
                    vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, payload.baudrate);
                return false;
            }
        }
        if (payload.word_length != 0U && payload.word_length != port->word_length())
            return false;
        if (payload.parity != 0U && payload.parity != port->parity())
            return false;
        return payload.stop_bits == 0U || payload.stop_bits == port->stop_bits();
    }

    default: return false;
    }
}

} // namespace

// 覆盖 TinyUSB 的弱定义, 后者对每个 vendor 请求一律 STALL。任何 vendor 类型请求
// 无论 recipient 是谁都会到达这里(usbd.c 先按 bmRequestType_bit.type 分发, 之后才
// 看 recipient), 因此本文件与外界共享的唯一命名空间就是 vendor_control.hpp 里的
// 请求码 -- DFU runtime 接口用 CLASS 请求, 不会冲突。
extern "C" bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    switch (stage) {
    case CONTROL_STAGE_SETUP: return handle_setup(rhport, request);
    case CONTROL_STAGE_DATA: return handle_data(request);
    default: return true; // 即 CONTROL_STAGE_ACK
    }
}
