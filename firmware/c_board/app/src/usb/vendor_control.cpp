#include "core/include/libhcs/protocol/vendor_control.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include <common/tusb_types.h>
#include <device/usbd.h>

#include "firmware/c_board/app/src/can/can.hpp"
#include "firmware/c_board/app/src/uart/uart.hpp"
#include "firmware/c_board/app/src/usb/vendor.hpp"
#include "firmware/common/app/src/usb/ep0_staging.hpp"

// EP0 配置通道 -- libhcs/protocol/vendor_control.hpp 的 c_board 半边。
//
// 本板此前**只有数据流路径**: 配置走 bulk 流里的 kUart*Config 字段, 而那条路是
// 只写的 —— 反序列化回调返回 bool 的含义是"这个字段我认得", 不是"这个操作成功
// 了", 所以板端拒绝的波特率会被主机当成成功。EP0 是本板第一次能说"我拒绝"。
//
// 执行上下文: TinyUSB 在 USB 中断里排队 setup 包, 在 tud_task() 中解码, 因此下面
// 全部代码运行在主循环上, 与 bulk 下行回调同线程、同趟主循环的同一位置。
//
// 不受会话门控: 主机在构造 board 对象时应用配置, 早于其 keepalive 线程开会话;
// 且会话已过期的板也必须应答读回。配置是传输层状态, 不是数据面状态。

namespace {

namespace vc = libhcs::core::protocol::vendor_control;

namespace can = libhcs::firmware::can;
namespace uart = libhcs::firmware::uart;
namespace usb = libhcs::firmware::usb;

// 机制部分（暂存缓冲、拒绝原因锁存、staged()/reply()）在共享头里, 四块板共用;
// 这里只引入名字。策略（各请求回答什么、能力位、下标空间）留在本文件 —— 本板与
// mc02/hpm_board 在那几点上不同（见下 caps 与 can_mode 的注释）。
using libhcs::firmware::usb::ep0::framing_matches;
using libhcs::firmware::usb::ep0::g_control_buffer;
using libhcs::firmware::usb::ep0::g_last_config_error;
using libhcs::firmware::usb::ep0::rate_plausible;
using libhcs::firmware::usb::ep0::record_config_error;
using libhcs::firmware::usb::ep0::reply;
using libhcs::firmware::usb::ep0::staged;

// EP0 的 UART 下标顺序即板端描述符表(core/include/libhcs/spec/c_board/uart.hpp):
// 0=DBUS(huart3), 1=UART1(huart6), 2=UART2(huart1)。本板三个口全部常驻, 没有
// RS-485 之类的编译期开关, 因此下标空间与存活端口数是同一个数。
constexpr uint8_t kUartIndexCount = 3;

// 本板是 bxCAN: 经典 CAN 控制器, 既收不到也发不出 FD 帧, 长帧更无从谈起。因此
// InterfacePayload 的两个能力位全清 —— 模式是固件的编译期属性(硬件根本没有 FD
// 能力; mc02/hpm_board 置 kCapCanModeSettable, 本板无从谈起), 长帧也不承载。
vc::InterfacePayload interface_payload() {
    vc::InterfacePayload payload{};
    payload.version = vc::kVersion;
    payload.can_count = static_cast<uint8_t>(can::kCanCount);
    payload.uart_count = kUartIndexCount;
    payload.caps = 0;
    // bxCAN 无 FD: 掩码恒为 0, 不必逐总线询问。
    payload.can_fd_mask = 0;
    return payload;
}

bool can_index_valid(uint16_t index) { return index < can::kCanCount; }

uart::Uart* uart_by_index(uint16_t index) {
    switch (index) {
    case 0: return uart::uart_dbus.try_get();
    case 1: return uart::uart1.try_get();
    case 2: return uart::uart2.try_get();
    default: return nullptr;
    }
}

// 本板所有总线恒为 classic: 没有可切换的模式, 也没有第二个真相源可问。保留成
// 函数而非常量, 是为了让 kGetCanConfig 的写法与其它板一致(将来换控制器时只改这里)。
vc::CanMode can_mode(uint16_t index) {
    (void)index;
    return vc::CanMode::kClassic;
}

bool handle_setup(uint8_t rhport, const tusb_control_request_t* request) {
    const auto index = request->wIndex;

    switch (static_cast<vc::Request>(request->bRequest)) {
    case vc::Request::kGetInterface: {
        if (request->bmRequestType != vc::kRequestTypeIn || index != 0)
            return false;
        if (!reply(rhport, request, interface_payload()))
            return false;
        // 读接口本身就是握手: 走到这里的主机已被告知通道数与能力位, 不可能是
        // 配置迁到 EP0 之前的旧主机 —— 而旧主机正是本任务要挡的那一类: 它会跳过
        // kGetInterface 直接开会话, 其带内配置写入被板子忽略, 于是"配了但没生效"
        // 静默发生。只有此后才允许开会话, 见 Vendor::session_control_deserialized_callback()。
        usb::vendor->set_ep0_handshake_done(true);
        return true;
    }

    case vc::Request::kGetCanConfig: {
        if (request->bmRequestType != vc::kRequestTypeIn || !can_index_valid(index)) {
            record_config_error(
                vc::Request::kGetCanConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        // 硬件事实而非"上次请求"。bxCAN 的位时序由 CubeMX 生成的波特率预设决定,
        // 运行期不可改(改要进 INIT 重配, CubeMX 预设钉死), 因此这里
        // 报的是端口表的编译期常量。数据段速率与采样点对本板无意义, 恒 0。
        return reply(
            rhport, request,
            vc::CanConfigPayload{
                .mode = static_cast<uint8_t>(can_mode(index)),
                .control = 0,
                .reserved0 = 0,
                .arbitration_baudrate = 0,
                .data_baudrate = 0,
                .nominal_sample_point = 0,
                .data_sample_point = 0,
                .reserved1 = 0,
            });
    }

    case vc::Request::kGetUartConfig: {
        if (request->bmRequestType != vc::kRequestTypeIn || index >= kUartIndexCount) {
            record_config_error(
                vc::Request::kGetUartConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        const uart::Uart* port = uart_by_index(index);
        if (port == nullptr) {
            record_config_error(
                vc::Request::kGetUartConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        // 硬件事实而非"上次请求": 波特率从实际编程的 BRR 重构, 帧格式从活寄存器
        // 解码; control 恒为 0。divisor/oversample 是速率真正的整数形式, 主机靠
        // 这两个整数判定切换是否生效(它不知道本板的内核时钟, 无法复算分频器)。
        return reply(
            rhport, request,
            vc::UartConfigPayload{
                .baudrate = port->effective_baudrate(),
                .divisor = port->divisor_u16(),
                .oversample = port->oversample(),
                .word_length = static_cast<uint8_t>(port->word_length()),
                .parity = static_cast<uint8_t>(port->parity()),
                .stop_bits = static_cast<uint8_t>(port->stop_bits()),
                .control = 0,
                .reserved = 0,
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

    case vc::Request::kSetCanConfig:
        // 本板无 kCapCanModeSettable: 任何试图改模式的请求都在 data 段被拒。
        if (request->bmRequestType != vc::kRequestTypeOut || !can_index_valid(index)
            || request->wLength != sizeof(vc::CanConfigPayload)) {
            record_config_error(
                vc::Request::kSetCanConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        return tud_control_xfer(rhport, request, g_control_buffer, request->wLength);

    case vc::Request::kSetUartConfig:
        if (request->bmRequestType != vc::kRequestTypeOut || index >= kUartIndexCount
            || request->wLength != sizeof(vc::UartConfigPayload)) {
            record_config_error(
                vc::Request::kSetUartConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        if (uart_by_index(index) == nullptr) {
            record_config_error(
                vc::Request::kSetUartConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
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
        // 速率与采样点是核对不是配置: 本板 bxCAN 的位时序由 CubeMX 预设钉死,
        // 运行期改不了。非零即核对, 但本板恒为 classic 且报 0, 因此只要主机报了
        // 非零值就一定不符 —— 这正是要让它 STALL 的情形。
        if (payload.arbitration_baudrate != 0 || payload.data_baudrate != 0
            || payload.nominal_sample_point != 0 || payload.data_sample_point != 0) {
            record_config_error(
                vc::Request::kSetCanConfig, index,
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable,
                payload.arbitration_baudrate);
            return false;
        }
        // 本板不设 kCapCanModeSettable, 且硬件无 FD 能力: 请求 classic 才 ACK,
        // 请求 FD 一律 STALL(apply 位在本板无语义: 没有可切的模式)。
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
        uart::Uart* port = uart_by_index(index);
        if (port == nullptr) {
            record_config_error(
                vc::Request::kSetUartConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }

        // 先全量校验后统一提交: 帧格式先纯校验(check_framing 不碰寄存器), 波特率
        // 先解不写(solve_brr 不碰寄存器), 断言字段也全部前置, 提交放最后。因此
        // STALL 严格等于"什么都没改"。
        if (!port->check_framing(payload.word_length, payload.parity, payload.stop_bits)) {
            record_config_error(
                vc::Request::kSetUartConfig, index,
                vc::ConfigErrorReason::kConfigErrorFramingUnsupported);
            return false;
        }

        uint32_t brr = 0;
        if (payload.baudrate != 0U && !port->solve_brr(payload.baudrate, brr)) {
            record_config_error(
                vc::Request::kSetUartConfig, index,
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, payload.baudrate);
            return false;
        }
        // 宽松兜底前置到写入之前: 解出的速率离请求太远同样算不可表示。
        if (payload.baudrate != 0U && !rate_plausible(payload.baudrate, port->baudrate_for(brr))) {
            record_config_error(
                vc::Request::kSetUartConfig, index,
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, payload.baudrate);
            return false;
        }

        if ((payload.control & vc::kUartConfigApply) != 0U) {
            if (payload.baudrate != 0U) {
                // 调用方回显了分频器/过采样就要求它与求解结果相等 —— 这一步让
                // "整条身份一起断言"成立。此时还没写寄存器, 故不满足即 STALL 且无损。
                if (payload.divisor != 0U && payload.divisor != static_cast<uint16_t>(brr)) {
                    record_config_error(
                        vc::Request::kSetUartConfig, index,
                        vc::ConfigErrorReason::kConfigErrorVerifyFailed, payload.baudrate);
                    return false;
                }
                if (payload.oversample != 0U && payload.oversample != port->oversample()) {
                    record_config_error(
                        vc::Request::kSetUartConfig, index,
                        vc::ConfigErrorReason::kConfigErrorVerifyFailed, payload.baudrate);
                    return false;
                }
                port->commit_brr(payload.baudrate, brr);
                // 写入后再回读, 确认 BRR 真的收下了。走到这里失败说明硬件没有
                // 照做 —— 与"求解器拒绝"是两回事, 见 kConfigErrorVerifyFailed。
                if (!port->verify_baudrate(static_cast<uint16_t>(brr), port->oversample()))
                    [[unlikely]] {
                    record_config_error(
                        vc::Request::kSetUartConfig, index,
                        vc::ConfigErrorReason::kConfigErrorVerifyFailed, payload.baudrate);
                    return false;
                }
            } else if (payload.divisor != 0U || payload.oversample != 0U) {
                // 只给了分频器却没给速率: 没有"切换后"可言, 只能对当前值断言。
                if (payload.divisor != 0U && payload.divisor != port->divisor_u16()) {
                    record_config_error(
                        vc::Request::kSetUartConfig, index,
                        vc::ConfigErrorReason::kConfigErrorVerifyFailed, payload.divisor);
                    return false;
                }
                if (payload.oversample != 0U && payload.oversample != port->oversample()) {
                    record_config_error(
                        vc::Request::kSetUartConfig, index,
                        vc::ConfigErrorReason::kConfigErrorVerifyFailed, payload.oversample);
                    return false;
                }
            }
            port->commit_framing(payload.word_length, payload.parity, payload.stop_bits);
            // 写后回读帧格式: 主机不再回读, ACK 必须等于已生效。
            if (!framing_matches(*port, payload.word_length, payload.parity, payload.stop_bits))
                [[unlikely]] {
                record_config_error(
                    vc::Request::kSetUartConfig, index,
                    vc::ConfigErrorReason::kConfigErrorVerifyFailed);
                return false;
            }
            return true;
        }

        // 无 kUartConfigApply: 纯断言, 每个非零字段都必须与端口当前状态一致。
        // 分频器精确比对(它本就是整数), 帧格式精确比对。
        if (payload.baudrate != 0U && payload.divisor == 0U) {
            // 只给了速率没给分频器: 无法验证速率是否真的落成了那个数, 但至少
            // 要求端口当前跑的就是这个请求速率。保留一层宽松兜底抓"离谱到不可能"
            // 的值 —— 严格的判据是分频器。
            if (!rate_plausible(payload.baudrate, port->effective_baudrate())) {
                record_config_error(
                    vc::Request::kSetUartConfig, index,
                    vc::ConfigErrorReason::kConfigErrorVerifyFailed, payload.baudrate);
                return false;
            }
        }
        if (payload.divisor != 0U && payload.divisor != port->divisor_u16()) {
            record_config_error(
                vc::Request::kSetUartConfig, index, vc::ConfigErrorReason::kConfigErrorVerifyFailed,
                payload.divisor);
            return false;
        }
        if (payload.oversample != 0U && payload.oversample != port->oversample()) {
            record_config_error(
                vc::Request::kSetUartConfig, index, vc::ConfigErrorReason::kConfigErrorVerifyFailed,
                payload.oversample);
            return false;
        }
        if (!framing_matches(*port, payload.word_length, payload.parity, payload.stop_bits)) {
            record_config_error(
                vc::Request::kSetUartConfig, index,
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
