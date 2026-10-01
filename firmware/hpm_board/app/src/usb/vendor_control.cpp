#include "core/include/libhcs/protocol/vendor_control.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

#include <common/tusb_types.h>
#include <device/usbd.h>
#include <hpm_clock_drv.h>
#include <hpm_soc.h>

#include "firmware/common/app/src/usb/ep0_staging.hpp"
#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/diag/latency.hpp"
#include "firmware/hpm_board/app/src/uart/uart.hpp"
#include "firmware/hpm_board/app/src/usb/usb_descriptors.hpp"
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

// 机制部分（暂存缓冲、拒绝原因锁存、staged()/reply()）在共享头里，四块板共用；
// 这里只引入名字。策略（各请求回答什么、能力位、下标空间）留在本文件 —— 本板与
// mc02 在那几点上确实不同，见共享头的说明。
using libhcs::firmware::usb::ep0::framing_matches;
using libhcs::firmware::usb::ep0::g_control_buffer;
using libhcs::firmware::usb::ep0::g_last_config_error;
using libhcs::firmware::usb::ep0::rate_plausible;
using libhcs::firmware::usb::ep0::record_config_error;
using libhcs::firmware::usb::ep0::reply;
using libhcs::firmware::usb::ep0::staged;

namespace board = libhcs::firmware::board;

bool can_index_valid(uint16_t index) { return index < can::can_count(); }

bool uart_index_valid(uint16_t index) { return index < uart::kUartCount; }

// 该总线当前的帧型: 控制器构造前由端口表作答, 此后以运行值为准(主机可能已经 kSetCanConfig 切过)。
// 以下 bus_* 的调用方都须先过 can_index_valid(): 单 CAN 的 hpm5321 上表尾控制器焊盘实为 LED 阴极。
board::CanMode bus_mode(uint16_t index) {
    const can::Can* bus = can::can_array[index].try_get();
    return bus != nullptr ? bus->mode() : board::can_port(index).mode;
}

constexpr vc::CanMode to_wire(board::CanMode mode) {
    return mode == board::CanMode::kCanFd ? vc::CanMode::kCanFd : vc::CanMode::kClassic;
}

constexpr board::CanMode from_wire(vc::CanMode mode) {
    return mode == vc::CanMode::kCanFd ? board::CanMode::kCanFd : board::CanMode::kClassic;
}

// kGetCanConfig 的时序: 控制器已构造就是 NBTP/DBTP 重构的硬件事实, 否则为上电默认值。
can::Can::TimingIdentity bus_timing(uint16_t index) {
    const can::Can* bus = can::can_array[index].try_get();
    return bus != nullptr ? bus->timing_identity()
                          : can::Can::timing_of(can::Can::default_setting(bus_mode(index)));
}

// 板子对自身的描述, 按请求现场组装而非缓存: hpm5321 镜像服务两块 PCB,
// can_port_count() 是对板标识的运行时读取, 启动时烘焙一份等于给已有唯一来源的
// 东西再造第二个事实来源。FD 掩码报的是当前帧型, 同 mc02。
vc::InterfacePayload interface_payload() {
    vc::InterfacePayload payload{};
    payload.version = vc::kVersion;
    payload.can_count = static_cast<uint8_t>(can::can_count());
    payload.uart_count = static_cast<uint8_t>(uart::kUartCount);
    // MCAN 的 RX 元素与 TX 缓冲都配成 64 字节, 长帧双向承载
    // (can.cpp: read_uplink / handle_downlink)。帧型与速率都由主机下发, 见 kSetCanConfig。
    payload.caps = vc::kCapCanFdLongFrames | vc::kCapCanModeSettable | vc::kCapCanRateSettable;
    for (std::size_t i = 0; i < can::can_count(); ++i) {
        if (bus_mode(static_cast<uint16_t>(i)) == board::CanMode::kCanFd)
            payload.can_fd_mask |= static_cast<uint8_t>(1U << i);
    }
    return payload;
}

bool handle_setup(uint8_t rhport, const tusb_control_request_t* request) {
    const auto index = request->wIndex;

    // Windows WCID (MS OS 2.0): BOS 平台能力广告了 kMsOsVendorCode; 该码的
    // vendor IN 请求且 wIndex 0x0007 = GET_MS_OS_20_DESCRIPTOR (Microsoft 规范
    // 固定值)。集合直接从 flash 常量应答 -- 它有 848 字节, g_control_buffer 只有
    // 64, 不能走 reply() 的拷贝路径; EP0 IN 传输对 buffer 只读, 与 usbd.c 应答
    // 配置描述符(同样在 flash)一致。wLength 要求精确等于集合总长: Windows 按
    // BOS 里广告的 wDescriptorSetLength 请求, 别的长度说明对端在说别的版本。
    // 与 libhcs 的 0x40..0x47 不冲突(此码 0x21)。
    if (request->bmRequestType == vc::kRequestTypeIn
        && request->bRequest == libhcs::firmware::usb::UsbDescriptors::kMsOsVendorCode) {
        if (index != 0x0007
            || request->wLength != libhcs::firmware::usb::UsbDescriptors::kMsOs20SetLength)
            return false;
        // 记录主机类型: 只有 Windows 会取 MS OS 2.0 集, usbd_edpt_clear_stall 据此
        // 决定 CLEAR_FEATURE(HALT) 是否复位数据 toggle (Linux DMTool 失步的根因)。
        // 每次枚举 Windows 都会重取, 重复置位无害; 换主机的场景由总线复位清零。
        usbd_note_ms_os_20_fetch();
        // TinyUSB 的 tud_control_xfer 只收非 const 缓冲, 描述符本体是 constexpr 表。
        return tud_control_xfer(
            rhport, request,
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
            const_cast<uint8_t*>(libhcs::firmware::usb::UsbDescriptors::get_ms_os_20_set()),
            libhcs::firmware::usb::UsbDescriptors::kMsOs20SetLength);
    }

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
        // 硬件事实而非"上次请求": 当前帧型 + 寄存器重构的速率与采样点。经典模式下
        // 控制器关 FD, 没有数据段, 两项如实报 0。
        const auto timing = bus_timing(index);
        return reply(
            rhport, request,
            vc::CanConfigPayload{
                .mode = std::to_underlying(to_wire(bus_mode(index))),
                .control = 0,
                .reserved0 = 0,
                .arbitration_baudrate = timing.arbitration_baudrate,
                .data_baudrate = timing.data_baudrate,
                .nominal_sample_point = timing.nominal_sample_point,
                .data_sample_point = timing.data_sample_point,
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
        // 帧格式从 LCR 解码; control 恒为 0。divisor/oversample 一并上报 ——
        // 它们是速率真正的整数形式, 主机就是靠这两个整数判定切换是否生效,
        // 因为主机的内核时钟与板子不同, 无法自行复算分频器。见 UartDivisor。
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
                .rx_polarity = static_cast<uint8_t>(port->rx_polarity()),
            });
    }

    // SET 的每条 STALL 都记锁存: 主机失败时只读一次原因, 漏记会读到更早那次的陈旧原因。
    case vc::Request::kSetCanConfig:
        if (request->bmRequestType != vc::kRequestTypeOut || !can_index_valid(index)
            || request->wLength != sizeof(vc::CanConfigPayload)) {
            record_config_error(
                vc::Request::kSetCanConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
        // 先接受数据段, 值等到达后再校验。
        return tud_control_xfer(rhport, request, g_control_buffer, request->wLength);

    case vc::Request::kSetUartConfig:
        if (request->bmRequestType != vc::kRequestTypeOut || !uart_index_valid(index)
            || request->wLength != sizeof(vc::UartConfigPayload)
            || uart::uart_array[index].try_get() == nullptr) {
            record_config_error(
                vc::Request::kSetUartConfig, index, vc::ConfigErrorReason::kConfigErrorBadIndex);
            return false;
        }
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
        const auto reject = [&](vc::ConfigErrorReason reason, uint32_t value = 0) {
            record_config_error(vc::Request::kSetCanConfig, index, reason, value);
            return false;
        };
        if (payload.mode > static_cast<uint8_t>(vc::CanMode::kCanFd))
            return reject(vc::ConfigErrorReason::kConfigErrorBadRequest, payload.mode);
        if ((payload.control & ~(vc::kCanConfigApply | vc::kCanConfigApplyTiming)) != 0U)
            return reject(vc::ConfigErrorReason::kConfigErrorBadRequest, payload.control);
        can::Can* bus = can::can_array[index].try_get();
        if (bus == nullptr)
            return reject(vc::ConfigErrorReason::kConfigErrorBadIndex);

        const bool apply_mode = (payload.control & vc::kCanConfigApply) != 0U;
        const bool apply_timing = (payload.control & vc::kCanConfigApplyTiming) != 0U;
        const board::CanMode want = from_wire(static_cast<vc::CanMode>(payload.mode));

        // 目标设置: 在 libhcs 当前设置上改帧型; 带 kCanConfigApplyTiming 时速率字段是设置
        // (非零即新速率, 零即沿用), 由 host 下发 -- 速率是接线事实, 固件不该写死。
        can::Can::BusSetting target = bus->libhcs_setting();
        target.mode = want;
        if (apply_timing) {
            if (payload.arbitration_baudrate != 0U)
                target.arbitration_baudrate = payload.arbitration_baudrate;
            if (payload.data_baudrate != 0U) {
                // 经典总线没有数据段: 给经典目标下发数据段速率即预期有误。
                if (want == board::CanMode::kClassic)
                    return reject(
                        vc::ConfigErrorReason::kConfigErrorUnsupportedMode, payload.data_baudrate);
                target.data_baudrate = payload.data_baudrate;
            }
        }

        // 核对字段: 采样点永远是核对(本板钉死 87.5%); 速率只在不带 kCanConfigApplyTiming 时
        // 是核对。基准: 纯核对且帧型不变时比硬件事实, 否则比目标设置应用后应有的时序。
        const auto expected = (!apply_timing && want == bus_mode(index))
                                ? bus_timing(index)
                                : can::Can::timing_of(target);
        const auto asserted_ok = [&](uint32_t asserted, uint32_t actual) {
            return asserted == 0U || asserted == actual
                || reject(vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, asserted);
        };
        if ((!apply_timing
             && (!asserted_ok(payload.arbitration_baudrate, expected.arbitration_baudrate)
                 || !asserted_ok(payload.data_baudrate, expected.data_baudrate)))
            || !asserted_ok(payload.nominal_sample_point, expected.nominal_sample_point)
            || !asserted_ok(payload.data_sample_point, expected.data_sample_point))
            return false;

        // 帧型只在带 kCanConfigApply 时改, 否则必须与当前一致。
        if (!apply_mode && want != bus_mode(index))
            return reject(vc::ConfigErrorReason::kConfigErrorUnsupportedMode, payload.mode);
        if (!apply_mode && !apply_timing)
            return true;

        // 与 mc02 不同, 本板切帧型/速率要重跑控制器初始化: 经典必须关掉控制器 FD 能力, 由硬件
        // 保证 2.0 总线上不出现任何 FD 位, 而不是只翻 Tx 元素的 FDF。设置与现状一致时不重初始化
        // (只记下 libhcs 的选择, 会话建立时按它还原)。排队中的旧帧作废 -- 静默链路后再切的
        // 义务在主机, 与波特率切换同一约定。
        if (!bus->apply_setting(target)) {
            const bool data_changed =
                want == board::CanMode::kCanFd && apply_timing && payload.data_baudrate != 0U;
            return reject(
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable,
                data_changed ? target.data_baudrate : target.arbitration_baudrate);
        }
        // 写后回读: 主机不再回读, ACK 前确认帧型、两段速率与采样点都已如所求。
        if (bus_mode(index) != want || bus_timing(index) != can::Can::timing_of(target))
            [[unlikely]]
            return reject(vc::ConfigErrorReason::kConfigErrorVerifyFailed, payload.mode);
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
        // 切波特率(set_baudrate 内部先求解、求解失败即在碰任何寄存器之前返回,
        // 连在途 TX DMA 都不拆), 最后落帧格式。因此 STALL 严格等于"什么都没改"。
        //
        // divisor/oversample 是**断言**不是指令: 非零就必须与切换后实际写入的
        // 一致。主机先 GET 再 SET 同样的值, 就是在一次传输里断言整个端口的电气
        // 身份 —— 与 kSetCanConfig 的时间字段同一性质。波特率不再用百分比容差
        // 比对: 请求值本就不是求解器的不动点(921600 在 80 MHz 上得 909090),
        // 只有分频器整数是可比的事实。见 UartDivisor。
        if (!port->check_framing(
                payload.word_length, payload.parity, payload.stop_bits, payload.rx_polarity)) {
            record_config_error(
                vc::Request::kSetUartConfig, index,
                vc::ConfigErrorReason::kConfigErrorFramingUnsupported);
            return false;
        }

        if ((payload.control & vc::kUartConfigApply) != 0U) {
            // 求解与断言全部前置到动寄存器之前, 写入放最后。这是本板能做到的最
            // 强形态: solve_divisor() 是纯函数(复制自 SDK 的求解器, 见
            // uart.hpp), 因此"求解器拒绝"与"调用方回显的期望不符"这两类 STALL
            // 都发生在拆 TX DMA 之前 —— 一个寄存器、一个在途字节都没动。
            uint16_t want_divisor = 0;
            uint8_t want_oversample = 0;
            if (payload.baudrate != 0U) {
                if (!uart::Uart::solve_divisor(
                        port->clock_hz(), payload.baudrate, want_divisor, want_oversample)) {
                    record_config_error(
                        vc::Request::kSetUartConfig, index,
                        vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, payload.baudrate);
                    return false;
                }
                // 宽松兜底前置到写入之前: 解出的速率离请求太远同样算不可表示。
                if (!rate_plausible(
                        payload.baudrate, port->baudrate_for(want_divisor, want_oversample))) {
                    record_config_error(
                        vc::Request::kSetUartConfig, index,
                        vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, payload.baudrate);
                    return false;
                }
                // 调用方回显了分频器/过采样就要求它与求解结果相等 —— 这一步让
                // "整条身份一起断言"成立: 主机说不出自己期望的分频器, 就不该
                // 假装验证过速率。此时还没写任何寄存器, 故不满足即 STALL 且无损。
                if (payload.divisor != 0U && payload.divisor != want_divisor) {
                    record_config_error(
                        vc::Request::kSetUartConfig, index,
                        vc::ConfigErrorReason::kConfigErrorVerifyFailed, payload.baudrate);
                    return false;
                }
                if (payload.oversample != 0U && payload.oversample != want_oversample) {
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

            // 校验全部通过, 开始提交。
            if (payload.baudrate != 0U) {
                // set_baudrate 内部会再解一次(同一求解器), 因此这里不会再失败;
                // 仍保留判断以免将来 SDK 求解器与复制品分道扬镳时静默继续。
                if (!port->set_baudrate(payload.baudrate)) [[unlikely]] {
                    record_config_error(
                        vc::Request::kSetUartConfig, index,
                        vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, payload.baudrate);
                    return false;
                }
                // 写入后再回读, 确认 DLL/DLM/OSCR 真的收下了。走到这里失败说明
                // 硬件没有照做 —— 与"求解器拒绝"是两回事, 见 kConfigErrorVerifyFailed。
                if (!port->verify_baudrate(want_divisor, want_oversample)) [[unlikely]] {
                    record_config_error(
                        vc::Request::kSetUartConfig, index,
                        vc::ConfigErrorReason::kConfigErrorVerifyFailed, payload.baudrate);
                    return false;
                }
            }
            port->commit_framing(payload.word_length, payload.parity, payload.stop_bits);
            // 写后回读帧格式: 主机不再回读, ACK 必须等于已生效。
            if (!framing_matches(
                    *port, payload.word_length, payload.parity, payload.stop_bits,
                    payload.rx_polarity)) [[unlikely]] {
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
            // 的值 —— 严格的判据是分频器, 这条只拦明显不在同一量级的输入。
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
        // 帧格式断言不符也要记锁存, 与 mc02 / c_board 一致。
        if (!framing_matches(
                *port, payload.word_length, payload.parity, payload.stop_bits,
                payload.rx_polarity)) {
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
