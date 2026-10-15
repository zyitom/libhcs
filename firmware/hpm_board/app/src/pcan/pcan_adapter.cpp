#include "firmware/hpm_board/app/src/pcan/pcan_adapter.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>

#include <device/usbd_pvt.h>
#include <hpm_otp_drv.h>
#include <hpm_ptpc_drv.h>
#include <hpm_soc.h>
#include <tusb.h>

#include "board_app.hpp"
#include "core/include/libhcs/protocol/can_dlc.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"
#include "firmware/common/app/src/utility/ring_buffer.hpp"
#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/led/led.hpp"

// PCAN-USB FD 仿真的实现: USB 应用类驱动、uCAN 命令分发、CAN 收发与记录流。
// 协议格式见 pcan_protocol.hpp, 与默认人格(DMTool + libhcs)的关系见 pcan_adapter.hpp。
//
// 板子是纯转发桥(仓库根 AGENTS.md「架构边界」)。凡是要板子自己做事的 PCAN 命令
// -- 设备端自发自收、总线负载统计、写持久 device id -- 一律不做; 查询类回报
// 硬件事实。SINGLE_SHOT 是逐帧标志而 MCAN 的自动重传是控制器级开关, 不逐帧
// 跟随(PCAN_PROTOCOL.md 记录该限制); 本路径沿用 reconfigure_timing 的自动重传
// 开(与真实适配器默认一致, 也与 DMTool 路径同一取舍)。

namespace libhcs::firmware::pcan {

namespace {

using protocol::Opcode;

// PTPC 原始 {秒, 纳秒} 换成真实纳秒(数字模式步长校正, 与 CAN 驱动的
// kCanTimestampNsPerUs 同一修正); 再除 1000 得 PCAN 的 us 时间戳。64 位除法,
// 只在主循环做 -- 与 dm_adapter.cpp 的 to_real_ns 同一取舍。
constexpr uint64_t to_real_ns(uint32_t sec, uint32_t ns) {
    constexpr uint64_t divisor = board::kCanTimestampNsPerUs;
    const uint64_t raw = (uint64_t{sec} * 1'000'000'000U) + ns;
    return (raw / divisor * 1000U) + (raw % divisor * 1000U / divisor);
}

constexpr uint64_t to_timestamp_us(uint32_t sec, uint32_t ns) {
    return to_real_ns(sec, ns) / 1000U;
}

// 此刻的 PTPC0 时间, 与 CAN 接收时间戳同一时基(CALIBRATION 记录用)。秒在两次
// 读之间进位时重读, 保证 {秒, 纳秒} 属于同一时刻。
uint64_t now_timestamp_us() {
    uint32_t sec = ptpc_get_timestamp_second(HPM_PTPC, PTPC_PTPC_0);
    while (true) {
        const uint32_t ns = ptpc_get_timestamp_ns(HPM_PTPC, PTPC_PTPC_0);
        const uint32_t sec_after = ptpc_get_timestamp_second(HPM_PTPC, PTPC_PTPC_0);
        if (sec_after == sec)
            return to_timestamp_us(sec, ns);
        sec = sec_after;
    }
}

// PCAN 通道即本板 CAN 序号; 单 CAN 板上通道 1 不存在(命令作废, 帧丢弃)。
can::Can* channel_can(uint8_t channel) {
    if (channel >= kChannelCount || channel >= can::can_count())
        return nullptr;
    return can::can_array[channel].try_get();
}

// 设备 32 位序列号: OTP UUID 折叠一次, 主循环上下文首次调用起缓存。
// iSerialNumber(十进制串)与 fw_info.ser_no 两处同源。
uint32_t computed_serial_number() {
    static uint32_t serial = [] {
        uint32_t value = 0;
        for (uint32_t i = 0; i < 4; ++i)
            value = (value * 0x9E3779B1U) ^ otp_read_from_shadow(OTP_SOC_UUID_IDX + i);
        // 高位保非零, 十进制串至少一位且不超过 u32 的 10 位。
        return value | 0x8000'0000U;
    }();
    return serial;
}

// 每通道 32 位 device id(DEVID_SET 只改 RAM; 上电默认全 1, 与真卡一致)。
// 只在主循环读写(命令分发与 EP0 应答)。
std::array<uint32_t, kChannelCount>& device_ids() {
    static std::array<uint32_t, kChannelCount> ids{0xFFFFFFFFU, 0xFFFFFFFFU};
    return ids;
}

// ---- USB 端点缓冲(非缓存段, 与 TinyUSB 类驱动同一放法) ----

struct EndpointBuffers {
    TUD_EPBUF_DEF(cmd_out, CFG_TUD_VENDOR_EPSIZE);
    TUD_EPBUF_DEF(data_out0, CFG_TUD_VENDOR_EPSIZE);
    TUD_EPBUF_DEF(data_out1, CFG_TUD_VENDOR_EPSIZE);
    TUD_EPBUF_DEF(data_in, CFG_TUD_VENDOR_EPSIZE);
};
CFG_TUD_MEM_SECTION EndpointBuffers g_endpoint_buffers;

class Adapter : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Adapter>;

    Adapter() {
        for (auto& channel : channels_)
            channel.filter.fill(0xFFFFFFFFU);
    }

    // ---- USB 应用类驱动(tud_task 里调用) ----

    // 只认接口 0(uCAN vendor); DFU runtime 接口交给内建驱动。
    uint16_t open(uint8_t rhport, const tusb_desc_interface_t* interface, uint16_t max_len) {
        if (interface->bInterfaceClass != TUSB_CLASS_VENDOR_SPECIFIC
            || interface->bInterfaceNumber != 0)
            return 0;

        rhport_ = rhport;
        const auto* end = reinterpret_cast<const uint8_t*>(interface) + max_len;
        const uint8_t* descriptor = tu_desc_next(interface);
        while (tu_desc_in_bounds(descriptor, end)) {
            const uint8_t type = tu_desc_type(descriptor);
            if (type == TUSB_DESC_INTERFACE || type == TUSB_DESC_INTERFACE_ASSOCIATION)
                break;
            if (type == TUSB_DESC_ENDPOINT) {
                const auto* endpoint = reinterpret_cast<const tusb_desc_endpoint_t*>(descriptor);
                if (!usbd_edpt_open(rhport, endpoint))
                    return 0;
                const uint8_t address = endpoint->bEndpointAddress;
                const auto packet_size = tu_edpt_packet_size(endpoint);
                if (address == protocol::kEpCmdOut) {
                    cmd_out_ = packet_size;
                } else if (address == protocol::kEpDataIn) {
                    data_in_ = packet_size;
                } else if (address == protocol::kEpDataOut0 || address == protocol::kEpDataOut1) {
                    data_out_[address == protocol::kEpDataOut0 ? 0 : 1] = packet_size;
                }
                // kEpCmdIn(0x81): Linux 不读, 不挂接收; 应答一律走 data IN。
            }
            descriptor = tu_desc_next(descriptor);
        }

        opened_ = true;
        arm_out(protocol::kEpCmdOut);
        arm_out(protocol::kEpDataOut0);
        arm_out(protocol::kEpDataOut1);
        publish();
        return static_cast<uint16_t>(descriptor - reinterpret_cast<const uint8_t*>(interface));
    }

    void on_transfer(uint8_t endpoint_address, xfer_result_t result, uint32_t size) {
        if (tu_edpt_dir(endpoint_address) == TUSB_DIR_IN)
            return; // data IN 空出: 下一轮 poll() 自会续上
        if (result != XFER_RESULT_SUCCESS) {
            arm_out(endpoint_address);
            return;
        }
        if (endpoint_address == protocol::kEpCmdOut) {
            on_command_packet(std::span{g_endpoint_buffers.cmd_out}.first(
                std::min<std::size_t>(size, CFG_TUD_VENDOR_EPSIZE)));
        } else if (endpoint_address == protocol::kEpDataOut0) {
            on_tx_packet(
                0, std::span{g_endpoint_buffers.data_out0}.first(
                       std::min<std::size_t>(size, CFG_TUD_VENDOR_EPSIZE)));
        } else if (endpoint_address == protocol::kEpDataOut1) {
            on_tx_packet(
                1, std::span{g_endpoint_buffers.data_out1}.first(
                       std::min<std::size_t>(size, CFG_TUD_VENDOR_EPSIZE)));
        }
        arm_out(endpoint_address);
    }

    void on_usb_reset() {
        opened_ = false;
        cmd_out_ = 0;
        data_in_ = 0;
        data_out_ = {};
        reset();
    }

    // ---- EP0(pcan_control.cpp 的 DRVLD) ----

    void set_driver_loaded(bool loaded) { driver_loaded_ = loaded; }

    // LED 判据: PCAN 的"驱动已加载且至少一路总线上线"。
    [[nodiscard]] bool session_established() const {
        return driver_loaded_ && internal::g_rx_mask.load(std::memory_order::relaxed) != 0;
    }

    // ---- ISR 侧 ----

    void push_can_rx(std::size_t can_index, const RxEvent& event) {
        if (can_index >= channels_.size())
            return;
        // 标准帧先过接收位图(位 = 1 收); 扩展帧位图不覆盖, 直接收 -- 与真实硬件
        // 一致。位图由主循环整字更新, 这里整字读。
        if (!event.extended
            && ((channels_[can_index].filter[event.id >> 5] & (1U << (event.id & 31U))) == 0U))
            return;
        if (!rx_queues_[can_index].push_back(event)) {
            overruns_[can_index].fetch_add(1, std::memory_order::relaxed);
            led::led->uplink_buffer_full();
        }
    }

    // ---- 主循环 ----

    void poll() {
        if (!opened_ || !in_ready())
            return;

        const auto out = std::span{g_endpoint_buffers.data_in}.first(data_in_ - 1U);
        std::size_t used = 0;

        used = emit_periodic(out, used);
        used = emit_rx_frames(out, used);
        used = emit_overruns(out, used);

        if (used != 0)
            send_in(used);
    }

    // 1 kHz tick 块(pcan::poll_tick): 周期记录的排期与状态变化检测。
    void poll_tick(uint32_t tick) {
        if ((tick % kCalibrationPeriodTicks) != 0U)
            return;
        if (!opened_)
            return;

        bool pending = false;
        if (options_calibration_) {
            calibration_due_ = internal::g_rx_mask.load(std::memory_order::relaxed) != 0;
            pending |= calibration_due_;
        }

        // ERROR 选项开着的通道: 错计数变化发 ERROR 记录(内核的 get_berr_counter
        // 依赖它), warning/passive/busoff 变化发 STATUS 记录。
        for (std::size_t i = 0; i < channels_.size(); ++i) {
            auto& channel = channels_[i];
            if (!channel.error_option || channel.mode == ChannelMode::kReset)
                continue;
            can::Can* can = channel_can(static_cast<uint8_t>(i));
            if (can == nullptr)
                continue;
            const auto status = can->read_status();
            if (status.tec != channel.tec || status.rec != channel.rec) {
                channel.tec = status.tec;
                channel.rec = status.rec;
                channel.error_pending = true;
                pending = true;
            }
            const uint8_t bits =
                status.flags & (data::kCanErrorPassive | data::kCanWarning | data::kCanBusOff);
            if (!channel.status_valid || bits != channel.status_bits) {
                channel.status_valid = true;
                channel.status_bits = bits;
                channel.status_pending = true;
                pending = true;
            }
        }
        if (pending)
            publish();
    }

    // 总线复位: 回到"未会话"。协议状态全清; 控制器电气状态不动 -- 真实适配器在
    // 主机意外拔出后同样保持原样, 新主机的 ifup 序列会整体重配。
    void reset() {
        for (auto& channel : channels_) {
            channel.mode = ChannelMode::kReset;
            channel.timing_known = false;
            channel.data_known = false;
            channel.fd = false;
            channel.non_iso = false;
            channel.error_option = false;
            channel.error_pending = false;
            channel.status_pending = false;
            channel.barrier_pending = false;
            channel.status_valid = false;
            channel.status_bits = 0;
            channel.tec = 0;
            channel.rec = 0;
            channel.filter.fill(0xFFFFFFFFU);
        }
        options_calibration_ = false;
        calibration_due_ = false;
        for (auto& queue : rx_queues_)
            (void)queue.clear();
        for (auto& overrun : overruns_)
            overrun.store(0, std::memory_order::relaxed);
        for (auto& seen : overrun_seen_)
            seen = 0;
        led::led->set_forced_pattern(led::ForcedPattern::kNone);
        for (uint8_t channel = 0; channel < kChannelCount; ++channel)
            led::led->set_channel_active(channel, false);
        publish();
    }

private:
    enum class ChannelMode : uint8_t {
        kReset,  // RESET_MODE: 控制器在 init 模式, 不上线(上电默认, 与真卡一致)
        kNormal, // NORMAL_MODE
        kListenOnly // LISTEN_ONLY_MODE(MCAN CCCR.MON, 不应答)
    };

    struct Channel {
        ChannelMode mode = ChannelMode::kReset;
        bool timing_known = false; // 收到过 TIMING_SLOW
        bool data_known = false;   // 收到过 TIMING_FAST(FD; 否则按经典模式配置)
        bool fd = false;           // 最近一次应用的总线帧型(data_known 的快照)
        bool non_iso = false;      // CANDFDISO 选项被清(= non-ISO)
        bool error_option = false; // uCAN ERROR 选项
        // TIMING 命令存这里, NORMAL/LISTEN 上线时应用; 数据段字段仅在 fd 时使用。
        can::Can::PhaseTiming nominal{};
        can::Can::PhaseTiming data{};
        // 周期检测的缓存(read_status 的快照)。
        bool error_pending = false;
        bool status_pending = false;
        bool barrier_pending = false;
        bool status_valid = false;
        uint8_t status_bits = 0;
        uint8_t tec = 0;
        uint8_t rec = 0;
        // 标准 ID 接收位图(internal::g_filter 的主循环侧真值; 拷贝一份在 Adapter
        // 里同步写, 两处同值 -- g_filter 是 ISR 读的那份)。
        std::array<uint32_t, kFilterRows> filter{};
    };

    static constexpr uint32_t kCalibrationPeriodTicks = 100; // ~100 ms

    // ---- 端点操作 ----

    void arm_out(uint8_t address) {
        uint8_t* buffer = nullptr;
        uint16_t size = 0;
        if (address == protocol::kEpCmdOut) {
            buffer = g_endpoint_buffers.cmd_out;
            size = cmd_out_;
        } else if (address == protocol::kEpDataOut0) {
            buffer = g_endpoint_buffers.data_out0;
            size = data_out_[0];
        } else if (address == protocol::kEpDataOut1) {
            buffer = g_endpoint_buffers.data_out1;
            size = data_out_[1];
        }
        if (buffer == nullptr || size == 0)
            return;
        (void)usbd_edpt_xfer(rhport_, address, buffer, size, false);
    }

    [[nodiscard]] bool in_ready() const {
        return opened_ && data_in_ != 0 && !usbd_edpt_busy(rhport_, protocol::kEpDataIn);
    }

    void send_in(std::size_t size) {
        if (!usbd_edpt_claim(rhport_, protocol::kEpDataIn)) [[unlikely]]
            return;
        (void)usbd_edpt_xfer(
            rhport_, protocol::kEpDataIn, g_endpoint_buffers.data_in, static_cast<uint16_t>(size),
            false);
    }

    // ---- 命令流 ----

    void on_command_packet(std::span<const uint8_t> bytes) {
        protocol::for_each_command(
            bytes, [this](const protocol::Command& command) { handle_command(command); });
    }

    void handle_command(const protocol::Command& command) {
        const uint8_t channel = protocol::channel_of(command.opcode_channel);
        const auto opcode = static_cast<Opcode>(protocol::opcode_of(command.opcode_channel));

        switch (opcode) {
        case Opcode::kResetMode: set_channel_mode(channel, ChannelMode::kReset); return;
        case Opcode::kNormalMode: set_channel_mode(channel, ChannelMode::kNormal); return;
        case Opcode::kListenOnlyMode: set_channel_mode(channel, ChannelMode::kListenOnly); return;
        case Opcode::kTimingSlow: {
            const auto timing = protocol::decode_timing_slow(command.args);
            if (channel < channels_.size()) {
                channels_[channel].nominal = {
                    .prescaler = timing.timing.prescaler,
                    .seg1 = timing.timing.seg1,
                    .seg2 = timing.timing.seg2,
                    .sjw = timing.timing.sjw,
                };
                channels_[channel].timing_known = true;
                apply_bus_state(channel);
            }
            return;
        }
        case Opcode::kTimingFast: {
            const auto timing = protocol::decode_timing_fast(command.args);
            if (channel < channels_.size()) {
                channels_[channel].data = {
                    .prescaler = timing.timing.prescaler,
                    .seg1 = timing.timing.seg1,
                    .seg2 = timing.timing.seg2,
                    .sjw = timing.timing.sjw,
                };
                channels_[channel].data_known = true;
                apply_bus_state(channel);
            }
            return;
        }
        case Opcode::kSetStdFilter:
        case Opcode::kFilterStd: {
            const auto mask = opcode == Opcode::kSetStdFilter
                                ? protocol::decode_set_std_filter(command.args)
                                : protocol::decode_filter_std(command.args);
            apply_filter(channel, mask);
            return;
        }
        case Opcode::kTxAbort:
            // 软件发送队列由主循环每轮排空, 几乎恒空; 作废语义由 MCAN 的发送
            // 取消与 TXBCF 计数承担, 这里无事可做。
            return;
        case Opcode::kWrErrCnt: {
            if (can::Can* can = channel_can(channel))
                can->clear_error_counters();
            return;
        }
        case Opcode::kSetEnOption:
        case Opcode::kClrDisOption: {
            const auto options = protocol::decode_options(command.args);
            apply_options(channel, options, opcode == Opcode::kSetEnOption);
            return;
        }
        case Opcode::kRxBarrier:
            if (channel < channels_.size()) {
                channels_[channel].barrier_pending = true;
                publish();
            }
            return;
        case Opcode::kClockSet:
            // 本板恒 80 MHz(fw_info 与 CLK_SET 声明一致), 模式只存不生效。
            return;
        case Opcode::kDeviceIdSet:
            if (channel < kChannelCount)
                device_ids()[channel] = protocol::load32(command.args.data() + 2);
            return;
        case Opcode::kLedSet: {
            using led::ForcedPattern;
            switch (command.args[0]) {
            case 1: led::led->set_forced_pattern(ForcedPattern::kBlinkFast); break;
            case 2: led::led->set_forced_pattern(ForcedPattern::kBlinkSlow); break;
            case 3: led::led->set_forced_pattern(ForcedPattern::kOn); break;
            case 4: led::led->set_forced_pattern(ForcedPattern::kOff); break;
            default: led::led->set_forced_pattern(ForcedPattern::kNone); break; // 0 = 设备态
            }
            return;
        }
        default: return; // 未知命令: 真实固件的宽容行为是忽略
        }
    }

    // 模式切换 + 按当前意图重摆控制器。RESET: 撤下总线; NORMAL/LISTEN: 按存的
    // 时序重配(带新的 FD/ISO 意图)再上线; 时序未知(主机还没发 TIMING)时按上电
    // 默认配置直接上线, 与真卡一致。
    void set_channel_mode(uint8_t channel, ChannelMode mode) {
        if (channel >= channels_.size())
            return;
        channels_[channel].mode = mode;
        apply_bus_state(channel);
        led::led->set_channel_active(channel, mode != ChannelMode::kReset);
        publish();
    }

    void apply_bus_state(uint8_t channel) {
        can::Can* can = channel_can(channel);
        if (can == nullptr)
            return;
        auto& state = channels_[channel];

        if (state.mode == ChannelMode::kReset) {
            can->set_bus_off();
            return;
        }
        if (state.timing_known) {
            const bool fd = state.data_known;
            // TIMING 的 10 位分频超出 MCAN 的 9 位字段(80 MHz 下低于 ~5 kbps 才
            // 可能): 拒收这组时序, 保持旧配置 -- 总线照常上线。
            const auto fits = [](const can::Can::PhaseTiming& phase) {
                return phase.prescaler != 0 && phase.prescaler <= 512U && phase.seg1 != 0
                    && phase.seg2 != 0 && phase.sjw != 0;
            };
            if (fits(state.nominal) && (!fd || fits(state.data)))
                (void)can->reconfigure_timing(fd, state.nominal, state.data);
            state.fd = fd;
        }
        can->set_bus_on(state.mode == ChannelMode::kListenOnly, state.non_iso && state.fd);
    }

    void apply_filter(uint8_t channel, const protocol::FilterMask& mask) {
        if (channel >= channels_.size())
            return;
        const uint16_t first = mask.row < kFilterRows ? mask.row : 0;
        const uint16_t last = mask.row < kFilterRows ? mask.row + 1U : kFilterRows;
        for (uint16_t row = first; row < last; ++row) {
            channels_[channel].filter[row] = mask.mask;
            internal::g_filter[channel][row] = mask.mask;
        }
    }

    void apply_options(uint8_t channel, const protocol::Options& options, bool enable) {
        if ((options.usb & protocol::kUsbOptionCalibration) != 0)
            options_calibration_ = enable;

        if (channel >= channels_.size())
            return;
        auto& state = channels_[channel];

        if ((options.ucan & protocol::kOptionError) != 0) {
            state.error_option = enable;
            if (!enable) {
                state.error_pending = false;
                state.status_pending = false;
                state.status_valid = false;
            }
        }
        if ((options.ucan & protocol::kOptionCanFdIso) != 0) {
            state.non_iso = !enable; // 选项置位 = ISO 模式, 清除 = non-ISO
            // 在线状态下切 ISO: set_bus_on 的 CCCR 往返短暂离线, 与真实固件改
            // CCCR 的形态一致。
            if (state.mode != ChannelMode::kReset)
                apply_bus_state(channel);
        }
        // kOptionBusload: 总线负载统计不做(内核不发; PCAN_PROTOCOL.md 记录)。
    }

    // ---- TX ----

    // 一包一串记录, 逐帧转发。队列满丢弃并点 LED(与 DMTool 路径同一决策:
    // 背压 hold 在不应答的总线上会永久卡死端点)。
    void on_tx_packet(std::size_t channel, std::span<const uint8_t> bytes) {
        auto rest = bytes;
        while (const auto request = protocol::parse_tx_frame(rest)) {
            transmit(static_cast<uint8_t>(channel), *request);
            rest = rest.subspan(request->encoded_size);
        }
    }

    void transmit(uint8_t channel, const protocol::TxFrame& request) {
        can::Can* can = channel_can(channel);
        const uint8_t dlc = request.channel_dlc >> 4;
        const auto payload = std::min<std::size_t>(
            core::protocol::payload_length(dlc), std::min<std::size_t>(request.data.size(), 64));

        const bool extended = (request.flags & protocol::kTxFlagExtId) != 0;
        const bool id_valid = extended || request.can_id <= 0x7FFU;
        if (can == nullptr || !id_valid)
            return;

        const bool ok = can->handle_downlink_as(
            {
                .can_id = request.can_id,
                .can_data = std::as_bytes(request.data.first(payload)),
                .is_extended_can_id = extended,
                .is_remote_transmission = (request.flags & protocol::kTxFlagRtr) != 0,
            },
            {
                .wire_dlc = dlc, // 长帧 DLC 9-15 直传
                .fd = (request.flags & protocol::kTxFlagExtDataLen) != 0,
                .bitrate_switch = (request.flags & protocol::kTxFlagBitrateSwitch) != 0,
            });
        if (!ok)
            led::led->downlink_buffer_full();
        // SINGLE_SHOT 逐帧标志不跟随(MCAN 的重传开关是控制器级的), ESI 由硬件
        // 生成, SELF_RECEIVE 回显默认关(见 PCAN_PROTOCOL.md)。
    }

    // ---- 记录流泵 ----

    // 一次 IN 传输严格小于端点包长(短包结尾; 内核对满包结尾的传输要等下一包),
    // 且记录绝不跨传输(内核遇跨 URB 记录报错)。encode_* 放不下时返回 0, 剩余
    // 记录留在队列里等下一轮。
    std::size_t emit_periodic(std::span<uint8_t> out, std::size_t used) {
        if (calibration_due_) {
            const std::size_t written = protocol::encode_calibration(
                {.timestamp_us = now_timestamp_us()}, out.subspan(used));
            if (written == 0)
                return used;
            used += written;
            calibration_due_ = false;
        }
        for (std::size_t i = 0; i < channels_.size(); ++i) {
            auto& channel = channels_[i];
            const uint8_t bits = static_cast<uint8_t>(i);

            if (channel.barrier_pending) {
                const std::size_t written = protocol::encode_status(
                    {.timestamp_us = now_timestamp_us(),
                     .channel_bits = static_cast<uint8_t>(bits | protocol::kStatusRxBarrier)},
                    out.subspan(used));
                if (written == 0)
                    return used;
                used += written;
                channel.barrier_pending = false;
            }
            if (channel.status_pending) {
                const std::size_t written = protocol::encode_status(
                    {.timestamp_us = now_timestamp_us(),
                     .channel_bits = static_cast<uint8_t>(bits | channel.status_bits)},
                    out.subspan(used));
                if (written == 0)
                    return used;
                used += written;
                channel.status_pending = false;
            }
            if (channel.error_pending) {
                const std::size_t written = protocol::encode_error(
                    {
                        .timestamp_us = now_timestamp_us(),
                        .channel = bits,
                        .tx_err_cnt = channel.tec,
                        .rx_err_cnt = channel.rec,
                    },
                    out.subspan(used));
                if (written == 0)
                    return used;
                used += written;
                channel.error_pending = false;
            }
        }
        return used;
    }

    std::size_t emit_rx_frames(std::span<uint8_t> out, std::size_t used) {
        for (std::size_t channel = 0; channel < rx_queues_.size(); ++channel) {
            auto& queue = rx_queues_[channel];
            while (const RxEvent* event = queue.peek_front()) {
                uint16_t flags = 0;
                if (event->extended)
                    flags |= protocol::kRxFlagExtId;
                if (event->remote)
                    flags |= protocol::kRxFlagRtr;
                if (event->fd)
                    flags |= protocol::kRxFlagExtDataLen;
                if (event->bitrate_switch)
                    flags |= protocol::kRxFlagBitrateSwitch;
                if (event->error_state)
                    flags |= protocol::kRxFlagErrorStateInd;

                const auto payload = std::min<std::size_t>(
                    core::protocol::payload_length(event->dlc), event->data.size());
                const std::size_t written = protocol::encode_can_rx(
                    {
                        .timestamp_us = to_timestamp_us(event->timestamp_sec, event->timestamp_ns),
                        .can_id = event->id,
                        .data = std::span{event->data}.first(payload),
                        .channel_dlc = static_cast<uint8_t>(channel | (event->dlc << 4)),
                        .flags = flags,
                    },
                    out.subspan(used));
                if (written == 0)
                    return used; // 放不下: 留在队列等下一轮
                used += written;
                queue.pop_front([](const RxEvent&) noexcept {});
            }
        }
        return used;
    }

    std::size_t emit_overruns(std::span<uint8_t> out, std::size_t used) {
        for (std::size_t i = 0; i < overruns_.size(); ++i) {
            const uint32_t count = overruns_[i].load(std::memory_order::relaxed);
            if (count == overrun_seen_[i])
                continue;
            overrun_seen_[i] = count;
            const std::size_t written = protocol::encode_overrun(
                {.timestamp_us = now_timestamp_us(), .channel = static_cast<uint8_t>(i)},
                out.subspan(used));
            if (written == 0)
                return used;
            used += written;
        }
        return used;
    }

    // 把主循环侧状态发布给 ISR 与主循环闸(pcan_adapter.hpp)。
    void publish() const {
        uint8_t rx_mask = 0;
        uint8_t service = 0;
        if (opened_) {
            for (std::size_t i = 0; i < channels_.size(); ++i) {
                if (channels_[i].mode != ChannelMode::kReset)
                    rx_mask |= 1U << i;
            }
        }
        if (rx_mask != 0)
            service |= kServiceRx;
        if (calibration_due_)
            service |= kServicePeriodic;
        for (const auto& channel : channels_) {
            if (channel.error_pending || channel.status_pending || channel.barrier_pending) {
                service |= kServicePeriodic;
                break;
            }
        }
        internal::g_rx_mask.store(rx_mask, std::memory_order::release);
        internal::g_service.store(service, std::memory_order::release);
    }

    uint8_t rhport_ = 0;
    bool opened_ = false;
    uint16_t cmd_out_ = 0;
    uint16_t data_in_ = 0;
    std::array<uint16_t, 2> data_out_{};

    std::array<Channel, kChannelCount> channels_{};

    // 1 Mbit 经典帧满载约 9 帧/ms, 泵每主循环一轮至少一次; 32 深(与 dmtool 同
    // 一量纲), 满了丢帧 + OVERRUN 记录。
    std::array<utility::RingBuffer<RxEvent, 32>, kChannelCount> rx_queues_;
    std::array<std::atomic<uint32_t>, kChannelCount> overruns_{};
    std::array<uint32_t, kChannelCount> overrun_seen_{};

    bool options_calibration_ = false;
    bool calibration_due_ = false;
    bool driver_loaded_ = false;
};

constinit Adapter::Lazy adapter{};

template <typename F>
void with_adapter(F&& f) {
    if (auto* instance = adapter.try_get())
        std::forward<F>(f)(*instance);
}

// ---- TinyUSB 应用类驱动 ----
//
// 承接接口 0 的五个 bulk 端点。回调全部从 tud_task() 在主循环运行(xfer_isr 为
// 空), CAN ISR 因此只与 pcan_adapter.hpp 的闸和队列打交道。

void driver_init() { adapter.init(); }

bool driver_deinit() { return true; }

void driver_reset(uint8_t rhport) {
    (void)rhport;
    with_adapter([](Adapter& a) { a.on_usb_reset(); });
}

uint16_t driver_open(uint8_t rhport, const tusb_desc_interface_t* interface, uint16_t max_len) {
    return adapter->open(rhport, interface, max_len);
}

bool driver_control_xfer(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    // 本接口不支持类请求; 端点标准请求(CLEAR_FEATURE)由 usbd 处理。
    (void)rhport;
    (void)stage;
    (void)request;
    return false;
}

bool driver_xfer(uint8_t rhport, uint8_t endpoint, xfer_result_t result, uint32_t size) {
    (void)rhport;
    adapter->on_transfer(endpoint, result, size);
    return true;
}

constexpr usbd_class_driver_t kDrivers[]{
    {
     .name = "PCAN",
     .init = driver_init,
     .deinit = driver_deinit,
     .reset = driver_reset,
     .open = driver_open,
     .control_xfer_cb = driver_control_xfer,
     .xfer_cb = driver_xfer,
     .xfer_isr = nullptr,
     .sof = nullptr,
     },
};

} // namespace

namespace internal {

void poll_slow() { adapter->poll(); }

} // namespace internal

// ISR 侧只有闸打开时才会调到, 而闸只能由构造好的实例打开。
void push_can_rx(std::size_t can_index, const RxEvent& event) {
    adapter->push_can_rx(can_index, event);
}

void poll_tick(uint32_t tick) {
    with_adapter([&tick](Adapter& a) { a.poll_tick(tick); });
}

void set_driver_loaded(bool loaded) {
    with_adapter([loaded](Adapter& a) { a.set_driver_loaded(loaded); });
}

bool session_established() {
    bool established = false;
    with_adapter([&established](Adapter& a) { established = a.session_established(); });
    return established;
}

uint32_t serial_number() { return computed_serial_number(); }

uint32_t device_id(std::size_t channel) {
    return channel < kChannelCount ? device_ids()[channel] : 0U;
}

// TinyUSB 回调, 与 vendor 回调一样从 tud_task() 在主循环运行。
extern "C" {

usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count) {
    *driver_count = static_cast<uint8_t>(std::size(kDrivers));
    return kDrivers;
}

// mount(SET_CONFIGURATION 成功)绝不能动适配器: open() 刚在这之前把端点挂上,
// 在这里清状态等于每次配置成功后自毁(实测 2026-10-15: bulk OUT 全程 NAK,
// -110)。总线复位的清理走 driver_reset(usbd 对每个驱动调 reset); umount(解
// 配置/拔出)才清协议状态。挂起/恢复保持状态 -- 真实适配器同此, 主机恢复后
// 继续 ifup 流程。
void tud_mount_cb() {}

void tud_umount_cb() {
    with_adapter([](Adapter& a) { a.on_usb_reset(); });
}

void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; }

void tud_resume_cb() {}

} // extern "C"

} // namespace libhcs::firmware::pcan
