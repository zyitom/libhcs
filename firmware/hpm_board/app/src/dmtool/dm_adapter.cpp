#include "firmware/hpm_board/app/src/dmtool/dm_adapter.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <utility>

#include <device/usbd_pvt.h>
#include <hpm_otp_drv.h>
#include <hpm_ptpc_drv.h>
#include <hpm_soc.h>
#include <tusb.h>

#include "board_app.hpp"
#include "core/include/libhcs/data/datas.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/hpm_board/app/src/can/can.hpp"
#include "firmware/hpm_board/app/src/diag/latency.hpp"
#include "firmware/hpm_board/app/src/dmtool/dm_persist.hpp"
#include "firmware/hpm_board/app/src/led/led.hpp"
#include "firmware/hpm_board/app/src/link/uplink.hpp"
#include "firmware/hpm_board/app/src/uart/uart.hpp"
#include "firmware/hpm_board/app/src/usb/usb_descriptors.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"
#include "firmware/hpm_board/app/src/utility/ring_buffer.hpp"

// DMTool 仿真的实现: USB 应用类驱动、命令应答、CAN 收发与记录流、CDC 串口桥。
// 协议格式见 dm_protocol.hpp, 与 libhcs 的共存规则见 dm_adapter.hpp。
//
// 取舍: 板子是纯转发桥(仓库根 AGENTS.md「架构边界」)。DMTool 命令里凡是要板子
// 自己做事或改持久状态的 -- 设备端重复发送与 ID/数据自增、自测、运行时改总线
// 速率、固件升级、写 SN -- 一律不做, 并如实回失败或只做转发那一部分; 查询类命令
// 回报硬件事实, 不回显请求值。

namespace libhcs::firmware::dmtool {

namespace {

using protocol::AckStatus;
using protocol::Command;
using protocol::Direction;
using usb::DmInterface;

constexpr uint8_t kCdcInterface = 0; // 唯一的 CDC 实例

// 读版本的应答: 产品串加结尾 NUL。DMTool 原样显示它, 并对它 strlen 后找 "boot"
// 判断设备是否停在 bootloader(CheckIsBoot) -- 所以 NUL 必须随负载发出。
constexpr auto kVersionPayload = [] {
    constexpr std::string_view version = usb::UsbDescriptors::product_string();
    static_assert(version.find("boot") == std::string_view::npos);
    std::array<uint8_t, version.size() + 1> bytes{};
    std::ranges::transform(version, bytes.begin(), [](char c) { return static_cast<uint8_t>(c); });
    return bytes;
}();

// 应答负载上限, 最长的是读 SN(42 字节序列号 + NUL)。整条应答小于全速包长 64,
// 任何速率下都是一个短包: 主机的读立即完成, 5A 也恰是本次传输的最后一个字节。
constexpr std::size_t kMaxAckPayload = 48;
constexpr std::size_t kMaxAckSize = kMaxAckPayload + protocol::kAckOverhead;
static_assert(kVersionPayload.size() <= kMaxAckPayload);
static_assert(kMaxAckSize < 64);

// PTPC 原始 {秒, 纳秒} 换成真实纳秒。数字模式下纳秒计数每拍只走 floor(1e9 / f)
// ns, 比真实时间慢; 与 CAN 驱动的微秒换算(board::kCanTimestampNsPerUs)是同一个
// 修正。64 位除法, 所以在主循环里做。
constexpr uint64_t to_real_ns(uint32_t sec, uint32_t ns) {
    constexpr uint64_t divisor = board::kCanTimestampNsPerUs;
    const uint64_t raw = (uint64_t{sec} * 1'000'000'000U) + ns;
    return (raw / divisor * 1000U) + (raw % divisor * 1000U / divisor);
}
static_assert(to_real_ns(0, board::kCanTimestampNsPerUs) == 1000);

// 此刻的 PTPC0 时间, 与 CAN 接收时间戳同一时基(给发送回显打戳)。秒在两次读之间
// 进位时重读, 保证 {秒, 纳秒} 属于同一时刻。
std::pair<uint32_t, uint32_t> ptpc_now() {
    uint32_t sec = ptpc_get_timestamp_second(HPM_PTPC, PTPC_PTPC_0);
    while (true) {
        const uint32_t ns = ptpc_get_timestamp_ns(HPM_PTPC, PTPC_PTPC_0);
        const uint32_t sec_after = ptpc_get_timestamp_second(HPM_PTPC, PTPC_PTPC_0);
        if (sec_after == sec)
            return {sec, ns};
        sec = sec_after;
    }
}

// DMTool 通道即本板 CAN 序号; 单 CAN 板上通道 1 不存在。
can::Can* channel_can(uint8_t channel) {
    if (channel >= kChannelCount || channel >= can::can_count())
        return nullptr;
    return can::can_array[channel].try_get();
}

// 控制器位时序折成 DMTool 的字段。DMTool 按 80 MHz 位时钟解释分频, 时钟不同或字段
// 超出一字节时无法表达。
std::optional<protocol::PhaseTiming>
    to_dm_timing(const can::Can::PhaseTiming& timing, uint32_t clock_hz) {
    if (clock_hz != protocol::BaudrateConfig::kClockHz)
        return std::nullopt;
    if (std::max({timing.prescaler, timing.seg1, timing.seg2, timing.sjw}) > UINT8_MAX)
        return std::nullopt;
    return protocol::PhaseTiming{
        .seg1 = static_cast<uint8_t>(timing.seg1),
        .seg2 = static_cast<uint8_t>(timing.seg2),
        .sjw = static_cast<uint8_t>(timing.sjw),
        .prescaler = static_cast<uint8_t>(timing.prescaler),
    };
}

// 主机设的串口波特率与 UART 实际速率差在 3%(UART 自身的容差)以内。
bool baudrate_matches(uint32_t requested, uint32_t actual) {
    if (requested == 0 || actual == 0)
        return false;
    const uint32_t diff = requested > actual ? requested - actual : actual - requested;
    return uint64_t{diff} * 100U <= uint64_t{actual} * 3U;
}

struct EchoEvent {
    CanFrameEvent frame;
    uint8_t channel;
    bool delivered;
};

protocol::CanRecord
    make_record(const CanFrameEvent& event, uint8_t channel, Direction direction, bool delivered) {
    return {
        .id = event.id,
        .flags = event.flags,
        .direction = direction,
        .delivered = delivered,
        .dlc = event.dlc,
        .channel = channel,
        .timestamp_ns = to_real_ns(event.timestamp_sec, event.timestamp_ns),
        // 记录负载 = payload_length(DLC) 字节(长帧 DLC 9-15 -> 12-64), 不是
        // DLC 数值本身 -- 此前按 DLC 截断, 长帧尾部字节全变 0。
        .data = std::span{event.data}.first(
            std::min<std::size_t>(protocol::payload_length(event.dlc), event.data.size())),
    };
}

// ---- USB 端点 ----

// 三个 DMTool 接口各一对 bulk 端点的 DMA 缓冲, 与 TinyUSB 类驱动的端点缓冲同一
// 放法(非缓存段、按 CFG_TUD_MEM_ALIGN 对齐)。记录流与应答直接编码进 IN 缓冲。
struct EndpointBuffers {
    TUD_EPBUF_DEF(out, CFG_TUD_VENDOR_EPSIZE);
    TUD_EPBUF_DEF(in, CFG_TUD_VENDOR_EPSIZE);
};
CFG_TUD_MEM_SECTION std::array<EndpointBuffers, usb::kDmInterfaceCount> g_endpoint_buffers;

struct Endpoint {
    uint8_t address = 0; // 0 = 未打开
    uint16_t packet_size = 0;
};

struct InterfaceEndpoints {
    Endpoint out;
    Endpoint in;
};

class Adapter : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Adapter>;

    Adapter() {
        persist::Config stored{};
        if (persist::load(stored))
            saved_config_ = stored;
    }

    // ---- USB 应用类驱动(tud_task 里调用) ----

    // 只认 DMTool 的三个接口(0-2, vendor 类); 其余返回 0, 交给内置驱动。
    uint16_t open(uint8_t rhport, const tusb_desc_interface_t* interface, uint16_t max_len) {
        if (interface->bInterfaceClass != TUSB_CLASS_VENDOR_SPECIFIC
            || interface->bInterfaceNumber >= usb::kDmInterfaceCount)
            return 0;

        rhport_ = rhport;
        const uint8_t index = interface->bInterfaceNumber;
        auto& endpoints = endpoints_[index];
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
                const Endpoint opened{
                    .address = endpoint->bEndpointAddress,
                    .packet_size = tu_edpt_packet_size(endpoint),
                };
                (tu_edpt_dir(opened.address) == TUSB_DIR_IN ? endpoints.in : endpoints.out) =
                    opened;
            }
            descriptor = tu_desc_next(descriptor);
        }
        arm_out(index);
        return static_cast<uint16_t>(descriptor - reinterpret_cast<const uint8_t*>(interface));
    }

    // OUT 收到一包: 处理完再重挂(同 TinyUSB vendor 类的次序, 缓冲在处理期间不会被
    // 控制器覆写)。IN 完成无事可做 -- 端点空出, 下一轮 poll() 自会续上。
    void on_transfer(uint8_t endpoint_address, xfer_result_t result, uint32_t size) {
        if (tu_edpt_dir(endpoint_address) == TUSB_DIR_IN)
            return;
        for (std::size_t index = 0; index < endpoints_.size(); ++index) {
            const auto& out = endpoints_[index].out;
            if (out.address != endpoint_address)
                continue;
            if (result == XFER_RESULT_SUCCESS)
                on_out_packet(
                    static_cast<DmInterface>(index),
                    std::span{g_endpoint_buffers[index].out}.first(
                        std::min<std::size_t>(size, CFG_TUD_VENDOR_EPSIZE)),
                    size < out.packet_size);
            // kCan 通道 hold 时不重挂: 帧留在 DMA 缓冲, 队列疏干后由 poll() 续帧。
            arm_out(index);
            return;
        }
    }

    // 总线复位: 端点随之关闭, 状态作废。
    void on_usb_reset() {
        endpoints_ = {};
        isolated_ = false;
        reset();
    }

    // ---- 端点隔离(libhcs 会话期间, 见 dm_adapter.hpp) ----

    void isolate() {
        isolated_ = true;
        // STALL 顺带 flush 掉在途传输(dcd_ci_hs 的 dcd_edpt_stall), 之后控制器对
        // 这些端点的每个令牌都直接回 STALL, 不产生中断与事件。
        for_each_endpoint([this](uint8_t address) { usbd_edpt_stall(rhport_, address); });
    }

    void release() {
        if (!isolated_)
            return;
        isolated_ = false;
        for_each_endpoint([this](uint8_t address) { usbd_edpt_clear_stall(rhport_, address); });
        for (std::size_t index = 0; index < endpoints_.size(); ++index)
            arm_out(static_cast<uint8_t>(index));
    }

    // 主机对本驱动某端点发了 CLEAR_FEATURE(ENDPOINT_HALT); usbd 已经先执行了
    // usbd_edpt_clear_stall。隔离期间要重新 stall -- DMTool 每条命令前都对 0x02 /
    // 0x82 做 clear_halt, 不重新 stall 的话, 用户在 DMTool 里点一下按钮就能把命令
    // 送进来。不在隔离期时什么都不做(clear_halt 与 data toggle 的问题见
    // DMTOOL_PROTOCOL.md 6.4, 修法待定)。
    void on_clear_halt(uint8_t address) {
        if (isolated_ && interface_of(address))
            usbd_edpt_stall(rhport_, address);
    }

    // ---- CDC ----

    void on_cdc_rx() const {
        // 桥没开时照样把 FIFO 读空: 没按 UART 波特率打开的串口(ModemManager 之类
        // 的探测)发来的字节不能留着, 更不能上 UART。
        std::array<uint8_t, 64> chunk{};
        while (const uint32_t count = tud_cdc_n_read(kCdcInterface, chunk.data(), chunk.size())) {
            if (!cdc_bridge_)
                continue;
            if (auto* uart = uart::uart_array[0].try_get())
                uart->handle_downlink({.uart_data = std::as_bytes(std::span{chunk}.first(count))});
        }
    }

    void on_cdc_line_state(bool dtr) {
        cdc_dtr_ = dtr;
        update_cdc_bridge();
    }

    // 标准 VCP 语义: 主机设的线路编码(Linux termios / Windows SetCommState ->
    // CDC SET_LINE_CODING, 两个操作系统走同一条标准请求)真实下发到板上 UART,
    // 而非只做速率比对。仅在 libhcs 会话之外转发(UART 此时属于桥); 速率求解
    // 失败时保持原值, 桥因速率失配保持关闭, 主机换个合法速率即恢复。
    // 编码映射: CDC 校验 0=无 1=奇 2=偶 -> libhcs 1=无 3=奇 2=偶 (mark/space
    // 不支持, 保持不变); CDC 停止 0=1 1=1.5 2=2 -> libhcs 1=1 其余=2 (16550
    // 的 1.5 停止位只在 5 位字长存在); 数据位 7/8 直传, 其余保持不变。
    void on_cdc_line_coding(uint32_t bit_rate, uint8_t stop, uint8_t parity, uint8_t data_bits) {
        cdc_bit_rate_ = bit_rate;
        if (!link::uplink_enabled()) {
            if (auto* uart = uart::uart_array[0].try_get(); uart != nullptr) {
                if (bit_rate != 0U)
                    (void)uart->set_baudrate(bit_rate);
                uint32_t parity_out = 0U;
                if (parity == 1U)
                    parity_out = 3U;
                else if (parity == 2U)
                    parity_out = 2U;
                else if (parity == 0U)
                    parity_out = 1U;
                const uint32_t stop_out = stop == 0U ? 1U : 2U;
                uart->commit_framing(
                    (data_bits == 7U || data_bits == 8U) ? data_bits : 0U, parity_out, stop_out);
            }
        }
        update_cdc_bridge();
    }

    // ---- ISR 侧 ----

    void push_can_rx(std::size_t can_index, const CanFrameEvent& event) {
        if (can_index < rx_queues_.size())
            (void)rx_queues_[can_index].push_back(event);
    }

    void push_uart_rx(std::span<const std::byte> bytes) {
        auto next = bytes.begin();
        (void)uart_to_cdc_.push_back_n([&next]() noexcept { return *next++; }, bytes.size());
    }

    // ---- 主循环(只在没有 libhcs 会话的轮次) ----

    void poll() {
        if (save_pending_) {
            save_pending_ = false;
            (void)persist::store(save_snapshot_); // 失败静默: ACK 已发出
        }
        const uint8_t service = internal::g_service.load(std::memory_order::relaxed);
        if ((service & kServiceAck) != 0)
            flush_pending_ack();
        if ((service & kServiceCapture) != 0) {
            pump_stream(DmInterface::kData, [this](std::span<uint8_t> out) {
                std::size_t used = 0;
                for (std::size_t channel = 0; channel < rx_queues_.size(); ++channel) {
                    used = encode_from(
                        rx_queues_[channel], out, used, [channel](const CanFrameEvent& event) {
                            return make_record(
                                event, static_cast<uint8_t>(channel), Direction::kReceive, false);
                        });
                }
                return used;
            });
            pump_stream(DmInterface::kCan, [this](std::span<uint8_t> out) {
                return encode_from(echo_queue_, out, 0, [](const EchoEvent& echo) {
                    return make_record(
                        echo.frame, echo.channel, Direction::kTransmit, echo.delivered);
                });
            });
        }
        if ((service & kServiceCdcBridge) != 0)
            pump_uart_to_cdc();
    }

    // 总线复位与 libhcs 让位共用: 回到"DMTool 与 CDC 都没在用"。先撤闸, ISR 从此
    // 不再入队, 之后清队列(主循环是这些队列的消费者, 清空是消费侧操作)。端点保持
    // 原样: libhcs 让位时总线仍是配置好的。
    void reset() {
        capture_mask_ = 0;
        cdc_dtr_ = false;
        cdc_bridge_ = false;
        pending_ack_size_ = 0;
        preset_applied_ = false;
        publish();
        for (uint8_t channel = 0; channel < kChannelCount; ++channel)
            led::led->set_channel_active(channel, false);

        for (auto& queue : rx_queues_)
            (void)queue.clear();
        (void)echo_queue_.clear();
        (void)uart_to_cdc_.clear();
        command_parser_.reset();
    }

private:
    // ---- USB 端点操作 ----

    void arm_out(uint8_t index) {
        const auto& out = endpoints_[index].out;
        if (out.address == 0)
            return;
        (void)usbd_edpt_xfer(
            rhport_, out.address, g_endpoint_buffers[index].out, out.packet_size, false);
    }

    [[nodiscard]] bool in_ready(DmInterface interface) const {
        const auto& in = endpoints_[std::to_underlying(interface)].in;
        return in.address != 0 && !usbd_edpt_busy(rhport_, in.address);
    }

    // 以 IN 缓冲前 size 字节发起一次传输; 调用方先确认过 in_ready()。
    void send_in(DmInterface interface, std::size_t size) {
        const auto index = std::to_underlying(interface);
        const auto& in = endpoints_[index].in;
        if (!usbd_edpt_claim(rhport_, in.address)) [[unlikely]]
            return;
        (void)usbd_edpt_xfer(
            rhport_, in.address, g_endpoint_buffers[index].in, static_cast<uint16_t>(size), false);
    }

    // 端点地址 -> DMTool 接口序号; 不是本驱动的端点返回空。
    [[nodiscard]] std::optional<uint8_t> interface_of(uint8_t address) const {
        for (std::size_t index = 0; index < endpoints_.size(); ++index) {
            const auto& pair = endpoints_[index];
            if (address != 0 && (pair.in.address == address || pair.out.address == address))
                return static_cast<uint8_t>(index);
        }
        return std::nullopt;
    }

    template <typename F>
    void for_each_endpoint(F f) const {
        for (const auto& pair : endpoints_) {
            for (const auto& endpoint : {pair.out, pair.in}) {
                if (endpoint.address != 0)
                    f(endpoint.address);
            }
        }
    }

    void on_out_packet(DmInterface interface, std::span<const uint8_t> packet, bool end) {
        switch (interface) {
        case DmInterface::kCommand:
            command_parser_.feed(packet, end, [this](const protocol::CommandParser::Frame& frame) {
                handle_command(frame);
            });
            return;
        case DmInterface::kCan: {
            // 一帧一次传输是 DMTool 的用法, 连续多帧也照样逐帧拆。队列满: 帧丢弃
            // (2026-09-14 决策)。背压 hold 实测有致命缺陷: 电机 CAN 不应答(擦除
            // 窗口)时 M_CAN FIFO 被自动重传永久钉死, 队列永不疏干, hold 永不释放,
            // EP 0x03 整体卡死 -- 除非把队列加深到能装下整个擦除窗口(待测量)。
            auto rest = packet;
            while (const auto request = protocol::parse_transmit_request(rest)) {
                if (!transmit(*request))
                    led::led->downlink_buffer_full(); // 队列满丢弃(2026-09-14 决策)
                rest = rest.subspan(request->encoded_size);
            }
            return;
        }
        case DmInterface::kData:
            // 只有心跳(A5 10 ... 5A), 真适配器也不回应。
            return;
        }
    }

    // ---- 命令 ----

    void handle_command(const protocol::CommandParser::Frame& frame) {
        ensure_session_timing();
        const uint8_t command = frame.command;
        const auto payload = frame.payload;
        switch (static_cast<Command>(command)) {
        case Command::kReadVersion: ack(command, AckStatus::kOk, kVersionPayload); return;
        case Command::kGetUuid: ack(command, AckStatus::kOk, read_uuid()); return;
        case Command::kGetSerial: reply_serial(command); return;
        case Command::kStartCapture: ack(command, start_capture(payload)); return;
        case Command::kStopCapture: ack(command, stop_capture(payload)); return;
        case Command::kSetupBaudrate: ack(command, check_baudrate(payload)); return;
        case Command::kGetBaudrate: reply_baudrate(command, payload); return;

        // 本板没有可保存、可恢复出厂的参数(总线参数是编译期事实), 现状就是"已保存的
        // 出厂状态"; 应用本就不在 bootloader 里; 设备端从不重复发送, 没有可停的。
        case Command::kSaveParameters: ack(command, save_config()); return;
        case Command::kRecoveryFactory:
        case Command::kJumpOutLoader:
        case Command::kStopPeriodicSend: ack(command, AckStatus::kOk); return;

        // 固件升级、自测、写 SN 一律不做。回失败让 DMTool 报错, 绝不让它以为升级
        // 成功了(IAP 数据包超出解析器负载上限, 同样走到这里)。
        case Command::kIapPacket:
        case Command::kIapEnd:
        case Command::kStartTest:
        case Command::kWriteSerial: ack(command, AckStatus::kFailed); return;
        }
        ack(command, AckStatus::kFailed); // 未知命令
    }

    // 每次会话把总线摆到 DMTool 认得的时序, 只做一次(preset_applied_ 由
    // reset() 清除, 即总线复位与 libhcs 让位各算一次新会话)。
    //
    // 复位后控制器跑的是 SDK 求解解(分频1/seg1 69/seg2 10), 它不在 DMTool 的
    // 预设表(can_seg_table / can_fd_seg_table)里 -- "更新配置"按表回查会查不到,
    // 界面毫无反应。套用预设后读回值与表项逐一对应, 更新/配置/保存的闭环才成立。
    //
    // 触发点是会话的第一条命令而不是第一次 START_CAP: DMTool 的打开顺序随版本
    // 而异(2.1.6.7 打开即 START_CAP, 更早的版本先读版本/读波特率), 连接前先读
    // 一次波特率同样要读到表里有的值。
    void ensure_session_timing() {
        if (preset_applied_ || link::uplink_enabled())
            return;
        preset_applied_ = true;

        // 每通道: flash 里有保存配置(0x10 写入, 掉电保持)则优先套用, 否则
        // 套用 DMTool 的 1M/5M 默认预设(见 dm_persist.hpp / can.hpp)。
        constexpr can::Can::PhaseTiming default_nominal{
            .prescaler = 2, .seg1 = 29, .seg2 = 10, .sjw = 2};
        constexpr can::Can::PhaseTiming default_data{
            .prescaler = 2, .seg1 = 5, .seg2 = 2, .sjw = 2};
        for (std::size_t i = 0; i < can::can_count(); ++i) {
            auto* can = can::can_array[i].try_get();
            if (can == nullptr) [[unlikely]]
                continue;
            if (!saved_config_) {
                (void)can->reconfigure_timing(true, default_nominal, default_data);
                continue;
            }
            const auto& st = (*saved_config_)[i];
            const can::Can::PhaseTiming nominal{
                .prescaler = st.nominal_prescaler,
                .seg1 = st.nominal_seg1,
                .seg2 = st.nominal_seg2,
                .sjw = st.nominal_sjw};
            const can::Can::PhaseTiming data{
                .prescaler = st.data_prescaler,
                .seg1 = st.data_seg1,
                .seg2 = st.data_seg2,
                .sjw = st.data_sjw};
            (void)can->reconfigure_timing(st.fd, nominal, data);
        }
    }

    AckStatus start_capture(std::span<const uint8_t> payload) {
        // libhcs 会话期间不接 DMTool: libhcs 优先(见 dm_adapter.hpp)。
        if (payload.size() != 1 || channel_can(payload[0]) == nullptr || link::uplink_enabled())
            return AckStatus::kFailed;
        const uint8_t channel = payload[0];
        (void)rx_queues_[channel].clear(); // 采集关着时生产者不入队, 清的只是旧帧
        capture_mask_ = static_cast<uint8_t>(capture_mask_ | (1U << channel));
        publish();
        // 通道指示灯常亮: 让用户看到"哪一路被选为采集通道"(故障灯语优先)。
        led::led->set_channel_active(channel, true);
        return AckStatus::kOk;
    }

    // 保存配置(0x10): 把两路控制器当前的位时序写 flash, 掉电保持; 下次会话
    // 首次采集时套用(见 start_capture)。
    //
    // 写入是异步的: ACK(含 DMTool 的 500ms 同步读超时)先回, 擦扇区(~45-400ms,
    // 全局关中断)随后在主循环执行。同步写曾实测拖垮 ACK 超时并卡死 DMTool
    // 界面。代价: 保存后 ~0.4s 内掉电可能丢这次保存; 写期间 CAN RX 停,
    // 高速率下瞬时丢帧 -- 保存是显式低频动作, 可接受。
    AckStatus save_config() {
        persist::Config config{};
        for (std::size_t i = 0; i < persist::kChannelCount; ++i) {
            const auto* can = i < can::can_count() ? can::can_array[i].try_get() : nullptr;
            if (can == nullptr) {
                // 不存在的通道存默认预设, 保持布局完整。
                config[i] = {
                    .fd = true,
                    .nominal_prescaler = 2,
                    .nominal_seg1 = 29,
                    .nominal_seg2 = 10,
                    .nominal_sjw = 2,
                    .data_prescaler = 2,
                    .data_seg1 = 5,
                    .data_seg2 = 2,
                    .data_sjw = 2};
                continue;
            }
            const auto timing = can->bit_timing();
            const auto to_byte = [](uint32_t v) -> std::optional<uint8_t> {
                return v <= 255U ? std::optional<uint8_t>(static_cast<uint8_t>(v)) : std::nullopt;
            };
            const auto np = to_byte(timing.nominal.prescaler);
            const auto dp = to_byte(timing.data.prescaler);
            if (!np || !dp)
                return AckStatus::kFailed;
            config[i] = {
                .fd = can->is_fd(),
                .nominal_prescaler = *np,
                .nominal_seg1 = static_cast<uint8_t>(timing.nominal.seg1),
                .nominal_seg2 = static_cast<uint8_t>(timing.nominal.seg2),
                .nominal_sjw = static_cast<uint8_t>(timing.nominal.sjw),
                .data_prescaler = *dp,
                .data_seg1 = static_cast<uint8_t>(timing.data.seg1),
                .data_seg2 = static_cast<uint8_t>(timing.data.seg2),
                .data_sjw = static_cast<uint8_t>(timing.data.sjw)};
        }
        save_snapshot_ = config;
        save_pending_ = true;
        saved_config_ = config; // 立即生效于本会话; flash 写入随后完成
        return AckStatus::kOk;
    }

    AckStatus stop_capture(std::span<const uint8_t> payload) {
        // DMTool 切通道时会先停另一路, 单 CAN 板上停一路不存在的通道也算成功。
        if (payload.size() != 1 || payload[0] >= kChannelCount)
            return AckStatus::kFailed;
        const uint8_t channel = payload[0];
        capture_mask_ = static_cast<uint8_t>(capture_mask_ & ~(1U << channel));
        publish();
        (void)rx_queues_[channel].clear();
        led::led->set_channel_active(channel, false);
        if (capture_mask_ == 0)
            (void)echo_queue_.clear();
        return AckStatus::kOk;
    }

    // 设置波特率: 按 DMTool 给定的 TQ 参数真正重配控制器(上板实测: 校验式实现
    // 会在用户选择任何非编译期速率时回失败, DMTool 因此报配置失败)。DMTool 按
    // 80 MHz 位时钟解释分频; 本板位时钟不同时, 只有能整除缩放的分频可表达同一
    // 速率, 其余拒绝。参数范围(段数上下限、数据段分频上限)交给 SDK 的低级校验,
    // 拒绝时 reconfigure_timing 恢复编译期配置, 端口保持原样。经典模式(fd=0)
    // 对 FD 控制器是合法请求 -- DMTool 的 CAN2.0/FDCAN 开关随命令到来。
    static AckStatus check_baudrate(std::span<const uint8_t> payload) {
        const auto config = protocol::BaudrateConfig::decode(payload);
        if (!config.has_value())
            return AckStatus::kFailed;
        can::Can* can = channel_can(config->channel);
        if (can == nullptr)
            return AckStatus::kFailed;

        constexpr uint32_t dm_clock_hz = protocol::BaudrateConfig::kClockHz;
        const uint32_t board_clock = can->bit_timing().clock_hz;
        if (board_clock == 0)
            return AckStatus::kFailed;

        const auto scale_prescaler = [&](uint32_t dm_prescaler) -> std::optional<uint32_t> {
            const uint64_t scaled = static_cast<uint64_t>(dm_prescaler) * dm_clock_hz;
            if (dm_prescaler == 0 || scaled % board_clock != 0)
                return std::nullopt;
            return static_cast<uint32_t>(scaled / board_clock);
        };
        const auto nominal_prescaler = scale_prescaler(config->nominal.prescaler);
        // 经典模式(fd=0)不用数据段, DMTool 在 CAN2.0 档发送的数据段字段为 0,
        // 跳过缩放与校验。
        const auto data_prescaler = config->fd
                                      ? scale_prescaler(config->data.prescaler)
                                      : decltype(scale_prescaler(config->nominal.prescaler))(1U);
        if (!nominal_prescaler || !data_prescaler)
            return AckStatus::kFailed;

        // protocol::PhaseTiming 与 Can::PhaseTiming 字段同名同义, 但类型不同:
        // 显式换型, 分频换成板级时钟下的等价值。
        const auto nominal = can::Can::PhaseTiming{
            .prescaler = *nominal_prescaler,
            .seg1 = config->nominal.seg1,
            .seg2 = config->nominal.seg2,
            .sjw = config->nominal.sjw,
        };
        const auto data = can::Can::PhaseTiming{
            .prescaler = *data_prescaler,
            .seg1 = config->data.seg1,
            .seg2 = config->data.seg2,
            .sjw = config->data.sjw,
        };

        if (config->fd && (data.seg1 == 0 || data.seg2 == 0 || data.sjw == 0))
            return AckStatus::kFailed;

        return can->reconfigure_timing(config->fd, nominal, data) ? AckStatus::kOk
                                                                  : AckStatus::kFailed;
    }

    // 读波特率回硬件事实: 控制器里实际的位时序, 而不是最近一次设置的请求值。
    void reply_baudrate(uint8_t command, std::span<const uint8_t> payload) {
        const can::Can* can = payload.size() == 1 ? channel_can(payload[0]) : nullptr;
        if (can == nullptr) {
            ack(command, AckStatus::kFailed);
            return;
        }

        const auto timing = can->bit_timing();
        const auto nominal = to_dm_timing(timing.nominal, timing.clock_hz);
        const auto data = can->is_fd() ? to_dm_timing(timing.data, timing.clock_hz)
                                       : std::optional{protocol::PhaseTiming{}};
        if (!nominal || !data) {
            ack(command, AckStatus::kFailed);
            return;
        }

        const protocol::BaudrateConfig config{
            .channel = payload[0],
            .fd = can->is_fd(),
            .nominal = *nominal,
            .data = *data,
        };
        ack(command, AckStatus::kOk, config.encode());
    }

    static std::array<uint8_t, OTP_SOC_UUID_LEN> read_uuid() {
        std::array<uint8_t, OTP_SOC_UUID_LEN> bytes{};
        for (std::size_t i = 0; i < bytes.size(); ++i) {
            const uint32_t word =
                otp_read_from_shadow(OTP_SOC_UUID_IDX + static_cast<uint32_t>(i / 4));
            bytes[i] = static_cast<uint8_t>(word >> (8U * (i % 4)));
        }
        return bytes;
    }

    void reply_serial(uint8_t command) {
        std::array<uint8_t, kMaxAckPayload> bytes{};
        const auto serial = usb::usb_descriptors->serial_string().substr(0, bytes.size() - 1);
        std::ranges::transform(
            serial, bytes.begin(), [](char c) { return static_cast<uint8_t>(c); });
        ack(command, AckStatus::kOk, std::span{bytes}.first(serial.size() + 1));
    }

    // 端点空着就直接编码进 IN 缓冲发出; 否则只留最新一条, 等端点空出后由 poll()
    // 补发。DMTool 一问一答, 实际不会积压。
    void ack(uint8_t command, AckStatus status, std::span<const uint8_t> payload = {}) {
        if (pending_ack_size_ == 0 && in_ready(DmInterface::kCommand)) {
            const auto frame = protocol::build_ack(
                command_in_buffer().first(kMaxAckSize), command, status, payload);
            if (!frame.empty()) [[likely]]
                send_in(DmInterface::kCommand, frame.size());
            return;
        }
        pending_ack_size_ = protocol::build_ack(pending_ack_, command, status, payload).size();
        publish();
    }

    void flush_pending_ack() {
        if (pending_ack_size_ == 0 || !in_ready(DmInterface::kCommand))
            return;
        std::ranges::copy(
            std::span{pending_ack_}.first(pending_ack_size_), command_in_buffer().begin());
        send_in(DmInterface::kCommand, pending_ack_size_);
        pending_ack_size_ = 0;
        publish();
    }

    static std::span<uint8_t> command_in_buffer() {
        return g_endpoint_buffers[std::to_underlying(DmInterface::kCommand)].in;
    }

    // ---- CAN 发送 ----

    // 只转发: 帧型按请求逐帧走(端口模式是上限, 见 Can::handle_downlink_as) --
    // 界面上勾的 FD/BRS 就是这一帧的帧型, 达妙电机 bootloader 那种"经典帧"的
    // 请求因此不会被当成 FD 发出去。设备端重复发送(发送次数 > 1)与 ID/数据自增
    // 不实现, 每个请求只发一次。回显报的是实际上线的帧型, 发不出去的帧回显为
    // 发送失败。
    bool transmit(const protocol::TransmitRequest& request) {
        can::Can* can = channel_can(request.channel);
        const bool id_valid = request.flags.extended || request.id <= 0x7FFU;
        const bool delivered = can != nullptr && id_valid && request.payload.size() <= 64
                            && !link::uplink_enabled(); // libhcs 会话期间 DMTool 不上总线

        // 实际上线的帧型: 经典端口发不出 FD, BRS 只在 FD 帧上有意义。
        const bool fd = can != nullptr && can->is_fd() && request.flags.fd;
        const bool bitrate_switch = fd && request.flags.bitrate_switch;

        bool delivered_ok = true;
        if (delivered) {
            diag::latency::abandon_downlink(); // 不是 libhcs 包, 不进它的延迟拆解
            delivered_ok = can->handle_downlink_as(
                {
                    .can_id = request.id,
                    .can_data = std::as_bytes(request.payload),
                    .is_extended_can_id = request.flags.extended,
                    .is_remote_transmission = request.flags.remote,
                },
                {
                    .wire_dlc = request.dlc, // 长帧 DLC 9-15 直传
                    .fd = request.flags.fd,
                    .bitrate_switch = request.flags.bitrate_switch,
                });
        }

        // DMTool 表格里的发送行只来自 0x83 回显, 采集本路时才回。
        if (request.channel >= kChannelCount || ((capture_mask_ >> request.channel) & 1U) == 0)
            return true; // 回显未开: 无记录可发
        const protocol::CanFrameFlags flags{
            .extended = request.flags.extended,
            .remote = request.flags.remote,
            .fd = fd,
            .bitrate_switch = bitrate_switch,
        };
        const auto [sec, ns] = ptpc_now();
        CanFrameEvent frame{
            .id = request.id,
            .timestamp_sec = sec,
            .timestamp_ns = ns,
            .dlc = request.dlc,
            .flags = flags,
        };
        const auto copied = std::min(request.payload.size(), frame.data.size());
        std::ranges::copy(request.payload.first(copied), frame.data.begin());
        (void)echo_queue_.push_back(
            {.frame = frame, .channel = request.channel, .delivered = delivered});
        return delivered_ok;
    }

    // ---- 记录流 ----

    // 记录流一次 IN 传输的上限: 严格小于端点包长, 传输必以短包结束。主机那边是
    // 32 KB 的同步读: 以满包结尾的传输要等到下一个包或 1 s 超时, 超时读到的数据
    // DMTool 直接丢弃; 零长度传输又被它当成 USB 故障(can_rec_thread_func)。
    //
    // encoder(out) 把记录直接编码进端点的 IN 缓冲并返回字节数, 非 0 才发出。
    // 端点上一次传输未完成时整轮跳过, 帧留在队列里。
    template <typename Encoder>
    void pump_stream(DmInterface interface, Encoder&& encoder) {
        if (!in_ready(interface))
            return;
        const auto index = std::to_underlying(interface);
        const std::size_t limit = endpoints_[index].in.packet_size - 1U;
        const std::size_t used =
            std::forward<Encoder>(encoder)(std::span{g_endpoint_buffers[index].in}.first(limit));
        if (used != 0)
            send_in(interface, used);
    }

    // 从 queue 头部取事件编码进 out[used..], 放不下的留在队列里。
    template <typename Event, std::size_t depth, typename ToRecord>
    static std::size_t encode_from(
        utility::RingBuffer<Event, depth>& queue, std::span<uint8_t> out, std::size_t used,
        ToRecord to_record) {
        while (const Event* event = queue.peek_front()) {
            const std::size_t written =
                protocol::encode_record(to_record(*event), out.subspan(used));
            if (written == 0)
                break;
            used += written;
            (void)queue.pop_front([](const Event&) noexcept {});
        }
        return used;
    }

    // ---- CDC 串口桥 ----

    // 桥只在串口已打开(DTR)、主机设的波特率与板上 UART 实际速率一致、且 libhcs
    // 不在时接通。波特率不去改 UART -- 那是 libhcs 经 EP0 管的配置, 串口工具关掉
    // 后 libhcs 必须看到它原来的样子; 反过来要求主机按 UART 的速率打开, 也顺带把
    // ModemManager 一类按 9600/115200 探测的程序挡在 UART 之外。
    void update_cdc_bridge() {
        const auto* uart = uart::uart_array[0].try_get();
        cdc_bridge_ = cdc_dtr_ && uart != nullptr && !link::uplink_enabled()
                   && baudrate_matches(cdc_bit_rate_, uart->effective_baudrate());
        publish();
        if (!cdc_bridge_)
            (void)uart_to_cdc_.clear();
    }

    void pump_uart_to_cdc() {
        std::array<uint8_t, 64> chunk{};
        bool wrote = false;
        while (uart_to_cdc_.readable() != 0) {
            const uint32_t room = tud_cdc_n_write_available(kCdcInterface);
            if (room == 0)
                break;
            std::size_t count = 0;
            (void)uart_to_cdc_.pop_front_n(
                [&chunk, &count](const std::byte& byte) noexcept {
                    chunk[count++] = std::to_integer<uint8_t>(byte);
                },
                std::min<std::size_t>(room, chunk.size()));
            (void)tud_cdc_n_write(kCdcInterface, chunk.data(), count);
            wrote = true;
        }
        if (wrote)
            (void)tud_cdc_n_write_flush(kCdcInterface);
    }

    // ---- 闸 ----

    // 把主循环侧的状态发布给 ISR 与主循环闸(dm_adapter.hpp)。
    void publish() const {
        uint8_t service = 0;
        if (capture_mask_ != 0)
            service |= kServiceCapture;
        if (cdc_bridge_)
            service |= kServiceCdcBridge;
        if (pending_ack_size_ != 0)
            service |= kServiceAck;
        internal::g_capture_mask.store(capture_mask_, std::memory_order::release);
        internal::g_service.store(service, std::memory_order::release);
    }

    uint8_t rhport_ = 0;
    std::array<InterfaceEndpoints, usb::kDmInterfaceCount> endpoints_{};

    protocol::CommandParser command_parser_;

    // 1 Mbit 经典帧满载约 9 帧/ms, 主循环一轮不到 1 us, 只有 USB 端点在忙(一次传输
    // 最多 21 条)时才会攒, 32 条足够。满了丢帧, 与 libhcs 路径一致。
    std::array<utility::RingBuffer<CanFrameEvent, 32>, kChannelCount> rx_queues_;
    utility::RingBuffer<EchoEvent, 16> echo_queue_;
    // 921600 波特下约 11 ms 的量。
    utility::RingBuffer<std::byte, 1024> uart_to_cdc_;

    std::array<uint8_t, kMaxAckSize> pending_ack_{};
    std::size_t pending_ack_size_ = 0;

    // 主循环侧的真值, 经 publish() 发布。
    uint8_t capture_mask_ = 0;
    bool cdc_dtr_ = false;
    uint32_t cdc_bit_rate_ = 0;
    bool cdc_bridge_ = false;
    // 本会话是否已套用过预设/保存的时序(见 ensure_session_timing)。reset() 清除。
    bool preset_applied_ = false;
    // libhcs 会话期间端点处于 STALL 隔离(isolate / release)。
    // ---- 下行背压(2026-09-22, 电机 IAP 突发零丢帧) ----
    // CAN TX 软件队列满时, 帧留在 EP 0x03 的 DMA 缓冲, 端点不重挂(主机 NAK
    // 自适应), 队列疏干后从断点续帧。仅 kCan 通道; 命令/心跳即时处理。
    bool isolated_ = false;
    // 待写的 flash 保存(保存配置 ACK 先回, 写入随后)。
    bool save_pending_ = false;
    persist::Config save_snapshot_{};
    // 保存配置(0x10)写入 flash 的持久化配置, init 时加载。存在的通道在会话
    // 首次采集时优先于 1M/5M 默认预设套用。
    std::optional<persist::Config> saved_config_;
};

constinit Adapter::Lazy adapter{};

// USB 与 libhcs 侧入口。hpm6e8y 共用这些代码, 实例由驱动的 init 回调在 tud_init
// 时构造, 早于任何回调; 仍用 try_get 兜底, 未构造时是空操作。
template <typename F>
void with_adapter(F&& f) {
    if (auto* instance = adapter.try_get())
        std::forward<F>(f)(*instance);
}

// ---- TinyUSB 应用类驱动 ----
//
// 承接 DMTool 的三个接口(usb_descriptors.hpp 的 DmInterface)。它们若走 TinyUSB
// 的 vendor 类, libhcs 就不再是 vendor 实例 0, bulk 回调里的实例比较与发送路径
// 都会多出指令; 自带驱动后 vendor 类照旧只有 libhcs 一个实例。usbd 先问应用
// 驱动, open() 只认接口 0-2, 其余接口原样落到内置驱动。

void driver_init() { adapter.init(); }

bool driver_deinit() { return true; }

void driver_reset(uint8_t rhport) {
    (void)rhport;
    with_adapter([](Adapter& a) { a.on_usb_reset(); });
}

uint16_t driver_open(uint8_t rhport, const tusb_desc_interface_t* interface, uint16_t max_len) {
    return adapter->open(rhport, interface, max_len);
}

// 只会收到转给端点所属驱动的标准请求(CLEAR_FEATURE 等, usbd 不看返回值)与指向
// 本接口的类请求; 后者本驱动一概不支持。usbd 对标准端点请求只在 SETUP 阶段调用
// 一次, 且在它回状态包之前 -- 隔离期间的重新 stall 因此赶在主机下一次传输之前。
bool driver_control_xfer(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    (void)rhport;
    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_STANDARD)
        return false;
    if (stage == CONTROL_STAGE_SETUP
        && request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT
        && request->bRequest == TUSB_REQ_CLEAR_FEATURE
        && request->wValue == TUSB_REQ_FEATURE_EDPT_HALT) {
        with_adapter([request](Adapter& a) { a.on_clear_halt(tu_u16_low(request->wIndex)); });
    }
    return true;
}

bool driver_xfer(uint8_t rhport, uint8_t endpoint, xfer_result_t result, uint32_t size) {
    (void)rhport;
    adapter->on_transfer(endpoint, result, size);
    return true;
}

constexpr usbd_class_driver_t kDriver{
    .name = "DMTOOL",
    .init = driver_init,
    .deinit = driver_deinit,
    .reset = driver_reset,
    .open = driver_open,
    .control_xfer_cb = driver_control_xfer,
    .xfer_cb = driver_xfer,
    .xfer_isr = nullptr,
    .sof = nullptr,
};

} // namespace

namespace internal {

void poll_slow() { adapter->poll(); }

} // namespace internal

// ISR 侧只有闸打开时才会调到, 而闸只能由构造好的实例打开。
void push_can_rx(std::size_t can_index, const CanFrameEvent& event) {
    adapter->push_can_rx(can_index, event);
}

void uart_rx_without_session(
    const std::byte* data, std::size_t size, const std::byte* data2, std::size_t size2) {
    if ((internal::g_service.load(std::memory_order::relaxed) & kServiceCdcBridge) == 0)
        return;
    adapter->push_uart_rx({data, size});
    adapter->push_uart_rx({data2, size2});
}

void on_libhcs_session() {
    // libhcs 会话开始: DMTool 会话可能把总线改成了别的速率, 还原编译期位时序
    // 与端口 FD 模式, 让 libhcs 期望的总线参数成立(87.5% 采样点那套实测结果)。
    for (std::size_t i = 0; i < can::can_count(); ++i) {
        if (auto* can = can::can_array[i].try_get(); can != nullptr)
            (void)can->restore_default_timing();
    }
    with_adapter([](Adapter& a) {
        a.reset();
        a.isolate();
    });
}

void on_libhcs_session_end() {
    with_adapter([](Adapter& a) { a.release(); });
}

// TinyUSB 回调, 与 vendor 回调一样从 tud_task() 在主循环运行。
extern "C" {

usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count) {
    *driver_count = 1;
    return &kDriver;
}

void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
    with_adapter([](Adapter& a) { a.on_cdc_rx(); });
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)rts;
    with_adapter([dtr](Adapter& a) { a.on_cdc_line_state(dtr); });
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding) {
    (void)itf;
    with_adapter([p_line_coding](Adapter& a) {
        a.on_cdc_line_coding(
            p_line_coding->bit_rate, p_line_coding->stop_bits, p_line_coding->parity,
            p_line_coding->data_bits);
    });
}

} // extern "C"

} // namespace libhcs::firmware::dmtool
