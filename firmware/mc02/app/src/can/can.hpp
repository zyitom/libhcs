#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <fdcan.h>
#include <stm32h7xx_hal_fdcan.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/spec/mc02/ports.hpp"
#include "core/src/link/port.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/common/app/src/utility/event_counter.hpp"
#include "firmware/common/app/src/utility/latched_bus_error.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"
#include "firmware/common/app/src/utility/ring_buffer.hpp"
#include "firmware/mc02/app/src/led/led.hpp"
#include "firmware/mc02/app/src/sync/sof.hpp"
#include "firmware/mc02/app/src/sync/sof_capture.hpp"
#include "firmware/mc02/app/src/utility/loop_work.hpp"

namespace libhcs::firmware::can {

namespace vc = libhcs::core::protocol::vendor_control;
namespace link = libhcs::core::link;

// 帧类型是总线的属性, 不属于单个帧。曾按主机设置的头部位逐帧选择, 该位已废弃
// (见 core/src/protocol/protocol.hpp 的 CanHeaderLayout); 模式现固定于此, 主机
// 在构造握手时经 EP0 配置通道(usb/vendor_control.cpp)读回, 并据此校验其预期。
//
// 控制器本身无论如何都保持 FD 能力: CubeMX 将每个 FDCAN 配为
// FrameFormat = FDCAN_FRAME_FD_BRS, 且开启 FD 的 M_CAN 仍能接收对端的经典帧,
// 因此 kCanFd 条目在接收方向零成本, 只决定本板往总线上发什么。三路总线均运行
// CAN-FD(仲裁 1 Mbit/s / 数据段 5 Mbit/s), 正是本板驱动的电机所要求的;
// 某路总线的对端不支持 FD 时, 修改这里的条目即可, 不做运行时切换。
enum class CanMode : uint8_t {
    kClassic,
    kCanFd,
};

struct CanPort {
    FDCAN_HandleTypeDef* handle;
    data::DataId data_id; // 该连接器的丝印编号
    CanMode mode;
};

// 表序即总线序: 下标 0 对应丝印 CAN1, 依此类推, 与 can.cpp 的 ISR 分发一致。身份引用
// spec/mc02/ports.hpp 的具名描述符, 行序由 ports.hpp 的 static_assert 钉死。
inline constexpr CanPort kCanPorts[] = {
    {.handle = &hfdcan1, .data_id = spec::mc02::Spec::Cans::kCan1.data_id, .mode = CanMode::kCanFd},
    {.handle = &hfdcan2, .data_id = spec::mc02::Spec::Cans::kCan2.data_id, .mode = CanMode::kCanFd},
    {.handle = &hfdcan3, .data_id = spec::mc02::Spec::Cans::kCan3.data_id, .mode = CanMode::kCanFd},
};
inline constexpr size_t kCanCount = std::size(kCanPorts);
static_assert(kCanCount == 3);

class Can : private core::utility::Immovable {
public:
    // 这个驱动作为 EP0 口能做什么(kGetPortList 原样上报): 帧型可切(只翻 Tx 元素的
    // FDF/BRS, 控制器保持 FD 能力); 速率不可设(位时序由 CubeMX 预设钉死, 声明里的速率
    // 只作核对); 不承载长帧(RX FIFO 元素仍是 8 字节, 扩容前不广告"能发不能收"的能力)。
    static constexpr uint8_t kPortCapabilities = spec::kCanCapModeSettable;

    using Lazy = utility::Lazy<Can, FDCAN_HandleTypeDef*, uint32_t>;

    Can(FDCAN_HandleTypeDef* hal_can_handle, uint32_t hal_filter_index)
        : hal_can_handle_(hal_can_handle) {
        // 防呆: 表引用的控制器必须是 CubeMX 真的初始化过的(MX_FDCANx_Init 后
        // State=READY; 从未初始化则为 RESET)。kCanPorts 表与 .ioc 漂移在此即刻
        // 报错, 而不是等到线上行为诡异。
        core::utility::assert_always(hal_can_handle_->State != HAL_FDCAN_STATE_RESET);
        canfd_ = kCanPorts[diag_index()].mode == CanMode::kCanFd;
        config_can(hal_filter_index);
    }

    // 本总线当前实际发送的帧型, 可经 EP0 配置通道在运行时切换
    // (usb/vendor_control.cpp 的清单应用)。控制器自身保持
    // FD 能力 -- CubeMX 以 FDCAN_FRAME_FD_BRS 启动它, 这里不进 INIT 模式 -- 切换
    // 只改本驱动写进 Tx 元素的 FDF/BRS 标志; 收方向两个模式都是超集。
    [[nodiscard]] bool fd_mode() const { return canfd_; }
    void set_fd_mode(bool fd) { canfd_ = fd; }

    // ---- 启停: 总线只在主机声明之后才工作 ----
    //
    // 构造只把控制器配置好并留在 INIT 模式: 不上总线、不应答别人的帧、不发错误帧,
    // 也没有任何中断。声明清单里有这一路, 就是主机声明要用这条总线,
    // 控制器随即启动(core 的清单应用 -> resume()); 新的清单不再声明它、或会话结束
    // (ports::Registry::suspend_all())时停掉。帧型先于启动设好, 所以控制器一上总线发的就是
    // 主机要的帧型。
    //
    // 只在主循环调用(EP0 处理器与会话状态机都经 tud_task() / 主循环到达)。
    [[nodiscard]] bool started() const { return started_; }

    void start() {
        if (started_)
            return;
        constexpr auto ok = HAL_OK;
        core::utility::assert_always(HAL_FDCAN_Start(hal_can_handle_) == ok);
        core::utility::assert_always(
            HAL_FDCAN_ActivateNotification(hal_can_handle_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)
            == ok);
        // 总线关闭恢复: 本板是转发桥, 瞬时线路故障(下游节点拔掉、无 ACK)不能让端口
        // 挂死到重启为止。该通知触发 HAL_FDCAN_ErrorStatusCallback(见 can.cpp),
        // 重启 bus-off 恢复流程, 端口可自行复活。
        core::utility::assert_always(
            HAL_FDCAN_ActivateNotification(hal_can_handle_, FDCAN_IT_BUS_OFF, 0) == ok);
        // 协议错误(仲裁段 / 数据段)各进一次中断, 当场读 PSR 锁存错误码(note_bus_errors)。
        // 不能等主循环: LEC/DLEC 在下一帧成功收发后就被硬件清成 kNone, 总线忙时 250 ms 一读
        // 几乎永远看不到错误码。单发模式下每个出错的帧至多一次, 中断数以帧率为上限。
        core::utility::assert_always(
            HAL_FDCAN_ActivateNotification(
                hal_can_handle_, FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR, 0)
            == ok);
        started_ = true;
        loop::set(loop::bit(kCanPorts[diag_index()].data_id));
    }

    // 定义在 can.cpp: 要清本控制器在 transmit_pending_mask_ 里的那一位。
    void stop();

    // 软件 TX 环的深度。
    static constexpr size_t kTransmitQueueSize = 64;

    // CAN 转发热路径。函数体在 can.cpp 中定义并放入零等待 ITCM(.itcm 段),
    // 使最坏转发延迟不受 I-cache miss 和 FLASH-XIP 取指抖动影响。
    // 必须 out-of-line: inline/COMDAT 函数体放自定义 section 会触发 GCC section 类型冲突。
    void handle_downlink(const data::CanDataView& data);
    void handle_uplink(data::DataId field_id, core::protocol::Serializer& serializer);

    // 把本控制器的软件队列排入硬件 Tx FIFO。只有 FIFO 满过队列才会有内容, 故
    // 几乎总为空; 判空内联在此, 空队列零调用开销。主循环走
    // drain_pending_transmits(), 这个单总线形式留给只关心单个控制器的调用方。
    bool try_transmit() {
        if (transmit_buffer_.readable() == 0) [[likely]]
            return false;
        return drain_transmit_queue();
    }

    // 三路控制器共用的主循环入口, 只在至少有一条总线被声明(loop::active 的 kCanBuses
    // 位)时调用。transmit_pending_mask_ 每控制器一位, 表示其队列可能仍有帧, 三路全空的
    // 常见情况因此只需一次加载加一次分支。
    //
    // mask 是普通数据而非原子量: 读写双方都只在主循环运行 -- handle_downlink 仅
    // 经 tud_task() 到达(TinyUSB vendor class 未注册 xfer_isr, tud_vendor_rx_cb
    // 不会在 USB 中断里运行), 排空也只发生在这里。不变量: 队列非空时对应位必为 1。
    // handle_downlink 在每次入队尝试后置位, drain_transmit_queue 确认队列已空才
    // 清位。debug 构建在空闲路径上校验该不变量。
    static void drain_pending_transmits() {
        if (transmit_pending_mask_ == 0) [[likely]] {
            core::utility::assert_debug_lazy([]() noexcept { return transmit_queues_empty(); });
        } else {
            drain_pending_transmits_slow();
        }
    }

    // ES0491 (STM32H72x/73x) 2.22.3 的软件守护, 机制见 can.cpp 的
    // recover_stuck_transmits()。逐控制器跑一遍。由主循环的毫秒杂务调用(每毫秒一次,
    // 且只在有总线被声明时): 判据是"挂起超过 20 ms", 毫秒一查绰绰有余, 而主循环的
    // 每一趟不必为它付任何东西。定义在 can.cpp, 因为 can1/2/3 声明在本类之后。
    static void recover_all_stuck_transmits();

    // 仅 HAL_FDCAN_ErrorStatusCallback(IR.BO 的置位/清零两个沿, 中断上下文)调用。
    // bus-off 恢复期间(129*11 个隐性位, 约 1.4 ms)挂起的发送请求合法地保持 TXBRP
    // 非零, 守护以该标志为屏蔽位; DTCM 内对齐字节写在 M7 上是原子的。
    void note_bus_off(bool bus_off) { bus_off_ = bus_off; }

    // ---- 端口接口(core/src/link/ 的通用 CAN 操作按这一组原语工作) ----
    //
    // 全部是冷路径(EP0 的清单声明与读回)。本板的速率由 CubeMX 预设钉死, 声明里的
    // 速率只是核对 -- setting()/expected_of() 因此都指向硬件事实; 能改的只有帧型。
    [[nodiscard]] bool running() const { return started_; }
    void suspend() { stop(); }
    void resume() { start(); }
    [[nodiscard]] bool fd_now() const { return canfd_; }
    // 寄存器预设重构的位时序事实(定义在 can.cpp)。
    [[nodiscard]] link::CanTimingValue timing() const;
    [[nodiscard]] link::CanSetting setting() const {
        const link::CanTimingValue t = timing();
        return {
            .fd = canfd_,
            .arbitration_baudrate = t.arbitration_baudrate,
            .data_baudrate = t.data_baudrate};
    }
    [[nodiscard]] link::CanTimingValue expected_of(const link::CanSetting&) const {
        return timing(); // 帧型不影响上报的速率(控制器保持 FD 能力), 时序不变
    }
    // 应用 = 切 Tx 元素的 FDF/BRS 标志; 控制器不进 INIT, 速率不变。回读即 canfd_。
    [[nodiscard]] bool apply_setting(const link::CanSetting& s) {
        set_fd_mode(s.fd);
        return fd_now() == s.fd;
    }
    void read_config(vc::CanConfigPayload& out) const;
    // 运行时状态, 每个 keepalive 轮次在主循环读一次(见 can.cpp)。
    [[nodiscard]] data::CanStatusView read_status();
    // 协议错误中断(line0_isr)里调用: 读一次 PSR, 把真实错误码交给锁存。
    void note_bus_errors();
    [[nodiscard]] link::PortStatus describe() const { return {.running = started_, .fd = canfd_}; }

private:
    bool drain_transmit_queue();
    static void drain_pending_transmits_slow();
    static bool transmit_queues_empty();

    void recover_stuck_transmits();

    bool bus_off_ = false;

    // 控制器是否在总线上。只在主循环写; RX / bus-off 中断只读, DTCM 内对齐字节的
    // 读写在 M7 上是原子的。
    bool started_ = false;

    // 非 0 表示"TXBRP 非零且非 bus-off"自该 HAL_GetTick 毫秒起持续; 0 表示无。
    uint32_t stuck_request_since_ms_ = 0;

    // diag_index() 对应的位, 置位表示该控制器的队列可能仍有帧; 放在控制器旁的
    // 零等待 DTCM。见 drain_pending_transmits()。
    [[gnu::section(".dtcm")]] static inline constinit uint32_t transmit_pending_mask_ = 0;

    void config_can(uint32_t hal_filter_index) {
        FDCAN_FilterTypeDef filter_config;

        filter_config.IdType = FDCAN_STANDARD_ID;
        filter_config.FilterIndex = hal_filter_index;
        filter_config.FilterType = FDCAN_FILTER_MASK;
        filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter_config.FilterID1 = 0x0000;
        filter_config.FilterID2 = 0x0000;

        constexpr auto ok = HAL_OK;
        core::utility::assert_always(HAL_FDCAN_ConfigFilter(hal_can_handle_, &filter_config) == ok);

        // 扩展帧靠下面的全局过滤器放行, 而非独立过滤器: CubeMX 配置 ExtFiltersNbr = 0,
        // message RAM 里没有扩展过滤器表; HAL 只用 assert_param(已编译掉)校验下标,
        // 此处再调 ConfigFilter 会把元素写进 RX FIFO0 的 RAM 区且不生效。
        // 显式配置 GFC 也免于依赖寄存器复位值。远程帧仍走正常过滤,
        // 由 handle_uplink() 归一化为空负载。
        core::utility::assert_always(
            HAL_FDCAN_ConfigGlobalFilter(
                hal_can_handle_, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)
            == ok);

        // 硬件 RX 时间戳: 外部计数器(TSCC.TSS = 10), 即 TIM3 [RM0468 61.5.8], 在帧起始沿锁进
        // RX 元素 R1[15:0]; handle_uplink() 经 sync::sof_capture::stamp_of() 把它换成共享微帧
        // 轴上的位置(CanDataView::sof_stamp)。内部计数器不用: 一拍是一个标称位时间
        // (1 Mbit/s 下 1 us), 与 SOF 也不在同一个计数器上。总是开(TIM3 不走就锁 0, 没人
        // 读); 必须在 READY 态执行(start() 里的 HAL_FDCAN_Start 之前)。
        //
        // TIM3 只有 16 位: 帧起始到接收中断若超过它的回绕窗口, 时间戳认不出过了几圈。
        // 按本总线的标称位时间估最长的帧(经典扩展帧 8 字节、填充位最多, 约 160 位)
        // 加 100 us 中断余量, 窗口盖不住就不给这条总线的帧打戳。1 Mbit/s 是 260 us,
        // 窗口 357 us(TIM3 7.3 ns 一拍, 回绕 477 us 的 3/4); 本板速率由 .ioc 钉死在
        // 1M/5M, 低于约 620 kbit/s 的总线才会不打戳。窗口在第一次开时间基准时才定下
        // (sof_capture::configure()), 所以这里只算帧龄上界, 接收中断逐帧问。
        core::utility::assert_always(
            HAL_FDCAN_EnableTimestampCounter(hal_can_handle_, FDCAN_TIMESTAMP_EXTERNAL) == ok);
        const auto& init = hal_can_handle_->Init;
        const std::uint64_t kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
        const std::uint64_t bit_tq =
            init.NominalPrescaler * (1U + init.NominalTimeSeg1 + init.NominalTimeSeg2);
        if (kernel_hz != 0U) {
            const std::uint64_t longest_frame_ns = (160U * bit_tq * 1'000'000'000U) / kernel_hz;
            longest_frame_age_ns_ = static_cast<std::uint32_t>(longest_frame_ns + 100'000U);
        }

        // 发送延迟补偿(TDC)。数据段速率 =
        // 80 MHz / (DataPrescaler 1 * (1 + DataTimeSeg1 13 + DataTimeSeg2 2)) = 5 Mbit/s,
        // 即一位 200 ns, 主采样点在 14/16 = 87.5%, 也就是 175 ns。高速 CAN 收发器的
        // 环路延迟通常 120-255 ns, 可能超过该采样点: 发送数据段时节点会过早回读自己的位,
        // 读到上一位而报 bit error。TDC 把回读移到二级采样点
        // (实测环路延迟 + TdcOffset), 跟随收发器而不是假定它足够快。
        //
        // TdcOffset 以数据段时间量子为单位, 取 DataPrescaler * DataTimeSeg1 --
        // 让二级采样点落在位内同一相对位置的标准做法。TdcFilter = 0 关闭滤波窗口,
        // 收发器无毛刺问题时的常规选择。
        //
        // STM32F407 完全没有 CAN-FD, 所以 c_board 没有对应逻辑。
        // 必须在 READY 态执行, 即 start() 里的 HAL_FDCAN_Start 之前。
        const uint32_t tdc_offset =
            hal_can_handle_->Init.DataPrescaler * hal_can_handle_->Init.DataTimeSeg1;
        core::utility::assert_always(
            HAL_FDCAN_ConfigTxDelayCompensation(hal_can_handle_, tdc_offset, 0) == ok);
        core::utility::assert_always(HAL_FDCAN_EnableTxDelayCompensation(hal_can_handle_) == ok);

        // 控制器留在 READY(INIT 模式), 不在此启动: 见 start()。
    }

    // 遥测用的逻辑编号, 由外设实例推导而非存储: 构造函数不带索引参数,
    // 为一个纯诊断值新增参数会波及所有调用点。
    [[nodiscard]] std::size_t diag_index() const {
        if (hal_can_handle_->Instance == FDCAN1)
            return 0;
        if (hal_can_handle_->Instance == FDCAN2)
            return 1;
        return 2;
    }

    FDCAN_HandleTypeDef* hal_can_handle_;

    // 本控制器往总线上发送的帧格式, 构造时由 kCanPorts 固定。
    bool canfd_ = false;

    // 帧起始到接收中断最长可能隔多久(ns, 见 config_can()): 时间基准开着、且它装得进
    // TIM3 的回绕窗口时, 收到的帧才带共享微帧轴上的时间戳。0 = 内核时钟读不出, 不打戳。
    std::uint32_t longest_frame_age_ns_ = 0;

    // 板子自己丢的帧(read_status() 报给主机): 下行撞上发送队列满(主循环写),
    // 上行撞上上行批量池满(RX 中断写)。
    utility::EventCounter tx_dropped_;
    utility::EventCounter rx_dropped_;
    // 单发作废的帧(push_to_hardware() 清点, 主循环写)与 RX FIFO0 溢出(read_status() 记)。
    utility::EventCounter cancelled_frames_;
    utility::EventCounter rx_lost_;
    // 最近一次真实错误码(仲裁段 / 数据段), 只协议错误中断(note_bus_errors)写。
    utility::LatchedBusError last_bus_error_;
    utility::LatchedBusError last_data_bus_error_;
    // 本次启动以来写过帧的发送槽(只在主循环读写; stop() 清零)。bus-off 恢复不清: 那在
    // 中断里, 清了就与主循环抢写; 代价是恢复后至多一个 FIFO 深度的槽可能被多计作废。
    uint32_t used_tx_slots_ = 0;

    struct TransmitMailboxData {
        uint32_t identifier; // Tx 元素 T0: ID + XTD/RTR 标志
        uint32_t control;    // Tx 元素 T1: DLC + FDF/BRS 标志
        uint32_t data[2];
    };

    // 读取控制器 Tx FIFO/队列的空闲元素数, 以及向其写入一个元素。两者都在转发热路径上
    // (.itcm, 定义在 can.cpp), 由 handle_downlink 的直写和 try_transmit 的排空共用。
    [[nodiscard]] uint32_t hardware_free_slots() const noexcept;
    void push_to_hardware(const TransmitMailboxData& mailbox_data) noexcept;

    // 硬件 32 元素 Tx FIFO 之后的溢出队列 -- 只有 FIFO 满时才会用到,
    // 否则 handle_downlink 直接写控制器。16 是从 c_board 继承的: 那边 bxCAN 只有
    // 三个发送邮箱, 16 深确有收益; 本芯片光 FIFO 就有 32, 旧的无条件入队反而把单个
    // 下行包限制在 16 帧 -- 只有直写 FIFO 的一半。64 与 hpm_board 一致,
    // 每路总线占 1 KB DTCM。深度常量在 public 区(kTransmitQueueSize)。
    utility::RingBuffer<TransmitMailboxData, kTransmitQueueSize> transmit_buffer_;
};

// 放在零等待 DTCM(.dtcm): RX 中断从这里读 hal_can_handle_/data_id_,
// 发送环形队列也在这里 -- 让转发热路径避开 AXI 总线。
[[gnu::section(".dtcm")]] inline constinit Can::Lazy can1{&hfdcan1, 0};
[[gnu::section(".dtcm")]] inline constinit Can::Lazy can2{&hfdcan2, 0};
[[gnu::section(".dtcm")]] inline constinit Can::Lazy can3{&hfdcan3, 0};

// 取 EP0 总线序(kCanPorts 顺序)对应的控制器。init() 之前返回 nullptr --
// EP0 处理器对这种总线直接 STALL 请求而不解引用, 与 hpm_board 未构造尾部槽位的
// 策略一致。
inline Can* can_by_index(size_t index) {
    switch (index) {
    case 0: return can1.get();
    case 1: return can2.get();
    case 2: return can3.get();
    default: return nullptr;
    }
}

// 三路 FDCAN 第 0 中断线的处理器(can.cpp, 放在 ITCM)写进向量表。App 在把向量表搬进
// DTCM 之后调用(app.cpp), 于是这三条接收中断从取向量到转发全程不碰 FLASH。
void install_interrupt_vectors(std::uint32_t* vectors);

} // namespace libhcs::firmware::can
