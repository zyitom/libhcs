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
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/led/led.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"
#include "firmware/mc02/app/src/utility/ring_buffer.hpp"

namespace libhcs::firmware::can {

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

// 表序即总线序: 下标 0 对应丝印 CAN1(kCan1), 依此类推, 与
// core/include/libhcs/spec/mc02/can.hpp 及 can.cpp 的 ISR 分发一致。三处需同步维护。
inline constexpr CanPort kCanPorts[] = {
    {.handle = &hfdcan1, .data_id = data::DataId::kCan1, .mode = CanMode::kCanFd},
    {.handle = &hfdcan2, .data_id = data::DataId::kCan2, .mode = CanMode::kCanFd},
    {.handle = &hfdcan3, .data_id = data::DataId::kCan3, .mode = CanMode::kCanFd},
};
inline constexpr size_t kCanCount = std::size(kCanPorts);
static_assert(kCanCount == 3);

class Can : private core::utility::Immovable {
public:
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
    // (usb/vendor_control.cpp 的 kSetCanConfig + kCanConfigApply)。控制器自身保持
    // FD 能力 -- CubeMX 以 FDCAN_FRAME_FD_BRS 启动它, 这里不进 INIT 模式 -- 切换
    // 只改本驱动写进 Tx 元素的 FDF/BRS 标志; 收方向两个模式都是超集。
    [[nodiscard]] bool fd_mode() const { return canfd_; }
    void set_fd_mode(bool fd) { canfd_ = fd; }

    // 软件 TX 环的深度。
    static constexpr size_t kTransmitQueueSize = 64;

    // CAN 转发热路径。函数体在 can.cpp 中定义并放入零等待 ITCM(.itcm 段),
    // 使最坏转发延迟不受 I-cache miss 和 FLASH-XIP 取指抖动影响。
    // 必须 out-of-line: inline/COMDAT 函数体放自定义 section 会触发 GCC section 类型冲突。
    void handle_downlink(const data::CanDataView& data);
    void handle_uplink(data::DataId field_id, core::protocol::Serializer& serializer);

    // 控制器错误状态, 供 EP0 状态查询(usb/vendor_control.cpp)使用。每次调用直接
    // 读硬件: PSR 的 LEC/DLEC 读后自清为"无变化", 缓存副本会永远报旧错误, 而跳过
    // 读取则会对唯一的另一读者隐瞒错误。那个读者是 bus-off 恢复路径, 它读的是
    // 协议状态里的 BusOff -- 电平标志, 不会自清 -- 因此两者不会互相抹掉对错误
    // 锁存的观察。
    struct Status {
        uint8_t tec, rec, last_error, data_last_error, flags;
        uint32_t tx_occurred, tx_cancelled, rx_frames, rx_fifo_level;
    };

    [[nodiscard]] Status status() const;

    // 把本控制器的软件队列排入硬件 Tx FIFO。只有 FIFO 满过队列才会有内容, 故
    // 几乎总为空; 判空内联在此, 空队列零调用开销。主循环走
    // drain_pending_transmits(), 这个单总线形式留给只关心单个控制器的调用方。
    bool try_transmit() {
        if (transmit_buffer_.readable() == 0) [[likely]]
            return false;
        return drain_transmit_queue();
    }

    // 三路控制器共用的主循环入口。transmit_pending_mask_ 每控制器一位, 表示其
    // 队列可能仍有帧, 三路全空的常见情况因此只需一次加载加一次分支。
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

        // ES0491 2.22.3 守护: 每 kStuckCheckPassInterval 趟主循环查一次各控制器的
        // 挂起发送请求(50-85 kHz 循环下约每 6-10 ms), 详见 recover_stuck_transmits()。
        // 三次 D2 域寄存器读摊在 512 趟上; 热路径只在命中检查的那一趟多付一次分支。
        if ((++stuck_check_phase_ & (kStuckCheckPassInterval - 1U)) == 0U) [[unlikely]]
            recover_all_stuck_transmits();
    }

    // 仅 HAL_FDCAN_ErrorStatusCallback(IR.BO 的置位/清零两个沿, 中断上下文)调用。
    // bus-off 恢复期间(129*11 个隐性位, 约 1.4 ms)挂起的发送请求合法地保持 TXBRP
    // 非零, 守护以该标志为屏蔽位; DTCM 内对齐字节写在 M7 上是原子的。
    void note_bus_off(bool bus_off) { bus_off_ = bus_off; }

private:
    bool drain_transmit_queue();
    static void drain_pending_transmits_slow();
    static bool transmit_queues_empty();

    // ES0491 (STM32H72x/73x) 2.22.3 的软件守护, 机制见 can.cpp 的
    // recover_stuck_transmits()。逐控制器跑一遍; 定义在 can.cpp, 因为 can1/2/3
    // 声明在本类之后, 类内看不到。
    static void recover_all_stuck_transmits();
    void recover_stuck_transmits();

    bool bus_off_ = false;

    // 非 0 表示"TXBRP 非零且非 bus-off"自该 HAL_GetTick 毫秒起持续; 0 表示无。
    uint32_t stuck_request_since_ms_ = 0;

    // 守护检查的节流相位与间隔: 主循环每 512 趟查一次。
    static constexpr uint32_t kStuckCheckPassInterval = 512;
    [[gnu::section(".dtcm")]] static inline constinit uint32_t stuck_check_phase_ = 0;

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

        // 不启用硬件 RX 时间戳: 内部计数器只有 16 位, 约 65.5 ms 回绕,
        // 无法承载协议要求的 32 位微秒时间戳。handle_uplink() 不填
        // CanDataView::timestamp_us, 开着只会白耗总线周期。若上行恢复上报时间戳,
        // 取消下面两行注释即可; 它们必须在控制器仍处于 READY 态时执行
        // (即 HAL_FDCAN_Start 之前)。
        //
        // 内部 16 位计数器, 一个 tick = 一个标称位时间, 在 1 Mbit/s 仲裁速率下即 1 us。
        // core::utility::assert_always(
        //     HAL_FDCAN_ConfigTimestampCounter(hal_can_handle_, FDCAN_TIMESTAMP_PRESC_1) == ok);
        // core::utility::assert_always(
        //     HAL_FDCAN_EnableTimestampCounter(hal_can_handle_, FDCAN_TIMESTAMP_INTERNAL) == ok);

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
        // 必须在 READY 态执行, 即下面 HAL_FDCAN_Start 之前。
        const uint32_t tdc_offset =
            hal_can_handle_->Init.DataPrescaler * hal_can_handle_->Init.DataTimeSeg1;
        core::utility::assert_always(
            HAL_FDCAN_ConfigTxDelayCompensation(hal_can_handle_, tdc_offset, 0) == ok);
        core::utility::assert_always(HAL_FDCAN_EnableTxDelayCompensation(hal_can_handle_) == ok);

        core::utility::assert_always(HAL_FDCAN_Start(hal_can_handle_) == ok);
        core::utility::assert_always(
            HAL_FDCAN_ActivateNotification(hal_can_handle_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)
            == ok);
        // 总线关闭恢复: 本板是转发桥, 瞬时线路故障(下游节点拔掉、无 ACK)不能让端口
        // 挂死到重启为止。该通知触发 HAL_FDCAN_ErrorStatusCallback(见 can.cpp),
        // 重启 bus-off 恢复流程, 端口可自行复活。
        core::utility::assert_always(
            HAL_FDCAN_ActivateNotification(hal_can_handle_, FDCAN_IT_BUS_OFF, 0) == ok);
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

    // 开机以来交给 serializer 的帧数, 由 EP0 状态查询上报。仅 RX 中断写、仅主
    // 循环读; Cortex-M 上对齐的 32 位读写是原子的, 且这里只关心数值变化, 无需同步。
    uint32_t forwarded_frames_ = 0;

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

} // namespace libhcs::firmware::can
