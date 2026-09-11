#pragma once

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

class Can : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Can, FDCAN_HandleTypeDef*, uint32_t>;

    Can(FDCAN_HandleTypeDef* hal_can_handle, uint32_t hal_filter_index)
        : hal_can_handle_(hal_can_handle) {
        config_can(hal_filter_index);
    }

    // CAN 转发热路径。函数体在 can.cpp 中定义并放入零等待 ITCM(.itcm 段),
    // 使最坏转发延迟不受 I-cache miss 和 FLASH-XIP 取指抖动影响。
    // 必须 out-of-line: inline/COMDAT 函数体放自定义 section 会触发 GCC section 类型冲突。
    void handle_downlink(const data::CanDataView& data);
    void handle_uplink(data::DataId field_id, core::protocol::Serializer& serializer);

    // Drain this controller's software queue into the hardware Tx FIFO. The
    // queue only fills when the FIFO was full, so it is almost always empty, and
    // the test is inline so an empty queue costs no call. The main loop uses
    // drain_pending_transmits() instead; this per-bus form stays for callers
    // that want a single controller.
    bool try_transmit() {
        if (transmit_buffer_.readable() == 0) [[likely]]
            return false;
        return drain_transmit_queue();
    }

    // Main-loop entry for all three controllers. transmit_pending_mask_ has one
    // bit per controller whose queue may hold frames, so a pass with nothing
    // queued on any bus -- nearly every pass -- is one load and one branch.
    //
    // The mask is plain data rather than an atomic because every reader and
    // writer runs in the main loop: handle_downlink is reached only from
    // tud_task() (the TinyUSB vendor class registers no xfer_isr, so
    // tud_vendor_rx_cb never runs in the USB interrupt), and the drains run from
    // here. Invariant: a controller's bit is set whenever its queue is
    // non-empty. handle_downlink sets it after every enqueue attempt, and
    // drain_transmit_queue clears it only after finding the queue empty. Debug
    // builds check the invariant on the idle path.
    static void drain_pending_transmits() {
        if (transmit_pending_mask_ == 0) [[likely]] {
            core::utility::assert_debug_lazy([]() noexcept { return transmit_queues_empty(); });
            return;
        }
        drain_pending_transmits_slow();
    }

private:
    bool drain_transmit_queue();
    static void drain_pending_transmits_slow();
    static bool transmit_queues_empty();

    // Bit diag_index() set while that controller's queue may hold frames, in
    // zero-wait DTCM next to the controllers. See drain_pending_transmits().
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
            HAL_FDCAN_ActivateNotification(
                hal_can_handle_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)
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
    // 每路总线占 1 KB DTCM。
    static constexpr size_t kTransmitQueueSize = 64;
    utility::RingBuffer<TransmitMailboxData, kTransmitQueueSize> transmit_buffer_;
};

// 放在零等待 DTCM(.dtcm): RX 中断从这里读 hal_can_handle_/data_id_,
// 发送环形队列也在这里 -- 让转发热路径避开 AXI 总线。
[[gnu::section(".dtcm")]] inline constinit Can::Lazy can1{&hfdcan1, 0};
[[gnu::section(".dtcm")]] inline constinit Can::Lazy can2{&hfdcan2, 0};
[[gnu::section(".dtcm")]] inline constinit Can::Lazy can3{&hfdcan3, 0};

} // namespace libhcs::firmware::can
