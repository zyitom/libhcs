#pragma once

#include <cstddef>
#include <cstdint>
#include <iterator>

#include <can.h>
#include <stm32f4xx_hal_can.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/spec/c_board/ports.hpp"
#include "core/src/link/port.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/led/led.hpp"
#include "firmware/c_board/app/src/utility/loop_work.hpp"
#include "firmware/common/app/src/utility/event_counter.hpp"
#include "firmware/common/app/src/utility/latched_bus_error.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"
#include "firmware/common/app/src/utility/ring_buffer.hpp"

namespace libhcs::firmware::can {

namespace vc = libhcs::core::protocol::vendor_control;
namespace link = libhcs::core::link;

struct CanPort {
    CAN_HandleTypeDef* handle;
    data::DataId data_id; // 该连接器的丝印编号
};

// 表序即总线序: 下标 0 对应丝印 CAN1。身份引用 spec/c_board/ports.hpp 的具名描述符, 行序
// 由 ports.hpp 的 static_assert 钉死。帧型不在表里: bxCAN 只有经典帧。与 mc02 的 kCanPorts
// 同一写法。
inline constexpr CanPort kCanPorts[] = {
    {.handle = &hcan1, .data_id = spec::c_board::Spec::Cans::kCan1.data_id},
    {.handle = &hcan2, .data_id = spec::c_board::Spec::Cans::kCan2.data_id},
};
inline constexpr size_t kCanCount = std::size(kCanPorts);
static_assert(kCanCount == 2);

class Can : private core::utility::Immovable {
public:
    // EP0 口能力: 无。bxCAN 没有 FD, 帧型与两段速率都是编译期事实, 声明只能核对。
    static constexpr uint8_t kPortCapabilities = 0;

    using Lazy = utility::Lazy<Can, CAN_HandleTypeDef*, uint32_t, uint32_t>;

    Can(CAN_HandleTypeDef* hal_can_handle, uint32_t hal_filter_bank,
        uint32_t hal_slave_start_filter_bank)
        : hal_can_handle_(hal_can_handle)
        , index_(index_of(hal_can_handle)) {
        core::utility::assert_always(index_ < kCanCount);
        config_can(hal_filter_bank, hal_slave_start_filter_bank);
    }

    // 这一路在 kCanPorts 里的下标与身份。下标由句柄查表得到, 不另设构造参数: 表与
    // can1/can2 的句柄对不上即在上电时断言失败。
    [[nodiscard]] std::size_t index() const { return index_; }
    [[nodiscard]] data::DataId data_id() const { return kCanPorts[index_].data_id; }

    // 句柄 -> 下标。中断回调(can.cpp)与构造函数共用。表外的句柄返回 kCanCount。
    [[nodiscard]] static std::size_t index_of(const CAN_HandleTypeDef* handle) {
        for (std::size_t index = 0; index < kCanCount; ++index) {
            if (kCanPorts[index].handle == handle)
                return index;
        }
        return kCanCount;
    }

    // 本控制器(STM32F407 bxCAN)没有 CAN-FD。协议也不再携带供驱动判断的逐帧类型
    // 标志: 帧型是总线的属性, 本板的总线构造上就是 classic -- CBoard 主机类不提供
    // 请求 FD 的途径。
    //
    // CAN 转发热路径, 函数体定义在 can.cpp, 与 mc02 / hpm 的主循环 CAN 入口同名同
    // 职责: handle_downlink 由 USB 下行路径逐总线调用, handle_uplink 在接收中断里
    // 排空 RX FIFO0, drain_pending_transmits 是主循环的发送泵。
    void handle_downlink(const data::CanDataView& data);
    void handle_uplink(data::DataId field_id, core::protocol::Serializer& serializer);
    bool drain_pending_transmits();

    // ---- 端口接口(core/src/link/ 的通用 CAN 操作按这一组原语工作) ----
    //
    // bxCAN: 无 FD 能力, 位时序由 CubeMX 预设钉死, 声明里的每一项都只是核对(apply_setting
    // 是空操作)。没声明的总线不上线: 上电与会话结束时控制器停在 INIT 模式(不收、不发、
    // 不应答, 线上看不见它), 声明了才 start。同一条线上别的板可能在跑 CAN-FD -- 一个上着
    // 总线的经典控制器会把 FD 帧当格式错误打掉。[实测 2026-10-07, CBoardBench]
    [[nodiscard]] bool running() const { return started_; }
    void suspend() { stop(); }
    void resume() { start(); }
    [[nodiscard]] bool fd_now() const { return false; }
    // 寄存器(BTR)反推的位时序事实(定义在 can.cpp, 与 mc02 的 timing() 同法)。
    [[nodiscard]] link::CanTimingValue timing() const;
    [[nodiscard]] link::CanSetting setting() const {
        const link::CanTimingValue t = timing();
        return {.fd = false, .arbitration_baudrate = t.arbitration_baudrate, .data_baudrate = 0U};
    }
    [[nodiscard]] link::CanTimingValue expected_of(const link::CanSetting&) const {
        return timing(); // 时序是预设, 什么设置应用后都一样
    }
    [[nodiscard]] bool apply_setting(const link::CanSetting&) const { return true; }
    void read_config(vc::CanConfigPayload& out) const;
    // 错误码中断(HAL_CAN_ErrorCallback, CAN_SCE 向量)里调用。HAL 已把 ESR.LEC 译成
    // ErrorCode 的位并清掉了 LEC, 这里按位还原成 data::CanLastError 交给锁存(唯一的写者),
    // 再复位 ErrorCode -- HAL 只往上或, 不清。定义在 can.cpp。
    void note_bus_errors();
    // 运行时状态(core/src/link/port_status.hpp), 每个 keepalive 轮次在主循环读一次。
    // bxCAN 的 ESR: TEC/REC/LEC 与 M_CAN 的 PSR/ECR 同语义(LEC 编码同序); LEC 在成功收发
    // 后复位为 none, 错误码多半已被错误中断锁存(note_bus_errors), 这里只读: 自己这次读到
    // 真实错误就用它, 否则用锁存的。本板没有 FD 数据相位, 如实报 kNone。定义在 can.cpp。
    [[nodiscard]] data::CanStatusView read_status();
    [[nodiscard]] link::PortStatus describe() const { return {.running = started_, .fd = false}; }

private:
    // 单发作废的清点, 与 hpm_board / mc02 同法(重用发送槽时看它上一次的结果): 本板单发
    // (CubeMX 的 AutoRetransmission = DISABLE), 输了仲裁或出错的帧就此放弃, 不动 TEC/REC。
    // 邮箱上一次的请求完成了(RQCPx)却没成功(TXOKx = 0)就是作废了一帧; 写 1 清 RQCPx,
    // TXOKx / ALSTx / TERRx 随之清零。三个邮箱在 TSR 里各占 8 位。没有别人碰这几位:
    // 发送邮箱中断没开, HAL_CAN_IRQHandler 不处理它们。只在主循环(发送泵)里写。
    void note_previous_outcome(uint32_t status, uint32_t mailbox);

    // 仲裁段速率与采样点(千分比), 从 BTR 寄存器反推 -- 寄存器才是硬件事实, CubeMX
    // 的 Init 只是初始化时的请求。BRP/TS1/TS2 都是"值减 1"编码, 同步段恒 1 TQ;
    // bxCAN 挂在 APB1 上。42 MHz / (3 * (1+10+3)) = 1 Mbit/s, 采样点 (1+10)/14 = 785‰
    // (截断, 与 mc02 同一算法)。本控制器没有数据段。定义在 can.cpp。
    [[nodiscard]] uint32_t arbitration_baudrate() const;
    [[nodiscard]] uint32_t arbitration_sample_point() const;

    struct BitTiming {
        uint32_t prescaler;
        uint32_t seg1;
        uint32_t seg2;
        [[nodiscard]] uint32_t total() const { return 1U + seg1 + seg2; }
    };

    [[nodiscard]] BitTiming bit_timing() const;

    void config_can(uint32_t hal_filter_bank, uint32_t hal_slave_start_filter_bank) {
        CAN_FilterTypeDef filter_config;

        filter_config.FilterBank = hal_filter_bank;
        filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
        filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
        filter_config.FilterIdHigh = 0x0000;
        filter_config.FilterIdLow = 0x0000;
        filter_config.FilterMaskIdHigh = 0x0000;
        filter_config.FilterMaskIdLow = 0x0000;
        filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        filter_config.FilterActivation = CAN_FILTER_ENABLE;
        filter_config.SlaveStartFilterBank = hal_slave_start_filter_bank;

        constexpr auto ok = HAL_OK;
        core::utility::assert_always(HAL_CAN_ConfigFilter(hal_can_handle_, &filter_config) == ok);
        // 不在这里 start: 没声明的总线不上线(resume())。通知在 INIT 模式下就能打开。
        // 错误码中断(LEC + ERR, 走 CAN_SCE 向量): 出错当场锁存错误码(note_bus_errors)。
        // 不能等主循环: ESR.LEC 在下一帧成功收发后就被硬件清成 none。
        core::utility::assert_always(
            HAL_CAN_ActivateNotification(
                hal_can_handle_,
                CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR)
            == ok);
    }

    void start() {
        if (started_)
            return;
        core::utility::assert_always(HAL_CAN_Start(hal_can_handle_) == HAL_OK);
        started_ = true;
        loop::set(loop::bit(data_id()));
    }

    // 定义在 can.cpp: 中止在途请求并清发送队列, 归还会话结束前的作废清点。
    void stop();

    CAN_HandleTypeDef* hal_can_handle_;
    std::size_t index_; // kCanPorts 下标, 见 index()
    // 声明了才 true(resume)。只在主循环读写: EP0 的清单处理、会话结束与下行分发都经
    // tud_task() 在主循环里跑。
    bool started_ = false;

    struct TransmitMailboxData {
        uint32_t identifier;                // CAN_TxMailBox_TypeDef::TIR
        uint32_t data_length_and_timestamp; // CAN_TxMailBox_TypeDef::TDTR
        uint32_t data[2];                   // CAN_TxMailBox_TypeDef::TDLR & TDHR
    };
    utility::RingBuffer<TransmitMailboxData, 16> transmit_buffer_;

    // 板子自己丢的帧(read_status() 报给主机): 下行撞上发送环满(主循环写),
    // 上行撞上上行批量池满(接收中断写)。
    utility::EventCounter tx_dropped_;
    utility::EventCounter rx_dropped_;
    // 单发作废的帧(note_previous_outcome(), 主循环写)。
    utility::EventCounter cancelled_frames_;
    // RX FIFO0 溢出(RF0R.FOVR0)的次数, 每次至少丢一帧(接收中断写)。
    utility::EventCounter rx_lost_;
    utility::LatchedBusError last_bus_error_; // 只错误中断(note_bus_errors)写
};

// CAN 的 TX 环放在零等待的 CCM(.ccmram, 上电时由 App::App() 拷入)。写入方是 USB
// 下行路径, 读取方是这里的转发循环, 全程只经 CPU(没有 DMA 碰它们), 放 CCM 让它们
// 不占 AHB 总线。
[[gnu::section(".ccmram")]] inline constinit Can::Lazy can1{&hcan1, 0, 14};
[[gnu::section(".ccmram")]] inline constinit Can::Lazy can2{&hcan2, 14, 14};

// 取 kCanPorts 下标对应的控制器。init() 之前返回 nullptr。与 mc02 的 can_by_index() 同名同义。
inline Can* can_by_index(std::size_t index) {
    switch (index) {
    case 0: return can1.try_get();
    case 1: return can2.try_get();
    default: return nullptr;
    }
}

} // namespace libhcs::firmware::can
