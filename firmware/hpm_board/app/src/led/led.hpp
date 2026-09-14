#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "firmware/hpm_board/app/src/led/gpio_led.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::led {

inline auto& led_backend = gpio_led;

// CAN 总线故障分类。刻意粗化: 控制器只能可靠区分"无人应答我的帧"与"线上位
// 出错"。具体协议错误(stuff/form/bit/CRC)逐帧波动, 并不对应单一物理原因 --
// 反接、缺 120R 终端电阻、短路、波特率不匹配、普通噪声都产生同样的混合 -- 故
// 合并为一个 BUS-ERROR 态, 而不是假装可以区分。每个状态渲染成一种明显不同、
// 易读的指示灯图案(见 Led::update() 的表)。
enum class CanFault : uint8_t {
    kNone = 0,    // 健康/无错误
    kNoAck,       // ACK 错误 -- 总线上只有自己/对端未上电/TX 线断
    kWiringFault, // Bit0 错误: 无法把总线驱动为显性 -- CAN_H/L 短接、反接或
                  // 断开(最常见的物理接线错误)
    kSignalError, // stuff/form/CRC(以及罕见的 Bit1 "显性卡死"): 出错的位, 其
                  // 原因 -- 缺 120R 终端电阻、波特率不匹配、噪声 -- 逐帧波动,
                  // 无法区分
    kBusOff,      // 控制器 bus-off -- 错误过多, 恢复中/离线
};

class Led {
public:
    using Lazy = utility::Lazy<Led>;

    Led() {
        led_backend.init();
        board::init_can_indicator_pins();
    }

    void reset() {
        uplink_full_reset_counter_.store(0, std::memory_order::relaxed);
        downlink_full_reset_counter_.store(0, std::memory_order::relaxed);
    }

    void uplink_buffer_full() {
        uplink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }

    void downlink_buffer_full() {
        downlink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }

    void update(uint32_t tick) {
        uint16_t uplink_full;
        do {
            uplink_full = uplink_full_reset_counter_.load(std::memory_order::relaxed);
            if (uplink_full == 0)
                break;
        } while (!uplink_full_reset_counter_.compare_exchange_weak(
            uplink_full, uplink_full - 1, std::memory_order::relaxed));

        uint16_t downlink_full;
        do {
            downlink_full = downlink_full_reset_counter_.load(std::memory_order::relaxed);
            if (downlink_full == 0)
                break;
        } while (!downlink_full_reset_counter_.compare_exchange_weak(
            downlink_full, downlink_full - 1, std::memory_order::relaxed));

        // 三通道 GPIO RGB LED(每色只有开/关, 无 PWM 调光)。颜色语言与 c_board
        // 指示灯一致, 两种板读法相同; 黄=红+绿、青=绿+蓝, 三个通道从不同时
        // 全亮以保证状态可读:
        //   常绿        = 主机会话已建立(数据转发中)
        //   绿色慢闪    = 存活, 等待主机会话
        //   黄色闪烁    = 上行(板 -> 主机)缓冲满
        //   青色闪烁    = 下行(主机 -> 板)缓冲满
        //   黄/青交替   = 双向拥塞
        const bool on = (tick & 128U) != 0;
        if (uplink_full && downlink_full) {
            led_backend->set_value(on ? 255 : 0, 255, on ? 0 : 255);
        } else if (uplink_full) {
            led_backend->set_value(on ? 255 : 0, on ? 255 : 0, 0);
        } else if (downlink_full) {
            led_backend->set_value(0, on ? 255 : 0, on ? 255 : 0);
        } else if (host_connected_.load(std::memory_order::relaxed)) {
            led_backend->set_value(0, 255, 0);
        } else {
            led_backend->set_value(0, (tick & 512U) ? 255 : 0, 0);
        }

        // CAN 总线指示灯语言: 每个 CAN 控制器一颗独立 LED, 取自板级
        // kCanIndicatorPins 表(没有指示灯的板为空)。四种控制器真正能区分的
        // 状态, 每种都是明显不同、易读的图案:
        //   熄灭      = 健康/无错误
        //   慢闪      = NO-ACK: 无人应答 -- 总线上只有自己、对端未上电或 TX 线断
        //   快闪      = 接线故障(Bit0): 总线驱动不成显性 -- CAN_H/L 短接、反接
        //              或断开
        //   双闪      = 信号错误: 出错的位 -- 缺 120R 终端电阻、波特率不匹配或
        //              噪声(无法区分)
        //   常亮      = BUS-OFF: 错误过多, 控制器恢复中/离线
        // CAN ISR 在每次错误中断时刷新对应控制器的故障, 此处自最后一次错误起
        // 约 5 s 后衰减回熄灭。循环上界取运行时数量而非表容量: hpm5321 镜像的
        // 表按双 CAN PCB 尺寸分配, 单 CAN PCB 则一个指示灯都没有 -- 它的
        // PB14/PB15 未焊, 驱动它们等于写本板不使用的焊盘。
        for (size_t i = 0; i < board::can_indicator_count(); ++i) {
            uint16_t timeout = can_fault_timeout_[i].load(std::memory_order::relaxed);
            if (timeout) {
                --timeout;
                can_fault_timeout_[i].store(timeout, std::memory_order::relaxed);
            }
            const CanFault fault =
                timeout ? can_fault_[i].load(std::memory_order::relaxed) : CanFault::kNone;
            bool led_on = false;
            switch (fault) {
            case CanFault::kNoAck: led_on = (tick % 1000U) < 500U; break;      // 约 1 Hz
            case CanFault::kWiringFault: led_on = (tick % 200U) < 100U; break; // 约 5 Hz
            case CanFault::kSignalError: {                                     // 两次快闪, 然后停顿
                const uint32_t phase = tick % 1200U;
                led_on = phase < 120U || (phase >= 240U && phase < 360U);
                break;
            }
            case CanFault::kBusOff: led_on = true; break; // 常亮
            case CanFault::kNone: led_on = false; break;
            }
            board::kCanIndicatorPins[i].set_active(led_on);
        }
    }

    void set_host_connected(bool connected) {
        host_connected_.store(connected, std::memory_order::relaxed);
    }

    // CAN 总线故障灯码记录: 由 CAN ISR 以控制器下标(0 起)和当前故障调用。
    // 超时让指示灯在最后一次错误中断后仍可见约 5 s。kNone 只刷新超时并保留
    // 最近的具体故障, 使不带新 LEC 的总线状态变化(warning/passive)不会抹掉
    // 刚刚报告过的故障。
    void report_can_fault(uint8_t can_index, CanFault fault) {
        // 这里守的是数组容量上界; 槽位是否真有 LED 由 update() 管: 给没有
        // LED 的槽位记录故障无害, 内存安全真正依赖的是这个检查。
        if (can_index >= kCanIndicatorCount)
            return;
        can_fault_timeout_[can_index].store(kCanFaultTimeoutTicks, std::memory_order::relaxed);
        if (fault != CanFault::kNone)
            can_fault_[can_index].store(fault, std::memory_order::relaxed);
    }

private:
    // 板级指示灯表的容量, 决定下方状态数组的大小。board::can_indicator_count()
    // 是实际存在的 LED 数; 两者只在 hpm5321 镜像上不同 -- 表按双 CAN PCB 尺寸
    // 分配, 而单 CAN PCB 一个指示灯都没有。
    static constexpr size_t kCanIndicatorCount = board::kCanIndicatorPins.size();

    // CAN 故障指示状态 -- 经原子存储保证 ISR 安全。每个 CAN 控制器一组
    // fault/timeout, 由 CAN ISR 在每次错误中断时刷新。
    static constexpr uint16_t kCanFaultTimeoutTicks = 5000; // 1 kHz tick 下的 5 s
    std::array<std::atomic<uint16_t>, kCanIndicatorCount> can_fault_timeout_{};
    std::array<std::atomic<CanFault>, kCanIndicatorCount> can_fault_{};

    std::atomic<uint16_t> uplink_full_reset_counter_{0};
    std::atomic<uint16_t> downlink_full_reset_counter_{0};
    std::atomic<bool> host_connected_{false};
};

inline constinit Led::Lazy led;

} // namespace libhcs::firmware::led
