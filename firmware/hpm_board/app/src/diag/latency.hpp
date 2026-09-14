#pragma once

#include <cstdint>

#include <hpm_csr_drv.h>

// 板侧延迟拆解, 可经 EP0 从量产镜像读取。
//
// 动机: 本链路上 CAN 往返约 118 us, 此前无法知道板子占其中多少。唯一的工具是
// libhcs_CAN_DIAG 构建, 其遥测骑在 DataId::kUart0 上, 会污染一切对该通道的测量
// (2026-09-04 曾为此付出六次重刷与一次 UART 假回归)。没有拆解, 每个延迟决策都
// 只能猜该攻哪一段。
//
// 计时两段, 全部在板内:
//
//   downlink  从进入 bulk OUT 完成回调, 到 CAN 帧写进控制器 TX FIFO。覆盖反序列
//             化、分发与 MCAN 写入。
//   uplink    从进入 CAN 接收中断, 到帧序列化进上行批。覆盖 FIFO 读取、归一化与
//             序列化。
//
// 刻意不计入: CAN 帧本身的线上时间(1M/5M FD 下 8 字节约 50 us)、USB 微帧量化、
// 主机自身的提交/唤醒路径 -- 它们是 118 us 的其余项, 本头文件的意义就是把板子的
// 份额对着它们量出来。
//
// 时钟源是 CSR_MCYCLE: 一条指令、核时钟分辨率、无副作用 -- 不像读外设寄存器那样
// 会扰动被测对象。can_diag 也因此用它; 这里两个调用点都在热路径上, 该性质更要紧。
namespace libhcs::firmware::diag::latency {

struct Segment {
    uint32_t count;
    uint32_t min_cycles;
    uint32_t max_cycles;
    uint64_t sum_cycles;

    void note(uint32_t cycles) {
        if (count == 0 || cycles < min_cycles)
            min_cycles = cycles;
        if (cycles > max_cycles)
            max_cycles = cycles;
        sum_cycles += cycles;
        ++count;
    }
};

// 由 USB 回调与 CAN 接收中断写入, 由主循环的 EP0 处理器读取。普通 32 位字:
// RV32 上对齐的加载存储是原子的; 64 位和即使撕裂, 也只会让一个本就是诊断量、
// 从不作控制输入的平均值失真。
inline Segment downlink{};
inline Segment uplink{};

// 由 bulk OUT 回调打开, 帧到达 TX FIFO 后关闭。0 表示"没有包在途", 即传输间隙的
// 状态; 从 EtherCAT 一侧到来的 CAN 帧看到的也是它。
inline uint32_t downlink_opened_at = 0;

// 两次 CSR_MCYCLE 读落在 tud_vendor_rx_cb, 两次落在 CAN 接收 ISR。rx_cb 路径上的
// 耗时按 1.2-1.4 倍折算成包率损失, 保持现在的体量即可; 更重的东西应放到
// libhcs_APP_CAN_DIAG 后面。
inline uint32_t now() { return static_cast<uint32_t>(hpm_csr_get_core_mcycle()); }

inline void open_downlink() { downlink_opened_at = now(); }

inline void close_downlink() {
    if (downlink_opened_at == 0)
        return;
    downlink.note(now() - downlink_opened_at);
}

inline void close_uplink(uint32_t opened_at) { uplink.note(now() - opened_at); }

inline void reset() {
    downlink = {};
    uplink = {};
}

} // namespace libhcs::firmware::diag::latency
