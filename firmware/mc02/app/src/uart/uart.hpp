#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <usart.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/led/led.hpp"
#include "firmware/mc02/app/src/uart/rx_buffer.hpp"
#include "firmware/mc02/app/src/uart/tx_buffer.hpp"
#include "firmware/mc02/app/src/usb/helper.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

namespace libhcs::firmware::uart {

// 两种端口共享的状态与行为: 身份标识、运行时波特率控制、接收字节向 USB 的
// 交接。自身不持有缓冲, 因此仅接收的 DBUS 口不必背着一个永远用不上的 3 KB
// TX ring。
class UartCommon : private core::utility::Immovable {
public:
    // ---- 运行时配置: 波特率与帧格式, 全部供 EP0 配置通道调用 ----
    //
    // 约定与 core/include/libhcs/protocol/vendor_control.hpp 的 UartConfigPayload
    // 一致: 先全量校验后统一提交, 任何字段非法时寄存器一个都不动, 控制传输的
    // STALL 严格等于"什么都没改"。切换窗口内到达的 RX 字节可能乱码, 主机应先
    // 静默链路。

    // 只解不写: 求给定速率的 BRR 值, 供处理器先校验再提交。逻辑对应 STM32H7
    // HAL 的 UART_SetConfig(): H7 的 UART 内核时钟来自按组可选的时钟源(并非
    // 简单的 PCLK)且经 Init.ClockPrescaler 分频, 除数必须由解析后的时钟源
    // 计算; 像 c_board 那样用 HAL_RCC_GetPCLKxFreq 会静默得到错误波特率。
    // 边界与 HAL 一致: 超出范围的除数会错配外设, 故拒绝而非写入垃圾值。
    [[nodiscard]] bool solve_brr(uint32_t baudrate, uint32_t& brr) const {
        if (baudrate == 0U) [[unlikely]]
            return false;

        const uint32_t kernel_clock_hz = peripheral_clock_hz();
        if (kernel_clock_hz == 0U) [[unlikely]]
            return false;

        static constexpr uint32_t kBrrMin = 0x10U;
        static constexpr uint32_t kBrrMax = 0xFFFFU;

        if (hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8) {
            const uint32_t usartdiv = UART_DIV_SAMPLING8(
                kernel_clock_hz, baudrate, hal_uart_handle_->Init.ClockPrescaler);
            if (usartdiv < kBrrMin || usartdiv > kBrrMax) [[unlikely]]
                return false;
            // 8 倍过采样把 BRR[3] 记为零并右移低半字节。
            brr = (usartdiv & 0xFFF0U) | ((usartdiv & 0x000FU) >> 1U);
        } else {
            const uint32_t usartdiv = UART_DIV_SAMPLING16(
                kernel_clock_hz, baudrate, hal_uart_handle_->Init.ClockPrescaler);
            if (usartdiv < kBrrMin || usartdiv > kBrrMax) [[unlikely]]
                return false;
            brr = usartdiv;
        }
        return true;
    }

    // 提交 solve_brr() 的结果。只在校验全部通过后调用, 无失败路径。
    void commit_brr(uint32_t baudrate, uint32_t brr) {
        hal_uart_handle_->Init.BaudRate = baudrate;
        hal_uart_handle_->Instance->BRR = brr;
    }

    // 应用帧格式。字长直接用数据位数(7/8; 9 位不提供 -- RX 环是字节 DMA, 第
    // 九位会被静默截断), 校验与停止位用协议编码(0 = 保持不变; 校验 1=无 2=偶
    // 3=奇; 停止位 1=1 2=2; 1.5 停止位不提供 -- 本系列控制器只在 5 位字长下
    // 实现它)。先全量校验后一次 UE 下拉内写完: M/PCE/PS/STOP 只能在 UE=0 时
    // 写, 窗口内到达的字节会失配。
    bool set_framing(uint32_t word_length, uint32_t parity, uint32_t stop_bits) {
        // 纯校验, 不碰寄存器。
        uint32_t m_bits = 0;
        if (word_length != 0U) {
            if (word_length == 7U)
                m_bits = USART_CR1_M1;
            else if (word_length == 8U)
                m_bits = 0U;
            else
                return false;
        }
        uint32_t parity_bits = 0;
        if (parity != 0U) {
            switch (parity) {
            case 1U: parity_bits = 0U; break;                           // 无校验
            case 2U: parity_bits = USART_CR1_PCE; break;                // 偶
            case 3U: parity_bits = USART_CR1_PCE | USART_CR1_PS; break; // 奇
            default: return false;
            }
        }
        uint32_t stop_reg = 0;
        if (stop_bits != 0U) {
            if (stop_bits == 1U)
                stop_reg = 0U;
            else if (stop_bits == 2U)
                stop_reg = USART_CR2_STOP_1;
            else
                return false;
        }

        auto* instance = hal_uart_handle_->Instance;
        const uint32_t cr1 = instance->CR1;
        uint32_t new_cr1 = cr1 & ~(USART_CR1_UE | USART_CR1_M | USART_CR1_PCE | USART_CR1_PS);
        if (word_length != 0U)
            new_cr1 |= m_bits;
        if (parity != 0U)
            new_cr1 |= parity_bits;

        instance->CR1 = new_cr1 & ~USART_CR1_UE; // UE=0, 帧格式字段可写
        if (stop_bits != 0U)
            instance->CR2 = (instance->CR2 & ~USART_CR2_STOP) | stop_reg;
        instance->CR1 = new_cr1; // UE 恢复
        return true;
    }

    // ---- 帧格式读回: 从活寄存器解码, 编码同上, 永不返回 0(0 只表示"跳过") ----

    [[nodiscard]] uint32_t word_length() const {
        const uint32_t m = hal_uart_handle_->Instance->CR1 & USART_CR1_M;
        // M0(9 位)不会由本驱动写出; 万一出现, 如实上报。
        if ((m & USART_CR1_M0) != 0U)
            return 9U;
        return (m & USART_CR1_M1) != 0U ? 7U : 8U;
    }

    [[nodiscard]] uint32_t parity() const {
        const uint32_t cr1 = hal_uart_handle_->Instance->CR1;
        if ((cr1 & USART_CR1_PCE) == 0U)
            return 1U;                               // 无校验
        return (cr1 & USART_CR1_PS) != 0U ? 3U : 2U; // 奇 : 偶
    }

    [[nodiscard]] uint32_t stop_bits() const {
        // 1.5 停止位的编码不会由本驱动写出(见 set_framing); 万一出现, 如实上报
        // 协议中无此编码的原始值没有意义, 按最近的 2 处理并注释于此。
        return (hal_uart_handle_->Instance->CR2 & USART_CR2_STOP) == USART_CR2_STOP_1 ? 2U : 1U;
    }

    // 实际编程的波特率, 从 BRR 反推而非 Init.BaudRate: 后者只是最近一次请求
    // 值, 被拒绝的请求两者都不会写。读 BRR 才能让主机区分"切换已生效"与
    // "切换被拒、仍按旧速率运行"。
    [[nodiscard]] uint32_t effective_baudrate() const {
        const uint32_t kernel_clock_hz = peripheral_clock_hz();
        const uint32_t brr = hal_uart_handle_->Instance->BRR & 0xFFFFU;
        if (!kernel_clock_hz || !brr) [[unlikely]]
            return 0;
        const uint32_t presc = UARTPrescTable[hal_uart_handle_->Init.ClockPrescaler & 0x0FU];
        const uint32_t clock = presc ? kernel_clock_hz / presc : kernel_clock_hz;
        if (hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8) {
            // 8 倍过采样把 BRR[3] 记为零、低半字节右移一位, 相除前先还原。
            const uint32_t usartdiv = (brr & 0xFFF0U) | ((brr & 0x0007U) << 1U);
            return usartdiv ? (2U * clock) / usartdiv : 0U;
        }
        return clock / brr;
    }

protected:
    UartCommon(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle)
        : data_id_(data_id)
        , hal_uart_handle_(hal_uart_handle) {}

    // RxBuffer 与 TxBuffer 各存一份句柄, 派生端口无法无限定地访问
    // hal_uart_handle_; 把所需的两处访问经此路由, 避免到处写 UartCommon:: 限定。
    [[nodiscard]] bool has_rx_error() const {
        constexpr uint32_t rx_error_mask =
            HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE;
        return (hal_uart_handle_->ErrorCode & rx_error_mask) != 0U;
    }

    void flag_dma_error() { hal_uart_handle_->ErrorCode |= HAL_UART_ERROR_DMA; }

    void handle_uplink(
        std::span<const std::byte> payload, std::span<const std::byte> payload2, bool is_idle) {
        // 无会话时写入的内容不保存: 主机到达时 activate_session() 会清 ring。
        // 无论如何 RxBuffer::try_dequeue() 都会推进 out_, 接收 ring 仍在此
        // 排空, 不会积压进其绕圈 fail-fast 路径。
        if (!usb::uplink_session_active())
            return;

        auto& serializer = usb::get_serializer();
        core::utility::assert_always(
            serializer.write_uart(
                data_id_, {.uart_data = payload, .idle_delimited = is_idle}, payload2)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    // 供给本 UART 波特率除数的内核时钟。
    //
    // H7 的 UART 时钟来自按组可选的时钟源并经 Init.ClockPrescaler 分频, 必须
    // 先解析时钟源频率才有意义; F407 的 UART 直接挂 APB 总线, c_board 用
    // HAL_RCC_GetPCLKxFreq() 两行即可, 勿照搬。
    //
    // 经 HAL 的 UART_GETCLOCKSOURCE 按外设实例解析, 与 UART_SetConfig() 计算
    // BRR 用的是同一宏。刻意不用 HAL_RCCEx_GetPeriphCLKFreq(): 本 HAL 版本该
    // 函数的 if/else 链覆盖 SAI/SPI/ADC/SDMMC/FDCAN 却没有 UART 分支,
    // RCC_PERIPHCLK_USART16910 与 RCC_PERIPHCLK_USART234578 都会落入末尾的
    // `else { frequency = 0; }` 返回 0。宏确实存在, 编译期不会有任何告警,
    // handle_config() 则因 kernel_clock_hz == 0 提前返回, 每次运行时波特率请求
    // 都被静默忽略, 端口停留在 CubeMX 的 115200。同板 UART7<->UART10 环回测不
    // 出该问题(两端同样忽略切换), 只有跨板对测才暴露。
    [[nodiscard]] uint32_t peripheral_clock_hz() const {
        UART_ClockSourceTypeDef clocksource = UART_CLOCKSOURCE_UNDEFINED;
        UART_GETCLOCKSOURCE(hal_uart_handle_, clocksource);

        switch (clocksource) {
        case UART_CLOCKSOURCE_D2PCLK1: return HAL_RCC_GetPCLK1Freq();
        case UART_CLOCKSOURCE_D2PCLK2: return HAL_RCC_GetPCLK2Freq();
        case UART_CLOCKSOURCE_CSI: return CSI_VALUE;
        case UART_CLOCKSOURCE_LSE: return LSE_VALUE;
        case UART_CLOCKSOURCE_HSI:
            // HSI 经分频器到达 UART, 仅当分频为 1 时裸 HSI_VALUE 才正确,
            // UART_SetConfig 同样移位。UART7/UART10 在 .ioc 中选择 HSI, 此路径
            // 是活的。
            if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U)
                return static_cast<uint32_t>(HSI_VALUE >> (__HAL_RCC_GET_HSI_DIVIDER() >> 3U));
            return static_cast<uint32_t>(HSI_VALUE);
        case UART_CLOCKSOURCE_PLL2: {
            PLL2_ClocksTypeDef pll2{};
            HAL_RCCEx_GetPLL2ClockFreq(&pll2);
            return pll2.PLL2_Q_Frequency;
        }
        case UART_CLOCKSOURCE_PLL3: {
            PLL3_ClocksTypeDef pll3{};
            HAL_RCCEx_GetPLL3ClockFreq(&pll3);
            return pll3.PLL3_Q_Frequency;
        }
        default: return 0;
        }
    }

    data::DataId data_id_;
    UART_HandleTypeDef* hal_uart_handle_;
};

// 全双工端口: USART1、UART7、USART10, CubeMX 为每个口接好 RX 与 TX 两条
// DMA 流。
class Uart
    : public UartCommon
    , private TxBuffer</*half_duplex=*/false>
    , private RxBuffer<Uart> {
    friend class RxBuffer<Uart>;

public:
    using Lazy = utility::Lazy<Uart, data::DataId, UART_HandleTypeDef*>;

    Uart(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle)
        : UartCommon(data_id, hal_uart_handle)
        , TxBuffer(hal_uart_handle, &hal_tx_dma_complete_callback, &hal_tx_dma_error_callback)
        , RxBuffer(hal_uart_handle) {}

    void handle_downlink(const data::UartDataView& data) {
        if (!TxBuffer::try_enqueue(data))
            led::led->downlink_buffer_full();
    }

    void try_transmit() {
        RxBuffer::try_dequeue();
        TxBuffer::try_dequeue();
    }

    void tx_complete_callback() { TxBuffer::tx_complete_callback(); }

    void uart_error_callback() {
        if (has_rx_error())
            RxBuffer::rx_error_callback();
    }

    void rx_dma_error_callback() {
        flag_dma_error();
        RxBuffer::rx_error_callback();
    }

    void tx_dma_error_callback() { TxBuffer::tx_error_callback(); }

    void rx_event_callback() { RxBuffer::uart_idle_event_callback(); }

private:
    static void hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_complete_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);
};

// 仅接收端口, 用于 UART5 (DBUS)。
//
// 无 TX 路径的两个独立原因: CubeMX 只给 UART5 接了 RX DMA 流
// (bsp/cubemx/Core/Src/usart.c 只声明 hdma_uart5_rx 而无 hdma_uart5_tx); 协议
// 也不会把下行路由到 kUartDbus, usb/vendor.hpp 的 uart_deserialized_callback
// 只分发 kUart1/kUart7/kUart10(libhcs_APP_RS485_ENABLE 时另含两个 RS-485 口)。
// 仅 kUartDbusConfig 路由至此, 落在 UartCommon::handle_config。
class UartRxOnly
    : public UartCommon
    , private RxBuffer<UartRxOnly> {
    friend class RxBuffer<UartRxOnly>;

public:
    using Lazy = utility::Lazy<UartRxOnly, data::DataId, UART_HandleTypeDef*>;

    UartRxOnly(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle)
        : UartCommon(data_id, hal_uart_handle)
        , RxBuffer(hal_uart_handle) {}

    // 与全双工端口同名, 便于 app.cpp 统一轮询所有端口; 此处仅排空接收 ring。
    void try_transmit() { RxBuffer::try_dequeue(); }

    void uart_error_callback() {
        if (has_rx_error())
            RxBuffer::rx_error_callback();
    }

    void rx_dma_error_callback() {
        flag_dma_error();
        RxBuffer::rx_error_callback();
    }

    void rx_event_callback() { RxBuffer::uart_idle_event_callback(); }

private:
    static void hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);
};

// 半双工 RS-485 端口, 用于 USART2。
//
// 线路上与 Uart 无任何差别: CubeMX 指定 PD4 为 USART2_DE, MX_USART2_UART_Init
// 调 HAL_RS485Ex_Init 置 CR3.DEM, USART 在起始位前、停止位后自动驱动 DE, 无需
// 软件参与; 原理图把收发器 RE# 接到同一网络, 驱动开启的全程接收器关闭, 端口
// 听不到自己的发送。RxBuffer 因此原样复用, 整个驱动里没有方向 GPIO。F407 的
// USART 无 DEM/DEP, c_board 做不到这一点。
//
// 区别在总线纪律, 全部位于 TxBuffer<true>, 见彼处 kTurnaroundDeadline 注释。
// 用独立类型而非运行时标志, 既让相关分支彻底离开三个全双工端口, 也使把这个口
// 交给"假定独占发送线"的代码成为不可能。
//
// ring 只有流式端口的八分之一: 总线主节点线上永远只有一次事务, 发请求、静默、
// 收一答, ring 只需覆盖单次交互加余量。256 字节在 4.8 Mbaud 下绕一圈 533 us,
// 主循环每 12.5 us 回来一次, 余量 42 倍; sample_write_position() 的落后断言在
// 半圈处, 仍有 21 倍。可容纳三个 78 字节应答; 发送侧 256 字节为七条 34 字节
// 命令, 32 个检查点是 ring 能装下包数的两倍, 两者都不会先耗尽。
//
// staging 刻意与 ring 等大: try_dequeue() 按它钳制每次突发, 跨两次突发的包会
// 在帧中间插入主循环静默, 对流无害, 对按总线静默定界的对端致命。
//
// 合计每口约 890 字节而非 5696; 两个口占 32 KB D2 SRAM 中的 1.8 KB 而非
// 11.4 KB, 这正是两者能够共存的理由。
inline constexpr size_t kRs485BufferSize = 256;
inline constexpr size_t kRs485CheckpointCount = 32;

class UartRs485
    : public UartCommon
    , private TxBuffer<
          /*half_duplex=*/true, kRs485BufferSize, kRs485BufferSize, kRs485CheckpointCount>
    , private RxBuffer<UartRs485, kRs485BufferSize> {
    friend class RxBuffer<UartRs485, kRs485BufferSize>;

public:
    using Lazy = utility::Lazy<UartRs485, data::DataId, UART_HandleTypeDef*>;

    UartRs485(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle)
        : UartCommon(data_id, hal_uart_handle)
        , TxBuffer(hal_uart_handle, &hal_tx_dma_complete_callback, &hal_tx_dma_error_callback)
        , RxBuffer(hal_uart_handle) {}

    void handle_downlink(const data::UartDataView& data) {
        if (!TxBuffer::try_enqueue(data))
            led::led->downlink_buffer_full();
    }

    // 把原始 IDLE 计数喂给 turnaround 门, 刻意不用 RxBuffer::try_dequeue()
    // 维护的 consumed_idle_count_: 总线是否重新空闲只取决于对端是否停口, 与
    // 主机是否已取走字节无关。
    void try_transmit() {
        RxBuffer::try_dequeue();
        TxBuffer::try_dequeue(RxBuffer::idle_count());
    }

    void tx_complete_callback() { TxBuffer::tx_complete_callback(); }

    void uart_error_callback() {
        if (has_rx_error())
            RxBuffer::rx_error_callback();
    }

    void rx_dma_error_callback() {
        flag_dma_error();
        RxBuffer::rx_error_callback();
    }

    void tx_dma_error_callback() { TxBuffer::tx_error_callback(); }

    // 刻意不在此中断里排空 ring。曾在 libhcs_APP_UART_RX_IN_ISR 开关下实测
    // ISR 排空方案: 确能省掉主循环每遍的 NDTR 读取(5410 -> 5133 周期/遍,
    // 101 -> 107 kHz), 但该时间本就富余(每个到达的 USB 包主循环已跑四遍), 且
    // 200 字节消息的往返耗时增加 26%, 因为仅按 IDLE 发布会放弃
    // RxBuffer::try_dequeue() 在字节仍在到达时按 kMinFragmentSize 边界的分块
    // 流式转发。数据见 firmware/mc02/AGENTS.md, 该开关因此被移除。
    void rx_event_callback() { RxBuffer::uart_idle_event_callback(); }

private:
    static void hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_complete_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);
};

// 位于 D2 SRAM (.d2_sram, 0x30000000)。每个对象自带 DMA ring, 而驱动 UART
// RX/TX 流的 DMA1/DMA2 是 D2 域主设备: ring 放在 D2 SRAM 使每次传输都留在域
// 内, 不跨 D2-D1 互连去 AXI SRAM 与 M7 争用。第二个原因(non-cacheable MPU
// 窗口下的 AXI SRAM 余量)与 app.cpp 开机需做的事, 见
// bsp/linker/STM32H723VGTx_APP.ld 的 .d2_sram 注释。不可像 can.hpp 那样挪进
// .dtcm: DTCM 仅核内可达, 指向它的 DMA 流会静默地什么都传不了。
[[gnu::section(".d2_sram")]] inline constinit Uart::Lazy uart1{data::DataId::kUart1, &huart1};
[[gnu::section(".d2_sram")]] inline constinit Uart::Lazy uart7{data::DataId::kUart7, &huart7};
[[gnu::section(".d2_sram")]] inline constinit Uart::Lazy uart10{data::DataId::kUart10, &huart10};
[[gnu::section(".d2_sram")]] inline constinit UartRxOnly::Lazy uart_dbus{
    data::DataId::kUartDbus, &huart5};

// RS-485 端口接在 USART2 / USART3, 按机壳丝印 (UART2 / UART3) 而非备用协议
// 槽位命名。与流式端口的差异见 UartRs485, 以及 tx_buffer.hpp 的
// kTurnaroundDeadline 注释。
//
// 电气事实, 依据原理图 CtrBoard-H7_V1.0-240124 第 5 页 (收发器 U5):
//   - 无回显。引脚 2 (RE#)与 3 (DE)接同一网络, 由 USART2_DE(PD04)驱动, 本端
//     发送全程接收器关闭, 发出的内容不会回来; R11 在此期间把 RO 拉到 3V3
//     维持空闲电平, 禁用的接收器不会呈现虚假起始位。上游无需滤回显。
//   - R17 把 DE 网络拉低, 端口上电即处于接收态, 软件运行前不占用总线。
//   - R15 在本端 A-B 间装了 120R 端接, 再加一路前先确认远端情况。
//   - DEAT/DEDT 在 .ioc 中为 16/16, 即 4.8 Mbaud、16 倍过采样下前后各一位
//     时间; CubeMX 默认的 0 不可用。
//
// 此前留开放的两个疑问均已实测排除 `[实测 2026-08-25]`:
//   - 本端发送结束后重新使能接收器不会引发虚假 IDLE, 否则 turnaround 门会
//     提前放行而碰撞。
//   - 一位时间的 DEAT 对该收发器足够; DEAT 过短会最先在最高波特率截断前沿。
// `[实测 2026-09-01]` USART2 对接 USART3 的单板链路复测同样全部通过。
//
// 实测发现: TX ring 装不下的下行包会被丢弃且仅有 LED 提示。257 B 起到不了
// 总线(256 B 及以下完好); 32 字节命令连发时第九条之后的全部丢失(12 条发 9、
// 24 条发 10, 总是丢尾、无残帧)。ring 容量即上述设计点, 但静默丢弃不是;
// mc02 的 CAN 也有同样缺口, 见 firmware/mc02/AGENTS.md 的"未做"注(hpm_board
// 的下行流控未移植)。
//
// libhcs_APP_RS485_ENABLE(默认开)时始终编译。它们就是机壳的 UART2/UART3, 不
// 是叠在备用 DataId 上的别名; 诊断构建改用 kUart0 输出, 与这两个口不再冲突。
// 关掉该开关即可连同 1.8 KB D2 SRAM 与每遍 NDTR 轮询一起去掉。
#ifdef libhcs_APP_RS485_ENABLE
[[gnu::section(".d2_sram")]] inline constinit UartRs485::Lazy uart2{data::DataId::kUart2, &huart2};

// 第二个 RS-485 口, USART3 经收发器 U6: 485_DIR1 由 PB14 上的 USART3_DE 驱动,
// 数据走 PD8/PD9, 总线接 P5 连接器。电路与上述 U5 相同, RE# 接 DE, R12 把 RO
// 拉到 3V3 使禁用的接收器维持空闲电平, R18 把 DE 网络拉低使端口上电即接收,
// R16 在本端装 120R 端接。
[[gnu::section(".d2_sram")]] inline constinit UartRs485::Lazy uart3{data::DataId::kUart3, &huart3};
#endif

} // namespace libhcs::firmware::uart
