#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>
#include <utility>

#include <hpm_common.h>
#include <hpm_soc.h>
#include <hpm_soc_ip_feature.h>
#include <hpm_uart_drv.h>
#include <hpm_uart_regs.h>

#include "board_app.hpp"
#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/hpm_board/app/src/dmtool/dm_adapter.hpp"
#include "firmware/hpm_board/app/src/led/led.hpp"
#include "firmware/hpm_board/app/src/link/uplink.hpp"
#include "firmware/hpm_board/app/src/uart/rx_buffer.hpp"
#include "firmware/hpm_board/app/src/uart/tx_buffer.hpp"
#include "firmware/hpm_board/app/src/uart/uart_port.hpp"
#include "firmware/hpm_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::uart {

using board::UartPort;

class Uart
    : private core::utility::Immovable
    , private TxBuffer
    , private RxBuffer<Uart> {
    friend class RxBuffer<Uart>;

public:
    using Lazy = utility::Lazy<Uart, UartPort, size_t>;

    // 定义移到下方 AHB SRAM 存储数组之后(out-of-line)。
    explicit Uart(UartPort port, size_t storage_index);

    [[nodiscard]] data::DataId data_id() const { return data_id_; }

    [[nodiscard]] data::DataId config_data_id() const { return config_data_id_; }

    // 主机经 EP0(usb/vendor_control.cpp)请求的运行时波特率切换。返回端口
    // 当前是否已运行在 `baudrate`; 返回 false 时硬件原样保留, 控制传输的
    // 状态阶段被 STALL, 主机由此得知失败 -- 旧的带内 kUart*Config 字段
    // 无法表达这一点。
    //
    // 切换窗口内到达的 RX 字节可能乱码 -- 线路速率在字符中途改变, 与对端
    // 无从同步。接受该情形; 约定主机先把链路静默下来。
    //
    // 在主循环里经 tud_task() 执行: 下方 DLAB 窗口只在发送 DMA 可证明已
    // 停止时才安全, 中断上下文给不出这个保证。
    bool set_baudrate(uint32_t baudrate) {
        if (baudrate == 0) [[unlikely]]
            return false;

        // 先解不写。求解器拒绝时一个寄存器都没碰 —— 包括下面的
        // abort_transmit()。这正是 D2: 旧实现无条件先拆在途 TX DMA 再交给
        // uart_set_baudrate(), 于是"STALL 严格等于什么都没改"为假: 求解失败
        // 时 DMA 已停、FIFO 里那批字节成了孤儿, 而主机以为自己什么都没改。
        uint16_t want_divisor = 0;
        uint8_t want_oversample = 0;
        if (!solve_divisor(uart_clock_hz_, baudrate, want_divisor, want_oversample)) [[unlikely]]
            return false;

        // 已交给 DMA 的字节会按新速率移位输出。
        //
        // 必须先于 uart_set_baudrate() 执行, 且不只为了不让在途字节按错误
        // 速率发出: LCR.DLAB 选通时, DLL 与 THR 共用偏移 0x20, DLM 与 IER
        // 共用 0x24。DLAB 置位期间 -- uart_set_baudrate() 为写入锁存器恰要
        // 先置它 -- 面向 THR 的 TX DMA 写入会落进分频锁存器。整个窗口内
        // DMA 必须可证明已停止, 而非指望它恰好空闲。
        TxBuffer::abort_transmit();

        const hpm_stat_t status = uart_set_baudrate(uart_base_, baudrate, uart_clock_hz_);
        // 无条件执行, 且先于其他任何触碰该端口的代码: uart_set_baudrate()
        // 开头置 DLAB, 但求解器拒收波特率时提前返回、不清除它, 该路径上
        // SDK 交还的端口 0x20 仍别名到分频锁存器。在这里清除可同时覆盖
        // 两种结局。
        uart_base_->LCR &= ~UART_LCR_DLAB_MASK;

        // 上面已用同一个求解器筛过, 走到这里再失败说明 SDK 的求解器与本类
        // 的复制品给出不同答案(见 solve_divisor 的说明)。分频器未被触碰,
        // 端口仍按旧速率运行, 队列中的字节依旧有效, 按原样返回。
        if (status != status_success) [[unlikely]]
            return false;

        // FIFO 里还留着被中止的 DMA 已推入的字节。它们早于新分频器, 现在
        // 发出会在新速率下打出一段乱码; 对端最终能重新同步, 但每次切换后
        // 的首帧都是垃圾。
        uart_reset_tx_fifo(uart_base_);

        snapshot_divisor();
        return true;
    }

    // 写入之后再回读, 确认寄存器真的收下了这次编程。EP0 的 kSetUartConfig 用它
    // 把"求解器接受"升级为"硬件确认": 求解通过但回读不符, 是 kConfigErrorVerifyFailed,
    // 与"求解器拒绝"(kConfigErrorRateUnrepresentable) 是两件事。
    //
    // 比对的是分频器整数与过采样, 不是波特率: 主机的内核时钟与板子不同, 波特率
    // 无法在两侧复算, 而请求值本身不是求解器的不动点。见 UartDivisor 的注释。
    //
    // 只在 set_baudrate() 成功之后调用: 此时 DMA 已由那一步停稳, snapshot_divisor()
    // 也已刷新, 这里读快照不会引入新的 DLAB 竞争。
    [[nodiscard]] bool
        verify_baudrate(uint16_t expected_divisor, uint8_t expected_oversample) const {
        return divisor_u16() == expected_divisor && oversample() == expected_oversample;
    }

    // ---- 帧格式: EP0 配置通道的运行时切换与读回 ----
    //
    // 编码与 core/include/libhcs/protocol/vendor_control.hpp 的 UartParity /
    // UartStopBits 一致(0 = 跳过; 校验 1=无 2=偶 3=奇; 停止位 1=1 2=2); 字长
    // 直接用数据位数(7/8)。LCR.WLS 只有 2 位, 硬件字长 5..8 位, 9 位不提供 --
    // RX 环也是字节 DMA; 1.5 停止位不提供 -- 16550 风格的 STB 只在 5 位字长
    // 下表示 1.5, 对 7/8 位字长它就是 2 个停止位, 提供别名只会说谎。

    // 纯校验, 不碰寄存器: 供 EP0 处理器在 set_baudrate() 之前排除非法组合,
    // 保证任一字段非法时 STALL 严格等于"什么都没改"。
    [[nodiscard]] bool
        check_framing(uint32_t word_length, uint32_t parity, uint32_t stop_bits) const {
        if (word_length != 0U && word_length != 7U && word_length != 8U)
            return false;
        if (parity != 0U && (parity < 1U || parity > 3U))
            return false;
        if (stop_bits != 0U && stop_bits != 1U && stop_bits != 2U)
            return false;
        // 2 个停止位要求字长 >= 6(与 uart_init() 同一约束); 字长与停止位同改时
        // 以新字长计。
        const uint32_t effective_word = word_length != 0U ? word_length : this->word_length();
        return stop_bits != 2U || effective_word >= 6U;
    }

    // 提交 check_framing() 已通过的帧格式。0 = 保持不变; 只改请求了的字段。
    // LCR 直写并保持 DLAB=0(set_baudrate 结束时已清, 这里再清一次防御), 与
    // set_baudrate 的锁存器舞步一样只在主循环执行。
    void commit_framing(uint32_t word_length, uint32_t parity, uint32_t stop_bits) {
        uint32_t lcr = uart_base_->LCR & ~UART_LCR_DLAB_MASK;
        if (parity != 0U) {
            lcr &= ~(UART_LCR_SPS_MASK | UART_LCR_EPS_MASK | UART_LCR_PEN_MASK);
            switch (parity) {
            case 2U: lcr |= UART_LCR_PEN_MASK | UART_LCR_EPS_MASK; break; // 偶
            case 3U: lcr |= UART_LCR_PEN_MASK; break;                     // 奇
            default: break;                                               // 无校验
            }
        }
        if (stop_bits != 0U) {
            lcr &= ~UART_LCR_STB_MASK;
            if (stop_bits == 2U)
                lcr |= UART_LCR_STB_MASK;
        }
        if (word_length != 0U) {
            lcr &= ~UART_LCR_WLS_MASK;
            lcr |= UART_LCR_WLS_SET(word_length == 7U ? word_length_7_bits : word_length_8_bits);
        }
        uart_base_->LCR = lcr;
    }

    // ---- 帧格式读回: 从 LCR 解码, 编码同上, 永不返回 0 ----

    [[nodiscard]] uint32_t word_length() const {
        // WLS 0..3 = 数据位 5..8; 本固件只会写 7/8, 其余值如实上报。
        return 5U + UART_LCR_WLS_GET(uart_base_->LCR);
    }

    [[nodiscard]] uint32_t parity() const {
        const uint32_t lcr = uart_base_->LCR;
        if ((lcr & UART_LCR_PEN_MASK) == 0U)
            return 1U;                                    // 无校验
        return (lcr & UART_LCR_EPS_MASK) != 0U ? 2U : 3U; // 偶 : 奇
    }

    [[nodiscard]] uint32_t stop_bits() const {
        // STB=1 对 >=6 位字长即 2 个停止位; 1.5 只存在于 5 位字长, 本固件不会
        // 写出(见 check_framing)。
        return (uart_base_->LCR & UART_LCR_STB_MASK) != 0U ? 2U : 1U;
    }

    ATTR_PLACE_AT(".fast")
    void handle_downlink(const data::UartDataView& data) {
        if (!TxBuffer::try_enqueue(data))
            led::led->downlink_buffer_full();
    }

    ATTR_PLACE_AT(".fast")
    void try_transmit() { TxBuffer::try_dequeue(); }

    void irq_handler() {
        if (uart_is_rxline_idle(uart_base_)) {
            uart_clear_rxline_idle_flag(uart_base_);
            RxBuffer::rx_idle_callback();
        }
    }

    // 计算波特率分频器所用的内核时钟, 以及实际写入的分频器值。为诊断
    // 记录而暴露: 分频器只有连同其时钟一起才可解释。
    //
    // 分频器取自快照而非按需回读, 因为回读需要置 LCR.DLAB, 而 DLAB 会
    // 改写 TX DMA 的写入地址(0x20 处的 THR 变为 DLL) -- 诊断读与进行中的
    // TX 竞争, 会用一个数据字节覆盖分频器, 端口无声死去。快照既便宜又
    // 正确: 除 init 与 set_baudrate 外无人改动分频器, 且两者都在 TX 停止
    // 时采样。
    [[nodiscard]] uint32_t clock_hz() const { return uart_clock_hz_; }
    [[nodiscard]] uint32_t divisor() const { return uart_divisor_; }
    [[nodiscard]] uint32_t oscr() const { return uart_base_->OSCR; }
    [[nodiscard]] UART_Type* base() const { return uart_base_; }

    // UartDivisor 的两个整数, 供 EP0 的 kGetUartConfig 上报与 kSetUartConfig
    // 回读比对。**分频器取自快照** -- 理由同 divisor() 上方: 按需回读要置
    // LCR.DLAB, 会与在途 TX DMA 争用 0x20。过采样取自活寄存器 OSCR(它不在
    // DLAB 的别名窗口里, 直接读安全), 按 SDK 的编码把 0 还原成 32。
    //
    // 这一对是主机唯一能用来判定"速率切换是否真的生效"的东西: 主机的内核时钟
    // 与板子的不同, 分频器无法在主机侧复算, 而请求的波特率又不是求解器的不动点
    // (921600 在 80 MHz 上得 909090)。见 UartDivisor 的注释。
    [[nodiscard]] uint16_t divisor_u16() const { return static_cast<uint16_t>(uart_divisor_); }

    [[nodiscard]] uint8_t oversample() const {
        const uint32_t osc_field = uart_base_->OSCR & UART_OSCR_OSC_MASK;
        return static_cast<uint8_t>(osc_field ? osc_field : 32U);
    }

    // 硬件实际运行的波特率, 由实际写入的分频器与过采样率重建 -- 而非
    // 主机请求的数值。这一区分正是意义所在: 被拒的请求不触碰分频器, 这里
    // 便仍报旧速率, 主机据此分辨成败。
    //
    // OSCR 存过采样率, 0 表示 32(SDK 如此编码, 因为该字段只有 5 位)。
    [[nodiscard]] uint32_t effective_baudrate() const {
        return baudrate_for(divisor_u16(), oversample());
    }

    // 给定 (分频器, 过采样倍数) 在本口时钟下的速率; EP0 用它在写入前核对求解结果。
    [[nodiscard]] uint32_t baudrate_for(uint32_t divisor, uint32_t oversample) const {
        if (!oversample || !divisor) [[unlikely]]
            return 0;
        return uart_clock_hz_ / (oversample * divisor);
    }

    // 只解不写: 求给定速率的 (divisor, 过采样倍数), 供 EP0 处理器先校验再提交。
    //
    // 为什么必须有它。uart_set_baudrate() 在自己开头置 LCR.DLAB, 而本类
    // set_baudrate() 里的 TxBuffer::abort_transmit() 必须早于该窗口(DLL 与 THR
    // 同址 0x20, DLAB 置位期间在途 TX DMA 的写会落进分频锁存器)。于是"先求解、
    // 失败即 STALL"与"必须已停 DMA 才能进 DLAB 窗口"这两条互相顶住: SDK 的
    // uart_set_baudrate() 是唯一求解入口, 而它自己就会开 DLAB。解法是把求解拆出来
    // 前置 —— 复制 SDK 的求解器(它不导出: hpm_uart_drv.c:66 是 static), 在碰任何
    // 寄存器之前先解一次; 解不出来就原样返回 false, 一个寄存器都没动, "STALL 严格
    // 等于什么都没改"才机械成立。
    //
    // 与 SDK 逐位一致是硬要求: SDK 在同一个 (clock, baudrate) 上给出同一答案, 因此
    // 本函数通过之后 uart_set_baudrate() 不会反过来失败 —— 它只会在"本函数已拒绝"
    // 的输入上失败, 而那条路径现在到不了。
    //
    // 这份复制**无法在编译期钉住**: SDK 把容差/范围/SCALE 全定义在
    // hpm_uart_drv.c 里(第 14-29 行), 不是头文件, 本翻译单元看不见(只有寄存器
    // 位域宏在 soc/*/ip/hpm_uart_regs.h 里可见, 那几项下面已 static_assert)。
    // 所以复制品的正确性由**运行期回读**兜底, 而不是由编译器: 写入之后立即回读
    // DLL/DLM/OSCR 与本函数算出的 (divisor, 过采样倍数) 比对, 不等即 verify_failed。
    // 若哪天 SDK 改了容差使两者分道扬镳, 症状是一个明确的 kConfigErrorVerifyFailed,
    // 而不是一个静默错误的波特率 —— 这正是本任务要消灭的那类失败。
    [[nodiscard]] static bool solve_divisor(
        uint32_t clock_hz, uint32_t baudrate, uint16_t& divisor_out, uint8_t& oversample_out) {
        // SDK 的常量, 逐一镜像(hpm_uart_drv.c:14-29)。
        constexpr uint32_t kMinimumBaudrate = 200U;
        constexpr uint32_t kTolerance = 3U;
        constexpr uint32_t kOscMax = 32U;
        constexpr uint32_t kOscMin = 8U;
        constexpr uint32_t kDivMax = 0xFFFFU;
        constexpr uint32_t kDivMin = 1U;
        constexpr uint32_t kScale = 1000U;
        // 循环上界是 SoC 头里可见的宏(HPM5301/5361 降到 30), SDK 同式取默认。
#ifdef UART_SOC_OVERSAMPLE_MAX
        constexpr uint32_t kOscLoopMax = UART_SOC_OVERSAMPLE_MAX;
#else
        constexpr uint32_t kOscLoopMax = kOscMax;
#endif

        // 只有寄存器位域宏在头文件里可见, 其余 SDK 常量在 .c 内, 见上方说明。
        static_assert(UART_OSCR_OSC_MASK == 0x1FU);
        static_assert(UART_DLL_DLL_MASK == 0xFFU);
        static_assert(UART_DLM_DLM_MASK == 0xFFU);

        if (clock_hz == 0U || baudrate == 0U) [[unlikely]]
            return false;
        // SDK 的入参闸门, 逐条对应。
        if (baudrate < kMinimumBaudrate || clock_hz / kDivMin < baudrate * kOscMin
            || clock_hz / kDivMax > baudrate * kOscMax) [[unlikely]]
            return false;

        const uint64_t tmp = (static_cast<uint64_t>(clock_hz) * kScale) / baudrate;

        for (uint32_t osc = kOscMin; osc <= kOscLoopMax; osc += 2) {
            // 带舍入的分频器(与 SDK 同式)。
            const uint32_t div = static_cast<uint32_t>((tmp + osc * (kScale / 2)) / (osc * kScale));
            if (div < kDivMin || div > kDivMax)
                continue;
            uint32_t delta = 0;
            const uint64_t achieved = static_cast<uint64_t>(div) * osc * kScale;
            if (achieved > tmp)
                delta = static_cast<uint32_t>(achieved - tmp);
            else if (achieved < tmp)
                delta = static_cast<uint32_t>(tmp - achieved);
            if (delta && (((delta * 100U) / tmp) > kTolerance))
                continue;
            divisor_out = static_cast<uint16_t>(div);
            // 报倍数而非 OSCR 编码(32 写进寄存器是 0), 与 oversample() 同域比较。
            oversample_out = static_cast<uint8_t>(osc);
            return true;
        }
        return false;
    }

private:
    // 调用方必须保证 TX DMA 已停止: 本函数会置 LCR.DLAB, 期间任何本想
    // 写入 THR 的 DMA 写都会落到分频锁存器上。
    void snapshot_divisor() {
        const uint32_t lcr = uart_base_->LCR;
        uart_base_->LCR = lcr | UART_LCR_DLAB_MASK;
        uart_divisor_ = ((uart_base_->DLM & 0xFFU) << 8) | (uart_base_->DLL & 0xFFU);
        uart_base_->LCR = lcr & ~UART_LCR_DLAB_MASK;
    }

    [[nodiscard]] uint32_t init_uart(uint32_t irq_num, uint32_t baudrate, parity_setting_t parity) {
        const uint32_t uart_clock = board::init_uart(uart_base_);

        uart_config_t config{};
        uart_default_config(uart_base_, &config);
        config.fifo_enable = true;
        config.dma_enable = true;
        config.src_freq_in_hz = uart_clock;
        config.tx_fifo_level = uart_tx_fifo_trg_not_full;
        config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
        config.baudrate = baudrate;
        config.parity = static_cast<uint8_t>(parity);

        static_assert(HPM_IP_FEATURE_UART_TX_IDLE_DETECT == 1); // NOLINT(misc-redundant-expression)
        config.txidle_config.idle_cond = uart_rxline_idle_cond_state_machine_idle;
        config.txidle_config.detect_enable = true;
        config.txidle_config.threshold = 16;

        static_assert(HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1); // NOLINT(misc-redundant-expression)
        config.rxidle_config.detect_enable = true;
        config.rxidle_config.detect_irq_enable = true;
        config.rxidle_config.idle_cond = uart_rxline_idle_cond_state_machine_idle;
        config.rxidle_config.threshold = 10;

        core::utility::assert_always(uart_init(uart_base_, &config) == status_success);
        intc_m_enable_irq_with_priority(irq_num, 1);

        return uart_clock;
    }

    void handle_uplink(
        std::span<const std::byte> payload, std::span<const std::byte> payload2, bool is_idle) {
        if (!link::uplink_enabled()) {
            // 没有 libhcs 会话: 交给 CDC 串口桥(桥没接通时它什么也不做)。冷函数调用
            // 而非内联判断, 上面 libhcs 分支的代码与没有 CDC 桥时一样。
            dmtool::uart_rx_without_session(
                payload.data(), payload.size(), payload2.data(), payload2.size());
            return;
        }

        auto& serializer = link::uplink_serializer();
        core::utility::assert_debug(
            serializer.write_uart(
                data_id_, {.uart_data = payload, .idle_delimited = is_idle}, payload2)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    const data::DataId data_id_;
    const data::DataId config_data_id_;
    UART_Type* uart_base_;
    // init 时捕获的源时钟: 运行时切波特率时, uart_set_baudrate 靠它重算
    // 分频器。
    uint32_t uart_clock_hz_;
    // 实际写入的分频器值, 由 snapshot_divisor() 在 init 时与每次切换后
    // 采样。为何不按需回读, 见上方访问器注释。
    uint32_t uart_divisor_ = 0;

public:
    // DMA 缓冲存储, 放在板级选定的非缓存区域(HPM5321 上是 AHB SRAM,
    // HPM6E80 core1 上是 AXI-SRAM 非缓存窗口), 免去 UART 数据通路上手工
    // l1c_dc_flush/invalidate 的开销。
    struct RxStorage {
        alignas(HPM_L1C_CACHELINE_SIZE) std::array<std::byte, RxBuffer<Uart>::kBufferSize> data;
        alignas(HPM_L1C_CACHELINE_SIZE) std::array<
            dma_mgr_linked_descriptor_t, RxBuffer<Uart>::kDmaDescriptorCount> descriptors;
    };
    struct TxStorage {
        alignas(HPM_L1C_CACHELINE_SIZE) std::array<std::byte, TxBuffer::kBufferSize> data;
        alignas(HPM_L1C_CACHELINE_SIZE) dma_mgr_linked_descriptor_t descriptor;
    };
};

constexpr size_t kUartCount = std::size(board::kUartPorts);

ATTR_PLACE_AT(libhcs_DMA_BUFFER_SECTION)
inline constinit Uart::RxStorage uart_rx_storage[kUartCount]{};
ATTR_PLACE_AT(libhcs_DMA_BUFFER_SECTION)
inline constinit Uart::TxStorage uart_tx_storage[kUartCount]{};

inline Uart::Uart(UartPort port, size_t storage_index)
    : TxBuffer(
          reinterpret_cast<UART_Type*>(port.base), port.dma_src_tx,
          uart_tx_storage[storage_index].data.data(), &uart_tx_storage[storage_index].descriptor)
    , RxBuffer(
          reinterpret_cast<UART_Type*>(port.base), port.dma_src_rx,
          uart_rx_storage[storage_index].data.data(),
          uart_rx_storage[storage_index].descriptors.data())
    , data_id_(port.data_id)
    , config_data_id_(port.config_data_id)
    , uart_base_(reinterpret_cast<UART_Type*>(port.base))
    , uart_clock_hz_(init_uart(port.irq_num, port.baudrate, port.parity)) {
    // 此处安全, 理由同 set_baudrate 中的调用: 尚无任何 TX 入队, DLAB 置位
    // 期间不可能有写 THR 的操作。
    snapshot_divisor();
}

namespace internal {

template <std::size_t index>
consteval Uart::Lazy make_uart() {
    return Uart::Lazy{board::kUartPorts[index], index};
}

template <std::size_t... indices>
consteval std::array<Uart::Lazy, sizeof...(indices)>
    make_uart_array(std::index_sequence<indices...>) {
    return {make_uart<indices>()...};
}

} // namespace internal

inline constinit auto uart_array =
    internal::make_uart_array(std::make_index_sequence<kUartCount>{});

} // namespace libhcs::firmware::uart
