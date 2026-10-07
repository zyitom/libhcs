#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <usart.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/spec/mc02/ports.hpp"
#include "core/src/link/port.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/common/app/src/utility/event_counter.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"
#include "firmware/mc02/app/src/led/led.hpp"
#include "firmware/mc02/app/src/uart/rx_buffer.hpp"
#include "firmware/mc02/app/src/uart/tx_buffer.hpp"
#include "firmware/mc02/app/src/usb/helper.hpp"
#include "firmware/mc02/app/src/utility/loop_work.hpp"

namespace libhcs::firmware::uart {

namespace vc = libhcs::core::protocol::vendor_control;

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

    // 提交 solve_brr() 的结果。只在校验全部通过后调用, 无失败路径。BRR 只能在 UE=0 时写
    // (RM0468 USART_BRR), 所以拉低 UE 写完再原样放回; 窗口内到达的字节会失配, 调用方
    // 必须先静默链路(与 commit_framing() 同)。
    void commit_brr(uint32_t baudrate, uint32_t brr) {
        hal_uart_handle_->Init.BaudRate = baudrate;
        auto* instance = hal_uart_handle_->Instance;
        const uint32_t cr1 = instance->CR1;
        instance->CR1 = cr1 & ~USART_CR1_UE;
        instance->BRR = brr;
        instance->CR1 = cr1;
    }

    // 应用帧格式前的**纯校验**, 不碰任何寄存器。EP0 处理器先调它排除非法组合,
    // 于是"帧格式非法"这条路一个寄存器都没动 —— 与波特率的 solve_brr() 对称,
    // 二者合起来使 STALL 严格等于"什么都没改"。编码见 commit_framing() 上方。
    [[nodiscard]] bool check_framing(
        uint32_t word_length, uint32_t parity, uint32_t stop_bits, uint32_t rx_polarity) const {
        if (rx_polarity > 2U)
            return false;
        if (word_length != 0U && word_length != 7U && word_length != 8U)
            return false;
        if (parity != 0U && (parity < 1U || parity > 3U))
            return false;
        if (stop_bits != 0U && stop_bits != 1U && stop_bits != 2U)
            return false;
        return true;
    }

    // 提交 check_framing() 已通过的帧格式。字长直接用数据位数(7/8; 9 位不提供 --
    // RX 环是字节 DMA, 第九位会被静默截断), 校验与停止位用协议编码(0 = 保持不变;
    // 校验 1=无 2=偶 3=奇; 停止位 1=1 2=2; 1.5 停止位不提供 -- 本系列控制器只在
    // 5 位字长下实现它)。一次 UE 下拉内写完: M/PCE/PS/STOP 只能在 UE=0 时写,
    // 窗口内到达的字节会失配, 故调用方必须先静默链路。
    //
    // 本函数无失败路径 —— 校验已在 check_framing() 完成, 这正是拆开的目的: 旧
    // 的 set_framing() 把校验与写入揉在一起, 校验失败时虽已提前 return, 但
    // 一旦有人在这中间插入一次寄存器写, "STALL 即未改"就悄悄失效了。
    //
    // 接收极性(0 = 保持; 1 = 正常; 2 = 反相)是 MCU 引脚上的极性: DBUS 口板上
    // 已有硬件反相器, DBUS/SBUS 用正常, iBUS 用反相把反相器抵消掉。RXINV 在
    // CR2, 与 STOP 同样只能在 UE=0 时写, 所以放进同一个窗口。
    void commit_framing(
        uint32_t word_length, uint32_t parity, uint32_t stop_bits, uint32_t rx_polarity) {
        // M 管的是整帧位数(数据位 + 校验位), 不是数据位: 8 数据位加偶校验要 9 位帧
        // (M0), 这正是 CubeMX 给 DBUS 的 WORDLENGTH_9B + PARITY_EVEN。所以字长与
        // 校验必须合起来算: 先把没请求的那一半补成当前值, 再换算成 M。
        const uint32_t data_bits = word_length != 0U ? word_length : this->word_length();
        const uint32_t parity_code = parity != 0U ? parity : this->parity();
        uint32_t parity_bits = 0;
        switch (parity_code) {
        case 2U: parity_bits = USART_CR1_PCE; break;                // 偶
        case 3U: parity_bits = USART_CR1_PCE | USART_CR1_PS; break; // 奇
        default: break;                                             // 无校验
        }
        const uint32_t frame_bits = data_bits + (parity_bits != 0U ? 1U : 0U);
        uint32_t m_bits = 0; // 8 位帧
        if (frame_bits == 7U)
            m_bits = USART_CR1_M1;
        else if (frame_bits == 9U)
            m_bits = USART_CR1_M0;
        uint32_t stop_reg = 0;
        if (stop_bits == 2U)
            stop_reg = USART_CR2_STOP_1;

        // new_cr1 保留进来时的 UE: 最后那次写回才是"恢复"。这里曾把 UE 也一并掩掉,
        // 于是"恢复"写回的是 UE=0 -- 任何一次经 EP0 的串口配置之后端口就整个停了,
        // 收发都不通 [2026-10-03 上板才发现, 此前这条路只编译过]。
        auto* instance = hal_uart_handle_->Instance;
        const uint32_t cr1 = instance->CR1;
        const uint32_t new_cr1 =
            (cr1 & ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS)) | m_bits | parity_bits;

        instance->CR1 = new_cr1 & ~USART_CR1_UE; // UE=0, 帧格式字段可写
        uint32_t cr2 = instance->CR2;
        if (stop_bits != 0U)
            cr2 = (cr2 & ~USART_CR2_STOP) | stop_reg;
        if (rx_polarity != 0U)
            cr2 = rx_polarity == 2U ? (cr2 | USART_CR2_RXINV) : (cr2 & ~USART_CR2_RXINV);
        instance->CR2 = cr2;
        instance->CR1 = new_cr1; // UE 回到进来时的值
    }

    // ---- 帧格式读回: 从活寄存器解码, 编码同上, 永不返回 0(0 只表示"跳过") ----

    // 数据位 = M 给出的帧位数 - 校验位。DBUS 的 9 位帧 + 偶校验读回 8。
    [[nodiscard]] uint32_t word_length() const {
        const uint32_t cr1 = hal_uart_handle_->Instance->CR1;
        uint32_t frame_bits = 8U;
        if ((cr1 & USART_CR1_M0) != 0U)
            frame_bits = 9U;
        else if ((cr1 & USART_CR1_M1) != 0U)
            frame_bits = 7U;
        return frame_bits - ((cr1 & USART_CR1_PCE) != 0U ? 1U : 0U);
    }

    [[nodiscard]] uint32_t parity() const {
        const uint32_t cr1 = hal_uart_handle_->Instance->CR1;
        if ((cr1 & USART_CR1_PCE) == 0U)
            return 1U;                               // 无校验
        return (cr1 & USART_CR1_PS) != 0U ? 3U : 2U; // 奇 : 偶
    }

    [[nodiscard]] uint32_t stop_bits() const {
        // 1.5 停止位的编码不会由本驱动写出(见 check_framing); 万一出现, 如实上报
        // 协议中无此编码的原始值没有意义, 按最近的 2 处理并注释于此。
        return (hal_uart_handle_->Instance->CR2 & USART_CR2_STOP) == USART_CR2_STOP_1 ? 2U : 1U;
    }

    // 一个字符在线上的位数: 起始位 + 数据位 + 校验位 + 停止位。
    [[nodiscard]] uint32_t bits_per_character() const {
        return 1U + word_length() + (parity() != 1U ? 1U : 0U) + stop_bits();
    }

    // 1 = 正常, 2 = 反相(CR2.RXINV)。
    [[nodiscard]] uint32_t rx_polarity() const {
        return (hal_uart_handle_->Instance->CR2 & USART_CR2_RXINV) != 0U ? 2U : 1U;
    }

    // 实际编程的波特率, 从 BRR 反推而非 Init.BaudRate: 后者只是最近一次请求
    // 值, 被拒绝的请求两者都不会写。读 BRR 才能让主机区分"切换已生效"与
    // "切换被拒、仍按旧速率运行"。
    [[nodiscard]] uint32_t effective_baudrate() const {
        return baudrate_for(hal_uart_handle_->Instance->BRR & 0xFFFFU);
    }

    // 给定 BRR 值在本口时钟下的速率; EP0 用它在写入前核对 solve_brr() 的结果。
    [[nodiscard]] uint32_t baudrate_for(uint32_t brr) const {
        const uint32_t kernel_clock_hz = peripheral_clock_hz();
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

    // UartDivisor 的两个整数, 供 EP0 的 kGetPortConfig 上报与清单声明
    // 回读比对。**divisor 就是 BRR 寄存器本身**(低 16 位), 不做任何归一化 ——
    // 与 solve_brr() 交给 commit_brr() 写进去的那个整数逐位相等, 因此这里报的
    // 既是硬件事实, 又正是 solve_brr() 的返回值, 两端比对不需要任何换算。
    //
    // 8 倍过采样下 BRR 的低四位有特殊编码, 但那也是"实际写入的值", 如实上报;
    // 换算成真实波特率是 effective_baudrate() 的事。
    [[nodiscard]] uint16_t divisor_u16() const {
        return static_cast<uint16_t>(hal_uart_handle_->Instance->BRR & 0xFFFFU);
    }

    [[nodiscard]] uint8_t oversample() const {
        return hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8 ? 8U : 16U;
    }

    // 写入之后再回读, 确认 BRR 真的收下了这次编程。与 hpm 的
    // verify_baudrate() 同一契约, 见 core vendor_control.hpp 的 UartDivisor。
    [[nodiscard]] bool
        verify_baudrate(uint16_t expected_divisor, uint8_t expected_oversample) const {
        return divisor_u16() == expected_divisor && oversample() == expected_oversample;
    }

    // ---- 端口接口(core/src/link/ 的通用 UART 操作按这一组原语工作) ----
    //
    // 全部是冷路径(EP0 的清单声明与读回)。solve/commit_baudrate 是 solve_brr/
    // commit_brr 的统一签名视图: 核心流程对三块板写同一份代码, BRR 的编码差异留在
    // 各自的求解器里。过采样是 Init 的编译期事实(CubeMX), 不随速率变。
    [[nodiscard]] bool running() const { return started_; }

    [[nodiscard]] bool solve(uint32_t baudrate, uint16_t& divisor, uint8_t& oversample) const {
        uint32_t brr = 0;
        if (!solve_brr(baudrate, brr))
            return false;
        divisor = static_cast<uint16_t>(brr);
        oversample = hal_uart_handle_->Init.OverSampling == UART_OVERSAMPLING_8 ? 8U : 16U;
        return true;
    }

    [[nodiscard]] bool commit_baudrate(uint32_t baudrate, uint16_t divisor, uint8_t) {
        commit_brr(baudrate, divisor);
        return true; // commit_brr 无失败路径: 校验已在 solve 阶段完成
    }

    // 统一签名视图: 速率由 BRR 与本口时钟决定, 过采样是 Init 的编译期事实,
    // 不参与重建。
    [[nodiscard]] uint32_t baudrate_for(uint16_t divisor, uint8_t) const {
        return baudrate_for(static_cast<uint32_t>(divisor));
    }

    [[nodiscard]] bool framing_matches(
        uint32_t word_length, uint32_t parity, uint32_t stop_bits, uint32_t rx_polarity) const {
        return (word_length == 0U || word_length == this->word_length())
            && (parity == 0U || parity == this->parity())
            && (stop_bits == 0U || stop_bits == this->stop_bits())
            && (rx_polarity == 0U || rx_polarity == this->rx_polarity());
    }

    // kGetPortConfig 的应答: 硬件事实而非"上次请求"。波特率从 BRR 重构, 帧格式从
    // 活寄存器解码, control 恒为 0。divisor/oversample 一并上报 -- 它们是速率真正的
    // 整数形式, 主机就是靠这两个整数判定切换是否生效。见 UartDivisor。
    void read_config(vc::UartConfigPayload& out) const {
        out = {
            .baudrate = effective_baudrate(),
            .divisor = divisor_u16(),
            .oversample = oversample(),
            .word_length = static_cast<uint8_t>(word_length()),
            .parity = static_cast<uint8_t>(parity()),
            .stop_bits = static_cast<uint8_t>(stop_bits()),
            .control = 0,
            .rx_polarity = static_cast<uint8_t>(rx_polarity()),
        };
    }

    [[nodiscard]] core::link::PortStatus describe() const {
        return {.running = started_, .fd = false};
    }

    // ---- 启停: 端口只在主机声明之后才工作 ----
    //
    // 上电时所有端口都是停的: 不收、不发、不产生中断, 主循环也不去轮询它的寄存器。
    // 清单里有这一路(声明被接受), 端口随即启动
    // (core 的清单应用 -> resume()); 新的清单不再声明它、或会话结束
    // (ports::Registry::suspend_all())时停掉。配置先于启动生效, 所以端口一起来就已经是主机要的
    // 速率和帧格式。
    //
    // 具体的启停在各端口类的 start() / stop() 里, 这里只是它们共用的状态位。
    [[nodiscard]] bool started() const { return started_; }

    // 端口身份(丝印号)。ports.hpp 的注册表在上电时核对它与 EP0 绑定的身份一致。
    [[nodiscard]] data::DataId data_id() const { return data_id_; }

    // 运行时状态(core/src/link/port_status.hpp), 每个 keepalive 轮次在主循环读一次。
    //
    // 接收错误按轮计, 与 hpm 同口径: 一个计数 = "这一轮里出现过这种错误", 每种每轮至多
    // +1。RX 路径有意不开 CR3.EIE / CR1.PEIE(见 RxBuffer::configure_rx_error_policy()),
    // 线路错误不进中断, HAL 错误回调也就不会为 PE/NE/FE 而来; 但 ISR 的 PE/FE/NE 是粘滞
    // 位, DMA 取走 RDR 不清, 只有写 ICR 才清(HAL_UART_IRQHandler 在错误中断关着时也不
    // 碰), 所以每轮读一次、记下、清掉, 不需要中断。溢出检测关着(OVRDIS), overrun 只来自
    // note_rx_errors()。
    [[nodiscard]] data::UartStatusView read_status() {
        USART_TypeDef* const instance = hal_uart_handle_->Instance;
        const uint32_t flags = instance->ISR & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE);
        if (flags != 0U) {
            uint32_t clear = 0U;
            if ((flags & USART_ISR_PE) != 0U) {
                parity_errors_.note();
                clear |= USART_ICR_PECF;
            }
            if ((flags & USART_ISR_FE) != 0U) {
                framing_errors_.note();
                clear |= USART_ICR_FECF;
            }
            if ((flags & USART_ISR_NE) != 0U) {
                noise_errors_.note();
                clear |= USART_ICR_NECF;
            }
            instance->ICR = clear;
        }
        return {
            .overrun = overrun_.count(),
            .parity = parity_errors_.count(),
            .framing = framing_errors_.count(),
            .noise = noise_errors_.count(),
            .tx_dropped = tx_dropped_.count(),
            .rx_dropped = rx_dropped_.count(),
        };
    }

protected:
    UartCommon(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle)
        : data_id_(data_id)
        , hal_uart_handle_(hal_uart_handle) {}

    // 启动即在 loop::active 里置本口的位(位号 = DataId), 主循环从下一圈起轮询它; 清位由
    // ports.hpp 的 poll_uarts() 在口停下且发完之后做。
    void mark_active() const { loop::set(loop::bit(data_id_)); }

    // RxBuffer 与 TxBuffer 各存一份句柄, 派生端口无法无限定地访问
    // hal_uart_handle_; 把所需的两处访问经此路由, 避免到处写 UartCommon:: 限定。
    //
    // HAL 错误回调(中断上下文)的第一步: 按 ErrorCode 的位各记一次, 返回是否有接收
    // 错误。HAL 对非阻塞错误(PE/NE/FE)回调后即清 ErrorCode, 阻塞错误(ORE)由
    // rx_error_callback() 重启接收时清, 所以每次回调看到的都是新发生的错误。
    [[nodiscard]] bool note_rx_errors() {
        const uint32_t code = hal_uart_handle_->ErrorCode;
        if ((code & HAL_UART_ERROR_ORE) != 0U)
            overrun_.note();
        if ((code & HAL_UART_ERROR_PE) != 0U)
            parity_errors_.note();
        if ((code & HAL_UART_ERROR_FE) != 0U)
            framing_errors_.note();
        if ((code & HAL_UART_ERROR_NE) != 0U)
            noise_errors_.note();
        constexpr uint32_t rx_error_mask =
            HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE;
        return (code & rx_error_mask) != 0U;
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
        const auto result = serializer.write_uart(
            data_id_, {.uart_data = payload, .idle_delimited = is_idle}, payload2);
        if (result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]]
            rx_dropped_.note();
        core::utility::assert_always(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
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
            // UART_SetConfig 同样移位。2026-10-04 起 .ioc 里所有串口都选 PLL3Q
            // (晶振), 此路径暂时不走; 留着让 .ioc 改回 HSI 时仍然算对。
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

    // 运行时状态的计数(read_status())。接收错误与上行丢弃在中断里记, 下行丢弃在
    // 主循环里记; 各自单写者。
    utility::EventCounter overrun_;
    utility::EventCounter parity_errors_;
    utility::EventCounter framing_errors_;
    utility::EventCounter noise_errors_;
    utility::EventCounter tx_dropped_;
    utility::EventCounter rx_dropped_;
    // 只在主循环读写(EP0 处理器、会话状态机、try_transmit 都在那里)。
    bool started_ = false;
};

// 全双工端口: USART1、UART7、USART10, CubeMX 为每个口接好 RX 与 TX 两条
// DMA 流。
class Uart
    : public UartCommon
    , private TxBuffer</*half_duplex=*/false>
    , private RxBuffer<Uart> {
    friend class RxBuffer<Uart>;

public:
    // EP0 口能力: 无。
    static constexpr uint8_t kPortCapabilities = 0;

    // 这个口交给 DMA 的全部内存, 与端口对象分开放(见本文件末尾的对象定义)。
    struct DmaMemory {
        RxBuffer::DmaMemory rx;
        TxBuffer::DmaMemory tx;
    };

    using Lazy = utility::Lazy<Uart, data::DataId, UART_HandleTypeDef*, DmaMemory*>;

    Uart(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle, DmaMemory* dma)
        : UartCommon(data_id, hal_uart_handle)
        , TxBuffer(
              hal_uart_handle, dma->tx, &hal_tx_dma_complete_callback, &hal_tx_dma_error_callback)
        , RxBuffer(hal_uart_handle, dma->rx) {
        update_line_rate(); // CubeMX 的初始速率与帧格式
    }

    // 速率与帧格式只在这两处变(EP0 清单应用): 写完按新的字符时间重算发送侧的包间空隙。
    // 遮住 UartCommon 的同名原语, core 的 uart_apply() 经端口类型调到这里。半双工的
    // UartRs485 不需要: 它的包间等待是总线换向(kTurnaroundDeadline), 不是帧间隔。
    bool commit_baudrate(uint32_t baudrate, uint16_t divisor, uint8_t oversample) {
        const bool committed = UartCommon::commit_baudrate(baudrate, divisor, oversample);
        update_line_rate();
        return committed;
    }

    void commit_framing(
        uint32_t word_length, uint32_t parity, uint32_t stop_bits, uint32_t rx_polarity) {
        UartCommon::commit_framing(word_length, parity, stop_bits, rx_polarity);
        update_line_rate();
    }

    void handle_downlink(const data::UartDataView& data) {
        // 没声明的端口不发: 主机往它写的字节直接丢弃。
        if (!started_)
            return;
        if (!TxBuffer::try_enqueue(data)) {
            tx_dropped_.note();
            led::led->downlink_buffer_full();
        }
    }

    // @return 这一口之后还要不要主循环来轮询(见 ports.hpp 的 poll_uarts())
    bool try_transmit() {
        if (started_) {
            RxBuffer::try_dequeue();
            TxBuffer::try_dequeue();
            return true;
        }
        if (draining_) {
            // 停口之前已经收下的字节照常发完, 发完就不再轮询。
            TxBuffer::try_dequeue();
            draining_ = !TxBuffer::drained();
        }
        return draining_;
    }

    void start() {
        if (started_)
            return;
        RxBuffer::start_rx();
        started_ = true;
        mark_active();
    }

    void stop() {
        if (!started_)
            return;
        started_ = false;
        RxBuffer::stop_rx();
        draining_ = !TxBuffer::drained();
    }

    // 端口接口的统一名(见 core/src/link/port_ops.hpp): 启停语义与本板"上电全停"
    // 的策略一致 -- 声明清单里没有的口回到停的状态。
    void suspend() { stop(); }
    void resume() { start(); }

    void tx_complete_callback() { TxBuffer::tx_complete_callback(); }

    void uart_error_callback() {
        if (note_rx_errors())
            RxBuffer::rx_error_callback();
    }

    void rx_dma_error_callback() {
        flag_dma_error();
        RxBuffer::rx_error_callback();
    }

    void tx_dma_error_callback() { TxBuffer::tx_error_callback(); }

    void rx_event_callback() { RxBuffer::uart_idle_event_callback(); }

private:
    void update_line_rate() { TxBuffer::set_line_rate(effective_baudrate(), bits_per_character()); }

    static void hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_complete_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    // 已停口、但发送 ring 里还有停口前收下的字节。
    bool draining_ = false;
};

// 仅接收端口, 用于 UART5 (DBUS)。
//
// 无 TX 路径的两个独立原因: CubeMX 只给 UART5 接了 RX DMA 流
// (bsp/cubemx/Core/Src/usart.c 只声明 hdma_uart5_rx 而无 hdma_uart5_tx); 协议
// 也不会把下行路由到 kUartDbus, usb/vendor.hpp 的 uart_deserialized_callback
// 只分发 kUart1/kUart2/kUart3/kUart7/kUart10(不含 DBUS)。
// 到这里的只有 EP0 清单里 kUartDbus 的声明与读回。
class UartRxOnly
    : public UartCommon
    , private RxBuffer<UartRxOnly> {
    friend class RxBuffer<UartRxOnly>;

public:
    // EP0 口能力: DBUS 口有 RX 反相(RXINV 抵消板上反相器, iBUS 接收机由此可用), 声明里的
    // rx_polarity 是设置。
    static constexpr uint8_t kPortCapabilities = spec::kUartCapRxPolaritySettable;

    using DmaMemory = RxBuffer::DmaMemory;

    using Lazy = utility::Lazy<UartRxOnly, data::DataId, UART_HandleTypeDef*, DmaMemory*>;

    UartRxOnly(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle, DmaMemory* dma)
        : UartCommon(data_id, hal_uart_handle)
        , RxBuffer(hal_uart_handle, *dma) {}

    // 与全双工端口同名, 便于统一轮询所有端口; 此处仅排空接收 ring。
    // @return 这一口之后还要不要主循环来轮询(见 ports.hpp 的 poll_uarts())
    bool try_transmit() {
        if (started_)
            RxBuffer::try_dequeue();
        return started_;
    }

    void start() {
        if (started_)
            return;
        RxBuffer::start_rx();
        started_ = true;
        mark_active();
    }

    void stop() {
        if (!started_)
            return;
        started_ = false;
        RxBuffer::stop_rx();
    }

    void suspend() { stop(); }
    void resume() { start(); }

    void uart_error_callback() {
        if (note_rx_errors())
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
    // EP0 口能力: 无。
    static constexpr uint8_t kPortCapabilities = 0;

    struct DmaMemory {
        RxBuffer::DmaMemory rx;
        TxBuffer::DmaMemory tx;
    };

    using Lazy = utility::Lazy<UartRs485, data::DataId, UART_HandleTypeDef*, DmaMemory*>;

    UartRs485(data::DataId data_id, UART_HandleTypeDef* hal_uart_handle, DmaMemory* dma)
        : UartCommon(data_id, hal_uart_handle)
        , TxBuffer(
              hal_uart_handle, dma->tx, &hal_tx_dma_complete_callback, &hal_tx_dma_error_callback)
        , RxBuffer(hal_uart_handle, dma->rx) {}

    void handle_downlink(const data::UartDataView& data) {
        // 没声明的端口不发: 主机往它写的字节直接丢弃。
        if (!started_)
            return;
        if (!TxBuffer::try_enqueue(data)) {
            tx_dropped_.note();
            led::led->downlink_buffer_full();
        }
    }

    // 把原始 IDLE 计数喂给 turnaround 门, 刻意不用 RxBuffer::try_dequeue()
    // 维护的 consumed_idle_count_: 总线是否重新空闲只取决于对端是否停口, 与
    // 主机是否已取走字节无关。
    // @return 这一口之后还要不要主循环来轮询(见 ports.hpp 的 poll_uarts())
    bool try_transmit() {
        if (started_) {
            RxBuffer::try_dequeue();
            TxBuffer::try_dequeue(RxBuffer::idle_count());
            return true;
        }
        if (draining_) {
            // 停口之前已经收下的事务照常发完, 发完就不再轮询。接收已停, IDLE 计数
            // 不再前进, turnaround 门退回它的超时。
            TxBuffer::try_dequeue(RxBuffer::idle_count());
            draining_ = !TxBuffer::drained();
        }
        return draining_;
    }

    void start() {
        if (started_)
            return;
        RxBuffer::start_rx();
        started_ = true;
        mark_active();
    }

    void stop() {
        if (!started_)
            return;
        started_ = false;
        RxBuffer::stop_rx();
        draining_ = !TxBuffer::drained();
    }

    // 端口接口的统一名(见 core/src/link/port_ops.hpp): 启停语义与本板"上电全停"
    // 的策略一致 -- 声明清单里没有的口回到停的状态。
    void suspend() { stop(); }
    void resume() { start(); }

    void tx_complete_callback() { TxBuffer::tx_complete_callback(); }

    void uart_error_callback() {
        if (note_rx_errors())
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
    // RxBuffer::try_dequeue() 在字节仍在到达时的分块流式转发(当时按 32 字节分块,
    // 2026-10-04 起按时间门槛 kHoldCycles)。数据见 firmware/mc02/AGENTS.md, 该开关
    // 因此被移除。
    void rx_event_callback() { RxBuffer::uart_idle_event_callback(); }

private:
    static void hal_rx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_complete_callback(DMA_HandleTypeDef* hal_dma_handle);

    static void hal_tx_dma_error_callback(DMA_HandleTypeDef* hal_dma_handle);

    // 已停口、但发送 ring 里还有停口前收下的字节。
    bool draining_ = false;
};

// 每个端口分成两块放 [2026-10-03]:
//
//   - DMA 读写的内存(ring 与 staging, 各口的 DmaMemory)在 D2 SRAM (.d2_sram,
//     0x30000000)。驱动 UART RX/TX 流的 DMA1/DMA2 是 D2 域主设备: 放在这里每次传输
//     都留在域内, 不跨 D2-D1 互连去 AXI SRAM 与 M7 争用。第二个原因(non-cacheable
//     MPU 窗口下的 AXI SRAM 余量)与 app.cpp 开机需做的事, 见
//     bsp/linker/STM32H723VGTx_APP.ld 的 .d2_sram 注释。这块不能进 .dtcm: DTCM 仅
//     核内可达, 指向它的 DMA 流会静默地什么都传不了。
//   - 端口对象本身(下标、计数、标志、检查点队列)在零等待的 DTCM (.dtcm), 和 CAN
//     对象一样。此前它跟着 ring 一起在 D2 SRAM 里, 而那一区是非缓存的: 主循环每圈
//     轮询一个已声明的端口要读写十来个成员, 每个都是一次跨域总线事务, 一个全双工
//     口每圈因此多花约 200 个周期。实测见 PACKET_RATE_LOG.md 4.5 节。
//
// 新增端口时两样都要写: 漏掉 section 属性的 DmaMemory 会落进 .bss, 那里目前恰好
// 也是非缓存的 AXI SRAM, 功能上能跑, 但每个字节都要跨域。
[[gnu::section(".d2_sram")]] inline constinit Uart::DmaMemory uart1_memory{};
[[gnu::section(".d2_sram")]] inline constinit Uart::DmaMemory uart7_memory{};
[[gnu::section(".d2_sram")]] inline constinit Uart::DmaMemory uart10_memory{};
[[gnu::section(".d2_sram")]] inline constinit UartRxOnly::DmaMemory uart_dbus_memory{};

[[gnu::section(".dtcm")]] inline constinit Uart::Lazy uart1{
    spec::mc02::Spec::Uarts::kUart1.data_id, &huart1, &uart1_memory};
[[gnu::section(".dtcm")]] inline constinit Uart::Lazy uart7{
    spec::mc02::Spec::Uarts::kUart7.data_id, &huart7, &uart7_memory};
[[gnu::section(".dtcm")]] inline constinit Uart::Lazy uart10{
    spec::mc02::Spec::Uarts::kUart10.data_id, &huart10, &uart10_memory};
[[gnu::section(".dtcm")]] inline constinit UartRxOnly::Lazy uart_dbus{
    spec::mc02::Spec::Uarts::kDbus.data_id, &huart5, &uart_dbus_memory};

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
// 始终编译 —— 它们就是机壳的 UART2/UART3, 不是叠在备用 DataId 上的别名;
// 诊断构建改用 kUart0 输出, 与这两个口不再冲突。
//
// 原先有 libhcs_APP_RS485_ENABLE 开关, 已取消 [2026-09-30, UART_EP0_MIGRATION.md
// 阶段 7]: 端口在每块同型板上都存在, 去掉它只会得到一份 EP0 下标空间带两个洞的
// 镜像 —— 主机分不清"这块板没有 UART2"与"这版镜像把 UART2 编掉了"。代价是
// ~1.8 KB D2 SRAM 常驻(Lazy 与其 DMA ring 在链接期就占位, 与是否 init() 无关),
// 已明确接受。
[[gnu::section(".d2_sram")]] inline constinit UartRs485::DmaMemory uart2_memory{};
[[gnu::section(".dtcm")]] inline constinit UartRs485::Lazy uart2{
    spec::mc02::Spec::Uarts::kUart2.data_id, &huart2, &uart2_memory};

// 第二个 RS-485 口, USART3 经收发器 U6: 485_DIR1 由 PB14 上的 USART3_DE 驱动,
// 数据走 PD8/PD9, 总线接 P5 连接器。电路与上述 U5 相同, RE# 接 DE, R12 把 RO
// 拉到 3V3 使禁用的接收器维持空闲电平, R18 把 DE 网络拉低使端口上电即接收,
// R16 在本端装 120R 端接。
[[gnu::section(".d2_sram")]] inline constinit UartRs485::DmaMemory uart3_memory{};
[[gnu::section(".dtcm")]] inline constinit UartRs485::Lazy uart3{
    spec::mc02::Spec::Uarts::kUart3.data_id, &huart3, &uart3_memory};

// 主循环要轮询的端口在 loop::active 里各占一位, 位号就是端口的 DataId(utility/loop_work.hpp):
// 主机声明过的, 加上停口之后发送 ring 还没排空的。端口的 start() 自己置位; stop() 不清位,
// 停掉的口在下一圈被轮询最后一次, 由 try_transmit() 的返回值决定是清位还是留着排空。
// 按位分发到端口对象的是注册表(ports.hpp 的 poll_uarts()), 这里不另立"第几路"的表。

} // namespace libhcs::firmware::uart
