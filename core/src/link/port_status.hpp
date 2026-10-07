#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <variant>

#include "core/include/libhcs/data/datas.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"

// 端口运行时状态的上报(data::SessionType::kPortStatus), 三块板共用。
//
// 运行时状态只走数据流, 不走 EP0: EP0 是配置面(握手、声明、查清单), 控制传输在
// USB 调度上排在 bulk 前面、主机侧是一次同步往返, 按轮询频率放大就是控制环的抖动
// 来源。这里的做法是"读寄存器/计数器 -> 与上次发出的比较 -> 变了才发", 挂在
// keepalive 应答后面(每轮 250 ms):
//   - 热路径零新增: 计数在各驱动的出错分支里累加, 读与比较只在 keepalive 那一下;
//   - 天然限流: 每口每轮至多一条, 状态不变不发;
//   - 无需同步: 账本只在主循环读写(反序列化与序列化同线程)。
//
// 状态有哪几种只在 data::PortStatusVariant 登记一次; 这里对它泛型, 不按种类各写一份,
// 也没有任何一种的特例: 读数就是状态(计数、电平, 以及驱动自己锁存的"最近一次错误"),
// 账本只比较。一个口的种类由它的 DataId 定死(视图的 is_for()), 所以账本是"每口一个
// variant"。链路本身(DataId::kSession, 下行流错误)也是这样一个口。
//
// 每个 kStart 处 reset(): 之后第一轮每个在跑的口都发一条, 不论变没变 -- 主机的基线,
// 主机那边因此从不需要清快照。计数是开机以来按 2^16 回绕的自由计数, 主机只看相邻两次
// 的差。一条记录(会话头 + 口号/长度 + 正文): 链路 12 字节、CAN 17 字节、UART 18 字节。

namespace libhcs::core::link {

// "上次发出的"表, 按 DataId 索引(紧凑字段号, 0..15), 每口一个 variant。新建时与刚
// reset() 一样: 每个口的第一次 offer 都发, 作基线。
class PortStatusLedger {
public:
    PortStatusLedger() noexcept { reset(); }

    // 与上次发出的比较: 不同(或 reset() 之后还没发过)为真。不改账本 -- 真写进上行流
    // 了才 commit(), 缓冲满丢掉的那条下一轮照样会再发。
    template <protocol::PortStatusKind View>
    [[nodiscard]] bool changed(data::DataId port, const View& reading) const noexcept {
        const auto slot = index(port);
        const auto* sent = std::get_if<View>(&sent_[slot]);
        return pending_baseline_[slot] || sent == nullptr || reading != *sent;
    }

    template <protocol::PortStatusKind View>
    void commit(data::DataId port, const View& sent) noexcept {
        const auto slot = index(port);
        sent_[slot] = sent;
        pending_baseline_[slot] = false;
    }

    // 新会话: 每个口下一次都发一条作基线。
    void reset() noexcept {
        sent_ = {};
        pending_baseline_.fill(true);
    }

private:
    static std::size_t index(data::DataId port) noexcept {
        const auto index = static_cast<std::size_t>(port);
        utility::assert_debug(index < kSlots);
        return index;
    }

    static constexpr std::size_t kSlots = 16;
    std::array<data::PortStatusVariant, kSlots> sent_{};
    std::array<bool, kSlots> pending_baseline_{};
};

// 一轮 keepalive 的上报出口: 板子把它的口逐个 offer 进来, 变了的立即写进上行流,
// 紧跟在刚写下的 kKeepaliveAck 之后。
class PortStatusRound {
public:
    PortStatusRound(
        PortStatusLedger& ledger, protocol::Serializer& serializer, std::uint32_t nonce) noexcept
        : ledger_(ledger)
        , serializer_(serializer)
        , nonce_(nonce) {}

    template <protocol::PortStatusKind View>
    void offer(data::DataId port, const View& reading) noexcept {
        if (!ledger_.changed(port, reading))
            return;
        // 上行缓冲满就不记账: 下一轮同样的变化还在, 再发一次。每条记录都是全量快照,
        // 晚到不会错。
        if (serializer_.write_port_status(nonce_, port, reading)
            == protocol::Serializer::SerializeResult::kSuccess)
            ledger_.commit(port, reading);
    }

private:
    PortStatusLedger& ledger_;
    protocol::Serializer& serializer_;
    std::uint32_t nonce_;
};

// 遍历注册表: 本 PCB 实有且在跑(被声明)、驱动提供 read_status() 的口, 读状态交给
// round。没声明的口挂起着, 不在总线上, 没有状态可报。哪类口参与, 只看驱动有没有这个
// 原语(port_ops.hpp 的 CanPortDriver / UartPortDriver 要求它), 不在这里列种类; 读出
// 的视图必须是这个口的(is_for), 接错是编译错误。编译期展开, 没有状态的口不生成代码。
template <typename Registry, typename Round>
void offer_port_status(Round& round) {
    Registry::for_each_live([&round](auto binding, auto& port) {
        if constexpr (requires {
                          port.read_status();
                          port.running();
                      }) {
            using View = decltype(port.read_status());
            static_assert(protocol::PortStatusKind<View>);
            static_assert(
                View::is_for(decltype(binding)::data_id),
                "a driver's read_status() returns a status of another port kind");
            if (port.running())
                round.offer(decltype(binding)::data_id, port.read_status());
        }
    });
}

} // namespace libhcs::core::link
