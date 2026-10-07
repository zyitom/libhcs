#pragma once

#include <cstdint>

#include "core/include/libhcs/data/datas.hpp"

namespace libhcs::core::link {

// 下行流错误账本, 三块板的固件共用这一份(各持一个实例)。反序列化拒收或解析失败时
// 经 error_callback() 记账; 它就是链路这个"口"(DataId::kSession)的运行时状态来源,
// status() 交给端口状态的上报(port_status.hpp), 与 CAN / UART 走同一条路: 变了才发、
// 每个 kStart 之后作基线重发 -- 限流与"有没有新错误"都由那边的账本判断, 这里不另记。
//
// 错误计数从上电起自由累加(按 2^16 回绕), 与其他口的计数同一口径, 主机取差; 传输序号
// 从本会话起算(begin_session()), 主机据此把 last_transfer 与它在这个会话里发出的下行
// 传输对上。
//
// 全部成员只在主循环线程读写(deserializer 与 serializer 同线程), 无需同步; 每字节热路径
// 不经过这里, note() 只在出错分支上跑。
class DownlinkErrors {
public:
    // 一条下行记录没能交付。field 是记录的字段号; 连字段号都读不出来时为
    // data::DataId::kExtend。
    void note(data::DownlinkError reason, data::DataId field) noexcept {
        ++status_.downlink_errors;
        status_.last_reason = reason;
        status_.last_field = field;
        // 当前传输还没结束(end_transfer() 尚未调用): 错误记在"下一次结束"的那个序号上,
        // 即本会话第 (transfers_ + 1) 次下行传输。
        status_.last_transfer = static_cast<std::uint16_t>(transfers_ + 1U);
    }

    // 一次下行传输结束(finish_downlink_transfer)。之后进来的错误归下一次传输。
    void end_transfer() noexcept { ++transfers_; }

    // 新会话: 传输序号从头数。错误计数不清 -- 它是自由计数, 清了主机取的差会跳。
    void begin_session() noexcept { transfers_ = 0; }

    [[nodiscard]] const data::LinkStatusView& status() const noexcept { return status_; }

private:
    data::LinkStatusView status_{};
    std::uint16_t transfers_ = 0;
};

} // namespace libhcs::core::link
