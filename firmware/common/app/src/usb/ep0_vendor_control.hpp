#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <common/tusb_types.h>
#include <device/usbd.h>

#include "core/include/libhcs/protocol/vendor_control.hpp"
#include "core/src/link/ep0.hpp"
#include "core/src/link/port.hpp"
#include "firmware/common/app/src/usb/ep0_staging.hpp"

// EP0 配置通道的 TinyUSB 胶水 -- 三块板共用一份。
//
// 请求分发、清单的两阶段提交、拒绝原因锁存的规则在核心(core/src/link/ep0.hpp); 这里只
// 把 TinyUSB 的 SETUP/DATA 两段搬给核心, 再把核心的决定交回 TinyUSB。各板的
// vendor_control.cpp 只剩自己的板级上下文(归属策略、时钟、时延分解)与一行转发。
//
// 运行位置见 ep0_staging.hpp: TinyUSB 在 tud_task() 里解码控制传输, 全部在主循环上。

namespace libhcs::firmware::usb::ep0 {

// 板级上下文的公共半边: 暂存缓冲、拒绝锁存、清单结果, 以及"本板没有这项"的默认
// 策略。各板从它派生, 只覆盖自己真有的:
//   on_manifest_begin() / on_manifest_failed()   归属交接(只有 hpm 有附加功能可交接)
//   on_manifest_accepted(now)                    必须由派生类给出: 声明生效后各板放行
//                                                会话的方式不同
//   now()                                        声明时限的时钟(只有 hpm 有时限)
//   read_latency(reset)                          kGetLatencyBreakdown(只有 hpm 有)
//   kBoardCaps / set_time_sync(on)               共享时间基准(hpm、mc02、c_board 有)
// 覆盖是静态的(核心以模板参数拿到派生类型), 没有虚调用。
class ContextBase {
public:
    // 核心把每个 IN 应答写进暂存缓冲; 最大的两个是清单结果与 hpm 的时延分解。
    static_assert(kStagingCapacity >= sizeof(vc::ManifestResultPayload));
    static_assert(kStagingCapacity >= sizeof(vc::LatencyBreakdownPayload));

    static void* reply_buffer() { return g_control_buffer; }

    static void record_error(vc::Request request, uint16_t index, core::link::PortOutcome outcome) {
        record_config_error(request, index, outcome.reason, outcome.value);
    }

    static void fill_last_error(vc::LastConfigErrorPayload& out) { out = g_last_config_error; }

    core::link::ep0::ManifestJournal& journal() { return journal_; }

    static void on_manifest_begin() {}
    static void on_manifest_failed() {}
    static uint64_t now() { return 0; }
    static bool read_latency(bool /*reset*/) { return false; }

    // 没有时间基准的板: 不报这项能力, 核心也就不会要求打开它(关掉总是成功)。
    static constexpr uint8_t kBoardCaps = 0;
    [[nodiscard]] static bool set_time_sync(bool on) { return !on; }

private:
    core::link::ep0::ManifestJournal journal_{};
};

namespace internal {

// SETUP/DATA 两段共用的请求视图: 把 TinyUSB 的 setup 包翻译成核心要问的四个答案。
struct RequestView {
    const tusb_control_request_t* request;

    [[nodiscard]] uint8_t brequest() const { return request->bRequest; }
    [[nodiscard]] uint16_t windex() const { return request->wIndex; }
    [[nodiscard]] uint16_t wlength() const { return request->wLength; }
    [[nodiscard]] bool is_in() const { return request->bmRequestType == vc::kRequestTypeIn; }
};

} // namespace internal

// tud_vendor_control_xfer_cb 的全部 libhcs 部分。返回 false = STALL。
template <typename Registry, typename Context>
bool vendor_control_xfer(
    Context& context, uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    const internal::RequestView view{request};
    switch (stage) {
    case CONTROL_STAGE_SETUP: {
        const auto decision = core::link::ep0::setup<Registry>(context, view);
        switch (decision.action) {
        case core::link::ep0::SetupAction::kReply:
            // wLength 精确匹配而非截断: 要求不同尺寸的主机说的是本接口的另一个版本,
            // 截短应答会让它把垃圾解码成合法回复。
            return request->wLength == decision.reply_size
                && tud_control_xfer(
                       rhport, request, static_cast<uint8_t*>(context.reply_buffer()),
                       static_cast<uint16_t>(decision.reply_size));
        case core::link::ep0::SetupAction::kAcceptOut:
            // 先接受数据段(核心已按 wLength 核对过形状, 不超过暂存容量), 值等到达后
            // 再校验。
            return tud_control_xfer(rhport, request, g_control_buffer, request->wLength);
        default: return false;
        }
    }
    case CONTROL_STAGE_DATA:
        // 在这里返回 false 会 STALL 状态段 -- 这正是配置搬到 EP0 的全部意义: 这是
        // bulk 流承载不了的应答。
        return core::link::ep0::data<Registry>(
            context, view,
            std::span{reinterpret_cast<const std::byte*>(g_control_buffer), request->wLength});
    default: return true; // CONTROL_STAGE_ACK
    }
}

} // namespace libhcs::firmware::usb::ep0
