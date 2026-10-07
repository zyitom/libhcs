#include "core/include/libhcs/protocol/vendor_control.hpp"

#include <cstdint>
#include <cstring>

#include <common/tusb_types.h>
#include <device/usbd.h>
#include <hpm_clock_drv.h>
#include <hpm_soc.h>

#include "firmware/common/app/src/usb/ep0_vendor_control.hpp"
#include "firmware/hpm_board/app/src/diag/latency.hpp"
#include "firmware/hpm_board/app/src/sync/sof.hpp"
#include "firmware/hpm_board/app/src/timer/timer.hpp"
#include "firmware/hpm_board/app/src/usb/usb_descriptors.hpp"
#include "firmware/hpm_board/app/src/usb/vendor.hpp"
#include "ports.hpp"

// EP0 配置通道 -- core/include/libhcs/protocol/vendor_control.hpp 的板端一半。
//
// 请求分发、清单的两阶段提交、错误锁存在 core/src/link/ep0.hpp(纯逻辑核心), TinyUSB
// 胶水在 firmware/common 的 ep0_vendor_control.hpp, 三块板共用; 本文件只是它对这块板
// 的实例化: 注册表来自 ports.hpp 的绑定(spec 口表 -> 驱动), 板级上下文在下面 --
// 归属交接与时延分解, 这两样只有本板有。
//
// 运行位置: TinyUSB 在 USB ISR 里排队 setup 包, 在 tud_task() 中解码, 因此下面的代码
// 全部运行在主循环上, 与 bulk 下行回调同线程、同轮次的同一点。这对 UART 路径至关重要:
// 应用波特率会中止发送 DMA 并置 LCR.DLAB, 期间发往 THR 的 DMA 写会落进除数锁存
// (uart/uart.hpp); 若改在中断上下文执行, 恰好重新引入该竞争。
//
// 不受会话门控: 主机在构造 board 对象时声明端口, 早于其 keepalive 线程开会话; 会话已
// 失效的板子也必须能应答读回。配置是传输层状态, 不是数据面状态。
//
// 配置即声明, 一件事务: kGetPortList 是纯读; kApplyManifest 校验通过后先从附加功能手里
// 接过板子, 再动端口, 全部生效才落定归 libhcs, 中途失败则原样还回去(usb/vendor.hpp 的
// BoardOwnership)。

namespace {

namespace vc = libhcs::core::protocol::vendor_control;
namespace latency = libhcs::firmware::diag::latency;

using Registry = libhcs::firmware::ports::Registry;

// 板级上下文: 归属交接的三个时刻与时延分解。其余(锁存、暂存、清单结果)是公共半边。
class BoardContext : public libhcs::firmware::usb::ep0::ContextBase {
public:
    static void on_manifest_begin() { libhcs::firmware::usb::vendor->begin_claim(); }
    static void on_manifest_accepted(uint64_t now) {
        libhcs::firmware::usb::vendor->commit_claim(now);
    }
    static void on_manifest_failed() { libhcs::firmware::usb::vendor->abort_claim(); }

    // 共享时间基准: 每个镜像都带, 清单要了才开(core 在端口应用之前调)。
    static constexpr uint8_t kBoardCaps = vc::kBoardCapTimeSync;
    [[nodiscard]] static bool set_time_sync(bool on) {
        if (on)
            return libhcs::firmware::sync::time_sync_start();
        libhcs::firmware::sync::time_sync_stop();
        return true;
    }

    // 声明时限的时钟, 与会话租约同一单位(core/src/link/ownership.hpp)。
    static uint64_t now() { return libhcs::firmware::timer::Timer::timestamp64_quarter_us(); }

    // kGetLatencyBreakdown, 本板独有。index 非零时读后清零累加器, 调用方可以夹住一段
    // 测量; 用 wIndex 而非 wValue: 主机侧辅助函数只暴露 wIndex。
    static bool read_latency(bool reset) {
        const auto down = latency::downlink;
        const auto up = latency::uplink;
        if (reset)
            latency::reset();
        const auto payload = vc::LatencyBreakdownPayload{
            .downlink_count = down.count,
            .downlink_min_cycles = down.count ? down.min_cycles : 0,
            .downlink_max_cycles = down.max_cycles,
            .downlink_sum_cycles = down.sum_cycles,
            .uplink_count = up.count,
            .uplink_min_cycles = up.count ? up.min_cycles : 0,
            .uplink_max_cycles = up.max_cycles,
            .uplink_sum_cycles = up.sum_cycles,
            .cpu_hz = clock_get_frequency(clock_cpu0),
            .reserved = 0,
        };
        std::memcpy(reply_buffer(), &payload, sizeof(payload));
        return true;
    }
};

BoardContext g_board_context;

// Windows WCID (MS OS 2.0): BOS 平台能力广告了 kMsOsVendorCode; 该码的 vendor IN 请求且
// wIndex 0x0007 = GET_MS_OS_20_DESCRIPTOR (Microsoft 规范固定值)。集合直接从 flash 常量
// 应答 -- 它有 848 字节, 暂存缓冲只有清单的容量, 不能走核心的拷贝路径; EP0 IN 传输对
// buffer 只读, 与 usbd.c 应答配置描述符(同样在 flash)一致。wLength 要求精确等于集合
// 总长: Windows 按 BOS 里广告的 wDescriptorSetLength 请求, 别的长度说明对端在说别的
// 版本。与 libhcs 的请求码不冲突(此码 0x21)。
bool is_ms_os_20_request(const tusb_control_request_t* request) {
    return request->bmRequestType == vc::kRequestTypeIn
        && request->bRequest == libhcs::firmware::usb::UsbDescriptors::kMsOsVendorCode;
}

bool answer_ms_os_20(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    if (stage != CONTROL_STAGE_SETUP)
        return true;
    if (request->wIndex != 0x0007
        || request->wLength != libhcs::firmware::usb::UsbDescriptors::kMsOs20SetLength)
        return false;
    // TinyUSB 的 tud_control_xfer 只收非 const 缓冲, 描述符本体是 constexpr 表。
    return tud_control_xfer(
        rhport, request,
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<uint8_t*>(libhcs::firmware::usb::UsbDescriptors::get_ms_os_20_set()),
        libhcs::firmware::usb::UsbDescriptors::kMsOs20SetLength);
}

} // namespace

// 覆盖 TinyUSB 的弱定义, 后者对每个 vendor 请求一律 STALL。任何 vendor 类型请求
// 无论 recipient 是谁都会到达这里(usbd.c 先按 bmRequestType_bit.type 分发, 之后才
// 看 recipient), 因此本文件与外界共享的唯一命名空间就是 vendor_control.hpp 里的
// 请求码 -- DFU runtime 接口用 CLASS 请求, 不会冲突。
extern "C" bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    if (is_ms_os_20_request(request))
        return answer_ms_os_20(rhport, stage, request);
    return libhcs::firmware::usb::ep0::vendor_control_xfer<Registry>(
        g_board_context, rhport, stage, request);
}
