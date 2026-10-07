#include "core/include/libhcs/protocol/vendor_control.hpp"

#include <cstdint>

#include <common/tusb_types.h>

#include "firmware/common/app/src/usb/ep0_vendor_control.hpp"
#include "firmware/mc02/app/src/ports.hpp"
#include "firmware/mc02/app/src/sync/sof.hpp"
#include "firmware/mc02/app/src/usb/vendor.hpp"

// EP0 配置通道 -- core/include/libhcs/protocol/vendor_control.hpp 的 mc02 半边。
//
// 请求分发、清单的两阶段提交、按口类型的设置校验与应用都在核心(core/src/link/), TinyUSB
// 胶水在 firmware/common 的 ep0_vendor_control.hpp, 三块板共用; 本文件只是它对这块板
// 的实例化: 注册表来自 ports.hpp 的绑定(spec 口表 -> 驱动), 板级上下文在下面。
//
// 与 hpm_board 的差异只有策略, 不再有第二份协议代码: 本板没有附加功能(DMTool/CDC),
// 没有归属可交接, 只有一把握手门 -- 清单被完整应用时 set_ep0_handshake_done(true), 在
// 那之前 kStart 一律被静默拒绝。上电全停是各驱动的构造状态: 声明的口由清单应用启动,
// 未声明的口由清单应用保持停止; 应用中途失败时核心把所有口挂起, 门保持原样。
//
// 执行上下文: TinyUSB 在 USB 中断里排队 setup 包, 在 tud_task() 中解码, 因此下面
// 全部代码运行在主循环上, 与 bulk 下行回调同线程、同趟主循环的同一位置。这对 UART
// 路径很关键: 应用波特率会在 TX DMA 运行中改写 BRR, 切换窗口内的 RX 字节可能被打乱
// (uart/uart.hpp)。若从中断上下文执行, 该窗口会扩大到 ISR 抢占的一切范围。

namespace {

// 板级上下文: 清单生效即放行会话。其余(锁存、暂存、清单结果、无交接)是公共半边。
class BoardContext : public libhcs::firmware::usb::ep0::ContextBase {
public:
    static void on_manifest_accepted(uint64_t /*now*/) {
        libhcs::firmware::usb::vendor->set_ep0_handshake_done(true);
    }

    // 共享时间基准: 每个镜像都带, 清单要了才开(core 在端口应用之前调)。
    static constexpr uint8_t kBoardCaps = libhcs::core::protocol::vendor_control::kBoardCapTimeSync;
    [[nodiscard]] static bool set_time_sync(bool on) {
        if (on)
            return libhcs::firmware::sync::time_sync_start();
        libhcs::firmware::sync::time_sync_stop();
        return true;
    }
};

BoardContext g_board_context;

} // namespace

// 覆盖 TinyUSB 的弱定义(后者 STALL 所有 vendor 请求)。任何 vendor 类型的请求
// 无论 recipient 为何都会到达此处(usbd.c 先按 bmRequestType_bit.type 分发, 之后
// 才看 recipient), 因此请求码共享的命名空间只有 vendor_control.hpp 这一处
// -- DFU runtime 接口使用 CLASS 请求, 不会相撞。
extern "C" bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    return libhcs::firmware::usb::ep0::vendor_control_xfer<libhcs::firmware::ports::Registry>(
        g_board_context, rhport, stage, request);
}
