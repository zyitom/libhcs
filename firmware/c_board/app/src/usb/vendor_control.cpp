#include "core/include/libhcs/protocol/vendor_control.hpp"

#include <cstdint>

#include <common/tusb_types.h>

#include "firmware/c_board/app/src/ports.hpp"
#include "firmware/c_board/app/src/sync/sof.hpp"
#include "firmware/c_board/app/src/usb/vendor.hpp"
#include "firmware/common/app/src/usb/ep0_vendor_control.hpp"

// EP0 配置通道 -- libhcs/protocol/vendor_control.hpp 的 c_board 半边。
//
// 请求分发、清单的两阶段提交、按口类型的设置校验与应用都在核心(core/src/link/), TinyUSB
// 胶水在 firmware/common 的 ep0_vendor_control.hpp, 三块板共用; 本文件只是它对这块板
// 的实例化: 注册表来自 ports.hpp 的绑定(spec 口表 -> 驱动), 板级上下文在下面。
//
// 本板的口常开(未改造): 清单里的设置项全是核对(bxCAN 的位时序钉死、IMU 上电即采样),
// "未声明的口不上线"在本板退化成"本板没这句话"。没有附加功能可交接, 只有一把握手门:
// 清单被完整应用时 set_ep0_handshake_done(true), 在那之前 kStart 一律被静默拒绝 --
// 旧主机会跳过握手直接开会话, 这道门让它开不起来。
//
// 执行上下文: TinyUSB 在 USB 中断里排队 setup 包, 在 tud_task() 中解码, 因此下面
// 全部代码运行在主循环上, 与 bulk 下行回调同线程、同趟主循环的同一位置。

namespace {

// 板级上下文: 清单生效即放行会话。其余(锁存、暂存、清单结果、无交接)是公共半边。
class BoardContext : public libhcs::firmware::usb::ep0::ContextBase {
public:
    static void on_manifest_accepted(uint64_t /*now*/) {
        libhcs::firmware::usb::vendor->set_ep0_handshake_done(true);
    }

    // 共享时间基准: 每个镜像都带, 清单要了才开(core 在端口应用之前调)。mc02 同款,
    // 只是本板没有硬件捕获要接, time_sync_start() 里没有 capture_init()。
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
// 无论 recipient 为何都会到达此处, 因此请求码共享的命名空间只有 vendor_control.hpp
// 这一处 -- DFU runtime 接口使用 CLASS 请求, 不会相撞。
extern "C" bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    return libhcs::firmware::usb::ep0::vendor_control_xfer<libhcs::firmware::ports::Registry>(
        g_board_context, rhport, stage, request);
}
