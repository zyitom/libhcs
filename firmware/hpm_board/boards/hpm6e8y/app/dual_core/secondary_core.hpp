#pragma once

#include <cstdint>

namespace libhcs::firmware::ecat {
struct XcoreChannel;
}

namespace libhcs::firmware::xcore {

// EtherCAT 归档后的残留: 双核(core-swap)布局的 core0 侧装载与跨核通道接口。
// 默认单核构建下整个文件编译为空, 下方函数由文件尾的内联 no-op 代替。

#if defined(libhcs_APP_RELEASE_CORE1) && libhcs_APP_RELEASE_CORE1

// 需关中断调用: board_init() 之后、release_core1() 之前。
void publish_channel();

// 已发布通道的访问器(数据面用); publish_channel() 之前为 nullptr。
ecat::XcoreChannel* channel();

// 须开中断调用, 且在 USB 与 CAN/UART 驱动起来之前(但晚于 flash RPC 服务器
// 挂接), 顺序依据见实现; 配套 wait_for_core1_eeprom()。
void release_core1();

// 阻塞等待 core1 报告其启动期 EEPROM 工作完成, 超时返回 false 且启动继续;
// 单核构建恒为 no-op 返回 true。
bool wait_for_core1_eeprom(std::uint32_t timeout_ms);

// 把 core1 的诊断环排空到 core0 控制台: core1 禁止 printf, 这是它唯一的
// 日志通路, 须由主循环泵送。
void poll_diagnostics();

// up 环刚有批量上行后敲 core1 门铃, 让它立即重发 ESC 输入镜像; 仅在成功
// push 之后调用(单字邮箱, 未决的敲门会被丢弃)。
void ring_uplink_doorbell();

#else

inline void publish_channel() {}
inline void release_core1() {}
inline bool wait_for_core1_eeprom(std::uint32_t) { return true; }
inline void poll_diagnostics() {}
inline void ring_uplink_doorbell() {}
inline ecat::XcoreChannel* channel() { return nullptr; }

#endif

} // namespace libhcs::firmware::xcore
