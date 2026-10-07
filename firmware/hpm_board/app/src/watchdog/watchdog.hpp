#pragma once

#include <cstdint>

#include <hpm_clock_drv.h>
#include <hpm_ewdg_drv.h>
#include <hpm_soc.h>

#include "core/src/utility/assert.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"

namespace libhcs::firmware::watchdog {

// EWDG 硬件看门狗：主循环 500ms 不喂狗即复位整板。覆盖一切死法——USB IRQ 自旋、
// 主循环挂死、任何未知的死循环——恢复靠硬件复位，不依赖软件路径还能跑。
// 32.768kHz 外部晶振计数；调试器 halt 时暂停，不打断 J-Link 会话。
// 默认不编译：看门狗把挂死变成"悄悄重启"，调试构建要能看到现场（libhcs_WATCHDOG，
// 默认 OFF；比赛镜像建议 ON）。移植自 librmcs firmware/rmcs_board 的 watchdog。
#if defined(libhcs_APP_WATCHDOG) && libhcs_APP_WATCHDOG

class Watchdog {
public:
    using Lazy = utility::Lazy<Watchdog>;

    Watchdog() {
        clock_add_to_group(clock_watchdog0, 0);

        ewdg_config_t config;
        ewdg_get_default_config(HPM_EWDG0, &config);

        config.enable_watchdog = true;
        config.int_rst_config.enable_timeout_reset = true;
        config.ctrl_config.use_lowlevel_timeout = false;
        config.ctrl_config.cnt_clk_sel = ewdg_cnt_clk_src_ext_osc_clk;
        config.ctrl_config.keep_running_in_debug_mode = false;
        config.ctrl_config.timeout_reset_us = kTimeoutUs;
        config.cnt_src_freq = kClockFrequencyHz;

        libhcs::core::utility::assert_always(ewdg_init(HPM_EWDG0, &config) == status_success);
    }

    static void feed() {
        libhcs::core::utility::assert_always(ewdg_refresh(HPM_EWDG0) == status_success);
    }

private:
    // 主循环周期 0.72~0.85us，500ms 是六个数量级的余量：只拦"真死"，不拦慢。
    static constexpr uint32_t kTimeoutUs = 500'000U;
    static constexpr uint32_t kClockFrequencyHz = 32'768U;
};

inline constinit Watchdog::Lazy watchdog;

#else

class Watchdog {
public:
    using Lazy = utility::Lazy<Watchdog>;

    Watchdog() = default;
    static void feed() {}
};

inline constinit Watchdog::Lazy watchdog;

#endif

} // namespace libhcs::firmware::watchdog
