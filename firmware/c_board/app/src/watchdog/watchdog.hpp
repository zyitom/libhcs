#pragma once

#include <cstdint>

#include <main.h>
#include <stm32f4xx_ll_iwdg.h>

#include "firmware/c_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware::watchdog {

// IWDG 硬件看门狗：主循环 500ms 不喂狗即复位整板。覆盖一切死法——USB IRQ 自旋、
// 主循环挂死、任何未知的死循环——恢复靠硬件复位，不依赖软件路径还能跑。
// LSI 32kHz / 64 分频 = 500Hz，重载 250 → 500ms；调试器 halt 时冻结，不打断
// J-Link 会话。IWDG 一旦启动无法停止（硬件设计），所以必须由开关控制是否编译。
// 默认不编译：看门狗把挂死变成"悄悄重启"，调试构建要能看到现场
// （libhcs_WATCHDOG，默认 OFF；比赛镜像建议 ON）。移植自 librmcs c_board。
#if defined(libhcs_APP_WATCHDOG) && libhcs_APP_WATCHDOG

class Watchdog {
public:
    using Lazy = utility::Lazy<Watchdog>;

    Watchdog() {
        // 调试器 halt 时冻结 IWDG（F4 在 APB1FZ）。
        DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

        LL_IWDG_Enable(IWDG);
        LL_IWDG_EnableWriteAccess(IWDG);
        LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_64);
        LL_IWDG_SetReloadCounter(IWDG, kReloadValue);
        while (LL_IWDG_IsReady(IWDG) == 0U) {}
        LL_IWDG_ReloadCounter(IWDG);
    }

    static void feed() { LL_IWDG_ReloadCounter(IWDG); }

private:
    // 32kHz LSI / 64 = 500Hz，重载 250 → 500ms。只拦"真死"，不拦慢。
    static constexpr uint32_t kReloadValue = 250U;
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
