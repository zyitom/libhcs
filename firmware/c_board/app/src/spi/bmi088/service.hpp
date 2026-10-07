#pragma once

#include <main.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/protocol/vendor_control.hpp"
#include "core/src/link/port.hpp"
#include "firmware/c_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/c_board/app/src/spi/bmi088/gyro.hpp"
#include "firmware/c_board/app/src/spi/bmi088/temperature.hpp"
#include "firmware/c_board/app/src/utility/loop_work.hpp"

namespace libhcs::firmware::spi::bmi088 {

namespace vc = libhcs::core::protocol::vendor_control;
namespace link = libhcs::core::link;

inline void service_pending_reads() {
    if (gyroscope->service_pending_read())
        return;
    if (accelerometer->service_pending_read())
        return;
    temperature->service_pending_read();
}

// ---- EP0 的 IMU 口驱动(spec/c_board/ports.hpp 清单里 DataId::kImu 的绑定) ----
//
// 本板的传感器上电即按编译期设置配好(没改造成"声明后才配置"), 清单里的 IMU 项因此
// 是**核对**而不是配置: 每个字段都必须正是本固件跑的那一档(6 g / 1600 Hz,
// 2000 deg/s / 2000 Hz 配 230 Hz 滤波, 与两个构造函数的默认参数一致); 声明必须说全,
// 缺项在核心里已被拒绝(port_ops::imu_validate)。挡位不符即拒绝。
//
// 没声明的 IMU 不工作, 与 mc02 同法: 两条数据就绪线(PC4 加速度计、PC5 陀螺仪, EXTI 4/5)
// 在 EXTI 里屏蔽, 温度不探测 -- 不进中断、不发起 SPI 读、不上报。上电与会话结束时挂起,
// 声明被接受(apply)才打开。传感器本身照常跑: 复位要阻塞几十毫秒, 不该放在会话结束的
// 路径上; 线被屏蔽后它出不出脉冲都不打扰 CPU。
class ImuPort {
public:
    // EP0 口能力: IMU 没有能力位。
    static constexpr uint8_t kPortCapabilities = 0;

    [[nodiscard]] bool running() const { return uplink_enabled.load(std::memory_order_relaxed); }

    // 启动代码在 MX_GPIO_Init() 之后也调一次: CubeMX 生成的初始化把这两条线开着。
    void suspend() {
        uplink_enabled.store(false, std::memory_order_relaxed);
        loop::clear(loop::kImu);
        CLEAR_BIT(EXTI->IMR, kDataReadyLines);
        __HAL_GPIO_EXTI_CLEAR_IT(kDataReadyLines);
        if (auto* accel = accelerometer.try_get())
            accel->drop_pending();
        if (auto* gyro = gyroscope.try_get())
            gyro->drop_pending();
        if (auto* thermometer = temperature.try_get())
            thermometer->drop_pending();
    }

    void resume() {
        if (running())
            return;
        __HAL_GPIO_EXTI_CLEAR_IT(kDataReadyLines);
        uplink_enabled.store(true, std::memory_order_relaxed);
        SET_BIT(EXTI->IMR, kDataReadyLines);
        loop::set(loop::kImu); // 主循环从下一圈起推 SPI 与温度探测(app.cpp)
    }

    [[nodiscard]] link::PortOutcome validate(const vc::ImuConfigPayload& payload) const {
        return check(payload);
    }

    // 被接受的声明就是上线(与 mc02 的 apply 一样: 核心不另调 resume)。
    [[nodiscard]] link::PortOutcome apply(const vc::ImuConfigPayload& payload) {
        const link::PortOutcome outcome = check(payload);
        if (outcome.ok())
            resume();
        return outcome;
    }

    // 声明里核对过的那组值, 供 kGetPortConfig 读回。
    void read_config(vc::ImuConfigPayload& out) const {
        out = {
            .accelerometer_rate_hz = 1600,
            .gyroscope_rate_hz = 2000,
            .gyroscope_bandwidth_hz = 230,
            .gyroscope_range_dps = 2000,
            .accelerometer_range_g = 6,
            .reserved = {},
        };
    }

    [[nodiscard]] link::PortStatus describe() const { return {.running = running(), .fd = false}; }

private:
    static constexpr uint32_t kDataReadyLines = INT1_ACC_Pin | INT1_GYRO_Pin;

    [[nodiscard]] static link::PortOutcome check(const vc::ImuConfigPayload& payload) {
        const auto refuse = [](uint32_t value) {
            return link::PortOutcome::refuse(
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, value);
        };
        if (payload.accelerometer_range_g != 6U)
            return refuse(payload.accelerometer_range_g);
        if (payload.accelerometer_rate_hz != 1600U)
            return refuse(payload.accelerometer_rate_hz);
        if (payload.gyroscope_range_dps != 2000U)
            return refuse(payload.gyroscope_range_dps);
        if (payload.gyroscope_rate_hz != 2000U || payload.gyroscope_bandwidth_hz != 230U)
            return refuse(payload.gyroscope_rate_hz);
        return link::PortOutcome::pass();
    }
};

// 板载 IMU 的口对象: 一颗传感器, 一个绑定。
inline constinit ImuPort imu_port{};

} // namespace libhcs::firmware::spi::bmi088
