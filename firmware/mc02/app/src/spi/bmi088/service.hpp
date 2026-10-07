#pragma once

#include <main.h>

#include "core/include/libhcs/data/datas.hpp"
#include "core/include/libhcs/protocol/vendor_control.hpp"
#include "core/src/link/port.hpp"
#include "firmware/mc02/app/src/spi/bmi088/accel.hpp"
#include "firmware/mc02/app/src/spi/bmi088/gyro.hpp"
#include "firmware/mc02/app/src/spi/bmi088/temperature.hpp"
#include "firmware/mc02/app/src/utility/loop_work.hpp"

namespace libhcs::firmware::spi::bmi088 {

namespace vc = libhcs::core::protocol::vendor_control;
namespace link = libhcs::core::link;

// ---- 板载 IMU 口: 启停、"在跑"状态、loop 位与轮询都在这一个对象里 ----
//
// 它是一路口, 与 CAN / UART 同一条规矩(usb/vendor_control.cpp): 清单里的 kImu 项
// 被接受就是声明, apply() 在那时才第一次去碰芯片; 会话结束时挂起、屏蔽数据就绪线。
// 没声明的 IMU 不产生中断、不占 SPI、在主循环里不存在(loop::active 的 kImu 位为 0)。
//
// 原先由编译开关 libhcs_APP_IMU_ENABLE 决定"这版镜像有没有 IMU", 已取消: 用不用是
// 主机的接线表说了算, 不是镜像的属性。
//
// 启停只在主循环被调到(EP0 处理器与会话状态机都经 tud_task() 到达); 轮询由 app.cpp
// 的主循环在 loop::active 的 kImu 位置着时调用。
class ImuPort {
public:
    // EP0 口能力: IMU 没有能力位。
    static constexpr uint8_t kPortCapabilities = 0;

    [[nodiscard]] bool running() const { return running_; }

    // 挂起: 屏蔽两条数据就绪线。传感器自己不去动它 -- 复位要阻塞三十多毫秒, 而会话
    // 结束的路径上不该有这种等待; 线被屏蔽后它出不出脉冲都不再打扰 CPU。
    // 启动代码在 MX_GPIO_Init() 之后也调一次: CubeMX 生成的初始化把这两条线开着,
    // 其余动作(清 loop 位、丢 pending)在上电状态上是空操作。
    void suspend() {
        CLEAR_BIT(EXTI->IMR1, kDataReadyLines);
        __HAL_GPIO_EXTI_CLEAR_IT(kDataReadyLines);
        loop::clear(loop::kImu);
        if (auto* accel = accelerometer.try_get())
            accel->drop_pending();
        if (auto* gyro = gyroscope.try_get())
            gyro->drop_pending();
        running_ = false;
    }

    // 本板没有"附加功能持有 IMU"的状态: IMU 只被清单声明启动, 不存在
    // "交还后要恢复采样"的情形。
    void resume() {}

    [[nodiscard]] link::PortOutcome validate(const vc::ImuConfigPayload& payload) const {
        Accelerometer::Range accel_range{};
        Accelerometer::DataRate accel_rate{};
        Gyroscope::Range gyro_range{};
        Gyroscope::DataRateAndBandwidth gyro_rate{};
        return decode(payload, accel_range, accel_rate, gyro_range, gyro_rate);
    }

    [[nodiscard]] link::PortOutcome apply(const vc::ImuConfigPayload& payload) {
        // 运行期改任意一个口或重连都会重放整份清单。设置没变、传感器在跑就不再 start():
        // 那是在 EP0 处理器(主循环)里阻塞约 40 ms, 期间 921600 的串口要进约 3.7 KB(环
        // 2 KB)、RS-485 环只有 256 B, 都会溢出。与 GPIO configure() 同一规矩。
        if (running_ && same_setting(payload, setting_))
            return link::PortOutcome::pass();
        Accelerometer::Range accel_range{};
        Accelerometer::DataRate accel_rate{};
        Gyroscope::Range gyro_range{};
        Gyroscope::DataRateAndBandwidth gyro_rate{};
        const auto outcome = decode(payload, accel_range, accel_rate, gyro_range, gyro_rate);
        if (!outcome.ok())
            return outcome;
        // 芯片没配上(不在、SPI 不通、寄存器回读不符)报 kConfigErrorVerifyFailed,
        // IMU 保持停止: 与"求解器拒绝"区分, 见 kConfigErrorVerifyFailed。
        if (!start(accel_range, accel_rate, gyro_range, gyro_rate))
            return link::PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorVerifyFailed);
        running_ = true;
        setting_ = payload;
        return link::PortOutcome::pass();
    }

    void read_config(vc::ImuConfigPayload& out) const { out = setting_; }

    [[nodiscard]] link::PortStatus describe() const { return {.running = running_, .fd = false}; }

    // 主循环入口, 只在 loop::active 的 kImu 位置着时调用: 每趟至多发起一次 BMI088 SPI
    // 读, 优先顺序为 gyro > accel > temperature。没有新样本的那些趟只是三次标志判断。
    void poll() {
        if (gyroscope->service_pending_read())
            return;
        if (accelerometer->service_pending_read())
            return;
        temperature->service_pending_read();
    }

    // 毫秒杂务, 同样只在 kImu 位置着时调用: 把到期的温度探针提升为 pending。探针 1 Hz
    // 一次, 到期判断要读定时器(D2 域外设访问), 每毫秒看一次足够, 不放在每趟主循环上。
    void poll_temperature_probe() { temperature->poll_pending_probe(); }

private:
    // 两条数据就绪线(PE10 加速度计、PE12 陀螺仪)在 EXTI 里的位。
    static constexpr uint32_t kDataReadyLines = INT1_ACC_Pin | INT1_GYRO_Pin;

    // 按给定设置初始化两颗芯片并开始采样。阻塞约 40 ms。返回 false 表示芯片没有按要求
    // 配上(不在、SPI 不通、寄存器回读不符), IMU 保持停止。
    [[nodiscard]] bool start(
        Accelerometer::Range accelerometer_range, Accelerometer::DataRate accelerometer_rate,
        Gyroscope::Range gyroscope_range, Gyroscope::DataRateAndBandwidth gyroscope_rate) {
        suspend();
        auto* accel = accelerometer.try_get();
        auto* gyro = gyroscope.try_get();
        auto* thermometer = temperature.try_get();
        if (accel == nullptr || gyro == nullptr || thermometer == nullptr)
            return false;
        if (!accel->configure(accelerometer_range, accelerometer_rate)
            || !gyro->configure(gyroscope_range, gyroscope_rate))
            return false;

        thermometer->restart();
        SET_BIT(EXTI->IMR1, kDataReadyLines);
        loop::set(loop::kImu);
        return true;
    }

    // 五个设置字段相等(保留字节不算设置)。
    [[nodiscard]] static constexpr bool
        same_setting(const vc::ImuConfigPayload& a, const vc::ImuConfigPayload& b) {
        return a.accelerometer_range_g == b.accelerometer_range_g
            && a.accelerometer_rate_hz == b.accelerometer_rate_hz
            && a.gyroscope_range_dps == b.gyroscope_range_dps
            && a.gyroscope_rate_hz == b.gyroscope_rate_hz
            && a.gyroscope_bandwidth_hz == b.gyroscope_bandwidth_hz;
    }

    // 请求里的物理量 -> 芯片档位。必须是芯片提供的档位, 查不到即拒绝并带上肇事值。
    // 没有"0 = 默认": 声明必须说全, 核心在调到这里之前已拒绝缺项(port_ops::imu_validate)。
    [[nodiscard]] static link::PortOutcome decode(
        const vc::ImuConfigPayload& payload, Accelerometer::Range& accel_range,
        Accelerometer::DataRate& accel_rate, Gyroscope::Range& gyro_range,
        Gyroscope::DataRateAndBandwidth& gyro_rate) {
        const auto refuse = [](uint32_t value) {
            return link::PortOutcome::refuse(
                vc::ConfigErrorReason::kConfigErrorRateUnrepresentable, value);
        };
        using Range = Accelerometer::Range;
        switch (payload.accelerometer_range_g) {
        case 3: accel_range = Range::k3G; break;
        case 6: accel_range = Range::k6G; break;
        case 12: accel_range = Range::k12G; break;
        case 24: accel_range = Range::k24G; break;
        default: return refuse(payload.accelerometer_range_g);
        }
        using AccelRate = Accelerometer::DataRate;
        switch (payload.accelerometer_rate_hz) {
        case 12: accel_rate = AccelRate::k12Hz; break; // 12.5 Hz
        case 25: accel_rate = AccelRate::k25Hz; break;
        case 50: accel_rate = AccelRate::k50Hz; break;
        case 100: accel_rate = AccelRate::k100Hz; break;
        case 200: accel_rate = AccelRate::k200Hz; break;
        case 400: accel_rate = AccelRate::k400Hz; break;
        case 800: accel_rate = AccelRate::k800Hz; break;
        case 1600: accel_rate = AccelRate::k1600Hz; break;
        default: return refuse(payload.accelerometer_rate_hz);
        }
        using GyroRange = Gyroscope::Range;
        switch (payload.gyroscope_range_dps) {
        case 2000: gyro_range = GyroRange::k2000; break;
        case 1000: gyro_range = GyroRange::k1000; break;
        case 500: gyro_range = GyroRange::k500; break;
        case 250: gyro_range = GyroRange::k250; break;
        case 125: gyro_range = GyroRange::k125; break;
        default: return refuse(payload.gyroscope_range_dps);
        }
        // 输出速率与滤波带宽是芯片里的同一个寄存器值, 成对给出, 必须是下表里的一对。
        using GyroRate = Gyroscope::DataRateAndBandwidth;
        struct Pair {
            uint16_t rate_hz, bandwidth_hz;
            GyroRate code;
        };
        static constexpr Pair kPairs[]{
            {2000, 532, GyroRate::k2000And532},
            {2000, 230, GyroRate::k2000And230},
            {1000, 116, GyroRate::k1000And116},
            { 400,  47,   GyroRate::k400And47},
            { 200,  23,   GyroRate::k200And23},
            { 100,  12,   GyroRate::k100And12},
            { 200,  64,   GyroRate::k200And64},
            { 100,  32,   GyroRate::k100And32},
        };
        bool known = false;
        for (const Pair& pair : kPairs) {
            if (pair.rate_hz == payload.gyroscope_rate_hz
                && pair.bandwidth_hz == payload.gyroscope_bandwidth_hz) {
                gyro_rate = pair.code;
                known = true;
                break;
            }
        }
        if (!known)
            return refuse(payload.gyroscope_rate_hz);
        return link::PortOutcome::pass();
    }

    bool running_ = false;
    vc::ImuConfigPayload setting_{};
};

// 板载 IMU 的口对象: 一颗传感器, 一个绑定。
inline constinit ImuPort imu_port{};

} // namespace libhcs::firmware::spi::bmi088
