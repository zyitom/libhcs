#pragma once

#include <atomic>
#include <chrono> // IWYU pragma: keep (https://github.com/llvm/llvm-project/issues/68213)
#include <cstddef>
#include <cstdint>

#include <main.h>

#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/common/app/src/utility/interrupt_lock.hpp"
#include "firmware/common/app/src/utility/lazy.hpp"
#include "firmware/mc02/app/src/spi/bmi088/base.hpp"
#include "firmware/mc02/app/src/spi/spi.hpp"
#include "firmware/mc02/app/src/timer/timer.hpp"
#include "firmware/mc02/app/src/usb/vendor.hpp"

namespace libhcs::firmware::spi::bmi088 {

struct AccelerometerTraits {
    static constexpr size_t kDummyBytes = 2;

    enum class RegisterAddress : uint8_t {
        kAccSoftReset = 0x7E,
        kAccPwrCtrl = 0x7D,
        kAccPwrConf = 0x7C,
        kAccSelfTest = 0x6D,
        kIntMapData = 0x58,
        kInt2IoCtrl = 0x54,
        kInt1IoCtrl = 0x53,
        kAccRange = 0x41,
        kAccConf = 0x40,
        kTempLsb = 0x23,
        kTempMsb = 0x22,
        kAccIntStat1 = 0x1D,
        kSensorTime2 = 0x1A,
        kSensorTime1 = 0x19,
        kSensorTime0 = 0x18,
        kAccZMsb = 0x17,
        kAccZLsb = 0x16,
        kAccYMsb = 0x15,
        kAccYLsb = 0x14,
        kAccXMsb = 0x13,
        kAccXLsb = 0x12,
        kAccStatus = 0x03,
        kAccErrReg = 0x02,
        kAccChipId = 0x00,
    };
};

class Accelerometer final
    : public AccelerometerTraits
    , private Bmi088Base<AccelerometerTraits> {
public:
    using Lazy = utility::Lazy<Accelerometer, Spi::Lazy*>;

    enum class Range : uint8_t { k3G = 0x00, k6G = 0x01, k12G = 0x02, k24G = 0x03 };
    enum class DataRate : uint8_t {
        k12Hz = 0x05,
        k25Hz = 0x06,
        k50Hz = 0x07,
        k100Hz = 0x08,
        k200Hz = 0x09,
        k400Hz = 0x0A,
        k800Hz = 0x0B,
        k1600Hz = 0x0C,
    };

    // 构造不碰芯片: 传感器只在主机声明了板载 IMU 之后才初始化(configure(), 由
    // service.hpp 的 start() 调用)。上电后的加速度计停在 suspend, 不出数据就绪脉冲。
    explicit Accelerometer(Spi::Lazy* spi)
        : Bmi088Base(spi, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin) {}

    // 复位芯片、按给定量程与输出速率配置并使能, 每个寄存器写后回读。阻塞约 5 ms,
    // 只在主循环调用(EP0 处理器)。任何一步对不上即返回 false -- 芯片不在、SPI 不通
    // 都落在这里, 由调用方回报给主机, 而不是停机。
    [[nodiscard]] bool configure(Range range, DataRate data_rate) {
        using namespace std::chrono_literals;

        if (!lock_bus())
            return false;

        // 哑读一次, 让加速度计进入 SPI 模式。
        read_register(RegisterAddress::kAccChipId);
        timer::timer->spin_wait(1ms);

        // 复位所有寄存器。
        write_register(RegisterAddress::kAccSoftReset, 0xB6);
        timer::timer->spin_wait(1ms);

        const bool configured =
            // "Who am I" 芯片 ID 校验。
            read_and_confirm(RegisterAddress::kAccChipId, 0x1E)
            // INT1 配为输出, 推挽, 低有效。
            && write_and_confirm(RegisterAddress::kInt1IoCtrl, 0b00001000)
            // 数据就绪中断映射到 INT1。
            && write_and_confirm(RegisterAddress::kIntMapData, 0b00000100)
            // 设置 ODR 与 OSR。
            && write_and_confirm(
                RegisterAddress::kAccConf, 0x80 | (0x02 << 4) | static_cast<uint8_t>(data_rate))
            // 设置量程。
            && write_and_confirm(RegisterAddress::kAccRange, static_cast<uint8_t>(range))
            // 切到 active 模式。
            && write_and_confirm(RegisterAddress::kAccPwrConf, 0x00)
            // 使能加速度计。
            && write_and_confirm(RegisterAddress::kAccPwrCtrl, 0x04);
        timer::timer->spin_wait(1ms);

        spi_.unlock();
        return configured;
    }

    // 丢掉还没服务的数据就绪记录(停止时调用)。
    void drop_pending() {
        const utility::InterruptLockGuard guard;
        has_pending_capture_timestamp_ = false;
    }

    void data_ready_callback(uint32_t capture_timestamp_quarter_us) {
        const utility::InterruptLockGuard guard;
        pending_capture_timestamp_quarter_us_ = capture_timestamp_quarter_us;
        has_pending_capture_timestamp_ = true;
    }

    bool service_pending_read() {
        // 先不关中断看一眼: 绝大多数趟主循环没有新样本, 不必为此关、开一次中断。
        // 看的瞬间中断刚好置位也无妨, 下一趟就服务到了。
        if (!has_pending_capture_timestamp_)
            return false;
        const utility::InterruptLockGuard guard;
        if (!has_pending_capture_timestamp_)
            return false;
        if (!read_async(RegisterAddress::kAccXLsb, 6))
            return false;

        active_capture_timestamp_quarter_us_.store(
            pending_capture_timestamp_quarter_us_, std::memory_order_relaxed);
        has_active_capture_timestamp_.store(true, std::memory_order_release);
        has_pending_capture_timestamp_ = false;
        return true;
    }

private:
    void transmit_receive_async_callback(size_t size) override {
        uint32_t active_capture_timestamp_quarter_us = 0;
        const bool has_active_capture_timestamp =
            has_active_capture_timestamp_.exchange(false, std::memory_order_acquire);
        if (has_active_capture_timestamp) [[likely]] {
            active_capture_timestamp_quarter_us =
                active_capture_timestamp_quarter_us_.load(std::memory_order_relaxed);
        }

        core::utility::assert_debug(!size || has_active_capture_timestamp);
        if (size && has_active_capture_timestamp) [[likely]] {
            auto& data = parse_rx_data(spi_.rx_buffer, size);
            handle_uplink(usb::vendor->serializer(), data, active_capture_timestamp_quarter_us);
        }
        spi_.unlock();
    }

    static void handle_uplink(
        core::protocol::Serializer& serializer, Data& data, uint32_t capture_timestamp_quarter_us) {
        const auto result = serializer.write_imu_accelerometer({
            .x = data.x,
            .y = data.y,
            .z = data.z,
            .timestamp_quarter_us = capture_timestamp_quarter_us,
        });
        core::utility::assert_debug(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    uint32_t pending_capture_timestamp_quarter_us_ = 0;
    std::atomic<uint32_t> active_capture_timestamp_quarter_us_{0};
    // 数据就绪中断置位、主循环清除; 两边改它时都关着中断。volatile: 主循环不关中断的
    // 那次预读必须每趟真的去读。
    volatile bool has_pending_capture_timestamp_ = false;
    std::atomic<bool> has_active_capture_timestamp_{false};
};

inline constinit Accelerometer::Lazy accelerometer(&spi1);

} // namespace libhcs::firmware::spi::bmi088
