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

struct GyroscopeTraits {
    static constexpr size_t kDummyBytes = 1;

    enum class RegisterAddress : uint8_t {
        kGyroSelfTest = 0x3C,
        kInt3Int4IoMap = 0x18,
        kInt3Int4IoConf = 0x16,
        kGyroIntCtrl = 0x15,
        kGyroSoftReset = 0x14,
        kGyroLpm1 = 0x11,
        kGyroBandwidth = 0x10,
        kGyroRange = 0x0F,
        kGyroIntStat1 = 0x0A,
        kRateZMsb = 0x07,
        kRateZLsb = 0x06,
        kRateYMsb = 0x05,
        kRateYLsb = 0x04,
        kRateXMsb = 0x03,
        kRateXLsb = 0x02,
        kGyroChipId = 0x00,
    };
};

class Gyroscope final
    : public GyroscopeTraits
    , private Bmi088Base<GyroscopeTraits> {
public:
    using Lazy = utility::Lazy<Gyroscope, Spi::Lazy*>;

    enum class Range : uint8_t {
        k2000 = 0x00,
        k1000 = 0x01,
        k500 = 0x02,
        k250 = 0x03,
        k125 = 0x04,
    };
    enum class DataRateAndBandwidth : uint8_t {
        k2000And532 = 0x00,
        k2000And230 = 0x01,
        k1000And116 = 0x02,
        k400And47 = 0x03,
        k200And23 = 0x04,
        k100And12 = 0x05,
        k200And64 = 0x06,
        k100And32 = 0x07,
    };

    // 构造不碰芯片: 传感器只在主机声明了板载 IMU 之后才初始化(configure(), 由
    // service.hpp 的 start() 调用)。上电后的陀螺仪没开新数据中断, 不出数据就绪脉冲。
    explicit Gyroscope(Spi::Lazy* spi)
        : Bmi088Base(spi, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin) {}

    // 复位芯片、按给定量程与输出速率/带宽配置, 每个寄存器写后回读。阻塞约 35 ms
    // (复位后要等 30 ms), 只在主循环调用(EP0 处理器)。任何一步对不上即返回 false。
    [[nodiscard]] bool configure(Range range, DataRateAndBandwidth rate) {
        using namespace std::chrono_literals;

        if (!lock_bus())
            return false;

        // 复位所有寄存器。
        write_register(RegisterAddress::kGyroSoftReset, 0xB6);
        timer::timer->spin_wait(30ms);

        const bool configured =
            // "Who am I" 芯片 ID 校验。
            read_and_confirm(RegisterAddress::kGyroChipId, 0x0F)
            // 使能新数据中断。
            && write_and_confirm(RegisterAddress::kGyroIntCtrl, 0x80)
            // INT3/INT4 均配为推挽, 低有效。
            && write_and_confirm(RegisterAddress::kInt3Int4IoConf, 0b0000)
            // 数据就绪中断映射到 INT3。
            && write_and_confirm(RegisterAddress::kInt3Int4IoMap, 0x01)
            // 设置 ODR 与滤波带宽。
            && write_and_confirm(RegisterAddress::kGyroBandwidth, 0x80 | static_cast<uint8_t>(rate))
            // 设置量程。
            && write_and_confirm(RegisterAddress::kGyroRange, static_cast<uint8_t>(range))
            // 切到 normal 模式。
            && write_and_confirm(RegisterAddress::kGyroLpm1, 0x00);

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
        if (!read_async(RegisterAddress::kRateXLsb, 6))
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
        const auto result = serializer.write_imu_gyroscope({
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

inline constinit Gyroscope::Lazy gyroscope(&spi1);

} // namespace libhcs::firmware::spi::bmi088
