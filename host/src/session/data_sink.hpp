#pragma once

#include <cstdint>

#include <libhcs/data/datas.hpp>
#include <libhcs/time/sample_time.hpp>

// v3 的推送式 DataCallback 从公开头文件里删了（v4 是拉取式通道），但链接引擎
// 内部仍用这个形状接收反序列化出来的上行记录：Board::Impl 实现它，把每条记录
// 写进对应通道的接收环。内容原样从 include/libhcs/data/callback.hpp 搬来。

namespace libhcs::data {

class DataCallback {
public:
    DataCallback() = default;
    DataCallback(const DataCallback&) = delete;
    DataCallback& operator=(const DataCallback&) = delete;
    DataCallback(DataCallback&&) = delete;
    DataCallback& operator=(DataCallback&&) = delete;
    virtual ~DataCallback() = default;

    [[nodiscard]] virtual bool can_receive_callback(
        DataId id, const CanDataView& data, const libhcs::time::SampleTime& time) = 0;

    [[nodiscard]] virtual bool uart_receive_callback(
        DataId id, const UartDataView& data, const libhcs::time::SampleTime& time) = 0;

    [[nodiscard]] virtual bool gpio_digital_read_result_callback(
        std::uint8_t line, const GpioDigitalDataView& data,
        const libhcs::time::SampleTime& time) = 0;
    [[nodiscard]] virtual bool gpio_analog_read_result_callback(
        std::uint8_t line, const GpioAnalogDataView& data,
        const libhcs::time::SampleTime& time) = 0;

    virtual void accelerometer_receive_callback(
        const ImuAccelerometerDataView& data, const libhcs::time::SampleTime& time) = 0;
    virtual void gyroscope_receive_callback(
        const ImuGyroscopeDataView& data, const libhcs::time::SampleTime& time) = 0;
    virtual void temperature_receive_callback(
        const ImuTemperatureDataView& data, const libhcs::time::SampleTime& time) = 0;

    virtual void time_status_callback(const TimeStatusView& data) { (void)data; }
    virtual void pulse_report_callback(const PulseReportView& data) { (void)data; }
};

} // namespace libhcs::data
