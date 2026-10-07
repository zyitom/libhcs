#pragma once

// The application's uplink callback interface.
//
// Lives on the host side on purpose (core/include/libhcs/data/datas.hpp keeps
// the wire-side views; firmware constructs those in its CAN interrupt and
// never routes a callback). Every data callback carries a
// libhcs::time::SampleTime: the SDK's IO thread computed when the record was
// taken while decoding it, so an application answers "how old is this data"
// by subtracting, and "how long between two samples of one board" by
// subtracting the `board` field -- it never touches a clock, a lock or a time
// object of its own. See libhcs/time/sample_time.hpp for what each field is
// worth.

#include <cstdint>

#include <libhcs/data/datas.hpp>
#include <libhcs/time/sample_time.hpp>

namespace libhcs::data {

/**
 * @brief Interface for consuming deserialized uplink data.
 *
 * This interface is invoked after the protocol layer has already identified the payload type and
 * decoded its contents. For callback families that are further multiplexed by the port's
 * `DataId`, the callback returns `bool` to report whether that port is valid for the concrete
 * implementation.
 *
 * Return `true` when the port is recognized and the payload has been dispatched.
 * Return `false` when deserialization succeeded but the `DataId` is unexpected, so the caller can
 * propagate that routing error to upper layers.
 *
 * IMU callbacks return `void` because each payload type maps to a single callback and requires no
 * additional route validation.
 */
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

    // GPIO samples, keyed by the line of the board's GPIO port (DataId::kGpio).
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

    // Shared time base status, one per keepalive period on a link with time sync
    // enabled. Non-pure and defaulted: an application that only wants a
    // synchronized clock never has to see the individual reports -- the SDK
    // feeds the axis and the per-record SampleTime regardless of whether this
    // is overridden.
    virtual void time_status_callback(const TimeStatusView& data) { (void)data; }

    // One completed hardware pulse exchange. See PulseReportView.
    virtual void pulse_report_callback(const PulseReportView& data) { (void)data; }
};

} // namespace libhcs::data
