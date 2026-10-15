#include <atomic>
#include <chrono>
#include <cstddef>
#include <future>
#include <semaphore>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <libhcs/testing/fake_transport.hpp>

#include "host/src/session/data_sink.hpp"
#include "host/src/session/engine.hpp"

namespace {

using namespace std::chrono_literals;

class BlockingSink final : public libhcs::data::DataCallback {
public:
    bool can_receive_callback(
        libhcs::data::DataId, const libhcs::data::CanDataView&,
        const libhcs::time::SampleTime&) override {
        can_count.fetch_add(1, std::memory_order_relaxed);
        if (block_next.exchange(false, std::memory_order_acq_rel)) {
            entered.release();
            proceed.acquire();
        }
        return true;
    }

    bool uart_receive_callback(
        libhcs::data::DataId, const libhcs::data::UartDataView&,
        const libhcs::time::SampleTime&) override {
        return true;
    }
    bool gpio_digital_read_result_callback(
        std::uint8_t, const libhcs::data::GpioDigitalDataView&,
        const libhcs::time::SampleTime&) override {
        return true;
    }
    bool gpio_analog_read_result_callback(
        std::uint8_t, const libhcs::data::GpioAnalogDataView&,
        const libhcs::time::SampleTime&) override {
        return true;
    }
    void accelerometer_receive_callback(
        const libhcs::data::ImuAccelerometerDataView&, const libhcs::time::SampleTime&) override {}
    void gyroscope_receive_callback(
        const libhcs::data::ImuGyroscopeDataView&, const libhcs::time::SampleTime&) override {}
    void temperature_receive_callback(
        const libhcs::data::ImuTemperatureDataView&, const libhcs::time::SampleTime&) override {}

    std::atomic<unsigned> can_count{0};
    std::atomic<bool> block_next{true};
    std::binary_semaphore entered{0};
    std::binary_semaphore proceed{0};
};

TEST(EngineLifecycle, StopQuiescesAnAdmittedReceiveBeforeRestart) {
    libhcs::FakeTransport transport;
    BlockingSink sink;
    libhcs::host::session::Engine engine{
        transport, sink, libhcs::host::session::Engine::ThreadConfig{}};
    engine.start();
    // io_loop starts with one immediate keepalive; let it finish so stop() is not
    // incidentally blocked on FakeTransport's mutex by that unrelated transmit.
    std::this_thread::sleep_for(50ms);

    const std::vector payload{std::byte{1}};
    std::thread injector{
        [&] { transport.inject_can_frame(libhcs::data::DataId::kCan1, 0x123, payload); }};
    const bool entered = sink.entered.try_acquire_for(1s);
    EXPECT_TRUE(entered);
    if (!entered) {
        // Keep the failure path well-formed: a late callback must not leave a
        // joinable thread behind and turn the useful assertion into terminate().
        sink.proceed.release();
        injector.join();
        engine.stop();
        return;
    }

    auto stopping = std::async(std::launch::async, [&] { engine.stop(); });
    EXPECT_EQ(stopping.wait_for(30ms), std::future_status::timeout);
    sink.proceed.release();
    EXPECT_EQ(stopping.wait_for(1s), std::future_status::ready);
    injector.join();
    EXPECT_EQ(sink.can_count.load(std::memory_order_relaxed), 1U);

    // The Transport stays alive across Engine stop/start. Records delivered while
    // stopped must not be retained and relabelled as belonging to the next session.
    transport.inject_can_frame(libhcs::data::DataId::kCan1, 0x124, payload);
    EXPECT_EQ(sink.can_count.load(std::memory_order_relaxed), 1U);

    engine.start();
    transport.inject_can_frame(libhcs::data::DataId::kCan1, 0x125, payload);
    EXPECT_EQ(sink.can_count.load(std::memory_order_relaxed), 2U);
    engine.stop();
}

} // namespace
