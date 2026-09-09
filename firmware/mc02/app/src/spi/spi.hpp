#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <main.h>
#include <spi.h>

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/mc02/app/src/utility/lazy.hpp"

namespace libhcs::firmware::spi {

class SpiModule : private core::utility::Immovable {
public:
    friend class Spi;

    SpiModule(GPIO_TypeDef* chip_select_port, uint16_t chip_select_pin)
        : chip_select_port_(chip_select_port)
        , chip_select_pin_(chip_select_pin) {}

    virtual ~SpiModule() = default;

protected:
    virtual void transmit_receive_async_callback(size_t size) = 0;

    GPIO_TypeDef* const chip_select_port_;
    const uint16_t chip_select_pin_;
};

class Spi : private core::utility::Immovable {
public:
    static constexpr size_t kMaxTransferSize = 32;

    using Lazy = utility::Lazy<Spi, SPI_HandleTypeDef*>;

    explicit Spi(SPI_HandleTypeDef* hal_spi_handle)
        : hal_spi_handle_(hal_spi_handle) {}

    bool is_locked() const { return locking_.test(std::memory_order::acquire); }

    bool try_lock() { return !locking_.test_and_set(std::memory_order::acquire); }

    void unlock() {
        core::utility::assert_debug_lazy([&]() noexcept { return is_locked(); });
        locking_.clear(std::memory_order::release);
    }

    void transmit_receive(SpiModule& module, size_t size) {
        core::utility::assert_debug(0 < size && size <= kMaxTransferSize);
        core::utility::assert_debug_lazy(
            [&]() noexcept { return is_locked() && hal_ready(); });

        begin_transfer(module, size);

        core::utility::assert_debug(
            HAL_SPI_TransmitReceive(
                hal_spi_handle_, tx_buffer, rx_buffer, tx_rx_size_, HAL_MAX_DELAY)
            == HAL_OK);

        finish_transfer();
    }

    // Uses DMA mode; completion is signaled through the SPI2 DMA stream IRQ.
    void transmit_receive_async(SpiModule& module, size_t size) {
        core::utility::assert_debug(0 < size && size <= kMaxTransferSize);
        core::utility::assert_debug_lazy(
            [&]() noexcept { return is_locked() && hal_ready(); });

        begin_transfer(module, size);

        core::utility::assert_debug(
            HAL_SPI_TransmitReceive_DMA(hal_spi_handle_, tx_buffer, rx_buffer, tx_rx_size_)
            == HAL_OK);
    }

    void it_transfer_complete_callback() { transmit_receive_async_callback(true); }

    void transmit_receive_async_callback(bool success) {
        if (!success) [[unlikely]]
            HAL_SPI_Abort(hal_spi_handle_);

        if (auto* module = finish_transfer())
            module->transmit_receive_async_callback(success ? tx_rx_size_ : 0);
    }

    // No-op: DMA completion is interrupt-driven, so no polling is needed.
    void update() {}

    alignas(4) uint8_t tx_buffer[kMaxTransferSize];
    alignas(4) uint8_t rx_buffer[kMaxTransferSize];

private:
    bool hal_ready() const { return hal_spi_handle_->State == HAL_SPI_STATE_READY; }

    void begin_transfer(SpiModule& module, size_t size) {
        spi_module_ = &module;
        tx_rx_size_ = static_cast<uint16_t>(size);
        select();
    }

    SpiModule* finish_transfer() {
        if (!spi_module_)
            return nullptr;
        deselect();
        auto* module = spi_module_;
        spi_module_ = nullptr;
        return module;
    }

    void select() {
        HAL_GPIO_WritePin(
            spi_module_->chip_select_port_, spi_module_->chip_select_pin_, GPIO_PIN_RESET);
    }

    void deselect() {
        HAL_GPIO_WritePin(
            spi_module_->chip_select_port_, spi_module_->chip_select_pin_, GPIO_PIN_SET);
    }

    SPI_HandleTypeDef* hal_spi_handle_;
    std::atomic_flag locking_;
    SpiModule* spi_module_{nullptr};
    uint16_t tx_rx_size_{0};
};

// BMI088 uses SPI2 on mc02 hardware.
inline constinit Spi::Lazy spi1(&hspi2);

} // namespace libhcs::firmware::spi
