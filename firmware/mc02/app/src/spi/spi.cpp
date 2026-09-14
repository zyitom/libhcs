#include "firmware/mc02/app/src/spi/spi.hpp"

#include <main.h>
#include <spi.h>

#include "core/src/utility/assert.hpp"

namespace libhcs::firmware::spi {

// HAL_SPI_TransmitReceive_DMA 完成时触发的回调。
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hal_spi_handle) {
    if (hal_spi_handle == &hspi2) {
        spi1->it_transfer_complete_callback();
    }
}

// NOLINTNEXTLINE(readability-inconsistent-declaration-parameter-name)
extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hal_spi_handle) {
    if (hal_spi_handle == &hspi2) {
        spi1->transmit_receive_async_callback(false);
    }
}

} // namespace libhcs::firmware::spi
