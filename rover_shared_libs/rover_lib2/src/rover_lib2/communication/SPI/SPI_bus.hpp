#ifndef ROVER_LIB2_COMMUNICATION_SPI_SPI_BUS_HPP
#define ROVER_LIB2_COMMUNICATION_SPI_SPI_BUS_HPP

#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/macros.hpp"

#include <driver/spi_master.h>
#include <driver/gpio.h>

DEFINE_LOG_NODE(SPIBus, Logger::eNodeState::OFF)

class SPIBus
{
    static constexpr uint16_t MIN_SPI_TRANSFER = 32U;
    static constexpr uint16_t MAX_SPI_TRANSFER = 4'092U;

  public:
    SPIBus(const spi_host_device_t spiHost_,
           const gpio_num_t pinMosi_,
           const gpio_num_t pinMiso_,
           const gpio_num_t pinSclk_,
           const uint16_t maxTransferSize_ = 4'092):
        _spiHost(CONSTRAIN(spiHost_, static_cast<spi_host_device_t>(0), spi_host_device_t::SPI_HOST_MAX))
    {
        ASSERT_COND_MSG_ARGS(spiHost_ < spi_host_device_t::SPI_HOST_MAX,
                             "Passed SPI Host invalid, value range is [0; %u]",
                             spi_host_device_t::SPI_HOST_MAX - 1U);

        ASSERT_COND_MSG_ARGS(maxTransferSize_ <= MAX_SPI_TRANSFER,
                             "Unsupported transfer size specified, valid range is: [%u; %u]",
                             MIN_SPI_TRANSFER,
                             MAX_SPI_TRANSFER);

        spi_bus_config_t config = {.mosi_io_num = pinMosi_,
                                   .miso_io_num = pinMiso_,
                                   .sclk_io_num = pinSclk_,
                                   .quadwp_io_num = -1,
                                   .quadhd_io_num = -1,
                                   .data4_io_num = -1,
                                   .data5_io_num = -1,
                                   .data6_io_num = -1,
                                   .data7_io_num = -1,
                                   .max_transfer_sz = maxTransferSize_,
                                   .flags = SPICOMMON_BUSFLAG_MASTER,
                                   .isr_cpu_id = esp_intr_cpu_affinity_t::ESP_INTR_CPU_AFFINITY_AUTO,
                                   .intr_flags = 0};

        esp_err_t retval = spi_bus_initialize(_spiHost, &config, SPI_DMA_CH_AUTO);
        switch (retval)
        {
            case ESP_OK:
                LOG_DEBUG(Logger::Nodes::SPIBus, "SPI bus inited successfully");
                break;
            case ESP_ERR_INVALID_ARG:
                ASSERT_MSG_ARGS("Invalid arguments in call to spi_bus_initialize(%d, 0x%p, %d)... Implementation error",
                                _spiHost,
                                &config,
                                SPI_DMA_CH_AUTO);
                break;
            case ESP_ERR_INVALID_STATE:
                ASSERT_MSG_ARGS("Trying to create SPI bus on SPI Host: %d, but host already in use... Implementation error",
                                _spiHost);
                break;
            case ESP_ERR_NOT_FOUND:
                ASSERT_MSG("No more DMA channel for SPI communication... Implementation error");
                break;
            case ESP_ERR_NO_MEM:
                ASSERT_MSG("No more memory to create SPI bus... Implementation error");
                break;
            default:
                ASSERT_MSG("Shouldn't fall here... Implementation error ");
                break;
        }
    }

    bool registerSPIDevice(const spi_device_interface_config_t* devConfig_, spi_device_handle_t& deviceHandle_)
    {
        if (!devConfig_)
        {
            return false;
        }

        esp_err_t retval = spi_bus_add_device(_spiHost, devConfig_, &deviceHandle_);
        switch (retval)
        {
            case ESP_OK:
                LOG_DEBUG(Logger::Nodes::SPIBus, "SPI device registered successfully");
                return true;
            case ESP_ERR_INVALID_ARG:
                ASSERT_MSG("Invalid arguments or configuration in call to spi_bus_add_device... Implementation error");
                return false;
            case ESP_ERR_INVALID_STATE:
                ASSERT_MSG_ARGS("Error on SPI bus or configured clock source unavailable... Implementation error");
                return false;
            case ESP_ERR_NOT_FOUND:
                ASSERT_MSG("No more CS channels for SPI communication... Implementation error, trying to register to many device "
                           "on same bus");
                return false;
            case ESP_ERR_NO_MEM:
                ASSERT_MSG_ARGS("No more memory to register SPI device on bus... Implementation error ");
                return false;
            default:
                ASSERT_MSG_ARGS("Should never fall here, retval: %d", retval);
                return false;
        }
    }

  private:
    spi_host_device_t _spiHost;
};

#endif  // ROVER_LIB2_COMMUNICATION_SPI_SPI_BUS_HPP
