#ifndef ROVER_LIB2_COMMUNICATION_SPI_SPI_DEVICE_HPP
#define ROVER_LIB2_COMMUNICATION_SPI_SPI_DEVICE_HPP

#include "rover_lib2/communication/SPI/SPI_bus.hpp"

#include <array>
#include <cstring>
#include <esp32-hal-spi.h>

DEFINE_LOG_NODE(SPIDevice, Logger::eNodeState::OFF);

class SPIDeviceT
{
  public:
    enum class eSPIMode : uint8_t
    {
        MODE_0 = SPI_MODE0,
        MODE_1 = SPI_MODE1,
        MODE_2 = SPI_MODE2,
        MODE_3 = SPI_MODE3,
    };

    enum class eReturnCode : uint8_t
    {
        TRANSMISSION_DONE_SUCCESS,
        TRANSMISSION_DONE_FAILED,
        TRANSMISSION_IN_PROGRESS,
        INVALID_STATE
    };
};

template<size_t MAX_MSG_LENGTH = 4>
class SPIDevice : public SPIDeviceT
{
    static constexpr uint8_t SPI_QUEUE_SIZE = 1UL;

    enum class eState : uint8_t
    {
        INVALID,
        READY_TO_WRITE,
        WAITING_ON_READ,
    };

  public:
    SPIDevice(SPIBus& bus_,
              gpio_num_t pinNCS,
              uint32_t clockSpeedHz_ = 1'000'000,
              uint16_t timeUSBeforeFirstBit_ = 0UL,
              uint16_t timeUSAfterLastBit_ = 0UL,
              eSPIMode mode_ = eSPIMode::MODE_1,
              spi_clock_source_t spiClockSource_ = spi_clock_source_t::SPI_CLK_SRC_DEFAULT):
        _currentState(eState::INVALID)
    {
        _config.command_bits = 0;
        _config.address_bits = 0;
        _config.dummy_bits = 0;
        _config.mode = TO_UNDERLYING(mode_);
        _config.clock_source = spiClockSource_;
        _config.duty_cycle_pos = 0;
        _config.cs_ena_pretrans = timeUSBeforeFirstBit_ * (clockSpeedHz_ / 1'000'000UL);
        _config.cs_ena_posttrans = timeUSAfterLastBit_ * (clockSpeedHz_ / 1'000'000UL);
        _config.clock_speed_hz = clockSpeedHz_;
        _config.spics_io_num = pinNCS;
        _config.flags = 0;
        _config.queue_size = SPI_QUEUE_SIZE;
        _config.pre_cb = nullptr;
        _config.post_cb = nullptr;

        if (bus_.registerSPIDevice(&_config, _deviceHandle))
        {
            _currentState = eState::READY_TO_WRITE;
        }
    }

    template<size_t DATA_LENGTH>
    bool writeData(const std::array<uint8_t, DATA_LENGTH>& data_)
    {
        static_assert(DATA_LENGTH <= MAX_MSG_LENGTH, "Trying to send too much SPI data for chosen MAX_MSG_LENGTH");

        if (_currentState != eState::READY_TO_WRITE)
        {
            return false;
        }

        std::memcpy(_txBuff.data(), data_.data(), data_.size());

        _ongoingTransaction.flags = 0;
        _ongoingTransaction.cmd = 0;
        _ongoingTransaction.addr = 0;
        _ongoingTransaction.length = data_.size() * 8U;
        _ongoingTransaction.rxlength = data_.size() * 8U;
        _ongoingTransaction.tx_buffer = _txBuff.data();
        _ongoingTransaction.rx_buffer = _rxBuff.data();
        _ongoingTransaction.user = NULL;

        esp_err_t statusCode = spi_device_queue_trans(_deviceHandle, &_ongoingTransaction, 0U);
        switch (statusCode)
        {
            case (ESP_OK):
                LOG_DEBUG(Logger::Nodes::SPIDevice, "Successfully sent SPI transaction in queue");
                _currentState = eState::WAITING_ON_READ;
                return true;
            case (ESP_ERR_INVALID_STATE):
                LOG_WARN(Logger::Nodes::SPIDevice, "Another transaction already in progress, can't add transaction to queue");
                return false;
            case (ESP_ERR_INVALID_ARG):
                LOG_FATAL(Logger::Nodes::SPIDevice, "Invalid call to spi_device_queue_trans, can't add transaction to queue");
                _currentState = eState::INVALID;
                ASSERT();
                return false;
            case (ESP_ERR_TIMEOUT):
                LOG_WARN(Logger::Nodes::SPIDevice, "Couldn't add transaction to queue, timeout");
                return false;
            case (ESP_ERR_NO_MEM):
                LOG_FATAL(Logger::Nodes::SPIDevice,
                          "No more memory to allocate DMA temporary buffer, can't add transaction to queue. Shouldn't happen, "
                          "implementation error");
                _currentState = eState::INVALID;
                ASSERT();
                return false;
            default:
                ASSERT_MSG_ARGS("Should never fall here, retval: %d", statusCode);
                return false;
        }
    }

    template<size_t DATA_LENGTH>
    eReturnCode readData(std::array<uint8_t, DATA_LENGTH>& rRecvData_)
    {
        static_assert(DATA_LENGTH <= MAX_MSG_LENGTH, "Trying to recv too much SPI data for chosen MAX_MSG_LENGTH");

        if (_currentState != eState::WAITING_ON_READ)
        {
            return eReturnCode::INVALID_STATE;
        }

        spi_transaction_t* ptransactions;
        esp_err_t statusCode = spi_device_get_trans_result(_deviceHandle, &ptransactions, 0U);
        switch (statusCode)
        {
            case ESP_OK:
                break;
            case ESP_ERR_TIMEOUT:
                LOG_DEBUG(Logger::Nodes::SPIDevice, "Timeout when retrieving data");
                return eReturnCode::TRANSMISSION_IN_PROGRESS;
            default:
                LOG_FATAL(Logger::Nodes::SPIDevice, "Error in spi_device_get_trans_result, implementation error");
                ASSERT();
                _currentState = eState::INVALID;
                return eReturnCode::TRANSMISSION_DONE_FAILED;
        }

        if (!ptransactions)
        {
            LOG_WARN(Logger::Nodes::SPIDevice, "Error in transaction, no data available...");
            return eReturnCode::TRANSMISSION_DONE_FAILED;
        }

        if ((ptransactions->length / 8UL) != DATA_LENGTH)
        {
            LOG_WARN(Logger::Nodes::SPIDevice,
                     "Mismatch in data received length. Exepected %lu received %lu. Dropping transaction",
                     DATA_LENGTH,
                     ptransactions->length / 8UL);
            _currentState = eState::READY_TO_WRITE;
            return eReturnCode::TRANSMISSION_DONE_FAILED;
        }

        size_t lengthToCopy = DATA_LENGTH < ptransactions->rxlength ? DATA_LENGTH : ptransactions->rxlength;
        std::memcpy(rRecvData_.data(), ptransactions->rx_buffer, lengthToCopy);

        _currentState = eState::READY_TO_WRITE;
        LOG_DEBUG(Logger::Nodes::SPIDevice, "Successfully recieved SPI transaction data");
        return eReturnCode::TRANSMISSION_DONE_SUCCESS;
    }

  private:
    spi_device_interface_config_t _config;
    spi_device_handle_t _deviceHandle;

    eState _currentState;

    spi_transaction_t _ongoingTransaction = {};
    std::array<uint8_t, MAX_MSG_LENGTH> _txBuff = {};
    std::array<uint8_t, MAX_MSG_LENGTH> _rxBuff = {};
};

#endif  // ROVER_LIB2_COMMUNICATION_SPI_SPI_DEVICE_HPP
