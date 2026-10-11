#ifndef ROVER_LIB2_SENSORS_K30_HPP
#define ROVER_LIB2_SENSORS_K30_HPP

#include "rover_lib2/communication/I2C_utils.hpp"

class K30
{
    // Command high nibble (low nibble carries byte count for RAM/EEPROM ops)
    static constexpr uint8_t CMD_WRITE_RAM = 0x1;
    static constexpr uint8_t CMD_READ_RAM = 0x2;
    static constexpr uint8_t CMD_WRITE_EE = 0x3;
    static constexpr uint8_t CMD_READ_EE = 0x4;

    // RAM memory map
    static constexpr uint8_t RAM_ADDR_CO2 = 0x08;           // 2 bytes, MSB first
    static constexpr uint8_t RAM_ADDR_ERROR_STATUS = 0x1E;  // 1 byte

    // EEPROM memory map
    static constexpr uint8_t EE_ADDR_I2C_ADDRESS = 0x00;  // 1 byte

    static constexpr uint8_t DEFAULT_ADDRESS = 0x68;
    static constexpr uint32_t REQUEST_PROCESS_DELAY_MS = 20;  // tWAIT per SenseAir spec
    static constexpr uint8_t STATUS_COMPLETE_BIT = 0x01;
    static constexpr uint8_t MAX_PAYLOAD_BYTES = 16;

  public:
    explicit K30(TwoWire& wireInterface_, uint8_t address_ = DEFAULT_ADDRESS):
        _wireInterface(wireInterface_),
        _i2cAddress(address_)
    {
    }

    void init()
    {
        uint8_t dummy[2] = {0};
        _isInitialized = readMemory(CMD_READ_RAM, RAM_ADDR_CO2, 2, dummy);
    }

    /**
     * @brief Reads current CO2 concentration in ppm.
     * @return CO2 in ppm, or -1 on communication/checksum failure.
     */
    int32_t getCO2()
    {
        if (!_isInitialized)
        {
            return -1;
        }

        uint8_t data[2] = {0};
        if (!readMemory(CMD_READ_RAM, RAM_ADDR_CO2, 2, data))
        {
            return -1;
        }

        return (static_cast<int32_t>(data[0]) << 8) | data[1];
    }

    /**
     * @brief Reads the sensor's self-diagnostic error status byte.
     * @return Error bitfield (0 = no errors), or -1 on communication/checksum failure.
     */
    int16_t getErrorStatus()
    {
        if (!_isInitialized)
        {
            return -1;
        }

        uint8_t data[1] = {0};
        if (!readMemory(CMD_READ_RAM, RAM_ADDR_ERROR_STATUS, 1, data))
        {
            return -1;
        }

        return static_cast<int16_t>(data[0]);
    }

    /**
     * @brief Writes a new I2C address to EEPROM. The sensor must be power
     *        cycled before it responds on the new address.
     * @param newAddr_ New 7-bit I2C address (0x00-0x7F)
     * @return true if the sensor confirmed the EEPROM write
     */
    bool changeAddress(uint8_t newAddr_)
    {
        uint8_t data[1] = {newAddr_};
        return writeMemory(CMD_WRITE_EE, EE_ADDR_I2C_ADDRESS, data, 1);
    }

    bool isInitialized() const
    {
        return _isInitialized;
    }

  private:
    static uint8_t checksum(const uint8_t* buf_, uint8_t count_)
    {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < count_; ++i)
        {
            sum += buf_[i];
        }
        return sum;
    }

    // Sends a Read RAM/EEPROM request, waits for processing, then reads
    // back status + data + checksum and validates both before returning.
    bool readMemory(uint8_t cmdNibble_, uint8_t memAddr_, uint8_t numBytes_, uint8_t* outData_)
    {
        uint8_t command = static_cast<uint8_t>((cmdNibble_ << 4) | (numBytes_ & 0x0F));
        uint8_t addrHigh = 0x00;
        uint8_t addrLow = memAddr_;

        uint8_t requestBuf[3] = {command, addrHigh, addrLow};
        uint8_t requestChecksum = checksum(requestBuf, 3);

        _wireInterface.beginTransmission(_i2cAddress);
        _wireInterface.write(command);
        _wireInterface.write(addrHigh);
        _wireInterface.write(addrLow);
        _wireInterface.write(requestChecksum);
        if (_wireInterface.endTransmission() != 0)
        {
            return false;
        }

        delay(REQUEST_PROCESS_DELAY_MS);

        uint8_t responseLength = static_cast<uint8_t>(numBytes_ + 2);  // status + data + checksum
        _wireInterface.requestFrom(_i2cAddress, responseLength);
        if (_wireInterface.available() != responseLength)
        {
            return false;
        }

        uint8_t response[MAX_PAYLOAD_BYTES + 2] = {0};
        for (uint8_t i = 0; i < responseLength; ++i)
        {
            response[i] = _wireInterface.read();
        }

        uint8_t status = response[0];
        uint8_t receivedChecksum = response[responseLength - 1];
        uint8_t computedChecksum = checksum(response, static_cast<uint8_t>(responseLength - 1));

        bool complete = (status & STATUS_COMPLETE_BIT) != 0;
        bool commandMatches = ((status >> 4) == cmdNibble_);
        bool checksumOk = (receivedChecksum == computedChecksum);

        if (!complete || !commandMatches || !checksumOk)
        {
            return false;
        }

        for (uint8_t i = 0; i < numBytes_; ++i)
        {
            outData_[i] = response[1 + i];
        }

        return true;
    }

    // Sends a Write RAM/EEPROM request, waits for processing, then reads
    // back the 2-byte completion response and validates it.
    bool writeMemory(uint8_t cmdNibble_, uint8_t memAddr_, const uint8_t* data_, uint8_t numBytes_)
    {
        uint8_t command = static_cast<uint8_t>((cmdNibble_ << 4) | (numBytes_ & 0x0F));
        uint8_t addrHigh = 0x00;
        uint8_t addrLow = memAddr_;

        uint8_t requestBuf[3 + MAX_PAYLOAD_BYTES] = {command, addrHigh, addrLow};
        for (uint8_t i = 0; i < numBytes_; ++i)
        {
            requestBuf[3 + i] = data_[i];
        }
        uint8_t requestChecksum = checksum(requestBuf, static_cast<uint8_t>(3 + numBytes_));

        _wireInterface.beginTransmission(_i2cAddress);
        _wireInterface.write(command);
        _wireInterface.write(addrHigh);
        _wireInterface.write(addrLow);
        for (uint8_t i = 0; i < numBytes_; ++i)
        {
            _wireInterface.write(data_[i]);
        }
        _wireInterface.write(requestChecksum);
        if (_wireInterface.endTransmission() != 0)
        {
            return false;
        }

        delay(REQUEST_PROCESS_DELAY_MS);

        _wireInterface.requestFrom(_i2cAddress, static_cast<uint8_t>(2));  // status + checksum
        if (_wireInterface.available() != 2)
        {
            return false;
        }

        uint8_t status = _wireInterface.read();
        uint8_t receivedChecksum = _wireInterface.read();

        bool complete = (status & STATUS_COMPLETE_BIT) != 0;
        bool commandMatches = ((status >> 4) == cmdNibble_);
        bool checksumOk = (receivedChecksum == status);  // checksum covers the single status byte

        return complete && commandMatches && checksumOk;
    }

    TwoWire& _wireInterface;
    uint8_t _i2cAddress;
    bool _isInitialized = false;
};

#endif  // ROVER_LIB2_SENSORS_K30_HPP