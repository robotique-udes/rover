#ifndef ROVER_LIB2_SENSORS_INA219_HPP
#define ROVER_LIB2_SENSORS_INA219_HPP

#include "rover_lib2/communication/I2C_utils.hpp"
#include <limits>
#include <cmath>

class INA219
{
    // Register addresses
    static constexpr uint8_t REG_CONFIG = 0x00;
    static constexpr uint8_t REG_SHUNT_VOLTAGE = 0x01;
    static constexpr uint8_t REG_BUS_VOLTAGE = 0x02;
    static constexpr uint8_t REG_POWER = 0x03;
    static constexpr uint8_t REG_CURRENT = 0x04;
    static constexpr uint8_t REG_CALIBRATION = 0x05;

    // Configuration Register Bits
    static constexpr uint16_t CONFIG_RESET_BIT = 0x8000;
    static constexpr uint16_t CONFIG_BUS_VOLTAGE_RANGE_32V = 0x2000;
    static constexpr uint16_t CONFIG_GAIN_320MV = 0x1800;  // ±320mV range
    static constexpr uint16_t CONFIG_BADCRES_12BIT = 0x0180;
    static constexpr uint16_t CONFIG_SADCRES_12BIT_128S = 0x0078;
    static constexpr uint16_t CONFIG_MODE_SHUNT_BUS_CONTINUOUS = 0x0007;

    // Constants for calculations
    static constexpr float SHUNT_VOLTAGE_LSB = 0.00001F;  // 10µV per bit
    static constexpr float BUS_VOLTAGE_LSB = 0.004F;      // 4mV per bit
    static constexpr float CAL_FACTOR = 0.04096F;         // Calibration factor from datasheet

  public:
    INA219(TwoWire& wireInterface_, uint8_t address_, float shuntResistanceOhms_, float maxExpectedAmps_):
        _wireInterface(wireInterface_),
        _i2cAddress(address_),
        _shuntResistanceOhms(shuntResistanceOhms_),
        _maxExpectedAmps(maxExpectedAmps_)
    {
        calculateCalibration();
    }

    void init()
    {
        reset();
        delay(1);

        uint16_t config = CONFIG_BUS_VOLTAGE_RANGE_32V | CONFIG_GAIN_320MV | CONFIG_BADCRES_12BIT | CONFIG_SADCRES_12BIT_128S
                          | CONFIG_MODE_SHUNT_BUS_CONTINUOUS;

        I2CUtils::writeRegister<uint16_t>(_wireInterface, _i2cAddress, REG_CONFIG, config);
        I2CUtils::writeRegister<uint16_t>(_wireInterface, _i2cAddress, REG_CALIBRATION, _calibrationValue);

        _isInitialized = true;
    }

    void reset()
    {
        I2CUtils::writeRegister<uint16_t>(_wireInterface, _i2cAddress, REG_CONFIG, CONFIG_RESET_BIT);
        _isInitialized = false;
    }

    float getCurrent()
    {
        if (!_isInitialized)
        {
            return 0.0F;
        }

        int16_t rawCurrent = I2CUtils::readRegister<int16_t>(_wireInterface, _i2cAddress, REG_CURRENT);
        return static_cast<float>(rawCurrent) * _currentLSB;
    }

    /**
     * @brief Get bus voltage measurement in Volts
     * @return Bus voltage in Volts
     */
    float getBusVoltage()
    {
        if (!_isInitialized)
        {
            return 0.0F;
        }

        uint16_t rawVoltage = I2CUtils::readRegister<uint16_t>(_wireInterface, _i2cAddress, REG_BUS_VOLTAGE);
        uint16_t voltageData = (rawVoltage >> 3);
        return static_cast<float>(voltageData) * BUS_VOLTAGE_LSB;
    }

    float getShuntVoltage()
    {
        if (!_isInitialized)
        {
            return 0.0F;
        }

        int16_t rawShuntVoltage = I2CUtils::readRegister<int16_t>(_wireInterface, _i2cAddress, REG_SHUNT_VOLTAGE);
        return static_cast<float>(rawShuntVoltage) * SHUNT_VOLTAGE_LSB;
    }

    float getPower()
    {
        if (!_isInitialized)
        {
            return 0.0F;
        }

        uint16_t rawPower = I2CUtils::readRegister<uint16_t>(_wireInterface, _i2cAddress, REG_POWER);
        return static_cast<float>(rawPower) * _powerLSB;
    }

    bool isBusVoltageReady()
    {
        if (!_isInitialized)
        {
            return false;
        }

        uint16_t busVoltageReg = I2CUtils::readRegister<uint16_t>(_wireInterface, _i2cAddress, REG_BUS_VOLTAGE);
        return (busVoltageReg & 0x0002) != 0;  // CNVR bit (bit 1)
    }

    bool isMathOverflow()
    {
        if (!_isInitialized)
        {
            return false;
        }

        uint16_t busVoltageReg = I2CUtils::readRegister<uint16_t>(_wireInterface, _i2cAddress, REG_BUS_VOLTAGE);
        return (busVoltageReg & 0x0001) != 0;  // OVF bit (bit 0)
    }

    float getCurrentLSB() const
    {
        return _currentLSB;
    }
    float getPowerLSB() const
    {
        return _powerLSB;
    }
    uint16_t getCalibrationValue() const
    {
        return _calibrationValue;
    }
    bool isInitialized() const
    {
        return _isInitialized;
    }

  private:
    void calculateCalibration()
    {
        float minLSB = _maxExpectedAmps / 32767.0F;

        // Round up to a nice number for Current_LSB
        // Common values: 0.1mA, 1mA, 10mA, 100mA, etc.
        if (minLSB <= 0.0001F)
        {
            _currentLSB = 0.0001F;  // 0.1mA
        }
        else if (minLSB <= 0.001F)
        {
            _currentLSB = 0.001F;  // 1mA
        }
        else if (minLSB <= 0.01F)
        {
            _currentLSB = 0.01F;  // 10mA
        }
        else if (minLSB <= 0.1F)
        {
            _currentLSB = 0.1F;  // 100mA
        }
        else
        {
            _currentLSB = minLSB;  // Use calculated value if too large
        }

        float calValue = CAL_FACTOR / (_currentLSB * _shuntResistanceOhms);
        _calibrationValue = static_cast<uint16_t>(std::round(calValue));

        if (_calibrationValue == 0)
        {
            _calibrationValue = 1;
        }

        _powerLSB = 20.0F * _currentLSB;
    }

    TwoWire& _wireInterface;
    uint8_t _i2cAddress;
    float _shuntResistanceOhms;
    float _maxExpectedAmps;

    uint16_t _calibrationValue = 0U;
    float _currentLSB = 0.0F;
    float _powerLSB = 0.0F;
    bool _isInitialized = false;
};

#endif  // ROVER_LIB2_SENSORS_INA219_HPP
