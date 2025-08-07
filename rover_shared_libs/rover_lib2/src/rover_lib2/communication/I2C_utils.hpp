#ifndef ROVER_LIB2_COMMUNICATION_I2C_UTILS_HPP
#define ROVER_LIB2_COMMUNICATION_I2C_UTILS_HPP

#include "Wire.h"
#include <type_traits>

namespace I2CUtils
{
    template<std::integral T>
    void writeRegister(TwoWire& wireInterface_, uint8_t address_, uint8_t register_, T value_)
    {
        wireInterface_.beginTransmission(address_);
        wireInterface_.write(register_);

        constexpr size_t byteCount = sizeof(T);
        for (size_t i = 0; i < byteCount; ++i)
        {
            const size_t bitToShiftRight = (8 * (byteCount - 1 - i));
            wireInterface_.write(static_cast<uint8_t>((value_ >> bitToShiftRight) & 0xFF));
        }

        wireInterface_.endTransmission();
    }

    template<std::integral T>
    T readRegister(TwoWire& wireInterface_, uint8_t address_, uint8_t register_)
    {
        wireInterface_.beginTransmission(address_);
        wireInterface_.write(register_);
        wireInterface_.endTransmission(false);
        wireInterface_.requestFrom(address_, sizeof(T));

        T value = static_cast<T>(0U);

        constexpr size_t byteCount = sizeof(T);
        for (size_t i = 0; i < byteCount; ++i)
        {
            const size_t bitToShiftLeft = (8 * (byteCount - 1 - i));
            uint8_t wordValue = wireInterface_.read();
            value |= static_cast<T>(wordValue << bitToShiftLeft);
        }

        return value;
    }

};  // namespace I2CUtils

#endif  // ROVER_LIB2_COMMUNICATION_I2C_UTILS_HPP
