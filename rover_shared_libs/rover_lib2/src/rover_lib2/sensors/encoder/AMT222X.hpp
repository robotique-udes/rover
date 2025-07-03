#ifndef ROVER_LIB2_SENSORS_ENCODER_AMT222X_HPP
#define ROVER_LIB2_SENSORS_ENCODER_AMT222X_HPP

#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/sensors/encoder/encoder.hpp"
#include "rover_lib2/communication/SPI/SPI_device.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/watchdog.hpp"
#include "rover_lib2/helpers/one_shot_timer.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/storage/NVS_data_handle.hpp"

#include <bit>
#include <array>

DEFINE_LOG_NODE(AMT222X, Logger::eNodeState::OFF);

/**
 * @brief AMT222X is meant to provide a universal encoder object for any type of AMT222* encoders by keeping track of the position
 * data between power cycle. This is meant to mimick a absolute multiturn encoder. The position data will stay valid as long has
 * the encoder hasn't moved between cycle which makes it great for non-backdrivable robotic joints.
 *
 * This class only use the read position spi command and manage the rest internally to enable support for AMT222A and AMT222C
 * which only have this command in common
 */
class AMT222X : public Encoder<AMT222X>
{
    // Clock speed this low necessary because the ESP-IDF doesn't support adding clean delay between bytes in same transaction...
    // and AMT222X Requires 2.5us between bytes in same transaction.
    static constexpr uint32_t SPI_CLOCK_SPEED_HZ = 250'000UL;
    static constexpr uint16_t SPI_TIME_BEFORE_FIRST_BIT_US = 3U;
    static constexpr uint16_t SPI_TIME_AFTER_LAST_BIT_US = 3U;
    static constexpr SPIDeviceT::eSPIMode SPI_MODE = SPIDeviceT::eSPIMode::MODE_0;

    static constexpr uint64_t LOOP_PERIOD_US = 1UL;
    static constexpr uint64_t WATCHDOG_DATA_VALID_PERIOD_US = 500ULL;
    static constexpr uint64_t MIN_TIME_BETWEEN_SPEED_CALC_US = 25'000ULL;
    static constexpr uint64_t ENC_BOOT_TIME_US = 50ULL;

    static constexpr size_t TRANSACTION_MAX_LENGTH = 2UL;
    static constexpr std::array<uint8_t, 2U> CMD_READ_POSITION = {0x00, 0x00};

    static constexpr const char* NVS_KEY_TURN_COUNT = "AMT_TURN_CTN";
    static constexpr const char* NVS_KEY_LAST_QUADRANT = "AMT_QUADRANT";
    static constexpr const char* NVS_KEY_CALIB_OFFSET = "AMT_CALIB";

    static constexpr uint16_t VALID_DATA_BIT_MASK = 0b0011'1111'1111'1100;  // Only these bits contains the actual encoder message
    static constexpr uint16_t EVEN_CHECKSUM_RESULT_BIT_MASK = 0b0100'0000'0000'0000U;  // bit where the even checksum result is
    static constexpr uint16_t EVEN_CHECKSUM_BIT_MASK = 0b0001'0101'0101'0101U;  // bits on which the even checksum is calculated
    static constexpr uint16_t ODD_CHECKSUM_RESULT_BIT_MASK = 0b1000'0000'0000'0000U;  // bit where the odd checksum result is
    static constexpr uint16_t ODD_CHECKSUM_BIT_MASK = 0b0010'1010'1010'1010U;  // bits on which the odd checksum is calculated

    enum class eState : uint8_t
    {
        ASK_POSITION,
        READ_POSITION,
    };

  public:
    AMT222X(SPIBus& spiBus_, gpio_num_t pinCS_, const char* nvsNamespace_, bool reversed_ = false):
        _spiDevice(spiBus_, pinCS_, SPI_CLOCK_SPEED_HZ, SPI_TIME_BEFORE_FIRST_BIT_US, SPI_TIME_AFTER_LAST_BIT_US, SPI_MODE),
        _currentState(eState::ASK_POSITION),
        loopExec(LOOP_PERIOD_US),
        _dataValidWatchdog(WATCHDOG_DATA_VALID_PERIOD_US),
        _lastQuadrant(nvsNamespace_, NVS_KEY_LAST_QUADRANT, 0U),
        _turnCount(nvsNamespace_, NVS_KEY_TURN_COUNT, 0U),
        _calibOffset(nvsNamespace_, NVS_KEY_CALIB_OFFSET, 0.0F),
        _reversed(reversed_)
    {
    }

    void __init(void)
    {
        if (!_lastQuadrant.dataInSync() || !_turnCount.dataInSync())
        {
            LOG_WARN(Logger::Nodes::AMT222X, "Persistant data couldn't be read, calib necessary");
            _dataValidNVS = false;
        }
        else
        {
            _dataValidNVS = true;
        }

        _dtSpeedCalc.restart();
    }

    void __update(void)
    {
        if (loopExec.isReady())
        {
            switch (_currentState)
            {
                case eState::ASK_POSITION:
                    if (this->sendPositionRequest())
                    {
                        _currentState = eState::READ_POSITION;
                    }
                    break;
                case eState::READ_POSITION:
                    if (this->readPosition())
                    {
                        uint8_t currentQuadrant = 0U;
                        if (_encoderPosition >= 0.0F && _encoderPosition < (std::numbers::pi_v<float> / 2.0F))
                        {
                            currentQuadrant = 1U;
                        }
                        else if (_encoderPosition >= (std::numbers::pi_v<float> / 2.0F)
                                 && _encoderPosition < (std::numbers::pi_v<float>))
                        {
                            currentQuadrant = 2U;
                        }
                        else if (_encoderPosition >= (std::numbers::pi_v<float>)&&_encoderPosition
                                 < (3.0F * std::numbers::pi_v<float> / 2.0F))
                        {
                            currentQuadrant = 3U;
                        }
                        else if (_encoderPosition >= (3.0F * std::numbers::pi_v<float> / 2.0F)
                                 && _encoderPosition < (2.0F * std::numbers::pi_v<float>))
                        {
                            currentQuadrant = 4U;
                        }
                        else
                        {
                            LOG_WARN(Logger::Nodes::AMT222X, "_encoderPosition: %f", _encoderPosition);
                            ASSERT_MSG("Should never fall here, implementation error");
                        }

                        if (_lastQuadrant.getValue() == 4 && currentQuadrant == 1)
                        {
                            _turnCount.writeValue(_turnCount.getValue() + 1);
                        }
                        else if (_lastQuadrant.getValue() == 1 && currentQuadrant == 4)
                        {
                            _turnCount.writeValue(_turnCount.getValue() - 1);
                        }

                        _lastQuadrant.writeValue(currentQuadrant);
                        _currentPosition = _encoderPosition + std::numbers::pi_v<float> * 2.0F * _turnCount.getValue();
                        _currentState = eState::ASK_POSITION;
                    }
                    break;
            }
        }
    }

    bool _dataIsValid(void)
    {
        return _dataValidWatchdog.isOk() && _dataValidNVS;
    }

    float _getPosition(void)
    {
        return (_currentPosition + _calibOffset.getValue());
    }

    float _getSpeed(void)
    {
        return _currentSpeed;
    }

    void _calib(float offset_)
    {
        ASSERT_COND(offset_ <= static_cast<int16_t>(std::numeric_limits<int16_t>::max())
                    || offset_ >= static_cast<int16_t>(std::numeric_limits<int16_t>::min()));

        int16_t calibTurnCount = static_cast<int16_t>(ROUND(offset_ / (2.0F * std::numbers::pi_v<float>)));
        float actualoffset = offset_ - (2.0F * std::numbers::pi_v<float> * static_cast<float>(calibTurnCount));

        float calibOffset = actualoffset - _encoderPosition;

        bool calibValid = false;
        calibValid = _calibOffset.writeValue(calibOffset);
        calibValid &= _turnCount.writeValue(calibTurnCount);
        _dataValidNVS = calibValid;
    }

  private:
    bool sendPositionRequest(void)
    {
        return _spiDevice.writeData(CMD_READ_POSITION);
    }

    /**
     * @brief Returns true on transmission complete and not valid. Use the watchdog status to confirm if the data is valid
     */
    bool readPosition(void)
    {
        std::array<uint8_t, sizeof(CMD_READ_POSITION)> data;
        switch (_spiDevice.readData(data))
        {
            case SPIDeviceT::eReturnCode::TRANSMISSION_IN_PROGRESS:
                return false;
            case SPIDeviceT::eReturnCode::INVALID_STATE:
                [[fallthrough]];
            case SPIDeviceT::eReturnCode::TRANSMISSION_DONE_FAILED:
                return true;
            case SPIDeviceT::eReturnCode::TRANSMISSION_DONE_SUCCESS:
                break;
        }

        if (!this->validateChecksum(std::array<uint8_t, 2U>{data[0], data[1]}))
        {
            return true;
        }

        uint16_t newPos = data[0] << 8 | data[1];
        newPos &= VALID_DATA_BIT_MASK;
        newPos >>= 2;

        if (_reversed)
        {
            _encoderPosition = MAP(static_cast<float>(newPos),
                                   0.0F,
                                   static_cast<float>((1U << 12) - 1U),
                                   ((2.0F * std::numbers::pi_v<float>)-0.000'001F),
                                   0.0F);
        }
        else
        {
            _encoderPosition = MAP(static_cast<float>(newPos),
                                   0.0F,
                                   static_cast<float>((1U << 12) - 1U),
                                   0.0F,
                                   ((2.0F * std::numbers::pi_v<float>)-0.000'001F));
        }

        _dataValidWatchdog.reset();

        if (_dtSpeedCalc.getTime() >= MIN_TIME_BETWEEN_SPEED_CALC_US)
        {
            _currentSpeed = (_encoderPosition - _lastPosition) * (1'000'000.0F / static_cast<float>(_dtSpeedCalc.getTime()));
            _dtSpeedCalc.restart();
            _lastPosition = _encoderPosition;
        }

        return true;
    }

    bool validateChecksum(std::array<uint8_t, 2UL> bytes_)
    {
        uint16_t word = bytes_[0] << 8 | bytes_[1];

        bool evenCheckExpected = word & EVEN_CHECKSUM_RESULT_BIT_MASK;
        uint16_t evenBits = word & EVEN_CHECKSUM_BIT_MASK;
        bool evenXorResult = static_cast<bool>(std::popcount(evenBits) % 2);
        bool evenChecksumValid = (evenCheckExpected == (!evenXorResult));

        bool oddCheckExpected = word & ODD_CHECKSUM_RESULT_BIT_MASK;
        uint16_t oddBits = word & ODD_CHECKSUM_BIT_MASK;
        bool oddXorResult = static_cast<bool>(std::popcount(oddBits) % 2);
        bool oddChecksumValid = (oddCheckExpected == (!oddXorResult));

        return (evenChecksumValid && oddChecksumValid);
    }

    SPIDevice<TRANSACTION_MAX_LENGTH> _spiDevice;
    eState _currentState;
    LoopTimer<uint64_t, Time::micros> loopExec;

    float _encoderPosition = 0.0F;  // Constrained around 2*PI
    float _currentPosition = 0.0F;
    float _lastPosition = 0.0F;
    float _currentSpeed = 0.0F;
    Watchdog<uint64_t, Time::micros> _dataValidWatchdog;
    Chrono<uint64_t, Time::micros> _dtSpeedCalc;

    bool _dataValidNVS = false;
    NVSDataHandle<uint8_t> _lastQuadrant;
    NVSDataHandle<int16_t> _turnCount;
    NVSDataHandle<float> _calibOffset;

    bool _reversed;
};

#endif  // ROVER_LIB2_SENSORS_ENCODER_AMT222X_HPP
