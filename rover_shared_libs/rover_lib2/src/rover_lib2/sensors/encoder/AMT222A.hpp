#ifndef ROVER_LIB2_SENSORS_ENCODER_AMT222A_HPP
#define ROVER_LIB2_SENSORS_ENCODER_AMT222A_HPP

#include "rover_lib2/sensors/encoder/encoder.hpp"
#include "rover_lib2/communication/SPI/SPI_device.hpp"
#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/watchdog.hpp"
#include "rover_lib2/helpers/one_shot_timer.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/filters/none.hpp"

#include <bit>
#include <array>

DEFINE_LOG_NODE(AMT222A, Logger::eNodeState::OFF);

namespace Encoders
{

    /**
     * @brief AMT222A Absolute one turn encoder driver. Allow absolute and persistent calibration inside encoder itself. Only
     * support one turn and will always return a position value between [0; 2PI[]
     *
     */
    template<Filters::Filter FilterPosT = Filters::None, Filters::Filter FilterSpeedT = Filters::None>
    class AMT222A
    {
        // Clock speed this low necessary because the ESP-IDF doesn't support adding clean delay between bytes in same
        // transaction... and AMT222X Requires 2.5us between bytes in same transaction.
        static constexpr uint32_t SPI_CLOCK_SPEED_HZ = 250'000UL;
        static constexpr uint64_t LOOP_PERIOD_US = 1UL;
        static constexpr uint64_t WATCHDOG_DATA_VALID_PERIOD = 500ULL;
        static constexpr uint64_t MIN_TIME_BETWEEN_SPEED_CALC_US = 25'000ULL;
        static constexpr uint64_t ENC_BOOT_TIME_US = 50ULL;

        static constexpr size_t TRANSACTION_MAX_LENGTH = 2UL;
        static constexpr std::array<uint8_t, 2U> CMD_READ_POSITION = {0x00, 0x00};
        static constexpr std::array<uint8_t, 2U> CMD_RESET = {0x00, 0x60};
        static constexpr std::array<uint8_t, 2U> CMD_CALIB = {0x00, 0x70};

        static constexpr uint16_t VALID_DATA_BIT_MASK
            = 0b0011'1111'1111'1100;  // Only these bits contains the actual encoder message
        static constexpr uint16_t EVEN_CHECKSUM_RESULT_BIT_MASK
            = 0b0100'0000'0000'0000U;  // bit where the even checksum result is
        static constexpr uint16_t EVEN_CHECKSUM_BIT_MASK
            = 0b0001'0101'0101'0101U;  // bits on which the even checksum is calculated
        static constexpr uint16_t ODD_CHECKSUM_RESULT_BIT_MASK = 0b1000'0000'0000'0000U;  // bit where the odd checksum result is
        static constexpr uint16_t ODD_CHECKSUM_BIT_MASK = 0b0010'1010'1010'1010U;  // bits on which the odd checksum is calculated

        enum class eState : uint8_t
        {
            READY,
            ASK_POSITION,
            READ_POSITION,
            EXEC_CALIB,
            READ_CALIB,
            WAIT_POWER_ON
        };

      public:
        AMT222A(SPIBus& spiBus_,
                gpio_num_t pinCS_,
                bool reversed_ = false,
                FilterPosT posFilter_ = Filters::None(),
                FilterSpeedT speedFilter_ = Filters::None()):
            _spiDevice(spiBus_, pinCS_, SPI_CLOCK_SPEED_HZ, 3U, 3U, SPIDeviceT::eSPIMode::MODE_0),
            _filterPos(posFilter_),
            _filterSpeed(speedFilter_),
            _reversed(reversed_)
        {
        }

        void init(void)
        {
            _dtSpeedCalc.restart();
        }

        void update(void)
        {
            if (loopExec.isReady())
            {
                switch (_currentState)
                {
                    case eState::READY:
                        if (_calibRequested)
                        {
                            _currentState = eState::EXEC_CALIB;
                        }
                        else
                        {
                            _currentState = eState::ASK_POSITION;
                        }
                        break;
                    case eState::ASK_POSITION:
                        if (this->sendPositionRequest())
                        {
                            _currentState = eState::READ_POSITION;
                        }
                        break;
                    case eState::READ_POSITION:
                        if (this->readPosition())
                        {
                            _currentState = eState::READY;
                        }
                        break;
                    case eState::EXEC_CALIB:
                        if (this->sendCalib())
                        {
                            _currentState = eState::READ_CALIB;
                        }
                        break;
                    case eState::READ_CALIB:
                        if (this->calibDone())
                        {
                            _timerTimingDelay = decltype(_timerTimingDelay)(ENC_BOOT_TIME_US);
                            _currentState = eState::WAIT_POWER_ON;
                        }
                        break;
                    case eState::WAIT_POWER_ON:
                        if (_timerTimingDelay.isReady())
                        {
                            _calibRequested = false;
                            _currentState = eState::READY;
                        }
                        break;
                    default:
                        ASSERT_MSG("Shouldn't fall here, implementation error");
                        _currentState = eState::READY;  // TODO Set case failure
                        break;
                }
            }
        }

        bool dataIsValid(void) const
        {
            return _dataValidWatchdog.isOk();
        }

        float getPosition(void) const
        {
            return CONSTRAIN_TO_CIRCLE(_calibOffset + _currentPosition);
        }

        float getSpeed(void) const
        {
            return _currentSpeed;
        }

        void calib(float offset_)
        {
            LOG_DEBUG(Logger::Nodes::AMT222A, "Calibration requested with offset: %f", offset_);
            offset_ = CONSTRAIN_TO_CIRCLE(offset_);

            float calibOffset = offset_ - _currentPosition;

            _calibRequested = true;
            _calibOffset = calibOffset;
        }

      private:
        bool sendPositionRequest(void)
        {
            return _spiDevice.writeData(CMD_READ_POSITION);
        }

        bool sendCalib(void)
        {
            return _spiDevice.writeData(CMD_CALIB);
        }

        bool calibDone(void)
        {
            std::array<uint8_t, sizeof(CMD_READ_POSITION)> data;
            if (_spiDevice.readData(data) != SPIDeviceT::eReturnCode::TRANSMISSION_IN_PROGRESS)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        /**
         * @brief Returns true on transmission complete and not valid. Use the watchdog status to confirm if the data is valid
         *
         * @return true
         * @return false
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
                default:
                    ASSERT_MSG("Shouldn't fall here, implementation error");
                    return false;
            }

            if (!this->validateChecksum(std::array<uint8_t, 2U>{data[0], data[1]}))
            {
                return true;
            }

            uint16_t newPos = data[0] << 8 | data[1];
            newPos &= VALID_DATA_BIT_MASK;
            newPos >>= 2;

            float currentPosTemp = 0.0F;

            if (_reversed)
            {
                currentPosTemp = MAP(static_cast<float>(newPos),
                                     0.0F,
                                     static_cast<float>((1U << 12) - 1U),
                                     0.0F,
                                     ((2.0F * std::numbers::pi_v<float>)-0.000'001F));
            }
            else
            {
                currentPosTemp = MAP(static_cast<float>(newPos),
                                     0.0F,
                                     static_cast<float>((1U << 12) - 1U),
                                     ((2.0F * std::numbers::pi_v<float>)-0.000'001F),
                                     0.0F);
            }

            if (_isFirstRead)
            {
                _filterPos.reset(currentPosTemp);
                _currentPosition = currentPosTemp;
                _lastPosition = _currentPosition;
            }
            else
            {
                _currentPosition = _filterPos.addValue(currentPosTemp);
            }

            _dataValidWatchdog.reset();

            if (_dtSpeedCalc.getTime() >= MIN_TIME_BETWEEN_SPEED_CALC_US)
            {
                float currentSpeedTemp
                    = (_currentPosition - _lastPosition) * (1'000'000.0F / static_cast<float>(_dtSpeedCalc.getTime()));
                if (_isFirstRead)
                {
                    _filterSpeed.reset(currentSpeedTemp);
                }
                else
                {
                    _currentSpeed = _filterSpeed.addValue(currentSpeedTemp);
                }

                _dtSpeedCalc.restart();
                _lastPosition = _currentPosition;
            }

            _isFirstRead = false;

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
        eState _currentState = {eState::READY};
        LoopTimer<uint64_t, &Time::micros> loopExec = {LOOP_PERIOD_US};

        bool _calibRequested = false;
        bool _isFirstRead = true;
        float _calibOffset = 0.0F;
        float _currentPosition = 0.0F;
        float _lastPosition = 0.0F;
        float _currentSpeed = 0.0F;

        FilterPosT _filterPos;
        FilterSpeedT _filterSpeed;

        Watchdog<uint64_t, &Time::micros> _dataValidWatchdog = {WATCHDOG_DATA_VALID_PERIOD};
        Chrono<uint64_t, &Time::micros> _dtSpeedCalc;
        OneShotTimer<uint64_t, &Time::micros> _timerTimingDelay = {0};

        bool _reversed;

        VALIDATE_CONCEPT(Encoder, AMT222A);
    };

}  // namespace Encoders

#endif  // ROVER_LIB2_SENSORS_ENCODER_AMT222A_HPP
